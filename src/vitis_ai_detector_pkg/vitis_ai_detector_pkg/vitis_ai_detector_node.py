#!/usr/bin/env python3

import csv
import json
import os
import select
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from my_interfaces.msg import Detection, DetectionArray


BOX_COLORS = {
    "car": (0, 255, 0),
    "bicycle": (255, 180, 0),
    "person": (0, 80, 255),
}


def _default_model_path():
    """Installed share/ copy of the deployed model.

    The model ships inside this package, so it resolves without any
    /home/<user>/... assumption. The launch file passes the same path
    explicitly; this default keeps a bare `ros2 run` working.
    """
    try:
        share = get_package_share_directory("vitis_ai_detector_pkg")
    except Exception:
        return ""
    return os.path.join(share, "models", "yolov3_tiny_7class.xmodel")


def _default_worker_script_path():
    """YOLO worker, which lives next to this file."""
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vitis_ai_worker_yolo.py"
    )


def ros_image_to_bgr(msg):
    encoding = msg.encoding.lower()

    if encoding in ("bgr8", "rgb8", "8uc3"):
        channels = 3
    elif encoding in ("bgra8", "rgba8"):
        channels = 4
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    else:
        raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

    expected_row_bytes = msg.width * channels
    expected_size = msg.height * msg.step

    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
    except TypeError:
        data = np.asarray(msg.data, dtype=np.uint8)
    if data.size < expected_size:
        raise RuntimeError(f"Image data too small: {data.size} < {expected_size}")

    data = data[:expected_size]
    rows = data.reshape((msg.height, msg.step))
    pixels = rows[:, :expected_row_bytes]

    if channels == 1:
        mono = pixels.reshape((msg.height, msg.width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    image = pixels.reshape((msg.height, msg.width, channels))

    if encoding in ("bgr8", "8uc3"):
        return image.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


class VitisAiDetectorNode(Node):
    def __init__(self):
        super().__init__("vitis_ai_detector_node")

        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("worker_script_path", _default_worker_script_path())
        self.declare_parameter("detector_mode", "worker")
        self.declare_parameter("worker_log_path", "")
        self.declare_parameter("worker_softmax", "auto")
        self.declare_parameter("send_resized_input", True)
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/detections")
        self.declare_parameter("overlay_topic", "/vitis_ai_detector/overlay")
        self.declare_parameter("overlay_width", 480)
        self.declare_parameter("overlay_height", 360)
        self.declare_parameter("process_period_sec", 0.0)
        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("worker_startup_timeout_sec", 30.0)
        self.declare_parameter("publish_empty_detections", True)
        self.declare_parameter("publish_overlay", False)
        self.declare_parameter("publish_compressed_overlay", False)
        self.declare_parameter("overlay_jpeg_quality", 70)
        self.declare_parameter("log_period_sec", 1.0)
        # When set, per-frame timing is buffered in memory and written to this
        # CSV file on shutdown (instead of being printed to the terminal).
        self.declare_parameter("metrics_csv_path", "")
        # >0: auto-collect exactly this many seconds (measured from the first
        # processed frame, so camera/worker startup is excluded), then write the
        # CSV automatically. 0 = collect until shutdown.
        self.declare_parameter("metrics_duration_sec", 0.0)

        self.model_path = self.get_parameter("model_path").value
        self.worker_script_path = self.get_parameter("worker_script_path").value
        self.detector_mode = str(self.get_parameter("detector_mode").value)
        self.worker_log_path = self.get_parameter("worker_log_path").value
        self.worker_softmax = self.get_parameter("worker_softmax").value
        self.send_resized_input = bool(
            self.get_parameter("send_resized_input").value
        )
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.overlay_width = int(self.get_parameter("overlay_width").value)
        self.overlay_height = int(self.get_parameter("overlay_height").value)
        self.process_period_sec = float(self.get_parameter("process_period_sec").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.worker_startup_timeout_sec = float(
            self.get_parameter("worker_startup_timeout_sec").value
        )
        self.publish_empty_detections = bool(
            self.get_parameter("publish_empty_detections").value
        )
        self.publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self.publish_compressed_overlay = bool(
            self.get_parameter("publish_compressed_overlay").value
        )
        self.overlay_jpeg_quality = int(
            self.get_parameter("overlay_jpeg_quality").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.metrics_csv_path = str(self.get_parameter("metrics_csv_path").value)
        # None = CSV disabled (print to terminal as before); list = collect rows.
        self.metrics_rows = [] if self.metrics_csv_path else None
        self.metrics_duration_sec = float(
            self.get_parameter("metrics_duration_sec").value
        )
        self.metrics_start_ns = None
        self.metrics_window_closed = False

        self.last_process_time_ns = None
        self.last_log_time_ns = None
        self.worker_process = None
        self.worker_input_height = None
        self.worker_input_width = None
        self.last_worker_timing = None

        # Pipelining: the subscription callback only stores the newest frame; a
        # dedicated worker thread consumes it back-to-back (worker_loop /
        # process_frame), so the DPU worker is never idle waiting for the next
        # callback and the camera is never starved by a busy detector.
        self.frame_lock = threading.Lock()
        self.latest_msg = None
        self.frame_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.worker_thread = None

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            self.output_topic,
            output_qos,
        )
        self.overlay_publisher = None
        if self.publish_overlay:
            self.overlay_publisher = self.create_publisher(
                Image,
                self.overlay_topic,
                image_qos,
            )
        self.compressed_overlay_publisher = None
        if self.publish_compressed_overlay:
            self.compressed_overlay_publisher = self.create_publisher(
                CompressedImage,
                f"{self.overlay_topic}/compressed",
                image_qos,
            )

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos,
        )

        if self.detector_mode != "worker":
            raise RuntimeError(
                f"Unsupported detector_mode: {self.detector_mode!r} "
                "(only 'worker' is supported)"
            )
        self.start_worker()

        self.worker_thread = threading.Thread(
            target=self.worker_loop, name="vitis_ai_worker_loop", daemon=True
        )
        self.worker_thread.start()

        self.get_logger().info(
            f"Vitis-AI detector started in {self.detector_mode} mode: "
            f"{self.input_topic} -> {self.output_topic}"
        )
        self.get_logger().info(
            "Low-latency policy: sensor QoS depth=1, latest frame only, "
            f"process_period_sec={self.process_period_sec:.3f}, "
            f"send_resized_input={self.send_resized_input}, "
            f"publish_overlay={self.publish_overlay}, "
            f"publish_compressed_overlay={self.publish_compressed_overlay}"
        )

    def image_callback(self, msg):
        # Lightweight: keep only the newest frame and hand off to the worker
        # thread. Returns immediately so the executor never blocks on the DPU.
        with self.frame_lock:
            self.latest_msg = msg
            self.frame_event.set()

    def worker_loop(self):
        # Dedicated thread: process frames back-to-back so the DPU worker is
        # never idle waiting for the next subscription callback. Always picks the
        # newest available frame (latest-frame-only, minimal latency).
        while not self.shutdown_event.is_set():
            if not self.frame_event.wait(timeout=0.5):
                continue
            with self.frame_lock:
                msg = self.latest_msg
                self.latest_msg = None
                self.frame_event.clear()
            if msg is None:
                continue
            if not self.should_process_frame():
                continue
            self.process_frame(msg)

    def process_frame(self, msg):
        processing_start_ns = time.perf_counter_ns()
        # Frame age at the moment we start processing = camera capture -> here
        # (USB + realsense node + DDS + subscription queue + handoff). This is
        # the part that happens BEFORE processing_ms.
        frame_age_in_ms = self.frame_age_ms(msg)
        log_this_frame = self.should_log()
        try:
            image = ros_image_to_bgr(msg)
            image_ready_ns = time.perf_counter_ns()

            detections = self.detect_with_worker(image)
            detection_done_ns = time.perf_counter_ns()

            out_msg = DetectionArray()
            out_msg.header = msg.header
            out_msg.detections = detections

            if detections or self.publish_empty_detections:
                self.publisher.publish(out_msg)
            detection_publish_done_ns = time.perf_counter_ns()

            if self.publish_overlay or self.publish_compressed_overlay:
                if log_this_frame:
                    self.get_logger().info("Publishing low-resolution overlay")
                overlay, scale_x, scale_y = self.prepare_overlay(image)
                overlay = self.draw_detections(
                    overlay,
                    detections,
                    scale_x=scale_x,
                    scale_y=scale_y,
                )
                if self.publish_overlay and self.overlay_publisher is not None:
                    self.overlay_publisher.publish(
                        self.make_overlay_msg(overlay, msg.header)
                    )
                if (
                    self.publish_compressed_overlay
                    and self.compressed_overlay_publisher is not None
                ):
                    self.compressed_overlay_publisher.publish(
                        self.make_compressed_overlay_msg(overlay, msg.header)
                    )
            overlay_done_ns = time.perf_counter_ns()

            # --- per-frame metrics (cheap arithmetic, computed every frame) ---
            # detect_ms = img_ms + worker_call_ms
            # worker_call_ms = worker_ms (pure compute) + ipc_overhead_ms
            # worker_ms = pre_ms + dpu_ms + post_ms (reported by the worker)
            processing_ms = (overlay_done_ns - processing_start_ns) / 1e6
            detection_ms = (detection_done_ns - processing_start_ns) / 1e6
            detection_publish_ms = (
                detection_publish_done_ns - detection_done_ns
            ) / 1e6
            overlay_ms = (overlay_done_ns - detection_publish_done_ns) / 1e6
            img_ms = (image_ready_ns - processing_start_ns) / 1e6
            worker_call_ms = (detection_done_ns - image_ready_ns) / 1e6
            frame_age_ms = self.frame_age_ms(msg)
            wt = self.last_worker_timing or {}
            worker_ms = wt.get("worker_ms")
            pre_ms = wt.get("pre_ms")
            dpu_ms = wt.get("dpu_ms")
            post_ms = wt.get("post_ms")
            ipc_overhead_ms = (
                worker_call_ms - worker_ms if worker_ms is not None else None
            )

            if self.metrics_rows is not None and not self.metrics_window_closed:
                if self.metrics_start_ns is None:
                    self.metrics_start_ns = processing_start_ns
                    if self.metrics_duration_sec > 0:
                        self.get_logger().info(
                            "Metrics window started: collecting "
                            f"{self.metrics_duration_sec:.0f}s from the first frame"
                        )
                self.metrics_rows.append({
                    "stamp_sec": msg.header.stamp.sec,
                    "stamp_nanosec": msg.header.stamp.nanosec,
                    "count": len(detections),
                    "image_w": msg.width,
                    "image_h": msg.height,
                    "processing_ms": self._round(processing_ms),
                    "detect_ms": self._round(detection_ms),
                    "img_ms": self._round(img_ms),
                    "worker_call_ms": self._round(worker_call_ms),
                    "ipc_overhead_ms": self._round(ipc_overhead_ms),
                    "pre_ms": self._round(pre_ms),
                    "dpu_ms": self._round(dpu_ms),
                    "post_ms": self._round(post_ms),
                    "worker_ms": self._round(worker_ms),
                    "detection_publish_ms": self._round(detection_publish_ms),
                    "overlay_ms": self._round(overlay_ms),
                    "age_in_ms": self._round(frame_age_in_ms),
                    "frame_age_ms": self._round(frame_age_ms),
                })
                if (
                    self.metrics_duration_sec > 0
                    and (time.perf_counter_ns() - self.metrics_start_ns) / 1e9
                    >= self.metrics_duration_sec
                ):
                    self.save_metrics_csv()
                    self.metrics_window_closed = True
                    self.get_logger().info(
                        f"Metrics window {self.metrics_duration_sec:.0f}s complete: "
                        f"{len(self.metrics_rows)} rows saved to "
                        f"{self.metrics_csv_path}. You can stop the launch now "
                        f"(Ctrl-C)."
                    )

            if log_this_frame:
                if self.metrics_rows is not None:
                    if self.metrics_window_closed:
                        self.get_logger().info(
                            f"Metrics window done: {len(self.metrics_rows)} rows "
                            f"saved. Idle (Ctrl-C to exit)."
                        )
                    else:
                        self.get_logger().info(
                            f"Collecting metrics: {len(self.metrics_rows)} rows "
                            f"(CSV -> {self.metrics_csv_path})"
                        )
                else:
                    age_text = (
                        f"{frame_age_ms:.1f}"
                        if frame_age_ms is not None else "unknown"
                    )
                    age_in_text = (
                        f"{frame_age_in_ms:.1f}"
                        if frame_age_in_ms is not None else "unknown"
                    )
                    if worker_ms is not None:
                        breakdown = (
                            f"img_ms={img_ms:.1f}, "
                            f"worker_call_ms={worker_call_ms:.1f}, "
                            f"pre_ms={pre_ms:.1f}, "
                            f"dpu_ms={dpu_ms:.1f}, "
                            f"post_ms={post_ms:.1f}, "
                            f"worker_ms={worker_ms:.1f}, "
                            f"ipc_overhead_ms={ipc_overhead_ms:.1f}, "
                        )
                    else:
                        breakdown = (
                            f"img_ms={img_ms:.1f}, "
                            f"worker_call_ms={worker_call_ms:.1f}, "
                        )
                    self.get_logger().info(
                        f"Published detections: count={len(detections)}, "
                        f"image={msg.width}x{msg.height}, mode={self.detector_mode}, "
                        f"processing_ms={processing_ms:.1f}, "
                        f"detect_ms={detection_ms:.1f}, "
                        f"{breakdown}"
                        f"detection_publish_ms={detection_publish_ms:.1f}, "
                        f"overlay_ms={overlay_ms:.1f}, "
                        f"age_in_ms={age_in_text}, "
                        f"frame_age_ms={age_text}"
                    )

        except subprocess.TimeoutExpired:
            self.get_logger().error("Vitis-AI detector timeout")
        except Exception as exc:
            self.get_logger().error(f"Detection failed: {exc}")

    def start_worker(self):
        if self.worker_process is not None and self.worker_process.poll() is None:
            return

        command = [
            sys.executable,
            self.worker_script_path,
            "--model",
            self.model_path,
            "--softmax",
            self.worker_softmax,
        ]
        if self.worker_log_path:
            command.extend(["--log-file", self.worker_log_path])

        self.get_logger().info("Starting Vitis-AI worker process")
        self.worker_process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        try:
            ready = self.read_worker_json(
                "worker ready",
                timeout_sec=self.worker_startup_timeout_sec,
            )
        except Exception:
            self.stop_worker()
            raise
        if ready.get("status") != "ready":
            raise RuntimeError(f"Unexpected worker ready response: {ready}")

        input_shape = ready.get("input_shape")
        if not isinstance(input_shape, list) or len(input_shape) != 4:
            raise RuntimeError(f"Invalid worker input shape: {input_shape}")
        self.worker_input_height = int(input_shape[1])
        self.worker_input_width = int(input_shape[2])

        self.get_logger().info(
            f"Vitis-AI worker ready: input_shape={input_shape}"
        )

    def stop_worker(self):
        process = self.worker_process
        self.worker_process = None
        self.worker_input_height = None
        self.worker_input_width = None
        if process is None:
            return

        if process.poll() is None:
            try:
                process.stdin.close()
            except Exception:
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.kill(process.pid, signal.SIGKILL)

    def restart_worker(self):
        self.stop_worker()
        self.start_worker()

    def read_worker_json(self, label, timeout_sec=None):
        process = self.worker_process
        if process is None or process.stdout is None:
            raise RuntimeError("Worker process is not running")

        if timeout_sec is None:
            timeout_sec = self.timeout_sec

        ready, _, _ = select.select([process.stdout], [], [], timeout_sec)
        if not ready:
            raise subprocess.TimeoutExpired(label, timeout_sec)

        line = process.stdout.readline()
        if not line:
            returncode = process.poll()
            stderr = ""
            if process.stderr is not None and returncode is not None:
                stderr = process.stderr.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"Worker closed stdout during {label}: "
                f"returncode={returncode}, stderr={stderr}"
            )

        return json.loads(line.decode("utf-8"))

    def detect_with_worker(self, image):
        image = np.ascontiguousarray(image, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise RuntimeError(f"Expected BGR image with 3 channels, got {image.shape}")

        if self.worker_process is None or self.worker_process.poll() is not None:
            self.restart_worker()

        source_height, source_width, _ = image.shape
        worker_image = image
        if self.send_resized_input:
            if self.worker_input_width is None or self.worker_input_height is None:
                raise RuntimeError("Worker input dimensions are not available")
            if (
                source_width != self.worker_input_width
                or source_height != self.worker_input_height
            ):
                worker_image = cv2.resize(
                    image,
                    (self.worker_input_width, self.worker_input_height),
                    interpolation=cv2.INTER_LINEAR,
                )

        worker_image = np.ascontiguousarray(worker_image, dtype=np.uint8)
        height, width, channels = worker_image.shape
        request = {
            "height": int(height),
            "width": int(width),
            "channels": int(channels),
            "data_len": int(worker_image.nbytes),
            "source_height": int(source_height),
            "source_width": int(source_width),
        }

        try:
            self.worker_process.stdin.write(json.dumps(request).encode("utf-8") + b"\n")
            self.worker_process.stdin.write(worker_image.tobytes())
            self.worker_process.stdin.flush()
            response = self.read_worker_json("worker inference")
        except (BrokenPipeError, RuntimeError, subprocess.TimeoutExpired):
            self.restart_worker()
            raise

        if response.get("error"):
            raise RuntimeError(f"worker error: {response['error']}")

        self.last_worker_timing = response.get("timing")
        return self.json_items_to_detections(response.get("detections", []))

    def json_items_to_detections(self, items):
        detections = []
        for item in items:
            det = Detection()
            det.class_id = int(item["class_id"])
            det.class_name = str(item["class_name"])
            det.confidence = float(item["confidence"])
            det.center_x = float(item["center_x"])
            det.center_y = float(item["center_y"])
            det.width = float(item["width"])
            det.height = float(item["height"])
            detections.append(det)

        return detections

    def prepare_overlay(self, image):
        source_height, source_width = image.shape[:2]
        target_width = self.overlay_width
        target_height = self.overlay_height

        if target_width <= 0 or target_height <= 0:
            return image.copy(), 1.0, 1.0

        if target_width == source_width and target_height == source_height:
            return image.copy(), 1.0, 1.0

        overlay = cv2.resize(
            image,
            (target_width, target_height),
            interpolation=cv2.INTER_AREA,
        )
        return (
            overlay,
            target_width / float(source_width),
            target_height / float(source_height),
        )

    def draw_detections(self, image, detections, scale_x=1.0, scale_y=1.0):
        for det in detections:
            xmin = int(round((det.center_x - det.width / 2.0) * scale_x))
            ymin = int(round((det.center_y - det.height / 2.0) * scale_y))
            xmax = int(round((det.center_x + det.width / 2.0) * scale_x))
            ymax = int(round((det.center_y + det.height / 2.0) * scale_y))

            xmin = max(0, min(image.shape[1] - 1, xmin))
            xmax = max(0, min(image.shape[1] - 1, xmax))
            ymin = max(0, min(image.shape[0] - 1, ymin))
            ymax = max(0, min(image.shape[0] - 1, ymax))

            color = BOX_COLORS.get(det.class_name, (0, 255, 0))
            label = f"{det.class_name} {det.confidence:.2f}"

            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)

            text_y = max(20, ymin - 8)
            cv2.putText(
                image,
                label,
                (xmin, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

        return image

    def make_overlay_msg(self, image, header):
        image = np.ascontiguousarray(image, dtype=np.uint8)

        msg = Image()
        msg.header = header
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * image.shape[2])
        msg.data = image.tobytes()
        return msg

    def make_compressed_overlay_msg(self, image, header):
        quality = max(1, min(100, self.overlay_jpeg_quality))
        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if not success:
            raise RuntimeError("Failed to encode compressed overlay")

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg

    def should_process_frame(self):
        if self.process_period_sec <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds

        if self.last_process_time_ns is None:
            self.last_process_time_ns = now_ns
            return True

        elapsed_sec = (now_ns - self.last_process_time_ns) / 1e9
        if elapsed_sec < self.process_period_sec:
            return False

        self.last_process_time_ns = now_ns
        return True

    def should_log(self):
        now_ns = self.get_clock().now().nanoseconds

        if self.last_log_time_ns is None:
            self.last_log_time_ns = now_ns
            return True

        elapsed_sec = (now_ns - self.last_log_time_ns) / 1e9
        if elapsed_sec < self.log_period_sec:
            return False

        self.last_log_time_ns = now_ns
        return True

    def frame_age_ms(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            return None

        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        if age_ns < 0:
            return None
        return age_ns / 1e6

    @staticmethod
    def _round(value, ndigits=3):
        # CSV-friendly: round floats, blank for missing values.
        if value is None:
            return ""
        return round(float(value), ndigits)

    def stop_pipeline(self):
        # Stop the worker thread before flushing metrics / killing the worker
        # process, so metrics_rows is no longer being appended.
        self.shutdown_event.set()
        self.frame_event.set()  # wake the loop if it is waiting
        if self.worker_thread is not None:
            self.worker_thread.join(timeout=3.0)
            self.worker_thread = None

    def save_metrics_csv(self):
        if not self.metrics_csv_path or not self.metrics_rows:
            return
        fieldnames = [
            "stamp_sec", "stamp_nanosec", "count", "image_w", "image_h",
            "processing_ms", "detect_ms", "img_ms", "worker_call_ms",
            "ipc_overhead_ms", "pre_ms", "dpu_ms", "post_ms", "worker_ms",
            "detection_publish_ms", "overlay_ms", "age_in_ms", "frame_age_ms",
        ]
        try:
            directory = os.path.dirname(self.metrics_csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(
                self.metrics_csv_path, "w", newline="", encoding="utf-8"
            ) as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.metrics_rows)
            self.get_logger().info(
                f"Saved {len(self.metrics_rows)} metric rows to "
                f"{self.metrics_csv_path}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to save metrics CSV: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None

    # Make SIGTERM (e.g. pkill) behave like Ctrl-C so the finally block runs
    # and the metrics CSV is still written on shutdown.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        node = VitisAiDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_pipeline()
            node.save_metrics_csv()
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
