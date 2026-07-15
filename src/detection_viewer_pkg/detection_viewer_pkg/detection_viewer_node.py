#!/usr/bin/env python3
"""Desktop-side bbox overlay viewer for the KV260 perception pipeline.

Runs on the DESKTOP, not the board. Subscribes to the board's compressed color
stream and /detections, joins them by exact header stamp, draws the boxes, and
shows the result. The board only compresses; all drawing happens here.

Why not draw on the board: the detector's own overlay path costs ~44 ms/frame on
top of a 37.6 ms detect, which busts the 66.6 ms budget for 15 Hz. Drawing here
costs the board nothing.

Wire contract (verified against the board 2026-07-16, see
docs/vision/desktop_viewer_plan.md):
  - Detection bbox coords are pixels in the ORIGINAL color frame, center+size.
    The worker already undoes the 416x416 letterbox, so do NOT rescale here.
  - DetectionArray.header is a byte copy of the source Image header, so an
    exact (sec, nanosec) join is valid.
  - /detections is RELIABLE; the compressed image inherits SYSTEM_DEFAULT
    (= RELIABLE), and a BEST_EFFORT subscriber is compatible with it.
"""

import argparse
import sys
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from my_interfaces.msg import DetectionArray

# Keyed by class_name, not class_id: the string ships in every Detection, and
# keying on the id would silently mis-label if decode_meta.json's order ever
# changed. Do NOT copy vitis_ai_detector_node.BOX_COLORS — it still holds the
# retired SSD classes {car, bicycle, person} and intersects the current 6-class
# model in exactly zero entries.
CLASS_COLORS = {
    "apple": (60, 60, 220),
    "orange": (0, 150, 255),
    "banana": (60, 220, 220),
    "tennis_ball": (60, 220, 60),
    "mustard_bottle": (200, 140, 60),
    "person": (220, 120, 220),
}
UNKNOWN_COLOR = (200, 200, 200)


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


class DetectionViewer(Node):
    def __init__(self, image_topic, detections_topic, buffer_len, window):
        super().__init__("detection_viewer_node")
        self.window = window
        self.buffer_len = buffer_len
        # stamp -> decoded BGR frame. OrderedDict as a bounded FIFO: frames
        # arrive at ~2x the detection rate, so the join target is always recent.
        self.frames = OrderedDict()

        # BEST_EFFORT on the image: the publisher is RELIABLE, which is
        # compatible, and asking for RELIABLE over the network would invite
        # retransmissions and latency for data we are about to overwrite anyway.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Must match the publisher's RELIABLE — a BEST_EFFORT subscriber would
        # also match, but detections are small and we want every one of them.
        det_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CompressedImage, image_topic, self.on_image, image_qos)
        self.create_subscription(DetectionArray, detections_topic, self.on_detections, det_qos)

        self.n_img = 0
        self.n_det = 0
        self.n_hit = 0
        self.n_miss = 0
        self.create_timer(5.0, self.report)

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"image:      {image_topic}")
        self.get_logger().info(f"detections: {detections_topic}")
        self.get_logger().info("waiting for both topics... (q or ESC in the window to quit)")

    def on_image(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # always BGR, whatever was encoded
        if frame is None:
            self.get_logger().warn("imdecode failed", throttle_duration_sec=5.0)
            return
        self.frames[stamp_key(msg.header)] = frame
        while len(self.frames) > self.buffer_len:
            self.frames.popitem(last=False)
        self.n_img += 1

    def on_detections(self, msg):
        self.n_det += 1
        key = stamp_key(msg.header)
        frame = self.frames.get(key)
        if frame is None:
            # Not an error per se: the detector may have processed a frame whose
            # compressed twin we dropped, or we joined the graph mid-stream.
            # A persistently high miss rate means the join is broken, not lossy.
            self.n_miss += 1
            return
        self.n_hit += 1
        self.render(frame.copy(), msg)

    def render(self, frame, msg):
        for det in msg.detections:
            # Coords are already in this frame's pixel space — no scaling.
            xmin = int(round(det.center_x - det.width / 2.0))
            ymin = int(round(det.center_y - det.height / 2.0))
            xmax = int(round(det.center_x + det.width / 2.0))
            ymax = int(round(det.center_y + det.height / 2.0))
            color = CLASS_COLORS.get(det.class_name, UNKNOWN_COLOR)
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ytop = max(0, ymin - th - 4)
            cv2.rectangle(frame, (xmin, ytop), (xmin + tw + 4, ytop + th + 4), color, -1)
            cv2.putText(frame, label, (xmin + 2, ytop + th),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        hud = f"{len(msg.detections)} det  {frame.shape[1]}x{frame.shape[0]}"
        cv2.putText(frame, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.window, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()

    def report(self):
        if self.n_img == 0 and self.n_det == 0:
            self.get_logger().warn(
                "no images AND no detections — check discovery: "
                "ros2 topic list --no-daemon --spin-time 8")
            return
        if self.n_img == 0:
            self.get_logger().warn("detections but NO images — is the compressed plugin "
                                   "installed and the camera restarted?")
        elif self.n_det == 0:
            self.get_logger().warn("images but NO detections — is my_interfaces built from "
                                   "the SAME commit as the board?")
        total = self.n_hit + self.n_miss
        rate = (100.0 * self.n_hit / total) if total else 0.0
        self.get_logger().info(
            f"img={self.n_img} det={self.n_det} join_hit={self.n_hit} "
            f"miss={self.n_miss} ({rate:.0f}% joined)")
        self.n_img = self.n_det = self.n_hit = self.n_miss = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/camera/camera/color/image_raw/compressed")
    ap.add_argument("--detections-topic", default="/detections")
    ap.add_argument("--buffer", type=int, default=30)
    ap.add_argument("--window", default="KV260 detections")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = DetectionViewer(args.image_topic, args.detections_topic, args.buffer, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
