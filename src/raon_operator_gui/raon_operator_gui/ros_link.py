#!/usr/bin/env python3
"""ROS 2 side of the operator console — runs OFF the Qt GUI thread.

Everything expensive (JPEG decode, the stamp join, the bbox drawing) happens in
an executor thread and only the finished BGR frame crosses into the GUI thread
via a queued Qt signal. Qt widgets are never touched from here — that is the one
rule that makes this safe.

The join, the QoS and the coordinate convention are lifted verbatim from
detection_viewer_pkg/detection_viewer_node.py, which has field evidence behind
it (100% join, 2026-07-16..22). See docs/vision/desktop_viewer_plan.md §4.
Do NOT "simplify" the two-sided join: the two streams race, either can arrive
first, and a one-sided join silently drops 1-5% of frames.
"""

from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from my_interfaces.msg import DetectionArray, PickTarget3D

# Keyed by class_name, never class_id: the id ordering comes from
# decode_meta.json and would silently mis-label if it ever changed.
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


class RosSignals(QObject):
    """The only bridge from ROS threads to the GUI thread.

    Qt delivers these across threads as queued connections, so the slots run on
    the GUI thread even though emit() is called from an executor thread.
    """

    frame_ready = pyqtSignal(object)        # drawn BGR ndarray
    detections_ready = pyqtSignal(object)   # DetectionArray
    pick_target_ready = pyqtSignal(object)  # PickTarget3D
    stats_ready = pyqtSignal(object)        # dict


class RosLink(Node):
    def __init__(self, image_topic, detections_topic, pick_target_topic, buffer_len=30):
        super().__init__("raon_operator_gui")
        self.signals = RosSignals()
        self.buffer_len = buffer_len

        # Two-sided join state (see module docstring).
        self.frames = OrderedDict()   # stamp -> decoded BGR   ("image arrived first")
        self.pending = OrderedDict()  # stamp -> DetectionArray ("detection arrived first")
        self.pending_len = 10
        self.last_rendered = None

        # Displayed FPS = how often a joined pair is actually produced. Gated by
        # /detections (~15 Hz), not by the 30 Hz image stream.
        self.last_render_t = None
        self.fps_ema = None

        # BEST_EFFORT on the image: the publisher is RELIABLE (compatible), and
        # retransmitting a frame we are about to overwrite only adds latency.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # RELIABLE to match the publishers — detections and targets are tiny and
        # we want every one of them.
        small_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CompressedImage, image_topic, self.on_image, image_qos)
        self.create_subscription(DetectionArray, detections_topic, self.on_detections, small_qos)
        self.create_subscription(PickTarget3D, pick_target_topic, self.on_pick_target, small_qos)

        self.n_img = self.n_det = 0
        self.n_hit = self.n_late = self.n_drop = self.n_stale = 0
        self.create_timer(1.0, self.emit_stats)

    # ---------------- subscriptions ----------------

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("imdecode failed", throttle_duration_sec=5.0)
            return
        key = stamp_key(msg.header)
        self.frames[key] = frame
        while len(self.frames) > self.buffer_len:
            self.frames.popitem(last=False)
        self.n_img += 1

        waiting = self.pending.pop(key, None)
        if waiting is not None:
            self.n_late += 1
            self.try_render(key, frame, waiting)

    def on_detections(self, msg):
        self.n_det += 1
        self.signals.detections_ready.emit(msg)
        key = stamp_key(msg.header)
        frame = self.frames.get(key)
        if frame is None:
            # Not a miss — the image may still be in flight. Only aging out of
            # this queue is a real drop.
            self.pending[key] = msg
            while len(self.pending) > self.pending_len:
                self.pending.popitem(last=False)
                self.n_drop += 1
            return
        self.n_hit += 1
        self.try_render(key, frame, msg)

    def on_pick_target(self, msg):
        self.signals.pick_target_ready.emit(msg)

    # ---------------- render ----------------

    def try_render(self, key, frame, msg):
        # Monotonic guard: a pathologically late image must not redraw an older
        # frame over a newer one, which reads as a stutter.
        if self.last_rendered is not None and key < self.last_rendered:
            self.n_stale += 1
            return
        self.last_rendered = key
        self.signals.frame_ready.emit(self.draw(frame.copy(), msg))

    def draw(self, frame, msg):
        for det in msg.detections:
            # Coords are already pixels in THIS frame — the worker undid the
            # 416x416 letterbox. Never rescale here.
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
        return frame

    # ---------------- stats ----------------

    def emit_stats(self):
        # Counter semantics are kept identical to detection_viewer_node so the
        # two tools' logs stay comparable. Note a stale pair is counted BOTH in
        # hit/late (its image was found) and in stale (it was then skipped), so
        # join_pct reads slightly low whenever stale > 0 — which has never
        # happened in the field. Not a bug; do not "fix" it into a divergence.
        drawn = self.n_hit + self.n_late
        total = drawn + self.n_drop + self.n_stale
        self.signals.stats_ready.emit({
            "img_hz": self.n_img, "det_hz": self.n_det,
            "hit": self.n_hit, "late": self.n_late,
            "drop": self.n_drop, "stale": self.n_stale,
            "join_pct": (100.0 * drawn / total) if total else 0.0,
            "pending": len(self.pending),
        })
        self.n_img = self.n_det = 0
        self.n_hit = self.n_late = self.n_drop = self.n_stale = 0


class RclpySpinner(QThread):
    """MultiThreadedExecutor inside a QThread — rqt's own pattern
    (rqt_gui_py/rclpy_spinner.py).

    Do NOT replace this with a QTimer calling spin_once(): a SingleThreaded
    executor runs at most ONE callback per call, and this console needs ~45/s
    (30 Hz images + 15 Hz detections + targets).
    """

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._abort = False

    def run(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self._node)
        while rclpy.ok() and not self._abort:
            executor.spin_once(timeout_sec=0.2)
        executor.remove_node(self._node)

    def quit(self):
        self._abort = True
        super().quit()
