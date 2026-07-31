#!/usr/bin/env python3
"""Phase 1 operator console — OBSERVE ONLY.

This window issues no commands and cannot move the robot. It exists to put the
video, the detections and the base-frame target the robot actually consumes on
one screen, and to make a dead link obvious. Command widgets arrive in a later
phase once the board publishes state (today the app has no ROS 2 publisher at
all, so any button here would be a blind write-only remote).

Run this INSTEAD OF detection_viewer_node, not alongside it — two subscribers
double the board's JPEG egress for no benefit.
"""

import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .video_pane import VideoPane

# No frame for this long => the link is presumed dead. ~15 Hz nominal, so 1.5 s
# is ~22 missed frames: long enough not to flicker, short enough to notice.
LINK_TIMEOUT_S = 1.5

OK_STYLE = "color:#3ddc84; font-weight:bold;"
BAD_STYLE = "color:#ff5f56; font-weight:bold;"
DIM_STYLE = "color:#808088;"


class MainWindow(QMainWindow):
    def __init__(self, signals, topics):
        super().__init__()
        self.setWindowTitle("RAON-RT operator console — OBSERVE ONLY")
        self.resize(1280, 720)

        self.video = VideoPane()
        right = QWidget()
        right.setFixedWidth(360)
        rl = QVBoxLayout(right)
        rl.addWidget(self._build_link_box())
        rl.addWidget(self._build_detections_box())
        rl.addWidget(self._build_target_box())
        rl.addStretch(1)

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(self.video, 1)
        layout.addWidget(right, 0)
        self.setCentralWidget(central)
        self.statusBar().showMessage(
            f"image: {topics['image']}    detections: {topics['detections']}"
            f"    target: {topics['pick_target']}")

        # FPS is measured here, on the frames actually shown, so it reports what
        # the operator sees rather than what the network delivered.
        self._last_frame_t = None
        self._fps_ema = None
        self._last_frame_wall = 0.0

        signals.frame_ready.connect(self.on_frame, Qt.QueuedConnection)
        signals.detections_ready.connect(self.on_detections, Qt.QueuedConnection)
        signals.pick_target_ready.connect(self.on_pick_target, Qt.QueuedConnection)
        signals.stats_ready.connect(self.on_stats, Qt.QueuedConnection)

        self._watchdog = QTimer(self)
        self._watchdog.timeout.connect(self._check_link)
        self._watchdog.start(500)

    # ---------------- widgets ----------------

    def _build_link_box(self):
        box = QGroupBox("Link")
        g = QGridLayout(box)
        self.lbl_link = QLabel("NO SIGNAL")
        self.lbl_link.setStyleSheet(BAD_STYLE)
        self.lbl_fps = QLabel("–")
        self.lbl_rates = QLabel("–")
        self.lbl_join = QLabel("–")
        for row, (name, w) in enumerate([
            ("status", self.lbl_link), ("shown FPS", self.lbl_fps),
            ("img / det Hz", self.lbl_rates), ("join", self.lbl_join),
        ]):
            label = QLabel(name)
            label.setStyleSheet(DIM_STYLE)
            g.addWidget(label, row, 0)
            g.addWidget(w, row, 1)
        return box

    def _build_detections_box(self):
        box = QGroupBox("Detections")
        v = QVBoxLayout(box)
        self.tbl = QTableWidget(0, 3)
        self.tbl.setHorizontalHeaderLabels(["class", "conf", "center px"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tbl.setSelectionMode(QTableWidget.NoSelection)
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.tbl.setMinimumHeight(180)
        v.addWidget(self.tbl)
        return box

    def _build_target_box(self):
        # /pick_target_base is the message the robot app actually consumes, so
        # showing it is the difference between watching the camera and watching
        # what the robot will be told.
        box = QGroupBox("Pick target  (base_link — what the robot consumes)")
        g = QGridLayout(box)
        self.lbl_tgt_valid = QLabel("–")
        self.lbl_tgt_class = QLabel("–")
        self.lbl_tgt_xyz = QLabel("–")
        self.lbl_tgt_age = QLabel("–")
        for row, (name, w) in enumerate([
            ("valid", self.lbl_tgt_valid), ("class", self.lbl_tgt_class),
            ("x / y / z [m]", self.lbl_tgt_xyz), ("last update", self.lbl_tgt_age),
        ]):
            label = QLabel(name)
            label.setStyleSheet(DIM_STYLE)
            g.addWidget(label, row, 0)
            g.addWidget(w, row, 1)
        self._last_target_wall = None
        return box

    # ---------------- slots (GUI thread) ----------------

    def on_frame(self, bgr):
        self.video.set_frame(bgr)
        now = time.monotonic()
        if self._last_frame_t is not None:
            dt = now - self._last_frame_t
            if dt > 0.0:
                inst = 1.0 / dt
                self._fps_ema = inst if self._fps_ema is None else 0.2 * inst + 0.8 * self._fps_ema
                self.lbl_fps.setText(f"{self._fps_ema:.1f}")
        self._last_frame_t = now
        self._last_frame_wall = now

    def on_detections(self, msg):
        self.tbl.setRowCount(len(msg.detections))
        for row, det in enumerate(msg.detections):
            self.tbl.setItem(row, 0, QTableWidgetItem(det.class_name))
            self.tbl.setItem(row, 1, QTableWidgetItem(f"{det.confidence:.2f}"))
            self.tbl.setItem(
                row, 2, QTableWidgetItem(f"{det.center_x:.0f}, {det.center_y:.0f}"))

    def on_pick_target(self, msg):
        valid = bool(msg.target_valid) and bool(msg.depth_valid)
        self.lbl_tgt_valid.setText("LOCKED" if valid else "no target")
        self.lbl_tgt_valid.setStyleSheet(OK_STYLE if valid else DIM_STYLE)
        self.lbl_tgt_class.setText(msg.class_name or "–")
        self.lbl_tgt_xyz.setText(f"{msg.x:+.3f}  {msg.y:+.3f}  {msg.z:+.3f}")
        self._last_target_wall = time.monotonic()

    def on_stats(self, s):
        self.lbl_rates.setText(f"{s['img_hz']} / {s['det_hz']}")
        # drop>0 means the link lost data; stale>0 means frames arrived out of
        # order. Both are link health, not viewer bugs.
        self.lbl_join.setText(
            f"{s['join_pct']:.0f}%  (drop {s['drop']}, stale {s['stale']})")
        self.lbl_join.setStyleSheet(OK_STYLE if s["drop"] == 0 else BAD_STYLE)

    def _check_link(self):
        alive = (time.monotonic() - self._last_frame_wall) < LINK_TIMEOUT_S
        self.lbl_link.setText("LIVE" if alive else "NO SIGNAL")
        self.lbl_link.setStyleSheet(OK_STYLE if alive else BAD_STYLE)
        if not alive:
            self.lbl_fps.setText("–")
        if self._last_target_wall is not None:
            self.lbl_tgt_age.setText(f"{time.monotonic() - self._last_target_wall:.1f} s ago")
