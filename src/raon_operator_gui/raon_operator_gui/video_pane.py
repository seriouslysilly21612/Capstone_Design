#!/usr/bin/env python3
"""The video widget. GUI-thread only."""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel


class VideoPane(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(424, 240)
        self.setStyleSheet("background-color: #101014; color: #808088;")
        self.setText("waiting for the compressed stream…")
        self._pixmap = None

    def set_frame(self, bgr):
        """Take an OpenCV BGR ndarray and show it.

        Format_BGR888 means NO channel swap — the JPEG payload really is BGR
        (compressed_image_transport converts to bgr8 before encoding even though
        the Image encoding field says rgb8). Adding a cvtColor here would turn
        the orange blue; that was verified in the field.

        strides[0] is passed explicitly because a cropped/non-contiguous frame
        would otherwise be read with the wrong pitch, and .copy() detaches the
        QImage from the numpy buffer, which Qt would otherwise outlive.
        """
        h, w = bgr.shape[:2]
        image = QImage(bgr.data, w, h, bgr.strides[0], QImage.Format_BGR888).copy()
        self._pixmap = QPixmap.fromImage(image)
        self._rescale()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self):
        if self._pixmap is None:
            return
        # Scale for display only — never draw on a resized frame. The boxes were
        # computed in 848x480 space and the 1.767 aspect makes a single uniform
        # scale wrong for y.
        self.setPixmap(self._pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
