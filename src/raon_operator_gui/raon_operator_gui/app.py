#!/usr/bin/env python3
"""Entry point: start Qt on the main thread, rclpy on a QThread."""

import argparse
import signal
import sys

import rclpy
from PyQt5.QtWidgets import QApplication

from .main_window import MainWindow
from .ros_link import RclpySpinner, RosLink


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/camera/camera/color/image_raw/compressed")
    ap.add_argument("--detections-topic", default="/detections")
    ap.add_argument("--pick-target-topic", default="/pick_target_base")
    ap.add_argument("--buffer", type=int, default=30)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = RosLink(args.image_topic, args.detections_topic,
                   args.pick_target_topic, args.buffer)

    app = QApplication(sys.argv)
    # Qt swallows SIGINT while its event loop runs; restoring the default
    # handler makes Ctrl-C in the launching terminal work as expected.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    window = MainWindow(node.signals, {
        "image": args.image_topic,
        "detections": args.detections_topic,
        "pick_target": args.pick_target_topic,
    })
    window.show()

    spinner = RclpySpinner(node)
    spinner.start()

    try:
        rc = app.exec_()
    finally:
        spinner.quit()
        spinner.wait(2000)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
