import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from my_interfaces.msg import Detection, DetectionArray


class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')

        self.declare_parameter('log_period_sec', 1.0)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.last_detection_log_time_ns = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            '/detections',
            10
        )

        self.get_logger().info('Mock detector started')

    def image_callback(self, msg: Image):
        det = Detection()

        det.class_id = 0
        det.class_name = 'object'
        det.confidence = 0.90

        det.center_x = float(msg.width) / 2.0
        det.center_y = float(msg.height) / 2.0
        det.width = float(msg.width) / 4.0
        det.height = float(msg.height) / 4.0

        detections = DetectionArray()
        detections.header = msg.header
        detections.detections.append(det)

        self.publisher.publish(detections)

        if self.should_log_detection():
            self.get_logger().info(
                f'Published detection array: count={len(detections.detections)}, '
                f'class={det.class_name}, conf={det.confidence:.2f}, '
                f'cx={det.center_x:.1f}, cy={det.center_y:.1f}'
            )

    def should_log_detection(self):
        now_ns = self.get_clock().now().nanoseconds

        if self.last_detection_log_time_ns is None:
            self.last_detection_log_time_ns = now_ns
            return True

        elapsed_sec = (now_ns - self.last_detection_log_time_ns) / 1e9
        if elapsed_sec < self.log_period_sec:
            return False

        self.last_detection_log_time_ns = now_ns
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MockDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
