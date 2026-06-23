import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('topic_name', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
       # self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device: {device_id}')
            raise RuntimeError(f'Cannot open camera device {device_id}')

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Camera publisher started. device_id={device_id}, '
            f'topic={topic_name}, size={width}x{height}, fps={fps}'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
