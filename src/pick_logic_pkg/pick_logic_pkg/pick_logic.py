import rclpy
from rclpy.node import Node

from my_interfaces.msg import Detection, DetectionArray, PickTarget


class PickLogicNode(Node):
    def __init__(self):
        super().__init__('pick_logic_node')

        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/pick_target')

        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('allowed_classes', ['object'])

        self.declare_parameter('image_width', 848)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('edge_margin_px', 30)

        self.declare_parameter('min_bbox_area_px', 400.0)
        self.declare_parameter('max_bbox_area_ratio', 0.5)

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.allowed_classes = list(self.get_parameter('allowed_classes').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.edge_margin_px = int(self.get_parameter('edge_margin_px').value)

        self.min_bbox_area_px = float(self.get_parameter('min_bbox_area_px').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.last_accept_log_time_ns = None
        self.last_reject_log_time_ns = None

        self.subscription = self.create_subscription(
            DetectionArray,
            input_topic,
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            PickTarget,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Pick logic node started: {input_topic} -> {output_topic}'
        )

    def detections_callback(self, msg: DetectionArray):
        if len(msg.detections) == 0:
            self.publish_invalid_target('empty_detection_array')
            return

        first_reject_reason = None
        first_rejected_detection = None

        for det in msg.detections:
            accepted, reason = self.is_detection_acceptable(det)

            if accepted:
                target = self.make_pick_target(det)
                target.target_valid = True
                self.publisher.publish(target)

                if self.should_log_accept():
                    self.get_logger().info(
                        f'Accepted target: class={target.class_name}, '
                        f'conf={target.confidence:.2f}, '
                        f'cx={target.center_x:.1f}, cy={target.center_y:.1f}, '
                        f'w={target.width:.1f}, h={target.height:.1f}, '
                        f'candidates={len(msg.detections)}'
                    )
                return

            if first_reject_reason is None:
                first_reject_reason = reason
                first_rejected_detection = det

        self.publish_invalid_target(
            first_reject_reason or 'no_acceptable_detection',
            first_rejected_detection
        )

    def publish_invalid_target(self, reason, det=None):
        target = PickTarget()
        target.target_valid = False

        if det is not None:
            target.class_id = det.class_id
            target.class_name = det.class_name
            target.confidence = det.confidence
            target.center_x = det.center_x
            target.center_y = det.center_y
            target.width = det.width
            target.height = det.height

        if self.should_log_reject():
            self.get_logger().warn(
                f'Rejected detection array: reason={reason}'
            )

        if self.publish_invalid_targets:
            self.publisher.publish(target)

    def make_pick_target(self, msg: Detection):
        target = PickTarget()
        target.class_id = msg.class_id
        target.class_name = msg.class_name
        target.confidence = msg.confidence
        target.center_x = msg.center_x
        target.center_y = msg.center_y
        target.width = msg.width
        target.height = msg.height
        return target

    def is_detection_acceptable(self, msg: Detection):
        if msg.confidence < self.min_confidence:
            return False, 'confidence_below_threshold'

        if self.allowed_classes and msg.class_name not in self.allowed_classes:
            return False, 'class_not_allowed'

        if msg.width <= 0.0 or msg.height <= 0.0:
            return False, 'invalid_bbox_size'

        left = msg.center_x - msg.width / 2.0
        right = msg.center_x + msg.width / 2.0
        top = msg.center_y - msg.height / 2.0
        bottom = msg.center_y + msg.height / 2.0

        if left < self.edge_margin_px:
            return False, 'bbox_too_close_to_left_edge'

        if right > self.image_width - self.edge_margin_px:
            return False, 'bbox_too_close_to_right_edge'

        if top < self.edge_margin_px:
            return False, 'bbox_too_close_to_top_edge'

        if bottom > self.image_height - self.edge_margin_px:
            return False, 'bbox_too_close_to_bottom_edge'

        bbox_area = float(msg.width * msg.height)
        image_area = float(self.image_width * self.image_height)
        max_bbox_area = image_area * self.max_bbox_area_ratio

        if bbox_area < self.min_bbox_area_px:
            return False, 'bbox_area_too_small'

        if bbox_area > max_bbox_area:
            return False, 'bbox_area_too_large'

        return True, 'accepted'

    def should_log_accept(self):
        should_log, now_ns = self.should_log(self.last_accept_log_time_ns)
        if should_log:
            self.last_accept_log_time_ns = now_ns
        return should_log

    def should_log_reject(self):
        should_log, now_ns = self.should_log(self.last_reject_log_time_ns)
        if should_log:
            self.last_reject_log_time_ns = now_ns
        return should_log

    def should_log(self, last_log_time_ns):
        now_ns = self.get_clock().now().nanoseconds

        if last_log_time_ns is None:
            return True, now_ns

        elapsed_sec = (now_ns - last_log_time_ns) / 1e9
        return elapsed_sec >= self.log_period_sec, now_ns


def main(args=None):
    rclpy.init(args=args)
    node = PickLogicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
