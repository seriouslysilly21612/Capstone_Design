import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point

from my_interfaces.msg import PickTarget3D


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Unit quaternion -> 3x3 rotation matrix (same math as tf2)."""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


class PickTargetBaseNode(Node):
    def __init__(self):
        super().__init__('pick_target_base_node')

        self.declare_parameter('input_topic', '/pick_target_3d')
        self.declare_parameter('output_topic', '/pick_target_base')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('transform_timeout_sec', 0.2)

        self.declare_parameter('require_depth_valid', True)
        self.declare_parameter('min_camera_z_m', 0.20)
        self.declare_parameter('max_camera_z_m', 1.50)

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)
        # base_link->camera TF is static (placeholder / fixed calibration):
        # look it up once, cache (R, t), and transform with one matmul instead
        # of a tf2 buffer lookup + do_transform_point per message. Set false
        # if the TF ever becomes dynamic.
        self.declare_parameter('static_tf_cache', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.target_frame = self.get_parameter('target_frame').value
        self.transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value
        )

        self.require_depth_valid = bool(
            self.get_parameter('require_depth_valid').value
        )
        self.min_camera_z_m = float(
            self.get_parameter('min_camera_z_m').value
        )
        self.max_camera_z_m = float(
            self.get_parameter('max_camera_z_m').value
        )

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(
            self.get_parameter('log_period_sec').value
        )
        self.static_tf_cache = bool(
            self.get_parameter('static_tf_cache').value
        )
        self._tf_cache = {}   # source frame_id -> (R 3x3, t 3)
        self.last_warn_log_time_ns = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PickTarget3D,
            input_topic,
            self.target_callback,
            10
        )

        self.pub = self.create_publisher(
            PickTarget3D,
            output_topic,
            10
        )

        self.get_logger().info(
            f'pick_target_base_node started: {input_topic} -> {output_topic}, '
            f'target_frame={self.target_frame}'
        )
        self.get_logger().info(
            f'Filters: require_depth_valid={self.require_depth_valid}, '
            f'camera_z_range=[{self.min_camera_z_m:.2f}, '
            f'{self.max_camera_z_m:.2f}]'
        )

    def target_callback(self, msg: PickTarget3D):
        out = self.make_output_from_input(msg)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame

        accepted, reason = self.is_input_acceptable(msg)

        if not accepted:
            out.target_valid = False
            out.depth_valid = False
            self.log_warn_throttled(
                reason,
                f'Rejected 3D target before TF: reason={reason}, '
                f'input_frame={msg.header.frame_id}, '
                f'target_valid={msg.target_valid}, '
                f'depth_valid={msg.depth_valid}, z={msg.z:.3f}'
            )
            self.publish_invalid_if_enabled(out)
            return

        # Fast path: cached static (R, t) — no tf2 lookup, one matmul.
        cached = (
            self._tf_cache.get(msg.header.frame_id)
            if self.static_tf_cache else None
        )
        if cached is not None:
            R, t = cached
            p = R @ np.array(
                [msg.x, msg.y, msg.z], dtype=np.float64) + t
            out.target_valid = True
            out.depth_valid = True
            out.x = float(p[0])
            out.y = float(p[1])
            out.z = float(p[2])
            self.pub.publish(out)
            return

        point_in = PointStamped()
        point_in.header = msg.header
        point_in.point.x = float(msg.x)
        point_in.point.y = float(msg.y)
        point_in.point.z = float(msg.z)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.transform_timeout_sec)
            )

            point_out = do_transform_point(point_in, transform)

            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.target_frame
            out.target_valid = True
            out.depth_valid = True
            out.x = float(point_out.point.x)
            out.y = float(point_out.point.y)
            out.z = float(point_out.point.z)

            self.pub.publish(out)

            if self.static_tf_cache:
                q = transform.transform.rotation
                tr = transform.transform.translation
                R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
                t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
                p = R @ np.array(
                    [msg.x, msg.y, msg.z], dtype=np.float64) + t
                diff = max(
                    abs(p[0] - point_out.point.x),
                    abs(p[1] - point_out.point.y),
                    abs(p[2] - point_out.point.z),
                )
                self._tf_cache[msg.header.frame_id] = (R, t)
                self.get_logger().info(
                    f'Static TF cached ({msg.header.frame_id} -> '
                    f'{self.target_frame}), matmul vs tf2 max diff '
                    f'{diff:.3e} m'
                )

        except TransformException as e:
            out.target_valid = False
            out.depth_valid = False

            self.log_warn_throttled(
                'tf_transform_failed',
                f'Failed to transform target from {msg.header.frame_id} '
                f'to {self.target_frame}: {e}'
            )

            self.publish_invalid_if_enabled(out)

    def make_output_from_input(self, msg: PickTarget3D):
        out = PickTarget3D()

        out.target_valid = msg.target_valid
        out.depth_valid = msg.depth_valid

        out.class_id = msg.class_id
        out.class_name = msg.class_name
        out.confidence = msg.confidence

        out.center_x = msg.center_x
        out.center_y = msg.center_y
        out.width = msg.width
        out.height = msg.height

        out.x = 0.0
        out.y = 0.0
        out.z = 0.0

        return out

    def is_input_acceptable(self, msg: PickTarget3D):
        if not msg.target_valid:
            return False, 'target_not_valid'

        if self.require_depth_valid and not msg.depth_valid:
            return False, 'depth_not_valid'

        if msg.header.frame_id == '':
            return False, 'empty_input_frame'

        if msg.z < self.min_camera_z_m:
            return False, 'camera_z_too_close'

        if msg.z > self.max_camera_z_m:
            return False, 'camera_z_too_far'

        return True, 'accepted'

    def publish_invalid_if_enabled(self, msg: PickTarget3D):
        if self.publish_invalid_targets:
            self.pub.publish(msg)

    def log_warn_throttled(self, key, message):
        now_ns = self.get_clock().now().nanoseconds
        last_log_time_ns = self.last_warn_log_time_ns.get(key)

        if last_log_time_ns is not None:
            elapsed_sec = (now_ns - last_log_time_ns) / 1e9
            if elapsed_sec < self.log_period_sec:
                return

        self.last_warn_log_time_ns[key] = now_ns
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickTargetBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
