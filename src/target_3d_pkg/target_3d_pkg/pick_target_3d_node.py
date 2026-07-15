import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import Extrinsics

from my_interfaces.msg import PickTarget, PickTarget3D


# ===== Pinhole helpers (no distortion: D435i color/depth are rectified, d=0) =====

def deproject(fx, fy, cx, cy, u, v, z):
    """Pixel (u, v) + depth z -> 3D point in that camera's optical frame."""
    return np.array(
        [(u - cx) * z / fx, (v - cy) * z / fy, z],
        dtype=np.float64,
    )


def project(fx, fy, cx, cy, point):
    """3D point in a camera's optical frame -> pixel (u, v)."""
    z = point[2]
    return point[0] * fx / z + cx, point[1] * fy / z + cy


def color_pixel_to_depth_pixel_loop(
    u_c, v_c, depth_img, depth_scale, dmin, dmax,
    d_fx, d_fy, d_cx, d_cy,
    c_fx, c_fy, c_cx, c_cy,
    R_cd, t_cd, R_dc, t_dc,
):
    """Reference implementation (per-pixel Python loop). Kept only for the
    epipolar_ab_check cross-validation of the vectorized version below."""
    # Segment endpoints in the depth image (color ray at dmin and dmax).
    p_min = R_cd @ deproject(c_fx, c_fy, c_cx, c_cy, u_c, v_c, dmin) + t_cd
    u_min, v_min = project(d_fx, d_fy, d_cx, d_cy, p_min)
    p_max = R_cd @ deproject(c_fx, c_fy, c_cx, c_cy, u_c, v_c, dmax) + t_cd
    u_max, v_max = project(d_fx, d_fy, d_cx, d_cy, p_max)

    h, w = depth_img.shape[:2]
    steps = max(1, int(math.ceil(max(abs(u_max - u_min), abs(v_max - v_min)))))

    best = None
    best_dist = -1.0
    for i in range(steps + 1):
        a = i / steps
        ui = int(round(u_min + (u_max - u_min) * a))
        vi = int(round(v_min + (v_max - v_min) * a))
        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            continue
        raw = depth_img[vi, ui]
        if raw == 0:
            continue
        z = float(raw) * depth_scale
        if not (dmin <= z <= dmax):
            continue
        # Re-project this depth pixel into the color image and compare.
        p_c = R_dc @ deproject(d_fx, d_fy, d_cx, d_cy, ui, vi, z) + t_dc
        if p_c[2] <= 0:
            continue
        u_chk, v_chk = project(c_fx, c_fy, c_cx, c_cy, p_c)
        dist = (u_chk - u_c) ** 2 + (v_chk - v_c) ** 2
        if best_dist < 0 or dist < best_dist:
            best_dist = dist
            best = (ui, vi)
    return best


def color_pixel_to_depth_pixel(
    u_c, v_c, depth_img, depth_scale, dmin, dmax,
    d_fx, d_fy, d_cx, d_cy,
    c_fx, c_fy, c_cx, c_cy,
    R_cd, t_cd, R_dc, t_dc,
):
    """Find the depth-image pixel that corresponds to color pixel (u_c, v_c).

    This is the rs2_project_color_pixel_to_depth_pixel algorithm: the color
    pixel back-projects to a ray; that ray maps to a line segment in the depth
    image (between dmin and dmax); the depth pixel on that segment whose own
    back-projection re-projects closest to (u_c, v_c) wins.

    Vectorized over the whole segment (single fancy-index gather + one (3,N)
    matmul). Numerically equivalent to the loop reference: np.rint matches
    int(round()) (both round-half-to-even), mask order preserves the candidate
    set, and argmin returns the first minimum like the loop's strict '<'.
    Returns (ui, vi) or None if no valid depth lies on the segment.
    """
    p_min = R_cd @ deproject(c_fx, c_fy, c_cx, c_cy, u_c, v_c, dmin) + t_cd
    u_min, v_min = project(d_fx, d_fy, d_cx, d_cy, p_min)
    p_max = R_cd @ deproject(c_fx, c_fy, c_cx, c_cy, u_c, v_c, dmax) + t_cd
    u_max, v_max = project(d_fx, d_fy, d_cx, d_cy, p_max)

    h, w = depth_img.shape[:2]
    steps = max(1, int(math.ceil(max(abs(u_max - u_min), abs(v_max - v_min)))))

    a = np.arange(steps + 1, dtype=np.float64) / steps
    ui = np.rint(u_min + (u_max - u_min) * a).astype(np.intp)
    vi = np.rint(v_min + (v_max - v_min) * a).astype(np.intp)

    ok = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    ui, vi = ui[ok], vi[ok]
    if ui.size == 0:
        return None

    raw = depth_img[vi, ui]
    z = raw.astype(np.float64) * depth_scale
    ok = (raw != 0) & (z >= dmin) & (z <= dmax)
    ui, vi, z = ui[ok], vi[ok], z[ok]
    if ui.size == 0:
        return None

    # Back-project all candidates at once ((ui - cx) * z / fx: same op order
    # as the scalar deproject) and map depth->color with one matmul.
    pts = np.empty((3, ui.size), dtype=np.float64)
    pts[0] = (ui - d_cx) * z / d_fx
    pts[1] = (vi - d_cy) * z / d_fy
    pts[2] = z
    p_c = R_dc @ pts + t_dc[:, None]

    ok = p_c[2] > 0
    if not ok.all():
        p_c = p_c[:, ok]
        ui, vi = ui[ok], vi[ok]
        if ui.size == 0:
            return None

    u_chk = p_c[0] * c_fx / p_c[2] + c_cx
    v_chk = p_c[1] * c_fy / p_c[2] + c_cy
    dist = (u_chk - u_c) ** 2 + (v_chk - v_c) ** 2
    j = int(np.argmin(dist))
    return int(ui[j]), int(vi[j])


class PickTarget3DNode(Node):
    def __init__(self):
        super().__init__('pick_target_3d_node')

        # ===== Parameters =====
        self.declare_parameter('pick_target_topic', '/pick_target')
        self.declare_parameter(
            'depth_topic', '/camera/camera/depth/image_rect_raw'
        )
        self.declare_parameter(
            'depth_info_topic', '/camera/camera/depth/camera_info'
        )
        self.declare_parameter(
            'color_info_topic', '/camera/camera/color/camera_info'
        )
        self.declare_parameter(
            'extrinsics_topic', '/camera/camera/extrinsics/depth_to_color'
        )
        self.declare_parameter('output_topic', '/pick_target_3d')

        self.declare_parameter('depth_scale_16uc1', 0.001)
        self.declare_parameter('patch_radius', 2)   # 2 -> 5x5 patch
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 2.00)
        self.declare_parameter('log_period_sec', 1.0)
        # Cross-check the vectorized epipolar search against the loop
        # reference on every pick (costs the old loop time again) — turn off
        # once live mismatch count stays 0.
        self.declare_parameter('epipolar_ab_check', False)

        pick_target_topic = self.get_parameter('pick_target_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        color_info_topic = self.get_parameter('color_info_topic').value
        extrinsics_topic = self.get_parameter('extrinsics_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.depth_scale_16uc1 = float(
            self.get_parameter('depth_scale_16uc1').value
        )
        self.patch_radius = int(self.get_parameter('patch_radius').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.epipolar_ab_check = bool(
            self.get_parameter('epipolar_ab_check').value
        )
        self.ab_checked = 0
        self.ab_mismatch = 0
        self.last_warn_log_time_ns = {}

        # ===== Runtime state =====
        # Depth arrives at stream rate but is consumed only at pick time —
        # store the raw msg and build a zero-copy numpy view lazily.
        self.latest_depth_msg = None
        self._depth_view_checked = False

        # depth intrinsics
        self.d_fx = self.d_fy = self.d_cx = self.d_cy = None
        self.depth_frame_id = ''
        # color intrinsics
        self.c_fx = self.c_fy = self.c_cx = self.c_cy = None
        # extrinsics: depth->color (R_dc, t_dc) and its inverse color->depth
        self.R_dc = None
        self.t_dc = None
        self.R_cd = None
        self.t_cd = None

        # ===== QoS =====
        # Depth image is sensor data (best effort).
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Extrinsics is published once (latched / transient_local).
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ===== ROS interfaces =====
        self.pick_target_sub = self.create_subscription(
            PickTarget, pick_target_topic, self.pick_target_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, sensor_qos
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo, depth_info_topic, self.depth_info_callback, 10
        )
        self.color_info_sub = self.create_subscription(
            CameraInfo, color_info_topic, self.color_info_callback, 10
        )
        self.extrinsics_sub = self.create_subscription(
            Extrinsics, extrinsics_topic, self.extrinsics_callback, latched_qos
        )

        self.target_3d_pub = self.create_publisher(
            PickTarget3D, output_topic, 10
        )

        # Prune the static-value subscriptions shortly after startup (P2).
        self.prune_timer = self.create_timer(1.0, self.prune_static_subs)

        self.get_logger().info(
            'pick_target_3d_node started (raw-depth reverse projection): '
            f'{pick_target_topic} + {depth_topic} -> {output_topic}'
        )

    def depth_callback(self, msg: Image):
        # Store only — conversion/validation happens at pick time (~16 Hz),
        # not at depth stream rate.
        self.latest_depth_msg = msg

    def depth_msg_to_view(self, msg):
        """Zero-copy numpy view of a depth Image msg + meters-per-unit scale.

        Equivalent to cv_bridge 'passthrough' (which is also a view when
        step == width*itemsize); D435i is little-endian (is_bigendian=0).
        Returns (depth_img, eff_scale) or (None, None) on bad encoding.
        """
        if msg.encoding == '16UC1':
            img = np.frombuffer(msg.data, dtype=np.dtype('<u2')).reshape(
                msg.height, msg.step // 2)[:, :msg.width]
            return img, self.depth_scale_16uc1
        if msg.encoding == '32FC1':
            img = np.frombuffer(msg.data, dtype=np.dtype('<f4')).reshape(
                msg.height, msg.step // 4)[:, :msg.width]
            return img, 1.0
        return None, None

    def depth_info_callback(self, msg: CameraInfo):
        self.d_fx = float(msg.k[0])
        self.d_fy = float(msg.k[4])
        self.d_cx = float(msg.k[2])
        self.d_cy = float(msg.k[5])
        self.depth_frame_id = msg.header.frame_id

    def color_info_callback(self, msg: CameraInfo):
        self.c_fx = float(msg.k[0])
        self.c_fy = float(msg.k[4])
        self.c_cx = float(msg.k[2])
        self.c_cy = float(msg.k[5])

    def extrinsics_callback(self, msg: Extrinsics):
        # rs2 extrinsics rotation is COLUMN-major: R[i, j] = rotation[i + 3*j].
        R = np.array(msg.rotation, dtype=np.float64).reshape((3, 3), order='F')
        t = np.array(msg.translation, dtype=np.float64)
        self.R_dc = R              # depth -> color
        self.t_dc = t
        self.R_cd = R.T            # color -> depth (inverse rotation)
        self.t_cd = -R.T @ t

    def prune_static_subs(self):
        """Drop camera_info/extrinsics subscriptions once their (static)
        values arrived — realsense republishes them per frame (30 Hz x2) and
        the per-message executor dispatch is pure waste after the first one.
        Runs on a timer so we never destroy a subscription from inside its
        own callback."""
        if self.depth_info_sub and self.d_fx is not None:
            self.destroy_subscription(self.depth_info_sub)
            self.depth_info_sub = None
        if self.color_info_sub and self.c_fx is not None:
            self.destroy_subscription(self.color_info_sub)
            self.color_info_sub = None
        if self.extrinsics_sub and self.R_dc is not None:
            self.destroy_subscription(self.extrinsics_sub)
            self.extrinsics_sub = None
        if (self.depth_info_sub is None and self.color_info_sub is None
                and self.extrinsics_sub is None):
            self.destroy_timer(self.prune_timer)
            self.prune_timer = None
            self.get_logger().info(
                'Static camera_info/extrinsics captured — subscriptions pruned'
            )

    def intrinsics_ready(self):
        return (
            None not in (self.d_fx, self.d_fy, self.d_cx, self.d_cy)
            and None not in (self.c_fx, self.c_fy, self.c_cx, self.c_cy)
        )

    def pick_target_callback(self, msg: PickTarget):
        out = PickTarget3D()

        # copy 2D info
        out.target_valid = msg.target_valid
        out.class_id = msg.class_id
        out.class_name = msg.class_name
        out.confidence = msg.confidence
        out.center_x = float(msg.center_x)
        out.center_y = float(msg.center_y)
        out.width = float(msg.width)
        out.height = float(msg.height)

        out.depth_valid = False
        out.x = 0.0
        out.y = 0.0
        out.z = 0.0

        if not msg.target_valid:
            self.target_3d_pub.publish(out)
            return

        depth_msg = self.latest_depth_msg
        if depth_msg is None:
            self.log_warn_throttled('no_depth_image', 'No depth image yet')
            self.target_3d_pub.publish(out)
            return

        if not self.intrinsics_ready():
            self.log_warn_throttled('no_intrinsics', 'No camera intrinsics yet')
            self.target_3d_pub.publish(out)
            return

        if self.R_dc is None:
            self.log_warn_throttled(
                'no_extrinsics', 'No depth->color extrinsics yet'
            )
            self.target_3d_pub.publish(out)
            return

        depth_img, eff_scale = self.depth_msg_to_view(depth_msg)
        if depth_img is None:
            self.log_warn_throttled(
                'bad_depth_encoding',
                f'Unsupported depth encoding: {depth_msg.encoding}',
            )
            self.target_3d_pub.publish(out)
            return

        if not self._depth_view_checked:
            self._depth_view_checked = True
            try:
                from cv_bridge import CvBridge
                ref = CvBridge().imgmsg_to_cv2(
                    depth_msg, desired_encoding='passthrough')
                self.get_logger().info(
                    'depth frombuffer view == cv_bridge: '
                    f'{bool(np.array_equal(ref, depth_img))} '
                    f'(bigendian={depth_msg.is_bigendian})'
                )
            except Exception as e:
                self.get_logger().warn(f'depth view check skipped: {e}')

        u_c = float(msg.center_x)
        v_c = float(msg.center_y)

        args = (
            u_c, v_c, depth_img, eff_scale,
            self.min_depth_m, self.max_depth_m,
            self.d_fx, self.d_fy, self.d_cx, self.d_cy,
            self.c_fx, self.c_fy, self.c_cx, self.c_cy,
            self.R_cd, self.t_cd, self.R_dc, self.t_dc,
        )
        found = color_pixel_to_depth_pixel(*args)

        if self.epipolar_ab_check:
            ref = color_pixel_to_depth_pixel_loop(*args)
            self.ab_checked += 1
            if ref != found:
                self.ab_mismatch += 1
                self.get_logger().warn(
                    f'epipolar A/B mismatch: vec={found} loop={ref} '
                    f'({self.ab_mismatch}/{self.ab_checked})'
                )
            elif self.ab_checked % 500 == 0:
                self.get_logger().info(
                    f'epipolar A/B: {self.ab_checked} checked, '
                    f'{self.ab_mismatch} mismatches'
                )

        if found is None:
            self.log_warn_throttled(
                'no_valid_depth', 'No valid depth for bbox center (reverse proj)'
            )
            self.target_3d_pub.publish(out)
            return

        ui, vi = found
        z_m = self.get_depth_m_from_patch(depth_img, ui, vi, depth_msg.encoding)
        if z_m is None:
            self.log_warn_throttled(
                'no_valid_depth_patch', 'No valid depth in patch around match'
            )
            self.target_3d_pub.publish(out)
            return

        # 3D point in the DEPTH optical frame.
        point = deproject(self.d_fx, self.d_fy, self.d_cx, self.d_cy, ui, vi, z_m)

        out.header = depth_msg.header
        if out.header.frame_id == '':
            out.header.frame_id = self.depth_frame_id

        out.depth_valid = True
        out.x = float(point[0])
        out.y = float(point[1])
        out.z = float(point[2])

        self.target_3d_pub.publish(out)

    def get_depth_m_from_patch(self, depth_img, u, v, encoding):
        r = self.patch_radius
        u0 = max(0, u - r)
        u1 = min(depth_img.shape[1] - 1, u + r)
        v0 = max(0, v - r)
        v1 = min(depth_img.shape[0] - 1, v + r)

        patch = depth_img[v0:v1 + 1, u0:u1 + 1]

        if encoding == '16UC1':
            valid = patch[patch > 0].astype(np.float32)
            if valid.size == 0:
                return None
            valid_m = valid * self.depth_scale_16uc1
        elif encoding == '32FC1':
            valid = patch[np.isfinite(patch) & (patch > 0)].astype(np.float32)
            if valid.size == 0:
                return None
            valid_m = valid
        else:
            return None

        z_m = float(np.median(valid_m))
        if z_m < self.min_depth_m or z_m > self.max_depth_m:
            return None
        return z_m

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
    node = PickTarget3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
