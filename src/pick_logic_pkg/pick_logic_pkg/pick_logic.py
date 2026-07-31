import math
import signal
import time

import rclpy
from rclpy.node import Node

from my_interfaces.msg import Detection, DetectionArray, PickTarget
from pick_logic_pkg.metrics_recorder import (
    MetricsRecorder,
    age_ms,
    write_summary_csv,
    _round,
)


class PickLogicNode(Node):
    """Pick-target selection, v2 (2026-07-26).

    v1 picked the FIRST detection that passed the gates (decode order =
    arbitrary) and was stateless across frames. v2 keeps the same gates but
    adds, in order:

      L4 person guard   — a NEAR person (bbox area over threshold) invalidates
                          the target and latches for person_clear_sec.
      L1 gates          — unchanged: confidence / class / edge / area.
      L2 selection      — ALL passing candidates are scored
                          (w_conf·conf + w_center·center-proximity + w_area·size)
                          and the best wins. `desired_class` ('' = any allowed)
                          restricts to one class; it is a LIVE parameter — the
                          robot-side operator menu sets it at runtime
                          (`ros2 param set /pick_logic_node desired_class apple`).
      L3 stability      — the winner must persist stability_min_frames
                          (class + center-distance match) before target_valid;
                          a different candidate must beat the current target by
                          hysteresis_margin for hysteresis_frames to take over
                          (no per-frame flapping); the target survives
                          lost_timeout_frames of absence before being dropped.

    Output contract is unchanged: one PickTarget per DetectionArray,
    target_valid true/false (invalid publishing controlled by
    publish_invalid_targets). New reject reasons: person_guard,
    desired_class_not_visible, target_unstable.
    """

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

        # --- v2: selection / stability / safety ---
        self.declare_parameter('desired_class', '')       # LIVE: '' = auto-best
        self.declare_parameter('score_w_conf', 0.5)
        self.declare_parameter('score_w_center', 0.3)
        self.declare_parameter('score_w_area', 0.2)

        self.declare_parameter('stability_min_frames', 3)
        self.declare_parameter('stability_match_px', 60.0)
        self.declare_parameter('lost_timeout_frames', 5)
        self.declare_parameter('hysteresis_margin', 0.08)
        self.declare_parameter('hysteresis_frames', 3)

        self.declare_parameter('person_guard_enable', True)
        self.declare_parameter('person_class_name', 'person')
        self.declare_parameter('person_min_confidence', 0.3)
        self.declare_parameter('person_area_ratio_threshold', 0.06)
        self.declare_parameter('person_clear_sec', 2.0)

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)

        # Metrics (off by default). Performance CSV = one row per DetectionArray;
        # yield CSV = one reject-tally row at shutdown. Both write once, no
        # per-frame I/O, exactly like vitis_ai_detector_node.
        self.declare_parameter('metrics_csv_path', '')
        self.declare_parameter('metrics_duration_sec', 0.0)
        self.declare_parameter('yield_csv_path', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.allowed_classes = list(self.get_parameter('allowed_classes').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.edge_margin_px = int(self.get_parameter('edge_margin_px').value)

        self.min_bbox_area_px = float(self.get_parameter('min_bbox_area_px').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)

        self.score_w_conf = float(self.get_parameter('score_w_conf').value)
        self.score_w_center = float(self.get_parameter('score_w_center').value)
        self.score_w_area = float(self.get_parameter('score_w_area').value)

        self.stability_min_frames = int(self.get_parameter('stability_min_frames').value)
        self.stability_match_px = float(self.get_parameter('stability_match_px').value)
        self.lost_timeout_frames = int(self.get_parameter('lost_timeout_frames').value)
        self.hysteresis_margin = float(self.get_parameter('hysteresis_margin').value)
        self.hysteresis_frames = int(self.get_parameter('hysteresis_frames').value)

        self.person_guard_enable = bool(self.get_parameter('person_guard_enable').value)
        self.person_class_name = str(self.get_parameter('person_class_name').value)
        self.person_min_confidence = float(self.get_parameter('person_min_confidence').value)
        self.person_area_ratio_threshold = float(
            self.get_parameter('person_area_ratio_threshold').value)
        self.person_clear_sec = float(self.get_parameter('person_clear_sec').value)

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.last_accept_log_time_ns = None
        self.last_reject_log_time_ns = None

        # --- v2 runtime state ---
        self._desired_class_cached = str(self.get_parameter('desired_class').value)
        self._person_guard_until_ns = 0
        # Current target track: dict(class_name, cx, cy, score,
        #                            stable, missing) or None
        self._track = None
        self._challenger_class = None
        self._challenger_frames = 0

        self.metrics = MetricsRecorder(
            self.get_logger(),
            str(self.get_parameter('metrics_csv_path').value),
            float(self.get_parameter('metrics_duration_sec').value),
        )
        self.yield_csv_path = str(self.get_parameter('yield_csv_path').value)
        self.frames_total = 0
        self.frames_accepted = 0
        self.frames_empty = 0
        self.reject_tally = {}

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
            f'Pick logic node (v2) started: {input_topic} -> {output_topic}'
        )

    # ------------------------------------------------------------------ #
    # main callback
    # ------------------------------------------------------------------ #

    def detections_callback(self, msg: DetectionArray):
        compute_start_ns = time.perf_counter_ns()
        # DetectionArray.header.stamp IS the original color capture time (the
        # detector propagates it). Carry it into every PickTarget so downstream
        # nodes can compute true E2E latency.
        capture_stamp = msg.header.stamp
        in_age = (age_ms(self.get_clock().now().nanoseconds, capture_stamp)
                  if self.metrics.enabled else None)
        self.frames_total += 1

        # desired_class is a live parameter (set by the robot-side operator
        # menu). Re-read every frame; reset tracking state on change.
        desired = str(self.get_parameter('desired_class').value)
        if desired != self._desired_class_cached:
            self.get_logger().info(
                f"desired_class: '{self._desired_class_cached}' -> '{desired}'"
                f' — tracking state reset')
            self._desired_class_cached = desired
            self._reset_track()

        # L4: near-person safety latch (scanned over the RAW detections, before
        # any gate — person is not in allowed_classes on purpose).
        now_ns = self.get_clock().now().nanoseconds
        if self.person_guard_enable and self._scan_person(msg.detections):
            self._person_guard_until_ns = now_ns + int(self.person_clear_sec * 1e9)
        if now_ns < self._person_guard_until_ns:
            self._reset_track()
            self._finish_invalid('person_guard', capture_stamp,
                                 compute_start_ns, in_age, len(msg.detections))
            return

        if len(msg.detections) == 0:
            self.frames_empty += 1
            self._step_track_missing()
            self._finish_invalid('empty_detection_array', capture_stamp,
                                 compute_start_ns, in_age, 0)
            return

        # L1 gates -> candidate list (optionally restricted to desired_class)
        candidates = []
        first_reject_reason = None
        first_rejected_detection = None
        for det in msg.detections:
            accepted, reason = self.is_detection_acceptable(det)
            if accepted and desired and det.class_name != desired:
                accepted, reason = False, 'desired_class_not_visible'
            if accepted:
                candidates.append(det)
            elif first_reject_reason is None:
                first_reject_reason = reason
                first_rejected_detection = det

        if not candidates:
            self._step_track_missing()
            reason = first_reject_reason or 'no_acceptable_detection'
            self._finish_invalid(reason, capture_stamp, compute_start_ns,
                                 in_age, len(msg.detections),
                                 first_rejected_detection)
            return

        # L2 scoring
        scored = [(self._score(det), det) for det in candidates]
        scored.sort(key=lambda t: t[0], reverse=True)

        # L3 stability + hysteresis
        det = self._update_track(scored)
        if det is None or self._track['stable'] < self.stability_min_frames:
            self._finish_invalid('target_unstable', capture_stamp,
                                 compute_start_ns, in_age,
                                 len(msg.detections),
                                 det if det is not None else scored[0][1])
            return

        target = self.make_pick_target(det)
        target.target_valid = True
        target.capture_stamp = capture_stamp
        self.publisher.publish(target)
        self.frames_accepted += 1

        if self.should_log_accept():
            self.get_logger().info(
                f'Accepted target: class={target.class_name}, '
                f'conf={target.confidence:.2f}, '
                f'cx={target.center_x:.1f}, cy={target.center_y:.1f}, '
                f'score={self._track["score"]:.3f}, '
                f'stable={self._track["stable"]}, '
                f'candidates={len(msg.detections)}'
                + (f', desired={desired}' if desired else '')
            )
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=len(msg.detections),
                             target_valid=True)

    # ------------------------------------------------------------------ #
    # v2 helpers
    # ------------------------------------------------------------------ #

    def _scan_person(self, detections):
        image_area = float(self.image_width * self.image_height)
        for det in detections:
            if det.class_name != self.person_class_name:
                continue
            if det.confidence < self.person_min_confidence:
                continue
            area_ratio = float(det.width * det.height) / image_area
            if area_ratio >= self.person_area_ratio_threshold:
                if self.should_log_reject():
                    self.get_logger().warn(
                        f'person_guard: person at area_ratio='
                        f'{area_ratio:.3f} (>= {self.person_area_ratio_threshold})'
                        f' — targets latched invalid for {self.person_clear_sec}s')
                return True
        return False

    def _score(self, det: Detection):
        # confidence term
        s = self.score_w_conf * float(det.confidence)
        # center-proximity term: 1 at image center, 0 at the corner
        dx = (float(det.center_x) - self.image_width / 2.0)
        dy = (float(det.center_y) - self.image_height / 2.0)
        half_diag = math.hypot(self.image_width / 2.0, self.image_height / 2.0)
        s += self.score_w_center * (1.0 - min(math.hypot(dx, dy) / half_diag, 1.0))
        # size term: normalized against the max allowed bbox area
        max_area = float(self.image_width * self.image_height) * self.max_bbox_area_ratio
        s += self.score_w_area * min(float(det.width * det.height) / max_area, 1.0)
        return s

    def _reset_track(self):
        self._track = None
        self._challenger_class = None
        self._challenger_frames = 0

    def _step_track_missing(self):
        if self._track is None:
            return
        self._track['missing'] += 1
        if self._track['missing'] > self.lost_timeout_frames:
            self._reset_track()

    def _update_track(self, scored):
        """Consume the scored candidate list, update the persistent track,
        and return the Detection to publish this frame (or None)."""
        best_score, best = scored[0]

        # No current target: adopt the best candidate as provisional.
        if self._track is None:
            self._track = {'class_name': best.class_name,
                           'cx': float(best.center_x), 'cy': float(best.center_y),
                           'score': best_score, 'stable': 1, 'missing': 0}
            self._challenger_class = None
            self._challenger_frames = 0
            return best

        # Find the candidate that continues the current track.
        cont = None
        cont_score = 0.0
        for score, det in scored:
            if det.class_name != self._track['class_name']:
                continue
            d = math.hypot(float(det.center_x) - self._track['cx'],
                           float(det.center_y) - self._track['cy'])
            if d <= self.stability_match_px:
                cont, cont_score = det, score
                break

        if cont is None:
            # Current target absent this frame.
            self._step_track_missing()
            if self._track is None:      # timed out -> adopt best fresh
                return self._update_track(scored)
            return None                  # keep waiting (invalid this frame)

        # Track continues.
        self._track.update({'cx': float(cont.center_x),
                            'cy': float(cont.center_y),
                            'score': cont_score,
                            'stable': self._track['stable'] + 1,
                            'missing': 0})

        # Hysteresis: a different candidate must beat us clearly and repeatedly.
        if best is not cont and best_score > cont_score + self.hysteresis_margin:
            if self._challenger_class == best.class_name:
                self._challenger_frames += 1
            else:
                self._challenger_class = best.class_name
                self._challenger_frames = 1
            if self._challenger_frames >= self.hysteresis_frames:
                self.get_logger().info(
                    f'target switch: {self._track["class_name"]} -> '
                    f'{best.class_name} '
                    f'({best_score:.3f} > {cont_score:.3f} + {self.hysteresis_margin})')
                self._track = {'class_name': best.class_name,
                               'cx': float(best.center_x),
                               'cy': float(best.center_y),
                               'score': best_score, 'stable': 1, 'missing': 0}
                self._challenger_class = None
                self._challenger_frames = 0
                return best
        else:
            self._challenger_class = None
            self._challenger_frames = 0

        return cont

    def _finish_invalid(self, reason, capture_stamp, compute_start_ns,
                        in_age, n_candidates, det=None):
        self._tally(reason)
        self.publish_invalid_target(reason, capture_stamp, det)
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=n_candidates, target_valid=False)

    # ------------------------------------------------------------------ #
    # unchanged v1 plumbing below
    # ------------------------------------------------------------------ #

    def _tally(self, reason):
        self.reject_tally[reason] = self.reject_tally.get(reason, 0) + 1

    def _record_metrics(self, capture_stamp, compute_start_ns, in_age,
                        n_candidates, target_valid):
        if not self.metrics.enabled:
            return
        now_ns = self.get_clock().now().nanoseconds
        compute_ms = (time.perf_counter_ns() - compute_start_ns) / 1e6
        self.metrics.add({
            'capture_sec': capture_stamp.sec,
            'capture_nanosec': capture_stamp.nanosec,
            'pick_logic_in_age_ms': _round(in_age),
            'pick_logic_compute_ms': _round(compute_ms),
            'pick_logic_out_age_ms': _round(age_ms(now_ns, capture_stamp)),
            'n_candidates': n_candidates,
            'target_valid': int(target_valid),
        })

    def publish_invalid_target(self, reason, capture_stamp, det=None):
        target = PickTarget()
        target.target_valid = False
        target.capture_stamp = capture_stamp

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

    def write_yield_summary(self):
        if not self.yield_csv_path:
            return
        total = max(self.frames_total, 1)
        row = {
            'node': 'pick_logic',
            'frames_total': self.frames_total,
            'frames_accepted': self.frames_accepted,
            'target_valid_rate': round(self.frames_accepted / total, 4),
            'frames_empty': self.frames_empty,
        }
        for reason, count in sorted(self.reject_tally.items()):
            row[f'reject_{reason}'] = count
        write_summary_csv(self.get_logger(), self.yield_csv_path, [row])


def main(args=None):
    rclpy.init(args=args)
    node = PickLogicNode()

    # Make SIGTERM (launch escalation, pkill) behave like Ctrl-C so the finally
    # block still writes the metrics/yield CSVs on shutdown.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 2차 시그널 차단: 그룹 Ctrl-C + launch 자체 전파로 SIGINT가 한 번 더
        # 들어와 finally 안의 CSV 재저장을 도중 절단시켰다(2026-07-31 실증).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.metrics.save()
        node.write_yield_summary()
        node.destroy_node()
        # SIGINT 시 rclpy의 전역 signal handler가 이미 context를 shutdown 시켰으므로
        # 여기서 또 부르면 "rcl_shutdown already called" 예외가 난다. ok()로 가드.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
