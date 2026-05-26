#!/usr/bin/env python3
"""X-axis odometry scale calibration helper for AGV Pro."""

import math
import os
import shlex
import statistics
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener

CONTROL_RATE_HZ = 20.0
MAX_SPEED_LIMIT = 0.30
TF_TIMEOUT_SEC = 0.5
STOP_REPEAT_COUNT = 5


class OdomLinearCalib(Node):
    """Run repeated X-axis odom tests and compute the final scale from cached samples."""

    def __init__(self):
        super().__init__('odom_linear_calib')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('start_test', False)
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.10)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('timeout', 30.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'idle'
        self.start_pose = None
        self.direction_sign = 1.0
        self.signed_target_distance = 1.0
        self.target_distance = 1.0
        self.command_speed = 0.10
        self.tolerance = 0.01
        self.odom_linear_scale_correction = 1.0
        self.timeout = 30.0
        self.start_time = None
        self.last_odom_distance = 0.0
        self.last_log_time = self.get_clock().now()

        self.samples = []
        self.samples_lock = threading.Lock()
        self.pending_sample = None

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self.on_timer)
        threading.Thread(target=self._stdin_loop, daemon=True).start()

        self.get_logger().info(
            'odom_linear_calib ready. Set params, set start_test:=true for each run, '
            'use positive test_distance for forward and negative for backward, '
            'then enter the measured ground error in cm after the robot stops. '
            'Enter 0 to finish and print the cached scale summary; enter any text to skip a verification run.'
        )

    def _stdin_loop(self):
        while True:
            line = sys.stdin.readline()
            if line == '':
                return
            self._handle_input_line(line.strip())

    def _handle_input_line(self, text):
        if not text:
            self.get_logger().info(
                'Input ignored. After a successful run enter ground error in cm '
                '(+over target along motion direction, -short), or enter 0 to finish.'
            )
            return

        try:
            value = float(text)
        except ValueError:
            self._skip_pending_sample(text)
            return

        if value == 0.0 and not text.startswith(('+', '-')):
            with self.samples_lock:
                had_pending_sample = self.pending_sample is not None
                self.pending_sample = None
                if self.state == 'awaiting_input':
                    self.state = 'idle'
            if had_pending_sample:
                self.get_logger().warn('Pending run was not recorded because finish input 0 was entered.')
            self._print_summary()
            return

        self._record_pending_sample(value)

    def on_timer(self):
        if self.state == 'running':
            self._run_test_step()
            return

        if self.state == 'awaiting_input':
            if self.get_parameter('start_test').value:
                self.get_logger().warn(
                    'A finished run is waiting for ground-error input; record it before starting again.'
                )
                self._reset_start_test()
            return

        if self.get_parameter('start_test').value:
            self._start_test()

    def _start_test(self):
        config = self._read_test_config()
        if config is None:
            self._reset_start_test()
            return

        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().warn('Cannot start test: odom transform is not available.')
            self._reset_start_test()
            return

        self.signed_target_distance = config['signed_test_distance']
        self.target_distance = config['target_distance']
        self.command_speed = config['speed']
        self.tolerance = config['tolerance']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.timeout = config['timeout']
        self.direction_sign = float(config['direction_sign'])
        self.start_pose = pose
        self.start_time = self.get_clock().now()
        self.last_odom_distance = 0.0
        self.state = 'running'

        self.get_logger().info(
            f'Start X odom calibration: direction={int(self.direction_sign)}, '
            f'signed_target={self.signed_target_distance:.3f} m, '
            f'target={self.target_distance:.3f} m, speed={self.command_speed:.3f} m/s, '
            f'odom_linear_scale_correction={self.odom_linear_scale_correction:.6f}'
        )

    def _run_test_step(self):
        pose = self._lookup_pose()
        if pose is None:
            self._finish_test('failed_tf', publish_warning=True)
            return

        raw_progress, lateral_drift = self._calculate_progress(pose)
        corrected_progress = raw_progress * self.odom_linear_scale_correction
        error = corrected_progress - self.target_distance
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.last_odom_distance = raw_progress

        if corrected_progress >= self.target_distance - self.tolerance:
            self._finish_test('succeeded', raw_progress, corrected_progress, lateral_drift, elapsed)
            return

        if elapsed > self.timeout:
            self._finish_test('timeout', raw_progress, corrected_progress, lateral_drift, elapsed)
            return

        cmd = Twist()
        cmd.linear.x = self.direction_sign * self.command_speed
        self.cmd_vel_pub.publish(cmd)
        self._log_progress(raw_progress, corrected_progress, error, lateral_drift, elapsed)

    def _finish_test(
        self,
        status,
        odom_distance=None,
        corrected_distance=None,
        lateral_drift=None,
        elapsed=None,
        publish_warning=False,
    ):
        self._stop_robot()
        self._reset_start_test()

        if odom_distance is None:
            odom_distance = self.last_odom_distance
        if corrected_distance is None:
            corrected_distance = odom_distance * self.odom_linear_scale_correction
        if lateral_drift is None:
            lateral_drift = 0.0
        if elapsed is None and self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed is None:
            elapsed = 0.0

        if status == 'succeeded' and odom_distance > 0.0:
            pending_sample = {
                'direction': int(self.direction_sign),
                'signed_target_distance': self.signed_target_distance,
                'target_distance': self.target_distance,
                'odom_distance': odom_distance,
                'corrected_distance': corrected_distance,
                'lateral_drift': lateral_drift,
                'elapsed': elapsed,
                'used_correction': self.odom_linear_scale_correction,
            }
            with self.samples_lock:
                self.pending_sample = pending_sample
                self.state = 'awaiting_input'
            self.get_logger().info(
                'Run is waiting for measured ground error. '
                'Enter cm error now: +over target along motion direction, -short of target, '
                '+0/-0 for exact target, 0 to finish, or any text to skip this run.'
            )
        else:
            self.state = 'idle'

        msg = (
            f'Calibration {status}: odom_distance={odom_distance:.4f} m, '
            f'corrected_distance={corrected_distance:.4f} m, '
            f'lateral_drift={lateral_drift:.4f} m, elapsed={elapsed:.2f} s, '
            f'target={self.target_distance:.4f} m, '
            f'used_correction={self.odom_linear_scale_correction:.6f}.'
        )
        if publish_warning:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)

    def _read_test_config(self):
        test_distance = self.get_parameter('test_distance').value
        speed = abs(self.get_parameter('speed').value)
        tolerance = max(self.get_parameter('tolerance').value, 0.0)
        correction = self.get_parameter('odom_linear_scale_correction').value
        timeout = self.get_parameter('timeout').value

        if test_distance == 0.0:
            self.get_logger().error(
                'test_distance must not be 0.0 m. Use a positive value for forward, negative for backward.'
            )
            return None
        if speed <= 0.0:
            self.get_logger().error('speed must be greater than 0.0 m/s.')
            return None
        if timeout <= 0.0:
            self.get_logger().error('timeout must be greater than 0.0 s.')
            return None
        if correction <= 0.0:
            self.get_logger().error('odom_linear_scale_correction must be greater than 0.0.')
            return None
        if speed > MAX_SPEED_LIMIT:
            self.get_logger().warn(
                f'speed {speed:.3f} m/s exceeds internal safety limit '
                f'{MAX_SPEED_LIMIT:.3f} m/s; clipping command speed.'
            )
            speed = MAX_SPEED_LIMIT

        direction_sign = 1 if test_distance > 0.0 else -1
        return {
            'direction_sign': direction_sign,
            'signed_test_distance': test_distance,
            'target_distance': abs(test_distance),
            'speed': speed,
            'tolerance': tolerance,
            'odom_linear_scale_correction': correction,
            'timeout': timeout,
        }

    def _lookup_pose(self):
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        try:
            trans = self.tf_buffer.lookup_transform(
                odom_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=TF_TIMEOUT_SEC),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}')
            return None

        translation = trans.transform.translation
        rotation = trans.transform.rotation
        return (
            translation.x,
            translation.y,
            self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _calculate_progress(self, pose):
        x, y, _ = pose
        start_x, start_y, start_yaw = self.start_pose
        dx = x - start_x
        dy = y - start_y
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)

        forward_delta = dx * cos_yaw + dy * sin_yaw
        lateral_drift = -dx * sin_yaw + dy * cos_yaw
        progress = self.direction_sign * forward_delta
        return progress, lateral_drift

    def _log_progress(self, raw_progress, corrected_progress, error, lateral_drift, elapsed):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1e9:
            return
        self.get_logger().info(
            f'odom_distance={raw_progress:.3f} m, '
            f'corrected_distance={corrected_progress:.3f} m, '
            f'error={error:+.3f} m, lateral_drift={lateral_drift:.3f} m, '
            f'elapsed={elapsed:.1f} s'
        )
        self.last_log_time = now

    def _skip_pending_sample(self, reason):
        with self.samples_lock:
            if self.pending_sample is None:
                self.get_logger().warn(
                    f'Input "{reason}" ignored. No pending successful run is waiting for input.'
                )
                return

            skipped_sample = self.pending_sample
            self.pending_sample = None
            self.state = 'idle'

        self.get_logger().info(
            f'Skipped pending run by input "{reason}": '
            f'direction={skipped_sample["direction"]:+d}, '
            f'signed_target={skipped_sample["signed_target_distance"]:.4f} m, '
            f'odom={skipped_sample["odom_distance"]:.4f} m, '
            f'corrected={skipped_sample["corrected_distance"]:.4f} m, '
            f'used_correction={skipped_sample["used_correction"]:.6f}. '
            'This run will not be used in the final scale summary.'
        )

    def _record_pending_sample(self, ground_error_cm):
        with self.samples_lock:
            if self.pending_sample is None:
                self.get_logger().warn(
                    'No pending successful run. Set start_test:=true first, wait for the robot to stop, '
                    'then enter the measured cm error.'
                )
                return

            actual_distance = self.pending_sample['target_distance'] + ground_error_cm / 100.0
            if actual_distance <= 0.0:
                self.get_logger().error(
                    f'Invalid measured result: target + error = {actual_distance:.4f} m. '
                    'Re-enter the cm error for this pending run.'
                )
                return

            sample = dict(self.pending_sample)
            sample['ground_error_cm'] = ground_error_cm
            sample['actual_distance'] = actual_distance
            sample['scale'] = actual_distance / sample['odom_distance']
            self.samples.append(sample)
            sample_index = len(self.samples)
            direction_index = sum(
                1 for recorded_sample in self.samples
                if recorded_sample['direction'] == sample['direction']
            )
            self.pending_sample = None
            self.state = 'idle'

        self.get_logger().info(
            f'Recorded sample #{sample_index} overall, direction {sample["direction"]:+d} #{direction_index}: '
            f'actual={actual_distance:.4f} m, '
            f'ground_error={ground_error_cm:+.2f} cm, odom={sample["odom_distance"]:.4f} m, '
            f'scale={sample["scale"]:.6f}. Set start_test:=true for the next run, or enter 0 to finish.'
        )

    def _print_summary(self):
        with self.samples_lock:
            samples = list(self.samples)

        if not samples:
            self.get_logger().warn('No successful calibration samples have been recorded yet.')
            return

        self.get_logger().info('========== X ODOM SCALE SUMMARY ==========')
        for index, sample in enumerate(samples, start=1):
            self.get_logger().info(
                f'#{index:02d} direction={sample["direction"]:+d}, '
                f'signed_target={sample["signed_target_distance"]:.4f} m, '
                f'target={sample["target_distance"]:.4f} m, '
                f'actual={sample["actual_distance"]:.4f} m, '
                f'ground_error={sample["ground_error_cm"]:+.2f} cm, '
                f'odom={sample["odom_distance"]:.4f} m, '
                f'corrected={sample["corrected_distance"]:.4f} m, '
                f'lateral_drift={sample["lateral_drift"]:.4f} m, '
                f'used_correction={sample["used_correction"]:.6f}, '
                f'scale={sample["scale"]:.6f}'
            )

        self._print_scale_stats('all', samples)
        for direction in (1, -1):
            direction_samples = [sample for sample in samples if sample['direction'] == direction]
            if direction_samples:
                self._print_scale_stats(f'direction={direction:+d}', direction_samples)
        self.get_logger().info('Restart this node to clear cached samples.')

    def _print_scale_stats(self, label, samples):
        scales = [sample['scale'] for sample in samples]
        mean_scale = statistics.fmean(scales)
        std_scale = statistics.pstdev(scales) if len(scales) > 1 else 0.0
        self.get_logger().info(
            f'{label}: samples={len(scales)}, recommended_odometry.scale_x={mean_scale:.6f}, '
            f'std={std_scale:.6f}, min={min(scales):.6f}, max={max(scales):.6f}'
        )

    def _publish_stop(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def _stop_robot(self):
        for _ in range(STOP_REPEAT_COUNT):
            self._publish_stop()

    def _stop_robot_with_ros_cli(self):
        topic = shlex.quote(self.cmd_vel_topic)
        zero_twist = (
            '"{linear: {x: 0.0, y: 0.0, z: 0.0}, '
            'angular: {x: 0.0, y: 0.0, z: 0.0}}"'
        )
        os.system(
            f'timeout 2s ros2 topic pub --once {topic} '
            f'geometry_msgs/msg/Twist {zero_twist} >/dev/null 2>&1'
        )

    def _reset_start_test(self):
        self.set_parameters([
            Parameter('start_test', Parameter.Type.BOOL, False),
        ])

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OdomLinearCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node._stop_robot_with_ros_cli()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
