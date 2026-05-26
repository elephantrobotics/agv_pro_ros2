#!/usr/bin/env python3
"""Yaw odometry scale calibration helper for AGV Pro."""

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
MAX_ANGULAR_SPEED_LIMIT = 0.50
TF_TIMEOUT_SEC = 0.5
STOP_REPEAT_COUNT = 5


class OdomYawCalib(Node):
    """Run repeated yaw odom tests and compute the final scale from cached samples."""

    def __init__(self):
        super().__init__('odom_yaw_calib')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('start_test', False)
        self.declare_parameter('test_angle', 360.0)
        self.declare_parameter('speed', 0.20)
        self.declare_parameter('tolerance', 2.0)
        self.declare_parameter('odom_yaw_scale_correction', 1.0)
        self.declare_parameter('timeout', 60.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'idle'
        self.direction_sign = 1.0
        self.signed_target_angle_deg = 360.0
        self.target_angle_deg = 360.0
        self.target_angle = math.radians(360.0)
        self.command_speed = 0.20
        self.tolerance_deg = 2.0
        self.tolerance = math.radians(2.0)
        self.odom_yaw_scale_correction = 1.0
        self.timeout = 60.0
        self.start_time = None
        self.prev_yaw = None
        self.accumulated_yaw = 0.0
        self.last_odom_angle = 0.0
        self.last_log_time = self.get_clock().now()

        self.samples = []
        self.samples_lock = threading.Lock()
        self.pending_sample = None

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self.on_timer)
        threading.Thread(target=self._stdin_loop, daemon=True).start()

        self.get_logger().info(
            'odom_yaw_calib ready. Set params, set start_test:=true for each run, '
            'use positive test_angle for positive angular.z and negative for negative angular.z, '
            'then enter the measured ground yaw error in deg after the robot stops. '
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
                'Input ignored. After a successful run enter ground yaw error in deg '
                '(+over target along rotation direction, -short), or enter 0 to finish.'
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
                    'A finished run is waiting for ground-yaw-error input; record it before starting again.'
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

        self.signed_target_angle_deg = config['signed_test_angle']
        self.target_angle_deg = config['target_angle']
        self.target_angle = math.radians(config['target_angle'])
        self.command_speed = config['speed']
        self.tolerance_deg = config['tolerance']
        self.tolerance = math.radians(config['tolerance'])
        self.odom_yaw_scale_correction = config['odom_yaw_scale_correction']
        self.timeout = config['timeout']
        self.direction_sign = float(config['direction_sign'])
        self.prev_yaw = pose[2]
        self.accumulated_yaw = 0.0
        self.start_time = self.get_clock().now()
        self.last_odom_angle = 0.0
        self.state = 'running'

        self.get_logger().info(
            f'Start yaw odom calibration: direction={int(self.direction_sign)}, '
            f'signed_target={self.signed_target_angle_deg:.1f} deg, '
            f'target={self.target_angle_deg:.1f} deg, speed={self.command_speed:.3f} rad/s, '
            f'odom_yaw_scale_correction={self.odom_yaw_scale_correction:.6f}'
        )

    def _run_test_step(self):
        pose = self._lookup_pose()
        if pose is None:
            self._finish_test('failed_tf', publish_warning=True)
            return

        raw_progress = self._calculate_yaw_progress(pose)
        raw_angle = max(raw_progress, 0.0)
        corrected_angle = raw_angle * self.odom_yaw_scale_correction
        error = corrected_angle - self.target_angle
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.last_odom_angle = raw_angle

        if corrected_angle >= self.target_angle - self.tolerance:
            self._finish_test('succeeded', raw_angle, corrected_angle, elapsed)
            return

        if elapsed > self.timeout:
            self._finish_test('timeout', raw_angle, corrected_angle, elapsed)
            return

        cmd = Twist()
        cmd.angular.z = self.direction_sign * self.command_speed
        self.cmd_vel_pub.publish(cmd)
        self._log_progress(raw_angle, corrected_angle, error, elapsed)

    def _finish_test(
        self,
        status,
        odom_angle=None,
        corrected_angle=None,
        elapsed=None,
        publish_warning=False,
    ):
        self._stop_robot()
        self._reset_start_test()

        if odom_angle is None:
            odom_angle = self.last_odom_angle
        if corrected_angle is None:
            corrected_angle = odom_angle * self.odom_yaw_scale_correction
        if elapsed is None and self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed is None:
            elapsed = 0.0

        if status == 'succeeded' and odom_angle > 0.0:
            pending_sample = {
                'direction': int(self.direction_sign),
                'signed_target_angle_deg': self.signed_target_angle_deg,
                'target_angle_deg': self.target_angle_deg,
                'odom_angle_deg': math.degrees(odom_angle),
                'corrected_angle_deg': math.degrees(corrected_angle),
                'elapsed': elapsed,
                'used_correction': self.odom_yaw_scale_correction,
            }
            with self.samples_lock:
                self.pending_sample = pending_sample
                self.state = 'awaiting_input'
            self.get_logger().info(
                'Run is waiting for measured ground yaw error. '
                'Enter deg error now: +over target along rotation direction, -short of target, '
                '+0/-0 for exact target, 0 to finish, or any text to skip this run.'
            )
        else:
            self.state = 'idle'

        msg = (
            f'Calibration {status}: odom_angle={math.degrees(odom_angle):.2f} deg, '
            f'corrected_angle={math.degrees(corrected_angle):.2f} deg, '
            f'elapsed={elapsed:.2f} s, target={self.target_angle_deg:.2f} deg, '
            f'used_correction={self.odom_yaw_scale_correction:.6f}.'
        )
        if publish_warning:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)

    def _read_test_config(self):
        test_angle = self.get_parameter('test_angle').value
        speed = abs(self.get_parameter('speed').value)
        tolerance = max(self.get_parameter('tolerance').value, 0.0)
        correction = self.get_parameter('odom_yaw_scale_correction').value
        timeout = self.get_parameter('timeout').value

        if test_angle == 0.0:
            self.get_logger().error(
                'test_angle must not be 0.0 deg. Use a positive value for one yaw direction, '
                'negative for the opposite direction.'
            )
            return None
        if speed <= 0.0:
            self.get_logger().error('speed must be greater than 0.0 rad/s.')
            return None
        if timeout <= 0.0:
            self.get_logger().error('timeout must be greater than 0.0 s.')
            return None
        if correction <= 0.0:
            self.get_logger().error('odom_yaw_scale_correction must be greater than 0.0.')
            return None
        if speed > MAX_ANGULAR_SPEED_LIMIT:
            self.get_logger().warn(
                f'speed {speed:.3f} rad/s exceeds internal safety limit '
                f'{MAX_ANGULAR_SPEED_LIMIT:.3f} rad/s; clipping command speed.'
            )
            speed = MAX_ANGULAR_SPEED_LIMIT

        direction_sign = 1 if test_angle > 0.0 else -1
        return {
            'direction_sign': direction_sign,
            'signed_test_angle': test_angle,
            'target_angle': abs(test_angle),
            'speed': speed,
            'tolerance': tolerance,
            'odom_yaw_scale_correction': correction,
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

        rotation = trans.transform.rotation
        return (
            trans.transform.translation.x,
            trans.transform.translation.y,
            self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _calculate_yaw_progress(self, pose):
        current_yaw = pose[2]
        delta = math.atan2(
            math.sin(current_yaw - self.prev_yaw),
            math.cos(current_yaw - self.prev_yaw),
        )
        self.accumulated_yaw += delta
        self.prev_yaw = current_yaw
        return self.direction_sign * self.accumulated_yaw

    def _log_progress(self, raw_angle, corrected_angle, error, elapsed):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1e9:
            return
        self.get_logger().info(
            f'odom_angle={math.degrees(raw_angle):.1f} deg, '
            f'corrected_angle={math.degrees(corrected_angle):.1f} deg, '
            f'error={math.degrees(error):+.1f} deg, elapsed={elapsed:.1f} s'
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
            f'signed_target={skipped_sample["signed_target_angle_deg"]:.2f} deg, '
            f'odom={skipped_sample["odom_angle_deg"]:.2f} deg, '
            f'corrected={skipped_sample["corrected_angle_deg"]:.2f} deg, '
            f'used_correction={skipped_sample["used_correction"]:.6f}. '
            'This run will not be used in the final scale summary.'
        )

    def _record_pending_sample(self, ground_error_deg):
        with self.samples_lock:
            if self.pending_sample is None:
                self.get_logger().warn(
                    'No pending successful run. Set start_test:=true first, wait for the robot to stop, '
                    'then enter the measured deg error.'
                )
                return

            actual_angle_deg = self.pending_sample['target_angle_deg'] + ground_error_deg
            if actual_angle_deg <= 0.0:
                self.get_logger().error(
                    f'Invalid measured result: target + error = {actual_angle_deg:.2f} deg. '
                    'Re-enter the deg error for this pending run.'
                )
                return

            sample = dict(self.pending_sample)
            sample['ground_error_deg'] = ground_error_deg
            sample['actual_angle_deg'] = actual_angle_deg
            sample['scale'] = math.radians(actual_angle_deg) / math.radians(sample['odom_angle_deg'])
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
            f'actual={actual_angle_deg:.2f} deg, '
            f'ground_error={ground_error_deg:+.2f} deg, odom={sample["odom_angle_deg"]:.2f} deg, '
            f'scale={sample["scale"]:.6f}. Set start_test:=true for the next run, or enter 0 to finish.'
        )

    def _print_summary(self):
        with self.samples_lock:
            samples = list(self.samples)

        if not samples:
            self.get_logger().warn('No successful calibration samples have been recorded yet.')
            return

        self.get_logger().info('========== YAW ODOM SCALE SUMMARY ==========')
        for index, sample in enumerate(samples, start=1):
            self.get_logger().info(
                f'#{index:02d} direction={sample["direction"]:+d}, '
                f'signed_target={sample["signed_target_angle_deg"]:.2f} deg, '
                f'target={sample["target_angle_deg"]:.2f} deg, '
                f'actual={sample["actual_angle_deg"]:.2f} deg, '
                f'ground_error={sample["ground_error_deg"]:+.2f} deg, '
                f'odom={sample["odom_angle_deg"]:.2f} deg, '
                f'corrected={sample["corrected_angle_deg"]:.2f} deg, '
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
            f'{label}: samples={len(scales)}, recommended_odometry.scale_theta={mean_scale:.6f}, '
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
    node = OdomYawCalib()
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
