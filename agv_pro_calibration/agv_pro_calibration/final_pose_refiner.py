#!/usr/bin/env python3
"""Yaw-only final pose refinement helper for AGV Pro."""

import math
import os
import shlex

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

class FinalPoseRefiner(Node):
    """Refine only the final map->base_footprint yaw with direct low-speed cmd_vel."""

    def __init__(self):
        super().__init__('final_pose_refiner')
        self.param_prefix = 'final_pose_refiner_'
        self.start_param = f'{self.param_prefix}start'
        self.cancel_param = f'{self.param_prefix}cancel'
        self.auto_start_param = f'{self.param_prefix}auto_start_on_nav_success'
        self.log_separator = '------------------------------------------------------------'
        self.cmd_vel_topic = '/cmd_vel'
        self.goal_topic = '/goal_pose'
        self.action_goal_topic = '/final_pose_refiner/goal_pose'
        self.nav_status_topic = '/navigate_to_pose/_action/status'
        self.nav2_status_topic = '/navigate_to_pose_nav2/_action/status'
        self.global_frame = 'map'
        self.base_frame = 'base_footprint'

        self._declare_param('status_topic', '/final_pose_refiner/status')
        self.declare_parameter(self.start_param, False)
        self.declare_parameter(self.cancel_param, False)
        self.declare_parameter(self.auto_start_param, False)
        self._declare_param('handoff_distance', 0.20)
        self._declare_param('yaw_tolerance', 0.04)
        self._declare_param('settle_time', 0.5)
        self._declare_param('timeout', 20.0)
        self._declare_param('k_yaw', 0.5)
        self._declare_param('max_wz', 0.35)
        self._declare_param('min_cmd_w', 0.006)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self._param('status_topic'), 10)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._on_goal,
            10,
        )
        self.action_goal_sub = self.create_subscription(
            PoseStamped,
            self.action_goal_topic,
            self._on_goal,
            10,
        )
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self._on_nav_status,
            10,
        )
        self.nav2_status_sub = self.create_subscription(
            GoalStatusArray,
            self.nav2_status_topic,
            self._on_nav_status,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'idle'
        self.goal = None
        self.target = None
        self.waiting_for_nav_success = False
        self.active_nav_goal_ids = set()
        self.refined_nav_goal_ids = set()
        self.start_time = None
        self.settle_start_time = None
        self.last_log_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / 20.0, self.on_timer)

        self.get_logger().info(
            'final_pose_refiner ready in yaw-only mode. A navigation proxy may submit the '
            f'target and set {self.start_param}:=true, or these may be provided manually.'
        )

    def _declare_param(self, name, value):
        self.declare_parameter(f'{self.param_prefix}{name}', value)

    def _param(self, name):
        return self.get_parameter(f'{self.param_prefix}{name}').value

    def _on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f'Ignoring goal in frame "{msg.header.frame_id}". Expected "{self.global_frame}".'
            )
            return

        q = msg.pose.orientation
        target_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.target = (msg.pose.position.x, msg.pose.position.y, target_yaw)
        self.waiting_for_nav_success = True
        self.active_nav_goal_ids.clear()
        self.get_logger().info(
            f'Updated refine target from topic: x={msg.pose.position.x:.4f}, '
            f'y={msg.pose.position.y:.4f}, yaw={math.degrees(target_yaw):.2f} deg'
        )

    def _on_nav_status(self, msg):
        if not self.get_parameter(self.auto_start_param).value:
            return

        if self.state != 'idle' or self.target is None or not self.waiting_for_nav_success:
            return

        for status in msg.status_list:
            goal_id = tuple(status.goal_info.goal_id.uuid)
            if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                self.active_nav_goal_ids.add(goal_id)
            elif status.status == GoalStatus.STATUS_SUCCEEDED:
                if (
                    goal_id in self.active_nav_goal_ids and
                    goal_id not in self.refined_nav_goal_ids
                ):
                    self.refined_nav_goal_ids.add(goal_id)
                    self.get_logger().info(
                        'Detected Nav2 goal succeeded; starting final yaw refinement.'
                    )
                    self._start_refine()
                    return
            elif status.status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                if goal_id in self.active_nav_goal_ids:
                    self.waiting_for_nav_success = False
                    self.active_nav_goal_ids.discard(goal_id)

    def on_timer(self):
        if self.get_parameter(self.cancel_param).value:
            if self.state == 'running':
                self._finish_refine('canceled')
            else:
                self._reset_cancel_refine()
            return

        if self.state == 'running':
            self._run_refine_step()
            return

        if self.get_parameter(self.start_param).value:
            self._start_refine()

    def _start_refine(self):
        self._reset_cancel_refine()
        if self.target is None:
            self.get_logger().warn(
                'Cannot start final refinement: no /goal_pose has been received yet.'
            )
            self._publish_status('no_goal')
            self._reset_start_refine()
            return

        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().warn('Cannot start final refinement: TF is not available.')
            self._publish_status('failed_tf')
            self._reset_start_refine()
            return

        target = self.target
        distance, yaw_error = self._calculate_error(pose, target)
        handoff_distance = max(self._param('handoff_distance'), 0.0)
        if distance > handoff_distance:
            self.get_logger().warn(
                f'Cannot start final refinement: distance={distance:.3f} m exceeds '
                f'handoff_distance={handoff_distance:.3f} m.'
            )
            self._publish_status('handoff_distance_exceeded')
            self._reset_start_refine()
            return

        self.goal = target
        self.waiting_for_nav_success = False
        self.start_time = self.get_clock().now()
        self.settle_start_time = None
        self.last_log_time = self.get_clock().now()
        self.state = 'running'
        self._publish_status('running')
        self.get_logger().info(
            f'\n{self.log_separator}\n'
            'FINAL YAW REFINE START\n'
            f'target=({target[0]:.4f}, {target[1]:.4f}, {math.degrees(target[2]):.2f} deg)\n'
            f'initial_distance={distance:.3f} m, '
            f'initial_yaw_error={math.degrees(yaw_error):+.2f} deg\n'
            f'{self.log_separator}'
        )

    def _run_refine_step(self):
        pose = self._lookup_pose()
        if pose is None:
            self._finish_refine('failed_tf', warn=True)
            return

        distance, yaw_error = self._calculate_error(pose, self.goal)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        yaw_tolerance = max(self._param('yaw_tolerance'), 0.0)
        settle_time = max(self._param('settle_time'), 0.0)
        timeout = self._param('timeout')

        if timeout > 0.0 and elapsed > timeout:
            self._finish_refine('timeout', pose, distance, yaw_error, warn=True)
            return

        if abs(yaw_error) <= yaw_tolerance:
            now = self.get_clock().now()
            if self.settle_start_time is None:
                self.settle_start_time = now
                self._publish_stop()
            elif (now - self.settle_start_time).nanoseconds / 1e9 >= settle_time:
                self._finish_refine('succeeded', pose, distance, yaw_error)
                return
            else:
                self._publish_stop()
            self._log_progress(pose, distance, yaw_error, elapsed, Twist())
            return

        self.settle_start_time = None
        cmd = self._make_yaw_command(yaw_error)
        self.cmd_vel_pub.publish(cmd)
        self._log_progress(pose, distance, yaw_error, elapsed, cmd)

    def _make_yaw_command(self, yaw_error):
        cmd = Twist()
        max_wz = max(abs(self._param('max_wz')), 0.0)
        cmd.angular.z = self._clip(self._param('k_yaw') * yaw_error, -max_wz, max_wz)
        cmd.angular.z = self._apply_min_abs(cmd.angular.z, self._param('min_cmd_w'))
        return cmd

    def _finish_refine(self, status, pose=None, distance=None, yaw_error=None, warn=False):
        self._stop_robot()
        self._reset_start_refine()
        self._reset_cancel_refine()
        self.state = 'idle'
        self.settle_start_time = None
        self._publish_status(status)

        if pose is not None and distance is not None and yaw_error is not None:
            msg = (
                f'\n{self.log_separator}\n'
                f'FINAL YAW REFINE END: {status}\n'
                f'distance={distance:.4f} m, '
                f'yaw_error={math.degrees(yaw_error):+.2f} deg, '
                f'pose=({pose[0]:.4f}, {pose[1]:.4f}, {math.degrees(pose[2]):.2f} deg)\n'
                f'{self.log_separator}'
            )
        else:
            msg = (
                f'\n{self.log_separator}\n'
                f'FINAL YAW REFINE END: {status}\n'
                f'{self.log_separator}'
            )

        if warn:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)

    def _lookup_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
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

    def _calculate_error(self, pose, target):
        x, y, yaw = pose
        target_x, target_y, target_yaw = target
        distance = math.hypot(target_x - x, target_y - y)
        yaw_error = self._normalize_angle(target_yaw - yaw)
        return distance, yaw_error

    def _log_progress(self, pose, distance, yaw_error, elapsed, cmd):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1e9:
            return

        self.get_logger().info(
            f'[FINAL YAW REFINE RUNNING] '
            f'distance={distance:.3f} m, yaw_error={math.degrees(yaw_error):+.2f} deg, '
            f'elapsed={elapsed:.1f} s, cmd_wz={cmd.angular.z:+.3f}, '
            f'pose=({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg)'
        )
        self.last_log_time = now

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_stop(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def _stop_robot(self):
        for _ in range(5):
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

    def _reset_start_refine(self):
        self.set_parameters([
            Parameter(self.start_param, Parameter.Type.BOOL, False),
        ])

    def _reset_cancel_refine(self):
        self.set_parameters([
            Parameter(self.cancel_param, Parameter.Type.BOOL, False),
        ])

    @staticmethod
    def _clip(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def _apply_min_abs(value, min_abs):
        min_abs = max(abs(min_abs), 0.0)
        if value == 0.0 or abs(value) >= min_abs:
            return value
        return math.copysign(min_abs, value)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = FinalPoseRefiner()
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
