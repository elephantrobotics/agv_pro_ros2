#!/usr/bin/env python3
"""Transparent final-refinement proxy for Nav2 pose navigation actions."""

import threading
import time
from copy import deepcopy

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class NavigateToPoseRefinerProxy(Node):
    """Forward pose-navigation actions and complete them after final yaw refinement."""

    TERMINAL_REFINER_STATUSES = {
        'succeeded',
        'timeout',
        'failed_tf',
        'no_goal',
        'handoff_distance_exceeded',
        'canceled',
    }
    STARTLESS_REFINER_FAILURES = {
        'failed_tf',
        'no_goal',
        'handoff_distance_exceeded',
    }

    def __init__(self):
        super().__init__('navigate_to_pose_refiner_proxy')
        self.public_goal_topic = '/goal_pose'
        self.refiner_goal_topic = '/final_pose_refiner/goal_pose'
        self.refiner_status_topic = '/final_pose_refiner/status'
        self.refiner_param_service = '/final_pose_refiner/set_parameters'
        self.start_param = 'final_pose_refiner_start'
        self.cancel_param = 'final_pose_refiner_cancel'

        self.declare_parameter('nav2_server_timeout_sec', 5.0)
        self.declare_parameter('refiner_service_timeout_sec', 2.0)
        self.declare_parameter('refiner_wait_timeout_sec', 25.0)
        self.declare_parameter('require_refinement', True)
        self.declare_parameter('debug_print', False)

        self.callback_group = ReentrantCallbackGroup()
        self.goal_pub = self.create_publisher(PoseStamped, self.refiner_goal_topic, 10)
        self.refiner_status_sub = self.create_subscription(
            String,
            self.refiner_status_topic,
            self._on_refiner_status,
            10,
            callback_group=self.callback_group,
        )
        self.refiner_param_client = self.create_client(
            SetParameters,
            self.refiner_param_service,
            callback_group=self.callback_group,
        )

        self._refinement_lock = threading.Lock()
        self._status_condition = threading.Condition()
        self._status_sequence = 0
        self._status_history = []
        self.routes = []
        self._add_route(
            'NavigateToPose',
            NavigateToPose,
            '/navigate_to_pose',
            '/navigate_to_pose_nav2',
            lambda request: request.pose,
        )
        self._add_route(
            'NavigateThroughPoses',
            NavigateThroughPoses,
            '/navigate_through_poses',
            '/navigate_through_poses_nav2',
            lambda request: request.poses[-1] if request.poses else None,
        )
        self.topic_nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )
        self.goal_topic_sub = self.create_subscription(
            PoseStamped,
            self.public_goal_topic,
            self._on_goal_pose,
            10,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Navigation refinement proxy ready for NavigateToPose, NavigateThroughPoses, '
            'and /goal_pose; public tasks complete after final yaw refinement.'
        )

    def _add_route(self, label, action_type, public_name, nav2_name, final_pose_getter):
        route = {
            'label': label,
            'action_type': action_type,
            'public_name': public_name,
            'nav2_name': nav2_name,
            'final_pose_getter': final_pose_getter,
        }
        route['client'] = ActionClient(
            self,
            action_type,
            nav2_name,
            callback_group=self.callback_group,
        )
        route['server'] = ActionServer(
            self,
            action_type,
            public_name,
            execute_callback=lambda handle, current=route: self._execute_callback(current, handle),
            goal_callback=lambda request, current=route: self._goal_callback(current, request),
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.routes.append(route)
        self._debug(f'{label} route: {public_name} -> {nav2_name}')

    def _goal_callback(self, route, goal_request):
        if not route['client'].server_is_ready():
            self.get_logger().warn(
                f"{route['label']} Nav2 action server {route['nav2_name']} is not ready; "
                'rejecting goal.'
            )
            return GoalResponse.REJECT

        final_pose = route['final_pose_getter'](goal_request)
        if final_pose is not None:
            self._publish_refiner_goal(final_pose)
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _on_goal_pose(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)
        if not self.topic_nav_client.server_is_ready():
            self.get_logger().error(
                'Cannot forward /goal_pose: public NavigateToPose is unavailable.'
            )
            return

        send_future = self.topic_nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_topic_goal_response)

    def _on_topic_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to forward /goal_pose to NavigateToPose: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('/goal_pose navigation goal was rejected.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_topic_goal_result)

    def _on_topic_goal_result(self, future):
        try:
            action_result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to receive /goal_pose navigation result: {exc}')
            return

        if action_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'/goal_pose navigation ended with action status {action_result.status}.'
            )

    def _execute_callback(self, route, goal_handle):
        nav_result = self._forward_to_nav2(route, goal_handle)
        if nav_result is None:
            return route['action_type'].Result()

        result, status = nav_result
        if status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            return result
        if status != GoalStatus.STATUS_SUCCEEDED:
            goal_handle.abort()
            return result

        final_pose = route['final_pose_getter'](goal_handle.request)
        refine_status = 'succeeded'
        if final_pose is not None:
            refine_status = self._refine_final_pose(goal_handle, final_pose)

        if refine_status == 'succeeded':
            goal_handle.succeed()
        elif refine_status == 'canceled':
            goal_handle.canceled()
        else:
            self.get_logger().error(
                f"{route['label']} completed in Nav2 but final refinement ended with "
                f"status '{refine_status}'."
            )
            goal_handle.abort()
        return result

    def _forward_to_nav2(self, route, goal_handle):
        timeout = float(self.get_parameter('nav2_server_timeout_sec').value)
        if not route['client'].wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"Nav2 action server {route['nav2_name']} is not available.")
            goal_handle.abort()
            return None

        send_future = route['client'].send_goal_async(
            deepcopy(goal_handle.request),
            feedback_callback=lambda message: self._relay_feedback(goal_handle, message),
        )
        if not self._wait_for_future(send_future, timeout):
            self.get_logger().error(f"Timed out forwarding {route['label']} goal to Nav2.")
            goal_handle.abort()
            return None

        try:
            nav_goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to forward {route['label']} goal to Nav2: {exc}")
            goal_handle.abort()
            return None

        if nav_goal_handle is None or not nav_goal_handle.accepted:
            self.get_logger().error(f"Forwarded {route['label']} goal was rejected by Nav2.")
            goal_handle.abort()
            return None

        result_future = nav_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_nav_goal(nav_goal_handle)
                goal_handle.canceled()
                return None
            time.sleep(0.05)

        if not result_future.done():
            goal_handle.abort()
            return None

        try:
            nav_result = result_future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get Nav2 {route['label']} result: {exc}")
            goal_handle.abort()
            return None

        result = (
            nav_result.result
            if nav_result and nav_result.result
            else route['action_type'].Result()
        )
        return result, nav_result.status

    def _refine_final_pose(self, goal_handle, pose):
        if not self.get_parameter('require_refinement').value:
            return 'succeeded'

        with self._refinement_lock:
            if goal_handle.is_cancel_requested:
                return 'canceled'

            self._publish_refiner_goal(pose)
            start_sequence = self._status_snapshot()
            if not self._set_refiner_parameter(self.start_param, True):
                return 'unavailable'

            wait_timeout = float(self.get_parameter('refiner_wait_timeout_sec').value)
            deadline = time.monotonic() + max(wait_timeout, 0.0)
            saw_running = False
            sequence = start_sequence
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._set_refiner_parameter(self.cancel_param, True)
                    return 'canceled'

                updates = self._wait_for_status_updates(sequence, deadline)
                if updates is None:
                    self.get_logger().error('Timed out waiting for final pose refinement result.')
                    self._set_refiner_parameter(self.cancel_param, True)
                    return 'timeout'

                for sequence, status in updates:
                    if status == 'running':
                        saw_running = True
                    elif status in self.TERMINAL_REFINER_STATUSES:
                        if saw_running or status in self.STARTLESS_REFINER_FAILURES:
                            return status
            return 'canceled'

    def _set_refiner_parameter(self, name, value):
        timeout = float(self.get_parameter('refiner_service_timeout_sec').value)
        if not self.refiner_param_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'Final pose refiner parameter service {self.refiner_param_service} '
                'is unavailable.'
            )
            return False

        parameter = ParameterMsg()
        parameter.name = name
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.refiner_param_client.call_async(request)
        if not self._wait_for_future(future, timeout):
            self.get_logger().error(f'Timed out setting final pose refiner parameter {name}.')
            return False

        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = response.results[0].reason if response and response.results else ''
            self.get_logger().error(f'Failed to set final pose refiner parameter {name}: {reason}')
            return False
        return True

    def _on_refiner_status(self, message):
        with self._status_condition:
            self._status_sequence += 1
            self._status_history.append((self._status_sequence, message.data))
            self._status_history = self._status_history[-32:]
            self._status_condition.notify_all()

    def _status_snapshot(self):
        with self._status_condition:
            return self._status_sequence

    def _wait_for_status_updates(self, sequence, deadline):
        with self._status_condition:
            while rclpy.ok():
                updates = [item for item in self._status_history if item[0] > sequence]
                if updates:
                    return updates
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._status_condition.wait(timeout=min(remaining, 0.1))
        return None

    def _publish_refiner_goal(self, pose):
        refiner_goal = deepcopy(pose)
        refiner_goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(refiner_goal)

    @staticmethod
    def _relay_feedback(goal_handle, feedback_message):
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback_message.feedback)

    def _debug(self, message):
        if self.get_parameter('debug_print').value:
            self.get_logger().info(message)

    @staticmethod
    def _wait_for_future(future, timeout_sec):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return done.wait(timeout_sec)

    def _cancel_nav_goal(self, nav_goal_handle):
        cancel_future = nav_goal_handle.cancel_goal_async()
        self._wait_for_future(cancel_future, 2.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseRefinerProxy()
    executor = MultiThreadedExecutor(num_threads=6)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
