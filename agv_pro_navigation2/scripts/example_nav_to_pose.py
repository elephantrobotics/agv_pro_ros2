#! /usr/bin/env python3

import argparse
import sys

import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""


def parse_arguments():
    parser = argparse.ArgumentParser(description='Send navigation test goals.')
    parser.add_argument(
        '--localization',
        choices=('amcl', 'slam'),
        default='amcl',
        help='Localization backend started by navigation2_active.launch.py.')
    parser.add_argument(
        'targets',
        nargs='*',
        help='Waypoint letters to execute once, for example AB. Omit to loop ABCDE.')
    return parser.parse_args()


def set_initial_pose(navigator: BasicNavigator, x: float, y: float, oz: float, ow: float):
    """
    Set the initial pose of the robot for the active localization backend.

    Args:
        navigator (BasicNavigator): The navigator instance controlling the robot.
        x (float): Initial X position in the map frame.
        y (float): Initial Y position in the map frame.
        oz (float): Orientation Z component (quaternion).
        ow (float): Orientation W component (quaternion).
    """
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    initial_pose.pose.position.y = y
    initial_pose.pose.orientation.z = oz
    initial_pose.pose.orientation.w = ow
    navigator.setInitialPose(initial_pose)


def make_goal_pose(navigator: BasicNavigator, x: float, y: float, oz: float, ow: float) -> PoseStamped:
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.z = oz
    goal_pose.pose.orientation.w = ow
    return goal_pose


def navigate_to_goal(navigator: BasicNavigator, goal_pose: PoseStamped, verbose: bool = False) -> bool:
    """
    Navigate the robot to a target goal pose.

    Args:
        navigator (BasicNavigator): The navigator instance controlling the robot.
        goal_pose (PoseStamped): Goal pose in the map frame.
        verbose (bool, optional): If True, prints navigation feedback such as estimated arrival time. Default is False.

    Returns:
        bool: True if navigation succeeded, False otherwise.
    """
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and verbose:
            remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimated time of arrival: {remaining:.0f} seconds")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        return True
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    return False

if __name__ == '__main__':
    cli_args = parse_arguments()
    rclpy.init()
    navigator = BasicNavigator()

    if cli_args.localization == 'slam':
        # slam_toolbox on Humble is not a Nav2 lifecycle-managed localizer.
        navigator._waitForNodeToActivate('bt_navigator')
        navigator.info('Nav2 is ready for use!')
    else:
        # AMCL obtains its initial origin pose from agvpro.yaml.
        navigator.initial_pose_received = True
        navigator.waitUntilNav2Active()

    # Try to load waypoints from YAML, fallback to hardcoded defaults
    waypoints = {}
    try:
        with open('waypoints.yaml', 'r') as f:
            waypoints = (yaml.safe_load(f) or {}).get('waypoints', {})
    except FileNotFoundError:
        with open('waypoints.yaml', 'w') as f:
            yaml.dump({'waypoints': {}}, f, default_flow_style=False)

    goals = {
        'A': waypoints.get('A', [4.89649,-0.617371,0.706899,0.707315]),
        'B': waypoints.get('B', [0.90387,-0.446105,0.273676,0.961822]),
        'C': waypoints.get('C', [4.46734,-0.532388,0.969886,-0.243558]),
        'D': waypoints.get('D', [-0.0233348,0.00798563,0.999322,0.0368173]),
        'E': waypoints.get('E', [2.33651,-0.440663,0.937234,-0.3487]),
    }

    args = cli_args.targets
    loop_targets = False
    if args:
        targets = []
        for arg in args:
            for c in arg.upper():
                if c in goals:
                    targets.append(c)
    else:
        targets = ['A', 'B', 'C', 'D', 'E']
        loop_targets = True
        print('No target arguments provided; running A-B-C-D-E repeatedly. Press Ctrl+C to stop.')

    if not targets:
        print('No valid target names provided. Use names such as A, B, C, D, E or AB.')
        rclpy.shutdown()
        sys.exit(1)

    try:
        cycle_index = 1
        while rclpy.ok():
            if loop_targets:
                print(f'============= cycle {cycle_index}: ABCDE =============')

            for name in targets:
                if not rclpy.ok():
                    break

                x_goal, y_goal, orientation_z, orientation_w = goals[name]
                input(f'============={name}==================\n')
                goal_pose = make_goal_pose(navigator, x_goal, y_goal, orientation_z, orientation_w)
                success = navigate_to_goal(navigator, goal_pose)
                print("Navigation result:", goals[name], success)

            if not loop_targets:
                break
            cycle_index += 1
    except KeyboardInterrupt:
        print('Navigation loop interrupted by user.')
        navigator.cancelTask()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
