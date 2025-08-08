#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""

def set_initial_pose(navigator: BasicNavigator, x: float, y: float, oz: float, ow: float):
    """
    Set the initial pose of the robot for AMCL localization.

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


def navigate_to_goal(navigator: BasicNavigator, x: float, y: float, oz: float, ow: float, verbose: bool = False) -> bool:
    """
    Navigate the robot to a target goal pose.

    Args:
        navigator (BasicNavigator): The navigator instance controlling the robot.
        x (float): Goal X position in the map frame.
        y (float): Goal Y position in the map frame.
        oz (float): Orientation Z component (quaternion).
        ow (float): Orientation W component (quaternion).
        verbose (bool, optional): If True, prints navigation feedback such as estimated arrival time. Default is False.

    Returns:
        bool: True if navigation succeeded, False otherwise.
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.z = oz
    goal_pose.pose.orientation.w = ow

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
    rclpy.init()
    navigator = BasicNavigator()

    # Set robot initial pose
    # set_initial_pose(navigator, x=-1.9248794317245483, y=-0.5366987586021423, oz=-1.8463129131030735e-06, ow=0.9999999999982956)

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    goal_A = [1.6766083240509033,0.37930558800697327,-0.03491306994337919, 0.9993903529387947]
    goal_B = [-0.5062443017959595,1.559376835823059,0.6869307039904945,0.7267229237578264]

    x_goal, y_goal, orientation_z, orientation_w = goal_A
    success = navigate_to_goal(navigator, x_goal, y_goal, orientation_z, orientation_w)
    print("Navigation result:", success)

    x_goal, y_goal, orientation_z, orientation_w = goal_B
    success = navigate_to_goal(navigator, x_goal, y_goal, orientation_z, orientation_w)
    print("Navigation result:", success)

    rclpy.shutdown()
