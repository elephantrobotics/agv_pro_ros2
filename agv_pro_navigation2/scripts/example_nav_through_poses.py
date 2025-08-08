#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


"""
Basic navigation demo to go to poses.
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

def create_pose(navigator: BasicNavigator, x, y, z, w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose

def nav_through_pose(navigator: BasicNavigator, goal_poses, verbose: bool = False) -> bool:
    
    nav_start = navigator.get_clock().now()
    navigator.goThroughPoses(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and verbose:
            remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimated time of arrival: {remaining:.0f} seconds")

    # Do something depending on the return code
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
    # set_initial_pose(navigator, x=-1.9248794317245483, y=-0.5366987586021423, oz=-1.8463129131030735e-06, ow=0.9999999999982956)\
    
    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    way1_goals = [
        [0.5062479972839355, -0.5562516450881958, -0.011363976322727884, 0.9999354279362925],
        [1.7874977588653564, -0.6250066757202148, 0.7002746726771927, 0.7138735061667792],
        [1.3625057935714722, 1.5999948978424072, 0.9999809382375775, 0.006174395637980891],
        [-1.4687445163726807, 1.487505555152893, -0.9025521753719631, 0.43058050435584877]
    ]
    way2_goals = [
        [0.5062479972839355, -0.5562516450881958, -0.011363976322727884, 0.9999354279362925],
        [1.7874977588653564, -0.6250066757202148, 0.7002746726771927, 0.7138735061667792],
        [1.3625057935714722, 1.5999948978424072, 0.9999809382375775, 0.006174395637980891],
        [-1.4687445163726807, 1.487505555152893, -0.9025521753719631, 0.43058050435584877]
    ]

    goal_poses_1 = [create_pose(navigator, *g) for g in way1_goals]
    goal_poses_2 = [create_pose(navigator, *g) for g in way2_goals]
    
    result1 = nav_through_pose(navigator, goal_poses_1, verbose=True)
    print(f"First segment navigation result: {result1}")
    
    if result1 ==True:
        result2 = nav_through_pose(navigator, goal_poses_2, verbose=True)
        print(f"Second segment navigation result: {result2}")

    rclpy.shutdown()