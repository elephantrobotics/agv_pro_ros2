#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import ReplaceString

import lifecycle_msgs.msg
import os

def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10p.yaml')
    driver_params = ReplaceString(source_file=driver_dir, replacements={'/lslidar_driver_node:': '/**:'})

    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_params,
                                            {'frame_id': [robot_name, '/laser_link'],
                                             'scan_topic': 'scan'}],
                                )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        driver_node,
    ])
