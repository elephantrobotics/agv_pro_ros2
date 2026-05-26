import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('agv_pro_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = 'agvpro.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('agv_pro_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('agv_pro_navigation2'),
        'rviz',
        'agvpro_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        GroupAction(actions=[SetRemap(
            src='/goal_pose',
            dst='/goal_pose_nav2',
        )] + [
            SetRemap(
                src=f'/{action}/_action/{suffix}',
                dst=f'/{action}_nav2/_action/{suffix}')
            for action in ('navigate_to_pose', 'navigate_through_poses')
            for suffix in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
        ] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, '/bringup_launch.py']),
                launch_arguments={
                    'map': map_dir,
                    'params_file': param_dir,
                }.items()),
        ], scoped=True),

        Node(
            package='agv_pro_calibration',
            executable='navigate_to_pose_refiner_proxy',
            name='navigate_to_pose_refiner_proxy',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='agv_pro_calibration',
            executable='final_pose_refiner',
            name='final_pose_refiner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'final_pose_refiner_auto_start_on_nav_success': False,
            }]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
