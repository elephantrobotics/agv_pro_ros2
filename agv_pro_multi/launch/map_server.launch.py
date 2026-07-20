import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('agv_pro_navigation2')
    workspace_dir = os.path.abspath(os.path.join(nav_dir, '..', '..', '..', '..'))
    map_pkg_dir = os.path.join(workspace_dir, 'src', 'agv_pro_navigation2')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(map_pkg_dir, 'map', 'map.yaml'),
            description='Full path to the shared map yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml,
                         'topic_name': '/map', 'frame_id': 'map'}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server']}]),
    ])
