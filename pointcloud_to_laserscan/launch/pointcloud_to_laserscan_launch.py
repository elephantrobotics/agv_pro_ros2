from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='scanner', default_value='unilidar',
            description='Namespace for Unitree LiDAR topics, used to organize topic names'
        ),

        # Make sure the Unitree LiDAR node is already running
        # and publishing point cloud data to the /unilidar/cloud topic

        # ========== Unitree PointCloud to LaserScan Converter ==========
        # Convert Unitree LiDAR 3D point cloud data into 2D LaserScan data
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',

            remappings=[
                ('cloud_in', '/unilidar/cloud'),  # Input: Unitree LiDAR point cloud topic
                ('scan', '/scan')                 # Output: standard LaserScan topic
            ],
            parameters=[{
                'min_height': 0.35,               # Minimum height: filter points below this height
                'max_height': 0.45,               # Maximum height: keep obstacles, filter ceiling points
                'angle_min': -3.14159,            # Minimum scan angle: -π radians (-180 degrees)
                'angle_max': 3.14159,             # Maximum scan angle:  π radians (180 degrees)
                'angle_increment': 0.00436,       # Angular resolution: π/720 radians (~0.25 degrees)
                'scan_time': 0.1,                 # Scan time: 0.1s (10 Hz publishing rate, suitable for navigation)
                'range_min': 0.1,                 # Minimum valid range: avoid self-reflections
                'range_max': 100.0,               # Maximum valid range: Unitree LiDAR effective range
                'use_inf': True,                  # Use infinity for missing readings (LaserScan standard)
                'inf_epsilon': 1.0                # Epsilon value for infinity handling
            }],
            name='unitree_pointcloud_to_laserscan'  # Node name
        )
    ])

