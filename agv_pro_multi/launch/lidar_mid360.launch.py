import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


multi_topic = 0
data_src = 0
output_type = 0
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

livox_config_dir = os.path.join(get_package_share_directory('livox_ros_driver2'), 'config')
rviz_config_path = os.path.join(livox_config_dir, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(livox_config_dir, 'MID360_config.json')


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    use_mid360_rviz = LaunchConfiguration('use_mid360_rviz')
    xfer_format = LaunchConfiguration('xfer_format')
    publish_freq = LaunchConfiguration('publish_freq')

    frame_id = [robot_name, '/laser_link']

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
    )
    declare_scan_min_arg = DeclareLaunchArgument(
        'scan_min_height',
        default_value='0.35',
    )
    declare_scan_max_arg = DeclareLaunchArgument(
        'scan_max_height',
        default_value='0.45',
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'use_mid360_rviz',
        default_value='false',
        description='Enable RViz for MID360 LiDAR visualization',
    )
    declare_xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='0',
        description='MID360 format: 0=Pointcloud2(PointXYZRTL), 1=customized pointcloud format',
    )
    declare_publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value='10.0',
        description='MID360 LiDAR publish frequency in Hz',
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': xfer_format},
            {'multi_topic': multi_topic},
            {'data_src': data_src},
            {'publish_freq': publish_freq},
            {'output_data_type': output_type},
            {'frame_id': frame_id},
            {'lvx_file_path': lvx_file_path},
            {'user_config_path': user_config_path},
            {'cmdline_input_bd_code': cmdline_bd_code},
        ],
        remappings=[('/livox/lidar', 'livox/lidar'), ('/livox/imu', 'livox/imu')],
    )
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'min_height': ParameterValue(scan_min_height, value_type=float),
            'max_height': ParameterValue(scan_max_height, value_type=float),
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[('cloud_in', 'livox/lidar'), ('scan', 'scan')],
    )
    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='mid360_lidar_rviz',
        output='screen',
        arguments=['--display-config', rviz_config_path],
        condition=IfCondition(
            PythonExpression(
                ["'", use_mid360_rviz, "' == 'true' and '", xfer_format, "' == '0'"]
            )
        ),
    )

    return LaunchDescription(
        [
            declare_robot_name_arg,
            declare_scan_min_arg,
            declare_scan_max_arg,
            declare_rviz_arg,
            declare_xfer_format_arg,
            declare_publish_freq_arg,
            livox_driver,
            pointcloud_to_laserscan,
            livox_rviz,
        ]
    )
