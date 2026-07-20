import os

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    use_l2_rviz = LaunchConfiguration('use_l2_rviz')

    cloud_frame = [robot_name, '/laser_link']
    imu_frame = [robot_name, '/unilidar_imu']

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1'
    )
    declare_scan_min_arg = DeclareLaunchArgument(
        'scan_min_height',
        default_value='0.35'
    )
    declare_scan_max_arg = DeclareLaunchArgument(
        'scan_max_height',
        default_value='0.45'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'use_l2_rviz',
        default_value='false',
        description='Whether to launch RViz for Unitree L2 LiDAR visualization'
    )

    pkg_share = get_package_share_directory('unitree_lidar_ros2')
    rviz_config_file = os.path.join(pkg_share,
                                    'rviz',
                                    'view.rviz')

    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'initialize_type': 1},
                {'work_mode': 8},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM1'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6201},
                {'local_ip': '192.168.1.2'},

                {'cloud_frame': cloud_frame},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': imu_frame},
                {'imu_topic': "unilidar/imu"},
                ],
    )

    # 3D point cloud -> 2D scan
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
        remappings=[('cloud_in', 'unilidar/cloud'), ('scan', 'scan')]
    )

    # Run Rviz
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       arguments=['-d', rviz_config_file],
       condition=IfCondition(use_l2_rviz),
       output='log'
    )
    return LaunchDescription(
        [
            declare_robot_name_arg,
            declare_scan_min_arg,
            declare_scan_max_arg,
            declare_rviz_arg,
            node1,
            pointcloud_to_laserscan,
            rviz_node,
        ]
    )
