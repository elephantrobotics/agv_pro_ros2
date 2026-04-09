import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
# xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
# publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'laser_link'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

def generate_launch_description():

    use_mid360_rviz = LaunchConfiguration('use_mid360_rviz')
    xfer_format = LaunchConfiguration('xfer_format')
    publish_freq = LaunchConfiguration('publish_freq')

    declare_rviz_arg = DeclareLaunchArgument(
        'use_mid360_rviz',
        default_value='false',
        description='Enable RViz for MID360 LiDAR visualization'
    )

    declare_xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='0',
        description='MID360 format: 0=Pointcloud2(PointXYZRTL), 1=customized pointcloud format'
    )

    declare_publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value='10.0',
        description='MID360 LiDAR publish frequency in Hz (e.g., 5.0, 10.0, 20.0, 50.0)'
    )

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='mid360_lidar_rviz',
            output='screen',
            arguments=['--display-config', rviz_config_path],
            condition=IfCondition(
                PythonExpression([
                    "'", use_mid360_rviz, "' == 'true' and '",
                    xfer_format, "' == '0'"
                ])
            )
        )

    return LaunchDescription([
        declare_rviz_arg,
        declare_xfer_format_arg,
        declare_publish_freq_arg,
        livox_driver,
        livox_rviz,
    ])