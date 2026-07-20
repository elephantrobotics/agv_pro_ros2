import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def lidar_cond(enable_lidar, lidar_type, expected_type):
    return IfCondition(
        PythonExpression([
            "'", enable_lidar, "' == 'true' and '",
            lidar_type, "' == '", expected_type, "'",
        ])
    )


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name', default_value='robot1',
            description='Robot namespace, e.g. robot1 / robot2'),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/agvpro_controller',
            description='Chassis controller serial port'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Whether to launch lidar drivers'),
        DeclareLaunchArgument(
            'lidar_type', default_value='n10p',
            description='Lidar type: n10p | mid360 | l2'),
        DeclareLaunchArgument('scan_min_height', default_value='0.35'),
        DeclareLaunchArgument('scan_max_height', default_value='0.45'),
    ]

    robot_name = LaunchConfiguration('robot_name')
    port_name = LaunchConfiguration('port_name')
    enable_lidar = LaunchConfiguration('enable_lidar')
    lidar_type = LaunchConfiguration('lidar_type')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')


    urdf_file = os.path.join(
        get_package_share_directory('agv_pro_description'),
        'urdf', 'agv_pro.urdf')
    robot_description_content = Command([
        'xacro ', urdf_file, ' namespace:=', robot_name, '/'])

    agv_pro_node = Node(
        package='agv_pro_base',
        executable='agv_pro_node',
        name='agv_pro_node',
        output='screen',
        parameters=[{'port_name': port_name, 'namespace': robot_name}],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    launch_dir = os.path.join(get_package_share_directory('agv_pro_multi'), 'launch')

    def lidar_include(launch_file, expected_type, with_height=False):
        args = {'robot_name': robot_name}
        if with_height:
            args['scan_min_height'] = scan_min_height
            args['scan_max_height'] = scan_max_height
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=args.items(),
            condition=lidar_cond(enable_lidar, lidar_type, expected_type))

    group = GroupAction([
        PushRosNamespace(robot_name),
        agv_pro_node,
        joint_state_pub,
        robot_state_pub,
        lidar_include('lidar_n10p.launch.py', 'n10p'),
        lidar_include('lidar_mid360.launch.py', 'mid360', with_height=True),
        lidar_include('lidar_l2.launch.py', 'l2', with_height=True),
    ])

    return LaunchDescription([*declared_arguments, group])
