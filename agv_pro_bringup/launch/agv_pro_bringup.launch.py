import os
from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import Command,LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def include_lidar(pkg_name, launch_file, lidar_type, expected_type):

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                launch_file
            )
        ),
        condition=PythonExpression(
            ["'", lidar_type, "' == '", expected_type, "'"]
        )
    )

def generate_launch_description():

    port_name_arg = LaunchConfiguration('port_name',default='/dev/agvpro_controller')
    namespace = LaunchConfiguration('namespace', default='')
    lidar_type = LaunchConfiguration('lidar_type', default='n10p')

    urdf_file = os.path.join(
        get_package_share_directory('agv_pro_description'),
        'urdf',
        'agv_pro.urdf'
    )

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' namespace:=',
        PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
    ])

    declare_port_name_arg = DeclareLaunchArgument(
        'port_name', 
        default_value=port_name_arg,
        description='port name, e.g. /dev/ttyACM0'
    )

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for nodes'
    )

    declare_lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value=lidar_type,
        description='Lidar type: n10p | mid360 | l2'
    )

    ns_action = PushRosNamespace(namespace)

    agv_pro_node = Node(
        package='agv_pro_base',
        executable='agv_pro_node',
        name='agv_pro_node',
        output='screen',
        parameters=[{
            'port_name': port_name_arg,
            'namespace': namespace,             
        }],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    lidar_launchs = [
        include_lidar('lslidar_driver', 'lsn10p_launch.py', lidar_type, 'n10p'),
        include_lidar('livox_ros_driver2', 'msg_MID360_launch.py', lidar_type, 'mid360'),
        include_lidar('unitree_lidar_ros2', 'launch.py', lidar_type, 'l2'),
    ]

    return LaunchDescription(
        [
            declare_port_name_arg,
            declare_namespace_arg,
            declare_lidar_type_arg,
            ns_action,
            agv_pro_node,
            joint_state_pub,
            robot_state_pub,
            *lidar_launchs,
        ]
    )