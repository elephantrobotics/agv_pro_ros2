import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_pro_multi')
    src_pkg_dir = os.path.abspath(
        os.path.join(pkg_dir, '..', '..', '..', '..', 'src', 'agv_pro_multi'))
    src_param_dir = os.path.join(src_pkg_dir, 'param')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    robot_name = LaunchConfiguration('robot_name')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name', default_value='robot1',
            description='Robot namespace, e.g. robot1 / robot2'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(src_param_dir, 'agv_pro_nav_multi.yaml'),
            description='Self-namespacing Nav2 param template'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
    ]

    base_frame = [robot_name, '/base_footprint']
    base_frame_link = [robot_name, '/base_link']

    replaced_params = ReplaceString(
        source_file=params_file,
        replacements={
            '<robot_namespace>': robot_name,
            '<multi_param_dir>': src_param_dir,
        })
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=replaced_params,
            root_key=robot_name,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    action_remaps = [SetRemap('goal_pose', 'goal_pose_nav2')] + [
        SetRemap(
            '{}/_action/{}'.format(action, suffix),
            '{}_nav2/_action/{}'.format(action, suffix))
        for action in ('navigate_to_pose', 'navigate_through_poses')
        for suffix in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
    ]

    # amcl reads the shared /map published by a standalone map_server (map_server.launch.py)
    nav_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),
            SetRemap('map', '/map'),
            SetRemap('map_updates', '/map_updates'),
        ] + action_remaps + [
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
                remappings=[('map', '/map')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                             'node_names': ['amcl'], 'bond_timeout': 60.0}]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
                launch_arguments={
                    'namespace': robot_name,
                    'use_sim_time': use_sim_time,
                    'params_file': replaced_params,
                    'autostart': autostart,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items()),
        ],
        scoped=True)

    refiner_group = GroupAction([
        Node(
            package='agv_pro_calibration',
            executable='navigate_to_pose_refiner_proxy',
            name='navigate_to_pose_refiner_proxy',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'nav2_server_timeout_sec': 60.0}],
            remappings=tf_remaps),
        Node(
            package='agv_pro_calibration',
            executable='final_pose_refiner',
            name='final_pose_refiner',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'global_frame': 'map',
                'final_pose_refiner_auto_start_on_nav_success': False,
                'final_pose_refiner_max_wz': 0.65,
                'final_pose_refiner_yaw_tolerance': 0.08,
                'final_pose_refiner_handoff_distance': 0.3,
            }],
            remappings=tf_remaps),
        Node(
            package='agv_pro_multi',
            executable='robot_pose_publisher',
            name='robot_pose_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_id': robot_name,
                'base_frame': base_frame_link,
            }],
            remappings=tf_remaps),
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(src_pkg_dir, 'rviz', 'nav_multi.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen')

    return LaunchDescription([*declared_arguments, nav_group, refiner_group, rviz_node])
