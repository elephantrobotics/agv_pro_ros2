# Requirements:
#   Compile agv_pro_base and agv_pro_bringup packages
#
# Example:
#   Bringup agvpro:
#     $ ros2 launch agv_pro_bringup agv_pro_bringup.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_demos agvpro_scan_demo.launch.py
#
#   Teleop:
#     $ ros2 run teleop_twist_keyboard teleop_twist_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'True' or localization == 'true'
    icp_odometry = LaunchConfiguration('icp_odometry').perform(context)
    icp_odometry = icp_odometry == 'True' or icp_odometry == 'true'
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    arguments = []
    if localization:
        parameters['Mem/IncrementalMemory'] = 'False'
        parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
               
    remappings=[
          ('scan', '/scan')]
    if icp_odometry:
        remappings.append(('odom', 'icp_odom'))
        # modified nav2 params to use icp_odom instead odom frame
        nav2_params_file = PathJoinSubstitution(
            [FindPackageShare('rtabmap_demos'), 'params', 'agvpro_scan_nav2_params.yaml']
        )
    else:
        # original nav2 params
        nav2_params_file = PathJoinSubstitution(
            [FindPackageShare('agv_pro_navigation2'), 'param', 'agvpro.yaml']
        )

    # Directories
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')

    # Paths
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    
    return [
        # Nodes to launch

        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('use_sim_time', 'false'),
                ('params_file', nav2_params_file)
            ]
        ),

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch])
        ),

        # ICP odometry (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('icp_odometry')),
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[parameters, 
                        {'odom_frame_id':'icp_odom',
                         'guess_frame_id':'odom'}],
            remappings=remappings),
        
        # SLAM:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ]

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'icp_odometry', default_value='false',
            description='Launch ICP odometry on top of wheel odometry.'),

        OpaqueFunction(function=launch_setup)
    ])
