# Requirements:
#   Compile agv_pro_base and agv_pro_bringup packages
#
# Example:
#   Bringup agvpro:
#     $ ros2 launch agv_pro_bringup agv_pro_bringup.launch.py
#
#   Bringup orbbec gemini2 camera:
#     $ ros2 launch orbbec_camera gemini2.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_demos agvpro_rgbd_demo.launch.py
#
#   Teleop:
#     $ ros2 run teleop_twist_keyboard teleop_twist_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image',       '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image',     '/camera/depth/image_raw')]
    
    # Directories
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('rtabmap_demos'), 'params', 'agvpro_rgbd_nav2_params.yaml']
    )

    # Paths
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

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

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
        
        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/depth_registered/points')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
            remappings=[('cloud', '/camera/depth_registered/points'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])
