import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_dm_nav_dir = FindPackageShare('map_dm_nav')
    nav2_bringup_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([FindPackageShare("map_dm_nav"), "params", "warehouse_world.yaml"]),
        description="Full path to the yaml map file",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([map_dm_nav_dir, 'params', 'nav2_husarion.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition':'True',
        }.items(),
    )

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            nav2_bringup_launch,
        ]
    )
