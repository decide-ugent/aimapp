import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument)
from launch.substitutions import (LaunchConfiguration)
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    aimapp_dir = get_package_share_directory('aimapp')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')
    map_start_pose = LaunchConfiguration('map_start_pose')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(aimapp_dir,
                                    'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to existing map file (.yaml) for localization mode. If empty, starts in mapping mode.')

    declare_map_start_pose_cmd = DeclareLaunchArgument(
        'map_start_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='Initial pose [x, y, theta] for continuing from a saved map')

    # Déclare le node slam_toolbox
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,  # Load existing map if provided
                'map_start_pose': map_start_pose,  # Initial pose for continuing mapping
                'map_start_at_dock': False
            }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_map_start_pose_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld

