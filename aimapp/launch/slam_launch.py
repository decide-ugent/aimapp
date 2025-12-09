import os
import ast

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction)
from launch.substitutions import (LaunchConfiguration)
from launch_ros.actions import LifecycleNode

def launch_slam_with_params(context, *args, **kwargs):
    """Function to properly parse map_start_pose array"""
    use_sim_time = LaunchConfiguration('use_sim_time')
    aimapp_dir = get_package_share_directory('aimapp')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')
    map_start_pose_str = LaunchConfiguration('map_start_pose').perform(context)

    # Parse the map_start_pose string to a list of floats
    try:
        map_start_pose_str = map_start_pose_str.strip()
        if not map_start_pose_str.startswith('['):
            map_start_pose_str = '[' + map_start_pose_str + ']'
        map_start_pose = ast.literal_eval(map_start_pose_str)

        # Ensure it's a list of floats
        map_start_pose = [float(x) for x in map_start_pose]
    except:
        # Fall back to default if parsing fails
        map_start_pose = [0.0, 0.0, 0.0]
        print(f"Warning: Could not parse map_start_pose '{map_start_pose_str}', using default [0.0, 0.0, 0.0]")

    # Déclare le node slam_toolbox with properly parsed parameters
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,  # Load existing map if provided
                'map_start_pose': map_start_pose,  # Parsed as actual list of floats
                'map_start_at_dock': False
            }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    return [start_async_slam_toolbox_node]


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

    # Use OpaqueFunction to properly parse the map_start_pose parameter
    launch_slam_node = OpaqueFunction(function=launch_slam_with_params)

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_map_start_pose_cmd)
    ld.add_action(launch_slam_node)

    return ld

