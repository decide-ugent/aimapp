import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_pkg_dir = get_package_share_directory('aimapp')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch',  'worlds_launch')
   
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    x_pose = LaunchConfiguration('x', default='0.0')
    y_pose = LaunchConfiguration('y', default='0.0')
    #aws_robomaker_warehouse_smaller_launch
    #no_roof_small_warehouse_launch
    #warehouse_mini_launch
    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/warehouse_mini_launch.py'])
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'namespace':'gazebo',
        }.items()
    )

    rviz_config_file = os.path.join(get_package_share_directory('aimapp'), 'rviz', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()

    ld.add_action(warehouse_world_cmd)

    #ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(rviz_node)
    return ld