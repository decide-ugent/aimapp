
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
import time

def generate_launch_description():

    bag_dir = LaunchConfiguration('bag_dir', default="enter_path")

    rviz_config_file = os.path.join(get_package_share_directory('aimapp'), 'rviz', 'rviz_config_rosbag.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    time.sleep(4)
    rosbag = ExecuteProcess(
                cmd=['ros2', 'bag', 'play', '--loop', bag_dir],
                output='screen'
            )
    
    ld = LaunchDescription()

    ld.add_action(rviz_node)
    ld.add_action(rosbag)

    return ld