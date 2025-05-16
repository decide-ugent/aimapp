from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration

DESKTOP_RESULTS_FOLDER=os.getcwd()


load_model = LaunchConfiguration('model_dir', default='None')
goal_path = LaunchConfiguration('goal_path', default='None')

def generate_launch_description():
    ld =  LaunchDescription()
    agent_node = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='main.py',
            arguments=['-load_model',load_model, '-goal_path',goal_path]
        )
    panorama_action = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='get_pano_multiple_camera_action.py',
            remappings=[
                ('/agent/cmd_vel', '/cmd_vel'),
                ('/agent/odom','/odometry/filtered'),
                ('/agent/scan', '/scan_filtered'),
                ('/agent/camera_front/image_raw', '/XXX/camera_front/image_raw'),
                ('/agent/camera_right/image_raw', '/XXX/camera_right/image_raw'),
                ('/agent/camera_left/image_raw', '/XXX/camera_left/image_raw'),
            ]
        )

    panorama_360_action = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='get_360_camera_action.py',
            remappings=[
                ('/agent/odom','/odometry/filtered'),
                ('/agent/scan', '/scan_filtered'),
            ]
        )
    

    potential_flied_action = Node(
            package='map_dm_nav',
            executable='potential_field_action.py',
            namespace='agent',
            remappings=[
                ('/agent/cmd_vel','/cmd_vel'),
                ('/agent/odom','/odometry/filtered'),
                ('/agent/scan','/scan_filtered'),
            ]
        )

    ld.add_action(agent_node)
    # ld.add_action(panorama_action)
    ld.add_action(potential_flied_action)
    ld.add_action(panorama_360_action)
    # ld.add_action(rosbag)
    return ld