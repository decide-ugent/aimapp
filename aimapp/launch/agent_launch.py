from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration

DESKTOP_RESULTS_FOLDER=os.getcwd()


model_dir = LaunchConfiguration('model_dir', default='None')
goal_path = LaunchConfiguration('goal_path', default='None')
influence_radius = LaunchConfiguration('influence_radius', default='1.6')
n_actions = LaunchConfiguration('n_actions', default='17')
lookahead_node_creation = LaunchConfiguration('lookahead_node_creation', default='8')

def generate_launch_description():
    ld =  LaunchDescription()
    agent_node = Node(
            package='aimapp',
            namespace='agent',
            executable='main.py',
            arguments=['-model_dir',model_dir, '-goal_path',goal_path,
                      '-influence_radius', influence_radius,
                      '-n_actions', n_actions,
                      '-lookahead_node_creation', lookahead_node_creation]
        )
    panorama_action = Node(
            package='aimapp',
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
            package='aimapp',
            namespace='agent',
            executable='get_360_camera_action.py', #'get_360_camera_remote_action.py
            remappings=[
                ('/agent/odom','/odometry/filtered'),
                ('/agent/scan', '/scan_filtered'),
            ]
        )
    

    potential_flied_action = Node(
            package='aimapp',
            executable='potential_field_action.py',
            namespace='agent',
            remappings=[
                ('/agent/cmd_vel','/cmd_vel'),
                ('/agent/odometry/filtered','/odometry/filtered'),
                ('/agent/scan_filtered','/scan_filtered'),
            ]
        )
    
    move_straight_action = Node(
            package='aimapp',
            executable='move_straight_action.py',
            namespace='agent',
            remappings=[
                ('/agent/cmd_vel','/cmd_vel'),
                ('/agent/odometry/filtered','/odometry/filtered'),
                # ('/agent/scan_filtered','/scan_filtered'),
            ]
        )

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'model_dir',
        default_value='None',
        description='Directory containing saved model'
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_path',
        default_value='None',
        description='Path to goal file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'influence_radius',
        default_value='1.6',
        description='Influence radius for exploration model'
    ))
    ld.add_action(DeclareLaunchArgument(
        'n_actions',
        default_value='17',
        description='Number of actions for the exploration model'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lookahead_node_creation',
        default_value='8',
        description='Lookahead distance for node creation in exploration'
    ))

    ld.add_action(agent_node)
    # ld.add_action(panorama_action)
    # ld.add_action(potential_flied_action)
    # ld.add_action(move_straight_action)
    ld.add_action(panorama_360_action)
    # ld.add_action(rosbag)
    return ld