from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration

DESKTOP_RESULTS_FOLDER=os.getcwd()

def generate_launch_description():
    ld =  LaunchDescription()

    test_id = LaunchConfiguration('test_id', default='None')
    goal_ob_id = LaunchConfiguration('goal_ob_id', default='-1')
    goal_pose_id = LaunchConfiguration('goal_pose_id', default='-1')
    start_node_id = LaunchConfiguration('start_node_id', default='-1')
    influence_radius = LaunchConfiguration('influence_radius', default='1.6')
    n_actions = LaunchConfiguration('n_actions', default='17')
    lookahead_node_creation = LaunchConfiguration('lookahead_node_creation', default='8')
    skip_double_check = LaunchConfiguration('skip_double_check', default='false')

    
    panorama_360_action = Node(
            package='aimapp',
            # namespace='agent',
            executable='get_360_camera_action.py',
            remappings=[
                ('/odom','/odometry/filtered'),
                ('/scan', '/scan_filtered'),
            ]
        )
    

    aif_process = Node(
            package='aimapp',
            executable='action_process_no_motion.py',
            arguments=['-test_id',test_id,
                      '-goal_ob_id', goal_ob_id, '-goal_pose_id', goal_pose_id,
                      '-start_node_id', start_node_id,
                      '-influence_radius', influence_radius,
                      '-n_actions', n_actions,
                      '-lookahead_node_creation', lookahead_node_creation,
                      '-skip_double_check', skip_double_check]
            # namespace='agent',
            # remappings=[
            #     ('/odom','/odometry/filtered'),
            #     ('/scan', '/scan'),
            # ]
        )
    
    goal_client = Node(
            package='aimapp',
            executable='nav2_client.py',
            arguments=['-continuous']
            # namespace='agent',
            # remappings=[
            #     ('/odom','/odometry/filtered'),
            #     ('/scan', '/scan'),
            # ]
        )

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'test_id',
        default_value='None',
        description='Test ID for continuing previous experiment'
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_ob_id',
        default_value='-1',
        description='Goal observation ID (-1 if not specified)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'goal_pose_id',
        default_value='-1',
        description='Goal pose ID (-1 if not specified)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'start_node_id',
        default_value='-1',
        description='Starting node ID in the model (-1 = use latest node)'
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
    ld.add_action(DeclareLaunchArgument(
        'skip_double_check',
        default_value='false',
        description='Skip double observation check for visited states (faster but less robust)'
    ))

    ld.add_action(aif_process)
    ld.add_action(panorama_360_action)
    # ld.add_action(goal_client)
    return ld