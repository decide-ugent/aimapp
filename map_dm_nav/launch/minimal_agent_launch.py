from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration

DESKTOP_RESULTS_FOLDER=os.getcwd()

def generate_launch_description():
    ld =  LaunchDescription()

    test_id = LaunchConfiguration('test_id', default='None')
    
    panorama_360_action = Node(
            package='map_dm_nav',
            # namespace='agent',
            executable='get_360_camera_action.py',
            remappings=[
                ('/odom','/odometry/filtered'),
                ('/scan', '/scan_filtered'),
            ]
        )
    

    aif_process = Node(
            package='map_dm_nav',
            executable='action_process_no_motion.py',
            arguments=['-test_id',test_id]
            # namespace='agent',
            # remappings=[
            #     ('/odom','/odometry/filtered'),
            #     ('/scan', '/scan'),
            # ]
        )

    ld.add_action(aif_process)
    ld.add_action(panorama_360_action)
    # ld.add_action(rosbag)
    return ld