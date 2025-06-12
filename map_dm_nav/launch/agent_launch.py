from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
import re
from launch.substitutions import LaunchConfiguration

DESKTOP_RESULTS_FOLDER=os.getcwd()


model_dir = LaunchConfiguration('model_dir', default='None')
goal_path = LaunchConfiguration('goal_path', default='None')

def find_folder_with_highest_number(directory_path):
    # List all directories in the given directory
    folders = [f for f in os.listdir(directory_path) if os.path.isdir(os.path.join(directory_path, f))]
    # Pattern to match folder names with numerical suffixes
    pattern = re.compile(r'(\d+)$')
    highest_number = -1
    highest_folder_id = -1
    
    for folder in folders:
        match = pattern.search(folder)
        if match:
            number = int(match.group(1))
            if number > highest_number:
                highest_number = number
                highest_folder_id = folder
  
    return highest_folder_id

highest_folder_id = int(find_folder_with_highest_number(DESKTOP_RESULTS_FOLDER+'/tests'))

def generate_launch_description():
    ld =  LaunchDescription()
    agent_node = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='main.py',
            arguments=['-model_dir',model_dir, '-goal_path',goal_path]
        )
    panorama_action = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='get_pano_multiple_camera_action.py',
            remappings=[
                ('/agent/cmd_vel', '/gazebo/cmd_vel'),
                ('/agent/odom','/odom'),
                ('/agent/scan', '/gazebo/scan'),
                ('/agent/camera_front/image_raw', '/gazebo/camera_front/image_raw'),
                ('/agent/camera_right/image_raw', '/gazebo/camera_right/image_raw'),
                ('/agent/camera_left/image_raw', '/gazebo/camera_left/image_raw'),
            ]
        )

    panorama_360_action = Node(
            package='map_dm_nav',
            namespace='agent',
            executable='get_360_camera_action.py',
            remappings=[
                ('/agent/odom','/odom'),
                ('/agent/scan', '/gazebo/scan'),
            ]
        )
    

    potential_flied_action = Node(
            package='map_dm_nav',
            executable='potential_field_action.py',
            namespace='agent',
            remappings=[
                ('/agent/cmd_vel','/gazebo/cmd_vel'),
                ('/agent/odom','/odom'),
                ('/agent/scan','/gazebo/scan'),
            ]
        )
    # rosbag = ExecuteProcess(
    #         cmd=['ros2', 'bag', 'record', 'world/camera/image_raw', '-o', 'tests/above_view_test_'+str(highest_folder_id+1)],
    #     )
   
    ld.add_action(agent_node)
    ld.add_action(panorama_action)
    ld.add_action(potential_flied_action)
    # ld.add_action(panorama_360_action)
    # ld.add_action(rosbag)
    return ld