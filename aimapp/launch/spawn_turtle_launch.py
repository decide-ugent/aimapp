import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

# Set the environment variable
os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi_plus'

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    x_pose = LaunchConfiguration('x', default='0.0')
    y_pose = LaunchConfiguration('y', default='0.0')
    namespace = LaunchConfiguration('namespace', default='gazebo')
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'namespace': namespace,
        }.items()
    )

    remove_turtlebot_cmd = ExecuteProcess(
        cmd=[os.path.join(get_package_share_directory('aimapp').replace(\
            'share', 'lib'), 'remove_turtlebot3_from_gazebo.py'), namespace],
        name='remove_turtlebot3',
        output='screen',
    )

    reset_odometry_cmd = Node(
        package='aimapp',  # Replace with your package name
        executable='reset_turtlebot3_odom.py',  # Ensure this matches your script name
        output='screen',
        arguments=[
            '-x', x_pose,
            '-y', y_pose,
            '-namespace', namespace,
        ],
        # remappings=[
        #     ('/odom', 'gazebo/odom')
        # ]
    )


    ld = LaunchDescription()

    ld.add_action(remove_turtlebot_cmd)
    
    # Add a handler to launch the spawn process after the removal process completes
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=remove_turtlebot_cmd,
            on_exit=[spawn_turtlebot_cmd]
        )
    ))

    ld.add_action(reset_odometry_cmd)
    # Add the TurtleBot3 robot state publisher and spawn actions
    #ld.add_action(spawn_turtlebot_cmd)
    return ld