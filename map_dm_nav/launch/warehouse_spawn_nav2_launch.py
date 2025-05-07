import os
import time
import re


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='gazebo')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='gazebo',
        description='Namespace of the robot'
    )

    
    #==========================  WORLD ====================================#
    warehouse_pkg_dir = get_package_share_directory('map_dm_nav')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch',  'worlds_launch')
    #aws_robomaker_warehouse_smaller_launch
    #no_roof_small_warehouse_launch
    #warehouse_mini_launch
    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/aws_robomaker_warehouse_smaller_launch.py']),
        launch_arguments={
            'headless': 'False',
        }.items()
    )

    
    #==========================  ROBOT ====================================#
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi_plus'
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    x_pose = LaunchConfiguration('x', default='0.0')
    y_pose = LaunchConfiguration('y', default='0.0')
    
    declare_x_pose = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position for spawn'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position for spawn'
    )
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
        cmd=[os.path.join(get_package_share_directory('map_dm_nav').replace(\
            'share', 'lib'), 'remove_turtlebot3_from_gazebo.py'), namespace],
        name='remove_turtlebot3',
        output='screen',
    )

    reset_odometry_cmd = Node(
        package='map_dm_nav',  # Replace with your package name
        executable='reset_turtlebot3_odom.py',  # Ensure this matches your script name
        output='screen',
        arguments=[
            '-x', x_pose,
            '-y', y_pose,
            '-namespace', namespace,
        ],
        remappings=[
            ('/odom', 'gazebo/odom')
        ]
    )


    #==========================  NAV2 ====================================#
    namespace = "gazebo"
    
    nav2_yaml = os.path.join(get_package_share_directory('map_dm_nav'), 'params', 'nav2_humble_params.yaml')
    map_file = os.path.join(get_package_share_directory('map_dm_nav'), 'params', 'warehouse_world.yaml')
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/odom', 'odom'),
        ('scan', f'/{namespace}/scan'),
        ('/cmd_vel', f'/{namespace}/cmd_vel')
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename':map_file}],
        remappings=remappings
    )
        
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml],
        remappings=remappings
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml],
        remappings=remappings)

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml],
        remappings=remappings)
        
    recoveries_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_yaml],
        output='screen',
        remappings=remappings)

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml],
        remappings=remappings)

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_yaml],
        remappings=remappings
        )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[nav2_yaml])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server',
                                    'map_saver',
                                    'amcl',
                                    'controller_server',
                                    'planner_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'waypoint_follower',
                
                    ]}])

    #========================== AGENt =====================================#


    load_model = LaunchConfiguration('model_dir', default='None')
    goal_path = LaunchConfiguration('goal_path', default='None')

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
    

    #========================== LAUNCH ====================================#
    ld = LaunchDescription()

    # First declare the arguments:
    ld.add_action(declare_namespace)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)


    ld.add_action(warehouse_world_cmd)

    ld.add_action(
        TimerAction(
            period=30.0,
            actions=[remove_turtlebot_cmd]
        )
    )
    # Add a handler to launch the spawn process after the removal process completes
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=remove_turtlebot_cmd,
            on_exit=[spawn_turtlebot_cmd]
        )
    ))
    # Also, reset odometry after 60 sec (optional: depends if you want to wait)
    ld.add_action(
        TimerAction(
            period=30.0,
            actions=[reset_odometry_cmd]
        )
    )

    # Wait 120 seconds from beginning to start Nav2 stack (map server etc)
    ld.add_action(
        TimerAction(
            period=30.0,
            actions=[
                map_server,
                map_saver,
                amcl,
                controller_server,
                planner_server,
                recoveries_server,
                bt_navigator,
                waypoint_follower,
                lifecycle_manager
            ]
        )
    )

    ld.add_action(
        TimerAction(
            period=100.0,
            actions=[
                agent_node,
                panorama_action,
                potential_flied_action
            ]
        )
    )

   
    return ld