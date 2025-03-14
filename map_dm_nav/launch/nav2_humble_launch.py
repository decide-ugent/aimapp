import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    
    nav2_wfd_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_yaml = os.path.join(get_package_share_directory('map_dm_nav'), 'params', 'nav2_humble_params.yaml')
    map_file = os.path.join(get_package_share_directory('map_dm_nav'), 'params', 'warehouse_world.yaml')
    rviz_file = os.path.join(get_package_share_directory('map_dm_nav'), 'rviz', 'nav2_default_view.rviz')


    namespace = "gazebo"
    param_substitutions = {
        'use_sim_time': 'True',
        # 'yaml_filename': map_file
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/odom', 'odom'),
        ('scan', f'/{namespace}/scan'),
        ('/cmd_vel', f'/{namespace}/cmd_vel')
    ]

    # nav2_yaml = RewrittenYaml(
    #     source_file=nav2_yaml1,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True
    # )
    
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
    
    return LaunchDescription([  
        map_server,
        map_saver,
        amcl,
        controller_server,
        planner_server,
        recoveries_server,   
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        # rviz_cmd,
    ])
 
    #TUTO
    # return LaunchDescription([     
    #     Node(
    #         package='nav2_controller',
    #         executable='controller_server',
    #         name='controller_server',
    #         output='screen',
    #         parameters=[controller_yaml]),

    #     Node(
    #         package='nav2_planner',
    #         executable='planner_server',
    #         name='planner_server',
    #         output='screen',
    #         parameters=[planner_yaml]),
            
    #     Node(
    #         package='nav2_behaviors',
    #         executable='behavior_server',
    #         name='recoveries_server',
    #         parameters=[recovery_yaml],
    #         output='screen'),

    #     Node(
    #         package='nav2_bt_navigator',
    #         executable='bt_navigator',
    #         name='bt_navigator',
    #         output='screen',
    #         parameters=[bt_navigator_yaml]),

    #     Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_pathplanner',
    #         output='screen',
    #         parameters=[{'autostart': True},
    #                     {'node_names': ['planner_server',
    #                                     'controller_server',
    #                                     'recoveries_server',
    #                                     'bt_navigator']}]
    #     ),

    #     Node(
    #         package='cartographer_ros', 
    #         executable='cartographer_node', 
    #         name='cartographer_node',
    #         output='screen',
    #         parameters=[{'use_sim_time': True}],
    #         arguments=['-configuration_directory', cartographer_config_dir,
    #                    '-configuration_basename', configuration_basename]

    #     ),

    #     Node(
    #         package='cartographer_ros',
    #         executable='cartographer_occupancy_grid_node',
    #         output='screen',
    #         name='occupancy_grid_node',
    #         parameters=[{'use_sim_time': True}],
    #         arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    #     ),

    #     Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='map_server',
    #         output='screen',
    #         parameters=[{'use_sim_time': True}, 
    #                     {'yaml_filename':map_file}]
    #     ),
            
    #     Node(
    #         package='nav2_amcl',
    #         executable='amcl',
    #         name='amcl',
    #         output='screen',
    #         parameters=[nav2_yaml]
    #     ),
        
    # ])


