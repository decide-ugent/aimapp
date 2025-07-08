cmake_minimum_required(VERSION 3.5)
project(map_dm_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(map_dm_nav_actions REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  #service_interface/potential_field_server.py #unused
  map_dm_nav/motion/potential_field_client.py
  map_dm_nav/motion/potential_field_action.py
  map_dm_nav/motion/move_straight_client.py
  map_dm_nav/motion/move_straight_action.py
  map_dm_nav/motion/nav2_client.py
  map_dm_nav/motion/align_odom_to_belief.py
  map_dm_nav/obs_transf/get_panorama_action.py
  map_dm_nav/obs_transf/get_panorama_client.py
  map_dm_nav/obs_transf/get_360_camera_action.py
  map_dm_nav/obs_transf/get_360_camera_remote_action.py
  map_dm_nav/obs_transf/get_360_camera_client.py
  map_dm_nav/obs_transf/get_pano_multiple_camera_action.py
  map_dm_nav/obs_transf/get_pano_multiple_camera_client.py

  map_dm_nav/main.py
  map_dm_nav/action_process_no_motion.py

  simulation_tools/remove_turtlebot3_from_gazebo.py
  simulation_tools/reset_turtlebot3_odom.py
  simulation_tools/simulation_overview_data_save.py
  simulation_tools/shift_husarion_odom.py
  simulation_tools/node_visualizer.py

  map_dm_nav/obs_transf/save_cam_in_folder.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params)
  # install(DIRECTORY rviz/
  # DESTINATION share/${PROJECT_NAME}/rviz)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
