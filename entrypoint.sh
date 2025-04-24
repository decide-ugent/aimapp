#!/bin/sh

. /opt/ros/humble/setup.sh
. /home/ros2_ws/install/setup.sh

cd /home/ros2_ws
colcon build --packages-select map_dm_nav_actions
colcon build --packages-select map_dm_nav

# Keep the container running
tail -f /dev/null