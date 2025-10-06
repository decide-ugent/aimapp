#!/bin/sh

. /opt/ros/humble/setup.sh
. /home/ros2_ws/install/setup.sh

cd /home/ros2_ws
colcon build --packages-select aimapp_actions
colcon build --packages-select aimapp

# Keep the container running
tail -f /dev/null