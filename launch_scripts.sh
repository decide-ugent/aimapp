#!/bin/bash
sudo apt update
sudo apt install xterm python3-pip
sudo apt install ros-humble-colcon-common-extensions

cd /project_ghent/ddtinguy/ros2_ws
rosdep fix-permissions && rosdep update --include-eol-distros \
    && rosdep install --from-paths ./ -i -y --rosdistro humble --ignore-src

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /project_ghent/ddtinguy/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export NUMBA_THREADING_LAYER=workqueue" >> ~/.bashrc

source /opt/ros/humble/setup.sh
colcon build
source /opt/ros/humble/setup.sh
source install/setup.bash

cd /project_ghent/ddtinguy/ros2_ws/src/map_dm_nav/map_dm_nav/map_dm_nav
pip install .

cd /project_ghent/ddtinguy/ros2_ws
export GAZEBO_MODEL_PATH=`pwd`/models
source /usr/share/gazebo/setup.bash
ros2 launch map_dm_nav warehouse_spawn_nav2_launch.py x:=0.0 y:=0.0

sleep 30

xterm -hold -e "
    cd /project_ghent/ddtinguy/ros2_ws;
    ros2 launch map_dm_nav agent_launch.py;
    exec bash" &

sleep 10
xterm -hold -e "
    cd /project_ghent/ddtinguy/ros2_ws;
    ros2 run map_dm_nav simulation_overview_data_save.py;
    exec bash" &