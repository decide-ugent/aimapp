#!/bin/bash

# Launch all ROS2 nodes for AUTONOMOUS EXPLORATION
# Created for aimapp project

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)_autonomous"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"

# Terminal 1: Joy2Twist gamepad controller
gnome-terminal --tab --title="Joy2Twist" -- bash -c "
echo 'Starting Joy2Twist gamepad controller...'
ros2 launch joy2twist gamepad_controller.launch.py joy2twist_params_file:=/home/husarion/joy2twist.yaml 2>&1 | tee $LOG_DIR/joy2twist.log
exec bash"

sleep 2

# Terminal 2: Node visualizer
gnome-terminal --tab --title="Visualizer" -- bash -c "
echo 'Starting node visualizer...'
ros2 run aimapp node_visualizer.py 2>&1 | tee $LOG_DIR/node_visualizer.log
exec bash"

sleep 2

# Terminal 3: Start rosbot and shift odom
gnome-terminal --tab --title="Rosbot-Odom" -- bash -c "
echo 'Starting rosbot...'
bash start_rosbot.sh 2>&1 | tee $LOG_DIR/start_rosbot.log
echo 'Shifting husarion odom...'
ros2 run aimapp shift_husarion_odom.py 0.0 0.0 2>&1 | tee $LOG_DIR/shift_odom.log
exec bash"

sleep 2

# Terminal 4: Nav2 husarion launch
gnome-terminal --tab --title="Nav2" -- bash -c "
echo 'Starting Nav2 husarion...'
ros2 launch aimapp nav2_husarion_launch.py 2>&1 | tee $LOG_DIR/nav2_husarion.log
exec bash"

sleep 2

# Terminal 5: FULL Agent launch (autonomous)
gnome-terminal --tab --title="Agent-Auto" -- bash -c "
echo 'Starting AUTONOMOUS agent...'
echo 'The agent will explore fully autonomously without manual intervention.'
ros2 launch aimapp agent_launch.py 2>&1 | tee $LOG_DIR/agent_launch.log
exec bash"

sleep 2

# Terminal 6: Save data
gnome-terminal --tab --title="SaveData" -- bash -c "
echo 'Starting save data node...'
ros2 run aimapp save_data.py 2>&1 | tee $LOG_DIR/save_data.log
exec bash"

echo "All terminals launched for AUTONOMOUS EXPLORATION. Check logs in: $LOG_DIR"
