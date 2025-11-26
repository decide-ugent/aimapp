#!/bin/bash

# Launch all ROS2 nodes in separate terminals with logging
# Created for aimapp project

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"

# Terminal 1: Joy2Twist gamepad controller
gnome-terminal --tab --title="Joy2Twist" -- bash -c "
echo 'Starting Joy2Twist gamepad controller...'
ros2 launch joy2twist gamepad_controller.launch.py joy2twist_params_file:=/home/husarion/joy2twist.yaml 2>&1 
exec bash"
# | tee $LOG_DIR/joy2twist.log

sleep 2

# Terminal 2: Node visualizer
gnome-terminal --tab --title="Visualizer" -- bash -c "
echo 'Starting node visualizer...'
ros2 run aimapp node_visualizer.py 2>&1
exec bash"

sleep 2

# Terminal 3: Start rosbot and shift odom
gnome-terminal --tab --title="Rosbot-Odom" -- bash -c "
echo 'Starting rosbot...'
bash start_rosbot.sh 2>&1 
echo 'Shifting husarion odom...'
ros2 run aimapp shift_husarion_odom.py 0.0 0.0 2>&1 
exec bash"

sleep 2

# Terminal 4: Nav2 husarion launch
gnome-terminal --tab --title="Nav2" -- bash -c "
echo 'Starting Nav2 husarion...'
ros2 launch aimapp nav2_husarion_launch.py 2>&1 
exec bash"

sleep 2

# Terminal 5: Minimal agent launch
gnome-terminal --tab --title="Agent" -- bash -c "
echo 'Starting minimal agent...'
ros2 launch aimapp minimal_agent_launch.py 2>&1 
exec bash"
#| tee $LOG_DIR/minimal_agent.log
sleep 2

# Terminal 6: Save data
gnome-terminal --tab --title="SaveData" -- bash -c "
echo 'Starting save data node...'
ros2 run aimapp save_data.py 2>&1 
exec bash"

sleep 2

# Terminal 7: AIF Process action (with placeholders to fill)
gnome-terminal --tab --title="AIF-Action" -- bash -c "
echo 'Waiting to send AIF Process action...'
echo 'Fill in the goal_reached and action values below:'
echo '  goal_reached: TOFILL'
echo '  action: TOFILL'
echo ''
echo 'Example command:'
echo 'ros2 action send_goal aif_process map_dm_nav_actions/action/AIFProcess \"{goal_reached: true, action: 1}\"'
echo ''
read -p 'Press Enter to send action'
ros2 action send_goal aif_process map_dm_nav_actions/action/AIFProcess \"{goal_reached: TOFILL, action: TOFILL}\" 2>&1 
exec bash"

sleep 2

sleep 2

# Terminal 8: Interactive terminal for sending goals
gnome-terminal --tab --title="Send-Goals" -- bash -c "
echo '=========================================='
echo 'Terminal for sending navigation goals'
echo '=========================================='
echo ''
echo '' ## OPTION 1 ##
echo 'The nav2_client_node_goal.py is running in continuous mode.'
echo 'Send goals (must be adjacent nodes) using the topic interface:'
echo ''
echo 'Example commands:'
echo '  ros2 topic pub --once /nav2_client_goal_node std_msgs/msg/Int32 \"data: 1\"'
echo ''
echo 'Replace the data value with your desired goal node number.'
echo ''
echo '' ## OPTION 2 ##
echo '' Use /send_goal.sh node_goal
echo ''
echo 'Example commands:'
echo '' /send_goal.sh 3  
echo '=========================================='
echo ''
exec bash"

#echo 'Check logs in: $LOG_DIR'
