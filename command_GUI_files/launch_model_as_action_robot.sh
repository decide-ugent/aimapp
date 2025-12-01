#!/bin/bash

# Launch all ROS2 nodes in separate terminals with logging on remote robot
# Created for aimapp project

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="10.10.131.145"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_ROS_DIR="ros2_ws"

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"
echo "Connecting to robot at: $ROBOT_SSH"

# Check if SSH key-based authentication is set up
echo "Checking SSH connection..."
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT_SSH" echo "SSH connection successful" 2>/dev/null; then
    echo "ERROR: Cannot connect to robot via SSH without password."
    echo "Please set up SSH key-based authentication first:"
    echo ""
    echo "1. Generate SSH key (if you don't have one):"
    echo "   ssh-keygen -t rsa -b 4096"
    echo ""
    echo "2. Copy your public key to the robot:"
    echo "   ssh-copy-id $ROBOT_SSH"
    echo ""
    echo "3. Test the connection:"
    echo "   ssh $ROBOT_SSH 'echo Connection successful'"
    echo ""
    echo "After setting up passwordless SSH, run this script again."
    exit 1
fi

echo "SSH connection verified. Launching terminals..."

# Terminal 1: Joy2Twist gamepad controller
gnome-terminal --tab --title="Joy2Twist" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Starting Joy2Twist gamepad controller...;
ros2 launch joy2twist gamepad_controller.launch.py joy2twist_params_file:=/home/husarion/joy2twist.yaml 2>&1
\"'
exec bash"


sleep 2

# Terminal 3: Start rosbot and shift odom
gnome-terminal --tab --title="Rosbot-Odom" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
echo Starting rosbot...;
bash start_rosbot.sh 2>&1;
echo "To attach: tmux attach -t rosbot_startup "
echo "To switch windows: Press Ctrl-b, then n or p"
echo "To detach: Press Ctrl-b, then d"
echo "To kill the session: tmux kill-session -t rosbot_startup"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Shifting husarion odom...;
ros2 run aimapp shift_husarion_odom.py 0.0 0.0 2>&1
\"'
exec bash"

sleep 2

# Terminal 4: Nav2 husarion launch
gnome-terminal --tab --title="Nav2" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Starting Nav2 husarion...;
ros2 launch aimapp nav2_husarion_launch.py 2>&1
\"'
exec bash"

sleep 2

# Terminal 5: Minimal agent launch
gnome-terminal --tab --title="Agent" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Starting minimal agent...;
ros2 launch aimapp minimal_agent_launch.py 2>&1
\"'
exec bash"

sleep 2

# Terminal 2: Node visualizer
gnome-terminal --tab --title="Visualizer" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Starting node visualizer...;
ros2 run aimapp node_visualizer.py 2>&1
\"'
exec bash"

sleep 2

# Terminal 6: Save data
gnome-terminal --tab --title="SaveData" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Starting save data node...;
ros2 run aimapp save_data.py 2>&1
\"'
exec bash"

sleep 2


# Terminal 7: AIF Process action (with placeholders to fill)
gnome-terminal --tab --title="AIF-Action" -- bash -c "
ssh -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR
source install/setup.bash
echo Waiting to send AIF Process action...;
echo Fill in the goal_reached and action values below:;
echo   goal_reached: TOFILL;
echo   action: TOFILL;
echo;
echo Example command:;
echo ros2 action send_goal aif_process aimapp_actions/action/AIFProcess \\\"{goal_reached: true, action: 1}\\\";
echo;
read -p Press\ Enter\ to\ send\ action;
ros2 action send_goal aif_process aimapp_actions/action/AIFProcess \\\"{goal_reached: TOFILL, action: TOFILL}\\\" 2>&1
\"'
exec bash"

sleep 2

# Terminal 8: Action Selector GUI - runs locally, not on robot
gnome-terminal --tab --title="Action-Selector-GUI" -- bash -c "
echo 'Starting Action Selector GUI (local)...'
echo 'This GUI will display possible actions from AIF Process'
echo 'and allow you to select which pose to navigate to next.'
echo ''
echo 'Make sure ROS_DOMAIN_ID matches the robot!'
echo ''
python3 src/aimapp/command_GUI_files/action_selector_with_subscriber.py 2>&1
exec bash"

sleep 2

# Terminal 9: Interactive terminal for sending goals (backup/manual method)
# gnome-terminal --tab --title="Send-Goals-Manual" -- bash -c "
# ssh -X $ROBOT_SSH 'bash -l -c \"
# echo ==========================================;
# echo Manual Terminal for sending navigation goals;
# echo ==========================================;
# echo;
# echo RECOMMENDED: Use the Action Selector GUI instead!;
# echo;
# echo The nav2_client.py is running in continuous mode.;
# echo Send pose goals using the GUI or topic interface:;
# echo;
# echo Example commands:;
# echo   ros2 topic pub --once /nav2_client_goal_pose geometry_msgs/msg/PoseStamped \\\"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}\\\";
# echo;
# echo ==========================================;
# echo
# \"'
# exec bash"

echo "All terminals launched. Check logs in: $LOG_DIR"
echo "Note: GUI runs locally, all other nodes run on robot at $ROBOT_SSH"
