#!/bin/bash

# Launch all ROS2 nodes in separate terminals with logging on remote robot
# Created for aimapp project
#
# Usage: ./launch_model_as_action_robot.sh [test_id]
# Example: ./launch_model_as_action_robot.sh 5

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="10.10.131.145"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_ROS_DIR="ros2_ws"

# Get test_id from command line argument (optional)
TEST_ID="${1:-None}"

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"
echo "Connecting to robot at: $ROBOT_SSH"
echo "Test ID: $TEST_ID"

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

# Extract initial pose from model if TEST_ID is provided
ODOM_X="0.0"
ODOM_Y="0.0"

if [ "$TEST_ID" != "None" ]; then
    echo "Extracting initial pose from test $TEST_ID..."
    echo "Connecting to robot and searching for model file..."

    # Create a Python script to extract current_pose from model.pkl
    POSE_OUTPUT=$(ssh $ROBOT_SSH "cd $ROBOT_ROS_DIR && source install/setup.bash && python3 -c \"
import pickle
import os
import sys

test_id = '$TEST_ID'
test_dir = os.path.join('tests', test_id)

# Print debug info to stderr so it doesn't interfere with coordinate output
print(f'DEBUG: Current directory: {os.getcwd()}', file=sys.stderr)
print(f'DEBUG: Looking for test directory: {test_dir}', file=sys.stderr)

if not os.path.exists(test_dir):
    print(f'ERROR: Test directory not found: {test_dir}', file=sys.stderr)
    sys.exit(1)

model_path = os.path.join(test_dir, 'model.pkl')
print(f'DEBUG: Looking for model at: {model_path}', file=sys.stderr)

if not os.path.exists(model_path):
    print(f'ERROR: model.pkl not found in {test_dir}', file=sys.stderr)
    sys.exit(1)

try:
    print('DEBUG: Loading model.pkl...', file=sys.stderr)
    with open(model_path, 'rb') as f:
        model = pickle.load(f)

    print('DEBUG: Model loaded successfully', file=sys.stderr)

    if hasattr(model, 'current_pose'):
        pose = model.current_pose
        print(f'DEBUG: Found current_pose: {pose}', file=sys.stderr)
        # Output only the coordinates to stdout (this gets captured)
        print(f'{pose[0]} {pose[1]}')
    else:
        print('ERROR: model.current_pose attribute not found', file=sys.stderr)
        sys.exit(1)
except Exception as e:
    print(f'ERROR: Failed to load model: {e}', file=sys.stderr)
    sys.exit(1)
\" 2>&1")

    # Capture the exit code
    EXIT_CODE=$?

    # Display all output (includes both stdout and stderr due to 2>&1)
    echo "$POSE_OUTPUT"

    if [ $EXIT_CODE -eq 0 ]; then
        # Parse the output - get the last line which should be the coordinates
        COORDS=$(echo "$POSE_OUTPUT" | tail -n 1)
        ODOM_X=$(echo $COORDS | awk '{print $1}')
        ODOM_Y=$(echo $COORDS | awk '{print $2}')

        # Check if we got valid numbers
        if [[ "$ODOM_X" =~ ^-?[0-9]+\.?[0-9]*$ ]] && [[ "$ODOM_Y" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then
            echo "SUCCESS!: Extracted pose from model: x=$ODOM_X, y=$ODOM_Y"
        else
            echo "WARNING: Could not parse coordinates, using default (0.0, 0.0)"
            ODOM_X="0.0"
            ODOM_Y="0.0"
        fi
    else
        echo "WARNING: Could not extract pose from model, using default (0.0, 0.0)"
        ODOM_X="0.0"
        ODOM_Y="0.0"
    fi
fi

echo "Using odometry shift: x=-$ODOM_X, y=-$ODOM_Y"

# Calculate negated values for odometry shift
NEG_ODOM_X=$(python3 -c "print(-float('$ODOM_X'))")
NEG_ODOM_Y=$(python3 -c "print(-float('$ODOM_Y'))")





# # # Terminal 1: Joy2Twist gamepad controller
gnome-terminal --tab --title="Joy2Twist" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting Joy2Twist gamepad controller...\"
echo \"Press Ctrl-C to stop this node\"
ros2 launch joy2twist gamepad_controller.launch.py joy2twist_params_file:=/home/husarion/joy2twist.yaml 2>&1
bash
'"


sleep 2

# Terminal 3: Start rosbot and shift odom
gnome-terminal --tab --title="Rosbot-Odom" -- bash -c "
ssh -t -X $ROBOT_SSH '
source $ROBOT_ROS_DIR/install/setup.bash
echo \"Starting rosbot!\"
bash start_rosbot.sh 2>&1
echo \"To attach: tmux attach -t rosbot_startup\"
echo \"To switch windows: Press Ctrl-b, then n or p\"
echo \"To detach: Press Ctrl-b, then d\"
echo \"To kill the session: tmux kill-session -t rosbot_startup\"
echo \"\"
echo \"Press Ctrl-C to stop this node\"
cd $ROBOT_ROS_DIR
ros2 run aimapp shift_husarion_odom.py $NEG_ODOM_X $NEG_ODOM_Y 2>&1  
bash
'"

# # This shift is useful when we do not restart the robot but just the model (thus initial position is not 0.0 0.0)


# # echo \"\"
# # echo \"Waiting for EKF node to be ready...\"
# # sleep 3
# # echo \"Setting initial EKF pose to x=$ODOM_X, y=$ODOM_Y using set_pose service...\"
# # ros2 topic pub --once /set_pose geometry_msgs/msg/PoseWithCovarianceStamped \"{
# #   header: {
# #     stamp: {sec: 0, nanosec: 0},
# #     frame_id: 'odom'
# #   },
# #   pose: {
# #     pose: {
# #       position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0},
# #       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
# #     },
# #     covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
# #                  0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
# #                  0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
# #                  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
# #                  0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
# #                  0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
# #   }
# # }\"
# # echo \"EKF initial pose set successfully!\"
# # echo \"\"
# # echo \"Shifting husarion odom with x=$NEG_ODOM_X, y=$NEG_ODOM_Y...\"
# # echo \"Press Ctrl-C to stop this node\"

sleep 2

# Determine which SLAM map to load
SLAM_MAP_ARG=""
if [ "$TEST_ID" != "None" ]; then
    echo "Checking for saved SLAM map from test $TEST_ID..."
    # Check if SLAM map exists on robot
    SLAM_MAP_EXISTS=$(ssh $ROBOT_SSH "test -f $ROBOT_ROS_DIR/tests/$TEST_ID/slam_map.posegraph && echo 'yes' || echo 'no'")

    if [ "$SLAM_MAP_EXISTS" = "yes" ]; then
        # Get absolute path to SLAM map on robot (without extension)
        SLAM_MAP_PATH=$(ssh $ROBOT_SSH "cd ~ && realpath $ROBOT_ROS_DIR/tests/$TEST_ID/slam_map 2>/dev/null || echo ''")
        # Remove extension if present
        SLAM_MAP_PATH="${SLAM_MAP_PATH%.posegraph}"

        if [ -n "$SLAM_MAP_PATH" ]; then
            SLAM_MAP_ARG="map_file:=$SLAM_MAP_PATH"
            echo "SLAM will load and continue from saved map: $SLAM_MAP_PATH"
        fi
    else
        echo "No saved SLAM map found, starting fresh mapping"
    fi
fi

# Wait for rosbot to fully initialize before starting SLAM
echo "Waiting for rosbot initialization to complete..."
sleep 10

# Terminal: SLAM on robot
gnome-terminal --tab --title="SLAM-Robot" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting SLAM on robot...\"
if [ -n \"$SLAM_MAP_ARG\" ]; then
    echo \"Loading saved SLAM map from test $TEST_ID\"
    ros2 launch aimapp slam_launch.py $SLAM_MAP_ARG use_sim_time:=false 2>&1
else
    echo \"Starting fresh SLAM mapping\"
    ros2 launch aimapp slam_launch.py use_sim_time:=false 2>&1
fi
bash
'"

sleep 5

# Determine which map to use for Nav2
MAP_ARG=""
if [ "$TEST_ID" != "None" ]; then
    # Get absolute path to map on robot
    MAP_PATH=$(ssh $ROBOT_SSH "cd ~ && realpath $ROBOT_ROS_DIR/tests/$TEST_ID/map.yaml 2>/dev/null || echo ''")

    if [ -n "$MAP_PATH" ]; then
        MAP_ARG="map:=$MAP_PATH"
        echo "Nav2 will use saved map from test $TEST_ID at: $MAP_PATH"
    else
        echo "No saved map found for test $TEST_ID, using default map"
    fi
fi

# Wait for SLAM to initialize before starting Nav2
echo "Waiting for SLAM to initialize..."
sleep 5

# Terminal 4: Nav2 husarion launch
gnome-terminal --tab --title="Nav2" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting Nav2 husarion...\"
if [ -n \"$MAP_ARG\" ]; then
    echo \"Using map: $MAP_ARG\"
    ros2 launch aimapp nav2_husarion_launch.py $MAP_ARG 2>&1 &
else
    ros2 launch aimapp nav2_husarion_launch.py 2>&1 &
fi
NAV2_PID=\$!
echo \"Nav2 launched with PID \$NAV2_PID\"
echo \"\"
echo \"Waiting for AMCL to be ready...\"
sleep 10
echo \"Setting AMCL initial pose to x=$ODOM_X, y=$ODOM_Y...\"
# Publish initial pose multiple times with current timestamp
for i in {1..5}; do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"{
      header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        },
        covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]
      }
    }\" &
    sleep 0.5
done
wait
echo \"AMCL initial pose set successfully!\"
echo \"Press Ctrl-C to stop this node\"
wait \$NAV2_PID
bash
'"




# ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
#   header: {
#     stamp: {sec: 0, nanosec: 0},
#     frame_id: 'map'
#   },
#   pose: {
#     pose: {
#       position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0},
#       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#     },
#     covariance: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
#                  0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
#                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]
#   }}"

  
sleep 2

# # Terminal 5: Minimal agent launch
gnome-terminal --tab --title="Agent" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting minimal agent with test_id=$TEST_ID...\"
echo \"Press Ctrl-C to stop this node\"
ros2 launch aimapp minimal_agent_launch.py test_id:=$TEST_ID 2>&1
bash
'"

sleep 2

# # Terminal 2: Node visualizer
gnome-terminal --tab --title="Visualizer" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting node visualizer...\"
echo \"Press Ctrl-C to stop this node\"
ros2 run aimapp node_visualizer.py 2>&1
bash
'"

sleep 2

# # Terminal 6: Save data
gnome-terminal --tab --title="SaveData" -- bash -c "
ssh -t -X $ROBOT_SSH 'bash -l -c \"
cd $ROBOT_ROS_DIR;
source install/setup.bash;
echo Starting save data node...;
echo Press Ctrl-C to stop this node;
ros2 run aimapp save_data.py 2>&1; 
exec bash
# \"'"


sleep 2


# # Terminal 7: Nav2 Client continuous mode
gnome-terminal --tab --title="Nav2-Client" -- bash -c "
ssh -t -X $ROBOT_SSH '
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"Starting nav2_client in continuous mode...\"
echo \"Press Ctrl-C to stop this node\"
ros2 run aimapp nav2_client.py --continuous 2>&1
bash
'"

# Terminal 7: AIF Process action (with placeholders to fill)
# gnome-terminal --tab --title="AIF-Action" -- bash -c "
# ssh -t -X $ROBOT_SSH 'bash -l -c \"
# cd $ROBOT_ROS_DIR;
# source install/setup.bash;
# echo Waiting to send AIF Process action...;
# echo Fill in the goal_reached and action values below:;
# echo   goal_reached: TOFILL;
# echo   action: TOFILL;
# echo;
# echo Example command:;
# echo ros2 action send_goal aif_process aimapp_actions/action/AIFProcess \\\"{goal_reached: true, action: 1}\\\";
# echo;
# echo Press Ctrl-C to cancel;
# read -p Press\ Enter\ to\ send\ action;
# ros2 action send_goal aif_process aimapp_actions/action/AIFProcess \\\"{goal_reached: TOFILL, action: TOFILL}\\\" 2>&1
# \"'
# exec bash"


# sleep 2

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

echo "All terminals launched. Check logs in: $LOG_DIR"
echo "Note: GUI runs locally, all other nodes run on robot at $ROBOT_SSH"
