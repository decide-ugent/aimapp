#!/bin/bash

# Launch all ROS2 nodes in separate terminals with logging on remote robot
# Created for aimapp project
#
# Usage: ./launch_model_as_action_robot.sh [test_id] [goal_ob_id] [goal_pose_id] [start_node_id] [influence_radius] [n_actions] [lookahead_node_creation] [skip_double_check]
# Example: ./launch_model_as_action_robot.sh 5 10 -1 0 1.6 17 8 false

# Apply CycloneDDS configuration for laptop-side ROS2 nodes
#export CYCLONEDDS_URI=file:///home/idlab332/workspace/ros_ws/cyclonedds_laptop.xml

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="192.168.1.2"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_ROS_DIR="ros2_ws"

# Get command line arguments (all optional with defaults)
TEST_ID="${1:-None}"
GOAL_OB_ID="${2:--1}"
GOAL_POSE_ID="${3:--1}"
START_NODE_ID="${4:--1}"
INFLUENCE_RADIUS="${5:-1.6}"
N_ACTIONS="${6:-17}"
LOOKAHEAD_NODE_CREATION="${7:-8}"
SKIP_DOUBLE_CHECK="${8:-false}"

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"
echo "Connecting to robot at: $ROBOT_SSH"
echo "Test ID: $TEST_ID"
echo "Goal Observation ID: $GOAL_OB_ID"
echo "Goal Pose ID: $GOAL_POSE_ID"
echo "Starting Node ID: $START_NODE_ID"
echo "Model Parameters: influence_radius=$INFLUENCE_RADIUS, n_actions=$N_ACTIONS, lookahead_node_creation=$LOOKAHEAD_NODE_CREATION"
echo "Skip Double Check: $SKIP_DOUBLE_CHECK"

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

# Sync time with robot
echo "Syncing time with robot..."
CURRENT_DATE=$(date +"%Y-%m-%d %H:%M:%S")
ssh "$ROBOT_SSH" "sudo date -s '$CURRENT_DATE'" >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "Time synced successfully: $CURRENT_DATE"
else
    echo "WARNING: Failed to sync time with robot (continuing anyway)"
fi

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
start_node_id = $START_NODE_ID
test_dir = os.path.join('tests', test_id)

# Print debug info to stderr so it doesn't interfere with coordinate output
print(f'DEBUG: Current directory: {os.getcwd()}', file=sys.stderr)
print(f'DEBUG: Looking for test directory: {test_dir}', file=sys.stderr)
print(f'DEBUG: start_node_id = {start_node_id}', file=sys.stderr)

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

    # If start_node_id is specified (>= 0), use pose from that node
    if start_node_id >= 0:
        if hasattr(model, 'PoseMemory') and hasattr(model.PoseMemory, 'id_to_pose'):
            pose = model.PoseMemory.id_to_pose(start_node_id)
            print(f'DEBUG: Using pose from node {start_node_id}: {pose}', file=sys.stderr)
            print(f'{pose[0]} {pose[1]}')
        else:
            print('ERROR: model.PoseMemory.id_to_pose not found', file=sys.stderr)
            sys.exit(1)
    else:
        # Use current_pose if no start_node_id specified
        if hasattr(model, 'current_pose'):
            pose = model.current_pose
            print(f'DEBUG: Using model.current_pose: {pose}', file=sys.stderr)
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

# Copy model from robot to laptop if TEST_ID is provided
if [ "$TEST_ID" != "None" ]; then
    echo "=========================================="
    echo "Copying model from robot to laptop..."
    echo "=========================================="

    # Create local tests directory if it doesn't exist
    LOCAL_TEST_DIR="tests/$TEST_ID"
    mkdir -p "$LOCAL_TEST_DIR"

    # Copy model.pkl from robot
    echo "Fetching model.pkl from robot..."
    scp "${ROBOT_SSH}:${ROBOT_ROS_DIR}/tests/${TEST_ID}/model.pkl" "$LOCAL_TEST_DIR/" 2>/dev/null

    if [ -f "$LOCAL_TEST_DIR/model.pkl" ]; then
        echo "SUCCESS: Model copied to $LOCAL_TEST_DIR/model.pkl"
    else
        echo "WARNING: Could not copy model.pkl from robot"
        echo "Agent may fail to load model"
    fi

    # Also copy observations directory if it exists
    echo "Fetching observations from robot..."
    scp -r "${ROBOT_SSH}:${ROBOT_ROS_DIR}/tests/${TEST_ID}/observations" "$LOCAL_TEST_DIR/" 2>/dev/null

    if [ -d "$LOCAL_TEST_DIR/observations" ]; then
        echo "SUCCESS: Observations copied to $LOCAL_TEST_DIR/observations"
    else
        echo "INFO: No observations directory found (may not exist yet)"
    fi

    echo "=========================================="
    echo ""
fi

# # # # # Terminal 1: Joy2Twist gamepad controller
# gnome-terminal --tab --title="Joy2Twist" -- bash -c "
# ssh -t -X $ROBOT_SSH '
# cd $ROBOT_ROS_DIR
# source install/setup.bash
# echo \"Starting Joy2Twist gamepad controller...\"
# echo \"Press Ctrl-C to stop this node\"
# ros2 launch joy2twist gamepad_controller.launch.py joy2twist_params_file:=/home/husarion/joy2twist.yaml 2>&1
# bash
# '"


# # sleep 2
# # ----------------------------------------------------- INIT ROBOT -------------------------------------------------

# # Terminal 3: Start rosbot and initialize EKF pose
gnome-terminal --tab --title="Rosbot-Init" -- bash -c "
ssh -t -X $ROBOT_SSH '
echo \"Starting rosbot!\"
bash start_rosbot.sh 2>&1
cd $ROBOT_ROS_DIR
source install/setup.bash
echo \"\"
echo \"To attach to rosbot: tmux attach -t rosbot_startup\"
echo \"To detach: Press Ctrl-b, then d\"
echo \"\"
echo \"Waiting for EKF node to be ready...\"
sleep 5

if [ \"$TEST_ID\" != \"None\" ]; then
    echo \"Setting initial EKF pose to x=$ODOM_X, y=$ODOM_Y using /set_pose topic...\"

    # Publish to /set_pose topic multiple times to ensure EKF receives it
    for i in 1 2 3; do
        ros2 topic pub --once /set_pose geometry_msgs/msg/PoseWithCovarianceStamped \"{
          header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'odom'
          },
          pose: {
            pose: {
              position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0},
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            },
            covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
          }
        }\"
        sleep 1
    done

    echo \"EKF initial pose set successfully!\"
else
    echo \"No test_id - using default pose (0, 0)\"
fi

echo \"\"
ros2 run aimapp shift_husarion_odom.py 0.0 0.0 2>&1 | tee $LOG_DIR/shift_husarion_odom.log 
echo \"Press Ctrl-C to exit\"
bash
'
"

# # # This shift is useful when we do not restart the robot but just the model (thus initial position is not 0.0 0.0)


# # # echo \"\"
# # # echo \"Waiting for EKF node to be ready...\"
# # # sleep 3
# # # echo \"Setting initial EKF pose to x=$ODOM_X, y=$ODOM_Y using set_pose service...\"
# # # ros2 topic pub --once /set_pose geometry_msgs/msg/PoseWithCovarianceStamped \"{
# # #   header: {
# # #     stamp: {sec: 0, nanosec: 0},
# # #     frame_id: 'odom'
# # #   },
# # #   pose: {
# # #     pose: {
# # #       position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0},
# # #       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
# # #     },
# # #     covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
# # #                  0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
# # #                  0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
# # #                  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
# # #                  0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
# # #                  0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
# # #   }
# # # }\"
# # # echo \"EKF initial pose set successfully!\"
# # # echo \"\"
# # # echo \"Shifting husarion odom with x=$NEG_ODOM_X, y=$NEG_ODOM_Y...\"
# # # echo \"Press Ctrl-C to stop this node\"


# Determine which Nav2 and SLAM maps to load
MAP_FILE=""
SLAM_MAP_ARG=""

if [ "$TEST_ID" != "None" ]; then
    echo "=========================================="
    echo "Fetching maps from robot for test $TEST_ID..."
    echo "=========================================="

    # Create local directory for maps
    LOCAL_MAP_DIR="$PWD/latest_slam_map/test_${TEST_ID}"
    mkdir -p "$LOCAL_MAP_DIR"

    REMOTE_MAP_DIR="$ROBOT_ROS_DIR/tests/$TEST_ID"

    # ===== FETCH NAV2 MAP FILES (map.yaml and map.pgm) =====
    echo ""
    echo "--- Fetching Nav2 map files (.yaml and .pgm) ---"
    scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.yaml" "$LOCAL_MAP_DIR/" 2>/dev/null
    scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.pgm" "$LOCAL_MAP_DIR/" 2>/dev/null

    if [ -f "$LOCAL_MAP_DIR/map.yaml" ] && [ -f "$LOCAL_MAP_DIR/map.pgm" ]; then
        # Update the image path in map.yaml to point to local file
        sed -i "s|image:.*|image: $LOCAL_MAP_DIR/map.pgm|g" "$LOCAL_MAP_DIR/map.yaml"
        MAP_FILE="$LOCAL_MAP_DIR/map.yaml"
        echo "SUCCESS: Nav2 map files downloaded to $LOCAL_MAP_DIR"
        echo "  - map.yaml: $LOCAL_MAP_DIR/map.yaml"
        echo "  - map.pgm: $LOCAL_MAP_DIR/map.pgm"
    else
        echo "WARNING: Could not find Nav2 map files (map.yaml/map.pgm) in robot:$REMOTE_MAP_DIR"
        echo "Nav2 will start without a map"
    fi

    # ===== FETCH SLAM MAP FILES (slam_map.data and slam_map.posegraph) =====
    echo ""
    echo "--- Checking for SLAM map files (.data and .posegraph) ---"

    # Check if SLAM map already exists in the test-specific directory
    if [ -f "$LOCAL_MAP_DIR/slam_map.data" ] && [ -f "$LOCAL_MAP_DIR/slam_map.posegraph" ]; then
        # Use existing SLAM map from test directory (don't overwrite)
        SLAM_MAP_ARG="$LOCAL_MAP_DIR/slam_map"
        echo "FOUND: Using existing SLAM map in test directory:"
        echo "  - slam_map.data: $LOCAL_MAP_DIR/slam_map.data"
        echo "  - slam_map.posegraph: $LOCAL_MAP_DIR/slam_map.posegraph"
        echo "SLAM will load and continue from existing test map: $LOCAL_MAP_DIR/slam_map"
    else
        # No SLAM map in test directory, check latest_slam_map
        LAPTOP_SLAM_DIR="latest_slam_map"

        if [ -f "$LAPTOP_SLAM_DIR/slam_map.posegraph" ] && [ -f "$LAPTOP_SLAM_DIR/slam_map.data" ]; then
            # Copy SLAM map files to the test-specific directory
            cp "$LAPTOP_SLAM_DIR/slam_map.data" "$LOCAL_MAP_DIR/" 2>/dev/null
            cp "$LAPTOP_SLAM_DIR/slam_map.posegraph" "$LOCAL_MAP_DIR/" 2>/dev/null

            if [ -f "$LOCAL_MAP_DIR/slam_map.data" ] && [ -f "$LOCAL_MAP_DIR/slam_map.posegraph" ]; then
                # Use local path (without extension, SLAM toolbox will add it)
                SLAM_MAP_ARG="$LOCAL_MAP_DIR/slam_map"
                echo "SUCCESS: SLAM map files copied from $LAPTOP_SLAM_DIR to $LOCAL_MAP_DIR"
                echo "  - slam_map.data: $LOCAL_MAP_DIR/slam_map.data"
                echo "  - slam_map.posegraph: $LOCAL_MAP_DIR/slam_map.posegraph"
                echo "SLAM will load and continue from copied map: $LOCAL_MAP_DIR/slam_map"
            else
                echo "WARNING: Failed to copy SLAM map files, starting fresh mapping"
            fi
        else
            echo "No saved SLAM map found in $LAPTOP_SLAM_DIR, starting fresh SLAM mapping"
        fi
    fi

    echo "=========================================="
    echo ""
else
    echo "No test_id provided, starting with fresh maps"
fi

# Wait for rosbot to fully initialize before starting SLAM
echo "Waiting for rosbot initialization to complete..."
sleep 5

# Determine which map to use for Nav2
MAP_ARG=""
if [ -n "$MAP_FILE" ]; then
    MAP_ARG="$MAP_FILE"
    echo "Nav2 will use local saved map from test $TEST_ID at: $MAP_FILE"
else
    echo "No saved map available, Nav2 will start without a map"
fi

# # ----------------------------------------------------- INIT NAV2 -------------------------------------------------

#ssh -t -X $ROBOT_SSH '
#cd $ROBOT_ROS_DIR
# Terminal 4: Nav2 husarion launch (LAPTOP)
gnome-terminal --tab --title="Nav2" -- bash -c "
source install/setup.bash
echo \"Starting Nav2 husarion...\"
if [ -n \"$MAP_ARG\" ]; then
    echo \"Using map: $MAP_ARG\"
    ros2 launch aimapp nav2_husarion_launch.py map:=$MAP_ARG 2>&1 | tee $LOG_DIR/nav2_husarion_launch.log &
else
    ros2 launch aimapp nav2_husarion_launch.py 2>&1 | tee $LOG_DIR/nav2_husarion_launch.log &
fi
NAV2_PID=\$!
echo \"Nav2 launched with PID \$NAV2_PID\"
echo \"\"
echo \"Waiting for Nav2 and AMCL to be ready...\"
echo \"Checking if /initialpose topic is available...\"

# Wait for the /initialpose topic to be available (indicates AMCL is ready)
TIMEOUT=30
ELAPSED=0
while [ \$ELAPSED -lt \$TIMEOUT ]; do
    if ros2 topic list | grep -q '/initialpose'; then
        echo \"AMCL is ready! /initialpose topic found.\"
        break
    fi
    echo \"Waiting for AMCL... (\$ELAPSED/\$TIMEOUT seconds)\"
    sleep 2
    ELAPSED=\$((ELAPSED + 2))
done

if [ \$ELAPSED -ge \$TIMEOUT ]; then
    echo \"WARNING: Timeout waiting for AMCL to start. Proceeding anyway...\"
fi

sleep 2
echo \"\"
echo \"Setting AMCL initial pose to x=$ODOM_X, y=$ODOM_Y...\"
echo \"Publishing initial pose 7 times to ensure Nav2 receives it...\"

# Publish initial pose multiple times sequentially (not in background)
for i in {1..7}; do
    echo \"  Publish attempt \$i/7\"
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
    }\"
    sleep 1
done

echo \"AMCL initial pose published successfully!\"
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

  
echo "Waiting for AMCL to fully stabilize map->odom transform..."
sleep 10

# # ----------------------------------------------------- SLAM -------------------------------------------------

# Terminal: SLAM on laptop
# Note: Variables must be properly expanded before being passed to bash -c
if [ -n "$SLAM_MAP_ARG" ]; then
    echo "Launching SLAM with saved map deserialization: $SLAM_MAP_ARG"
    gnome-terminal --tab --title="SLAM-Laptop" -- bash -c "
    source install/setup.bash
    echo '=========================================='
    echo 'Starting SLAM on laptop with map reload...'
    echo 'Map file: $SLAM_MAP_ARG'
    echo '=========================================='
    echo ''
    echo 'Step 1: Launching SLAM without map (will deserialize after startup)...'
    ros2 launch aimapp slam_launch.py use_sim_time:=false 2>&1 &
    SLAM_PID=\$!

    echo 'Waiting for SLAM services to become available...'
    TIMEOUT=60
    ELAPSED=0
    while [ \$ELAPSED -lt \$TIMEOUT ]; do
        if ros2 service list | grep -q '/slam_toolbox/pause_new_measurements'; then
            echo \"SLAM services ready!\"
            break
        fi
        echo \"Waiting for SLAM... (\$ELAPSED/\$TIMEOUT seconds)\"
        sleep 2
        ELAPSED=\$((ELAPSED + 2))
    done

    if [ \$ELAPSED -ge \$TIMEOUT ]; then
        echo \"ERROR: SLAM services not available after \$TIMEOUT seconds\"
        echo \"Continuing without deserialization...\"
        wait \$SLAM_PID
        bash
        exit 1
    fi

    echo 'Waiting additional time for SLAM to fully initialize sensor...'
    sleep 7

    echo ''
    echo 'Step 2: Pausing new measurements before deserialization...'
    timeout 12 ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/Empty
    if [ \$? -ne 0 ]; then
        echo \"WARNING: Failed to pause measurements (timeout or error)\"
        echo \"Attempting to continue anyway...\"
    fi
    sleep 1

    echo ''
    echo 'Step 3: Deserializing saved posegraph...'
    echo '  (Note: AMCL has already set map->odom transform)'
    timeout 15 ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \"{filename: '$SLAM_MAP_ARG', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}\"
    if [ \$? -ne 0 ]; then
        echo \"WARNING: Deserialization failed or timed out\"
    else
        echo \"Deserialization completed!\"
    fi
    sleep 5

    echo ''
    echo 'Step 4: Resuming measurements to continue mapping...'
    timeout 10 ros2 service call /slam_toolbox/resume_new_measurements std_srvs/srv/Empty
    if [ \$? -ne 0 ]; then
        echo \"WARNING: Failed to resume measurements\"
    fi

    echo ''
    echo '=========================================='
    echo 'SLAM ready - map reloaded and continuing!'
    echo '=========================================='
    wait \$SLAM_PID
    bash
    "
else
    echo "Launching SLAM with fresh mapping"
    gnome-terminal --tab --title="SLAM-Laptop" -- bash -c "
    source install/setup.bash
    echo '=========================================='
    echo 'Starting SLAM with fresh mapping...'
    echo '=========================================='
    ros2 launch aimapp slam_launch.py use_sim_time:=false 2>&1
    bash
    "
fi

# # ----------------------------------------------------- NODE SAVE SLAM MAP -------------------------------------------------

sleep 5

gnome-terminal --tab --title="SLAM-MAP-SAVER" -- bash -c "
source install/setup.bash
echo \"Starting SLAM MAP saver...\"
echo \"This will save SLAM map when /shifted_odom is received\"
ros2 run aimapp save_slam_map.py 2>&1
bash
"
# # ----------------------------------------------------- INIT AGENT -------------------------------------------------


# Wait for SLAM to initialize before starting Nav2
# echo "Waiting for SLAM to initialize..."
sleep 3

# Terminal 5: Minimal agent launch (LAPTOP)
gnome-terminal --tab --title="Agent-Laptop" -- bash -c "
source install/setup.bash
echo \"Starting minimal agent on LAPTOP with test_id=$TEST_ID...\"
echo \"Goal Parameters: goal_ob_id=$GOAL_OB_ID, goal_pose_id=$GOAL_POSE_ID, start_node_id=$START_NODE_ID\"
echo \"Model Parameters: influence_radius=$INFLUENCE_RADIUS, n_actions=$N_ACTIONS, lookahead=$LOOKAHEAD_NODE_CREATION\"
echo \"Skip Double Check: $SKIP_DOUBLE_CHECK\"
echo \"Press Ctrl-C to stop this node\"
ros2 launch aimapp minimal_agent_launch.py test_id:=$TEST_ID goal_ob_id:=$GOAL_OB_ID goal_pose_id:=$GOAL_POSE_ID start_node_id:=$START_NODE_ID influence_radius:=$INFLUENCE_RADIUS n_actions:=$N_ACTIONS lookahead_node_creation:=$LOOKAHEAD_NODE_CREATION skip_double_check:=$SKIP_DOUBLE_CHECK 2>&1 | tee $LOG_DIR/minimal_agent_launch.log
bash
"

sleep 2

# # ----------------------------------------------------- INIT VISU -------------------------------------------------

# # Terminal 2: Node visualizer (LAPTOP)
gnome-terminal --tab --title="Visualizer-Laptop" -- bash -c "
source install/setup.bash
echo \"Starting node visualizer on LAPTOP...\"
echo \"Press Ctrl-C to stop this node\"
ros2 run aimapp node_visualizer.py 2>&1 | tee $LOG_DIR/node_visualisation.log
bash
"

sleep 2

# # ----------------------------------------------------- INIT SAVE DATA -------------------------------------------------


# # Terminal 6: Save data
# gnome-terminal --tab --title="SaveData" -- bash -c "
# ssh -t -X $ROBOT_SSH 'bash -l -c \"
# cd $ROBOT_ROS_DIR;
# source install/setup.bash;
# echo Starting save data node...;
# echo Press Ctrl-C to stop this node;
# ros2 run aimapp save_data.py 2>&1; 
# exec bash
# '"


sleep 2
# # ----------------------------------------------------- INIT NAV2 CLIENT -------------------------------------------------

#ssh -t -X $ROBOT_SSH '
# cd $ROBOT_ROS_DIR

# Terminal 7: Nav2 Client continuous mode (LAPTOP)
gnome-terminal --tab --title="Nav2-Client" -- bash -c "
source install/setup.bash
echo \"Starting nav2_client in continuous mode...\"
echo \"Press Ctrl-C to stop this node\"
ros2 run aimapp nav2_client.py --continuous 2>&1 | tee $LOG_DIR/nav2_client.log 
bash
'"

sleep 2
# # ----------------------------------------------------- INIT ACTION GUI -------------------------------------------------

Terminal 8: Action Selector GUI - runs locally, not on robot
gnome-terminal --tab --title="Action-Selector-GUI" -- bash -c "
echo 'Starting Action Selector GUI (local)...'
echo 'This GUI will display possible actions from AIF Process'
echo 'and allow you to select which pose to navigate to next.'
echo ''
echo 'Make sure ROS_DOMAIN_ID matches the robot!'
echo ''
python3 src/aimapp/command_GUI_files/action_selector_with_subscriber.py 2>&1
exec bash"


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


echo "All terminals launched. Check logs in: $LOG_DIR"
echo "Note: GUI runs locally, all other nodes run on robot at $ROBOT_SSH"


#  ros2 topic pub --once /nav2_client_goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 4.33, y: -0.78}, orientation: {w: 1.0}}}"