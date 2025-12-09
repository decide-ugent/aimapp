#!/bin/bash

# Launch all ROS2 nodes for LAPTOP usage
# Created for aimapp project - Laptop side
#
# Usage: ./launch_slam_rviz_laptop.sh [test_id]
# Example: ./launch_slam_rviz_laptop.sh 5

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="192.168.1.2"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_ROS_DIR="ros2_ws"

# Get test_id from command line argument (optional)
TEST_ID="${1:-None}"

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)_laptop"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"
echo "Test ID: $TEST_ID"

# # Fetch map from robot if test_id is provided
# MAP_FILE=""
# if [ "$TEST_ID" != "None" ]; then
#     echo "Fetching saved map from robot for test $TEST_ID..."

#     # Create local directory for maps
#     LOCAL_MAP_DIR="$HOME/.ros/maps/test_${TEST_ID}"
#     mkdir -p "$LOCAL_MAP_DIR"

#     # Find the latest step directory on the robot
#     # LATEST_STEP=$(ssh $ROBOT_SSH "cd $ROBOT_ROS_DIR/tests/$TEST_ID && ls -d step_* 2>/dev/null | sort -V | tail -n 1")

#     # if [ -n "$LATEST_STEP" ]; then
#     #     echo "Found latest step: $LATEST_STEP"
#     REMOTE_MAP_DIR="$ROBOT_ROS_DIR/tests/$TEST_ID"

#     # Copy map files from robot
#     echo "Copying map files from robot..."
#     scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.yaml" "$LOCAL_MAP_DIR/" 2>/dev/null
#     scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.pgm" "$LOCAL_MAP_DIR/" 2>/dev/null

#     if [ -f "$LOCAL_MAP_DIR/map.yaml" ] && [ -f "$LOCAL_MAP_DIR/map.pgm" ]; then
#         # Update the image path in map.yaml to point to local file
#         sed -i "s|image:.*|image: $LOCAL_MAP_DIR/map.pgm|g" "$LOCAL_MAP_DIR/map.yaml"
#         MAP_FILE="$LOCAL_MAP_DIR/map.yaml"
#         echo "SUCCESS: Map files downloaded to $LOCAL_MAP_DIR"
#     else

#         echo "WARNING: Could not find map files in $REMOTE_MAP_DIR"
#         echo "SLAM will start in mapping mode instead of localization mode"
#     fi
# else
#     echo "WARNING: No step directories found for test $TEST_ID"
#     echo "SLAM will start in mapping mode instead of localization mode"
# # fi
# fi

# # Terminal 1: SLAM Launch
# # Determine if we should load an existing SLAM map
# SLAM_MAP_ARG=""
# if [ "$TEST_ID" != "None" ]; then
#     echo "Checking for saved SLAM map from test $TEST_ID..."
#     # Check if SLAM map exists on robot (note: SLAM toolbox saves with .posegraph extension)
#     SLAM_MAP_EXISTS=$(ssh $ROBOT_SSH "test -f $ROBOT_ROS_DIR/tests/$TEST_ID/slam_map.posegraph && echo 'yes' || echo 'no'")

#     if [ "$SLAM_MAP_EXISTS" = "yes" ]; then
#         # Get absolute path to SLAM map on robot (without extension, SLAM toolbox will add it)
#         SLAM_MAP_PATH=$(ssh $ROBOT_SSH "cd ~ && realpath $ROBOT_ROS_DIR/tests/$TEST_ID/slam_map 2>/dev/null || echo ''")
#         # Remove the extension if it's there
#         SLAM_MAP_PATH="${SLAM_MAP_PATH%.posegraph}"

#         if [ -n "$SLAM_MAP_PATH" ]; then
#             SLAM_MAP_ARG="map_file:=$SLAM_MAP_PATH"
#             echo "SLAM will load and continue from saved map: $SLAM_MAP_PATH"
#         else
#             echo "Could not get SLAM map path, starting fresh mapping"
#         fi
#     else
#         echo "No saved SLAM map found, starting fresh mapping"
#     fi
# fi

# gnome-terminal --tab --title="SLAM-Mapping" -- bash -c "
# echo 'Starting SLAM launch...'
# if [ \"$TEST_ID\" != \"None\" ] && [ -n \"$SLAM_MAP_ARG\" ]; then
#     echo 'Loading saved SLAM map from test $TEST_ID'
#     echo 'SLAM will continue building upon the existing map'
#     ros2 launch aimapp slam_launch.py $SLAM_MAP_ARG2>&1 | tee $LOG_DIR/slam_launch.log
# else
#     echo 'Starting fresh SLAM mapping'
#     ros2 launch aimapp slam_launch.py 2>&1 | tee $LOG_DIR/slam_launch.log
# fi
# exec bash"

# use_sim_time:=false 
sleep 2

# Terminal 2: RViz2 with manual motion config
gnome-terminal --tab --title="RViz2" -- bash -c "
echo 'Starting RViz2 with navigation view...'
echo ''
if [ \"$TEST_ID\" != \"None\" ]; then
    echo 'Map Visualization:'
    echo '  - /map: Live SLAM map from robot (continuous updates)'
    echo '  - Robot uses saved map from test $TEST_ID for navigation'
fi
echo ''
rviz2 -d src/aimapp/aimapp/rviz/nav2_default_view.rviz 2>&1 | tee $LOG_DIR/rviz2.log
exec bash"

sleep 2

#Terminal 3: Odometry monitor (shifted)
gnome-terminal --tab --title="Odom-Monitor" -- bash -c "
echo '=========================================='
echo 'Monitoring /odometry/shifted'
echo 'Checking robot position updates...'
echo '=========================================='
echo ''
echo 'This will show position updates continuously.'
echo 'If updates stop, you may have lost contact with the robot.'
echo ''
ros2 topic echo /odometry/shifted | grep -A 3 'position' 2>&1
exec bash"

echo ""
echo "=========================================="
echo "All laptop terminals launched"
echo "=========================================="
echo "Logs directory: $LOG_DIR"
if [ "$TEST_ID" != "None" ]; then
    echo ""
    echo "Continuing from Test ID: $TEST_ID"
    echo "  - Nav2 (on robot): Using saved map for navigation/localization"
    echo "  - SLAM (laptop): Continuously updating the map"
    echo "  - RViz (laptop): Displays live SLAM map"
fi
echo "=========================================="
