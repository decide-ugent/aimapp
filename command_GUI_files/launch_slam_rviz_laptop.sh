#!/bin/bash

# Launch all ROS2 nodes for LAPTOP usage
# Created for aimapp project - Laptop side
#
# Usage: ./launch_slam_rviz_laptop.sh [test_id]
# Example: ./launch_slam_rviz_laptop.sh 5

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="10.10.131.145"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_ROS_DIR="ros2_ws"

# Get test_id from command line argument (optional)
TEST_ID="${1:-None}"

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)_laptop"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"
echo "Test ID: $TEST_ID"

# Fetch map from robot if test_id is provided
MAP_FILE=""
if [ "$TEST_ID" != "None" ]; then
    echo "Fetching saved map from robot for test $TEST_ID..."

    # Create local directory for maps
    LOCAL_MAP_DIR="$HOME/.ros/maps/test_${TEST_ID}"
    mkdir -p "$LOCAL_MAP_DIR"

    # Find the latest step directory on the robot
    # LATEST_STEP=$(ssh $ROBOT_SSH "cd $ROBOT_ROS_DIR/tests/$TEST_ID && ls -d step_* 2>/dev/null | sort -V | tail -n 1")

    # if [ -n "$LATEST_STEP" ]; then
    #     echo "Found latest step: $LATEST_STEP"
    REMOTE_MAP_DIR="$ROBOT_ROS_DIR/tests/$TEST_ID"

    # Copy map files from robot
    echo "Copying map files from robot..."
    scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.yaml" "$LOCAL_MAP_DIR/" 2>/dev/null
    scp "${ROBOT_SSH}:${REMOTE_MAP_DIR}/map.pgm" "$LOCAL_MAP_DIR/" 2>/dev/null

    if [ -f "$LOCAL_MAP_DIR/map.yaml" ] && [ -f "$LOCAL_MAP_DIR/map.pgm" ]; then
        # Update the image path in map.yaml to point to local file
        sed -i "s|image:.*|image: $LOCAL_MAP_DIR/map.pgm|g" "$LOCAL_MAP_DIR/map.yaml"
        MAP_FILE="$LOCAL_MAP_DIR/map.yaml"
        echo "SUCCESS: Map files downloaded to $LOCAL_MAP_DIR"
    else
        echo "WARNING: Could not find map files in $REMOTE_MAP_DIR"
        echo "SLAM will start in mapping mode instead of localization mode"
    fi
else
    echo "WARNING: No step directories found for test $TEST_ID"
    echo "SLAM will start in mapping mode instead of localization mode"
# fi
fi

# Terminal 1: SLAM Launch
if [ -n "$MAP_FILE" ]; then
    echo "Starting SLAM in localization mode with map: $MAP_FILE"
    gnome-terminal --tab --title="SLAM-Localization" -- bash -c "
    echo 'Starting SLAM launch in LOCALIZATION mode...'
    echo 'Using map from: $MAP_FILE'
    ros2 launch aimapp slam_launch.py map_file:=$MAP_FILE 2>&1 | tee $LOG_DIR/slam_launch.log
    exec bash"
else
    echo "Starting SLAM in mapping mode (no existing map)"
    gnome-terminal --tab --title="SLAM-Mapping" -- bash -c "
    echo 'Starting SLAM launch in MAPPING mode...'
    ros2 launch aimapp slam_launch.py 2>&1 | tee $LOG_DIR/slam_launch.log
    exec bash"
fi

sleep 2

# Terminal 2: RViz2 with manual motion config
gnome-terminal --tab --title="RViz2" -- bash -c "
echo 'Starting RViz2 with manual_motion configuration...'
rviz2 -d src/aimapp/aimapp/rviz/nav2_default_view.rviz 2>&1 | tee $LOG_DIR/rviz2.log
exec bash"

sleep 2

# Terminal 3: Odometry monitor (shifted)
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

echo "All laptop terminals launched. Check logs in: $LOG_DIR"
