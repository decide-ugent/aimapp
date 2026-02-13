#!/bin/bash

# Record a complete run of the robot system
# Captures ROS topics for experiment documentation
# (Agent logs are already saved by the system)
#
# Usage: ./record_run.sh [test_id] [run_name]
# Example: ./record_run.sh 5 "exploration_run_1"

TEST_ID="${1:-}"
RUN_NAME="${2:-}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Validate inputs
if [ -z "$TEST_ID" ]; then
    echo "ERROR: test_id is required"
    echo "Usage: $0 [test_id] [run_name]"
    echo "Example: $0 5 exploration_run_1"
    exit 1
fi

# Create output directory structure
if [ -n "$RUN_NAME" ]; then
    BASE_DIR="$HOME/experiment_recordings/test_${TEST_ID}/${RUN_NAME}_${TIMESTAMP}"
else
    BASE_DIR="$HOME/experiment_recordings/test_${TEST_ID}/run_${TIMESTAMP}"
fi

ROSBAG_DIR="${BASE_DIR}"

mkdir -p "$ROSBAG_DIR"

echo "=========================================="
echo "Recording Robot Run"
echo "=========================================="
echo "Test ID: $TEST_ID"
if [ -n "$RUN_NAME" ]; then
    echo "Run name: $RUN_NAME"
fi
echo "Timestamp: $TIMESTAMP"
echo "Output directory: $BASE_DIR"
echo ""
echo "Recording ROS topics (Agent logs saved separately by system)"
echo ""
echo "Press Ctrl-C to stop recording"
echo "=========================================="
echo ""

# Check ROS 2
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    echo "Please source your ROS 2 workspace:"
    echo "  source install/setup.bash"
    exit 1
fi
echo "✓ ROS 2 environment ready"
echo ""

# Create metadata file
METADATA_FILE="${BASE_DIR}/run_metadata.txt"
{
    echo "=========================================="
    echo "Run Metadata"
    echo "=========================================="
    echo "Test ID: $TEST_ID"
    echo "Run name: $RUN_NAME"
    echo "Timestamp: $TIMESTAMP"
    echo "Start time: $(date)"
    echo "Host: $(hostname)"
    echo "User: $(whoami)"
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
    echo ""
    echo "Note: Agent logs are saved separately by the system"
    echo "=========================================="
} > "$METADATA_FILE"

echo "Metadata saved to: $METADATA_FILE"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "=========================================="
    echo "Stopping recording..."
    echo "=========================================="

    # Update metadata with end time
    {
        echo ""
        echo "End time: $(date)"
        echo "=========================================="
    } >> "$METADATA_FILE"

    echo ""
    echo "Recording complete!"
    echo "Data saved to: $BASE_DIR"
    echo ""
    echo "Files:"
    ls -lh "$BASE_DIR"
    echo ""
    echo "To replay:"
    echo "  ros2 bag play $BASE_DIR"
    echo ""
    echo "=========================================="

    exit 0
}

trap cleanup INT TERM

# Start rosbag recording (foreground - main process)
echo "Starting ROS topic recording..."
echo "Recording to: $BASE_DIR"
echo ""
echo "Topics:"
echo "  - Odometry: /odometry/filtered, /odometry/shifted, /shifted_odom"
echo "  - Transforms: /tf, /tf_static"
echo "  - Navigation: /cmd_vel, /initial_pose, /nav2_client_goal_pose, /nav2_navigation_result"
echo "  - Sensors: /scan, /scan_filtered, /joy, /diagnostics"
echo "  - Mapping: /map, costmaps (global/local)"
echo "  - AIF Visualization: /waypoints, /visitable_nodes, /goal_pose_marker, /mcts_reward_visualization"
echo ""
echo "Press Ctrl-C to stop"
echo "=========================================="
echo ""

cd "$BASE_DIR"

ros2 bag record \
    /odometry/filtered \
    /odometry/shifted \
    /shifted_odom \
    /imu_broadcaster/imu \
    /odometry/wheels \
    /tf \
    /tf_static \
    /cmd_vel \
    /diagnostics \
    /joy \
    /map \
    /scan \
    /scan_filtered \
    /global_costmap/costmap \
    /global_costmap/costmap_updates \
    /local_costmap/costmap \
    /local_costmap/costmap_updates \
    /waypoints \
    /visitable_nodes \
    /goal_pose_marker \
    /mcts_reward_visualization \
    /nav2_navigation_result \
    /initial_pose \
    /nav2_client_goal_pose \
    -o run_${TIMESTAMP}

# Cleanup will be called by trap when rosbag stops
cleanup
