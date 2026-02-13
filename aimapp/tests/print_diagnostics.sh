#!/bin/bash
#
# Print all /diagnostics messages from a ROS2 bag
#
# Usage: ./print_diagnostics.sh <bag_directory>
#
# Example:
#   ./print_diagnostics.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534/run_20251218_144534
#

BAG_DIR="${1:-}"

if [ -z "$BAG_DIR" ]; then
    echo "ERROR: bag directory is required"
    echo ""
    echo "Usage: $0 <bag_directory>"
    echo ""
    echo "Example:"
    echo "  $0 ~/experiment_recordings/test_0/run_20251218_144534"
    exit 1
fi

if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: Bag directory not found: $BAG_DIR"
    exit 1
fi

# Check ROS 2 environment
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    echo "Please source your ROS 2 workspace:"
    echo "  source install/setup.bash"
    exit 1
fi

echo "=========================================="
echo "Printing /diagnostics from bag"
echo "=========================================="
echo "Bag: $BAG_DIR"
echo "=========================================="
echo ""

# Play bag and echo diagnostics topic
ros2 bag play "$BAG_DIR" --topics /diagnostics -r 20 &
PLAY_PID=$!

sleep 1

ros2 topic echo /diagnostics

# Kill bag player when done
kill $PLAY_PID 2>/dev/null
wait $PLAY_PID 2>/dev/null

echo ""
echo "=========================================="
echo "Done"
echo "=========================================="
