#!/bin/bash
#
# Play ROS2 bag at custom rate with synchronized agent logs in separate terminals
# Launches two independent processes for complete isolation
#
# Usage: ./play_bag_with_logs.sh <experiment_dir> [rate]
#
# Examples:
#   ./play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534
#   ./play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 1.0  # normal speed
#   ./play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 20.0  # 20x speed
#

EXPERIMENT_DIR="${1:-}"
RATE="${2:-3.0}"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -z "$EXPERIMENT_DIR" ]; then
    echo "ERROR: experiment_dir is required"
    echo ""
    echo "Usage: $0 <experiment_dir> [rate]"
    echo ""
    echo "Examples:"
    echo "  $0 ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534"
    echo "  $0 ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 1.0  # normal speed"
    echo "  $0 ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 20.0  # 20x speed"
    echo ""
    exit 1
fi

# Check if experiment directory exists
if [ ! -d "$EXPERIMENT_DIR" ]; then
    echo "ERROR: Experiment directory not found: $EXPERIMENT_DIR"
    exit 1
fi

# Check ROS 2 environment
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    echo "Please source your ROS 2 workspace:"
    echo "  source install/setup.bash"
    exit 1
fi

# Find log file
LOG_FILE=$(find "$EXPERIMENT_DIR" -name "minimal_agent_launch.log" | head -1)
if [ -z "$LOG_FILE" ]; then
    echo "ERROR: Could not find minimal_agent_launch.log in $EXPERIMENT_DIR"
    exit 1
fi

# Find bag directory
BAG_DIR=$(find "$EXPERIMENT_DIR" -name "metadata.yaml" -exec dirname {} \; | grep "run_" | head -1)
if [ -z "$BAG_DIR" ]; then
    echo "ERROR: Could not find ROS2 bag in $EXPERIMENT_DIR"
    exit 1
fi

# Find steps_data.csv
STEPS_CSV=$(find "$EXPERIMENT_DIR" -name "steps_data.csv" | head -1)
if [ -z "$STEPS_CSV" ]; then
    echo "ERROR: Could not find steps_data.csv in $EXPERIMENT_DIR"
    exit 1
fi

echo "=========================================="
echo "ROS2 Bag Playback with Synchronized Logs"
echo "=========================================="
echo "Experiment: $EXPERIMENT_DIR"
echo "Log file:   $LOG_FILE"
echo "Bag dir:    $BAG_DIR"
echo "Steps CSV:  $STEPS_CSV"
echo "Rate:       ${RATE}x speed"
echo ""
echo "Launching separate terminals and background processes..."
echo "  1. Agent logs (terminal)"
echo "  2. ROS2 bag (terminal)"
echo "  3. Odometry follower (background) - syncs robot with bag odometry"
#echo "  4. Steps replay from CSV (background)"
echo "=========================================="
echo ""

# Launch odometry follower in background
# This script reads /odometry/filtered/bag and /odom, publishes cmd_vel to sync robot with bag
# After 7 MCTS messages, will reset odometry to bag position
#python3 "$SCRIPT_DIR/move_robot_bag_odom.py" > /dev/null 2>&1 &
#ODOM_FOLLOWER_PID=$!

# gnome-terminal -- bash -c "echo 'ODOMETRY FOLLOWER LOGS'; python3 '$SCRIPT_DIR/move_robot_bag_odom.py'; echo ''; echo 'Press Enter to close...'; read" & ODOM_FOLLOWER_PID=$!
# echo "Started odometry follower (PID: $ODOM_FOLLOWER_PID)"
# sleep 0.5



# Launch steps replay in background (uses PotentialField to reach goals from CSV)
# Pass use_sim_time parameter for bag playback synchronization
#python3 "$SCRIPT_DIR/replay_steps_from_csv.py" --csv "$STEPS_CSV" --action potential_field --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
#REPLAY_PID=$!
#echo "Started steps replay from CSV (PID: $REPLAY_PID)"
#sleep 0.5

# Launch logs in separate terminal
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "echo 'AGENT LOGS TERMINAL'; python3 '$SCRIPT_DIR/print_logs_only.py' '$LOG_FILE' --rate $RATE; echo ''; echo 'Press Enter to close...'; read" &
    LOG_PID=$!

    echo ""
    echo "Launched terminal:"
    echo "  - Agent logs (PID: $LOG_PID)"
    echo ""
    echo "Background processes:"
    echo "  - Odometry follower (PID: $ODOM_FOLLOWER_PID)"
    echo ""
    echo "Starting ROS2 bag playback in this terminal..."
    echo "Press Ctrl+C to stop playback"
    echo "=========================================="
    echo ""

    # Small delay before starting bag
    sleep 1

    # Run bag playback in foreground (this terminal)
    ros2 bag play "$BAG_DIR" --rate $RATE --clock --remap /cmd_vel:=/cmd_vel_original 

elif command -v xterm &> /dev/null; then
    xterm -title "Agent Logs" -e "python3 '$SCRIPT_DIR/print_logs_only.py' '$LOG_FILE' --rate $RATE; echo ''; echo 'Press Enter to close...'; read" &
    LOG_PID=$!

    echo ""
    echo "Launched xterm window:"
    echo "  - Agent logs (PID: $LOG_PID)"
    echo ""
    echo "Background processes:"
    echo "  - Odometry follower (PID: $ODOM_FOLLOWER_PID)"
    echo ""
    echo "Starting ROS2 bag playback in this terminal..."
    echo "Press Ctrl+C to stop playback"
    echo "=========================================="
    echo ""

    # Small delay before starting bag
    sleep 1

    # Run bag playback in foreground (this terminal)
    ros2 bag play "$BAG_DIR" --rate $RATE --clock --remap /cmd_vel:=/cmd_vel_original 
else
    echo "ERROR: No terminal emulator found (tried gnome-terminal, xterm)"
    echo ""
    echo "You can run manually:"
    echo ""
    echo "  Background processes (run these first):"
    echo "    python3 $SCRIPT_DIR/move_robot_bag_odom.py > /dev/null 2>&1 &"
    echo ""
    echo "  Terminal 1 (Logs):"
    echo "    python3 $SCRIPT_DIR/print_logs_only.py '$LOG_FILE' --rate $RATE"
    echo ""
    echo "  Current terminal (Bag playback):"
    echo "    ros2 bag play '$BAG_DIR' --rate $RATE --clock --remap /cmd_vel:=/cmd_vel_original  "
    echo ""
    exit 1
fi
