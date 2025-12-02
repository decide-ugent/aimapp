#!/bin/bash

# Launch all ROS2 nodes for LAPTOP usage
# Created for aimapp project - Laptop side

# Create log directory
LOG_DIR="$HOME/aimapp_logs/$(date +%Y%m%d_%H%M%S)_laptop"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"

# Terminal 1: SLAM Launch
gnome-terminal --tab --title="SLAM" -- bash -c "
echo 'Starting SLAM launch...'
ros2 launch aimapp slam_launch.py 2>&1 | tee $LOG_DIR/slam_launch.log
exec bash"

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
