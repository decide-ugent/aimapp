#!/bin/bash

#NOTE: the .sh have been written by claude

# Simple launcher for the exploration mode GUI
# This script should be run from the ROS workspace root directory
# It launches the visual interface to choose exploration mode

# Get the current working directory (should be ROS workspace)
ROS_WS_DIR="$(pwd)"

# Check if we're in a ROS workspace (has src directory)
if [ ! -d "src/aimapp" ]; then
    echo "Error: This script must be run from the ROS workspace root directory"
    echo "Current directory: $ROS_WS_DIR"
    echo "Expected structure: <ros_ws>/src/aimapp"
    exit 1
fi

# Check if Python3 and tkinter are available
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    exit 1
fi

# Launch the GUI from the command_GUI_files directory
SCRIPT_DIR="$ROS_WS_DIR/src/aimapp/command_GUI_files"
python3 "$SCRIPT_DIR/launch_exploration_gui.py"
