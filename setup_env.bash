#!/bin/bash

# Check if the input argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <input>"
    echo "Input should be 1:env start, 2:agent start, or 3:model start."
    exit 1
fi

# Read the input argument
input=$1

# Perform different actions based on the input value
case $input in
    1)
        source install/setup.bash 
        ros2 launch aimapp warehouse_launch.py
        ;;
    2)
        source install/setup.bash 
        ros2 launch aimapp spawn_turtle_launch.py x:=0.0 y:=0.0
        ;;
    3)
        source install/setup.bash 
        ros2 launch aimapp agent_launch.py
        ;;
    *)
        echo "Invalid input. Please provide 1:env start, 2:agent start, or 3:model start."
        exit 1
        ;;
esac