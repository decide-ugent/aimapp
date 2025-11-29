#!/bin/bash

# Helper script to send navigation goals to nav2_client_node_goal.py
# Usage: ./send_goal.sh <goal_node_number>

if [ $# -eq 0 ]; then
    echo "Usage: $0 <goal_node_number>"
    echo "Example: $0 1"
    echo ""
    echo "This will send a goal to the nav2_client_node_goal.py running in continuous mode."
    exit 1
fi

GOAL_NODE=$1

echo "Sending goal node: $GOAL_NODE"
ros2 topic pub --once /nav2_client_goal_node std_msgs/msg/Int32 "data: $GOAL_NODE"

if [ $? -eq 0 ]; then
    echo "Goal $GOAL_NODE sent successfully!"
else
    echo "Failed to send goal. Make sure nav2_client_node_goal.py is running with --continuous flag."
    exit 1
fi
