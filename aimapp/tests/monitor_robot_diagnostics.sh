#!/bin/bash

# Comprehensive Robot Diagnostics Monitor
# Monitors memory, CPU, micro-ROS, and system health during long navigation runs
# Saves all data for later analysis
#
# Usage: ./monitor_robot_diagnostics.sh [session_name]
# Example: ./monitor_robot_diagnostics.sh "45min_nav_test"

ROBOT_USER="husarion"
ROBOT_IP="192.168.1.2"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"

SESSION_NAME="${1:-robot_diag_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="$HOME/robot_diagnostics/${SESSION_NAME}"

mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo "ROBOT DIAGNOSTICS MONITOR"
echo "=========================================="
echo "Session: $SESSION_NAME"
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "This will monitor:"
echo "  - Memory usage (every 45s)"
echo "  - micro-ROS agent status (every 45s)"
echo "  - System health snapshot (start/end)"
echo "  - USB/serial errors (continuous)"
echo ""
echo "Press Ctrl-C to stop all monitoring"
echo "=========================================="
echo ""

# Trap to cleanup on exit
cleanup() {
    echo ""
    echo "=========================================="
    echo "Stopping all monitors..."
    echo "=========================================="

    # Kill all background jobs
    jobs -p | xargs -r kill 2>/dev/null

    echo ""
    echo "Generating summary report..."

    # Create summary report
    cat > "$OUTPUT_DIR/SUMMARY.txt" << EOF
========================================
ROBOT DIAGNOSTICS SESSION SUMMARY
========================================
Session: $SESSION_NAME
Start time: $(cat "$OUTPUT_DIR/session_start.txt" 2>/dev/null || echo "Unknown")
End time: $(date)
Output directory: $OUTPUT_DIR

========================================
FILES COLLECTED
========================================
$(ls -lh "$OUTPUT_DIR")

========================================
QUICK ANALYSIS
========================================

Memory Usage:
$(tail -5 "$OUTPUT_DIR/memory_log.csv" 2>/dev/null || echo "No memory data")

micro-ROS Agent Status:
$(tail -5 "$OUTPUT_DIR/microros_monitor.log" 2>/dev/null || echo "No micro-ROS data")

System Errors:
$(grep -i "error\|fail\|killed" "$OUTPUT_DIR/dmesg_monitor.log" 2>/dev/null | tail -10 || echo "No errors detected")

========================================
NEXT STEPS
========================================
1. Analyze memory log:
   python3 ./src/aimapp/aimapp/tests/analyze_memory_log.py $OUTPUT_DIR/memory_log.csv

2. Check micro-ROS status:
   cat $OUTPUT_DIR/microros_monitor.log

3. Review system health:
   diff $OUTPUT_DIR/health_start.txt $OUTPUT_DIR/health_end.txt

4. Check for USB/serial errors:
   grep -i "error\|fail" $OUTPUT_DIR/dmesg_monitor.log

========================================
EOF

    cat "$OUTPUT_DIR/SUMMARY.txt"
    echo ""
    echo "All diagnostics saved to: $OUTPUT_DIR"
    echo ""
    exit 0
}

trap cleanup INT TERM

# Record session start time
date > "$OUTPUT_DIR/session_start.txt"

# 1. Initial system health snapshot
echo "Taking initial health snapshot..."
ssh "$ROBOT_SSH" 'bash -s' < "$(dirname "$0")/check_robot_health.sh" > "$OUTPUT_DIR/health_start.txt" 2>&1
echo "✓ Health snapshot saved"

# 2. Start memory monitoring (every 45s)
echo "Starting memory monitor..."
(
    echo "timestamp,uptime_sec,load_avg_1min,mem_total_mb,mem_used_mb,mem_free_mb,mem_available_mb,swap_total_mb,swap_used_mb,swap_free_mb,top_process_name,top_process_mem_mb" > "$OUTPUT_DIR/memory_log.csv"

    while true; do
        TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
        DATA=$(ssh "$ROBOT_SSH" 'bash -s' << 'ENDSSH'
UPTIME_SEC=$(awk '{print $1}' /proc/uptime)
LOAD_AVG=$(uptime | awk -F'load average:' '{print $2}' | awk -F',' '{print $1}' | tr -d ' ')
MEM_INFO=$(free -m | awk 'NR==2 {printf "%s,%s,%s,%s", $2, $3, $4, $7}')
SWAP_INFO=$(free -m | awk 'NR==3 {printf "%s,%s,%s", $2, $3, $4}')
TOP_PROC=$(ps aux --sort=-%mem | awk 'NR==2 {printf "%s,%.0f", $11, $6/1024}')
echo "$UPTIME_SEC,$LOAD_AVG,$MEM_INFO,$SWAP_INFO,$TOP_PROC"
ENDSSH
)
        echo "$TIMESTAMP,$DATA" >> "$OUTPUT_DIR/memory_log.csv"
        sleep 45
    done
) &
MEMORY_PID=$!
echo "✓ Memory monitor started (PID: $MEMORY_PID)"

# 3. Monitor micro-ROS agent (every 45s)
echo "Starting micro-ROS monitor..."
(
    echo "=== micro-ROS Agent Monitor ===" > "$OUTPUT_DIR/microros_monitor.log"
    echo "Started: $(date)" >> "$OUTPUT_DIR/microros_monitor.log"
    echo "" >> "$OUTPUT_DIR/microros_monitor.log"

    while true; do
        echo "--- $(date) ---" >> "$OUTPUT_DIR/microros_monitor.log"

        # Get micro-ROS agent process info
        MICRO_INFO=$(ssh "$ROBOT_SSH" "ps aux | grep micro_ros_agent | grep -v grep")

        if [ -n "$MICRO_INFO" ]; then
            echo "$MICRO_INFO" >> "$OUTPUT_DIR/microros_monitor.log"

            # Extract memory usage
            MEM_MB=$(echo "$MICRO_INFO" | awk '{printf "%.1f", $6/1024}')
            CPU=$(echo "$MICRO_INFO" | awk '{print $3}')
            echo "  Memory: ${MEM_MB}MB, CPU: ${CPU}%" >> "$OUTPUT_DIR/microros_monitor.log"
        else
            echo "WARNING: micro-ROS agent NOT RUNNING!" >> "$OUTPUT_DIR/microros_monitor.log"
        fi

        echo "" >> "$OUTPUT_DIR/microros_monitor.log"
        sleep 45
    done
) &
MICROROS_PID=$!
echo "✓ micro-ROS monitor started (PID: $MICROROS_PID)"

# 4. Monitor /tf topic rate (every 45s)
echo "Starting /tf monitor..."
(
    echo "=== /tf Topic Monitor ===" > "$OUTPUT_DIR/tf_monitor.log"
    echo "Started: $(date)" >> "$OUTPUT_DIR/tf_monitor.log"
    echo "" >> "$OUTPUT_DIR/tf_monitor.log"

    while true; do
        echo "--- $(date) ---" >> "$OUTPUT_DIR/tf_monitor.log"

        # Check /tf publishing rate (timeout after 5 seconds)
        TF_RATE=$(timeout 5 ros2 topic hz /tf 2>&1 | grep "average rate" || echo "No data")
        echo "/tf rate: $TF_RATE" >> "$OUTPUT_DIR/tf_monitor.log"

        echo "" >> "$OUTPUT_DIR/tf_monitor.log"
        sleep 45
    done
) &
TF_PID=$!
echo "✓ /tf monitor started (PID: $TF_PID)"

# 5. Monitor kernel messages (continuous)
echo "Starting kernel message monitor..."
ssh "$ROBOT_SSH" 'dmesg -w' > "$OUTPUT_DIR/dmesg_monitor.log" 2>&1 &
DMESG_PID=$!
echo "✓ Kernel monitor started (PID: $DMESG_PID)"

# 6. Monitor ROS node list (every 60s)
echo "Starting ROS node monitor..."
(
    echo "=== ROS Nodes Monitor ===" > "$OUTPUT_DIR/ros_nodes_monitor.log"
    echo "Started: $(date)" >> "$OUTPUT_DIR/ros_nodes_monitor.log"
    echo "" >> "$OUTPUT_DIR/ros_nodes_monitor.log"

    while true; do
        echo "--- $(date) ---" >> "$OUTPUT_DIR/ros_nodes_monitor.log"
        ros2 node list 2>&1 | tee -a "$OUTPUT_DIR/ros_nodes_monitor.log"
        NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
        echo "Total nodes: $NODE_COUNT" >> "$OUTPUT_DIR/ros_nodes_monitor.log"
        echo "" >> "$OUTPUT_DIR/ros_nodes_monitor.log"
        sleep 60
    done
) &
NODES_PID=$!
echo "✓ ROS nodes monitor started (PID: $NODES_PID)"

echo ""
echo "=========================================="
echo "All monitors running!"
echo "=========================================="
echo "Logs being written to: $OUTPUT_DIR"
echo ""
echo "Monitor status:"
echo "  Memory: Running (updating every 45s)"
echo "  micro-ROS: Running (updating every 45s)"
echo "  /tf topic: Running (updating every 45s)"
echo "  Kernel msgs: Running (continuous)"
echo "  ROS nodes: Running (updating every 60s)"
echo ""
echo "Press Ctrl-C when navigation run is complete"
echo "=========================================="
echo ""

# Wait for user interrupt
while true; do
    sleep 1
done
