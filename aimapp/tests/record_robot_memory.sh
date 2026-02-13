#!/bin/bash

# Monitor robot memory usage during operation
# Usage: ./record_robot_memory.sh [output_file]
# Run this BEFORE starting your navigation run

ROBOT_USER="husarion"
ROBOT_IP="192.168.1.2"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"

OUTPUT_FILE="${1:-robot_memory_log_$(date +%Y%m%d_%H%M%S).csv}"

echo "=========================================="
echo "ROBOT MEMORY MONITOR"
echo "=========================================="
echo "Logging to: $OUTPUT_FILE"
echo "Sampling every 30 seconds"
echo "Press Ctrl-C to stop"
echo "=========================================="
echo ""

# Create CSV header
echo "timestamp,uptime_sec,load_avg_1min,mem_total_mb,mem_used_mb,mem_free_mb,mem_available_mb,swap_total_mb,swap_used_mb,swap_free_mb,top_process_name,top_process_mem_mb" > "$OUTPUT_FILE"

# Monitor loop
while true; do
    # Get current timestamp
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")

    # Collect data from robot in one SSH session
    DATA=$(ssh "$ROBOT_SSH" 'bash -s' << 'ENDSSH'
# Get uptime in seconds
UPTIME_SEC=$(awk '{print $1}' /proc/uptime)

# Get load average (1 minute)
LOAD_AVG=$(uptime | awk -F'load average:' '{print $2}' | awk -F',' '{print $1}' | tr -d ' ')

# Get memory info (in MB)
MEM_INFO=$(free -m | awk 'NR==2 {printf "%s,%s,%s,%s", $2, $3, $4, $7}')

# Get swap info (in MB)
SWAP_INFO=$(free -m | awk 'NR==3 {printf "%s,%s,%s", $2, $3, $4}')

# Get top memory consuming process
TOP_PROC=$(ps aux --sort=-%mem | awk 'NR==2 {printf "%s,%.0f", $11, $6/1024}')

# Output all data comma-separated
echo "$UPTIME_SEC,$LOAD_AVG,$MEM_INFO,$SWAP_INFO,$TOP_PROC"
ENDSSH
)

    # Append to CSV
    echo "$TIMESTAMP,$DATA" >> "$OUTPUT_FILE"

    # Display current status
    echo "[$TIMESTAMP] Logged - Available mem: $(echo $DATA | awk -F',' '{print $6}')MB, Swap used: $(echo $DATA | awk -F',' '{print $9}')MB"

    # Wait 30 seconds
    sleep 30
done
