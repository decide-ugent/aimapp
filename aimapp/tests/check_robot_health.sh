#!/bin/bash

# Robot Health Check Script
# Run this on the Husarion robot to diagnose system issues
# Usage: ./check_robot_health.sh

echo "=========================================="
echo "ROBOT HEALTH CHECK"
echo "=========================================="
echo "Timestamp: $(date)"
echo "Hostname: $(hostname)"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print status
print_status() {
    local status=$1
    local message=$2
    if [ "$status" == "OK" ]; then
        echo -e "${GREEN}✓${NC} $message"
    elif [ "$status" == "WARN" ]; then
        echo -e "${YELLOW}⚠${NC} $message"
    else
        echo -e "${RED}✗${NC} $message"
    fi
}

# 1. System Load
echo "=========================================="
echo "1. SYSTEM LOAD"
echo "=========================================="
uptime
LOAD_AVG=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')
if (( $(echo "$LOAD_AVG < 2.0" | bc -l) )); then
    print_status "OK" "Load average is normal: $LOAD_AVG"
elif (( $(echo "$LOAD_AVG < 4.0" | bc -l) )); then
    print_status "WARN" "Load average is elevated: $LOAD_AVG"
else
    print_status "ERROR" "Load average is very high: $LOAD_AVG (possible I/O bottleneck)"
fi
echo ""

# 2. Memory Usage
echo "=========================================="
echo "2. MEMORY USAGE"
echo "=========================================="
free -h
AVAILABLE_MEM_MB=$(free -m | awk 'NR==2 {print $7}')
if [ "$AVAILABLE_MEM_MB" -gt 1000 ]; then
    print_status "OK" "Available memory: ${AVAILABLE_MEM_MB}MB"
elif [ "$AVAILABLE_MEM_MB" -gt 500 ]; then
    print_status "WARN" "Available memory is low: ${AVAILABLE_MEM_MB}MB"
else
    print_status "ERROR" "Available memory is critical: ${AVAILABLE_MEM_MB}MB"
fi
echo ""

# 3. Swap Usage
echo "=========================================="
echo "3. SWAP USAGE"
echo "=========================================="
cat /proc/meminfo | grep -i swap
SWAP_USED_KB=$(cat /proc/meminfo | grep "^SwapTotal:" | awk '{print $2}')
SWAP_FREE_KB=$(cat /proc/meminfo | grep "^SwapFree:" | awk '{print $2}')
SWAP_USED_PERCENT=$(echo "scale=2; (($SWAP_USED_KB - $SWAP_FREE_KB) * 100) / $SWAP_USED_KB" | bc 2>/dev/null || echo "0")
if (( $(echo "$SWAP_USED_PERCENT < 10" | bc -l) )); then
    print_status "OK" "Swap usage: ${SWAP_USED_PERCENT}%"
elif (( $(echo "$SWAP_USED_PERCENT < 50" | bc -l) )); then
    print_status "WARN" "Swap usage elevated: ${SWAP_USED_PERCENT}% (system may be running low on RAM)"
else
    print_status "ERROR" "Heavy swap usage: ${SWAP_USED_PERCENT}% (system is thrashing!)"
fi
echo ""

# 4. Disk Space
echo "=========================================="
echo "4. DISK SPACE"
echo "=========================================="
df -h
ROOT_USAGE=$(df -h / | awk 'NR==2 {print $5}' | tr -d '%')
if [ "$ROOT_USAGE" -lt 80 ]; then
    print_status "OK" "Root disk usage: ${ROOT_USAGE}%"
elif [ "$ROOT_USAGE" -lt 90 ]; then
    print_status "WARN" "Root disk usage is high: ${ROOT_USAGE}%"
else
    print_status "ERROR" "Root disk is nearly full: ${ROOT_USAGE}% (will cause slowdowns!)"
fi
echo ""

# 5. CPU Frequency (check for thermal throttling)
echo "=========================================="
echo "5. CPU FREQUENCY (Throttling Check)"
echo "=========================================="
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]; then
    CUR_FREQ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)
    MAX_FREQ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null)
    if [ -n "$CUR_FREQ" ] && [ -n "$MAX_FREQ" ]; then
        echo "Current frequency: $(($CUR_FREQ / 1000)) MHz"
        echo "Max frequency: $(($MAX_FREQ / 1000)) MHz"
        FREQ_PERCENT=$(echo "scale=2; ($CUR_FREQ * 100) / $MAX_FREQ" | bc)
        if (( $(echo "$FREQ_PERCENT > 80" | bc -l) )); then
            print_status "OK" "CPU running at ${FREQ_PERCENT}% of max frequency"
        else
            print_status "WARN" "CPU may be throttled: ${FREQ_PERCENT}% of max frequency"
        fi
    else
        print_status "WARN" "Could not read CPU frequency"
    fi
else
    print_status "WARN" "CPU frequency scaling not available"
fi
echo ""

# 6. Top Memory-Consuming Processes
echo "=========================================="
echo "6. TOP MEMORY-CONSUMING PROCESSES"
echo "=========================================="
ps aux --sort=-%mem | head -6
echo ""

# 7. Top CPU-Consuming Processes
echo "=========================================="
echo "7. TOP CPU-CONSUMING PROCESSES"
echo "=========================================="
ps aux --sort=-%cpu | head -6
echo ""

# 8. I/O Wait (requires sysstat package)
echo "=========================================="
echo "8. I/O STATISTICS"
echo "=========================================="
if command -v iostat &> /dev/null; then
    echo "Running iostat for 3 seconds..."
    iostat -x 1 3 | tail -20
    print_status "OK" "iostat available - check %iowait column (should be <10%)"
else
    print_status "WARN" "iostat not installed (install with: sudo apt install sysstat)"
fi
echo ""

# 9. Check for Stuck Processes (D state - uninterruptible sleep)
echo "=========================================="
echo "9. PROCESSES IN UNINTERRUPTIBLE SLEEP (D state)"
echo "=========================================="
STUCK_PROCS=$(ps aux | grep " D" | grep -v grep)
if [ -z "$STUCK_PROCS" ]; then
    print_status "OK" "No processes stuck in I/O wait"
else
    print_status "ERROR" "Processes stuck in I/O wait detected:"
    echo "$STUCK_PROCS"
fi
echo ""

# 10. Recent System Errors
echo "=========================================="
echo "10. RECENT SYSTEM ERRORS (last 30 lines of dmesg)"
echo "=========================================="
if [ "$EUID" -eq 0 ]; then
    dmesg | tail -30
    ERROR_COUNT=$(dmesg | grep -i "error\|fail\|critical" | wc -l)
    if [ "$ERROR_COUNT" -eq 0 ]; then
        print_status "OK" "No errors in dmesg"
    else
        print_status "WARN" "$ERROR_COUNT error/warning messages in dmesg"
    fi
else
    echo "Not running as root - skipping dmesg (run with sudo for full diagnostics)"
fi
echo ""

# 11. ROS-specific checks (if ROS is running)
echo "=========================================="
echo "11. ROS PROCESSES"
echo "=========================================="
ROS_PROCS=$(ps aux | grep -E "ros2|python3.*aimapp" | grep -v grep | wc -l)
if [ "$ROS_PROCS" -gt 0 ]; then
    echo "Found $ROS_PROCS ROS-related processes:"
    ps aux | grep -E "ros2|python3.*aimapp" | grep -v grep | head -10
else
    print_status "OK" "No ROS processes running"
fi
echo ""

# Summary
echo "=========================================="
echo "HEALTH CHECK COMPLETE"
echo "=========================================="
echo ""
echo "Legend:"
echo -e "  ${GREEN}✓${NC} = OK"
echo -e "  ${YELLOW}⚠${NC} = Warning"
echo -e "  ${RED}✗${NC} = Critical"
echo ""
