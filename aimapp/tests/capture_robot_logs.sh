#!/bin/bash

# Capture logs from remote robot's tmux sessions
# This script SSHs into the robot and captures logs from the start_rosbot.sh tmux sessions
#
# Usage: ./capture_robot_logs.sh [debug_name] [duration_seconds]
# Example: ./capture_robot_logs.sh "turn_issue_1" 60

# Robot SSH configuration
ROBOT_USER="husarion"
ROBOT_IP="192.168.1.2"
ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
ROBOT_HOME="/home/husarion"

DEBUG_NAME="${1:-debug_session}"
DURATION="${2:-0}"  # 0 means capture continuously until Ctrl-C
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create output directory
OUTPUT_DIR="$HOME/robot_logs/${DEBUG_NAME}_${TIMESTAMP}"

mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo "Robot Log Capture"
echo "=========================================="
echo "Debug name: $DEBUG_NAME"
echo "Timestamp: $TIMESTAMP"
echo "Output directory: $OUTPUT_DIR"
echo "Robot: $ROBOT_SSH"
echo ""

# Check SSH connection
echo "Checking SSH connection to robot..."
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT_SSH" echo "Connected" 2>/dev/null; then
    echo "ERROR: Cannot connect to robot via SSH"
    echo "Please ensure:"
    echo "  1. Robot is powered on and connected to network"
    echo "  2. IP address is correct ($ROBOT_IP)"
    echo "  3. SSH key-based authentication is set up"
    exit 1
fi
echo "SSH connection verified"
echo ""

# Find tmux sessions on robot
echo "Looking for tmux sessions on robot..."
TMUX_SESSIONS=$(ssh "$ROBOT_SSH" "tmux list-sessions 2>/dev/null" || echo "")

if [ -z "$TMUX_SESSIONS" ]; then
    echo "WARNING: No tmux sessions found on robot"
    echo "Make sure start_rosbot.sh is running on the robot"
    echo ""
    echo "Current tmux sessions:"
    echo "(none)"
else
    echo "Found tmux sessions:"
    echo "$TMUX_SESSIONS"
fi
echo ""

# Capture bash script to run on robot
LOG_CAPTURE_SCRIPT=$(cat <<'REMOTE_SCRIPT'
#!/bin/bash

OUTPUT_DIR="$1"
DURATION="$2"
TIMESTAMP="$3"

mkdir -p "$OUTPUT_DIR"

echo "Starting log capture on robot..."
echo "Output: $OUTPUT_DIR"
echo ""

# Function to capture logs from a tmux window
capture_tmux_window() {
    local session="$1"
    local window="$2"
    local output_file="$3"

    echo "Capturing logs from tmux session '$session' window '$window'..."

    # Capture pane content
    tmux capture-pane -t "${session}:${window}" -p -S -1000000 > "$output_file" 2>/dev/null

    if [ $? -eq 0 ] && [ -s "$output_file" ]; then
        echo "  ✓ Saved to: $output_file"
    else
        echo "  ✗ Failed to capture (window may not exist)"
        rm -f "$output_file"
    fi
}

# Get list of tmux sessions
SESSIONS=$(tmux list-sessions -F "#{session_name}" 2>/dev/null)

if [ -z "$SESSIONS" ]; then
    echo "No tmux sessions found. Creating log anyway..."
    echo "No tmux sessions active at $(date)" > "$OUTPUT_DIR/no_sessions.txt"
else
    echo "Found tmux sessions: $SESSIONS"
    echo ""

    # For each session, get all windows
    for SESSION in $SESSIONS; do
        echo "Processing session: $SESSION"
        WINDOWS=$(tmux list-windows -t "$SESSION" -F "#{window_index}:#{window_name}" 2>/dev/null)

        if [ -z "$WINDOWS" ]; then
            echo "  No windows found in session $SESSION"
            continue
        fi

        for WINDOW in $WINDOWS; do
            WINDOW_INDEX=$(echo "$WINDOW" | cut -d: -f1)
            WINDOW_NAME=$(echo "$WINDOW" | cut -d: -f2)

            # Sanitize window name for filename
            SAFE_NAME=$(echo "$WINDOW_NAME" | tr ' /' '__')
            OUTPUT_FILE="$OUTPUT_DIR/${SESSION}_${WINDOW_INDEX}_${SAFE_NAME}_${TIMESTAMP}.log"

            capture_tmux_window "$SESSION" "$WINDOW_INDEX" "$OUTPUT_FILE"
        done
        echo ""
    done
fi

# Capture system information
echo "Capturing system information..."
{
    echo "=========================================="
    echo "Robot System Information"
    echo "Captured at: $(date)"
    echo "=========================================="
    echo ""
    echo "--- ROS 2 Node List ---"
    ros2 node list 2>/dev/null || echo "Failed to get node list"
    echo ""
    echo "--- ROS 2 Topic List ---"
    ros2 topic list 2>/dev/null || echo "Failed to get topic list"
    echo ""
    echo "--- Active Processes ---"
    ps aux | grep -E "ros2|python|launch" | grep -v grep
    echo ""
    echo "--- Network Interfaces ---"
    ip addr show
    echo ""
    echo "--- Disk Usage ---"
    df -h
    echo ""
    echo "--- Memory Info ---"
    free -h
    echo ""
    echo "=========================================="
} > "$OUTPUT_DIR/system_info_${TIMESTAMP}.txt"
echo "  ✓ Saved system info to: $OUTPUT_DIR/system_info_${TIMESTAMP}.txt"

# Capture CPU usage snapshot
echo "Capturing CPU usage snapshot..."
{
    echo "=========================================="
    echo "CPU Usage Snapshot"
    echo "Captured at: $(date)"
    echo "=========================================="
    echo ""
    echo "--- CPU Information ---"
    lscpu 2>/dev/null || echo "lscpu not available"
    echo ""
    echo "--- Top CPU Processes (snapshot) ---"
    top -bn1 | head -n 20
    echo ""
    echo "--- CPU Usage per Core ---"
    mpstat -P ALL 1 1 2>/dev/null || echo "mpstat not available (install sysstat package)"
    echo ""
    echo "=========================================="
} > "$OUTPUT_DIR/cpu_usage_${TIMESTAMP}.txt"
echo "  ✓ Saved CPU usage to: $OUTPUT_DIR/cpu_usage_${TIMESTAMP}.txt"

# Capture GPU usage if available (jtop for Jetson or nvidia-smi for other NVIDIA GPUs)
echo "Checking for GPU monitoring tools..."
if command -v jtop &> /dev/null; then
    echo "  Found jtop (Jetson platform), capturing GPU stats..."
    {
        echo "=========================================="
        echo "GPU Usage (jtop) - Jetson Platform"
        echo "Captured at: $(date)"
        echo "=========================================="
        echo ""
        echo "Note: jtop requires sudo. Attempting to capture stats..."
        echo ""
        # Try to get jtop stats (requires proper permissions)
        timeout 5 jtop --text 2>/dev/null || echo "Unable to capture jtop stats (may require sudo or permissions)"
        echo ""
        echo "--- Tegra Stats (alternative) ---"
        timeout 3 tegrastats --interval 1000 2>/dev/null | head -n 3 || echo "tegrastats not available"
        echo ""
        echo "=========================================="
    } > "$OUTPUT_DIR/gpu_usage_${TIMESTAMP}.txt"
    echo "  ✓ Saved GPU usage (jtop) to: $OUTPUT_DIR/gpu_usage_${TIMESTAMP}.txt"
elif command -v nvidia-smi &> /dev/null; then
    echo "  Found nvidia-smi, capturing GPU stats..."
    {
        echo "=========================================="
        echo "GPU Usage (nvidia-smi)"
        echo "Captured at: $(date)"
        echo "=========================================="
        echo ""
        nvidia-smi
        echo ""
        echo "--- GPU Process Details ---"
        nvidia-smi pmon -c 1 2>/dev/null || echo "pmon not available"
        echo ""
        echo "=========================================="
    } > "$OUTPUT_DIR/gpu_usage_${TIMESTAMP}.txt"
    echo "  ✓ Saved GPU usage (nvidia-smi) to: $OUTPUT_DIR/gpu_usage_${TIMESTAMP}.txt"
else
    echo "  No GPU monitoring tools found (jtop or nvidia-smi)"
    {
        echo "=========================================="
        echo "GPU Usage"
        echo "Captured at: $(date)"
        echo "=========================================="
        echo ""
        echo "No GPU monitoring tools available (jtop or nvidia-smi not found)"
        echo ""
        echo "If this is a Jetson platform, install jtop:"
        echo "  sudo pip3 install jetson-stats"
        echo ""
        echo "=========================================="
    } > "$OUTPUT_DIR/gpu_usage_${TIMESTAMP}.txt"
    echo "  ⚠ No GPU tools available, created placeholder file"
fi

echo ""
echo "Log capture complete!"
echo "Files saved to: $OUTPUT_DIR"
REMOTE_SCRIPT
)

# Copy capture script to robot
echo "Copying log capture script to robot..."
echo "$LOG_CAPTURE_SCRIPT" | ssh "$ROBOT_SSH" "cat > /tmp/capture_logs_${TIMESTAMP}.sh && chmod +x /tmp/capture_logs_${TIMESTAMP}.sh"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to copy capture script to robot"
    exit 1
fi

# Create remote output directory
REMOTE_OUTPUT_DIR="${ROBOT_HOME}/debug_logs/${TIMESTAMP}"
echo "Creating remote output directory: $REMOTE_OUTPUT_DIR"
ssh "$ROBOT_SSH" "mkdir -p $REMOTE_OUTPUT_DIR"

# Execute capture script on robot
if [ "$DURATION" -gt 0 ]; then
    echo ""
    echo "Capturing logs for $DURATION seconds..."
    echo "=========================================="
    ssh "$ROBOT_SSH" "/tmp/capture_logs_${TIMESTAMP}.sh $REMOTE_OUTPUT_DIR $DURATION $TIMESTAMP"
else
    echo ""
    echo "Capturing logs (Press Ctrl-C to stop)..."
    echo "=========================================="
    ssh "$ROBOT_SSH" "/tmp/capture_logs_${TIMESTAMP}.sh $REMOTE_OUTPUT_DIR 0 $TIMESTAMP"
fi

echo ""
echo "=========================================="
echo "Downloading logs from robot..."
echo "=========================================="

# Download logs from robot
scp -r "${ROBOT_SSH}:${REMOTE_OUTPUT_DIR}/*" "$OUTPUT_DIR/" 2>/dev/null

if [ $? -eq 0 ]; then
    echo "✓ Logs downloaded successfully"
    echo ""
    echo "Local logs saved to:"
    echo "  $OUTPUT_DIR"
    echo ""
    echo "Files:"
    ls -lh "$OUTPUT_DIR/"

    # Optionally clean up remote logs
    read -p "Delete logs from robot? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ssh "$ROBOT_SSH" "rm -rf $REMOTE_OUTPUT_DIR /tmp/capture_logs_${TIMESTAMP}.sh"
        echo "Remote logs deleted"
    else
        echo "Remote logs kept at: $REMOTE_OUTPUT_DIR"
    fi
else
    echo "WARNING: Failed to download some logs from robot"
    echo "Remote logs are still available at: $REMOTE_OUTPUT_DIR on the robot"
fi

echo ""
echo "=========================================="
echo "Log capture complete!"
echo "=========================================="
