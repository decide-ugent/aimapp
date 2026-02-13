# Recording and Debug Scripts

This directory contains tools for recording experiment runs and debugging TF/odometry/navigation issues.

---
Most Likely Culprit: micro-ROS Connection
The microROS agent (connects robot firmware to ROS2) probably crashes or loses connection after ~26 minutes. Quick test to confirm: During your next run, monitor the micro-ROS agent process:

ssh husarion@192.168.1.2 'watch -n 5 "ps aux | grep micro"'
If micro-ROS dies around minute 26, that's your smoking gun. Why would micro-ROS die?
Memory leak in the micro-ROS agent
Firmware buffer overflow after extended operation
Network buffer exhaustion (DDS middleware)
The memory monitoring script will show if the micro_ros_agent process is growing. Run it during your next session!

watch -n 5 "ps aux | grep micro"
watch -n 30 "ps aux | grep micro_ros_agent | grep -v grep"
dmesg -w | grep -i "usb\|tty\|micro"
## Recording Experiment Runs

### `record_run.sh` - Record ROS Topics for Experiments

Records all navigation and AIF-related ROS topics during an experimental run.

**Usage**:
```bash
./src/aimapp/aimapp/tests/record_run.sh [test_id] [run_name]
```

**Examples**:
```bash
./src/aimapp/aimapp/tests/record_run.sh 5 "exploration_run_1"
./src/aimapp/aimapp/tests/record_run.sh 7
```

**Topics Recorded**:
- **Odometry & TF**: `/odometry/filtered`, `/odometry/shifted`, `/shifted_odom`, `/tf`, `/tf_static`
- **Navigation**: `/cmd_vel`, `/initial_pose`, `/nav2_client_goal_pose`, `/nav2_navigation_result`
- **Sensors**: `/scan`, `/scan_filtered`, `/joy`, `/diagnostics`
- **Mapping**: `/map`, global/local costmaps with updates
- **AIF Visualization**: `/waypoints`, `/visitable_nodes`, `/goal_pose_marker`, `/mcts_reward_visualization`

**Output**: `~/experiment_recordings/test_[id]/[run_name]_TIMESTAMP/`
- `run_TIMESTAMP_0.db3` - RosBag file
- `run_metadata.txt` - Run information

**Note**: Agent logs are already saved separately by the system.

**Replay**:
```bash
ros2 bag play ~/experiment_recordings/test_5/exploration_run_1_*/
```

./src/aimapp/aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 3.0


---

## Debugging TF/Odometry Issues

### `debug_tf_issue.sh` **Main Debug Script**

When you observe TF synchronization or odometry issues (e.g., robot turns but TF base_link doesn't update), use this script to capture comprehensive debug data. **This runs independently of your test experiments** - use when troubleshooting issues.

**Usage**:
```bash
./src/aimapp/aimapp/tests/debug_tf_issue.sh [debug_name]
```

**Examples**:
```bash
./src/aimapp/aimapp/tests/debug_tf_issue.sh "turn_issue_1"
./src/aimapp/aimapp/tests/debug_tf_issue.sh "tf_not_updating"
```

**What it does**:

Launches **2 terminal windows** simultaneously:

1. **RosBag-Recording**: Records basic TF and odometry topics
   - `/odometry/filtered`, `/imu_broadcaster/imu`
   - `/tf`, `/tf_static`
   - `/cmd_vel`, `/scan`, `/map`, `/joy`, `/diagnostics`
   - `/cartesian_impedance_controller/equilibrium_pose`

2. **Robot-Logs**: Captures robot system state
   - All tmux session logs
   - **CPU usage** (lscpu, top, mpstat)
   - **GPU usage** (jtop for Jetson, nvidia-smi for other GPUs)
   - ROS nodes/topics, network, disk, memory info

**When to use**: When you see the TF issue occurring (independent of experiments)

**How to stop**: Press **Ctrl-C** in both terminal windows

**Output**:
- RosBag: `~/rosbag_debug/[debug_name]_TIMESTAMP/`
- Robot logs: `~/robot_logs/[debug_name]_TIMESTAMP/`
  - `system_info_TIMESTAMP.txt`
  - `cpu_usage_TIMESTAMP.txt`
  - `gpu_usage_TIMESTAMP.txt`
  - `[tmux_window]_TIMESTAMP.log`

**Analysis**:
```bash
# Play back rosbag
ros2 bag play ~/rosbag_debug/turn_issue_1_*/

# Check CPU usage
cat ~/robot_logs/turn_issue_1_*/cpu_usage_*.txt

# Check GPU usage
cat ~/robot_logs/turn_issue_1_*/gpu_usage_*.txt

# View tmux logs
less ~/robot_logs/turn_issue_1_*/*.log
```

---

## Supporting Scripts

These scripts are called by the main scripts above. You generally don't need to run them directly.

### `record_tf_debug.sh`

Records basic TF/odometry topics to rosbag. Called by `debug_tf_issue.sh`.

### `capture_robot_logs.sh`

Captures robot tmux logs and system resource usage (CPU/GPU). Called by `debug_tf_issue.sh`.

Includes:
- All tmux window logs from the robot
- CPU monitoring (lscpu, top, mpstat)
- GPU monitoring (jtop for Jetson, nvidia-smi for NVIDIA GPUs)
- System information (ROS nodes, topics, network, memory, disk)

---

## Output Directory Structure

### Experiment Recordings
```
~/experiment_recordings/
└── test_5/
    └── exploration_run_1_20231218_143022/
        ├── run_20231218_143022_0.db3
        └── run_metadata.txt
```

### Debug Recordings
```
~/rosbag_debug/
└── turn_issue_1_20231218_143022/
    └── tf_debug_20231218_143022_0.db3

~/robot_logs/
└── turn_issue_1_20231218_143022/
    ├── system_info_20231218_143022.txt
    ├── cpu_usage_20231218_143022.txt
    ├── gpu_usage_20231218_143022.txt
    └── [various_tmux_logs].log
```

---

## ⚙️ Requirements

### On Laptop
- ROS 2 workspace sourced (`source install/setup.bash`)
- SSH key-based authentication to robot
- `gnome-terminal` (for debug_tf_issue.sh)

### On Robot (husarion@192.168.1.2)
- `tmux` with active sessions (for log capture)
- ROS 2 nodes running
- **Optional**: `jtop` for GPU monitoring on Jetson
  ```bash
  sudo pip3 install jetson-stats
  ```
- **Optional**: `sysstat` for detailed CPU stats
  ```bash
  sudo apt install sysstat
  ```

---

## 🔧 Setup SSH Keys

If you haven't set up SSH keys yet:

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096

# Copy to robot
ssh-copy-id husarion@192.168.1.2

# Test connection
ssh husarion@192.168.1.2 'echo Connection successful'
```

---

## 💡 Tips

1. **Use descriptive names**:
   ```bash
   # For debugging (independent of experiments)
   ./debug_tf_issue.sh "robot_turns_but_tf_stuck"

   # For experiment recordings (linked to test_id)
   ./record_run.sh 5 "baseline_exploration"
   ```

2. **Start recording before the issue occurs** (for debugging) to capture the transition

3. **Check disk space** before long recordings:
   ```bash
   df -h ~
   ```

4. **Compress old recordings** to save space:
   ```bash
   tar -czf recordings_archive.tar.gz ~/experiment_recordings/ ~/rosbag_debug/ ~/robot_logs/
   ```

---

## 🐛 Troubleshooting

### "Cannot connect to robot via SSH"
- Ensure robot is powered on: `ping 192.168.1.2`
- Set up SSH keys: `ssh-copy-id husarion@192.168.1.2`

### "ros2 command not found"
- Source your workspace: `source install/setup.bash`

### "No tmux sessions found"
- Check if `launch_model_as_action_robot.sh` is running on robot
- View available sessions: `ssh husarion@192.168.1.2 "tmux list-sessions"`

### "No GPU monitoring tools found"
- For Jetson: `sudo pip3 install jetson-stats`
- For other NVIDIA: Install CUDA toolkit with nvidia-smi

### RosBag recording fails
- Check if topics exist: `ros2 topic list`
- Check disk space: `df -h`
- Ensure ROS_DOMAIN_ID matches robot

---

---

## Robot Health Monitoring

### `check_robot_health.sh` - System Health Check

Quick diagnostic script to check robot system health (memory, CPU, disk, processes).

**Usage**:
```bash
# On robot
./check_robot_health.sh

# Or from laptop via SSH
ssh husarion@192.168.1.2 'bash -s' < ./src/aimapp/aimapp/tests/check_robot_health.sh
```

**Checks**:
- System load average
- Memory usage and availability
- Swap usage (indicates memory pressure)
- Disk space
- CPU frequency (thermal throttling detection)
- Top memory/CPU consuming processes
- Stuck processes (I/O wait)
- Recent system errors (requires sudo)

**Output**: Color-coded health indicators (✓ OK, ⚠ Warning, ✗ Critical)

---

### `monitor_robot_diagnostics.sh` - **ALL-IN-ONE Monitor (RECOMMENDED)**

Comprehensive monitoring tool that tracks everything during navigation runs and saves all data for later analysis.

**Usage**:
```bash
# Start ALL monitors before your navigation run
./src/aimapp/aimapp/tests/monitor_robot_diagnostics.sh "30min_test"

# Run your navigation in another terminal...
./command_GUI_files/launch_model_as_action_robot.sh 0

# When done, press Ctrl-C in the monitor terminal
# Analysis summary is automatically generated
```

**What it monitors:**
- Memory usage (every 30s) → `memory_log.csv`
- micro-ROS agent status (every 30s) → `microros_monitor.log`
- /tf topic rate (every 30s) → `tf_monitor.log`
- Kernel messages (continuous) → `dmesg_monitor.log`
- ROS node list (every 60s) → `ros_nodes_monitor.log`
- System health snapshots (start/end) → `health_start.txt`, `health_end.txt`

**Output**: `~/robot_diagnostics/[session_name]/`
- All logs saved with timestamps
- Automatic summary report generated on exit
- Ready for analysis scripts

**When to use**:
- **Debugging the 26-minute slowdown issue**
- Long navigation runs (>20 minutes)
- Investigating memory leaks, crashes, or performance degradation

---

### `record_robot_memory.sh` - Memory Usage Monitor (Standalone)

Records only robot memory and CPU metrics. **Use `monitor_robot_diagnostics.sh` instead for complete diagnostics.**

**Usage**:
```bash
# Start monitoring BEFORE your navigation run
./src/aimapp/aimapp/tests/record_robot_memory.sh &

# Run your navigation...

# Stop monitoring (Ctrl-C or kill)
pkill -f record_robot_memory
```

**Output**: `robot_memory_log_YYYYMMDD_HHMMSS.csv`
- Samples every 30 seconds
- Records: timestamp, uptime, load average, memory (total/used/free/available), swap usage, top process

**When to use**: During long navigation runs (>20 minutes) to diagnose:
- Memory leaks
- Swap thrashing
- Process memory growth
- Resource exhaustion

---

### `analyze_memory_log.py` - Memory Log Analyzer

Analyzes memory monitoring logs to detect leaks and generate visualizations.

**Usage**:
```bash
python3 ./src/aimapp/aimapp/tests/analyze_memory_log.py robot_memory_log_20251218_153045.csv
```

**Output**:
- **Console**: Summary statistics, leak detection, critical warnings
- **PNG plot**: Memory usage, swap, load average, and top process over time

**Features**:
- Memory leak rate calculation (MB/minute)
- Swap thrashing detection
- Critical event timeline (when did swap start?)
- Top memory-consuming processes
- Visual trend analysis

**Example workflow**:
```bash
# 1. Start memory monitoring
./src/aimapp/aimapp/tests/record_robot_memory.sh &

# 2. Run your 30-minute navigation task
./command_GUI_files/launch_model_as_action_robot.sh 0

# 3. After run completes, stop monitoring
pkill -f record_robot_memory

# 4. Analyze the log
python3 ./src/aimapp/aimapp/tests/analyze_memory_log.py robot_memory_log_*.csv
```

---

## Replaying Experiments with Synchronized Logs

### `play_bag_with_logs.sh` - Replay with Agent Logs

Replays a recorded experiment at custom speed while printing agent logs synchronized to the bag's timestamp progression. Perfect for reviewing experiments and understanding what the agent was doing at each moment.

**Usage**:
```bash
./src/aimapp/aimapp/tests/play_bag_with_logs.sh <experiment_dir> [rate]
```

**Examples**:
```bash
# Play at 3x speed (default)
./src/aimapp/aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534

# Play at normal speed
./src/aimapp/aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 1.0

# Play at 5x speed
./src/aimapp/aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 5.0
```

**Controls**:
- **Ctrl+Z**: Pause playback (pauses both bag and logs)
- **`fg`**: Resume playback from where you paused
- **Ctrl+C**: Stop playback completely

**What it does**:
- Plays the ROS2 bag at the specified rate (e.g., 3x = 3 times faster)
- Prints agent logs (`minimal_agent_launch.log`) synchronized to bag time
- Logs appear in the terminal at the correct moment relative to the bag playback
- Both bag and logs pause/resume together

**When to use**:
- Reviewing recorded experiments
- Understanding agent decision-making timeline
- Debugging issues that occurred during a run
- Creating demonstrations at faster speed

**Output**: Terminal shows both bag playback status and timestamped agent logs

---

## 📝 Quick Reference

| Task | Command |
|------|---------|
| Record experiment run | `./record_run.sh 5 "run_name"` |
| **Replay with logs (3x speed)** | `./play_bag_with_logs.sh ~/experiment_recordings/test_0/run_*/` |
| Debug TF issue | `./debug_tf_issue.sh "issue_name"` |
| Check robot health | `./check_robot_health.sh` |
| **Monitor ALL diagnostics** | `./monitor_robot_diagnostics.sh "session_name"` |
| Monitor memory only | `./record_robot_memory.sh &` |
| Analyze memory log | `python3 analyze_memory_log.py robot_memory_log_*.csv` |
| Replay rosbag only | `ros2 bag play ~/experiment_recordings/test_5/*/` |
| Replay debug bag | `ros2 bag play ~/rosbag_debug/issue_name_*/` |
| View CPU usage | `cat ~/robot_logs/issue_name_*/cpu_usage_*.txt` |
| View GPU usage | `cat ~/robot_logs/issue_name_*/gpu_usage_*.txt` |
| View robot logs | `less ~/robot_logs/issue_name_*/*.log` |
