# How to Replay Experiments with Synchronized Logs

This guide shows how to replay recorded ROS2 bag experiments while viewing the agent's logs synchronized to the same timeline.

## Quick Start

```bash
# Navigate to your workspace
cd ~/workspace/ros_ws/src/aimapp

# Play an experiment at 3x speed (default)
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534
```

## What You'll See

The script will:
1. **Find your data**: Automatically locate the ROS2 bag and agent logs in the experiment folder
2. **Display info**: Show bag start time, duration, and expected playback time
3. **Play synchronized content**:
   - ROS2 topics (visible in RViz if open)
   - Agent logs printed to terminal at the correct moments

### Example Output

```
================================================================================
ROS2 BAG PLAYBACK WITH SYNCHRONIZED LOGS
================================================================================
Experiment dir: /home/idlab332/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534
Log file:       .../20251218_124855/minimal_agent_launch.log
Bag directory:  .../run_20251218_144534
Playback rate:  3.0x
================================================================================

Reading bag information...
Bag start time: 2025-12-18 14:45:36.042034
Bag duration:   0:31:12.583370 (1872.6s)
Playback will take approximately 624.2s

Loaded 342 log entries from .../minimal_agent_launch.log

================================================================================
AGENT LOGS (synchronized with bag playback)
================================================================================

[BAG] Starting playback...
[INFO] [1766059290.886130144] [generate_panorama_from_ricoh_camera]: Node generate_panorama initialised
[BAG] Playing at 3.0x rate
[INFO] [1766059298.910832128] [panorama_client]: panorama client node has been started.
[INFO] [1766059298.940965184] [aif_process]: Loading model from /home/husarion/ros2_ws/tests/0
[INFO] [1766059335.687483552] [aif_process]: possible action 0, next node 1 and pose [1.69, 0.33]
... (logs appear line-by-line as the bag time advances)
```

**Key behavior**: Logs appear **incrementally** as the bag plays, synchronized to the bag's `/clock` topic. Each log line appears at the correct moment relative to when it originally occurred during recording.

## Usage Examples

### Replay at Different Speeds

```bash
# Normal speed (1x) - takes full duration
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 1.0

# Fast review (3x) - default, good for quick review
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 3.0

# Very fast (5x) - for overview only
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 5.0
```

### With RViz Visualization

```bash
# Terminal 1: Start RViz with your config
ros2 run rviz2 rviz2 -d ~/workspace/ros_ws/src/aimapp/aimapp/rviz/nav2_default_view.rviz

# Terminal 2: Play the bag with logs
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534
```

You'll see the robot moving in RViz while the agent's decision logs appear in the terminal.

## Playback Controls

### Pause Playback
```bash
# Press Ctrl+Z to pause both the bag and logs
^Z
[1]+  Stopped    ./aimapp/tests/play_bag_with_logs.sh ...
```

### Resume Playback
```bash
# Type 'fg' and press Enter to resume
fg
# Playback continues from where it paused
```

### Stop Playback
```bash
# Press Ctrl+C to stop completely
^C
Stopping playback...
Playback stopped.
```

## Finding Your Experiments

Experiments are stored in `~/experiment_recordings/`:

```bash
# List all test runs
ls -l ~/experiment_recordings/

# Find a specific test
ls -l ~/experiment_recordings/test_0/

# List all experiments for test 0
ls -l ~/experiment_recordings/test_0/
```

Example structure:
```
~/experiment_recordings/
└── test_0/
    ├── reach_aerial_drone_test1_20251218_130158/
    │   ├── run_20251218_130158/          # ROS2 bag
    │   │   ├── run_20251218_130158_0.db3
    │   │   └── metadata.yaml
    │   ├── 20251218_124855/              # Agent logs folder
    │   │   └── minimal_agent_launch.log
    │   └── 3/                            # Test data (steps, model, etc.)
    └── reach_aerial_drone_test2_20251218_144534/
        └── ...
```

## Tips

1. **Start RViz first** if you want to see the robot's motion and sensor data
2. **Use 3x speed** as a default for reviewing - fast enough to save time but slow enough to understand
3. **Pause with Ctrl+Z** when you see something interesting, then resume with `fg`
4. **Filter logs** by piping to grep if you want specific info:
   ```bash
   ./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/*/  | grep "MCTS"
   ```

## Troubleshooting

### "ros2 command not found"
```bash
# Source your ROS2 workspace
source ~/workspace/ros_ws/install/setup.bash
```

### "Experiment directory not found"
Make sure you're pointing to the full experiment directory, not just the test folder:
```bash
# Wrong
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0

# Correct
./aimapp/tests/play_bag_with_logs.sh ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534
```

### "Could not find minimal_agent_launch.log"
Check that your experiment was recorded with the agent running. The log should be in a timestamped subfolder:
```bash
find ~/experiment_recordings/test_0 -name "minimal_agent_launch.log"
```

### Bag and logs not synchronized
Make sure you're using the `--clock` option (the script does this automatically). If using RViz, make sure it's set to use simulation time.

## Python Script Usage

If you prefer using the Python script directly:

```bash
# Basic usage
python3 ./aimapp/tests/play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534

# With custom rate
python3 ./aimapp/tests/play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 --rate 5.0

# Help
python3 ./aimapp/tests/play_bag_with_logs.py --help
```

---

For more information, see [DEBUG_SCRIPTS_README.md](DEBUG_SCRIPTS_README.md)
