#!/usr/bin/env python3
"""
Play ROS2 bag at custom rate while printing agent logs at the same rate.
Both bag and logs play independently based on elapsed wall time.

Usage:
    python3 play_bag_with_logs.py <experiment_dir> [--rate RATE]

Example:
    python3 play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 --rate 3.0
"""

import argparse
import subprocess
import sys
import time
import re
import os
from pathlib import Path
from threading import Thread, Event
from datetime import datetime, timedelta


class LogPrinter:
    """Prints logs at a given playback rate based on wall time."""

    def __init__(self, log_file, rate=1.0):
        self.log_file = log_file
        self.rate = rate
        self.stop_event = Event()

        # Parse all log entries
        self.log_entries = self._parse_logs()
        print(f"Loaded {len(self.log_entries)} log entries from {log_file}")

        if self.log_entries:
            self.first_log_time = self.log_entries[0][0]
            self.last_log_time = self.log_entries[-1][0]
            self.log_duration = self.last_log_time - self.first_log_time
            print(f"Log duration: {self.log_duration:.2f}s ({self.log_duration/60:.1f} minutes)")
            print(f"Playback will take: {self.log_duration/self.rate:.2f}s")

    def _parse_logs(self):
        """Parse log file and extract timestamps."""
        entries = []

        if not os.path.exists(self.log_file):
            print(f"Warning: Log file not found: {self.log_file}")
            return entries

        with open(self.log_file, 'r') as f:
            for line in f:
                # Extract ROS timestamp from log line
                match = re.search(r'\[(?:INFO|WARN|ERROR|DEBUG)\] \[(\d+\.\d+)\]', line)
                if match:
                    timestamp = float(match.group(1))
                    entries.append((timestamp, line.rstrip()))

        # Sort by timestamp
        entries.sort(key=lambda x: x[0])
        return entries

    def run(self):
        """Print logs at the specified playback rate."""
        if not self.log_entries:
            print("No logs to print")
            return

        print("\n" + "="*80)
        print("AGENT LOGS (playing at {:.1f}x speed)".format(self.rate))
        print("="*80 + "\n")

        start_time = time.time()
        log_index = 0
        first_log_time = self.log_entries[0][0]

        while log_index < len(self.log_entries) and not self.stop_event.is_set():
            # Calculate how much time has elapsed in real world (scaled by rate)
            elapsed_real = time.time() - start_time
            elapsed_log = elapsed_real * self.rate

            # Current log time we should be at
            current_log_time = first_log_time + elapsed_log

            # Print all logs up to current time
            while log_index < len(self.log_entries):
                log_time, log_line = self.log_entries[log_index]

                if log_time <= current_log_time:
                    print(log_line)
                    sys.stdout.flush()
                    log_index += 1
                else:
                    break

            # Small sleep to avoid busy-waiting
            time.sleep(0.01)  # 10ms

        print("\n" + "="*80)
        print(f"LOG PLAYBACK COMPLETE - Printed {log_index}/{len(self.log_entries)} logs")
        print("="*80 + "\n")

    def stop(self):
        """Stop the log printer."""
        self.stop_event.set()


class BagPlayer:
    """Manages ROS2 bag playback."""

    def __init__(self, bag_dir, rate=1.0):
        self.bag_dir = bag_dir
        self.rate = rate
        self.process = None

    def play(self):
        """Start playing the bag."""
        cmd = [
            'ros2', 'bag', 'play',
            str(self.bag_dir),
            '--rate', str(self.rate)
        ]

        print(f"\nStarting bag playback at {self.rate}x speed...")
        print(f"Command: {' '.join(cmd)}\n")

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )

        # Print bag playback output
        for line in self.process.stdout:
            print(f"[BAG] {line.rstrip()}")

        return self.process.wait()

    def stop(self):
        """Stop the bag playback."""
        if self.process:
            self.process.terminate()
            self.process.wait()


def get_bag_info(bag_dir):
    """Extract bag start time and duration from metadata."""
    result = subprocess.run(
        ['ros2', 'bag', 'info', str(bag_dir)],
        capture_output=True,
        text=True
    )

    output = result.stdout

    # Extract start time (in seconds since epoch)
    start_match = re.search(r'Start:\s+.*\((\d+\.\d+)\)', output)
    if not start_match:
        raise ValueError("Could not extract start time from bag info")
    start_time = float(start_match.group(1))

    # Extract duration (in seconds)
    duration_match = re.search(r'Duration:\s+([\d.]+)s', output)
    if not duration_match:
        raise ValueError("Could not extract duration from bag info")
    duration = float(duration_match.group(1))

    return start_time, duration


def find_log_file(experiment_dir):
    """Find the minimal_agent_launch.log file in the experiment directory."""
    experiment_path = Path(experiment_dir)
    log_files = list(experiment_path.glob('**/minimal_agent_launch.log'))

    if not log_files:
        raise FileNotFoundError(
            f"Could not find minimal_agent_launch.log in {experiment_dir}"
        )

    if len(log_files) > 1:
        print("Warning: Multiple log files found, using the first one:")
        for lf in log_files:
            print(f"  - {lf}")
        print()

    return log_files[0]


def find_bag_dir(experiment_dir):
    """Find the ROS2 bag directory in the experiment directory."""
    experiment_path = Path(experiment_dir)
    bag_dirs = [d.parent for d in experiment_path.glob('**/metadata.yaml')]

    if not bag_dirs:
        raise FileNotFoundError(
            f"Could not find ROS2 bag (metadata.yaml) in {experiment_dir}"
        )

    # Filter to only run_* directories
    run_bags = [d for d in bag_dirs if d.name.startswith('run_')]

    if run_bags:
        bag_dir = run_bags[0]
    else:
        bag_dir = bag_dirs[0]

    if len(bag_dirs) > 1:
        print("Warning: Multiple bags found, using:")
        print(f"  {bag_dir}\n")

    return bag_dir


def main():
    parser = argparse.ArgumentParser(
        description='Play ROS2 bag with synchronized log output',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Play at 3x speed (default)
  python3 play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534

  # Play at normal speed
  python3 play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 --rate 1.0

  # Play at 20x speed
  python3 play_bag_with_logs.py ~/experiment_recordings/test_0/reach_aerial_drone_test2_20251218_144534 --rate 20.0

Controls:
  - Ctrl+C: Stop playback
  - Ctrl+Z: Pause (suspend the entire process)
  - fg: Resume from pause
        """
    )

    parser.add_argument(
        'experiment_dir',
        type=str,
        help='Path to experiment recording directory'
    )

    parser.add_argument(
        '--rate',
        type=float,
        default=3.0,
        help='Playback rate (default: 3.0 for 3x speed)'
    )

    args = parser.parse_args()

    # Validate experiment directory
    experiment_dir = Path(args.experiment_dir).expanduser()
    if not experiment_dir.exists():
        print(f"Error: Experiment directory not found: {experiment_dir}")
        sys.exit(1)

    try:
        # Find log file and bag directory
        log_file = find_log_file(experiment_dir)
        bag_dir = find_bag_dir(experiment_dir)

        print("="*80)
        print("ROS2 BAG PLAYBACK WITH SYNCHRONIZED LOGS")
        print("="*80)
        print(f"Experiment dir: {experiment_dir}")
        print(f"Log file:       {log_file}")
        print(f"Bag directory:  {bag_dir}")
        print(f"Playback rate:  {args.rate}x")
        print("="*80 + "\n")

        # Get bag information
        print("Reading bag information...")
        bag_start_time, bag_duration = get_bag_info(bag_dir)

        start_dt = datetime.fromtimestamp(bag_start_time)
        duration_td = timedelta(seconds=bag_duration)

        print(f"Bag start time: {start_dt}")
        print(f"Bag duration:   {duration_td} ({bag_duration:.1f}s)")
        print(f"At {args.rate}x speed, bag will take approximately {bag_duration/args.rate:.1f}s")
        print()

        # Create log printer and bag player
        log_printer = LogPrinter(log_file, args.rate)
        bag_player = BagPlayer(bag_dir, args.rate)

        # Start log printer in background thread
        log_thread = Thread(target=log_printer.run, daemon=True)
        log_thread.start()

        # Small delay to let logs start first (since they start before bag)
        time.sleep(0.5)

        try:
            # Play bag in main thread (blocking)
            return_code = bag_player.play()
            print(f"\nBag playback finished with return code: {return_code}")

        except KeyboardInterrupt:
            print("\n\nStopping playback...")
            bag_player.stop()
            log_printer.stop()
            print("Playback stopped.")

        # Wait for log thread to finish
        print("\nWaiting for logs to complete...")
        log_thread.join(timeout=5.0)

        print("\n" + "="*80)
        print("PLAYBACK COMPLETE")
        print("="*80)

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
