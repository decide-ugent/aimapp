#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header
import numpy as np
import time
from typing import Dict, List, Tuple, Optional
import pickle
import os
from pathlib import Path
# Import the existing pickle functions from the visualization tools
from map_dm_nav.visualisation_tools import pickle_load_model, get_save_data_dir


class NodeVisualizer(Node):
    """
    ROS2 node that visualizes the discovered nodes/states from the high-level navigation algorithm in RViz2.
    
    This node reads the model's agent_state_mapping and publishes visualization markers showing:
    - Node positions as spheres (colored by observation type)
    - State numbers as text labels
    - Connections between nodes (transitions from B matrix)
    - Current robot position
    """

    def __init__(self):
        super().__init__('node_visualizer')
        
        # QoS profile for reliable delivery
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers
        self.nodes_pub = self.create_publisher(
            MarkerArray,
            '/visitable_nodes',
            qos_profile
        )
        
        self.connections_pub = self.create_publisher(
            MarkerArray,
            '/node_connections', 
            qos_profile
        )
        
        self.robot_pose_pub = self.create_publisher(
            Marker,
            '/robot_current_node',
            qos_profile
        )
        
        # Timer for periodic updates (check for new files every 1 second for responsiveness)
        self.update_timer = self.create_timer(1.0, self.update_visualization)
        
        # Add a simple heartbeat counter to verify timer is working
        self.timer_calls = 0
        
        # Model data storage
        self.current_model = None
        self.tests_directory = get_save_data_dir("/home/husarion/ros2_ws")
        self.last_model_timestamp = 0
        
        # Visualization parameters
        self.node_scale = 0.3  # Size of node spheres
        self.text_scale = 0.2  # Size of text labels
        self.connection_width = 0.05  # Width of connection lines
        self.z_height = 0.1  # Height above ground for markers
        
        # Color mapping for different observation types (40 observations max)
        self.color_map = {
                    -1: ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),  # Unknown/Ghost nodes - Gray
                    0: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),   # Observation 0 - Red
                    1: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),   # Observation 1 - Green
                    2: ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Observation 2 - Blue
                    3: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),   # Observation 3 - Yellow
                    4: ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),   # Observation 4 - Magenta
                    5: ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8),   # Observation 5 - Cyan
                    6: ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),   # Observation 6 - Orange
                    7: ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.8),   # Observation 7 - Purple
                    8: ColorRGBA(r=0.0, g=0.5, b=0.5, a=0.8),   # Observation 8 - Teal
                    9: ColorRGBA(r=0.7, g=0.1, b=0.1, a=0.8),   # Observation 9 - Dark Red
                    10: ColorRGBA(r=0.1, g=0.7, b=0.1, a=0.8),  # Observation 10 - Dark Green
                    11: ColorRGBA(r=0.1, g=0.1, b=0.7, a=0.8),  # Observation 11 - Dark Blue
                    12: ColorRGBA(r=0.8, g=0.8, b=0.1, a=0.8),  # Observation 12 - Olive
                    13: ColorRGBA(r=0.8, g=0.1, b=0.8, a=0.8),  # Observation 13 - Dark Magenta
                    14: ColorRGBA(r=0.1, g=0.8, b=0.8, a=0.8),  # Observation 14 - Dark Cyan
                    15: ColorRGBA(r=1.0, g=0.6, b=0.2, a=0.8),  # Observation 15 - Amber
                    16: ColorRGBA(r=0.6, g=0.2, b=1.0, a=0.8),  # Observation 16 - Violet
                    17: ColorRGBA(r=0.2, g=1.0, b=0.6, a=0.8),  # Observation 17 - Sea Green
                    18: ColorRGBA(r=0.9, g=0.4, b=0.4, a=0.8),  # Observation 18 - Salmon
                    19: ColorRGBA(r=0.4, g=0.9, b=0.4, a=0.8),  # Observation 19 - Lime
                    20: ColorRGBA(r=0.4, g=0.4, b=0.9, a=0.8),  # Observation 20 - Royal Blue
                    21: ColorRGBA(r=0.9, g=0.9, b=0.4, a=0.8),  # Observation 21 - Pastel Yellow
                    22: ColorRGBA(r=0.9, g=0.4, b=0.9, a=0.8),  # Observation 22 - Orchid
                    23: ColorRGBA(r=0.4, g=0.9, b=0.9, a=0.8),  # Observation 23 - Turquoise
                    24: ColorRGBA(r=0.7, g=0.3, b=0.3, a=0.8),  # Observation 24 - Rust
                    25: ColorRGBA(r=0.3, g=0.7, b=0.3, a=0.8),  # Observation 25 - Medium Green
                    26: ColorRGBA(r=0.3, g=0.3, b=0.7, a=0.8),  # Observation 26 - Navy Blue
                    27: ColorRGBA(r=0.8, g=0.7, b=0.3, a=0.8),  # Observation 27 - Gold
                    28: ColorRGBA(r=0.8, g=0.3, b=0.7, a=0.8),  # Observation 28 - Fuchsia
                    29: ColorRGBA(r=0.3, g=0.8, b=0.7, a=0.8),  # Observation 29 - Aquamarine
                    30: ColorRGBA(r=0.6, g=0.2, b=0.2, a=0.8),  # Observation 30 - Deep Red
                    31: ColorRGBA(r=0.2, g=0.6, b=0.2, a=0.8),  # Observation 31 - Deep Green
                    32: ColorRGBA(r=0.2, g=0.2, b=0.6, a=0.8),  # Observation 32 - Deep Blue
                    33: ColorRGBA(r=0.7, g=0.6, b=0.2, a=0.8),  # Observation 33 - Mustard
                    34: ColorRGBA(r=0.7, g=0.2, b=0.6, a=0.8),  # Observation 34 - Plum
                    35: ColorRGBA(r=0.2, g=0.7, b=0.6, a=0.8),  # Observation 35 - Seafoam
                    36: ColorRGBA(r=0.9, g=0.5, b=0.5, a=0.8),  # Observation 36 - Light Coral
                    37: ColorRGBA(r=0.5, g=0.9, b=0.5, a=0.8),  # Observation 37 - Mint
                    38: ColorRGBA(r=0.5, g=0.5, b=0.9, a=0.8),  # Observation 38 - Periwinkle
                    39: ColorRGBA(r=0.9, g=0.9, b=0.5, a=0.8),  # Observation 39 - Pastel Green
                    40: ColorRGBA(r=0.9, g=0.5, b=0.9, a=0.8),  # Observation 40 - Lavender
                    41: ColorRGBA(r=0.5, g=0.9, b=0.9, a=0.8),  # Observation 41 - Sky Blue
                    42: ColorRGBA(r=0.8, g=0.4, b=0.0, a=0.8),  # Observation 42 - Burnt Orange
                    43: ColorRGBA(r=0.0, g=0.4, b=0.8, a=0.8),  # Observation 43 - Cobalt Blue
                    44: ColorRGBA(r=0.8, g=0.0, b=0.4, a=0.8),  # Observation 44 - Raspberry
                    45: ColorRGBA(r=0.0, g=0.8, b=0.4, a=0.8),  # Observation 45 - Spring Green
                }
        
        # Default color for unknown observation types
        self.default_color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8)
        
        self.get_logger().info('=== Node Visualizer STARTING ===')
        self.get_logger().info('Published topics:')
        self.get_logger().info('  - /visitable_nodes (MarkerArray): Node positions and states')
        self.get_logger().info('  - /node_connections (MarkerArray): Connections between nodes')
        self.get_logger().info('  - /robot_current_node (Marker): Current robot position')
        self.get_logger().info('Add these topics to RViz2 to visualize the discovered nodes!')
        self.get_logger().info('Timer set to check for new files every 1 second')
        self.get_logger().info('=== Node Visualizer INITIALIZED ===')
        
        # Timer will start automatically when rclpy.spin() is called
        self.get_logger().info('Timer will start when node begins spinning...')

    def load_current_model(self) -> Optional[any]:
        """
        Load the current model from the most recent pickle file.
        Monitors both run directories and step subdirectories for the latest model.
        Returns the model if successfully loaded, None otherwise.
        """
        try:
            # Look for the most recent model pickle file
            # In Docker container: NodeVisualizer is in /home/ros2_ws/src/high_level_nav_warehouse/high_level_nav_warehouse/high_level_nav_warehouse/obs_transf/
            # high_nav_results is in: /home/ros2_ws/src/high_level_nav_warehouse/
            results_dir = Path(self.tests_directory)
            
            # Always log path info to monitor directory access
            self.get_logger().debug(f"Checking for models in: {results_dir}")
            self.get_logger().debug(f"Directory exists: {results_dir.exists()}")
            
            if not results_dir.exists():
                self.get_logger().warning("high_nav_results directory not found")
                return None
            
            # Get all numbered run directories
            run_dirs = [d for d in results_dir.iterdir() if d.is_dir() and d.name.isdigit()]
            
            # Log run directories periodically to monitor changes
            if not hasattr(self, '_last_runs_log') or (time.time() - getattr(self, '_last_runs_log', 0)) > 10.0:
                self.get_logger().info(f"Found run directories: {[d.name for d in run_dirs]}")
                self._last_runs_log = time.time()
            
            if not run_dirs:
                self.get_logger().debug("No run directories found")
                return None
            
            # ALWAYS get the highest numbered run (most recent) - even if it's empty
            latest_run_dir = max(run_dirs, key=lambda x: int(x.name))
            self.get_logger().debug(f"Latest run directory: {latest_run_dir}")
            
            # ONLY check the latest run directory - never use older runs
            model_file = None
            latest_time = 0
            found_model = False
            
            # Check for model.pkl in the run directory itself first (final model)
            main_model_file = latest_run_dir / "model.pkl"
            self.get_logger().debug(f"Checking main model file: {main_model_file}, exists: {main_model_file.exists()}")
            
            if main_model_file.exists():
                mtime = main_model_file.stat().st_mtime
                if mtime > latest_time:
                    latest_time = mtime
                    model_file = str(latest_run_dir)
                    found_model = True
                    self.get_logger().debug(f"Found main model file with timestamp: {mtime}")
            
            # Check ALL step subdirectories for ANY model.pkl files (live updates)
            
            # Always scan for all directories in the run directory
            all_dirs = [d for d in latest_run_dir.iterdir() if d.is_dir()]
            step_dirs = [d for d in latest_run_dir.iterdir() if d.is_dir() and d.name.startswith("step_")]
            
            # Always check for changes in step directories
            current_step_count = len(step_dirs)
            current_step_names = set([d.name for d in step_dirs])
            
            if not hasattr(self, '_last_step_count'):
                self._last_step_count = -1
            if not hasattr(self, '_last_step_names'):
                self._last_step_names = set()
            
            # Check if step directories changed (count OR actual directory names)
            step_dirs_changed = (current_step_count != self._last_step_count or 
                               current_step_names != self._last_step_names)
            
            # Always log when step directories change, plus periodic updates
            if step_dirs_changed or not hasattr(self, '_last_step_log') or (time.time() - getattr(self, '_last_step_log', 0)) > 5.0:
                self.get_logger().info(f"All directories in run {latest_run_dir.name}: {[d.name for d in all_dirs]}")
                self.get_logger().info(f"Found step directories in run {latest_run_dir.name}: {[d.name for d in step_dirs]}")
                if step_dirs_changed:
                    self.get_logger().info(f"Step directories changed!")
                    self.get_logger().info(f"  Count: {self._last_step_count} -> {current_step_count}")
                    self.get_logger().info(f"  Names: {sorted(self._last_step_names)} -> {sorted(current_step_names)}")
                
                self._last_step_log = time.time()
                self._last_step_count = current_step_count
                self._last_step_names = current_step_names
            
            if step_dirs:
                # Sort by step number to get them in order
                step_dirs.sort(key=lambda x: int(x.name.split('_')[1]) if x.name.split('_')[1].isdigit() else -1)
                
                # Find the MOST RECENT step file that exists (highest timestamp)
                for step_dir in step_dirs:
                    step_model_file = step_dir / "model.pkl"
                    
                    # Always check each step file - no logging limits
                    self.get_logger().debug(f"Checking step file: {step_model_file}")
                    self.get_logger().debug(f"  Exists: {step_model_file.exists()}")
                    if step_model_file.exists():
                        self.get_logger().debug(f"  Timestamp: {step_model_file.stat().st_mtime}")
                    
                    if step_model_file.exists():
                        mtime = step_model_file.stat().st_mtime
                        if mtime > latest_time:
                            latest_time = mtime
                            model_file = str(step_dir)
                            found_model = True
                        self.get_logger().debug(f"Step {step_dir.name} model file: exists, timestamp: {mtime}")
                    else:
                        self.get_logger().debug(f"Step {step_dir.name} model file: does not exist yet")
            
            # If no model found in latest run, that's OK - we're waiting for new files
            if not found_model:
                self.get_logger().debug(f"No model files found in latest run {latest_run_dir.name} yet - waiting for new steps...")
                return None
            
            # Load the model if we found a new or updated file
            if model_file and latest_time > self.last_model_timestamp:
                self.get_logger().info(f"Loading model from: {model_file}")
                # Use the existing pickle_load_model function
                model = pickle_load_model(model_file)
                self.last_model_timestamp = latest_time
                
                # Log info about the current run and step
                if "step_" in model_file:
                    run_num = latest_run_dir.name
                    step_name = Path(model_file).name
                    self.get_logger().info(f"Visualizing Run {run_num}, {step_name}")
                else:
                    run_num = latest_run_dir.name
                    self.get_logger().info(f"Visualizing Run {run_num}")
                    
                return model
            elif model_file:
                self.get_logger().debug(f"Model file found but not newer than cached version: {model_file}")
            else:
                self.get_logger().debug("No model files found")
            
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            
        return None

    def get_color_for_observation(self, obs_id: int) -> ColorRGBA:
        """Get color for a specific observation ID."""
        return self.color_map.get(obs_id, self.default_color)

    def create_node_markers(self, agent_state_mapping: Dict, frame_id: str = "odom") -> MarkerArray:
        """
        Create marker array for visualizing nodes.
        
        Args:
            agent_state_mapping: Dictionary mapping poses to state info
            frame_id: Frame for the markers
            
        Returns:
            MarkerArray containing node visualization markers
        """
        marker_array = MarkerArray()
        
        # Clear previous markers first
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        marker_id = 0
        
        for (x, y), data in agent_state_mapping.items():
            state = data.get('state', -1)
            obs_id = data.get('ob', -1)
            
            # Create sphere marker for the node
            node_marker = Marker()
            node_marker.header.frame_id = frame_id
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.ns = "nodes"
            node_marker.id = marker_id
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            
            # Position
            node_marker.pose.position.x = float(x)
            node_marker.pose.position.y = float(y)
            node_marker.pose.position.z = self.z_height
            node_marker.pose.orientation.w = 1.0
            
            # Scale
            node_marker.scale = Vector3(x=self.node_scale, y=self.node_scale, z=self.node_scale)
            
            # Color based on observation type
            node_marker.color = self.get_color_for_observation(obs_id)
            
            marker_array.markers.append(node_marker)
            marker_id += 1
            
            # Create text marker for state number
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "state_labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position slightly above the node
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = self.z_height + 0.2
            text_marker.pose.orientation.w = 1.0
            
            # Scale
            text_marker.scale.z = self.text_scale
            
            # Color (white text)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            # Text content
            text_marker.text = f"S{state}\nO{obs_id}"
            
            marker_array.markers.append(text_marker)
            marker_id += 1
        
        return marker_array

    def create_connection_markers(self, agent_state_mapping: Dict, B_matrix: np.ndarray, 
                                 possible_directions: Dict, pose_dist: float, 
                                 frame_id: str = "odom") -> MarkerArray:
        """
        Create marker array for visualizing connections between nodes.
        
        Args:
            agent_state_mapping: Dictionary mapping poses to state info
            B_matrix: Transition matrix
            possible_directions: Dictionary of possible actions/directions
            pose_dist: Distance between poses
            frame_id: Frame for the markers
            
        Returns:
            MarkerArray containing connection visualization markers
        """
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        marker_id = 0
        
        # Create direction mapping
        direction_mapping = {}
        for angle_str, action_id in possible_directions.items():
            if angle_str == 'STAY':
                direction_mapping[action_id] = (0, 0)
            else:
                # Convert angle to motion vector
                angle_deg = float(angle_str)
                angle_rad = np.deg2rad(angle_deg)
                dx = pose_dist * np.cos(angle_rad)
                dy = pose_dist * np.sin(angle_rad)
                direction_mapping[action_id] = (dx, dy)
        
        # Draw connections based on B matrix
        for (x, y), data in agent_state_mapping.items():
            state = data.get('state', -1)
            
            for action_id, (dx, dy) in direction_mapping.items():
                next_x, next_y = x + dx, y + dy
                
                if (next_x, next_y) in agent_state_mapping:
                    next_state = agent_state_mapping[(next_x, next_y)].get('state', -1)
                    
                    # Check if there's a significant transition probability
                    if (state >= 0 and next_state >= 0 and 
                        state < B_matrix.shape[1] and next_state < B_matrix.shape[0] and
                        action_id < B_matrix.shape[2]):
                        
                        transition_prob = B_matrix[next_state, state, action_id]
                        
                        if transition_prob > 0.1:  # Only show significant connections
                            # Create line marker for connection
                            line_marker = Marker()
                            line_marker.header.frame_id = frame_id
                            line_marker.header.stamp = self.get_clock().now().to_msg()
                            line_marker.ns = "connections"
                            line_marker.id = marker_id
                            line_marker.type = Marker.LINE_STRIP
                            line_marker.action = Marker.ADD
                            
                            # Line points
                            start_point = Point(x=float(x), y=float(y), z=self.z_height)
                            end_point = Point(x=float(next_x), y=float(next_y), z=self.z_height)
                            line_marker.points = [start_point, end_point]
                            
                            # Scale (line width proportional to transition probability)
                            line_width = max(0.02, transition_prob * self.connection_width)
                            line_marker.scale.x = line_width
                            
                            # Color (intensity based on transition probability)
                            intensity = min(1.0, transition_prob * 2)
                            line_marker.color = ColorRGBA(r=intensity, g=intensity, b=0.0, a=0.7)
                            
                            marker_array.markers.append(line_marker)
                            marker_id += 1
        
        return marker_array

    def create_robot_pose_marker(self, robot_pose: Tuple[float, float], frame_id: str = "odom") -> Marker:
        """
        Create marker for current robot position.
        
        Args:
            robot_pose: Current robot pose (x, y)
            frame_id: Frame for the marker
            
        Returns:
            Marker for robot position
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_pose"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(robot_pose[0])
        marker.pose.position.y = float(robot_pose[1])
        marker.pose.position.z = self.z_height
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale = Vector3(x=0.4, y=0.4, z=0.2)
        
        # Color (bright orange for visibility)
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
        
        return marker

    def update_visualization(self):
        """
        Main update loop - loads model from pickle files and publishes visualization markers.
        """
        try:
            # Increment timer call counter and debug message to show the method is being called
            self.timer_calls += 1
            
            
            # Load the current model from pickle files
            model = self.load_current_model()
            if model is None:
                # Always log this on the first few attempts to help debugging
                if not hasattr(self, '_debug_attempts'):
                    self._debug_attempts = 0
                self._debug_attempts += 1
                
                if self._debug_attempts <= 3 or \
                   (not hasattr(self, '_last_no_model_log') or \
                    (time.time() - getattr(self, '_last_no_model_log', 0)) > 10.0):
                    self.get_logger().info("Waiting for new model data in the latest run directory...")
                    self._last_no_model_log = time.time()
                return
                
            try:
                # Get agent state mapping (discovered nodes)
                agent_state_mapping = model.get_agent_state_mapping()
                if not agent_state_mapping:
                    self.get_logger().debug("No nodes discovered yet")
                    return
                
                # Get model parameters for connections and robot pose
                B_matrix = model.get_B()
                possible_directions = model.get_possible_directions()
                pose_dist = model.get_pose_dist()
                
                # Get current robot pose if available
                current_pose = None
                if hasattr(model, 'current_pose') and model.current_pose is not None:
                    current_pose = model.current_pose
                    
                # Log model info occasionally
                num_nodes = len(agent_state_mapping)
                if not hasattr(self, '_last_model_log') or \
                   (time.time() - getattr(self, '_last_model_log', 0)) > 2.0:
                    self.get_logger().debug(f"Model loaded with {num_nodes} nodes, B_matrix shape: {B_matrix.shape if B_matrix is not None else 'None'}")
                    self._last_model_log = time.time()
                
            except Exception as e:
                self.get_logger().error(f"Error extracting data from model: {e}")
                return
            
            # Create and publish node markers
            try:
                node_markers = self.create_node_markers(agent_state_mapping)
                self.nodes_pub.publish(node_markers)
                self.get_logger().debug(f"Published {len(node_markers.markers)} node markers")
                
                # Create and publish connection markers if we have B matrix data
                if B_matrix is not None and possible_directions is not None and pose_dist is not None:
                    connection_markers = self.create_connection_markers(
                        agent_state_mapping, B_matrix, possible_directions, pose_dist
                    )
                    self.connections_pub.publish(connection_markers)
                    self.get_logger().debug(f"Published {len(connection_markers.markers)} connection markers")
                
                # Create and publish robot pose marker
                if current_pose is not None:
                    robot_marker = self.create_robot_pose_marker(current_pose)
                    self.robot_pose_pub.publish(robot_marker)
                    self.get_logger().debug(f"Published robot pose marker at {current_pose}")
                
                # Log status periodically
                num_nodes = len(agent_state_mapping)
                if hasattr(self, '_last_log_time'):
                    if time.time() - self._last_log_time > 5.0:  # Log every 5 seconds
                        self.get_logger().info(f"Visualizing {num_nodes} discovered nodes")
                        self._last_log_time = time.time()
                else:
                    self.get_logger().info(f"Visualizing {num_nodes} discovered nodes")
                    self._last_log_time = time.time()
                    
            except Exception as e:
                self.get_logger().error(f"Error updating visualization: {e}")
                import traceback
                self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            
        except Exception as e:
            # Catch any exceptions that could break the timer
            self.get_logger().error(f"CRITICAL ERROR in update_visualization: {e}")
            import traceback
            self.get_logger().error(f"Full traceback: {traceback.format_exc()}")
            # Don't re-raise the exception - let the timer continue


def main(args=None):
    rclpy.init(args=args)
    
    node_visualizer = NodeVisualizer()
    
    try:
        rclpy.spin(node_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        node_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
