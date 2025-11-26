#!/usr/bin/env python3
import rclpy
import os
from pathlib import Path
import pandas as pd
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
from std_msgs.msg import Header, Int32

import argparse
from action_msgs.msg import GoalStatus


class Nav2ClientGoalNode(Node):
    def __init__(self, use_topic_interface=False):
        super().__init__('Nav2Client')
        self.get_logger().info('Nav2Client node has been started.')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.motion_status = GoalStatus.STATUS_EXECUTING
        self.motion_result = None
        self.pose = None
        self.use_topic_interface = use_topic_interface

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos_profile=qos_profile
        )
        self.odom = None
        self.tests_directory = os.getcwd() + '/tests' #expect to be in ros directory

        # Create a topic subscriber to receive goal node requests
        if use_topic_interface:
            self.goal_node_sub = self.create_subscription(
                Int32,
                '/nav2_client_goal_node',
                self.goal_node_callback,
                10
            )
            self.get_logger().info('Topic interface enabled: Subscribe to /nav2_client_goal_node to send goals')
        else:
            self.get_logger().info('Use command line arguments or programmatic interface to send goals')

    def odom_callback(self, msg):
        self.odom = msg

    def goal_node_callback(self, msg):
        """
        Callback for receiving goal node requests via topic.
        """
        goal_node = msg.data
        self.get_logger().info(f'Received goal node request: {goal_node}')
        status, pose = self.go_to_pose(goal_node)
        self.get_logger().info(
            f'Goal {goal_node} at pose: {pose} reached {status} with nav2')

    def set_initial_pose(self):
        """
        Publish the initial pose to /initialpose.
        """
        rclpy.spin_once(self, timeout_sec=3)
        # time.sleep(3)
        if self.odom is None:
            self.get_logger().error('set_initial_pose: no odom received')
            return
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = Header()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.odom.header.stamp
        odom_pose = self.odom.pose.pose
        # Set position
        initial_pose.pose.pose.position = odom_pose.position
        initial_pose.pose.pose.orientation = odom_pose.orientation
        initial_pose.pose.covariance = [0.25      , 0.        , 0.        , 0.        , 0.        ,
       0.        , 0.        , 0.25      , 0.        , 0.        ,
       0.        , 0.        , 0.        , 0.        , 0.        ,
       0.        , 0.        , 0.        , 0.        , 0.        ,
       0.        , 0.        , 0.        , 0.        , 0.        ,
       0.        , 0.        , 0.        , 0.        , 0.        ,
       0.        , 0.        , 0.        , 0.        , 0.        ,
       0.06853892]

        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info(f'Initial pose set to x: {odom_pose.position.x}, y: {odom_pose.position.y}.')  
        self.odom = None
    
    def yaw_to_quaternion(self,yaw):
        """
        Convert yaw (radians) to quaternion (z, w).
        """
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    
    def goal_response_callback(self, future):
        """ Get regular goal_responses"""

        self.set_initial_pose()
        time.sleep(1)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal nav2 rejected :(')
            self.handle_goal_rejection()
            return False, [0,0]

        self.get_logger().info('Goal nav2 accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Get pano action result"""
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
        self.motion_status = future.result().status 
        self.motion_result = result

    def feedback_callback(self, feedback_msg):
        """ Get the motion action feedback"""
        feedback = feedback_msg.feedback
        self.pose = [feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y]
        # self.get_logger().info('motion feedback current distance to goal: {0}'.format(feedback.distance_remaining))

    def handle_goal_rejection(self):
        """ Handle goal rejection by setting motion_status and notifying send_goal() """
        self.motion_status = GoalStatus.STATUS_ABORTED
        self.motion_result = [0,0]
        self.get_logger().warn('Goal was rejected by Nav2.')


    def send_goal(self, pose):
        """
        Send a goal to the Nav2 NavigateToPose action server.
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]

        self.get_logger().info(f'Sending nav2 goal as x: {pose[0]}, y: {pose[1]}.')
        self.motion_status = GoalStatus.STATUS_EXECUTING
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future
    
    def get_possible_next_nodes_from_data(self) -> dict:
        """
        Load the current model from the most recent test file.
        Monitors both run directories and step subdirectories for the latest model.
        Returns the model if successfully loaded, None otherwise.
        """
                
        # Log run directories periodically to monitor changes (we can have this node running for several tests)
        if not hasattr(self, '_last_runs_log') or (time.time() - getattr(self, '_last_runs_log', 0)) > 10.0:
            #self.get_logger().info(f"Found run directories: {[d.name for d in run_dirs]}")
            self._last_runs_log = time.time()
            # Look for the most recent model 
            results_dir = Path(self.tests_directory)
            
            # Always log path info to monitor directory access
            # self.get_logger().debug(f"Checking for models in: {results_dir}")
            # self.get_logger().debug(f"Directory exists: {results_dir.exists()}")
            
            if not results_dir.exists():
                self.get_logger().warning("aimapp_results directory not found")
                return None
        
            # Get all numbered run directories
            self.run_dirs = [d for d in results_dir.iterdir() if d.is_dir() and d.name.isdigit()]
            #self.get_logger().info(f"run_dirs {run_dirs}")
        
        if not hasattr(self, 'run_dirs') or not self.run_dirs:
            self.get_logger().debug("No run directories found")
            return None
        
        # ALWAYS get the highest numbered run (most recent) - even if it's empty
        latest_run_dir = max(self.run_dirs, key=lambda x: int(x.name))
        self.get_logger().debug(f"Latest run directory: {latest_run_dir}")
        
        
        # Check for steps_data.csv in the run directory itself first (final model)
        main_model_file = latest_run_dir / "steps_data.csv"
        #self.get_logger().info(f"Checking main model file: {main_model_file}, exists: {main_model_file.exists()}")
        
        if not main_model_file.exists():
            self.get_logger().debug("No steps_data.csv found")
            return None
            
        csv_file = pd.read_csv(main_model_file)
        csv_file = csv_file.loc[csv_file['possible_next_nodes'].notnull()]# excluding all rows where 'possible_next_nodes' is empty
            
        return csv_file['possible_next_nodes'][-1], csv_file['possible_next_actions'][-1]
    

    def go_to_pose(self, goal_node:int)-> bool:
        """ send pose to action and wait for result (goal reached)"""        
        possible_next_nodes_poses_dict, next_actions =  self.get_possible_next_nodes_from_data()
        try:
            goal_pose = possible_next_nodes_poses_dict[goal_node]
            goal_index = possible_next_nodes_poses_dict.keys().index(goal_node)
        except:
            self.get_logger.warn(f'The desired goal node {goal_node} is not in adjacent nodes {list(possible_next_nodes_poses_dict.keys())}')
            return False, self.pose
        
        future = self.send_goal(goal_pose)
        rclpy.spin_until_future_complete(self, future)

        while self.motion_status <= 3 and self.motion_status > 0:
            rclpy.spin_once(self)
        if self.motion_status == 4:  # SUCCEEDED
            goal_reached = True
            self.get_logger().info(
                    'Goal '+ str(goal_node) + ' at pose: ' + str(goal_pose) + 'reached with nav2 with action ' + str(goal_index) )
        else:
            goal_reached = False
            self.get_logger().warn('Goal not reached with nav2')
        
        return goal_reached, self.pose
    

def main(node_goal=None, use_topic_interface=False):
    rclpy.init()

    nav_client = Nav2ClientGoalNode(use_topic_interface=use_topic_interface)

    if use_topic_interface:
        # Keep spinning to receive goal requests via topic
        nav_client.get_logger().info('Nav2 Client running in continuous mode. Send goals via topic /nav2_client_goal_node')
        nav_client.get_logger().info('Example: ros2 topic pub /nav2_client_goal_node std_msgs/msg/Int32 "data: 1"')
        try:
            rclpy.spin(nav_client)
        except KeyboardInterrupt:
            nav_client.get_logger().info('Shutting down Nav2 Client...')
    elif node_goal is not None:
        # Single goal mode (original behavior)
        status, pose = nav_client.go_to_pose(node_goal)
        nav_client.get_logger().info(
            'Goal '+ str(node_goal) + ' at pose: ' + str(pose) + 'reached '+ str(status) +' with nav2')
    else:
        nav_client.get_logger().error('No goal provided and topic interface not enabled!')

    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Reach position x,y')
    parser.add_argument('--node', type=int, default=None, help='node goal (single goal mode)')
    parser.add_argument('--continuous', action='store_true', help='Enable continuous mode using topic interface')
    args = parser.parse_args()

    if args.continuous:
        main(use_topic_interface=True)
    elif args.node is not None:
        main(node_goal=args.node)
    else:
        parser.print_help()
        print("\nError: Either provide --node <goal_id> for single goal or --continuous for continuous mode")