#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
from std_msgs.msg import Header
import threading

import argparse
from action_msgs.msg import GoalStatus
from aimapp_actions.msg import NavigationResult


class Nav2Client(Node):
    def __init__(self, continuous=False):
        super().__init__('Nav2Client')
        self.get_logger().info('Nav2Client node has been started.')
        # self.cli = self.create_client(PotentialField, 'potential_field')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = PotentialField.Request()
        #
        self.motion_status = GoalStatus.STATUS_EXECUTING
        self.motion_result = None
        self.pose = None
        self.continuous = continuous
        self.goal_lock = threading.Lock()

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

        # Publisher for navigation results
        self.nav_result_pub = self.create_publisher(
            NavigationResult,
            '/nav2_navigation_result',
            qos_profile
        )

        # Subscribe to pose goals if in continuous mode
        if self.continuous:
            self.pose_goal_sub = self.create_subscription(
                PoseStamped,
                '/nav2_client_goal_pose',
                self.pose_goal_callback,
                10
            )
            self.get_logger().info('Continuous mode enabled: Listening for pose goals on /nav2_client_goal_pose')

            # Store the goal pose for result publishing
            self.current_goal_pose = None
            self.navigation_in_progress = False


    def odom_callback(self, msg):
        self.odom = msg
    
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

        # Don't call set_initial_pose here - it causes blocking issues
        # self.set_initial_pose()
        # time.sleep(1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal nav2 rejected :(')
            self.handle_goal_rejection()
            return False, [0,0]

        self.get_logger().info('Goal nav2 accepted :)')
        self.motion_status = GoalStatus.STATUS_ACCEPTED  # Set to 2
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.get_logger().info('Waiting for navigation to complete...')

    def get_result_callback(self, future):
        """ Get pano action result"""
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation result received! Status: {status}')
        self.motion_status = status
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

    def pose_goal_callback(self, msg):
        """
        Callback when a new pose goal is received (continuous mode only).
        Sends the pose to Nav2 and waits for result in a separate thread.
        """
        pose = [msg.pose.position.x, msg.pose.position.y]

        self.get_logger().info(f'Received pose goal: x={pose[0]:.2f}, y={pose[1]:.2f}')

        # Check if navigation is already in progress
        if self.navigation_in_progress:
            self.get_logger().warn(f'Navigation already in progress, ignoring new goal: {pose}')
            return

        # Store the goal pose for result publishing
        self.current_goal_pose = pose.copy()
        self.navigation_in_progress = True

        # Start navigation in a separate thread to avoid blocking
        nav_thread = threading.Thread(target=self.navigate_and_publish_result, args=(pose,))
        nav_thread.daemon = True
        nav_thread.start()

    def navigate_and_publish_result(self, goal_pose):
        """
        Navigate to the goal and publish the result when complete.
        This runs in a separate thread.
        """
        try:
            self.get_logger().info(f'Starting navigation to x={goal_pose[0]:.2f}, y={goal_pose[1]:.2f}')

            # Send goal asynchronously (don't use go_to_pose as it blocks)
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('NavigateToPose action server not available!')
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_pose[0]
            goal_msg.pose.pose.position.y = goal_pose[1]

            self.get_logger().info(f'Sending nav2 goal as x: {goal_pose[0]}, y: {goal_pose[1]}.')
            self.motion_status = GoalStatus.STATUS_EXECUTING
            
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)

            # Wait for goal to be accepted first
            self.get_logger().info('Waiting for goal to be accepted by Nav2...')
            goal_timeout = 10.0
            goal_start = time.time()
            while send_goal_future.done() is False:
                if time.time() - goal_start > goal_timeout:
                    self.get_logger().error('Timeout waiting for goal acceptance')
                    return
                time.sleep(0.1)

            self.get_logger().info(f'Goal accepted, status after acceptance: {self.motion_status}')

            # Wait a bit more for the result future to be set up
            time.sleep(0.5)

            # Check if we have a result future
            if not hasattr(self, '_get_result_future') or self._get_result_future is None:
                self.get_logger().error('Result future was not set up properly!')
                return

            self.get_logger().info('Result future is set up, monitoring for completion...')

            # Wait for the navigation to complete by polling motion_status
            # The callbacks will update motion_status when navigation completes
            self.get_logger().info(f'Starting to monitor navigation status (current: {self.motion_status})')
            timeout = 120.0  # 2 minutes timeout
            start_time = time.time()

            while self.motion_status <= 3 and self.motion_status > 0:
                # Also check if result future is done
                if hasattr(self, '_get_result_future') and self._get_result_future.done():
                    self.get_logger().info('Result future completed, checking status...')
                    break

                if time.time() - start_time > timeout:
                    self.get_logger().warn('Navigation timeout reached')
                    break
                time.sleep(0.1)
                if (time.time() - start_time) % 5.0 < 0.1:  # Log every 5 seconds
                    self.get_logger().info(f'Still navigating... status: {self.motion_status}')

            # Force check the result future one more time
            if hasattr(self, '_get_result_future') and self._get_result_future.done():
                try:
                    result = self._get_result_future.result()
                    self.motion_status = result.status
                    self.get_logger().info(f'Manually retrieved result status: {self.motion_status}')
                except Exception as e:
                    self.get_logger().error(f'Error retrieving result: {e}')

            # Determine if goal was reached
            if self.motion_status == 4:  # SUCCEEDED
                goal_reached = True
                self.get_logger().info(f'Goal {goal_pose} reached with nav2')
            else:
                goal_reached = False
                self.get_logger().warn(f'Goal not reached with nav2, status: {self.motion_status}')

            # Get final pose from feedback
            final_pose = self.pose if self.pose is not None else [0.0, 0.0]

            # Publish navigation result
            result_msg = NavigationResult()
            result_msg.goal_reached = goal_reached
            result_msg.final_pose_x = float(final_pose[0])
            result_msg.final_pose_y = float(final_pose[1])
            result_msg.goal_pose_x = float(goal_pose[0])
            result_msg.goal_pose_y = float(goal_pose[1])

            self.nav_result_pub.publish(result_msg)
            self.get_logger().info(f'Published navigation result #{i+1}: goal_reached={goal_reached}, '
                                    f'final_pose=[{final_pose[0]:.2f}, {final_pose[1]:.2f}]')
            time.sleep(0.1)

            self.get_logger().info(f'Navigation complete: goal_reached={goal_reached}')

        finally:
            # Always reset the flag when done (even if there was an error)
            self.navigation_in_progress = False
            self.get_logger().info('Navigation thread completed, ready for new goals')


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
    

    def go_to_pose(self, goal_pose:list)-> bool:
        """ send pose to action and wait for result (goal reached)"""
        future = self.send_goal(goal_pose)
        rclpy.spin_until_future_complete(self, future)

        while self.motion_status <= 3 and self.motion_status > 0:
            rclpy.spin_once(self)
        if self.motion_status == 4:  # SUCCEEDED
            goal_reached = True
            self.get_logger().info(
                    'Goal'+ str(goal_pose) +'reached with nav2')
        else:
            goal_reached = False
            self.get_logger().warn('Goal not reached with nav2')
        
        return goal_reached, self.pose
    

def main(x=None, y=None, t='False', continuous=False):
    rclpy.init()

    if continuous:
        # Continuous mode: listen for pose goals on topic
        nav_client = Nav2Client(continuous=True)
        nav_client.get_logger().info('Running in continuous mode - waiting for pose goals on /nav2_client_goal_pose')

        try:
            rclpy.spin(nav_client)
        except KeyboardInterrupt:
            nav_client.get_logger().info('Shutting down Nav2 Client')
        finally:
            nav_client.destroy_node()
            rclpy.shutdown()
    else:
        # Single goal mode: send one goal and exit
        if t == 'False':
            initial_x = None
            initial_y = None
        elif t == 'True':
            initial_x = 0.0
            initial_y = 0.0

        rclpy.spin_once(Nav2Client(), timeout_sec=3)
        nav_client = Nav2Client(continuous=False)

        pose = [x,y]

        status, pose = nav_client.go_to_pose(pose)

        nav_client.get_logger().info(
                        'Goal'+ str(pose) +'reached '+ str(status) +' with nav2')

        # %s' %  str(response.status))

        nav_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Reach position x,y')
    parser.add_argument('--x', type=float, default=0.0,  help='x goal value')
    parser.add_argument('--y', type=float, default=0.0,  help='y goal value')
    parser.add_argument('--t', type=str, default='False',  help='init pose setup')
    parser.add_argument('-continuous', '--continuous', action='store_true', help='Run in continuous mode listening for pose goals')
    args = parser.parse_args()
    main(args.x, args.y, args.t, args.continuous)