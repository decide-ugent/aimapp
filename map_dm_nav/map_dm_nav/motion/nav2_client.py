#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
from std_msgs.msg import Header

import argparse
from action_msgs.msg import GoalStatus


class Nav2Client(Node):
    def __init__(self):
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
            '/odom',
            self.odom_callback,
            qos_profile=qos_profile
        )
        self.odom = None


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
    

def main(x,y, t='False'):
    rclpy.init()

    if t == 'False':
        initial_x = None
        initial_y = None
    elif t == 'True':
        initial_x = 0.0
        initial_y = 0.0
    
    rclpy.spin_once(Nav2Client(), timeout_sec=3)
    nav_client = Nav2Client()

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
    args = parser.parse_args()
    main(args.x,args.y, args.t)