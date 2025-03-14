#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
import sys
# from pathlib import Path
# import os

"""
This code exists so that gazebo odometry is not considered 
Gazebo odom is perfect and considers the world coordoinates, 
while we want the agent coordinates instead (starts at 0,0 at init)

"""
class ResetTurtlebot3Odom(Node):
    def __init__(self,x,y,namespace):
        super().__init__('reset_turtlebot3_odom')

        # self.get_logger().info(str(x) +  str(y)+ str(namespace))
        self.x_offset = float(x)
        self.y_offset = float(y)
        self.topic_namespace = str(namespace)
        self.added_odom_noise = [0,0]

        # Subscriber to /odom
        self.odom_subscription = self.create_subscription(
            Odometry,
            f'{self.topic_namespace}/odom',
            self.odom_callback,
            10
        )

        self.believed_odom_sub = self.create_subscription(
            Point,
            '/believed_odom',
            self.believed_odom_callback,
            10
        )

        self.believed_odom_sub = self.create_subscription(
            Point,
            '/odom_noise',
            self.odom_noise_callback,
            10
        )

        # Publisher to namespace/odom
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

    def odom_callback(self,msg):
        # Modify the odometry message by subtracting the offsets
        self.real_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        msg.pose.pose.position.x = msg.pose.pose.position.x - self.x_offset + self.added_odom_noise[0]
        msg.pose.pose.position.y = msg.pose.pose.position.y - self.y_offset + self.added_odom_noise[1]

        # Publish the modified odometry message
        self.odom_publisher.publish(msg)

    def believed_odom_callback(self, msg):
        """ reset offset so to match believed odom"""
        m = 0
        self.get_logger().info('desired odoms %s and real odom %s'%(str([msg.x, msg.y]), str([round(self.real_odom[0]+self.added_odom_noise[0],2),round(self.real_odom[1]+self.added_odom_noise[1],2)])))
        x_offset =  (self.real_odom[0]+self.added_odom_noise[0]) - msg.x
        y_offset =  (self.real_odom[1]+self.added_odom_noise[1]) - msg.y

        self.get_logger().info('x and y offsets %f %f'%(x_offset, y_offset))
        self.get_logger().info('prev x and y offsets %f %f'%(self.x_offset, self.y_offset))
        #only replace curent offset if the difference is consequent. 
        #Else we consider odom correct thus not to induce drift meaninglessly
        if abs(x_offset - self.x_offset) > 0.4:
            self.x_offset = x_offset
            m+=1

        if abs(y_offset - self.y_offset) > 0.4:
            self.y_offset = y_offset
            m+=1
        if m> 0:
            self.get_logger().info('modified odom offsets [%f, %f]'%(self.x_offset, self.y_offset))

    def odom_noise_callback(self, msg):
        self.added_odom_noise = [msg.x, msg.y]

def main(args=None):
    rclpy.init(args=args)
    x = sys.argv[2] if len(sys.argv) > 2 else 0.0
    y = sys.argv[4] if len(sys.argv) > 4 else 0.0
    namespace = sys.argv[6] if len(sys.argv) > 6 else 'agent'
 
    reset_turtlebot3_odom = ResetTurtlebot3Odom(x,y,namespace)

    rclpy.spin(reset_turtlebot3_odom)

    reset_turtlebot3_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

