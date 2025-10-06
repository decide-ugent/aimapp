#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sys
import pandas as pd


class PathPublisher(Node):
    def __init__(self,odom_folder):
        super().__init__('path_publisher')
               
        self.path_pub = self.create_publisher(
            Path,
            '/followed_path',
            10
        )
        self.get_logger().info('odom_folder '+odom_folder)
        csv_file = pd.read_csv(odom_folder)
        husarion_odom = csv_file['husarion_odom']

        self.from_odom_to_pose(husarion_odom)

    def from_odom_to_pose(self,husarion_odom):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in husarion_odom:
            pose = eval(pose)            
            posestamp = PoseStamped()
            posestamp.pose.position.x = float(pose[0])
            posestamp.pose.position.y = float(pose[1])
            posestamp.pose.orientation.w = 1.0
            path.poses.append(posestamp)

        self.path_pub.publish(path)
        self.get_logger().info('published path once')
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        self.path_pub.publish(path)
        



def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run aimapp path_publisher <path_to_csv_file> ")
        return
    odom_folder = sys.argv[2] if len(sys.argv) > 2 else ''

    path_pub = PathPublisher(odom_folder)

    try:
        rclpy.spin(path_pub)
    except KeyboardInterrupt:
        pass
    finally:
        path_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    
    main()