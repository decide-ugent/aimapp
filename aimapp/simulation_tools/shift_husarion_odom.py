#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys

class OdomShiftNode(Node):
    def __init__(self, shift_x: float, shift_y: float):
        super().__init__('odom_shift_node')
        self.shift = [shift_x, shift_y]

        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            '/shifted_odom',
            self.shift_callback,
            10
        )
        self.sensor_odom = [0.0, 0.0]  # Initialize sensor odometry position
        self.pub = self.create_publisher(Odometry, '/odometry/shifted', 10)

        self.pub_ekf = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 10) #This is EKF reset

        self.get_logger().info(f'OdomShiftNode initialized with shift: x={shift_x}, y={shift_y}')

    def odom_callback(self, msg: Odometry):
        shifted_msg = Odometry()
        shifted_msg.header = msg.header
        shifted_msg.child_frame_id = msg.child_frame_id
        shifted_msg.pose = msg.pose
        shifted_msg.twist = msg.twist

        self.sensor_odom= [shifted_msg.pose.pose.position.x , shifted_msg.pose.pose.position.y]

        # Only reset EKF if there's an actual shift to apply
        needs_reset = (self.shift[0] != 0.0 or self.shift[1] != 0.0)

        shifted_msg.pose.pose.position.x -= self.shift[0]
        shifted_msg.pose.pose.position.y -= self.shift[1]

        self.pub.publish(shifted_msg)

        if needs_reset:
            #reset ekf pose as well, to view the TF frame correctly aligned (Want to test)
            ekf_odom = PoseWithCovarianceStamped()
            ekf_odom.header = msg.header
            ekf_odom.header.frame_id = 'odom'
            ekf_odom.pose = shifted_msg.pose
            self.pub_ekf.publish(ekf_odom)

            self.shift[0] = 0.0  #since we reset EKf, the shift must be nullified.
            self.shift[1] = 0.0
            self.get_logger().info('EKF pose reset applied')

    def shift_callback(self, msg: Odometry):
        shifted_msg = Odometry()
        shifted_msg.header = msg.header
        shifted_msg.child_frame_id = msg.child_frame_id
        shifted_msg.pose = msg.pose
        shifted_msg.twist = msg.twist

        self.shift[0] = self.sensor_odom[0] - shifted_msg.pose.pose.position.x
        self.shift[1] = self.sensor_odom[1] - shifted_msg.pose.pose.position.y
        self.get_logger().info(f'changing shift to: x={self.shift[0]}, y={self.shift[1]}')


def main(args=None):
    rclpy.init(args=args)

    # Parse shift from command line arguments
    if len(sys.argv) != 3:
        print("Usage: ros2 run <your_package> shift_husarion_odom.py <x_shift> <y_shift> ")
        return
    # print(sys.argv[1], sys.argv[2])
    shift_x = float(sys.argv[1])
    shift_y = float(sys.argv[2])

    node = OdomShiftNode(shift_x, shift_y)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
