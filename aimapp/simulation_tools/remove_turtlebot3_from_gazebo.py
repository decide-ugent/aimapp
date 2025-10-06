#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import threading
import sys

class RemoveTurtlebot3(Node):

    def __init__(self, namespace):
        super().__init__('remove_turtlebot3')
        self.namespace = namespace
        self.cli = self.create_client(DeleteEntity, '/delete_entity')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available.')
            break
        self.req = DeleteEntity.Request()
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        self.odom_subscription  # prevent unused variable warning
        self.request_sent = False
        self.shutdown_requested = False
        self.timeout = 3  # Timeout in seconds for checking odom topic
        self.timer = None

    def start_timer(self):
        self.timer = threading.Timer(self.timeout, self.timer_callback)
        self.timer.start()

    def timer_callback(self):
        if not self.request_sent and not self.shutdown_requested:
            self.get_logger().info('Timeout reached. No odom topic detected.')
            self.shutdown()

    def odom_callback(self, msg):
        if not self.request_sent and not self.shutdown_requested:
            self.get_logger().info('Odom topic detected, removing TurtleBot3...')
            if self.timer:
                self.timer.cancel()
            self.send_request()

    def send_request(self):
        self.req.name = 'waffle_pi'
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.future_callback)
        self.request_sent = True

    def future_callback(self, future):
        try:
            if future.result() is not None:
                self.get_logger().info('TurtleBot3 removed successfully')
            else:
                self.get_logger().info('Service call failed')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')
        self.shutdown()

    def shutdown(self):
        self.get_logger().info('Shutting down RemoveTurtlebot3 node...')
        self.shutdown_requested = True
        if self.timer:
            self.timer.cancel()  # Cancel the timer if it's still running
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    namespace = sys.argv[1] 
    remove_turtlebot3 = RemoveTurtlebot3(namespace)
    remove_turtlebot3.start_timer()

    try:
        rclpy.spin(remove_turtlebot3)
    except KeyboardInterrupt:
        pass
    finally:
        if not remove_turtlebot3.shutdown_requested:
            remove_turtlebot3.shutdown()

if __name__ == '__main__':
    main()