#!/usr/bin/env python3
"""
Subscribes to /cmd_vel from bag playback and republishes with scaled velocities.
This allows the robot to move the correct distance when bag is played at higher speeds.

Usage:
    python3 scale_cmd_vel.py --scale 3.0

Then in another terminal:
    ros2 bag play <bag_path> --rate 3.0 --remap /cmd_vel:=/cmd_vel_original

OPTIONAL TIMEOUT FEATURE:
    To enable cmd_vel timeout (stops robot if no new commands received, like rosbot_ros),
    uncomment the timeout-related lines in __init__ and cmd_vel_callback methods.
    This prevents TurtleBot3 Gazebo from keeping cmd_vel in memory indefinitely.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse


class CmdVelScaler(Node):
    def __init__(self, scale_factor=1.0):
        super().__init__('cmd_vel_scaler')
        self.scale_factor = scale_factor

        # Subscribe to original cmd_vel from bag (remapped)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_original',
            self.cmd_vel_callback,
            10
        )

        # Publish scaled cmd_vel to robot
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Smoothing: keep track of last published command
        self.last_msg = None
        self.smoothing_factor = 0.7  # 0 = no smoothing, 1 = full smoothing

        # Debouncing: ignore very small changes in angular velocity
        self.angular_threshold = 0.05  # rad/s threshold for angular changes

        # ============ CMD_VEL TIMEOUT FEATURE ============
        # Uncomment the 3 lines below to enable timeout functionality
        # (stops robot if no cmd_vel received within timeout period, like rosbot_ros)
        # Also uncomment the corresponding lines in cmd_vel_callback method (search for "TIMEOUT")

        self.enable_timeout = True
        self.cmd_vel_timeout = 0.5  # seconds (default: 0.5s like rosbot)
        self.last_cmd_vel_time = self.get_clock().now()
        self.timeout_timer = self.create_timer(0.05, self.check_timeout_callback)  # 20 Hz

        # ================================================

        self.get_logger().info(f'cmd_vel_scaler node started with scale factor: {self.scale_factor}')
        self.get_logger().info(f'Smoothing factor: {self.smoothing_factor}, Angular threshold: {self.angular_threshold}')
        self.get_logger().info('Subscribing to: /cmd_vel_original')
        self.get_logger().info('Publishing to: /cmd_vel')

    def cmd_vel_callback(self, msg):
        """Scale velocity values with smoothing and republish"""
        # ============ CMD_VEL TIMEOUT FEATURE ============
        # Update timestamp when receiving new cmd_vel (uncomment if timeout enabled)
        if hasattr(self, 'enable_timeout') and self.enable_timeout:
            self.last_cmd_vel_time = self.get_clock().now()
        # ================================================

        scaled_msg = Twist()

        # Scale linear velocities
        scaled_linear_x = msg.linear.x * self.scale_factor
        scaled_linear_y = msg.linear.y * self.scale_factor
        scaled_linear_z = msg.linear.z * self.scale_factor

        # Scale angular velocities
        scaled_angular_x = msg.angular.x * self.scale_factor
        scaled_angular_y = msg.angular.y * self.scale_factor
        scaled_angular_z = msg.angular.z * self.scale_factor

        # Linear velocities - no smoothing, just scaled
        scaled_msg.linear.x = scaled_linear_x
        scaled_msg.linear.y = scaled_linear_y
        scaled_msg.linear.z = scaled_linear_z

        # Apply smoothing only to angular velocities
        if self.last_msg is not None:
            # Smooth angular velocities with threshold
            angular_diff = abs(scaled_angular_z - self.last_msg.angular.z)
            if angular_diff < self.angular_threshold:
                # Small change - keep previous angular velocity to avoid jitter
                scaled_msg.angular.z = self.last_msg.angular.z
            else:
                # Significant change - apply smoothing
                scaled_msg.angular.z = (self.smoothing_factor * self.last_msg.angular.z +
                                       (1 - self.smoothing_factor) * scaled_angular_z)

            scaled_msg.angular.x = (self.smoothing_factor * self.last_msg.angular.x +
                                   (1 - self.smoothing_factor) * scaled_angular_x)
            scaled_msg.angular.y = (self.smoothing_factor * self.last_msg.angular.y +
                                   (1 - self.smoothing_factor) * scaled_angular_y)
        else:
            # First message - no smoothing
            scaled_msg.angular.x = scaled_angular_x
            scaled_msg.angular.y = scaled_angular_y
            scaled_msg.angular.z = scaled_angular_z

        # Store this message for next smoothing iteration
        self.last_msg = scaled_msg

        self.publisher.publish(scaled_msg)

    def check_timeout_callback(self):
        """
        Timer callback to check for cmd_vel timeout.
        Continuously publishes zero velocities when timeout has elapsed since last cmd_vel.

        This method is only called if timeout feature is enabled (see __init__).
        Mimics rosbot_ros behavior where cmd_vel commands don't persist indefinitely.

        Continuously sends stop commands to ensure Gazebo keeps the robot stopped.
        """
        if not hasattr(self, 'enable_timeout') or not self.enable_timeout:
            return

        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9

        if time_since_last_cmd > self.cmd_vel_timeout:
            # Continuously publish zero velocities while in timeout state
            # This ensures the robot stays stopped in Gazebo
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)

            # Log only once when first entering timeout state
            # (check if last_msg was non-zero to detect state change)
            if self.last_msg is not None and (
                self.last_msg.linear.x != 0.0 or
                self.last_msg.linear.y != 0.0 or
                self.last_msg.angular.z != 0.0
            ):
                self.get_logger().info(
                    f'cmd_vel timeout ({self.cmd_vel_timeout}s) - continuously sending stop commands'
                )

            # Update last message to track timeout state
            self.last_msg = stop_cmd


def main(args=None):
    parser = argparse.ArgumentParser(description='Scale cmd_vel velocities for faster bag replay')
    parser.add_argument('--scale', type=float, default=3.0,
                       help='Velocity scale factor (default: 3.0)')

    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    scaler = CmdVelScaler(scale_factor=parsed_args.scale)

    try:
        rclpy.spin(scaler)
    except KeyboardInterrupt:
        pass
    finally:
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
