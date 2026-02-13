#!/usr/bin/env python3
"""
Synchronizes robot movement with bag playback by matching odometry.

This script reads odometry from both:
- /odometry/filtered/bag (from bag playback, remapped)
- /odom (real robot odometry)

And publishes cmd_vel commands to make the real robot follow the bag trajectory
as closely as possible by minimizing the odometry difference.

Usage:
    # In terminal 1 - Start this script
    python3 move_robot_bag_odom.py

    # In terminal 2 - Play bag with odometry remapped
    ros2 bag play <bag_path> --remap /odometry/filtered:=/odometry/filtered/bag

The script uses a proportional controller to minimize position and orientation errors.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OdometryFollower(Node):
    """
    Node that makes the robot follow bag odometry by matching positions.
    """

    def __init__(self):
        super().__init__('odometry_follower')

        # Declare parameters for tuning
        self.declare_parameter('linear_kp', 1.0)  # Proportional gain for linear velocity
        self.declare_parameter('angular_kp', 2.0)  # Proportional gain for angular velocity
        self.declare_parameter('max_linear_vel', 0.26)  # Max linear velocity (m/s)
        self.declare_parameter('max_angular_vel', 0.15)  # Max angular velocity (rad/s)
        self.declare_parameter('position_tolerance', 0.05)  # Position error tolerance (m)
        self.declare_parameter('angle_tolerance', 0.2)  # Angle error tolerance (rad)
        self.declare_parameter('control_rate', 20.0)  # Control loop rate (Hz)
        self.declare_parameter('scale_factor', 1.0)  # Velocity scale factor for cmd_vel_scaler mode

        # Get parameters
        control_rate = self.update_parameters()

        # QoS profile for odometry (typically uses BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for odometry
        self.bag_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered/bag',
            self.bag_odom_callback,
            qos_profile
        )

        self.robot_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.robot_odom_callback,
            qos_profile
        )

        # Subscriber for laser scan (obstacle detection)
        # Use RELIABLE QoS for scan (different from odometry which uses BEST_EFFORT)
        scan_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos_profile
        )

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for set_pose (odometry reset)
        self.set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/set_pose',
            10
        )

        # Store latest odometry messages
        self.bag_odom = None
        self.robot_odom = None

        # Obstacle detection
        self.obstacle_detected = False
        self.obstacle_side = 0  # -1: left clearer, +1: right clearer, 0: center/both
        self.obstacle_min_distance = 0.35  # Minimum distance to obstacle in front
        self.obstacle_distance_threshold = 0.35  # 35cm
        self.left_side_distance = 10.0  # Distance to obstacle on left side
        self.right_side_distance = 10.0  # Distance to obstacle on right side

        # Wall-following state
        self.wall_following_active = False  # Track if actively avoiding obstacle
        self.wall_following_start_yaw = None  # Yaw when wall-following started
        self.wall_following_turn_threshold = math.radians(75)  # Must turn 75° before exiting

        # MCTS message tracking for odometry reset
        self.mcts_message_count = 0
        self.mcts_messages_threshold = 8
        self.last_mcts_time = None
        self.mcts_cooldown = 10.0  # seconds
        self.pose_reset_done = False  # Track if we already did the pose reset
        self.current_linear_velocity = 0.0  # Track current commanded velocity

        self.desired_wall_distance = 0.3
        # Subscribe to MCTS visualization for odometry reset trigger
        self.mcts_sub = self.create_subscription(
            MarkerArray,
            '/mcts_reward_visualization',
            self.mcts_callback,
            10
        )

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )

        # Statistics
        self.position_error_sum = 0.0
        self.angle_error_sum = 0.0
        self.control_iterations = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Odometry Follower with Pose Reset started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Linear Kp: {self.linear_kp}')
        self.get_logger().info(f'  Angular Kp: {self.angular_kp}')
        self.get_logger().info(f'  Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
        self.get_logger().info(f'  MCTS threshold for pose reset: {self.mcts_messages_threshold} messages')
        self.get_logger().info(f'  MCTS cooldown: {self.mcts_cooldown}s')
        self.get_logger().info('Waiting for odometry messages...')
        self.get_logger().info('Will reset odometry to bag position after 7 MCTS messages (when stopped)')

    def bag_odom_callback(self, msg):
        """Store bag odometry (target trajectory)"""
        self.bag_odom = msg

    def robot_odom_callback(self, msg):
        """Store robot odometry (current position)"""
        self.robot_odom = msg

    def mcts_callback(self, msg: MarkerArray):
        """
        Count non-consecutive MCTS messages.
        After 7 messages with 10s cooldown, trigger odometry pose reset when robot is stopped.
        """
        current_time = self.get_clock().now()

        # Check cooldown
        if self.last_mcts_time is not None:
            time_since_last = (current_time - self.last_mcts_time).nanoseconds / 1e9
            if time_since_last < self.mcts_cooldown:
                # Within cooldown - ignore message
                return
        
        # Update timestamp and increment counter
        self.last_mcts_time = current_time
        self.mcts_message_count += 1


        self.get_logger().info(f'MCTS message {self.mcts_message_count}/{self.mcts_messages_threshold} received')

        # Check if we should trigger pose reset (every step after 7 first moves)
        if (self.mcts_message_count >= self.mcts_messages_threshold and
            abs(self.current_linear_velocity) < 0.01):  # Robot is stopped

            self.trigger_pose_reset()

    def trigger_pose_reset(self):
        """
        Reset robot odometry position to match bag position.
        Position: from bag odometry
        Orientation: keep current robot orientation
        """
        if self.bag_odom is None or self.robot_odom is None:
            self.get_logger().warn('Cannot reset pose - odometry data not available')
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = self.robot_odom.header
        pose_msg.header.frame_id = 'map'  # EKF expects set_pose in map frame
        pose_msg.pose = self.robot_odom.pose
        # Position from bag - only Y to maintain axis alignment
        pose_msg.pose.pose.position.y = self.bag_odom.pose.pose.position.y

        # Set covariance (low values = high confidence)
        pose_msg.pose.covariance = self.bag_odom.pose.covariance

        # Publish
        self.set_pose_pub.publish(pose_msg)
        #self.pose_reset_done = True

        self.get_logger().warn('=' * 60)
        self.get_logger().warn('ODOMETRY POSE RESET TRIGGERED')
        self.get_logger().warn(f'New position: x={pose_msg.pose.pose.position.x:.3f}, y={pose_msg.pose.pose.position.y:.3f}')
        self.get_logger().warn('Orientation: kept from current robot pose')
        self.get_logger().warn('=' * 60)

    def scan_callback(self, msg: LaserScan):
        """
        Check for obstacles in front AND on sides of the robot for wall-following.
        Since the lidar is mounted backward, the front of the robot corresponds to
        the back readings of the lidar (around index 0 and len-1, or π radians).

        We check:
        - Front cone (±22.5°): for forward obstacle detection
        - Left/Right sides (±90°): for wall-following distance maintenance
        """
        if len(msg.ranges) == 0:
            self.obstacle_detected = False
            self.obstacle_side = 0
            return

        num_readings = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # The lidar is mounted backward, so "front" of robot is at π (180°) in lidar frame
        front_angle = math.pi  # 180° in lidar frame = front of robot
        front_cone_half_angle = math.radians(60)  # ±60° = 120° front cone (wide to force full turn)
        side_detection_range = math.radians(90)  # ±90° for side detection

        obstacle_in_front = False
        front_left_min_dist = float('inf')
        front_right_min_dist = float('inf')
        front_overall_min_dist = float('inf')

        # Side obstacles (for wall-following)
        left_side_min_dist = float('inf')
        right_side_min_dist = float('inf')

        for i in range(num_readings):
            # Calculate angle for this reading
            reading_angle = angle_min + i * angle_increment

            # Angle relative to front
            angle_from_front = self.normalize_angle(reading_angle - front_angle)
            distance = msg.ranges[i]

            if not np.isfinite(distance):
                continue

            # Check front cone (±22.5°)
            if abs(angle_from_front) <= front_cone_half_angle:
                front_overall_min_dist = min(front_overall_min_dist, distance)

                if distance < self.obstacle_distance_threshold:
                    obstacle_in_front = True

                    if angle_from_front > 0:  # Left side of front
                        front_left_min_dist = min(front_left_min_dist, distance)
                    else:  # Right side of front
                        front_right_min_dist = min(front_right_min_dist, distance)

            # Check side obstacles for wall-following (within ±90°)
            elif abs(angle_from_front) <= side_detection_range:
                if distance < self.obstacle_distance_threshold:
                    # Positive angle = left side, Negative angle = right side
                    if angle_from_front > 0:
                        left_side_min_dist = min(left_side_min_dist, distance)
                    else:
                        right_side_min_dist = min(right_side_min_dist, distance)

        self.obstacle_detected = obstacle_in_front
        self.obstacle_min_distance = front_overall_min_dist if front_overall_min_dist != float('inf') else self.obstacle_distance_threshold

        # Store side distances for wall-following
        self.left_side_distance = left_side_min_dist if left_side_min_dist != float('inf') else 10.0
        self.right_side_distance = right_side_min_dist if right_side_min_dist != float('inf') else 10.0

        # Determine which side to turn when obstacle in front
        if obstacle_in_front:
            if front_left_min_dist > front_right_min_dist + 0.05:  # Left is clearer
                self.obstacle_side = -1  # Turn left
            elif front_right_min_dist > front_left_min_dist + 0.05:  # Right is clearer
                self.obstacle_side = 1  # Turn right
            else:
                self.obstacle_side = 0  # Both sides equally blocked
        else:
            self.obstacle_side = 0

    def quaternion_to_yaw(self, quat):
        """
        Convert quaternion to yaw angle.

        Args:
            quat: Quaternion (x, y, z, w)

        Returns:
            Yaw angle in radians
        """
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi]

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def update_parameters(self):
        # Get parameters
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        control_rate = self.get_parameter('control_rate').value
        return control_rate
    
    
    def control_callback(self):
        """
        Main control loop - compute and publish cmd_vel to follow bag trajectory.
        """
        # Check if we have both odometry messages
        if self.bag_odom is None or self.robot_odom is None:
            return

        self.update_parameters()
        # Extract positions
        bag_x = self.bag_odom.pose.pose.position.x
        bag_y = self.bag_odom.pose.pose.position.y
        bag_yaw = self.quaternion_to_yaw(self.bag_odom.pose.pose.orientation)

        robot_x = self.robot_odom.pose.pose.position.x
        robot_y = self.robot_odom.pose.pose.position.y
        robot_yaw = self.quaternion_to_yaw(self.robot_odom.pose.pose.orientation)

        # Compute errors in global frame
        error_x = bag_x - robot_x
        error_y = bag_y - robot_y
        position_error = math.sqrt(error_x**2 + error_y**2)

        # Compute angle to target position
        angle_to_target = math.atan2(error_y, error_x)

        # Compute heading error (difference between current heading and desired heading)
        heading_error = self.normalize_angle(bag_yaw - robot_yaw)

        # Compute angle error relative to robot's current heading
        angle_error = self.normalize_angle(angle_to_target - robot_yaw)

        # Control strategy:
        # 1. If position error is significant, move towards target position
        # 2. Also correct heading to match target heading

        cmd_vel = Twist()

        # Control strategy: Hybrid approach
        # 1. When far from target: turn toward target position to close distance gap
        # 2. When close to target: match bag's heading for precise trajectory following

        # Check if we're within tolerances
        position_ok = position_error <= self.position_tolerance
        heading_ok = abs(heading_error) <= self.angle_tolerance

        if position_ok:
            # Position is good - stop completely (don't correct heading if already at position)
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            # Hybrid angular control: blend angle_to_target and heading_error
            # When far away: prioritize turning toward position (angle_error)
            # When close: prioritize matching heading (heading_error)
            position_weight = min(1.0, position_error / 0.5)  # 1.0 when >0.5m away, decreases closer

            # Blended angular velocity
            angular_velocity = self.angular_kp * (
                position_weight * angle_error +           # Turn toward position when far
                (1.0 - position_weight) * heading_error   # Match heading when close
            )

            # Linear velocity: proportional to position error, reduced if not facing target
            # Use angle_error (toward target) for forward motion check
            linear_velocity = self.linear_kp * position_error * math.cos(angle_error)

            # Apply velocity limits
            linear_velocity = max(-self.max_linear_vel, min(self.max_linear_vel, linear_velocity))
            angular_velocity = max(-self.max_angular_vel, min(self.max_angular_vel, angular_velocity))

            # Only move forward if we're roughly facing the target position
            if abs(angle_error) > math.pi / 2:
                # If target is behind us, only rotate toward it first
                linear_velocity = 0.0

            # Check if we have wall on sides (wall-following mode)
            wall_on_left = self.left_side_distance < self.obstacle_distance_threshold
            wall_on_right = self.right_side_distance < self.obstacle_distance_threshold

            # Obstacle avoidance: if obstacle detected in front, follow around it toward the goal
            if self.obstacle_detected and linear_velocity > 0:
                # Critical distance for backing away (15cm)
                self.critical_distance = 0.25

                if position_error > self.position_tolerance:  # Far from goal
                    # if self.obstacle_min_distance < self.critical_distance:
                    #     # TOO CLOSE - back away while turning toward clearer side
                    #     linear_velocity = -0.1  # Slow backward motion
                    #     if self.obstacle_side != 0:
                    #         angular_velocity = self.obstacle_side * self.max_angular_vel * 0.8
                    #     self.get_logger().warn(f'BACKING AWAY: dist={self.obstacle_min_distance:.3f}m < {self.critical_distance}m')
                    # else:
                   
                        # Wall-following: turn sharply to be parallel to wall, then progress along it
                    wall_error_left = self.desired_wall_distance - self.left_side_distance
                    wall_error_right = self.desired_wall_distance - self.right_side_distance

                    # Use the closer wall for control (pick wall with larger positive error = closer wall)
                    if abs(wall_error_left) < abs(wall_error_right):
                        wall_error = wall_error_left
                        wall_sign = -1  # Left wall: turn right when too close
                    else:
                        wall_error = wall_error_right
                        wall_sign = 1  # Right wall: turn left when too close

                    # Safety: never go below critical distance
                    if self.right_side_distance < self.critical_distance:
                        linear_velocity = -0.08  # Back away faster
                        angular_velocity = -self.max_angular_vel * 0.9  # Turn left away from right wall
                    elif self.left_side_distance < self.critical_distance:
                        linear_velocity = -0.08  # Back away faster
                        angular_velocity = self.max_angular_vel * 0.9  # Turn right away from left wall
                    else:
                        # Adaptive linear velocity based on clearance
                        clearance_factor = min(self.obstacle_min_distance / self.obstacle_distance_threshold, 1.0)
                        linear_velocity = self.max_linear_vel * 0.7 * clearance_factor  # 0 to 70% speed

                        # Angular velocity based on wall error (clamped to avoid huge values)
                        wall_error_clamped = max(-0.15, min(0.15, wall_error))  # Clamp to ±15cm
                        angular_velocity = self.angular_kp * wall_error_clamped * wall_sign

                        # Bias slightly forward to keep goal progress
                        if position_error > self.position_tolerance :
                            angular_velocity *= 0.7  # Keep 70% of wall correction
                        else:
                            # Close to goal - stop forward motion, only rotate
                            linear_velocity = 0.0
                            self.get_logger().warn('Obstacle near goal - stopping forward, rotation only')        
                

                # Apply angular velocity limits
                angular_velocity = max(-self.max_angular_vel, min(self.max_angular_vel, angular_velocity))

        # Set velocities
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

        # Track linear velocity for pose reset trigger
        self.current_linear_velocity = linear_velocity

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Update statistics
        self.position_error_sum += position_error
        self.angle_error_sum += abs(heading_error)
        self.control_iterations += 1

        # Log periodically (every 2 seconds at 20Hz = every 40 iterations)
        if self.control_iterations % 40 == 0:
            avg_pos_error = self.position_error_sum / self.control_iterations
            avg_angle_error = self.angle_error_sum / self.control_iterations

            self.get_logger().info(
                f'Pos err: {position_error:.3f}m (avg: {avg_pos_error:.3f}m) | '
                f'Robot yaw: {robot_yaw:.3f}rad, Bag yaw: {bag_yaw:.3f}rad, '
                f'Heading err: {heading_error:.3f}rad (avg: {avg_angle_error:.3f}rad) | '
                f'Cmd - Lin: {linear_velocity:.3f}m/s, Ang: {angular_velocity:.3f}rad/s'
            )


def main(args=None):
    rclpy.init(args=args)

    follower = OdometryFollower()

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        if follower.control_iterations > 0:
            avg_pos_error = follower.position_error_sum / follower.control_iterations
            avg_angle_error = follower.angle_error_sum / follower.control_iterations
            follower.get_logger().info(
                f'Final statistics - Average position error: {avg_pos_error:.3f}m, '
                f'Average angle error: {avg_angle_error:.3f}rad'
            )

        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
