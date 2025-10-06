#!/usr/bin/env python3
import rclpy
import numpy as np
import tf2_ros
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from aimapp_actions.action import Panorama   


class GeneratePanorama(Node):

    def __init__(self, cmd_angular_max:float=1.0):
        super().__init__('generate_panorama')
        self.last_image = None
        self.last_scan = None
        self.robot_odom = None
        self.tf_theta = None
        self.tf_buffer = tf2_ros.Buffer()

        self.execution_rate =  self.create_rate(1) #sec = 1/Hz

        
        """************************************************************
        ** Initialise ROS subscribers
        ************************************************************"""
        self.lidar_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.lidar_callback,
                10)
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                5)
        
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel", 
            qos_profile= qos_policy)
        
        self.image_sub  # prevent unused variable warning
        self.lidar_sub
        self.odom_sub 

        self.cmd_angular_max = cmd_angular_max
        self.angular_vel_reductor = 0.6
        self.tf_theta = None

        """************************************************************
        ** Initialise ROS action
        ************************************************************"""
        
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.action_server = ActionServer(
            node=self,
            action_type=Panorama,
            action_name="get_panorama",
            execute_callback=self.panorama_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            result_timeout=2000)
        
        self.get_logger().info('Node generate_panorama initialised')
    
    """************************************************************
    ** Handlers ROS action
    ************************************************************"""
        
    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def action_goal_exceptions(self, goal_handle):
        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            self.stop_rotation()
            return True

        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            self.stop_rotation()
            return True
        return False
    
    """************************************************************
    ** Callback ROS action
    ************************************************************"""
         
    def panorama_callback(self, goal_handle):
        """ 
        Service receiving a number of stop during turn around. At each stop, takes pic 
        Return list of Image and LidarScan observations taken at those goal poses
        """
        # self.get_logger().info('goal_angles'+str(goal_handle.request))
        
        goal_angles = goal_handle.request.goal_angles
        n_actions = goal_handle.request.n_actions
        feedback_msg = Panorama.Feedback()
        result = Panorama.Result()

        #Just to be safe and ensure we received all callback data
        #self.execution_rate.sleep()
        actions_range = self.generate_action_range(n_actions)

        while self.robot_odom is None :
            self.execution_rate.sleep()

        if self.tf_theta is None:
            self.get_tf_transform()

        goal_angles = self.order_orientations(self.robot_odom[2],goal_angles)
        
        images_compilation = []
        laser_compilation = []
        orientation_compilation = []
        
        for goal_angle in goal_angles:
            # self.get_logger().info('stop %f' % float(stop+1))
            self.get_logger().info('next desired orientation: %f, agent orientation: %f' % (goal_angle,self.robot_odom[2]))
            # next_goal_theta = (goal_angles) % (2*np.pi)
            self.reach_next_orientation(goal_angle, feedback_msg, goal_handle)
            #this while isn't safe
            while self.last_image is None or self.last_scan is None:
                self.execution_rate.sleep()
            images_compilation.append(self.last_image)
    
        
        self.stop_rotation()
        while self.last_scan is None :
            self.execution_rate.sleep()
        laser_compilation = self.compute_scan_dist(actions_range)

        result.panorama = images_compilation
        result.pano_scan = laser_compilation
        self.get_logger().info('Incoming response composed of : %s data'  % (str(len(images_compilation)))) 
        return result

    def reach_next_orientation(self,next_goal_theta:float, feedback_msg:object, goal_handle:object):
        #this is a security to compare comparable number when we are looping back from 2pi to 0
        if next_goal_theta <= 0.1 and self.robot_odom[2] >= 5.4: #2pi =~ 6.28
            next_goal_theta = np.clip(next_goal_theta +2*np.pi, a_min = 5.4, a_max=2*np.pi)
            #self.get_logger().info('modified goal orientation: %f ' % (next_goal_theta))

        while abs(self.robot_odom[2] - next_goal_theta) > 0.1 and rclpy.ok():
            if self.action_goal_exceptions(goal_handle):
                return Panorama.Result()
            # rclpy.spin_once(self)
            #self.get_logger().info('current odom: %s, goal to reach: %s' % (str(self.robot_odom), str(next_goal_theta)))
            speed = Twist()
            speed.angular.z = self.compute_angular_velocities(next_goal_theta)
            self.cmd_pub.publish(speed)
            feedback_msg.current_stop = next_goal_theta
            goal_handle.publish_feedback(feedback_msg)
            self.execution_rate.sleep()
    
    def stop_rotation(self)-> None:
        speed = Twist()
        self.cmd_pub.publish(speed)

    def compute_angular_velocities(self, angular_goal:float)->float:
        #Reduce velocity when we get closer to goal
        w_in = self.angular_vel_reductor * abs(angular_goal - self.robot_odom[2])
        w_in = max(min(w_in, self.cmd_angular_max), -self.cmd_angular_max)
        return w_in 
    
    def compute_scan_dist(self,actions_range) -> list:
        """This assumes a 360` lidar. Won't work with a forward lidar"""
        
        angle_min = self.last_scan.angle_min
        angle_max = self.last_scan.angle_max
        range_max = self.last_scan.range_max
        angle_increment = self.last_scan.angle_increment 
        scan = [range_max if (x == np.inf or np.isnan(x)) else x for x in self.last_scan.ranges]
        #self.get_logger().info(f'length scan:{len(scan)}')
        lidar_dist = []
        ob_dist_per_actions = [[] for a in range(len(actions_range))] 
        if self.tf_theta is None:
            theta_offset = 0.0
        else:
            theta_offset = self.tf_theta
        for id, scan_info in enumerate(scan):
            
            curr_ray_angle_rad = \
            (self.robot_odom[2] + theta_offset + \
             angle_min + (id * angle_increment))
            curr_ray_angle_deg = np.rad2deg(curr_ray_angle_rad) % 360 
            #self.get_logger().info(f'curr_ray_angle_deg, ob_dist: {id} {round(curr_ray_angle_rad,1)} {round(curr_ray_angle_deg,1)} {round(scan_info,2)}')
            action_id = next(key for key, value in actions_range.items() if curr_ray_angle_deg >= value[0] and curr_ray_angle_deg <= value[1] )
            ob_dist_per_actions[action_id].append(scan_info)

        lidar_dist = [np.mean(ob_dist_per_action) for ob_dist_per_action in ob_dist_per_actions]
        # for i in range(len(lidar_dist)):
        #     if np.isnan(lidar_dist[i]):
        #         self.get_logger().info('Incoming response composed of : %s data'  % (str(ob_dist_per_actions[i]))) 

        # self.get_logger().info(f'lidar_dist: {lidar_dist}')
        return lidar_dist
    
    def order_orientations(self,current_orientation:float, goal_orientations:list)-> list:
        """ order goal orientations to always turn right from where we are """
        # Normalise orientations to be safe
        current_orientation = current_orientation % (2 * np.pi)
        goal_orientations = [angle % (2 * np.pi) for angle in goal_orientations]
        
        # Calculate the angular difference from the current orientation
        angular_diffs = [(angle - current_orientation) % (2 * np.pi) for angle in goal_orientations]

        #order the orientation to always turn right
        paired_orientations = list(zip(goal_orientations, angular_diffs))
        paired_orientations.sort(key=lambda x: x[1])
        sorted_goal_orientations = [angle for angle, diff in paired_orientations]
        
        return sorted_goal_orientations

    def generate_action_range(self, n_actions):
        zone_range_deg = round(360/n_actions,1)
        n_actions_keys = np.arange(0, n_actions, 1)
        zone_spacing_deg = np.arange(0, 361, zone_range_deg)
        possible_actions = {}
        for action_key in n_actions_keys:
            possible_actions[action_key] = [round(zone_spacing_deg[action_key]), round(zone_spacing_deg[action_key+1]),]
    
        return possible_actions
    
    
    def get_tf_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'laser',      # source frame
                rclpy.time.Time())  # get the latest available
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF from base_link to laser not available.")
            return 
        
        euler_angles = ros_quaternion_to_euler(tf.transform.rotation)
        theta = np.round(euler_angles[2], 4)
        #I want theta between 0-4*pi, not 0-2pi/-2pi-0
        if theta < 0:
            theta += 2*np.pi
        
        self.tf_theta = theta
        self.get_logger().info(f'tf theta: {theta}')

    """************************************************************
    ** ROS CALLBACKS
    ************************************************************"""
    def image_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.encoding)
        self.last_image = msg
            
    def lidar_callback(self, msg):
        # self.get_logger().info('I heard scan: "%s"' % msg.range_max)
        self.last_scan = msg
        #print('self.last_scan in callback', self.last_scan.range_max)

    def odom_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)
        self.robot_odom = extract_robot_xy_theta(msg)

def extract_robot_xy_theta(odom_msg)->np.ndarray:
    """Return array with x, y and yaw in the list"""
    
    quaternion = odom_msg.pose.pose.orientation
    euler_angles = ros_quaternion_to_euler(quaternion)
    theta = np.round(euler_angles[2], 4)
    #I want theta between 0-4*pi, not 0-2pi/-2pi-0
    if theta < 0:
        theta += 2*np.pi

    robot_x = np.round(
        odom_msg.pose.pose.position.x,
        4
    )
    robot_y = np.round(
        odom_msg.pose.pose.position.y,
        4
    )
    robot_pos_xy_theta = np.array(
        [robot_x, robot_y, theta]
    )

    return robot_pos_xy_theta

def ros_quaternion_to_euler(quaternion)-> np.ndarray:
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def main(args=None):
    rclpy.init(args=args)

    generate_panorama = GeneratePanorama(cmd_angular_max=0.2)

    # rclpy.spin(generate_panorama)
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(generate_panorama)
    rclpy.spin(generate_panorama, executor=multi_thread_executor)
    generate_panorama.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


