#!/usr/bin/env python3
import rclpy
import numpy as np
import tf2_ros 

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from aimapp_actions.action import Panorama   
from aimapp.obs_transf.theta_record import kill_gphoto2_processes, \
      capture_image, get_latest_image_file, read_image


class GeneratePanorama360Cam(Node):

    def __init__(self, cmd_angular_max:float=1.0):
        super().__init__('generate_panorama_from_ricoh_camera')
        self.last_scan = None
        self.robot_odom = None
        self.tf_theta = None
        self.img_bridge = CvBridge()
        self.images_dir = "/home/husarion/Pictures/theta_ricoh_x"

        self.execution_rate =  self.create_rate(1) #sec = 1/Hz

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        """************************************************************
        ** Initialise ROS subscribers
        ************************************************************"""
        self.lidar_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.lidar_callback,
                10)        
        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                5)
        
        self.lidar_sub
        self.odom_sub 

        """************************************************************
        ** Initialise ROS action
        ************************************************************"""
        
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.action_server = ActionServer(
            node=self,
            action_type=Panorama,
            action_name="get_360_image",
            execute_callback=self.panorama_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            result_timeout=2000)
        

        self.get_tf_transform()
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
        Return list of Image observations taken at those goal poses
        LidarScan assumes 360' lidar
        """
        self.get_logger().info('goal_angles'+str(goal_handle.request))
        if self.tf_theta is None:
            self.get_tf_transform()
        goal_angles = goal_handle.request.goal_angles
        n_actions = goal_handle.request.n_actions
        feedback_msg = Panorama.Feedback()
        result = Panorama.Result()
        actions_range = self.generate_action_range(n_actions)

        #Take a picture with camera (might take a few try) 
        #Then get the latest image in folder and save it in a list
        kill_gphoto2_processes()
        capture_image(self.images_dir, iteration=0)

        latest_image_path = get_latest_image_file(self.images_dir)
        self.get_logger().info(f"Latest image: {latest_image_path}")
        image = read_image(latest_image_path, reduction=6)
        if image is None:
            self.get_logger().error("Failed to read the image. Exiting.")
            goal_handle.abort()
            return Panorama.Result()
        images_compilation = [self.img_bridge.cv2_to_imgmsg(image, encoding='rgb8')]

        while self.last_scan is None or self.robot_odom is None:
            self.execution_rate.sleep()
        laser_compilation = self.compute_scan_dist(actions_range)

        result.panorama = images_compilation
        result.pano_scan = laser_compilation
        self.get_logger().info("laser_compilation:" + str(laser_compilation))
        self.get_logger().info('Incoming response composed of : %s data'  % (str(len(images_compilation)))) 
        goal_handle.succeed() 
        return result


    def compute_scan_dist(self,actions_range) -> float:
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
            if scan_info < 0.18 and curr_ray_angle_deg > 178 and curr_ray_angle_deg < 183: #Position of the camera 
                # self.get_logger().info(f'angle:{curr_ray_angle_deg}, lidar_dist: {scan_info}') 
                continue
            ob_dist_per_actions[action_id].append(scan_info)

        lidar_dist = [np.mean(ob_dist_per_action) for ob_dist_per_action in ob_dist_per_actions]
        # for i in range(len(lidar_dist)):
        #     if np.isnan(lidar_dist[i]):
        #         self.get_logger().info('Incoming response composed of : %s data'  % (str(ob_dist_per_actions[i]))) 

        # self.get_logger().info(f'lidar_dist: {lidar_dist}')
        return lidar_dist
    
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
    def lidar_callback(self, msg):
        if self.last_scan is None:
            self.get_logger().info('I heard: scan')
        self.last_scan = msg
        #print('self.last_scan in callback', self.last_scan.range_max)

    def odom_callback(self, msg):
        if self.robot_odom is None:
            self.get_logger().info('I heard: odom')
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

    generate_panorama = GeneratePanorama360Cam(cmd_angular_max=0.2)

    # rclpy.spin(generate_panorama)
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(generate_panorama)
    rclpy.spin(generate_panorama, executor=multi_thread_executor)
    generate_panorama.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


