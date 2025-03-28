#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from map_dm_nav_actions.action import Panorama   
from map_dm_nav.obs_transf.theta_record import kill_gphoto2_processes, \
      capture_image, get_latest_image_file, read_image


class GeneratePanorama360Cam(Node):

    def __init__(self, cmd_angular_max:float=1.0):
        super().__init__('generate_panorama_from_ricoh_camera')
        self.last_scan = None
        self.robot_odom = None
        self.images_dir = "/home/theta_ricoh_x/images"

        self.execution_rate =  self.create_rate(1) #sec = 1/Hz

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
                '/odom',
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
        
        goal_angles = goal_handle.request.goal_angles
        feedback_msg = Panorama.Feedback()
        result = Panorama.Result()

        #Take a picture with camera (might take a few try) 
        #Then get the latest image in folder and save it in a list
        kill_gphoto2_processes()
        capture_image(self.images_dir, iteration=0)
        latest_image_path = get_latest_image_file(self.images_dir)
        # print(f"Latest image: {latest_image_path}")
        images_compilation = list(read_image(latest_image_path))

        while self.last_scan or self.robot_odom is None :
            self.execution_rate.sleep()

        orientation_compilation, laser_compilation = self.compute_scan_dist()
        self.get_logger().info(str(type(orientation_compilation)) + str(orientation_compilation))
        result.panorama = images_compilation
        result.pano_scan = laser_compilation
        result.orientations = orientation_compilation
        self.get_logger().info('Incoming response composed of : %s data'  % (str(len(images_compilation)))) 
        return result

    
    def compute_scan_dist(self) -> float:
        """This assumes a 360` lidar. Won't work with a forward lidar"""
        
        angle_min = self.last_scan.angle_min
        angle_max = self.last_scan.angle_max
        range_max = self.last_scan.range_max
        step = self.last_scan.angle_increment
        scan = [range_max if (x == np.inf or np.isnan(x)) else x for x in self.last_scan.ranges]
        scan_range  = len(scan)

        angle_per_step = (angle_max + abs(angle_min)) / step #rad/step

        #How many step left and right should we check lidar as well
        step_range = int(np.round(0.1/angle_per_step /2))#rad so ~6deg 
        desired_angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
        desired_angles = [float(angle) for angle in desired_angles]

        angles = angle_min + np.arange(scan_range) * step
        angles = np.mod(angles + self.robot_odom[2], 2 * np.pi) 
        
        
        
        lidar_dist = []
        deg_angles = []
        for desired_angle in desired_angles:
            # Find the closest LIDAR measurement to the desired angle
            idx = (np.abs(angles - desired_angle)).argmin()
            obstacle_dist = 0
            #this in two lines to consider negative values as idx
            for i in range(idx-step_range, idx+step_range+1,1):
                if i >= scan_range: 
                    i = scan_range -i 
                obstacle_dist+=scan[i]
            obstacle_dist = obstacle_dist/(step_range*2+1)
            print('desired_angle,',np.rad2deg(desired_angle),'idx',idx,'dist', obstacle_dist) 
            lidar_dist.append(obstacle_dist)
            deg_angles.append(np.rad2deg(desired_angle))


        self.get_logger().info('desired_angle: %s' % str(deg_angles))
        self.get_logger().info('lidar_dist: %s' % str(lidar_dist))
      
        return desired_angles, lidar_dist
    

    """************************************************************
    ** ROS CALLBACKS
    ************************************************************"""
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

    generate_panorama = GeneratePanorama360Cam(cmd_angular_max=0.2)

    # rclpy.spin(generate_panorama)
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(generate_panorama)
    rclpy.spin(generate_panorama, executor=multi_thread_executor)
    generate_panorama.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


