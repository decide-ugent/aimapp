#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os
import csv
import math
import numpy as np

class SaveImages(Node):
    def __init__(self):
        super().__init__('save_images')
        self.subscription = self.create_subscription(
            Image,
            '/gazebo/camera_front/image_raw',  
            self.image_front_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            Image,
            '/gazebo/camera_left/image_raw',  
            self.image_left_callback,
            10)

        self.subscription3 = self.create_subscription(
            Image,
            '/gazebo/camera_right/image_raw',  
            self.image_right_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/gazebo/scan',
            self.scan_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/gazebo/odom',
            self.odom_callback,
            10)
        

        
        

        self.bridge = CvBridge()
        self.position = None
        self.scan = None
        self.angle_min = None

        self.possible_actions = {0: [0, 30],
                                1: [30, 60],
                                2: [60, 90],
                                3: [90, 120],
                                4: [120, 150],
                                5: [150, 180],
                                6: [180, 210],
                                7: [210, 240],
                                8: [240, 270],
                                9: [270, 300],
                                10: [300, 330],
                                11: [330, 360]}
        self.angle_increment = None
        self.csv_filename = 'pose_obs.csv'
        if not os.path.exists(self.csv_filename):
            self.create_csv_file()

        self.current_folder = None
        self.all_done = [0,0,0]

        self.current_folder = self.get_next_folder()
        self.get_logger().info('folder_name ' + self.current_folder)
        # self.image_count = self.get_image_count()
    
    def create_csv_file(self):
        # Create the CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            # Define the headers
            headers = [
                'state',
                'Pose x for agent',
                'Pose Y for agent',
                'Pose x in ware',
                'Pose y in ware',
                'odom x',
                'odom y',	
                'theta',

                'observation',
                'ob_id',
                'ob_dists'
            ]
            writer.writerow(headers)
        self.get_logger().info(f'Created CSV file: {self.csv_filename}')

    def get_next_folder(self):
        index = 0
        while True:
            folder_name = f'images/image{index}'
            if not os.path.exists(folder_name) or len([f for f in os.listdir(folder_name) if f.endswith(('.jpg', '.png'))]) < 6:
                if not os.path.exists(folder_name):
                    os.makedirs(folder_name)
                return folder_name
            index += 1

    def get_image_count(self):
        return len([f for f in os.listdir(self.current_folder) if f.endswith(('.jpg', '.png'))])

    def save_image(self, image_count, msg):
        if self.current_folder is None or self.all_done[int(image_count)] == 1: 
            return
        
        if len([f for f in os.listdir(self.current_folder) if image_count in f]) > 0:
            image_count += "bis"
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = os.path.join(self.current_folder, f'image_{image_count}.jpg')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved_front: {filename}')
        self.all_done[int(image_count[0])] += 1 

        if sum(self.all_done) == 3:
            self.save_data_csv()

    def compute_scan_dist(self) -> float:
        """This assumes a 360` lidar. Won't work with a forward lidar"""
        
        angle_min = self.angle_min
        angle_max = self.angle_max
        #self.get_logger().info(f'len(ranges): {angle_increment} {len(scan)}')
        angle_increment = (abs(angle_min) + abs(angle_max)) / (np.pi*2)
        scan = self.scan
        
        self.get_logger().info(f'len(ranges): {angle_increment} {len(scan)}')
        self.get_logger().info(f'np.rad2deg(angle_min): {np.rad2deg(angle_min)}')
        self.get_logger().info(f'np.rad2deg(self.position[2]): {np.rad2deg(self.position[2])}')
        
        
        lidar_dist = []
        ob_dist_per_action = [[] for a in range(len(self.possible_actions))] 
        for id, scan_info in enumerate(scan):
            
            curr_ray_angle_deg_prev = \
            (np.rad2deg(self.position[2]) + \
             np.rad2deg(angle_min) + (id * angle_increment))
            curr_ray_angle_deg = curr_ray_angle_deg_prev % 360 
            self.get_logger().info(f'curr_ray_angle_deg, ob_dist: {id} {round(curr_ray_angle_deg_prev,1)} {round(curr_ray_angle_deg,1)} {round(scan_info,2)}')
            action_id = next(key for key, value in self.possible_actions.items() if curr_ray_angle_deg >= value[0] and curr_ray_angle_deg <= value[1] )
            ob_dist_per_action[action_id].append(scan_info)
            #self.get_logger().info(f'ob_dist_per_action[action_id]: {ob_dist_per_action[action_id]}')
        self.get_logger().info(f'ob_dist_per_action: {ob_dist_per_action}')

        lidar_dist = [np.mean(ob_dist_per_action) for ob_dist_per_action in ob_dist_per_action]
        #self.get_logger().info(f'lidar_dist: {lidar_dist}')
        return lidar_dist
    

    def image_front_callback(self, msg):
        image_count = "0"
        self.save_image(image_count, msg)

    def image_left_callback(self, msg):
        image_count = "1"
        self.save_image(image_count, msg)

    def image_right_callback(self, msg):
        image_count = "2"
        self.save_image(image_count, msg)
    
    def odom_callback(self,msg):
        if self.position is None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quaternion = msg.pose.pose.orientation
            theta = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
            self.position = (x, y, theta)
            self.get_logger().info(f'Odometry received: x={x}, y={y}, theta={theta}')


    def scan_callback(self,msg):
        self.scan = [msg.range_max if (x == np.inf or np.isnan(x)) else x for x in msg.ranges]
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

    def save_data_csv(self):
        if self.position is None or self.current_folder is None or self.scan is None:
            return
        ob_dist = self.compute_scan_dist()
        with open(self.csv_filename, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            row_data = [None] * 5
            row_data.extend([self.position[0],self.position[1],self.position[2], self.current_folder, int(self.current_folder[-1]), ob_dist])
            writer.writerow(row_data)
        rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)
    data_saver = SaveImages()
    data_saver.get_logger().info('TEST')
    try:
        rclpy.spin(data_saver)
    except KeyboardInterrupt:
        pass
    finally:
        data_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()