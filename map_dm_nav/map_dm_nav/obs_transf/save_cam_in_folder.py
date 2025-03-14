#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os
import csv
import math

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

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/gazebo/odom',
            self.odom_callback,
            10)
        
        self.bridge = CvBridge()
        self.position = None
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
                "odom", 
                'observation'
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

    def save_data_csv(self):
        if self.position is None or self.current_folder is None:
            return
        with open(self.csv_filename, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            row_data = [self.position, self.current_folder]
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