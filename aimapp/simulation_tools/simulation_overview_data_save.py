#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
import csv


from aimapp.visualisation_tools import plot_state_in_map_wt_gt, pickle_load_model

class DataSaver(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/world/camera/image_raw', 
            self.camera_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', 
            self.camera_robot_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # self.gt_odom_sub = self.create_subscription(
        #     Odometry,
        #     '/gazebo/odom',
        #     self.real_odom_callback,
        #     10
        # )

        self.gt_odom_sub = self.create_subscription(
            Odometry,
            '/gazebo/odom',
            self.real_odom_callback,
            10
        )
        
        # self.subscription2 = self.create_subscription(
        #     Image,
        #     '/internal_map',  
        #     self.map_callback,
        #     10
        # )

        experiment = 'ours'
        self.save_img_folder = experiment+'/overview'
        self.save_map_folder = experiment+'/map'
        self.csv_file_path = experiment+'/records.csv'
        os.makedirs(self.save_img_folder, exist_ok=True)
        os.makedirs(self.save_map_folder, exist_ok=True)
        self.create_csv_file()
        
        # Timer to control image saving interval
        self.timer = self.create_timer(5.0, self.save_data)

        self.bridge = CvBridge()
        # Track the time the node started
        self.start_time = time.time()

        # Store the most recent image
        self.overview_image = None
        self.robot_image = None
        # self.map_data = None
        self.odom= None
        self.real_odom = None

    def odom_callback(self, msg):

        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        self.odom = (robot_x, robot_y)

    def real_odom_callback(self, msg):
        """ real according to sensors"""
        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        self.real_odom = (robot_x, robot_y)


    def camera_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.overview_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def camera_robot_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.robot_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    # def map_callback(self, msg):
    #     # Store the latest map data
    #     self.map_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_latest_model(self):
        try:
            return pickle_load_model()
        except (FileNotFoundError, EOFError):
            return None

    def create_map(self, model):
        ''' create internal map given the model and convert it to cv2'''
        if self.odom is None or model is None:
            return None
        fig = plot_state_in_map_wt_gt(model, self.odom)
        fig.canvas.draw()
        map = np.array(fig.canvas.renderer.buffer_rgba())
        plt.close(fig)
        # Convert RGBA to BGR
        map = cv2.cvtColor(map, cv2.COLOR_RGBA2BGR)
        return map

    def create_csv_file(self):
        # Create the CSV file and write headers
        with open(self.csv_file_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            # Define the headers
            headers = [
                "Time", 
                "odom", 
                'real_odom'
            ]
            writer.writerow(headers)
        self.get_logger().info(f'Created CSV file: {self.csv_file_path}')

    def save_image(self, elapsed_time, image, folder):
        if image is not None :
            filename = os.path.join(folder, f'{elapsed_time}.jpg')
            cv2.imwrite(filename, image)
            # self.get_logger().info(f'Saved image: {filename}')
    
    def save_csv_data(self, elapsed_time):
        with open(self.csv_file_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            row_data = [elapsed_time]

            # Odometry data
            if self.odom:
                row_data.extend([self.odom])
            else:
                row_data.extend([None])
            #sensors odom
            if self.real_odom:
                row_data.extend([self.real_odom])
            else:
                row_data.extend([None])

            # Write the row to the CSV
            writer.writerow(row_data)
        # self.get_logger().info(f'Saved additional data to {self.csv_file_path}')

    def save_data(self):
        elapsed_time = int(time.time() - self.start_time)
        #simulation top view
        self.save_image(elapsed_time, self.overview_image, self.save_img_folder)
        #Real robot internal view
        self.save_image(elapsed_time, self.robot_image, self.save_img_folder)
        model = self.get_latest_model()
        map = self.create_map(model)
        self.save_image(elapsed_time, map, self.save_map_folder)
        self.save_csv_data(elapsed_time)

# def plot_state_in_map_wt_gt(model:object, gt_odom:list, odom:list=None) -> np.ndarray: 
#     dim = max(25, int(model.get_n_states()/2))
#     fig, ax = plt.subplots(figsize=(dim,dim))  
#     circle=plt.Circle((gt_odom[1], gt_odom[0]),radius=0.21,fill=True, color='0.5') #linestyle='dotted'
#     plt.gca().add_patch(circle)
#     if odom is not None:
#         circle2=plt.Circle((odom[1], odom[0]),radius=0.15,fill=True, color='0.2') #linestyle='dotted'
#         plt.gca().add_patch(circle2)
#     plt.plot()
#     fig = plot_state_in_map(model.get_B(),model.get_agent_state_mapping(), fig_ax=[fig,ax])
#     return fig


def main(args=None):
    rclpy.init(args=args)
    data_saver = DataSaver()

    try:
        rclpy.spin(data_saver)
    except KeyboardInterrupt:
        pass
    finally:
        data_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()