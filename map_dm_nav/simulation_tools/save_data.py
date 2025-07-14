#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
import csv


from map_dm_nav.visualisation_tools import plot_state_in_map_wt_gt, pickle_load_model

class DataSaver(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
               
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/shifted',
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
            '/odometry/filtered',
            self.sensor_odom_callback,
            10
        )

        self.visitable_nodes = self.create_subscription(MarkerArray,
            '/visitable_nodes',
            self.visitable_nodes_callback,
            10
        )

        
        # self.subscription2 = self.create_subscription(
        #     Image,
        #     '/internal_map',  
        #     self.map_callback,
        #     10
        # )

        experiment = 'ours'
        self.csv_file_path = experiment+'/records.csv'
        os.makedirs(experiment, exist_ok=True)
        self.create_csv_file()
        
        # Timer to control image saving interval
        self.timer = self.create_timer(1.0, self.save_data)

        self.bridge = CvBridge()
        # Track the time the node started
        self.start_time = None
        self.current_time = None

        # self.map_data = None
        self.odom= None
        self.husarion_odom = None
        self.visitable_nodes = {}
        self.travelled_dist  = 0
        self.husarion_travelled_dist = 0

        self.cpu = None
        self.ram = None


    def odom_callback(self, msg):

        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        if self.odom is not None:
            travelled_distance = np.sqrt((robot_x - self.odom[0])**2 + (robot_y - self.odom[1])**2)
            self.travelled_dist += travelled_distance
        self.odom = (robot_x, robot_y)
        
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec
            self.current_time = self.start_time 
        self.current_time = self.current_time - self.start_time 

    def sensor_odom_callback(self, msg):
        # Process the sensor odometry data
        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        if self.husarion_odom is not None:
            husarion_travelled_distance = np.sqrt((robot_x - self.husarion_odom[0])**2 + (robot_y - self.husarion_odom[1])**2)
            self.husarion_travelled_dist += husarion_travelled_distance
        self.husarion_odom = (robot_x, robot_y)
        
    def visitable_nodes_callback(self, marker_array_msg):
        # Process the visitable nodes if needed
        for node_marker in marker_array_msg.markers:
            if node_marker.ns != 'state_labels':
                continue

            text = node_marker.text
            text_splitted = text.split('\n')
            state = int(text_splitted[0][1:])
        
            x = node_marker.pose.position.x 
            y = node_marker.pose.position.y 
            z = node_marker.pose.position.z
            observation = int(text_splitted[1][1:])
            self.visitable_nodes[state] = {'pose': (x, y, z), 'observation': observation}

    def get_latest_model(self):
        try:
            return pickle_load_model()
        except (FileNotFoundError, EOFError):
            return None

    def create_csv_file(self):
        # Create the CSV file and write headers
        with open(self.csv_file_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            # Define the headers
            headers = [
                "Time", 
                "odom", 
                "husarion_odom",
                'husarion_travelled_dist',
                'visitable_nodes',
                'travelled_distance',
            ]
            writer.writerow(headers)
        self.get_logger().info(f'Created CSV file: {self.csv_file_path}')

    def save_image(self, elapsed_time, image, folder):
        if image is not None :
            filename = os.path.join(folder, f'{elapsed_time}.jpg')
            cv2.imwrite(filename, image)
            # self.get_logger().info(f'Saved image: {filename}')
    
    def save_csv_data(self):
        if self.current_time is None:
            return
        with open(self.csv_file_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            row_data = [self.current_time]

            # Odometry data
            if self.odom:
                row_data.extend([self.odom])
            else:
                row_data.extend([None])
            #sensors odom
            if self.husarion_odom:
                row_data.extend([self.husarion_odom])
            else:
                row_data.extend([None])
            
            if self.husarion_travelled_dist:
                row_data.extend([self.husarion_travelled_dist])
            else:
                row_data.extend([None])

            if self.visitable_nodes:
                row_data.extend([self.visitable_nodes])
            else:
                row_data.extend([None])

            if self.travelled_dist:
                row_data.extend([self.travelled_dist])
            else:
                row_data.extend([None])
            # Write the row to the CSV
            writer.writerow(row_data)
        # self.get_logger().info(f'Saved additional data to {self.csv_file_path}')

    def save_data(self):
        model = self.get_latest_model()
        self.save_csv_data()

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