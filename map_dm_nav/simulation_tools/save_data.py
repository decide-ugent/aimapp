#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray

from cv_bridge import CvBridge
import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
import csv
import psutil

try:
    from mocap4r2_msgs.msg import RigidBodies
except :
    print('mocap4r2_msgs to connect to qualisys bodies does not exist. Please install mocap4ros2_qualisys')

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

        self.odom_sub = self.create_subscription(
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

        self.subscription2 = self.create_subscription(
            OccupancyGrid,
            '/map',  
            self.map_callback,
            10
        )

        self.gt_odom_sub = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.mocap_odom_callback,
            10)
        
        self.cpu_per = self.create_subscription(
            Float32,
            '/cpu_percent',  
            self.cpu_callback,
            10
        )

        self.ram_perc_pub = self.create_subscription(
            Float32,
            '/ram_percent',
            self.ram_perc_callback,
            10
        )

        self.ram_used_pub = self.create_subscription(
            Float32,
            '/ram_used',
            self.ram_used_callback,
            10
        )


        
        experiment = 'ours'
        self.csv_file_path = experiment+'/records.csv'
        self.save_map_folder = experiment+ '/map'
        os.makedirs(self.save_map_folder, exist_ok=True)
        os.makedirs(experiment, exist_ok=True)
        self.create_csv_file()
        
        # Timer to control image saving interval
        self.timer = self.create_timer(1.0, self.save_data)

        self.bridge = CvBridge()
        # Track the time the node started
        self.start_time = None
        self.current_time = None

        self.counter = 0
        self.map_data = None
        self.odom= None
        self.husarion_odom = None
        self.gt_odom = None
        self.visitable_nodes = {}
        self.visited_cells = None
        self.travelled_dist  = 0
        self.husarion_travelled_dist = 0

        self.cpu = None
        self.ram = None
        self.cpu = None
        self.ram_per = None
        self.ram_used = None


    def odom_callback(self, msg):

        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        if self.odom is not None:
            travelled_distance = np.sqrt((robot_x - self.odom[0])**2 + (robot_y - self.odom[1])**2)
            self.travelled_dist += travelled_distance
        self.odom = (robot_x, robot_y)
        
        # if self.start_time is None:
        #     self.start_time = abs(msg.header.stamp.sec)#*(1/10**9)
        #     self.current_time = self.start_time 
        # current_time = abs(msg.header.stamp.sec)#*(1/10**9)
        # self.current_time = current_time - self.start_time 

    def mocap_odom_callback(self, msg):
        last_body = msg.rigidbodies[49]
        self.gt_odom = (np.round(last_body.pose.position.x,2),  np.round(last_body.pose.position.y,2))


    def sensor_odom_callback(self, msg):
        # Process the sensor odometry data
        robot_x = np.round(msg.pose.pose.position.x,2)
        robot_y = np.round(msg.pose.pose.position.y,2)
        if self.husarion_odom is not None:
            husarion_travelled_distance = np.sqrt((robot_x - self.husarion_odom[0])**2 + (robot_y - self.husarion_odom[1])**2)
            self.husarion_travelled_dist += husarion_travelled_distance
        self.husarion_odom = (robot_x, robot_y)

        if self.start_time is None:
            self.start_time = abs(msg.header.stamp.sec)#*(1/10**9)
            self.current_time = self.start_time 
        current_time = abs(msg.header.stamp.sec)#*(1/10**9)
        self.current_time = current_time - self.start_time 
        
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
    def cpu_callback(self, msg):
        self.cpu = float(msg.data)

    def ram_perc_callback(self, msg):
        self.ram_per = float(msg.data)

    def ram_used_callback(self, msg):
        self.ram_used = float(msg.data)
    def save_map(self):
        # Save the map only if we have received it
        if self.map_data is None:
            self.get_logger().info('No map data received yet.')
            return
    
        map_image = self.generate_map()

        # Save the image
        map_file_name = os.path.join(self.save_map_folder, f"map_{self.current_time}")
        image_path = f"{map_file_name}.pgm"
        cv2.imwrite(image_path, map_image)
        self.get_logger().info(f'Saved map image {image_path}')

    def generate_map(self):
        if self.map_data is not None:
            # Convert OccupancyGrid to a numpy array
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin = self.map_data.info.origin

            map_array = np.array(self.map_data.data).reshape((height, width))

            # Count the number of cells that are not -1 (known cells)
            self.visited_cells = np.count_nonzero(map_array != -1)
            print(f"Number of known cells (not -1): {self.visited_cells}")
            
            # Convert map data to an image: -1 -> unknown, 0 -> free, 100 -> occupied
            map_image = np.zeros_like(map_array, dtype=np.uint8)
            map_image[map_array == -1] = 205  # Unknown
            map_image[map_array == 0] = 255   # Free
            map_image[map_array == 100] = 0   # Occupied
            return map_image
        return None

    def save_map_wt_robot(self):
        map_image = self.generate_map()
        if map_image is not None:
            # if self.odom is not None:
            #     width = self.map_data.info.width
            #     height = self.map_data.info.height
            #     resolution = self.map_data.info.resolution
            #     origin = self.map_data.info.origin

            #     robot_x = int((self.odom[0] - origin.position.x) / resolution)
            #     robot_y = height - int((self.odom[1] - origin.position.y) / resolution)
                
            #     # Draw a circle representing the robot's position on the map
            #     cv2.circle(map_image, (robot_x, robot_y), 5, (0, 0, 255), -1)

            # Save the map image with the robot's position
            map_file_name = os.path.join(self.save_map_folder, f"map_wt_robot_{self.current_time}")
            image_path = f"{map_file_name}.pgm"
            self.get_logger().info(str(type(map_image)))
            self.get_logger().info(f'Saving map image {image_path}')
            cv2.imwrite(image_path, map_image)
            self.get_logger().info(f'Saved map image {image_path}')


    def map_callback(self, msg):
        # Store the latest map data
        self.map_data = msg


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
                "gt_odom",
                'husarion_travelled_dist',
                'visitable_nodes',
                'travelled_distance',
                'visited_cells',
                'cpu_usage',
                'ram_usage',
                'ram_used_gb'
            ]
            writer.writerow(headers)
        self.get_logger().info(f'Created CSV file: {self.csv_file_path}')

    def save_image(self, elapsed_time, image, folder):
        if image is not None :
            filename = os.path.join(folder, f'{elapsed_time}.jpg')
            cv2.imwrite(filename, image)
            # self.get_logger().info(f'Saved image: {filename}')

    def record_memory_usage(self):
        self.cpu = psutil.cpu_percent(interval=1)
        self.ram = psutil.virtual_memory()
        # print("CPU usage (%):", self.cpu)
        # print("RAM usage (%):", self.ram.percent)
        # print("RAM used (GB):", round(self.ram.used / 1e9, 2))


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

            #GT odom
            if self.gt_odom:
                row_data.extend([self.gt_odom])
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

            if self.visited_cells:
                row_data.extend([self.visited_cells])
            else:
                row_data.extend([None])

            if self.cpu:
                row_data.extend([self.cpu])
            elif self.cpu_per:
                row_data.extend([self.cpu_per])
            else:
                row_data.extend([None])
            if self.ram:
                row_data.extend([self.ram.percent, round(self.ram.used / 1e9, 2)])
            elif self.ram_per and self.ram_used:
                row_data.extend([self.ram_per, self.ram_used])
            else:
                row_data.extend([None, None])
            # Write the row to the CSV
            writer.writerow(row_data)
        # self.get_logger().info(f'Saved additional data to {self.csv_file_path}')

    def save_data(self):
        #model = self.get_latest_model()
        #self.record_memory_usage()
        self.save_csv_data()
        if self.counter % 5 == 0:
            self.get_logger().info(f'self.gt_odom {self.gt_odom}')
            self.save_map_wt_robot()
        self.counter+=1


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