#!/usr/bin/env python3
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from slam_toolbox.srv import SerializePoseGraph


class SaveSLAMMAP(Node):
    def __init__(self):
        super().__init__('save_slam_map')

        self.tests_directory = os.getcwd() + '/latest_slam_map'
        self.is_saving = False

        self.sub = self.create_subscription(
            Odometry,
            '/shifted_odom',
            self.odom_callback,
            10
        )

        # Create client for SLAM toolbox's serialize service
        self.slam_serialize_client = self.create_client(
            SerializePoseGraph,
            '/slam_toolbox/serialize_map'
        )

        # Wait for the service to be available
        self.get_logger().info('Waiting for SLAM toolbox serialize service...')
        while not self.slam_serialize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SLAM serialize service not available, waiting...')

        self.get_logger().info('SLAM serialize service is ready')
        self.get_logger().info(f'Will save SLAM map to: {self.tests_directory}')
        self.get_logger().info('Waiting for odometry message to trigger save...')


    def odom_callback(self, msg):
        if not self.is_saving:
            self.is_saving = True
            self.get_logger().info(f'odometry received, saving SLAM map to {self.tests_directory}')
            self.save_slam_map(self.tests_directory)

    def save_slam_map(self, store_path: str):
        """
        Save SLAM map in serialized format using SLAM toolbox service.
        This allows SLAM to reload and continue from the saved map.
        """
        store_path = Path(store_path)

        # Create directory if it doesn't exist
        store_path.mkdir(parents=True, exist_ok=True)

        if not self.slam_serialize_client.service_is_ready():
            self.get_logger().warn('SLAM toolbox serialize service not available, skipping SLAM map save')
            self.is_saving = False
            return

        # Path for SLAM serialized map (without extension, SLAM toolbox adds it)
        map_filename = str(store_path / 'slam_map')
        
        if os.path. exists(map_filename + 'posegraph'):
            os.remove(map_filename + 'posegraph')
            os.remove(map_filename + 'data')

        request = SerializePoseGraph.Request()
        request.filename = map_filename

        self.get_logger().info(f'Saving SLAM map to: {map_filename}')

        # Call service asynchronously with a callback
        future = self.slam_serialize_client.call_async(request)
        future.add_done_callback(lambda f: self.slam_save_callback(f, map_filename))

    def slam_save_callback(self, future, map_filename):
        """Callback when SLAM map save completes"""
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f'SLAM map saved successfully to {map_filename}')
                self.get_logger().info(f'Result code: {result.result}')
            else:
                self.get_logger().error('SLAM service call completed but returned None')
        except Exception as e:
            self.get_logger().error(f'SLAM map save failed with exception: {str(e)}')
        finally:
            self.is_saving = False

def main(args=None):
    rclpy.init(args=args)
    node = SaveSLAMMAP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
