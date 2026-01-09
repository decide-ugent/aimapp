#!/usr/bin/env python3
"""
MCTS Reward Visualizer for RViz
Visualizes MCTS tree node rewards as colored spheres with:
- Color gradient: white (low reward) to black (high reward)
- Larger spheres for top 5 highest reward nodes
- Line strips showing the best path through high-reward nodes
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np


class MCTSRewardVisualiser:
    """
    Visualizes MCTS node rewards in RViz with color-coded spheres and path lines.
    """

    def __init__(self, node, model, frame_id='map'):
        """
        Initialize the MCTS reward visualizer.

        Args:
            node: ROS2 node for publishing and logging
            model: The agent model containing PoseMemory
            frame_id: Frame ID for the markers (default: 'map')
        """
        self.node = node
        self.model = model
        self.frame_id = frame_id

        # Create publisher for marker array
        self.marker_pub = self.node.create_publisher(
            MarkerArray,
            '/mcts_reward_visualization',
            10
        )

    def collect_nodes_from_tree(self, root_node):
        """
        Recursively collect all nodes from MCTS tree with their rewards and positions.

        Args:
            root_node: Root node of the MCTS tree

        Returns:
            list: List of dicts containing node info (id, x, y, reward, N, avg_reward, parent_id)
        """
        nodes_list = []
        visited = set()

        def collect_nodes(node, parent_id=None):
            if node.id in visited:
                return
            visited.add(node.id)

            # Get the pose for this node
            pose = self.model.PoseMemory.id_to_pose(id=node.id)

            if pose is not None:
                avg_reward = node.total_reward / node.N if node.N > 0 else node.state_reward
                nodes_list.append({
                    'id': node.id,
                    'x': pose[0],
                    'y': pose[1],
                    'reward': node.state_reward,
                    'N': node.N,
                    'avg_reward': avg_reward,
                    'parent_id': parent_id
                })

            # Recursively collect children
            if node.has_children_nodes():
                for action_id, child_node in node.childs.items():
                    collect_nodes(child_node, parent_id=node.id)

        collect_nodes(root_node)
        return nodes_list

    def find_best_path(self, nodes_list, root_id):
        """Find path from root to the node with highest individual reward."""
        if not nodes_list:
            return []
        
        # Find the node with the highest reward
        best_node = max(nodes_list, key=lambda n: n['reward'])
        
        # Trace back from best node to root
        path = [best_node['id']]
        current_id = best_node['id']
        node_map = {n['id']: n for n in nodes_list}
        
        while current_id != root_id and current_id in node_map:
            parent_id = node_map[current_id]['parent_id']
            if parent_id is None:
                break
            path.insert(0, parent_id)
            current_id = parent_id
        
        return path

    def get_color_from_gradient(self, normalized_value):
        """
        Get RGBA color from white (low) to black (high) gradient.

        Args:
            normalized_value: Value between 0 and 1 (0=white, 1=black)

        Returns:
            tuple: (r, g, b, a) values between 0 and 1
        """
        # Linear gradient: white (1,1,1) to black (0,0,0)
        grayscale = 1.0 - normalized_value
        return (grayscale, grayscale, grayscale, 0.8)

    def publish_mcts_rewards(self, root_node, top_k=5):
        """
        Publish MCTS node rewards as colored spheres and best path as line strip.

        Args:
            root_node: Root node of the MCTS tree
            top_k: Number of top reward nodes to highlight with larger spheres
        """
        marker_array = MarkerArray()

        # Collect nodes and rewards
        nodes_list = self.collect_nodes_from_tree(root_node)

        if not nodes_list:
            self.node.get_logger().warning('No nodes to visualize in MCTS tree')
            return

        # Extract rewards and normalize
        rewards = np.array([n['reward'] for n in nodes_list])
        reward_min, reward_max = rewards.min(), rewards.max()

        if reward_max > reward_min:
            normalized_rewards = (rewards - reward_min) / (reward_max - reward_min)
        else:
            # All rewards are the same
            normalized_rewards = np.ones_like(rewards) * 0.5

        # Sort nodes by reward to find top K
        sorted_nodes = sorted(enumerate(nodes_list),
                            key=lambda x: x[1]['reward'],
                            reverse=True)
        top_k_indices = set([idx for idx, _ in sorted_nodes[:top_k]])

        # Create sphere markers for all nodes
        for i, node_data in enumerate(nodes_list):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "mcts_reward_nodes"
            marker.id = node_data['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = node_data['x']
            marker.pose.position.y = node_data['y']
            marker.pose.position.z = 0.3  # Elevated above ground

            # Size - larger for top K nodes
            if i in top_k_indices:
                base_scale = 0.45  # Larger size for top nodes
            else:
                base_scale = 0.30  # Regular size

            marker.scale.x = base_scale
            marker.scale.y = base_scale
            marker.scale.z = base_scale

            # Color from white (low) to black (high) gradient
            r, g, b, a = self.get_color_from_gradient(normalized_rewards[i])
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker.lifetime.sec = 0  # Permanent until deleted
            marker_array.markers.append(marker)

        # Find and visualize best path (not really related to MCTS choosing strategy, but tested anyway)
        #best_path = self.find_best_path(nodes_list, root_node.id)
        best_path = []
        if len(best_path) > 1:
            # Create line strip marker for best path
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.node.get_clock().now().to_msg()
            line_marker.ns = "mcts_best_path"
            line_marker.id = 999999  # Unique ID for path
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = 0.05  # Line width

            # Color: bright yellow/gold for visibility
            line_marker.color.r = 1.0
            line_marker.color.g = 0.84
            line_marker.color.b = 0.0
            line_marker.color.a = 0.9

            line_marker.lifetime.sec = 0

            # Add points to line strip
            node_map = {n['id']: n for n in nodes_list}
            for node_id in best_path:
                if node_id in node_map:
                    point = Point()
                    point.x = node_map[node_id]['x']
                    point.y = node_map[node_id]['y']
                    point.z = 0.3  # Same height as spheres
                    line_marker.points.append(point)

            marker_array.markers.append(line_marker)

        # Publish all markers
        self.marker_pub.publish(marker_array)
        self.node.get_logger().info(
            f'Published {len(nodes_list)} MCTS reward nodes, '
            f'top {min(top_k, len(nodes_list))} highlighted, '
            f'best path with {len(best_path)} nodes'
        )

    def clear_visualization(self):
        """Clear all MCTS visualization markers from RViz."""
        marker_array = MarkerArray()

        # Delete all node markers
        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = self.node.get_clock().now().to_msg()
        delete_marker.ns = "mcts_reward_nodes"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Delete path marker
        delete_path = Marker()
        delete_path.header.frame_id = self.frame_id
        delete_path.header.stamp = self.node.get_clock().now().to_msg()
        delete_path.ns = "mcts_best_path"
        delete_path.action = Marker.DELETEALL
        marker_array.markers.append(delete_path)

        self.marker_pub.publish(marker_array)
        self.node.get_logger().info('Cleared MCTS visualisation')


# Standalone usage example
def main(args=None):
    """
    Example standalone node for testing MCTS visualisation.
    In practice, this will be integrated into action_process_no_motion.py
    """
    rclpy.init(args=args)
    node = rclpy.create_node('mcts_reward_visualiser_test')

    node.get_logger().info('MCTS Reward Visualiser test node started')
    node.get_logger().info('This is a test node - actual visualisation happens in action_process_no_motion.py')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
