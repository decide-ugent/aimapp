#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np

class StateTransitionVisualizer(Node):
    def __init__(self, B: np.ndarray, state_mapping: dict):
        super().__init__('state_transition_visualizer')
        
        self.publisher = self.create_publisher(MarkerArray, 'state_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)  # Publish every 1 second
        
        self.B = B
        self.state_mapping = state_mapping
        self.color_map = self.get_cmap()

    def get_cmap(self):
        """ Returns a discrete colormap for observations """
        custom_colors = np.array([
            [255, 255, 255],  # white
            [255, 0, 0],      # red
        ]) / 256  # Normalize to 0-1
        return custom_colors

    def create_state_marker(self, state_id, x, y, color):
        """ Create a sphere marker for each state """
        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        marker.ns = "states"
        marker.id = state_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Keep in 2D plane

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Fully visible

        return marker

    def create_transition_marker(self, marker_id, start_pos, end_pos, weight):
        """ Create a line marker for state transitions """
        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        marker.ns = "transitions"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = weight * 0.1  # Line width proportional to transition weight
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = [
            {'x': start_pos[0], 'y': start_pos[1], 'z': 0.0},
            {'x': end_pos[0], 'y': end_pos[1], 'z': 0.0}
        ]

        return marker

    def publish_markers(self):
        """ Publish states and transitions as a MarkerArray in ROS2 """
        marker_array = MarkerArray()
        state_id_map = {}  # Map state number to position

        # Step 1: Create state markers
        unique_obs = np.sort(list({v['ob'] for v in self.state_mapping.values()}))
        ob_to_color = {ob: self.color_map[i] for i, ob in enumerate(unique_obs)}

        for (x, y), data in self.state_mapping.items():
            state_id = data['state']
            color = ob_to_color.get(data['ob'], [1, 1, 1])  # Default to white
            state_marker = self.create_state_marker(state_id, x, y, color)
            marker_array.markers.append(state_marker)
            state_id_map[state_id] = (x, y)

        # Step 2: Create transition markers
        marker_id = 100  # Start ID for transitions
        for action in range(self.B.shape[2]):  # Iterate over actions
            for next_state in range(self.B.shape[0]):
                for prev_state in range(self.B.shape[1]):
                    weight = self.B[next_state, prev_state, action]
                    if weight > 0.005 and prev_state in state_id_map and next_state in state_id_map:
                        start_pos = state_id_map[prev_state]
                        end_pos = state_id_map[next_state]
                        transition_marker = self.create_transition_marker(marker_id, start_pos, end_pos, weight)
                        marker_array.markers.append(transition_marker)
                        marker_id += 1

        # Publish the MarkerArray
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = StateTransitionVisualizer(ours.B[0], ours.agent_state_mapping)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
