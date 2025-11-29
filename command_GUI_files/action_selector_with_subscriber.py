#!/usr/bin/env python3
"""
Action Selector GUI with integrated AIF Process result subscriber
Automatically updates when new action results are available
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray, Float64MultiArray
from geometry_msgs.msg import Point, PoseStamped
from aimapp_actions.action import AIFProcess
import threading


class ActionSelectorGUI(Node):
    def __init__(self):
        super().__init__('action_selector_gui')

        # ROS2 setup - Publisher to send selected pose to nav2_client
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/nav2_client_goal_pose', 10)

        # Subscriber to listen for possible actions from AIF Process
        # We'll publish the result from action_process_no_motion.py to these topics
        self.actions_sub = self.create_subscription(
            Int32MultiArray,
            '/aif_possible_actions',
            self.actions_callback,
            10
        )

        self.nodes_sub = self.create_subscription(
            Int32MultiArray,
            '/aif_possible_nodes',
            self.nodes_callback,
            10
        )

        self.goals_sub = self.create_subscription(
            Float64MultiArray,
            '/aif_reachable_goals',
            self.goals_callback,
            10
        )

        # Internal state
        self.possible_actions = []
        self.possible_nodes = []
        self.reachable_goals = []

        # GUI setup
        self.root = tk.Tk()
        self.root.title("AIMAPP Action Selector")
        self.root.geometry("900x700")
        self.setup_ui()

        self.get_logger().info('Action Selector GUI initialized')
        self.log_message("Action Selector GUI started")
        self.log_message("Waiting for AIF Process action results on /aif_possible_actions and /aif_possible_nodes...")

    def setup_ui(self):
        """Setup the GUI layout"""
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)

        # Title
        title_label = ttk.Label(
            main_frame,
            text="Action Selector - Controlled Motion",
            font=("Arial", 18, "bold")
        )
        title_label.grid(row=0, column=0, pady=(0, 15), sticky=tk.W)

        # Instructions
        instructions = ttk.Label(
            main_frame,
            text="Select an action from the list below to send it as the next navigation goal.",
            font=("Arial", 10),
            foreground="gray"
        )
        instructions.grid(row=1, column=0, pady=(0, 10), sticky=tk.W)

        # Actions section
        actions_frame = ttk.LabelFrame(main_frame, text="Available Actions from AIF Process", padding="15")
        actions_frame.grid(row=2, column=0, pady=(0, 15), sticky=(tk.W, tk.E, tk.N, tk.S))
        actions_frame.columnconfigure(0, weight=1)
        actions_frame.rowconfigure(0, weight=1)

        # Scrollable frame for action buttons
        canvas = tk.Canvas(actions_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(actions_frame, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        self.action_buttons_frame = self.scrollable_frame

        # Initial info label
        self.info_label = ttk.Label(
            self.action_buttons_frame,
            text="Waiting for action results from AIF Process...\n\n"
                 "Execute an AIF Process action in the 'AIF-Action' terminal\n"
                 "to see available next actions here.",
            font=("Arial", 11),
            foreground="gray",
            justify=tk.CENTER
        )
        self.info_label.grid(row=0, column=0, pady=30)

        # Log section
        log_frame = ttk.LabelFrame(main_frame, text="Activity Log", padding="10")
        log_frame.grid(row=3, column=0, pady=(0, 10), sticky=(tk.W, tk.E))
        log_frame.columnconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=10,
            wrap=tk.WORD,
            font=("Courier", 9),
            bg="#f5f5f5"
        )
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        self.log_text.config(state=tk.DISABLED)

        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, pady=(5, 0))

        ttk.Button(
            button_frame,
            text="Clear Log",
            command=self.clear_log,
            width=15
        ).grid(row=0, column=0, padx=5)

        ttk.Button(
            button_frame,
            text="Exit",
            command=self.on_closing,
            width=15
        ).grid(row=0, column=1, padx=5)

    def log_message(self, message):
        """Add a message to the log window"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.get_logger().info(message)

    def clear_log(self):
        """Clear the log window"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)

    def actions_callback(self, msg):
        """Callback when new possible actions are received"""
        self.possible_actions = list(msg.data)
        self.log_message(f"Received {len(self.possible_actions)} possible actions: {self.possible_actions}")
        self.check_and_update_gui()

    def nodes_callback(self, msg):
        """Callback when new possible nodes are received"""
        self.possible_nodes = list(msg.data)
        self.log_message(f"Received {len(self.possible_nodes)} possible nodes: {self.possible_nodes}")
        self.check_and_update_gui()

    def goals_callback(self, msg):
        """Extract reachable goals coordinates (flattened x,y pairs)"""
        # Data is [x1, y1, x2, y2, ...], convert to list of (x,y) tuples
        coords = list(msg.data)
        self.reachable_goals = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
        self.log_message(f"Received {len(self.reachable_goals)} reachable goal coordinates")
        self.check_and_update_gui()

    def check_and_update_gui(self):
        """Update GUI only when we have all three pieces of data"""
        if (self.possible_actions and self.possible_nodes and self.reachable_goals and
            len(self.possible_actions) == len(self.possible_nodes) == len(self.reachable_goals)):
            self.root.after(0, self.update_action_buttons)

    def update_action_buttons(self):
        """Update the GUI with buttons for each possible action"""
        # Clear existing widgets
        for widget in self.action_buttons_frame.winfo_children():
            widget.destroy()

        if not self.possible_actions:
            self.info_label = ttk.Label(
                self.action_buttons_frame,
                text="No actions available yet.\n\n"
                     "Execute an AIF Process action to see available next actions.",
                font=("Arial", 11),
                foreground="orange",
                justify=tk.CENTER
            )
            self.info_label.grid(row=0, column=0, pady=30)
            return

        # Header
        header = ttk.Label(
            self.action_buttons_frame,
            text=f"Select one of the {len(self.possible_actions)} available actions to send as navigation goal:",
            font=("Arial", 11, "bold")
        )
        header.grid(row=0, column=0, columnspan=3, pady=(5, 15), sticky=tk.W)

        # Create a button for each action
        for idx, action in enumerate(self.possible_actions):
            row = (idx // 3) + 1
            col = idx % 3

            # Get corresponding node and goal
            node_value = None
            goal_coords = None

            if idx < len(self.possible_nodes):
                node_value = self.possible_nodes[idx]

            if idx < len(self.reachable_goals):
                goal_coords = self.reachable_goals[idx]

            # Create frame for each action
            action_frame = ttk.Frame(self.action_buttons_frame, relief=tk.RIDGE, borderwidth=2, padding=8)
            action_frame.grid(row=row, column=col, padx=8, pady=8, sticky=(tk.W, tk.E, tk.N, tk.S))

            # Action ID
            ttk.Label(
                action_frame,
                text=f"Action: {action}",
                font=("Arial", 13, "bold"),
                foreground="#1a5490"
            ).pack(anchor=tk.W)

            # Node ID
            if node_value is not None:
                ttk.Label(
                    action_frame,
                    text=f"Node: {node_value}",
                    font=("Arial", 10),
                    foreground="#2d5016"
                ).pack(anchor=tk.W, pady=(3, 0))

            # Goal coordinates
            if goal_coords is not None:
                ttk.Label(
                    action_frame,
                    text=f"Goal: ({goal_coords[0]:.2f}, {goal_coords[1]:.2f})",
                    font=("Arial", 9),
                    foreground="#505050"
                ).pack(anchor=tk.W, pady=(2, 0))

            # Separator
            ttk.Separator(action_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(5, 5))

            # Send button
            ttk.Button(
                action_frame,
                text="Send Goal",
                command=lambda a=action, n=node_value, g=goal_coords: self.send_node_goal(a, n, g),
                width=14
            ).pack(pady=(0, 0))

        self.log_message(f"Updated GUI with {len(self.possible_actions)} action options")

    def send_node_goal(self, action, node, goal_coords):
        """Send selected pose as navigation goal"""
        if goal_coords is None:
            self.log_message(f"ERROR: No goal coordinates for action {action}")
            return

        self.log_message(f"=" * 50)
        self.log_message(f"SENDING GOAL:")
        self.log_message(f"  Action: {action}")
        self.log_message(f"  Node: {node}")
        self.log_message(f"  Goal Coordinates: ({goal_coords[0]:.2f}, {goal_coords[1]:.2f})")
        self.log_message(f"=" * 50)

        # Publish PoseStamped to the topic that nav2_client.py is listening to
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_coords[0]
        goal_msg.pose.position.y = goal_coords[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No rotation

        self.goal_pose_pub.publish(goal_msg)

        self.log_message(f"Published pose ({goal_coords[0]:.2f}, {goal_coords[1]:.2f}) to /nav2_client_goal_pose topic")
        self.log_message(f"Navigation to pose initiated!")

    def on_closing(self):
        """Handle window closing"""
        self.get_logger().info("Shutting down Action Selector GUI")
        self.root.quit()
        self.root.destroy()

    def spin_gui(self):
        """Main loop to process ROS2 callbacks and GUI events"""
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

        # Start ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()

        # Start GUI main loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    gui_node = ActionSelectorGUI()

    try:
        gui_node.spin_gui()
    except Exception as e:
        gui_node.get_logger().error(f'Error in GUI: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
