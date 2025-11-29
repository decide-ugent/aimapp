#!/usr/bin/env python3
"""
Action Selector GUI for AIMAPP Controlled Motion Exploration
Displays possible actions from AIFProcess and allows manual selection
Sends selected action/node as navigation goal to nav2_client_goal_node
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from aimapp_actions.action import AIFProcess
from std_msgs.msg import Int32
import threading


class ActionSelectorGUI(Node):
    def __init__(self):
        super().__init__('action_selector_gui')

        # ROS2 setup - Publisher to send selected node to nav2_client
        self.goal_node_pub = self.create_publisher(Int32, '/nav2_client_goal_pose', 10)

        # Action client to monitor AIF Process
        self.aif_action_client = ActionClient(self, AIFProcess, 'aif_process')

        # Internal state
        self.possible_actions = []
        self.possible_nodes = []
        self.last_result = None

        # GUI setup
        self.root = tk.Tk()
        self.root.title("AIMAPP Action Selector")
        self.root.geometry("900x700")
        self.setup_ui()

        # Timer to periodically check for new action results
        self.check_timer = self.create_timer(1.0, self.check_for_updates)

        self.get_logger().info('Action Selector GUI initialized')
        self.log_message("Action Selector GUI started")
        self.log_message("Waiting for AIF Process action results...")

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

    def check_for_updates(self):
        """Periodically check if AIF action server is available"""
        # This is a placeholder - in practice, the action result will be
        # published by action_process_no_motion.py and we need to subscribe to it
        pass

    def update_actions(self, possible_actions, possible_nodes):
        """Update the available actions (called when new AIF result is received)"""
        self.possible_actions = possible_actions
        self.possible_nodes = possible_nodes

        self.log_message(f"Received {len(self.possible_actions)} possible actions: {self.possible_actions}")
        if self.possible_nodes:
            self.log_message(f"Corresponding nodes: {self.possible_nodes}")

        # Update GUI
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
        header.grid(row=0, column=0, columnspan=4, pady=(5, 15), sticky=tk.W)

        # Create a button for each action
        for idx, action in enumerate(self.possible_actions):
            row = (idx // 4) + 1
            col = idx % 4

            # Get corresponding node if available
            node_info = ""
            node_value = None
            if idx < len(self.possible_nodes):
                node_value = self.possible_nodes[idx]
                node_info = f"\nNode: {node_value}"

            # Create frame for each action
            action_frame = ttk.Frame(self.action_buttons_frame, relief=tk.RIDGE, borderwidth=2, padding=5)
            action_frame.grid(row=row, column=col, padx=8, pady=8, sticky=(tk.W, tk.E, tk.N, tk.S))

            ttk.Label(
                action_frame,
                text=f"Action {action}",
                font=("Arial", 12, "bold")
            ).pack()

            if node_info:
                ttk.Label(
                    action_frame,
                    text=node_info,
                    font=("Arial", 9),
                    foreground="blue"
                ).pack()

            ttk.Button(
                action_frame,
                text="Send Goal",
                command=lambda a=action, n=node_value: self.send_node_goal(a, n),
                width=12
            ).pack(pady=(5, 0))

        self.log_message(f"Updated GUI with {len(self.possible_actions)} action buttons")

    def send_node_goal(self, action, node):
        """Send selected node as navigation goal via the send_next_node_goal.sh script"""
        if node is None:
            self.log_message(f"ERROR: No node mapping for action {action}")
            return

        self.log_message(f"=" * 50)
        self.log_message(f"SENDING GOAL: Action {action} -> Node {node}")
        self.log_message(f"=" * 50)

        # Publish to the topic that nav2_client_goal_node is listening to
        goal_msg = Int32()
        goal_msg.data = node
        self.goal_node_pub.publish(goal_msg)

        self.log_message(f"Published node {node} to /nav2_client_goal_node topic")
        self.log_message(f"Navigation to node {node} initiated!")

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
