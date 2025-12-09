#!/usr/bin/env python3
"""
Visual User Interface for AIMAPP Goal Reaching Mode
Allows choosing between Controlled and Autonomous goal reaching with goal specification
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import sys

class GoalReachingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AIMAPP Goal Reaching Launcher")
        self.root.geometry("750x700")
        self.root.resizable(True, True)

        # Set up the UI
        self.setup_ui()

        # Get the ROS workspace directory (current working directory)
        self.ros_ws_dir = os.getcwd()
        # Scripts are located in src/aimapp/command_GUI_files relative to ROS workspace
        self.script_dir = os.path.join(self.ros_ws_dir, "src/aimapp/command_GUI_files")

    def setup_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Title
        title_label = ttk.Label(
            main_frame,
            text="AIMAPP Goal Reaching Mode",
            font=("Arial", 18, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # Subtitle
        subtitle_label = ttk.Label(
            main_frame,
            text="Navigate to a specific goal observation or pose",
            font=("Arial", 10)
        )
        subtitle_label.grid(row=1, column=0, columnspan=2, pady=(0, 30))

        # Goal Specification Frame
        goal_frame = ttk.LabelFrame(
            main_frame,
            text="Goal Specification",
            padding="15"
        )
        goal_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15), sticky=(tk.W, tk.E))

        # Goal observation ID
        goal_ob_frame = ttk.Frame(goal_frame)
        goal_ob_frame.grid(row=0, column=0, pady=(0, 10), sticky=tk.W)
        ttk.Label(
            goal_ob_frame,
            text="Goal Observation ID:",
            font=("Arial", 9)
        ).grid(row=0, column=0, padx=(0, 10))
        self.goal_ob_var = tk.StringVar(value="-1")
        ttk.Entry(
            goal_ob_frame,
            textvariable=self.goal_ob_var,
            width=10
        ).grid(row=0, column=1)
        ttk.Label(
            goal_ob_frame,
            text="(-1 = not specified)",
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, padx=(10, 0))

        # Goal pose ID
        goal_pose_frame = ttk.Frame(goal_frame)
        goal_pose_frame.grid(row=1, column=0, pady=(0, 10), sticky=tk.W)
        ttk.Label(
            goal_pose_frame,
            text="Goal Pose ID:",
            font=("Arial", 9)
        ).grid(row=0, column=0, padx=(0, 10))
        self.goal_pose_var = tk.StringVar(value="-1")
        ttk.Entry(
            goal_pose_frame,
            textvariable=self.goal_pose_var,
            width=10
        ).grid(row=0, column=1)
        ttk.Label(
            goal_pose_frame,
            text="(-1 = not specified)",
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, padx=(10, 0))

        # Test ID input
        test_id_frame = ttk.Frame(goal_frame)
        test_id_frame.grid(row=2, column=0, pady=(5, 0), sticky=tk.W)
        ttk.Label(
            test_id_frame,
            text="Test ID (preferred):",
            font=("Arial", 9, "bold")
        ).grid(row=0, column=0, padx=(0, 10))
        self.test_id_var = tk.StringVar(value="")
        ttk.Entry(
            test_id_frame,
            textvariable=self.test_id_var,
            width=15
        ).grid(row=0, column=1)
        ttk.Label(
            test_id_frame,
            text="(Load existing model, e.g., '5')",
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, padx=(10, 0))

        # Starting node ID input
        start_node_frame = ttk.Frame(goal_frame)
        start_node_frame.grid(row=3, column=0, pady=(5, 0), sticky=tk.W)
        ttk.Label(
            start_node_frame,
            text="Starting Node ID:",
            font=("Arial", 9)
        ).grid(row=0, column=0, padx=(0, 10))
        self.start_node_var = tk.StringVar(value="-1")
        ttk.Entry(
            start_node_frame,
            textvariable=self.start_node_var,
            width=10
        ).grid(row=0, column=1)
        ttk.Label(
            start_node_frame,
            text="(-1 = use latest node in model)",
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, padx=(10, 0))

        # Mode selection variable
        self.mode_var = tk.StringVar(value="controlled")

        # Controlled Motion Frame
        controlled_frame = ttk.LabelFrame(
            main_frame,
            text="Controlled Goal Reaching",
            padding="15"
        )
        controlled_frame.grid(row=3, column=0, columnspan=2, pady=(15, 15), sticky=(tk.W, tk.E))

        ttk.Radiobutton(
            controlled_frame,
            text="Manual Control to Goal",
            variable=self.mode_var,
            value="controlled"
        ).grid(row=0, column=0, sticky=tk.W, pady=(0, 10))

        controlled_desc = tk.Text(
            controlled_frame,
            height=4,
            width=70,
            wrap=tk.WORD,
            font=("Arial", 9),
            bg="#f0f0f0",
            relief=tk.FLAT
        )
        controlled_desc.grid(row=1, column=0, pady=(0, 10))
        controlled_desc.insert("1.0",
            "• Action process dissociated from model process\n"
            "• Requires user interaction to send goals to adjacent nodes\n"
            "• Launches: launch_model_as_action_robot.sh with goal parameters\n"
            "• Uses action_process_no_motion.py"
        )
        controlled_desc.config(state=tk.DISABLED)

        # Autonomous Frame
        autonomous_frame = ttk.LabelFrame(
            main_frame,
            text="Fully Autonomous Goal Reaching",
            padding="15"
        )
        autonomous_frame.grid(row=4, column=0, columnspan=2, pady=(0, 20), sticky=(tk.W, tk.E))

        ttk.Radiobutton(
            autonomous_frame,
            text="Full Autonomy to Goal",
            variable=self.mode_var,
            value="autonomous"
        ).grid(row=0, column=0, sticky=tk.W, pady=(0, 10))

        autonomous_desc = tk.Text(
            autonomous_frame,
            height=3,
            width=70,
            wrap=tk.WORD,
            font=("Arial", 9),
            bg="#f0f0f0",
            relief=tk.FLAT
        )
        autonomous_desc.grid(row=1, column=0, pady=(0, 10))
        autonomous_desc.insert("1.0",
            "• Complete autonomous navigation to goal\n"
            "• Agent makes all decisions independently\n"
            "• Uses main.py with goal-oriented navigation"
        )
        autonomous_desc.config(state=tk.DISABLED)

        # Button frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=(10, 0))

        # Launch button
        launch_btn = ttk.Button(
            button_frame,
            text="Launch Goal Reaching",
            command=self.launch_goal_reaching,
            width=25
        )
        launch_btn.grid(row=0, column=0, padx=5)

        # Back button
        back_btn = ttk.Button(
            button_frame,
            text="Back to Main Menu",
            command=self.back_to_main,
            width=25
        )
        back_btn.grid(row=0, column=1, padx=5)

        # Exit button
        exit_btn = ttk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit,
            width=25
        )
        exit_btn.grid(row=0, column=2, padx=5)

        # Status label
        self.status_label = ttk.Label(
            main_frame,
            text="Ready to launch",
            font=("Arial", 9, "italic"),
            foreground="gray"
        )
        self.status_label.grid(row=6, column=0, columnspan=2, pady=(20, 0))

    def launch_goal_reaching(self):
        mode = self.mode_var.get()

        # Get goal parameters
        goal_ob_id = self.goal_ob_var.get().strip()
        goal_pose_id = self.goal_pose_var.get().strip()
        start_node_id = self.start_node_var.get().strip()
        test_id = self.test_id_var.get().strip()

        # Validate test_id
        if not test_id or test_id.lower() == "none":
            messagebox.showerror(
                "Missing Test ID",
                "Test ID is required for goal reaching mode.\n"
                "Please enter an existing test ID to load the model."
            )
            return

        # Validate goal IDs and start node ID are integers
        try:
            goal_ob_int = int(goal_ob_id)
            goal_pose_int = int(goal_pose_id)
            start_node_int = int(start_node_id)
        except ValueError:
            messagebox.showerror(
                "Invalid IDs",
                "Please enter valid integer values:\n"
                "- Goal Observation ID: integer (e.g., 5 or -1)\n"
                "- Goal Pose ID: integer (e.g., 10 or -1)\n"
                "- Starting Node ID: integer (e.g., 0 or -1)"
            )
            return

        # Check that at least one goal is specified
        if goal_ob_int == -1 and goal_pose_int == -1:
            messagebox.showwarning(
                "No Goal Specified",
                "At least one goal (observation or pose) must be specified.\n"
                "Please enter a valid goal ID (>= 0)."
            )
            return

        if mode == "controlled":
            script_name = "launch_model_as_action_robot.sh"
            mode_name = "Controlled Goal Reaching"
        else:
            script_name = "launch_autonomous_exploration.sh"  # Will use main.py with goals
            mode_name = "Fully Autonomous Goal Reaching"

        script_path = os.path.join(self.script_dir, script_name)

        # Check if script exists
        if not os.path.exists(script_path):
            messagebox.showerror(
                "Error",
                f"Launch script not found:\n{script_path}\n\n"
                f"Please ensure the script exists in the correct location."
            )
            return

        # Prepare confirm message
        confirm_msg = f"Launch {mode_name}?\n\n"
        confirm_msg += f"This will execute:\n{script_name}\n"
        confirm_msg += f"\nTest ID: {test_id}\n"
        confirm_msg += f"Starting Node ID: {start_node_id}\n"
        confirm_msg += f"Goal Observation ID: {goal_ob_id}\n"
        confirm_msg += f"Goal Pose ID: {goal_pose_id}\n"
        confirm_msg += f"\nMultiple terminal windows will open."

        # Confirm launch
        confirm = messagebox.askyesno("Confirm Launch", confirm_msg)

        if not confirm:
            return

        # Update status
        self.status_label.config(
            text=f"Launching {mode_name}...",
            foreground="blue"
        )
        self.root.update()

        try:
            # Make script executable
            os.chmod(script_path, 0o755)

            # Launch the script from ROS workspace directory
            # Pass: test_id goal_ob_id goal_pose_id start_node_id
            subprocess.Popen(['bash', script_path, test_id, goal_ob_id, goal_pose_id, start_node_id],
                           cwd=self.ros_ws_dir)

            # Update status
            self.status_label.config(
                text=f"✓ {mode_name} launched successfully!",
                foreground="green"
            )

            # Show success message
            messagebox.showinfo(
                "Success",
                f"{mode_name} has been launched!\n\n"
                f"The robot will navigate to the specified goal.\n"
                f"Check the terminal windows for logs and status."
            )

        except Exception as e:
            messagebox.showerror(
                "Launch Error",
                f"Failed to launch goal reaching:\n\n{str(e)}"
            )
            self.status_label.config(
                text="Launch failed",
                foreground="red"
            )

    def back_to_main(self):
        """Go back to main menu"""
        main_script = os.path.join(self.script_dir, "GUI_main.py")

        try:
            # Close this window
            self.root.destroy()

            # Launch main GUI
            subprocess.Popen(['python3', main_script], cwd=self.ros_ws_dir)
        except Exception as e:
            messagebox.showerror(
                "Error",
                f"Failed to return to main menu:\n{str(e)}"
            )

def main():
    root = tk.Tk()
    app = GoalReachingGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
