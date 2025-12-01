#!/usr/bin/env python3
"""
Visual User Interface for AIMAPP Exploration Mode Selection
Allows choosing between Controlled Motion and Fully Autonomous exploration
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import sys

class ExplorationLauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AIMAPP Exploration Launcher")
        self.root.geometry("750x650")
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
            text="AIMAPP Exploration Mode Selector",
            font=("Arial", 18, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # Subtitle
        subtitle_label = ttk.Label(
            main_frame,
            text="Choose your exploration mode for the Husarion robot",
            font=("Arial", 10)
        )
        subtitle_label.grid(row=1, column=0, columnspan=2, pady=(0, 30))

        # Mode selection variable
        self.mode_var = tk.StringVar(value="controlled")

        # Controlled Motion Frame
        controlled_frame = ttk.LabelFrame(
            main_frame,
            text="Controlled Motion Exploration",
            padding="15"
        )
        controlled_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15), sticky=(tk.W, tk.E))

        ttk.Radiobutton(
            controlled_frame,
            text="Manual Goal Control",
            variable=self.mode_var,
            value="controlled"
        ).grid(row=0, column=0, sticky=tk.W, pady=(0, 10))

        controlled_desc = tk.Text(
            controlled_frame,
            height=6,
            width=70,
            wrap=tk.WORD,
            font=("Arial", 9),
            bg="#f0f0f0",
            relief=tk.FLAT
        )
        controlled_desc.grid(row=1, column=0, pady=(0, 10))
        controlled_desc.insert("1.0",
            "• Ideal for areas where obstacles can't be detected by Lidar\n"
            "• No bumpers on the Husarion robot\n"
            "• Action process dissociated from model process\n"
            "• Launches: launch_model_as_action_robot.sh\n"
            "• Includes terminals for manual goal sending (AIF Process & Nav2 goals)\n"
            "• Requires user interaction to send goals to adjacent nodes"
        )
        controlled_desc.config(state=tk.DISABLED)

        # Test ID input for controlled mode
        test_id_frame = ttk.Frame(controlled_frame)
        test_id_frame.grid(row=2, column=0, pady=(5, 0), sticky=tk.W)

        ttk.Label(
            test_id_frame,
            text="Test ID (optional):",
            font=("Arial", 9)
        ).grid(row=0, column=0, padx=(0, 10))

        self.test_id_var = tk.StringVar(value="None")
        test_id_entry = ttk.Entry(
            test_id_frame,
            textvariable=self.test_id_var,
            width=15
        )
        test_id_entry.grid(row=0, column=1)

        ttk.Label(
            test_id_frame,
            text="(Use existing model, e.g., '5')",
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, padx=(10, 0))

        # Autonomous Frame
        autonomous_frame = ttk.LabelFrame(
            main_frame,
            text="Fully Autonomous Exploration",
            padding="15"
        )
        autonomous_frame.grid(row=3, column=0, columnspan=2, pady=(0, 20), sticky=(tk.W, tk.E))

        ttk.Radiobutton(
            autonomous_frame,
            text="Full Autonomy",
            variable=self.mode_var,
            value="autonomous"
        ).grid(row=0, column=0, sticky=tk.W, pady=(0, 10))

        autonomous_desc = tk.Text(
            autonomous_frame,
            height=6,
            width=70,
            wrap=tk.WORD,
            font=("Arial", 9),
            bg="#f0f0f0",
            relief=tk.FLAT
        )
        autonomous_desc.grid(row=1, column=0, pady=(0, 10))
        autonomous_desc.insert("1.0",
            "• Complete autonomous exploration without manual intervention\n"
            "• Agent makes all decisions independently\n"
            "• Best for well-defined environments with reliable Lidar coverage"
        )
        autonomous_desc.config(state=tk.DISABLED)

        # Button frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=(10, 0))

        # Launch button
        launch_btn = ttk.Button(
            button_frame,
            text="Launch Exploration",
            command=self.launch_exploration,
            width=25
        )
        launch_btn.grid(row=0, column=0, padx=5)

        # Exit button
        exit_btn = ttk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit,
            width=25
        )
        exit_btn.grid(row=0, column=1, padx=5)

        # Status label
        self.status_label = ttk.Label(
            main_frame,
            text="Ready to launch",
            font=("Arial", 9, "italic"),
            foreground="gray"
        )
        self.status_label.grid(row=5, column=0, columnspan=2, pady=(20, 0))

    def launch_exploration(self):
        mode = self.mode_var.get()

        if mode == "controlled":
            script_name = "launch_model_as_action_robot.sh"
            mode_name = "Controlled Motion Exploration"
            # Get test_id for controlled mode
            test_id = self.test_id_var.get().strip()
            if not test_id or test_id.lower() == "none":
                test_id = "None"
        else:
            script_name = "launch_autonomous_exploration.sh"
            mode_name = "Fully Autonomous Exploration"
            test_id = None

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
        if test_id and mode == "controlled":
            confirm_msg += f"\nTest ID: {test_id}\n"
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
            if mode == "controlled" and test_id:
                # Pass test_id as argument for controlled mode
                subprocess.Popen(['bash', script_path, test_id], cwd=self.ros_ws_dir)
            else:
                subprocess.Popen(['bash', script_path], cwd=self.ros_ws_dir)

            # Update status
            self.status_label.config(
                text=f"✓ {mode_name} launched successfully!",
                foreground="green"
            )

            # Show success message
            messagebox.showinfo(
                "Success",
                f"{mode_name} has been launched!\n\n"
                f"Check the terminal windows for logs and status."
            )

        except Exception as e:
            messagebox.showerror(
                "Launch Error",
                f"Failed to launch exploration:\n\n{str(e)}"
            )
            self.status_label.config(
                text="Launch failed",
                foreground="red"
            )

def main():
    root = tk.Tk()
    app = ExplorationLauncherGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
