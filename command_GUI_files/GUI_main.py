#!/usr/bin/env python3
"""
Main GUI for AIMAPP - Choose between Exploration and Goal Reaching
"""

import tkinter as tk
from tkinter import ttk
import subprocess
import os
import sys

class MainGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AIMAPP Main Launcher")
        self.root.geometry("600x400")
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

        # Configure grid weights for centering
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)

        # Title
        title_label = ttk.Label(
            main_frame,
            text="AIMAPP Robot Control System",
            font=("Arial", 20, "bold")
        )
        title_label.grid(row=0, column=0, pady=(0, 10))

        # Subtitle
        subtitle_label = ttk.Label(
            main_frame,
            text="Select Operation Mode",
            font=("Arial", 12)
        )
        subtitle_label.grid(row=1, column=0, pady=(0, 40))

        # Button frame for centering
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=2, column=0)

        # Exploration button
        exploration_btn = ttk.Button(
            button_frame,
            text="🗺️  Exploration Mode",
            command=self.launch_exploration_gui,
            width=30
        )
        exploration_btn.grid(row=0, column=0, pady=15, padx=20)

        # Exploration description
        exploration_desc = ttk.Label(
            button_frame,
            text="Explore unknown environments autonomously or with manual control",
            font=("Arial", 9),
            foreground="gray"
        )
        exploration_desc.grid(row=1, column=0, pady=(0, 30))

        # Goal Reaching button
        goal_btn = ttk.Button(
            button_frame,
            text="🎯  Goal Reaching Mode",
            command=self.launch_goal_gui,
            width=30
        )
        goal_btn.grid(row=2, column=0, pady=15, padx=20)

        # Goal description
        goal_desc = ttk.Label(
            button_frame,
            text="Navigate to specific observations or poses in the model",
            font=("Arial", 9),
            foreground="gray"
        )
        goal_desc.grid(row=3, column=0, pady=(0, 30))

        # Exit button
        exit_btn = ttk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit,
            width=30
        )
        exit_btn.grid(row=4, column=0, pady=15, padx=20)

    def launch_exploration_gui(self):
        """Launch the exploration mode GUI"""
        exploration_script = os.path.join(self.script_dir, "launch_exploration_gui.py")

        if not os.path.exists(exploration_script):
            tk.messagebox.showerror(
                "Error",
                f"Exploration GUI script not found:\n{exploration_script}"
            )
            return

        try:
            # Close this window
            self.root.destroy()

            # Launch exploration GUI
            subprocess.Popen(['python3', exploration_script], cwd=self.ros_ws_dir)
        except Exception as e:
            tk.messagebox.showerror(
                "Launch Error",
                f"Failed to launch exploration GUI:\n{str(e)}"
            )

    def launch_goal_gui(self):
        """Launch the goal reaching mode GUI"""
        goal_script = os.path.join(self.script_dir, "launch_goal_gui.py")

        if not os.path.exists(goal_script):
            tk.messagebox.showerror(
                "Error",
                f"Goal Reaching GUI script not found:\n{goal_script}"
            )
            return

        try:
            # Close this window
            self.root.destroy()

            # Launch goal reaching GUI
            subprocess.Popen(['python3', goal_script], cwd=self.ros_ws_dir)
        except Exception as e:
            tk.messagebox.showerror(
                "Launch Error",
                f"Failed to launch goal reaching GUI:\n{str(e)}"
            )

def main():
    root = tk.Tk()
    app = MainGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
