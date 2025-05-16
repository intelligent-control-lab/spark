import os
import time
import numpy as np
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import argparse
import signal


def get_latest_event_file(log_dir):
    """Get the latest TensorBoard event file"""
    event_files = [f for f in os.listdir(log_dir) if f.startswith("events.out.tfevents")]
    if not event_files:
        return None
    return os.path.join(log_dir, sorted(event_files)[-1])  # Pick latest file

def read_tensorboard_tags(log_file):
    """Read all available scalar tags from a TensorBoardX log file"""
    event_acc = EventAccumulator(log_file)
    event_acc.Reload()
    return event_acc.Tags()["scalars"]

def read_tensorboard_scalars(log_file, tags):
    """Read scalar values from TensorBoardX log file"""
    event_acc = EventAccumulator(log_file)
    event_acc.Reload()
    
    scalar_data = {}
    for tag in event_acc.Tags()["scalars"]:
        if tag in tags or "phi" in tag or "traj" in tag:
            events = event_acc.Scalars(tag)
            steps = [e.step for e in events]
            values = [e.value for e in events]
            scalar_data[tag] = (steps, values)
    
    return scalar_data

import numpy as np

class Plotter():
    def __init__(self, ax, tag, window_size):
        self.ax = ax
        self.tag = tag  # Fix: Ensure `tag` is assigned correctly
        self.window_size = window_size  # Fix: Store window_size
        
    
    def plot(self, data):
        if "phi" in self.tag:
            self.plot_phi(data)
        elif "traj" in self.tag:
            self.window_size = 100
            self.plot_traj(data)
    
    def plot_phi(self, data):
        if self.tag in data:
            steps, values = data[self.tag]
            if len(steps) > 0 and len(values) > 0:
                self.ax.clear()
                self.line, = self.ax.plot([], [], label=self.tag)
                
                if len(steps) >= self.window_size:  # Fix: Use `self.window_size`
                    xlim_min = steps[-self.window_size]
                    xlim_max = steps[-1]
                else:
                    xlim_min = 0
                    xlim_max = self.window_size
                
                steps = steps[xlim_min: xlim_max]
                values = values[xlim_min: xlim_max]
                self.line.set_xdata(steps)
                self.line.set_ydata(values)
                values_min = np.min(values)
                values_max = np.max(values)
                self.ax.set_ylim(np.minimum(values_min - 0.01, -0.05), np.maximum(values_max + 0.01, 0.05))
                self.ax.set_xlim(xlim_min, xlim_max)
                self.ax.hlines(y=0, xmin = xlim_min, xmax = xlim_max, color='black', linestyle='--', linewidth=0.5)
                
                self.ax.legend()
                self.ax.relim()
                self.ax.autoscale_view()
    
    def plot_traj(self, data):
        tag_split = self.tag.split("_")
        name = '_'.join(tag_split[1:])
        phi0_name = "phi0_" + name
        phi0dot_name = "phi0dot_" + name
        phi_k_name = "phi_k_" + name
        if phi0_name in data and phi0dot_name in data and phi_k_name in data:
            phi0_steps, phi0_values = data[phi0_name]
            phi0dot_steps, phi0dot_values = data[phi0dot_name]
            _, phi_k_values = data[phi_k_name]
            phi_k = phi_k_values[-1]
            if len(phi0_steps) == 0 or len(phi0dot_steps) == 0:
                return  # Fix: Handle empty data cases

            # Ensure `step` is a valid index
            step = min(len(phi0_values), len(phi0dot_values))  

            phi0_values = phi0_values[:step]
            phi0dot_values = phi0dot_values[:step]

            if step >= 1:
                if len(phi0_values) >= self.window_size:
                    phi0_values = phi0_values[-self.window_size:]
                    phi0dot_values = phi0dot_values[-self.window_size:]

                # Fix: Ensure proper axis limits (commented out for now)
                # self.ax.set_xlim(-np.abs(phi0_values[-1]) * 1.2, np.abs(phi0_values[-1]) * 1.2)
                # self.ax.set_ylim(-np.abs(phi0dot_values[-1]) * 1.2, np.abs(phi0dot_values[-1]) * 1.2)
                self.ax.clear()
                self.line, = self.ax.plot([], [], label=self.tag)
                self.line.set_xdata(phi0_values)
                self.line.set_ydata(phi0dot_values)
                self.ax.set_xlim(-0.2, 0.2)
                self.ax.set_ylim(-0.2, 0.2)
                self.ax.vlines(x=0, ymin = -100,ymax = 100, color='black', linestyle='--', linewidth=0.5)
                self.ax.hlines(y=0, xmin = -100,xmax = 100, color='black', linestyle='--', linewidth=0.5)
                self.ax.scatter(phi0_values[-1], phi0dot_values[-1], color='r', s=10)
                
                # Define x values
                
                if phi_k != 0:
                    x = np.linspace(-10, 10, 100)
                    self.ax.plot(x, - x/phi_k, color='red', linestyle='--', label="phi = 0")
                self.ax.legend()
                self.ax.relim()
                self.ax.autoscale_view()

        

# Signal handler for SIGINT (Ctrl+C)
def signal_handler(sig, frame):
    global shutdown_flag
    print("\nReceived SIGINT, shutting down...")
    shutdown_flag = True



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot tags with a directory path")

    # Define the arguments
    parser.add_argument('log_path', type=str, help="Directory path for plotting")
    parser.add_argument('tags', type=str, nargs='+', help="Tags to plot")

    # Parse the arguments
    args = parser.parse_args()

    # Access the directory path and tags
    log_path = args.log_path
    tags = args.tags

    window_size = 1000
    # Set up a flag to handle graceful shutdown
    shutdown_flag = False
    # Register the signal handler for SIGINT
    signal.signal(signal.SIGINT, signal_handler)

    if log_path:
        all_tags = read_tensorboard_tags(log_path)
        print("Available Tags:", all_tags)
        
        user_tags = args.tags
        print("Selected Tags:", user_tags)
        if not user_tags:
            print("No valid tags selected. Exiting.")
        else:
            num_plots = len(user_tags)
            
            rows = int(np.ceil(num_plots / 2))
            cols = int(np.ceil(num_plots / rows))
            # Create subplots based on user-selected tags
            fig, axes = plt.subplots(rows, cols, figsize=(2*cols, 2*rows))
            axes = axes.flatten()
            plt.ion()  # Interactive mode
            plt.subplots_adjust(hspace=0.2, wspace=0.2)
            fig_manager = plt.get_current_fig_manager()
            fig_manager.window.wm_geometry("+3000+300")  
            
            
            plotter_list = []
            for i, tag in enumerate(user_tags):
                plotter_list.append(Plotter(axes[i], tag, window_size))
            # Real-time update loop
            while not shutdown_flag:
                scalar_data = read_tensorboard_scalars(log_path, user_tags)
                for plotter in plotter_list:
                    plotter.plot(scalar_data)
                plt.draw()
                plt.pause(0.001)  # Update every second
                time.sleep(0.001)

    else:
        print("No TensorBoard event file found.")
