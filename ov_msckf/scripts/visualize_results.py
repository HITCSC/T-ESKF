#!/usr/bin/env python3
"""
Simple visualization script for dual-wheel robot simulation results
"""
import numpy as np
import matplotlib.pyplot as plt
import argparse
import csv
import os

def load_simulation_data(filename):
    """Load simulation data from CSV file"""
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found")
        return None
    
    data = {
        'timestamp': [],
        'robot_x': [], 'robot_y': [], 'robot_theta': [], 'robot_v': [], 'robot_omega': [],
        'desired_x': [], 'desired_y': [], 'desired_theta': [], 'desired_v': [], 'desired_omega': [],
        'linear_accel': [], 'angular_accel': []
    }
    
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            for key in data.keys():
                data[key].append(float(row[key]))
    
    # Convert to numpy arrays
    for key in data.keys():
        data[key] = np.array(data[key])
    
    return data

def plot_trajectory_comparison(data, title="Robot Trajectory"):
    """Plot robot trajectory vs desired trajectory"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Trajectory plot
    ax1.plot(data['desired_x'], data['desired_y'], 'b--', linewidth=2, label='Desired Path', alpha=0.8)
    ax1.plot(data['robot_x'], data['robot_y'], 'r-', linewidth=1.5, label='Robot Path')
    ax1.scatter(data['robot_x'][0], data['robot_y'][0], c='green', s=100, marker='o', label='Start')
    ax1.scatter(data['robot_x'][-1], data['robot_y'][-1], c='red', s=100, marker='x', label='End')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title(f'{title} - Path Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Error plot
    position_error = np.sqrt((data['robot_x'] - data['desired_x'])**2 + 
                           (data['robot_y'] - data['desired_y'])**2)
    ax2.plot(data['timestamp'], position_error, 'r-', linewidth=1.5, label='Position Error')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_title(f'{title} - Tracking Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_control_signals(data, title="Control Signals"):
    """Plot control signals over time"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Linear acceleration
    ax1.plot(data['timestamp'], data['linear_accel'], 'b-', linewidth=1.5)
    ax1.set_ylabel('Linear Acceleration (m/s²)')
    ax1.set_title(f'{title} - Linear Acceleration')
    ax1.grid(True, alpha=0.3)
    
    # Angular acceleration
    ax2.plot(data['timestamp'], data['angular_accel'], 'r-', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Acceleration (rad/s²)')
    ax2.set_title(f'{title} - Angular Acceleration')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_velocity_comparison(data, title="Velocity Tracking"):
    """Plot velocity tracking performance"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Linear velocity
    ax1.plot(data['timestamp'], data['desired_v'], 'b--', linewidth=2, label='Desired')
    ax1.plot(data['timestamp'], data['robot_v'], 'r-', linewidth=1.5, label='Actual')
    ax1.set_ylabel('Linear Velocity (m/s)')
    ax1.set_title(f'{title} - Linear Velocity')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Angular velocity
    ax2.plot(data['timestamp'], data['desired_omega'], 'b--', linewidth=2, label='Desired')
    ax2.plot(data['timestamp'], data['robot_omega'], 'r-', linewidth=1.5, label='Actual')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title(f'{title} - Angular Velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def compute_statistics(data):
    """Compute and print simulation statistics"""
    position_error = np.sqrt((data['robot_x'] - data['desired_x'])**2 + 
                           (data['robot_y'] - data['desired_y'])**2)
    
    # Angle difference with wrap-around handling
    angle_diff = data['robot_theta'] - data['desired_theta']
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
    orientation_error = np.abs(angle_diff)
    
    print("=== Simulation Statistics ===")
    print(f"Position Error:")
    print(f"  Mean: {np.mean(position_error):.4f} m")
    print(f"  Max:  {np.max(position_error):.4f} m")
    print(f"  Std:  {np.std(position_error):.4f} m")
    
    print(f"Orientation Error:")
    print(f"  Mean: {np.mean(orientation_error):.4f} rad ({np.degrees(np.mean(orientation_error)):.2f}°)")
    print(f"  Max:  {np.max(orientation_error):.4f} rad ({np.degrees(np.max(orientation_error)):.2f}°)")
    print(f"  Std:  {np.std(orientation_error):.4f} rad ({np.degrees(np.std(orientation_error)):.2f}°)")
    
    print(f"Control Statistics:")
    print(f"  Linear Accel - Mean: {np.mean(np.abs(data['linear_accel'])):.4f}, Max: {np.max(np.abs(data['linear_accel'])):.4f}")
    print(f"  Angular Accel - Mean: {np.mean(np.abs(data['angular_accel'])):.4f}, Max: {np.max(np.abs(data['angular_accel'])):.4f}")

def main():
    parser = argparse.ArgumentParser(description='Visualize dual-wheel robot simulation results')
    parser.add_argument('filename', help='CSV file containing simulation results')
    parser.add_argument('--title', default='Robot Simulation', help='Title for plots')
    parser.add_argument('--save', action='store_true', help='Save plots as PNG files')
    parser.add_argument('--show', action='store_true', default=True, help='Show plots')
    
    args = parser.parse_args()
    
    # Load data
    data = load_simulation_data(args.filename)
    if data is None:
        return
    
    # Compute and print statistics
    compute_statistics(data)
    
    # Create plots
    fig1 = plot_trajectory_comparison(data, args.title)
    fig2 = plot_control_signals(data, args.title)
    fig3 = plot_velocity_comparison(data, args.title)
    
    # Save plots if requested
    if args.save:
        basename = os.path.splitext(args.filename)[0]
        fig1.savefig(f'{basename}_trajectory.png', dpi=300, bbox_inches='tight')
        fig2.savefig(f'{basename}_control.png', dpi=300, bbox_inches='tight')
        fig3.savefig(f'{basename}_velocity.png', dpi=300, bbox_inches='tight')
        print(f"Plots saved as {basename}_*.png")
    
    # Show plots if requested
    if args.show:
        plt.show()

if __name__ == '__main__':
    main()