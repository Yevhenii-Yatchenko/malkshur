#!/usr/bin/env python3
"""
Navigation PID Control Data Analysis and Visualization Tool

This script reads CSV data logged by the navigation PID control system and generates
plots focused on PID performance analysis, similar to plot_position_pid_data.py.

Usage:
    python3 plot_navigation_data.py [csv_file]

If no file is specified, it will use the most recent navigation PID CSV file.
"""

import sys
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from datetime import datetime
import argparse
import re
import shutil
from pathlib import Path


def find_latest_csv():
    """Find the most recent navigation PID CSV file."""
    csv_files = glob.glob('logs/csv/navigation/navigation_pid_*.csv')
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getctime)


def load_csv_data(filepath):
    """Load and preprocess CSV data."""
    try:
        df = pd.read_csv(filepath)

        # Ensure elapsed_time is available
        if 'elapsed_time' not in df.columns and 'timestamp' in df.columns:
            df['elapsed_time'] = df['timestamp'] - df['timestamp'].iloc[0]

        # Calculate velocity magnitude
        if 'commanded_vel_x' in df.columns and 'commanded_vel_y' in df.columns:
            df['vel_magnitude'] = np.sqrt(df['commanded_vel_x']**2 + df['commanded_vel_y']**2)

        return df

    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def plot_pid_components_x(df, fig, gs):
    """Plot PID components for X-axis."""
    ax = fig.add_subplot(gs[0, 0])

    if all(col in df.columns for col in ['pid_x_p', 'pid_x_i', 'pid_x_d']):
        ax.plot(df['elapsed_time'], df['pid_x_p'], 'b-', label='P term', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['pid_x_i'], 'g-', label='I term', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['pid_x_d'], 'r-', label='D term', linewidth=1.5, alpha=0.8)

        # Add total output if available
        if 'pid_x_output' in df.columns:
            ax.plot(df['elapsed_time'], df['pid_x_output'], 'k--', label='Total', linewidth=2, alpha=0.6)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('PID Output')
    ax.set_title('X-Axis PID Components')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_pid_components_y(df, fig, gs):
    """Plot PID components for Y-axis."""
    ax = fig.add_subplot(gs[0, 1])

    if all(col in df.columns for col in ['pid_y_p', 'pid_y_i', 'pid_y_d']):
        ax.plot(df['elapsed_time'], df['pid_y_p'], 'b-', label='P term', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['pid_y_i'], 'g-', label='I term', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['pid_y_d'], 'r-', label='D term', linewidth=1.5, alpha=0.8)

        # Add total output if available
        if 'pid_y_output' in df.columns:
            ax.plot(df['elapsed_time'], df['pid_y_output'], 'k--', label='Total', linewidth=2, alpha=0.6)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('PID Output')
    ax.set_title('Y-Axis PID Components')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_errors(df, fig, gs):
    """Plot error tracking over time."""
    ax = fig.add_subplot(gs[0, 2])

    ax.plot(df['elapsed_time'], df['error_x'], 'b-', label='Error X', linewidth=1.5, alpha=0.8)
    ax.plot(df['elapsed_time'], df['error_y'], 'r-', label='Error Y', linewidth=1.5, alpha=0.8)

    if 'error_magnitude' in df.columns:
        ax.plot(df['elapsed_time'], df['error_magnitude'], 'k-', label='Magnitude', linewidth=2, alpha=0.6)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (pixels)')
    ax.set_title('Position Errors')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_velocity_commands(df, fig, gs):
    """Plot velocity commands from PID controllers."""
    ax = fig.add_subplot(gs[1, :2])

    ax.plot(df['elapsed_time'], df['commanded_vel_x'], 'b-', label='Vel X', linewidth=1.5, alpha=0.8)
    ax.plot(df['elapsed_time'], df['commanded_vel_y'], 'r-', label='Vel Y', linewidth=1.5, alpha=0.8)

    if 'vel_magnitude' in df.columns:
        ax.plot(df['elapsed_time'], df['vel_magnitude'], 'g--', label='Magnitude', linewidth=1.5, alpha=0.6)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (pixels/frame)')
    ax.set_title('Commanded Velocities')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_integral_terms(df, fig, gs):
    """Plot integral accumulator values."""
    ax = fig.add_subplot(gs[1, 2])

    if 'pid_x_integral' in df.columns and 'pid_y_integral' in df.columns:
        ax.plot(df['elapsed_time'], df['pid_x_integral'], 'b-', label='X Integral', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['pid_y_integral'], 'r-', label='Y Integral', linewidth=1.5, alpha=0.8)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Integral Accumulator')
    ax.set_title('Integral Terms (Anti-windup monitoring)')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_2d_trajectory(df, fig, gs):
    """Plot 2D position trajectory."""
    ax = fig.add_subplot(gs[2, 0])

    # Plot trajectory
    ax.plot(df['position_x'], df['position_y'], 'b-', linewidth=1.5, alpha=0.8)

    # Mark start and end
    ax.plot(df['position_x'].iloc[0], df['position_y'].iloc[0], 'go', markersize=8, label='Start')
    if len(df) > 1:
        ax.plot(df['position_x'].iloc[-1], df['position_y'].iloc[-1], 'ro', markersize=8, label='End')

    # Plot target if available
    if 'target_x' in df.columns and 'target_y' in df.columns:
        # Get unique targets
        targets = df[['target_x', 'target_y']].drop_duplicates()
        for idx, target in targets.iterrows():
            if target['target_x'] != 0 or target['target_y'] != 0:
                ax.plot(target['target_x'], target['target_y'], '*', markersize=12, label=f'Target')
                break  # Only show first non-zero target

    ax.set_xlabel('X Position (pixels)')
    ax.set_ylabel('Y Position (pixels)')
    ax.set_title('2D Trajectory')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')


def plot_injected_drift(df, fig, gs):
    """Plot injected virtual drift commands."""
    ax = fig.add_subplot(gs[2, 1])

    if 'injected_dx' in df.columns and 'injected_dy' in df.columns:
        ax.plot(df['elapsed_time'], df['injected_dx'], 'b-', label='Injected dx', linewidth=1.5, alpha=0.8)
        ax.plot(df['elapsed_time'], df['injected_dy'], 'r-', label='Injected dy', linewidth=1.5, alpha=0.8)

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Virtual Drift (pixels)')
    ax.set_title('Injected Virtual Drift')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)


def plot_pid_gains(df, fig, gs):
    """Display PID gains used."""
    ax = fig.add_subplot(gs[2, 2])

    # Extract gains if available
    gains_text = "PID Gains:\n\n"
    if 'pid_kp' in df.columns and len(df) > 0:
        kp = df['pid_kp'].iloc[0]
        ki = df['pid_ki'].iloc[0]
        kd = df['pid_kd'].iloc[0]
        gains_text += f"Kp = {kp:.4f}\n"
        gains_text += f"Ki = {ki:.6f}\n"
        gains_text += f"Kd = {kd:.4f}\n"
    else:
        gains_text += "Not available"

    # Add performance metrics
    if 'error_magnitude' in df.columns:
        final_error = df['error_magnitude'].iloc[-1] if len(df) > 0 else 0
        mean_error = df['error_magnitude'].mean()
        min_error = df['error_magnitude'].min()

        gains_text += f"\n\nPerformance:\n"
        gains_text += f"Final Error: {final_error:.1f} px\n"
        gains_text += f"Mean Error: {mean_error:.1f} px\n"
        gains_text += f"Min Error: {min_error:.1f} px"

    ax.text(0.1, 0.5, gains_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='center', family='monospace')
    ax.set_title('Configuration & Metrics')
    ax.axis('off')


def get_organized_plot_path(filename, base_dir="logs/csv", analysis=False):
    """Generate organized plot path: logs/csv/plots/navigation/YYYYMMDD/filename.png"""
    # Extract date from filename (navigation_pid_20251118_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path
    plots_dir = os.path.join(base_dir, "plots", "navigation", date_str)
    Path(plots_dir).mkdir(parents=True, exist_ok=True)

    if analysis:
        png_filename = filename.replace('.csv', '_analysis.png')
    else:
        png_filename = filename.replace('.csv', '.png')

    return os.path.join(plots_dir, png_filename)


def get_organized_csv_path(csv_path, base_dir="logs/csv"):
    """Generate organized CSV path: logs/csv/plots/navigation/YYYYMMDD/filename.csv"""
    filename = os.path.basename(csv_path)

    # Extract date from filename (navigation_pid_20251118_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path (same directory as PNG)
    plots_dir = os.path.join(base_dir, "plots", "navigation", date_str)
    Path(plots_dir).mkdir(parents=True, exist_ok=True)

    return os.path.join(plots_dir, filename)


def move_csv_to_organized_location(csv_path, base_dir="logs/csv"):
    """Move CSV file to organized location after PNG generation."""
    organized_csv_path = get_organized_csv_path(csv_path, base_dir)

    # Check if the CSV is already in the organized location
    if os.path.abspath(csv_path) == os.path.abspath(organized_csv_path):
        print(f"  CSV already in organized location: {csv_path}")
        return True

    try:
        # Move the CSV file to the organized location
        shutil.move(csv_path, organized_csv_path)
        print(f"  Moved CSV: {csv_path} -> {organized_csv_path}")
        return True
    except Exception as e:
        print(f"  Warning: Could not move CSV: {e}")
        return False


def generate_summary_stats(df):
    """Generate summary statistics for the navigation."""
    stats = {}

    # Time metrics
    stats['total_time'] = df['elapsed_time'].max()

    # Error metrics
    if 'error_magnitude' in df.columns:
        stats['min_error'] = df['error_magnitude'].min()
        stats['mean_error'] = df['error_magnitude'].mean()
        stats['max_error'] = df['error_magnitude'].max()
        stats['final_error'] = df['error_magnitude'].iloc[-1] if len(df) > 0 else 0

    # Velocity metrics
    if 'commanded_vel_x' in df.columns and 'commanded_vel_y' in df.columns:
        vel_magnitude = np.sqrt(df['commanded_vel_x']**2 + df['commanded_vel_y']**2)
        stats['max_velocity'] = vel_magnitude.max()
        stats['mean_velocity'] = vel_magnitude.mean()

    # PID metrics
    if 'pid_x_integral' in df.columns:
        stats['max_integral_x'] = df['pid_x_integral'].abs().max()
    if 'pid_y_integral' in df.columns:
        stats['max_integral_y'] = df['pid_y_integral'].abs().max()

    # Settling analysis (when error stays below threshold)
    if 'error_magnitude' in df.columns:
        threshold = 100  # pixels
        below_threshold = df['error_magnitude'] < threshold
        if below_threshold.any():
            first_below = df.loc[below_threshold, 'elapsed_time'].iloc[0]
            # Check if it stays below
            after_first = df[df['elapsed_time'] >= first_below]
            if (after_first['error_magnitude'] < threshold).all():
                stats['settling_time'] = first_below

    return stats


def main(csv_file=None):
    """Main function to generate navigation PID plots."""
    # Find CSV file
    if csv_file is None:
        csv_file = find_latest_csv()
        if csv_file is None:
            print("No navigation PID CSV files found in logs/csv/navigation/")
            return
        print(f"Using latest CSV file: {csv_file}")
    else:
        if not os.path.exists(csv_file):
            print(f"File not found: {csv_file}")
            return

    # Load data
    df = load_csv_data(csv_file)
    if df is None or len(df) == 0:
        print("No data to plot")
        return

    # Create figure with GridSpec
    fig = plt.figure(figsize=(18, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Generate plots
    plot_pid_components_x(df, fig, gs)
    plot_pid_components_y(df, fig, gs)
    plot_errors(df, fig, gs)
    plot_velocity_commands(df, fig, gs)
    plot_integral_terms(df, fig, gs)
    plot_2d_trajectory(df, fig, gs)
    plot_injected_drift(df, fig, gs)
    plot_pid_gains(df, fig, gs)

    # Add title with filename and timestamp
    filename = os.path.basename(csv_file)
    fig.suptitle(f'Navigation PID Control Analysis - {filename}', fontsize=16, y=0.98)

    # Generate and display summary statistics
    stats = generate_summary_stats(df)
    stats_text = f"Time: {stats.get('total_time', 0):.2f}s | "
    stats_text += f"Final Error: {stats.get('final_error', 0):.1f}px | "
    stats_text += f"Mean Error: {stats.get('mean_error', 0):.1f}px | "
    if 'settling_time' in stats:
        stats_text += f"Settling: {stats['settling_time']:.2f}s | "
    stats_text += f"Max Vel: {stats.get('max_velocity', 0):.1f}px/frame"

    fig.text(0.5, 0.02, stats_text, ha='center', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Save plot to organized location
    plot_path = get_organized_plot_path(filename)
    plt.savefig(plot_path, dpi=100, bbox_inches='tight')
    print(f"Plot saved to: {plot_path}")

    # Move CSV to organized location
    move_csv_to_organized_location(csv_file)

    # Show plot
    plt.show()

    # Print detailed statistics
    print("\n" + "="*60)
    print("Navigation PID Summary Statistics")
    print("="*60)
    print(f"Total Time: {stats.get('total_time', 0):.2f} seconds")
    print(f"Final Error: {stats.get('final_error', 0):.1f} pixels")
    print(f"Min Error: {stats.get('min_error', 0):.1f} pixels")
    print(f"Mean Error: {stats.get('mean_error', 0):.1f} pixels")
    print(f"Max Error: {stats.get('max_error', 0):.1f} pixels")

    if 'settling_time' in stats:
        print(f"Settling Time: {stats['settling_time']:.2f} seconds")

    print(f"\nVelocity Statistics:")
    print(f"Max Velocity: {stats.get('max_velocity', 0):.1f} pixels/frame")
    print(f"Mean Velocity: {stats.get('mean_velocity', 0):.1f} pixels/frame")

    if 'max_integral_x' in stats or 'max_integral_y' in stats:
        print(f"\nIntegral Windup:")
        print(f"Max Integral X: {stats.get('max_integral_x', 0):.1f}")
        print(f"Max Integral Y: {stats.get('max_integral_y', 0):.1f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot navigation PID control data from CSV files')
    parser.add_argument('csv_file', nargs='?', default=None,
                       help='Path to CSV file (optional, uses latest if not specified)')
    args = parser.parse_args()

    main(args.csv_file)