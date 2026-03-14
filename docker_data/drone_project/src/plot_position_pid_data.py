#!/usr/bin/python3
"""
Position Control Data Analysis and Visualization Tool

This script reads CSV data logged by the position control system and generates
comprehensive plots for analysis and PID tuning. It helps visualize:
- Position tracking performance
- Velocity estimation and control
- PID component contributions
- Control command outputs
- System stability and oscillations

Usage:
    python3 plot_position_pid_data.py [csv_file]

If no file is specified, it will use the most recent position control CSV file.
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
from pathlib import Path
import shutil


def find_latest_csv():
    """Find the most recent position control CSV file."""
    csv_files = glob.glob('logs/csv/position_control_*.csv')
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getctime)


def load_csv_data(filepath):
    """Load and preprocess CSV data."""
    try:
        df = pd.read_csv(filepath)

        # Calculate elapsed time from timestamps
        if 'timestamp' in df.columns:
            df['elapsed_time'] = df['timestamp'] - df['timestamp'].iloc[0]
        else:
            df['elapsed_time'] = df.index * 0.01  # Assume 100Hz if no timestamp

        # Convert matches_percent to confidence (0-1 range)
        if 'matches_percent' in df.columns:
            df['confidence'] = df['matches_percent'] / 100.0

        # Calculate position magnitude
        if 'filtered_x' in df.columns and 'filtered_y' in df.columns:
            df['position_magnitude'] = np.sqrt(df['filtered_x']**2 + df['filtered_y']**2)

        # Calculate velocity magnitude
        if 'velocity_x' in df.columns and 'velocity_y' in df.columns:
            df['velocity_magnitude'] = np.sqrt(df['velocity_x']**2 + df['velocity_y']**2)

        # Calculate PWM deviations from neutral (1500)
        neutral_pwm = 1500
        for col in ['rc_roll', 'rc_pitch', 'rc_yaw']:
            if col in df.columns:
                df[f'{col}_deviation'] = df[col] - neutral_pwm

        return df

    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def plot_position_tracking(df, fig, gs):
    """Plot position tracking performance - removed as per request."""
    # This function is now empty but kept for compatibility
    pass


def plot_velocity_control(df, fig, gs):
    """Plot velocity estimation and control."""
    # Velocity over time (spanning full width now)
    ax1 = fig.add_subplot(gs[0, :])
    if 'velocity_x' in df.columns and 'velocity_y' in df.columns:
        ax1.plot(df['elapsed_time'], df['velocity_x'], 'b-', label='X Velocity', alpha=0.8)
        ax1.plot(df['elapsed_time'], df['velocity_y'], 'r-', label='Y Velocity', alpha=0.8)

        # Add velocity setpoints if available
        if 'velocity_setpoint_x' in df.columns:
            ax1.plot(df['elapsed_time'], df['velocity_setpoint_x'], 'b--',
                    label='X Setpoint', alpha=0.5)
        if 'velocity_setpoint_y' in df.columns:
            ax1.plot(df['elapsed_time'], df['velocity_setpoint_y'], 'r--',
                    label='Y Setpoint', alpha=0.5)

        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_title('Velocity Control')
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)


def plot_pwm_commands(df, fig, gs):
    """Plot PWM control commands."""
    ax = fig.add_subplot(gs[1, :])

    if 'rc_roll' in df.columns and 'rc_pitch' in df.columns:
        ax.plot(df['elapsed_time'], df['rc_roll'], 'b-', label='Roll PWM', alpha=0.8)
        ax.plot(df['elapsed_time'], df['rc_pitch'], 'r-', label='Pitch PWM', alpha=0.8)

        if 'rc_yaw' in df.columns:
            ax.plot(df['elapsed_time'], df['rc_yaw'], 'g-', label='Yaw PWM', alpha=0.5)

        ax.axhline(y=1500, color='k', linestyle='--', alpha=0.3, label='Neutral')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('PWM Value')
        ax.set_title('Control Commands')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_ylim([1400, 1610])


def plot_pitch_roll_commands(df, fig, gs):
    """Plot separate pitch and roll PWM commands."""
    # Pitch PWM commands
    ax1 = fig.add_subplot(gs[2, 0])
    if 'rc_pitch' in df.columns:
        ax1.plot(df['elapsed_time'], df['rc_pitch'], 'r-', label='Pitch PWM', alpha=0.8)
        ax1.axhline(y=1500, color='k', linestyle='--', alpha=0.3, label='Neutral')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('PWM Value')
        ax1.set_title('Pitch Control Commands')
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([1400, 1610])

    # Roll PWM commands
    ax2 = fig.add_subplot(gs[2, 1])
    if 'rc_roll' in df.columns:
        ax2.plot(df['elapsed_time'], df['rc_roll'], 'b-', label='Roll PWM', alpha=0.8)
        ax2.axhline(y=1500, color='k', linestyle='--', alpha=0.3, label='Neutral')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('PWM Value')
        ax2.set_title('Roll Control Commands')
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim([1400, 1610])


def plot_position_pid_components(df, fig, gs):
    """Plot position PID component contributions."""
    # Position PID components for X
    ax1 = fig.add_subplot(gs[3, 0])
    pid_x_cols = ['pos_pid_x_p', 'pos_pid_x_i', 'pos_pid_x_d']
    if all(col in df.columns for col in pid_x_cols):
        ax1.plot(df['elapsed_time'], df['pos_pid_x_p'], 'r-', label='P', alpha=0.8)
        ax1.plot(df['elapsed_time'], df['pos_pid_x_i'], 'g-', label='I', alpha=0.8)
        ax1.plot(df['elapsed_time'], df['pos_pid_x_d'], 'b-', label='D', alpha=0.8)

        # Calculate and plot total output
        total_x = df['pos_pid_x_p'] + df['pos_pid_x_i'] + df['pos_pid_x_d']
        ax1.plot(df['elapsed_time'], total_x, 'k--', label='Total Output', linewidth=2)

        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Component Value')
        ax1.set_title('Position PID Components (X)')
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)

    # Position PID components for Y
    ax2 = fig.add_subplot(gs[3, 1])
    pid_y_cols = ['pos_pid_y_p', 'pos_pid_y_i', 'pos_pid_y_d']
    if all(col in df.columns for col in pid_y_cols):
        ax2.plot(df['elapsed_time'], df['pos_pid_y_p'], 'r-', label='P', alpha=0.8)
        ax2.plot(df['elapsed_time'], df['pos_pid_y_i'], 'g-', label='I', alpha=0.8)
        ax2.plot(df['elapsed_time'], df['pos_pid_y_d'], 'b-', label='D', alpha=0.8)

        # Calculate and plot total output
        total_y = df['pos_pid_y_p'] + df['pos_pid_y_i'] + df['pos_pid_y_d']
        ax2.plot(df['elapsed_time'], total_y, 'k--', label='Total Output', linewidth=2)

        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Component Value')
        ax2.set_title('Position PID Components (Y)')
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)


def plot_raw_measurements(df, fig, gs):
    """Plot raw pixel measurements from sky_anchor."""
    ax = fig.add_subplot(gs[4, 0])  # Row 4, left column

    if 'dx' in df.columns and 'dy' in df.columns:
        ax.plot(df['elapsed_time'], df['dx'], 'b-', label='dx (pixels)', alpha=0.8)
        ax.plot(df['elapsed_time'], df['dy'], 'r-', label='dy (pixels)', alpha=0.8)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Drift (pixels)')
        ax.set_title('Raw Visual Measurements')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_ylim([-150, 150])


def plot_confidence_and_altitude(df, fig, gs):
    """Plot measurement confidence and altitude."""
    ax1 = fig.add_subplot(gs[4, 1])  # Row 4, right column

    # Plot confidence on primary y-axis
    if 'confidence' in df.columns:
        color = 'tab:blue'
        ax1.plot(df['elapsed_time'], df['confidence'], color=color, label='Confidence', alpha=0.8)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Confidence', color=color)
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.set_ylim([0, 1.1])
        ax1.grid(True, alpha=0.3)

    # Plot altitude on secondary y-axis
    if 'altitude' in df.columns:
        ax2 = ax1.twinx()
        color = 'tab:orange'
        ax2.plot(df['elapsed_time'], df['altitude'], color=color, label='Altitude', alpha=0.8)
        ax2.set_ylabel('Altitude (m)', color=color)
        ax2.tick_params(axis='y', labelcolor=color)

    ax1.set_title('Measurement Quality')

    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    if 'altitude' in df.columns:
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='best')
    else:
        ax1.legend(loc='best')


def calculate_performance_metrics(df):
    """Calculate and return performance metrics."""
    metrics = {}

    if 'filtered_x' in df.columns and 'filtered_y' in df.columns:
        # Position metrics
        metrics['final_error_x'] = abs(df['filtered_x'].iloc[-1])
        metrics['final_error_y'] = abs(df['filtered_y'].iloc[-1])
        metrics['final_error_total'] = np.sqrt(metrics['final_error_x']**2 +
                                              metrics['final_error_y']**2)

        metrics['rms_error_x'] = np.sqrt(np.mean(df['filtered_x']**2))
        metrics['rms_error_y'] = np.sqrt(np.mean(df['filtered_y']**2))

        metrics['max_error_x'] = df['filtered_x'].abs().max()
        metrics['max_error_y'] = df['filtered_y'].abs().max()

        # Find settling time (within 5cm of target)
        threshold = 0.05
        settled = df[(df['filtered_x'].abs() < threshold) &
                    (df['filtered_y'].abs() < threshold)]
        if not settled.empty:
            metrics['settling_time'] = settled['elapsed_time'].iloc[0]
        else:
            metrics['settling_time'] = df['elapsed_time'].iloc[-1]

    if 'velocity_magnitude' in df.columns:
        metrics['mean_velocity'] = df['velocity_magnitude'].mean()
        metrics['max_velocity'] = df['velocity_magnitude'].max()

    if 'rc_roll_deviation' in df.columns and 'rc_pitch_deviation' in df.columns:
        metrics['mean_roll_cmd'] = df['rc_roll_deviation'].abs().mean()
        metrics['mean_pitch_cmd'] = df['rc_pitch_deviation'].abs().mean()
        metrics['control_effort'] = np.sqrt(df['rc_roll_deviation']**2 +
                                           df['rc_pitch_deviation']**2).mean()

    if 'confidence' in df.columns:
        metrics['mean_confidence'] = df['confidence'].mean()
        metrics['min_confidence'] = df['confidence'].min()

    return metrics


def plot_metrics_summary(metrics, fig, gs):
    """Plot performance metrics summary."""
    ax = fig.add_subplot(gs[5, 0])
    ax.axis('off')

    # Format metrics text
    text_lines = ["Performance Metrics Summary", "=" * 40]

    if 'final_error_x' in metrics:
        text_lines.extend([
            f"\nPosition Control:",
            f"  Final Error: X={metrics['final_error_x']:.3f}m, Y={metrics['final_error_y']:.3f}m",
            f"  Total Error: {metrics['final_error_total']:.3f}m",
            f"  RMS Error: X={metrics['rms_error_x']:.3f}m, Y={metrics['rms_error_y']:.3f}m",
            f"  Max Error: X={metrics['max_error_x']:.3f}m, Y={metrics['max_error_y']:.3f}m",
            f"  Settling Time (5cm): {metrics.get('settling_time', 'N/A'):.2f}s"
        ])

    if 'mean_velocity' in metrics:
        text_lines.extend([
            f"\nVelocity:",
            f"  Mean: {metrics['mean_velocity']:.3f} m/s",
            f"  Max: {metrics['max_velocity']:.3f} m/s"
        ])

    if 'control_effort' in metrics:
        text_lines.extend([
            f"\nControl Effort:",
            f"  Mean Roll Cmd: ±{metrics['mean_roll_cmd']:.1f} PWM",
            f"  Mean Pitch Cmd: ±{metrics['mean_pitch_cmd']:.1f} PWM",
            f"  Total Effort: {metrics['control_effort']:.1f} PWM"
        ])

    if 'mean_confidence' in metrics:
        text_lines.extend([
            f"\nMeasurement Quality:",
            f"  Mean Confidence: {metrics['mean_confidence']:.2%}",
            f"  Min Confidence: {metrics['min_confidence']:.2%}"
        ])

    metrics_text = '\n'.join(text_lines)

    ax.text(0.5, 0.5, metrics_text, transform=ax.transAxes,
           fontsize=10, verticalalignment='center', horizontalalignment='center',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
           family='monospace')


def plot_pid_coefficients(df, fig, gs):
    """Plot PID coefficients table."""
    ax = fig.add_subplot(gs[5, 1])
    ax.axis('off')

    # Extract PID coefficients - get first non-zero values or last values
    pid_coeffs = {}
    coeff_columns = [
        ('pos_pid_x_kp', 'pos_pid_x_ki', 'pos_pid_x_kd', 'Position X'),
        ('pos_pid_y_kp', 'pos_pid_y_ki', 'pos_pid_y_kd', 'Position Y'),
        ('angle_pid_kp', 'angle_pid_ki', 'angle_pid_kd', 'Angle'),
    ]

    # Format PID coefficients table
    text_lines = ["PID Coefficients (Experiment Settings)", "=" * 45]

    for kp_col, ki_col, kd_col, name in coeff_columns:
        if all(col in df.columns for col in [kp_col, ki_col, kd_col]):
            # Get the first non-zero values or use the first value
            kp = df[kp_col].loc[df[kp_col] != 0].iloc[0] if any(df[kp_col] != 0) else df[kp_col].iloc[0]
            ki = df[ki_col].loc[df[ki_col] != 0].iloc[0] if any(df[ki_col] != 0) else df[ki_col].iloc[0]
            kd = df[kd_col].loc[df[kd_col] != 0].iloc[0] if any(df[kd_col] != 0) else df[kd_col].iloc[0]

            text_lines.append(f"\n{name:12s} PID:")
            text_lines.append(f"  Kp = {kp:8.4f}")
            text_lines.append(f"  Ki = {ki:8.4f}")
            text_lines.append(f"  Kd = {kd:8.4f}")

    pid_text = '\n'.join(text_lines)

    ax.text(0.5, 0.5, pid_text, transform=ax.transAxes,
           fontsize=9, verticalalignment='center', horizontalalignment='center',
           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
           family='monospace')


def get_organized_png_path(csv_path, base_dir="logs/csv", analysis=True):
    """Generate organized PNG path: logs/csv/plots/position/YYYYMMDD/filename_analysis.png"""
    filename = os.path.basename(csv_path)

    # Extract date from filename (position_control_20250809_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path
    plots_dir = os.path.join(base_dir, "plots", "position", date_str)
    Path(plots_dir).mkdir(parents=True, exist_ok=True)

    if analysis:
        png_filename = filename.replace('.csv', '_analysis.png')
    else:
        png_filename = filename.replace('.csv', '.png')

    return os.path.join(plots_dir, png_filename)


def get_organized_csv_path(csv_path, base_dir="logs/csv"):
    """Generate organized CSV path: logs/csv/plots/position/YYYYMMDD/filename.csv"""
    filename = os.path.basename(csv_path)

    # Extract date from filename (position_control_20250809_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path (same directory as PNG)
    plots_dir = os.path.join(base_dir, "plots", "position", date_str)
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
        print(f"  Warning: Failed to move CSV file {csv_path}: {e}")
        return False


def scan_directory_for_csv_files(directory):
    """Scan directory for CSV files that don't have corresponding PNG files."""
    csv_files = []

    if not os.path.isdir(directory):
        print(f"Error: '{directory}' is not a directory")
        return csv_files

    for filename in os.listdir(directory):
        if filename.endswith('.csv') and 'position_control_' in filename:
            csv_path = os.path.join(directory, filename)
            png_path = get_organized_png_path(csv_path, directory, analysis=True)

            # Only include if PNG doesn't exist or CSV is newer
            if not os.path.exists(png_path) or os.path.getmtime(csv_path) > os.path.getmtime(png_path):
                csv_files.append(csv_path)

    return sorted(csv_files)


def process_csv_file(csv_path):
    """Process a single CSV file and generate its PNG plot."""
    # Get organized PNG path
    base_dir = os.path.dirname(csv_path) if 'logs/csv' in csv_path else "logs/csv"
    png_path = get_organized_png_path(csv_path, base_dir, analysis=True)

    print(f"Processing: {csv_path}")
    df = load_csv_data(csv_path)

    if df is None or df.empty:
        print(f"  Warning: No data found in {csv_path}")
        try:
            os.remove(csv_path)
            print(f"  Deleted empty CSV file: {csv_path}")
        except Exception as e:
            print(f"  Error deleting CSV file {csv_path}: {e}")
        return False

    try:
        # Calculate metrics
        metrics = calculate_performance_metrics(df)

        # Create figure with subplots (6 rows after removing velocity PID components)
        fig = plt.figure(figsize=(16, 20))
        gs = GridSpec(6, 2, figure=fig, hspace=0.3, wspace=0.3)

        # Add main title
        filename = os.path.basename(csv_path)
        fig.suptitle(f'Position Control Analysis - {filename}', fontsize=14, fontweight='bold')

        # Generate plots - simplified layout without velocity PID components
        plot_velocity_control(df, fig, gs)  # Row 0 (full width)
        plot_pwm_commands(df, fig, gs)      # Row 1 (full width)
        plot_pitch_roll_commands(df, fig, gs) # Row 2 (2 columns)
        plot_position_pid_components(df, fig, gs) # Row 3 (2 columns)
        plot_raw_measurements(df, fig, gs)  # Row 4 (left)
        plot_confidence_and_altitude(df, fig, gs) # Row 4 (right)
        plot_metrics_summary(metrics, fig, gs) # Row 5 (left)
        plot_pid_coefficients(df, fig, gs) # Row 5 (right)

        # Save plot
        plt.savefig(png_path, dpi=100, bbox_inches='tight')
        plt.close()
        print(f"  Generated: {png_path}")

        # Move CSV file to organized location
        if move_csv_to_organized_location(csv_path, base_dir):
            return True
        else:
            print(f"  Warning: PNG created but CSV move failed for {csv_path}")
            return True  # Still consider successful since PNG was created

    except Exception as e:
        print(f"  Error generating plot for {csv_path}: {e}")
        return False


def main():
    """Main function to run the analysis."""
    parser = argparse.ArgumentParser(description='Analyze position control CSV data')
    parser.add_argument('csv_file', nargs='?', help='CSV file to analyze (if not provided, processes directory)')
    parser.add_argument('--input_directory', '-i', help='Directory containing CSV files', default='logs/csv')
    parser.add_argument('--force', '-f', action='store_true', help='Force regeneration of PNG files even if they exist')
    parser.add_argument('--save', default=True, action='store_true', help='Save plots to file')
    parser.add_argument('--no-show', action='store_true', help='Do not display plots')
    args = parser.parse_args()

    # Single file or directory mode
    if args.csv_file:
        # Single file mode
        csv_file = args.csv_file
        print(f"Loading data from: {csv_file}")

        # Load data
        df = load_csv_data(csv_file)
        if df is None or df.empty:
            print("Failed to load data or file is empty")
            sys.exit(1)

        print(f"Loaded {len(df)} data points")
        print(f"Time span: {df['elapsed_time'].iloc[-1]:.2f} seconds")

        # Calculate metrics
        metrics = calculate_performance_metrics(df)

        # Create figure with subplots (6 rows after removing velocity PID components)
        fig = plt.figure(figsize=(16, 20))
        gs = GridSpec(6, 2, figure=fig, hspace=0.3, wspace=0.3)

        # Add main title
        filename = os.path.basename(csv_file)
        fig.suptitle(f'Position Control Analysis - {filename}', fontsize=14, fontweight='bold')

        # Generate plots - reordered and simplified without velocity PID
        print("Generating plots...")
        plot_velocity_control(df, fig, gs)  # Row 0 (full width)
        plot_pwm_commands(df, fig, gs)      # Row 1 (full width)
        plot_pitch_roll_commands(df, fig, gs) # Row 2
        plot_position_pid_components(df, fig, gs) # Row 3
        plot_raw_measurements(df, fig, gs)  # Row 4 (left)
        plot_confidence_and_altitude(df, fig, gs) # Row 4 (right)
        plot_metrics_summary(metrics, fig, gs) # Row 5 (left)
        plot_pid_coefficients(df, fig, gs) # Row 5 (right)

        # Save plots if requested
        if args.save:
            base_dir = os.path.dirname(csv_file) if 'logs/csv' in csv_file else "logs/csv"
            output_file = get_organized_png_path(csv_file, base_dir, analysis=True)
            plt.savefig(output_file, dpi=100, bbox_inches='tight')
            print(f"Plots saved to: {output_file}")

            # Move CSV file to organized location
            move_csv_to_organized_location(csv_file, base_dir)

        # Show plots unless disabled
        if not args.no_show:
            plt.show()

        # Print metrics to console
        print("\n" + "=" * 50)
        print("Performance Metrics:")
        print("=" * 50)
        for key, value in metrics.items():
            if isinstance(value, float):
                print(f"{key:20s}: {value:.3f}")
            else:
                print(f"{key:20s}: {value}")

    else:
        # Directory mode - batch processing
        input_directory = args.input_directory
        if not os.path.exists(input_directory):
            print(f"Error: Path '{input_directory}' not found")
            return

        if not os.path.isdir(input_directory):
            print(f'The input directory "{input_directory}" is a file, specify a directory!')
            return

        print(f"Scanning directory: {input_directory}")

        if args.force:
            # Process all position control CSV files
            csv_files = [os.path.join(input_directory, f) for f in os.listdir(input_directory)
                        if f.endswith('.csv') and 'position_control_' in f]
        else:
            # Only process CSV files without PNG or with newer timestamps
            csv_files = scan_directory_for_csv_files(input_directory)

        if not csv_files:
            print("No position control CSV files to process (all have up-to-date analysis PNG files)")
            return

        print(f"Found {len(csv_files)} position control CSV files to process")

        processed = 0
        for csv_file in csv_files:
            if process_csv_file(csv_file):
                processed += 1

        print(f"\nProcessed {processed}/{len(csv_files)} files successfully")


if __name__ == '__main__':
    main()