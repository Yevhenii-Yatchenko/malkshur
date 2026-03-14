#!/usr/bin/python3
"""
CSV PID Control Data Plotting Tool

Visualizes altitude control data from CSV files to assist with PID tuning.
Generates multiple plots showing system response, PID components, and performance metrics.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import os
from datetime import datetime
import pandas as pd
import re
from pathlib import Path
import shutil


def load_csv_data(filename):
    """Load control data from CSV file into pandas DataFrame."""
    try:
        df = pd.read_csv(filename)
        print(f"Loaded {len(df)} samples from CSV")
        return df
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def calculate_performance_metrics(df):
    """Calculate key performance metrics from CSV data."""
    if df is None or df.empty:
        return {}

    # Extract key columns
    times = df['timestamp'].values
    altitudes = df['current_altitude'].values
    filtered_altitudes = df['filtered_altitude'].values
    targets = df['target_altitude'].values
    errors = df['altitude_error'].values

    # Normalize time to start at 0
    times = times - times[0]

    metrics = {
        'avg_error': np.mean(np.abs(errors)),
        'max_error': np.max(np.abs(errors)),
        'std_error': np.std(errors),
        'final_error': errors[-1] if len(errors) > 0 else 0,
        'duration': times[-1] if len(times) > 0 else 0,
        'sample_rate': len(times) / times[-1] if times[-1] > 0 else 0
    }

    # Find settling time (time to reach and stay within 5% of target)
    if len(targets) > 0 and not all(targets == targets[0]):
        # Find when target changes
        target_changes = np.where(np.diff(targets) != 0)[0]
        if len(target_changes) > 0:
            last_change_idx = target_changes[-1] + 1
            final_target = targets[last_change_idx]

            if final_target != 0:
                threshold = 0.05 * abs(final_target)
                # Check from last target change onwards
                errors_after_change = np.abs(errors[last_change_idx:])
                settled_indices = np.where(errors_after_change < threshold)[0]

                if len(settled_indices) > 0:
                    first_settled = settled_indices[0]
                    if all(errors_after_change[first_settled:] < threshold):
                        metrics['settling_time'] = times[last_change_idx + first_settled] - times[last_change_idx]

    # Calculate overshoot
    if len(targets) > 0:
        # Look for overshoots relative to target
        overshoots = filtered_altitudes - targets
        positive_overshoots = overshoots[overshoots > 0]
        if len(positive_overshoots) > 0:
            max_overshoot = np.max(positive_overshoots)
            # Find the target value at max overshoot point
            overshoot_idx = np.argmax(overshoots)
            target_at_overshoot = targets[overshoot_idx]
            metrics['overshoot'] = max_overshoot
            metrics['overshoot_percent'] = (max_overshoot / target_at_overshoot) * 100 if target_at_overshoot != 0 else 0
        else:
            metrics['overshoot'] = 0
            metrics['overshoot_percent'] = 0

    return metrics


def plot_altitude_response(ax, df):
    """Plot altitude response over time with controller type color coding."""
    if df is None or df.empty:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    # Check if controller_type column exists
    if 'controller_type' in df.columns:
        # Separate data by controller type
        takeoff_mask = df['controller_type'] == 'takeoff'
        hold_mask = df['controller_type'] == 'hold'

        # Plot takeoff zones in orange/red tones
        if takeoff_mask.any():
            ax.plot(times[takeoff_mask], df['current_altitude'][takeoff_mask],
                   color='orange', alpha=0.3, label='Raw Altitude (Takeoff)')
            ax.plot(times[takeoff_mask], df['filtered_altitude'][takeoff_mask],
                   color='darkorange', label='Filtered Altitude (Takeoff)')

        # Plot hold zones in blue tones
        if hold_mask.any():
            ax.plot(times[hold_mask], df['current_altitude'][hold_mask],
                   color='lightblue', alpha=0.3, label='Raw Altitude (Hold)')
            ax.plot(times[hold_mask], df['filtered_altitude'][hold_mask],
                   color='blue', label='Filtered Altitude (Hold)')
    else:
        # Fallback for data without controller_type
        ax.plot(times, df['current_altitude'], 'b-', alpha=0.3, label='Raw Altitude')
        ax.plot(times, df['filtered_altitude'], 'b-', label='Filtered Altitude')

    ax.plot(times, df['target_altitude'], 'r--', label='Target')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Response')
    ax.legend()
    ax.grid(True, alpha=0.3)


def plot_velocity_tracking(ax, df):
    """Plot velocity setpoint vs actual velocity with controller type color coding."""
    if df is None or df.empty:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    # Check if controller_type column exists
    if 'controller_type' in df.columns:
        # Separate data by controller type
        takeoff_mask = df['controller_type'] == 'takeoff'
        hold_mask = df['controller_type'] == 'hold'

        # Plot takeoff zones in orange/red tones
        if takeoff_mask.any():
            ax.plot(times[takeoff_mask], df['estimated_velocity'][takeoff_mask],
                   color='darkorange', label='Actual Velocity (Takeoff)')
            ax.plot(times[takeoff_mask], df['velocity_setpoint'][takeoff_mask],
                   color='orange', linestyle='--', label='Velocity Setpoint (Takeoff)')

        # Plot hold zones in green tones
        if hold_mask.any():
            ax.plot(times[hold_mask], df['estimated_velocity'][hold_mask],
                   color='darkgreen', label='Actual Velocity (Hold)')
            ax.plot(times[hold_mask], df['velocity_setpoint'][hold_mask],
                   color='green', linestyle='--', label='Velocity Setpoint (Hold)')
    else:
        # Fallback for data without controller_type
        ax.plot(times, df['estimated_velocity'], 'g-', label='Actual Velocity')
        ax.plot(times, df['velocity_setpoint'], 'g--', label='Velocity Setpoint')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)


def plot_pid_components(ax, df, pid_type='position'):
    """Plot individual PID components with controller type color coding."""
    if df is None or df.empty:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    p_col = f'{pid_type}_p_term'
    i_col = f'{pid_type}_i_term'
    d_col = f'{pid_type}_d_term'
    total_col = f'{pid_type}_total_output'

    # Check if controller_type column exists
    if 'controller_type' in df.columns:
        # Separate data by controller type
        takeoff_mask = df['controller_type'] == 'takeoff'
        hold_mask = df['controller_type'] == 'hold'

        # Plot takeoff zones with warm colors
        if takeoff_mask.any():
            ax.plot(times[takeoff_mask], df[p_col][takeoff_mask], 'red', alpha=0.7, label='P term (Takeoff)')
            ax.plot(times[takeoff_mask], df[i_col][takeoff_mask], 'orange', alpha=0.7, label='I term (Takeoff)')
            ax.plot(times[takeoff_mask], df[d_col][takeoff_mask], 'gold', alpha=0.7, label='D term (Takeoff)')
            ax.plot(times[takeoff_mask], df[total_col][takeoff_mask], 'darkred',
                   linestyle='--', linewidth=2, alpha=0.8, label='Total Output (Takeoff)')

        # Plot hold zones with cool colors
        if hold_mask.any():
            ax.plot(times[hold_mask], df[p_col][hold_mask], 'blue', alpha=0.7, label='P term (Hold)')
            ax.plot(times[hold_mask], df[i_col][hold_mask], 'green', alpha=0.7, label='I term (Hold)')
            ax.plot(times[hold_mask], df[d_col][hold_mask], 'purple', alpha=0.7, label='D term (Hold)')
            ax.plot(times[hold_mask], df[total_col][hold_mask], 'darkblue',
                   linestyle='--', linewidth=2, alpha=0.8, label='Total Output (Hold)')
    else:
        # Fallback for data without controller_type
        ax.plot(times, df[p_col], 'r-', label='P term')
        ax.plot(times, df[i_col], 'g-', label='I term')
        ax.plot(times, df[d_col], 'b-', label='D term')
        ax.plot(times, df[total_col], 'k--', label='Total Output', linewidth=2)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Control Output')
    ax.set_title(f'{pid_type.capitalize()} PID Components')
    ax.legend()
    ax.grid(True, alpha=0.3)


def plot_throttle_output(ax, df):
    """Plot throttle output over time with controller type color coding."""
    if df is None or df.empty:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    # Check if controller_type column exists
    if 'controller_type' in df.columns:
        # Separate data by controller type
        takeoff_mask = df['controller_type'] == 'takeoff'
        hold_mask = df['controller_type'] == 'hold'

        # Plot takeoff zones in magenta/pink tones
        if takeoff_mask.any():
            ax.plot(times[takeoff_mask], df['throttle_output'][takeoff_mask],
                   color='magenta', alpha=0.8, label='Throttle PWM (Takeoff)')

        # Plot hold zones in purple tones
        if hold_mask.any():
            ax.plot(times[hold_mask], df['throttle_output'][hold_mask],
                   color='purple', alpha=0.8, label='Throttle PWM (Hold)')
    else:
        # Fallback for data without controller_type
        ax.plot(times, df['throttle_output'], 'm-', label='Throttle PWM')

    ax.axhline(y=1500, color='k', linestyle='--', alpha=0.5, label='Hover Throttle')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Throttle (PWM)')
    ax.set_title('Throttle Output')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim([1300, 1700])


def plot_errors(ax, df):
    """Plot altitude and velocity errors with controller type color coding."""
    if df is None or df.empty:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    # Check if controller_type column exists
    if 'controller_type' in df.columns:
        # Separate data by controller type
        takeoff_mask = df['controller_type'] == 'takeoff'
        hold_mask = df['controller_type'] == 'hold'

        # Plot takeoff zones
        if takeoff_mask.any():
            ax.plot(times[takeoff_mask], df['altitude_error'][takeoff_mask],
                   color='red', alpha=0.8, label='Altitude Error (Takeoff)')
            ax.plot(times[takeoff_mask], df['velocity_error'][takeoff_mask],
                   color='orange', alpha=0.8, label='Velocity Error (Takeoff)')

        # Plot hold zones
        if hold_mask.any():
            ax.plot(times[hold_mask], df['altitude_error'][hold_mask],
                   color='blue', alpha=0.8, label='Altitude Error (Hold)')
            ax.plot(times[hold_mask], df['velocity_error'][hold_mask],
                   color='green', alpha=0.8, label='Velocity Error (Hold)')
    else:
        # Fallback for data without controller_type
        ax.plot(times, df['altitude_error'], 'r-', label='Altitude Error')
        ax.plot(times, df['velocity_error'], 'b-', label='Velocity Error')

    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error')
    ax.set_title('Control Errors')
    ax.legend()
    ax.grid(True, alpha=0.3)


def add_controller_zones(ax, df):
    """Add background shading to highlight different controller zones."""
    if df is None or df.empty or 'controller_type' not in df.columns:
        return

    times = df['timestamp'].values - df['timestamp'].values[0]

    # Find controller transitions
    controller_changes = df['controller_type'].shift() != df['controller_type']
    change_indices = df[controller_changes].index.tolist()

    if not change_indices:
        return

    # Add first and last indices
    if 0 not in change_indices:
        change_indices.insert(0, 0)
    if len(df) - 1 not in change_indices:
        change_indices.append(len(df) - 1)

    # Add shaded regions
    for i in range(len(change_indices) - 1):
        start_idx = change_indices[i]
        end_idx = change_indices[i + 1]
        controller_type = df.iloc[start_idx]['controller_type']

        start_time = times[start_idx]
        end_time = times[end_idx]

        if controller_type == 'takeoff':
            ax.axvspan(start_time, end_time, alpha=0.1, color='orange', label='Takeoff Mode' if i == 0 else "")
        elif controller_type == 'hold':
            ax.axvspan(start_time, end_time, alpha=0.1, color='blue', label='Hold Mode' if i == 0 else "")


def generate_plots(df, output_filename=None):
    """Generate comprehensive plots from CSV data."""
    if output_filename is None:
        # Create timestamped subdirectory (legacy behavior)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join('plots', f'csv_session_{timestamp}')
        os.makedirs(session_dir, exist_ok=True)
        filename = os.path.join(session_dir, 'pid_analysis.png')
    else:
        # Use provided filename
        filename = output_filename
        session_dir = os.path.dirname(filename) or '.'

    # Calculate performance metrics
    metrics = calculate_performance_metrics(df)

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)

    # Plot 1: Altitude response
    ax1 = fig.add_subplot(gs[0, :])
    add_controller_zones(ax1, df)
    plot_altitude_response(ax1, df)

    # Plot 2: Velocity tracking
    ax2 = fig.add_subplot(gs[1, 0])
    add_controller_zones(ax2, df)
    plot_velocity_tracking(ax2, df)

    # Plot 3: Throttle output
    ax3 = fig.add_subplot(gs[1, 1])
    add_controller_zones(ax3, df)
    plot_throttle_output(ax3, df)

    # Plot 4: Position PID components
    ax4 = fig.add_subplot(gs[2, 0])
    add_controller_zones(ax4, df)
    plot_pid_components(ax4, df, 'position')

    # Plot 5: Velocity PID components
    ax5 = fig.add_subplot(gs[2, 1])
    add_controller_zones(ax5, df)
    plot_pid_components(ax5, df, 'velocity')

    # Add metrics text
    metrics_text = f"Performance Metrics:\n"
    metrics_text += f"Avg Error: {metrics.get('avg_error', 0):.3f} m\n"
    metrics_text += f"Max Error: {metrics.get('max_error', 0):.3f} m\n"
    metrics_text += f"Settling Time: {metrics.get('settling_time', 'N/A')}"
    if metrics.get('settling_time') is not None:
        metrics_text += f" s"
    metrics_text += f"\nOvershoot: {metrics.get('overshoot_percent', 0):.1f}%"

    fig.text(0.02, 0.98, metrics_text, transform=fig.transFigure,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Add config text (from first row of data)
    if not df.empty:
        config_text = f"PID Configuration:\n"
        config_text += f"Position: Kp={df['position_kp'].iloc[0]:.2f}, "
        config_text += f"Ki={df['position_ki'].iloc[0]:.3f}, "
        config_text += f"Kd={df['position_kd'].iloc[0]:.2f}\n"
        config_text += f"Velocity: Kp={df['velocity_kp'].iloc[0]:.1f}, "
        config_text += f"Ki={df['velocity_ki'].iloc[0]:.2f}, "
        config_text += f"Kd={df['velocity_kd'].iloc[0]:.2f}"

        fig.text(0.98, 0.98, config_text, transform=fig.transFigure,
                 fontsize=10, verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

    # Save plot
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {filename}")

    if output_filename is None:
        # Only save additional plots in legacy mode
        # Also save individual plots for detailed analysis
        save_individual_plots(df, session_dir)

        # Save analysis summary
        save_analysis_summary(df, metrics, session_dir)

    return metrics


def save_individual_plots(df, output_dir):
    """Save individual plots for detailed analysis."""
    if df is None or df.empty:
        return

    # Error over time plot
    fig, ax = plt.subplots(figsize=(10, 6))
    plot_errors(ax, df)
    plt.savefig(os.path.join(output_dir, 'errors.png'), dpi=300, bbox_inches='tight')
    plt.close()

    # Phase plot (altitude vs velocity)
    fig, ax = plt.subplots(figsize=(8, 8))
    altitude_errors = df['altitude_error'].values
    velocities = df['estimated_velocity'].values

    ax.plot(altitude_errors, velocities, 'b-', alpha=0.7)
    ax.scatter(altitude_errors[0], velocities[0], c='g', s=100, label='Start')
    ax.scatter(altitude_errors[-1], velocities[-1], c='r', s=100, label='End')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Altitude Error (m)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Phase Plot: Altitude Error vs Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'phase_plot.png'), dpi=300, bbox_inches='tight')
    plt.close()

    # PID integral terms over time
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    times = df['timestamp'].values - df['timestamp'].values[0]

    ax1.plot(times, df['position_integral'], 'r-', label='Position Integral')
    ax1.set_ylabel('Position Integral')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(times, df['velocity_integral'], 'b-', label='Velocity Integral')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity Integral')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.suptitle('PID Integral Wind-up Analysis')
    plt.savefig(os.path.join(output_dir, 'integral_windup.png'), dpi=300, bbox_inches='tight')
    plt.close()


def save_analysis_summary(df, metrics, output_dir):
    """Save analysis summary to text file."""
    if df is None or df.empty:
        return

    summary_path = os.path.join(output_dir, 'analysis_summary.txt')

    with open(summary_path, 'w') as f:
        f.write("CSV PID Control Analysis Summary\n")
        f.write("="*60 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        # Configuration (from first row)
        f.write("Configuration:\n")
        f.write(f"  Position PID: Kp={df['position_kp'].iloc[0]}, ")
        f.write(f"Ki={df['position_ki'].iloc[0]}, ")
        f.write(f"Kd={df['position_kd'].iloc[0]}\n")
        f.write(f"  Velocity PID: Kp={df['velocity_kp'].iloc[0]}, ")
        f.write(f"Ki={df['velocity_ki'].iloc[0]}, ")
        f.write(f"Kd={df['velocity_kd'].iloc[0]}\n\n")

        # Performance metrics
        f.write("Performance Metrics:\n")
        f.write(f"  Average Error: {metrics.get('avg_error', 0):.3f} m\n")
        f.write(f"  Max Error: {metrics.get('max_error', 0):.3f} m\n")
        f.write(f"  RMS Error: {metrics.get('std_error', 0):.3f} m\n")
        f.write(f"  Settling Time: {metrics.get('settling_time', 'N/A')}")
        if metrics.get('settling_time') is not None:
            f.write(" s\n")
        else:
            f.write("\n")
        f.write(f"  Overshoot: {metrics.get('overshoot_percent', 0):.1f}%\n")

        # Data summary
        f.write(f"\nData Summary:\n")
        f.write(f"  Duration: {metrics.get('duration', 0):.1f} seconds\n")
        f.write(f"  Samples: {len(df)}\n")
        f.write(f"  Sample Rate: {metrics.get('sample_rate', 0):.1f} Hz\n")

        # Statistical summary
        f.write(f"\nAltitude Statistics:\n")
        f.write(f"  Min: {df['filtered_altitude'].min():.3f} m\n")
        f.write(f"  Max: {df['filtered_altitude'].max():.3f} m\n")
        f.write(f"  Mean: {df['filtered_altitude'].mean():.3f} m\n")
        f.write(f"  Std Dev: {df['filtered_altitude'].std():.3f} m\n")

        f.write(f"\nThrottle Statistics:\n")
        f.write(f"  Min: {df['throttle_output'].min():.0f} PWM\n")
        f.write(f"  Max: {df['throttle_output'].max():.0f} PWM\n")
        f.write(f"  Mean: {df['throttle_output'].mean():.0f} PWM\n")
        f.write(f"  Std Dev: {df['throttle_output'].std():.1f} PWM\n")

    print(f"Analysis summary saved to: {summary_path}")


def print_tuning_advice(metrics, df):
    """Print PID tuning advice based on performance metrics."""
    print("\n" + "="*60)
    print("PID TUNING ADVICE")
    print("="*60)

    # Analyze settling time
    if metrics.get('settling_time') is None:
        print("⚠️  System did not settle - may need to increase gains or run longer test")
    elif metrics['settling_time'] > 10:
        print("⚠️  Slow settling time - consider increasing P gain")

    # Analyze overshoot
    if metrics.get('overshoot_percent', 0) > 20:
        print("⚠️  High overshoot - consider:")
        print("   - Reducing P gain in position controller")
        print("   - Increasing D gain in position controller")
    elif metrics.get('overshoot_percent', 0) < 5:
        print("✓  Low overshoot - system is well-damped")

    # Analyze steady-state error
    if abs(metrics.get('final_error', 0)) > 0.1:
        print("⚠️  Steady-state error present - consider:")
        print("   - Increasing I gain in position controller")
        print("   - Check for throttle saturation")

    # Check for integral windup
    if df is not None and not df.empty:
        max_position_integral = df['position_integral'].abs().max()
        max_velocity_integral = df['velocity_integral'].abs().max()

        if max_position_integral > 10:
            print("⚠️  Possible integral windup in position controller")
            print("   - Consider adding anti-windup limits")

        if max_velocity_integral > 100:
            print("⚠️  Possible integral windup in velocity controller")
            print("   - Consider adding anti-windup limits")

    print("\nData Quality:")
    print(f"  Sample Rate: {metrics.get('sample_rate', 0):.1f} Hz")
    print(f"  Duration: {metrics.get('duration', 0):.1f} seconds")

    if metrics.get('sample_rate', 0) < 10:
        print("⚠️  Low sample rate - may affect controller performance")

    print("="*60)


def get_organized_png_path(csv_path, base_dir="logs/csv"):
    """Generate organized PNG path: logs/csv/plots/altitude/YYYYMMDD/filename.png"""
    filename = os.path.basename(csv_path)

    # Extract date from filename (altitude_control_20250809_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path
    plots_dir = os.path.join(base_dir, "plots", "altitude", date_str)
    Path(plots_dir).mkdir(parents=True, exist_ok=True)

    png_filename = filename.replace('.csv', '.png')
    return os.path.join(plots_dir, png_filename)


def get_organized_csv_path(csv_path, base_dir="logs/csv"):
    """Generate organized CSV path: logs/csv/plots/altitude/YYYYMMDD/filename.csv"""
    filename = os.path.basename(csv_path)

    # Extract date from filename (altitude_control_20250809_115540.csv)
    date_match = re.search(r'(\d{8})', filename)
    if date_match:
        date_str = date_match.group(1)
    else:
        date_str = datetime.now().strftime("%Y%m%d")

    # Create organized path (same directory as PNG)
    plots_dir = os.path.join(base_dir, "plots", "altitude", date_str)
    Path(plots_dir).mkdir(parents=True, exist_ok=True)

    return os.path.join(plots_dir, filename)


def move_csv_to_organized_location(csv_path, base_dir="logs/csv"):
    """Move CSV file to organized location after PNG generation."""
    organized_csv_path = get_organized_csv_path(csv_path, base_dir)

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
        if filename.endswith('.csv') and 'altitude_control_' in filename:
            csv_path = os.path.join(directory, filename)
            png_path = get_organized_png_path(csv_path, directory)

            # Only include if PNG doesn't exist or CSV is newer
            if not os.path.exists(png_path) or os.path.getmtime(csv_path) > os.path.getmtime(png_path):
                csv_files.append(csv_path)

    return sorted(csv_files)


def process_csv_file(csv_path):
    """Process a single CSV file and generate its PNG plot."""
    # Get organized PNG path
    base_dir = os.path.dirname(csv_path) if 'logs/csv' in csv_path else "logs/csv"
    png_path = get_organized_png_path(csv_path, base_dir)

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
        generate_plots(df, png_path)
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
    parser = argparse.ArgumentParser(description='Plot PID control data from CSV files or directories')
    parser.add_argument('--input_directory', '-i', help='CSV file or directory containing CSV files', default='logs/csv')
    parser.add_argument('--force', '-f', action='store_true',
                        help='Force regeneration of PNG files even if they exist')

    args = parser.parse_args()
    input_directory = args.input_directory
    if not os.path.exists(input_directory):
        print(f"Error: Path '{input_directory}' not found")
        return

    # Check if path is a directory or file
    if not os.path.isdir(input_directory):
        print(f'the input directory "{input_directory}" is a file, specify a directory!')
        return

    # Directory mode - scan for CSV files
    print(f"Scanning directory: {input_directory}")

    if args.force:
        # Process all CSV files
        csv_files = [os.path.join(input_directory, f)
                     for f in os.listdir(input_directory) if f.endswith('.csv') and 'altitude_control_' in f]
    else:
        # Only process CSV files without PNG or with newer timestamps
        csv_files = scan_directory_for_csv_files(input_directory)

    if not csv_files:
        print("No CSV files to process (all have up-to-date PNG files)")
        return

    print(f"Found {len(csv_files)} CSV files to process")

    processed = 0
    for csv_file in csv_files:
        if process_csv_file(csv_file):
            processed += 1

    print(f"\nProcessed {processed}/{len(csv_files)} files successfully")


if __name__ == '__main__':
    main()