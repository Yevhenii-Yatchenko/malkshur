#!/usr/bin/env python3
"""
CSV Data Logger for Position Control System

Provides real-time CSV logging of position control data for analysis and debugging.
Logs drift measurements and PWM control outputs from PwmControlEstimator.
"""

import csv
import os
from datetime import datetime
from pathlib import Path
from threading import Timer
from typing import Dict, Any, Optional


class PositionCSVLogger:
    """Real-time CSV logger for position control data."""

    def __init__(self, filename: Optional[str] = None, log_dir: str = "logs/csv", start_timestamp: Optional[datetime] = None):
        """
        Initialize CSV logger with auto-generated filename.

        Args:
            filename: Optional custom filename. If None, generates timestamped name
            log_dir: Directory for CSV files (default: logs/csv)
            start_timestamp: Optional datetime for consistent session naming
        """
        # Create log directory if it doesn't exist
        Path(log_dir).mkdir(parents=True, exist_ok=True)

        # Generate filename if not provided
        if filename is None:
            # Use provided timestamp or current time
            timestamp_dt = start_timestamp if start_timestamp else datetime.now()
            timestamp = timestamp_dt.strftime("%Y%m%d_%H%M%S")
            filename = f"position_control_{timestamp}.csv"

        self.filepath = os.path.join(log_dir, filename)
        self.file = None
        self.writer = None
        self.headers_written = False

        # Define column headers in order
        self.headers = [
            # Input measurements
            'timestamp',
            'matches_percent',

            # Raw inputs (pixels)
            'dx',
            'dy',
            'target_dx_pixels',
            'target_dy_pixels',
            'angle_deg',

            # PWM control outputs
            'rc_roll',
            'rc_pitch',
            'rc_yaw',

            # PID-specific fields (when using PID control)
            'filtered_x',        # Filtered position X (meters)
            'filtered_y',        # Filtered position Y (meters)
            'velocity_x',        # Estimated velocity X (m/s)
            'velocity_y',        # Estimated velocity Y (m/s)
            'velocity_setpoint_x',  # Velocity setpoint from position PID
            'velocity_setpoint_y',  # Velocity setpoint from position PID
            'altitude',          # Current altitude for compensation

            # PID component tracking (optional)
            'pos_pid_x_p',       # Position PID X - P term
            'pos_pid_x_i',       # Position PID X - I term
            'pos_pid_x_d',       # Position PID X - D term
            'pos_pid_y_p',       # Position PID Y - P term
            'pos_pid_y_i',       # Position PID Y - I term
            'pos_pid_y_d',       # Position PID Y - D term
            'vel_pid_x_p',       # Velocity PID X - P term
            'vel_pid_x_i',       # Velocity PID X - I term
            'vel_pid_x_d',       # Velocity PID X - D term
            'vel_pid_y_p',       # Velocity PID Y - P term
            'vel_pid_y_i',       # Velocity PID Y - I term
            'vel_pid_y_d',       # Velocity PID Y - D term

            # PID Coefficients (for tracking between experiments)
            'pos_pid_x_kp',      # Position PID X - Kp coefficient
            'pos_pid_x_ki',      # Position PID X - Ki coefficient
            'pos_pid_x_kd',      # Position PID X - Kd coefficient
            'pos_pid_y_kp',      # Position PID Y - Kp coefficient
            'pos_pid_y_ki',      # Position PID Y - Ki coefficient
            'pos_pid_y_kd',      # Position PID Y - Kd coefficient
            'vel_pid_x_kp',      # Velocity PID X - Kp coefficient
            'vel_pid_x_ki',      # Velocity PID X - Ki coefficient
            'vel_pid_x_kd',      # Velocity PID X - Kd coefficient
            'vel_pid_y_kp',      # Velocity PID Y - Kp coefficient
            'vel_pid_y_ki',      # Velocity PID Y - Ki coefficient
            'vel_pid_y_kd',      # Velocity PID Y - Kd coefficient
            'angle_pid_kp',      # Angle PID - Kp coefficient
            'angle_pid_ki',      # Angle PID - Ki coefficient
            'angle_pid_kd',      # Angle PID - Kd coefficient
        ]

        self._open_file()

    def _open_file(self):
        """Open CSV file and write headers if needed."""
        try:
            # Check if file exists to determine mode
            file_exists = os.path.exists(self.filepath)
            mode = 'a' if file_exists else 'w'

            self.file = open(self.filepath, mode, newline='', buffering=1)  # Line buffered
            self.writer = csv.DictWriter(self.file, fieldnames=self.headers)

            # Only write header if file is new
            if not file_exists:
                self.writer.writeheader()
                self.file.flush()  # Ensure header is written immediately

            self.headers_written = True
        except Exception as e:
            print(f"Error opening CSV file {self.filepath}: {e}")
            raise

    def append(self, data: Dict[str, Any]):
        """
        Append position control data to CSV file.

        Args:
            data: Dictionary containing position control state
        """
        if not self.writer:
            return

        try:
            # Create row with all required fields (use 0 as default for missing values)
            row = {
                # Input measurements
                'timestamp': data.get('timestamp', 0),
                'matches_percent': data.get('matches_percent', 0),

                # Raw inputs (pixels)
                'dx': data.get('dx', 0),
                'dy': data.get('dy', 0),
                'target_dx_pixels': data.get('target_dx_pixels', 0),
                'target_dy_pixels': data.get('target_dy_pixels', 0),
                'angle_deg': data.get('angle_deg', 0),

                # PWM control outputs
                'rc_roll': data.get('rc_roll', 0),
                'rc_pitch': data.get('rc_pitch', 0),
                'rc_yaw': data.get('rc_yaw', 0),

                # PID-specific fields (when using PID control)
                'filtered_x': data.get('filtered_x', 0),
                'filtered_y': data.get('filtered_y', 0),
                'velocity_x': data.get('velocity_x', 0),
                'velocity_y': data.get('velocity_y', 0),
                'velocity_setpoint_x': data.get('velocity_setpoint_x', 0),
                'velocity_setpoint_y': data.get('velocity_setpoint_y', 0),
                'altitude': data.get('altitude', 0),

                # PID component tracking (optional)
                'pos_pid_x_p': data.get('pos_pid_x_p', 0),
                'pos_pid_x_i': data.get('pos_pid_x_i', 0),
                'pos_pid_x_d': data.get('pos_pid_x_d', 0),
                'pos_pid_y_p': data.get('pos_pid_y_p', 0),
                'pos_pid_y_i': data.get('pos_pid_y_i', 0),
                'pos_pid_y_d': data.get('pos_pid_y_d', 0),
                'vel_pid_x_p': data.get('vel_pid_x_p', 0),
                'vel_pid_x_i': data.get('vel_pid_x_i', 0),
                'vel_pid_x_d': data.get('vel_pid_x_d', 0),
                'vel_pid_y_p': data.get('vel_pid_y_p', 0),
                'vel_pid_y_i': data.get('vel_pid_y_i', 0),
                'vel_pid_y_d': data.get('vel_pid_y_d', 0),

                # PID Coefficients (for tracking between experiments)
                'pos_pid_x_kp': data.get('pos_pid_x_kp', 0),
                'pos_pid_x_ki': data.get('pos_pid_x_ki', 0),
                'pos_pid_x_kd': data.get('pos_pid_x_kd', 0),
                'pos_pid_y_kp': data.get('pos_pid_y_kp', 0),
                'pos_pid_y_ki': data.get('pos_pid_y_ki', 0),
                'pos_pid_y_kd': data.get('pos_pid_y_kd', 0),
                'vel_pid_x_kp': data.get('vel_pid_x_kp', 0),
                'vel_pid_x_ki': data.get('vel_pid_x_ki', 0),
                'vel_pid_x_kd': data.get('vel_pid_x_kd', 0),
                'vel_pid_y_kp': data.get('vel_pid_y_kp', 0),
                'vel_pid_y_ki': data.get('vel_pid_y_ki', 0),
                'vel_pid_y_kd': data.get('vel_pid_y_kd', 0),
                'angle_pid_kp': data.get('angle_pid_kp', 0),
                'angle_pid_ki': data.get('angle_pid_ki', 0),
                'angle_pid_kd': data.get('angle_pid_kd', 0)
            }

            # Write row to CSV
            self.writer.writerow(row)
            self.file.flush()  # Force write to disk for real-time logging

        except Exception as e:
            print(f"Error writing to CSV: {e}")

    def close(self):
        """Close the CSV file."""
        if self.file:
            try:
                self.file.close()
            except:
                pass
            self.file = None
            self.writer = None

    def __del__(self):
        """Ensure file is closed on deletion."""
        self.close()

    def __enter__(self):
        """Context manager support."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup."""
        self.close()
        return False
