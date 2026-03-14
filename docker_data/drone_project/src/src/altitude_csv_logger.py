#!/usr/bin/env python3
"""
CSV Data Logger for Altitude Control System

Provides real-time CSV logging of control system data for analysis and debugging.
Flattens nested PID state dictionaries into individual columns for easier analysis.
"""

import csv
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional


class AltitudeCSVLogger:
    """Real-time CSV logger for altitude control data."""

    def __init__(self, filename: Optional[str] = None, log_dir: str = "logs/csv", start_timestamp: Optional[datetime] = None, controller_type: str = "unknown"):
        """
        Initialize CSV logger with auto-generated filename.

        Args:
            filename: Optional custom filename. If None, generates timestamped name
            log_dir: Directory for CSV files (default: logs/csv)
            start_timestamp: Optional datetime for consistent session naming
            controller_type: Type of controller (takeoff/hold) for data labeling
        """
        # Create log directory if it doesn't exist
        Path(log_dir).mkdir(parents=True, exist_ok=True)

        # Store controller type for data labeling
        self.controller_type = controller_type

        # Generate filename if not provided
        if filename is None:
            # Use provided timestamp or current time
            timestamp_dt = start_timestamp if start_timestamp else datetime.now()
            timestamp = timestamp_dt.strftime("%Y%m%d_%H%M%S")
            filename = f"altitude_control_{timestamp}.csv"

        self.filepath = os.path.join(log_dir, filename)
        self.file = None
        self.writer = None
        self.headers_written = False

        # Define column headers in order
        self.headers = [
            # Controller identification
            'controller_type',

            # Basic measurements
            'timestamp',
            'current_altitude',
            'filtered_altitude',
            'target_altitude',
            'altitude_error',
            'velocity_setpoint',
            'estimated_velocity',
            'velocity_error',
            'throttle_adjustment',
            'throttle_output',

            # Position PID state
            'position_p_term',
            'position_i_term',
            'position_d_term',
            'position_total_output',
            'position_integral',
            'position_derivative_avg',
            'position_error_rms',
            'position_kp',
            'position_ki',
            'position_kd',

            # Velocity PID state
            'velocity_p_term',
            'velocity_i_term',
            'velocity_d_term',
            'velocity_total_output',
            'velocity_integral',
            'velocity_derivative_avg',
            'velocity_error_rms',
            'velocity_kp',
            'velocity_ki',
            'velocity_kd'
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
        Append control data to CSV file.

        Args:
            data: Dictionary containing control system state
        """
        if not self.writer:
            return

        try:
            # Flatten the data structure
            row = {
                # Controller identification
                'controller_type': self.controller_type,

                # Basic measurements
                'timestamp': data.get('timestamp', 0),
                'current_altitude': data.get('current_altitude', 0),
                'filtered_altitude': data.get('filtered_altitude', 0),
                'target_altitude': data.get('target_altitude', 0),
                'altitude_error': data.get('altitude_error', 0),
                'velocity_setpoint': data.get('velocity_setpoint', 0),
                'estimated_velocity': data.get('estimated_velocity', 0),
                'velocity_error': data.get('velocity_error', 0),
                'throttle_adjustment': data.get('throttle_adjustment', 0),
                'throttle_output': data.get('throttle_output', 0)
            }

            # Extract position PID state
            position_pid = data.get('position_pid', {})
            row.update({
                'position_p_term': position_pid.get('p_term', 0),
                'position_i_term': position_pid.get('i_term', 0),
                'position_d_term': position_pid.get('d_term', 0),
                'position_total_output': position_pid.get('total_output', 0),
                'position_integral': position_pid.get('integral', 0),
                'position_derivative_avg': position_pid.get('derivative_avg', 0),
                'position_error_rms': position_pid.get('error_rms', 0),
                'position_kp': position_pid.get('gains', {}).get('kp', 0),
                'position_ki': position_pid.get('gains', {}).get('ki', 0),
                'position_kd': position_pid.get('gains', {}).get('kd', 0)
            })

            # Extract velocity PID state
            velocity_pid = data.get('velocity_pid', {})
            row.update({
                'velocity_p_term': velocity_pid.get('p_term', 0),
                'velocity_i_term': velocity_pid.get('i_term', 0),
                'velocity_d_term': velocity_pid.get('d_term', 0),
                'velocity_total_output': velocity_pid.get('total_output', 0),
                'velocity_integral': velocity_pid.get('integral', 0),
                'velocity_derivative_avg': velocity_pid.get('derivative_avg', 0),
                'velocity_error_rms': velocity_pid.get('error_rms', 0),
                'velocity_kp': velocity_pid.get('gains', {}).get('kp', 0),
                'velocity_ki': velocity_pid.get('gains', {}).get('ki', 0),
                'velocity_kd': velocity_pid.get('gains', {}).get('kd', 0)
            })

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