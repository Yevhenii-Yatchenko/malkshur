#!/usr/bin/env python3
"""
CSV Data Logger for Navigation PID Control System

Provides real-time CSV logging focused on PID controller performance analysis.
Logs PID components (P, I, D terms) for both X and Y axes, similar to position_csv_logger.
"""

import csv
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional


class NavigationCSVLogger:
    """Real-time CSV logger for navigation PID control data."""

    def __init__(self, filename: Optional[str] = None, log_dir: str = "logs/csv/navigation"):
        """
        Initialize CSV logger with auto-generated filename.

        Args:
            filename: Optional custom filename. If None, generates timestamped name
            log_dir: Directory for CSV files (default: logs/csv/navigation)
        """
        # Create log directory if it doesn't exist
        Path(log_dir).mkdir(parents=True, exist_ok=True)

        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"navigation_pid_{timestamp}.csv"

        self.filepath = os.path.join(log_dir, filename)
        self.file = None
        self.writer = None
        self.headers_written = False
        self.start_time = None

        # Define column headers focused on PID analysis
        self.headers = [
            # Timing
            'timestamp',
            'elapsed_time',

            # Navigation targets
            'target_x',
            'target_y',

            # Current position (estimated)
            'position_x',
            'position_y',

            # Errors
            'error_x',
            'error_y',
            'error_magnitude',

            # Velocity commands (PID outputs)
            'commanded_vel_x',
            'commanded_vel_y',

            'pid_state',

            # PID X-axis components
            'pid_x_p',  # Proportional term
            'pid_x_i',  # Integral term
            'pid_x_d',  # Derivative term
            'pid_x_output',  # Total output
            'pid_x_error',  # Current error
            'pid_x_integral',  # Integral accumulator

            # PID Y-axis components
            'pid_y_p',  # Proportional term
            'pid_y_i',  # Integral term
            'pid_y_d',  # Derivative term
            'pid_y_output',  # Total output
            'pid_y_error',  # Current error
            'pid_y_integral',  # Integral accumulator

            # PID coefficients (for tracking)
            'pid_kp',
            'pid_ki',
            'pid_kd',

            # Virtual drift injection (output)
            'injected_dx',
            'injected_dy',
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
        Append navigation PID control data to CSV file.

        Args:
            data: Dictionary containing navigation state and PID control data
        """
        if not self.writer:
            return

        try:
            # Initialize start time on first append
            if self.start_time is None:
                self.start_time = data.get('timestamp', 0)

            # Calculate elapsed time
            current_timestamp = data.get('timestamp', 0)
            elapsed_time = current_timestamp - self.start_time if self.start_time else 0

            # Extract PID state
            pid_state = data.get('pid_state', {})
            pid_x = pid_state.get('pid_x', {})
            pid_y = pid_state.get('pid_y', {})

            # Create row with all required fields
            row = {
                # Timing
                'timestamp': current_timestamp,
                'elapsed_time': elapsed_time,

                # Navigation targets
                'target_x': data.get('target_x', 0),
                'target_y': data.get('target_y', 0),

                # Current position
                'position_x': data.get('position_x', 0),
                'position_y': data.get('position_y', 0),

                # Errors
                'error_x': data.get('error_x', 0),
                'error_y': data.get('error_y', 0),
                'error_magnitude': data.get('error_magnitude', 0),

                # Velocity commands
                'commanded_vel_x': data.get('commanded_vel_x', 0),
                'commanded_vel_y': data.get('commanded_vel_y', 0),

                # PID X-axis components
                'pid_x_p': pid_x.get('p_term', 0),
                'pid_x_i': pid_x.get('i_term', 0),
                'pid_x_d': pid_x.get('d_term', 0),
                'pid_x_output': pid_x.get('total_output', 0),
                'pid_x_error': pid_x.get('error', 0),
                'pid_x_integral': pid_x.get('integral', 0),

                # PID Y-axis components
                'pid_y_p': pid_y.get('p_term', 0),
                'pid_y_i': pid_y.get('i_term', 0),
                'pid_y_d': pid_y.get('d_term', 0),
                'pid_y_output': pid_y.get('total_output', 0),
                'pid_y_error': pid_y.get('error', 0),
                'pid_y_integral': pid_y.get('integral', 0),

                # PID coefficients (assume same for both axes)
                'pid_kp': pid_x.get('gains', {}).get('kp', 0),
                'pid_ki': pid_x.get('gains', {}).get('ki', 0),
                'pid_kd': pid_x.get('gains', {}).get('kd', 0),

                # Virtual drift injection
                'injected_dx': data.get('injected_dx', 0),
                'injected_dy': data.get('injected_dy', 0),
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