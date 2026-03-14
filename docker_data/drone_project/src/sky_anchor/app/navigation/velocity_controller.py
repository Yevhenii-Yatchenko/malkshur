"""
Dual PID velocity controller for smooth drone navigation.

Uses independent PID controllers for X and Y axes to provide smooth,
decoupled control with natural deceleration as error decreases.
"""

from __future__ import annotations

import sys
import os
import numpy as np
from typing import Tuple, Dict, Any

# Add parent directories to path to import from src
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from src.pid_controller import PIDController


class DualPIDVelocityController:
    """
    Dual PID velocity controller with independent X and Y axis control.

    Provides smooth velocity commands that naturally slow down as the error
    decreases, with independent tuning for each axis.
    """

    def __init__(self):
        """Initialize dual PID controllers with default parameters."""
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 2.25

        self.max_velocity = 60.0
        self.arrival_threshold = 50.0

        self.pid_x = PIDController(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            output_min=-self.max_velocity,
            output_max=self.max_velocity,
            integral_limit=1000
        )

        self.pid_y = PIDController(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            output_min=-self.max_velocity,
            output_max=self.max_velocity,
            integral_limit=1000
        )

        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0

    def calculate_command(self, error_x: float, error_y: float, dt: float = 0.033) -> Tuple[float, float]:
        """
        Calculate velocity command for both axes using PID control with smooth deceleration.

        Args:
            error_x: Position error in X axis (pixels)
            error_y: Position error in Y axis (pixels)
            dt: Time step (seconds)

        Returns:
            Tuple of (velocity_x, velocity_y) in pixels/frame
        """
        if error_x > 100 and error_y > 100:
            distance = np.sqrt(error_x**2 + error_y**2)
        else:
            distance = max(abs(error_x), abs(error_y))

        if distance < self.arrival_threshold:
            self.pid_x.reset()
            self.pid_y.reset()
            self.last_vel_x = 0.0
            self.last_vel_y = 0.0
            return (0.0, 0.0)

        vel_x = self.pid_x.update(error_x, 0)
        vel_y = self.pid_y.update(error_y, 0)

        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_vel_x = vel_x
        self.last_vel_y = vel_y

        return (vel_x, vel_y)

    def get_pid_state(self) -> Dict[str, Any]:
        """
        Get current PID state for logging and debugging.

        Returns:
            Dictionary containing PID states for both axes
        """
        return {
            'pid_x': self.pid_x.get_state(),
            'pid_y': self.pid_y.get_state(),
            'error_x': self.last_error_x,
            'error_y': self.last_error_y,
            'vel_x': self.last_vel_x,
            'vel_y': self.last_vel_y,
        }

    def reset(self):
        """Reset both PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
