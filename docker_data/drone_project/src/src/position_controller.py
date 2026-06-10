"""
Position Controller for XY stabilization using cascade PID control.

This module implements a dual-loop PID control system for horizontal position control:
- Outer loop: Position control (generates velocity setpoints)
- Inner loop: Velocity control (generates PWM commands)

The controller processes visual drift measurements from sky_anchor and generates
appropriate roll and pitch commands to maintain position.
"""

import time
from collections import deque
from datetime import datetime
from typing import Any, Dict, Literal, Optional, Tuple

import numpy as np

from .config.objects import PositionConfig
from .logger import get_logger
from .pid_controller import PIDController
from .position_config import (
    ALTITUDE_COMPENSATION, COORDINATE_SYSTEM, POSITION_CONTROL, POSITION_FILTERING,
    PWM_LIMITS)
from .position_csv_logger import PositionCSVLogger


class PositionController:
    """
    Cascade PID controller for XY position stabilization.

    Features:
    - Dual-loop control (position → velocity → PWM)
    - Altitude compensation for visual measurements
    - Confidence-based filtering
    - Velocity estimation from position changes
    - Anti-windup protection
    - CSV logging for analysis
    """

    def __init__(self,
                 start_timestamp: Optional[datetime] = None,
                 *,
                 config: Optional[PositionConfig] = None,
                 csv_logger=None):
        """
        Initialize position controller with configuration from position_config.

        Args:
            start_timestamp: Optional datetime for consistent session naming
            config: Optional injected PositionConfig (GRASP Step 7, LC-2).
                If None (default), the configuration is read from the
                position_config.py dicts exactly as before (same dicts,
                same values).  Also the source for the mode-switch gain
                restore in ``__enable_stabilization``.
            csv_logger: Optional injected CSV logger for control-data logging.
                If None (default), a file-writing PositionCSVLogger is created
                exactly as before.
        """
        if config is None:
            config = PositionConfig.from_dicts()
        self.__config = config

        # Mode state, per instance (Step 7 carried bullet: formerly a class
        # attribute shadowed on the first mode switch).
        self.__mode: Literal['stabilization', 'navigation'] = 'stabilization'

        self.logger = get_logger("position_controller", "logs/position_controller.log")
        # Initialize position PID controllers (outer loop)
        self.position_pid_x = PIDController(
            kp=config.pid_x.kp,
            ki=config.pid_x.ki,
            kd=config.pid_x.kd,
            output_min=-config.max_correction,
            output_max=config.max_correction,
            integral_limit=100
        )

        self.position_pid_y = PIDController(
            kp=config.pid_y.kp,
            ki=config.pid_y.ki,
            kd=config.pid_y.kd,
            output_min=-config.max_correction,
            output_max=config.max_correction,
            integral_limit=100
        )

        # Initialize angle PID controller for yaw
        self.angle_pid = PIDController(
            kp=config.angle_pid.kp,
            ki=config.angle_pid.ki,
            kd=config.angle_pid.kd,
            output_min=-config.max_correction,
            output_max=config.max_correction,
            integral_limit=30
        )

        # Position filtering
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_angle = 0.0
        self.position_initialized = False

        # Velocity estimation
        self.velocity_history_x = deque(maxlen=POSITION_FILTERING['velocity_filter_size'])
        self.velocity_history_y = deque(maxlen=POSITION_FILTERING['velocity_filter_size'])
        self.last_position_x = None
        self.last_position_y = None
        self.last_position_time = None
        self.estimated_velocity_x = 0.0
        self.estimated_velocity_y = 0.0

        # Breaking state
        self.last_roll_pwm = PWM_LIMITS['neutral']
        self.last_pitch_pwm = PWM_LIMITS['neutral']

        # Performance tracking
        self.last_update_time = None
        self.update_count = 0
        self.last_confidence = 0.0
        self.measurement_timeout_counter = 0

        # Navigation target tracking
        self.navigation_target_dx = 0.0
        self.navigation_target_dy = 0.0

        # CSV logging
        if csv_logger is None:
            csv_logger = PositionCSVLogger(start_timestamp=start_timestamp)
        self.csv_logger = csv_logger

        # State tracking
        self.is_active = False

        self.logger.info(f"Position controller initialized")

    def stop(self):
        self.csv_logger.writer = None

    def pixels_to_meters(self, pixels: float, altitude: float) -> float:
        """
        Convert pixel drift to meters using altitude compensation.

        Args:
            pixels: Drift in pixels
            altitude: Current altitude in meters

        Returns:
            Drift in meters
        """
        if not ALTITUDE_COMPENSATION['use_adaptive_scaling']:
            # Fixed scaling
            meters_per_pixel = 1.0 / ALTITUDE_COMPENSATION['pixels_per_meter_at_ref']
            return pixels * meters_per_pixel

        # Altitude-based scaling
        altitude = np.clip(altitude,
                          ALTITUDE_COMPENSATION['min_altitude'],
                          ALTITUDE_COMPENSATION['max_altitude'])

        # Scale based on altitude ratio
        scale_factor = altitude / ALTITUDE_COMPENSATION['reference_altitude']
        meters_per_pixel = scale_factor / ALTITUDE_COMPENSATION['pixels_per_meter_at_ref']

        return pixels * meters_per_pixel

    def __estimate_velocity(self, x_meters: float, y_meters: float, current_time: float) -> Tuple[float, float]:
        """
        Estimate velocity from position changes.

        Args:
            x_meters: Current X position in meters
            y_meters: Current Y position in meters
            current_time: Current time in seconds

        Returns:
            Estimated (vx, vy) in m/s
        """
        if self.last_position_x is not None and self.last_position_time is not None:
            dt = current_time - self.last_position_time
            if dt > 0:
                vx = (x_meters - self.last_position_x) / dt
                vy = (y_meters - self.last_position_y) / dt

                # Add to history
                self.velocity_history_x.append(vx)
                self.velocity_history_y.append(vy)

                # Average velocity
                if len(self.velocity_history_x) > 0:
                    self.estimated_velocity_x = sum(self.velocity_history_x) / len(self.velocity_history_x)
                    self.estimated_velocity_y = sum(self.velocity_history_y) / len(self.velocity_history_y)

        self.last_position_x = x_meters
        self.last_position_y = y_meters
        self.last_position_time = current_time

        return self.estimated_velocity_x, self.estimated_velocity_y

    def __enable_navigation(self):
        self.logger.info("Enabling navigation mode")
        self.position_pid_x.ki = 0.0
        self.position_pid_x.kd = 0.0

        self.position_pid_y.ki = 0.0
        self.position_pid_y.kd = 0.0
        self.reset()

        self.__mode = 'navigation'

    def __enable_stabilization(self):
        self.logger.info("Enabling stabilization mode")
        # Restore the coefficients from the injected config (GRASP Step 7,
        # LC-2; formerly read back from the global POSITION_PID_X/Y dicts --
        # the default config is built from those same dicts).
        self.position_pid_x.ki = self.__config.pid_x.ki
        self.position_pid_x.kd = self.__config.pid_x.kd

        self.position_pid_y.ki = self.__config.pid_y.ki
        self.position_pid_y.kd = self.__config.pid_y.kd
        self.reset()

        self.__mode = 'stabilization'

    def update(self,
               dx_pixels: float,
               dy_pixels: float,
               angle_deg: float,
               confidence: float,
               altitude: float,
               current_time: Optional[float] = None,
               target_dx_pixels: Optional[float] = None,
               target_dy_pixels: Optional[float] = None,
               navigation: bool = False) -> Dict[str, Any]:
        """
        Update position controller and return PWM commands.

        Args:
            dx_pixels: Horizontal drift in pixels from sky_anchor
            dy_pixels: Vertical drift in pixels from sky_anchor
            angle_deg: Angular drift in degrees
            confidence: Measurement confidence (0-1, match percentage).
                Informational only since GRASP Step 4 (logged to CSV, kept
                in last_confidence); it no longer switches modes.
            altitude: Current altitude in meters for compensation
            current_time: Current time in seconds
            target_dx_pixels: Active navigation target X (pixels), if available
            target_dy_pixels: Active navigation target Y (pixels), if available
            navigation: Explicit navigation flag from the sky_anchor payload
                (StabilizerReading.navigation).  True switches to navigation
                mode (position ki/kd zeroed), False switches back to
                stabilization mode (ki/kd restored from POSITION_PID_X/Y);
                each switch resets controller state.  Replaces the former
                magic ``confidence == 1.01`` sentinel (IE-3).  Defaults to
                False so callers that never navigate keep the historic
                stabilization-only behavior.

        Returns:
            Dictionary containing:
                - roll_pwm: Roll PWM command (1000-2000)
                - pitch_pwm: Pitch PWM command (1000-2000)
                - yaw_pwm: Yaw PWM command (1000-2000)
                - valid: Boolean indicating if commands are valid
                - debug_info: Dictionary with debug information
        """
        if navigation and self.__mode != 'navigation':
            self.__enable_navigation()
        elif not navigation and self.__mode != 'stabilization':
            self.__enable_stabilization()

        if current_time is None:
            current_time = time.time()

        # Initialize result
        result = {
            'roll_pwm': PWM_LIMITS['neutral'],
            'pitch_pwm': PWM_LIMITS['neutral'],
            'yaw_pwm': PWM_LIMITS['neutral'],
        }

        self.measurement_timeout_counter = 0
        self.last_confidence = confidence

        if target_dx_pixels is not None:
            self.navigation_target_dx = target_dx_pixels
        if target_dy_pixels is not None:
            self.navigation_target_dy = target_dy_pixels

        # Apply coordinate system configuration
        if COORDINATE_SYSTEM['invert_x']:
            dx_pixels = -dx_pixels
        if COORDINATE_SYSTEM['invert_y']:
            dy_pixels = -dy_pixels
        if COORDINATE_SYSTEM['invert_angle']:
            angle_deg = -angle_deg

        # Filter position measurements
        filtered_x, filtered_y, filtered_angle = dx_pixels, dy_pixels, angle_deg

        # Apply deadband
        if abs(filtered_x) < POSITION_CONTROL['deadband_x']:
            filtered_x = 0
        if abs(filtered_y) < POSITION_CONTROL['deadband_y']:
            filtered_y = 0
        if abs(filtered_angle) < POSITION_CONTROL['deadband_angle']:
            filtered_angle = 0

        # Estimate velocity
        vx, vy = self.__estimate_velocity(filtered_x, filtered_y, current_time)

        # Position control (outer loop) - generates velocity setpoints
        # Target position is 0,0 (no drift)
        velocity_setpoint_x = self.position_pid_x.update(0, filtered_x, current_time)
        velocity_setpoint_y = self.position_pid_y.update(0, filtered_y, current_time)

        # Velocity control (inner loop) - generates PWM adjustments
        roll_adjustment = velocity_setpoint_x
        pitch_adjustment = velocity_setpoint_y

        # roll_adjustment = velocity_setpoint_x
        # pitch_adjustment = velocity_setpoint_y

        # Angle control - generates yaw PWM
        yaw_adjustment = self.angle_pid.update(0, filtered_angle, current_time)

        # Calculate final PWM values
        roll_pwm = int(PWM_LIMITS['neutral'] + roll_adjustment)
        pitch_pwm = int(PWM_LIMITS['neutral'] + pitch_adjustment)
        yaw_pwm = int(PWM_LIMITS['neutral'] + yaw_adjustment)

        # Clamp PWM values
        roll_pwm = np.clip(roll_pwm, PWM_LIMITS['min_roll'], PWM_LIMITS['max_roll'])
        pitch_pwm = np.clip(pitch_pwm, PWM_LIMITS['min_pitch'], PWM_LIMITS['max_pitch'])
        yaw_pwm = np.clip(yaw_pwm, PWM_LIMITS['min_yaw'], PWM_LIMITS['max_yaw'])

        # Update state
        self.last_roll_pwm = roll_pwm
        self.last_pitch_pwm = pitch_pwm
        self.last_update_time = current_time
        self.update_count += 1
        self.is_active = True

        # Log to CSV
        csv_data = {
            'timestamp': current_time,
            'matches_percent': confidence * 100,
            'dx': dx_pixels,
            'dy': dy_pixels,
            'target_dx_pixels': self.navigation_target_dx,
            'target_dy_pixels': self.navigation_target_dy,
            'angle_deg': angle_deg,
            'rc_roll': roll_pwm,
            'rc_pitch': pitch_pwm,
            'rc_yaw': yaw_pwm,
            'filtered_x': filtered_x,
            'filtered_y': filtered_y,
            'velocity_x': vx,
            'velocity_y': vy,
            'velocity_setpoint_x': velocity_setpoint_x,
            'velocity_setpoint_y': velocity_setpoint_y,
            'altitude': altitude,
            # Add PID component tracking
            'pos_pid_x_p': self.position_pid_x.last_p_term if hasattr(self.position_pid_x, 'last_p_term') else 0,
            'pos_pid_x_i': self.position_pid_x.last_i_term if hasattr(self.position_pid_x, 'last_i_term') else 0,
            'pos_pid_x_d': self.position_pid_x.last_d_term if hasattr(self.position_pid_x, 'last_d_term') else 0,
            'pos_pid_y_p': self.position_pid_y.last_p_term if hasattr(self.position_pid_y, 'last_p_term') else 0,
            'pos_pid_y_i': self.position_pid_y.last_i_term if hasattr(self.position_pid_y, 'last_i_term') else 0,
            'pos_pid_y_d': self.position_pid_y.last_d_term if hasattr(self.position_pid_y, 'last_d_term') else 0,
            # Add PID coefficients for tracking between experiments
            'pos_pid_x_kp': self.position_pid_x.kp,
            'pos_pid_x_ki': self.position_pid_x.ki,
            'pos_pid_x_kd': self.position_pid_x.kd,
            'pos_pid_y_kp': self.position_pid_y.kp,
            'pos_pid_y_ki': self.position_pid_y.ki,
            'pos_pid_y_kd': self.position_pid_y.kd,
            'angle_pid_kp': self.angle_pid.kp,
            'angle_pid_ki': self.angle_pid.ki,
            'angle_pid_kd': self.angle_pid.kd,
        }
        self.csv_logger.append(csv_data)

        result.update({
            'roll_pwm': roll_pwm,
            'pitch_pwm': pitch_pwm,
            'yaw_pwm': yaw_pwm,
        })

        return result

    def reset(self):
        """Reset controller state."""
        self.position_pid_x.reset()
        self.position_pid_y.reset()
        self.angle_pid.reset()

        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_angle = 0.0
        self.position_initialized = False

        self.velocity_history_x.clear()
        self.velocity_history_y.clear()
        self.last_position_x = None
        self.last_position_y = None
        self.last_position_time = None
        self.estimated_velocity_x = 0.0
        self.estimated_velocity_y = 0.0

        self.measurement_timeout_counter = 0
        self.navigation_target_dx = 0.0
        self.navigation_target_dy = 0.0

        self.is_active = False

        self.logger.info("Position controller reset")

    def __del__(self):
        """Cleanup on deletion."""
        if hasattr(self, 'csv_logger'):
            self.csv_logger.close()
