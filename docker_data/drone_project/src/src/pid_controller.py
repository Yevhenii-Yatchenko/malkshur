import time
from collections import deque
import numpy as np
from .altitude_csv_logger import AltitudeCSVLogger
from src.altitude_config import ALTITUDE_PID_TAKEOFF, VELOCITY_PID_FLIGHT, LIMITS, THROTTLE, FILTERING, CONTROL, DEBUG


class PIDController:
    """
    PID Controller with anti-windup, derivative filtering, and output limiting.

    This controller is designed for altitude control of drones using ultrasonic sensors.
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 output_min=-100, output_max=100,
                 integral_limit=50, derivative_filter_size=5):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_limit: Anti-windup limit for integral term
            derivative_filter_size: Number of samples for derivative filtering
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        # State variables
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

        # Derivative filtering
        self.derivative_filter_size = derivative_filter_size
        self.derivative_history = deque(maxlen=derivative_filter_size)

        # Performance tracking
        self.error_history = deque(maxlen=100)

        # Component tracking for logging
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.last_output = 0.0

    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.last_error = None
        self.last_time = None
        self.derivative_history.clear()
        self.error_history.clear()

    def update(self, setpoint, measurement, current_time=None):
        """
        Calculate PID output.

        Args:
            setpoint: Desired value (target altitude)
            measurement: Current value (current altitude)
            current_time: Current time in seconds (if None, uses time.time())

        Returns:
            PID output value, clamped to output limits
        """
        if current_time is None:
            current_time = time.time()

        error = setpoint - measurement
        self.error_history.append(error)

        # Calculate dt
        if self.last_time is None:
            dt = 0.1  # Assume 10Hz for first iteration
        else:
            dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.1  # Prevent division by zero

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        # Check if we're at output limits before integrating
        if self.last_error is not None:
            prev_output = self.kp * self.last_error + self.ki * self.integral
            at_limit = (prev_output >= self.output_max and error > 0) or \
                      (prev_output <= self.output_min and error < 0)
            if not at_limit:
                self.integral += error * dt
        else:
            self.integral += error * dt

        if self.ki > 0:
            # Clamp integral to prevent windup
            self.integral = np.clip(self.integral,
                                  -self.integral_limit / self.ki,
                                  self.integral_limit / self.ki)
        i_term = self.ki * self.integral

        # Derivative term with filtering
        if self.last_error is not None:
            derivative = (error - self.last_error) / dt
            self.derivative_history.append(derivative)

            # Use filtered derivative (moving average)
            if len(self.derivative_history) > 0:
                filtered_derivative = sum(self.derivative_history) / len(self.derivative_history)
            else:
                filtered_derivative = 0.0

            d_term = self.kd * filtered_derivative
        else:
            d_term = 0.0

        # Calculate total output
        output = p_term + i_term + d_term

        # Clamp output
        output = np.clip(output, self.output_min, self.output_max)

        # Update state
        self.last_error = error
        self.last_time = current_time

        # Store components for logging
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term
        self.last_output = output

        return output

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def get_state(self):
        """Get current controller state for debugging and performance analysis."""
        derivative_avg = sum(self.derivative_history) / len(self.derivative_history) if self.derivative_history else 0
        error_rms = np.sqrt(np.mean(np.square(list(self.error_history)))) if self.error_history else 0

        # Calculate individual PID components for the last update
        p_term = self.kp * (self.last_error if self.last_error is not None else 0)
        i_term = self.ki * self.integral
        d_term = self.kd * derivative_avg

        return {
            'last_error': self.last_error,
            'integral': self.integral,
            'derivative_avg': derivative_avg,
            'error_rms': error_rms,
            'p_term': p_term,           # Proportional component
            'i_term': i_term,           # Integral component
            'd_term': d_term,           # Derivative component
            'total_output': p_term + i_term + d_term,
            'gains': {'kp': self.kp, 'ki': self.ki, 'kd': self.kd}
        }


class AltitudeController:
    """
    Specialized altitude controller for drones using ultrasonic sensors.

    Features:
    - Velocity estimation and control
    - Sensor filtering
    - Adaptive gains based on altitude
    - Safety limits
    """

    def __init__(self,
                 alt_kp=ALTITUDE_PID_TAKEOFF['kp'],
                 alt_ki=ALTITUDE_PID_TAKEOFF['ki'],
                 alt_kd=ALTITUDE_PID_TAKEOFF['kd'],

                 vel_kp=VELOCITY_PID_FLIGHT['kp'],
                 vel_ki=VELOCITY_PID_FLIGHT['ki'],
                 vel_kd=VELOCITY_PID_FLIGHT['kd'],

                 max_velocity=LIMITS['max_velocity'],
                 max_acceleration=LIMITS['max_acceleration'],
                 throttle_hover=THROTTLE['hover'],
                 throttle_min=THROTTLE['min'],
                 throttle_max=THROTTLE['max'],
                 altitude_filter_alpha=FILTERING['altitude_filter_alpha'],
                 velocity_filter_size=FILTERING['velocity_filter_size'],
                 start_timestamp=None,
                 *,
                 csv_logger=None):
        """
        Initialize altitude controller.

        Args:
            alt_kp, alt_ki, alt_kd: PID gains for altitude (position) control
            vel_kp, vel_ki, vel_kd: PID gains for velocity control
            max_velocity: Maximum vertical velocity in m/s
            max_acceleration: Maximum vertical acceleration in m/s^2
            throttle_hover: PWM value for hover throttle
            throttle_min: Minimum throttle PWM value
            throttle_max: Maximum throttle PWM value
            altitude_filter_alpha: Exponential filter coefficient for altitude
            velocity_filter_size: Number of samples for velocity averaging
            csv_logger: Optional injected CSV logger for control-data logging.
                If None (default), a file-writing AltitudeCSVLogger is created
                exactly as before.
        """
        # Position controller (outer loop)
        self.position_pid = PIDController(
            kp=alt_kp, ki=alt_ki, kd=alt_kd,
            output_min=-max_velocity, output_max=max_velocity,
            integral_limit=max_velocity * 0.5
        )

        # Velocity controller (inner loop)
        self.velocity_pid = PIDController(
            kp=vel_kp, ki=vel_ki, kd=vel_kd,
            output_min=-200, output_max=200,  # Throttle adjustment range
            integral_limit=100
        )

        # Throttle parameters
        self.throttle_hover = throttle_hover
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max

        # Limits
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        # Filtering
        self.altitude_filter_alpha = altitude_filter_alpha
        self.filtered_altitude = None

        # Velocity estimation
        self.velocity_history = deque(maxlen=velocity_filter_size)
        self.last_altitude = None
        self.last_altitude_time = None
        self.estimated_velocity = 0.0

        # State tracking
        self.is_active = False
        self.last_update_time = None

        # Throttle smoothing
        self.last_throttle = throttle_hover

        # Initialize CSV logger with session timestamp
        # Determine controller type for data labeling
        if csv_logger is None:
            csv_logger = AltitudeCSVLogger(start_timestamp=start_timestamp, controller_type='takeoff')
        self.csv_logger = csv_logger

        self.__last_reset_time = 0

    def reset(self, timestamp=None):
        """Reset controller state."""
        if timestamp is None:
            timestamp = time.time()

        if timestamp - self.__last_reset_time < 5.0:
            # Prevent multiple resets within 5 second
            return

        self.__last_reset_time = timestamp
        self.position_pid.reset()
        self.velocity_pid.reset()
        self.filtered_altitude = None
        self.velocity_history.clear()
        self.last_altitude = None
        self.last_altitude_time = None
        self.estimated_velocity = 0.0
        self.is_active = False

    def filter_altitude(self, current_altitude):
        """Apply exponential moving average filter to altitude."""
        if self.filtered_altitude is None:
            self.filtered_altitude = current_altitude
        else:
            self.filtered_altitude = (self.altitude_filter_alpha * current_altitude +
                                     (1 - self.altitude_filter_alpha) * self.filtered_altitude)
        return self.filtered_altitude

    def estimate_velocity(self, altitude, current_time):
        """Estimate vertical velocity from altitude changes."""
        if self.last_altitude is not None and self.last_altitude_time is not None:
            dt = current_time - self.last_altitude_time
            if dt > 0:
                velocity = (altitude - self.last_altitude) / dt
                self.velocity_history.append(velocity)

                # Use filtered velocity
                if len(self.velocity_history) > 0:
                    self.estimated_velocity = sum(self.velocity_history) / len(self.velocity_history)

        self.last_altitude = altitude
        self.last_altitude_time = current_time

        return self.estimated_velocity

    def update(self, target_altitude, current_altitude, current_time=None):
        """
        Update altitude controller and return throttle command.

        Args:
            target_altitude: Desired altitude in meters
            current_altitude: Current altitude reading in meters
            current_time: Current time in seconds

        Returns:
            Throttle PWM value (1000-2000)
        """
        if current_time is None:
            current_time = time.time()

        # Estimate velocity
        velocity = self.estimate_velocity(current_altitude, current_time)

        velocity_setpoint = self.position_pid.update(target_altitude, current_altitude, current_time)
        throttle_adjustment = self.velocity_pid.update(velocity_setpoint, velocity, current_time)

        # Calculate final throttle
        throttle = self.throttle_hover + throttle_adjustment

        # Apply limits
        throttle = np.clip(throttle, self.throttle_min, self.throttle_max)

        # Apply rate limiting to prevent sudden jumps
        if 'rate_limit' in THROTTLE:
            max_change = THROTTLE['rate_limit']
            throttle_change = throttle - self.last_throttle
            if abs(throttle_change) > max_change:
                throttle = self.last_throttle + np.sign(throttle_change) * max_change

        # Apply exponential filter for smoother output
        if 'throttle_filter_alpha' in CONTROL:
            alpha = CONTROL['throttle_filter_alpha']
            throttle = alpha * throttle + (1 - alpha) * self.last_throttle

        self.last_throttle = throttle

        self.is_active = True
        self.last_update_time = current_time

        # Record control data for plotting
        if DEBUG.get('plot_data', False):
            control_data = {
                'timestamp': current_time,
                'current_altitude': current_altitude,
                'filtered_altitude': current_altitude,
                'target_altitude': target_altitude,
                'altitude_error': target_altitude - current_altitude,
                'velocity_setpoint': velocity_setpoint,
                'estimated_velocity': velocity,
                'velocity_error': velocity_setpoint - velocity,
                'throttle_adjustment': throttle_adjustment,
                'throttle_output': throttle,
                'position_pid': self.position_pid.get_state(),
                'velocity_pid': self.velocity_pid.get_state()
            }

            # Write to CSV for real-time logging
            self.csv_logger.append(control_data)

        return int(throttle)

    def get_state(self):
        """Get comprehensive controller state for debugging and performance analysis."""
        position_state = self.position_pid.get_state()
        velocity_state = self.velocity_pid.get_state()

        return {
            'filtered_altitude': self.filtered_altitude,
            'estimated_velocity': self.estimated_velocity,
            'position_pid': position_state,
            'velocity_pid': velocity_state,
            'is_active': self.is_active,
            'throttle_hover': self.throttle_hover,
            'throttle_limits': {
                'min': self.throttle_min,
                'max': self.throttle_max
            },
            'velocity_limits': {
                'max': self.max_velocity,
                'max_accel': self.max_acceleration
            },
            'performance_summary': {
                'position_error_rms': position_state.get('error_rms', 0),
                'velocity_error_rms': velocity_state.get('error_rms', 0),
                'position_p_contribution': abs(position_state.get('p_term', 0)),
                'position_i_contribution': abs(position_state.get('i_term', 0)),
                'position_d_contribution': abs(position_state.get('d_term', 0)),
                'velocity_p_contribution': abs(velocity_state.get('p_term', 0)),
                'velocity_i_contribution': abs(velocity_state.get('i_term', 0)),
                'velocity_d_contribution': abs(velocity_state.get('d_term', 0))
            }
        }

    def set_hover_throttle(self, throttle):
        """Update hover throttle estimate."""
        self.throttle_hover = throttle

    def emergency_stop(self):
        """Reset controller for emergency stop."""
        self.reset()
        return self.throttle_min