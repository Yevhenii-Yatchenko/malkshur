"""
Configuration parameters for altitude control system.

This file contains tuning parameters for the PID controllers used in altitude stabilization.
Adjust these values based on your drone's characteristics and sensor setup.
"""

# Altitude PID Controller Parameters
# These control how the drone tracks altitude setpoints
# Note: Gains are tuned for cascade control with velocity loop
# Updated gains to reduce oscillations and improve stability
ALTITUDE_PID_TAKEOFF = {
    'kp': 5,  # Increased for faster initial response
    'ki': 2,   # Increased for better error elimination
    'kd': 5,   # Increased for better damping
}

# Velocity PID Controller Parameters
# These control how the drone tracks velocity setpoints (cascade control)
# Split into separate configurations for flight and hold modes
VELOCITY_PID_FLIGHT = {
    'kp': 100,     # Proportional gain - maintains dynamic response for maneuvers
    'ki': 0,      # Integral gain - allows quick error correction during flight
    'kd': 0,    # Derivative gain - moderate damping for responsiveness
}

# Physical Limits
# Updated for more conservative and stable flight
LIMITS = {
    'max_velocity': 0.8,        # Reduced from 0.4 for improved stability
    'max_acceleration': 1,    # Maximum vertical acceleration in m/s^2
    'max_altitude': 10.0,       # Maximum allowed altitude in meters
    'min_altitude': 0.2,        # Minimum altitude (ground clearance) in meters
}

# Throttle Parameters
THROTTLE = {
    'hover': 1500,      # Updated hover estimate based on log analysis
    'min': 1000,        # Minimum safe throttle - increased to prevent sudden drops
    'max': 1800,        # Maximum safe throttle - reduced for smoother control
    'takeoff_ramp': 5, # Throttle increment during takeoff ramp
    'land_base': 1400,  # Base throttle for landing
    'rate_limit': 100,   # Reduced from 50 for smoother throttle changes
}

# Sensor Filtering
FILTERING = {
    'altitude_filter_alpha': 0.95,  # Increased from 0.9 for faster response
    'velocity_filter_size': 5,      # Reduced from 10 for quicker velocity updates
    'outlier_threshold': 0.3,       # Reject altitude changes larger than this (meters)
}

# Control Loop Parameters
CONTROL = {
    'update_rate': 100,             # Control loop frequency in Hz - reduced for consistent timing
    'deadband': 0.02,               # Altitude error deadband in meters - tighter control
    'integral_limit': 30,           # Anti-windup limit for integral terms - reduced
    'derivative_filter_size': 5,    # Samples for derivative filtering - faster response
    'throttle_filter_alpha': 0.8,   # Exponential filter for throttle smoothing
}

# Takeoff Parameters
TAKEOFF = {
    'initial_throttle': 1100,       # Starting throttle for takeoff
    'max_throttle': 1700,           # Maximum throttle during takeoff ramp
    'climb_detect_threshold': 0.2,  # Altitude gain to detect climb start
    'timeout': 30,                  # Maximum takeoff time in seconds
    'target_tolerance': 0.3,        # Tolerance for reaching target altitude
}

# Landing Parameters
LANDING = {
    'descent_rate': 0.5,            # Target descent rate in m/s
    'final_altitude': 0.1,          # Target altitude for touchdown
    'throttle_reduction_rate': 10,  # Throttle reduction per update cycle
    'min_throttle': 1200,          # Minimum throttle during landing
}

# Safety Parameters
SAFETY = {
    'sensor_timeout': 0.5,          # Maximum age of sensor reading in seconds
    'altitude_rate_limit': 3.0,     # Maximum altitude change rate in m/s
    'emergency_descent_rate': 1.0,  # Descent rate for emergency landing
    'min_battery_voltage': 14.0,    # Minimum battery voltage for flight
}

# Debug/Logging Parameters
# Enhanced logging for PID performance monitoring and tuning
DEBUG = {
    'log_interval': 5,              # Log detailed info every N control cycles (reduced for better monitoring)
    'plot_data': True,              # Enable data collection for plotting and analysis
    'verbose': True,                # Enable verbose debug output for tuning
    'log_pid_components': True,     # Log individual P, I, D terms
    'log_performance_metrics': True, # Log settling time, overshoot, steady-state error
    'log_throttle_saturation': True, # Track when throttle hits limits
    'log_control_timing': True,     # Monitor control loop timing performance
}
