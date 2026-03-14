"""
Configuration parameters for XY position control system.

This file contains tuning parameters for the PID controllers used in horizontal position stabilization.
Adjust these values based on your drone's characteristics and sky_anchor visual tracking performance.
"""

POSITION_KP = 1.2
POSITION_KI = 0.05
POSITION_KD = 0.8

DEADBAND = 5

POSITION_PID_X = {
    'kp': POSITION_KP,
    'ki': POSITION_KI,
    'kd': POSITION_KD,
}

POSITION_PID_Y = {
    'kp': POSITION_KP,
    'ki': POSITION_KI,
    'kd': POSITION_KD,
}

# Angle/Yaw PID Controller Parameters
# Controls rotation correction based on angle drift
ANGLE_PID = {
    'kp': 5.0,    # Proportional gain - converts angle error (degrees) to yaw rate
    'ki': 0.1,    # Integral gain - eliminates steady-state rotation
    'kd': 1.0,    # Derivative gain - dampens rotation changes
}

MAX_PWM = 1600
MIN_PWM = 1400

# PWM Output Limits
# PWM ranges for roll, pitch, and yaw control
PWM_LIMITS = {
    'neutral': 1500,              # Neutral PWM value (no movement)
    'min_roll': MIN_PWM,            # Minimum roll PWM
    'max_roll': MAX_PWM,            # Maximum roll PWM
    'min_pitch': MIN_PWM,           # Minimum pitch PWM
    'max_pitch': MAX_PWM,           # Maximum pitch PWM
    'min_yaw': MIN_PWM,             # Minimum yaw PWM
    'max_yaw': MAX_PWM,             # Maximum yaw PWM
    'max_correction': 100,
}

# Sensor Filtering
POSITION_FILTERING = {
    'velocity_filter_size': 5,        # Number of samples for velocity averaging
    'angle_filter_alpha': 0.8,        # Exponential filter for angle measurements
}

# Altitude Compensation
# Parameters for converting pixel drift to real-world distances
ALTITUDE_COMPENSATION = {
    'reference_altitude': 2.0,         # Reference altitude for calibration (meters)
    'pixels_per_meter_at_ref': 500,   # Pixels per meter at reference altitude
    'use_adaptive_scaling': True,     # Enable altitude-based scaling
    'min_altitude': 0.5,              # Minimum altitude for compensation (meters)
    'max_altitude': 10.0,             # Maximum altitude for compensation (meters)
}

# Control Loop Parameters
POSITION_CONTROL = {
    'update_rate': 1,               # Control loop frequency in Hz
    'deadband_x': DEADBAND,              # Position error deadband in meters (reduced from 0.05 for better precision)
    'deadband_y': DEADBAND,              # Position error deadband in meters (reduced from 0.05 for better precision)
    'deadband_angle': 1.0,           # Angle error deadband in degrees
    'integral_limit': 1,            # REDUCED from 1.0 for tighter anti-windup protection
}


# Coordinate System Configuration
COORDINATE_SYSTEM = {
    'invert_x': True,                # Invert X-axis if needed for your setup
    'invert_y': True,               # Invert Y-axis if needed for your setup
    'invert_angle': True,            # Invert angle if needed for your setup
    'use_ned_frame': False,          # Use North-East-Down frame (False = use camera frame)
}