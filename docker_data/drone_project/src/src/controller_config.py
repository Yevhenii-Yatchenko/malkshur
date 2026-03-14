"""
Controller configuration for switching between real hardware and Gazebo simulation
"""

import os

# Check if we're running in Gazebo simulation mode
USE_GAZEBO = os.environ.get('USE_GAZEBO', 'false').lower() == 'true'

# MAVLink connection configuration
if USE_GAZEBO:
    # Gazebo/ArduPilot SITL configuration
    MAVLINK_CONNECTION = {
        'type': 'tcp',
        'host': os.environ.get('MAVLINK_HOST', 'host.docker.internal'),
        'port': int(os.environ.get('MAVLINK_PORT', 11345)),
        'baud': None,  # Not used for TCP
    }
    # Alternative ports to try if primary fails
    MAVLINK_FALLBACK_PORTS = [11345, 5763, 5762, 14550, 14551]
else:
    # Real hardware configuration
    MAVLINK_CONNECTION = {
        'type': 'usb',
        'vid': '1a86',  # USB Vendor ID
        'pid': '7523',  # USB Product ID
        'baud': 57600,
    }
    MAVLINK_FALLBACK_PORTS = []

# Altitude sensor configuration
ALTITUDE_SOURCE = 'barometer' if USE_GAZEBO else 'lidar'

# Battery monitoring
ENABLE_BATTERY_MONITOR = not USE_GAZEBO

# Sky anchor subprocess
if USE_GAZEBO:
    # In Docker/Gazebo, sky_anchor runs in same container
    # Allow override via environment variable for non-Docker Gazebo setups
    SKY_ANCHOR_PATH = os.environ.get('SKY_ANCHOR_PATH', '/drone_project/sky_anchor/main.py')
else:
    # Real hardware path
    SKY_ANCHOR_PATH = '/home/jetson/Documents/DroneProject/sky_anchor/main.py'

# Logging
LOG_LEVEL = os.environ.get('DRONE_LOG_LEVEL', 'INFO')

# High rate message configuration for barometer mode
BAROMETER_UPDATE_RATE = 100  # Hz (for Gazebo mode)

ARM_IN = float(os.environ.get('ARM_IN', '0').lower())