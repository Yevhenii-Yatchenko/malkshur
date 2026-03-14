"""
Detection System Configuration
Configures object recognition client (Docker) and server (monitoring)
"""

import os

# Check if we're running in Gazebo simulation mode
USE_GAZEBO = os.environ.get('USE_GAZEBO', 'false').lower() == 'true'

# ============================================================================
# Detection Client (Docker) Configuration
# ============================================================================

# Docker scripts for different environments
DETECTION_DOCKER_SCRIPT_GAZEBO = os.environ.get(
    'DETECTION_DOCKER_SCRIPT_GAZEBO',
    '../jetson-inference/run-rtx4090.sh'
)
DETECTION_DOCKER_SCRIPT_HARDWARE = os.environ.get(
    'DETECTION_DOCKER_SCRIPT_HARDWARE',
    '../jetson-inference/run-jetson.sh'
)

# Select script based on environment
DETECTION_DOCKER_SCRIPT = DETECTION_DOCKER_SCRIPT_GAZEBO if USE_GAZEBO else DETECTION_DOCKER_SCRIPT_HARDWARE

# Docker container name
DETECTION_DOCKER_CONTAINER = os.environ.get(
    'DETECTION_DOCKER_CONTAINER',
    'jetson-rtx4090' if USE_GAZEBO else 'jetson-inference'
)

# Docker password (optional, can be empty string)
DETECTION_DOCKER_PASSWORD = os.environ.get('DETECTION_DOCKER_PASSWORD', '')

# Detection model paths (inside Docker container)
DETECTION_MODEL_GAZEBO = os.environ.get(
    'DETECTION_MODEL_GAZEBO',
    'models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx'
)
DETECTION_MODEL_HARDWARE = os.environ.get(
    'DETECTION_MODEL_HARDWARE',
    'models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx'
)

DETECTION_MODEL = DETECTION_MODEL_GAZEBO if USE_GAZEBO else DETECTION_MODEL_HARDWARE

# Detection labels
DETECTION_LABELS_GAZEBO = os.environ.get(
    'DETECTION_LABELS_GAZEBO',
    'models/ONNXs/labels.txt'
)
DETECTION_LABELS_HARDWARE = os.environ.get(
    'DETECTION_LABELS_HARDWARE',
    'models/ONNXs/labels.txt'
)

DETECTION_LABELS = DETECTION_LABELS_GAZEBO if USE_GAZEBO else DETECTION_LABELS_HARDWARE

# Camera input (inside Docker)
DETECTION_CAMERA_INPUT = os.environ.get(
    'DETECTION_CAMERA_INPUT',
    'csi://0'
)

# Detection threshold
DETECTION_THRESHOLD = float(os.environ.get('DETECTION_THRESHOLD', '0.5'))

# Detection script path inside Docker
DETECTION_SCRIPT_PATH = os.environ.get(
    'DETECTION_SCRIPT_PATH',
    'data/dd_smart_agent/tmp_gazebo.py'
)

# ============================================================================
# Detection Server Configuration
# ============================================================================

# Server listen address and port
DETECTION_SERVER_HOST = os.environ.get('DETECTION_SERVER_HOST', '0.0.0.0')
DETECTION_SERVER_PORT = int(os.environ.get('DETECTION_SERVER_PORT', '5000'))

# Logging
DETECTION_SERVER_LOG = os.environ.get(
    'DETECTION_SERVER_LOG',
    'logs/detection_server.log'
)
DETECTION_CLIENT_LOG = os.environ.get(
    'DETECTION_CLIENT_LOG',
    'logs/detection_client.log'
)

# ============================================================================
# Intercept Mode Parameters
# ============================================================================

# Confidence threshold for target recognition
INTERCEPT_CONFIDENCE_THRESHOLD = float(
    os.environ.get('INTERCEPT_CONFIDENCE_THRESHOLD', '0.5')
)

# Timeout without detection data (seconds)
INTERCEPT_TIMEOUT_SECONDS = float(
    os.environ.get('INTERCEPT_TIMEOUT_SECONDS', '3.0')
)

# Deadband for direction vector components
INTERCEPT_DEADBAND_X = float(
    os.environ.get('INTERCEPT_DEADBAND_X', '0.15')
)  # Yaw deadband
INTERCEPT_DEADBAND_Y = float(
    os.environ.get('INTERCEPT_DEADBAND_Y', '0.2')
)  # Altitude deadband

# Control gains
INTERCEPT_YAW_GAIN = float(
    os.environ.get('INTERCEPT_YAW_GAIN', '100')
)  # PWM per unit of direction_vector[0]

INTERCEPT_ALTITUDE_STEP = float(
    os.environ.get('INTERCEPT_ALTITUDE_STEP', '0.01')
)  # meters per cycle

INTERCEPT_PITCH_OFFSET = int(
    os.environ.get('INTERCEPT_PITCH_OFFSET', '20')
)  # Forward movement PWM offset from neutral (1500)

# Roll control during intercept (keep neutral)
INTERCEPT_ROLL_NEUTRAL = 1500

# Base PWM values
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000
