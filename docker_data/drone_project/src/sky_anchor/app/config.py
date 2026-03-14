from __future__ import annotations

import os
from typing import Optional

from dotenv import load_dotenv


APP_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(APP_DIR, '..', '..'))
DEFAULT_SKY_ANCHOR_LOG_PATH = os.path.join(REPO_ROOT, 'logs', 'sky_anchor_main.log')


def get_bool_env(var_name: str, default: bool = False) -> bool:
    val = os.environ.get(var_name, str(default))
    return val.lower() in ("1", "true", "yes", "on")


dotenv_path = os.path.join(APP_DIR, '..', '.env')
load_dotenv(dotenv_path)

# Camera & Flight Controller
DRONE_CAMERA_TYPE: str = os.environ.get("DRONE_CAMERA_TYPE", "USB")
CAMERA_INDEX: int = int(os.environ.get("DRONE_CAMERA_INDEX", 0))
CAPTURE_WIDTH: int = int(os.environ.get("DRONE_CAPTURE_WIDTH", 1280))
CAPTURE_HEIGHT: int = int(os.environ.get("DRONE_CAPTURE_HEIGHT", 720))
CAPTURE_FPS: int = int(os.environ.get("DRONE_CAPTURE_FPS", 30))
NORMALIZE_TYPE: int = int(os.environ.get("NORMALIZE_TYPE", 0))

# Gazebo camera topic (for GAZEBO camera type)
GAZEBO_CAMERA_TOPIC: str = os.environ.get("GAZEBO_CAMERA_TOPIC",
    "/world/default/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image")

FC_TYPE: Optional[str] = os.environ.get("DRONE_FC_TYPE", None)
FC_DEVICE: Optional[str] = os.environ.get("DRONE_FC_DEVICE", None)
FC_BAUDRATE: int = int(os.environ.get("DRONE_FC_BAUDRATE", "0"))  # Use "0" as default to avoid TypeError

# Shift & Angle Thresholds
SHIFT_THRESHOLD: float = float(os.environ.get("DRONE_SHIFT_THRESHOLD", 5.0))
ANGLE_THRESHOLD: float = float(os.environ.get("DRONE_ANGLE_THRESHOLD", 3.0))

# ORB Features & Debug
ORB_NFEATURES: int = int(os.environ.get("DRONE_ORB_NFEATURES", 500))
DEBUG_MODE: bool = get_bool_env("DRONE_DEBUG", False)
ENABLE_CUDA: bool = get_bool_env("ENABLE_CUDA", False)

# Sleep Time (used in main.py loop)
SLEEP_TIME: float = float(os.environ.get("DRONE_SLEEP_TIME", 0.01))

# Performance / Visual Test Parameters
PERF_TEST_TIME: int = int(os.environ.get("PERF_TEST_TIME", 60))
PERF_MIN_DIST: float = float(os.environ.get("PERF_MIN_DIST", 10.0))
PERF_MAX_DIST: float = float(os.environ.get("PERF_MAX_DIST", 50.0))
PERF_MIN_ANGLE: float = float(os.environ.get("PERF_MIN_ANGLE", 5.0))
PERF_MAX_ANGLE: float = float(os.environ.get("PERF_MAX_ANGLE", 30.0))

MAX_FEATURE_FAILS: int = int(os.environ.get("MAX_FEATURE_FAILS", 5))
PROCESSING_TIMES_PER_ITER: int = int(os.environ.get("PROCESSING_TIMES_PER_ITER", 5))

# Logging
SKY_ANCHOR_LOG_PATH: str = os.environ.get("SKY_ANCHOR_LOG_PATH", DEFAULT_SKY_ANCHOR_LOG_PATH)
FRAME_DEBUG_DIR: str = os.environ.get("FRAME_DEBUG_DIR", "debug_logs")

# Profiling
PROFILE_SAMPLE_FRAMES: int = int(os.environ.get("PROFILE_SAMPLE_FRAMES", 120))

# Navigation System (always enabled, inactive unless target is set)
NAV_MAX_REF_AGE_FRAMES: int = int(os.environ.get("NAV_MAX_REF_AGE_FRAMES", 300))
NAV_MAX_DRIFT_PIXELS: float = float(os.environ.get("NAV_MAX_DRIFT_PIXELS", 20000.0))
NAV_UPDATE_ON_WAYPOINT: bool = get_bool_env("NAV_UPDATE_ON_WAYPOINT", True)
NAV_MAX_STEP_PIXELS: float = float(os.environ.get("NAV_MAX_STEP_PIXELS", 5.0))
NAV_PROPORTIONAL_GAIN: float = float(os.environ.get("NAV_PROPORTIONAL_GAIN", 1.0))

# Navigation Command Server (TCP API, always enabled)
NAV_COMMAND_SERVER_PORT: int = int(os.environ.get("NAV_COMMAND_SERVER_PORT", 8889))
