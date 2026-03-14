from __future__ import annotations

from typing import Optional
import os

import cv2

from app.config import (
    CAMERA_INDEX,
    CAPTURE_FPS,
    CAPTURE_HEIGHT,
    CAPTURE_WIDTH,
    DRONE_CAMERA_TYPE,
)


class CameraInitializer:
    # v4l2-ctl -d /dev/video0 --all
    # v4l2-ctl -d /dev/video0 --list-formats-ext
    def __init__(self, logger) -> None:
        self.logger = logger

    def __get_usb_camera(self) -> Optional[cv2.VideoCapture]:
        self.logger.info("Initializing USB camera...")
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            self.logger.error(f"Cannot open USB camera at index={CAMERA_INDEX}")
            return None

        self.logger.info("Configuring USB camera...")
        # Configure camera for high-speed capture
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # Enable auto-exposure briefly

        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

        # Set fast manual exposure settings for high frame rate
        # cap.set(cv2.CAP_PROP_EXPOSURE, -7)      # Fast exposure (~8ms)
        # cap.set(cv2.CAP_PROP_GAIN, 50)          # Compensate brightness with gain

        # cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)  # Enforce target FPS
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # Minimize buffer delay

        self.logger.info(
            f"USB Camera initialized (index={CAMERA_INDEX}, {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}@{CAPTURE_FPS}fps)."
        )
        return cap

    def __get_sci_camera(self) -> Optional[cv2.VideoCapture]:
        self.logger.info("Initializing SCI (MIPI CSI) camera with GStreamer...")

        def gstreamer_pipeline(
            sensor_id: int = 0,
            capture_width: int = 1280,
            capture_height: int = 720,
            framerate: int = 30,
        ) -> str:
            return (
                f'nvarguscamerasrc sensor-id={sensor_id} ! '
                f'video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, '
                f'format=(string)NV12, framerate={framerate}/1 ! '
                f'nvvidconv ! video/x-raw, format=(string)BGRx ! '
                f'videoconvert ! video/x-raw, format=(string)BGR ! '
                f'appsink max-buffers=1 drop=true sync=false'
            )

        cap = cv2.VideoCapture(
            gstreamer_pipeline(CAMERA_INDEX, CAPTURE_WIDTH, CAPTURE_HEIGHT, CAPTURE_FPS),
            cv2.CAP_GSTREAMER
        )

        if not cap.isOpened():
            self.logger.error("Cannot open SCI camera (MIPI CSI).")
            return None

        self.logger.info(
            f"SCI Camera initialized (sensor_id={CAMERA_INDEX}, {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}@{CAPTURE_FPS}fps).")
        return cap

    def __get_gazebo_camera(self) -> Optional[cv2.VideoCapture]:
        """Get Gazebo camera bridge as VideoCapture-compatible object"""
        self.logger.info("Initializing Gazebo camera bridge...")

        # Import here to avoid dependency when not using Gazebo
        try:
            import sys
            sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            from app.config import GAZEBO_CAMERA_TOPIC

            # Use host.docker.internal when running in Docker
            host = 'host.docker.internal' if 'docker' in os.environ.get('HOSTNAME', '') else 'localhost'

            # Override with actual host IP if provided
            gazebo_host = os.environ.get('GAZEBO_HOST', '10.124.153.35')  # Use your host IP

            # Check if it's a classic Gazebo topic (starts with /gazebo/)
            if GAZEBO_CAMERA_TOPIC.startswith('/gazebo/'):
                self.logger.info("Using classic Gazebo bridge (gazebo.msgs format)")
                from gazebo_classic_bridge import GazeboClassicCameraBridge
                bridge = GazeboClassicCameraBridge(
                    gazebo_host=gazebo_host,
                    camera_topic=GAZEBO_CAMERA_TOPIC
                )

            bridge.start()
            self.logger.info(f"Gazebo camera bridge initialized (host={host}, topic={GAZEBO_CAMERA_TOPIC})")
            return bridge

        except Exception as e:
            self.logger.error(f"Failed to initialize Gazebo camera bridge: {e}")
            return None

    def get_camera(self) -> Optional[cv2.VideoCapture]:
        if DRONE_CAMERA_TYPE.lower() == "usb":
            return self.__get_usb_camera()
        elif DRONE_CAMERA_TYPE.lower() == "sci":
            return self.__get_sci_camera()
        elif DRONE_CAMERA_TYPE.lower() == "gazebo":
            return self.__get_gazebo_camera()
        else:
            self.logger.error(f"Unknown DRONE_CAMERA_TYPE={DRONE_CAMERA_TYPE}")
            return None
