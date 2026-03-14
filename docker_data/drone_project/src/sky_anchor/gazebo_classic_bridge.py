#!/usr/bin/env python3
"""
Classic Gazebo Camera Bridge - For Gazebo 11 and older (gazebo.msgs format)
"""

# Apply pygazebo fix before importing
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
fix_script = os.path.join(current_dir, 'fix_pygazebo.py')
if os.path.exists(fix_script):
    import subprocess
    subprocess.run([sys.executable, fix_script], capture_output=True)

import cv2
import numpy as np
import threading
import time
import asyncio
import pygazebo
from pygazebo.msg import image_stamped_pb2
import os
from typing import Optional

DEFAULT_OUTPUT_FPS = 30.0
import sys

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.logger import get_logger


class GazeboClassicCameraBridge:
    """Bridge between classic Gazebo camera topic and OpenCV VideoCapture"""

    def __init__(self, gazebo_host='localhost', gazebo_port=11345,
                 camera_topic='/gazebo/default/iris_demo/iris_demo/gimbal_small_2d/tilt_link/camera/image'):
        """
        Initialize classic Gazebo camera bridge.

        Args:
            gazebo_host: Gazebo master host
            gazebo_port: Gazebo master port (default 11345)
            camera_topic: Gazebo camera topic path
        """
        self.logger = get_logger("gazebo_classic_bridge", "logs/gazebo_camera.log")
        self.gazebo_host = gazebo_host
        self.gazebo_port = gazebo_port
        self.camera_topic = camera_topic

        self.latest_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.RLock()
        self.frame_condition = threading.Condition(self.frame_lock)
        self.running = False
        self.connected = False
        self.frame_count = 0
        self.last_frame_time = 0
        self._last_returned_frame_count = 0
        self._last_returned_time = 0.0
        self._last_returned_timestamp = 0.0

        self.max_output_fps = DEFAULT_OUTPUT_FPS
        self._min_output_interval = 1.0 / self.max_output_fps

        # Frame dimensions (will be updated from first received frame)
        self.frame_width = 640
        self.frame_height = 480

        # Async event loop for pygazebo
        self._loop = None
        self._thread = None
        self._manager = None
        self._subscriber = None

    def start(self):
        """Start the Gazebo camera bridge"""
        if self.running:
            self.logger.warning("Bridge already running")
            return

        self.running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()

        # Wait for connection
        timeout = 10.0
        start_time = time.time()
        while not self.connected and time.time() - start_time < timeout:
            time.sleep(0.1)

        if not self.connected:
            self.logger.error(f"Failed to connect to Gazebo after {timeout} seconds")
            self.stop()
            raise RuntimeError("Failed to connect to classic Gazebo")

        self.logger.info(f"Connected to Gazebo camera at {self.gazebo_host}:{self.gazebo_port}")

    def _run_async_loop(self):
        """Run the async event loop in a thread"""
        asyncio.set_event_loop(asyncio.new_event_loop())
        self._loop = asyncio.get_event_loop()
        self._loop.run_until_complete(self._connect_and_subscribe())

    async def _connect_and_subscribe(self):
        """Connect to Gazebo and subscribe to camera topic"""
        try:
            self.logger.info(f"Connecting to classic Gazebo at {self.gazebo_host}:{self.gazebo_port}")

            # Connect to Gazebo
            self._manager = await pygazebo.connect((self.gazebo_host, self.gazebo_port))
            self.connected = True
            self.logger.info("Connected to Gazebo manager")

            # Subscribe to camera topic
            self.logger.info(f"Subscribing to camera topic: {self.camera_topic}")
            self._subscriber = self._manager.subscribe(
                self.camera_topic,
                'gazebo.msgs.ImageStamped',
                self._camera_callback
            )

            # Keep running
            while self.running:
                await asyncio.sleep(0.1)

        except Exception as e:
            self.logger.error(f"Gazebo connection error: {e}")
            self.connected = False

    def _camera_callback(self, data):
        """Process incoming camera data from Gazebo"""
        try:
            # Parse the message
            msg = image_stamped_pb2.ImageStamped()
            msg.ParseFromString(data)

            # Extract image data
            img = msg.image
            img_data = img.data
            width = img.width
            height = img.height

            # Get pixel format - it's an integer enum, not a string
            # Common formats: 1=RGB8, 2=RGBA8, 5=L8 (grayscale)
            pixel_format = img.pixel_format if hasattr(img, 'pixel_format') else None

            # Update frame dimensions
            self.frame_width = width
            self.frame_height = height

            # Determine format based on data size if format field is not available
            data_size = len(img_data)

            if data_size == width * height * 3:
                # RGB format
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                img_array = img_array.reshape((height, width, 3))
                # Convert RGB to BGR for OpenCV
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            elif data_size == width * height * 4:
                # RGBA format
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                img_array = img_array.reshape((height, width, 4))
                # Convert RGBA to BGR for OpenCV
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
            elif data_size == width * height:
                # Grayscale
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                img_array = img_array.reshape((height, width))
                # Convert to BGR for consistency
                img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
            else:
                self.logger.warning(f"Unexpected data size: {data_size} for {width}x{height} image")
                return

            # Store the frame
            with self.frame_condition:
                self.latest_frame = img_array.copy()
                self.frame_count += 1
                self.last_frame_time = time.time()
                frame_count = self.frame_count
                self.frame_condition.notify_all()

            if frame_count % 30 == 0:  # Log every 30 frames
                fps = self._calculate_fps()
                self.logger.info(f"Received {frame_count} frames, FPS: {fps:.1f}, Size: {width}x{height}")

        except Exception as e:
            self.logger.error(f"Error processing camera data: {e}")

    def read(self):
        """
        Read a frame from the Gazebo camera.

        Returns:
            tuple: (success, frame) where success is bool and frame is numpy array
        """
        with self.frame_condition:
            while self.running:
                has_new_frame = (
                    self.latest_frame is not None and
                    self.frame_count > self._last_returned_frame_count
                )

                if not has_new_frame:
                    self.frame_condition.wait(timeout=0.1)
                    continue

                if self.max_output_fps and self._last_returned_timestamp > 0.0:
                    target_time = self._last_returned_timestamp + self._min_output_interval
                    now = time.perf_counter()
                    if now < target_time:
                        self.frame_condition.wait(timeout=target_time - now)
                        continue

                frame = self.latest_frame.copy()
                frame_id = self.frame_count

                self._last_returned_frame_count = frame_id
                self._last_returned_timestamp = time.perf_counter()
                self._last_returned_time = time.time()
                return True, frame

        return False, None

    def isOpened(self):
        """Check if the bridge is connected and running"""
        return self.connected and self.running

    def get(self, prop):
        """Get camera properties (OpenCV VideoCapture compatible)"""
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return self.frame_width
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return self.frame_height
        elif prop == cv2.CAP_PROP_FPS:
            return self.max_output_fps if self.max_output_fps else self._calculate_fps()
        else:
            return 0

    def set(self, prop, value):
        """Set camera properties (not supported for Gazebo)"""
        if prop == cv2.CAP_PROP_FPS:
            if value and value > 0:
                self.max_output_fps = float(value)
                self._min_output_interval = 1.0 / self.max_output_fps
                self.logger.info(f"Output FPS limit set to {self.max_output_fps:.2f}")
            else:
                self.max_output_fps = None
                self._min_output_interval = 0.0
                self.logger.info("Output FPS limit disabled")
            return True

        self.logger.warning(f"Cannot set property {prop} on Gazebo camera")
        return False

    def release(self):
        """Release the camera bridge"""
        self.stop()

    def stop(self):
        """Stop the Gazebo camera bridge"""
        self.logger.info("Stopping Gazebo camera bridge")
        self.running = False
        self.connected = False
        try:
            with self.frame_condition:
                self.frame_condition.notify_all()
        except AttributeError:
            pass

        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _calculate_fps(self):
        """Calculate current FPS"""
        if self.last_frame_time == 0:
            return 0.0
        return 1.0 / max(0.001, time.time() - self.last_frame_time)


def test_bridge():
    """Test the classic Gazebo camera bridge"""
    # Use host.docker.internal when running in Docker, localhost otherwise
    host = 'host.docker.internal' if 'docker' in os.environ.get('HOSTNAME', '') else 'localhost'

    # Override with host IP if in Docker
    if 'docker' in os.environ.get('HOSTNAME', ''):
        host = 'host.docker.internal'

    logger = get_logger("gazebo_camera_test", "logs/gazebo_camera_test.log")
    logger.info(f"Testing classic Gazebo camera bridge with host: {host}")

    # Create and start bridge
    bridge = GazeboClassicCameraBridge(gazebo_host="10.124.153.35")
    bridge.start()

    # Skip window creation in headless mode
    headless = os.environ.get('DISPLAY') is None
    if not headless:
        cv2.namedWindow('Gazebo Camera', cv2.WINDOW_NORMAL)

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            ret, frame = bridge.read()

            if ret:
                frame_count += 1

                # Add FPS text
                fps = frame_count / (time.time() - start_time)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Display the frame if not headless
                if not headless:
                    cv2.imshow('Gazebo Camera', frame)

                if frame_count % 30 == 0:
                    logger.info(f"Processed {frame_count} frames, FPS: {fps:.1f}, Size: {frame.shape}")
            else:
                logger.debug("No frame available")
                time.sleep(0.01)

            # Exit on 'q' if display is available, otherwise exit after 5 seconds
            if not headless:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            elif time.time() - start_time > 5:  # Run for 5 seconds in headless mode
                logger.info("Test duration complete (5 seconds)")
                break

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    finally:
        bridge.stop()
        if not headless:
            cv2.destroyAllWindows()
        logger.info(f"Test completed. Total frames: {frame_count}")


if __name__ == "__main__":
    test_bridge()
