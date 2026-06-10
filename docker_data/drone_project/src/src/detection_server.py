#!/usr/bin/env python3
"""
Detection Server - Monitors and receives object recognition data from Docker client
"""

import json
import socket
import threading
import time
from datetime import datetime
from typing import Optional, Dict, Any, Tuple

from src.domain.types import DetectionReading
from src.logger import get_logger
from src.detection_config import (
    DETECTION_SERVER_HOST,
    DETECTION_SERVER_PORT,
    DETECTION_SERVER_LOG,
    INTERCEPT_CONFIDENCE_THRESHOLD,
    INTERCEPT_TIMEOUT_SECONDS,
)


class DetectionServer:
    """
    TCP server that receives detection data from Docker recognition client.

    Runs in background thread and maintains latest detection data.
    Thread-safe access to detection results.
    """

    def __init__(
        self,
        host: str = DETECTION_SERVER_HOST,
        port: int = DETECTION_SERVER_PORT,
        log_file: str = DETECTION_SERVER_LOG,
        logger=None,
        intercept_timeout_s: float = INTERCEPT_TIMEOUT_SECONDS,
        intercept_min_confidence: float = INTERCEPT_CONFIDENCE_THRESHOLD,
    ):
        """
        Initialize detection server.

        Args:
            host: Server bind address
            port: Server port
            log_file: Log file path
            logger: Optional external logger
            intercept_timeout_s: Default maximum age of the latest detection
                for ``get_active_target`` (GRASP Step 7 carried bullet: the
                server owns the INTERCEPT_* thresholds; the controller no
                longer passes them per call)
            intercept_min_confidence: Default inclusive confidence floor for
                ``get_active_target``
        """
        self.host = host
        self.port = port
        self.log_file = log_file
        self.intercept_timeout_s = intercept_timeout_s
        self.intercept_min_confidence = intercept_min_confidence
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        self.server_socket: Optional[socket.socket] = None

        # Latest detection data (thread-safe)
        self._latest_data: Optional[Dict[str, Any]] = None
        self._data_lock = threading.Lock()
        self._last_update_time: float = 0.0

        # Logger
        self.logger = logger or get_logger(
            "detection_server",
            log_file,
            log_level="INFO"
        )

    def start(self) -> bool:
        """
        Start the detection server in a background thread.

        Returns:
            True if started successfully, False otherwise
        """
        if self.is_running():
            self.logger.warning("Detection server already running")
            return False

        self.logger.info(f"Starting detection server on {self.host}:{self.port}")
        self.running = True
        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=True,
            name="DetectionServerThread"
        )
        self.server_thread.start()
        time.sleep(0.2)

        if self.is_running():
            self.logger.warning("Detection server started successfully")
            return True
        else:
            self.logger.error("Detection server failed to start")
            return False

    def stop(self) -> bool:
        """
        Stop the detection server gracefully.

        Returns:
            True if stopped successfully, False otherwise
        """
        if not self.is_running():
            self.logger.warning("Detection server not running")
            return False

        self.logger.info("Stopping detection server")
        self.running = False

        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception as e:
                self.logger.error(f"Error closing server socket: {e}")

        # Wait for thread to finish
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=2)

        self.logger.warning("Detection server stopped")
        return True

    def is_running(self) -> bool:
        """Check if server is currently running."""
        return self.running and self.server_thread and self.server_thread.is_alive()

    def get_latest_detection(self) -> Optional[Dict[str, Any]]:
        """
        Get the latest detection data (thread-safe).

        Returns:
            Dictionary with detection data or None if no data available
        """
        with self._data_lock:
            return self._latest_data.copy() if self._latest_data else None

    def get_time_since_last_detection(self) -> float:
        """
        Get time elapsed since last detection update (seconds).

        Returns:
            Elapsed time in seconds, or float('inf') if no data received yet
        """
        with self._data_lock:
            if self._last_update_time == 0.0:
                return float('inf')
            return time.time() - self._last_update_time

    def get_active_target(
        self,
        timeout_s: Optional[float] = None,
        min_confidence: Optional[float] = None,
    ) -> Optional[DetectionReading]:
        """
        Return the current detection as a typed reading if it is actionable.

        Owns the data-validity decisions that previously lived in
        DroneController.__updateThrottle (GRASP Step 4, IE-1/IE-2): a target
        is active only if a detection exists, it arrived less than
        ``timeout_s`` seconds ago, and its confidence is at least
        ``min_confidence``.  Direction-vector extraction (including the
        historic defaults for missing fields) happens in
        DetectionReading.from_payload.

        Args:
            timeout_s: Maximum age of the latest detection (strictly less
                than, matching the former ``< INTERCEPT_TIMEOUT_SECONDS``).
                None (default) uses the constructor-owned threshold (GRASP
                Step 7 carried bullet).
            min_confidence: Inclusive confidence floor (matching the former
                ``>= INTERCEPT_CONFIDENCE_THRESHOLD``).  None (default) uses
                the constructor-owned threshold.

        Returns:
            DetectionReading for an active target, or None.  Malformed
            payloads (non-object JSON, wrong field types -- _handle_client
            stores whatever json.loads returned) are logged and treated as
            no target; they never raise into the control loop.
        """
        if timeout_s is None:
            timeout_s = self.intercept_timeout_s
        if min_confidence is None:
            min_confidence = self.intercept_min_confidence
        try:
            data = self.get_latest_detection()
            if not data:
                return None
            if self.get_time_since_last_detection() >= timeout_s:
                return None
            reading = DetectionReading.from_payload(data)
        except (TypeError, ValueError, AttributeError, KeyError) as e:
            self.logger.error(f"Dropping malformed detection payload: {e}")
            return None

        if reading.confidence < min_confidence:
            return None
        return reading

    def _log(self, message: str) -> None:
        """Write message to log file with timestamp."""
        try:
            with open(self.log_file, 'a') as f:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                f.write(f"[{timestamp}] {message}\n")
                f.flush()
        except Exception as e:
            self.logger.error(f"Failed to write to log file: {e}")

    def _run_server(self) -> None:
        """Main server loop (runs in background thread)."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(100)
            self.server_socket.settimeout(1.0)

            msg = f"Detection server listening on {self.host}:{self.port}"
            self.logger.info(msg)
            self._log(msg)

            while self.running:
                try:
                    conn, addr = self.server_socket.accept()
                    msg = f"Client connected: {addr[0]}:{addr[1]}"
                    self.logger.info(msg)
                    self._log(msg)
                    threading.Thread(
                        target=self._handle_client,
                        args=(conn, addr),
                        daemon=True
                    ).start()
                except socket.timeout:
                    continue
                except OSError:
                    break
                except Exception as e:
                    if self.running:
                        msg = f"Server error: {e}"
                        self.logger.error(msg)
                        self._log(msg)

        except Exception as e:
            msg = f"Fatal server error: {e}"
            self.logger.error(msg)
            self._log(msg)
        finally:
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass

    def _handle_client(self, conn: socket.socket, addr: Tuple[str, int]) -> None:
        """Handle individual client connection and extract detection data."""
        try:
            with conn:
                data = conn.recv(4096)
                if not data:
                    msg = f"Empty data from {addr[0]}:{addr[1]}"
                    self.logger.warning(msg)
                    self._log(msg)
                    return

                try:
                    data_dict = json.loads(data.decode())

                    # Update latest data (thread-safe)
                    with self._data_lock:
                        self._latest_data = data_dict
                        self._last_update_time = time.time()

                    # Log detection with details
                    timestamp = time.strftime('%H:%M:%S')
                    if 'class_id' in data_dict and 'confidence' in data_dict:
                        conf = data_dict.get('confidence', 0)

                        # Get direction vector info
                        direction_vector = data_dict.get('direction_vector', {})
                        direction = direction_vector.get('direction', [0, 0, 0])
                        dir_x = direction[0] if len(direction) > 0 else 0.0
                        dir_y = direction[1] if len(direction) > 1 else 0.0

                        msg = (
                            f"[{timestamp}] 📥 Detection from {addr[0]}:{addr[1]} - "
                            f"class={data_dict['class_id']}, conf={conf:.2%}, "
                            f"dir_x={dir_x:+.3f}, dir_y={dir_y:+.3f}"
                        )
                        self.logger.warning(msg)
                        self._log(msg)
                    else:
                        msg = f"[{timestamp}] Unknown data format from {addr[0]}:{addr[1]}"
                        self.logger.warning(msg)
                        self._log(msg)

                    # Full data log (DEBUG level)
                    self._log(f"Full data: {json.dumps(data_dict)}")

                    # Send acknowledgment
                    response = {"status": "ok", "timestamp": time.time()}
                    conn.sendall(json.dumps(response).encode())

                except json.JSONDecodeError as e:
                    msg = f"Invalid JSON from {addr[0]}:{addr[1]}: {e}"
                    self.logger.error(msg)
                    self._log(msg)

        except Exception as e:
            msg = f"Client handling error from {addr[0]}:{addr[1]}: {e}"
            self.logger.error(msg)
            self._log(msg)
