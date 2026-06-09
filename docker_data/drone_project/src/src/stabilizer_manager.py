#!/usr/bin/env python3
"""
Stabilizer Manager
Manages sky anchor stabilizer process and client connection
"""

import sys
import subprocess
import threading
import time
from typing import Optional

from src.domain.types import StabilizerReading
from src.sky_anchor_client import SkyAnchorClient
from src.logger import get_logger
from src import controller_config


class StabilizerManager:
    """
    Manages the sky anchor stabilizer system.

    Responsibilities:
    - Start stabilizer process
    - Connect to stabilizer server
    - Poll stabilizer readings and decide their freshness (Information
      Expert: the timestamp de-duplication that used to live in
      DroneController.__updateThrottle is owned here since GRASP Step 4)
    - Manage connection state
    """

    def __init__(self, stabilizer_path: str, host: str = 'localhost', port: int = 8888, logger=None) -> None:
        """
        Initialize stabilizer manager.

        Args:
            stabilizer_path: Path to sky_anchor main script
            host: Host where stabilizer server runs
            port: Port for stabilizer server
            logger: Optional logger instance
        """
        self.__logger = logger or get_logger(
            "stabilizer_manager",
            "logs/stabilizer_manager.log",
            log_level=controller_config.LOG_LEVEL
        )
        self.__stabilizer_path = stabilizer_path
        self.__client = SkyAnchorClient(host, port)

        self.__process_started = False
        self.__should_connect = False
        self.__connected = False

        self.__running = False
        self.__thread: Optional[threading.Thread] = None
        self.__connection_check_interval = 0.1

        # Timestamp of the last reading handed out by poll_new().  Starts at
        # 0 exactly like the controller's former __last_xy_update field.
        self.__last_consumed_timestamp = 0

    def start_stabilizer_process(self) -> None:
        """Start the sky anchor stabilizer process and connection thread."""
        if self.__process_started:
            self.__logger.warning("Stabilizer process already started")
            return

        python_exe = sys.executable
        subprocess.Popen(
            [python_exe, self.__stabilizer_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True
        )

        self.__logger.warning(f"Sky anchor process started from {self.__stabilizer_path}")
        self.__should_connect = True
        self.__process_started = True

        self.__start_connection_thread()

    def __start_connection_thread(self) -> None:
        """Start background thread for connection attempts."""
        if self.__running:
            return

        self.__running = True
        self.__thread = threading.Thread(target=self.__connection_loop, daemon=True)
        self.__thread.start()
        self.__logger.info("Stabilizer connection thread started")

    def __connection_loop(self) -> None:
        """Background thread that attempts to connect to stabilizer."""
        while self.__running:
            try:
                if self.__should_connect and not self.__connected:
                    result = self.__client.connect()
                    if result:
                        self.__should_connect = False
                        self.__connected = True
                        self.__logger.warning("Connected to stabilizer successfully!")
                        self.__running = False
                        break

                time.sleep(self.__connection_check_interval)

            except Exception as e:
                self.__logger.error(f"Error in connection loop: {e}")
                time.sleep(self.__connection_check_interval)

        self.__logger.info("Stabilizer connection thread terminated")

    def attempt_connection(self) -> bool:
        """
        Attempt to connect to stabilizer if needed.

        Deprecated: Connection now happens automatically in background thread.
        This method is kept for backward compatibility.

        Returns:
            True if connected, False otherwise
        """
        return self.__connected

    def poll_new(self) -> Optional[StabilizerReading]:
        """
        Return the latest stabilizer reading if it has not been consumed yet.

        Freshness is decided by the producer-side timestamp: a reading is
        handed out at most once (the same timestamp is never returned twice).
        Returns None when not connected, when nothing has arrived yet, or
        when the latest reading was already consumed by a previous call.
        """
        if not self.__connected:
            return None

        try:
            reading = self.__client.tick()
        except Exception as e:
            self.__logger.error(f"Error getting stabilizer data: {e}")
            return None

        if reading is None:
            return None
        if reading.timestamp == self.__last_consumed_timestamp:
            return None

        self.__last_consumed_timestamp = reading.timestamp
        return reading

    @property
    def is_connected(self) -> bool:
        """Check if connected to stabilizer."""
        return self.__connected

    @property
    def is_process_started(self) -> bool:
        """Check if stabilizer process was started."""
        return self.__process_started

    def cleanup(self) -> None:
        """Clean up resources."""
        self.__running = False
        if self.__thread and self.__thread.is_alive():
            self.__thread.join(timeout=2.0)
        # Note: We don't kill the subprocess as it may need to continue running
        # The client connection will be closed automatically
        self.__logger.info("StabilizerManager cleanup complete")