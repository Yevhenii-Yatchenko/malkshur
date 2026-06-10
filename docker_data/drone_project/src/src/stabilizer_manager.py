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

    def __init__(self, stabilizer_path: str, host: str = 'localhost', port: int = 8888, logger=None,
                 client: Optional[SkyAnchorClient] = None) -> None:
        """
        Initialize stabilizer manager.

        Args:
            stabilizer_path: Path to sky_anchor main script
            host: Host where stabilizer server runs (used only when no
                client is injected)
            port: Port for stabilizer server (used only when no client is
                injected)
            logger: Optional logger instance
            client: Optional injected SkyAnchorClient (GRASP Step 7, LC-3;
                the composition root passes one explicitly).  If None
                (default), a SkyAnchorClient(host, port) is created exactly
                as before.
        """
        self.__logger = logger or get_logger(
            "stabilizer_manager",
            "logs/stabilizer_manager.log",
            log_level=controller_config.LOG_LEVEL
        )
        self.__stabilizer_path = stabilizer_path
        self.__client = client if client is not None else SkyAnchorClient(host, port)

        self.__process_started = False
        self.__should_connect = False
        self.__connected = False

        self.__running = False
        self.__thread: Optional[threading.Thread] = None
        self.__connection_check_interval = 0.1

        # Timestamp of the last reading handed out by poll_new().  Starts at
        # 0 exactly like the controller's former __last_xy_update field.
        self.__last_consumed_timestamp = 0

        # Version-skew tripwire latch (Step 7 carried bullet), see poll_new().
        self.__skew_warned = False

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

    def poll_new(self) -> Optional[StabilizerReading]:
        """
        Return the latest stabilizer reading if it has not been consumed yet.

        Freshness is decided by the producer-side timestamp: a reading is
        handed out at most once (the same timestamp is never returned twice).
        Returns None when not connected, when nothing has arrived yet, or
        when the latest reading was already consumed by a previous call.
        """
        if not self.is_connected:
            return None

        try:
            reading = self.__client.tick()
        except Exception as e:
            self.__logger.error(f"Error getting stabilizer data: {e}")
            return None

        if reading is None:
            return None

        # Version-skew tripwire (Step 7 carried bullet): a navigation frame
        # from a producer that predates the explicit ``navigation`` flag
        # (GRASP Step 4) still carries the historic matches_percent
        # placeholder (101.0) but parses with navigation=False -- meaning
        # navigation frames would silently be treated as stabilization
        # frames.  Warn once per process, never drop the reading.
        if (not self.__skew_warned
                and reading.matches_percent > 100
                and not reading.navigation):
            self.__skew_warned = True
            self.__logger.warning(
                "Stabilizer payload skew: matches_percent="
                f"{reading.matches_percent} arrived with navigation=False -- "
                "the sky_anchor producer likely predates the explicit "
                "navigation flag; navigation frames will be treated as "
                "stabilization frames"
            )

        if reading.timestamp == self.__last_consumed_timestamp:
            return None

        self.__last_consumed_timestamp = reading.timestamp
        return reading

    @property
    def is_connected(self) -> bool:
        """Check if connected to stabilizer, re-synced with client health.

        GRASP Step 5 carried bullet: the manager's flag is flipped True once
        by the connection thread, but the client's receive thread can die
        later (server disconnect, receive error).  Without this re-sync the
        manager would report "connected" forever, silently wedging both the
        stabilization gate and the post-intercept re-enable.  The drop is
        one-way and logged once; reconnect handling is intentionally
        unchanged (none beyond the existing connection thread).
        """
        if self.__connected and not self.__client.is_connected():
            self.__connected = False
            self.__logger.warning(
                "Stabilizer client connection lost (receive thread down)"
            )
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