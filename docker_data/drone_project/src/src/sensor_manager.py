#!/usr/bin/env python3
"""
Sensor Manager
Abstraction layer for altitude sensors (LIDAR and Barometer)
"""

from abc import ABC, abstractmethod
from typing import Optional
import time
from pymavlink import mavutil

from src.logger import get_logger
from src.lidar_sensor import LidarSensor
from src.altitude_config import CONTROL
from src import controller_config


class UpdateFrequencyReporter:
    """Tracks and reports update frequency for sensors."""

    def __init__(self, logger, report_interval: float = 1.0, component_name: str = "Sensor"):
        """
        Initialize the frequency reporter.

        Args:
            logger: Logger instance for reporting
            report_interval: Time interval in seconds between reports
            component_name: Name of the component being tracked
        """
        self.__logger = logger
        self.__component_name = component_name
        self.__update_count = 0
        self.__last_frequency_report_time = time.time()
        self.__frequency_report_interval = report_interval

    def record_update(self) -> None:
        """Record an update and report frequency if interval has passed."""
        self.__update_count += 1
        self.__report_update_frequency()

    def __report_update_frequency(self) -> None:
        """Report the update frequency."""
        current_time = time.time()
        elapsed_time = current_time - self.__last_frequency_report_time

        if elapsed_time >= self.__frequency_report_interval:
            updates_per_second = self.__update_count / elapsed_time
            self.__logger.info(f'{self.__component_name} update frequency: {updates_per_second:.2f} Hz ({self.__update_count} updates in {elapsed_time:.2f}s)')

            # Reset counters for next interval
            self.__update_count = 0
            self.__last_frequency_report_time = current_time


class AltitudeSensor(ABC):
    """Abstract base class for altitude sensors."""

    @abstractmethod
    def get_altitude(self) -> Optional[float]:
        """
        Get current altitude measurement.

        Returns:
            Altitude in meters or None if no valid reading
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """Start the sensor."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop the sensor and clean up resources."""
        pass


class LidarAltitudeSensor(AltitudeSensor):
    """LIDAR-based altitude measurement for real hardware."""

    def __init__(self, measurement_rate: int = 200) -> None:
        """
        Initialize LIDAR altitude sensor.

        Args:
            measurement_rate: Measurement rate in Hz
        """
        self.__logger = get_logger("lidar_altitude_sensor", "logs/lidar_altitude_sensor.log", log_level=controller_config.LOG_LEVEL)
        self.__lidar = LidarSensor(measurement_rate=measurement_rate)

    def start(self) -> None:
        """Start the LIDAR sensor."""
        self.__lidar.start()

        if not self.__lidar.wait_for_distance(timeout=2.0):
            self.__logger.error("Failed to get initial LIDAR measurement!")
            self.__lidar.cleanup()
            raise RuntimeError("LIDAR sensor initialization failed")

        self.__logger.info("LIDAR altitude sensor started successfully")

    def get_altitude(self) -> Optional[float]:
        """
        Get altitude from LIDAR sensor.

        Returns:
            Altitude in meters or None
        """
        altitude = self.__lidar.distance

        if altitude is not None:
            self.__logger.debug(f'LIDAR altitude: {altitude:.3f}m')
        else:
            self.__logger.warning('LIDAR: No valid reading')

        return altitude

    def stop(self) -> None:
        """Stop the LIDAR sensor."""
        if self.__lidar:
            self.__lidar.cleanup()
            self.__logger.info("LIDAR altitude sensor stopped")


class BarometerAltitudeSensor(AltitudeSensor):
    """Barometer-based altitude measurement for Gazebo simulation."""

    def __init__(self, mavlink_connection: mavutil.mavlink_connection) -> None:
        """
        Initialize barometer altitude sensor.

        Args:
            mavlink_connection: MAVLink connection to read barometer data
        """
        self.__logger = get_logger("barometer_altitude_sensor", "logs/barometer_altitude_sensor.log", log_level=controller_config.LOG_LEVEL)
        self.__mavlink = mavlink_connection

        self.__last_value = 0.0
        self.__frequency_reporter = UpdateFrequencyReporter(self.__logger, component_name="Barometer")

    def start(self) -> None:
        """Start the barometer sensor (no initialization needed)."""
        self.__logger.info("Using barometer for altitude (Gazebo mode)")

    def get_altitude(self) -> Optional[float]:
        """
        Get altitude from barometer with caching.

        Returns:
            Altitude in meters or None
        """
        return self.__read_barometer_altitude()

    def __read_barometer_altitude(self) -> Optional[float]:
        """
        Read altitude from barometer/MAVLink messages.

        Returns:
            Altitude in meters or None
        """
        try:
            # Use LOCAL_POSITION_NED for accurate local altitude in Gazebo
            msg = self.__mavlink.recv_match(type='VFR_HUD', blocking=False)
            # self.__logger.info(f"Barometer msg {msg}")

            if msg and hasattr(msg, 'alt'):
                self.__last_value = msg.alt
                self.__frequency_reporter.record_update()
                self.__logger.debug(f'Barometer altitude: {self.__last_value:.3f}m (VFR_HUD)')

        except Exception as e:
            self.__logger.error(f'Error reading barometer altitude: {e}')

        return self.__last_value

    def stop(self) -> None:
        """Stop the barometer sensor (no cleanup needed)."""
        self.__logger.info("Barometer altitude sensor stopped")


class SensorManager:
    """Factory for creating altitude sensors based on configuration."""

    @staticmethod
    def create_sensor(mavlink_connection: Optional[mavutil.mavlink_connection] = None) -> AltitudeSensor:
        """
        Create appropriate altitude sensor based on configuration.

        Args:
            mavlink_connection: MAVLink connection (required for barometer mode)

        Returns:
            AltitudeSensor instance

        Raises:
            ValueError: If configuration is invalid
        """
        if controller_config.ALTITUDE_SOURCE == 'lidar':
            return LidarAltitudeSensor(measurement_rate=CONTROL['update_rate'])
        elif controller_config.ALTITUDE_SOURCE == 'barometer':
            if not mavlink_connection:
                raise ValueError("MAVLink connection required for barometer mode")
            return BarometerAltitudeSensor(mavlink_connection)
        else:
            raise ValueError(f"Unknown altitude source: {controller_config.ALTITUDE_SOURCE}")

    @staticmethod
    def get_altitude_source() -> str:
        """Get the configured altitude source."""
        return controller_config.ALTITUDE_SOURCE