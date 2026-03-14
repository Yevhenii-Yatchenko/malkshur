#!/usr/bin/env python3
"""
MAVLink Connection Manager
Handles all MAVLink communication with the flight controller
"""

from typing import Optional, Dict
from pymavlink import mavutil
import pyudev
import time

from src.logger import get_logger
from src import controller_config


class MAVLinkManager:
    """
    Manages MAVLink connection to flight controller.

    Responsibilities:
    - Establish and maintain MAVLink connection (TCP or USB)
    - Configure message streaming rates
    - Send flight commands (arm, disarm, mode changes)
    - Send RC channel overrides
    - Query flight status
    """

    def __init__(self) -> None:
        """Initialize MAVLink manager."""
        self.__logger = get_logger("mavlink_manager", "logs/mavlink_manager.log", log_level=controller_config.LOG_LEVEL)
        self.__master: Optional[mavutil.mavlink_connection] = None
        self.__connection_type = controller_config.MAVLINK_CONNECTION['type']

    def connect(self) -> mavutil.mavlink_connection:
        """
        Establish MAVLink connection based on configuration.

        Returns:
            MAVLink connection object

        Raises:
            RuntimeError: If connection fails
        """
        if self.__connection_type == 'tcp':
            self.__connect_tcp()
        else:
            self.__connect_usb()

        # Wait for heartbeat
        self.__logger.info("Waiting for heartbeat from drone...")
        self.__master.wait_heartbeat()
        self.__logger.info("Heartbeat received, connection established!")

        # Configure message rates
        self.__configure_message_rates()

        return self.__master

    def __connect_tcp(self) -> None:
        """Connect to ArduPilot SITL via TCP (for Gazebo simulation)."""
        host = controller_config.MAVLINK_CONNECTION['host']
        port = controller_config.MAVLINK_CONNECTION['port']

        # Try primary port first
        connection_strings = [f"tcp:{host}:{port}"]
        # Add fallback ports
        for fallback_port in controller_config.MAVLINK_FALLBACK_PORTS:
            if fallback_port != port:
                connection_strings.append(f"tcp:{host}:{fallback_port}")

        connected = False
        for conn_str in connection_strings:
            try:
                self.__logger.info(f"Trying TCP connection: {conn_str}")
                self.__master = mavutil.mavlink_connection(conn_str, autoreconnect=True)
                connected = True
                self.__logger.info(f"Connected via TCP to {conn_str}")
                break
            except Exception as e:
                self.__logger.warning(f"Failed to connect to {conn_str}: {e}")
                continue

        if not connected:
            raise RuntimeError(f"Cannot connect to ArduPilot SITL at {host}")

    def __connect_usb(self) -> None:
        """Connect to flight controller via USB (for real hardware)."""
        port = self.__select_usb_port(
            controller_config.MAVLINK_CONNECTION['vid'],
            controller_config.MAVLINK_CONNECTION['pid']
        )
        if not port:
            raise RuntimeError(
                f"Cannot find USB device with VID={controller_config.MAVLINK_CONNECTION['vid']}, "
                f"PID={controller_config.MAVLINK_CONNECTION['pid']}"
            )

        self.__logger.info(f"Connecting to USB port: {port}")
        self.__master = mavutil.mavlink_connection(
            port,
            baud=controller_config.MAVLINK_CONNECTION['baud'],
            autoreconnect=True
        )

    @staticmethod
    def __select_usb_port(vid: str, pid: str) -> Optional[str]:
        """
        Select the appropriate USB serial port based on VID/PID.

        Args:
            vid: USB Vendor ID
            pid: USB Product ID

        Returns:
            Device path or None if not found
        """
        context = pyudev.Context()
        for device in context.list_devices(subsystem='tty'):
            if device.get('ID_VENDOR_ID') == vid and device.get('ID_MODEL_ID') == pid:
                return device.device_node
        return None

    def __configure_message_rates(self) -> None:
        """Configure MAVLink message streaming rates based on mode."""
        if controller_config.USE_GAZEBO:
            # High rate updates for barometer mode
            self.__master.mav.request_data_stream_send(
                self.__master.target_system,
                self.__master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                controller_config.BAROMETER_UPDATE_RATE,
                1
            )
            self.__configure_high_rate_messages()
        else:
            # Standard rate for LIDAR mode
            self.__master.mav.request_data_stream_send(
                self.__master.target_system,
                self.__master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                1,
                1
            )

    def __configure_high_rate_messages(self) -> None:
        """
        Request high-frequency updates for key altitude messages (Gazebo mode).

        """
        interval_us = int(1_000_000 / controller_config.BAROMETER_UPDATE_RATE)
        message_ids = (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
        )

        for message_id in message_ids:
            try:
                self.__master.mav.command_long_send(
                    self.__master.target_system,
                    self.__master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
            except Exception as exc:
                self.__logger.warning(f"Failed to set message interval for id {message_id}: {exc}")

    def send_rc_override(self, roll: int, pitch: int, throttle: int, yaw: int) -> None:
        """
        Send RC channel override command to flight controller.

        Args:
            roll: Roll PWM value (1000-2000, neutral=1500)
            pitch: Pitch PWM value (1000-2000, neutral=1500)
            throttle: Throttle PWM value (1000-2000)
            yaw: Yaw PWM value (1000-2000, neutral=1500)
        """
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[0] = roll
        rc_channel_values[1] = pitch
        rc_channel_values[2] = throttle
        rc_channel_values[3] = yaw

        self.__logger.debug(
            f"RC override - Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}"
        )

        self.__master.mav.rc_channels_override_send(
            self.__master.target_system,
            self.__master.target_component,
            *rc_channel_values
        )

    def set_mode(self, mode: str) -> None:
        """
        Change flight mode.

        Args:
            mode: Flight mode name (e.g., 'GUIDED', 'STABILIZE', 'ALT_HOLD')
        """
        if not self.__master or not self.__master.target_system:
            self.__logger.error("Error: Not connected to the drone.")
            return

        mode_id = self.__master.mode_mapping()[mode]
        self.__master.mav.set_mode_send(
            self.__master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.__logger.info(f"Requested to set mode to {mode}")

    def arm(self) -> bool:
        """Arm the drone motors with timeout."""
        self.__logger.info("Sending arm command...")

        # Check current mode first
        if hasattr(self.__master, 'flightmode'):
            self.__logger.info(f"Current flight mode: {self.__master.flightmode}")

        # Send arm command
        self.__master.arducopter_arm()

        # Wait for arming with timeout (max 3 seconds)
        timeout = 3
        start_time = time.time()

        while not self.__master.motors_armed():
            if time.time() - start_time > timeout:
                self.__logger.error(f"Arming failed - timeout after {timeout} seconds")
                self.__logger.error("Possible causes:")
                self.__logger.error("  1. Pre-arm checks failed (check GPS, IMU, battery)")
                self.__logger.error("  2. Wrong flight mode (try STABILIZE first)")
                self.__logger.error("  3. Safety switch engaged (if hardware)")
                self.__logger.error("  4. Throttle not at minimum")

                # Try to get more info about why arming failed
                if hasattr(self.__master, 'flightmode'):
                    self.__logger.error(f"  Current mode: {self.__master.flightmode}")
                return False
            time.sleep(0.1)

        self.__logger.info("Drone armed successfully!")
        return True

    def disarm(self) -> None:
        """Disarm the drone motors with timeout."""
        self.__logger.info("Sending disarm command...")
        self.__master.arducopter_disarm()

        # Wait for disarming with timeout (max 5 seconds)
        timeout = 5
        start_time = time.time()

        while self.__master.motors_armed():
            if time.time() - start_time > timeout:
                self.__logger.error(f"Disarming failed - timeout after {timeout} seconds")
                return
            time.sleep(0.1)

        self.__logger.info("Drone disarmed!")

    @property
    def is_armed(self) -> bool:
        """Check if the drone is currently armed."""
        return self.__master.motors_armed()

    @property
    def is_landing(self) -> bool:
        """Check if the drone is in landing mode."""
        if hasattr(self.__master, 'flightmode'):
            return self.__master.flightmode == 'LAND'
        return False

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Get current attitude (roll, pitch, yaw).

        Returns:
            Dictionary with roll, pitch, yaw in radians and timestamp, or None
        """
        try:
            msg = self.__master.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
            if msg:
                return {
                    'roll': msg.roll,
                    'pitch': msg.pitch,
                    'yaw': msg.yaw,
                    'timestamp': time.time()
                }
            else:
                self.__logger.warning("No ATTITUDE message received")
                return None
        except Exception as e:
            self.__logger.error(f"Error getting attitude: {e}")
            return None

    @property
    def master(self) -> mavutil.mavlink_connection:
        """Get the underlying MAVLink connection object."""
        return self.__master

    def close(self) -> None:
        """Close the MAVLink connection."""
        if self.__master:
            self.__master.close()
            self.__logger.info("MAVLink connection closed")