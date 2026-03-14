#!/usr/bin/env python3
"""
Threaded Battery Monitor for MAVLink-based drones
Continuously monitors battery status in background thread
Provides thread-safe access to battery voltage, current, and other metrics
"""

import time
import threading
from collections import deque
from datetime import datetime
import statistics
import pyudev
from pymavlink import mavutil


class BatteryMonitor:
    """
    Thread-safe battery monitor with continuous background measurements

    Features:
    - Separate MAVLink connection for battery monitoring
    - Runs measurements in separate thread
    - Always has latest battery data available
    - Thread-safe access to measurements
    - Built-in filtering and averaging
    - Measurement statistics and health monitoring
    """

    def __init__(self,
                 master,
                 update_rate=1.0,  # Hz
                 averaging_window=10,
                 min_voltage=10.0,  # Minimum safe voltage
                 max_voltage=17.0,  # Maximum voltage (4S LiPo fully charged)
                 critical_voltage=10.5,  # Critical low voltage threshold
                 warning_voltage=11.1,  # Warning voltage threshold
                 controller_logger=None,  # Optional logger for controller integration
                 status_print_interval=1.0):  # Interval for status printing in seconds
        """
        Initialize the Battery Monitor

        Args:
            master: MAVLink connection object
            update_rate: Desired measurements per second (Hz)
            averaging_window: Number of recent measurements to average
            min_voltage: Minimum safe voltage
            max_voltage: Maximum expected voltage
            critical_voltage: Critical low voltage threshold
            warning_voltage: Warning voltage threshold
            controller_logger: Optional logger to write status messages to controller log
            status_print_interval: Interval between status prints in seconds
        """
        # Connection parameters
        self.__master = master

        # Measurement parameters
        self.__update_rate = update_rate
        self.__update_interval = 1.0 / update_rate
        self.__averaging_window = averaging_window
        self.__min_voltage = min_voltage
        self.__max_voltage = max_voltage
        self.__critical_voltage = critical_voltage
        self.__warning_voltage = warning_voltage

        # Status printing
        self.__controller_logger = controller_logger
        self.__status_print_interval = status_print_interval
        self.__last_status_print_time = 0

        # Thread-safe data storage
        self.__lock = threading.Lock()
        self.__latest_voltage = None
        self.__latest_current = None
        self.__latest_remaining = None  # Remaining battery percentage
        self.__latest_timestamp = None
        self.__latest_consumed_mah = None  # Consumed mAh
        self.__latest_energy_consumed = None  # Energy consumed in J
        self.__latest_temperature = None  # Battery temperature if available
        self.__voltage_history = deque(maxlen=averaging_window)
        self.__current_history = deque(maxlen=averaging_window)
        self.__all_measurements = deque(maxlen=1000)  # Keep last 1000 for stats

        # Cell information
        self.__cell_count = None
        self.__cell_voltages = []

        # Statistics
        self.__total_measurements = 0
        self.__failed_measurements = 0
        self.__start_time = time.time()
        self.__max_voltage_seen = 0
        self.__min_voltage_seen = float('inf')
        self.__total_current_drawn = 0  # Total current drawn over time

        # Thread control
        self.__running = False
        self.__thread = None
        self.__reconnect_attempts = 0
        self.__max_reconnect_attempts = 5

        # Logger
        from src.logger import get_logger
        self.__logger = get_logger("battery_monitor", "logs/battery_monitor.log", log_level="INFO")

    def _select_port(self) -> str:
        """Select the appropriate serial port based on VID/PID"""
        context = pyudev.Context()
        for device in context.list_devices(subsystem='tty'):
            if device.get('ID_VENDOR_ID') == self.vid and device.get('ID_MODEL_ID') == self.pid:
                return device.device_node
        return None

    def start(self) -> None:
        """Start the battery monitoring thread"""
        if self.__running:
            return

        try:
            # Request battery status stream
            self.__request_battery_stream()

            # Start monitoring thread
            self.__running = True
            self.__thread = threading.Thread(target=self.__monitoring_loop, daemon=True)
            self.__thread.start()

            # Wait for first measurement
            timeout = time.time() + 5.0
            while self.__latest_voltage is None and time.time() < timeout:
                time.sleep(0.1)

            if self.__latest_voltage is None:
                self.__logger.warning("No battery data received within timeout")

        except Exception as e:
            self.__logger.error(f"Failed to start battery monitor: {e}")
            raise

    def stop(self) -> None:
        """Stop the battery monitoring thread"""
        self.__running = False
        if self.__thread and self.__thread.is_alive():
            self.__thread.join(timeout=2.0)

        if self.__master:
            self.__master.close()
            self.__master = None

    def __request_battery_stream(self) -> None:
        """Request battery status data stream from flight controller"""
        try:
            # Request BATTERY_STATUS at specified rate
            self.__master.mav.request_data_stream_send(
                self.__master.target_system,
                self.__master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                int(self.__update_rate),
                1  # Enable
            )

            # Also specifically request battery status
            self.__master.mav.command_long_send(
                self.__master.target_system,
                self.__master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
                int(1000000 / self.__update_rate),  # Interval in microseconds
                0, 0, 0, 0, 0
            )
        except Exception as e:
            self.__logger.error(f"Failed to request battery stream: {e}")

    def __reconnect(self) -> bool:
        """Attempt to reconnect to MAVLink"""
        if self.__reconnect_attempts >= self.__max_reconnect_attempts:
            self.__logger.error("Max reconnection attempts reached")
            return False

        try:
            self.__reconnect_attempts += 1
            self.__logger.info(f"Reconnection attempt {self.__reconnect_attempts}")

            if self.__master:
                self.__master.close()

            time.sleep(1.0)

            port = self._select_port()
            if not port:
                self.__logger.error("No device found for reconnection")
                return False

            self.__master = mavutil.mavlink_connection(port, baud=self.baudrate, autoreconnect=True)
            self.__master.wait_heartbeat(timeout=5)
            self.__request_battery_stream()

            self.__reconnect_attempts = 0
            self.__logger.info("Reconnection successful")
            return True

        except Exception as e:
            self.__logger.error(f"Reconnection failed: {e}")
            return False

    def __monitoring_loop(self) -> None:
        """Background thread that continuously monitors battery status"""
        consecutive_failures = 0

        while self.__running:
            try:
                # Get battery status message with timeout
                battery_msg = self.__master.recv_match(
                    type='BATTERY_STATUS',
                    blocking=True,
                    timeout=self.__update_interval
                )

                if battery_msg:
                    self.__process_battery_message(battery_msg)
                    self.__print_status()
                    consecutive_failures = 0
                else:
                    # No message received within timeout
                    self.__failed_measurements += 1
                    consecutive_failures += 1

                    if consecutive_failures > 10:
                        self.__logger.warning("Multiple consecutive failures, attempting reconnect")
                        if not self.__reconnect():
                            time.sleep(5.0)

            except Exception as e:
                self.__logger.error(f"Error in monitoring loop: {e}")
                self.__failed_measurements += 1
                consecutive_failures += 1

                if consecutive_failures > 10:
                    if not self.__reconnect():
                        time.sleep(5.0)
                else:
                    time.sleep(self.__update_interval)

    def __process_battery_message(self, msg) -> None:
        """Process a BATTERY_STATUS message"""
        try:
            timestamp = datetime.now()

            # Extract battery data
            voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else None
            remaining = msg.battery_remaining if msg.battery_remaining != -1 else None

            # Additional data if available
            consumed_mah = msg.current_consumed if hasattr(msg, 'current_consumed') else None
            energy_consumed = msg.energy_consumed if hasattr(msg, 'energy_consumed') else None
            temperature = msg.temperature / 100.0 if hasattr(msg, 'temperature') and msg.temperature != 32767 else None

            self.__logger.warning(f"Processing battery message: "
                                f"Voltage={voltage}, Current={current}, Remaining={remaining}, "
                                f"Consumed mAh={consumed_mah}, Energy Consumed={energy_consumed}, "
                                f"Temperature={temperature}")

            # Validate voltage
            if voltage and self.__min_voltage <= voltage <= self.__max_voltage:
                with self.__lock:
                    self.__latest_voltage = voltage
                    self.__latest_current = current
                    self.__latest_remaining = remaining
                    self.__latest_timestamp = timestamp
                    self.__latest_consumed_mah = consumed_mah
                    self.__latest_energy_consumed = energy_consumed
                    self.__latest_temperature = temperature

                    # Update history
                    self.__voltage_history.append(voltage)
                    if current is not None:
                        self.__current_history.append(current)
                    self.__all_measurements.append((timestamp, voltage, current))

                    # Update statistics
                    self.__max_voltage_seen = max(self.__max_voltage_seen, voltage)
                    self.__min_voltage_seen = min(self.__min_voltage_seen, voltage)
                    if current:
                        self.__total_current_drawn += current * self.__update_interval / 3600  # Ah

                    # Detect cell count from voltage
                    if self.__cell_count is None:
                        self.__cell_count = self.__detect_cell_count(voltage)

                    # Store individual cell voltages if available
                    if len(msg.voltages) > 1:
                        self.__cell_voltages = [v/1000.0 for v in msg.voltages if v != 65535]

                self.__total_measurements += 1

                # Log warnings for low voltage
                if voltage < self.__critical_voltage:
                    self.__logger.critical(f"CRITICAL: Battery voltage {voltage:.2f}V is below critical threshold!")
                elif voltage < self.__warning_voltage:
                    self.__logger.warning(f"WARNING: Battery voltage {voltage:.2f}V is low")

            else:
                self.__failed_measurements += 1
                if voltage:
                    self.__logger.warning(f"Invalid voltage reading: {voltage:.2f}V")

        except Exception as e:
            self.__logger.error(f"Error processing battery message: {e}")
            self.__failed_measurements += 1

    def __print_status(self) -> None:
        """Print battery status to controller logger if enough time has passed"""
        current_time = time.time()

        # Check if enough time has passed since last print
        if current_time - self.__last_status_print_time < self.__status_print_interval:
            return

        # Update last print time
        self.__last_status_print_time = current_time

        # Only print if controller logger was provided
        if self.__controller_logger is None:
            return

        # Get current battery status
        voltage = self.voltage
        current = self.current
        remaining = self.remaining_percent

        # Only print if we have valid data
        if voltage is None:
            return

        # Format the message
        current_str = f"{current:.2f}" if current is not None else "N/A"
        remaining_str = f"{remaining}" if remaining is not None else "N/A"

        # Determine status
        if self.is_critical:
            status = "CRITICAL"
            log_func = self.__controller_logger.critical
        elif self.is_warning:
            status = "WARNING"
            log_func = self.__controller_logger.warning
        else:
            status = "OK"
            log_func = self.__controller_logger.info

        # Log the status
        message = f"Battery: {voltage:.2f}V, {current_str}A, {remaining_str}% - {status}"
        log_func(message)

    def __detect_cell_count(self, voltage: float) -> int:
        """Detect battery cell count from voltage"""
        # Typical voltage ranges per cell
        if 3.0 <= voltage <= 4.3:
            return 1  # 1S
        elif 6.0 <= voltage <= 8.6:
            return 2  # 2S
        elif 9.0 <= voltage <= 12.9:
            return 3  # 3S
        elif 12.0 <= voltage <= 17.2:
            return 4  # 4S
        elif 15.0 <= voltage <= 21.5:
            return 5  # 5S
        elif 18.0 <= voltage <= 25.8:
            return 6  # 6S
        else:
            return None

    # ===== Public Properties for Easy Access =====

    @property
    def voltage(self) -> float:
        """Get the latest battery voltage in volts"""
        with self.__lock:
            return self.__latest_voltage

    @property
    def current(self) -> float:
        """Get the latest battery current in amps"""
        with self.__lock:
            return self.__latest_current

    @property
    def remaining_percent(self) -> int:
        """Get the remaining battery percentage"""
        with self.__lock:
            return self.__latest_remaining

    @property
    def voltage_averaged(self) -> float:
        """Get averaged voltage over the window"""
        with self.__lock:
            if not self.__voltage_history:
                return None
            return statistics.mean(self.__voltage_history)

    @property
    def current_averaged(self) -> float:
        """Get averaged current over the window"""
        with self.__lock:
            if not self.__current_history:
                return None
            return statistics.mean(self.__current_history)

    @property
    def consumed_mah(self) -> float:
        """Get consumed capacity in mAh"""
        with self.__lock:
            return self.__latest_consumed_mah

    @property
    def energy_consumed(self) -> float:
        """Get energy consumed in Joules"""
        with self.__lock:
            return self.__latest_energy_consumed

    @property
    def temperature(self) -> float:
        """Get battery temperature in Celsius"""
        with self.__lock:
            return self.__latest_temperature

    @property
    def cell_count(self) -> int:
        """Get detected number of battery cells"""
        with self.__lock:
            return self.__cell_count

    @property
    def cell_voltages(self) -> list:
        """Get individual cell voltages if available"""
        with self.__lock:
            return self.__cell_voltages.copy() if self.__cell_voltages else []

    @property
    def timestamp(self) -> datetime:
        """Get timestamp of the latest measurement"""
        with self.__lock:
            return self.__latest_timestamp

    @property
    def age(self) -> float:
        """Get age of the latest measurement in seconds"""
        with self.__lock:
            if self.__latest_timestamp is None:
                return None
            return (datetime.now() - self.__latest_timestamp).total_seconds()

    @property
    def is_critical(self) -> bool:
        """Check if battery voltage is critically low"""
        voltage = self.voltage
        return voltage is not None and voltage < self.__critical_voltage

    @property
    def is_warning(self) -> bool:
        """Check if battery voltage is in warning range"""
        voltage = self.voltage
        return voltage is not None and voltage < self.__warning_voltage

    @property
    def success_rate(self) -> float:
        """Get measurement success rate as percentage"""
        if self.__total_measurements == 0:
            return 0.0
        return ((self.__total_measurements - self.__failed_measurements) /
                self.__total_measurements * 100)

    # ===== Blocking Mode Methods =====

    def get_battery_status_blocking(self, timeout: float = 5.0) -> dict:
        """
        Get battery status in blocking mode

        Args:
            timeout: Maximum time to wait for battery data

        Returns:
            dict: Battery status data or None if timeout
        """
        if not self.__running:
            self.start()

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.voltage is not None:
                return {
                    'voltage': self.voltage,
                    'current': self.current,
                    'remaining_percent': self.remaining_percent,
                    'consumed_mah': self.consumed_mah,
                    'temperature': self.temperature,
                    'cell_count': self.cell_count,
                    'is_critical': self.is_critical,
                    'is_warning': self.is_warning,
                    'timestamp': self.timestamp
                }
            time.sleep(0.1)

        return None

    def wait_for_battery_data(self, timeout: float = 5.0) -> bool:
        """Wait for valid battery data to be available"""
        start = time.time()
        while self.voltage is None:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return True

    # ===== Utility Methods =====

    def get_statistics(self) -> dict:
        """Get detailed statistics about battery monitor performance"""
        with self.__lock:
            if not self.__all_measurements:
                return None

            voltages = [v for _, v, _ in self.__all_measurements if v is not None]
            currents = [c for _, _, c in self.__all_measurements if c is not None]

        runtime = time.time() - self.__start_time

        return {
            'current_voltage': self.voltage,
            'current_current': self.current,
            'averaged_voltage': self.voltage_averaged,
            'averaged_current': self.current_averaged,
            'remaining_percent': self.remaining_percent,
            'consumed_mah': self.consumed_mah,
            'temperature': self.temperature,
            'cell_count': self.cell_count,
            'min_voltage_seen': self.__min_voltage_seen if self.__min_voltage_seen != float('inf') else None,
            'max_voltage_seen': self.__max_voltage_seen if self.__max_voltage_seen > 0 else None,
            'voltage_std_dev': statistics.stdev(voltages) if len(voltages) > 1 else 0,
            'current_std_dev': statistics.stdev(currents) if len(currents) > 1 else 0,
            'total_current_drawn_ah': self.__total_current_drawn,
            'success_rate': self.success_rate,
            'actual_rate_hz': self.__total_measurements / runtime if runtime > 0 else 0,
            'total_measurements': self.__total_measurements,
            'failed_measurements': self.__failed_measurements,
            'measurement_age': self.age,
            'runtime_seconds': runtime
        }

    def set_voltage_thresholds(self, warning: float = None, critical: float = None) -> None:
        """Update voltage thresholds"""
        if warning is not None:
            self.__warning_voltage = warning
        if critical is not None:
            self.__critical_voltage = critical

    def reset_statistics(self) -> None:
        """Reset all statistics"""
        with self.__lock:
            self.__total_measurements = 0
            self.__failed_measurements = 0
            self.__max_voltage_seen = 0
            self.__min_voltage_seen = float('inf')
            self.__total_current_drawn = 0
            self.__start_time = time.time()

    def cleanup(self) -> None:
        """Clean up resources"""
        self.stop()

    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, _exc_type, _exc_val, _exc_tb) -> None:
        """Context manager exit"""
        self.cleanup()

    def __del__(self) -> None:
        """Destructor"""
        try:
            self.cleanup()
        except:
            pass


# Example usage and testing
if __name__ == "__main__":
    print("Testing Battery Monitor")
    print("======================\n")

    # Create and start monitor
    monitor = BatteryMonitor(
        vid="1a86",
        pid="7523",
        update_rate=1.0,  # 1 Hz updates
        warning_voltage=11.1,  # 3S LiPo warning
        critical_voltage=10.5  # 3S LiPo critical
    )

    print("Starting battery monitor...")
    try:
        monitor.start()
    except RuntimeError as e:
        print(f"Failed to start monitor: {e}")
        exit(1)

    # Test blocking mode
    print("\nTesting blocking mode...")
    status = monitor.get_battery_status_blocking(timeout=5.0)
    if status:
        print(f"Battery Status (Blocking):")
        for key, value in status.items():
            print(f"  {key}: {value}")
    else:
        print("Failed to get battery status in blocking mode")

    print("\nContinuous monitoring (Ctrl+C to stop):")
    print("-" * 60)

    try:
        while True:
            # Access battery data anytime - always gets latest value
            voltage = monitor.voltage
            current = monitor.current
            remaining = monitor.remaining_percent
            temp = monitor.temperature
            age = monitor.age

            if voltage is not None:
                # Status indicator
                if monitor.is_critical:
                    status = "CRITICAL"
                elif monitor.is_warning:
                    status = "WARNING"
                else:
                    status = "OK"

                print(f"Voltage: {voltage:5.2f}V | "
                      f"Current: {current:5.2f}A | "
                      f"Remaining: {remaining:3d}% | "
                      f"Temp: {temp:4.1f}°C | "
                      f"Age: {age:4.3f}s | "
                      f"Status: {status:<8}", end='\r')
            else:
                print("No battery data available" + " " * 30, end='\r')

            time.sleep(0.1)  # Update display at 10Hz

    except KeyboardInterrupt:
        print("\n\n" + "-" * 60)
        print("Battery Monitor Statistics:")
        stats = monitor.get_statistics()
        if stats:
            for key, value in stats.items():
                if isinstance(value, float):
                    print(f"  {key}: {value:.2f}")
                else:
                    print(f"  {key}: {value}")

    finally:
        monitor.cleanup()
        print("\nCleanup complete")