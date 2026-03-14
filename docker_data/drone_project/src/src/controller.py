from datetime import datetime
import time
from threading import Timer

from src.mavlink_manager import MAVLinkManager
from src.sensor_manager import SensorManager
from src.stabilizer_manager import StabilizerManager
from src.logger import get_logger
from src.pid_controller import AltitudeController
from src.position_controller import PositionController
from src.altitude_config import CONTROL, DEBUG
from src.battery_monitor import BatteryMonitor
from src.signal_handler import SignalHandler
from src.command_handler import CommandHandler
from src.controller_config import (
    ENABLE_BATTERY_MONITOR, SKY_ANCHOR_PATH, LOG_LEVEL, ARM_IN
)
from src.detection_server import DetectionServer
from src.detection_client import DockerDetectionClient
from src.detection_config import (
    INTERCEPT_CONFIDENCE_THRESHOLD,
    INTERCEPT_TIMEOUT_SECONDS,
    INTERCEPT_DEADBAND_X,
    INTERCEPT_DEADBAND_Y,
    INTERCEPT_YAW_GAIN,
    INTERCEPT_ALTITUDE_STEP,
    INTERCEPT_PITCH_OFFSET,
    INTERCEPT_ROLL_NEUTRAL,
    PWM_NEUTRAL,
)


class DroneController:
    """The class is intended to controll the drone using MAVLink protocol."""
    __target_altitude = 0.2
    __current_altitude = None
    __pitch_base = 1500
    __roll_base = 1500
    __yaw_base = 1500
    __last_xy_update = 0

    __is_up = False

    __throttle_base = 1000

    # Intercept mode state
    __intercept_mode = False

    def __set_throttle_base(self, value):
        """Set throttle base with upper limit enforcement."""
        if value >= 1800:
            value = 1800
        self.__throttle_base = value


    def __init__(self):
        self.logger = get_logger("controller", "logs/controller.log", log_level=LOG_LEVEL)

        self.__mavlink = MAVLinkManager()
        master = self.__mavlink.connect()

        self.__sensor = SensorManager.create_sensor(mavlink_connection=master)
        self.__sensor.start()

        self.__stabilizer = StabilizerManager(
            stabilizer_path=SKY_ANCHOR_PATH,
            logger=self.logger
        )

        if ENABLE_BATTERY_MONITOR:
            self.battery_monitor = BatteryMonitor(
                master=master,
                controller_logger=self.logger
            )
            self.battery_monitor.start()
        else:
            self.battery_monitor = None
            self.logger.info("Battery monitor disabled (Gazebo mode)")

        timestamp = datetime.now()
        self.__altitude_controller = AltitudeController(start_timestamp=timestamp)
        self.__position_controller = PositionController(start_timestamp=timestamp)

        # Detection system (object recognition)
        self.__detection_server = DetectionServer(logger=self.logger)
        self.__detection_client = DockerDetectionClient(logger=self.logger)

        self.signal_handler = SignalHandler()

        self.__command_handler = CommandHandler(logger=self.logger)
        self.__register_commands()
        self.__command_handler.start_telnet_processing()

        if ARM_IN:
            Timer(ARM_IN, self.__startAutoArming).start()

    def __ensureArmed(self):
        self.logger.info("Auto-arming motors...")
        if self.is_armed:
            self.logger.info("Motors are already armed.")
            return

        self.__startAutoArming()

    def __startAutoArming(self):
        # Set to STABILIZE mode first (doesn't require GPS)
        self.logger.info("Setting STABILIZE mode before arming...")
        self._set_mode(["STABILIZE"])
        time.sleep(1)

        self.logger.info("Attempting to arm motors...")
        self._armingDisarming([0])
        self.logger.info("Armed? Checking in 2 seconds...")

        Timer(2, self.__ensureArmed).start()

    def __register_commands(self) -> None:
        self.__command_handler.register_command('mode', self._set_mode)
        self.__command_handler.register_command('arm', self._armingDisarming)
        self.__command_handler.register_command('setHeight', self._setHeight)
        self.__command_handler.register_command('move', self._move)
        self.__command_handler.register_command('land', self._land)
        self.__command_handler.register_command('stabilize', self._stabilize)
        self.__command_handler.register_command('recognizeClient', self._recognize_client)
        self.__command_handler.register_command('monitor', self._monitor)

    def _stabilize(self, params):
        self.__stabilizer.start_stabilizer_process()

    def _recognize_client(self, params):
        """
        Control Docker detection client.

        Args:
            params: ['start'|'stop'|'1'|'0']
        """
        if not params or len(params) == 0:
            self.logger.warning("recognizeClient requires parameter: start/stop or 1/0")
            return

        action = params[0].lower()

        if action in ['start', '1']:
            if self.__detection_client.is_running():
                self.logger.warning("Detection client already running")
            else:
                self.logger.info("Starting detection client (Docker)")
                success = self.__detection_client.start()
                if success:
                    self.logger.warning("Detection client started successfully")
                else:
                    self.logger.error("Failed to start detection client")

        elif action in ['stop', '0']:
            if not self.__detection_client.is_running():
                self.logger.warning("Detection client not running")
            else:
                self.logger.info("Stopping detection client")
                self.__detection_client.stop()

        else:
            self.logger.warning(f"Invalid parameter for recognizeClient: {action}. Use start/stop or 1/0")

    def _monitor(self, params):
        """
        Control detection monitoring server.

        Args:
            params: ['start'|'stop'|'1'|'0']
        """
        if not params or len(params) == 0:
            self.logger.warning("monitor requires parameter: start/stop or 1/0")
            return

        action = params[0].lower()

        if action in ['start', '1']:
            if self.__detection_server.is_running():
                self.logger.warning("Detection server already running")
            else:
                self.logger.info("Starting detection monitoring server")
                success = self.__detection_server.start()
                if success:
                    self.logger.warning("Detection server started successfully")
                else:
                    self.logger.error("Failed to start detection server")

        elif action in ['stop', '0']:
            if not self.__detection_server.is_running():
                self.logger.warning("Detection server not running")
            else:
                self.logger.info("Stopping detection server")
                self.__detection_server.stop()

        else:
            self.logger.warning(f"Invalid parameter for monitor: {action}. Use start/stop or 1/0")

    def __set_rc_channel_pwm(self):
        self.logger.debug(f"Altitude: {self.__get_current_altitude()} (Target: {self.__target_altitude});"
                         f" RC channels - Roll: {self.__roll_base}, Pitch: {self.__pitch_base}, "
                         f"Throttle: {self.__throttle_base}, Yaw: {self.__yaw_base}")

        self.__mavlink.send_rc_override(
            roll=self.__roll_base,
            pitch=self.__pitch_base,
            throttle=self.__throttle_base,
            yaw=self.__yaw_base
        )

    def _set_mode(self, params):
        mode = params[0]
        self.__mavlink.set_mode(mode)

    def __get_current_altitude(self):
        self.__current_altitude = self.__sensor.get_altitude()
        return self.__current_altitude

    def _setHeight(self, params):
        self.__is_up = True
        distance = params[0]
        self.__current_altitude = self.__get_current_altitude()
        if self.__current_altitude is None:
            self.logger.error("Не вдалося зчитати поточну висоту.")
            return

        self.__target_altitude = float(distance)
        self.__altitude_controller.position_pid.reset()
        self.logger.info(f"Задана цільова висота: {self.__target_altitude} м (поточна: {self.__current_altitude:.2f} м).")

    def _armingDisarming(self, params):
        isArming = params[0]
        if isArming == 0:
            success = self.__mavlink.arm()
            if success:
                time.sleep(1)
                self._setHeight([5])
            else:
                self.logger.error("Failed to arm the drone - check logs for details")
        elif isArming == 1:
            # Disarming - stop all detection systems
            self.logger.info("Disarming - stopping detection systems")

            # Stop detection client (Docker)
            if self.__detection_client.is_running():
                self.logger.info("Stopping detection client")
                self.__detection_client.stop()

            # Stop detection server
            if self.__detection_server.is_running():
                self.logger.info("Stopping detection server")
                self.__detection_server.stop()

            # Exit intercept mode
            if self.__intercept_mode:
                self.logger.info("Exiting intercept mode")
                self.__intercept_mode = False

            # Disarm drone
            self.__mavlink.disarm()
            self.__is_up = False
            self.__altitude_controller.reset()
        else:
            self.logger.error("Wrong parameter for arming or disarming")

    @property
    def is_armed(self):
        return self.__mavlink.is_armed

    @property
    def is_landing(self):
        return self.__mavlink.is_landing

    def _move(self, params):
        self.__roll_base = params[0]
        self.__pitch_base = params[1]
        self.__yaw_base = params[2]
        self.logger.info(f"Move: self.__yaw_base={self.__yaw_base}, pitch_base={self.__pitch_base}, roll_base={self.__roll_base}")

    def _land(self, params):
        self.logger.info("Initiating controlled landing...")
        self.__pitch_base = 1500
        self.__roll_base = 1500

        current_altitude = self.__get_current_altitude()
        if current_altitude is None:
            self.__set_throttle_base(1300)
            self.logger.warning("No altitude reading - emergency landing mode")
            return

        self.__target_altitude = 0.1
        self.logger.info(f"Landing from {current_altitude:.2f}m")

    def __updateThrottle(self):
        current_altitude = self.__get_current_altitude()

        # Check for intercept mode (target recognition)
        if self.__detection_server.is_running():
            detection_data = self.__detection_server.get_latest_detection()
            time_since_detection = self.__detection_server.get_time_since_last_detection()

            # Check if we have recent high-confidence detection
            if detection_data and time_since_detection < INTERCEPT_TIMEOUT_SECONDS:
                confidence = detection_data.get('confidence', 0.0)

                if confidence >= INTERCEPT_CONFIDENCE_THRESHOLD:
                    # High confidence detection - enter intercept mode
                    if not self.__intercept_mode:
                        self.logger.warning(f"INTERCEPT MODE ACTIVATED (confidence: {confidence:.2%})")
                        self.__intercept_mode = True
                        self.__intercept_start_altitude = current_altitude
                        # Disable stabilization during intercept
                        # (stabilizer keeps running but we ignore its output)

                    # Extract direction vector
                    direction_vector = detection_data.get('direction_vector', {})
                    direction = direction_vector.get('direction', [0, 0, 0])

                    dir_x = direction[0] if len(direction) > 0 else 0.0  # Horizontal (-0.5 to 0.5)
                    dir_y = direction[1] if len(direction) > 1 else 0.0  # Vertical (-0.5 to 0.5)

                    # YAW control (centering horizontally)
                    if abs(dir_x) > INTERCEPT_DEADBAND_X:
                        # Positive dir_x = target is right, need to yaw right
                        yaw_correction = int(dir_x * INTERCEPT_YAW_GAIN)
                        self.__yaw_base = PWM_NEUTRAL + yaw_correction
                    else:
                        self.__yaw_base = PWM_NEUTRAL  # Centered

                    # ALTITUDE control (centering vertically)
                    if abs(dir_y) > INTERCEPT_DEADBAND_Y:
                        # Positive dir_y = target is above, need to climb
                        altitude_correction = dir_y * INTERCEPT_ALTITUDE_STEP
                        self.__target_altitude += altitude_correction
                        self.logger.debug(f"Altitude adjustment: {altitude_correction:+.2f}m -> {self.__target_altitude:.2f}m")

                    # PITCH control (move forward toward target)
                    self.__pitch_base = PWM_NEUTRAL + INTERCEPT_PITCH_OFFSET

                    # ROLL neutral (no lateral movement)
                    self.__roll_base = INTERCEPT_ROLL_NEUTRAL

                    self.logger.debug(
                        f"Intercept: conf={confidence:.2%}, dir_x={dir_x:+.3f}, dir_y={dir_y:+.3f}, "
                        f"yaw={self.__yaw_base}, pitch={self.__pitch_base}"
                    )

                    # Skip stabilization logic below
                    # Continue to altitude control at the end
                    if current_altitude is None:
                        self.logger.error("No valid altitude reading! Maintaining current throttle.")
                        self.__set_rc_channel_pwm()
                        return

                    try:
                        new_throttle = self.__altitude_controller.update(
                            target_altitude=self.__target_altitude,
                            current_altitude=current_altitude
                        )
                        self.__set_throttle_base(new_throttle)
                        self.__set_rc_channel_pwm()
                    except Exception as e:
                        self.logger.error(f"Error in altitude control: {e}")

                    return  # Exit early, skip stabilization

            # No recent detection or low confidence
            if self.__intercept_mode and time_since_detection >= INTERCEPT_TIMEOUT_SECONDS:
                self.logger.warning(
                    f"INTERCEPT MODE DEACTIVATED (no detection for {INTERCEPT_TIMEOUT_SECONDS}s)"
                )
                self.__intercept_mode = False
                # Stop forward movement
                self.__pitch_base = PWM_NEUTRAL
                self.__roll_base = PWM_NEUTRAL
                self.__yaw_base = PWM_NEUTRAL
                # Fix altitude at current position
                if current_altitude is not None:
                    self.__target_altitude = current_altitude
                    self.logger.info(f"Holding altitude at {current_altitude:.2f}m")
                # Re-enable stabilization
                if not self.__stabilizer.is_connected:
                    self.logger.info("Re-enabling stabilization after intercept")
                    self._stabilize([])

        # Normal stabilization mode (if not intercepting)
        if self.__stabilizer.is_connected and not self.__intercept_mode:
            data = self.__stabilizer.get_stabilizer_data()
            if data:
                timestamp = data.get('timestamp', self.__last_xy_update)
                if self.__last_xy_update != timestamp:
                    dx = data.get('dx')
                    dy = data.get('dy')
                    matches_percent = data.get('matches_percent')
                    target_dx = data.get('target_dx_pixels')
                    target_dy = data.get('target_dy_pixels')

                    pid_output = self.__position_controller.update(
                        dx_pixels=dx,
                        dy_pixels=dy,
                        angle_deg=0,
                        confidence=matches_percent / 100.0,
                        altitude=current_altitude,
                        target_dx_pixels=target_dx,
                        target_dy_pixels=target_dy,
                    )

                    self.__roll_base = pid_output['roll_pwm']
                    self.__pitch_base = pid_output['pitch_pwm']
                    self.__yaw_base = pid_output['yaw_pwm']
                    self.__last_xy_update = timestamp
        else:
            if current_altitude is not None and abs(current_altitude - self.__target_altitude) < 0.1:
                self._stabilize([])

        if current_altitude is None:
            self.logger.error("No valid altitude reading! Maintaining current throttle.")
            self.__set_rc_channel_pwm()
            return

        try:
            new_throttle = self.__altitude_controller.update(
                target_altitude=self.__target_altitude,
                current_altitude=current_altitude
            )
            self.__set_throttle_base(new_throttle)
            self.__set_rc_channel_pwm()
        except Exception as e:
            self.logger.error(f"Error in altitude control: {e}")

    def __iterate(self) -> bool:
        """
        Execute one control loop iteration.

        Returns:
            True to continue loop, False to exit
        """
        try:
            self.__get_current_altitude()

            if self.__is_up:
                self.__updateThrottle()

                if not self.is_armed:
                    self.logger.warning("Motors are not armed! Cannot control altitude.")
                    return False
                if self.is_landing:
                    self.logger.info("Landing detected - exiting control loop")
                    return False

        except Exception as e:
            self.logger.error(f"Unexpected error in main loop: {str(e)}")

        return True

    def loop(self):
        target_loop_time = 1.0 / CONTROL['update_rate']

        while not self.signal_handler.shutdown_requested:
            start_time = time.time()

            if not self.__iterate():
                break

            loop_duration = time.time() - start_time

            if loop_duration > target_loop_time:
                self.logger.warning(
                    f"Control loop overrun: {loop_duration*1000:.1f}ms "
                    f"(target: {target_loop_time*1000:.1f}ms)"
                )
            else:
                sleep_time = target_loop_time - loop_duration
                time.sleep(sleep_time)

        self.logger.info("Main loop exited - performing cleanup")
        self.cleanup()

    def cleanup(self):
        self.logger.info("Cleaning up detection systems")

        # Stop detection systems
        if self.__detection_client.is_running():
            self.__detection_client.stop()

        if self.__detection_server.is_running():
            self.__detection_server.stop()

        self.__command_handler.cleanup()
        self.__stabilizer.cleanup()

        if self.battery_monitor:
            self.battery_monitor.stop()

        self.__sensor.stop()
        self.__mavlink.close()
