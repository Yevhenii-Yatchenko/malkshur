import time
from threading import Timer

from src.altitude_config import CONTROL
from src.controller_config import ARM_IN


class DroneController:
    """The class is intended to controll the drone using MAVLink protocol.

    Thin orchestrator since GRASP Step 7 (LC-3): every collaborator is
    constructed by the composition root (``src/app.py``, the only place
    that builds the object graph) and injected here.  ``__init__`` only
    stores them, registers the command handlers (they are bound methods, so
    they cannot exist before the controller does), starts the telnet
    command processing thread, and arms the optional ARM_IN auto-arm timer
    -- in exactly that order, as before.
    """

    def __init__(
        self,
        *,
        logger,
        mavlink,
        sensor,
        stabilizer,
        battery_monitor,
        altitude_controller,
        rc,
        detection_server,
        detection_client,
        intercept_guidance,
        stabilization,
        signal_handler,
        command_handler,
    ):
        """
        Args:
            logger: controller logger (logs/controller.log).
            mavlink: connected MAVLinkManager.
            sensor: started AltitudeSensor (LIDAR or barometer).
            stabilizer: StabilizerManager (sky_anchor process + client).
            battery_monitor: started BatteryMonitor, or None (Gazebo mode).
            altitude_controller: AltitudeController (cascade altitude PID).
            rc: RCSetpoints, the single owner of the RC PWM bases.
            detection_server: DetectionServer (owns target-validity rules).
            detection_client: DockerDetectionClient.
            intercept_guidance: InterceptGuidance state machine.
            stabilization: StabilizationBehavior (vision XY hold; holds the
                PositionController, which the controller itself never
                touches directly).
            signal_handler: SignalHandler (SIGTERM -> shutdown flag).
            command_handler: CommandHandler with its TelnetServer; commands
                are registered and processing is started here.
        """
        self.logger = logger
        self.__mavlink = mavlink
        self.__sensor = sensor
        self.__stabilizer = stabilizer
        self.battery_monitor = battery_monitor
        self.__altitude_controller = altitude_controller
        self.__rc = rc
        self.__detection_server = detection_server
        self.__detection_client = detection_client
        self.__intercept_guidance = intercept_guidance
        self.__stabilization = stabilization
        self.signal_handler = signal_handler
        self.__command_handler = command_handler

        # Loop state, per instance (Step 7 carried bullet: formerly class
        # attributes shadowed on first assignment).
        self.__current_altitude = None
        self.__is_up = False

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
        self.logger.debug(f"Altitude: {self.__get_current_altitude()} (Target: {self.__rc.target_altitude});"
                         f" RC channels - Roll: {self.__rc.roll}, Pitch: {self.__rc.pitch}, "
                         f"Throttle: {self.__rc.throttle}, Yaw: {self.__rc.yaw}")

        self.__mavlink.send_rc_override(
            roll=self.__rc.roll,
            pitch=self.__rc.pitch,
            throttle=self.__rc.throttle,
            yaw=self.__rc.yaw
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

        self.__rc.target_altitude = float(distance)
        self.__altitude_controller.position_pid.reset()
        self.logger.info(f"Задана цільова висота: {self.__rc.target_altitude} м (поточна: {self.__current_altitude:.2f} м).")

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
            self.__intercept_guidance.exit_intercept()

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
        self.__rc.roll = params[0]
        self.__rc.pitch = params[1]
        self.__rc.yaw = params[2]
        self.logger.info(f"Move: self.__yaw_base={self.__rc.yaw}, pitch_base={self.__rc.pitch}, roll_base={self.__rc.roll}")

    def _land(self, params):
        self.logger.info("Initiating controlled landing...")
        self.__rc.pitch = 1500
        self.__rc.roll = 1500

        current_altitude = self.__get_current_altitude()
        if current_altitude is None:
            self.__rc.throttle = 1300
            self.logger.warning("No altitude reading - emergency landing mode")
            return

        self.__rc.target_altitude = 0.1
        self.logger.info(f"Landing from {current_altitude:.2f}m")

    def __updateThrottle(self):
        """Thin orchestrator (GRASP Step 5/6): pick a flight behavior, apply
        the setpoints it returns, then run altitude control and send RC."""
        current_altitude = self.__get_current_altitude()

        # Check for intercept mode (target recognition).  Data validity is
        # the detection server's call -- including the INTERCEPT_* thresholds
        # it owns since Step 7; the state machine lives in InterceptGuidance;
        # the running check stays here.
        if self.__detection_server.is_running():
            target = self.__detection_server.get_active_target()
            result = self.__intercept_guidance.update(
                target=target,
                current_altitude=current_altitude,
                target_altitude=self.__rc.target_altitude,
            )
            if result.setpoints is not None:
                self.__rc.apply(result.setpoints)
            if result.reenable_stabilization and not self.__stabilizer.is_connected:
                # Re-enable stabilization after the intercept deactivated
                # (the handoff: an explicit intent since Step 6, executed
                # here synchronously -- same iteration, before the
                # stabilization behavior runs, as the former in-update
                # side effect did).
                self.logger.info("Re-enabling stabilization after intercept")
                self.__stabilizer.start_stabilizer_process()
            if result.active:
                # Active intercept: skip stabilization, go straight to
                # altitude control.
                self.__run_altitude_control(current_altitude)
                return

        # Normal stabilization mode (if not intercepting)
        setpoints = self.__stabilization.update(
            current_altitude=current_altitude,
            target_altitude=self.__rc.target_altitude,
            intercept_active=self.__intercept_guidance.is_intercepting,
        )
        if setpoints is not None:
            self.__rc.apply(setpoints)

        self.__run_altitude_control(current_altitude)

    def __run_altitude_control(self, current_altitude):
        """Altitude PID -> throttle base -> RC override send."""
        if current_altitude is None:
            self.logger.error("No valid altitude reading! Maintaining current throttle.")
            self.__set_rc_channel_pwm()
            return

        try:
            new_throttle = self.__altitude_controller.update(
                target_altitude=self.__rc.target_altitude,
                current_altitude=current_altitude
            )
            self.__rc.throttle = new_throttle
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
