"""Composition root: the one place that builds the flight-controller graph.

GRASP Step 7 (REFACTORING_PLAN.md, LC-2/LC-3): ``DroneController.__init__``
used to double as the composition root, constructing every subsystem inline
(and some subsystems constructed their own collaborators: CommandHandler its
TelnetServer, StabilizerManager its SkyAnchorClient, the PID controllers
their CSV loggers and config).  ``build_controller()`` now constructs the
whole object graph explicitly, in exactly the order the old ``__init__``
did, so the startup sequence -- log lines, thread starts, socket binds, the
MAVLink connect -- is unchanged:

1.  controller logger (the process's first ``get_logger`` call, as before);
2.  MAVLinkManager + connect (TCP for Gazebo, USB for hardware);
3.  altitude sensor from the SensorManager factory, started;
4.  StabilizerManager with its SkyAnchorClient injected (localhost:8888);
5.  BatteryMonitor (hardware) or None (Gazebo);
6.  Altitude/Position controllers with config objects (built 1:1 from the
    ``*_config.py`` dicts) and their file CSV loggers passed explicitly --
    same constructor arguments the controllers' None-fallbacks produce,
    including the single shared session timestamp;
7.  RCSetpoints with the throttle ceiling passed explicitly;
8.  detection server (owning the INTERCEPT_* thresholds) + Docker client;
9.  flight behaviors (InterceptGuidance, StabilizationBehavior);
10. SignalHandler;
11. CommandHandler with its TelnetServer injected (0.0.0.0:2323);
12. DroneController, which registers commands, starts telnet processing and
    arms the ARM_IN auto-arm timer in its (now thin) ``__init__``.

Nothing else in the codebase constructs DroneController; the entry point is
``run_controller.py`` (and the ``xbee_process_com.py`` shim the Docker
entrypoints call).
"""

from datetime import datetime

from src.altitude_config import THROTTLE
from src.altitude_csv_logger import AltitudeCSVLogger
from src.battery_monitor import BatteryMonitor
from src.command_handler import CommandHandler
from src.config.objects import AltitudeConfig, PositionConfig
from src.controller import DroneController
from src.controller_config import (
    ENABLE_BATTERY_MONITOR, SKY_ANCHOR_PATH, LOG_LEVEL
)
from src.detection_client import DockerDetectionClient
from src.detection_config import (
    INTERCEPT_CONFIDENCE_THRESHOLD,
    INTERCEPT_TIMEOUT_SECONDS,
)
from src.detection_server import DetectionServer
from src.flight.intercept import InterceptGuidance
from src.flight.setpoints import RCSetpoints
from src.flight.stabilization import StabilizationBehavior
from src.logger import get_logger
from src.mavlink_manager import MAVLinkManager
from src.pid_controller import AltitudeController
from src.position_controller import PositionController
from src.position_csv_logger import PositionCSVLogger
from src.sensor_manager import SensorManager
from src.signal_handler import SignalHandler
from src.sky_anchor_client import SkyAnchorClient
from src.stabilizer_manager import StabilizerManager
from src.telnet_server import TelnetServer

# Wiring constants that used to hide inside the collaborators' defaults.
TELNET_HOST = '0.0.0.0'
TELNET_PORT = 2323          # remote command interface
SKY_ANCHOR_HOST = 'localhost'
SKY_ANCHOR_PORT = 8888      # sky_anchor vision wire


def build_controller() -> DroneController:
    """Build and return a fully wired DroneController, ready for .loop()."""
    logger = get_logger("controller", "logs/controller.log", log_level=LOG_LEVEL)

    mavlink = MAVLinkManager()
    master = mavlink.connect()

    sensor = SensorManager.create_sensor(mavlink_connection=master)
    sensor.start()

    stabilizer = StabilizerManager(
        stabilizer_path=SKY_ANCHOR_PATH,
        logger=logger,
        client=SkyAnchorClient(SKY_ANCHOR_HOST, SKY_ANCHOR_PORT),
    )

    if ENABLE_BATTERY_MONITOR:
        battery_monitor = BatteryMonitor(
            master=master,
            controller_logger=logger
        )
        battery_monitor.start()
    else:
        battery_monitor = None
        logger.info("Battery monitor disabled (Gazebo mode)")

    # One session timestamp shared by both CSV loggers, exactly like the
    # controllers' own fallbacks used to receive.
    timestamp = datetime.now()
    altitude_controller = AltitudeController(
        config=AltitudeConfig.from_dicts(),
        csv_logger=AltitudeCSVLogger(
            start_timestamp=timestamp, controller_type='takeoff'
        ),
    )
    position_controller = PositionController(
        config=PositionConfig.from_dicts(),
        csv_logger=PositionCSVLogger(start_timestamp=timestamp),
    )

    # RC PWM state (GRASP Step 6, IE-4): single owner of the four RC bases
    # + altitude target; the throttle ceiling enters the graph here.
    rc = RCSetpoints(throttle_max=THROTTLE['max'])

    # Detection system (object recognition); the server owns the intercept
    # data-validity thresholds.
    detection_server = DetectionServer(
        logger=logger,
        intercept_timeout_s=INTERCEPT_TIMEOUT_SECONDS,
        intercept_min_confidence=INTERCEPT_CONFIDENCE_THRESHOLD,
    )
    detection_client = DockerDetectionClient(logger=logger)

    # Flight behaviors extracted from __updateThrottle (GRASP Step 5): the
    # intercept state machine and the vision stabilization branch.
    intercept_guidance = InterceptGuidance(
        detection_server=detection_server,
        logger=logger,
    )
    stabilization = StabilizationBehavior(
        stabilizer=stabilizer,
        position_controller=position_controller,
    )

    signal_handler = SignalHandler()

    command_handler = CommandHandler(
        logger=logger,
        telnet_host=TELNET_HOST,
        telnet_port=TELNET_PORT,
        telnet_server=TelnetServer(host=TELNET_HOST, port=TELNET_PORT),
    )

    return DroneController(
        logger=logger,
        mavlink=mavlink,
        sensor=sensor,
        stabilizer=stabilizer,
        battery_monitor=battery_monitor,
        altitude_controller=altitude_controller,
        rc=rc,
        detection_server=detection_server,
        detection_client=detection_client,
        intercept_guidance=intercept_guidance,
        stabilization=stabilization,
        signal_handler=signal_handler,
        command_handler=command_handler,
    )
