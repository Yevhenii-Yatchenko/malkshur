"""Composition tests for the Step 7 composition root (src/app.py).

``build_controller()`` is the one place that constructs the flight
controller's object graph (LC-2/LC-3).  These tests build the real graph
with only the process-external boundaries replaced:

- ``MAVLinkManager`` (network/USB) and the sensor factory;
- ``TelnetServer`` (TCP listener on :2323);
- the file CSV loggers (so no logs/csv files are created and the exact
  constructor arguments can be asserted);
- ``SignalHandler`` (process-global SIGTERM hook) and ``BatteryMonitor``
  (hardware; not constructed in Gazebo mode anyway);
- ``src.controller.ARM_IN`` is forced to 0 (the container sets ARM_IN=1.0,
  which would otherwise start a real auto-arm timer).

Everything else -- StabilizerManager + SkyAnchorClient, the PID
controllers with their config objects, RCSetpoints, DetectionServer,
the flight behaviors, CommandHandler, DroneController -- is real, and the
assertions pin the SHAPE of the wired graph.  No subprocess is spawned and
no socket is opened at build time (Popen/binds happen on later commands).
"""

from datetime import datetime
from types import SimpleNamespace
from unittest import mock

import pytest

import src.app
from src.altitude_config import ALTITUDE_PID_TAKEOFF, THROTTLE
from src.controller import DroneController
from src.controller_config import ENABLE_BATTERY_MONITOR, SKY_ANCHOR_PATH
from src.detection_config import (
    INTERCEPT_CONFIDENCE_THRESHOLD,
    INTERCEPT_TIMEOUT_SECONDS,
)
from src.detection_server import DetectionServer
from src.position_config import POSITION_PID_X
from src.sky_anchor_client import SkyAnchorClient

pytestmark = [pytest.mark.unit]


@pytest.fixture
def built(monkeypatch):
    """build_controller() with the I/O boundaries mocked; yields the
    controller plus the boundary mocks, and cleans the controller up."""
    monkeypatch.setattr("src.controller.ARM_IN", 0)
    with mock.patch("src.app.MAVLinkManager", autospec=True) as mavlink_cls, \
            mock.patch("src.app.SensorManager", autospec=True) as sensor_mgr, \
            mock.patch("src.app.AltitudeCSVLogger", autospec=True) as alt_csv_cls, \
            mock.patch("src.app.PositionCSVLogger", autospec=True) as pos_csv_cls, \
            mock.patch("src.app.BatteryMonitor", autospec=True) as battery_cls, \
            mock.patch("src.app.SignalHandler", autospec=True) as signal_cls, \
            mock.patch("src.app.TelnetServer") as telnet_cls:
        controller = src.app.build_controller()
        try:
            yield SimpleNamespace(
                controller=controller,
                mavlink_cls=mavlink_cls,
                sensor_mgr=sensor_mgr,
                alt_csv_cls=alt_csv_cls,
                pos_csv_cls=pos_csv_cls,
                battery_cls=battery_cls,
                signal_cls=signal_cls,
                telnet_cls=telnet_cls,
            )
        finally:
            controller.cleanup()


class TestBuildController:
    def test_returns_a_drone_controller(self, built):
        assert isinstance(built.controller, DroneController)

    def test_mavlink_connected_and_sensor_started_from_its_master(self, built):
        built.mavlink_cls.assert_called_once_with()
        mavlink = built.mavlink_cls.return_value
        mavlink.connect.assert_called_once_with()
        built.sensor_mgr.create_sensor.assert_called_once_with(
            mavlink_connection=mavlink.connect.return_value
        )
        sensor = built.sensor_mgr.create_sensor.return_value
        sensor.start.assert_called_once_with()
        # ... and the controller received exactly these instances.
        assert built.controller._DroneController__mavlink is mavlink
        assert built.controller._DroneController__sensor is sensor

    def test_stabilizer_got_an_injected_sky_anchor_client(self, built):
        stabilizer = built.controller._DroneController__stabilizer
        client = stabilizer._StabilizerManager__client
        assert isinstance(client, SkyAnchorClient)
        assert (client.host, client.port) == ('localhost', 8888)
        assert (
            stabilizer._StabilizerManager__stabilizer_path == SKY_ANCHOR_PATH
        )

    def test_battery_monitor_per_mode(self, built):
        """Gazebo: None and never constructed; hardware: built and
        started (the assertion adapts to the resolved config)."""
        if ENABLE_BATTERY_MONITOR:
            assert built.controller.battery_monitor is built.battery_cls.return_value
            built.battery_cls.return_value.start.assert_called_once_with()
        else:
            assert built.controller.battery_monitor is None
            built.battery_cls.assert_not_called()

    def test_controllers_got_explicit_file_csv_loggers(self, built):
        """LC-1 closed at the root: the file loggers are constructed by
        app.py with the same arguments the controllers' None-fallbacks
        would use, including ONE shared session timestamp."""
        built.alt_csv_cls.assert_called_once()
        built.pos_csv_cls.assert_called_once()
        alt_kwargs = built.alt_csv_cls.call_args.kwargs
        pos_kwargs = built.pos_csv_cls.call_args.kwargs
        assert alt_kwargs["controller_type"] == "takeoff"
        assert isinstance(alt_kwargs["start_timestamp"], datetime)
        assert alt_kwargs["start_timestamp"] is pos_kwargs["start_timestamp"]

        altitude_controller = (
            built.controller._DroneController__altitude_controller
        )
        assert altitude_controller.csv_logger is built.alt_csv_cls.return_value
        position_controller = (
            built.controller._DroneController__stabilization
            ._StabilizationBehavior__position_controller
        )
        assert position_controller.csv_logger is built.pos_csv_cls.return_value

    def test_controllers_got_the_production_config_values(self, built):
        altitude_controller = (
            built.controller._DroneController__altitude_controller
        )
        assert altitude_controller.position_pid.kp == ALTITUDE_PID_TAKEOFF["kp"]
        assert altitude_controller.throttle_max == THROTTLE["max"]
        position_controller = (
            built.controller._DroneController__stabilization
            ._StabilizationBehavior__position_controller
        )
        assert position_controller.position_pid_x.kp == POSITION_PID_X["kp"]

    def test_command_handler_got_the_injected_telnet_server(self, built):
        """LC-3: app.py constructs the TelnetServer (0.0.0.0:2323) and the
        handler starts that exact instance."""
        built.telnet_cls.assert_called_once_with(host='0.0.0.0', port=2323)
        handler = built.controller._DroneController__command_handler
        assert (
            handler._CommandHandler__telnet_server
            is built.telnet_cls.return_value
        )
        built.telnet_cls.return_value.start.assert_called_once_with()

    def test_flight_commands_are_registered(self, built):
        handler = built.controller._DroneController__command_handler
        assert handler.get_registered_commands() == [
            'mode', 'arm', 'setHeight', 'move', 'land', 'stabilize',
            'recognizeClient', 'monitor',
        ]

    def test_detection_server_owns_the_intercept_thresholds(self, built):
        server = built.controller._DroneController__detection_server
        assert isinstance(server, DetectionServer)
        assert server.intercept_timeout_s == INTERCEPT_TIMEOUT_SECONDS
        assert server.intercept_min_confidence == INTERCEPT_CONFIDENCE_THRESHOLD
        # The intercept guidance watches the SAME server instance.
        guidance = built.controller._DroneController__intercept_guidance
        assert guidance._InterceptGuidance__detection_server is server

    def test_stabilization_behavior_holds_the_controllers_stabilizer(
        self, built
    ):
        behavior = built.controller._DroneController__stabilization
        assert (
            behavior._StabilizationBehavior__stabilizer
            is built.controller._DroneController__stabilizer
        )

    def test_rc_setpoints_ceiling_is_the_config_throttle_max(self, built):
        rc = built.controller._DroneController__rc
        rc.throttle = THROTTLE["max"] + 500
        assert rc.throttle == THROTTLE["max"]

    def test_signal_handler_installed(self, built):
        built.signal_cls.assert_called_once_with()
        assert built.controller.signal_handler is built.signal_cls.return_value

    def test_telnet_processing_thread_is_running(self, built):
        """DroneController.__init__ starts command processing after
        registering the commands, exactly as before."""
        handler = built.controller._DroneController__command_handler
        assert handler._CommandHandler__running is True
        thread = handler._CommandHandler__thread
        assert thread is not None and thread.daemon and thread.is_alive()
