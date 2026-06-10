"""Orchestration tests for the (Step 7) thin DroneController.

With the composition root in src/app.py, ``DroneController.__init__`` only
stores injected collaborators, registers the command handlers, starts
telnet processing and arms the optional ARM_IN timer -- so the controller
is finally constructible with plain mocks (no MAVLink, no sockets).

Pinned here:

- the startup wiring order (8 commands registered, then processing starts)
  and that construction touches NO hardware (connect/start are app.py's
  job);
- the auto-arm Timer wiring (ARM_IN) and the auto-setHeight(5)-on-arm
  behavior (load-bearing per the plan's risk list);
- the loop-state attributes are per instance (Step 7 carried bullet:
  formerly class attributes);
- the re-enable consumption line in __updateThrottle -- the Step 6 review
  gap: when InterceptGuidance raises ``reenable_stabilization`` and the
  stabilizer is NOT connected, the controller logs once and calls
  ``start_stabilizer_process()`` before running the stabilization behavior;
  when the stabilizer IS connected it does neither;
- the ``active`` gate: an active intercept skips the stabilization behavior
  and goes straight to altitude control + RC send;
- ``get_active_target()`` is called WITHOUT per-call thresholds (the
  detection server owns them since Step 7).

The ARM_IN module constant is forced per test (the container sets
ARM_IN=1.0, which would otherwise start real timers).
"""

from types import SimpleNamespace
from unittest import mock

import pytest

from src.controller import DroneController
from src.domain.types import AttitudeSetpoints
from src.flight.intercept import InterceptResult
from src.flight.setpoints import RCSetpoints

pytestmark = [pytest.mark.unit]


def make_controller(**overrides):
    """A DroneController over mocks (real RCSetpoints, so PWM/altitude
    state flows realistically through the orchestration under test)."""
    deps = dict(
        logger=mock.Mock(),
        mavlink=mock.Mock(),
        sensor=mock.Mock(),
        stabilizer=mock.Mock(),
        battery_monitor=None,
        altitude_controller=mock.Mock(),
        rc=RCSetpoints(),
        detection_server=mock.Mock(),
        detection_client=mock.Mock(),
        intercept_guidance=mock.Mock(),
        stabilization=mock.Mock(),
        signal_handler=mock.Mock(),
        command_handler=mock.Mock(),
    )
    deps.update(overrides)
    return DroneController(**deps), SimpleNamespace(**deps)


@pytest.fixture
def env(monkeypatch):
    monkeypatch.setattr("src.controller.ARM_IN", 0)
    controller, deps = make_controller()
    # Baseline collaborator behavior for the orchestration tests.
    deps.sensor.get_altitude.return_value = 5.0
    deps.altitude_controller.update.return_value = 1500
    deps.stabilization.update.return_value = None
    deps.intercept_guidance.is_intercepting = False
    return controller, deps


def update_throttle(controller):
    controller._DroneController__updateThrottle()


REENABLE_RESULT = InterceptResult(
    setpoints=AttitudeSetpoints(
        roll_pwm=1500, pitch_pwm=1500, yaw_pwm=1500, target_altitude=4.5
    ),
    active=False,
    reenable_stabilization=True,
)


class TestThinConstruction:
    def test_construction_touches_no_hardware(self, monkeypatch):
        """__init__ must not connect/start anything: that is app.py's job."""
        monkeypatch.setattr("src.controller.ARM_IN", 0)
        controller, deps = make_controller()
        deps.mavlink.connect.assert_not_called()
        deps.sensor.start.assert_not_called()
        deps.sensor.get_altitude.assert_not_called()
        deps.stabilizer.start_stabilizer_process.assert_not_called()

    def test_registers_commands_then_starts_telnet_processing(
        self, monkeypatch
    ):
        monkeypatch.setattr("src.controller.ARM_IN", 0)
        controller, deps = make_controller()
        registered = [
            call.args[0]
            for call in deps.command_handler.register_command.call_args_list
        ]
        assert registered == [
            'mode', 'arm', 'setHeight', 'move', 'land', 'stabilize',
            'recognizeClient', 'monitor',
        ]
        # Processing starts only after every command is registered.
        method_names = [
            name for name, args, kwargs in deps.command_handler.mock_calls
        ]
        assert method_names.index('start_telnet_processing') > max(
            index for index, name in enumerate(method_names)
            if name == 'register_command'
        )

    def test_loop_state_is_per_instance(self, monkeypatch):
        """Step 7 carried bullet: __current_altitude/__is_up moved from
        class attributes into __init__."""
        monkeypatch.setattr("src.controller.ARM_IN", 0)
        controller, _ = make_controller()
        assert '_DroneController__current_altitude' in vars(controller)
        assert '_DroneController__is_up' in vars(controller)
        assert not hasattr(DroneController, '_DroneController__current_altitude')
        assert not hasattr(DroneController, '_DroneController__is_up')


class TestAutoArmTimer:
    def test_no_timer_when_arm_in_unset(self, monkeypatch):
        monkeypatch.setattr("src.controller.ARM_IN", 0)
        with mock.patch("src.controller.Timer") as timer_cls:
            make_controller()
        timer_cls.assert_not_called()

    def test_timer_armed_with_the_env_delay(self, monkeypatch):
        monkeypatch.setattr("src.controller.ARM_IN", 30.0)
        with mock.patch("src.controller.Timer") as timer_cls:
            controller, _ = make_controller()
        timer_cls.assert_called_once()
        delay, callback = timer_cls.call_args.args
        assert delay == 30.0
        assert callback == controller._DroneController__startAutoArming
        timer_cls.return_value.start.assert_called_once_with()


class TestArmAutoSetsHeight:
    def test_successful_arm_sets_target_altitude_to_five(
        self, env, monkeypatch
    ):
        """Load-bearing side effect kept verbatim: arm,0 -> setHeight(5)."""
        monkeypatch.setattr("src.controller.time.sleep", lambda seconds: None)
        controller, deps = env
        deps.mavlink.arm.return_value = True
        controller._armingDisarming([0])
        rc = deps.rc
        assert rc.target_altitude == 5.0
        deps.altitude_controller.position_pid.reset.assert_called_once_with()

    def test_failed_arm_leaves_target_untouched(self, env):
        controller, deps = env
        deps.mavlink.arm.return_value = False
        controller._armingDisarming([0])
        assert deps.rc.target_altitude == 0.2
        deps.logger.error.assert_called_once()


class TestReenableConsumption:
    """The Step 6 review gap: the controller-side consumption of the
    ``reenable_stabilization`` intent."""

    def _drive(self, env, *, connected):
        controller, deps = env
        deps.detection_server.is_running.return_value = True
        deps.detection_server.get_active_target.return_value = None
        deps.intercept_guidance.update.return_value = REENABLE_RESULT
        deps.stabilizer.is_connected = connected
        update_throttle(controller)
        return controller, deps

    def test_intent_restarts_stabilizer_when_disconnected(self, env):
        controller, deps = self._drive(env, connected=False)
        deps.logger.info.assert_any_call(
            "Re-enabling stabilization after intercept"
        )
        deps.stabilizer.start_stabilizer_process.assert_called_once_with()
        # active=False on the deactivation iteration: the normal
        # stabilization pass still runs afterwards, as before.
        deps.stabilization.update.assert_called_once()

    def test_intent_is_a_noop_when_still_connected(self, env):
        controller, deps = self._drive(env, connected=True)
        deps.stabilizer.start_stabilizer_process.assert_not_called()
        assert (
            mock.call("Re-enabling stabilization after intercept")
            not in deps.logger.info.call_args_list
        )

    def test_deactivation_setpoints_are_applied_to_rc(self, env):
        controller, deps = self._drive(env, connected=True)
        assert deps.rc.target_altitude == 4.5
        deps.altitude_controller.update.assert_called_once_with(
            target_altitude=4.5, current_altitude=5.0
        )


class TestUpdateThrottleOrchestration:
    def test_detection_thresholds_are_not_passed_per_call(self, env):
        """Step 7 carried bullet, consumer side: the controller calls
        get_active_target() bare; the server owns the thresholds."""
        controller, deps = env
        deps.detection_server.is_running.return_value = True
        deps.detection_server.get_active_target.return_value = None
        deps.intercept_guidance.update.return_value = InterceptResult(
            setpoints=None, active=False, reenable_stabilization=False
        )
        update_throttle(controller)
        deps.detection_server.get_active_target.assert_called_once_with()

    def test_active_intercept_skips_stabilization(self, env):
        controller, deps = env
        deps.detection_server.is_running.return_value = True
        deps.detection_server.get_active_target.return_value = mock.Mock()
        deps.intercept_guidance.update.return_value = InterceptResult(
            setpoints=AttitudeSetpoints(
                roll_pwm=1500, pitch_pwm=1520, yaw_pwm=1530,
                target_altitude=5.005,
            ),
            active=True,
            reenable_stabilization=False,
        )
        update_throttle(controller)
        deps.stabilization.update.assert_not_called()
        deps.altitude_controller.update.assert_called_once_with(
            target_altitude=5.005, current_altitude=5.0
        )
        deps.mavlink.send_rc_override.assert_called_once_with(
            roll=1500, pitch=1520, throttle=1500, yaw=1530
        )

    def test_idle_path_runs_stabilization_then_altitude_control(self, env):
        controller, deps = env
        deps.detection_server.is_running.return_value = False
        update_throttle(controller)
        deps.detection_server.get_active_target.assert_not_called()
        deps.intercept_guidance.update.assert_not_called()
        deps.stabilization.update.assert_called_once_with(
            current_altitude=5.0,
            target_altitude=0.2,
            intercept_active=False,
        )
        deps.altitude_controller.update.assert_called_once_with(
            target_altitude=0.2, current_altitude=5.0
        )
        deps.mavlink.send_rc_override.assert_called_once_with(
            roll=1500, pitch=1500, throttle=1500, yaw=1500
        )

    def test_stabilization_setpoints_are_applied(self, env):
        controller, deps = env
        deps.detection_server.is_running.return_value = False
        deps.stabilization.update.return_value = AttitudeSetpoints(
            roll_pwm=1480, pitch_pwm=1510, yaw_pwm=1495
        )
        update_throttle(controller)
        deps.mavlink.send_rc_override.assert_called_once_with(
            roll=1480, pitch=1510, throttle=1500, yaw=1495
        )
        # Stabilization never touches the altitude target.
        assert deps.rc.target_altitude == 0.2
