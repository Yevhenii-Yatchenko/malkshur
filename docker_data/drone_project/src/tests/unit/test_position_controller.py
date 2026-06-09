"""Unit tests for PositionController (src/position_controller.py):
pixels_to_meters, the navigation-flag mode state machine in update()
(Step 4, IE-3 -- the former confidence-sentinel), and the Step 3
csv_logger constructor injection (LC-1).

I/O notes: since Step 3 the CSV logger is injected through the constructor's
``csv_logger=`` parameter (a NullCSVLogger here -- no mkdir/open under
logs/csv/), so the Step 2 mock.patch of PositionCSVLogger is gone.
__init__ still eagerly creates a file logger via get_logger
(logs/position_controller.log), so that one remains patched in the module
namespace; the rest of construction (the three real PIDControllers, config
binding) runs for real.

The expected values below are literals on purpose: with the current
position_config.ALTITUDE_COMPENSATION (reference_altitude=2.0,
pixels_per_meter_at_ref=500, adaptive scaling on, altitude clipped to
[0.5, 10.0]) the conversion is

    meters = pixels * clip(altitude, 0.5, 10.0) / 1000.0

A change to either the formula or the config is a behavior change and should
fail these tests.
"""

from unittest import mock

import pytest

# Shared null-object logger of the characterization suite (importable as a
# bare module: tests/unit/ is on sys.path under pytest's prepend import mode).
from generate_characterization import NullCSVLogger
from src.position_config import (
    ALTITUDE_COMPENSATION, POSITION_PID_X, POSITION_PID_Y, PWM_LIMITS)
from src.position_controller import PositionController

pytestmark = [pytest.mark.unit, pytest.mark.pid]


@pytest.fixture
def controller():
    null_logger = NullCSVLogger()
    with mock.patch("src.position_controller.get_logger", return_value=mock.Mock()):
        instance = PositionController(csv_logger=null_logger)
        # Sanity: the injected null logger really is in place (Step 3).
        assert instance.csv_logger is null_logger
        yield instance


class TestConfigAssumptions:
    """Pin the config branch the golden numbers below depend on."""

    def test_adaptive_scaling_is_enabled(self):
        assert ALTITUDE_COMPENSATION["use_adaptive_scaling"] is True

    def test_reference_calibration(self):
        assert ALTITUDE_COMPENSATION["reference_altitude"] == pytest.approx(2.0)
        assert ALTITUDE_COMPENSATION["pixels_per_meter_at_ref"] == 500
        assert ALTITUDE_COMPENSATION["min_altitude"] == pytest.approx(0.5)
        assert ALTITUDE_COMPENSATION["max_altitude"] == pytest.approx(10.0)


class TestPixelsToMeters:
    @pytest.mark.parametrize(
        "pixels, altitude, expected_meters",
        [
            # Known conversions at the reference altitude (2.0 m):
            # 500 px == 1 m, so 100 px == 0.2 m.
            (100.0, 2.0, 0.2),
            (500.0, 2.0, 1.0),
            (1234.0, 2.0, 2.468),
            # Scaling is linear in altitude.
            (100.0, 1.0, 0.1),
            (100.0, 4.0, 0.4),
            (250.0, 8.0, 2.0),
            # Zero pixels -> zero meters at any altitude.
            (0.0, 2.0, 0.0),
            (0.0, 9.0, 0.0),
            (0.0, 0.0, 0.0),
            # Negative drift converts symmetrically.
            (-100.0, 2.0, -0.2),
            (-250.0, 4.0, -1.0),
            (-1.0, 2.0, -0.002),
        ],
    )
    def test_known_conversions(self, controller, pixels, altitude, expected_meters):
        result = controller.pixels_to_meters(pixels, altitude)
        assert result == pytest.approx(expected_meters)

    @pytest.mark.parametrize(
        "pixels, altitude, expected_meters",
        [
            # Altitude is clipped to [0.5, 10.0] before scaling.
            (100.0, 0.0, 0.05),     # below min -> behaves like 0.5 m
            (100.0, 0.25, 0.05),
            (100.0, -3.0, 0.05),    # negative altitude also clips to 0.5 m
            (100.0, 50.0, 1.0),     # above max -> behaves like 10.0 m
            # Exact boundary values pass through unclipped.
            (100.0, 0.5, 0.05),
            (100.0, 10.0, 1.0),
        ],
    )
    def test_altitude_clipping(self, controller, pixels, altitude, expected_meters):
        result = controller.pixels_to_meters(pixels, altitude)
        assert result == pytest.approx(expected_meters)

    def test_linear_in_pixels_at_fixed_altitude(self, controller):
        base = controller.pixels_to_meters(10.0, 3.0)
        assert controller.pixels_to_meters(30.0, 3.0) == pytest.approx(3.0 * base)
        assert controller.pixels_to_meters(-10.0, 3.0) == pytest.approx(-base)


def _update(controller, *, confidence, dx=0.0, dy=0.0, angle=0.0, step=0, **kwargs):
    """Drive update() with deterministic time; only the fields under test vary."""
    return controller.update(
        dx_pixels=dx,
        dy_pixels=dy,
        angle_deg=angle,
        confidence=confidence,
        altitude=2.0,
        current_time=step * 0.02,
        **kwargs,
    )


class TestNavigationFlagStateMachine:
    """Pin the navigation-flag mode state machine at the top of update().

    Producer/consumer contract since Step 4 (producer side is pinned in
    tests/unit/test_command_modifier.py):

    - While navigating, sky_anchor's CommandModifier emits
      ``navigation=True`` in the payload (and keeps the historic
      matches_percent=101.0 placeholder, now informational only).
      src/sky_anchor_client.py parses the payload into a StabilizerReading
      and src/controller.py forwards ``navigation=reading.navigation``.
    - navigation=True  -> navigation mode: position-PID ki/kd zeroed
      (__enable_navigation), controller state reset.
    - navigation=False -> stabilization mode: ki/kd restored from the
      POSITION_PID_X/Y config globals (__enable_stabilization), state reset.
    - The flag is authoritative on EVERY call; transitions happen only when
      the flag disagrees with the current mode (no reset spam within a mode).
    - confidence is informational only: the deleted ``confidence == 1.01``
      sentinel must NOT switch modes anymore (no knife-edge float equality
      left in the mode logic).

    The mode flag itself is name-mangled, so these tests assert the observable
    consequences instead: the live gains on the public position_pid_x/y
    PIDControllers, the reset side effects on public state, and the resulting
    PWM outputs.
    """

    def _assert_stabilization_gains(self, controller):
        assert controller.position_pid_x.ki == POSITION_PID_X["ki"]
        assert controller.position_pid_x.kd == POSITION_PID_X["kd"]
        assert controller.position_pid_y.ki == POSITION_PID_Y["ki"]
        assert controller.position_pid_y.kd == POSITION_PID_Y["kd"]

    def _assert_navigation_gains(self, controller):
        assert controller.position_pid_x.ki == 0.0
        assert controller.position_pid_x.kd == 0.0
        assert controller.position_pid_y.ki == 0.0
        assert controller.position_pid_y.kd == 0.0

    def test_starts_in_stabilization_with_config_gains(self, controller):
        self._assert_stabilization_gains(controller)

    def test_navigation_flag_switches_to_navigation_and_zeroes_ki_kd(
        self, controller
    ):
        _update(controller, confidence=1.01, navigation=True)
        self._assert_navigation_gains(controller)

    def test_navigation_keeps_kp_and_angle_pid_untouched(self, controller):
        angle_gains = (
            controller.angle_pid.kp, controller.angle_pid.ki, controller.angle_pid.kd,
        )
        _update(controller, confidence=1.01, navigation=True)
        assert controller.position_pid_x.kp == POSITION_PID_X["kp"]
        assert controller.position_pid_y.kp == POSITION_PID_Y["kp"]
        assert (
            controller.angle_pid.kp, controller.angle_pid.ki, controller.angle_pid.kd,
        ) == angle_gains

    def test_flag_false_restores_gains_from_config_globals(self, controller):
        _update(controller, confidence=1.01, navigation=True, step=0)
        self._assert_navigation_gains(controller)
        _update(controller, confidence=0.99, navigation=False, step=1)
        self._assert_stabilization_gains(controller)

    def test_omitted_flag_defaults_to_stabilization(self, controller):
        """Backward compatibility: callers that never navigate (and never
        pass the parameter) keep the historic stabilization-only behavior."""
        _update(controller, confidence=1.01, navigation=True, step=0)
        self._assert_navigation_gains(controller)
        _update(controller, confidence=0.85, step=1)  # no navigation kwarg
        self._assert_stabilization_gains(controller)

    @pytest.mark.parametrize(
        "confidence", [0.0, 0.85, 1.0, 1.005, 1.0099999999, 1.01, 1.02, 2.0]
    )
    def test_confidence_alone_never_enters_navigation(self, controller, confidence):
        """The sentinel comparison is DELETED: no confidence value -- not
        even the old exact wire value 101.0 / 100.0 == 1.01 -- may flip the
        mode while navigation=False."""
        _update(controller, confidence=confidence, navigation=False)
        self._assert_stabilization_gains(controller)

    @pytest.mark.parametrize("confidence", [0.0, 0.85, 1.0, 1.01, 2.0])
    def test_flag_is_authoritative_regardless_of_confidence(
        self, controller, confidence
    ):
        """The flag alone drives the mode in BOTH directions, decoupled from
        whatever the (informational) confidence value happens to be."""
        _update(controller, confidence=confidence, navigation=True, step=0)
        self._assert_navigation_gains(controller)
        _update(controller, confidence=confidence, navigation=False, step=1)
        self._assert_stabilization_gains(controller)

    def test_mode_switch_resets_velocity_estimate_and_navigation_targets(
        self, controller
    ):
        # Build up observable state in stabilization mode...
        _update(controller, confidence=0.85, dx=20.0, step=0,
                target_dx_pixels=42.0, target_dy_pixels=-17.0)
        _update(controller, confidence=0.85, dx=40.0, step=1)
        assert controller.estimated_velocity_x != 0.0
        assert controller.navigation_target_dx == 42.0
        assert controller.navigation_target_dy == -17.0
        # ...the switching update wipes it via reset() before acting.
        _update(controller, confidence=1.01, navigation=True, step=2)
        assert controller.estimated_velocity_x == 0.0
        assert controller.estimated_velocity_y == 0.0
        assert controller.navigation_target_dx == 0.0
        assert controller.navigation_target_dy == 0.0

    def test_repeated_flag_does_not_reset_within_a_mode(self, controller):
        """Only flag/mode DISAGREEMENT switches (and resets); repeating the
        same flag must not wipe accumulated state on every call."""
        _update(controller, confidence=1.01, navigation=True, step=0)
        _update(controller, confidence=1.01, navigation=True, dx=20.0, step=1)
        _update(controller, confidence=1.01, navigation=True, dx=40.0, step=2)
        # A reset between steps 1 and 2 would have left the estimate at 0.
        assert controller.estimated_velocity_x != 0.0

    def test_navigation_mode_outputs_are_pure_proportional(self, controller):
        """With ki = kd = 0 the position PIDs become stateless P controllers:
        identical drift inputs must produce identical PWM outputs on every
        call (no integral creep, no derivative kick)."""
        _update(controller, confidence=1.01, navigation=True, step=0)
        results = [
            _update(controller, confidence=1.01, navigation=True,
                    dx=-20.0, dy=10.0, step=step)
            for step in (1, 2, 3)
        ]
        # COORDINATE_SYSTEM inverts both axes: dx=-20 -> +20 px (error -20),
        # dy=+10 -> -10 px (error +10); with kp=1.2 the pure-P outputs are
        # roll = int(1500 - 24.0) = 1476 and pitch = int(1500 + 12.0) = 1512.
        expected_roll = int(PWM_LIMITS["neutral"] + POSITION_PID_X["kp"] * -20.0)
        expected_pitch = int(PWM_LIMITS["neutral"] + POSITION_PID_Y["kp"] * 10.0)
        assert [result["roll_pwm"] for result in results] == [expected_roll] * 3
        assert [result["pitch_pwm"] for result in results] == [expected_pitch] * 3

    def test_zero_drift_in_navigation_returns_neutral_pwm(self, controller):
        _update(controller, confidence=1.01, navigation=True, step=0)
        result = _update(controller, confidence=1.01, navigation=True, step=1)
        assert result["roll_pwm"] == PWM_LIMITS["neutral"]
        assert result["pitch_pwm"] == PWM_LIMITS["neutral"]
        assert result["yaw_pwm"] == PWM_LIMITS["neutral"]


class TestCsvLoggerInjection:
    """Step 3 (LC-1): the ``csv_logger=`` constructor injection contract.

    - csv_logger omitted/None -> a file-writing PositionCSVLogger is created
      exactly as before (backward compatibility; the class is patched here so
      the test itself stays free of file I/O);
    - csv_logger injected -> NO PositionCSVLogger is constructed (no file
      I/O) and update() routes its log write to the injected object.
    """

    def test_default_still_creates_file_logger_exactly_as_before(self):
        ts = mock.sentinel.start_timestamp
        with mock.patch("src.position_controller.PositionCSVLogger") as csv_cls, \
                mock.patch("src.position_controller.get_logger",
                           return_value=mock.Mock()):
            instance = PositionController(start_timestamp=ts)
        csv_cls.assert_called_once_with(start_timestamp=ts)
        assert instance.csv_logger is csv_cls.return_value

    def test_injected_logger_is_used_and_no_file_logger_is_created(self):
        fake = mock.Mock(name="injected_csv_logger")
        with mock.patch("src.position_controller.PositionCSVLogger") as csv_cls, \
                mock.patch("src.position_controller.get_logger",
                           return_value=mock.Mock()):
            instance = PositionController(csv_logger=fake)
        csv_cls.assert_not_called()
        assert instance.csv_logger is fake

    def test_update_logs_to_the_injected_logger(self):
        fake = mock.Mock(name="injected_csv_logger")
        with mock.patch("src.position_controller.PositionCSVLogger") as csv_cls, \
                mock.patch("src.position_controller.get_logger",
                           return_value=mock.Mock()):
            instance = PositionController(csv_logger=fake)
        _update(instance, confidence=0.85, dx=10.0, dy=-5.0, angle=0.5)
        csv_cls.assert_not_called()
        assert fake.append.call_count == 1
        logged = fake.append.call_args[0][0]
        assert logged["timestamp"] == pytest.approx(0.0)
        assert logged["matches_percent"] == pytest.approx(85.0)
        assert logged["altitude"] == pytest.approx(2.0)
