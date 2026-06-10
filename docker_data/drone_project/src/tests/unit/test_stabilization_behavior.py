"""Unit tests for StabilizationBehavior (src/flight/stabilization.py).

Step 5 (HC-1) extracted the stabilization branch of
DroneController.__updateThrottle.  Contract pinned here:

- gate: position corrections only run while the stabilizer manager reports
  connected AND no intercept is active; otherwise the auto-trigger branch
  runs instead (including the historic quirk that the trigger branch is
  reached while connected-but-intercepting);
- reading -> PWM passthrough: every StabilizerReading field maps onto the
  PositionController.update() call exactly as the controller used to map it
  (dx/dy, the hardwired angle_deg=0, matches_percent/100 confidence, the
  altitude, navigation targets, navigation flag), and the returned PWM trio
  comes back as AttitudeSetpoints with the target altitude untouched;
- the navigation flag propagates into the REAL PositionController mode
  state machine (ki/kd zeroed and restored -- the Step 4 contract);
- altitude-reached trigger: start_stabilizer_process() is invoked whenever
  the drone is strictly within 0.1 m of its target and corrections are not
  running; the once-only semantics live in StabilizerManager's
  __process_started flag (repeat triggers must NOT respawn the process),
  pinned end-to-end with a real StabilizerManager.

The position controller is the REAL one (with the characterization suite's
NullCSVLogger injected, Step 3) -- only the I/O edges (stabilizer manager,
file loggers, subprocess spawn) are test doubles.  The PWM literals follow
the same pure-proportional reasoning as test_position_controller.py:
navigation mode zeroes ki/kd, so outputs are time-independent P terms.

Step 6 (review absorption): ``intercept_active`` lost its ``False`` default
(it existed for test ergonomics only), so every call site here passes the
gate input explicitly, exactly like the controller does.
"""

from unittest import mock

import pytest

from generate_characterization import NullCSVLogger
from src.domain.types import AttitudeSetpoints, StabilizerReading
from src.flight.stabilization import StabilizationBehavior
from src.position_config import POSITION_PID_X, POSITION_PID_Y, PWM_LIMITS
from src.position_controller import PositionController
from src.stabilizer_manager import StabilizerManager

pytestmark = [pytest.mark.unit]


def make_reading(dx=0.0, dy=0.0, matches=85.0, timestamp=1.0,
                 navigation=False, target_dx=None, target_dy=None):
    return StabilizerReading(
        dx=dx, dy=dy, angle_deg=0.0, matches_percent=matches,
        timestamp=timestamp, navigation=navigation,
        target_dx_pixels=target_dx, target_dy_pixels=target_dy,
    )


@pytest.fixture
def position_controller():
    with mock.patch("src.position_controller.get_logger",
                    return_value=mock.Mock()):
        yield PositionController(csv_logger=NullCSVLogger())


@pytest.fixture
def stabilizer():
    stab = mock.Mock()
    # Plain attribute: the behavior reads ``is_connected`` (a property on
    # the real StabilizerManager) without calling it.
    stab.is_connected = True
    stab.poll_new.return_value = None
    return stab


@pytest.fixture
def behavior(stabilizer, position_controller):
    return StabilizationBehavior(
        stabilizer=stabilizer, position_controller=position_controller
    )


class TestGate:
    def test_not_connected_skips_polling_and_returns_none(
        self, behavior, stabilizer, position_controller
    ):
        stabilizer.is_connected = False
        assert behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        ) is None
        stabilizer.poll_new.assert_not_called()
        assert position_controller.update_count == 0

    def test_intercept_active_suppresses_corrections_even_when_connected(
        self, behavior, stabilizer, position_controller
    ):
        stabilizer.poll_new.return_value = make_reading(dx=50.0)
        result = behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=True
        )
        assert result is None
        stabilizer.poll_new.assert_not_called()
        assert position_controller.update_count == 0

    def test_no_fresh_reading_returns_none_without_running_the_pid(
        self, behavior, stabilizer, position_controller
    ):
        stabilizer.poll_new.return_value = None
        assert behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        ) is None
        stabilizer.poll_new.assert_called_once_with()
        assert position_controller.update_count == 0


class TestReadingPassthrough:
    def test_zero_drift_reading_returns_neutral_setpoints(
        self, behavior, stabilizer
    ):
        """Real math: zero pixel error -> zero P/I/D on every axis ->
        exactly neutral PWM, independent of wall-clock dt."""
        stabilizer.poll_new.return_value = make_reading(dx=0.0, dy=0.0)
        result = behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )
        assert result == AttitudeSetpoints(
            roll_pwm=PWM_LIMITS["neutral"],
            pitch_pwm=PWM_LIMITS["neutral"],
            yaw_pwm=PWM_LIMITS["neutral"],
        )
        assert result.target_altitude is None

    def test_reading_fields_map_onto_position_controller_call(
        self, behavior, stabilizer, position_controller
    ):
        """Pin the exact argument mapping (incl. the hardwired angle_deg=0
        and the matches/100 confidence) and that the returned setpoints are
        exactly the real update()'s PWM trio."""
        recorded = []
        real_update = position_controller.update

        def recording_update(**kwargs):
            output = real_update(**kwargs)
            recorded.append((kwargs, output))
            return output

        position_controller.update = recording_update
        stabilizer.poll_new.return_value = make_reading(
            dx=12.5, dy=-3.25, matches=85.0, timestamp=10.0,
            target_dx=42.0, target_dy=-17.0,
        )

        result = behavior.update(
            current_altitude=4.5, target_altitude=5.0, intercept_active=False
        )

        assert len(recorded) == 1
        kwargs, output = recorded[0]
        assert kwargs == {
            "dx_pixels": 12.5,
            "dy_pixels": -3.25,
            "angle_deg": 0,
            "confidence": 0.85,
            "altitude": 4.5,
            "target_dx_pixels": 42.0,
            "target_dy_pixels": -17.0,
            "navigation": False,
        }
        assert result == AttitudeSetpoints(
            roll_pwm=output["roll_pwm"],
            pitch_pwm=output["pitch_pwm"],
            yaw_pwm=output["yaw_pwm"],
        )
        assert result.target_altitude is None

    def test_navigation_mode_pwm_is_pure_proportional_passthrough(
        self, behavior, stabilizer
    ):
        """With navigation readings the real position PIDs are pure P
        (ki=kd=0), so the PWM literals are exact and time-independent:
        COORDINATE_SYSTEM inverts both axes, kp=1.2 -> roll 1476/pitch 1512
        (same reasoning as test_position_controller.py)."""
        stabilizer.poll_new.return_value = make_reading(
            navigation=True, timestamp=1.0
        )
        behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )

        stabilizer.poll_new.return_value = make_reading(
            dx=-20.0, dy=10.0, navigation=True, timestamp=2.0
        )
        result = behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )

        expected_roll = int(PWM_LIMITS["neutral"] + POSITION_PID_X["kp"] * -20.0)
        expected_pitch = int(PWM_LIMITS["neutral"] + POSITION_PID_Y["kp"] * 10.0)
        assert result.roll_pwm == expected_roll == 1476
        assert result.pitch_pwm == expected_pitch == 1512
        assert result.yaw_pwm == PWM_LIMITS["neutral"]


class TestNavigationFlagPropagation:
    def _assert_stabilization_gains(self, controller):
        assert controller.position_pid_x.ki == POSITION_PID_X["ki"]
        assert controller.position_pid_x.kd == POSITION_PID_X["kd"]
        assert controller.position_pid_y.ki == POSITION_PID_Y["ki"]
        assert controller.position_pid_y.kd == POSITION_PID_Y["kd"]

    def test_navigation_reading_switches_the_real_controller_mode(
        self, behavior, stabilizer, position_controller
    ):
        stabilizer.poll_new.return_value = make_reading(
            navigation=True, matches=101.0, timestamp=1.0
        )
        behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )
        assert position_controller.position_pid_x.ki == 0.0
        assert position_controller.position_pid_x.kd == 0.0
        assert position_controller.position_pid_y.ki == 0.0
        assert position_controller.position_pid_y.kd == 0.0

    def test_flag_false_restores_stabilization_gains(
        self, behavior, stabilizer, position_controller
    ):
        stabilizer.poll_new.return_value = make_reading(
            navigation=True, timestamp=1.0
        )
        behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )
        stabilizer.poll_new.return_value = make_reading(
            navigation=False, timestamp=2.0
        )
        behavior.update(
            current_altitude=2.0, target_altitude=5.0, intercept_active=False
        )
        self._assert_stabilization_gains(position_controller)


class TestAltitudeReachedTrigger:
    @pytest.fixture
    def disconnected(self, behavior, stabilizer):
        stabilizer.is_connected = False
        return behavior, stabilizer

    def test_fires_when_strictly_within_window(self, disconnected):
        behavior, stabilizer = disconnected
        # abs(4.9 - 5.0) == 0.09999... < 0.1 (the production float edge).
        assert behavior.update(
            current_altitude=4.9, target_altitude=5.0, intercept_active=False
        ) is None
        stabilizer.start_stabilizer_process.assert_called_once_with()

    def test_fires_when_slightly_above_target(self, disconnected):
        behavior, stabilizer = disconnected
        behavior.update(
            current_altitude=5.05, target_altitude=5.0, intercept_active=False
        )
        stabilizer.start_stabilizer_process.assert_called_once_with()

    @pytest.mark.parametrize(
        "current, target",
        [
            (1.0, 1.1),     # abs diff 0.10000000000000009 -> NOT < 0.1
            (4.875, 5.0),   # exact 0.125
            (2.0, 5.0),     # far away
        ],
    )
    def test_does_not_fire_at_or_beyond_the_window(
        self, disconnected, current, target
    ):
        behavior, stabilizer = disconnected
        behavior.update(
            current_altitude=current, target_altitude=target,
            intercept_active=False,
        )
        stabilizer.start_stabilizer_process.assert_not_called()

    def test_does_not_fire_without_an_altitude_reading(self, disconnected):
        behavior, stabilizer = disconnected
        behavior.update(
            current_altitude=None, target_altitude=5.0, intercept_active=False
        )
        stabilizer.start_stabilizer_process.assert_not_called()

    def test_fires_every_iteration_while_condition_holds(self, disconnected):
        """The behavior re-triggers each loop; the once-only semantics are
        the StabilizerManager's (pinned below with the real manager)."""
        behavior, stabilizer = disconnected
        behavior.update(
            current_altitude=5.0, target_altitude=5.0, intercept_active=False
        )
        behavior.update(
            current_altitude=5.0, target_altitude=5.0, intercept_active=False
        )
        assert stabilizer.start_stabilizer_process.call_count == 2

    def test_fires_even_while_connected_if_intercept_active(
        self, behavior, stabilizer
    ):
        """Historic quirk preserved: the trigger branch is the plain else of
        the connected-and-not-intercepting gate, so a connected stabilizer
        still gets the (no-op) start call during an intercept at target
        altitude."""
        behavior.update(
            current_altitude=5.0, target_altitude=5.0, intercept_active=True
        )
        stabilizer.start_stabilizer_process.assert_called_once_with()
        stabilizer.poll_new.assert_not_called()


class TestTriggerOnceOnlySemantics:
    def test_repeated_triggers_spawn_the_process_exactly_once(self):
        """End-to-end with a REAL StabilizerManager: the trigger may fire on
        every iteration, but only the first call actually spawns the
        sky_anchor process (__process_started); repeats are warn-and-return.
        Only the I/O edges are doubled: the subprocess spawn and the TCP
        client (whose connect never succeeds here, keeping the manager
        disconnected so the trigger branch stays reachable)."""
        position = mock.Mock()
        with mock.patch("src.stabilizer_manager.SkyAnchorClient") as client_cls, \
                mock.patch("src.stabilizer_manager.subprocess.Popen") as popen:
            client_cls.return_value.connect.return_value = False
            manager = StabilizerManager(
                stabilizer_path="unused/sky_anchor/main.py", logger=mock.Mock()
            )
            behavior = StabilizationBehavior(
                stabilizer=manager, position_controller=position
            )
            try:
                assert manager.is_process_started is False
                assert behavior.update(
                    current_altitude=5.0, target_altitude=5.0,
                    intercept_active=False,
                ) is None
                assert manager.is_process_started is True
                assert behavior.update(
                    current_altitude=5.0, target_altitude=5.0,
                    intercept_active=False,
                ) is None
                assert popen.call_count == 1
                position.update.assert_not_called()
            finally:
                manager.cleanup()
