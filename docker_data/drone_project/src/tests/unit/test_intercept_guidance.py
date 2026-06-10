"""Unit tests for InterceptGuidance (src/flight/intercept.py).

Step 5 (HC-1/IE-1) extracted the intercept state machine out of
DroneController.__updateThrottle; the numeric logic moved VERBATIM and is
pinned here against the resolved src/detection_config.py values:

- activation: any non-None DetectionReading activates the mode (the
  confidence floor / staleness threshold are the DetectionServer's job --
  pinned in test_detection_server.py -- so even a low-confidence reading
  object activates if the server handed it out), logging once per
  activation;
- yaw correction: PWM_NEUTRAL + int(dir_x * INTERCEPT_YAW_GAIN) outside the
  +/-INTERCEPT_DEADBAND_X deadband (strict >, int() truncates toward zero),
  neutral inside;
- altitude stepping: target_altitude + dir_y * INTERCEPT_ALTITUDE_STEP per
  call outside the +/-INTERCEPT_DEADBAND_Y deadband, untouched inside;
- pitch is ALWAYS neutral + INTERCEPT_PITCH_OFFSET and roll ALWAYS
  INTERCEPT_ROLL_NEUTRAL while a target is active (no deadband on these);
- timeout deactivation: only while the mode is active AND
  get_time_since_last_detection() >= INTERCEPT_TIMEOUT_SECONDS (the time
  query short-circuits when the mode is inactive, exactly like the former
  inline ``and``); deactivation returns neutral attitude, holds altitude at
  the current reading (or leaves the target unchanged when there is no
  reading), and re-enables stabilization via the manager IF it is not
  connected (the handoff);
- exit_intercept(): the disarm path, log + flag reset only when active.

Collaborators (detection server for the staleness query, stabilizer manager
for the handoff, the controller's logger) are constructor-injected, so the
tests drive them as mocks; there is no controller import anywhere.
"""

from unittest import mock

import pytest

from src.detection_config import (
    INTERCEPT_ALTITUDE_STEP,
    INTERCEPT_DEADBAND_X,
    INTERCEPT_DEADBAND_Y,
    INTERCEPT_PITCH_OFFSET,
    INTERCEPT_ROLL_NEUTRAL,
    INTERCEPT_TIMEOUT_SECONDS,
    INTERCEPT_YAW_GAIN,
    PWM_NEUTRAL,
)
from src.domain.types import AttitudeSetpoints, DetectionReading
from src.flight.intercept import InterceptGuidance

pytestmark = [pytest.mark.unit]


def reading(confidence=0.9, dir_x=0.0, dir_y=0.0):
    return DetectionReading(confidence=confidence, dir_x=dir_x, dir_y=dir_y)


@pytest.fixture
def env():
    detection_server = mock.Mock()
    detection_server.get_time_since_last_detection.return_value = 0.0
    stabilizer = mock.Mock()
    # Plain attribute: InterceptGuidance reads ``is_connected`` (a property
    # on the real StabilizerManager) without calling it.
    stabilizer.is_connected = True
    logger = mock.Mock()
    guidance = InterceptGuidance(
        detection_server=detection_server, stabilizer=stabilizer, logger=logger
    )
    return guidance, detection_server, stabilizer, logger


class TestConfigAssumptions:
    """Pin the resolved config values the literals below depend on (the
    container sets none of the INTERCEPT_* environment overrides)."""

    def test_intercept_parameters(self):
        assert INTERCEPT_DEADBAND_X == pytest.approx(0.15)
        assert INTERCEPT_DEADBAND_Y == pytest.approx(0.2)
        assert INTERCEPT_YAW_GAIN == pytest.approx(100.0)
        assert INTERCEPT_ALTITUDE_STEP == pytest.approx(0.01)
        assert INTERCEPT_PITCH_OFFSET == 20
        assert INTERCEPT_ROLL_NEUTRAL == 1500
        assert PWM_NEUTRAL == 1500
        assert INTERCEPT_TIMEOUT_SECONDS == pytest.approx(3.0)


class TestActivation:
    def test_starts_outside_intercept_mode(self, env):
        guidance, _, _, _ = env
        assert guidance.is_intercepting is False

    def test_target_activates_intercept_mode(self, env):
        guidance, _, _, logger = env
        result = guidance.update(
            target=reading(confidence=0.9),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        assert guidance.is_intercepting is True
        assert isinstance(result, AttitudeSetpoints)
        logger.warning.assert_called_once_with(
            "INTERCEPT MODE ACTIVATED (confidence: 90.00%)"
        )

    def test_activation_logs_once_while_mode_stays_active(self, env):
        guidance, _, _, logger = env
        for step in range(3):
            guidance.update(
                target=reading(), current_altitude=5.0, target_altitude=5.0
            )
        assert logger.warning.call_count == 1

    def test_any_reading_handed_in_activates_regardless_of_confidence(self, env):
        """The confidence floor lives in DetectionServer.get_active_target
        (>= INTERCEPT_CONFIDENCE_THRESHOLD, pinned in
        test_detection_server.py); the guidance trusts whatever reading it
        receives -- exactly like the former inline branch did."""
        guidance, _, _, _ = env
        guidance.update(
            target=reading(confidence=0.01),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        assert guidance.is_intercepting is True

    def test_no_target_never_activates(self, env):
        guidance, _, _, _ = env
        assert guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        ) is None
        assert guidance.is_intercepting is False


class TestYawCorrection:
    def _yaw(self, env, dir_x):
        guidance, _, _, _ = env
        result = guidance.update(
            target=reading(dir_x=dir_x),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        return result.yaw_pwm

    @pytest.mark.parametrize(
        "dir_x, expected_yaw",
        [
            # Positive dir_x (target right) -> yaw right of neutral.
            (0.25, 1525),   # int(0.25 * 100) == 25
            (0.5, 1550),
            # Negative dir_x -> yaw left; int() truncates toward zero
            # (int(-15.5...) == -15, NOT floor's -16).
            (-0.155, 1485),
            (-0.5, 1450),
            # Just outside the deadband.
            (0.16, 1516),
            (-0.16, 1484),
        ],
    )
    def test_yaw_outside_deadband(self, env, dir_x, expected_yaw):
        assert self._yaw(env, dir_x) == expected_yaw

    @pytest.mark.parametrize(
        "dir_x",
        [0.0, 0.1, -0.1, 0.15, -0.15],  # boundary is strict >: 0.15 is inside
    )
    def test_yaw_neutral_inside_deadband(self, env, dir_x):
        assert self._yaw(env, dir_x) == PWM_NEUTRAL


class TestAltitudeStepping:
    def _target_altitude(self, env, dir_y, target_altitude=5.0):
        guidance, _, _, _ = env
        result = guidance.update(
            target=reading(dir_y=dir_y),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        return result.target_altitude

    def test_positive_dir_y_steps_target_up(self, env):
        # 5.0 + 0.5 * 0.01
        assert self._target_altitude(env, dir_y=0.5) == pytest.approx(5.005)

    def test_negative_dir_y_steps_target_down(self, env):
        assert self._target_altitude(env, dir_y=-0.5) == pytest.approx(4.995)

    @pytest.mark.parametrize(
        "dir_y",
        [0.0, 0.1, -0.1, 0.2, -0.2],  # boundary is strict >: 0.2 is inside
    )
    def test_inside_deadband_leaves_target_unchanged(self, env, dir_y):
        assert self._target_altitude(env, dir_y=dir_y) == 5.0

    def test_steps_accumulate_across_iterations_via_feedback(self, env):
        """The controller feeds the returned target back in on the next
        loop, so a persistent offset climbs one step per iteration."""
        guidance, _, _, _ = env
        target = 5.0
        seen = []
        for _ in range(3):
            result = guidance.update(
                target=reading(dir_y=0.5),
                current_altitude=5.0,
                target_altitude=target,
            )
            target = result.target_altitude
            seen.append(target)
        assert seen == pytest.approx([5.005, 5.010, 5.015])


class TestPitchAndRoll:
    @pytest.mark.parametrize(
        "dir_x, dir_y",
        [(0.0, 0.0), (0.5, 0.5), (-0.5, -0.5), (0.1, -0.1)],
    )
    def test_pitch_forward_and_roll_neutral_whenever_target_active(
        self, env, dir_x, dir_y
    ):
        """No deadband on pitch/roll: forward pitch offset and neutral roll
        are commanded on EVERY active-target iteration."""
        guidance, _, _, _ = env
        result = guidance.update(
            target=reading(dir_x=dir_x, dir_y=dir_y),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        assert result.pitch_pwm == PWM_NEUTRAL + INTERCEPT_PITCH_OFFSET == 1520
        assert result.roll_pwm == INTERCEPT_ROLL_NEUTRAL == 1500


class TestTimeoutDeactivation:
    def _activate(self, env):
        guidance, _, _, _ = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        assert guidance.is_intercepting

    def test_staleness_is_not_queried_while_mode_inactive(self, env):
        """Short-circuit pin: the former inline ``and`` only consulted
        get_time_since_last_detection() when the mode was active."""
        guidance, detection_server, _, _ = env
        guidance.update(target=None, current_altitude=5.0, target_altitude=5.0)
        detection_server.get_time_since_last_detection.assert_not_called()

    def test_fresh_gap_keeps_mode_active_and_returns_none(self, env):
        """The 'limbo' state: detections paused (or low-confidence ones
        still arriving) but the timeout not yet elapsed -> the mode stays
        active and no setpoints are produced."""
        self._activate(env)
        guidance, detection_server, _, _ = env
        detection_server.get_time_since_last_detection.return_value = 2.999
        assert guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        ) is None
        assert guidance.is_intercepting is True

    @pytest.mark.parametrize("age", [3.0, 4.5, float("inf")])
    def test_timeout_deactivates_with_neutral_attitude_and_altitude_hold(
        self, env, age
    ):
        """>= semantics (exactly AT the timeout deactivates); inf is what
        get_time_since_last_detection returns when nothing was ever
        received."""
        self._activate(env)
        guidance, detection_server, _, logger = env
        detection_server.get_time_since_last_detection.return_value = age
        result = guidance.update(
            target=None, current_altitude=4.2, target_altitude=5.0
        )
        assert guidance.is_intercepting is False
        assert result == AttitudeSetpoints(
            roll_pwm=PWM_NEUTRAL,
            pitch_pwm=PWM_NEUTRAL,
            yaw_pwm=PWM_NEUTRAL,
            target_altitude=4.2,
        )
        logger.warning.assert_called_with(
            "INTERCEPT MODE DEACTIVATED (no detection for 3.0s)"
        )
        logger.info.assert_any_call("Holding altitude at 4.20m")

    def test_deactivation_without_altitude_reading_leaves_target_unchanged(
        self, env
    ):
        self._activate(env)
        guidance, detection_server, _, logger = env
        detection_server.get_time_since_last_detection.return_value = 10.0
        result = guidance.update(
            target=None, current_altitude=None, target_altitude=5.0
        )
        # None target altitude == "leave the controller's target as is".
        assert result.target_altitude is None
        assert not any(
            "Holding altitude" in str(call)
            for call in logger.info.call_args_list
        )

    def test_deactivation_happens_once(self, env):
        self._activate(env)
        guidance, detection_server, _, _ = env
        detection_server.get_time_since_last_detection.return_value = 10.0
        assert guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        ) is not None
        # Mode is now inactive: further no-target updates are inert.
        assert guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        ) is None


class TestStabilizationReenableHandoff:
    def _deactivate(self, env, connected):
        guidance, detection_server, stabilizer, _ = env
        stabilizer.is_connected = connected
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        detection_server.get_time_since_last_detection.return_value = 10.0
        return guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )

    def test_deactivation_restarts_stabilization_when_not_connected(self, env):
        _, _, stabilizer, logger = env
        self._deactivate(env, connected=False)
        stabilizer.start_stabilizer_process.assert_called_once_with()
        logger.info.assert_any_call("Re-enabling stabilization after intercept")

    def test_deactivation_leaves_connected_stabilizer_alone(self, env):
        _, _, stabilizer, logger = env
        self._deactivate(env, connected=True)
        stabilizer.start_stabilizer_process.assert_not_called()
        assert not any(
            "Re-enabling stabilization" in str(call)
            for call in logger.info.call_args_list
        )


class TestExitIntercept:
    def test_exit_while_active_resets_flag_and_logs(self, env):
        guidance, _, _, logger = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        guidance.exit_intercept()
        assert guidance.is_intercepting is False
        logger.info.assert_called_once_with("Exiting intercept mode")

    def test_exit_while_inactive_is_a_no_op(self, env):
        """Mirrors the disarm handler's former ``if self.__intercept_mode``
        guard: no log line when there was nothing to exit."""
        guidance, _, _, logger = env
        guidance.exit_intercept()
        assert guidance.is_intercepting is False
        logger.info.assert_not_called()


class TestScriptedSequence:
    def test_full_intercept_lifecycle(self, env):
        """Activation -> corrections -> loss (limbo) -> timeout deactivation
        with handoff -> reactivation, with the altitude target fed back
        exactly like the controller's loop does."""
        guidance, detection_server, stabilizer, _ = env
        stabilizer.is_connected = False
        target_altitude = 5.0

        # 1) Target appears right of center and above: activate + correct.
        result = guidance.update(
            target=reading(confidence=0.8, dir_x=0.3, dir_y=0.5),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        target_altitude = result.target_altitude
        assert guidance.is_intercepting is True
        assert result.yaw_pwm == 1530          # 1500 + int(0.3 * 100)
        assert result.pitch_pwm == 1520        # forward offset
        assert result.roll_pwm == 1500
        assert target_altitude == pytest.approx(5.005)

        # 2) Target centered within both deadbands: hold attitude/altitude.
        result = guidance.update(
            target=reading(confidence=0.9, dir_x=0.05, dir_y=-0.1),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        target_altitude = result.target_altitude
        assert (result.roll_pwm, result.pitch_pwm, result.yaw_pwm) == (
            1500, 1520, 1500,
        )
        assert target_altitude == pytest.approx(5.005)

        # 3) Detections stop; gap below the timeout keeps the mode active.
        detection_server.get_time_since_last_detection.return_value = 1.0
        assert guidance.update(
            target=None, current_altitude=5.1, target_altitude=target_altitude
        ) is None
        assert guidance.is_intercepting is True
        stabilizer.start_stabilizer_process.assert_not_called()

        # 4) Timeout: neutral attitude, hold current altitude, handoff.
        detection_server.get_time_since_last_detection.return_value = 3.0
        result = guidance.update(
            target=None, current_altitude=5.1, target_altitude=target_altitude
        )
        if result.target_altitude is not None:
            target_altitude = result.target_altitude
        assert guidance.is_intercepting is False
        assert (result.roll_pwm, result.pitch_pwm, result.yaw_pwm) == (
            1500, 1500, 1500,
        )
        assert target_altitude == pytest.approx(5.1)
        stabilizer.start_stabilizer_process.assert_called_once_with()

        # 5) A new target reactivates from scratch.
        result = guidance.update(
            target=reading(confidence=0.7, dir_x=-0.2, dir_y=0.0),
            current_altitude=5.1,
            target_altitude=target_altitude,
        )
        assert guidance.is_intercepting is True
        assert result.yaw_pwm == 1480          # 1500 + int(-0.2 * 100)
