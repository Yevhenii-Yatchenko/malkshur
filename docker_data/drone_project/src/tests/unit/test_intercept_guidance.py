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
  inline ``and``); deactivation returns neutral attitude and holds altitude
  at the current reading (or leaves the target unchanged when there is no
  reading);
- exit_intercept(): the disarm path, log + flag reset only when active.

Step 6 (review absorption): update() returns a frozen InterceptResult
instead of Optional[AttitudeSetpoints]:

- ``active`` is the controller's skip-stabilization gate -- True exactly
  when a target drove corrections this iteration (the former
  ``target is not None`` re-derivation), False on the deactivation
  iteration and in limbo even though ``is_intercepting`` may stay True;
- ``reenable_stabilization`` replaces the in-update side effect: the
  guidance no longer holds a StabilizerManager at all; the deactivation
  iteration raises the intent and DroneController.__updateThrottle performs
  the actual ``if not stabilizer.is_connected: start_stabilizer_process()``
  synchronously right after update() (so the is_connected check still
  happens at deactivation time).

Collaborators (detection server for the staleness query, the controller's
logger) are constructor-injected, so the tests drive them as mocks; there
is no controller import anywhere.
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
from src.flight.intercept import InterceptGuidance, InterceptResult

pytestmark = [pytest.mark.unit]


def reading(confidence=0.9, dir_x=0.0, dir_y=0.0):
    return DetectionReading(confidence=confidence, dir_x=dir_x, dir_y=dir_y)


@pytest.fixture
def env():
    detection_server = mock.Mock()
    detection_server.get_time_since_last_detection.return_value = 0.0
    logger = mock.Mock()
    guidance = InterceptGuidance(
        detection_server=detection_server, logger=logger
    )
    return guidance, detection_server, logger


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
        guidance, _, _ = env
        assert guidance.is_intercepting is False

    def test_target_activates_intercept_mode(self, env):
        guidance, _, logger = env
        result = guidance.update(
            target=reading(confidence=0.9),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        assert guidance.is_intercepting is True
        assert isinstance(result, InterceptResult)
        assert isinstance(result.setpoints, AttitudeSetpoints)
        assert result.active is True
        assert result.reenable_stabilization is False
        logger.warning.assert_called_once_with(
            "INTERCEPT MODE ACTIVATED (confidence: 90.00%)"
        )

    def test_activation_logs_once_while_mode_stays_active(self, env):
        guidance, _, logger = env
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
        guidance, _, _ = env
        guidance.update(
            target=reading(confidence=0.01),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        assert guidance.is_intercepting is True

    def test_no_target_never_activates(self, env):
        guidance, _, _ = env
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result == InterceptResult(
            setpoints=None, active=False, reenable_stabilization=False
        )
        assert guidance.is_intercepting is False


class TestActiveGate:
    """``active`` is the controller's skip-stabilization gate: it must equal
    the former ``target is not None`` condition, NOT the mode flag."""

    def test_active_true_on_every_target_iteration(self, env):
        guidance, _, _ = env
        for _ in range(3):
            result = guidance.update(
                target=reading(), current_altitude=5.0, target_altitude=5.0
            )
            assert result.active is True

    def test_limbo_is_not_active_even_though_mode_flag_stays_set(self, env):
        """Fresh-gap limbo: the old controller fell through to the
        stabilization branch here (its gate was on the target, not the
        mode), so active must be False while is_intercepting stays True."""
        guidance, detection_server, _ = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        detection_server.get_time_since_last_detection.return_value = 1.0
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result.active is False
        assert guidance.is_intercepting is True

    def test_deactivation_iteration_is_not_active_despite_setpoints(self, env):
        """The deactivation iteration produces setpoints but the old
        controller still ran its normal stabilization pass afterwards --
        active must be False even though setpoints is not None (the two are
        deliberately independent fields)."""
        guidance, detection_server, _ = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        detection_server.get_time_since_last_detection.return_value = 10.0
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result.setpoints is not None
        assert result.active is False


class TestYawCorrection:
    def _yaw(self, env, dir_x):
        guidance, _, _ = env
        result = guidance.update(
            target=reading(dir_x=dir_x),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        return result.setpoints.yaw_pwm

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
        guidance, _, _ = env
        result = guidance.update(
            target=reading(dir_y=dir_y),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        return result.setpoints.target_altitude

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
        guidance, _, _ = env
        target = 5.0
        seen = []
        for _ in range(3):
            result = guidance.update(
                target=reading(dir_y=0.5),
                current_altitude=5.0,
                target_altitude=target,
            )
            target = result.setpoints.target_altitude
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
        guidance, _, _ = env
        result = guidance.update(
            target=reading(dir_x=dir_x, dir_y=dir_y),
            current_altitude=5.0,
            target_altitude=5.0,
        )
        setpoints = result.setpoints
        assert setpoints.pitch_pwm == PWM_NEUTRAL + INTERCEPT_PITCH_OFFSET == 1520
        assert setpoints.roll_pwm == INTERCEPT_ROLL_NEUTRAL == 1500


class TestTimeoutDeactivation:
    def _activate(self, env):
        guidance, _, _ = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        assert guidance.is_intercepting

    def test_staleness_is_not_queried_while_mode_inactive(self, env):
        """Short-circuit pin: the former inline ``and`` only consulted
        get_time_since_last_detection() when the mode was active."""
        guidance, detection_server, _ = env
        guidance.update(target=None, current_altitude=5.0, target_altitude=5.0)
        detection_server.get_time_since_last_detection.assert_not_called()

    def test_fresh_gap_keeps_mode_active_and_returns_idle_result(self, env):
        """The 'limbo' state: detections paused (or low-confidence ones
        still arriving) but the timeout not yet elapsed -> the mode stays
        active and no setpoints/intents are produced."""
        self._activate(env)
        guidance, detection_server, _ = env
        detection_server.get_time_since_last_detection.return_value = 2.999
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result == InterceptResult(
            setpoints=None, active=False, reenable_stabilization=False
        )
        assert guidance.is_intercepting is True

    @pytest.mark.parametrize("age", [3.0, 4.5, float("inf")])
    def test_timeout_deactivates_with_neutral_attitude_and_altitude_hold(
        self, env, age
    ):
        """>= semantics (exactly AT the timeout deactivates); inf is what
        get_time_since_last_detection returns when nothing was ever
        received."""
        self._activate(env)
        guidance, detection_server, logger = env
        detection_server.get_time_since_last_detection.return_value = age
        result = guidance.update(
            target=None, current_altitude=4.2, target_altitude=5.0
        )
        assert guidance.is_intercepting is False
        assert result == InterceptResult(
            setpoints=AttitudeSetpoints(
                roll_pwm=PWM_NEUTRAL,
                pitch_pwm=PWM_NEUTRAL,
                yaw_pwm=PWM_NEUTRAL,
                target_altitude=4.2,
            ),
            active=False,
            reenable_stabilization=True,
        )
        logger.warning.assert_called_with(
            "INTERCEPT MODE DEACTIVATED (no detection for 3.0s)"
        )
        logger.info.assert_any_call("Holding altitude at 4.20m")

    def test_deactivation_without_altitude_reading_leaves_target_unchanged(
        self, env
    ):
        self._activate(env)
        guidance, detection_server, logger = env
        detection_server.get_time_since_last_detection.return_value = 10.0
        result = guidance.update(
            target=None, current_altitude=None, target_altitude=5.0
        )
        # None target altitude == "leave the controller's target as is".
        assert result.setpoints.target_altitude is None
        assert result.reenable_stabilization is True
        assert not any(
            "Holding altitude" in str(call)
            for call in logger.info.call_args_list
        )

    def test_deactivation_happens_once(self, env):
        self._activate(env)
        guidance, detection_server, _ = env
        detection_server.get_time_since_last_detection.return_value = 10.0
        first = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert first.setpoints is not None
        assert first.reenable_stabilization is True
        # Mode is now inactive: further no-target updates are inert.
        second = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert second == InterceptResult(
            setpoints=None, active=False, reenable_stabilization=False
        )


class TestReenableStabilizationIntent:
    """Step 6: the re-enable handoff is an intent, not a side effect.  The
    guidance no longer touches a StabilizerManager; the actual
    ``if not is_connected: start_stabilizer_process()`` (and its log line)
    lives in DroneController.__updateThrottle, executed synchronously right
    after update() on the same iteration."""

    def test_only_the_deactivation_iteration_raises_the_intent(self, env):
        guidance, detection_server, _ = env
        # Activation iteration: no intent.
        result = guidance.update(
            target=reading(), current_altitude=5.0, target_altitude=5.0
        )
        assert result.reenable_stabilization is False
        # Limbo iteration: no intent.
        detection_server.get_time_since_last_detection.return_value = 1.0
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result.reenable_stabilization is False
        # Deactivation iteration: intent raised exactly once.
        detection_server.get_time_since_last_detection.return_value = 10.0
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result.reenable_stabilization is True
        # Inert afterwards.
        result = guidance.update(
            target=None, current_altitude=5.0, target_altitude=5.0
        )
        assert result.reenable_stabilization is False

    def test_result_is_immutable(self, env):
        """Frozen value object: the controller cannot mutate the intents."""
        guidance, _, _ = env
        result = guidance.update(
            target=reading(), current_altitude=5.0, target_altitude=5.0
        )
        with pytest.raises(Exception):
            result.active = False


class TestExitIntercept:
    def test_exit_while_active_resets_flag_and_logs(self, env):
        guidance, _, logger = env
        guidance.update(target=reading(), current_altitude=5.0, target_altitude=5.0)
        guidance.exit_intercept()
        assert guidance.is_intercepting is False
        logger.info.assert_called_once_with("Exiting intercept mode")

    def test_exit_while_inactive_is_a_no_op(self, env):
        """Mirrors the disarm handler's former ``if self.__intercept_mode``
        guard: no log line when there was nothing to exit."""
        guidance, _, logger = env
        guidance.exit_intercept()
        assert guidance.is_intercepting is False
        logger.info.assert_not_called()


class TestScriptedSequence:
    def test_full_intercept_lifecycle(self, env):
        """Activation -> corrections -> loss (limbo) -> timeout deactivation
        with the re-enable intent -> reactivation, with the altitude target
        fed back exactly like the controller's loop does."""
        guidance, detection_server, _ = env
        target_altitude = 5.0

        # 1) Target appears right of center and above: activate + correct.
        result = guidance.update(
            target=reading(confidence=0.8, dir_x=0.3, dir_y=0.5),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        setpoints = result.setpoints
        target_altitude = setpoints.target_altitude
        assert guidance.is_intercepting is True
        assert result.active is True
        assert setpoints.yaw_pwm == 1530       # 1500 + int(0.3 * 100)
        assert setpoints.pitch_pwm == 1520     # forward offset
        assert setpoints.roll_pwm == 1500
        assert target_altitude == pytest.approx(5.005)

        # 2) Target centered within both deadbands: hold attitude/altitude.
        result = guidance.update(
            target=reading(confidence=0.9, dir_x=0.05, dir_y=-0.1),
            current_altitude=5.0,
            target_altitude=target_altitude,
        )
        setpoints = result.setpoints
        target_altitude = setpoints.target_altitude
        assert (setpoints.roll_pwm, setpoints.pitch_pwm, setpoints.yaw_pwm) == (
            1500, 1520, 1500,
        )
        assert target_altitude == pytest.approx(5.005)

        # 3) Detections stop; gap below the timeout keeps the mode active.
        detection_server.get_time_since_last_detection.return_value = 1.0
        result = guidance.update(
            target=None, current_altitude=5.1, target_altitude=target_altitude
        )
        assert result == InterceptResult(
            setpoints=None, active=False, reenable_stabilization=False
        )
        assert guidance.is_intercepting is True

        # 4) Timeout: neutral attitude, hold current altitude, intent up.
        detection_server.get_time_since_last_detection.return_value = 3.0
        result = guidance.update(
            target=None, current_altitude=5.1, target_altitude=target_altitude
        )
        setpoints = result.setpoints
        if setpoints.target_altitude is not None:
            target_altitude = setpoints.target_altitude
        assert guidance.is_intercepting is False
        assert result.active is False
        assert result.reenable_stabilization is True
        assert (setpoints.roll_pwm, setpoints.pitch_pwm, setpoints.yaw_pwm) == (
            1500, 1500, 1500,
        )
        assert target_altitude == pytest.approx(5.1)

        # 5) A new target reactivates from scratch.
        result = guidance.update(
            target=reading(confidence=0.7, dir_x=-0.2, dir_y=0.0),
            current_altitude=5.1,
            target_altitude=target_altitude,
        )
        assert guidance.is_intercepting is True
        assert result.active is True
        assert result.setpoints.yaw_pwm == 1480  # 1500 + int(-0.2 * 100)
