"""Intercept guidance: the target-interception state machine.

GRASP Step 5 (REFACTORING_PLAN.md, HC-1/IE-1): this state machine -- the
``__intercept_mode`` flag, activation on an active target, deadband/yaw/
altitude-step corrections, and timeout deactivation with the stabilization
re-enable handoff -- used to live inline in
``DroneController.__updateThrottle``.  All numeric logic (deadbands, yaw
gain, altitude step, mode enter/exit conditions, log messages) is moved
VERBATIM from the controller; only the mechanism changed: instead of
mutating the controller's RC base attributes in place, :meth:`update`
returns :class:`AttitudeSetpoints` intents which the controller applies.

Division of labor (unchanged from Step 4):

- ``DetectionServer.get_active_target()`` owns data validity (staleness,
  confidence floor, direction-vector extraction) and hands the controller a
  typed :class:`DetectionReading` -- the controller passes it through here.
- ``InterceptGuidance`` owns the intercept state machine itself.
- The detection-server *running* check and the RC base/altitude-PID plumbing
  stay in the controller (RCSetpoints is Step 6, composition root Step 7).

Collaborators are constructor-injected (no import of controller.py):

- ``detection_server``: queried only for ``get_time_since_last_detection()``
  inside the timeout-deactivation condition, exactly where the controller
  used to call it (and, thanks to ``and`` short-circuiting, only when the
  intercept mode is actually active -- preserved).
- ``stabilizer``: the StabilizerManager; deactivation re-enables
  stabilization through it (``is_connected`` check +
  ``start_stabilizer_process()``, the body of the controller's
  ``_stabilize`` command).
"""

from typing import Optional

from src.detection_config import (
    INTERCEPT_TIMEOUT_SECONDS,
    INTERCEPT_DEADBAND_X,
    INTERCEPT_DEADBAND_Y,
    INTERCEPT_YAW_GAIN,
    INTERCEPT_ALTITUDE_STEP,
    INTERCEPT_PITCH_OFFSET,
    INTERCEPT_ROLL_NEUTRAL,
    PWM_NEUTRAL,
)
from src.domain.types import AttitudeSetpoints, DetectionReading


class InterceptGuidance:
    """State machine that converts active target detections into attitude
    and altitude intents, owning the intercept mode flag."""

    def __init__(self, detection_server, stabilizer, logger):
        """
        Args:
            detection_server: DetectionServer; only
                ``get_time_since_last_detection()`` is consulted (for the
                timeout-deactivation condition).
            stabilizer: StabilizerManager; deactivation re-enables
                stabilization through it.
            logger: the controller's logger (intercept log lines keep going
                to logs/controller.log exactly as before).
        """
        self.__detection_server = detection_server
        self.__stabilizer = stabilizer
        self.__logger = logger

        # Intercept mode state (formerly DroneController.__intercept_mode).
        self.__intercept_mode = False
        # Recorded at activation; not consumed anywhere yet (kept verbatim
        # from the controller, where it was likewise write-only state).
        self.__intercept_start_altitude: Optional[float] = None

    @property
    def is_intercepting(self) -> bool:
        """True while the intercept mode is active.

        Note the historic stickiness is preserved: if the detection server
        is stopped mid-intercept (``monitor,stop``), update() is no longer
        consulted and the flag stays True until ``exit_intercept()`` (the
        disarm path) or a later reactivation/timeout flips it.
        """
        return self.__intercept_mode

    def exit_intercept(self) -> None:
        """Force-exit intercept mode (the disarm path).

        Verbatim move of the disarm handler's block: log + flag reset only
        happen when the mode was actually active.
        """
        if self.__intercept_mode:
            self.__logger.info("Exiting intercept mode")
            self.__intercept_mode = False

    def update(
        self,
        target: Optional[DetectionReading],
        current_altitude: Optional[float],
        target_altitude: float,
    ) -> Optional[AttitudeSetpoints]:
        """Advance the state machine for one control-loop iteration.

        Args:
            target: the active target from
                ``DetectionServer.get_active_target()``, or None.
            current_altitude: latest altitude reading (may be None).
            target_altitude: the controller's current altitude target.

        Returns:
            AttitudeSetpoints to apply, or None when nothing changes:

            - target present -> intercept corrections (activating the mode
              first if needed); ``is_intercepting`` is True afterwards and
              the controller must skip stabilization for this iteration.
            - no target, mode active and detection timeout elapsed ->
              deactivation: neutral attitude, altitude hold at the current
              reading, stabilization re-enable handoff.
            - otherwise -> None.
        """
        if target is not None:
            return self.__intercept(target, current_altitude, target_altitude)
        return self.__maybe_deactivate(current_altitude)

    def __intercept(
        self,
        target: DetectionReading,
        current_altitude: Optional[float],
        target_altitude: float,
    ) -> AttitudeSetpoints:
        """The active-target branch, moved verbatim from __updateThrottle."""
        # High confidence detection - enter intercept mode
        if not self.__intercept_mode:
            self.__logger.warning(f"INTERCEPT MODE ACTIVATED (confidence: {target.confidence:.2%})")
            self.__intercept_mode = True
            self.__intercept_start_altitude = current_altitude
            # Disable stabilization during intercept
            # (stabilizer keeps running but we ignore its output)

        dir_x = target.dir_x  # Horizontal (-0.5 to 0.5)
        dir_y = target.dir_y  # Vertical (-0.5 to 0.5)

        # YAW control (centering horizontally)
        if abs(dir_x) > INTERCEPT_DEADBAND_X:
            # Positive dir_x = target is right, need to yaw right
            yaw_correction = int(dir_x * INTERCEPT_YAW_GAIN)
            yaw_base = PWM_NEUTRAL + yaw_correction
        else:
            yaw_base = PWM_NEUTRAL  # Centered

        # ALTITUDE control (centering vertically)
        new_target_altitude = target_altitude
        if abs(dir_y) > INTERCEPT_DEADBAND_Y:
            # Positive dir_y = target is above, need to climb
            altitude_correction = dir_y * INTERCEPT_ALTITUDE_STEP
            new_target_altitude += altitude_correction
            self.__logger.debug(f"Altitude adjustment: {altitude_correction:+.2f}m -> {new_target_altitude:.2f}m")

        # PITCH control (move forward toward target)
        pitch_base = PWM_NEUTRAL + INTERCEPT_PITCH_OFFSET

        # ROLL neutral (no lateral movement)
        roll_base = INTERCEPT_ROLL_NEUTRAL

        self.__logger.debug(
            f"Intercept: conf={target.confidence:.2%}, dir_x={dir_x:+.3f}, dir_y={dir_y:+.3f}, "
            f"yaw={yaw_base}, pitch={pitch_base}"
        )

        return AttitudeSetpoints(
            roll_pwm=roll_base,
            pitch_pwm=pitch_base,
            yaw_pwm=yaw_base,
            target_altitude=new_target_altitude,
        )

    def __maybe_deactivate(
        self, current_altitude: Optional[float]
    ) -> Optional[AttitudeSetpoints]:
        """The no-target branch, moved verbatim from __updateThrottle.

        Note the short-circuit: ``get_time_since_last_detection()`` is only
        queried while the intercept mode is active, exactly as before.  A
        fresh-but-low-confidence stream therefore keeps the mode active
        ("limbo") without producing setpoints -- also as before.
        """
        # No recent detection or low confidence
        if (self.__intercept_mode
                and self.__detection_server.get_time_since_last_detection()
                >= INTERCEPT_TIMEOUT_SECONDS):
            self.__logger.warning(
                f"INTERCEPT MODE DEACTIVATED (no detection for {INTERCEPT_TIMEOUT_SECONDS}s)"
            )
            self.__intercept_mode = False
            # Fix altitude at current position
            new_target_altitude = None
            if current_altitude is not None:
                new_target_altitude = current_altitude
                self.__logger.info(f"Holding altitude at {current_altitude:.2f}m")
            # Re-enable stabilization
            if not self.__stabilizer.is_connected:
                self.__logger.info("Re-enabling stabilization after intercept")
                self.__stabilizer.start_stabilizer_process()
            # Stop forward movement (neutral attitude; None target altitude
            # means "leave the controller's target unchanged")
            return AttitudeSetpoints(
                roll_pwm=PWM_NEUTRAL,
                pitch_pwm=PWM_NEUTRAL,
                yaw_pwm=PWM_NEUTRAL,
                target_altitude=new_target_altitude,
            )
        return None
