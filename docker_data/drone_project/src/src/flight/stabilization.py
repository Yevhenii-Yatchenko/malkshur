"""Stabilization behavior: vision-based XY hold through PositionController.

GRASP Step 5 (REFACTORING_PLAN.md, HC-1): this is the stabilization branch
of the former ``DroneController.__updateThrottle``, moved verbatim:

- while the stabilizer is connected (and intercept is not active), each
  fresh :class:`StabilizerReading` from ``StabilizerManager.poll_new()`` is
  fed to the (injected) PositionController and the resulting roll/pitch/yaw
  PWM values are returned as :class:`AttitudeSetpoints` intents (previously
  assigned straight onto the controller's RC bases);
- otherwise the auto-trigger applies: once the drone is within 0.1 m of its
  target altitude, ``start_stabilizer_process()`` is invoked -- the body of
  the controller's ``_stabilize`` command.  The trigger fires on EVERY
  iteration while its condition holds; the once-only semantics live in
  ``StabilizerManager.__process_started`` (repeat calls log a warning and
  return), exactly as before.

Historic quirk preserved on purpose: the auto-trigger branch is the plain
``else`` of the connected-and-not-intercepting gate, so it is also reached
while the stabilizer IS connected but an intercept is in progress.

The behavior owns no numeric state of its own; collaborators are
constructor-injected (no import of controller.py).
"""

from typing import Optional

from src.domain.types import AttitudeSetpoints


class StabilizationBehavior:
    """Consumes stabilizer readings and returns attitude intents, plus the
    "enable stabilization once target altitude is reached" auto-trigger."""

    def __init__(self, stabilizer, position_controller):
        """
        Args:
            stabilizer: StabilizerManager (connection state, poll_new(),
                start_stabilizer_process()).
            position_controller: PositionController instance whose update()
                turns pixel drift into roll/pitch/yaw PWM.
        """
        self.__stabilizer = stabilizer
        self.__position_controller = position_controller

    def update(
        self,
        current_altitude: Optional[float],
        target_altitude: float,
        intercept_active: bool = False,
    ) -> Optional[AttitudeSetpoints]:
        """Run one stabilization iteration.

        Args:
            current_altitude: latest altitude reading (may be None).
            target_altitude: the controller's current altitude target.
            intercept_active: True while InterceptGuidance holds the
                intercept mode (the former ``not self.__intercept_mode``
                side of the gate); suppresses position corrections.

        Returns:
            AttitudeSetpoints with the PositionController's PWM output when
            a fresh reading was processed, otherwise None (target altitude
            is never touched by stabilization).
        """
        if self.__stabilizer.is_connected and not intercept_active:
            # poll_new() returns each reading at most once (timestamp
            # de-duplication is owned by the StabilizerManager).
            reading = self.__stabilizer.poll_new()
            if reading:
                pid_output = self.__position_controller.update(
                    dx_pixels=reading.dx,
                    dy_pixels=reading.dy,
                    angle_deg=0,
                    confidence=reading.confidence,
                    altitude=current_altitude,
                    target_dx_pixels=reading.target_dx_pixels,
                    target_dy_pixels=reading.target_dy_pixels,
                    navigation=reading.navigation,
                )

                return AttitudeSetpoints(
                    roll_pwm=pid_output['roll_pwm'],
                    pitch_pwm=pid_output['pitch_pwm'],
                    yaw_pwm=pid_output['yaw_pwm'],
                )
        else:
            if current_altitude is not None and abs(current_altitude - target_altitude) < 0.1:
                self.__stabilizer.start_stabilizer_process()
        return None
