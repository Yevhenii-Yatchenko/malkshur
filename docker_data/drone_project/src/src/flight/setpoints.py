"""RC setpoint state: the single owner of the RC PWM bases and their limits.

GRASP Step 6 (REFACTORING_PLAN.md, IE-4): ``DroneController`` used to keep
the four RC PWM bases and the altitude target as five loose attributes, with
a ``__set_throttle_base`` helper whose hardcoded ``1800`` silently duplicated
``THROTTLE['max']`` from ``src/altitude_config.py``.  :class:`RCSetpoints`
replaces the attributes and the helper; the throttle ceiling is sourced from
the config, so the limit has exactly one home.

What is (and is NOT) clamped here -- replicating exactly what the old
controller enforced, where it enforced it:

- ``throttle``: upper clamp at ``THROTTLE['max']`` (the former
  ``value >= 1800 -> 1800``).  There never was a controller-level lower
  clamp: ``THROTTLE['min']`` is enforced inside ``AltitudeController.update``
  (np.clip on its own output before shaping), and that stays the owner of
  the lower bound.
- ``roll`` / ``pitch`` / ``yaw``: NO clamping, exactly as before.  The
  PWM_LIMITS window is owned by ``PositionController`` (np.clip on its own
  outputs); the intercept corrections are config-derived offsets that were
  never clamped at the controller level.  Adding a clamp here would
  duplicate the existing owner -- the IE-4 violation in the other direction.

All four PWM setters normalize to built-in ``int``.  This is a no-op for
every live writer: ``AltitudeController.update`` returns ``int(throttle)``,
the intercept setpoints are Python ints, telnet ``move`` params arrive as
ints from ``CommandHandler.parse_message``, and ``PositionController``'s
numpy ints pack into the MAVLink uint16 RC channels byte-identically whether
or not they pass through ``int()`` (pymavlink packs via ``struct``, which
consumes numpy ints through ``__index__``).  The normalization simply makes
the ``int`` annotations on :class:`AttitudeSetpoints` true by the time the
values land here.  (Degenerate input only: a non-numeric ``move`` param now
fails at set time, inside the command handler's try/except, instead of
poisoning the RC state and failing on every subsequent MAVLink send.)

``target_altitude`` is plain float state (no normalization, no limits --
none existed before).
"""

from src.altitude_config import THROTTLE
from src.domain.types import AttitudeSetpoints


class RCSetpoints:
    """Owns the roll/pitch/yaw/throttle PWM bases plus the altitude target."""

    def __init__(self, throttle_max: int = THROTTLE['max']):
        """
        Args:
            throttle_max: Throttle ceiling PWM (GRASP Step 7 carried bullet:
                injectable so the composition root passes it explicitly;
                defaults to ``THROTTLE['max']`` from altitude_config exactly
                as the Step 6 inline read did).
        """
        self.__throttle_max = throttle_max
        # Initial values moved verbatim from the former DroneController
        # class attributes (neutral attitude, idle throttle, ground-hug
        # altitude target).
        self.__roll = 1500
        self.__pitch = 1500
        self.__yaw = 1500
        self.__throttle = 1000
        self.__target_altitude = 0.2

    @property
    def roll(self) -> int:
        return self.__roll

    @roll.setter
    def roll(self, value) -> None:
        self.__roll = int(value)

    @property
    def pitch(self) -> int:
        return self.__pitch

    @pitch.setter
    def pitch(self, value) -> None:
        self.__pitch = int(value)

    @property
    def yaw(self) -> int:
        return self.__yaw

    @yaw.setter
    def yaw(self, value) -> None:
        self.__yaw = int(value)

    @property
    def throttle(self) -> int:
        return self.__throttle

    @throttle.setter
    def throttle(self, value) -> None:
        """The former ``DroneController.__set_throttle_base``, with the
        hardcoded ``1800`` replaced by its config source (IE-4; a
        constructor parameter since Step 7).  Upper clamp only -- see the
        module docstring for why there is no lower one here."""
        if value >= self.__throttle_max:
            value = self.__throttle_max
        self.__throttle = int(value)

    @property
    def target_altitude(self) -> float:
        return self.__target_altitude

    @target_altitude.setter
    def target_altitude(self, value: float) -> None:
        self.__target_altitude = value

    def apply(self, setpoints: AttitudeSetpoints) -> None:
        """Apply a flight behavior's attitude intents (the former
        ``DroneController.__apply_setpoints``): roll/pitch/yaw always,
        ``target_altitude`` only when the behavior set one (``None`` means
        "leave the current target unchanged")."""
        self.roll = setpoints.roll_pwm
        self.pitch = setpoints.pitch_pwm
        self.yaw = setpoints.yaw_pwm
        if setpoints.target_altitude is not None:
            self.target_altitude = setpoints.target_altitude
