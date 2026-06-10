"""Frozen config objects for the values the controllers consume at construction.

GRASP Step 7 (REFACTORING_PLAN.md, LC-2): ``AltitudeController`` used to bind
its tuning as default arguments evaluated at import time
(``alt_kp=ALTITUDE_PID_TAKEOFF['kp']`` and friends), and
``PositionController`` both read the gain dicts inline in ``__init__`` and
reached back into the global ``POSITION_PID_X/Y`` dicts from
``__enable_stabilization`` to restore coefficients.  These dataclasses make
that configuration an explicit, injectable value: the composition root
(``src/app.py``) is the one place where config enters the object graph, and
the controllers fall back to the same module dicts when nothing is injected
(so direct construction -- tests, characterization -- behaves exactly as
before).

Scope (deliberately minimal, no behavior change):

- The dataclasses carry exactly what the controllers consume while WIRING
  THEMSELVES UP (PID gains, PID output bounds, throttle/velocity limits,
  filter parameters).
- The hot-loop reads stay untouched: ``AltitudeController.update`` keeps
  reading ``THROTTLE['rate_limit']`` / ``CONTROL`` / ``DEBUG`` and
  ``PositionController.update`` keeps reading ``COORDINATE_SYSTEM`` /
  ``POSITION_CONTROL`` / ``PWM_LIMITS`` / ``ALTITUDE_COMPENSATION`` from the
  config modules, including the historic ``'key' in dict`` presence checks.
  Migrating those is not needed for LC-2 and would risk drift.

Values are imported 1:1 from the existing config modules -- the ``from_dicts``
factories read the dicts, no number is ever copied by hand, so tuning stays
byte-identical and ``altitude_config.py`` / ``position_config.py`` remain the
single place to tune.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from src.altitude_config import (
    ALTITUDE_PID_TAKEOFF,
    FILTERING,
    LIMITS,
    THROTTLE,
    VELOCITY_PID_FLIGHT,
)
from src.position_config import (
    ANGLE_PID,
    POSITION_PID_X,
    POSITION_PID_Y,
    PWM_LIMITS,
)


@dataclass(frozen=True)
class PIDGains:
    """One kp/ki/kd triple, the shape every gain dict in the project has."""

    kp: float
    ki: float
    kd: float

    @classmethod
    def from_dict(cls, gains: Mapping[str, float]) -> "PIDGains":
        """Build from a ``{'kp': ..., 'ki': ..., 'kd': ...}`` config dict.

        Values pass through unconverted (no ``float()`` coercion), so the
        controllers receive exactly the objects the dicts hold.
        """
        return cls(kp=gains["kp"], ki=gains["ki"], kd=gains["kd"])


@dataclass(frozen=True)
class AltitudeConfig:
    """Everything ``AltitudeController.__init__`` consumes.

    Mirrors the controller's former parameter list one-to-one (same groups,
    same dict keys); see ``from_dicts`` for the dict each field comes from.
    """

    altitude_pid: PIDGains      # ALTITUDE_PID_TAKEOFF (outer/position loop)
    velocity_pid: PIDGains      # VELOCITY_PID_FLIGHT (inner loop)
    max_velocity: float         # LIMITS['max_velocity']
    max_acceleration: float     # LIMITS['max_acceleration']
    throttle_hover: int         # THROTTLE['hover']
    throttle_min: int           # THROTTLE['min']
    throttle_max: int           # THROTTLE['max']
    altitude_filter_alpha: float  # FILTERING['altitude_filter_alpha']
    velocity_filter_size: int   # FILTERING['velocity_filter_size']

    @classmethod
    def from_dicts(cls) -> "AltitudeConfig":
        """The production configuration, read 1:1 from altitude_config.py."""
        return cls(
            altitude_pid=PIDGains.from_dict(ALTITUDE_PID_TAKEOFF),
            velocity_pid=PIDGains.from_dict(VELOCITY_PID_FLIGHT),
            max_velocity=LIMITS["max_velocity"],
            max_acceleration=LIMITS["max_acceleration"],
            throttle_hover=THROTTLE["hover"],
            throttle_min=THROTTLE["min"],
            throttle_max=THROTTLE["max"],
            altitude_filter_alpha=FILTERING["altitude_filter_alpha"],
            velocity_filter_size=FILTERING["velocity_filter_size"],
        )


@dataclass(frozen=True)
class PositionConfig:
    """The PID wiring ``PositionController`` consumes.

    Carries the three gain triples plus the shared PID output bound.  This
    is exactly the set LC-2 flagged: the ``__init__`` gain reads and the
    ``__enable_stabilization`` restore that used to reach back into the
    global ``POSITION_PID_X/Y`` dicts (it restores from this object now).
    """

    pid_x: PIDGains             # POSITION_PID_X
    pid_y: PIDGains             # POSITION_PID_Y
    angle_pid: PIDGains         # ANGLE_PID (yaw)
    max_correction: float       # PWM_LIMITS['max_correction'] (PID output bound)

    @classmethod
    def from_dicts(cls) -> "PositionConfig":
        """The production configuration, read 1:1 from position_config.py."""
        return cls(
            pid_x=PIDGains.from_dict(POSITION_PID_X),
            pid_y=PIDGains.from_dict(POSITION_PID_Y),
            angle_pid=PIDGains.from_dict(ANGLE_PID),
            max_correction=PWM_LIMITS["max_correction"],
        )
