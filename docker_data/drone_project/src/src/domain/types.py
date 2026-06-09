"""Typed readings for the data crossing process/network boundaries.

GRASP Step 4 (REFACTORING_PLAN.md, IE-2/IE-3): the JSON payloads from the
sky_anchor vision process and the detection client used to travel through
the system as raw dicts, with every consumer fishing keys out by hand.
These frozen dataclasses are the single place that knows the wire formats.

Both ends of each wire live in this repository:

- Stabilizer wire (TCP :8888): producer is
  ``sky_anchor/app/vision/evaluator.py`` (``ShiftCommand.to_payload()``)
  plus ``sky_anchor/app/command_publisher.py`` (adds ``timestamp``);
  consumer is ``src/sky_anchor_client.py`` which parses each JSON line
  into a :class:`StabilizerReading`.
- Detection wire: producer is the dd_shahed detection client
  (``dd_shahed/src/utils.cpp`` builds the JSON); consumer is
  ``src/detection_server.py`` which exposes :class:`DetectionReading`
  through ``get_active_target()``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class StabilizerReading:
    """One sky_anchor measurement as received over TCP :8888.

    Wire payload fields (see the producer modules referenced above):

    - ``dx`` / ``dy``: drift in pixels (during navigation: injected
      virtual drift from CommandModifier).
    - ``angle_deg``: rotation drift in degrees.
    - ``matches_percent``: ORB feature-match percentage (0-100 on normal
      frames).  While navigating the producer keeps emitting the historic
      ``101.0`` so the numeric stream (CSV logs, plots) is unchanged, but
      since Step 4 the value is informational only -- mode switching is
      driven exclusively by :attr:`navigation`.
    - ``timestamp``: producer-side ``time.time()`` added by
      CommandPublisher; used by StabilizerManager for de-duplication.
    - ``navigation``: explicit flag, ``True`` while the producer's
      CommandModifier has an active waypoint target.  Replaces the magic
      ``confidence == 1.01`` sentinel (IE-3).  Defaults to ``False`` so
      payloads from producers that predate the field still parse.
    - ``target_dx_pixels`` / ``target_dy_pixels``: active navigation
      target, only present in the payload while a target is set.
    """

    dx: float
    dy: float
    angle_deg: float
    matches_percent: float
    timestamp: float
    navigation: bool = False
    target_dx_pixels: Optional[float] = None
    target_dy_pixels: Optional[float] = None

    @property
    def confidence(self) -> float:
        """``matches_percent`` mapped onto the 0-1 range PositionController uses.

        This is the ``matches_percent / 100.0`` mapping that previously
        lived inline in ``DroneController.__updateThrottle``.  Note that
        the navigation placeholder maps exactly: 101.0 / 100.0 == 1.01 in
        IEEE-754 doubles -- but nothing compares against that value anymore.
        """
        return self.matches_percent / 100.0

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "StabilizerReading":
        """Parse one decoded JSON payload from the :8888 wire.

        Raises ``TypeError`` if the payload is valid JSON but not an object
        (``null``, a number, a string, an array) and ``KeyError`` if a
        field the producer always sends is missing -- the caller treats
        either as a malformed payload and drops it.  ``navigation`` and
        the target fields are optional by design.
        """
        if not isinstance(payload, Mapping):
            raise TypeError(
                "stabilizer payload must be a JSON object, "
                f"got {type(payload).__name__}"
            )
        target_dx = payload.get("target_dx_pixels")
        target_dy = payload.get("target_dy_pixels")
        return cls(
            dx=float(payload["dx"]),
            dy=float(payload["dy"]),
            angle_deg=float(payload["angle_deg"]),
            matches_percent=float(payload["matches_percent"]),
            timestamp=float(payload["timestamp"]),
            navigation=bool(payload.get("navigation", False)),
            target_dx_pixels=None if target_dx is None else float(target_dx),
            target_dy_pixels=None if target_dy is None else float(target_dy),
        )


@dataclass(frozen=True)
class DetectionReading:
    """One object detection as received from the detection client.

    Carries exactly what the intercept logic consumes: the detection
    confidence and the first two components of
    ``direction_vector.direction`` (horizontal / vertical offset of the
    target from the image center, each nominally in [-0.5, 0.5]).
    """

    confidence: float
    dir_x: float
    dir_y: float
    class_id: Optional[int] = None

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "DetectionReading":
        """Parse one decoded detection JSON payload.

        Raises ``TypeError`` if the payload is valid JSON but not an object
        (the caller treats that as a malformed payload and drops it).
        Otherwise mirrors the defaults the controller historically applied
        while dict fishing: missing confidence -> 0.0, missing/short
        direction vector -> zero components.
        """
        if not isinstance(payload, Mapping):
            raise TypeError(
                "detection payload must be a JSON object, "
                f"got {type(payload).__name__}"
            )
        direction = payload.get("direction_vector", {}).get("direction", [0, 0, 0])
        return cls(
            confidence=float(payload.get("confidence", 0.0)),
            dir_x=float(direction[0]) if len(direction) > 0 else 0.0,
            dir_y=float(direction[1]) if len(direction) > 1 else 0.0,
            class_id=payload.get("class_id"),
        )
