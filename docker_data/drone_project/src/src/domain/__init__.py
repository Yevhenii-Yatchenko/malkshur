"""Typed domain data for the flight-controller process (GRASP Step 4, IE-2/IE-3).

Frozen dataclasses that replace the raw dicts previously passed around
between the network edges (sky_anchor TCP :8888, detection TCP server) and
the control logic.
"""

from src.domain.types import DetectionReading, StabilizerReading

__all__ = ["DetectionReading", "StabilizerReading"]
