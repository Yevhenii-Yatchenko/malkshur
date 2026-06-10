"""Typed configuration value objects (GRASP Step 7, LC-2).

The tuning numbers themselves stay in the existing ``src/altitude_config.py``
and ``src/position_config.py`` dict modules -- the dataclasses here are
constructed FROM those dicts (never hand-copied), so the tuning remains
byte-identical and single-sourced.
"""

from src.config.objects import AltitudeConfig, PIDGains, PositionConfig

__all__ = [
    "AltitudeConfig",
    "PIDGains",
    "PositionConfig",
]
