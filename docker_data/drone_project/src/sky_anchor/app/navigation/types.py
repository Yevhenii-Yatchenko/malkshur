"""
Shared types for the navigation system.

This module defines the core data structures used throughout the navigation system.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class NavigationTarget:
    """
    Represents a navigation waypoint in pixel coordinates.

    Coordinates are relative to the ORIGINAL reference frame at navigation start.
    After reference updates, the target coordinates remain in the original frame,
    but accumulated position is reset.

    """
    dx_pixels: float
    dy_pixels: float
    tolerance_pixels: float = 25.0

    def __str__(self) -> str:
        return f"Target({self.dx_pixels:.1f}, {self.dy_pixels:.1f}, tol={self.tolerance_pixels:.1f})"
