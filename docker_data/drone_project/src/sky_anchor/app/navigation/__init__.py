"""
Navigation module for sky_anchor.

This module provides autonomous navigation capabilities by combining:
1. Command modification - Inject virtual drift to navigate toward waypoints
2. Reference frame management - Periodic updates to prevent drift accumulation

Public API:
    - NavigationCoordinator: Main interface for navigation system
    - NavigationTarget: Waypoint specification

Example:
    from app.navigation import NavigationCoordinator, NavigationTarget

    nav = NavigationCoordinator(logger)
    nav.set_target(NavigationTarget(dx_pixels=100, dy_pixels=50))

    # In main loop:
    modified_cmd, should_update_ref = nav.process(shift_cmd)
"""

from .coordinator import NavigationCoordinator
from .types import NavigationTarget

__all__ = [
    "NavigationCoordinator",
    "NavigationTarget",
]
