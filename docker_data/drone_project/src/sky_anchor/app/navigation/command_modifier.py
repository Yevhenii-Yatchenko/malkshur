"""
Command modification for navigation.

This module handles modifying drift commands to navigate toward waypoints while
maintaining the stabilization benefits of the sky_anchor system.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from ..logger import UnbufferedLogger
from ..vision import ShiftCommand
from .types import NavigationTarget
from .velocity_controller import DualPIDVelocityController
from .navigation_csv_logger import NavigationCSVLogger


class CommandModifier:
    """
    Modifies shift commands to navigate toward waypoints.

    Strategy: Inject "virtual drift" opposite to the desired direction.
    If we want to move +50 pixels in X, we pretend there's -50 drift.
    The position controller will then correct by moving +50 pixels.

    This approach leverages the existing stabilization system without
    requiring changes to the main controller.
    """

    __logger: UnbufferedLogger
    __velocity_controller: DualPIDVelocityController

    __target: Optional[NavigationTarget] = None
    __position_x: float = 0.0
    __position_y: float = 0.0
    __last_update_time: Optional[float] = None

    @property
    def __error_x(self) -> float:
        return self.__target.dx_pixels - self.__position_x if self.__target else 0.0

    @property
    def __error_y(self) -> float:
        return self.__target.dy_pixels - self.__position_y if self.__target else 0.0

    def __init__(self, logger: UnbufferedLogger) -> None:
        """
        Initialize command modifier with dual PID velocity controller.
        """
        self.__logger = logger
        self.__velocity_controller = DualPIDVelocityController()
        self.__nav_logger = NavigationCSVLogger()
        self.__logger.info("CommandModifier initialized with DualPIDVelocityController and CSV logging")

    def set_target(self, target: NavigationTarget) -> None:
        """
        Set navigation target and reset velocity controller.
        """
        self.__target = target
        self.__last_update_time = None

        # Reset velocity controller for new navigation
        self.__velocity_controller.reset()

        self.__logger.info(f"Navigation target set: {self.__target}")

    def get_target(self) -> Optional[NavigationTarget]:
        """
        Return the currently active target (if any).
        """
        return self.__target

    def __update_position(self, dx: float, dy: float) -> None:
        """
        Update internal position estimate.

        """
        if self.__target is None:
            return

        self.__position_x += dx
        self.__position_y += dy

    def __check_if_target_reached(self) -> bool:
        """
        Check if navigation __target is reached.
        """
        if self.__target is None:
            return False

        distance = math.sqrt(self.__error_x**2 + self.__error_y**2)
        result = distance < self.__target.tolerance_pixels

        if result:
            self.__logger.info(
                f"Target reached! Target={self.__target}; Current Position=({self.__position_x:.1f}, {self.__position_y:.1f})"
            )

        return result

    def __makeCommand(self, shift_cmd: ShiftCommand) -> ShiftCommand:
        """
        Create modified command using hybrid velocity controller.
        """
        current_time = time.time()
        if self.__last_update_time is None:
            dt = 0.01
        else:
            dt = current_time - self.__last_update_time
            dt = max(0.001, min(dt, 0.1))

        self.__last_update_time = current_time

        vel_x, vel_y = self.__velocity_controller.calculate_command(
            self.__error_x,
            self.__error_y,
            dt
        )

        return ShiftCommand(
            dx=-vel_x,
            dy=-vel_y,
            angle_deg=0.0,
            matches_percent=101.0,
        )

    def modify(self, shift_cmd: ShiftCommand) -> tuple[ShiftCommand, bool]:
        """
        Modify shift command for navigation.
        """
        if self.__target is None:
            return (shift_cmd, False)

        self.__update_position(shift_cmd.dx, shift_cmd.dy)

        if self.__check_if_target_reached():
            self.__target = None
            return (shift_cmd, True)

        modified_cmd = self.__makeCommand(shift_cmd)

        pid_state = self.__velocity_controller.get_pid_state()
        error_magnitude = math.sqrt(self.__error_x**2 + self.__error_y**2)

        nav_data = {
            'timestamp': time.time(),
            'target_x': self.__target.dx_pixels if self.__target else 0,
            'target_y': self.__target.dy_pixels if self.__target else 0,
            'position_x': self.__position_x,
            'position_y': self.__position_y,
            'error_x': self.__error_x,
            'error_y': self.__error_y,
            'error_magnitude': error_magnitude,
            'commanded_vel_x': pid_state['vel_x'],
            'commanded_vel_y': pid_state['vel_y'],
            'pid_state': pid_state,
            'injected_dx': modified_cmd.dx,
            'injected_dy': modified_cmd.dy,
        }
        self.__nav_logger.append(nav_data)

        self.__logger.info(
            f"Command modified old: {shift_cmd} → new: {modified_cmd} | "
            f"Target=({self.__target.dx_pixels:.1f}, {self.__target.dy_pixels:.1f}) "
            f"Position=({self.__position_x:.1f}, {self.__position_y:.1f}) "
            f"Velocity: X={pid_state['vel_x']:.1f}, Y={pid_state['vel_y']:.1f}"
        )

        return (modified_cmd, False)
