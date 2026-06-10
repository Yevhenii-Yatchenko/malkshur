"""
Navigation coordinator.

This module coordinates the navigation system by orchestrating both
command modification and reference frame management.
"""
from __future__ import annotations
import os

from ..logger import UnbufferedLogger
from ..vision import ShiftCommand
from .command_modifier import CommandModifier
from .types import NavigationTarget
from threading import Timer


class NavigationCoordinator:
    """Coordinates navigation and reference frame management.

    This class provides a unified interface for the navigation system,
    orchestrating both command modification (for navigation) and reference
    update decisions (for drift prevention).

    """
    __command_timers: list[Timer]

    def __init__(self, logger: UnbufferedLogger) -> None:
        """Initialize navigation coordinator.

        Reads configuration from environment variables via config module.

        """
        self.logger = logger
        self.command_modifier = CommandModifier(logger)
        self.active = False

        self.logger.info("NavigationCoordinator initialized")

        self.__command_timers = [
            Timer(5, lambda: self.set_target(NavigationTarget(dx_pixels = 1000, dy_pixels = 0))),
            Timer(5, lambda: self.set_target(NavigationTarget(dx_pixels = 1000, dy_pixels = 1000))),
            Timer(5, lambda: self.set_target(NavigationTarget(dx_pixels = 0, dy_pixels = 1000))),
            Timer(5, lambda: self.set_target(NavigationTarget(dx_pixels = 0, dy_pixels = 0))),
        ]

        if os.environ.get("TEST_NAVIGATION", "0") != "1":
            self.__command_timers = []

        self.__schedule_command()

    def __schedule_command(self) -> None:
        timer = self.__command_timers.pop(0) if self.__command_timers else None
        if timer is not None:
            timer.start()

    def set_target(self, target: NavigationTarget) -> None:
        """Set navigation target and activate navigation."""
        self.command_modifier.set_target(target)
        self.active = True
        self.logger.info(f"Navigation activated: {target}")

    def process(self, shift_cmd: ShiftCommand) -> tuple[ShiftCommand, bool]:
        """Process shift command through navigation system.

        This is the main entry point called from the controller loop.
        """

        update_reference = False
        if self.active:
            modified_cmd, target_reached = self.command_modifier.modify(shift_cmd)
            update_reference = not target_reached

            if target_reached:
                self.logger.info("Navigation target reached")
                self.active = False

                self.__schedule_command()
        else:
            modified_cmd = shift_cmd

        target = self.command_modifier.get_target()
        if target is not None:
            modified_cmd.target_dx_pixels = target.dx_pixels
            modified_cmd.target_dy_pixels = target.dy_pixels
        else:
            modified_cmd.target_dx_pixels = None
            modified_cmd.target_dy_pixels = None

        return (modified_cmd, update_reference)
