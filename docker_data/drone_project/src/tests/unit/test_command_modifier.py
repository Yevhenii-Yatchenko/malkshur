"""Unit tests for the navigation sentinel emitted by CommandModifier
(sky_anchor/app/navigation/command_modifier.py) -- the PRODUCER side of the
confidence-sentinel contract.

Contract pinned across the process boundary:

- While a navigation target is active, CommandModifier.__makeCommand emits
  modified ShiftCommands with ``matches_percent=101.0`` (a literal).
- src/controller.py forwards ``confidence = matches_percent / 100.0`` into
  PositionController.update, whose mode state machine treats
  ``confidence == 1.01`` as the navigation trigger
  (tests/unit/test_position_controller.py pins that consumer side).
- 101.0 / 100.0 == 1.01 holds EXACTLY in IEEE-754 doubles (correctly rounded
  division; both sides round to the same double), so the consumer's equality
  check is reliable -- but only for this exact wire value.

CommandModifier.__init__ eagerly creates a NavigationCSVLogger (mkdir + open
file under logs/csv/navigation/), so that class is patched in the module
namespace; the DualPIDVelocityController and its PIDControllers are real
(pure in-memory math).  The injected dx/dy depend on wall-clock dt inside
__makeCommand, so only the deterministic fields are asserted.
"""

from unittest import mock

import pytest

from sky_anchor.app.navigation.command_modifier import CommandModifier
from sky_anchor.app.navigation.types import NavigationTarget
from sky_anchor.app.vision import ShiftCommand

pytestmark = [pytest.mark.unit]


@pytest.fixture
def modifier():
    with mock.patch(
        "sky_anchor.app.navigation.command_modifier.NavigationCSVLogger"
    ):
        yield CommandModifier(logger=mock.Mock())


def make_command(matches_percent=87):
    return ShiftCommand(dx=0, dy=0, angle_deg=0, matches_percent=matches_percent)


class TestNavigationSentinelEmission:
    def test_without_target_command_passes_through_unmodified(self, modifier):
        command = make_command(matches_percent=87)
        modified, reached = modifier.modify(command)
        assert modified is command
        assert reached is False
        assert modified.matches_percent == 87  # no sentinel without a target

    def test_active_navigation_emits_matches_percent_exactly_101(self, modifier):
        modifier.set_target(NavigationTarget(dx_pixels=400.0, dy_pixels=-300.0))
        modified, reached = modifier.modify(make_command())
        assert reached is False
        assert modified is not None
        assert modified.matches_percent == 101.0
        assert modified.angle_deg == 0.0  # navigation never injects rotation

    def test_emitted_sentinel_maps_exactly_onto_consumer_trigger(self, modifier):
        """src/controller.py divides by 100.0; PositionController compares the
        result with == 1.01.  Exact float round-trip, no tolerance."""
        modifier.set_target(NavigationTarget(dx_pixels=400.0, dy_pixels=-300.0))
        modified, _ = modifier.modify(make_command())
        assert modified.matches_percent / 100.0 == 1.01

    def test_reached_target_returns_original_command_not_sentinel(self, modifier):
        # The internal position estimate starts at (0, 0), so a target inside
        # the default 25 px tolerance is "reached" on the first modify() call.
        modifier.set_target(NavigationTarget(dx_pixels=3.0, dy_pixels=4.0))
        command = make_command(matches_percent=87)
        modified, reached = modifier.modify(command)
        assert reached is True
        assert modified is command
        assert modified.matches_percent == 87
        assert modifier.get_target() is None  # target consumed on arrival
