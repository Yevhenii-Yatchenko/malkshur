"""Unit tests for the navigation signal emitted by CommandModifier
(sky_anchor/app/navigation/command_modifier.py) -- the PRODUCER side of the
cross-process navigation contract.

Contract pinned across the process boundary (Step 4, IE-3):

- While a navigation target is active, CommandModifier.__makeCommand emits
  modified ShiftCommands with the EXPLICIT flag ``navigation=True`` -- this
  flag (serialized as the payload field ``"navigation"``) is what
  PositionController keys its mode switching on
  (tests/unit/test_position_controller.py pins that consumer side;
  tests/unit/test_domain_types.py pins the payload round-trip).
- The historic ``matches_percent=101.0`` placeholder is STILL emitted
  alongside (deliberate Step 4 decision: the numeric matches_percent stream
  feeding CSV logs/plots stays unchanged), but it is informational only --
  the consumer's ``confidence == 1.01`` comparison was deleted.
- Unmodified pass-through commands carry ``navigation=False`` (the
  dataclass default) and their real matches_percent.

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


class TestNavigationFlagEmission:
    def test_without_target_command_passes_through_unmodified(self, modifier):
        command = make_command(matches_percent=87)
        modified, reached = modifier.modify(command)
        assert modified is command
        assert reached is False
        assert modified.navigation is False  # no flag without a target
        assert modified.matches_percent == 87

    def test_active_navigation_sets_explicit_navigation_flag(self, modifier):
        modifier.set_target(NavigationTarget(dx_pixels=400.0, dy_pixels=-300.0))
        modified, reached = modifier.modify(make_command())
        assert reached is False
        assert modified is not None
        assert modified.navigation is True
        assert modified.angle_deg == 0.0  # navigation never injects rotation

    def test_active_navigation_keeps_emitting_matches_percent_101(self, modifier):
        """Deliberate Step 4 decision, pinned explicitly: the numeric stream
        keeps the historic 101.0 placeholder during navigation (CSV logs and
        plots stay byte-identical); only the navigation flag is the control
        signal now."""
        modifier.set_target(NavigationTarget(dx_pixels=400.0, dy_pixels=-300.0))
        modified, _ = modifier.modify(make_command())
        assert modified.matches_percent == 101.0
        # The consumer-side mapping (StabilizerReading.confidence) still
        # lands exactly on 1.01 -- informational only, nothing compares it.
        assert modified.matches_percent / 100.0 == 1.01

    def test_payload_carries_the_navigation_field_on_both_paths(self, modifier):
        """The flag must actually cross the wire: to_payload() includes
        "navigation" on modified AND pass-through commands (additive format
        change -- a pre-Step 4 client simply ignores the extra key)."""
        passthrough, _ = modifier.modify(make_command())
        assert passthrough.to_payload()["navigation"] is False
        modifier.set_target(NavigationTarget(dx_pixels=400.0, dy_pixels=-300.0))
        modified, _ = modifier.modify(make_command())
        assert modified.to_payload()["navigation"] is True

    def test_reached_target_returns_original_command_not_modified(self, modifier):
        # The internal position estimate starts at (0, 0), so a target inside
        # the default 25 px tolerance is "reached" on the first modify() call.
        modifier.set_target(NavigationTarget(dx_pixels=3.0, dy_pixels=4.0))
        command = make_command(matches_percent=87)
        modified, reached = modifier.modify(command)
        assert reached is True
        assert modified is command
        assert modified.navigation is False
        assert modified.matches_percent == 87
        assert modifier.get_target() is None  # target consumed on arrival
