"""Unit tests for StabilizerManager.poll_new() (src/stabilizer_manager.py).

Step 4 (IE-2) moved the freshness decision -- the producer-timestamp
de-duplication that DroneController used to do with its __last_xy_update
field -- into the data's owner.  Contract pinned here:

- not connected -> None (regardless of what the client holds);
- nothing received yet -> None;
- a reading is handed out AT MOST ONCE: polling again while the client
  still holds the same reading returns None;
- a reading with a different timestamp is handed out again;
- the initial "last consumed" timestamp is 0, mirroring the controller's
  former __last_xy_update = 0 (so a reading stamped exactly 0 is treated
  as already consumed -- historic edge kept on purpose);
- client errors are swallowed (logged) and surface as None.

StabilizerManager still constructs its own SkyAnchorClient (LC-3 is a
Step 7 concern), so the class is patched in the module namespace and the
connected flag is set through the name-mangled attribute.
"""

from unittest import mock

import pytest

from src.domain.types import StabilizerReading
from src.stabilizer_manager import StabilizerManager

pytestmark = [pytest.mark.unit]


def make_reading(timestamp, navigation=False):
    return StabilizerReading(
        dx=1.0, dy=2.0, angle_deg=0.0, matches_percent=90.0,
        timestamp=timestamp, navigation=navigation,
    )


@pytest.fixture
def manager_and_client():
    with mock.patch("src.stabilizer_manager.SkyAnchorClient") as client_cls:
        manager = StabilizerManager(
            stabilizer_path="unused/sky_anchor/main.py", logger=mock.Mock()
        )
    client = client_cls.return_value
    client.tick.return_value = None
    # Connection state is normally flipped by the background connect thread;
    # set it directly until Step 7 makes the client injectable.
    manager._StabilizerManager__connected = True
    return manager, client


class TestPollNew:
    def test_returns_none_when_not_connected(self, manager_and_client):
        manager, client = manager_and_client
        client.tick.return_value = make_reading(timestamp=10.0)
        manager._StabilizerManager__connected = False
        assert manager.poll_new() is None
        client.tick.assert_not_called()

    def test_returns_none_when_nothing_received_yet(self, manager_and_client):
        manager, client = manager_and_client
        assert manager.poll_new() is None

    def test_returns_a_fresh_reading_exactly_once(self, manager_and_client):
        manager, client = manager_and_client
        reading = make_reading(timestamp=10.0, navigation=True)
        client.tick.return_value = reading
        assert manager.poll_new() is reading
        # Same reading still latest on the client -> already consumed.
        assert manager.poll_new() is None
        assert manager.poll_new() is None

    def test_new_timestamp_is_returned_again(self, manager_and_client):
        manager, client = manager_and_client
        first = make_reading(timestamp=10.0)
        second = make_reading(timestamp=10.02)
        client.tick.return_value = first
        assert manager.poll_new() is first
        client.tick.return_value = second
        assert manager.poll_new() is second
        assert manager.poll_new() is None

    def test_initial_consumed_timestamp_is_zero(self, manager_and_client):
        """Mirror of the controller's former __last_xy_update = 0: a reading
        stamped exactly 0 looks already-consumed on the first poll."""
        manager, client = manager_and_client
        client.tick.return_value = make_reading(timestamp=0)
        assert manager.poll_new() is None

    def test_client_error_is_swallowed_and_returns_none(self, manager_and_client):
        manager, client = manager_and_client
        client.tick.side_effect = RuntimeError("socket exploded")
        assert manager.poll_new() is None
