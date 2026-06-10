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

Step 5 carried bullet (client health re-sync): is_connected/poll_new()
must reflect the CLIENT's health -- a dead receive thread
(client.is_connected() False) drops the manager's connected flag instead
of reporting "connected" forever.  The drop is one-way (no hidden
reconnect; reconnecting remains the connection thread's job).

Step 7 (LC-3): the SkyAnchorClient is constructor-injected, so the tests
hand in a plain mock; the connected flag is still set through the
name-mangled attribute (it is normally flipped by the background connect
thread, which the tests never start).

Step 7 carried bullet (version-skew tripwire): a reading whose
matches_percent exceeds 100 while navigation is False means the producer
predates the explicit navigation flag (the historic 101.0 placeholder
without its Step 4 companion field) -- poll_new() warns ONCE per manager
and never drops the reading.
"""

from unittest import mock

import pytest

from src.domain.types import StabilizerReading
from src.stabilizer_manager import StabilizerManager

pytestmark = [pytest.mark.unit]


def make_reading(timestamp, navigation=False, matches_percent=90.0):
    return StabilizerReading(
        dx=1.0, dy=2.0, angle_deg=0.0, matches_percent=matches_percent,
        timestamp=timestamp, navigation=navigation,
    )


@pytest.fixture
def manager_and_client():
    client = mock.Mock()
    client.tick.return_value = None
    # Healthy receive thread unless a test says otherwise (Step 5 re-sync).
    client.is_connected.return_value = True
    manager = StabilizerManager(
        stabilizer_path="unused/sky_anchor/main.py",
        logger=mock.Mock(),
        client=client,
    )
    # Connection state is normally flipped by the background connect thread;
    # set it directly (the tests never spawn the thread).
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


class TestVersionSkewTripwire:
    """Step 7 carried bullet: matches_percent > 100 with navigation=False
    is an old-producer navigation frame; warn once, never drop data."""

    def _skew_warnings(self, manager):
        logger = manager._StabilizerManager__logger
        return [
            call for call in logger.warning.call_args_list
            if "skew" in call.args[0]
        ]

    def test_skewed_reading_warns_once_and_is_still_handed_out(
        self, manager_and_client
    ):
        manager, client = manager_and_client
        skewed = make_reading(
            timestamp=10.0, matches_percent=101.0, navigation=False
        )
        client.tick.return_value = skewed
        assert manager.poll_new() is skewed
        warnings = self._skew_warnings(manager)
        assert len(warnings) == 1
        assert "navigation=False" in warnings[0].args[0]

    def test_warns_once_not_per_reading(self, manager_and_client):
        manager, client = manager_and_client
        for step in range(3):
            client.tick.return_value = make_reading(
                timestamp=10.0 + step, matches_percent=101.0,
                navigation=False,
            )
            manager.poll_new()
        assert len(self._skew_warnings(manager)) == 1

    def test_repeat_polls_of_the_same_reading_do_not_spam(
        self, manager_and_client
    ):
        manager, client = manager_and_client
        client.tick.return_value = make_reading(
            timestamp=10.0, matches_percent=101.0, navigation=False
        )
        manager.poll_new()
        # Same reading still latest -> de-duplicated to None, no new warning.
        assert manager.poll_new() is None
        assert len(self._skew_warnings(manager)) == 1

    def test_navigation_placeholder_with_flag_is_not_skew(
        self, manager_and_client
    ):
        """A current producer emits 101.0 WITH navigation=True -- fine."""
        manager, client = manager_and_client
        client.tick.return_value = make_reading(
            timestamp=10.0, matches_percent=101.0, navigation=True
        )
        manager.poll_new()
        assert self._skew_warnings(manager) == []

    @pytest.mark.parametrize("matches_percent", [0.0, 90.0, 100.0])
    def test_normal_confidence_never_warns(
        self, manager_and_client, matches_percent
    ):
        """Strictly > 100: the legitimate 0-100 range (including exactly
        100) stays silent."""
        manager, client = manager_and_client
        client.tick.return_value = make_reading(
            timestamp=10.0, matches_percent=matches_percent,
            navigation=False,
        )
        manager.poll_new()
        assert self._skew_warnings(manager) == []


class TestClientHealthResync:
    """Step 5 carried bullet: a dead client receive thread must surface."""

    def test_healthy_client_keeps_manager_connected(self, manager_and_client):
        manager, client = manager_and_client
        assert manager.is_connected is True
        reading = make_reading(timestamp=10.0)
        client.tick.return_value = reading
        assert manager.poll_new() is reading

    def test_dead_client_drops_is_connected(self, manager_and_client):
        manager, client = manager_and_client
        client.is_connected.return_value = False
        assert manager.is_connected is False

    def test_dead_client_makes_poll_new_return_none_without_polling(
        self, manager_and_client
    ):
        manager, client = manager_and_client
        client.is_connected.return_value = False
        client.tick.return_value = make_reading(timestamp=10.0)
        assert manager.poll_new() is None
        client.tick.assert_not_called()

    def test_drop_is_one_way_with_no_hidden_reconnect(self, manager_and_client):
        """Once dropped, the flag stays down even if the client claims
        health again -- reconnecting is the connection thread's job, which
        re-sync deliberately does not replicate."""
        manager, client = manager_and_client
        client.is_connected.return_value = False
        assert manager.is_connected is False
        client.is_connected.return_value = True
        assert manager.is_connected is False
        assert manager.poll_new() is None

    def test_client_health_is_not_queried_before_first_connect(
        self, manager_and_client
    ):
        """The re-sync only ever downgrades an established connection: a
        manager that never connected short-circuits without touching the
        client."""
        manager, client = manager_and_client
        manager._StabilizerManager__connected = False
        assert manager.is_connected is False
        client.is_connected.assert_not_called()
