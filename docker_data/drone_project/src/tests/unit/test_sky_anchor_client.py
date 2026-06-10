"""Unit tests for the SkyAnchorClient receive loop (src/sky_anchor_client.py).

Step 4 review fix: a syntactically valid JSON line that is not an object
(``null``, ``5``, ``"x"``, ``[]``) used to raise AttributeError inside
StabilizerReading.from_payload, escape the per-line catch tuple, hit the
outer handler, and kill the receive thread permanently (connected=False,
no reconnect -- and StabilizerManager never re-syncs, so poll_new() would
silently return None forever).  Pinned here: every malformed line is
dropped, the loop survives, and a SUBSEQUENT good line still produces a
reading.

The loop body is exercised synchronously: ``_receive_data`` runs on the
test thread against a scripted socket whose ``recv`` hands out one wire
chunk per call and then stops the client via the loop condition.  No real
sockets, threads, or log files are involved (``get_logger`` is patched).
"""

import json
import socket
from unittest import mock

import pytest

from src.sky_anchor_client import SkyAnchorClient

pytestmark = [pytest.mark.unit]


GOOD_LINE = json.dumps(
    {
        "dx": 12,
        "dy": -5,
        "angle_deg": 2,
        "matches_percent": 87,
        "navigation": False,
        "timestamp": 111.5,
    }
)


@pytest.fixture
def client():
    with mock.patch("src.sky_anchor_client.get_logger", return_value=mock.Mock()):
        yield SkyAnchorClient()


def run_receive_loop(client, lines):
    """Feed wire lines through _receive_data, one recv() chunk per line.

    Once the script is drained, recv() flips ``running`` off and raises a
    socket timeout, so the loop exits through its normal ``while`` check
    without touching ``connected`` -- meaning ``connected is True``
    afterwards if and only if no line tripped the outer (thread-killing)
    exception handler.
    """
    chunks = [(line + "\n").encode("utf-8") for line in lines]

    def scripted_recv(_size):
        if chunks:
            return chunks.pop(0)
        client.running = False
        raise socket.timeout()

    client.client_socket = mock.Mock()
    client.client_socket.recv.side_effect = scripted_recv
    client.running = True
    client.connected = True
    client._receive_data()


class TestReceiveLoopSurvivesMalformedLines:
    NON_DICT_JSON_LINES = ["null", "5", '"x"', "[]"]

    @pytest.mark.parametrize("bad_line", NON_DICT_JSON_LINES)
    def test_non_dict_json_line_is_dropped_and_no_reading_appears(
        self, client, bad_line
    ):
        run_receive_loop(client, [bad_line])
        assert client.tick() is None
        assert client.connected is True  # outer handler never tripped

    @pytest.mark.parametrize(
        "bad_line",
        NON_DICT_JSON_LINES
        + [
            '{"dx": 1}',        # dict-shaped garbage: required keys missing
            '{"dx": "abc", "dy": 0, "angle_deg": 0, '
            '"matches_percent": 0, "timestamp": 1.0}',  # wrong field type
            "not json at all",  # JSONDecodeError path
        ],
    )
    def test_subsequent_good_line_still_produces_a_reading(self, client, bad_line):
        run_receive_loop(client, [bad_line, GOOD_LINE])
        reading = client.tick()
        assert reading is not None
        assert (reading.dx, reading.dy) == (12.0, -5.0)
        assert reading.matches_percent == 87.0
        assert reading.timestamp == 111.5
        assert client.connected is True

    def test_burst_of_non_dict_lines_then_good_line(self, client):
        run_receive_loop(client, self.NON_DICT_JSON_LINES + [GOOD_LINE])
        reading = client.tick()
        assert reading is not None
        assert reading.timestamp == 111.5
        assert client.connected is True

    def test_good_line_alone_produces_a_reading(self, client):
        """Harness sanity check: the scripted socket drives the real path."""
        run_receive_loop(client, [GOOD_LINE])
        reading = client.tick()
        assert reading is not None
        assert reading.navigation is False
