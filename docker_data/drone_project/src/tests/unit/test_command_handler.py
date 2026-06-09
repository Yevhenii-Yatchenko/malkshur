"""Unit tests for CommandHandler parsing/dispatch (src/command_handler.py).

CAUTION (LC-3 in the refactoring plan): constructing CommandHandler eagerly
creates AND starts a TelnetServer (real TCP listener on port 2323) and, when
no logger is passed, opens a log file under logs/.  Production code must not
change in Step 2, so the tests neutralize both side effects:

- ``src.command_handler.TelnetServer`` is patched in the module namespace,
  so no socket is ever opened;
- an explicit mock ``logger`` is passed, so ``get_logger`` is never called
  and nothing is written under logs/.

Several asserted parsing behaviors are current quirks, documented on purpose
(e.g. float-looking params stay strings, empty params become 0).
"""

import json
from unittest import mock

import pytest

from src.command_handler import CommandHandler

pytestmark = [pytest.mark.unit, pytest.mark.communication]


@pytest.fixture
def telnet_cls():
    """Patch the TelnetServer symbol inside the command_handler module."""
    with mock.patch("src.command_handler.TelnetServer", autospec=True) as cls:
        yield cls


@pytest.fixture
def handler(telnet_cls):
    """A CommandHandler with no real sockets and no real log files."""
    return CommandHandler(logger=mock.Mock())


class TestConstructionSideEffects:
    """Verify the LC-3 coupling is neutralized (and document it exists)."""

    def test_constructor_creates_and_starts_telnet_server(self, telnet_cls, handler):
        telnet_cls.assert_called_once_with(host="0.0.0.0", port=2323)
        telnet_cls.return_value.start.assert_called_once_with()

    def test_constructor_forwards_custom_host_and_port(self, telnet_cls):
        CommandHandler(logger=mock.Mock(), telnet_host="127.0.0.1", telnet_port=4242)
        telnet_cls.assert_called_once_with(host="127.0.0.1", port=4242)

    def test_cleanup_stops_telnet_server(self, telnet_cls, handler):
        handler.cleanup()
        telnet_cls.return_value.stop.assert_called_once_with()


class TestParseMessage:
    @pytest.mark.parametrize(
        "message, expected_key, expected_params",
        [
            ("mode,GUIDED", "mode", ["GUIDED"]),
            ("setHeight,5", "setHeight", [5]),
            ("takeoff,10", "takeoff", [10]),
            ("move,3,-4", "move", [3, -4]),
            ("land", "land", []),
            ("stabilize", "stabilize", []),
            # int() strips whitespace, so " 5" parses as an integer.
            ("arm, 5", "arm", [5]),
            # Quirk: non-integer params stay strings (no float parsing).
            ("goto,2.5,north", "goto", ["2.5", "north"]),
            # Quirk: empty params become integer 0.
            ("arm,", "arm", [0]),
            ("mode,GUIDED,", "mode", ["GUIDED", 0]),
            # Quirk: empty message yields empty-string key, not None.
            ("", "", []),
            (",", "", [0]),
            # Quirk: string params keep their surrounding whitespace.
            ("mode, GUIDED", "mode", [" GUIDED"]),
        ],
    )
    def test_parse_message_table(self, handler, message, expected_key, expected_params):
        assert handler.parse_message(message) == (expected_key, expected_params)

    @pytest.mark.parametrize("bad_input", [None, 123, ["mode", "GUIDED"]])
    def test_parse_message_non_string_returns_none_pair(self, handler, bad_input):
        assert handler.parse_message(bad_input) == (None, None)


class TestParseJsonMessage:
    def test_extracts_msg_field(self, handler):
        assert handler.parse_json_message('{"msg": "setHeight,5"}') == "setHeight,5"

    def test_ignores_extra_fields(self, handler):
        payload = json.dumps({"msg": "mode,GUIDED", "extra": 1})
        assert handler.parse_json_message(payload) == "mode,GUIDED"

    @pytest.mark.parametrize(
        "bad_payload",
        [
            "not-json",
            "",
            "{broken",
            "[1, 2, 3]",        # valid JSON, but not an object
            '"bare string"',    # valid JSON, but not an object
        ],
    )
    def test_malformed_payload_returns_none(self, handler, bad_payload):
        assert handler.parse_json_message(bad_payload) is None

    def test_missing_msg_field_returns_none(self, handler):
        assert handler.parse_json_message('{"other": "x"}') is None
        assert handler.parse_json_message("{}") is None


class TestExecuteCommand:
    def test_executes_registered_handler_with_params(self, handler):
        received = []
        handler.register_command("setHeight", received.append)
        result = handler.execute_command("setHeight", [5])
        assert result["success"] is True
        assert received == [[5]]

    def test_unknown_command_fails(self, handler):
        result = handler.execute_command("warpDrive", [9])
        assert result["success"] is False
        assert "not found" in result["message"]

    def test_none_key_fails(self, handler):
        result = handler.execute_command(None, [])
        assert result["success"] is False

    def test_handler_exception_is_caught(self, handler):
        def boom(params):
            raise ValueError("kaput")

        handler.register_command("boom", boom)
        result = handler.execute_command("boom", [])
        assert result["success"] is False
        assert "kaput" in result["message"]

    def test_get_registered_commands(self, handler):
        handler.register_command("a", lambda params: None)
        handler.register_command("b", lambda params: None)
        assert handler.get_registered_commands() == ["a", "b"]


class TestProcessJsonCommand:
    """End-to-end: JSON -> command string -> dispatch (no sockets involved)."""

    def test_valid_json_command_round_trip(self, handler):
        received = []
        handler.register_command("setHeight", received.append)
        result = handler.process_json_command('{"msg": "setHeight,7"}')
        assert result["success"] is True
        assert received == [[7]]

    def test_invalid_json_reports_invalid_message(self, handler):
        result = handler.process_json_command("definitely not json")
        assert result == {"success": False, "message": "Invalid JSON message"}

    def test_non_string_msg_reports_invalid_format(self, handler):
        # "msg" present but not a string: parse_message then fails.
        result = handler.process_json_command('{"msg": 123}')
        assert result == {"success": False, "message": "Invalid command format"}

    def test_unknown_command_reports_failure(self, handler):
        result = handler.process_json_command('{"msg": "warpDrive,9"}')
        assert result["success"] is False
        assert "not found" in result["message"]
