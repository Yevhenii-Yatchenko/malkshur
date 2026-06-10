"""Unit tests for the typed wire readings (src/domain/types.py, Step 4 IE-2/IE-3).

StabilizerReading is the consumer-side type for the sky_anchor TCP :8888
payload; DetectionReading is the consumer-side type for the detection
client's payload.  Both ends of the stabilizer wire live in this repository,
so TestStabilizerWireRoundTrip drives the REAL producer serialization
(ShiftCommand.to_payload + the CommandPublisher timestamp + json round-trip)
into the real consumer parser -- the cross-process contract in one test.
"""

import dataclasses
import json

import pytest

from sky_anchor.app.vision import ShiftCommand
from src.domain.types import DetectionReading, StabilizerReading

pytestmark = [pytest.mark.unit]


def stabilizer_payload(**overrides):
    """A payload exactly as the current producer emits it."""
    payload = {
        "dx": 12,
        "dy": -5,
        "angle_deg": 2,
        "matches_percent": 87,
        "navigation": False,
        "timestamp": 1234567890.123,
    }
    payload.update(overrides)
    return payload


class TestStabilizerReadingParsing:
    def test_parses_all_wire_fields(self):
        reading = StabilizerReading.from_payload(stabilizer_payload())
        assert reading.dx == 12.0
        assert reading.dy == -5.0
        assert reading.angle_deg == 2.0
        assert reading.matches_percent == 87.0
        assert reading.timestamp == 1234567890.123
        assert reading.navigation is False
        assert reading.target_dx_pixels is None
        assert reading.target_dy_pixels is None

    def test_parses_navigation_true(self):
        reading = StabilizerReading.from_payload(
            stabilizer_payload(navigation=True, matches_percent=101.0)
        )
        assert reading.navigation is True
        assert reading.matches_percent == 101.0

    def test_missing_navigation_defaults_to_false(self):
        """Old-producer compatibility: the field is additive, payloads
        without it must still parse (as non-navigation)."""
        payload = stabilizer_payload()
        del payload["navigation"]
        reading = StabilizerReading.from_payload(payload)
        assert reading.navigation is False

    def test_parses_optional_navigation_targets_when_present(self):
        reading = StabilizerReading.from_payload(
            stabilizer_payload(target_dx_pixels=1000.0, target_dy_pixels=-250.5)
        )
        assert reading.target_dx_pixels == 1000.0
        assert reading.target_dy_pixels == -250.5

    @pytest.mark.parametrize(
        "missing", ["dx", "dy", "angle_deg", "matches_percent", "timestamp"]
    )
    def test_missing_required_field_raises_key_error(self, missing):
        """The client treats KeyError as a malformed payload and drops it."""
        payload = stabilizer_payload()
        del payload[missing]
        with pytest.raises(KeyError):
            StabilizerReading.from_payload(payload)

    @pytest.mark.parametrize("payload", [None, 5, "x", []])
    def test_non_mapping_payload_raises_type_error(self, payload):
        """Valid JSON that is not an object (``null``, ``5``, ``"x"``,
        ``[]``) must raise TypeError -- which is in the client's per-line
        catch tuple -- not AttributeError, which used to escape it and
        kill the receive thread permanently."""
        with pytest.raises(TypeError, match="must be a JSON object"):
            StabilizerReading.from_payload(payload)

    def test_confidence_property_maps_matches_percent_onto_unit_range(self):
        """The /100.0 mapping moved from controller.py into the reading."""
        assert StabilizerReading.from_payload(
            stabilizer_payload(matches_percent=87)
        ).confidence == pytest.approx(0.87)
        assert StabilizerReading.from_payload(
            stabilizer_payload(matches_percent=0)
        ).confidence == 0.0

    def test_navigation_placeholder_still_maps_exactly_to_1_01(self):
        """101.0 / 100.0 == 1.01 holds exactly in IEEE-754 -- pinned because
        the value still flows into last_confidence and the CSV stream, even
        though nothing compares against it anymore."""
        reading = StabilizerReading.from_payload(
            stabilizer_payload(matches_percent=101.0, navigation=True)
        )
        assert reading.confidence == 1.01

    def test_reading_is_frozen(self):
        reading = StabilizerReading.from_payload(stabilizer_payload())
        with pytest.raises(dataclasses.FrozenInstanceError):
            reading.dx = 99.0


class TestStabilizerWireRoundTrip:
    """Producer -> JSON -> consumer, using the real code on both ends."""

    @staticmethod
    def _round_trip(command, timestamp=111.5):
        payload = command.to_payload()
        payload["timestamp"] = timestamp  # what CommandPublisher.publish adds
        return StabilizerReading.from_payload(json.loads(json.dumps(payload)))

    def test_normal_frame_round_trip(self):
        command = ShiftCommand(dx=12, dy=-5, angle_deg=2, matches_percent=87)
        reading = self._round_trip(command)
        assert (reading.dx, reading.dy, reading.angle_deg) == (12.0, -5.0, 2.0)
        assert reading.matches_percent == 87.0
        assert reading.navigation is False
        assert reading.timestamp == 111.5
        assert reading.target_dx_pixels is None and reading.target_dy_pixels is None

    def test_navigation_frame_round_trip(self):
        """A command exactly as CommandModifier emits it while navigating:
        explicit flag plus the historic 101.0 placeholder."""
        command = ShiftCommand(
            dx=-3, dy=7, angle_deg=0.0, matches_percent=101.0, navigation=True,
            target_dx_pixels=1000.0, target_dy_pixels=0.0,
        )
        reading = self._round_trip(command)
        assert reading.navigation is True
        assert reading.matches_percent == 101.0
        assert reading.confidence == 1.01  # informational, not a trigger
        assert reading.target_dx_pixels == 1000.0
        assert reading.target_dy_pixels == 0.0


def detection_payload():
    """Shape produced by the dd_shahed client (dd_shahed/src/utils.cpp)."""
    return {
        "image_info": {"format": "opencv_bgr", "width": 640, "height": 480},
        "coordinates": {"x_min": -10.0, "y_min": -5.0},
        "class_id": 1,
        "confidence": 0.91,
        "direction_vector": {
            "direction": [0.25, -0.1, 1.0],
            "magnitude": 1.05,
            "magnitude_normalized": 0.52,
        },
    }


class TestDetectionReadingParsing:
    def test_parses_confidence_class_and_direction_components(self):
        reading = DetectionReading.from_payload(detection_payload())
        assert reading.confidence == pytest.approx(0.91)
        assert reading.class_id == 1
        assert reading.dir_x == pytest.approx(0.25)
        assert reading.dir_y == pytest.approx(-0.1)

    def test_missing_confidence_defaults_to_zero(self):
        payload = detection_payload()
        del payload["confidence"]
        assert DetectionReading.from_payload(payload).confidence == 0.0

    @pytest.mark.parametrize(
        "direction, expected",
        [
            ([], (0.0, 0.0)),          # empty vector -> both default
            ([0.3], (0.3, 0.0)),       # short vector -> dir_y defaults
            ([0.3, -0.2], (0.3, -0.2)),
        ],
    )
    def test_short_direction_vectors_default_per_component(
        self, direction, expected
    ):
        """Mirrors the controller's historic per-index guards."""
        payload = detection_payload()
        payload["direction_vector"]["direction"] = direction
        reading = DetectionReading.from_payload(payload)
        assert (reading.dir_x, reading.dir_y) == expected

    def test_missing_direction_vector_defaults_to_zero_components(self):
        payload = detection_payload()
        del payload["direction_vector"]
        reading = DetectionReading.from_payload(payload)
        assert (reading.dir_x, reading.dir_y) == (0.0, 0.0)

    @pytest.mark.parametrize("payload", [None, 5, "x", []])
    def test_non_mapping_payload_raises_type_error(self, payload):
        """Same guard as StabilizerReading: a valid-JSON non-object must
        raise TypeError, which DetectionServer.get_active_target catches."""
        with pytest.raises(TypeError, match="must be a JSON object"):
            DetectionReading.from_payload(payload)

    def test_reading_is_frozen(self):
        reading = DetectionReading.from_payload(detection_payload())
        with pytest.raises(dataclasses.FrozenInstanceError):
            reading.confidence = 0.0
