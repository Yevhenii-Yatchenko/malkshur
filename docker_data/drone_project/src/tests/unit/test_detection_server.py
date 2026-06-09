"""Unit tests for DetectionServer.get_active_target() (src/detection_server.py).

Step 4 (IE-1/IE-2) moved the detection data-validity decisions out of
DroneController.__updateThrottle into the data's owner.  Contract pinned
here (the intercept state MACHINE stays in the controller and is out of
scope):

- no detection received -> None;
- detection older than ``timeout_s`` -> None (strictly-less-than freshness,
  matching the former ``time_since < INTERCEPT_TIMEOUT_SECONDS``);
- confidence below ``min_confidence`` -> None; exactly AT the floor passes
  (matching the former ``confidence >= INTERCEPT_CONFIDENCE_THRESHOLD``);
- otherwise a typed DetectionReading with the direction components
  extracted exactly as the controller used to do it.

The server is never start()ed: get_active_target only reads the
_latest_data/_last_update_time pair that _handle_client maintains, so the
tests seed those directly (no sockets, no threads).
"""

import time
from unittest import mock

import pytest

from src.detection_server import DetectionServer
from src.domain.types import DetectionReading

pytestmark = [pytest.mark.unit]


def make_server(detection=None, age_s=None):
    server = DetectionServer(logger=mock.Mock())
    if detection is not None:
        server._latest_data = detection
        server._last_update_time = time.time() - (age_s or 0.0)
    return server


def detection(confidence=0.9, direction=(0.25, -0.1, 1.0), class_id=1):
    return {
        "class_id": class_id,
        "confidence": confidence,
        "direction_vector": {"direction": list(direction)},
    }


class TestGetActiveTarget:
    def test_no_detection_received_returns_none(self):
        server = make_server()
        assert server.get_active_target(timeout_s=2.0, min_confidence=0.5) is None

    def test_fresh_confident_detection_returns_typed_reading(self):
        server = make_server(detection(confidence=0.9), age_s=0.0)
        target = server.get_active_target(timeout_s=10.0, min_confidence=0.5)
        assert isinstance(target, DetectionReading)
        assert target.confidence == pytest.approx(0.9)
        assert target.dir_x == pytest.approx(0.25)
        assert target.dir_y == pytest.approx(-0.1)
        assert target.class_id == 1

    def test_stale_detection_returns_none(self):
        server = make_server(detection(confidence=0.9), age_s=5.0)
        assert server.get_active_target(timeout_s=1.0, min_confidence=0.5) is None

    def test_low_confidence_detection_returns_none(self):
        server = make_server(detection(confidence=0.49), age_s=0.0)
        assert server.get_active_target(timeout_s=10.0, min_confidence=0.5) is None

    def test_confidence_exactly_at_floor_passes(self):
        """>= semantics, matching the former controller-side comparison."""
        server = make_server(detection(confidence=0.5), age_s=0.0)
        target = server.get_active_target(timeout_s=10.0, min_confidence=0.5)
        assert target is not None
        assert target.confidence == pytest.approx(0.5)

    def test_missing_confidence_field_defaults_to_zero_and_fails_floor(self):
        payload = detection()
        del payload["confidence"]
        server = make_server(payload, age_s=0.0)
        assert server.get_active_target(timeout_s=10.0, min_confidence=0.5) is None

    def test_missing_direction_vector_yields_zero_components(self):
        server = make_server(
            {"class_id": 2, "confidence": 0.9}, age_s=0.0
        )
        target = server.get_active_target(timeout_s=10.0, min_confidence=0.5)
        assert target is not None
        assert (target.dir_x, target.dir_y) == (0.0, 0.0)

    def test_latest_detection_dict_is_not_mutated(self):
        payload = detection()
        server = make_server(payload, age_s=0.0)
        before = {key: value for key, value in payload.items()}
        server.get_active_target(timeout_s=10.0, min_confidence=0.5)
        assert payload == before
