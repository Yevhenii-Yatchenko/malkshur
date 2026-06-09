"""Unit tests for PositionController.pixels_to_meters (src/position_controller.py).

CAUTION (LC-1 in the refactoring plan): PositionController.__init__ eagerly
creates a PositionCSVLogger (mkdir + open file under logs/csv/) and a file
logger via get_logger (logs/position_controller.log).  Production code must
not change in Step 2, so both are patched in the module namespace; the rest
of construction (the three real PIDControllers, config binding) runs for real.

The expected values below are literals on purpose: with the current
position_config.ALTITUDE_COMPENSATION (reference_altitude=2.0,
pixels_per_meter_at_ref=500, adaptive scaling on, altitude clipped to
[0.5, 10.0]) the conversion is

    meters = pixels * clip(altitude, 0.5, 10.0) / 1000.0

A change to either the formula or the config is a behavior change and should
fail these tests.
"""

from unittest import mock

import pytest

from src.position_config import ALTITUDE_COMPENSATION
from src.position_controller import PositionController

pytestmark = [pytest.mark.unit, pytest.mark.pid]


@pytest.fixture
def controller():
    with mock.patch("src.position_controller.PositionCSVLogger") as csv_cls, \
            mock.patch("src.position_controller.get_logger", return_value=mock.Mock()):
        instance = PositionController()
        # Sanity: the file-writing logger really was replaced by the mock.
        assert instance.csv_logger is csv_cls.return_value
        yield instance


class TestConfigAssumptions:
    """Pin the config branch the golden numbers below depend on."""

    def test_adaptive_scaling_is_enabled(self):
        assert ALTITUDE_COMPENSATION["use_adaptive_scaling"] is True

    def test_reference_calibration(self):
        assert ALTITUDE_COMPENSATION["reference_altitude"] == pytest.approx(2.0)
        assert ALTITUDE_COMPENSATION["pixels_per_meter_at_ref"] == 500
        assert ALTITUDE_COMPENSATION["min_altitude"] == pytest.approx(0.5)
        assert ALTITUDE_COMPENSATION["max_altitude"] == pytest.approx(10.0)


class TestPixelsToMeters:
    @pytest.mark.parametrize(
        "pixels, altitude, expected_meters",
        [
            # Known conversions at the reference altitude (2.0 m):
            # 500 px == 1 m, so 100 px == 0.2 m.
            (100.0, 2.0, 0.2),
            (500.0, 2.0, 1.0),
            (1234.0, 2.0, 2.468),
            # Scaling is linear in altitude.
            (100.0, 1.0, 0.1),
            (100.0, 4.0, 0.4),
            (250.0, 8.0, 2.0),
            # Zero pixels -> zero meters at any altitude.
            (0.0, 2.0, 0.0),
            (0.0, 9.0, 0.0),
            (0.0, 0.0, 0.0),
            # Negative drift converts symmetrically.
            (-100.0, 2.0, -0.2),
            (-250.0, 4.0, -1.0),
            (-1.0, 2.0, -0.002),
        ],
    )
    def test_known_conversions(self, controller, pixels, altitude, expected_meters):
        result = controller.pixels_to_meters(pixels, altitude)
        assert result == pytest.approx(expected_meters)

    @pytest.mark.parametrize(
        "pixels, altitude, expected_meters",
        [
            # Altitude is clipped to [0.5, 10.0] before scaling.
            (100.0, 0.0, 0.05),     # below min -> behaves like 0.5 m
            (100.0, 0.25, 0.05),
            (100.0, -3.0, 0.05),    # negative altitude also clips to 0.5 m
            (100.0, 50.0, 1.0),     # above max -> behaves like 10.0 m
            # Exact boundary values pass through unclipped.
            (100.0, 0.5, 0.05),
            (100.0, 10.0, 1.0),
        ],
    )
    def test_altitude_clipping(self, controller, pixels, altitude, expected_meters):
        result = controller.pixels_to_meters(pixels, altitude)
        assert result == pytest.approx(expected_meters)

    def test_linear_in_pixels_at_fixed_altitude(self, controller):
        base = controller.pixels_to_meters(10.0, 3.0)
        assert controller.pixels_to_meters(30.0, 3.0) == pytest.approx(3.0 * base)
        assert controller.pixels_to_meters(-10.0, 3.0) == pytest.approx(-base)
