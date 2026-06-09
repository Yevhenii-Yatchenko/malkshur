"""Unit tests for ShiftEvaluator._apply_deadband (sky_anchor vision pipeline).

``_apply_deadband`` is a staticmethod with explicit threshold arguments, so it
is tested directly without constructing a ShiftEvaluator (whose __init__ pulls
in a ShiftEstimator and env-based config).  Importing the module is side-effect
free apart from reading environment/.env configuration.

Semantics pinned down here (current behavior):
- |value| <= threshold_low   -> 0                      (inclusive deadband)
- |value| >= threshold_high  -> +/- int(threshold_high) (inclusive clamp)
- otherwise                  -> int(value)              (truncation toward zero)
"""

import pytest

from sky_anchor.app.vision.evaluator import ShiftEvaluator

pytestmark = [pytest.mark.unit]

# Threshold pairs as built in ShiftEvaluator.__init__: high = low * 100.
SHIFT_LOW, SHIFT_HIGH = 1.5, 150.0
ANGLE_LOW, ANGLE_HIGH = 3.0, 300.0


class TestDeadbandZone:
    @pytest.mark.parametrize(
        "value",
        [0.0, 0.5, 1.0, 1.4999, 1.5, -0.5, -1.4999, -1.5],
    )
    def test_values_inside_deadband_return_zero(self, value):
        assert ShiftEvaluator._apply_deadband(value, SHIFT_LOW, SHIFT_HIGH) == 0

    def test_low_boundary_is_inclusive_both_signs(self):
        assert ShiftEvaluator._apply_deadband(SHIFT_LOW, SHIFT_LOW, SHIFT_HIGH) == 0
        assert ShiftEvaluator._apply_deadband(-SHIFT_LOW, SHIFT_LOW, SHIFT_HIGH) == 0


class TestPassThroughZone:
    @pytest.mark.parametrize(
        "value, expected",
        [
            (1.51, 1),      # quirk: int() truncation, almost-deadband becomes 1
            (2.0, 2),
            (5.7, 5),       # truncation, not rounding
            (149.999, 149),
            (-1.51, -1),
            (-2.0, -2),
            (-5.7, -5),     # truncation toward zero, not floor (-5, not -6)
            (-149.999, -149),
        ],
    )
    def test_values_between_thresholds_are_truncated_ints(self, value, expected):
        assert ShiftEvaluator._apply_deadband(value, SHIFT_LOW, SHIFT_HIGH) == expected

    def test_sign_is_preserved(self):
        positive = ShiftEvaluator._apply_deadband(42.7, SHIFT_LOW, SHIFT_HIGH)
        negative = ShiftEvaluator._apply_deadband(-42.7, SHIFT_LOW, SHIFT_HIGH)
        assert positive == 42
        assert negative == -42


class TestClampZone:
    @pytest.mark.parametrize(
        "value, expected",
        [
            (150.0, 150),    # high boundary is inclusive
            (151.0, 150),
            (1e6, 150),
            (-150.0, -150),
            (-151.0, -150),
            (-1e6, -150),
        ],
    )
    def test_values_at_or_beyond_high_threshold_clamp(self, value, expected):
        assert ShiftEvaluator._apply_deadband(value, SHIFT_LOW, SHIFT_HIGH) == expected


class TestAngleStyleThresholds:
    """Same function as used for angle_deg (low=ANGLE_THRESHOLD, high=low*100)."""

    @pytest.mark.parametrize(
        "value, expected",
        [
            (2.9, 0),
            (3.0, 0),
            (-3.0, 0),
            (3.5, 3),
            (299.0, 299),
            (300.0, 300),
            (301.0, 300),
            (-301.0, -300),
        ],
    )
    def test_angle_thresholds(self, value, expected):
        assert ShiftEvaluator._apply_deadband(value, ANGLE_LOW, ANGLE_HIGH) == expected


class TestResultType:
    def test_result_is_always_int(self):
        for value in (0.0, 1.0, 2.7, -2.7, 150.0, -150.0, 999.0):
            result = ShiftEvaluator._apply_deadband(value, SHIFT_LOW, SHIFT_HIGH)
            assert isinstance(result, int)
