"""Unit tests for ShiftEvaluator (sky_anchor vision pipeline).

``_apply_deadband`` is a staticmethod with explicit threshold arguments, so it
is tested directly without constructing a ShiftEvaluator.  The __init__
threshold wiring (including the hard-wired ``high = low * 100`` coupling) is
tested separately with only the estimator factory mocked out (the real
__init__ pulls in a ShiftEstimator and env-based config).  Importing the
module is side-effect free apart from reading environment/.env configuration.

Semantics pinned down here (current behavior):
- |value| <= threshold_low   -> 0                      (inclusive deadband)
- |value| >= threshold_high  -> +/- int(threshold_high) (inclusive clamp)
- otherwise                  -> int(value)              (truncation toward zero)
"""

from unittest import mock

import pytest

from sky_anchor.app.config import ANGLE_THRESHOLD, SHIFT_THRESHOLD
from sky_anchor.app.vision.evaluator import ShiftEvaluator

pytestmark = [pytest.mark.unit]

# Local threshold pairs mirroring the SHAPE built in ShiftEvaluator.__init__
# (high = low * 100); the wiring itself is pinned in TestInitThresholdWiring.
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
    @pytest.mark.parametrize("value", [0.0, 1.0, 2.7, -2.7, 150.0, -150.0, 999.0])
    def test_result_is_always_int(self, value):
        result = ShiftEvaluator._apply_deadband(value, SHIFT_LOW, SHIFT_HIGH)
        assert isinstance(result, int)


@pytest.fixture
def wired_evaluator():
    """A real ShiftEvaluator with only the estimator factory mocked out.

    __init__ otherwise runs for real, so the thresholds inspected below are
    the production wiring (bound from sky_anchor.app.config), not a
    re-statement of it.  Returns (evaluator, estimator_mock).
    """
    with mock.patch(
        "sky_anchor.app.vision.evaluator.get_shift_estimator"
    ) as factory:
        evaluator = ShiftEvaluator(logger=mock.Mock())
    return evaluator, factory.return_value


class TestInitThresholdWiring:
    """Pin __init__'s threshold coupling (evaluator.py):

        _shift_threshold_low  = SHIFT_THRESHOLD
        _shift_threshold_high = SHIFT_THRESHOLD * 100

    The clamp is hard-wired to 100x the configured deadband (and evaluate()
    derives the angle clamp the same way, inline), so changing
    DRONE_SHIFT_THRESHOLD/DRONE_ANGLE_THRESHOLD silently moves BOTH ends of
    the band.  Quirk pinned, not endorsed.
    """

    def test_shift_thresholds_wired_from_config_with_high_100x_low(
        self, wired_evaluator
    ):
        evaluator, _ = wired_evaluator
        assert evaluator._shift_threshold_low == SHIFT_THRESHOLD
        assert evaluator._shift_threshold_high == SHIFT_THRESHOLD * 100

    def test_angle_threshold_wired_from_config(self, wired_evaluator):
        evaluator, _ = wired_evaluator
        assert evaluator._angle_threshold == ANGLE_THRESHOLD

    def test_evaluate_clamps_at_100x_the_configured_deadband(self, wired_evaluator):
        evaluator, estimator = wired_evaluator
        estimator.estimate_shift.return_value = (
            SHIFT_THRESHOLD * 100 + 1000.0,     # dx far beyond the clamp
            -(SHIFT_THRESHOLD * 100 + 1000.0),  # dy far beyond, negative
            ANGLE_THRESHOLD * 100 + 1000.0,     # angle far beyond its clamp
            87.6,
        )
        command = evaluator.evaluate(reference=mock.Mock(), current=mock.Mock())
        assert command.dx == int(SHIFT_THRESHOLD * 100)
        assert command.dy == -int(SHIFT_THRESHOLD * 100)
        assert command.angle_deg == int(ANGLE_THRESHOLD * 100)
        assert command.matches_percent == 87  # int() truncation on the way out

    def test_evaluate_zeroes_values_at_the_configured_deadband(self, wired_evaluator):
        evaluator, estimator = wired_evaluator
        estimator.estimate_shift.return_value = (
            SHIFT_THRESHOLD,    # inclusive low boundary -> 0
            -SHIFT_THRESHOLD,
            ANGLE_THRESHOLD,
            99.0,
        )
        command = evaluator.evaluate(reference=mock.Mock(), current=mock.Mock())
        assert command.dx == 0
        assert command.dy == 0
        assert command.angle_deg == 0
