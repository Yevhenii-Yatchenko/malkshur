"""Unit tests for RCSetpoints (src/flight/setpoints.py).

Step 6 (IE-4) replaced DroneController's four loose RC PWM base attributes,
its altitude-target attribute, and the ``__set_throttle_base`` helper whose
hardcoded ``1800`` duplicated ``THROTTLE['max']``.  Contract pinned here:

- initial state is the controller's former class-attribute defaults
  (neutral attitude, idle throttle, 0.2 m target);
- throttle: upper clamp at THROTTLE['max'] with the former ``>=`` semantics,
  and -- deliberately -- NO lower clamp: the controller level never had one
  (THROTTLE['min'] is enforced inside AltitudeController.update, which stays
  its owner), so RCSetpoints must not invent it;
- roll/pitch/yaw: NO clamping at all, exactly like the former bare
  attributes (the PWM_LIMITS window is owned by PositionController's
  np.clip on its own outputs);
- every PWM setter normalizes to built-in ``int``; for the live writers
  this is a no-op -- pinned against MAVLink's struct packing for the
  numpy-int values PositionController actually produces;
- target_altitude is plain pass-through state (no coercion, no limits);
- apply(): the former DroneController.__apply_setpoints -- roll/pitch/yaw
  always, target_altitude only when the behavior set one (None means
  "leave unchanged"), throttle never.
"""

import struct

import numpy as np
import pytest

from src.altitude_config import THROTTLE
from src.domain.types import AttitudeSetpoints
from src.flight.setpoints import RCSetpoints
from src.position_config import PWM_LIMITS

pytestmark = [pytest.mark.unit]


@pytest.fixture
def rc():
    return RCSetpoints()


class TestConfigAssumptions:
    """Pin the config values the limit literals below depend on."""

    def test_throttle_limits(self):
        assert THROTTLE['max'] == 1800
        assert THROTTLE['min'] == 1000


class TestInitialState:
    """The former DroneController class-attribute defaults, verbatim."""

    def test_initial_values(self, rc):
        assert rc.roll == 1500
        assert rc.pitch == 1500
        assert rc.yaw == 1500
        assert rc.throttle == 1000
        assert rc.target_altitude == 0.2

    def test_initial_pwm_values_are_builtin_ints(self, rc):
        for value in (rc.roll, rc.pitch, rc.yaw, rc.throttle):
            assert type(value) is int


class TestThrottleUpperClamp:
    """The former ``if value >= 1800: value = 1800``, now THROTTLE['max']."""

    @pytest.mark.parametrize(
        "value",
        [1801, 1900, 2000, 10_000],
    )
    def test_above_max_clamps_to_config_max(self, rc, value):
        rc.throttle = value
        assert rc.throttle == THROTTLE['max'] == 1800

    def test_exactly_max_stays_at_max(self, rc):
        # >= semantics: the boundary itself maps onto the max (a numeric
        # no-op, kept for exactness with the old helper).
        rc.throttle = THROTTLE['max']
        assert rc.throttle == THROTTLE['max']

    def test_just_below_max_passes_through(self, rc):
        rc.throttle = THROTTLE['max'] - 1
        assert rc.throttle == 1799

    @pytest.mark.parametrize("value", [1000, 1300, 1500, 1799])
    def test_normal_range_passes_through(self, rc, value):
        rc.throttle = value
        assert rc.throttle == value


class TestThrottleCeilingInjection:
    """Step 7 carried bullet: the ceiling is a constructor parameter that
    the composition root passes explicitly, defaulting to THROTTLE['max']
    (so a bare RCSetpoints() behaves exactly as before -- pinned by
    TestThrottleUpperClamp above)."""

    def test_custom_ceiling_clamps_above(self):
        rc = RCSetpoints(throttle_max=1700)
        rc.throttle = 1701
        assert rc.throttle == 1700

    def test_custom_ceiling_keeps_ge_semantics(self):
        """Exactly AT the ceiling already clamps (>=, as the former
        hardcoded ``value >= 1800`` did)."""
        rc = RCSetpoints(throttle_max=1700)
        rc.throttle = 1700
        assert rc.throttle == 1700

    def test_below_custom_ceiling_passes_through(self):
        rc = RCSetpoints(throttle_max=1700)
        rc.throttle = 1699
        assert rc.throttle == 1699


class TestThrottleHasNoLowerClampHere:
    """The controller level never clamped the throttle floor; the owner of
    THROTTLE['min'] is AltitudeController.update (np.clip on its output).
    RCSetpoints replicating a min clamp would duplicate that owner."""

    @pytest.mark.parametrize("value", [999, 900, 0])
    def test_below_config_min_passes_through(self, rc, value):
        rc.throttle = value
        assert rc.throttle == value


class TestRollPitchYawAreNotClamped:
    """Exactly like the former bare attributes: no controller-level window.
    PositionController already clips its own outputs to PWM_LIMITS before
    they ever reach RCSetpoints; intercept offsets were never clamped."""

    @pytest.mark.parametrize("attr", ["roll", "pitch", "yaw"])
    @pytest.mark.parametrize("value", [800, 1399, 1601, 2300])
    def test_values_outside_position_window_pass_through(self, rc, attr, value):
        # Sanity: pick values outside the PositionController window so a
        # sneaky clamp would be caught.
        assert value < PWM_LIMITS['min_roll'] or value > PWM_LIMITS['max_roll']
        setattr(rc, attr, value)
        assert getattr(rc, attr) == value


class TestIntNormalization:
    """All PWM setters return built-in ints; for the live writers (telnet
    ints, intercept Python ints, PositionController numpy ints, the altitude
    PID's int(throttle)) this is value-preserving."""

    @pytest.mark.parametrize("attr", ["roll", "pitch", "yaw", "throttle"])
    def test_numpy_int_is_normalized_to_builtin_int(self, rc, attr):
        setattr(rc, attr, np.int64(1510))
        value = getattr(rc, attr)
        assert type(value) is int
        assert value == 1510

    def test_position_controller_style_clip_output_round_trips(self, rc):
        """The real stabilization path: int() of a float adjustment, then
        np.clip against PWM_LIMITS -- a numpy.int64 by the time it reaches
        RCSetpoints."""
        clipped = np.clip(
            int(PWM_LIMITS['neutral'] + 12.3),
            PWM_LIMITS['min_roll'],
            PWM_LIMITS['max_roll'],
        )
        assert isinstance(clipped, np.integer)
        rc.roll = clipped
        assert type(rc.roll) is int
        assert rc.roll == int(clipped) == 1512

    def test_normalized_value_packs_identically_to_numpy_original(self, rc):
        """The MAVLink no-op proof: pymavlink packs the RC channels with
        struct ('<H' uint16 fields), which consumes numpy ints via
        __index__ -- the bytes are identical before and after int()."""
        original = np.int64(1476)
        rc.roll = original
        assert struct.pack('<H', rc.roll) == struct.pack('<H', original)

    def test_numpy_throttle_above_max_clamps_then_normalizes(self, rc):
        rc.throttle = np.int64(2000)
        assert type(rc.throttle) is int
        assert rc.throttle == THROTTLE['max']


class TestTargetAltitude:
    """Plain state: the controller's former __target_altitude attribute."""

    def test_set_and_get_float(self, rc):
        rc.target_altitude = 5.0
        assert rc.target_altitude == 5.0

    def test_no_coercion_or_limits(self, rc):
        # _setHeight float()s its telnet param before setting; the landing
        # path assigns 0.1; intercept feeds floats back.  RCSetpoints adds
        # no coercion of its own (and no min/max -- none existed).
        rc.target_altitude = 0.1
        assert rc.target_altitude == 0.1
        rc.target_altitude = 50.0
        assert rc.target_altitude == 50.0


class TestApply:
    """The former DroneController.__apply_setpoints, moved to the owner."""

    def test_applies_attitude_and_altitude(self, rc):
        rc.apply(AttitudeSetpoints(
            roll_pwm=1480, pitch_pwm=1520, yaw_pwm=1530, target_altitude=4.2,
        ))
        assert (rc.roll, rc.pitch, rc.yaw) == (1480, 1520, 1530)
        assert rc.target_altitude == 4.2

    def test_none_target_altitude_leaves_current_target_unchanged(self, rc):
        rc.target_altitude = 5.0
        rc.apply(AttitudeSetpoints(roll_pwm=1500, pitch_pwm=1500, yaw_pwm=1500))
        assert rc.target_altitude == 5.0

    def test_apply_never_touches_throttle(self, rc):
        rc.throttle = 1444
        rc.apply(AttitudeSetpoints(
            roll_pwm=1480, pitch_pwm=1520, yaw_pwm=1530, target_altitude=4.2,
        ))
        assert rc.throttle == 1444

    def test_apply_normalizes_numpy_setpoints_to_builtin_ints(self, rc):
        """StabilizationBehavior's setpoints carry numpy ints straight from
        np.clip; after apply() the stored bases are built-in ints with the
        same values."""
        rc.apply(AttitudeSetpoints(
            roll_pwm=np.int64(1476),
            pitch_pwm=np.int64(1512),
            yaw_pwm=np.int64(1500),
        ))
        assert (rc.roll, rc.pitch, rc.yaw) == (1476, 1512, 1500)
        for value in (rc.roll, rc.pitch, rc.yaw):
            assert type(value) is int
