"""Unit tests for the frozen config objects (src/config/objects.py).

GRASP Step 7 (LC-2): the dataclasses are constructed FROM the existing
``altitude_config.py`` / ``position_config.py`` dicts -- never hand-copied --
so the contract pinned here is field-by-field equality with those dicts
(any drift between the two representations is a bug by definition) plus
immutability, and that the controllers actually consume the objects:

- default construction (no config argument) wires the exact dict values,
  byte-identical to the pre-Step-7 import-time defaults;
- an injected config is respected, including PositionController's
  ``__enable_stabilization`` gain restore, which used to reach back into
  the global POSITION_PID_X/Y dicts and now restores from the injected
  object.
"""

import dataclasses
from unittest import mock

import pytest

from src.altitude_config import (
    ALTITUDE_PID_TAKEOFF,
    FILTERING,
    LIMITS,
    THROTTLE,
    VELOCITY_PID_FLIGHT,
)
from src.config.objects import AltitudeConfig, PIDGains, PositionConfig
from src.pid_controller import AltitudeController
from src.position_config import (
    ANGLE_PID,
    POSITION_PID_X,
    POSITION_PID_Y,
    PWM_LIMITS,
)
from src.position_controller import PositionController

pytestmark = [pytest.mark.unit]


class TestPIDGains:
    def test_from_dict_maps_the_three_keys(self):
        gains = PIDGains.from_dict({"kp": 1.5, "ki": 0.2, "kd": 0.7})
        assert (gains.kp, gains.ki, gains.kd) == (1.5, 0.2, 0.7)

    def test_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            PIDGains(kp=1, ki=2, kd=3).kp = 9


class TestAltitudeConfigEqualsDicts:
    """from_dicts() must mirror altitude_config.py exactly, field by field."""

    def test_values_match_the_dicts(self):
        config = AltitudeConfig.from_dicts()
        assert config.altitude_pid == PIDGains.from_dict(ALTITUDE_PID_TAKEOFF)
        assert config.velocity_pid == PIDGains.from_dict(VELOCITY_PID_FLIGHT)
        assert config.max_velocity == LIMITS["max_velocity"]
        assert config.max_acceleration == LIMITS["max_acceleration"]
        assert config.throttle_hover == THROTTLE["hover"]
        assert config.throttle_min == THROTTLE["min"]
        assert config.throttle_max == THROTTLE["max"]
        assert config.altitude_filter_alpha == FILTERING["altitude_filter_alpha"]
        assert config.velocity_filter_size == FILTERING["velocity_filter_size"]

    def test_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            AltitudeConfig.from_dicts().throttle_max = 2000


class TestPositionConfigEqualsDicts:
    """from_dicts() must mirror position_config.py exactly, field by field."""

    def test_values_match_the_dicts(self):
        config = PositionConfig.from_dicts()
        assert config.pid_x == PIDGains.from_dict(POSITION_PID_X)
        assert config.pid_y == PIDGains.from_dict(POSITION_PID_Y)
        assert config.angle_pid == PIDGains.from_dict(ANGLE_PID)
        assert config.max_correction == PWM_LIMITS["max_correction"]

    def test_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            PositionConfig.from_dicts().max_correction = 1


class TestAltitudeControllerConsumesConfig:
    def test_default_construction_wires_the_dict_values(self):
        """No config argument -> same wiring as the former import-time
        default arguments (this is what keeps the characterization golden
        master byte-identical)."""
        controller = AltitudeController(csv_logger=mock.Mock())
        assert controller.position_pid.kp == ALTITUDE_PID_TAKEOFF["kp"]
        assert controller.position_pid.ki == ALTITUDE_PID_TAKEOFF["ki"]
        assert controller.position_pid.kd == ALTITUDE_PID_TAKEOFF["kd"]
        assert controller.position_pid.output_max == LIMITS["max_velocity"]
        assert controller.position_pid.output_min == -LIMITS["max_velocity"]
        assert controller.velocity_pid.kp == VELOCITY_PID_FLIGHT["kp"]
        assert controller.velocity_pid.ki == VELOCITY_PID_FLIGHT["ki"]
        assert controller.velocity_pid.kd == VELOCITY_PID_FLIGHT["kd"]
        assert controller.throttle_hover == THROTTLE["hover"]
        assert controller.throttle_min == THROTTLE["min"]
        assert controller.throttle_max == THROTTLE["max"]
        assert controller.last_throttle == THROTTLE["hover"]
        assert controller.max_velocity == LIMITS["max_velocity"]
        assert controller.max_acceleration == LIMITS["max_acceleration"]
        assert (
            controller.altitude_filter_alpha
            == FILTERING["altitude_filter_alpha"]
        )
        assert (
            controller.velocity_history.maxlen
            == FILTERING["velocity_filter_size"]
        )

    def test_injected_config_is_respected(self):
        config = AltitudeConfig(
            altitude_pid=PIDGains(kp=9.0, ki=8.0, kd=7.0),
            velocity_pid=PIDGains(kp=6.0, ki=5.0, kd=4.0),
            max_velocity=2.0,
            max_acceleration=3.0,
            throttle_hover=1555,
            throttle_min=1111,
            throttle_max=1777,
            altitude_filter_alpha=0.5,
            velocity_filter_size=7,
        )
        controller = AltitudeController(config=config, csv_logger=mock.Mock())
        assert controller.position_pid.kp == 9.0
        assert controller.position_pid.output_max == 2.0
        assert controller.velocity_pid.kp == 6.0
        assert controller.throttle_hover == 1555
        assert controller.throttle_min == 1111
        assert controller.throttle_max == 1777
        assert controller.altitude_filter_alpha == 0.5
        assert controller.velocity_history.maxlen == 7


class TestPositionControllerConsumesConfig:
    def test_default_construction_wires_the_dict_values(self):
        controller = PositionController(csv_logger=mock.Mock())
        assert controller.position_pid_x.kp == POSITION_PID_X["kp"]
        assert controller.position_pid_x.ki == POSITION_PID_X["ki"]
        assert controller.position_pid_x.kd == POSITION_PID_X["kd"]
        assert controller.position_pid_y.kp == POSITION_PID_Y["kp"]
        assert controller.angle_pid.kp == ANGLE_PID["kp"]
        assert controller.angle_pid.ki == ANGLE_PID["ki"]
        assert controller.angle_pid.kd == ANGLE_PID["kd"]
        assert (
            controller.position_pid_x.output_max
            == PWM_LIMITS["max_correction"]
        )
        assert (
            controller.angle_pid.output_min
            == -PWM_LIMITS["max_correction"]
        )

    def test_injected_config_is_respected(self):
        config = PositionConfig(
            pid_x=PIDGains(kp=2.0, ki=0.9, kd=0.8),
            pid_y=PIDGains(kp=3.0, ki=0.7, kd=0.6),
            angle_pid=PIDGains(kp=4.0, ki=0.5, kd=0.4),
            max_correction=50,
        )
        controller = PositionController(config=config, csv_logger=mock.Mock())
        assert controller.position_pid_x.kp == 2.0
        assert controller.position_pid_y.kp == 3.0
        assert controller.angle_pid.kp == 4.0
        assert controller.position_pid_x.output_max == 50
        assert controller.position_pid_x.output_min == -50

    def test_mode_switch_restores_gains_from_injected_config(self):
        """``__enable_stabilization`` used to read the global
        POSITION_PID_X/Y dicts; it must restore from the injected config
        (identical values for the default config, so behavior is
        unchanged in production)."""
        config = PositionConfig(
            pid_x=PIDGains(kp=2.0, ki=0.9, kd=0.8),
            pid_y=PIDGains(kp=3.0, ki=0.7, kd=0.6),
            angle_pid=PIDGains(kp=4.0, ki=0.5, kd=0.4),
            max_correction=50,
        )
        controller = PositionController(config=config, csv_logger=mock.Mock())

        # navigation=True zeroes the position ki/kd (unchanged behavior)...
        controller.update(
            dx_pixels=1.0, dy_pixels=1.0, angle_deg=0.0, confidence=0.9,
            altitude=5.0, current_time=100.0, navigation=True,
        )
        assert controller.position_pid_x.ki == 0.0
        assert controller.position_pid_x.kd == 0.0
        assert controller.position_pid_y.ki == 0.0

        # ... and the switch back restores the INJECTED gains, not the
        # global dicts.
        controller.update(
            dx_pixels=1.0, dy_pixels=1.0, angle_deg=0.0, confidence=0.9,
            altitude=5.0, current_time=100.1, navigation=False,
        )
        assert controller.position_pid_x.ki == 0.9
        assert controller.position_pid_x.kd == 0.8
        assert controller.position_pid_y.ki == 0.7
        assert controller.position_pid_y.kd == 0.6

    def test_mode_state_is_per_instance(self):
        """Step 7 carried bullet: ``__mode`` migrated from a class attribute
        to an instance attribute in __init__."""
        first = PositionController(csv_logger=mock.Mock())
        second = PositionController(csv_logger=mock.Mock())
        first.update(
            dx_pixels=1.0, dy_pixels=1.0, angle_deg=0.0, confidence=0.9,
            altitude=5.0, current_time=100.0, navigation=True,
        )
        # The first instance is in navigation mode (ki zeroed); the second
        # must be untouched.
        assert first.position_pid_x.ki == 0.0
        assert second.position_pid_x.ki == POSITION_PID_X["ki"]
        assert "_PositionController__mode" in vars(first)
        assert "_PositionController__mode" in vars(second)
