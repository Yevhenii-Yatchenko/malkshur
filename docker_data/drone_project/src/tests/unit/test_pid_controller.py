"""Unit tests for the real PIDController math (src/pid_controller.py).

Part of the GRASP refactoring safety net (REFACTORING_PLAN.md Step 2):
these tests pin down the CURRENT behavior of the controller before any
production code changes.  Some asserted behaviors are quirks rather than
ideals (e.g. the dt=0.1 first-iteration assumption) -- they are documented
on purpose so refactoring-induced changes are caught.

All tests are fully deterministic: time is injected via the ``current_time``
parameter, no wall clock, no I/O, no mocks of the math.

Step 3 (LC-1) adds TestAltitudeControllerCsvLoggerInjection: construction
wiring tests for AltitudeController's new ``csv_logger=`` parameter (only the
file-writing logger class is mocked there -- never the math).
"""

from unittest import mock

import pytest

from src.pid_controller import AltitudeController, PIDController

pytestmark = [pytest.mark.unit, pytest.mark.pid]


class TestStepResponse:
    """Proportional response: direction and magnitude."""

    def test_positive_error_gives_positive_output(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        # error = 5.0 - 3.0 = 2.0 -> p_term = 2.0 * 2.0
        assert pid.update(5.0, 3.0, current_time=0.0) == pytest.approx(4.0)

    def test_negative_error_gives_negative_output(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        # Measurement above setpoint -> controller must push down.
        assert pid.update(3.0, 5.0, current_time=0.0) == pytest.approx(-4.0)

    def test_output_scales_linearly_with_error(self):
        small = PIDController(kp=2.0, ki=0.0, kd=0.0).update(1.0, 0.0, current_time=0.0)
        large = PIDController(kp=2.0, ki=0.0, kd=0.0).update(4.0, 0.0, current_time=0.0)
        assert large == pytest.approx(4.0 * small)

    def test_zero_error_gives_zero_output(self):
        pid = PIDController(kp=5.0, ki=0.0, kd=0.0)
        assert pid.update(2.5, 2.5, current_time=0.0) == pytest.approx(0.0)


class TestFirstIterationDt:
    """Documented quirk: the first update assumes dt = 0.1 s (10 Hz)."""

    def test_first_update_assumes_dt_of_100ms(self):
        # Integral-only controller: output = ki * error * dt exposes dt.
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        output = pid.update(2.0, 0.0, current_time=42.0)
        assert output == pytest.approx(1.0 * 2.0 * 0.1)
        assert pid.integral == pytest.approx(0.2)

    def test_non_positive_dt_falls_back_to_100ms(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        assert pid.update(2.0, 0.0, current_time=10.0) == pytest.approx(0.2)
        # Same timestamp again (dt == 0) -> dt forced back to 0.1.
        assert pid.update(2.0, 0.0, current_time=10.0) == pytest.approx(0.4)
        # Time going backwards (dt < 0) -> dt forced back to 0.1.
        assert pid.update(2.0, 0.0, current_time=9.0) == pytest.approx(0.6)


class TestDerivative:
    """Derivative term: first-call suppression and moving-average filtering."""

    def test_derivative_is_zero_on_first_update(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=10.0)
        # Large error but no error history yet -> d_term must be 0.
        assert pid.update(100.0, 0.0, current_time=0.0) == pytest.approx(0.0)

    def test_derivative_responds_to_error_change(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=2.0)
        pid.update(0.0, 0.0, current_time=0.0)          # error 0.0
        output = pid.update(0.0, -1.0, current_time=0.5)  # error 1.0
        # derivative = (1.0 - 0.0) / 0.5 = 2.0 -> d_term = 2.0 * 2.0
        assert output == pytest.approx(4.0)

    def test_derivative_is_moving_average_filtered(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=2.0, derivative_filter_size=5)
        pid.update(0.0, 0.0, current_time=0.0)
        pid.update(0.0, -1.0, current_time=0.5)           # derivative 2.0
        output = pid.update(0.0, -1.0, current_time=1.0)  # derivative 0.0
        # Filtered derivative = mean([2.0, 0.0]) = 1.0 -> d_term = 2.0 * 1.0
        assert output == pytest.approx(2.0)


class TestOutputClamping:
    def test_output_clamped_to_default_max(self):
        pid = PIDController(kp=50.0, ki=0.0, kd=0.0)  # defaults: +/-100
        assert pid.update(10.0, 0.0, current_time=0.0) == pytest.approx(100.0)

    def test_output_clamped_to_default_min(self):
        pid = PIDController(kp=50.0, ki=0.0, kd=0.0)
        assert pid.update(0.0, 10.0, current_time=0.0) == pytest.approx(-100.0)

    def test_output_clamped_to_custom_limits(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-7.5, output_max=7.5)
        assert pid.update(2.0, 0.0, current_time=0.0) == pytest.approx(7.5)
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-7.5, output_max=7.5)
        assert pid.update(0.0, 2.0, current_time=0.0) == pytest.approx(-7.5)

    def test_output_exactly_at_limit_passes_through(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, output_min=-2.0, output_max=2.0)
        assert pid.update(2.0, 0.0, current_time=0.0) == pytest.approx(2.0)


class TestAntiWindup:
    def test_integral_clamped_to_integral_limit(self):
        # Clamp bound is integral_limit / ki = 5.0; i_term saturates at
        # ki * 5.0 = integral_limit = 10.0.
        pid = PIDController(kp=0.0, ki=2.0, kd=0.0, integral_limit=10.0)
        outputs = [
            pid.update(5.0, 0.0, current_time=0.5 * step) for step in range(10)
        ]
        # Unclamped, the integral would reach 0.5 + 9 * 2.5 = 23.0.
        assert pid.integral == pytest.approx(5.0)
        assert outputs[-1] == pytest.approx(10.0)
        assert max(outputs) == pytest.approx(10.0)

    def test_integration_freezes_while_saturated_high(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0, output_min=-5.0, output_max=5.0,
                            integral_limit=50.0)
        out1 = pid.update(10.0, 0.0, current_time=0.0)
        assert out1 == pytest.approx(5.0)            # saturated at output_max
        assert pid.integral == pytest.approx(1.0)    # first step integrated (10 * 0.1)
        out2 = pid.update(10.0, 0.0, current_time=0.1)
        # Previous output was at the limit and error still positive ->
        # conditional integration must freeze the integral.
        assert pid.integral == pytest.approx(1.0)
        assert out2 == pytest.approx(5.0)

    def test_integration_freezes_while_saturated_low(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0, output_min=-5.0, output_max=5.0,
                            integral_limit=50.0)
        out1 = pid.update(-10.0, 0.0, current_time=0.0)
        assert out1 == pytest.approx(-5.0)
        assert pid.integral == pytest.approx(-1.0)
        out2 = pid.update(-10.0, 0.0, current_time=0.1)
        assert pid.integral == pytest.approx(-1.0)
        assert out2 == pytest.approx(-5.0)


class TestReset:
    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.update(5.0, 0.0, current_time=0.0)
        pid.update(5.0, 1.0, current_time=0.1)
        pid.reset()
        assert pid.integral == 0.0
        assert pid.last_error is None
        assert pid.last_time is None
        assert len(pid.derivative_history) == 0
        assert len(pid.error_history) == 0

    def test_update_after_reset_behaves_like_first_update(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        pid.update(3.0, 0.0, current_time=0.0)
        pid.update(3.0, 0.0, current_time=1.0)
        pid.reset()
        # First update after reset uses the documented dt = 0.1 assumption.
        assert pid.update(2.0, 0.0, current_time=100.0) == pytest.approx(0.2)


class TestAltitudeControllerCsvLoggerInjection:
    """Step 3 (LC-1): the ``csv_logger=`` constructor injection contract.

    - csv_logger omitted/None -> a file-writing AltitudeCSVLogger is created
      exactly as before (backward compatibility; the class is patched here so
      the test itself stays free of file I/O);
    - csv_logger injected -> NO AltitudeCSVLogger is constructed (no file
      I/O) and update() routes its log write to the injected object
      (DEBUG['plot_data'] is True in altitude_config, so update() logs on
      every call).
    """

    def test_default_still_creates_file_logger_exactly_as_before(self):
        ts = mock.sentinel.start_timestamp
        with mock.patch("src.pid_controller.AltitudeCSVLogger") as csv_cls:
            controller = AltitudeController(start_timestamp=ts)
        csv_cls.assert_called_once_with(
            start_timestamp=ts, controller_type='takeoff'
        )
        assert controller.csv_logger is csv_cls.return_value

    def test_injected_logger_is_used_and_no_file_logger_is_created(self):
        fake = mock.Mock(name="injected_csv_logger")
        with mock.patch("src.pid_controller.AltitudeCSVLogger") as csv_cls:
            controller = AltitudeController(csv_logger=fake)
        csv_cls.assert_not_called()
        assert controller.csv_logger is fake

    def test_update_logs_to_the_injected_logger(self):
        from src.altitude_config import DEBUG

        # Precondition for the call-count assert below.
        assert DEBUG.get("plot_data") is True
        fake = mock.Mock(name="injected_csv_logger")
        with mock.patch("src.pid_controller.AltitudeCSVLogger") as csv_cls:
            controller = AltitudeController(csv_logger=fake)
        throttle = controller.update(
            target_altitude=5.0, current_altitude=4.5, current_time=0.0
        )
        csv_cls.assert_not_called()
        assert isinstance(throttle, int)
        assert fake.append.call_count == 1
        logged = fake.append.call_args[0][0]
        assert logged["target_altitude"] == pytest.approx(5.0)
        assert logged["current_altitude"] == pytest.approx(4.5)
        assert logged["throttle_output"] == pytest.approx(controller.last_throttle)
