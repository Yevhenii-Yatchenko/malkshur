"""Golden-master (characterization) tests for the control math.

This is the regression safety net for the GRASP refactoring (Step 2 of
REFACTORING_PLAN.md; deterministic stand-in for the skipped SITL golden runs).

``tests/unit/generate_characterization.py`` is the single source of truth for
the synthetic profiles AND the run functions: this test replays the exact
same code paths used at generation time and asserts the outputs still match
the sequences stored in ``tests/unit/data/altitude_characterization.json``.

Step 3+ migration note (logger injection): the recorded outputs are
logging-independent -- the controllers only ever WRITE to their loggers,
never read from them -- so the construction/patch wiring inside the run
functions could be re-wired during refactoring WITHOUT regenerating the
JSON.  Step 3 did exactly that: the run functions now inject an explicit
``gen.NullCSVLogger`` through the new ``csv_logger=`` constructor parameter
instead of ``mock.patch``-ing the CSV logger classes, and this replay still
asserting against the UNCHANGED recorded sequences is the proof that the
injection refactoring preserved behavior.  Regenerate only when a behavior
change to the control math is intentional and reviewed.  CAUTION: the
backward-compat API (``csv_logger=None`` -> the constructor creates a real
file-writing logger) means the replay must inject an explicit null-object
logger -- passing ``None`` would silently re-enable file I/O.

If these tests fail during refactoring, the control behavior changed.
"""

import json
import os

import pytest

# ``generate_characterization`` is imported as a plain top-level module.
# This relies on pytest's default "prepend" import mode: tests/unit/ has no
# __init__.py, so pytest inserts this directory into sys.path and the
# generator is importable by bare file name.  If the suite ever switches to
# importlib import mode or grows __init__.py files, revisit this import.
import generate_characterization as gen

pytestmark = [pytest.mark.unit, pytest.mark.pid]


@pytest.fixture(scope="module")
def golden():
    if not os.path.exists(gen.DATA_FILE):
        pytest.fail(
            "Golden-master data file missing: {}\n"
            "Generate it (from the project root, inside the container) with:\n"
            "  python3 tests/unit/generate_characterization.py".format(gen.DATA_FILE)
        )
    with open(gen.DATA_FILE) as fh:
        return json.load(fh)


@pytest.fixture(scope="module")
def altitude_replay():
    """One shared replay of the (deterministic) altitude run."""
    return gen.run_altitude_characterization()


@pytest.fixture(scope="module")
def position_replay():
    """One shared replay of the (deterministic) position run."""
    return gen.run_position_characterization()


class TestGoldenMasterFile:
    def test_metadata_is_recorded(self, golden):
        metadata = golden["metadata"]
        assert metadata["generator"] == "tests/unit/generate_characterization.py"
        assert metadata["generated_at"]
        assert metadata["git_sha_at_generation"]
        assert golden["altitude_controller"]["class"] == (
            "src.pid_controller.AltitudeController"
        )
        assert golden["pid_controller"]["class"] == "src.pid_controller.PIDController"
        assert golden["position_controller"]["class"] == (
            "src.position_controller.PositionController"
        )

    def test_pid_constructor_args_match_generator(self, golden):
        # The stored run must describe the same controller the replay builds.
        assert golden["pid_controller"]["constructor_args"] == gen.PID_CONSTRUCTOR_ARGS

    def test_profile_dimensions_match_generator(self, golden):
        alt_profile = golden["altitude_controller"]["profile"]
        assert alt_profile["steps"] == gen.ALTITUDE_STEPS
        assert alt_profile["dt"] == pytest.approx(gen.ALTITUDE_DT)
        assert alt_profile["target_altitude"] == pytest.approx(gen.ALTITUDE_TARGET)
        pid_profile = golden["pid_controller"]["profile"]
        assert pid_profile["steps"] == gen.PID_STEPS
        assert pid_profile["dt"] == pytest.approx(gen.PID_DT)
        pos_profile = golden["position_controller"]["profile"]
        assert pos_profile["steps"] == gen.POSITION_STEPS
        assert pos_profile["dt"] == pytest.approx(gen.POSITION_DT)
        # Segment layout (incl. the exact 1.01 sentinel) must match verbatim.
        assert pos_profile["segments"] == gen.POSITION_SEGMENTS


class TestAltitudeControllerCharacterization:
    def test_input_profile_matches_stored_inputs(self, golden):
        """Distinguish 'profile drifted' from 'behavior drifted'."""
        stored = golden["altitude_controller"]["inputs"]["current_altitude"]
        regenerated = [gen.altitude_profile(step) for step in range(gen.ALTITUDE_STEPS)]
        assert len(stored) == gen.ALTITUDE_STEPS
        assert regenerated == pytest.approx(stored, rel=1e-9)

    def test_replay_matches_recorded_throttle_sequence(self, golden, altitude_replay):
        expected = golden["altitude_controller"]["outputs"]
        actual = altitude_replay["throttle_int"]
        assert len(actual) == gen.ALTITUDE_STEPS == len(expected)
        # Integers compare exactly -- no approx.
        assert actual == expected

    def test_replay_matches_recorded_float_throttle_sequence(
        self, golden, altitude_replay
    ):
        """The pre-truncation floats catch drift the int sequence may mask."""
        expected = golden["altitude_controller"]["outputs_float"]
        actual = altitude_replay["throttle_float"]
        assert len(actual) == gen.ALTITUDE_STEPS == len(expected)
        assert actual == pytest.approx(expected, rel=1e-9)

    def test_recorded_ints_are_truncations_of_recorded_floats(self, golden):
        """update() returns int(throttle) of the float held in last_throttle."""
        floats = golden["altitude_controller"]["outputs_float"]
        ints = golden["altitude_controller"]["outputs"]
        assert [int(value) for value in floats] == ints

    def test_resolved_config_matches_recorded_config(self, golden, altitude_replay):
        """Triage aid: 'config drifted' vs 'math drifted'.

        If this fails together with the sequence tests, the divergence is
        (at least partly) a config change; if the sequences fail while this
        passes, the math itself changed.
        """
        assert altitude_replay["resolved_config"] == (
            golden["altitude_controller"]["resolved_config"]
        )

    def test_recorded_throttle_stays_within_configured_limits(self, golden):
        """Semantic sanity on top of the byte-level golden master."""
        from src.altitude_config import THROTTLE

        outputs = golden["altitude_controller"]["outputs"]
        assert all(THROTTLE["min"] <= value <= THROTTLE["max"] for value in outputs)
        floats = golden["altitude_controller"]["outputs_float"]
        assert all(THROTTLE["min"] <= value <= THROTTLE["max"] for value in floats)


class TestPidControllerCharacterization:
    def test_input_profile_matches_stored_inputs(self, golden):
        stored = golden["pid_controller"]["inputs"]["setpoint"]
        regenerated = [gen.pid_setpoint_profile(step) for step in range(gen.PID_STEPS)]
        assert len(stored) == gen.PID_STEPS
        assert regenerated == pytest.approx(stored, rel=1e-9)

    def test_replay_matches_recorded_output_sequence(self, golden):
        expected = golden["pid_controller"]["outputs"]
        actual = gen.run_pid_characterization()
        assert len(actual) == gen.PID_STEPS == len(expected)
        assert actual == pytest.approx(expected, rel=1e-9)

    def test_recorded_outputs_respect_clamp_limits(self, golden):
        outputs = golden["pid_controller"]["outputs"]
        low = gen.PID_CONSTRUCTOR_ARGS["output_min"]
        high = gen.PID_CONSTRUCTOR_ARGS["output_max"]
        assert all(low <= value <= high for value in outputs)
        # The profile is designed to saturate the output limit early on.
        assert outputs[0] == pytest.approx(high)


class TestPositionControllerCharacterization:
    def test_input_profiles_match_stored_inputs(self, golden):
        """Distinguish 'profile drifted' from 'behavior drifted'."""
        stored = golden["position_controller"]["inputs"]
        for name, profile in (
            ("dx_pixels", gen.position_dx_profile),
            ("dy_pixels", gen.position_dy_profile),
            ("angle_deg", gen.position_angle_profile),
            ("altitude", gen.position_altitude_profile),
        ):
            regenerated = [profile(step) for step in range(gen.POSITION_STEPS)]
            assert len(stored[name]) == gen.POSITION_STEPS, name
            assert regenerated == pytest.approx(stored[name], rel=1e-9), name

    def test_confidence_profile_matches_exactly_including_sentinel(self, golden):
        """The mode switch compares confidence == 1.01, so the stored values
        must be BIT-identical to what the replay feeds in -- no approx."""
        stored = golden["position_controller"]["inputs"]["confidence"]
        regenerated = [
            gen.position_confidence_profile(step) for step in range(gen.POSITION_STEPS)
        ]
        assert regenerated == stored
        assert 1.01 in stored  # the navigation segment really is recorded

    def test_replay_matches_recorded_pwm_sequences(self, golden, position_replay):
        recorded = golden["position_controller"]["outputs"]
        for axis in ("roll_pwm", "pitch_pwm", "yaw_pwm"):
            expected = recorded[axis]
            actual = position_replay["outputs"][axis]
            assert len(actual) == gen.POSITION_STEPS == len(expected), axis
            # Integers compare exactly -- no approx.
            assert actual == expected, axis

    def test_resolved_config_matches_recorded_config(self, golden, position_replay):
        """Triage aid: 'config drifted' vs 'math drifted' (see altitude twin)."""
        assert position_replay["resolved_config"] == (
            golden["position_controller"]["resolved_config"]
        )

    def test_recorded_pwm_stays_within_recorded_clip_limits(self, golden):
        """Semantic sanity on top of the byte-level golden master."""
        config = golden["position_controller"]["resolved_config"]
        outputs = golden["position_controller"]["outputs"]
        for axis in ("roll", "pitch", "yaw"):
            low, high = config["pwm_clip"][axis]
            assert all(low <= value <= high for value in outputs[f"{axis}_pwm"]), axis

    def test_recorded_outputs_are_not_degenerate(self, golden):
        """Guard against a profile that never leaves the deadband: every axis
        must actually command corrections away from neutral at some point."""
        neutral = golden["position_controller"]["resolved_config"]["pwm_neutral"]
        outputs = golden["position_controller"]["outputs"]
        for axis in ("roll_pwm", "pitch_pwm", "yaw_pwm"):
            assert any(value != neutral for value in outputs[axis]), axis
