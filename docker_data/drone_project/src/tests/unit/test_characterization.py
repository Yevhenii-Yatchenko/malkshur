"""Golden-master (characterization) tests for the control math.

This is the regression safety net for the GRASP refactoring (Step 2 of
REFACTORING_PLAN.md; deterministic stand-in for the skipped SITL golden runs).

``tests/unit/generate_characterization.py`` is the single source of truth for
the synthetic profiles AND the run functions: this test replays the exact
same code paths used at generation time (including the AltitudeCSVLogger
patch) and asserts the outputs still match the sequences stored in
``tests/unit/data/altitude_characterization.json``.

If these tests fail during refactoring, the control behavior changed.  Only
regenerate the JSON when a behavior change is intentional and reviewed.
"""

import json
import os

import pytest

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


class TestAltitudeControllerCharacterization:
    def test_input_profile_matches_stored_inputs(self, golden):
        """Distinguish 'profile drifted' from 'behavior drifted'."""
        stored = golden["altitude_controller"]["inputs"]["current_altitude"]
        regenerated = [gen.altitude_profile(step) for step in range(gen.ALTITUDE_STEPS)]
        assert len(stored) == gen.ALTITUDE_STEPS
        assert regenerated == pytest.approx(stored, rel=1e-9)

    def test_replay_matches_recorded_throttle_sequence(self, golden):
        expected = golden["altitude_controller"]["outputs"]
        actual = gen.run_altitude_characterization()
        assert len(actual) == gen.ALTITUDE_STEPS == len(expected)
        assert actual == pytest.approx(expected, rel=1e-9)

    def test_recorded_throttle_stays_within_configured_limits(self, golden):
        """Semantic sanity on top of the byte-level golden master."""
        from src.altitude_config import THROTTLE

        outputs = golden["altitude_controller"]["outputs"]
        assert all(THROTTLE["min"] <= value <= THROTTLE["max"] for value in outputs)


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
