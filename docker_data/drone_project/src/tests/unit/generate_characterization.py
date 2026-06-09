#!/usr/bin/env python3
"""Golden-master (characterization) data generator for the control math.

Refactoring safety net (REFACTORING_PLAN.md Step 2).  This script drives the
CURRENT production controllers with fixed, formula-defined synthetic profiles
and records the full output sequences into::

    tests/unit/data/altitude_characterization.json

``tests/unit/test_characterization.py`` replays the exact same profiles
through the same code paths (the run functions below are shared between
generation and replay) and asserts the outputs still match the stored
sequences.  Any behavior change in the control math during refactoring will
show up as a mismatch.

Regenerate (ONLY when a behavior change is intentional and reviewed), from the
project root inside the container::

    docker exec -e GIT_SHA=$(git rev-parse HEAD) malkshur_droneproject \
        python3 tests/unit/generate_characterization.py

Determinism notes:
- Profiles are pure formulas (logistic + sinusoid) -- no randomness.
- ``current_time`` is injected as exact multiples of the step period -- no
  wall-clock dependence.
- The ``AltitudeCSVLogger`` is patched out identically here and in the replay
  test, so no log files are written and conditions are identical.
"""

import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone
from unittest import mock

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir)
)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

DATA_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "data", "altitude_characterization.json"
)

# ---------------------------------------------------------------------------
# Profile 1: AltitudeController.update over a synthetic takeoff
# ---------------------------------------------------------------------------
# 300 steps of exactly 0.01 s (3 s total).  The altitude follows a logistic
# climb from ~0 m to ~5 m with a growing sinusoidal ripple that overshoots the
# 5.0 m target near t ~ 2.3 s.  The controller is built with its production
# defaults (bound from src/altitude_config.py at import time -- they ARE the
# current behavior).

ALTITUDE_STEPS = 300
ALTITUDE_DT = 0.01
ALTITUDE_TARGET = 5.0

ALTITUDE_PROFILE_FORMULA = (
    "altitude(t) = 5.0 / (1 + exp(-4.0 * (t - 1.0)))"
    " + 0.2 * sin(2 * pi * t) * (t / 3.0)"
    " with t = step * 0.01, step = 0..299; target_altitude = 5.0."
    " Logistic takeoff 0 -> ~5 m with growing sinusoidal overshoot ripple."
)


def altitude_profile(step):
    """Deterministic synthetic altitude (meters) for a given step index."""
    t = step * ALTITUDE_DT
    logistic = 5.0 / (1.0 + math.exp(-4.0 * (t - 1.0)))
    ripple = 0.2 * math.sin(2.0 * math.pi * t) * (t / 3.0)
    return logistic + ripple


def run_altitude_characterization():
    """Drive a default AltitudeController over the takeoff profile.

    Returns the full list of throttle outputs (ints).  The CSV logger is
    patched out so nothing is written under logs/ -- this patch is part of
    the recorded conditions and must stay identical in the replay test
    (it does: the test calls this very function).
    """
    from src.pid_controller import AltitudeController

    with mock.patch("src.pid_controller.AltitudeCSVLogger"):
        controller = AltitudeController()
        outputs = []
        for step in range(ALTITUDE_STEPS):
            throttle = controller.update(
                target_altitude=ALTITUDE_TARGET,
                current_altitude=altitude_profile(step),
                current_time=step * ALTITUDE_DT,
            )
            outputs.append(int(throttle))
    return outputs


# ---------------------------------------------------------------------------
# Profile 2: bare PIDController.update over a fixed error profile
# ---------------------------------------------------------------------------
# 200 steps of exactly 0.02 s (4 s total).  The setpoint follows a decaying
# oscillation while the measurement stays at 0.0, so the error sweeps both
# signs and decays.  The initial error (6.0 * kp = 12.0) saturates the +/-10
# output limit, exercising output clamping and the conditional anti-windup
# branch.  Gains are explicit literals so the run is self-contained.

PID_STEPS = 200
PID_DT = 0.02

PID_CONSTRUCTOR_ARGS = {
    "kp": 2.0,
    "ki": 0.5,
    "kd": 1.5,
    "output_min": -10.0,
    "output_max": 10.0,
    "integral_limit": 5.0,
    "derivative_filter_size": 5,
}

PID_PROFILE_FORMULA = (
    "setpoint(t) = 6.0 * cos(2.0 * t) * exp(-0.3 * t), measurement = 0.0,"
    " with t = step * 0.02, step = 0..199."
    " Decaying oscillation; the initial error saturates the +/-10 output"
    " limit (clamping + conditional anti-windup paths exercised)."
)


def pid_setpoint_profile(step):
    """Deterministic synthetic setpoint for a given step index."""
    t = step * PID_DT
    return 6.0 * math.cos(2.0 * t) * math.exp(-0.3 * t)


def run_pid_characterization():
    """Drive a bare PIDController over the fixed error profile.

    Returns the full list of outputs (floats).
    """
    from src.pid_controller import PIDController

    pid = PIDController(**PID_CONSTRUCTOR_ARGS)
    outputs = []
    for step in range(PID_STEPS):
        output = pid.update(
            setpoint=pid_setpoint_profile(step),
            measurement=0.0,
            current_time=step * PID_DT,
        )
        outputs.append(float(output))
    return outputs


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------

def _git_sha():
    """Git SHA at generation time.

    Inside the container the repository's .git directory is not mounted, so
    the SHA is normally passed in via the GIT_SHA environment variable (see
    module docstring).  Falls back to asking git directly, then to "unknown".
    """
    sha = os.environ.get("GIT_SHA")
    if sha:
        return sha.strip()
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=PROJECT_ROOT,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


def build_dataset():
    """Build the complete golden-master dataset (metadata + both runs)."""
    return {
        "metadata": {
            "purpose": (
                "Golden-master characterization of the CURRENT control math, "
                "recorded before any production refactoring (Step 2 of the "
                "GRASP refactoring plan). Outputs must not change unless a "
                "behavior change is intentional."
            ),
            "generator": "tests/unit/generate_characterization.py",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "git_sha_at_generation": _git_sha(),
        },
        "altitude_controller": {
            "class": "src.pid_controller.AltitudeController",
            "constructor": (
                "AltitudeController() with production defaults bound from "
                "src/altitude_config.py at import time"
            ),
            "csv_logger": "src.pid_controller.AltitudeCSVLogger patched out (mock)",
            "profile": {
                "description": ALTITUDE_PROFILE_FORMULA,
                "steps": ALTITUDE_STEPS,
                "dt": ALTITUDE_DT,
                "target_altitude": ALTITUDE_TARGET,
            },
            "inputs": {
                "current_altitude": [
                    altitude_profile(step) for step in range(ALTITUDE_STEPS)
                ],
            },
            "outputs": run_altitude_characterization(),
        },
        "pid_controller": {
            "class": "src.pid_controller.PIDController",
            "constructor_args": PID_CONSTRUCTOR_ARGS,
            "profile": {
                "description": PID_PROFILE_FORMULA,
                "steps": PID_STEPS,
                "dt": PID_DT,
                "measurement": 0.0,
            },
            "inputs": {
                "setpoint": [pid_setpoint_profile(step) for step in range(PID_STEPS)],
            },
            "outputs": run_pid_characterization(),
        },
    }


def main():
    dataset = build_dataset()
    os.makedirs(os.path.dirname(DATA_FILE), exist_ok=True)
    with open(DATA_FILE, "w") as fh:
        json.dump(dataset, fh, indent=2)
        fh.write("\n")

    alt = dataset["altitude_controller"]["outputs"]
    pid = dataset["pid_controller"]["outputs"]
    print(f"Wrote {DATA_FILE}")
    print(f"  git_sha_at_generation: {dataset['metadata']['git_sha_at_generation']}")
    print(f"  altitude_controller: {len(alt)} outputs, "
          f"first={alt[0]}, last={alt[-1]}, min={min(alt)}, max={max(alt)}")
    print(f"  pid_controller:      {len(pid)} outputs, "
          f"first={pid[0]!r}, last={pid[-1]!r}")


if __name__ == "__main__":
    main()
