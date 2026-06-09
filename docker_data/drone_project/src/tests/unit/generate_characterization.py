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
- Profiles are pure formulas (logistic / sinusoid / piecewise confidence) --
  no randomness.
- ``current_time`` is injected as exact multiples of the step period -- no
  wall-clock dependence.
- The file-writing CSV loggers are replaced by an explicit ``NullCSVLogger``
  injected via the Step 3 ``csv_logger=`` constructor parameter, and
  ``get_logger`` is patched out, so no log files are written.

Step 3+ migration note (logger injection):
The recorded outputs are logging-independent: the controllers only ever WRITE
to their loggers (append/info calls on pure snapshots of already-computed
state), never read anything back, so logging cannot influence the control
math.  The construction/patch wiring inside the run functions below could
therefore be re-wired during refactoring WITHOUT regenerating this JSON --
and Step 3 did exactly that (explicit ``NullCSVLogger`` injection instead of
``mock.patch``-ing the CSV logger classes); regenerate only for an
intentional, reviewed behavior change to the control math.  CAUTION: the
backward-compat API (``csv_logger=None`` -> the constructor creates a real
file-writing logger) means the replay must inject an explicit null-object
logger -- passing ``None`` would silently re-enable file I/O.
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


class NullCSVLogger:
    """No-op stand-in for the file-writing CSV loggers (Step 3 injection).

    A real object with the full surface the controllers touch on their
    ``csv_logger``: ``append(data)`` (called from update()), ``close()``
    (PositionController.__del__) and an assignable ``writer`` attribute
    (PositionController.stop() sets ``csv_logger.writer = None``).  Injecting
    this explicitly is REQUIRED for an I/O-free run: passing ``csv_logger=None``
    makes the constructors create real file-writing loggers (the
    backward-compat default).
    """

    def __init__(self):
        self.writer = None

    def append(self, data):
        pass

    def close(self):
        pass


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


def _pid_snapshot(pid):
    """Resolved parameters of a constructed PIDController (JSON-safe natives)."""
    return {
        "kp": pid.kp,
        "ki": pid.ki,
        "kd": pid.kd,
        "output_min": pid.output_min,
        "output_max": pid.output_max,
        "integral_limit": pid.integral_limit,
    }


def _altitude_resolved_config(controller):
    """Snapshot of the AltitudeController configuration in effect for the run.

    Recorded so a future mismatch can be triaged as "config drifted" vs "math
    drifted".  Values are read back from the constructed controller (i.e. the
    constructor-default resolution of src/altitude_config.py), except the last
    two, which ``update()`` reads from the altitude_config THROTTLE/CONTROL
    dicts on EVERY call (rate_limit has always effectively been
    THROTTLE['rate_limit']; the dead ``self.throttle_rate_limit = 50``
    constructor attribute that used to shadow it was removed in Step 3).
    """
    from src.altitude_config import CONTROL, THROTTLE

    return {
        "altitude_pid": _pid_snapshot(controller.position_pid),
        "velocity_pid": _pid_snapshot(controller.velocity_pid),
        "max_velocity": controller.max_velocity,
        "max_acceleration": controller.max_acceleration,
        "throttle_hover": controller.throttle_hover,
        "throttle_min": controller.throttle_min,
        "throttle_max": controller.throttle_max,
        "altitude_filter_alpha": controller.altitude_filter_alpha,
        "velocity_filter_size": controller.velocity_history.maxlen,
        # Read from config at update() time, not from the controller instance:
        "throttle_rate_limit": THROTTLE.get("rate_limit"),
        "throttle_filter_alpha": CONTROL.get("throttle_filter_alpha"),
    }


def run_altitude_characterization():
    """Drive a default AltitudeController over the takeoff profile.

    Returns a dict with:
    - ``throttle_int``:   the int throttle sequence as returned by update()
    - ``throttle_float``: the pre-truncation float throttle per step.
      ``update()`` assigns the post-limit/post-filter float to
      ``controller.last_throttle`` immediately before ``return int(throttle)``
      (and that float seeds the next step's rate limiter/filter), so reading
      ``last_throttle`` after each call captures the exact pre-int value.
    - ``resolved_config``: see _altitude_resolved_config().

    An explicit NullCSVLogger is injected via the Step 3 ``csv_logger=``
    parameter so nothing is written under logs/ (before Step 3 this was a
    ``mock.patch`` of AltitudeCSVLogger; the JSON was NOT regenerated for the
    re-wiring -- outputs are logging-independent, see module docstring,
    including the csv_logger=None caveat).
    """
    from src.pid_controller import AltitudeController

    controller = AltitudeController(csv_logger=NullCSVLogger())
    resolved_config = _altitude_resolved_config(controller)
    throttle_int = []
    throttle_float = []
    for step in range(ALTITUDE_STEPS):
        throttle = controller.update(
            target_altitude=ALTITUDE_TARGET,
            current_altitude=altitude_profile(step),
            current_time=step * ALTITUDE_DT,
        )
        throttle_int.append(int(throttle))
        throttle_float.append(float(controller.last_throttle))
    return {
        "throttle_int": throttle_int,
        "throttle_float": throttle_float,
        "resolved_config": resolved_config,
    }


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
# Profile 3: PositionController.update across the confidence-sentinel modes
# ---------------------------------------------------------------------------
# 360 steps of exactly 0.02 s (7.2 s total, the ~50 Hz loop rate).  dx/dy are
# multi-frequency sinusoids that sweep both signs across the +/-5 px deadband;
# the angle crosses the +/-1 deg deadband.  The confidence input is piecewise
# constant and drives the mode state machine in update()
# (src/position_controller.py):
#
#   confidence == 1.01 -> __enable_navigation():   position ki/kd zeroed, reset
#   confidence <  1.0  -> __enable_stabilization(): ki/kd restored from
#                         POSITION_PID_X/Y, reset
#
# 1.01 is the navigation sentinel: sky_anchor's CommandModifier emits
# matches_percent=101.0 while navigating and src/controller.py forwards
# confidence = matches_percent / 100.0 (101.0 / 100.0 == 1.01 exactly in
# IEEE-754).  The three segments exercise stabilization -> navigation ->
# back-to-stabilization, including both reset() calls at the switches.
# Altitude is an input of update() but only flows into CSV logging (it is
# recorded anyway as part of the driven signature).

POSITION_STEPS = 360
POSITION_DT = 0.02

# Segment boundaries are half-open step ranges [start, stop).
POSITION_SEGMENTS = {
    "stabilization": {"start": 0, "stop": 120, "confidence": 0.85},
    "navigation": {"start": 120, "stop": 240, "confidence": 1.01},
    "stabilization_return": {"start": 240, "stop": 360, "confidence": 0.9},
}

POSITION_PROFILE_FORMULA = (
    "dx(t) = 30*sin(1.3*t) + 8*cos(4.1*t); dy(t) = 25*cos(0.9*t) - 6*sin(3.7*t);"
    " angle(t) = 2.5*sin(1.7*t); altitude(t) = 5.0 + 0.5*sin(0.5*t);"
    " with t = step * 0.02, step = 0..359."
    " confidence: steps 0-119 -> 0.85 (stabilization), 120-239 -> 1.01"
    " (navigation sentinel: ki/kd zeroed + reset), 240-359 -> 0.9 (back to"
    " stabilization: ki/kd restored from POSITION_PID_X/Y + reset)."
    " Drift sweeps both signs across the +/-5 px deadband and the angle"
    " crosses the +/-1 deg deadband."
)


def position_dx_profile(step):
    """Deterministic synthetic dx drift (pixels) for a given step index."""
    t = step * POSITION_DT
    return 30.0 * math.sin(1.3 * t) + 8.0 * math.cos(4.1 * t)


def position_dy_profile(step):
    """Deterministic synthetic dy drift (pixels) for a given step index."""
    t = step * POSITION_DT
    return 25.0 * math.cos(0.9 * t) - 6.0 * math.sin(3.7 * t)


def position_angle_profile(step):
    """Deterministic synthetic angular drift (degrees) for a given step index."""
    t = step * POSITION_DT
    return 2.5 * math.sin(1.7 * t)


def position_altitude_profile(step):
    """Deterministic synthetic altitude (meters) for a given step index."""
    t = step * POSITION_DT
    return 5.0 + 0.5 * math.sin(0.5 * t)


def position_confidence_profile(step):
    """Piecewise-constant confidence; 1.01 is the exact navigation sentinel."""
    for segment in POSITION_SEGMENTS.values():
        if segment["start"] <= step < segment["stop"]:
            return segment["confidence"]
    raise ValueError(f"step {step} outside the defined profile segments")


def _position_resolved_config(controller):
    """Snapshot of the PositionController configuration in effect for the run.

    PID gains are the construction-time (stabilization-mode) resolution of
    src/position_config.py; update() later mutates ki/kd in place on mode
    switches, which is exactly the behavior the recorded outputs pin down.
    """
    from src.position_config import COORDINATE_SYSTEM, POSITION_CONTROL, PWM_LIMITS

    return {
        "position_pid_x": _pid_snapshot(controller.position_pid_x),
        "position_pid_y": _pid_snapshot(controller.position_pid_y),
        "angle_pid": _pid_snapshot(controller.angle_pid),
        "pwm_neutral": PWM_LIMITS["neutral"],
        "pwm_clip": {
            "roll": [PWM_LIMITS["min_roll"], PWM_LIMITS["max_roll"]],
            "pitch": [PWM_LIMITS["min_pitch"], PWM_LIMITS["max_pitch"]],
            "yaw": [PWM_LIMITS["min_yaw"], PWM_LIMITS["max_yaw"]],
        },
        "deadband": {
            "x": POSITION_CONTROL["deadband_x"],
            "y": POSITION_CONTROL["deadband_y"],
            "angle": POSITION_CONTROL["deadband_angle"],
        },
        "coordinate_system": {
            "invert_x": COORDINATE_SYSTEM["invert_x"],
            "invert_y": COORDINATE_SYSTEM["invert_y"],
            "invert_angle": COORDINATE_SYSTEM["invert_angle"],
        },
        "velocity_filter_size": controller.velocity_history_x.maxlen,
    }


def run_position_characterization():
    """Drive a default PositionController over the three-segment profile.

    Returns a dict with:
    - ``outputs``: full int sequences for roll_pwm / pitch_pwm / yaw_pwm (the
      complete return value of update() -- it returns exactly these three
      keys).
    - ``resolved_config``: see _position_resolved_config().

    An explicit NullCSVLogger is injected via the Step 3 ``csv_logger=``
    parameter and get_logger is patched out so nothing is written under
    logs/ (before Step 3 PositionCSVLogger was ``mock.patch``-ed instead;
    the JSON was NOT regenerated for the re-wiring -- outputs are
    logging-independent, see module docstring, including the csv_logger=None
    caveat).
    """
    from src.position_controller import PositionController

    with mock.patch("src.position_controller.get_logger",
                    return_value=mock.Mock()):
        controller = PositionController(csv_logger=NullCSVLogger())
        resolved_config = _position_resolved_config(controller)
        outputs = {"roll_pwm": [], "pitch_pwm": [], "yaw_pwm": []}
        for step in range(POSITION_STEPS):
            result = controller.update(
                dx_pixels=position_dx_profile(step),
                dy_pixels=position_dy_profile(step),
                angle_deg=position_angle_profile(step),
                confidence=position_confidence_profile(step),
                altitude=position_altitude_profile(step),
                current_time=step * POSITION_DT,
            )
            # np.clip makes these numpy ints; convert for JSON.
            outputs["roll_pwm"].append(int(result["roll_pwm"]))
            outputs["pitch_pwm"].append(int(result["pitch_pwm"]))
            outputs["yaw_pwm"].append(int(result["yaw_pwm"]))
    return {"outputs": outputs, "resolved_config": resolved_config}


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
    """Build the complete golden-master dataset (metadata + all three runs)."""
    altitude_run = run_altitude_characterization()
    position_run = run_position_characterization()
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
            "csv_logger": (
                "explicit NullCSVLogger injected via the Step 3 csv_logger= "
                "constructor parameter (no file-writing logger created)"
            ),
            "resolved_config": altitude_run["resolved_config"],
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
            "outputs": altitude_run["throttle_int"],
            "outputs_float": altitude_run["throttle_float"],
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
        "position_controller": {
            "class": "src.position_controller.PositionController",
            "constructor": (
                "PositionController() with production defaults bound from "
                "src/position_config.py at import time"
            ),
            "patches": (
                "src.position_controller.get_logger patched out (mock); "
                "explicit NullCSVLogger injected via the Step 3 csv_logger= "
                "constructor parameter (no file-writing logger created)"
            ),
            "resolved_config": position_run["resolved_config"],
            "profile": {
                "description": POSITION_PROFILE_FORMULA,
                "steps": POSITION_STEPS,
                "dt": POSITION_DT,
                "segments": POSITION_SEGMENTS,
            },
            "inputs": {
                "dx_pixels": [
                    position_dx_profile(step) for step in range(POSITION_STEPS)
                ],
                "dy_pixels": [
                    position_dy_profile(step) for step in range(POSITION_STEPS)
                ],
                "angle_deg": [
                    position_angle_profile(step) for step in range(POSITION_STEPS)
                ],
                "altitude": [
                    position_altitude_profile(step) for step in range(POSITION_STEPS)
                ],
                "confidence": [
                    position_confidence_profile(step) for step in range(POSITION_STEPS)
                ],
            },
            "outputs": position_run["outputs"],
        },
    }


def main():
    dataset = build_dataset()
    os.makedirs(os.path.dirname(DATA_FILE), exist_ok=True)
    with open(DATA_FILE, "w") as fh:
        json.dump(dataset, fh, indent=2)
        fh.write("\n")

    alt = dataset["altitude_controller"]["outputs"]
    alt_float = dataset["altitude_controller"]["outputs_float"]
    pid = dataset["pid_controller"]["outputs"]
    pos = dataset["position_controller"]["outputs"]
    print(f"Wrote {DATA_FILE}")
    print(f"  git_sha_at_generation: {dataset['metadata']['git_sha_at_generation']}")
    print(f"  altitude_controller: {len(alt)} outputs, "
          f"first={alt[0]}, last={alt[-1]}, min={min(alt)}, max={max(alt)}; "
          f"float first={alt_float[0]!r}, last={alt_float[-1]!r}")
    print(f"  pid_controller:      {len(pid)} outputs, "
          f"first={pid[0]!r}, last={pid[-1]!r}")
    for axis in ("roll_pwm", "pitch_pwm", "yaw_pwm"):
        seq = pos[axis]
        print(f"  position_controller {axis}: {len(seq)} outputs, "
              f"first={seq[0]}, last={seq[-1]}, min={min(seq)}, max={max(seq)}")


if __name__ == "__main__":
    main()
