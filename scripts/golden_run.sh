#!/usr/bin/env bash
# golden_run.sh — Execute one reproducible flight scenario and collect artifacts.
#
# Scenario:
#   1. Clean state (kill leftover processes, reset Gazebo poses, restart ArduPilot).
#   2. Start the flight controller (it auto-arms and auto-takes-off to 5 m).
#   3. Wait until altitude >= 4.5 m (timeout 90 s).
#   4. Hold for 90 s.
#   5. Send "land," via the telnet interface (controlled descent to ~0.2 m).
#   6. After touchdown is observed, send "mode,LAND": the controller exits via its
#      designed "Landing detected" path and ArduPilot disarms on the ground.
#      NOTE: the controller does NOT exit after "land," alone — its landing logic
#      floors the throttle around 1400 PWM, so ArduPilot never auto-disarms
#      (verified empirically: the drone idles at ~0.2 m indefinitely).
#   7. Collect artifacts into scripts/golden_runs/<UTC-timestamp>/:
#      new CSV files, controller stdout log, metadata.json.
#
# Exit code: 0 = run completed, 1 = run FAILED (reason printed and recorded).

set -euo pipefail

###############################################################################
# Paths & constants
###############################################################################
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CSV_HOST_DIR="${REPO_ROOT}/docker_data/drone_project/logs/csv"
RUNS_DIR="${REPO_ROOT}/scripts/golden_runs"

TAKEOFF_ALT=4.5          # altitude threshold to consider takeoff complete (m)
TOUCHDOWN_ALT=0.35       # altitude threshold to consider the drone landed (m)
HOLD_DURATION=90         # seconds to hold at altitude
TAKEOFF_TIMEOUT=90       # seconds before declaring takeoff FAILED
EXIT_TIMEOUT=120         # seconds after land command for controller to exit
AP_TIMEOUT=90            # seconds for ArduPilot to become ready after restart

TELNET_HOST=localhost
TELNET_PORT=2323

###############################################################################
# Run directory
###############################################################################
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUNS_DIR}/${RUN_TS}"
mkdir -p "${RUN_DIR}"

LOG="${RUN_DIR}/controller_stdout.log"
ALT_FILE="${RUN_DIR}/.alt_telemetry"

CTRL_PID=""
POLL_PID=""
T_CTRL_START=""
T_ALT_REACHED=""
T_LAND=""
T_TOUCHDOWN=""
T_MODE_LAND=""
T_EXIT=""
CTRL_EXIT=""

###############################################################################
# Helpers
###############################################################################
now_ts() { python3 -c "import time; print(time.time())"; }

# Killing the host-side `docker exec` client does NOT kill the process inside
# the container, so always pkill in the container as well.
kill_in_container() {
    docker exec malkshur_droneproject pkill -f xbee_process_com 2>/dev/null || true
    docker exec malkshur_droneproject pkill -f sky_anchor 2>/dev/null || true
    docker exec malkshur_droneproject pkill -f GOLDEN_ALT_POLL 2>/dev/null || true
}

cleanup_on_exit() {
    [[ -n "${CTRL_PID}" ]] && kill "${CTRL_PID}" 2>/dev/null || true
    [[ -n "${POLL_PID}" ]] && kill "${POLL_PID}" 2>/dev/null || true
    kill_in_container
}
trap cleanup_on_exit EXIT

write_metadata() {
    # $1 = exit_status, $2 = failure_reason (empty on success)
    META_RUN_TS="${RUN_TS}" META_RUN_DIR="${RUN_DIR}" META_LOG="${LOG}" \
    META_EXIT_STATUS="$1" META_FAILURE_REASON="$2" \
    META_T_CTRL_START="${T_CTRL_START}" META_T_ALT_REACHED="${T_ALT_REACHED}" \
    META_T_LAND="${T_LAND}" META_T_TOUCHDOWN="${T_TOUCHDOWN}" \
    META_T_MODE_LAND="${T_MODE_LAND}" META_T_EXIT="${T_EXIT}" \
    META_CTRL_EXIT="${CTRL_EXIT}" \
    python3 - <<'PYEOF'
import json, os, re
from datetime import datetime, timezone

env = os.environ

def f(name):
    v = env.get(name, "")
    return float(v) if v else None

def arm_time_from_log(log_path):
    """Wall-clock arm time from '[INFO] <ts>: Drone armed successfully!'.

    The controller runs in a UTC container, so its log timestamps are UTC.
    """
    try:
        with open(log_path) as fh:
            for line in fh:
                if "Drone armed successfully" in line:
                    m = re.search(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)", line)
                    if m:
                        return datetime.strptime(
                            m.group(1), "%Y-%m-%d %H:%M:%S.%f"
                        ).replace(tzinfo=timezone.utc).timestamp()
    except OSError:
        pass
    return None

run_dir = env["META_RUN_DIR"]
meta = {
    "run_id": env["META_RUN_TS"],
    "wall_times": {
        "controller_start": f("META_T_CTRL_START"),
        "arm": arm_time_from_log(env["META_LOG"]),
        "altitude_reached": f("META_T_ALT_REACHED"),
        "land_command": f("META_T_LAND"),
        "touchdown": f("META_T_TOUCHDOWN"),
        "land_mode_command": f("META_T_MODE_LAND"),
        # Closest observable to "disarm": the controller exits its loop right
        # after the LAND mode switch / disarm is detected.
        "controller_exit": f("META_T_EXIT"),
    },
    "durations_s": {},
    "controller_exit_code": int(env["META_CTRL_EXIT"]) if env.get("META_CTRL_EXIT") else None,
    "exit_status": int(env["META_EXIT_STATUS"]),
    "artifacts": sorted(x for x in os.listdir(run_dir) if not x.startswith(".")),
}
if env.get("META_FAILURE_REASON"):
    meta["failure_reason"] = env["META_FAILURE_REASON"]

wt = meta["wall_times"]
pairs = [
    ("time_to_altitude", "controller_start", "altitude_reached"),
    ("hold_duration", "altitude_reached", "land_command"),
    ("land_to_exit", "land_command", "controller_exit"),
    ("total_flight", "controller_start", "controller_exit"),
]
for name, a, b in pairs:
    if wt[a] is not None and wt[b] is not None:
        meta["durations_s"][name] = wt[b] - wt[a]

with open(os.path.join(run_dir, "metadata.json"), "w") as fh:
    json.dump(meta, fh, indent=2)
print(json.dumps(meta, indent=2))
PYEOF
}

fail() {
    echo "[golden_run] FAILED: $1" >&2
    write_metadata 1 "$1" > /dev/null || true
    exit 1
}

send_telnet_cmd() {
    # Plain TCP line protocol: "command,params\n"
    local cmd="$1"
    GOLDEN_CMD="${cmd}" GOLDEN_HOST="${TELNET_HOST}" GOLDEN_PORT="${TELNET_PORT}" \
    python3 - <<'PYEOF'
import os, socket, time
cmd = os.environ["GOLDEN_CMD"]
s = socket.socket()
s.settimeout(5)
s.connect((os.environ["GOLDEN_HOST"], int(os.environ["GOLDEN_PORT"])))
s.settimeout(2)
try:
    s.recv(4096)  # drain welcome banner, if any
except socket.timeout:
    pass
s.sendall((cmd + "\n").encode())
time.sleep(0.5)
try:
    s.recv(4096)
except socket.timeout:
    pass
s.close()
print(f"telnet command sent: {cmd}")
PYEOF
}

# First altitude sample at/after epoch $1 that is >= $2; prints its epoch.
alt_first_above() {
    awk -v t0="$1" -v th="$2" \
        '$1=="ALT" && $2>=t0 && $3>=th {print $2; exit}' "${ALT_FILE}" 2>/dev/null || true
}

# Epoch of the 2nd of two consecutive samples <= $2 after epoch $1 (touchdown).
alt_settled_below() {
    awk -v t0="$1" -v th="$2" \
        '$1=="ALT" && $2>t0 { if ($3<=th) {c++; if (c>=2) {print $2; exit}} else c=0 }' \
        "${ALT_FILE}" 2>/dev/null || true
}

# Most recent altitude sample (m).
alt_latest() {
    awk '$1=="ALT" {a=$3} END {print a}' "${ALT_FILE}" 2>/dev/null || true
}

# Returns success if latest altitude exceeds $1 (runaway detector).
alt_above() {
    local latest
    latest="$(alt_latest)"
    [[ -n "${latest}" ]] && awk -v a="${latest}" -v th="$1" 'BEGIN {exit !(a>th)}'
}

###############################################################################
# 1. Clean state
###############################################################################
echo "[golden_run] === Run ${RUN_TS} ==="
echo "[golden_run] 1. Cleaning state..."
kill_in_container
sleep 1

echo "[golden_run]    Resetting Gazebo model poses..."
docker exec malkshur_gazebo gz world --reset-models 2>/dev/null || true
sleep 2

echo "[golden_run]    Restarting ArduPilot container..."
docker restart malkshur_ardupilot > /dev/null

echo "[golden_run]    Waiting for ArduPilot EKF to be ready..."
AP_START=$SECONDS
while ! docker logs --since 2m malkshur_ardupilot 2>&1 | grep -q "origin set"; do
    if [[ $((SECONDS - AP_START)) -ge ${AP_TIMEOUT} ]]; then
        fail "ArduPilot did not become ready within ${AP_TIMEOUT}s (no 'origin set' in logs)"
    fi
    sleep 3
done
echo "[golden_run]    ArduPilot ready after $((SECONDS - AP_START))s"

###############################################################################
# 2. Snapshot CSV dir, start altitude telemetry poller
###############################################################################
echo "[golden_run] 2. Snapshotting CSV directory..."
ls -1 "${CSV_HOST_DIR}" 2>/dev/null | sort > "${RUN_DIR}/.pre_snapshot"
touch "${RUN_DIR}/.run_marker"

# Independent altitude telemetry: one persistent pymavlink process on the
# spare MAVLink port, streaming "ALT <epoch> <alt_m>" lines for the whole run.
ALT_POLL_CODE=$(cat <<'PY'
# GOLDEN_ALT_POLL
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection("tcp:localhost:5762")
m.wait_heartbeat(timeout=15)
m.mav.request_data_stream_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
deadline = time.time() + 480  # self-terminate well after any plausible run end
while time.time() < deadline:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg is not None:
        print("ALT %.3f %.3f" % (time.time(), msg.relative_alt / 1000.0), flush=True)
PY
)
echo "[golden_run]    Starting altitude telemetry poller..."
docker exec malkshur_droneproject python3 -u -c "${ALT_POLL_CODE}" \
    > "${ALT_FILE}" 2> "${RUN_DIR}/.alt_telemetry_err" &
POLL_PID=$!

POLL_START=$SECONDS
while [[ ! -s "${ALT_FILE}" ]]; do
    if [[ $((SECONDS - POLL_START)) -ge 30 ]]; then
        fail "Altitude telemetry unavailable (no MAVLink data on tcp:localhost:5762)"
    fi
    sleep 1
done
echo "[golden_run]    Altitude telemetry up."

###############################################################################
# 3. Start the flight controller (auto-arms, auto-takes-off to 5 m)
###############################################################################
echo "[golden_run] 3. Starting flight controller..."
T_CTRL_START="$(now_ts)"
docker exec malkshur_droneproject python3 xbee_process_com.py > "${LOG}" 2>&1 &
CTRL_PID=$!
echo "[golden_run]    Controller PID: ${CTRL_PID}"

###############################################################################
# 4. Wait until altitude >= TAKEOFF_ALT
###############################################################################
echo "[golden_run] 4. Waiting for altitude >= ${TAKEOFF_ALT} m (timeout ${TAKEOFF_TIMEOUT}s)..."
TAKEOFF_START=$SECONDS
while true; do
    if ! kill -0 "${CTRL_PID}" 2>/dev/null; then
        fail "Controller process exited unexpectedly during takeoff phase"
    fi
    T_ALT_REACHED="$(alt_first_above "${T_CTRL_START}" "${TAKEOFF_ALT}")"
    if [[ -n "${T_ALT_REACHED}" ]]; then
        echo "[golden_run]    Altitude ${TAKEOFF_ALT} m reached at t=${T_ALT_REACHED}"
        break
    fi
    if [[ $((SECONDS - TAKEOFF_START)) -ge ${TAKEOFF_TIMEOUT} ]]; then
        fail "Takeoff timeout: drone did not reach ${TAKEOFF_ALT} m within ${TAKEOFF_TIMEOUT}s"
    fi
    sleep 2
done

###############################################################################
# 5. Hold at altitude
###############################################################################
echo "[golden_run] 5. Holding at altitude for ${HOLD_DURATION}s..."
HOLD_START=$SECONDS
while [[ $((SECONDS - HOLD_START)) -lt ${HOLD_DURATION} ]]; do
    if ! kill -0 "${CTRL_PID}" 2>/dev/null; then
        fail "Controller process exited unexpectedly during hold phase"
    fi
    # Fail fast on the known runaway destabilization (position-hold loses
    # vision track, drone climbs and drifts away): nominal hold is 5 m +- ~0.5.
    if alt_above 7.0; then
        fail "Destabilized during hold: altitude $(alt_latest) m (> 7.0 m); known position-hold runaway"
    fi
    sleep 5
    echo "[golden_run]    Hold: $((SECONDS - HOLD_START))/${HOLD_DURATION}s"
done

###############################################################################
# 6. Land and wait for controller exit
###############################################################################
echo "[golden_run] 6. Sending land command..."
T_LAND="$(now_ts)"
send_telnet_cmd "land,"

echo "[golden_run] 7. Waiting for touchdown and controller exit (timeout ${EXIT_TIMEOUT}s)..."
LAND_WAIT_START=$SECONDS
while kill -0 "${CTRL_PID}" 2>/dev/null; do
    if [[ -z "${T_MODE_LAND}" ]]; then
        T_TOUCHDOWN="$(alt_settled_below "${T_LAND}" "${TOUCHDOWN_ALT}")"
        if [[ -n "${T_TOUCHDOWN}" ]]; then
            echo "[golden_run]    Touchdown at t=${T_TOUCHDOWN}; switching to LAND mode for disarm..."
            send_telnet_cmd "mode,LAND"
            T_MODE_LAND="$(now_ts)"
        fi
    fi
    if [[ -z "${T_TOUCHDOWN}" ]] && alt_above 10.0; then
        fail "Destabilized after land command: altitude $(alt_latest) m (> 10.0 m); known position-hold runaway"
    fi
    if [[ $((SECONDS - LAND_WAIT_START)) -ge ${EXIT_TIMEOUT} ]]; then
        if [[ -z "${T_TOUCHDOWN}" ]]; then
            fail "Drone never touched down (alt <= ${TOUCHDOWN_ALT} m) within ${EXIT_TIMEOUT}s after land command (destabilized?)"
        fi
        fail "Controller did not exit within ${EXIT_TIMEOUT}s after land command"
    fi
    sleep 2
done

CTRL_EXIT=0
wait "${CTRL_PID}" 2>/dev/null || CTRL_EXIT=$?
T_EXIT="$(now_ts)"
CTRL_PID=""
echo "[golden_run]    Controller exited with status ${CTRL_EXIT}"

# Stop the telemetry poller and any sky_anchor the controller spawned.
kill_in_container
kill "${POLL_PID}" 2>/dev/null || true
POLL_PID=""

###############################################################################
# 7. Collect CSV artifacts
###############################################################################
echo "[golden_run] 8. Collecting artifacts..."
NEW_CSV_COUNT=0
while IFS= read -r -d '' f; do
    FNAME="$(basename "$f")"
    # Copy (never move/delete): files are owned by root inside the container.
    if cp "$f" "${RUN_DIR}/${FNAME}"; then
        echo "[golden_run]    Copied: ${FNAME}"
        NEW_CSV_COUNT=$((NEW_CSV_COUNT + 1))
    else
        echo "[golden_run]    WARNING: could not copy ${FNAME}" >&2
    fi
done < <(find "${CSV_HOST_DIR}" -name "*.csv" -newer "${RUN_DIR}/.run_marker" -print0 2>/dev/null)

if [[ ${NEW_CSV_COUNT} -eq 0 ]]; then
    fail "No new CSV files found in ${CSV_HOST_DIR}"
fi

###############################################################################
# 8. Write metadata.json
###############################################################################
echo "[golden_run] 9. Writing metadata.json..."
write_metadata 0 ""

echo "[golden_run] === Run ${RUN_TS} COMPLETE ==="
echo "[golden_run] Artifacts in: ${RUN_DIR}"
