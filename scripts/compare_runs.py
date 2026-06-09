#!/usr/bin/env python3
"""
compare_runs.py — Analyze and compare golden run artifacts.

Subcommands:
  analyze <run_dir>
      Compute metrics from the altitude CSV in <run_dir>. Prints JSON.

  baseline <run_dir>...
      Aggregate >=1 run analyses into scripts/golden_runs/baseline_stats.json.

  check <run_dir>
      Compare <run_dir> against baseline_stats.json.
      Exit 0 = PASS, exit 1 = FAIL (reasons printed to stdout).

Metrics computed per run:
  time_to_alt_s       — seconds from controller start to reaching 4.5 m
  rmse_hold_m         — RMSE of altitude vs 5.0 m in the hold window
  sat_frac            — fraction of samples with throttle_output >= 1800
  mean_alt_m          — mean altitude over the hold window
  std_alt_m           — std-dev of altitude over the hold window
  columns             — frozenset of CSV column names (stored as sorted list)
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ALTITUDE_COL = "current_altitude"
THROTTLE_COL = "throttle_output"
TIMESTAMP_COL = "timestamp"

TARGET_ALT = 5.0          # m — nominal hold altitude
ALT_THRESHOLD = 4.5       # m — "reached altitude" threshold
HOLD_SETTLE = 5.0         # s — skip this many seconds after first reaching 4.5 m
THROTTLE_SAT = 1800       # PWM value considered saturated

# Absolute floors for regression check
FLOOR_RMSE = 0.1          # m
FLOOR_ALT = 0.1           # m  (for mean_alt_m, std_alt_m)
FLOOR_TIME = 5.0          # s
FLOOR_SATFRAC = 0.05      # fraction (5 %)

# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def _find_altitude_csv(run_dir: Path) -> Path:
    """Return the altitude CSV file in run_dir."""
    candidates = sorted(run_dir.glob("altitude_control_*.csv"))
    if not candidates:
        raise FileNotFoundError(
            f"No altitude_control_*.csv found in {run_dir}"
        )
    if len(candidates) > 1:
        # Prefer the most recently modified one
        candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def _read_csv(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
    return rows


# ---------------------------------------------------------------------------
# Core metrics computation
# ---------------------------------------------------------------------------

def _compute_metrics(run_dir: Path, metadata: dict | None = None) -> dict[str, Any]:
    csv_path = _find_altitude_csv(run_dir)
    rows = _read_csv(csv_path)

    if not rows:
        raise ValueError(f"Altitude CSV is empty: {csv_path}")

    column_names = list(rows[0].keys())

    # ------------------------------------------------------------------
    # Build parallel numeric arrays
    # ------------------------------------------------------------------
    timestamps: list[float] = []
    altitudes: list[float] = []
    throttles: list[float] = []

    for row in rows:
        try:
            t = float(row[TIMESTAMP_COL])
            a = float(row[ALTITUDE_COL])
            th = float(row.get(THROTTLE_COL, 0))
        except (ValueError, KeyError):
            continue
        timestamps.append(t)
        altitudes.append(a)
        throttles.append(th)

    n = len(timestamps)
    if n == 0:
        raise ValueError("No valid numeric rows found in altitude CSV")

    # ------------------------------------------------------------------
    # Time-to-altitude: from controller_start to first sample >= ALT_THRESHOLD
    # The CSV timestamps are controller-internal (seconds since controller start,
    # from altitude_config); if metadata has wall-clock times we use those.
    # Fallback: use the CSV timestamp of the first sample >= threshold minus
    # the CSV timestamp of the very first sample.
    # ------------------------------------------------------------------
    t0 = timestamps[0]
    t_alt_reached_csv: float | None = None
    idx_alt_reached: int | None = None
    for i, a in enumerate(altitudes):
        if a >= ALT_THRESHOLD:
            t_alt_reached_csv = timestamps[i]
            idx_alt_reached = i
            break

    if metadata is not None:
        wt = metadata.get("wall_times", {})
        ctrl_start = wt.get("controller_start")
        alt_wall = wt.get("altitude_reached")
        land_wall = wt.get("land_command")
        if ctrl_start and alt_wall:
            time_to_alt_s = alt_wall - ctrl_start
        elif t_alt_reached_csv is not None:
            time_to_alt_s = t_alt_reached_csv - t0
        else:
            time_to_alt_s = None
    else:
        time_to_alt_s = (t_alt_reached_csv - t0) if t_alt_reached_csv is not None else None

    # ------------------------------------------------------------------
    # Hold window: from (first sample >= threshold) + HOLD_SETTLE seconds,
    # until the land command timestamp (from metadata) or end of CSV.
    # ------------------------------------------------------------------
    if idx_alt_reached is None:
        # Drone never reached threshold; still compute metrics on full dataset
        hold_start_idx = 0
    else:
        settle_cutoff = timestamps[idx_alt_reached] + HOLD_SETTLE
        hold_start_idx = idx_alt_reached
        for i in range(idx_alt_reached, n):
            if timestamps[i] >= settle_cutoff:
                hold_start_idx = i
                break

    # Determine hold window end: the land command wall time. CSV timestamps
    # are epoch wall-clock (host and container share the clock), so when they
    # look like epoch values we can compare directly; otherwise fall back to
    # offsetting from controller start.
    hold_end_idx = n
    if metadata is not None:
        wt = metadata.get("wall_times", {})
        ctrl_start = wt.get("controller_start")
        land_wall = wt.get("land_command")
        if land_wall:
            if t0 > 1e9:  # epoch timestamps
                land_csv_t = land_wall
            elif ctrl_start:  # relative timestamps
                land_csv_t = t0 + (land_wall - ctrl_start)
            else:
                land_csv_t = None
            if land_csv_t is not None:
                for i in range(hold_start_idx, n):
                    if timestamps[i] >= land_csv_t:
                        hold_end_idx = i
                        break

    hold_alts = altitudes[hold_start_idx:hold_end_idx]
    hold_throttles = throttles[hold_start_idx:hold_end_idx]

    if not hold_alts:
        hold_alts = altitudes
        hold_throttles = throttles

    # RMSE vs TARGET_ALT
    sq_errors = [(a - TARGET_ALT) ** 2 for a in hold_alts]
    rmse = math.sqrt(sum(sq_errors) / len(sq_errors))

    # Saturation fraction
    sat_count = sum(1 for th in hold_throttles if th >= THROTTLE_SAT)
    sat_frac = sat_count / len(hold_throttles)

    # Mean and std altitude
    mean_alt = sum(hold_alts) / len(hold_alts)
    variance = sum((a - mean_alt) ** 2 for a in hold_alts) / len(hold_alts)
    std_alt = math.sqrt(variance)

    return {
        "time_to_alt_s": time_to_alt_s,
        "rmse_hold_m": rmse,
        "sat_frac": sat_frac,
        "mean_alt_m": mean_alt,
        "std_alt_m": std_alt,
        "columns": sorted(column_names),
        "n_samples_hold": len(hold_alts),
        "csv_file": str(csv_path),
    }


# ---------------------------------------------------------------------------
# Sub-commands
# ---------------------------------------------------------------------------

def cmd_analyze(args: list[str]) -> int:
    if len(args) < 1:
        print("Usage: compare_runs.py analyze <run_dir>", file=sys.stderr)
        return 2

    run_dir = Path(args[0])
    if not run_dir.is_dir():
        print(f"ERROR: not a directory: {run_dir}", file=sys.stderr)
        return 2

    meta_path = run_dir / "metadata.json"
    metadata = None
    if meta_path.exists():
        with open(meta_path) as f:
            metadata = json.load(f)

    metrics = _compute_metrics(run_dir, metadata)
    print(json.dumps(metrics, indent=2))
    return 0


def _baseline_path() -> Path:
    """Location of baseline_stats.json, relative to this script."""
    here = Path(__file__).parent
    return here / "golden_runs" / "baseline_stats.json"


def cmd_baseline(args: list[str]) -> int:
    if len(args) < 1:
        print("Usage: compare_runs.py baseline <run_dir>...", file=sys.stderr)
        return 2

    all_metrics: list[dict[str, Any]] = []
    for run_dir_str in args:
        run_dir = Path(run_dir_str)
        if not run_dir.is_dir():
            print(f"WARNING: skipping non-directory: {run_dir}", file=sys.stderr)
            continue
        meta_path = run_dir / "metadata.json"
        metadata = None
        if meta_path.exists():
            with open(meta_path) as f:
                metadata = json.load(f)
        if metadata is not None and metadata.get("exit_status") not in (0, None):
            print(f"WARNING: skipping FAILED run: {run_dir}", file=sys.stderr)
            continue
        try:
            m = _compute_metrics(run_dir, metadata)
            all_metrics.append(m)
        except Exception as e:
            print(f"WARNING: could not compute metrics for {run_dir}: {e}", file=sys.stderr)

    if not all_metrics:
        print("ERROR: no valid runs to aggregate", file=sys.stderr)
        return 2

    # Scalar metrics to aggregate (skip 'columns', 'n_samples_hold', 'csv_file')
    scalar_keys = ["time_to_alt_s", "rmse_hold_m", "sat_frac", "mean_alt_m", "std_alt_m"]

    stats: dict[str, Any] = {}
    for key in scalar_keys:
        values = [m[key] for m in all_metrics if m.get(key) is not None]
        if not values:
            stats[key] = {"mean": None, "std": None, "n": 0}
            continue
        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        std = math.sqrt(variance)
        stats[key] = {"mean": mean, "std": std, "n": len(values), "values": values}

    # Column set: must be identical across all runs; record it
    col_sets = [frozenset(m["columns"]) for m in all_metrics]
    common_cols = col_sets[0]
    for cs in col_sets[1:]:
        if cs != common_cols:
            print("WARNING: column sets differ across runs", file=sys.stderr)
            common_cols = common_cols & cs
    stats["columns"] = sorted(common_cols)
    stats["n_runs"] = len(all_metrics)
    stats["run_dirs"] = args

    out_path = _baseline_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(stats, f, indent=2)

    print(f"Baseline written to: {out_path}")
    print(json.dumps(stats, indent=2))
    return 0


def cmd_check(args: list[str]) -> int:
    if len(args) < 1:
        print("Usage: compare_runs.py check <run_dir>", file=sys.stderr)
        return 2

    run_dir = Path(args[0])
    if not run_dir.is_dir():
        print(f"ERROR: not a directory: {run_dir}", file=sys.stderr)
        return 2

    baseline_path = _baseline_path()
    if not baseline_path.exists():
        print(f"ERROR: baseline_stats.json not found at {baseline_path}", file=sys.stderr)
        return 2

    with open(baseline_path) as f:
        baseline = json.load(f)

    meta_path = run_dir / "metadata.json"
    metadata = None
    if meta_path.exists():
        with open(meta_path) as f:
            metadata = json.load(f)

    metrics = _compute_metrics(run_dir, metadata)

    failures: list[str] = []

    # Absolute floors per metric
    floors: dict[str, float] = {
        "time_to_alt_s": FLOOR_TIME,
        "rmse_hold_m": FLOOR_RMSE,
        "sat_frac": FLOOR_SATFRAC,
        "mean_alt_m": FLOOR_ALT,
        "std_alt_m": FLOOR_ALT,
    }

    scalar_keys = ["time_to_alt_s", "rmse_hold_m", "sat_frac", "mean_alt_m", "std_alt_m"]

    for key in scalar_keys:
        bstat = baseline.get(key, {})
        b_mean = bstat.get("mean")
        b_std = bstat.get("std")
        if b_mean is None:
            continue

        actual = metrics.get(key)
        if actual is None:
            failures.append(f"{key}: no value in run (baseline mean={b_mean:.4g})")
            continue

        floor = floors.get(key, 0.0)
        allowed = max(3 * (b_std or 0.0), floor)
        lo = b_mean - allowed
        hi = b_mean + allowed

        if not (lo <= actual <= hi):
            failures.append(
                f"{key}: {actual:.4g} outside [{lo:.4g}, {hi:.4g}] "
                f"(baseline mean={b_mean:.4g}, std={b_std:.4g}, floor={floor})"
            )

    # Column check
    baseline_cols = frozenset(baseline.get("columns", []))
    run_cols = frozenset(metrics.get("columns", []))
    if baseline_cols and run_cols != baseline_cols:
        missing = baseline_cols - run_cols
        extra = run_cols - baseline_cols
        if missing:
            failures.append(f"columns: missing {sorted(missing)}")
        if extra:
            failures.append(f"columns: unexpected {sorted(extra)}")

    if failures:
        print("FAIL")
        for reason in failures:
            print(f"  - {reason}")
        return 1
    else:
        print("PASS")
        print(json.dumps({k: metrics[k] for k in scalar_keys if k in metrics}, indent=2))
        return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

USAGE = """\
Usage:
  compare_runs.py analyze <run_dir>
  compare_runs.py baseline <run_dir>...
  compare_runs.py check <run_dir>
"""


def main() -> int:
    if len(sys.argv) < 2:
        print(USAGE, file=sys.stderr)
        return 2

    subcmd = sys.argv[1]
    rest = sys.argv[2:]

    if subcmd == "analyze":
        return cmd_analyze(rest)
    elif subcmd == "baseline":
        return cmd_baseline(rest)
    elif subcmd == "check":
        return cmd_check(rest)
    else:
        print(f"Unknown subcommand: {subcmd}\n{USAGE}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
