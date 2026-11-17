#!/usr/bin/env python3
"""Real box-lift policy trace -> replay file for fr3_command_response_probe (test A4/B4).

The trace stores the absolute joint targets the policy sent (``q_target_0..13``,
one row per policy step).  This script takes one arm, resamples the targets onto a
uniform grid with zero-order hold (the controller holds each target until the next
one), and writes them relative to the recorded start pose:

    <out>.csv   t_s, d_<joint>...   (target - q_start)
    <out>.json  q_start, source, dt, arm, time window, peak deltas

The probe refuses to replay unless the arm starts within
``replay_start_tolerance_rad`` of ``q_start``; the script prints the start pose.
Without a box the squeeze phase drives the plate past where the box would have
stopped it, so check the clearance or cut the window with --t0 / --t1.

The policy lifts fast (joint 2 about 1 rad/s, |delta| up to ~1.3 rad over a 10 s
episode); that is the real command stream, so replay it only with the workspace clear.

Example (5499 real run, 10.4 s):
    python3 scripts/tools/make_boxlift_replay.py \
        data/dual_fr3_boxlift_real/20261107_194728_296023_KST --arm left \
        --out data/command_response/replay/boxlift_5499_194728_left
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_trace(run_dir: Path) -> tuple[dict, np.ndarray, np.ndarray]:
    meta = json.loads((run_dir / "metadata.json").read_text())
    with open(run_dir / "trace.csv", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"{run_dir}: empty trace")
    n = len(meta["joint_names"])
    t = np.asarray([float(r["elapsed_s"]) for r in rows])
    q_target = np.asarray([[float(r[f"q_target_{i}"]) for i in range(n)] for r in rows])
    keep = np.all(np.isfinite(q_target), axis=1)
    return meta, t[keep], q_target[keep]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path, help="data/dual_fr3_boxlift_real/<stamp> (trace.csv + metadata.json)")
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--out", type=Path, required=True, help="output path without extension")
    parser.add_argument("--dt", type=float, default=0.05, help="replay grid (s), policy step 0.05")
    parser.add_argument("--t0", type=float, default=None, help="window start, trace elapsed_s")
    parser.add_argument("--t1", type=float, default=None, help="window end, trace elapsed_s")
    parser.add_argument("--fade_s", type=float, default=0.5,
                        help="ramp the first target in from q_start over this time (s)")
    args = parser.parse_args()

    meta, t, q_target = load_trace(args.run_dir)
    names = list(meta["joint_names"])
    cols = [i for i, name in enumerate(names) if name.startswith(f"{args.arm}_")]
    if len(cols) != 7:
        raise SystemExit(f"expected 7 {args.arm} joints in {names}")
    arm_names = [names[i] for i in cols]
    q_start = np.asarray(meta["joint_start_position"], dtype=np.float64)[cols]

    t0 = t[0] if args.t0 is None else args.t0
    t1 = t[-1] if args.t1 is None else args.t1
    if t1 <= t0:
        raise SystemExit("empty window")
    grid = np.arange(0.0, t1 - t0 + 1e-9, args.dt)
    # zero-order hold: the latest target at or before each grid time
    idx = np.searchsorted(t, grid + t0, side="right") - 1
    idx = np.clip(idx, 0, len(t) - 1)
    deltas = q_target[idx][:, cols] - q_start
    if args.t0 is not None and args.t0 > t[0]:
        # The window starts mid-run: the first sample can be far from q_start.
        ramp = max(int(round(args.fade_s / args.dt)), 1)
        weights = np.minimum(np.arange(1, len(grid) + 1) / ramp, 1.0)[:, None]
        deltas = deltas * weights

    args.out.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.out.with_suffix(".csv")
    with open(csv_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s"] + [f"d_{n}" for n in arm_names])
        for k, row in enumerate(deltas):
            writer.writerow([f"{grid[k]:.4f}"] + [f"{v:.6f}" for v in row])

    step = np.abs(np.diff(deltas, axis=0)) if len(deltas) > 1 else np.zeros((1, 7))
    info = {
        "source": str(args.run_dir),
        "arm": args.arm,
        "dt": args.dt,
        "window_s": [float(t0), float(t1)],
        "rows": int(len(grid)),
        "q_start": {n: float(v) for n, v in zip(arm_names, q_start)},
        "peak_abs_delta": {n: float(v) for n, v in zip(arm_names, np.max(np.abs(deltas), axis=0))},
        "peak_step_per_dt": {n: float(v) for n, v in zip(arm_names, np.max(step, axis=0))},
        "trace_mode": meta.get("mode", ""),
        "trace_start_mode": meta.get("start_mode", ""),
        "trace_end_reason": meta.get("end_reason", ""),
    }
    csv_path.with_suffix(".json").write_text(json.dumps(info, indent=2))
    print(f"wrote {csv_path} ({len(grid)} rows, {grid[-1]:.2f} s)")
    print("start pose (move_to_joint before start_probe):")
    print("  [" + ", ".join(f"{v:.4f}" for v in q_start) + "]")
    print("peak |delta| rad: " + ", ".join(f"{v:.3f}" for v in np.max(np.abs(deltas), axis=0)))
    print("peak step per dt rad: " + ", ".join(f"{v:.3f}" for v in np.max(step, axis=0)))


if __name__ == "__main__":
    main()
