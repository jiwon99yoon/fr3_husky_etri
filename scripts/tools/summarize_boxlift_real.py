"""Success rates of real-robot box-lift sessions per policy, from the etri node's ``results.csv`` (+ ``trace.csv``).

The etri ``ppo_boxlift_policy_node`` writes one row per session to ``experi_result/<log_task_name>/results*.csv`` at the
trained episode end (or at the stop/fault if the session ended earlier), with the box pose the policy saw (vision,
``box_position_offset_m`` applied) versus the goal. The node has no success flag; this applies the training criterion
offline, as for the 5499 report (``docs/dual_fr3_boxlift_5499_report.md``):

* success: box center to goal 3D distance < ``--goal_tol`` (default 0.10 m; ``--goal_tol2`` 0.05 m also reported) and
  tilt < ``--tilt_deg`` (15 deg). Training also needs both plates in contact for 1.25 s, which the robot cannot measure.
* sessions that ended before the trained episode end (operator stop, fault) count as failures; faults are listed.
* with the session folder (``trace.csv``; ``log_dir`` of the row, else ``--data_root``/<session>): ``lifted`` = the box
  center rose >= ``--lift_m`` (5 cm) above its start at some point, ``dropped`` = lifted but less than ``--lift_m``
  above the start at the end of the logged window up to the trained episode end. Box z is ``obs_30`` (the
  ``object_position`` term of the 56D observation).

Policy label: ``..._curriculum_<label>_model<it>.npz`` -> ``<label>`` (v2, nodelay, realdrop_sysid, ...), else the file
name. Rows are filtered by ``--mode`` (command) and ``--start_pose`` (hover; rows of older nodes have no start_pose).

    python scripts/tools/summarize_boxlift_real.py <etri>/experi_result/dual_fr3_boxlift_torque_eval/results*.csv \
        --data_root <etri>/data/dual_fr3_boxlift_torque_eval [--trials] [--md out.md]
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("results", nargs="+", type=Path)
ap.add_argument("--data_root", type=Path, default=None, help="folder with <session>/trace.csv (if log_dir is elsewhere)")
ap.add_argument("--mode", default="command", help="command, shadow or any")
ap.add_argument("--start_pose", default="hover", help="hover, pregrasp_table, ready_pose or any")
ap.add_argument("--since", default="", help="only sessions whose name sorts >= this (e.g. 20261124_1300)")
ap.add_argument("--goal_tol", type=float, default=0.10)
ap.add_argument("--goal_tol2", type=float, default=0.05)
ap.add_argument("--tilt_deg", type=float, default=15.0)
ap.add_argument("--lift_m", type=float, default=0.05)
ap.add_argument("--trials", action="store_true", help="also print every session")
ap.add_argument("--md", type=Path, default=None, help="write the tables as markdown")
args = ap.parse_args()

LABEL = re.compile(r"_curriculum_(.+)_model\d+\.npz$")


def label(model: str) -> str:
    m = LABEL.search(model)
    return m.group(1) if m else model.removesuffix(".npz")


def fnum(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def trace_for(row: dict) -> Path | None:
    candidates = [Path(row.get("log_dir", "")) / "trace.csv"]
    if args.data_root is not None:
        candidates.append(args.data_root / row["session"] / "trace.csv")
    return next((p for p in candidates if p.is_file()), None)


def lift_info(path: Path | None, episode_s: float) -> tuple[float, float]:
    """(max lift, lift at the trained episode end or the last row) of the observed box center z [m]."""
    if path is None:
        return math.nan, math.nan
    z, t = [], []
    with path.open(newline="", encoding="utf-8") as handle:
        for r in csv.DictReader(handle):
            z.append(fnum(r.get("obs_30")))
            t.append(fnum(r.get("elapsed_s")))
    z = [v for v in z if math.isfinite(v)]
    if not z:
        return math.nan, math.nan
    window = [v for v, s in zip(z, t) if not (s > episode_s + 1e-6)] or z
    return max(window) - z[0], window[-1] - z[0]


rows = []
for path in args.results:
    with path.open(newline="", encoding="utf-8") as handle:
        for r in csv.DictReader(handle):
            if args.mode != "any" and r.get("mode") != args.mode:
                continue
            if args.start_pose != "any" and r.get("start_pose", "") != args.start_pose:
                continue
            if r["session"] < args.since:
                continue
            reason = r.get("end_reason", "")
            m = re.search(r"trained episode end \(([\d.]+) s\)", reason)
            episode_s = float(m.group(1)) if m else fnum(r.get("duration_s"))
            err, tilt = fnum(r.get("error_xyz_norm")), fnum(r.get("tilt_deg"))
            completed = m is not None
            max_lift, end_lift = lift_info(trace_for(r), episode_s if math.isfinite(episode_s) else 1e9)
            rows.append({
                "session": r["session"], "policy": label(r.get("model", "")), "completed": completed,
                "fault": reason.startswith("fault"), "reason": reason, "err": err, "tilt": tilt,
                "ok1": completed and err < args.goal_tol and tilt < args.tilt_deg,
                "ok2": completed and err < args.goal_tol2 and tilt < args.tilt_deg,
                "goal_z": fnum(r.get("goal_z")), "start_x": fnum(r.get("start_x")), "start_y": fnum(r.get("start_y")),
                "max_lift": max_lift, "end_lift": end_lift,
                "lifted": max_lift >= args.lift_m, "dropped": max_lift >= args.lift_m and end_lift < args.lift_m,
            })

if not rows:
    raise SystemExit("no sessions left after the filters (check --mode / --start_pose / --since)")

order = list(dict.fromkeys(r["policy"] for r in sorted(rows, key=lambda r: r["session"])))
lines = [
    f"criterion: 3D error < {100 * args.goal_tol:.0f} cm (and < {100 * args.goal_tol2:.0f} cm) and tilt < {args.tilt_deg:.0f} deg "
    f"at the trained episode end; mode {args.mode}, start_pose {args.start_pose}",
    "",
    f"| policy | n | success ({100 * args.goal_tol:.0f} cm) | success ({100 * args.goal_tol2:.0f} cm) | 3D error median (completed) "
    "| tilt median | lifted | dropped after lift | faults | stopped early |",
    "|---|---|---|---|---|---|---|---|---|---|",
]
for name in order:
    group = [r for r in rows if r["policy"] == name]
    n = len(group)
    done = [r for r in group if r["completed"]]
    has_trace = [r for r in group if math.isfinite(r["max_lift"])]

    def pct(k: int) -> str:
        return f"{k}/{n} ({100.0 * k / n:.0f} %)"

    med = lambda key: statistics.median([r[key] for r in done]) if done else math.nan  # noqa: E731
    lifted = f"{sum(r['lifted'] for r in has_trace)}/{len(has_trace)}" if has_trace else "-"
    dropped = f"{sum(r['dropped'] for r in has_trace)}" if has_trace else "-"
    lines.append(
        f"| {name} | {n} | {pct(sum(r['ok1'] for r in group))} | {pct(sum(r['ok2'] for r in group))} "
        f"| {100 * med('err'):.1f} cm | {med('tilt'):.1f} deg | {lifted} | {dropped} "
        f"| {sum(r['fault'] for r in group)} | {sum(not r['completed'] and not r['fault'] for r in group)} |")

if args.trials:
    lines += ["", "| session | policy | box start (x, y) | goal z | end | 3D error | tilt | max lift | lift at end | result |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda r: r["session"]):
        result = "success" if r["ok1"] else ("fault" if r["fault"] else "fail")
        lines.append(
            f"| {r['session']} | {r['policy']} | ({r['start_x']:.3f}, {r['start_y']:+.3f}) | {r['goal_z']:.2f} "
            f"| {r['reason']} | {100 * r['err']:.1f} cm | {r['tilt']:.1f} | {100 * r['max_lift']:.1f} cm "
            f"| {100 * r['end_lift']:.1f} cm | {result} |")

text = "\n".join(lines)
print(text)
if args.md is not None:
    args.md.write_text(text + "\n", encoding="utf-8")
    print(f"\nwrote {args.md}")
