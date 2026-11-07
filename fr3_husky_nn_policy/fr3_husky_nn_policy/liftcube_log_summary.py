"""Summarize one cube-lift Sim2Real run directory (liftcube_logger output), mainly the Franka gripper delay.

    ros2 run fr3_husky_nn_policy liftcube_log_summary [run_dir]     # default: newest run under <package>/data

For every gripper goal the node sent: time to 'accepted', to the first finger motion seen in trace.csv
(|width - width at send| > --motion_m), and to the action 'result'. The sim gripper (implicit PD, stiffness 2e3)
closes in about 0.1 s, so compare these numbers with that.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def _float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _newest_run(root: Path) -> Path:
    runs = sorted(p.parent for p in root.glob("*/*/metadata.json"))
    if not runs:
        raise SystemExit(f"no run directories under {root}")
    return max(runs, key=lambda p: p.stat().st_mtime)


def _stats(values: list[float]) -> str:
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return "n/a"
    values.sort()
    return f"mean {sum(values) / len(values):.1f}, p95 {values[int(0.95 * (len(values) - 1))]:.1f}, max {values[-1]:.1f}"


def summarize(run_dir: Path, motion_m: float) -> None:
    meta = json.loads((run_dir / "metadata.json").read_text())
    rows = list(csv.DictReader((run_dir / "trace.csv").open()))
    events = list(csv.DictReader((run_dir / "gripper_events.csv").open()))
    print(f"run       {run_dir}")
    print(f"mode      {meta.get('mode')}, end: {meta.get('end_reason', '(not finalized)')}")
    print(f"cube      start {meta.get('cube_start_position_base')}, target {meta.get('target_position_base')}")
    print(f"steps     {len(rows)} ({len(rows) / 20.0:.2f} s at 20 Hz)")
    if not rows:
        return
    print(f"tick      {_stats([_float(r['tick_period_ms']) for r in rows])} ms (target 50)")
    print(f"cube age  received {_stats([_float(r['object_pose_age_ms']) for r in rows])} ms, "
          f"stamp {_stats([_float(r['object_stamp_age_ms']) for r in rows])} ms")
    print(f"joint err {_stats([1000 * _float(r['feedback_max_joint_error_rad']) for r in rows])} mrad (controller)")
    first, last = rows[0], rows[-1]
    lift = _float(last["obs_20"]) - _float(first["obs_20"])
    print(f"cube z    {_float(first['obs_20']):.3f} -> {_float(last['obs_20']):.3f} m (lift {100 * lift:+.1f} cm), "
          f"end distance to target {100 * math.dist([_float(last[f'obs_{i}']) for i in (18, 19, 20)], [_float(last[f'obs_{i}']) for i in (21, 22, 23)]):.1f} cm")
    print(f"width     {_float(first['finger_width_m']) * 1000:.1f} -> {_float(last['finger_width_m']) * 1000:.1f} mm")

    t = [_float(r["elapsed_s"]) for r in rows]
    width = [_float(r["finger_width_m"]) for r in rows]
    sends = [e for e in events if e["event"] in ("send", "would_send")]
    if not sends:
        print("gripper   no goals sent (policy never crossed the hysteresis)")
        return
    print("gripper   cmd     send[s]  accept[ms]  motion[ms]  result[ms]  result  width at result[mm]")
    for k, send in enumerate(sends):
        t0, command = _float(send["elapsed_s"]), send["command"]
        later = [e for e in events[events.index(send) + 1:] if e["command"] == command]
        nxt = sends[k + 1] if k + 1 < len(sends) else None
        if nxt is not None:
            later = [e for e in later if _float(e["elapsed_s"]) <= _float(nxt["elapsed_s"])]
        accepted = next((e for e in later if e["event"] in ("accepted", "rejected")), None)
        result = next((e for e in later if e["event"] == "result"), None)
        w0 = _float(send["finger_width_m"])
        motion = next((ti for ti, wi in zip(t, width) if ti >= t0 and abs(wi - w0) > motion_m), math.nan)
        ms = lambda e: f"{1000 * (_float(e['elapsed_s']) - t0):10.0f}" if e else f"{'-':>10}"  # noqa: E731
        motion_ms = f"{1000 * (motion - t0):10.0f}" if math.isfinite(motion) else f"{'-':>10}"
        res = f"{result['success']:>6}  {1000 * _float(result['finger_width_m']):8.1f}" if result else f"{'-':>6}"
        tag = "" if send["event"] == "send" else " (shadow)"
        print(f"          {command:6} {t0:8.2f}  {ms(accepted):>10}  {motion_ms}  {ms(result)}  {res}{tag}")
    print(f"          motion = first 20 Hz trace row with |width change| > {motion_m * 1000:.0f} mm (+-50 ms)")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_dir", nargs="?", help="run directory (default: newest under <package>/data)")
    parser.add_argument("--motion_m", type=float, default=0.002, help="finger width change counted as motion [m]")
    args = parser.parse_args(argv)
    run_dir = Path(args.run_dir).expanduser() if args.run_dir else _newest_run(Path(__file__).resolve().parents[2] / "data")
    summarize(run_dir, args.motion_m)


if __name__ == "__main__":
    main()
