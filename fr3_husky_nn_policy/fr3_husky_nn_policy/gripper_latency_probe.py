"""Measure Franka gripper command latency and finger speed on the real robot (no arm motion).

Sends franka_msgs Move / Grasp goals in cycles and records every gripper joint state, then reports per command:

- accept:   send -> goal accepted
- motion:   send -> first width sample that moved > ``motion_threshold_m`` from the width at send (upper bound)
- onset:    motion start extrapolated back from that sample with the measured speed (best estimate)
- 90%:      send -> width within 10 % of the travel from the final width
- speed:    finger-width speed from a linear fit of the width updates within 10-90 % of the travel, m/s of total width
- result:   send -> action result
- settle:   last width change -> result (time the server waits after the fingers stopped)

Times use the joint-state header stamp when it is set, else the receive time. The real franka_gripper node
publishes at ~10 Hz but the width value itself only changes every ~0.25 s (2026-11-08), so only samples whose
width changed (state updates) are used for speed and onset; ``update_interval`` reports that period.

Sequence per speed and cycle: open (Move to ``open_width``) -> grasp (Grasp to ``grasp_width``, empty
fingers by default) -> open; with ``--move-close`` also Move to ``move_close_width`` -> open, to compare Move and
Grasp latency. Output: ``<out>/<stamp>/{widths.csv, events.csv, summary.csv, summary.json}``.

    ros2 run fr3_husky_nn_policy gripper_latency_probe --side right --cycles 10 --speeds 0.05 0.1
    ros2 run fr3_husky_nn_policy gripper_latency_probe --analyze data/gripper_latency/<stamp>
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import time
from datetime import datetime
from pathlib import Path

import numpy as np

EVENT_FIELDS = ("index", "cycle", "command", "speed_mps", "target_width_m", "send_ns", "accept_ns", "result_ns",
                "accepted", "success", "error", "width_at_send_m", "width_at_result_m")
WIDTH_FIELDS = ("recv_ns", "stamp_ns", "width_m")


def _default_out_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "data" / "gripper_latency"


# --------------------------------------------------------------------------- analysis (ROS-free)
def _load(run_dir: Path):
    with (run_dir / "widths.csv").open() as f:
        rows = list(csv.DictReader(f))
    recv = np.array([int(r["recv_ns"]) for r in rows], dtype=np.int64)
    stamp = np.array([int(r["stamp_ns"]) for r in rows], dtype=np.int64)
    width = np.array([float(r["width_m"]) for r in rows])
    t = np.where(stamp > 0, stamp, recv)
    order = np.argsort(t, kind="stable")
    with (run_dir / "events.csv").open() as f:
        events = list(csv.DictReader(f))
    return t[order], width[order], recv[order], events


def analyze_command(t, w, event, motion_threshold_m=0.001):
    """Latency metrics (s) and speed (m/s) of one command from the width samples."""
    send = int(event["send_ns"])
    result = int(event["result_ns"]) if event["result_ns"] else None
    end = result if result else send + int(10e9)
    before = np.nonzero(t <= send)[0]
    if len(before) == 0:
        return None
    w0 = w[before[-1]]
    sel = (t > send) & (t <= end + int(0.3e9))
    ts, ws = t[sel], w[sel]
    out = {"index": int(event["index"]), "cycle": int(event["cycle"]), "command": event["command"],
           "speed_mps": float(event["speed_mps"]), "width_start_m": round(float(w0), 5),
           "accept_s": (int(event["accept_ns"]) - send) * 1e-9 if event["accept_ns"] else float("nan"),
           "result_s": (result - send) * 1e-9 if result else float("nan"),
           "success": event["success"]}
    moved = np.nonzero(np.abs(ws - w0) > motion_threshold_m)[0]
    if len(moved) == 0:
        out.update(motion_s=float("nan"), onset_s=float("nan"), t90_s=float("nan"), speed_mps_measured=float("nan"),
                   settle_s=float("nan"), width_end_m=round(float(ws[-1] if len(ws) else w0), 5), travel_m=0.0)
        return out
    w_end = ws[-1]
    travel = w_end - w0
    out["width_end_m"], out["travel_m"] = round(float(w_end), 5), round(float(travel), 5)
    out["motion_s"] = (ts[moved[0]] - send) * 1e-9
    frac = (ws - w0) / travel if abs(travel) > 1e-6 else np.zeros_like(ws)
    reach = np.nonzero(frac >= 0.9)[0]
    out["t90_s"] = (ts[reach[0]] - send) * 1e-9 if len(reach) else float("nan")
    # state updates: first sample of each new width value (repeated values are the same stale reading)
    update = np.r_[True, np.abs(np.diff(ws)) > 1e-4]
    ramp = update & (frac > 0.1) & (frac < 0.9)
    if ramp.sum() >= 2:
        slope = np.polyfit((ts[ramp] - send) * 1e-9, ws[ramp], 1)[0]
        out["speed_mps_measured"] = abs(float(slope))
    else:  # one update inside the ramp: first change -> 90 %
        out["speed_mps_measured"] = (abs(travel) * 0.9 - abs(ws[moved[0]] - w0)) / (out["t90_s"] - out["motion_s"]) \
            if len(reach) and out["t90_s"] > out["motion_s"] else float("nan")
    speed = out["speed_mps_measured"]
    out["onset_s"] = (out["motion_s"] - abs(ws[moved[0]] - w0) / speed
                      if np.isfinite(speed) and speed > 1e-3 else out["motion_s"])
    changes = np.nonzero(np.abs(np.diff(ws)) > 0.0005)[0]
    last_move = ts[changes[-1] + 1] if len(changes) else ts[moved[0]]
    out["settle_s"] = (result - last_move) * 1e-9 if result else float("nan")
    return out


def _stats(values):
    v = [x for x in values if np.isfinite(x)]
    if not v:
        return {"n": 0}
    return {"n": len(v), "median": statistics.median(v), "min": min(v), "max": max(v),
            "std": statistics.pstdev(v) if len(v) > 1 else 0.0}


def analyze_run(run_dir: Path, motion_threshold_m=0.001, quiet=False):
    t, w, recv, events = _load(run_dir)
    per = [m for m in (analyze_command(t, w, e, motion_threshold_m) for e in events) if m]
    keys = ("accept_s", "onset_s", "motion_s", "t90_s", "speed_mps_measured", "result_s", "settle_s",
            "width_end_m")
    groups = {}
    for m in per:
        if m["travel_m"] == 0.0 and m["command"] == "open" and m["width_start_m"] > 0.075:
            label = "open(already open)"
        else:
            label = m["command"]
        groups.setdefault((label, m["speed_mps"]), []).append(m)
    summary = []
    for (label, speed), ms in sorted(groups.items()):
        row = {"command": label, "speed_mps": speed}
        for k in keys:
            row[k] = _stats([m[k] for m in ms])
        summary.append(row)
    gaps = np.diff(recv) * 1e-9
    changes = t[1:][np.abs(np.diff(w)) > 1e-4]
    update_gaps = np.diff(changes) * 1e-9
    update_gaps = update_gaps[update_gaps < 0.6]  # only while the fingers move
    topic = {"samples": int(len(recv)), "rate_hz": float(1.0 / np.median(gaps)) if len(gaps) else float("nan"),
             "gap_p95_s": float(np.percentile(gaps, 95)) if len(gaps) else float("nan"),
             "gap_max_s": float(gaps.max()) if len(gaps) else float("nan"),
             "width_update_interval_median_s": float(np.median(update_gaps)) if len(update_gaps) else float("nan"),
             "width_update_interval_p95_s": float(np.percentile(update_gaps, 95)) if len(update_gaps) else float("nan")}
    with (run_dir / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(per[0].keys()) if per else ["index"])
        writer.writeheader()
        writer.writerows(per)
    (run_dir / "summary.json").write_text(json.dumps({"groups": summary, "joint_state_topic": topic}, indent=2) + "\n")
    if not quiet:
        print(f"run {run_dir}")
        print(f"joint states: {topic['samples']} samples, {topic['rate_hz']:.1f} Hz, gap p95 "
              f"{1e3 * topic['gap_p95_s']:.0f} ms, max {1e3 * topic['gap_max_s']:.0f} ms; width value changes every "
              f"{1e3 * topic['width_update_interval_median_s']:.0f} ms (p95 "
              f"{1e3 * topic['width_update_interval_p95_s']:.0f}) while moving")
        print(f"{'command':20s} {'speed':>6s} {'n':>3s}  {'accept ms':>11s} {'onset ms':>15s} {'motion ms':>15s} {'90% ms':>15s} "
              f"{'speed m/s':>15s} {'result ms':>15s} {'settle ms':>11s} {'end width mm':>16s}")
        fmt = lambda s, scale=1e3: (f"{scale * s['median']:6.0f} [{scale * s['min']:.0f}-{scale * s['max']:.0f}]"
                                    if s.get("n") else "-")
        fmt_v = lambda s: f"{s['median']:.3f} [{s['min']:.3f}-{s['max']:.3f}]" if s.get("n") else "-"
        for row in summary:
            print(f"{row['command']:20s} {row['speed_mps']:6.3f} {row['motion_s'].get('n', 0) or row['result_s'].get('n', 0):3d}"
                  f"  {fmt(row['accept_s']):>11s} {fmt(row['onset_s']):>15s} {fmt(row['motion_s']):>15s} {fmt(row['t90_s']):>15s} "
                  f"{fmt_v(row['speed_mps_measured']):>15s} {fmt(row['result_s']):>15s} {fmt(row['settle_s']):>11s} "
                  f"{fmt(row['width_end_m']):>16s}")
        print("median [min-max]. motion = send -> first width sample moved > "
              f"{1e3 * motion_threshold_m:.0f} mm (upper bound); onset = motion extrapolated back with the speed; "
              "speed and end width are total finger width (finger_joint1 = width / 2)")
    return summary


# --------------------------------------------------------------------------- probe (ROS)
def run_probe(args) -> Path:
    import rclpy
    from franka_msgs.action import Grasp, Move
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    from rclpy.signals import SignalHandlerOptions

    # Ctrl-C raises KeyboardInterrupt but keeps the context alive, so the recorded data can still be written.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("gripper_latency_probe")
    ns = f"/{args.side}_franka_gripper"
    finger_names = (f"{args.side}_fr3_finger_joint1", f"{args.side}_fr3_finger_joint2")
    samples = []  # buffered until the servers are up, so a failed start leaves no run directory
    latest = {"width": float("nan")}

    def on_joint_state(msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index and index[n] < len(msg.position) for n in finger_names):
            return
        width = float(sum(msg.position[index[n]] for n in finger_names))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        samples.append([node.get_clock().now().nanoseconds, stamp, f"{width:.6f}"])
        latest["width"] = width

    node.create_subscription(JointState, f"{ns}/joint_states", on_joint_state, 100)
    move_client = ActionClient(node, Move, f"{ns}/move")
    grasp_client = ActionClient(node, Grasp, f"{ns}/grasp")
    for client, name in ((move_client, "move"), (grasp_client, "grasp")):
        if not client.wait_for_server(timeout_sec=5.0):
            raise SystemExit(f"{ns}/{name} action server not available (robot launch with load_gripper:=true?)")
    t0 = time.monotonic()
    while not np.isfinite(latest["width"]) and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not np.isfinite(latest["width"]):
        raise SystemExit(f"no {ns}/joint_states with {finger_names}")

    out_dir = Path(args.out).expanduser() / datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%Z")
    out_dir.mkdir(parents=True)
    width_file = (out_dir / "widths.csv").open("w", newline="")
    width_writer = csv.writer(width_file)
    width_writer.writerow(WIDTH_FIELDS)

    def flush_samples():
        width_writer.writerows(samples)
        samples.clear()
        width_file.flush()

    (out_dir / "metadata.json").write_text(json.dumps({"started_at": datetime.now().astimezone().isoformat(),
                                                       "args": vars(args), "gripper_ns": ns}, indent=2) + "\n")
    event_file = (out_dir / "events.csv").open("w", newline="")
    event_writer = csv.DictWriter(event_file, fieldnames=EVENT_FIELDS)
    event_writer.writeheader()
    counter = {"i": 0}

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def send(cycle, command, speed, width):
        if command == "grasp":
            goal = Grasp.Goal()
            goal.width, goal.speed, goal.force = width, speed, args.force
            goal.epsilon.inner = goal.epsilon.outer = args.epsilon
            client = grasp_client
        else:
            goal = Move.Goal()
            goal.width, goal.speed = width, speed
            client = move_client
        row = {"index": counter["i"], "cycle": cycle, "command": command, "speed_mps": speed, "target_width_m": width,
               "width_at_send_m": f"{latest['width']:.6f}", "accept_ns": "", "result_ns": "", "accepted": "",
               "success": "", "error": "", "width_at_result_m": ""}
        counter["i"] += 1
        print(f"  cycle {cycle} {command:5s} speed {speed:.3f} -> width {width:.3f} ...", end="", flush=True)
        row["send_ns"] = node.get_clock().now().nanoseconds
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        handle = future.result()
        row["accept_ns"] = node.get_clock().now().nanoseconds
        row["accepted"] = bool(handle is not None and handle.accepted)
        if row["accepted"]:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
            if result_future.done():
                result = result_future.result().result
                row["result_ns"] = node.get_clock().now().nanoseconds
                row["success"], row["error"] = bool(result.success), str(result.error)
            else:
                row["error"] = f"no result within {args.timeout:.0f} s"
        row["width_at_result_m"] = f"{latest['width']:.6f}"
        event_writer.writerow(row)
        event_file.flush()
        flush_samples()
        print(f" accepted {row['accepted']}, success {row['success']}, "
              f"{(int(row['result_ns'] or row['accept_ns']) - row['send_ns']) * 1e-6:.0f} ms, "
              f"width {float(row['width_at_send_m']) * 1e3:.1f} -> {latest['width'] * 1e3:.1f} mm"
              + (f"  ({row['error']})" if row["error"] else ""))
        wait(args.settle)

    try:
        for speed in args.speeds:
            print(f"speed {speed} m/s")
            send(-1, "open", speed, args.open_width)  # start from open
            for cycle in range(args.cycles):
                send(cycle, "grasp", speed, args.grasp_width)
                send(cycle, "open", speed, args.open_width)
                if args.move_close:
                    send(cycle, "close", speed, args.move_close_width)
                    send(cycle, "open", speed, args.open_width)
        if args.idle_open:  # Move to the current width: pure command overhead
            for cycle in range(args.cycles):
                send(cycle, "open", args.speeds[-1], args.open_width)
    except KeyboardInterrupt:
        print("\ninterrupted; analysing what was recorded (the last command may still be running on the gripper)")
    finally:
        try:
            wait(0.3)
        except Exception:  # never lose the recorded data over the final spin
            pass
        flush_samples()
        event_file.close()
        width_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return out_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description="Franka gripper latency / speed probe (arms do not move)")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.1], help="m/s; the cube-lift node uses 0.1")
    parser.add_argument("--grasp-width", type=float, default=0.0)
    parser.add_argument("--epsilon", type=float, default=0.08)
    parser.add_argument("--force", type=float, default=30.0)
    parser.add_argument("--open-width", type=float, default=0.08)
    parser.add_argument("--move-close", action="store_true", help="also time Move-to-close (vs Grasp)")
    parser.add_argument("--move-close-width", type=float, default=0.01)
    parser.add_argument("--idle-open", action="store_true", help="also time Move to the current (open) width")
    parser.add_argument("--settle", type=float, default=1.0, help="s to wait after each result")
    parser.add_argument("--timeout", type=float, default=6.0, help="s per goal response / result")
    parser.add_argument("--motion-threshold", type=float, default=0.001, help="m of width change = motion start")
    parser.add_argument("--out", default=str(_default_out_dir()))
    parser.add_argument("--yes", action="store_true", help="skip the 'fingers clear?' prompt")
    parser.add_argument("--analyze", metavar="RUN_DIR", help="only re-analyse a recorded run")
    args = parser.parse_args(argv)
    if args.analyze:
        analyze_run(Path(args.analyze), args.motion_threshold)
        return
    if not args.yes:
        answer = input(f"{args.side} gripper will open/close {args.cycles} x {len(args.speeds)} times. "
                       "Fingers clear (nothing between them unless intended)? [y/N] ")
        if answer.strip().lower() != "y":
            return
    run_dir = run_probe(args)
    analyze_run(run_dir, args.motion_threshold)


if __name__ == "__main__":
    main()
