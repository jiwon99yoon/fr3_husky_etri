"""Per-run log of the cube-lift Sim2Real and cube pick-and-place policy nodes (ROS-free).

One directory per start_policy session::

    <log_root>/<task_name>/<YYYYmmdd_HHMMSS_ffffff_TZ>/
        metadata.json        model, node parameters, target, start/end time, end reason, step count
        trace.csv            one row per policy tick (timing, full observation (lift 36-D, pick-place 37-D), action,
                             joint target, gripper)
        gripper_events.csv   every gripper goal the node sends and the server's answers (accepted, result)

The observation columns are exactly the actor input, so a run can be replayed offline through the
same NPZ (``obs_*`` -> actor -> ``action_*``). The gripper columns and events give the command-to-motion
delay of the Franka gripper: compare ``gripper_closed_cmd`` with ``finger_width_m`` and the event times.
"""

from __future__ import annotations

import array
import csv
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import numpy as np

OBS_DIM = 36
ACTION_DIM = 8
ARM_DOF = 7

TIMING_FIELDS = (
    "elapsed_s",            # monotonic time since the session started
    "ros_time_ns",
    "step",                 # policy step index since start (0 = first command)
    "tick_period_ms",       # actual time since the previous tick (target = 50 ms)
    "joint_state_age_ms",   # oldest of the joint samples used in this tick
    "object_pose_age_ms",   # time since the cube pose was received by this node
    "object_stamp_age_ms",  # now - cube pose header stamp (includes vision processing latency)
    "inference_ms",         # observation build + actor + action processing
    "max_abs_delta_rad",    # max |q_target - q| sent this tick (<= 0.1 rad by construction)
    "feedback_max_joint_error_rad",  # latest controller feedback (NaN before the first one)
    "feedback_command_age_s",
)
GRIPPER_FIELDS = (
    "gripper_raw_action",   # actor output 7 (sim: close when < 0)
    "gripper_closed_cmd",   # 1 after hysteresis = close, 0 = open (what the node asked for)
    "finger_width_m",       # measured right finger_joint1 + finger_joint2
    "gripper_cmd_age_s",    # time since the last gripper goal was sent (NaN before the first one)
)


def fieldnames(obs_dim: int = OBS_DIM) -> tuple[str, ...]:
    return (
        *TIMING_FIELDS,
        *GRIPPER_FIELDS,
        *(f"obs_{i}" for i in range(obs_dim)),
        *(f"action_{i}" for i in range(ACTION_DIM)),
        *(f"q_target_{i}" for i in range(ARM_DOF)),
        *(f"object_quat_raw_{axis}" for axis in "xyzw"),  # base-frame cube orientation as received
    )


FIELDNAMES = fieldnames()
EVENT_FIELDS = ("elapsed_s", "ros_time_ns", "event", "command", "success", "detail", "finger_width_m")


class LiftCubeRunLogger:
    def __init__(self, log_root: Path, task_name: str, metadata: Mapping[str, Any], obs_dim: int = OBS_DIM):
        started_at = datetime.now().astimezone()
        safe_task = re.sub(r"[^A-Za-z0-9_.-]+", "_", task_name).strip("._") or "dual_fr3_liftcube"
        stamp = started_at.strftime("%Y%m%d_%H%M%S_%f_%Z")
        task_dir = Path(log_root).expanduser() / safe_task
        run_dir = task_dir / stamp
        suffix = 1
        while run_dir.exists():
            run_dir = task_dir / f"{stamp}_{suffix:02d}"
            suffix += 1
        run_dir.mkdir(parents=True)

        self.run_dir = run_dir
        self.metadata_path = run_dir / "metadata.json"
        self.trace_path = run_dir / "trace.csv"
        self.events_path = run_dir / "gripper_events.csv"
        self.metadata = {"started_at": started_at.isoformat(), **dict(metadata)}
        self.obs_dim = int(obs_dim)
        self.row_count = 0
        self.event_count = 0
        self.finalized = False
        self._write_metadata()
        self._file = self.trace_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(fieldnames(self.obs_dim))
        self._file.flush()
        self._events_file = self.events_path.open("w", newline="", encoding="utf-8")
        self._events = csv.writer(self._events_file)
        self._events.writerow(EVENT_FIELDS)
        self._events_file.flush()

    def append(self, timing: Mapping[str, float], gripper: Mapping[str, float], observation, action, q_target,
               object_quat_raw) -> None:
        if self.finalized:
            return
        missing = [name for name in (*TIMING_FIELDS, *GRIPPER_FIELDS) if name not in {**timing, **gripper}]
        if missing:
            raise ValueError(f"missing fields: {missing}")
        arrays = (
            (np.asarray(observation, dtype=np.float64), self.obs_dim, "observation"),
            (np.asarray(action, dtype=np.float64), ACTION_DIM, "action"),
            (np.asarray(q_target, dtype=np.float64), ARM_DOF, "q_target"),
            (np.asarray(object_quat_raw, dtype=np.float64), 4, "object_quat_raw"),
        )
        for array, size, name in arrays:
            if array.shape != (size,):
                raise ValueError(f"{name} must have {size} values, got {array.shape}")
        row = [timing[name] for name in TIMING_FIELDS] + [gripper[name] for name in GRIPPER_FIELDS]
        for array, _, _ in arrays:
            row.extend(f"{value:.9g}" for value in array)
        self._writer.writerow(row)
        self.row_count += 1
        # A crash or e-stop should still leave the trace up to the last tick on disk.
        self._file.flush()

    def event(self, elapsed_s: float, ros_time_ns: int, event: str, command: str, success="", detail="",
              finger_width_m=float("nan")) -> None:
        if self.finalized:
            return
        self._events.writerow([f"{elapsed_s:.6f}", ros_time_ns, event, command, success, detail, f"{finger_width_m:.6f}"])
        self.event_count += 1
        self._events_file.flush()

    def finalize(self, end_reason: str) -> Path:
        if self.finalized:
            return self.run_dir
        self.finalized = True
        self._file.close()
        self._events_file.close()
        self.metadata.update(
            {
                "ended_at": datetime.now().astimezone().isoformat(),
                "end_reason": end_reason,
                "trace_rows": self.row_count,
                "gripper_events": self.event_count,
            }
        )
        self._write_metadata()
        return self.run_dir

    def _write_metadata(self) -> None:
        self.metadata_path.write_text(json.dumps(self.metadata, indent=2, default=_json_default) + "\n")


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, array.array):  # rclpy double/int arrays after `ros2 param set`
        return value.tolist()
    raise TypeError(f"not JSON serializable: {type(value).__name__}")
