"""Per-run log of the box-lift policy node (ROS-free).

One directory per start_policy session::

    <log_root>/<task_name>/<YYYYmmdd_HHMMSS_ffffff_TZ>/
        metadata.json   manifest, node parameters, goal, start/end time, end reason, step count
        trace.csv       one row per policy tick (timing, full 56-D observation, action, joint target)

The observation columns are exactly the actor input, so a run can be replayed offline through the
same NPZ and compared with Isaac (``obs_*`` -> actor -> ``action_*``).
"""

from __future__ import annotations

import csv
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .boxlift_core import ACTION_DIM, OBS_DIM

TIMING_FIELDS = (
    "elapsed_s",            # monotonic time since the session started
    "ros_time_ns",
    "step",                 # policy step index since start (0 = first command)
    "tick_period_ms",       # actual time since the previous tick (target = policy_dt)
    "joint_state_age_ms",   # oldest of the 14 joint samples used in this tick
    "box_pose_age_ms",      # time since the box pose was received by this node
    "box_stamp_age_ms",     # now - box pose header stamp (includes vision processing latency)
    "inference_ms",         # observation build + actor + action processing
    "max_abs_delta_rad",    # max |q_target - q| sent this tick (<= action clip by construction)
    "feedback_max_joint_error_rad",  # latest controller feedback (NaN before the first one)
    "feedback_command_age_s",
)
FIELDNAMES = (
    *TIMING_FIELDS,
    *(f"obs_{i}" for i in range(OBS_DIM)),
    *(f"action_{i}" for i in range(ACTION_DIM)),
    *(f"q_target_{i}" for i in range(ACTION_DIM)),
    *(f"box_quat_raw_{axis}" for axis in "xyzw"),  # base-frame quaternion before canonicalization
)


class BoxLiftRunLogger:
    def __init__(self, log_root: Path, task_name: str, metadata: Mapping[str, Any]):
        started_at = datetime.now().astimezone()
        safe_task = re.sub(r"[^A-Za-z0-9_.-]+", "_", task_name).strip("._") or "dual_fr3_boxlift"
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
        self.metadata = {"started_at": started_at.isoformat(), **dict(metadata)}
        self.row_count = 0
        self.finalized = False
        self._write_metadata()
        self._file = self.trace_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(FIELDNAMES)
        self._file.flush()

    def append(self, timing: Mapping[str, float], observation, action, q_target, box_quat_raw) -> None:
        if self.finalized:
            return
        missing = [name for name in TIMING_FIELDS if name not in timing]
        if missing:
            raise ValueError(f"missing timing fields: {missing}")
        arrays = (
            (np.asarray(observation, dtype=np.float64), OBS_DIM, "observation"),
            (np.asarray(action, dtype=np.float64), ACTION_DIM, "action"),
            (np.asarray(q_target, dtype=np.float64), ACTION_DIM, "q_target"),
            (np.asarray(box_quat_raw, dtype=np.float64), 4, "box_quat_raw"),
        )
        for array, size, name in arrays:
            if array.shape != (size,):
                raise ValueError(f"{name} must have {size} values, got {array.shape}")
        row = [timing[name] for name in TIMING_FIELDS]
        for array, _, _ in arrays:
            row.extend(f"{value:.9g}" for value in array)
        self._writer.writerow(row)
        self.row_count += 1
        # A crash or e-stop should still leave the trace up to the last tick on disk.
        self._file.flush()

    def finalize(self, end_reason: str) -> Path:
        if self.finalized:
            return self.run_dir
        self.finalized = True
        self._file.close()
        self.metadata.update(
            {
                "ended_at": datetime.now().astimezone().isoformat(),
                "end_reason": end_reason,
                "trace_rows": self.row_count,
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
    raise TypeError(f"not JSON serializable: {type(value).__name__}")
