#!/usr/bin/env python3
"""Plot matched real/MuJoCo rich-torque-SysID trial responses.

The plot set contains one 14-panel overview for each response type:

* active-joint position tracking relative to captured q0,
* active-joint velocity,
* raw measured active-joint torque, and
* torque after removing each trial's initial q0-settle baseline.

Raw torque intentionally retains the real robot's gravity component.  The
baseline-removed plot is the clearer comparison of the excitation response
when MuJoCo runs gravity-off to emulate the FCI gravity-compensated plant.

Example:

    /home/dyros/anaconda3/envs/cRobotics/bin/python \
      scripts/tools/plot_sysid_rich_responses.py \
      --raw-root data/sysid_raw
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


JOINTS = tuple(f"{side}_fr3_joint{index}" for side in ("left", "right") for index in range(1, 8))
TRIAL_RE = re.compile(r"(?P<joint>(?:left|right)_fr3_joint[1-7])_sysid_rich_.*\.csv$")
COLORS = {"real": "#c23b22", "mujoco": "#2563a8"}


@dataclass(frozen=True)
class Trial:
    plant: str
    joint: str
    path: Path
    t: np.ndarray
    phase: np.ndarray
    q_ref_delta: np.ndarray
    q_rel: np.ndarray
    qd_ref: np.ndarray
    qd: np.ndarray
    tau: np.ndarray


def _float_column(rows: list[dict[str, str]], name: str) -> np.ndarray:
    try:
        return np.asarray([float(row[name]) for row in rows], dtype=float)
    except KeyError as error:
        raise ValueError(f"missing column {name}") from error


def load_trial(plant: str, joint: str, path: Path) -> Trial:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty CSV")
    t = _float_column(rows, "t_sec")
    q0 = _float_column(rows, f"q0_{joint}")
    return Trial(
        plant=plant,
        joint=joint,
        path=path,
        t=t - t[0],
        phase=np.asarray([row["phase"] for row in rows], dtype=object),
        q_ref_delta=_float_column(rows, "q_ref_delta_from_q0_rad"),
        q_rel=_float_column(rows, f"q_{joint}") - q0,
        qd_ref=_float_column(rows, "qd_ref_active_rad_s"),
        qd=_float_column(rows, f"qd_{joint}"),
        tau=_float_column(rows, f"tau_meas_{joint}"),
    )


def discover_trials(raw_root: Path) -> dict[tuple[str, str], Trial]:
    trials: dict[tuple[str, str], Trial] = {}
    for plant in ("real", "mujoco"):
        for path in sorted((raw_root / plant).glob("*_sysid_rich_*.csv")):
            match = TRIAL_RE.search(path.name)
            if not match:
                continue
            joint = match.group("joint")
            key = (plant, joint)
            if key in trials:
                raise ValueError(f"multiple trials for {plant}/{joint}: {trials[key].path}, {path}")
            trials[key] = load_trial(plant, joint, path)
    missing = [f"{plant}/{joint}" for plant in ("real", "mujoco") for joint in JOINTS if (plant, joint) not in trials]
    if missing:
        raise ValueError("missing matched trials: " + ", ".join(missing))
    return trials


def _downsample(trial: Trial, max_points: int = 12000) -> slice:
    return slice(None, None, max(1, int(np.ceil(trial.t.size / max_points))))


def _initial_baseline(trial: Trial) -> float:
    # q0_settle is also used at the final hold.  Limit to the first 0.5 s to
    # retain only the pre-excitation baseline.
    mask = (trial.t <= 0.5) & (trial.phase == "q0_settle")
    if not np.any(mask):
        mask = trial.t <= min(0.5, float(trial.t[-1]))
    return float(np.nanmedian(trial.tau[mask]))


def _plot_overview(
    trials: dict[tuple[str, str], Trial],
    out_path: Path,
    title: str,
    y_label: str,
    value,
    reference=None,
    zero_line: bool = False,
) -> None:
    figure, axes = plt.subplots(7, 2, figsize=(16, 22), sharex=False)
    for axis, joint in zip(axes.ravel(), JOINTS, strict=True):
        if reference is not None:
            ref_trial = trials[("real", joint)]
            stride = _downsample(ref_trial)
            axis.plot(ref_trial.t[stride], reference(ref_trial)[stride], "--", color="0.2", linewidth=1.0, label="reference")
        for plant in ("real", "mujoco"):
            trial = trials[(plant, joint)]
            stride = _downsample(trial)
            axis.plot(trial.t[stride], value(trial)[stride], color=COLORS[plant], linewidth=0.9, label=plant)
        if zero_line:
            axis.axhline(0.0, color="0.5", linewidth=0.6)
        axis.set_title(joint)
        axis.set_xlabel("time [s]")
        axis.set_ylabel(y_label)
        axis.grid(alpha=0.25)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(
        handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.978),
        ncol=len(labels), frameon=False,
    )
    figure.suptitle(title, y=0.995)
    figure.tight_layout(rect=(0, 0, 1, 0.962))
    figure.savefig(out_path, dpi=180)
    plt.close(figure)


def write_summary(trials: dict[tuple[str, str], Trial], out_path: Path) -> None:
    with out_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "plant", "joint", "source_csv", "samples", "duration_s", "median_rate_hz",
            "q_tracking_rms_rad", "qd_tracking_rms_rad_s", "tau_initial_baseline_nm",
            "tau_p05_nm", "tau_p95_nm",
        ])
        for joint in JOINTS:
            for plant in ("real", "mujoco"):
                trial = trials[(plant, joint)]
                dt = np.diff(trial.t)
                rate = 1.0 / np.median(dt[dt > 0.0])
                writer.writerow([
                    plant,
                    joint,
                    trial.path.name,
                    trial.t.size,
                    f"{trial.t[-1]:.6f}",
                    f"{rate:.3f}",
                    f"{np.sqrt(np.mean((trial.q_rel - trial.q_ref_delta) ** 2)):.8f}",
                    f"{np.sqrt(np.mean((trial.qd - trial.qd_ref) ** 2)):.8f}",
                    f"{_initial_baseline(trial):.8f}",
                    f"{np.nanpercentile(trial.tau, 5):.8f}",
                    f"{np.nanpercentile(trial.tau, 95):.8f}",
                ])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, default=Path("data/sysid_raw"))
    parser.add_argument("--output-dir", type=Path, default=Path("data/sysid_response_plots"))
    args = parser.parse_args()

    trials = discover_trials(args.raw_root)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    _plot_overview(
        trials, args.output_dir / "active_joint_position_tracking.png",
        "Rich SysID active-joint position response (relative to captured q0)",
        "position [rad]", lambda trial: trial.q_rel, reference=lambda trial: trial.q_ref_delta,
    )
    _plot_overview(
        trials, args.output_dir / "active_joint_velocity.png",
        "Rich SysID active-joint velocity response", "velocity [rad/s]",
        lambda trial: trial.qd, reference=lambda trial: trial.qd_ref,
        zero_line=True,
    )
    _plot_overview(
        trials, args.output_dir / "active_joint_torque_raw.png",
        "Rich SysID measured active-joint torque (raw; real includes gravity)",
        "measured torque [Nm]", lambda trial: trial.tau,
    )
    _plot_overview(
        trials, args.output_dir / "active_joint_torque_baseline_removed.png",
        "Rich SysID active-joint torque response (initial q0 baseline removed)",
        "torque change [Nm]", lambda trial: trial.tau - _initial_baseline(trial),
        zero_line=True,
    )
    write_summary(trials, args.output_dir / "summary.csv")
    print(f"Wrote plots and summary: {args.output_dir}")


if __name__ == "__main__":
    main()
