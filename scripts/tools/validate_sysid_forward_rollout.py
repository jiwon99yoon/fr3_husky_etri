#!/usr/bin/env python3
"""Forward-rollout validation of passive SysID candidates on rich 9/29 logs.

This tool is deliberately separate from ``fit_sysid_passive_parameters.py``.
The fit is an inverse-dynamics replay using measured real states; this tool
instead replays each logged q0-relative reference in a gravity-off MuJoCo
plant through the same 1 kHz effort-PD abstraction used by PolicyControl:

  tau = clip(Kp * (q_ref - q) - Kd * qdot, effort_limit),
  tau_applied = rate_limit(tau, previous_tau).

It compares three simulator plants against the recorded real active-joint
motion:

* current MJCF defaults;
* the unconstrained passive fit (including its fitted armature values);
* a conservative candidate that keeps the current armature and only changes
  fitted damping/frictionloss.

The constant residual bias is intentionally *not* applied.  It is not a
MuJoCo passive parameter.  This is a position/motion forward validation only:
the real ``tau_J`` log is a link-side FCI measurement and must not be compared
directly to gravity-off simulated actuator torque.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


JOINTS = tuple(
    f"{side}_fr3_joint{index}"
    for side in ("left", "right")
    for index in range(1, 8)
)
TRIAL_RE = re.compile(r"(?P<joint>(?:left|right)_fr3_joint[1-7])_sysid_rich_.*\.csv$")
KP = 80.0
KD = 4.0
TORQUE_RATE_NM_S = 1000.0
EFFORT_LIMITS = np.asarray([87.0] * 4 + [12.0] * 3 + [87.0] * 4 + [12.0] * 3)


@dataclass(frozen=True)
class Trial:
    plant: str
    joint: str
    active_index: int
    source: Path
    t: np.ndarray
    phase: np.ndarray
    q_ref_delta: np.ndarray
    q_rel: np.ndarray
    q0: np.ndarray


@dataclass(frozen=True)
class Dynamics:
    name: str
    armature: np.ndarray
    damping: np.ndarray
    frictionloss: np.ndarray


def _read_float(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def _load_trial(plant: str, path: Path) -> Trial:
    match = TRIAL_RE.search(path.name)
    if match is None:
        raise ValueError(f"unexpected SysID filename: {path}")
    joint = match.group("joint")
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    active = int(float(rows[0]["active_joint"]))
    expected = JOINTS.index(joint)
    if active != expected:
        raise ValueError(f"{path}: active_joint={active}, expected {expected} for {joint}")
    q0 = np.asarray([float(rows[0][f"q0_{name}"]) for name in JOINTS])
    q = _read_float(rows, f"q_{joint}")
    return Trial(
        plant=plant,
        joint=joint,
        active_index=active,
        source=path,
        t=_read_float(rows, "t_sec"),
        phase=np.asarray([row["phase"] for row in rows], dtype=object),
        q_ref_delta=_read_float(rows, "q_ref_delta_from_q0_rad"),
        q_rel=q - q0[active],
        q0=q0,
    )


def _discover_trials(raw_root: Path, plant: str) -> dict[str, Trial]:
    result: dict[str, Trial] = {}
    for path in sorted((raw_root / plant).glob("*_sysid_rich_*.csv")):
        trial = _load_trial(plant, path)
        if trial.joint in result:
            raise ValueError(f"duplicate {plant} trial for {trial.joint}")
        result[trial.joint] = trial
    missing = sorted(set(JOINTS) - set(result))
    if missing:
        raise ValueError(f"missing {plant} trials: {', '.join(missing)}")
    return result


def _load_fit(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    by_joint = {row["joint"]: row for row in rows}
    if set(by_joint) != set(JOINTS):
        raise ValueError("fit CSV must contain exactly the 14 FR3 joints")
    return {
        "armature": np.asarray([float(by_joint[j]["armature_kg_m2"]) for j in JOINTS]),
        "damping": np.asarray([float(by_joint[j]["damping_nm_s_rad"]) for j in JOINTS]),
        "frictionloss": np.asarray([float(by_joint[j]["frictionloss_nm"]) for j in JOINTS]),
    }


def _addresses(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    qadr, dadr, aadr = [], [], []
    for joint in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
        if jid < 0 or aid < 0:
            raise ValueError(f"model is missing joint/actuator {joint}")
        qadr.append(int(model.jnt_qposadr[jid]))
        dadr.append(int(model.jnt_dofadr[jid]))
        aadr.append(aid)
    return np.asarray(qadr), np.asarray(dadr), np.asarray(aadr)


def _make_model(mjcf: Path, dynamics: Dynamics) -> tuple[mujoco.MjModel, mujoco.MjData, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    model.opt.gravity[:] = 0.0
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    qadr, dadr, aadr = _addresses(model)
    model.dof_armature[dadr] = dynamics.armature
    model.dof_damping[dadr] = dynamics.damping
    model.dof_frictionloss[dadr] = dynamics.frictionloss
    model.actuator_ctrlrange[aadr, 0] = -EFFORT_LIMITS
    model.actuator_ctrlrange[aadr, 1] = EFFORT_LIMITS
    model.actuator_ctrllimited[aadr] = True
    return model, mujoco.MjData(model), (qadr, dadr, aadr)


def _rollout(mjcf: Path, dynamics: Dynamics, trial: Trial) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model, data, (qadr, dadr, aadr) = _make_model(mjcf, dynamics)
    dt = float(model.opt.timestep)
    n = int(np.floor(float(trial.t[-1]) / dt))
    t = (np.arange(n, dtype=np.float64) + 1.0) * dt
    reference = np.interp(t, trial.t, trial.q_ref_delta)

    mujoco.mj_resetData(model, data)
    data.qpos[qadr] = trial.q0
    data.qvel[dadr] = 0.0
    data.ctrl[aadr] = 0.0
    mujoco.mj_forward(model, data)

    q_active = np.empty(n, dtype=np.float64)
    qd_active = np.empty(n, dtype=np.float64)
    applied_active = np.empty(n, dtype=np.float64)
    previous_effort = np.zeros(len(JOINTS), dtype=np.float64)
    target = trial.q0.copy()
    for index in range(n):
        target[trial.active_index] = trial.q0[trial.active_index] + reference[index]
        q = data.qpos[qadr]
        qd = data.qvel[dadr]
        requested = KP * (target - q) - KD * qd
        requested = np.clip(requested, -EFFORT_LIMITS, EFFORT_LIMITS)
        step = TORQUE_RATE_NM_S * dt
        applied = previous_effort + np.clip(requested - previous_effort, -step, step)
        previous_effort = applied
        data.ctrl[aadr] = applied
        mujoco.mj_step(model, data)
        q_active[index] = data.qpos[qadr[trial.active_index]] - trial.q0[trial.active_index]
        qd_active[index] = data.qvel[dadr[trial.active_index]]
        applied_active[index] = data.qfrc_actuator[dadr[trial.active_index]]
    return t, q_active, qd_active, applied_active


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))


def _amplitude(signal: np.ndarray) -> float:
    return float(np.nanpercentile(signal, 95.0) - np.nanpercentile(signal, 5.0))


def _lag_seconds(sim_qd: np.ndarray, real_qd: np.ndarray, dt: float, max_s: float = 0.10) -> float:
    sim = sim_qd - np.mean(sim_qd)
    real = real_qd - np.mean(real_qd)
    maximum = int(round(max_s / dt))
    lags = np.arange(-maximum, maximum + 1)
    score = []
    for lag in lags:
        if lag < 0:
            left, right = sim[-lag:], real[:lag]
        elif lag > 0:
            left, right = sim[:-lag], real[lag:]
        else:
            left, right = sim, real
        denom = np.linalg.norm(left) * np.linalg.norm(right)
        score.append(float(left.dot(right) / denom) if denom > 0.0 else -np.inf)
    return float(lags[int(np.argmax(score))] * dt)


def _lagged_rmse(sim_q: np.ndarray, real_q: np.ndarray, lag_s: float, dt: float) -> float:
    shift = int(round(lag_s / dt))
    if shift < 0:
        sim_q, real_q = sim_q[-shift:], real_q[:shift]
    elif shift > 0:
        sim_q, real_q = sim_q[:-shift], real_q[shift:]
    return _rmse(sim_q, real_q)


def _phase_mask(trial: Trial, t: np.ndarray, split: str) -> np.ndarray:
    """Map logged phase labels to the uniform forward-rollout timeline."""
    if split == "all":
        return np.ones(t.size, dtype=bool)
    source_index = np.clip(np.searchsorted(trial.t, t, side="right") - 1, 0, trial.t.size - 1)
    phase = trial.phase[source_index].astype(str)
    if split == "train":
        return (
            ((np.char.find(phase, "fric_cruise_") >= 0) &
             ((np.char.find(phase, "v0.050") >= 0) | (np.char.find(phase, "v0.100") >= 0)))
            | (np.char.find(phase, "inertia_sine_f0.50") >= 0)
        )
    if split == "held_out":
        return (
            ((np.char.find(phase, "fric_cruise_") >= 0) &
             (np.char.find(phase, "v0.150") >= 0))
            | (np.char.find(phase, "inertia_sine_f1.00") >= 0)
        )
    raise ValueError(f"unsupported split: {split}")


def _plot(rollouts: dict[str, dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]], real: dict[str, Trial], output: Path) -> None:
    colors = {"default": "#2563a8", "fit_all": "#d97706", "fit_keep_armature": "#7c3aed"}
    labels = {"default": "current default", "fit_all": "fitted A/B/Fc", "fit_keep_armature": "fitted B/Fc, A=0.1"}
    figure, axes = plt.subplots(7, 2, figsize=(16, 22))
    for axis, joint in zip(axes.ravel(), JOINTS, strict=True):
        trial = real[joint]
        axis.plot(trial.t, np.rad2deg(trial.q_ref_delta), "--", color="0.2", lw=0.9, label="reference")
        axis.plot(trial.t, np.rad2deg(trial.q_rel), color="#c23b22", lw=0.9, label="real")
        for name in ("default", "fit_all", "fit_keep_armature"):
            t, q, _qd, _tau = rollouts[name][joint]
            axis.plot(t, np.rad2deg(q), color=colors[name], lw=0.85, label=labels[name])
        axis.set_title(joint)
        axis.set_xlabel("time [s]")
        axis.set_ylabel("q - q0 [deg]")
        axis.grid(alpha=0.25)
    handles, labels_out = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels_out, loc="upper center", ncol=5, frameon=False)
    figure.suptitle("9/29 rich SysID: gravity-off forward-rollout candidates", y=0.996)
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields = list(rows[0])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--raw-root", type=Path, default=Path("data/sysid_raw"))
    parser.add_argument("--fit-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    real = _discover_trials(args.raw_root, "real")
    recorded_mujoco = _discover_trials(args.raw_root, "mujoco")
    fitted = _load_fit(args.fit_csv)
    default = Dynamics("default", np.full(14, 0.1), np.full(14, 0.003), np.full(14, 0.2))
    candidates = (
        default,
        Dynamics("fit_all", fitted["armature"], fitted["damping"], fitted["frictionloss"]),
        Dynamics("fit_keep_armature", default.armature, fitted["damping"], fitted["frictionloss"]),
    )
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)

    rollouts: dict[str, dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]] = {x.name: {} for x in candidates}
    rows: list[dict[str, object]] = []
    replay_rows: list[dict[str, object]] = []
    for candidate in candidates:
        for joint in JOINTS:
            trial = real[joint]
            t, q, qd, tau = _rollout(args.mjcf, candidate, trial)
            rollouts[candidate.name][joint] = (t, q, qd, tau)
            real_q = np.interp(t, trial.t, trial.q_rel)
            real_qd = np.gradient(real_q, t)
            for split in ("all", "train", "held_out"):
                mask = _phase_mask(trial, t, split)
                q_sim, q_real = q[mask], real_q[mask]
                lag = _lag_seconds(qd[mask], real_qd[mask], float(t[1] - t[0]))
                rows.append({
                    "candidate": candidate.name,
                    "split": split,
                    "joint": joint,
                    "position_rmse_deg": np.rad2deg(_rmse(q_sim, q_real)),
                    "position_rmse_lag_comp_deg": np.rad2deg(_lagged_rmse(q_sim, q_real, lag, float(t[1] - t[0]))),
                    "velocity_lag_ms_real_relative_sim": 1000.0 * lag,
                    "sim_position_p2p_deg": np.rad2deg(_amplitude(q_sim)),
                    "real_position_p2p_deg": np.rad2deg(_amplitude(q_real)),
                    "sim_to_real_p2p_ratio": _amplitude(q_sim) / _amplitude(q_real) if _amplitude(q_real) > 1.0e-12 else np.nan,
                    "max_applied_pd_torque_nm": float(np.max(np.abs(tau[mask]))),
                })

        # Direct-replay fidelity against the recorded default MuJoCo trace is
        # checked only for the nominal default. It validates assumptions about
        # dt/controller/initial conditions before interpreting fitted changes.
        if candidate.name == "default":
            for joint in JOINTS:
                recorded = recorded_mujoco[joint]
                t, q, qd, _tau = _rollout(args.mjcf, candidate, recorded)
                logged_q = np.interp(t, recorded.t, recorded.q_rel)
                logged_qd = np.gradient(logged_q, t)
                lag = _lag_seconds(qd, logged_qd, float(t[1] - t[0]))
                replay_rows.append({
                    "joint": joint,
                    "replay_vs_recorded_mujoco_rmse_deg": np.rad2deg(_rmse(q, logged_q)),
                    "replay_vs_recorded_mujoco_lag_comp_rmse_deg": np.rad2deg(_lagged_rmse(q, logged_q, lag, float(t[1] - t[0]))),
                    "recorded_mujoco_relative_lag_ms": 1000.0 * lag,
                })

    _write_csv(output / "forward_position_metrics.csv", rows)
    _write_csv(output / "default_replay_fidelity.csv", replay_rows)
    _plot(rollouts, real, output / "active_joint_forward_position_overlay.png")

    def mean(candidate: str, key: str, split: str = "all") -> float:
        return float(np.mean([
            float(row[key]) for row in rows
            if row["candidate"] == candidate and row["split"] == split
        ]))

    report = f"""# 9/29 gravity-off MuJoCo forward-rollout validation

This is a closed-loop position-response replay, not an inverse-dynamics torque
fit. Every candidate uses the logged real q0 and q0-relative reference, 1 ms
MuJoCo stepping, Kp={KP:g}, Kd={KD:g}, the 87/12 Nm actuator limits, and the
1000 Nm/s effort rate limit. Robot gravity and contacts are disabled to retain
the existing Isaac/FCI control abstraction.

## Mean active-joint position metrics

| Candidate | zero-lag RMSE [deg] | lag-compensated RMSE [deg] | mean sim/real p2p ratio |
|---|---:|---:|---:|
| current default | {mean('default', 'position_rmse_deg'):.3f} | {mean('default', 'position_rmse_lag_comp_deg'):.3f} | {mean('default', 'sim_to_real_p2p_ratio'):.3f} |
| fitted A/B/Fc | {mean('fit_all', 'position_rmse_deg'):.3f} | {mean('fit_all', 'position_rmse_lag_comp_deg'):.3f} | {mean('fit_all', 'sim_to_real_p2p_ratio'):.3f} |
| fitted B/Fc, A=0.1 | {mean('fit_keep_armature', 'position_rmse_deg'):.3f} | {mean('fit_keep_armature', 'position_rmse_lag_comp_deg'):.3f} | {mean('fit_keep_armature', 'sim_to_real_p2p_ratio'):.3f} |

## Fit-held-out phase mean active-joint position metrics

Only v0.150 cruise and f1.00 sine phases, excluded from the inverse-dynamics fit, are used below.

| Candidate | zero-lag RMSE [deg] | lag-compensated RMSE [deg] | mean sim/real p2p ratio |
|---|---:|---:|---:|
| current default | {mean('default', 'position_rmse_deg', 'held_out'):.3f} | {mean('default', 'position_rmse_lag_comp_deg', 'held_out'):.3f} | {mean('default', 'sim_to_real_p2p_ratio', 'held_out'):.3f} |
| fitted A/B/Fc | {mean('fit_all', 'position_rmse_deg', 'held_out'):.3f} | {mean('fit_all', 'position_rmse_lag_comp_deg', 'held_out'):.3f} | {mean('fit_all', 'sim_to_real_p2p_ratio', 'held_out'):.3f} |
| fitted B/Fc, A=0.1 | {mean('fit_keep_armature', 'position_rmse_deg', 'held_out'):.3f} | {mean('fit_keep_armature', 'position_rmse_lag_comp_deg', 'held_out'):.3f} | {mean('fit_keep_armature', 'sim_to_real_p2p_ratio', 'held_out'):.3f} |

## Scope and decision rule

The fitted inverse-dynamics parameters are candidates, not accepted physical
values. In particular the fit's armatures reach their non-negative lower
bound. Therefore the conservative B/Fc-only candidate is reported alongside
the literal fit. Do not use the candidate in Isaac Lab or RL until it improves
position-response metrics **and** the default replay fidelity is adequate.

Real raw `tau_J` is intentionally not compared with gravity-off simulated
actuator effort here. Continue using the gravity-on inverse-dynamics held-out
torque RMSE for the torque-model check.

![Forward position overlay](active_joint_forward_position_overlay.png)
"""
    (output / "report.md").write_text(report, encoding="utf-8")
    metadata = {
        "mjcf": str(args.mjcf.resolve()),
        "fit_csv": str(args.fit_csv.resolve()),
        "raw_root": str(args.raw_root.resolve()),
        "gravity": [0.0, 0.0, 0.0],
        "pd": {"kp": KP, "kd": KD, "effort_rate_limit_nm_s": TORQUE_RATE_NM_S},
        "candidates": {
            c.name: {
                "armature": c.armature.tolist(),
                "damping": c.damping.tolist(),
                "frictionloss": c.frictionloss.tolist(),
            }
            for c in candidates
        },
    }
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    for candidate in ("default", "fit_all", "fit_keep_armature"):
        print(
            f"{candidate:<18} position RMSE={mean(candidate, 'position_rmse_deg'):.3f} deg, "
            f"lag-comp={mean(candidate, 'position_rmse_lag_comp_deg'):.3f} deg"
        )
    print(
        "default direct-replay fidelity: "
        f"mean RMSE={np.mean([float(r['replay_vs_recorded_mujoco_rmse_deg']) for r in replay_rows]):.3f} deg"
    )


if __name__ == "__main__":
    main()
