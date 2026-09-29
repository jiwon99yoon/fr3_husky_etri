#!/usr/bin/env python3
"""Separate gravity-compensation bias from joint friction in real rich-SysID CSVs.

On the real FR3 (torque mode) the FCI adds its own gravity compensation, so the
active joint obeys

    M q̈ + C q̇ + g_true = tau_cmd + g_franka - F(q̇)
    => y := tau_cmd - (M q̈ + C q̇) = (g_true - g_franka) + F(q̇)

The bias term depends on q only (even in q̇) while friction is odd in q̇, so the
bidirectional constant-velocity sweep of ``fr3_sysid_node`` separates them:

    y ≈ c0 + c1 (q - q0) + Fc sign(q̇) + Fv q̇ + Ja q̈

Two independent cross-checks are reported:

* ``settle_e`` / ``Kp*e``: the q0_settle PD equilibrium.  With a non-zero bias the
  arm rests where ``Kp * (q0 - q)`` cancels it (unless stiction holds it).
* ``E = tau_meas - g_model - tau_cmd`` regressed on ``[1, sign(q̇)]``.  ``E_c0``
  is ``g_franka - g_model``: a pattern such as J2 < 0, J4 > 0, J6 > 0 on both arms
  indicates an end-effector mass mismatch in the Desk profile.
* ``D = tau_meas - (M q̈ + C q̇ + g_model)`` checks the plate model against the
  link-side torque sensor.

``tau_meas`` must be the gravity-inclusive link-side torque (tau_J).  Build the
reference MJCF as described in ``about_sysid.md`` §8.3 (plate, no camera):

    /home/dyros/anaconda3/envs/cRobotics/bin/python \\
      scripts/tools/split_gravity_bias_friction.py \\
      --mjcf /tmp/dual_fr3_plate_nocam.xml \\
      --csv 'logs/sysid_raw/real/*_2609*.csv'
"""

from __future__ import annotations

import argparse
import csv
import glob
import re

import numpy as np

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - depends on the experiment PC
    raise SystemExit("mujoco python is required: pip install mujoco") from exc

from scipy.signal import butter, filtfilt

JOINTS = [f"{side}_fr3_joint{i}" for side in ("left", "right") for i in range(1, 8)]
TRIAL_RE = re.compile(r"((?:left|right)_fr3_joint[1-7])_sysid_rich_.*\.csv$")
# PolicyControl / Isaac reach v2 PD gains.
KP, KD = 80.0, 4.0
MOVING_QD_RAD_S = 0.01


def build_model(mjcf: str):
    model = mujoco.MjModel.from_xml_path(mjcf)
    # Pure rigid-body torque: passive terms are what the fit identifies.
    model.dof_damping[:] = 0.0
    model.dof_frictionloss[:] = 0.0
    model.dof_armature[:] = 0.0
    names = [model.joint(i).name for i in range(14)]
    if names != JOINTS:
        raise SystemExit(f"unexpected MJCF joint order: {names}")
    return model, mujoco.MjData(model)


def inverse(model, data, q, qd, qdd, gravity: bool) -> np.ndarray:
    model.opt.gravity[:] = (0.0, 0.0, -9.81) if gravity else (0.0, 0.0, 0.0)
    out = np.empty_like(q)
    for k in range(len(q)):
        data.qpos[:14] = q[k]
        data.qvel[:14] = qd[k]
        data.qacc[:14] = qdd[k]
        mujoco.mj_inverse(model, data)
        out[k] = data.qfrc_inverse[:14]
    return out


def analyse(model, data, path: str, joint: str, cutoff_hz: float) -> list[float]:
    j = JOINTS.index(joint)
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    col = lambda name: np.array([float(row[name]) for row in rows])
    t = col("t_sec")
    phase = np.array([row["phase"] for row in rows])
    q = np.stack([col(f"q_{n}") for n in JOINTS], axis=1)
    qd = np.stack([col(f"qd_{n}") for n in JOINTS], axis=1)
    tau = col(f"tau_meas_{joint}")
    q0 = col(f"q0_{joint}")[0]
    tau_cmd = KP * col("cmd_offset_rad") - KD * qd[:, j]

    fs = 1.0 / np.median(np.diff(t))
    b, a = butter(4, cutoff_hz / (fs / 2.0))
    qd_f = filtfilt(b, a, qd, axis=0)
    qdd = np.gradient(qd_f, t, axis=0)

    rbd = inverse(model, data, q, qd_f, qdd, gravity=False)[:, j]
    zeros = np.zeros_like(qd)
    g = inverse(model, data, q, zeros, zeros, gravity=True)[:, j]
    y = tau_cmd - rbd

    v = qd_f[:, j]
    active = ~np.isin(phase, ["q0_settle", "done"])
    moving = active & (np.abs(v) > MOVING_QD_RAD_S)
    X = np.column_stack([
        np.ones(moving.sum()), q[moving, j] - q0, np.sign(v[moving]), v[moving], qdd[moving, j],
    ])
    coef, *_ = np.linalg.lstsq(X, y[moving], rcond=None)
    residual = y[moving] - X @ coef
    r2 = 1.0 - residual.var() / y[moving].var()

    settle_e = float(np.median(q[phase == "q0_settle", j] - q0))
    D = tau - (rbd + g)
    E = tau - g - tau_cmd
    E_coef, *_ = np.linalg.lstsq(
        np.column_stack([np.ones(moving.sum()), np.sign(v[moving])]), E[moving], rcond=None
    )
    return [
        *coef, r2, float(np.sqrt(np.mean(residual**2))), settle_e, -KP * settle_e,
        float(D[active].mean()), float(np.sqrt(np.mean(D[active] ** 2))), *E_coef,
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mjcf", required=True, help="plate/no-camera dual FR3 MJCF")
    parser.add_argument("--csv", required=True, help="glob of real *_sysid_rich_*.csv files")
    parser.add_argument("--cutoff-hz", type=float, default=25.0)
    args = parser.parse_args()

    model, data = build_model(args.mjcf)
    header = ("joint bias@q0 dBias/dq Fc Fv Ja R2 rms settle_e Kp*e "
              "D_mean D_rms E_c0 E_Fc").split()
    print(f"{header[0]:18s}" + "".join(f"{h:>9s}" for h in header[1:]))
    for path in sorted(glob.glob(args.csv)):
        match = TRIAL_RE.search(path)
        if not match:
            continue
        values = analyse(model, data, path, match.group(1), args.cutoff_hz)
        print(f"{match.group(1):18s}" + "".join(f"{value:9.3f}" for value in values))


if __name__ == "__main__":
    main()
