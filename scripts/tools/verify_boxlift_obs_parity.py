#!/usr/bin/env python3
"""Verify that a deployment-side (numpy) pipeline reproduces the Isaac box-lift policy I/O.

Input: a trace written by ``dual_fr3_lab/scripts/tools/dump_boxlift_deploy_trace.py`` (PLAY task,
observation corruption off) and the actor exported from the same checkpoint with
``export_ppo_actor.py --output-activation tanh``.

Checks, each against the spec a real-robot node must implement:

1. Spec constants: arm joint order, default pose, joint-target limits, action scale/clip,
   observation term order and sizes.
2. Observation rebuild: the 56-D observation is rebuilt from raw states only (joint states
   selected by name, robot-root/box world poses, goal from episode-start box x/y + command z,
   node-held previous action) and compared term by term with the Isaac observation.
3. Actor: the numpy NPZ actor on the Isaac observation and on the rebuilt observation
   reproduces the Isaac deterministic action.
4. Action processing: clip(0.1 a, +-0.1) + measured q, clamped to the limits, equals the
   absolute target the Isaac action term held for the step.

It also reports observation/command ranges seen in the trace (useful as real-robot gating
limits) and writes ``report.md`` + ``summary.json`` in ``--output-dir``.

Example:
  python scripts/tools/verify_boxlift_obs_parity.py \
    --trace data/boxlift_obs_parity_261103/trace.npz \
    --actor data/boxlift_obs_parity_261103/actor.npz \
    --output-dir data/boxlift_obs_parity_261103

The script prints the checks and writes ``summary.json``; interpretation goes in ``report.md``.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "fr3_husky_nn_policy"))
from fr3_husky_nn_policy.numpy_actor import NumpyMLPActor  # noqa: E402

# ---------------------------------------------------------------- deployment spec (as handed over)
DUAL_ARM_JOINT_NAMES = [f"{side}_fr3_joint{i}" for side in ("left", "right") for i in range(1, 8)]
_ARM_DEFAULT = [-0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]  # j2..j7
SPEC_DEFAULT_POSE = np.asarray([-0.5236, *_ARM_DEFAULT, 0.5236, *_ARM_DEFAULT])
_LIMITS_ONE_ARM = [(-2.6937, 2.6937), (-1.7337, 1.7337), (-2.8507, 2.8507), (-2.9921, -0.2018),
                   (-2.7565, 2.7565), (0.5945, 4.4669), (-2.9659, 2.9659)]
SPEC_TARGET_LIMITS = np.asarray(_LIMITS_ONE_ARM * 2)
SPEC_ACTION_SCALE = 0.1
SPEC_ACTION_CLIP = 0.1
SPEC_GOAL_QUAT_XYZW = np.asarray([0.0, 0.0, 0.0, 1.0])
SPEC_TERMS = [("joint_pos", 14), ("joint_vel", 14), ("object_position", 3),
              ("target_object_position", 7), ("actions", 14), ("object_orientation", 4)]
TOL = 1e-4


# ---------------------------------------------------------------- quaternion helpers (x, y, z, w)
def quat_conj(q):
    return np.concatenate([-q[..., :3], q[..., 3:]], axis=-1)


def quat_mul(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], axis=-1)


def quat_rotate_inverse(q, v):
    """R(q)^T v for xyzw quaternions."""
    vq = np.concatenate([v, np.zeros(v.shape[:-1] + (1,))], axis=-1)
    return quat_mul(quat_mul(quat_conj(q), vq), q)[..., :3]


def yaw_of(q):
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------- deployment-side builders
def build_obs(q14, qd14, box_pos_b, goal7, prev_action, box_quat_b):
    """The exact layout a real node would assemble (all inputs already in base frame)."""
    quat = box_quat_b * np.where(box_quat_b[..., 3:4] >= 0.0, 1.0, -1.0)
    return np.concatenate([q14 - SPEC_DEFAULT_POSE, qd14, box_pos_b, goal7, prev_action, quat], axis=-1)


def start_xy_track(box_pos_b, ep_step):
    """Box x/y in base at the start of the current episode, per sample."""
    out = np.empty(box_pos_b.shape[:2] + (2,))
    current = box_pos_b[0, :, :2].copy()
    for t in range(box_pos_b.shape[0]):
        current[ep_step[t] == 0] = box_pos_b[t, ep_step[t] == 0, :2]
        out[t] = current
    return out


def _set(o, sl, value):
    o = o.copy()
    o[..., sl] = value
    return o


def _add(o, index, value):
    o = o.copy()
    o[..., index] += value
    return o


def _canon(q):
    return q * np.where(q[..., 3:4] >= 0.0, 1.0, -1.0)


def process_action(action, q14):
    delta = np.clip(SPEC_ACTION_SCALE * action, -SPEC_ACTION_CLIP, SPEC_ACTION_CLIP)
    return np.clip(q14 + delta, SPEC_TARGET_LIMITS[:, 0], SPEC_TARGET_LIMITS[:, 1]), delta


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--actor", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    tr = np.load(args.trace, allow_pickle=False)
    obs = tr["obs"].astype(np.float64)                      # (T, N, 56)
    act = tr["action"].astype(np.float64)                   # (T, N, 14)
    done = tr["done"]                                        # (T, N)
    T, N, _ = obs.shape
    results: dict[str, dict] = {}

    def check(name, value, tol=TOL, detail=""):
        ok = bool(value <= tol)
        results[name] = {"max_abs_err": float(value), "tol": tol, "pass": ok, "detail": detail}

    # ---- 1. spec constants
    names = [str(x) for x in tr["joint_names"]]
    arm_names = [str(x) for x in tr["arm_joint_names"]]
    sel = np.asarray([names.index(n) for n in DUAL_ARM_JOINT_NAMES])
    results["arm_joint_order"] = {"pass": arm_names == DUAL_ARM_JOINT_NAMES, "detail": ", ".join(arm_names)}
    check("default_pose", np.max(np.abs(tr["default_joint_pos"][sel] - SPEC_DEFAULT_POSE)), 1e-4)
    check("target_limits", np.max(np.abs(tr["target_limits"] - SPEC_TARGET_LIMITS)), 1e-4)
    check("action_scale", abs(float(tr["action_scale"]) - SPEC_ACTION_SCALE), 1e-9,
          f"clip cfg = {tr['action_clip']}; reference_mode = {tr['reference_mode']}")
    terms = list(zip([str(x) for x in tr["obs_term_names"]], [int(x) for x in tr["obs_term_dims"]]))
    results["obs_term_layout"] = {"pass": terms == SPEC_TERMS, "detail": str(terms)}

    # ---- 2. observation rebuild from raw states
    q = tr["joint_pos"][..., sel].astype(np.float64)
    qd = tr["joint_vel"][..., sel].astype(np.float64)
    root_p, root_q = tr["root_pos_w"].astype(np.float64), tr["root_quat_w"].astype(np.float64)
    box_p, box_q = tr["box_pos_w"].astype(np.float64), tr["box_quat_w"].astype(np.float64)
    box_pos_b = quat_rotate_inverse(root_q, box_p - root_p)
    box_quat_b = quat_mul(quat_conj(root_q), box_q)

    # node-held state: episode-start box x/y for the goal, previous action reset to 0
    ep_step = tr["episode_step"]
    goal = np.zeros((T, N, 7))
    prev = np.zeros((T, N, 14))
    start_xy = box_pos_b[0, :, :2].copy()
    for t in range(T):
        new_ep = ep_step[t] == 0
        start_xy[new_ep] = box_pos_b[t, new_ep, :2]
        goal[t, :, :2] = start_xy
        goal[t, :, 2] = tr["command"][t, :, 2]       # z: chosen by the node; taken from the sim command
        goal[t, :, 3:] = SPEC_GOAL_QUAT_XYZW
        if t > 0:
            prev[t] = np.where(done[t - 1][:, None], 0.0, act[t - 1])
    # the first recorded step of a never-reset env is also an episode start
    valid_prev = np.ones((T, N), bool)
    valid_prev[0] = ep_step[0] == 0
    # Training artifact: command resampling_time_range == episode length, so the sim re-samples
    # the goal (x/y = box x/y at that moment, new z) on the last step of each episode. A node keeps
    # the goal fixed (spec); count those samples separately instead of failing on them.
    resampled = np.abs(tr["command"][..., :2] - start_xy_track(box_pos_b, ep_step)).max(-1) > 1e-6
    valid_goal = ~resampled
    results["sim_goal_resampled_within_episode"] = {
        "pass": True,
        "detail": f"{int(resampled.sum())}/{T * N} samples, at episode steps "
                  f"{sorted(set(int(s) for s in ep_step[resampled]))} (excluded from goal/actor checks)",
    }

    rebuilt = build_obs(q, qd, box_pos_b, goal, prev, box_quat_b)
    offsets = np.cumsum([0] + [d for _, d in SPEC_TERMS])
    for (term, _), a, b in zip(SPEC_TERMS, offsets[:-1], offsets[1:]):
        mask = {"actions": valid_prev, "target_object_position": valid_goal}.get(term, np.ones((T, N), bool))
        err = np.max(np.abs(rebuilt[..., a:b] - obs[..., a:b])[mask])
        check(f"obs.{term}", err)
    check("command.goal_quat_is_identity_xyzw", np.max(np.abs(tr["command"][..., 3:] - SPEC_GOAL_QUAT_XYZW)), 1e-6)
    check("prev_action_buffer_vs_node", np.max(np.abs(tr["prev_action"] - prev)[valid_prev]))

    # ---- 3. actor
    actor = NumpyMLPActor(args.actor)
    np_act = np.stack([[actor(obs[t, n].astype(np.float32)) for n in range(N)] for t in range(T)]).astype(np.float64)
    np_act_rebuilt = np.stack([[actor(rebuilt[t, n].astype(np.float32)) for n in range(N)] for t in range(T)])
    check("actor.isaac_obs", np.max(np.abs(np_act - act)), 1e-4, f"sha256 {actor.source_sha256[:12]} vs trace {str(tr['checkpoint_sha256'])[:12]}")
    check("actor.rebuilt_obs", np.max(np.abs(np_act_rebuilt - act)[valid_prev & valid_goal]), 1e-4)
    results["actor.sha256_matches_checkpoint"] = {"pass": actor.source_sha256 == str(tr["checkpoint_sha256"])}

    # ---- 4. action processing
    target, delta = process_action(act, q)
    # held_target is in action-term joint order, verified above to equal DUAL_ARM_JOINT_NAMES.
    check("held_target", np.max(np.abs(target - tr["held_target"])[~done]), 1e-5)

    # ---- ranges useful for real-robot gating / node design
    first = ep_step == 0
    yaw = yaw_of(box_quat_b)
    clipped = np.abs(q + delta - target) > 1e-9
    stats = {
        "steps_x_envs": [int(T), int(N)],
        "root_height_w_m": [float(root_p[..., 2].min()), float(root_p[..., 2].max())],
        "box_start_pos_b_min": box_pos_b[first].min(0).round(4).tolist(),
        "box_start_pos_b_max": box_pos_b[first].max(0).round(4).tolist(),
        "box_start_yaw_deg": [float(np.degrees(yaw[first]).min()), float(np.degrees(yaw[first]).max())],
        "box_pos_b_min": box_pos_b.reshape(-1, 3).min(0).round(4).tolist(),
        "box_pos_b_max": box_pos_b.reshape(-1, 3).max(0).round(4).tolist(),
        "goal_z_range": [float(goal[..., 2].min()), float(goal[..., 2].max())],
        "action_saturated_frac(|a|>0.99)": float(np.mean(np.abs(act) > 0.99)),
        "max_abs_delta_rad": float(np.abs(delta).max()),
        "target_clipped_by_limits_frac": float(np.mean(clipped)),
        "max_abs_joint_vel_rad_s": float(np.abs(qd).max()),
        "frac_steps_where_|delta - qd*0.03|>0.11": float(np.mean(np.abs(delta - qd * 0.03) > 0.11)),
    }

    # ---- sensitivity of the action to plausible deployment mistakes (on valid samples)
    valid = valid_prev & valid_goal
    flip_z = np.asarray([0.0, 0.0, 1.0, 0.0])  # 180 deg about z: same box shape, different quaternion
    perturbations = {
        "goal_quat_wxyz_(1,0,0,0)": lambda o: _set(o, slice(34, 38), np.asarray([1.0, 0.0, 0.0, 0.0])),
        "box_quat_yaw_flipped_180deg": lambda o: _set(o, slice(52, 56), _canon(quat_mul(o[..., 52:56], flip_z))),
        "box_z_+0.013m_(base_height_0.392)": lambda o: _add(o, 30, 0.013),
        "box_xy_+0.01m_(1sigma_noise)": lambda o: _add(_add(o, 28, 0.01), 29, 0.01),
    }
    sensitivity = {}
    base_act = np_act_rebuilt[valid]
    for name, fn in perturbations.items():
        pert = fn(rebuilt[valid])
        pert_act = np.stack([actor(row.astype(np.float32)) for row in pert])
        diff = np.abs(pert_act - base_act)
        sensitivity[name] = {"mean_abs_action_change": float(diff.mean()), "max_abs_action_change": float(diff.max()),
                             "max_abs_delta_q_change_rad": float(0.1 * diff.max())}
    stats["sensitivity"] = sensitivity

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    all_pass = all(v["pass"] for v in results.values())
    summary = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "trace": str(Path(args.trace).resolve()), "actor": str(Path(args.actor).resolve()),
        "task": str(tr["task"]), "checkpoint": str(tr["checkpoint"]),
        "checkpoint_sha256": str(tr["checkpoint_sha256"]),
        "physics_dt": float(tr["physics_dt"]), "decimation": int(tr["decimation"]),
        "all_pass": all_pass, "checks": results, "stats": stats,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))

    lines = [f"- {'PASS' if v['pass'] else '**FAIL**'} `{k}`"
             + (f" — max|err| {v['max_abs_err']:.2e} (tol {v['tol']:.0e})" if "max_abs_err" in v else "")
             + (f" — {v['detail']}" if v.get("detail") else "") for k, v in results.items()]
    print("\n".join(lines))
    print(json.dumps(stats, indent=2))
    print(f"ALL PASS: {all_pass}")
    return summary


if __name__ == "__main__":
    main()
