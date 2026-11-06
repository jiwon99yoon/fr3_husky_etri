#!/usr/bin/env python3
"""Export a cabinet book-insertion RSL-RL checkpoint to an ETRI deployment NPZ.

Usage on the training PC (PyTorch + PyYAML available)::

    python export_cabinet_book_insert_actor.py \
        <run>/model_XXXX.pt /path/to/dual_fr3_cabinet_book_insert_model_XXXX.npz

The checkpoint's own params/env.yaml and params/agent.yaml are used to build the
embedded deployment manifest.  The resulting file contains only NumPy arrays and
JSON metadata; the ROS inference node does not need PyTorch or Isaac Lab.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import torch
import yaml

from .cabinet_book_insert_core import ACTION_DIM, JOINT_NAMES, OBSERVATION_TERMS

MANIFEST_SCHEMA = "dual_fr3_cabinet_book_insert_manifest/1"


class _ParamsLoader(yaml.SafeLoader):
    """Load Isaac's params YAML while treating its Python tags as plain data."""


def _construct_python_tag(loader, _suffix, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node, deep=True)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node, deep=True)
    return loader.construct_scalar(node)


_ParamsLoader.add_multi_constructor("tag:yaml.org,2002:python/", _construct_python_tag)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_joint_values(pattern_values, names, label):
    result = []
    for joint in names:
        matches = [value for pattern, value in pattern_values.items() if re.fullmatch(pattern, joint)]
        if len(matches) != 1:
            raise ValueError(f"{label}: {joint} matched {len(matches)} patterns in {pattern_values}")
        result.append(float(matches[0]))
    return result


def _load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.load(stream, Loader=_ParamsLoader)


def load_start_rows(table_path, start_open_deg, robot_base_pos):
    """Start joints/book positions the run reset from (one row per book start), mirroring reset_to_start_table.

    The open-door stage puts the left arm at the solved door-path pose closest to ``start_open_deg``."""
    with np.load(Path(table_path).expanduser(), allow_pickle=False) as table:
        names = [str(name) for name in table["joint_names"]]
        joint_pos = np.asarray(table["joint_pos"], dtype=np.float64)
        book_pos = np.asarray(table["book_pose_env"], dtype=np.float64)[:, :3] - np.asarray(robot_base_pos[:3])
        if start_open_deg > 0.0:
            path_id = int(np.argmin(np.abs(table["left_path_door"] - np.deg2rad(start_open_deg))))
            left = [names.index(f"left_fr3_joint{j}") for j in range(1, 8)]
            joint_pos[:, left] = table["left_path_q"][path_id]
    missing = [name for name in JOINT_NAMES if name not in names]
    if missing:
        raise ValueError(f"start table has no joints {missing}")
    rows = joint_pos[:, [names.index(name) for name in JOINT_NAMES]]
    return rows, book_pos


def build_manifest(env, agent, checkpoint, env_yaml, iteration, checkpoint_sha256, start_table=None):
    action_cfg = env["actions"]["arm_action"]
    names = tuple(str(name) for name in action_cfg["joint_names"])
    if names != JOINT_NAMES:
        raise ValueError(f"checkpoint joint order is {names!r}, expected {JOINT_NAMES!r}")
    policy_obs = env["observations"]["policy"]
    term_layout = []
    expected_funcs = {
        "joint_pos": "joint_pos_rel", "joint_vel": "joint_vel_rel",
        "door_angle": "door_angle", "book_pos": "book_pos_rel",
        "book_quat": "book_quat", "actions": "last_action",
    }
    sizes = dict(OBSERVATION_TERMS)
    for name, size in OBSERVATION_TERMS:
        term = policy_obs.get(name)
        if not isinstance(term, dict):
            raise ValueError(f"policy observation is missing term {name!r}")
        function = str(term.get("func", "")).split(":")[-1]
        if function != expected_funcs[name]:
            raise ValueError(f"observation {name} uses {function!r}, expected {expected_funcs[name]!r}")
        term_layout.append({"name": name, "size": size, "function": function})
    goal = policy_obs["book_pos"].get("params", {}).get("target_pos_b")
    if goal is None or len(goal) != 3:
        raise ValueError("book_pos observation has no 3-D target_pos_b")

    default = _resolve_joint_values(env["scene"]["robot"]["init_state"]["joint_pos"], names, "default position")
    clip_cfg = action_cfg.get("clip") or {}
    clip = clip_cfg.get(".*")
    if clip is None or len(clip) != 2:
        raise ValueError(f"unsupported action clip configuration: {clip_cfg!r}")
    target_limits = action_cfg.get("target_limits")
    if target_limits is None:
        raise ValueError("deployment requires explicit target_limits")
    if action_cfg.get("reference_mode") != "policy_step":
        raise ValueError("deployment exporter only supports reference_mode='policy_step'")

    agent_actor = agent.get("actor", {})
    if agent_actor.get("obs_normalization", False):
        raise ValueError("actor observation normalization is enabled; export path does not serialize its statistics")
    distribution = agent_actor.get("distribution_cfg") or {}
    distribution_class = str(distribution.get("class_name", ""))
    if not distribution_class.endswith("TanhSquashedGaussianDistribution"):
        raise ValueError(f"expected tanh-squashed Gaussian actor, got {distribution_class!r}")
    if agent_actor.get("activation") != "elu":
        raise ValueError(f"unsupported actor hidden activation {agent_actor.get('activation')!r}")

    reset_event = (env.get("events", {}).get("reset_start", {}) or {}).get("params", {})
    start_open_deg = float(reset_event.get("initial_open_deg", 0.0))
    if not 0.0 <= start_open_deg <= 120.0:
        raise ValueError(f"unsupported initial door angle {start_open_deg} deg")
    table_path = start_table or reset_event.get("table_path")
    if not table_path:
        raise ValueError("env.yaml reset_start has no table_path; pass --start-table")
    start_q, start_book = load_start_rows(table_path, start_open_deg, env["scene"]["robot"]["init_state"]["pos"])

    return {
        "schema": MANIFEST_SCHEMA,
        "provenance": {
            "checkpoint": checkpoint.name,
            "checkpoint_sha256": checkpoint_sha256,
            "iteration": int(iteration),
            "experiment_name": str(agent.get("experiment_name", "")),
            "run_name": str(agent.get("run_name", "")),
            "env_yaml_sha256": _sha256(env_yaml),
        },
        "actor": {
            "input_dim": sum(sizes.values()),
            "output_dim": ACTION_DIM,
            "hidden_dims": [int(v) for v in agent_actor.get("hidden_dims", [])],
            "activation": "elu",
            "output_activation": "tanh",
            "obs_normalization": False,
        },
        "joints": {"names": list(names), "default_position": default},
        "observation": {
            "frame": "base",
            "quaternion_order": "xyzw",
            "terms": term_layout,
            "book_position": "book_center_minus_insert_goal_in_base",
            "door_angle": "positive_when_open_rad",
            "joint_position": "measured_minus_robot_default",
            "previous_action": "previous_tanh_actor_output",
            "noise_training_std": {
                name: float((policy_obs[name].get("noise") or {}).get("std", 0.0))
                for name, _size in OBSERVATION_TERMS
            },
        },
        "goal": {"book_position_base": [float(v) for v in goal]},
        "start": {
            "door_angle_rad": float(np.deg2rad(start_open_deg)),
            "table_path": str(table_path),
            # rows the training reset drew from; start_policy refuses joints far from all of them
            "joint_position_rows": [[round(float(v), 6) for v in row] for row in start_q],
            "book_position_base_rows": [[round(float(v), 4) for v in row] for row in start_book],
        },
        "action": {
            "scale": float(action_cfg["scale"]),
            "clip": [float(clip[0]), float(clip[1])],
            "reference_mode": str(action_cfg["reference_mode"]),
            "target_limits": [[float(x) for x in row] for row in target_limits],
            "joint_command": "absolute_policy_step_target",
        },
        "policy": {
            "policy_dt": float(env["sim"]["dt"]) * int(env["decimation"]),
            "episode_length_s": float(env["episode_length_s"]),
        },
        "deployment_inputs": {
            "joint_states_topic": "/dual_fr3/joint_states",
            "book_pose_topic": "/cabinet_book_insert/book_pose",
            "door_angle_topic": "/cabinet_book_insert/door_angle",
            "book_pose_transform": "right link7 TF -> UMI TCP -> inverse fixed book grasp transform",
            "door_angle_transform": "left UMI TCP orbit about calibrated vertical cabinet hinge",
        },
    }


def export(checkpoint_path, output_path, env_yaml=None, start_table=None):
    checkpoint_path = Path(checkpoint_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    run_dir = checkpoint_path.parent
    env_yaml = Path(env_yaml).expanduser().resolve() if env_yaml else run_dir / "params" / "env.yaml"
    agent_yaml = env_yaml.parent / "agent.yaml"
    if not checkpoint_path.is_file() or not env_yaml.is_file() or not agent_yaml.is_file():
        raise FileNotFoundError(f"expected checkpoint, {env_yaml}, and {agent_yaml}")

    state = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    actor_state = state.get("actor_state_dict")
    if actor_state is None:
        raise KeyError("checkpoint has no actor_state_dict")
    layers = sorted(int(key.split(".")[1]) for key in actor_state if key.startswith("mlp.") and key.endswith(".weight"))
    if not layers or any(f"mlp.{index}.bias" not in actor_state for index in layers):
        raise ValueError("could not find actor MLP weights and biases")

    digest = _sha256(checkpoint_path)
    env, agent = _load_yaml(env_yaml), _load_yaml(agent_yaml)
    manifest = build_manifest(env, agent, checkpoint_path, env_yaml, state.get("iter", -1), digest, start_table)
    payload = {
        "format_version": np.asarray(2, dtype=np.int64),
        "activation": np.asarray("elu"),
        "output_activation": np.asarray("tanh"),
        "num_layers": np.asarray(len(layers), dtype=np.int64),
        "source_sha256": np.asarray(digest),
        "source_iteration": np.asarray(int(state.get("iter", -1)), dtype=np.int64),
        "manifest_json": np.asarray(json.dumps(manifest, separators=(",", ":"))),
    }
    for out_index, state_index in enumerate(layers):
        payload[f"weight_{out_index}"] = actor_state[f"mlp.{state_index}.weight"].detach().cpu().numpy().astype(np.float32)
        payload[f"bias_{out_index}"] = actor_state[f"mlp.{state_index}.bias"].detach().cpu().numpy().astype(np.float32)
    if payload["weight_0"].shape[1] != manifest["actor"]["input_dim"]:
        raise ValueError(f"checkpoint actor input is {payload['weight_0'].shape[1]}, manifest expects 50")
    if payload[f"weight_{len(layers)-1}"].shape[0] != ACTION_DIM:
        raise ValueError("checkpoint actor output is not 14-dimensional")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **payload)
    return output_path, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoint", type=Path, help="model_XXXX.pt in one RSL-RL run")
    parser.add_argument("output", type=Path, help="output portable .npz")
    parser.add_argument("--env-yaml", type=Path, help="params/env.yaml from the exact same run")
    parser.add_argument("--start-table", type=Path,
                        help="start_table.npz the run reset from (default: the path in env.yaml, i.e. the training PC)")
    args = parser.parse_args()
    path, manifest = export(args.checkpoint, args.output, args.env_yaml, args.start_table)
    print(f"wrote {path}")
    print(f"  experiment/run {manifest['provenance']['experiment_name']}/{manifest['provenance']['run_name']}")
    print(f"  iteration {manifest['provenance']['iteration']} sha256 {manifest['provenance']['checkpoint_sha256'][:12]}")
    print(f"  actor {manifest['actor']['input_dim']} -> {manifest['actor']['output_dim']}, "
          f"policy {1.0 / manifest['policy']['policy_dt']:.1f} Hz")
    print(f"  action step {manifest['action']['clip'][1]:.4f} rad "
          f"({manifest['action']['clip'][1] / manifest['policy']['policy_dt']:.2f} rad/s), "
          f"horizon {manifest['policy']['episode_length_s']:.1f} s, start door "
          f"{np.rad2deg(manifest['start']['door_angle_rad']):.0f} deg, "
          f"{len(manifest['start']['joint_position_rows'])} start rows")


if __name__ == "__main__":
    main()
