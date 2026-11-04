"""ROS-free core of the dual-FR3 plate box-lift policy deployment (numpy only).

Everything here must reproduce the Isaac Lab training interface exactly; it is unit tested
and checked against recorded Isaac traces (``scripts/tools/verify_boxlift_obs_parity.py``).
The ROS node only moves data in and out of these functions.

Conventions (Isaac Lab 3.0, as trained):

* quaternions are ``(x, y, z, w)``;
* all box and goal quantities are in the robot ``base`` frame (= training robot root);
* the box frame has its origin at the box center, x along the front-back edge, y along the
  edge the plates squeeze, z up.

Policy artifact: an NPZ written by ``dual_fr3_lab/scripts/tools/export_boxlift_actor.py``
that holds the actor weights and a ``manifest_json`` describing the training run.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from .numpy_actor import NumpyMLPActor
from .observation import DUAL_ARM_JOINT_NAMES

MANIFEST_SCHEMA = "dual_fr3_boxlift_manifest/1"
START_READY_POSE = "ready_pose"
START_PREGRASP_TABLE = "pregrasp_table"
# (term name, Isaac observation function, size), in policy input order
OBS_TERMS = (
    ("joint_pos", "joint_pos_rel", 14),
    ("joint_vel", "joint_vel_rel", 14),
    ("object_position", "object_position_in_robot_root_frame", 3),
    ("target_object_position", "generated_commands", 7),
    ("actions", "last_action", 14),
    ("object_orientation", "object_orientation_in_robot_root_frame", 4),
)
OBS_DIM = sum(size for _, _, size in OBS_TERMS)
ACTION_DIM = len(DUAL_ARM_JOINT_NAMES)
OBS_SLICES = {}
_offset = 0
for _name, _func, _size in OBS_TERMS:
    OBS_SLICES[_name] = slice(_offset, _offset + _size)
    _offset += _size
GOAL_QUAT_XYZW = np.asarray([0.0, 0.0, 0.0, 1.0])
_YAW_180_XYZW = np.asarray([0.0, 0.0, 1.0, 0.0])  # rotation of pi about the box's own z axis


# --------------------------------------------------------------------------- quaternions (x, y, z, w)
def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=-1,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.concatenate([-q[..., :3], q[..., 3:]], axis=-1)


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate ``v`` by the inverse of ``q`` (i.e. ``R(q)^T v``)."""
    v = np.asarray(v, dtype=np.float64)
    vq = np.concatenate([v, np.zeros(v.shape[:-1] + (1,))], axis=-1)
    return quat_mul(quat_mul(quat_conjugate(q), vq), q)[..., :3]


def yaw_from_quat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = np.moveaxis(np.asarray(q, dtype=np.float64), -1, 0)
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if not np.all(np.isfinite(q)) or np.any(norm < 1.0e-6):
        raise ValueError(f"invalid quaternion: {q}")
    return q / norm


def pose_in_base(position_w, quat_w, base_position_w, base_quat_w) -> tuple[np.ndarray, np.ndarray]:
    """Express a world pose in the base frame (as ``subtract_frame_transforms`` in Isaac Lab)."""
    position_b = quat_rotate_inverse(base_quat_w, np.asarray(position_w) - np.asarray(base_position_w))
    quat_b = quat_mul(quat_conjugate(base_quat_w), quat_w)
    return position_b, quat_b


def transform_pose(translation, rotation_xyzw, position, quat) -> tuple[np.ndarray, np.ndarray]:
    """Map a pose given in a source frame into a target frame.

    ``(translation, rotation_xyzw)`` is the source frame expressed in the target frame, i.e. a TF
    ``lookup_transform(target, source)``. Used to bring a camera-frame box pose into ``base``.
    """
    rotation = normalize_quat(rotation_xyzw)
    position_t = quat_rotate_inverse(quat_conjugate(rotation), position) + np.asarray(translation, dtype=np.float64)
    return position_t, quat_mul(rotation, normalize_quat(quat))


def canonicalize_box_quat(q: np.ndarray) -> np.ndarray:
    """Pick the representation of the 180-degree-symmetric box that training saw.

    The box looks identical after a half turn about its own z axis, so a detector may report
    either ``q`` or ``q * Rz(pi)``. Training spawned the box at small yaw, so keep the candidate
    with the smaller absolute yaw, then fix the sign to ``w >= 0`` as the training observation
    does. Idempotent; accepts batches.
    """
    q = normalize_quat(q)
    flipped = quat_mul(q, _YAW_180_XYZW)
    use_flipped = np.abs(yaw_from_quat(flipped)) < np.abs(yaw_from_quat(q))
    chosen = np.where(use_flipped[..., None], flipped, q)
    return chosen * np.where(chosen[..., 3:4] >= 0.0, 1.0, -1.0)


# --------------------------------------------------------------------------- manifest
def _pair(value, name: str) -> np.ndarray:
    pair = np.asarray(value, dtype=np.float64)
    if pair.shape != (2,) or not np.all(np.isfinite(pair)) or pair[0] > pair[1]:
        raise ValueError(f"{name} must be a finite (low, high) pair, got {value!r}")
    return pair


@dataclass(frozen=True)
class BoxLiftManifest:
    """Deployment-relevant facts of one training run (parsed ``manifest_json``)."""

    raw: dict
    joint_names: tuple[str, ...]
    default_position: np.ndarray       # (14,)
    action_scale: float
    action_clip: np.ndarray            # (2,)
    reference_mode: str
    target_limits: np.ndarray          # (14, 2)
    box_size: np.ndarray               # (3,) nominal x, y (squeeze), z
    box_size_scale_range: np.ndarray   # (3, 2)
    box_mass_range: np.ndarray | None  # (2,)
    start_x_range: np.ndarray          # (2,) base frame
    start_y_range: np.ndarray
    start_yaw_range: np.ndarray
    rest_center_z: float               # box center z in base when resting on the floor (sim)
    goal_z_range: np.ndarray           # (2,) base frame
    policy_dt: float
    episode_length_s: float
    root_height: float
    checkpoint_sha256: str
    observation_terms: tuple[tuple[str, str], ...]
    # Where training episodes start (manifest["start"]; exports without it are "ready_pose"):
    # "ready_pose": arms at default_position, box on the floor in the start x/y/yaw range.
    # "pregrasp_table": arms at one row of the embedded pre-grasp table (+ joint noise), box at that
    #                   row's pose (+ position/yaw noise).
    start_mode: str = START_READY_POSE
    pregrasp_joint_position: np.ndarray | None = None  # (rows, 14) manifest joint order
    pregrasp_box_pose: np.ndarray | None = None        # (rows, 7) base frame, position + quat xyzw
    start_joint_noise: float = 0.0                     # uniform +/- training reset noise
    start_box_pos_noise: float = 0.0
    start_box_yaw_noise: float = 0.0
    # Goal x/y = box start x/y + offset; per-axis (low, high) offset range sampled in training
    # (manifest goal.xy_offset_range_base). Exports without it trained a pure vertical lift: (0, 0).
    goal_xy_offset_range: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))

    @classmethod
    def from_dict(cls, raw: dict, arrays: Mapping[str, np.ndarray] | None = None) -> "BoxLiftManifest":
        """Parse ``manifest_json``; ``arrays`` are the NPZ arrays a pre-grasp table start refers to."""
        if raw.get("schema") != MANIFEST_SCHEMA:
            raise ValueError(f"unsupported manifest schema {raw.get('schema')!r}; expected {MANIFEST_SCHEMA!r}")
        try:
            joints, action, box, goal = raw["joints"], raw["action"], raw["box"], raw["goal"]
            scale_range = box["size_scale_range"]
            limits = np.asarray(action["target_limits"], dtype=np.float64)
            start = _parse_start(raw.get("start") or {"mode": START_READY_POSE}, arrays)
            offset_range = goal.get("xy_offset_range_base")
            if offset_range is not None:
                start["goal_xy_offset_range"] = np.stack(
                    [_pair(offset_range[a], f"goal.xy_offset_range_base.{a}") for a in "xy"])
            manifest = cls(
                raw=raw,
                joint_names=tuple(str(n) for n in joints["names"]),
                default_position=np.asarray(joints["default_position"], dtype=np.float64),
                action_scale=float(action["scale"]),
                action_clip=_pair(action["clip"], "action.clip"),
                reference_mode=str(action["reference_mode"]),
                target_limits=limits,
                box_size=np.asarray(box["size"], dtype=np.float64),
                box_size_scale_range=np.stack([_pair(scale_range[a], f"scale.{a}") for a in "xyz"]),
                box_mass_range=None if box.get("mass_range") is None else _pair(box["mass_range"], "mass"),
                start_x_range=_pair(box["start_x_range_base"], "start_x"),
                start_y_range=_pair(box["start_y_range_base"], "start_y"),
                start_yaw_range=_pair(box["start_yaw_range"], "start_yaw"),
                rest_center_z=float(box["rest_center_z_base"]),
                goal_z_range=_pair(goal["z_range_base"], "goal.z"),
                policy_dt=float(raw["policy"]["policy_dt"]),
                episode_length_s=float(raw["policy"]["episode_length_s"]),
                root_height=float(raw["robot"]["root_height_m"]),
                checkpoint_sha256=str(raw["provenance"]["checkpoint_sha256"]),
                observation_terms=tuple((str(n), str(f)) for n, f in raw["observation"]["terms"]),
                **start,
            )
        except (KeyError, TypeError) as error:
            raise ValueError(f"malformed box-lift manifest: {error!r}") from error
        if manifest.default_position.shape != (ACTION_DIM,):
            raise ValueError("manifest default_position must have 14 values")
        if limits.shape != (ACTION_DIM, 2) or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("manifest target_limits must be 14 increasing (low, high) pairs")
        if manifest.box_size.shape != (3,) or np.any(manifest.box_size <= 0.0):
            raise ValueError("manifest box size must be three positive lengths")
        if np.any(np.asarray(goal["quat_xyzw"], dtype=np.float64) != GOAL_QUAT_XYZW):
            raise ValueError("manifest goal quaternion must be the identity (0, 0, 0, 1)")
        return manifest

    def box_size_bounds(self) -> np.ndarray:
        """(3, 2) per-axis size range covered by training (nominal size x scale range)."""
        return self.box_size[:, None] * self.box_size_scale_range


def _parse_start(start: Mapping, arrays: Mapping[str, np.ndarray] | None) -> dict:
    mode = str(start.get("mode"))
    if mode == START_READY_POSE:
        return {"start_mode": mode}
    if mode != START_PREGRASP_TABLE:
        raise ValueError(f"unsupported start mode {mode!r}")
    names = start["arrays"]
    missing = [names[key] for key in ("joint_position", "box_pose_base") if arrays is None or names[key] not in arrays]
    if missing:
        raise ValueError(f"pre-grasp start needs the embedded table arrays {missing} from the same NPZ")
    joint_position = np.asarray(arrays[names["joint_position"]], dtype=np.float64)
    box_pose = np.asarray(arrays[names["box_pose_base"]], dtype=np.float64)
    rows = int(start["rows"])
    if joint_position.shape != (rows, ACTION_DIM) or box_pose.shape != (rows, 7):
        raise ValueError(f"pre-grasp table must be ({rows}, 14) joints and ({rows}, 7) box poses, "
                         f"got {joint_position.shape} and {box_pose.shape}")
    if rows == 0 or not (np.all(np.isfinite(joint_position)) and np.all(np.isfinite(box_pose))):
        raise ValueError("pre-grasp table is empty or not finite")
    noise = {key: float(start[key]) for key in ("joint_noise_rad", "box_pos_noise_m", "box_yaw_noise_rad")}
    if any(not np.isfinite(v) or v < 0.0 for v in noise.values()):
        raise ValueError(f"pre-grasp noise must be finite and non-negative, got {noise}")
    return {
        "start_mode": mode,
        "pregrasp_joint_position": joint_position,
        "pregrasp_box_pose": box_pose,
        "start_joint_noise": noise["joint_noise_rad"],
        "start_box_pos_noise": noise["box_pos_noise_m"],
        "start_box_yaw_noise": noise["box_yaw_noise_rad"],
    }


def validate_manifest(manifest: BoxLiftManifest, physical_box_size: Sequence[float],
                      size_tolerance: float = 0.005) -> None:
    """Raise ``ValueError`` listing every way this run does not match the deployed setup."""
    problems = []
    expected_terms = tuple((name, func) for name, func, _ in OBS_TERMS)
    if manifest.observation_terms != expected_terms:
        problems.append(f"observation terms {manifest.observation_terms} != {expected_terms}")
    if manifest.joint_names != tuple(DUAL_ARM_JOINT_NAMES):
        problems.append(f"joint order {manifest.joint_names} != {tuple(DUAL_ARM_JOINT_NAMES)}")
    if manifest.reference_mode != "policy_step":
        problems.append(f"reference_mode {manifest.reference_mode!r} != 'policy_step'")
    if not (manifest.action_scale > 0.0 and manifest.action_clip[0] < 0.0 < manifest.action_clip[1]):
        problems.append(f"invalid action scale/clip {manifest.action_scale}, {manifest.action_clip.tolist()}")
    physical = np.asarray(physical_box_size, dtype=np.float64)
    if physical.shape != (3,):
        problems.append(f"physical box size must be (x, y, z), got {physical_box_size!r}")
    else:
        bounds = manifest.box_size_bounds()
        outside = (physical < bounds[:, 0] - size_tolerance) | (physical > bounds[:, 1] + size_tolerance)
        if np.any(outside):
            problems.append(
                f"physical box {physical.tolist()} m is outside the trained box size "
                f"{bounds.round(4).tolist()} m (x, y=squeeze, z); this checkpoint was trained for another box"
            )
    if problems:
        raise ValueError("box-lift checkpoint rejected:\n  - " + "\n  - ".join(problems))


def load_boxlift_policy(model_path: str | Path, physical_box_size: Sequence[float],
                        size_tolerance: float = 0.005) -> tuple[NumpyMLPActor, BoxLiftManifest]:
    """Load the actor and its manifest, and refuse anything that does not fit the deployed box."""
    actor = NumpyMLPActor(model_path)
    with np.load(model_path, allow_pickle=False) as archive:
        if "manifest_json" not in archive:
            raise ValueError(
                f"{model_path} has no manifest_json; export it with "
                "dual_fr3_lab/scripts/tools/export_boxlift_actor.py"
            )
        raw = json.loads(str(archive["manifest_json"]))
        table_arrays = {name: archive[name] for name in archive.files if name.startswith("pregrasp_")}
        manifest = BoxLiftManifest.from_dict(raw, table_arrays)
    if (actor.input_dim, actor.output_dim) != (OBS_DIM, ACTION_DIM):
        raise ValueError(f"box-lift actor must be {OBS_DIM}->{ACTION_DIM}, got {actor.input_dim}->{actor.output_dim}")
    if actor.output_activation != "tanh":
        raise ValueError(f"box-lift actor must be tanh-bounded, got {actor.output_activation!r}")
    if actor.source_sha256 != manifest.checkpoint_sha256:
        raise ValueError("actor weights and manifest come from different checkpoints")
    validate_manifest(manifest, physical_box_size, size_tolerance)
    return actor, manifest


# --------------------------------------------------------------------------- observation / action
def make_goal(start_xy: Sequence[float], goal_z: float, manifest: BoxLiftManifest,
              xy_offset: Sequence[float] = (0.0, 0.0)) -> np.ndarray:
    """Goal term: box start x/y + xy_offset, chosen lift height z, identity quaternion (x, y, z, w)."""
    start_xy = np.asarray(start_xy, dtype=np.float64)
    if start_xy.shape != (2,) or not np.all(np.isfinite(start_xy)):
        raise ValueError(f"start_xy must be two finite values, got {start_xy!r}")
    low, high = manifest.goal_z_range
    if not (np.isfinite(goal_z) and low <= goal_z <= high):
        raise ValueError(f"goal z {goal_z} is outside the trained range [{low}, {high}] (base frame)")
    xy_offset = np.asarray(xy_offset, dtype=np.float64)
    if xy_offset.shape != (2,) or not np.all(np.isfinite(xy_offset)):
        raise ValueError(f"goal xy offset must be two finite values, got {xy_offset!r}")
    offset_range = manifest.goal_xy_offset_range
    if np.any(xy_offset < offset_range[:, 0] - 1e-9) or np.any(xy_offset > offset_range[:, 1] + 1e-9):
        raise ValueError(
            f"goal xy offset {xy_offset.tolist()} is outside the trained range "
            f"x {offset_range[0].tolist()}, y {offset_range[1].tolist()} (base frame)"
        )
    return np.concatenate([start_xy + xy_offset, [float(goal_z)], GOAL_QUAT_XYZW])


def build_observation(joint_position, joint_velocity, box_position_b, box_quat_b, goal,
                      previous_action, default_position) -> np.ndarray:
    """Assemble the 56-D policy input in training order (all poses in the base frame).

    The box quaternion is canonicalized here (180-degree yaw choice, ``w >= 0``), so callers can
    pass the detector output as is once it is expressed at the box center in the base frame.
    """
    parts = {
        "joint_position": (joint_position, (ACTION_DIM,)),
        "joint_velocity": (joint_velocity, (ACTION_DIM,)),
        "box_position_b": (box_position_b, (3,)),
        "box_quat_b": (box_quat_b, (4,)),
        "goal": (goal, (7,)),
        "previous_action": (previous_action, (ACTION_DIM,)),
        "default_position": (default_position, (ACTION_DIM,)),
    }
    values = {}
    for name, (value, shape) in parts.items():
        array = np.asarray(value, dtype=np.float64)
        if array.shape != shape:
            raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} contains NaN or Inf")
        values[name] = array
    observation = np.concatenate(
        [
            values["joint_position"] - values["default_position"],
            values["joint_velocity"],
            values["box_position_b"],
            values["goal"],
            values["previous_action"],
            canonicalize_box_quat(values["box_quat_b"]),
        ]
    ).astype(np.float32)
    assert observation.shape == (OBS_DIM,)
    return observation


def process_action(action, joint_position, manifest: BoxLiftManifest) -> tuple[np.ndarray, np.ndarray]:
    """Training action term (policy_step): returns (absolute joint target, joint offset)."""
    action = np.asarray(action, dtype=np.float64)
    joint_position = np.asarray(joint_position, dtype=np.float64)
    if action.shape[-1] != ACTION_DIM or joint_position.shape[-1] != ACTION_DIM:
        raise ValueError("action and joint_position must have 14 values")
    delta = np.clip(manifest.action_scale * action, manifest.action_clip[0], manifest.action_clip[1])
    target = np.clip(joint_position + delta, manifest.target_limits[:, 0], manifest.target_limits[:, 1])
    return target, target - joint_position


@dataclass(frozen=True)
class StartCheck:
    """Result of :func:`check_start_conditions`. ``table_row`` is the matched pre-grasp row (if any)."""

    reasons: tuple[str, ...]
    table_row: int | None = None

    @property
    def ok(self) -> bool:
        return not self.reasons


def _wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def check_start_conditions(joint_position, box_position_b, box_quat_b, manifest: BoxLiftManifest, *,
                           ready_tolerance_rad: float = 0.20, start_margin_m: float = 0.0,
                           yaw_margin_rad: float = 0.0, rest_z_tolerance_m: float = 0.03,
                           pregrasp_joint_margin_rad: float = 0.02) -> StartCheck:
    """Check that the robot and box are inside the start distribution the checkpoint trained on.

    ``ready_pose`` start: arms within ``ready_tolerance_rad`` of the default pose, box x/y/yaw inside the
    trained start range widened by ``start_margin_m`` / ``yaw_margin_rad``.

    ``pregrasp_table`` start: training reset the arms to one table row (+/- joint noise) and the box to that
    row's pose (+/- position/yaw noise). Accept only if some row has the arms within
    ``joint noise + pregrasp_joint_margin_rad`` on every joint AND the box within ``position noise +
    start_margin_m`` in x/y and ``yaw noise + yaw_margin_rad`` in yaw. ``ready_tolerance_rad`` is unused.

    Both: the box center must rest on the floor (``rest_z_tolerance_m``).
    """
    reasons = []
    joint_position = np.asarray(joint_position, dtype=np.float64)
    x, y, z = np.asarray(box_position_b, dtype=np.float64)
    yaw = float(yaw_from_quat(canonicalize_box_quat(box_quat_b)))
    table_row = None

    if manifest.start_mode == START_PREGRASP_TABLE:
        table_q, table_box = manifest.pregrasp_joint_position, manifest.pregrasp_box_pose
        joint_tolerance = manifest.start_joint_noise + pregrasp_joint_margin_rad
        position_tolerance = manifest.start_box_pos_noise + start_margin_m
        yaw_tolerance = manifest.start_box_yaw_noise + yaw_margin_rad
        joint_error = np.max(np.abs(table_q - joint_position), axis=1)
        position_error = np.max(np.abs(table_box[:, :2] - [x, y]), axis=1)
        yaw_error = np.abs(_wrap_angle(yaw_from_quat(canonicalize_box_quat(table_box[:, 3:7])) - yaw))
        score = np.maximum.reduce([joint_error / max(joint_tolerance, 1e-9),
                                   position_error / max(position_tolerance, 1e-9),
                                   yaw_error / max(yaw_tolerance, 1e-9)])
        row = int(np.argmin(score))
        table_row = row
        if score[row] > 1.0:
            table_row = None
            detail = []
            if joint_error[row] > joint_tolerance:
                detail.append(f"arms {joint_error[row]:.3f} rad off (allowed {joint_tolerance:.3f})")
            if position_error[row] > position_tolerance:
                detail.append(f"box x/y {position_error[row]:.3f} m off (allowed {position_tolerance:.3f})")
            if yaw_error[row] > yaw_tolerance:
                detail.append(f"box yaw {yaw_error[row]:.3f} rad off (allowed {yaw_tolerance:.3f})")
            reasons.append(
                f"not at a trained pre-grasp start: nearest table row {row} (box {table_box[row, 0]:.3f}, "
                f"{table_box[row, 1]:.3f}) has " + ", ".join(detail)
                + f"; closest arm match is {float(joint_error.min()):.3f} rad away"
            )
    else:
        ready_error = float(np.max(np.abs(joint_position - manifest.default_position)))
        if ready_error > ready_tolerance_rad:
            reasons.append(f"arms are {ready_error:.3f} rad from the training ready pose (> {ready_tolerance_rad})")
        for axis, value, (low, high) in (("x", x, manifest.start_x_range), ("y", y, manifest.start_y_range)):
            if not (low - start_margin_m <= value <= high + start_margin_m):
                reasons.append(f"box {axis}={value:.3f} m outside the trained start range [{low:.3f}, {high:.3f}]")
        low, high = manifest.start_yaw_range
        if not (low - yaw_margin_rad <= yaw <= high + yaw_margin_rad):
            reasons.append(f"box yaw={yaw:.3f} rad outside the trained start range [{low:.3f}, {high:.3f}]")

    if abs(z - manifest.rest_center_z) > rest_z_tolerance_m:
        reasons.append(
            f"box center z={z:.3f} m is not resting on the floor (trained {manifest.rest_center_z:.3f} "
            f"+/- {rest_z_tolerance_m})"
        )
    return StartCheck(tuple(reasons), table_row)
