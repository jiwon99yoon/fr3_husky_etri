"""Runtime contract for the dual-FR3 cabinet book-insertion actor."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .numpy_actor import NumpyMLPActor

MANIFEST_SCHEMA = "dual_fr3_cabinet_book_insert_manifest/1"
JOINT_NAMES = tuple(
    f"{side}_fr3_joint{index}"
    for side in ("left", "right")
    for index in range(1, 8)
)
OBSERVATION_TERMS = (
    ("joint_pos", 14),
    ("joint_vel", 14),
    ("door_angle", 1),
    ("book_pos", 3),
    ("book_quat", 4),
    ("actions", 14),
)
OBS_DIM = sum(size for _, size in OBSERVATION_TERMS)
ACTION_DIM = 14


def _finite_vector(value, shape, name):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite with shape {shape}, got {array!r}")
    return array


def normalize_quat_xyzw(quat):
    quat = _finite_vector(quat, (4,), "book quaternion")
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-8:
        raise ValueError("book quaternion has near-zero norm")
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat


@dataclass(frozen=True)
class CabinetBookManifest:
    raw: dict
    joint_names: tuple[str, ...]
    default_position: np.ndarray
    action_scale: float
    action_clip: np.ndarray
    target_limits: np.ndarray
    target_book_position_base: np.ndarray
    policy_dt: float
    episode_length_s: float
    checkpoint_sha256: str
    start_joint_position: np.ndarray | None = None  # (rows, 14) JOINT_NAMES order, the training reset rows
    start_book_position_base: np.ndarray | None = None  # (rows, 3)

    @classmethod
    def from_dict(cls, raw):
        if raw.get("schema") != MANIFEST_SCHEMA:
            raise ValueError(f"unsupported manifest schema {raw.get('schema')!r}")
        joints, action, observation, policy = (
            raw.get("joints", {}), raw.get("action", {}),
            raw.get("observation", {}), raw.get("policy", {})
        )
        terms = tuple((str(item["name"]), int(item["size"])) for item in observation.get("terms", []))
        if terms != OBSERVATION_TERMS:
            raise ValueError(f"actor observation layout mismatch: {terms!r}")
        names = tuple(str(name) for name in joints.get("names", []))
        if names != JOINT_NAMES:
            raise ValueError(f"joint order mismatch: {names!r}; expected {JOINT_NAMES!r}")
        default = _finite_vector(joints.get("default_position"), (ACTION_DIM,), "default_position")
        limits = _finite_vector(action.get("target_limits"), (ACTION_DIM, 2), "target_limits")
        if np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("target_limits must have increasing lower/upper bounds")
        clip = _finite_vector(action.get("clip"), (2,), "action.clip")
        if clip[0] >= 0.0 or clip[1] <= 0.0:
            raise ValueError("action.clip must span zero")
        reference_mode = action.get("reference_mode")
        if reference_mode != "policy_step":
            raise ValueError(f"unsupported action reference_mode {reference_mode!r}")
        goal = _finite_vector(raw.get("goal", {}).get("book_position_base"), (3,), "book goal")
        digest = str(raw.get("provenance", {}).get("checkpoint_sha256", ""))
        if len(digest) != 64:
            raise ValueError("manifest has no valid checkpoint SHA-256")
        scale, dt, horizon = float(action.get("scale")), float(policy.get("policy_dt")), float(policy.get("episode_length_s"))
        if not (np.isfinite(scale) and scale > 0.0 and np.isfinite(dt) and dt > 0.0 and np.isfinite(horizon) and horizon > 0.0):
            raise ValueError("invalid action scale or policy timing in manifest")
        start = raw.get("start", {})
        start_q = start_book = None
        if "joint_position_rows" in start:  # exports before 2026-11-06 have no start rows
            start_q = np.asarray(start["joint_position_rows"], dtype=np.float64)
            start_book = np.asarray(start["book_position_base_rows"], dtype=np.float64)
            if start_q.ndim != 2 or start_q.shape[1] != ACTION_DIM or start_book.shape != (len(start_q), 3):
                raise ValueError(f"start rows have shapes {start_q.shape} / {start_book.shape}")
        return cls(raw, names, default, scale, clip, limits, goal, dt, horizon, digest, start_q, start_book)


def nearest_start_row(joint_position, manifest: CabinetBookManifest):
    """(row, largest joint error in rad) of the training start row closest to ``joint_position`` (max-norm)."""
    if manifest.start_joint_position is None:
        raise ValueError("this NPZ has no start rows; re-export it with export_cabinet_book_insert_actor")
    q = _finite_vector(joint_position, (ACTION_DIM,), "joint_position")
    error = np.abs(manifest.start_joint_position - q).max(axis=1)
    row = int(np.argmin(error))
    return row, float(error[row])


def load_cabinet_policy(model_path: str | Path):
    model_path = Path(model_path).expanduser().resolve()
    actor = NumpyMLPActor(model_path)
    with np.load(model_path, allow_pickle=False) as archive:
        if "manifest_json" not in archive.files:
            raise ValueError("NPZ has no manifest_json; export the checkpoint with export_cabinet_book_insert_actor")
        manifest = CabinetBookManifest.from_dict(json.loads(str(archive["manifest_json"].item())))
    if actor.input_dim != OBS_DIM or actor.output_dim != ACTION_DIM:
        raise ValueError(f"actor is {actor.input_dim}->{actor.output_dim}; expected {OBS_DIM}->{ACTION_DIM}")
    if actor.output_activation != "tanh":
        raise ValueError(f"actor output activation is {actor.output_activation!r}, expected 'tanh'")
    if actor.source_sha256 != manifest.checkpoint_sha256:
        raise ValueError("actor checkpoint digest and embedded manifest do not match")
    return actor, manifest


def build_observation(joint_position, joint_velocity, door_angle_rad, book_position_base,
                      book_quaternion_base_xyzw, previous_action, manifest: CabinetBookManifest):
    q = _finite_vector(joint_position, (ACTION_DIM,), "joint_position")
    qd = _finite_vector(joint_velocity, (ACTION_DIM,), "joint_velocity")
    book_p = _finite_vector(book_position_base, (3,), "book_position_base")
    prev = _finite_vector(previous_action, (ACTION_DIM,), "previous_action")
    angle = float(door_angle_rad)
    if not np.isfinite(angle):
        raise ValueError("door angle must be finite")
    quat = normalize_quat_xyzw(book_quaternion_base_xyzw)
    obs = np.concatenate((
        q - manifest.default_position,
        qd,
        np.asarray([angle]),
        book_p - manifest.target_book_position_base,
        quat,
        prev,
    )).astype(np.float32)
    if obs.shape != (OBS_DIM,) or not np.all(np.isfinite(obs)):
        raise ValueError(f"invalid observation generated: shape={obs.shape}")
    return obs


def process_action(action, joint_position, manifest: CabinetBookManifest):
    action = _finite_vector(action, (ACTION_DIM,), "actor action")
    q = _finite_vector(joint_position, (ACTION_DIM,), "joint_position")
    # Match Isaac Lab: scale the tanh actor output, clamp the relative delta, add it
    # to the measured policy-step joint position, then clamp absolute targets.
    delta = np.clip(manifest.action_scale * action, manifest.action_clip[0], manifest.action_clip[1])
    target = np.clip(q + delta, manifest.target_limits[:, 0], manifest.target_limits[:, 1])
    return target, target - q
