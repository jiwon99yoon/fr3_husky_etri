import json
from pathlib import Path

import numpy as np
import pytest

from fr3_husky_nn_policy.boxlift_core import (
    ACTION_DIM,
    GOAL_QUAT_XYZW,
    MANIFEST_SCHEMA,
    OBS_DIM,
    OBS_SLICES,
    OBS_TERMS,
    BoxLiftManifest,
    build_observation,
    canonicalize_box_quat,
    check_start_conditions,
    load_boxlift_policy,
    make_goal,
    pose_in_base,
    process_action,
    quat_mul,
    quat_rotate_inverse,
    transform_pose,
    validate_manifest,
    yaw_from_quat,
)
from fr3_husky_nn_policy.observation import DUAL_ARM_JOINT_NAMES

NEW_BOX = (0.410, 0.310, 0.280)
OLD_BOX = (0.555, 0.475, 0.275)
_ARM = [-0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
DEFAULT_POSE = [-0.5236, *_ARM, 0.5236, *_ARM]
_LIMITS = [(-2.6937, 2.6937), (-1.7337, 1.7337), (-2.8507, 2.8507), (-2.9921, -0.2018),
           (-2.7565, 2.7565), (0.5945, 4.4669), (-2.9659, 2.9659)] * 2
REPO = Path(__file__).resolve().parents[2]
TRACE_DIR = REPO / "data" / "boxlift_obs_parity_261103"
PREGRASP_MODEL = REPO / "model" / "dual_fr3_boxlift_pregrasp_box410_model2999.npz"


def yaw_quat(yaw):
    return np.asarray([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])


def manifest_dict(box_size=NEW_BOX, reference_mode="policy_step", sha="abc123", scale=(1.0, 1.0)):
    return {
        "schema": MANIFEST_SCHEMA,
        "provenance": {"checkpoint_sha256": sha},
        "policy": {"physics_dt": 0.01, "decimation": 5, "policy_dt": 0.05, "episode_length_s": 5.0,
                   "output_activation": "tanh"},
        "observation": {"terms": [[name, func] for name, func, _ in OBS_TERMS]},
        "joints": {"names": list(DUAL_ARM_JOINT_NAMES), "default_position": DEFAULT_POSE},
        "action": {"scale": 0.1, "clip": [-0.1, 0.1], "reference_mode": reference_mode,
                   "target_limits": [list(pair) for pair in _LIMITS]},
        "box": {"size": list(box_size), "size_scale_range": {a: list(scale) for a in "xyz"},
                "nominal_mass": 0.5, "mass_range": [0.4, 0.6], "static_friction_range": [0.4, 1.2],
                "start_x_range_base": [0.45, 0.6], "start_y_range_base": [-0.1, 0.1],
                "start_yaw_range": [-0.1, 0.1], "rest_center_z_base": 0.5 * box_size[2] - 0.405},
        "goal": {"command_class": "VerticalLiftPoseCommand", "z_range_base": [0.15, 0.35],
                 "quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "robot": {"root_height_m": 0.405},
    }


@pytest.fixture
def manifest():
    return BoxLiftManifest.from_dict(manifest_dict())


def write_actor(path, manifest=None, sha="abc123", in_dim=OBS_DIM, extra_arrays=None):
    rng = np.random.default_rng(0)
    payload = {
        "format_version": np.asarray(2), "activation": np.asarray("elu"), "output_activation": np.asarray("tanh"),
        "num_layers": np.asarray(2), "source_sha256": np.asarray(sha), "source_iteration": np.asarray(1),
        "weight_0": rng.normal(size=(8, in_dim)).astype(np.float32), "bias_0": np.zeros(8, np.float32),
        "weight_1": rng.normal(size=(ACTION_DIM, 8)).astype(np.float32), "bias_1": np.zeros(ACTION_DIM, np.float32),
    }
    if manifest is not None:
        payload["manifest_json"] = np.asarray(json.dumps(manifest))
    payload.update(extra_arrays or {})
    np.savez(path, **payload)
    return path


# ----------------------------------------------------------------------------- quaternions
def test_quat_rotate_inverse_matches_rotation_matrix():
    q = quat_mul(yaw_quat(0.7), np.asarray([np.sin(0.2), 0.0, 0.0, np.cos(0.2)]))
    x, y, z, w = q
    rotation = np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    v = np.asarray([0.3, -0.2, 0.5])
    np.testing.assert_allclose(quat_rotate_inverse(q, v), rotation.T @ v, atol=1e-12)


def test_pose_in_base_removes_base_height():
    position_b, quat_b = pose_in_base([0.5, 0.0, 0.14], yaw_quat(0.05), [0.0, 0.0, 0.405], GOAL_QUAT_XYZW)
    np.testing.assert_allclose(position_b, [0.5, 0.0, -0.265], atol=1e-12)
    assert yaw_from_quat(quat_b) == pytest.approx(0.05)


def test_transform_pose_inverts_pose_in_base():
    # camera frame 0.6 m above the base, pitched down 90 deg and yawed 30 deg
    camera_quat = quat_mul(yaw_quat(np.pi / 6), [0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)])
    camera_pos = np.asarray([0.1, -0.05, 0.6])
    box_pos_b, box_quat_b = np.asarray([0.52, 0.03, -0.25]), yaw_quat(0.07)
    in_camera = pose_in_base(box_pos_b, box_quat_b, camera_pos, camera_quat)
    position, quat = transform_pose(camera_pos, camera_quat, *in_camera)
    np.testing.assert_allclose(position, box_pos_b, atol=1e-12)
    np.testing.assert_allclose(canonicalize_box_quat(quat), box_quat_b, atol=1e-12)


@pytest.mark.parametrize("yaw", [-0.3, -0.05, 0.0, 0.08, 0.3])
def test_canonicalize_box_quat_removes_half_turn_and_sign(yaw):
    expected = yaw_quat(yaw)
    for reported in (expected, -expected, quat_mul(expected, [0, 0, 1, 0]), -quat_mul(expected, [0, 0, 1, 0])):
        result = canonicalize_box_quat(reported)
        np.testing.assert_allclose(result, expected, atol=1e-12)
        assert result[3] >= 0.0
        np.testing.assert_allclose(canonicalize_box_quat(result), result, atol=1e-12)


def test_canonicalize_box_quat_rejects_invalid():
    with pytest.raises(ValueError):
        canonicalize_box_quat([0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        canonicalize_box_quat([np.nan, 0.0, 0.0, 1.0])


# ----------------------------------------------------------------------------- observation / action
def test_obs_layout_is_56_in_training_order():
    assert OBS_DIM == 56 and ACTION_DIM == 14
    assert [name for name, _, _ in OBS_TERMS] == [
        "joint_pos", "joint_vel", "object_position", "target_object_position", "actions", "object_orientation"]
    assert OBS_SLICES["target_object_position"] == slice(31, 38)
    assert OBS_SLICES["object_orientation"] == slice(52, 56)


def test_build_observation_places_every_term(manifest):
    q = manifest.default_position + 0.01
    qd = np.linspace(-0.5, 0.5, 14)
    previous = np.linspace(-1.0, 1.0, 14)
    goal = make_goal([0.5, 0.02], 0.25, manifest)
    flipped_box = quat_mul(yaw_quat(0.04), [0, 0, 1, 0])  # detector reported the other half turn
    obs = build_observation(q, qd, [0.5, 0.02, -0.265], flipped_box, goal, previous, manifest.default_position)
    assert obs.shape == (56,) and obs.dtype == np.float32
    np.testing.assert_allclose(obs[OBS_SLICES["joint_pos"]], 0.01, atol=1e-6)
    np.testing.assert_allclose(obs[OBS_SLICES["joint_vel"]], qd, atol=1e-6)
    np.testing.assert_allclose(obs[OBS_SLICES["object_position"]], [0.5, 0.02, -0.265], atol=1e-6)
    np.testing.assert_allclose(obs[OBS_SLICES["target_object_position"]], [0.5, 0.02, 0.25, 0, 0, 0, 1], atol=1e-6)
    np.testing.assert_allclose(obs[OBS_SLICES["actions"]], previous, atol=1e-6)
    np.testing.assert_allclose(obs[OBS_SLICES["object_orientation"]], yaw_quat(0.04), atol=1e-6)


def test_goal_quaternion_is_xyzw_identity_not_wxyz(manifest):
    # Regression: a wxyz goal (1, 0, 0, 0) changes actions by ~0.3 on average (parity report).
    goal = make_goal([0.5, 0.0], 0.2, manifest)
    np.testing.assert_array_equal(goal[3:], [0.0, 0.0, 0.0, 1.0])


def test_make_goal_rejects_untrained_height(manifest):
    with pytest.raises(ValueError, match="outside the trained range"):
        make_goal([0.5, 0.0], 0.40, manifest)


def test_goal_xy_offset_defaults_to_vertical_lift(manifest):
    np.testing.assert_array_equal(manifest.goal_xy_offset_range, np.zeros((2, 2)))
    np.testing.assert_allclose(make_goal([0.5, 0.02], 0.2, manifest)[:2], [0.5, 0.02])
    with pytest.raises(ValueError, match="goal xy offset"):
        make_goal([0.5, 0.0], 0.2, manifest, [0.02, 0.0])


def test_goal_xy_offset_from_manifest_range():
    raw = manifest_dict()
    raw["goal"]["xy_offset_range_base"] = {"x": [-0.05, 0.05], "y": [-0.03, 0.03]}
    shifted = BoxLiftManifest.from_dict(raw)
    goal = make_goal([0.5, 0.02], 0.2, shifted, [0.04, -0.03])
    np.testing.assert_allclose(goal[:3], [0.54, -0.01, 0.2])
    np.testing.assert_array_equal(goal[3:], [0.0, 0.0, 0.0, 1.0])
    with pytest.raises(ValueError, match="goal xy offset"):
        make_goal([0.5, 0.0], 0.2, shifted, [0.0, 0.04])
    raw["goal"]["xy_offset_range_base"] = {"x": [0.05, -0.05], "y": [0.0, 0.0]}
    with pytest.raises(ValueError, match="xy_offset_range_base.x"):
        BoxLiftManifest.from_dict(raw)


def test_build_observation_rejects_bad_inputs(manifest):
    good = dict(joint_position=manifest.default_position, joint_velocity=np.zeros(14),
                box_position_b=[0.5, 0.0, -0.265], box_quat_b=GOAL_QUAT_XYZW,
                goal=make_goal([0.5, 0.0], 0.2, manifest), previous_action=np.zeros(14),
                default_position=manifest.default_position)
    with pytest.raises(ValueError, match="shape"):
        build_observation(**{**good, "joint_position": np.zeros(9)})
    with pytest.raises(ValueError, match="NaN"):
        build_observation(**{**good, "box_position_b": [np.nan, 0.0, 0.0]})


def test_process_action_clips_offset_then_joint_limits(manifest):
    q = manifest.default_position.copy()
    q[1] = 1.70  # left j2 near its trained upper limit 1.7337
    action = np.full(14, 0.5)
    action[0] = 1.0
    action[7] = -1.0
    target, delta = process_action(action, q, manifest)
    assert delta[0] == pytest.approx(0.1) and delta[7] == pytest.approx(-0.1)
    assert delta[2] == pytest.approx(0.05)
    assert target[1] == pytest.approx(1.7337)  # clamped by the limit, not 1.75
    assert np.all(target >= manifest.target_limits[:, 0]) and np.all(target <= manifest.target_limits[:, 1])


# ----------------------------------------------------------------------------- manifest / loading
def test_manifest_accepts_matching_box(manifest):
    validate_manifest(manifest, NEW_BOX)


def test_manifest_rejects_checkpoint_trained_on_other_box():
    old = BoxLiftManifest.from_dict(manifest_dict(box_size=OLD_BOX))
    with pytest.raises(ValueError, match="trained for another box"):
        validate_manifest(old, NEW_BOX)


def test_manifest_box_size_respects_trained_scale_range():
    randomized = BoxLiftManifest.from_dict(manifest_dict(box_size=(0.40, 0.30, 0.28), scale=(0.95, 1.05)))
    validate_manifest(randomized, NEW_BOX)
    with pytest.raises(ValueError, match="another box"):
        validate_manifest(randomized, (0.50, 0.30, 0.28))


def test_manifest_rejects_physics_step_reference():
    with pytest.raises(ValueError, match="policy_step"):
        validate_manifest(BoxLiftManifest.from_dict(manifest_dict(reference_mode="physics_step")), NEW_BOX)


def test_manifest_rejects_reordered_observation():
    raw = manifest_dict()
    raw["observation"]["terms"][0], raw["observation"]["terms"][1] = raw["observation"]["terms"][1], raw["observation"]["terms"][0]
    with pytest.raises(ValueError, match="observation terms"):
        validate_manifest(BoxLiftManifest.from_dict(raw), NEW_BOX)


def test_manifest_rejects_wxyz_goal_quaternion():
    raw = manifest_dict()
    raw["goal"]["quat_xyzw"] = [1.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="identity"):
        BoxLiftManifest.from_dict(raw)


def test_load_boxlift_policy_round_trip(tmp_path):
    path = write_actor(tmp_path / "actor.npz", manifest_dict())
    actor, manifest = load_boxlift_policy(path, NEW_BOX)
    assert manifest.box_size.tolist() == list(NEW_BOX)
    out = actor(np.zeros(OBS_DIM, np.float32))
    assert out.shape == (14,) and np.all(np.abs(out) <= 1.0)


def test_load_boxlift_policy_requires_manifest(tmp_path):
    with pytest.raises(ValueError, match="no manifest_json"):
        load_boxlift_policy(write_actor(tmp_path / "actor.npz"), NEW_BOX)


def test_load_boxlift_policy_rejects_mixed_checkpoint(tmp_path):
    path = write_actor(tmp_path / "actor.npz", manifest_dict(sha="other"), sha="abc123")
    with pytest.raises(ValueError, match="different checkpoints"):
        load_boxlift_policy(path, NEW_BOX)


def test_load_boxlift_policy_rejects_wrong_input_size(tmp_path):
    path = write_actor(tmp_path / "actor.npz", manifest_dict(), in_dim=58)
    with pytest.raises(ValueError, match="56->14"):
        load_boxlift_policy(path, NEW_BOX)


# ----------------------------------------------------------------------------- start gating (ready pose)
def test_start_conditions_accept_trained_start(manifest):
    check = check_start_conditions(manifest.default_position, [0.5, 0.0, manifest.rest_center_z],
                                   yaw_quat(0.05), manifest)
    assert check.ok and check.reasons == () and check.table_row is None


def test_start_conditions_list_every_violation(manifest):
    q = manifest.default_position.copy()
    q[3] += 0.3
    check = check_start_conditions(q, [0.7, 0.2, manifest.rest_center_z + 0.1], yaw_quat(0.3), manifest)
    assert not check.ok
    text = "\n".join(check.reasons)
    for expected in ("ready pose", "box x", "box y", "box yaw", "not resting"):
        assert expected in text


def test_start_conditions_accept_half_turned_detection(manifest):
    reported = quat_mul(yaw_quat(0.05), [0, 0, 1, 0])
    assert check_start_conditions(manifest.default_position, [0.5, 0.0, manifest.rest_center_z],
                                  reported, manifest).ok


# ----------------------------------------------------------------------------- start gating (pre-grasp table)
TABLE_Q = np.stack([np.asarray(DEFAULT_POSE) + offset for offset in (2.0, 2.2, 2.4)])
TABLE_BOX = np.asarray([[0.46, 0.00, -0.265, 0, 0, 0, 1],
                        [0.48, 0.05, -0.265, 0, 0, np.sin(0.02), np.cos(0.02)],
                        [0.50, -0.05, -0.265, 0, 0, 0, 1]])


def pregrasp_manifest_dict():
    raw = manifest_dict()
    raw["start"] = {"mode": "pregrasp_table", "rows": 3, "box_pos_noise_m": 0.01, "box_yaw_noise_rad": 0.05,
                    "joint_noise_rad": 0.01,
                    "arrays": {"joint_position": "pregrasp_joint_position", "box_pose_base": "pregrasp_box_pose_base"}}
    return raw


@pytest.fixture
def pregrasp():
    return BoxLiftManifest.from_dict(pregrasp_manifest_dict(), {
        "pregrasp_joint_position": TABLE_Q, "pregrasp_box_pose_base": TABLE_BOX})


def test_old_exports_without_start_field_are_ready_pose(manifest):
    assert manifest.start_mode == "ready_pose" and manifest.pregrasp_joint_position is None


def test_pregrasp_accepts_a_row_within_training_noise(pregrasp):
    q = TABLE_Q[1] + 0.025  # 0.01 noise + 0.02 margin = 0.03 allowed
    check = check_start_conditions(q, [0.488, 0.042, -0.252], yaw_quat(0.02 + 0.04), pregrasp)
    assert check.ok and check.table_row == 1


def test_pregrasp_rejects_ready_pose_and_names_nearest_row(pregrasp):
    check = check_start_conditions(pregrasp.default_position, [0.46, 0.0, -0.265], yaw_quat(0.0), pregrasp,
                                   ready_tolerance_rad=10.0)  # must not be used in pre-grasp mode
    assert not check.ok and check.table_row is None
    assert "table row 0" in check.reasons[0] and "arms 2.000 rad off" in check.reasons[0]


def test_pregrasp_requires_box_at_the_same_row_as_the_arms(pregrasp):
    # arms at row 0, box placed where row 2 expects it
    check = check_start_conditions(TABLE_Q[0], [0.50, -0.05, -0.265], yaw_quat(0.0), pregrasp)
    assert not check.ok
    check = check_start_conditions(TABLE_Q[0], [0.475, 0.0, -0.265], yaw_quat(0.0), pregrasp)
    assert not check.ok and "box x/y 0.015 m off (allowed 0.010)" in check.reasons[0]
    widened = check_start_conditions(TABLE_Q[0], [0.475, 0.0, -0.265], yaw_quat(0.0), pregrasp,
                                     start_margin_m=0.01)
    assert widened.ok and widened.table_row == 0


def test_pregrasp_yaw_uses_canonical_box_orientation(pregrasp):
    reported = quat_mul(yaw_quat(0.03), [0, 0, 1, 0])  # same box, other half turn
    assert check_start_conditions(TABLE_Q[0], [0.46, 0.0, -0.265], reported, pregrasp).table_row == 0
    assert not check_start_conditions(TABLE_Q[0], [0.46, 0.0, -0.265], yaw_quat(0.08), pregrasp).ok


def test_pregrasp_manifest_needs_embedded_table(tmp_path):
    with pytest.raises(ValueError, match="embedded table"):
        BoxLiftManifest.from_dict(pregrasp_manifest_dict())
    with pytest.raises(ValueError, match="embedded table"):
        load_boxlift_policy(write_actor(tmp_path / "a.npz", pregrasp_manifest_dict()), NEW_BOX)
    path = write_actor(tmp_path / "b.npz", pregrasp_manifest_dict(), extra_arrays={
        "pregrasp_joint_position": TABLE_Q, "pregrasp_box_pose_base": TABLE_BOX})
    _, manifest = load_boxlift_policy(path, NEW_BOX)
    assert manifest.start_mode == "pregrasp_table" and manifest.pregrasp_box_pose.shape == (3, 7)


def test_pregrasp_rejects_inconsistent_table():
    with pytest.raises(ValueError, match="pre-grasp table must be"):
        BoxLiftManifest.from_dict(pregrasp_manifest_dict(), {
            "pregrasp_joint_position": TABLE_Q[:2], "pregrasp_box_pose_base": TABLE_BOX})


@pytest.mark.skipif(not PREGRASP_MODEL.is_file(), reason="Pregrasp model_2999 not available")
def test_pregrasp_model_2999_documented_candidate_a():
    """docs/dual_fr3_box_experi_step.md candidate A: move_to_joint row 1919, box at (0.460, 0.0), real z."""
    _, manifest = load_boxlift_policy(PREGRASP_MODEL, NEW_BOX)
    left = [-0.0741, 1.6246, -0.2056, -1.3818, -1.0845, 1.6946, 0.8721]
    right = [0.0759, 1.6278, 0.2027, -1.3784, 1.0821, 1.6903, 0.7089]
    check = check_start_conditions(left + right, [0.460, 0.0, -0.252], yaw_quat(0.0), manifest)
    assert check.ok and check.table_row == 1919
    at_ready = check_start_conditions(manifest.default_position, [0.460, 0.0, -0.252], yaw_quat(0.0), manifest)
    assert not at_ready.ok


# ----------------------------------------------------------------------------- Isaac trace regression
@pytest.mark.skipif(not (TRACE_DIR / "actor_manifest.npz").is_file(), reason="Isaac parity trace not available")
def test_core_reproduces_recorded_isaac_policy_io():
    trace = np.load(TRACE_DIR / "trace.npz")
    with np.load(TRACE_DIR / "actor_manifest.npz") as archive:
        manifest = BoxLiftManifest.from_dict(json.loads(str(archive["manifest_json"])))
    from fr3_husky_nn_policy.numpy_actor import NumpyMLPActor

    actor = NumpyMLPActor(TRACE_DIR / "actor_manifest.npz")
    names = [str(n) for n in trace["joint_names"]]
    select = [names.index(n) for n in DUAL_ARM_JOINT_NAMES]
    obs = trace["obs"]
    for t in range(0, obs.shape[0], 7):
        for n in range(obs.shape[1]):
            q = trace["joint_pos"][t, n, select]
            position_b, quat_b = pose_in_base(trace["box_pos_w"][t, n], trace["box_quat_w"][t, n],
                                              trace["root_pos_w"][t, n], trace["root_quat_w"][t, n])
            rebuilt = build_observation(
                q, trace["joint_vel"][t, n, select], position_b, quat_b,
                obs[t, n, OBS_SLICES["target_object_position"]],   # goal/previous action are node state,
                obs[t, n, OBS_SLICES["actions"]],                  # checked by the parity script
                manifest.default_position)
            np.testing.assert_allclose(rebuilt, obs[t, n], atol=1e-5)
            np.testing.assert_allclose(actor(rebuilt), trace["action"][t, n], atol=1e-4)
            if not trace["done"][t, n]:
                target, _ = process_action(trace["action"][t, n], q, manifest)
                np.testing.assert_allclose(target, trace["held_target"][t, n], atol=1e-5)
