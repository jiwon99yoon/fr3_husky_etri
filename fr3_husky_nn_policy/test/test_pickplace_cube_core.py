import csv
import math

import numpy as np

from fr3_husky_nn_policy.liftcube_logger import FIELDNAMES, LiftCubeRunLogger
from fr3_husky_nn_policy.pickplace_cube_core import (
    CUBE_REST_Z,
    OBS_DIM,
    READY_RIGHT_ARM,
    GripperCommandQueue,
    build_pickplace_observation,
    canonical_cube_quat,
    cube_start_error,
    cube_tilt_deg,
    goal_error,
    pick_place_result,
    tcp_position_base,
)


def _quat_z(deg):
    half = math.radians(deg) / 2.0
    return np.asarray([0.0, 0.0, math.sin(half), math.cos(half)])


def _quat_y(deg):
    half = math.radians(deg) / 2.0
    return np.asarray([0.0, math.sin(half), 0.0, math.cos(half)])


# sim right link0 in base: (0, -0.215, 0), yaw -30 deg (the ready joint1 +30 deg faces the arm forward)
SIM_MOUNT = (np.asarray([0.0, -0.215, 0.0]), _quat_z(-30.0))


def test_observation_layout():
    arm = READY_RIGHT_ARM + 0.1
    velocity = np.arange(7, dtype=np.float64)
    cube = np.asarray([0.55, -0.2, CUBE_REST_Z])
    tcp = np.asarray([0.30, -0.215, 0.41])
    goal = np.asarray([0.5, -0.35, CUBE_REST_Z])
    previous = np.arange(8, dtype=np.float64) * 0.01

    observation = build_pickplace_observation(arm, 0.04, velocity, cube, _quat_z(60.0), tcp, goal, previous, True)

    assert observation.shape == (OBS_DIM,)
    np.testing.assert_allclose(observation[0:7], 0.1, atol=1e-6)
    assert observation[7] == np.float32(0.04)
    np.testing.assert_array_equal(observation[8:15], velocity)
    np.testing.assert_allclose(observation[15:18], cube, atol=1e-6)
    np.testing.assert_allclose(observation[18:22], _quat_z(-30.0), atol=1e-6)  # 60 deg folds to -30 deg
    np.testing.assert_allclose(observation[22:25], cube - tcp, atol=1e-6)
    np.testing.assert_allclose(observation[25:28], goal, atol=1e-6)
    np.testing.assert_allclose(observation[28:36], previous, atol=1e-6)
    assert observation[36] == 1.0


def test_canonical_cube_quat_folds_yaw_and_keeps_tilt():
    for yaw, folded in ((0.0, 0.0), (44.0, 44.0), (60.0, -30.0), (100.0, 10.0), (-170.0, 10.0), (225.0, 45.0)):
        q = canonical_cube_quat(_quat_z(yaw))
        assert q[3] >= 0.0
        assert abs(abs(math.degrees(2.0 * math.atan2(q[2], q[3]))) - abs(folded)) < 1e-6
    tilted = canonical_cube_quat(-np.asarray([*(_quat_z(120.0)[:3]), _quat_z(120.0)[3]]))
    np.testing.assert_allclose(canonical_cube_quat(tilted), tilted, atol=1e-12)  # idempotent
    lean = np.asarray([0.0, math.sin(math.radians(10.0)), 0.0, math.cos(math.radians(10.0))])  # 20 deg about y
    assert abs(cube_tilt_deg(canonical_cube_quat(lean)) - 20.0) < 1e-6


def test_tcp_at_ready_pose():
    tcp = tcp_position_base(READY_RIGHT_ARM, *SIM_MOUNT)
    # dual_fr3_lab pick-place: ready TCP x 0.30, about 6.6 cm above the table top (0.345)
    np.testing.assert_allclose(tcp, [0.3069, -0.215, 0.4129], atol=1e-3)


def test_goal_and_start_checks():
    assert goal_error([0.55, -0.20]) is None
    assert "outside the trained" in goal_error([0.70, -0.20])
    assert "right link0 axis" in goal_error([0.65, -0.45])  # inside the rectangle, 0.69 m reach
    assert "from the cube" in goal_error([0.55, -0.20], [0.60, -0.25])
    assert goal_error([0.55, -0.20], [0.55, -0.35]) is None

    standing = np.asarray([0.50, -0.30, CUBE_REST_Z])
    assert cube_start_error(standing, _quat_z(30.0), 20.0) is None
    assert cube_start_error(standing + [-0.055, 0.0, 0.0], _quat_z(0.0), 20.0) is None  # 5 mm outside, vision noise
    assert "top-face marker" in cube_start_error(standing + [0.0, 0.0, 0.05], _quat_z(0.0), 20.0)
    assert "tilt" in cube_start_error(standing, _quat_y(30.0), 20.0)


def test_gripper_queue_matches_the_sim_delayed_gripper():
    queue = GripperCommandQueue()
    assert not queue.busy and queue.update(0.3) is None
    assert queue.update(-0.2) is True and queue.busy  # close sent
    assert queue.update(0.5) is None and queue.busy  # open requested while closing: waits
    queue.on_result()
    assert queue.busy  # the waiting open is not sent yet
    assert queue.update(0.5) is False and queue.busy and not queue.released
    queue.on_result()
    assert not queue.busy and queue.released
    assert queue.update(0.0) is None  # 0 is open, as in sim (close when < 0)


def test_pick_place_result():
    goal = np.asarray([0.55, -0.20, CUBE_REST_Z])
    cube = goal + [0.01, -0.01, 0.0]
    tcp = cube + [0.0, 0.0, 0.05 + 0.05]
    result = pick_place_result(cube, _quat_y(8.0), goal, tcp, released=True)
    assert result["success"] and not result["success_with_tilt"]  # 8 deg: vision tilt is not in success
    assert not pick_place_result(cube + [0.03, 0.0, 0.0], _quat_z(0.0), goal, tcp, True)["success"]
    # the policy closes the gripper again in the air after success: the open command is not part of success
    assert pick_place_result(cube, _quat_z(0.0), goal, tcp, released=False)["success"]
    assert not pick_place_result(cube, _quat_z(0.0), goal, tcp, True, cube_pose_fresh=False)["success"]
    assert not pick_place_result(cube, _quat_z(0.0), goal, cube + [0.0, 0.0, 0.07], True)["success"]


def test_logger_observation_width(tmp_path):
    assert sum(name.startswith("obs_") for name in FIELDNAMES) == 36  # cube lift unchanged
    logger = LiftCubeRunLogger(tmp_path, "pick_place", {"mode": "shadow"}, obs_dim=OBS_DIM)
    timing = {name: 0.0 for name in ("elapsed_s", "ros_time_ns", "step", "tick_period_ms", "joint_state_age_ms",
                                     "object_pose_age_ms", "object_stamp_age_ms", "inference_ms", "max_abs_delta_rad",
                                     "feedback_max_joint_error_rad", "feedback_command_age_s")}
    gripper = {"gripper_raw_action": 0.0, "gripper_closed_cmd": 0.0, "finger_width_m": 0.08, "gripper_cmd_age_s": 0.0}
    logger.append(timing, gripper, np.zeros(OBS_DIM), np.zeros(8), np.zeros(7), [0.0, 0.0, 0.0, 1.0])
    run_dir = logger.finalize("test")
    rows = list(csv.reader((run_dir / "trace.csv").open()))
    assert "obs_36" in rows[0] and len(rows[1]) == len(rows[0])
