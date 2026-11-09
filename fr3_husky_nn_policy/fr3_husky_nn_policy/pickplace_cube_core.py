"""Cube pick-and-place deployment core (ROS-free): the 37-D actor observation, the trained start/goal ranges, the
gripper command queue and the session result.

Training: dual_fr3_lab 3.0.0/6.0 ``tasks/manager_based/pick_place_cube``, tasks
``Isaac-PickPlace-Cube-DualFR3-UMI-GripperBusy{,-Tilt,-CriticPlus}-v0`` (2026-11-09). The three runs share this actor
contract (only the critic and the rewards differ), so any of their checkpoints runs on the same node.
Right FR3 + Franka hand with UMI fingertips, 5 x 5 x 10 cm cube standing on the 75 cm table (top base z 0.345, front
edge x 0.40), 20 Hz policy, 14 s episodes.

Actor observation (37-D, training order):
    0-6    right arm joint position - ready pose
    7      right_fr3_finger_joint1 position (absolute, 0 closed - 0.04 open; the real topic updates every ~0.27 s)
    8-14   right arm joint velocity
    15-17  cube centre position (base, vision)
    18-21  cube orientation (base, xyzw), folded over the 4 z-axis 90 deg symmetries (canonical_cube_quat)
    22-24  cube position - right UMI TCP (fingertip centre, FK)
    25-27  goal cube centre (base), z = CUBE_REST_Z
    28-35  previous action (7 arm + 1 gripper, tanh output)
    36     gripper busy: 1 from sending a goal until its result (and while a changed request waits to be sent)

TODO(UMI 실측): the right hand now has UMI fingertips but these were not measured on the real robot yet. Check and,
if they differ, fix the node or retrain:
  - finger_joint1 when grasping the 5 cm cube at 30 N: sim SIM_FINGER_GRASP_Q (23.4 mm); fully open: 0.04
  - TCP offset link7 -> fingertip centre: sim UMI_TCP_OFFSET (z 0.2844 m; same value the cabinet FK node uses)
"""

from __future__ import annotations

import math

import numpy as np

from .boxlift_core import normalize_quat, quat_mul, transform_pose, yaw_from_quat
from .fr3_kinematics import fr3_link7_in_link0
from .observation import DEFAULT_RIGHT_JOINT_POSITION

OBS_DIM = 37
ACTION_DIM = 8
EPISODE_LENGTH_S = 14.0
ACTION_SCALE = 0.1  # rad per unit action
ACTION_CLIP = 0.1  # rad, |q_target - q(t_k)| bound after the scale
READY_RIGHT_ARM = np.asarray(DEFAULT_RIGHT_JOINT_POSITION[:7], dtype=np.float64)
# sim arm target limits (dual_fr3_lab FR3_JOINT_POS_LIMITS, FR3 datasheet): q + delta is clamped to these as in training
TARGET_LOWER = np.asarray([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159], dtype=np.float64)
TARGET_UPPER = np.asarray([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159], dtype=np.float64)

TABLE_TOP_Z = 0.345
CUBE_SIZE = (0.05, 0.05, 0.10)
CUBE_REST_Z = TABLE_TOP_Z + 0.5 * CUBE_SIZE[2]  # standing cube centre; the goal z the policy sees
# Pick and place region (cube centre, base): this rectangle and within REACH_RADIUS of the right link0 axis (sim
# layout, link0 at y -0.215). The goal is at least GOAL_MIN_DIST from the cube.
CUBE_X = (0.45, 0.65)
CUBE_Y = (-0.45, 0.05)
RIGHT_LINK0_XY = (0.0, -0.215)
REACH_RADIUS = 0.62
GOAL_MIN_DIST = 0.10
START_TOL = 0.01  # cube start may be this far outside the region (vision noise; sim pose offset +-5 mm)
START_Z_TOL = 0.03  # standing cube centre within CUBE_REST_Z +- this at start
# success as in training: xy within XY_TOL, tilt < UPRIGHT_DEG, released, TCP HAND_CLEAR above the cube top
XY_TOL = 0.025
UPRIGHT_DEG = 5.0
HAND_CLEAR = 0.03

# TODO(UMI 실측): sim values, see the module docstring
UMI_TCP_OFFSET = (0.0, 0.0, 0.2844)
SIM_FINGER_GRASP_Q = 0.0234
SIM_FINGER_OPEN_Q = 0.04


def canonical_cube_quat(quat) -> np.ndarray:
    """As the sim's ``canonical_cube_quat``: the square-section cube is the same after any z-axis 90 deg turn, so take
    the candidate with the smallest rotation angle (largest |w|), sign w >= 0. A standing cube's yaw folds to +-45 deg.
    (x, y, z, w)."""
    q = normalize_quat(quat)
    best = None
    for k in range(4):
        half = 0.25 * math.pi * k
        candidate = quat_mul(q, np.asarray([0.0, 0.0, math.sin(half), math.cos(half)]))
        if best is None or abs(candidate[3]) > abs(best[3]):
            best = candidate
    return best if best[3] >= 0.0 else -best


def cube_tilt_deg(quat) -> float:
    """Angle between the cube's long (z) axis and base z, degrees (sim ``tilt``)."""
    x, y, _, _ = normalize_quat(quat)
    return math.degrees(math.acos(float(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))


def tcp_position_base(arm_position, mount_position, mount_quat, tcp_offset=UMI_TCP_OFFSET) -> np.ndarray:
    """Right UMI TCP in base: base <- link0 mount (TF), link0 <- link7 (FR3 DH FK), link7 <- TCP offset."""
    T = fr3_link7_in_link0(np.asarray(arm_position, dtype=np.float64))
    tcp_link0 = T[:3, :3] @ np.asarray(tcp_offset, dtype=np.float64) + T[:3, 3]
    return transform_pose(mount_position, mount_quat, tcp_link0, [0.0, 0.0, 0.0, 1.0])[0]


def build_pickplace_observation(arm_position, finger_position, arm_velocity, cube_position, cube_quat, tcp_position,
                                goal_position, previous_action, gripper_busy) -> np.ndarray:
    """Exact 37-D actor input (layout in the module docstring). ``cube_quat`` may be raw; it is folded here."""
    values = {
        "arm_position": (np.asarray(arm_position, dtype=np.float64), (7,)),
        "finger_position": (np.asarray([finger_position], dtype=np.float64).reshape(-1), (1,)),
        "arm_velocity": (np.asarray(arm_velocity, dtype=np.float64), (7,)),
        "cube_position": (np.asarray(cube_position, dtype=np.float64), (3,)),
        "cube_quat": (np.asarray(cube_quat, dtype=np.float64), (4,)),
        "tcp_position": (np.asarray(tcp_position, dtype=np.float64), (3,)),
        "goal_position": (np.asarray(goal_position, dtype=np.float64), (3,)),
        "previous_action": (np.asarray(previous_action, dtype=np.float64), (ACTION_DIM,)),
    }
    for name, (value, shape) in values.items():
        if value.shape != shape:
            raise ValueError(f"{name} must have shape {shape}, got {value.shape}")
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{name} contains NaN or Inf")
    cube = values["cube_position"][0]
    observation = np.concatenate(
        (
            values["arm_position"][0] - READY_RIGHT_ARM,
            values["finger_position"][0],
            values["arm_velocity"][0],
            cube,
            canonical_cube_quat(values["cube_quat"][0]),
            cube - values["tcp_position"][0],
            values["goal_position"][0],
            values["previous_action"][0],
            [1.0 if gripper_busy else 0.0],
        )
    ).astype(np.float32)
    if observation.shape != (OBS_DIM,):
        raise RuntimeError(f"Internal observation layout error: {observation.shape}")
    return observation


def _region_error(xy, tol: float, name: str) -> str | None:
    x, y = (float(v) for v in xy)
    if not (CUBE_X[0] - tol <= x <= CUBE_X[1] + tol and CUBE_Y[0] - tol <= y <= CUBE_Y[1] + tol):
        return f"{name} ({x:.3f}, {y:.3f}) outside the trained x {CUBE_X}, y {CUBE_Y}"
    reach = math.dist((x, y), RIGHT_LINK0_XY)
    if reach > REACH_RADIUS + tol:
        return f"{name} ({x:.3f}, {y:.3f}) is {reach:.3f} m from the right link0 axis (trained <= {REACH_RADIUS} m)"
    return None


def goal_error(goal_xy, cube_xy=None) -> str | None:
    """None when the goal x/y is a trained goal (region, reach, GOAL_MIN_DIST from the cube if given), else why not."""
    goal_xy = np.asarray(goal_xy, dtype=np.float64)
    if goal_xy.shape != (2,) or not np.all(np.isfinite(goal_xy)):
        return f"goal_xy_m must be two finite values, got {goal_xy.tolist()}"
    error = _region_error(goal_xy, 0.0, "goal")
    if error is None and cube_xy is not None:
        distance = math.dist(goal_xy, np.asarray(cube_xy, dtype=np.float64)[:2])
        if distance < GOAL_MIN_DIST:
            error = f"goal is {distance:.3f} m from the cube (trained >= {GOAL_MIN_DIST} m)"
    return error


def cube_start_error(cube_position, cube_quat, max_tilt_deg: float) -> str | None:
    """None when the cube stands inside the trained start region, else why not."""
    cube_position = np.asarray(cube_position, dtype=np.float64)
    error = _region_error(cube_position[:2], START_TOL, "cube")
    if error is not None:
        return error
    if abs(cube_position[2] - CUBE_REST_Z) > START_Z_TOL:
        return (f"cube centre z {cube_position[2]:.3f} is not a standing cube (trained {CUBE_REST_Z:.3f}; ~0.445 means "
                "the vision gives the top-face marker, not the cube centre)")
    tilt = cube_tilt_deg(cube_quat)
    if tilt > max_tilt_deg:
        return f"cube tilt {tilt:.1f} deg (> {max_tilt_deg:.0f}): stand it upright"
    return None


class GripperCommandQueue:
    """The sim's ``DelayedBinaryJointPositionAction`` on the real Franka gripper.

    Close when the gripper action is < 0 (no hysteresis, as in training). One goal at a time, never preempted: a
    request that changes while a goal is in flight waits, and only the latest one is sent after the result.
    ``busy`` is the sim's ``not settled``: a goal in flight, or a changed request not sent yet.
    """

    def __init__(self):
        self.request_close = False
        self.sent_close = False  # last command sent (the reset pose is open)
        self.in_flight = False

    @property
    def busy(self) -> bool:
        return self.in_flight or self.request_close != self.sent_close

    @property
    def released(self) -> bool:
        """The open command has finished (fingers open as far as the gripper went)."""
        return not self.sent_close and not self.in_flight

    def update(self, gripper_action: float) -> bool | None:
        """Record the policy's request; return the command to send now (True = grasp, False = open) or None."""
        self.request_close = bool(gripper_action < 0.0)
        if self.in_flight or self.request_close == self.sent_close:
            return None
        self.sent_close = self.request_close
        self.in_flight = True
        return self.sent_close

    def on_result(self) -> None:
        self.in_flight = False


def pick_place_result(cube_position, cube_quat, goal_position, tcp_position, released: bool,
                      cube_pose_fresh: bool = True) -> dict:
    """Cube vs goal for metadata.json / results.csv (and the node's per-tick success check).

    ``success`` is the sim's success stage: xy within XY_TOL and the fingertips HAND_CLEAR above the cube top (the sim
    also asks for no finger contact, which the hand above the cube top implies), on a fresh cube pose (a stale one may
    be a marker hidden by the hand that carries the cube). ``released`` (gripper open command done) is reported but
    left out: the trained policy asks to close again ~0.1 s after success, with the hand in the air above the cube
    (sim model_4000: 228/228 envs, success stage kept to the end in all of them).
    The tilt is reported but left out of ``success``: the vision reads a standing cube as 3-4 deg tilted (11-07
    cube-lift sessions, direction differs per session), close to the 5 deg limit. ``success_with_tilt`` adds
    tilt < UPRIGHT_DEG as in training.
    """
    cube = np.asarray(cube_position, dtype=np.float64)
    goal = np.asarray(goal_position, dtype=np.float64)
    error = cube - goal
    error_xy = float(np.linalg.norm(error[:2]))
    tilt = cube_tilt_deg(cube_quat)
    hand_clear = float(np.asarray(tcp_position, dtype=np.float64)[2] - (cube[2] + 0.5 * CUBE_SIZE[2]))
    success = error_xy < XY_TOL and hand_clear > HAND_CLEAR and bool(cube_pose_fresh)
    return {
        "cube_position_base": [round(float(v), 4) for v in cube],
        "goal_position_base": [round(float(v), 4) for v in goal],
        "error_xyz_m": [round(float(v), 4) for v in error],
        "error_xy_m": round(error_xy, 4),
        "tilt_deg": round(tilt, 2),
        "cube_yaw_deg": round(math.degrees(float(yaw_from_quat(canonical_cube_quat(cube_quat)))), 2),
        "hand_clear_m": round(hand_clear, 4),
        "released": bool(released),
        "cube_pose_fresh": bool(cube_pose_fresh),
        "success": bool(success),
        "success_with_tilt": bool(success and tilt < UPRIGHT_DEG),
    }
