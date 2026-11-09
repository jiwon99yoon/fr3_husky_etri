"""Deploy the Isaac Lab 3.0 cube pick-and-place actor (right arm + Franka hand with UMI fingertips) at 20 Hz.

Training tasks: ``Isaac-PickPlace-Cube-DualFR3-UMI-GripperBusy{,-Tilt,-CriticPlus}-v0`` (dual_fr3_lab 3.0.0/6.0), all
with the same 37-D actor (``pickplace_cube_core``). Contract (as the cube-lift Sim2Real node):

- arm action: q_target = clamp(q(t_k) + clip(0.1 * a, +-0.1 rad), sim target limits), an absolute target the
  controller holds until the next policy tick; Isaac-style direct PD on ``robot_name="dual"``, the left arm held at
  its joint positions when the session starts
- gripper action: close when a[7] < 0. The node sends ``franka_msgs`` Grasp/Move goals itself, one at a time and
  never preempted (``GripperCommandQueue``, the sim's delayed gripper); obs 36 is 1 from a send until its result

Goal: cube-centre x/y in base = ``goal_xy_m`` (centre, launch argument) + ``goal_offset_xy_m`` (``ros2 param set``
while IDLE, e.g. +-0.05 m around a fixed centre), z is the standing cube height. start_policy refuses a goal outside the trained region or closer than 10 cm to the cube.

Cube pose: ``geometry_msgs/PoseStamped`` of the CUBE CENTRE (``/cube_tf`` as in the cube-lift runs), any frame with
a TF to ``base``. A lost marker (the hand covers it while holding) does not stop the session: the last pose is kept,
as in training (vision dropout). Poses outside ``object_workspace_*`` are dropped as outliers.

TCP: FR3 DH FK of /dual_fr3/joint_states with the base <- right link0 mount from TF; at start_policy FK link7 is
checked against the TF link7 (``fk_tf_check_tolerance_m``).

Session end (as the cabinet book node): the node stops the controller goal once the success condition has held
``stop_on_success_s`` (the policy was trained without a success stop and keeps moving, closing the gripper again in
the air ~0.1 s after success), at the trained episode length (14 s), on stop_policy, or on a fault (stale arm/gripper
joint states, inference failure, gripper goal rejected). The arm then holds its position and the gripper stays as it
is. The final cube vs goal and the first success time go to metadata.json and one row of
``<result_root>/<log_task_name>/results.csv``.

TODO(UMI 실측): finger_joint1 with UMI tips on the 5 cm cube and the TCP offset are sim values (pickplace_cube_core).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from franka_msgs.action import Grasp, Move
from fr3_husky_msgs.action import RunPolicyControl
from fr3_husky_msgs.msg import PolicyJointCommand
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .boxlift_core import transform_pose
from .boxlift_logger import append_session_result
from .fr3_kinematics import fr3_link7_in_link0
from .liftcube_logger import LiftCubeRunLogger
from .numpy_actor import NumpyMLPActor
from .observation import DUAL_ARM_JOINT_NAMES
from .pickplace_cube_core import (
    ACTION_CLIP,
    ACTION_DIM,
    ACTION_SCALE,
    CUBE_REST_Z,
    OBS_DIM,
    REACH_RADIUS,
    READY_RIGHT_ARM,
    RIGHT_LINK0_XY,
    TABLE_TOP_Z,
    TARGET_LOWER,
    TARGET_UPPER,
    GripperCommandQueue,
    build_pickplace_observation,
    cube_start_error,
    goal_error,
    pick_place_result,
    tcp_position_base,
)

RIGHT_FINGER_NAMES = ("right_fr3_finger_joint1", "right_fr3_finger_joint2")
JOINT_NAMES = (*DUAL_ARM_JOINT_NAMES, *RIGHT_FINGER_NAMES)  # 7 left, 7 right, 2 right fingers
LEFT_ARM = slice(0, 7)
RIGHT_ARM = slice(7, 14)
FINGER1 = 14

# RunPolicyControl fields that the Isaac-style direct-PD path (isaac_relative_control=True) ignores.
# Same values as the Reach/box/lift nodes so all goals look identical to the controller.
_LEGACY_MAX_ACTUATOR_STEP_RAD = 0.001
_LEGACY_JOINT_VELOCITY_SCALE = 0.10
_LEGACY_JOINT_ACCELERATION_SCALE = 0.20
_LEGACY_RELATIVE_TARGET_REFRESH_HZ = 1000.0


class PPOPickPlaceCubePolicyNode(Node):
    def __init__(self):
        super().__init__("ppo_pickplace_cube_policy_node")
        share = Path(get_package_share_directory("fr3_husky_nn_policy"))

        self.declare_parameter(
            "model_path", str(share / "models" / "dual_fr3_pick_place_cube_gripper_busy_model4000.npz")
        )
        self.declare_parameter("joint_states_topic", "/dual_fr3/joint_states")
        self.declare_parameter("gripper_joint_states_topic", "/right_franka_gripper/joint_states")
        self.declare_parameter("object_pose_topic", "/cube_tf")
        self.declare_parameter("object_position_offset_m", [0.0, 0.0, 0.0])  # added to the vision cube centre (base)
        self.declare_parameter("object_size_m", [0.05, 0.05, 0.10])  # RViz marker only
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("right_link0_frame", "right_fr3_link0")
        self.declare_parameter("right_link7_frame", "right_fr3_link7")
        # TODO(UMI 실측): link7 -> UMI fingertip centre, the sim value (same as the cabinet FK node)
        self.declare_parameter("tcp_offset_position_m", [0.0, 0.0, 0.2844])
        self.declare_parameter("fk_tf_check_tolerance_m", 0.003)
        self.declare_parameter("command_topic", "/policy_joint_command")
        self.declare_parameter("control_action", "/fr3_policy_control")
        self.declare_parameter("gripper_action_ns", "/right_franka_gripper")
        self.declare_parameter("policy_rate_hz", 20.0)
        self.declare_parameter("episode_length_s", 14.0)  # trained episode length; the node stops the goal there
        # end the session once the success condition (pick_place_result on a fresh cube pose) has held this long;
        # 0 = run the whole episode (sim model_4000: success is kept to 14 s once reached, 228/228 envs)
        self.declare_parameter("stop_on_success_s", 0.5)
        self.declare_parameter("joint_state_timeout_s", 0.15)  # arm joints (/dual_fr3/joint_states, 1 kHz)
        # fingers: franka_gripper joint_states arrive at ~10 Hz with gaps up to ~0.15 s (2026-11-07 real runs)
        self.declare_parameter("gripper_joint_state_timeout_s", 0.5)
        self.declare_parameter("object_pose_timeout_s", 0.20)  # start_policy only; a lost marker never stops a session
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("max_policy_target_delta_rad", 0.20)
        self.declare_parameter("ready_tolerance_rad", 0.20)
        self.declare_parameter("min_open_finger_width_m", 0.07)
        # the vision reads a standing cube 3-4 deg tilted, once 15 deg (11-07); this only catches a lying cube
        self.declare_parameter("max_start_tilt_deg", 20.0)
        self.declare_parameter("control_gripper", True)
        # franka Grasp/Move goals (same values as the cube-lift node; 30 N = training grasp force)
        self.declare_parameter("grasp_width_m", 0.0)
        self.declare_parameter("grasp_speed_mps", 0.1)
        self.declare_parameter("grasp_force_n", 30.0)
        self.declare_parameter("grasp_epsilon_m", 0.08)
        self.declare_parameter("open_width_m", 0.08)
        self.declare_parameter("open_speed_mps", 0.1)
        self.declare_parameter("shadow_mode", True)
        # goal cube centre x/y (base). Trained: x 0.45-0.65, y -0.45-0.05, within 0.62 m of (0, -0.215), >= 0.10 m
        # from the cube
        self.declare_parameter("goal_xy_m", [0.50, -0.20])
        self.declare_parameter("goal_offset_xy_m", [0.0, 0.0])
        self.declare_parameter("object_workspace_min", [0.25, -0.70, 0.30])
        self.declare_parameter("object_workspace_max", [0.90, 0.30, 0.80])
        # RViz only: test region (green fill) = this x/y box within the right-arm reach; blue = trained rectangle
        self.declare_parameter("region_marker_x_m", [0.45, 0.55])
        self.declare_parameter("region_marker_y_m", [-0.45, 0.05])
        self.declare_parameter("log_root", "")
        self.declare_parameter("log_task_name", "dual_fr3_pick_place_cube")
        self.declare_parameter("result_root", "")  # "" = <repo>/experi_result

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.actor = NumpyMLPActor(self.get_parameter("model_path").value)
        if self.actor.input_dim != OBS_DIM or self.actor.output_dim != ACTION_DIM:
            raise ValueError(f"cube pick-place needs a {OBS_DIM}->{ACTION_DIM} actor (-GripperBusy tasks), "
                             f"got {self.actor.input_dim}->{self.actor.output_dim}")
        if self.actor.output_activation != "tanh":
            raise ValueError("the cube pick-place actor is tanh-squashed; export with --output-activation tanh")
        error = goal_error(self._goal_xy())
        if error is not None:
            raise ValueError(error)

        n = len(JOINT_NAMES)
        self.joint_position = np.full(n, np.nan, dtype=np.float64)
        self.joint_velocity = np.full(n, np.nan, dtype=np.float64)
        self.joint_update_ns = np.zeros(n, dtype=np.int64)
        self.object_position = np.full(3, np.nan, dtype=np.float64)
        self.object_quat = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.object_update_ns = 0
        self.object_stamp_ns = 0
        self.object_rejected = 0

        self.state = "IDLE"  # IDLE -> PENDING -> RUNNING -> (STOPPING) -> IDLE
        self.goal_handle = None
        self.end_reason = ""
        self.logger_: Optional[LiftCubeRunLogger] = None
        self.previous_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.target_position = np.zeros(3, dtype=np.float64)
        self.left_hold = np.zeros(7, dtype=np.float64)
        self.mount_position = np.zeros(3, dtype=np.float64)
        self.mount_quat = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.gripper = GripperCommandQueue()
        self.sequence = 0
        self.step_index = 0
        self.session_start_ns = 0
        self.last_tick_ns = 0
        self.gripper_sent_ns = 0
        self.clamped_steps = 0
        self.success_since_ns = 0
        self.first_success_s = float("nan")
        self.feedback_max_joint_error = float("nan")
        self.feedback_command_age = float("nan")
        self.last_status_log_ns = 0

        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self._joint_state_callback, 10)
        self.create_subscription(
            JointState, self.get_parameter("gripper_joint_states_topic").value, self._joint_state_callback, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(PoseStamped, self.get_parameter("object_pose_topic").value, self._object_pose_callback, 10)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.command_publisher = self.create_publisher(PolicyJointCommand, self.get_parameter("command_topic").value, command_qos)
        self.action_client = ActionClient(self, RunPolicyControl, self.get_parameter("control_action").value)
        ns = str(self.get_parameter("gripper_action_ns").value).rstrip("/")
        self.grasp_client = ActionClient(self, Grasp, f"{ns}/grasp")
        self.move_client = ActionClient(self, Move, f"{ns}/move")
        self.create_service(Trigger, "~/start_policy", self._start_service)
        self.create_service(Trigger, "~/stop_policy", self._stop_service)
        # RViz: goal (IDLE preview, then the latched goal), the cube as the policy sees it, trained region
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_marker_publisher = self.create_publisher(Marker, "~/goal_marker", latched)
        self.cube_marker_publisher = self.create_publisher(Marker, "~/cube_marker", 1)
        self.range_marker_publisher = self.create_publisher(MarkerArray, "~/range_markers", latched)
        self.range_marker_box = None
        self.add_on_set_parameters_callback(self._on_set_parameters)

        rate = float(self.get_parameter("policy_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("policy_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Loaded {self.actor.model_path} ({OBS_DIM}->{ACTION_DIM} tanh, source {self.actor.source_sha256[:12]}...), "
            f"mode={'SHADOW' if self.shadow_mode else 'COMMAND'}, rate={rate:.1f} Hz, "
            f"{float(self.get_parameter('episode_length_s').value):.1f} s horizon, gripper goals on {ns}, "
            f"goal xy {self._goal_xy().round(4).tolist()} (goal_xy_m + goal_offset_xy_m)"
        )

    # ------------------------------------------------------------------ goal
    def _goal_xy(self, changed=None) -> np.ndarray:
        """goal_xy_m + goal_offset_xy_m; ``changed`` overrides either while a param set is being validated."""
        changed = changed or {}
        value = lambda name: np.asarray(changed.get(name, self.get_parameter(name).value), dtype=np.float64)
        return value("goal_xy_m") + value("goal_offset_xy_m")

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        changed = {p.name: p.value for p in parameters if p.name in ("goal_xy_m", "goal_offset_xy_m")}
        if not changed:
            return SetParametersResult(successful=True)
        if self.state != "IDLE":
            return SetParametersResult(successful=False, reason=f"{', '.join(changed)} can only change while IDLE ({self.state})")
        if any(len(v) != 2 for v in changed.values()):
            return SetParametersResult(successful=False, reason="goal_xy_m / goal_offset_xy_m need 2 values")
        goal = self._goal_xy(changed)
        error = goal_error(goal)
        if error is not None:
            return SetParametersResult(successful=False, reason=error)
        self.get_logger().info(f"goal accepted: {goal.round(4).tolist()} ({', '.join(changed)} changed)")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def _vector_parameter(self, name: str, size: int = 3) -> np.ndarray:
        value = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if value.shape != (size,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain {size} finite values")
        return value

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _log_status(self, message: str) -> None:
        now_ns = self._now_ns()
        if now_ns - self.last_status_log_ns > 1_000_000_000:
            self.get_logger().warn(message)
            self.last_status_log_ns = now_ns

    def _joint_state_callback(self, msg: JointState) -> None:
        now_ns = self._now_ns()
        index = {name: i for i, name in enumerate(msg.name)}
        for out, name in enumerate(JOINT_NAMES):
            i = index.get(name)
            if i is None or i >= len(msg.position) or not np.isfinite(msg.position[i]):
                continue
            self.joint_position[out] = float(msg.position[i])
            if i < len(msg.velocity) and np.isfinite(msg.velocity[i]):
                self.joint_velocity[out] = float(msg.velocity[i])
            self.joint_update_ns[out] = now_ns

    def _object_pose_callback(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        position = np.asarray([p.x, p.y, p.z], dtype=np.float64)
        quat = np.asarray([o.x, o.y, o.z, o.w], dtype=np.float64)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-6:
            return
        quat = quat / np.linalg.norm(quat)
        frame = msg.header.frame_id or self.base_frame
        if frame != self.base_frame:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, RosTime()).transform
            except TransformException as error:
                self._log_status(f"Ignoring cube pose in '{frame}': {error}")
                return
            position, quat = transform_pose([t.translation.x, t.translation.y, t.translation.z],
                                            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], position, quat)
        position = position + self._vector_parameter("object_position_offset_m")
        lo, hi = self._vector_parameter("object_workspace_min"), self._vector_parameter("object_workspace_max")
        if not np.all((position >= lo) & (position <= hi)):
            # an outlier is treated as a lost marker: keep the last pose
            self.object_rejected += 1
            self._log_status(f"Dropping cube pose outside the workspace: {np.round(position, 3).tolist()}")
            return
        self.object_position = position
        self.object_quat = quat
        self.object_update_ns = self._now_ns()
        self.object_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _finger_width(self) -> float:
        return float(self.joint_position[FINGER1] + self.joint_position[FINGER1 + 1])

    def _joints_ready(self) -> tuple[bool, str]:
        if not np.all(np.isfinite(self.joint_position)) or not np.all(np.isfinite(self.joint_velocity[:14])):
            missing = [name for name, q in zip(JOINT_NAMES, self.joint_position) if not np.isfinite(q)]
            return False, f"waiting for joint states {missing or '(arm velocities)'}"
        age_s = (self._now_ns() - self.joint_update_ns) * 1e-9
        if np.any(age_s[:14] > float(self.get_parameter("joint_state_timeout_s").value)):
            return False, f"arm joint state is stale ({1e3 * float(np.max(age_s[:14])):.0f} ms)"
        if np.any(age_s[14:] > float(self.get_parameter("gripper_joint_state_timeout_s").value)):
            return False, f"gripper joint state is stale ({1e3 * float(np.max(age_s[14:])):.0f} ms)"
        return True, "ready"

    def _start_ready(self) -> tuple[bool, str]:
        ready, message = self._joints_ready()
        if not ready:
            return False, message
        if self.object_update_ns == 0:
            return False, "waiting for the cube pose"
        if self._now_ns() - self.object_update_ns > float(self.get_parameter("object_pose_timeout_s").value) * 1e9:
            return False, "cube pose is stale (marker not visible?)"
        error = cube_start_error(self.object_position, self.object_quat,
                                 float(self.get_parameter("max_start_tilt_deg").value))
        if error is None:
            error = goal_error(self._goal_xy(), self.object_position[:2])
        if error is not None:
            return False, error
        ready_error = float(np.max(np.abs(self.joint_position[RIGHT_ARM] - READY_RIGHT_ARM)))
        if ready_error > float(self.get_parameter("ready_tolerance_rad").value):
            return False, f"right arm is not at the training ready pose (max error {ready_error:.3f} rad)"
        width = self._finger_width()
        if width < float(self.get_parameter("min_open_finger_width_m").value):
            return False, f"open the right gripper first (width {width:.3f} m)"
        return True, "ready"

    def _lookup(self, frame: str) -> tuple[np.ndarray, np.ndarray]:
        t = self.tf_buffer.lookup_transform(self.base_frame, frame, RosTime()).transform
        return (np.asarray([t.translation.x, t.translation.y, t.translation.z], dtype=np.float64),
                np.asarray([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], dtype=np.float64))

    def _update_mount(self) -> Optional[str]:
        """base <- right link0 from TF, and FK link7 vs TF link7 with the arm still. None if OK, else the reason."""
        try:
            self.mount_position, self.mount_quat = self._lookup(str(self.get_parameter("right_link0_frame").value))
            tf_link7 = self._lookup(str(self.get_parameter("right_link7_frame").value))[0]
        except TransformException as error:
            return f"no TF for the right arm mount/link7: {error}"
        T = fr3_link7_in_link0(self.joint_position[RIGHT_ARM])
        fk_link7 = transform_pose(self.mount_position, self.mount_quat, T[:3, 3], [0.0, 0.0, 0.0, 1.0])[0]
        error_m = float(np.linalg.norm(fk_link7 - tf_link7))
        if error_m > float(self.get_parameter("fk_tf_check_tolerance_m").value):
            return f"FK link7 is {1e3 * error_m:.1f} mm from TF link7 (FK model does not match the URDF?)"
        self.get_logger().info(f"FK vs TF right link7: {1e3 * error_m:.2f} mm")
        return None

    def _tcp_position(self) -> np.ndarray:
        return tcp_position_base(self.joint_position[RIGHT_ARM], self.mount_position, self.mount_quat,
                                 self._vector_parameter("tcp_offset_position_m"))

    def _cube_pose_fresh(self) -> bool:
        return self._now_ns() - self.object_update_ns <= float(self.get_parameter("object_pose_timeout_s").value) * 1e9

    def _success_held(self, tick_ns: int) -> bool:
        """Track the success condition every tick (first_success_s); True once it has held stop_on_success_s."""
        success = pick_place_result(self.object_position, self.object_quat, self.target_position, self._tcp_position(),
                                    self.gripper.released, self._cube_pose_fresh())["success"]
        if not success:
            self.success_since_ns = 0
            return False
        if not self.success_since_ns:
            self.success_since_ns = tick_ns
            if np.isnan(self.first_success_s):
                self.first_success_s = (tick_ns - self.session_start_ns) * 1e-9
                self.get_logger().info(f"Success at {self.first_success_s:.2f} s")
        hold_s = float(self.get_parameter("stop_on_success_s").value)
        return hold_s > 0.0 and (tick_ns - self.success_since_ns) * 1e-9 >= hold_s

    # ------------------------------------------------------------------ session
    def _start_service(self, _request, response):
        response.success, response.message = self._request_start()
        if not response.success:
            self.get_logger().warn(f"start_policy refused: {response.message}")
        return response

    def _request_start(self) -> tuple[bool, str]:
        if self.state != "IDLE":
            return False, f"session already {self.state}"
        ready, message = self._start_ready()
        if not ready:
            return False, message
        error = self._update_mount()
        if error is not None:
            return False, error
        if not self.shadow_mode:
            if not self.action_client.server_is_ready():
                return False, "fr3_policy_control action server is not ready"
            if self.get_parameter("control_gripper").value and not (
                self.grasp_client.server_is_ready() and self.move_client.server_is_ready()
            ):
                return False, "franka gripper grasp/move action servers are not ready"

        self.target_position = np.append(self._goal_xy(), CUBE_REST_Z)
        self.goal_marker_publisher.publish(self._cube_marker("goal", self.target_position, (0.1, 0.9, 0.2, 0.5)))
        self.left_hold = self.joint_position[LEFT_ARM].copy()
        self.previous_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.gripper = GripperCommandQueue()
        self.sequence = 0
        self.step_index = 0
        self.gripper_sent_ns = 0
        self.clamped_steps = 0
        self.success_since_ns = 0
        self.first_success_s = float("nan")
        self.object_rejected = 0
        self.end_reason = ""
        self.session_start_ns = time.monotonic_ns()
        self.last_tick_ns = 0
        self._open_log()
        info = (f": cube {np.round(self.object_position, 3).tolist()}, goal {np.round(self.target_position, 3).tolist()} "
                "(base)")
        if self.shadow_mode:
            self.state = "RUNNING"
            return True, f"shadow session started{info} (inference and logging only, no commands)"

        goal = RunPolicyControl.Goal()
        goal.robot_name = "dual"
        goal.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        goal.max_duration_s = 0.0  # the node itself stops at the trained episode length
        goal.max_policy_target_delta_rad = float(self.get_parameter("max_policy_target_delta_rad").value)
        goal.max_actuator_step_rad = _LEGACY_MAX_ACTUATOR_STEP_RAD
        goal.joint_velocity_scale = _LEGACY_JOINT_VELOCITY_SCALE
        goal.joint_acceleration_scale = _LEGACY_JOINT_ACCELERATION_SCALE
        goal.isaac_relative_control = True
        goal.relative_target_refresh_hz = _LEGACY_RELATIVE_TARGET_REFRESH_HZ
        goal.control_gripper = False  # the dual path has no gripper; this node sends franka goals itself
        self.state = "PENDING"
        future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        return True, f"policy control goal requested{info}"

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._end_session("controller rejected the policy goal")
            return
        if self.state != "PENDING":  # stopped while the goal was in flight
            goal_handle.cancel_goal_async()
            return
        self.goal_handle = goal_handle
        self.state = "RUNNING"
        goal_handle.get_result_async().add_done_callback(self._result_callback)
        self.get_logger().info("Cube pick-place policy RUNNING. The gripper stays as it is when the session ends.")

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.feedback_max_joint_error = float(feedback.max_joint_error)
        self.feedback_command_age = float(feedback.command_age_s)

    def _result_callback(self, future) -> None:
        wrapped = future.result()
        result = None if wrapped is None else wrapped.result
        message = "no result" if result is None else f"success={result.success}, '{result.message}'"
        self.goal_handle = None
        self._end_session(self.end_reason or f"controller ended the goal: {message}")
        self.get_logger().info(f"policy control finished: {message}")

    def _stop_service(self, _request, response):
        if self.state == "IDLE":
            response.success, response.message = False, "no active session"
            return response
        self.end_reason = "operator stop"
        self._stop_controller_or_end()
        response.success, response.message = True, "stop requested"
        return response

    def _fault(self, message: str) -> None:
        self.get_logger().error(f"Cube pick-place fault: {message}")
        self.end_reason = f"fault: {message}"
        self._stop_controller_or_end()

    def _stop_controller_or_end(self) -> None:
        if self.goal_handle is not None:
            # The result callback closes the session once the controller has stopped.
            self.goal_handle.cancel_goal_async()
            self.state = "STOPPING"
        else:
            self._end_session(self.end_reason)

    def _end_session(self, reason: str) -> None:
        self.state = "IDLE"
        self.goal_handle = None
        if self.logger_ is not None:
            try:
                self._record_result(reason)
            except Exception as error:  # never lose the trace/metadata over the summary
                self.get_logger().error(f"Failed to record the session result: {error!r}")
            run_dir = self.logger_.finalize(reason)
            self.get_logger().info(f"Session ended ({reason}); {self.logger_.row_count} steps logged to {run_dir}")
            self.logger_ = None

    def _record_result(self, reason: str) -> None:
        """Final cube vs goal into metadata.json and one row of <result_root>/<task>/results.csv."""
        metadata = self.logger_.metadata
        final = pick_place_result(self.object_position, self.object_quat, self.target_position, self._tcp_position(),
                                  self.gripper.released, self._cube_pose_fresh())
        final["cube_pose_age_ms"] = round((self._now_ns() - self.object_update_ns) * 1e-6, 1)
        final["first_success_s"] = None if np.isnan(self.first_success_s) else round(self.first_success_s, 2)
        metadata["final_cube_vs_goal"] = final
        e = final["error_xyz_m"]
        self.get_logger().info(
            f"Final cube vs goal: error x {e[0]:+.3f} y {e[1]:+.3f} z {e[2]:+.3f} m (xy {final['error_xy_m']:.3f}), "
            f"tilt {final['tilt_deg']:.1f} deg, hand {final['hand_clear_m']:+.3f} m above the cube top, "
            f"released {final['released']}, success {final['success']} (with tilt {final['success_with_tilt']}), "
            f"first success {final['first_success_s']} s, cube pose age {final['cube_pose_age_ms']:.0f} ms")
        start = np.asarray(metadata["cube_start_position_base"], dtype=np.float64)
        row = {
            "session": self.logger_.run_dir.name,
            "model": Path(str(self.actor.model_path)).name,
            "mode": metadata["mode"],
            "end_reason": reason,
            "steps": self.logger_.row_count,
            "duration_s": round((time.monotonic_ns() - self.session_start_ns) * 1e-9, 2),
            **{f"start_{a}": round(float(v), 4) for a, v in zip("xyz", start)},
            **{f"goal_{a}": v for a, v in zip("xyz", final["goal_position_base"])},
            **{f"final_{a}": v for a, v in zip("xyz", final["cube_position_base"])},
            **{f"error_{a}": v for a, v in zip("xyz", e)},
            "error_xy_norm": final["error_xy_m"],
            "tilt_deg": final["tilt_deg"],
            "cube_yaw_deg": final["cube_yaw_deg"],
            "hand_clear_m": final["hand_clear_m"],
            "released": int(final["released"]),
            "success": int(final["success"]),
            "success_with_tilt": int(final["success_with_tilt"]),
            "first_success_s": final["first_success_s"],
            "finger_width_m": round(self._finger_width(), 4),
            "cube_pose_age_ms": final["cube_pose_age_ms"],
            "cube_pose_rejected": self.object_rejected,
            "clamped_steps": self.clamped_steps,
            "log_dir": str(self.logger_.run_dir),
        }
        configured = str(self.get_parameter("result_root").value).strip()
        root = Path(configured).expanduser() if configured else Path(__file__).resolve().parents[2] / "experi_result"
        try:
            path = append_session_result(root / str(self.get_parameter("log_task_name").value) / "results.csv", row)
            self.get_logger().info(f"Session result appended to {path}")
        except OSError as error:
            self.get_logger().error(f"Failed to write the session result: {error}")

    def _open_log(self) -> None:
        configured = str(self.get_parameter("log_root").value).strip()
        log_root = Path(configured).expanduser() if configured else Path(__file__).resolve().parents[2] / "data"
        names = (
            "joint_states_topic", "gripper_joint_states_topic", "object_pose_topic", "object_position_offset_m",
            "base_frame", "right_link0_frame", "tcp_offset_position_m", "gripper_action_ns", "policy_rate_hz",
            "episode_length_s", "stop_on_success_s", "joint_state_timeout_s", "gripper_joint_state_timeout_s", "object_pose_timeout_s",
            "command_timeout_s", "max_policy_target_delta_rad", "ready_tolerance_rad", "max_start_tilt_deg",
            "control_gripper", "grasp_width_m", "grasp_speed_mps", "grasp_force_n", "grasp_epsilon_m", "open_width_m",
            "open_speed_mps", "goal_xy_m", "goal_offset_xy_m", "object_workspace_min", "object_workspace_max", "shadow_mode",
        )
        try:
            self.logger_ = LiftCubeRunLogger(
                log_root,
                str(self.get_parameter("log_task_name").value),
                {
                    "model_path": str(self.actor.model_path),
                    "model_source_sha256": self.actor.source_sha256,
                    "mode": "shadow" if self.shadow_mode else "command",
                    "target_position_base": self.target_position,
                    "cube_start_position_base": self.object_position.copy(),
                    "cube_start_quat_base": self.object_quat.copy(),
                    # copy: the logger rewrites metadata at finalize, after joint_position was updated in place
                    "joint_start_position": self.joint_position.copy(),
                    "joint_names": list(JOINT_NAMES),
                    "right_link0_mount_base": {"position": self.mount_position.copy(), "quat": self.mount_quat.copy()},
                    "tcp_start_position_base": self._tcp_position(),
                    "parameters": {name: self.get_parameter(name).value for name in names},
                },
                obs_dim=OBS_DIM,
            )
            self.get_logger().info(f"Logging to {self.logger_.run_dir}")
        except (OSError, ValueError, TypeError) as error:
            self.logger_ = None
            self.get_logger().error(f"Failed to start cube pick-place logging: {error}")

    # ------------------------------------------------------------------ gripper
    def _gripper_event(self, event: str, command: str, success="", detail="") -> None:
        if self.logger_ is not None:
            self.logger_.event((time.monotonic_ns() - self.session_start_ns) * 1e-9, self._now_ns(), event, command,
                               success, detail, self._finger_width())

    def _send_gripper(self, close: bool) -> None:
        command = "grasp" if close else "open"
        self.gripper_sent_ns = time.monotonic_ns()
        if self.shadow_mode or not self.get_parameter("control_gripper").value:
            # no gripper server: the command finishes at once so the queue (and obs 36) does not stay busy
            self._gripper_event("would_send", command)
            self.gripper.on_result()
            return
        if close:
            goal = Grasp.Goal()
            goal.width = float(self.get_parameter("grasp_width_m").value)
            goal.speed = float(self.get_parameter("grasp_speed_mps").value)
            goal.force = float(self.get_parameter("grasp_force_n").value)
            eps = float(self.get_parameter("grasp_epsilon_m").value)
            goal.epsilon.inner, goal.epsilon.outer = eps, eps
            future = self.grasp_client.send_goal_async(goal)
        else:
            goal = Move.Goal()
            goal.width = float(self.get_parameter("open_width_m").value)
            goal.speed = float(self.get_parameter("open_speed_mps").value)
            future = self.move_client.send_goal_async(goal)
        self._gripper_event("send", command)
        # bind this session's queue: a result arriving after the session ended must not touch the next one
        queue = self.gripper
        future.add_done_callback(lambda f, c=command, g=queue: self._gripper_goal_response(f, c, g))

    def _gripper_goal_response(self, future, command: str, queue: GripperCommandQueue) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._gripper_event("rejected", command, success=False)
            queue.on_result()
            if queue is self.gripper and self.state in ("PENDING", "RUNNING"):
                self._fault(f"gripper {command} goal rejected")
            return
        self._gripper_event("accepted", command)
        handle.get_result_async().add_done_callback(lambda f, c=command, g=queue: self._gripper_result(f, c, g))

    def _gripper_result(self, future, command: str, queue: GripperCommandQueue) -> None:
        wrapped = future.result()
        result = None if wrapped is None else wrapped.result
        success = bool(result.success) if result is not None else False
        detail = "" if result is None else str(getattr(result, "error", ""))
        self._gripper_event("result", command, success=success, detail=detail)
        queue.on_result()

    # ------------------------------------------------------------------ RViz
    def _cube_marker(self, ns: str, position, rgba, quat=(0.0, 0.0, 0.0, 1.0)) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = ns, 0, Marker.CUBE, Marker.ADD
        p, o = marker.pose.position, marker.pose.orientation
        p.x, p.y, p.z = (float(v) for v in position)
        o.x, o.y, o.z, o.w = (float(v) for v in quat)
        marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in self.get_parameter("object_size_m").value)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _range_markers(self) -> MarkerArray:
        """Test region on the table (green fill): region_marker_x/y box within the right-arm reach radius."""
        cx, cy = RIGHT_LINK0_XY
        (x0, x1), (y0, y1) = self.range_marker_box
        z = TABLE_TOP_Z + 0.002
        xs, ys = np.meshgrid(np.linspace(x0, x1, 41), np.linspace(y0, y1, 101), indexing="ij")
        inside = np.hypot(xs - cx, ys - cy) <= REACH_RADIUS
        clear = Marker()
        clear.action = Marker.DELETEALL  # drops the old trained-rectangle line from a latched earlier run
        region = Marker()
        region.header.frame_id = self.base_frame
        region.ns, region.id, region.type, region.action = "reachable_region", 0, Marker.CUBE_LIST, Marker.ADD
        region.pose.orientation.w = 1.0
        region.scale.x, region.scale.y, region.scale.z = (x1 - x0) / 40, (y1 - y0) / 100, 0.001
        region.color.r, region.color.g, region.color.b, region.color.a = (0.2, 0.9, 0.3, 0.25)
        region.points = [Point(x=float(x), y=float(y), z=z) for x, y in zip(xs[inside], ys[inside])]
        return MarkerArray(markers=[clear, region])

    def _publish_markers(self) -> None:
        box = (tuple(self.get_parameter("region_marker_x_m").value), tuple(self.get_parameter("region_marker_y_m").value))
        if box != self.range_marker_box:  # first tick and after `ros2 param set ... region_marker_*`
            self.range_marker_box = box
            self.range_marker_publisher.publish(self._range_markers())
        if self.object_update_ns == 0:
            return
        self.cube_marker_publisher.publish(
            self._cube_marker("policy_cube", self.object_position, (1.0, 0.55, 0.0, 0.6), self.object_quat))
        if self.state != "IDLE":
            return
        # Preview: the goal start_policy would latch now (red: start_policy would refuse it)
        goal = np.append(self._goal_xy(), CUBE_REST_Z)
        ok = goal_error(goal[:2], self.object_position[:2]) is None
        self.goal_marker_publisher.publish(self._cube_marker("goal", goal, (0.1, 0.9, 0.2, 0.35) if ok else (0.9, 0.1, 0.1, 0.35)))

    # ------------------------------------------------------------------ policy tick
    def _tick(self) -> None:
        self._publish_markers()
        if self.state != "RUNNING":
            if self.state == "IDLE":
                ready, message = self._start_ready()
                if not ready:
                    self._log_status(f"idle: {message}")
            return
        ready, message = self._joints_ready()
        if not ready:
            self._fault(message)
            return
        tick_ns = time.monotonic_ns()
        if self._success_held(tick_ns):
            self.end_reason = f"success held {float(self.get_parameter('stop_on_success_s').value):.2f} s"
            self._stop_controller_or_end()
            return
        if (tick_ns - self.session_start_ns) * 1e-9 > float(self.get_parameter("episode_length_s").value):
            self.end_reason = "trained episode length reached"
            self._stop_controller_or_end()
            return

        now_ns = self._now_ns()
        q = self.joint_position.copy()
        try:
            observation = build_pickplace_observation(
                q[RIGHT_ARM], q[FINGER1], self.joint_velocity[RIGHT_ARM], self.object_position, self.object_quat,
                self._tcp_position(), self.target_position, self.previous_action, self.gripper.busy,
            )
            action = np.asarray(self.actor(observation), dtype=np.float64)
        except (ValueError, FloatingPointError) as error:
            self._fault(f"inference failed: {error}")
            return
        delta = np.clip(ACTION_SCALE * action[:7], -ACTION_CLIP, ACTION_CLIP)
        unclamped = q[RIGHT_ARM] + delta
        target = np.clip(unclamped, TARGET_LOWER, TARGET_UPPER)  # as the sim's target_limits
        if not np.array_equal(target, unclamped):
            self.clamped_steps += 1
        inference_ms = (time.monotonic_ns() - tick_ns) * 1e-6
        if not np.all(np.isfinite(target)):
            self._fault(f"non-finite right-arm target: {target.tolist()}")
            return

        if not self.shadow_mode:
            self.sequence += 1
            command = PolicyJointCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self.base_frame
            command.sequence = self.sequence
            command.robot_name = "dual"
            # policy_step as trained: absolute targets the controller holds until the next command (left arm held)
            command.target_positions = np.concatenate((self.left_hold, target)).tolist()
            command.relative_position_offsets = False
            command.gripper_action = 0.0
            self.command_publisher.publish(command)
        send = self.gripper.update(float(action[7]))
        if send is not None:
            self._send_gripper(send)
        self.previous_action = action.astype(np.float32)

        if self.logger_ is not None:
            try:
                self.logger_.append(
                    {
                        "elapsed_s": (tick_ns - self.session_start_ns) * 1e-9,
                        "ros_time_ns": now_ns,
                        "step": self.step_index,
                        "tick_period_ms": (tick_ns - self.last_tick_ns) * 1e-6 if self.last_tick_ns else float("nan"),
                        "joint_state_age_ms": (now_ns - int(self.joint_update_ns.min())) * 1e-6,
                        "object_pose_age_ms": (now_ns - self.object_update_ns) * 1e-6,
                        "object_stamp_age_ms": (now_ns - self.object_stamp_ns) * 1e-6 if self.object_stamp_ns else float("nan"),
                        "inference_ms": inference_ms,
                        "max_abs_delta_rad": float(np.max(np.abs(target - q[RIGHT_ARM]))),
                        "feedback_max_joint_error_rad": self.feedback_max_joint_error,
                        "feedback_command_age_s": self.feedback_command_age,
                    },
                    {
                        "gripper_raw_action": float(action[7]),
                        "gripper_closed_cmd": float(self.gripper.request_close),
                        "finger_width_m": self._finger_width(),
                        "gripper_cmd_age_s": (tick_ns - self.gripper_sent_ns) * 1e-9 if self.gripper_sent_ns else float("nan"),
                    },
                    observation, action, target, self.object_quat,
                )
            except (OSError, ValueError) as error:
                self._log_status(f"Skipping log row: {error}")
        self.last_tick_ns = tick_ns
        self.step_index += 1


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = PPOPickPlaceCubePolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            try:
                node.goal_handle.cancel_goal_async()
            except Exception:  # context already shut down by Ctrl-C; the controller stops on command timeout
                pass
        if node.logger_ is not None:
            node._end_session(node.end_reason or "node shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
