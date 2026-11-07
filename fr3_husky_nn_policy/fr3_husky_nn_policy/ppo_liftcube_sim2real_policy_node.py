"""Deploy the Isaac Lab 3.0 cube-lift Sim2Real-100Hz actor (right arm + Franka gripper) at 20 Hz.

Training task: ``Isaac-Lift-Cube-DualFR3-Sim2Real-100Hz-v0`` (dual_fr3_lab 3.0.0/6.0), standard Franka fingertips.
Contract (same as box/cabinet Sim2Real nodes):

- observation 36-D, identical layout to dual_fr3_lift_v3 (``build_liftcube_observation``)
- arm action: q_target = q(t_k) + clip(0.1 * a, +-0.1 rad), an absolute target the controller holds until the next
  policy tick; Isaac-style direct PD (``isaac_relative_control``) on ``robot_name="dual"``, the left arm held at its
  joint positions when the session starts
- gripper action: close when a[7] < 0 (with hysteresis). The controller's dual path does not drive grippers, so this
  node sends ``franka_msgs`` Grasp/Move goals itself and logs every goal and answer (gripper delay measurement).

Logs go to ``<log_root>/<log_task_name>/<stamp>/`` (``liftcube_logger``).
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
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .liftcube_logger import LiftCubeRunLogger
from .numpy_actor import NumpyMLPActor
from .observation import DEFAULT_RIGHT_JOINT_POSITION, DUAL_ARM_JOINT_NAMES, build_liftcube_observation

RIGHT_FINGER_NAMES = ("right_fr3_finger_joint1", "right_fr3_finger_joint2")
JOINT_NAMES = (*DUAL_ARM_JOINT_NAMES, *RIGHT_FINGER_NAMES)  # 7 left, 7 right, 2 right fingers
RIGHT_ARM = slice(7, 14)
RIGHT_WITH_FINGERS = list(range(7, 16))

LOWER_LIMITS = np.asarray([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508], dtype=np.float64)
UPPER_LIMITS = np.asarray([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508], dtype=np.float64)

ACTION_SCALE = 0.1  # rad per unit action (ReachRelativeJointPositionAction scale)
ACTION_CLIP = 0.1  # rad, |q_target - q(t_k)| bound after the scale

# RunPolicyControl fields that the Isaac-style direct-PD path (isaac_relative_control=True) ignores.
# Same values as the Reach/box nodes so all goals look identical to the controller.
_LEGACY_MAX_ACTUATOR_STEP_RAD = 0.001
_LEGACY_JOINT_VELOCITY_SCALE = 0.10
_LEGACY_JOINT_ACCELERATION_SCALE = 0.20
_LEGACY_RELATIVE_TARGET_REFRESH_HZ = 1000.0


class PPOLiftCubeSim2RealPolicyNode(Node):
    def __init__(self):
        super().__init__("ppo_liftcube_sim2real_policy_node")
        share = Path(get_package_share_directory("fr3_husky_nn_policy"))

        self.declare_parameter("model_path", str(share / "models" / "dual_fr3_lift_sim2real100hz_model4999.npz"))
        # arm joints from ros2_control (as the box/cabinet nodes); /joint_states is the 30 Hz joint_state_publisher merge
        self.declare_parameter("joint_states_topic", "/dual_fr3/joint_states")
        self.declare_parameter("gripper_joint_states_topic", "/right_franka_gripper/joint_states")
        self.declare_parameter("object_pose_topic", "/object_pose")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("command_topic", "/policy_joint_command")
        self.declare_parameter("control_action", "/fr3_policy_control")
        self.declare_parameter("gripper_action_ns", "/right_franka_gripper")
        self.declare_parameter("policy_rate_hz", 20.0)
        self.declare_parameter("joint_state_timeout_s", 0.15)
        self.declare_parameter("object_pose_timeout_s", 0.20)
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("max_duration_s", 8.0)  # trained episode length
        self.declare_parameter("max_policy_target_delta_rad", 0.20)
        self.declare_parameter("joint_limit_margin_rad", 0.02)
        self.declare_parameter("ready_tolerance_rad", 0.20)
        self.declare_parameter("min_open_finger_width_m", 0.07)
        self.declare_parameter("control_gripper", True)
        self.declare_parameter("gripper_hysteresis", 0.10)
        # franka Grasp/Move goals (same values as the controller's GripperGrasp/GripperOpen defaults)
        self.declare_parameter("grasp_width_m", 0.0)
        self.declare_parameter("grasp_speed_mps", 0.1)
        self.declare_parameter("grasp_force_n", 30.0)
        self.declare_parameter("grasp_epsilon_m", 0.08)
        self.declare_parameter("open_width_m", 0.08)
        self.declare_parameter("open_speed_mps", 0.1)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("auto_start", False)
        # training ranges (base frame): goal x 0.35-0.55, y -0.45--0.15, z 0.45-0.60; cube centre x 0.37-0.53,
        # y -0.37--0.13, z table + 0.05 (0.395)
        self.declare_parameter("target_position", [0.45, -0.30, 0.50])
        self.declare_parameter("target_min", [0.35, -0.45, 0.45])
        self.declare_parameter("target_max", [0.55, -0.15, 0.60])
        self.declare_parameter("object_start_min", [0.36, -0.38, 0.365])
        self.declare_parameter("object_start_max", [0.54, -0.12, 0.425])
        self.declare_parameter("object_workspace_min", [0.20, -0.60, 0.25])
        self.declare_parameter("object_workspace_max", [0.70, 0.10, 0.80])
        self.declare_parameter("log_root", "")
        self.declare_parameter("log_task_name", "dual_fr3_liftcube_sim2real100hz")

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.actor = NumpyMLPActor(self.get_parameter("model_path").value)
        if self.actor.input_dim != 36 or self.actor.output_dim != 8:
            raise ValueError(f"cube lift needs a 36->8 actor, got {self.actor.input_dim}->{self.actor.output_dim}")
        if self.actor.output_activation != "tanh":
            raise ValueError("the Sim2Real cube-lift actor is tanh-squashed; export with --output-activation tanh")

        n = len(JOINT_NAMES)
        self.joint_position = np.full(n, np.nan, dtype=np.float64)
        self.joint_velocity = np.full(n, np.nan, dtype=np.float64)
        self.joint_update_ns = np.zeros(n, dtype=np.int64)
        self.object_position = np.full(3, np.nan, dtype=np.float64)
        self.object_quat = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.object_update_ns = 0
        self.object_stamp_ns = 0

        self.state = "IDLE"  # IDLE -> PENDING -> RUNNING -> (STOPPING) -> IDLE
        self.goal_handle = None
        self.end_reason = ""
        self.logger_: Optional[LiftCubeRunLogger] = None
        self.previous_action = np.zeros(8, dtype=np.float32)
        self.target_position = np.zeros(3, dtype=np.float64)
        self.left_hold = np.zeros(7, dtype=np.float64)
        self.sequence = 0
        self.step_index = 0
        self.session_start_ns = 0
        self.last_tick_ns = 0
        self.gripper_closed = False
        self.gripper_sent_ns = 0
        self.feedback_max_joint_error = float("nan")
        self.feedback_command_age = float("nan")
        self.auto_start_requested = False
        self.last_status_log_ns = 0

        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self._joint_state_callback, 10)
        self.create_subscription(
            JointState, self.get_parameter("gripper_joint_states_topic").value, self._joint_state_callback, 10
        )
        self.create_subscription(PoseStamped, self.get_parameter("object_pose_topic").value, self._object_pose_callback, 10)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.command_publisher = self.create_publisher(PolicyJointCommand, self.get_parameter("command_topic").value, command_qos)
        self.action_client = ActionClient(self, RunPolicyControl, self.get_parameter("control_action").value)
        ns = str(self.get_parameter("gripper_action_ns").value).rstrip("/")
        self.grasp_client = ActionClient(self, Grasp, f"{ns}/grasp")
        self.move_client = ActionClient(self, Move, f"{ns}/move")
        self.create_service(Trigger, "~/start_policy", self._start_service)
        self.create_service(Trigger, "~/stop_policy", self._stop_service)

        rate = float(self.get_parameter("policy_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("policy_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Loaded {self.actor.model_path} (36->8 tanh, source {self.actor.source_sha256[:12]}...), "
            f"mode={'SHADOW' if self.shadow_mode else 'COMMAND'}, rate={rate:.1f} Hz, gripper goals on {ns}"
        )

    # ------------------------------------------------------------------ inputs
    def _vector_parameter(self, name: str) -> np.ndarray:
        value = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain three finite values")
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
            previous, previous_ns = self.joint_position[out], self.joint_update_ns[out]
            self.joint_position[out] = float(msg.position[i])
            # fingers: always differentiate the width (franka_gripper does not report a finger velocity)
            if out < 14 and i < len(msg.velocity) and np.isfinite(msg.velocity[i]):
                self.joint_velocity[out] = float(msg.velocity[i])
            elif np.isfinite(previous) and previous_ns > 0 and (now_ns - previous_ns) > 100_000:
                self.joint_velocity[out] = (self.joint_position[out] - previous) / ((now_ns - previous_ns) * 1e-9)
            self.joint_update_ns[out] = now_ns

    def _object_pose_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.base_frame:
            self._log_status(f"Ignoring object pose frame '{msg.header.frame_id}', expected '{self.base_frame}'")
            return
        p, o = msg.pose.position, msg.pose.orientation
        position = np.asarray([p.x, p.y, p.z], dtype=np.float64)
        if np.all(np.isfinite(position)):
            self.object_position = position
            self.object_quat = np.asarray([o.x, o.y, o.z, o.w], dtype=np.float64)
            self.object_update_ns = self._now_ns()
            self.object_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _finger_width(self) -> float:
        return float(self.joint_position[14] + self.joint_position[15])

    def _inputs_ready(self, at_start: bool) -> tuple[bool, str]:
        now_ns = self._now_ns()
        if not np.all(np.isfinite(self.joint_position)) or not np.all(np.isfinite(self.joint_velocity)):
            missing = [name for name, q in zip(JOINT_NAMES, self.joint_position) if not np.isfinite(q)]
            return False, f"waiting for joint states {missing or '(velocities)'}"
        if np.any(now_ns - self.joint_update_ns > float(self.get_parameter("joint_state_timeout_s").value) * 1e9):
            return False, "joint state is stale"
        if self.object_update_ns == 0:
            return False, "waiting for the cube pose"
        if now_ns - self.object_update_ns > float(self.get_parameter("object_pose_timeout_s").value) * 1e9:
            return False, "cube pose is stale (marker lost?)"
        lo, hi = self._vector_parameter("object_workspace_min"), self._vector_parameter("object_workspace_max")
        if not np.all((self.object_position >= lo) & (self.object_position <= hi)):
            return False, f"cube pose outside the workspace: {np.round(self.object_position, 3).tolist()}"
        if not at_start:
            return True, "ready"
        lo, hi = self._vector_parameter("object_start_min"), self._vector_parameter("object_start_max")
        if not np.all((self.object_position >= lo) & (self.object_position <= hi)):
            return False, (f"cube {np.round(self.object_position, 3).tolist()} outside the training start region "
                           f"{lo.tolist()} - {hi.tolist()}")
        target = self._vector_parameter("target_position")
        lo, hi = self._vector_parameter("target_min"), self._vector_parameter("target_max")
        if not np.all((target >= lo) & (target <= hi)):
            return False, f"target_position {target.tolist()} outside the training range {lo.tolist()} - {hi.tolist()}"
        ready_error = float(np.max(np.abs(self.joint_position[RIGHT_ARM] - DEFAULT_RIGHT_JOINT_POSITION[:7])))
        if ready_error > float(self.get_parameter("ready_tolerance_rad").value):
            return False, f"right arm is not at the training ready pose (max error {ready_error:.3f} rad)"
        width = self._finger_width()
        if width < float(self.get_parameter("min_open_finger_width_m").value):
            return False, f"open the right gripper first (width {width:.3f} m)"
        return True, "ready"

    # ------------------------------------------------------------------ session
    def _start_service(self, _request, response):
        response.success, response.message = self._request_start()
        if not response.success:
            self.get_logger().warn(f"start_policy refused: {response.message}")
        return response

    def _request_start(self) -> tuple[bool, str]:
        if self.state != "IDLE":
            return False, f"session already {self.state}"
        ready, message = self._inputs_ready(at_start=True)
        if not ready:
            return False, message
        if not self.shadow_mode:
            if not self.action_client.server_is_ready():
                return False, "fr3_policy_control action server is not ready"
            if self.get_parameter("control_gripper").value and not (
                self.grasp_client.server_is_ready() and self.move_client.server_is_ready()
            ):
                return False, "franka gripper grasp/move action servers are not ready"

        self.target_position = self._vector_parameter("target_position")
        self.left_hold = self.joint_position[:7].copy()
        self.previous_action = np.zeros(8, dtype=np.float32)
        self.sequence = 0
        self.step_index = 0
        self.gripper_closed = False
        self.gripper_sent_ns = 0
        self.end_reason = ""
        self.session_start_ns = time.monotonic_ns()
        self.last_tick_ns = 0
        self._open_log()
        info = (f": cube {np.round(self.object_position, 3).tolist()}, target {self.target_position.tolist()} (base)")
        if self.shadow_mode:
            self.state = "RUNNING"
            return True, f"shadow session started{info} (inference and logging only, no commands)"

        goal = RunPolicyControl.Goal()
        goal.robot_name = "dual"
        goal.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        goal.max_duration_s = float(self.get_parameter("max_duration_s").value)
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
        self.get_logger().info("Cube-lift policy RUNNING. The gripper stays as it is when the session ends.")

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
        self.get_logger().error(f"Cube-lift fault: {message}")
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
            run_dir = self.logger_.finalize(reason)
            self.get_logger().info(f"Session ended ({reason}); {self.logger_.row_count} steps logged to {run_dir}")
            self.logger_ = None

    def _open_log(self) -> None:
        configured = str(self.get_parameter("log_root").value).strip()
        log_root = Path(configured).expanduser() if configured else Path(__file__).resolve().parents[2] / "data"
        names = (
            "joint_states_topic", "gripper_joint_states_topic", "object_pose_topic", "base_frame", "gripper_action_ns", "policy_rate_hz",
            "joint_state_timeout_s", "object_pose_timeout_s", "command_timeout_s", "max_duration_s",
            "max_policy_target_delta_rad", "ready_tolerance_rad", "control_gripper", "gripper_hysteresis",
            "grasp_width_m", "grasp_speed_mps", "grasp_force_n", "grasp_epsilon_m", "open_width_m", "open_speed_mps",
            "target_min", "target_max", "object_start_min", "object_start_max", "shadow_mode",
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
                    "parameters": {name: self.get_parameter(name).value for name in names},
                },
            )
            self.get_logger().info(f"Logging to {self.logger_.run_dir}")
        except (OSError, ValueError, TypeError) as error:
            self.logger_ = None
            self.get_logger().error(f"Failed to start cube-lift logging: {error}")

    # ------------------------------------------------------------------ gripper
    def _gripper_event(self, event: str, command: str, success="", detail="") -> None:
        if self.logger_ is not None:
            self.logger_.event((time.monotonic_ns() - self.session_start_ns) * 1e-9, self._now_ns(), event, command,
                               success, detail, self._finger_width())

    def _send_gripper(self, close: bool) -> None:
        command = "grasp" if close else "open"
        self.gripper_sent_ns = time.monotonic_ns()
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
        future.add_done_callback(lambda f, c=command: self._gripper_goal_response(f, c))

    def _gripper_goal_response(self, future, command: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._gripper_event("rejected", command, success=False)
            self.get_logger().error(f"gripper {command} goal rejected")
            return
        self._gripper_event("accepted", command)
        handle.get_result_async().add_done_callback(lambda f, c=command: self._gripper_result(f, c))

    def _gripper_result(self, future, command: str) -> None:
        wrapped = future.result()
        result = None if wrapped is None else wrapped.result
        success = bool(result.success) if result is not None else False
        detail = "" if result is None else str(getattr(result, "error", ""))
        self._gripper_event("result", command, success=success, detail=detail)

    def _update_gripper(self, raw_action: float) -> None:
        threshold = float(self.get_parameter("gripper_hysteresis").value)
        closed = self.gripper_closed
        if raw_action < -threshold:
            closed = True
        elif raw_action > threshold:
            closed = False
        if closed == self.gripper_closed:
            return
        self.gripper_closed = closed
        if self.shadow_mode or not self.get_parameter("control_gripper").value:
            self._gripper_event("would_send", "grasp" if closed else "open")
            return
        self._send_gripper(closed)

    # ------------------------------------------------------------------ policy tick
    def _tick(self) -> None:
        if self.state != "RUNNING":
            if self.state == "IDLE":
                if (self.get_parameter("auto_start").value and not self.shadow_mode and not self.auto_start_requested
                        and self._inputs_ready(at_start=True)[0] and self.action_client.server_is_ready()):
                    self.auto_start_requested = True
                    success, message = self._request_start()
                    if not success:
                        self._log_status(message)
                else:
                    ready, message = self._inputs_ready(at_start=True)
                    if not ready:
                        self._log_status(f"idle: {message}")
            return
        ready, message = self._inputs_ready(at_start=False)
        if not ready:
            self._fault(message)
            return

        tick_ns = time.monotonic_ns()
        now_ns = self._now_ns()
        q = self.joint_position.copy()
        try:
            observation = build_liftcube_observation(
                q[RIGHT_WITH_FINGERS], self.joint_velocity[RIGHT_WITH_FINGERS], self.object_position,
                self.target_position, self.previous_action,
            )
            action = np.asarray(self.actor(observation), dtype=np.float64)
        except (ValueError, FloatingPointError) as error:
            self._fault(f"inference failed: {error}")
            return
        delta = np.clip(ACTION_SCALE * action[:7], -ACTION_CLIP, ACTION_CLIP)
        target = q[RIGHT_ARM] + delta
        inference_ms = (time.monotonic_ns() - tick_ns) * 1e-6
        margin = float(self.get_parameter("joint_limit_margin_rad").value)
        if not np.all(np.isfinite(target)) or not np.all((target >= LOWER_LIMITS + margin) & (target <= UPPER_LIMITS - margin)):
            self._fault(f"right-arm target outside the joint limits: {np.round(target, 3).tolist()}")
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
        self._update_gripper(float(action[7]))
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
                        "max_abs_delta_rad": float(np.max(np.abs(delta))),
                        "feedback_max_joint_error_rad": self.feedback_max_joint_error,
                        "feedback_command_age_s": self.feedback_command_age,
                    },
                    {
                        "gripper_raw_action": float(action[7]),
                        "gripper_closed_cmd": float(self.gripper_closed),
                        "finger_width_m": self._finger_width(),
                        "gripper_cmd_age_s": (tick_ns - self.gripper_sent_ns) * 1e-9 if self.gripper_sent_ns else float("nan"),
                    },
                    observation, action, target, self.object_quat,
                )
            except (OSError, ValueError) as error:
                self._log_status(f"Skipping log row: {error}")
        self.last_tick_ns = tick_ns
        self.step_index += 1
        # command mode: the controller ends the goal at max_duration_s; shadow mode has no controller goal
        if self.shadow_mode and (tick_ns - self.session_start_ns) * 1e-9 >= float(self.get_parameter("max_duration_s").value):
            self._end_session("shadow session reached max_duration_s")


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = PPOLiftCubeSim2RealPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        if node.logger_ is not None:
            node.logger_.finalize("node shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
