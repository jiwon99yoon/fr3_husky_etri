"""Deploy the dual_fr3_lab plate box-lift policy (56 -> 14, policy_step, 20 Hz) on the dual FR3.

Session flow (operator driven, no time limit in v1)::

    IDLE --~/start_policy--> PENDING --controller goal accepted--> RUNNING
      ^                       (shadow_mode: straight to RUNNING, nothing is published)  |
      +---------- ~/stop_policy, fault, or controller ends the goal --------------------+

Each tick at the checkpoint's policy rate (manifest ``policy_dt``)::

    joint state + box pose in `base` -> boxlift_core.build_observation -> actor
      -> boxlift_core.process_action -> absolute q_target (held by the controller until the next tick)

All task constants (box size, default pose, limits, scale/clip, goal and start ranges, rate) come from
the manifest inside the NPZ (``dual_fr3_lab/scripts/tools/export_boxlift_actor.py``); the node refuses
a checkpoint trained for another box.
start_policy is refused unless the arms and the box are inside the start distribution the checkpoint
trained on: the ready pose for ``start.mode=ready_pose``, or one row of the pre-grasp table embedded in
the NPZ for ``start.mode=pregrasp_table`` (move the arms there with MoveIt first, then place the box).

Goal: start_policy latches reference x/y + ``goal_xy_offset_m`` and the lift height ``goal_z_m``. The reference
(``goal_xy_reference``) is the box start x/y (default, training's rule) or ``range_center``, the center of the
trained absolute goal range (a fixed point, independent of where the box is). All three can be
changed between sessions with ``ros2 param set`` (refused while a session is PENDING/RUNNING or outside
training). A checkpoint trained with an absolute goal x/y range (manifest ``goal.xy_range_base``) refuses
start_policy when box start + offset leaves that range; the IDLE goal preview marker is red then.
RViz also gets the box as the policy sees it (``~/box_marker``, orange) and the trained box start centers
(``~/start_range_markers``, latched).

WARNING: stopping for any reason (stop service, fault, Ctrl-C, controller abort) ends the controller
goal. The controller then holds the measured joint positions, which releases the plate squeeze, so a
lifted box WILL DROP. A hold/lower mode is not implemented yet.

Box pose input (placeholder until the teammate's vision node is integrated): ``geometry_msgs/PoseStamped``
of the BOX CENTER (not the marker), x along the 410 mm edge, y along the 310 mm squeezed edge, z up.
Any frame with a TF to ``base`` is accepted. The 180-degree yaw ambiguity and quaternion sign are
handled in boxlift_core.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from fr3_husky_msgs.action import RunPolicyControl
from fr3_husky_msgs.msg import PolicyJointCommand
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .boxlift_core import (
    GOAL_QUAT_XYZW,
    box_goal_error,
    START_PREGRASP_TABLE,
    canonicalize_box_quat,
    check_goal_settings,
    goal_xy_from_settings,
    check_start_conditions,
    build_observation,
    load_boxlift_policy,
    make_goal,
    normalize_quat,
    process_action,
    transform_pose,
    yaw_from_quat,
)
from .boxlift_logger import BoxLiftRunLogger, append_session_result
from .observation import DUAL_ARM_JOINT_NAMES

# RunPolicyControl fields that the Isaac-style direct-PD path (isaac_relative_control=True) ignores.
# Same values as the Reach node so both goals look identical to the controller.
_LEGACY_MAX_ACTUATOR_STEP_RAD = 0.001
_LEGACY_JOINT_VELOCITY_SCALE = 0.10
_LEGACY_JOINT_ACCELERATION_SCALE = 0.20
_LEGACY_RELATIVE_TARGET_REFRESH_HZ = 1000.0


class PPOBoxLiftPolicyNode(Node):
    def __init__(self):
        super().__init__("ppo_boxlift_policy_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("box_size_m", [0.410, 0.310, 0.280])
        self.declare_parameter("box_size_tolerance_m", 0.005)
        self.declare_parameter("joint_states_topic", "/dual_fr3/joint_states")
        self.declare_parameter("box_pose_topic", "/box_pose")
        # Added to the box position after it is expressed in base (orientation untouched), e.g. to remove
        # a constant vision height offset so a resting box reads the trained rest height.
        self.declare_parameter("box_position_offset_m", [0.0, 0.0, 0.0])
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("command_topic", "/policy_joint_command")
        self.declare_parameter("control_action", "/fr3_policy_control")
        self.declare_parameter("goal_z_m", 0.15)
        self.declare_parameter("goal_xy_offset_m", [0.0, 0.0])
        self.declare_parameter("goal_xy_reference", "box_start")  # or range_center
        self.declare_parameter("joint_state_timeout_s", 0.15)
        self.declare_parameter("box_pose_timeout_s", 0.30)
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("max_duration_s", 0.0)
        # See config/dual_fr3_boxlift.yaml for why this is 0.2 and not the Reach default 0.11.
        self.declare_parameter("max_policy_target_delta_rad", 0.20)
        self.declare_parameter("ready_tolerance_rad", 0.20)
        self.declare_parameter("start_position_margin_m", 0.0)
        self.declare_parameter("start_yaw_margin_rad", 0.0)
        self.declare_parameter("rest_z_tolerance_m", 0.03)
        self.declare_parameter("pregrasp_joint_margin_rad", 0.02)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("log_root", "")
        self.declare_parameter("log_task_name", "dual_fr3_boxlift")
        # per-session summary CSV (final box vs goal); "" = <repo>/experi_result
        self.declare_parameter("result_root", "")

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            raise ValueError("model_path is required (an NPZ from export_boxlift_actor.py)")
        self.box_size = np.asarray(self.get_parameter("box_size_m").value, dtype=np.float64)
        self.actor, self.manifest = load_boxlift_policy(
            model_path, self.box_size, float(self.get_parameter("box_size_tolerance_m").value)
        )
        # fail at startup if z or the x/y offset is outside training; later changes are checked in _on_set_parameters
        goal_z, goal_xy_offset, goal_reference = self._goal_settings()
        check_goal_settings(goal_z, goal_xy_offset, self.manifest, goal_reference)
        self.max_policy_target_delta = float(self.get_parameter("max_policy_target_delta_rad").value)
        if not (float(self.manifest.action_clip[1]) < self.max_policy_target_delta <= 1.0):
            raise ValueError(
                "max_policy_target_delta_rad must exceed the trained action clip "
                f"{self.manifest.action_clip[1]} and be <= 1.0 (controller limit)"
            )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)

        self.joint_position = np.full(14, np.nan)
        self.joint_velocity = np.full(14, np.nan)
        self.joint_update_ns = np.zeros(14, dtype=np.int64)
        self.box_position_b: Optional[np.ndarray] = None
        self.box_position_offset = np.asarray(self.get_parameter("box_position_offset_m").value, dtype=np.float64)
        if self.box_position_offset.shape != (3,) or not np.all(np.isfinite(self.box_position_offset)):
            raise ValueError(f"box_position_offset_m must be 3 finite values, got {self.box_position_offset!r}")
        self.box_quat_b: Optional[np.ndarray] = None
        self.box_update_ns = 0
        self.box_stamp_ns = 0

        self.state = "IDLE"
        self.goal_handle = None
        self.goal = None
        self.start_table_row: Optional[int] = None
        self.previous_action = np.zeros(14, dtype=np.float64)
        self.step_index = 0
        self.sequence = 0
        self.session_start_ns = 0
        self.last_tick_ns = 0
        self.horizon_warned = False
        self.final_box_pose = None
        self.result_recorded = False
        self.feedback_max_joint_error = float("nan")
        self.feedback_command_age = float("nan")
        self.end_reason = ""
        self.logger_: Optional[BoxLiftRunLogger] = None
        self.last_status_log_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._joint_state_callback, 10
        )
        # Best-effort subscription matches both reliable and best-effort publishers.
        self.create_subscription(
            PoseStamped,
            self.get_parameter("box_pose_topic").value,
            self._box_pose_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.command_publisher = self.create_publisher(
            PolicyJointCommand,
            self.get_parameter("command_topic").value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        # RViz: goal box (latched, set at start_policy) and the box as the policy sees it (offset applied).
        self.goal_marker_publisher = self.create_publisher(
            Marker, "~/goal_marker",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.box_marker_publisher = self.create_publisher(Marker, "~/box_marker", 1)
        self.start_range_publisher = self.create_publisher(
            MarkerArray, "~/start_range_markers",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.start_range_publisher.publish(self._start_range_markers())
        self.action_client = ActionClient(self, RunPolicyControl, self.get_parameter("control_action").value)
        self.create_service(Trigger, "~/start_policy", self._start_service)
        self.create_service(Trigger, "~/stop_policy", self._stop_service)
        self.timer = self.create_timer(self.manifest.policy_dt, self._tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        offset_range, xy_range = self.manifest.goal_xy_offset_range, self.manifest.goal_xy_range
        trained_xy = (f"trained offset x {offset_range[0].tolist()}, y {offset_range[1].tolist()}" if xy_range is None
                      else f"trained absolute goal x {xy_range[0].tolist()}, y {xy_range[1].tolist()}")
        provenance = self.manifest.raw["provenance"]
        self.get_logger().info(
            f"Loaded {model_path}: {provenance.get('experiment_name')}/{provenance.get('run_name')} "
            f"iter {provenance.get('iteration')} (sha256 {self.manifest.checkpoint_sha256[:12]}), "
            f"box {self.manifest.box_size.tolist()} m, reference {self.manifest.reference_mode}, "
            f"{1.0 / self.manifest.policy_dt:.0f} Hz, {self.manifest.episode_length_s:.1f} s episodes, "
            f"start {self.manifest.start_mode}"
            + ("" if self.manifest.pregrasp_joint_position is None
               else f" ({len(self.manifest.pregrasp_joint_position)} table rows)")
            + f", goal z {goal_z:.3f} m, goal xy {goal_reference} + {goal_xy_offset.tolist()} m ({trained_xy}), "
            f"mode {'SHADOW' if self.shadow_mode else 'COMMAND'}"
        )

    # ------------------------------------------------------------------ goal settings
    def _goal_settings(self) -> tuple[float, np.ndarray, str]:
        """Current goal_z_m, goal_xy_offset_m, goal_xy_reference (read at every start_policy and IDLE preview)."""
        return (float(self.get_parameter("goal_z_m").value),
                np.asarray(self.get_parameter("goal_xy_offset_m").value, dtype=np.float64),
                str(self.get_parameter("goal_xy_reference").value))

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        changed = {p.name: p.value for p in parameters
                   if p.name in ("goal_z_m", "goal_xy_offset_m", "goal_xy_reference")}
        if not changed:
            return SetParametersResult(successful=True)
        if self.state != "IDLE":
            return SetParametersResult(successful=False,
                                       reason=f"goal settings can only change while IDLE (session {self.state})")
        goal_z, goal_xy_offset, goal_reference = self._goal_settings()
        try:
            check_goal_settings(float(changed.get("goal_z_m", goal_z)),
                                changed.get("goal_xy_offset_m", goal_xy_offset), self.manifest,
                                str(changed.get("goal_xy_reference", goal_reference)))
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.get_logger().info(
            "goal settings accepted: " + ", ".join(f"{k} {np.asarray(v).tolist()}" for k, v in changed.items()))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _log_status(self, message: str) -> None:
        now = self._now_ns()
        if now - self.last_status_log_ns > 1_000_000_000:
            self.get_logger().warn(message)
            self.last_status_log_ns = now

    def _joint_state_callback(self, msg: JointState) -> None:
        now = self._now_ns()
        index = {name: i for i, name in enumerate(msg.name)}
        for out, name in enumerate(DUAL_ARM_JOINT_NAMES):
            i = index.get(name)
            if i is None or i >= len(msg.position) or not np.isfinite(msg.position[i]):
                continue
            previous, previous_ns = self.joint_position[out], self.joint_update_ns[out]
            self.joint_position[out] = float(msg.position[i])
            if i < len(msg.velocity) and np.isfinite(msg.velocity[i]):
                self.joint_velocity[out] = float(msg.velocity[i])
            elif np.isfinite(previous) and previous_ns > 0 and (now - previous_ns) > 100_000:
                self.joint_velocity[out] = (self.joint_position[out] - previous) / ((now - previous_ns) * 1e-9)
            self.joint_update_ns[out] = now

    def _box_pose_callback(self, msg: PoseStamped) -> None:
        position = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        o = msg.pose.orientation
        try:
            quat = normalize_quat([o.x, o.y, o.z, o.w])
            if not np.all(np.isfinite(position)):
                raise ValueError("non-finite box position")
            frame = msg.header.frame_id or self.base_frame
            if frame != self.base_frame:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, RosTime()).transform
                position, quat = transform_pose(
                    [t.translation.x, t.translation.y, t.translation.z],
                    [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
                    position, quat,
                )
        except (ValueError, TransformException) as error:
            self._log_status(f"Ignoring box pose: {error}")
            return
        self.box_position_b, self.box_quat_b = position + self.box_position_offset, quat
        self.box_update_ns = self._now_ns()
        self.box_stamp_ns = RosTime.from_msg(msg.header.stamp).nanoseconds

    def _inputs_ready(self) -> tuple[bool, str]:
        now = self._now_ns()
        if not (np.all(np.isfinite(self.joint_position)) and np.all(np.isfinite(self.joint_velocity))):
            return False, "waiting for all 14 arm joint positions and velocities"
        if np.any(now - self.joint_update_ns > float(self.get_parameter("joint_state_timeout_s").value) * 1e9):
            return False, "joint state is stale"
        if self.box_position_b is None:
            return False, "waiting for the box pose"
        if now - self.box_update_ns > float(self.get_parameter("box_pose_timeout_s").value) * 1e9:
            return False, f"box pose is stale ({(now - self.box_update_ns) * 1e-6:.0f} ms)"
        return True, "ready"

    # ------------------------------------------------------------------ session control
    def _start_service(self, _request, response):
        response.success, response.message = self._request_start()
        if not response.success:
            self.get_logger().warn(f"start_policy refused: {response.message}")
        return response

    def _request_start(self) -> tuple[bool, str]:
        if self.state != "IDLE":
            return False, f"session already {self.state}"
        ready, message = self._inputs_ready()
        if not ready:
            return False, message
        start = check_start_conditions(
            self.joint_position, self.box_position_b, self.box_quat_b, self.manifest,
            ready_tolerance_rad=float(self.get_parameter("ready_tolerance_rad").value),
            start_margin_m=float(self.get_parameter("start_position_margin_m").value),
            yaw_margin_rad=float(self.get_parameter("start_yaw_margin_rad").value),
            rest_z_tolerance_m=float(self.get_parameter("rest_z_tolerance_m").value),
            pregrasp_joint_margin_rad=float(self.get_parameter("pregrasp_joint_margin_rad").value),
        )
        if not start.ok:
            return False, "; ".join(start.reasons)
        # Training: the goal x/y is the box start x/y (+ the trained offset), z is the lift height;
        # previous action starts at 0. range_center: the goal range center + offset instead.
        goal_z, goal_xy_offset, goal_reference = self._goal_settings()
        try:
            goal_base = make_goal(self.box_position_b[:2], goal_z, self.manifest, goal_xy_offset, goal_reference)
        except ValueError as error:
            return False, str(error)
        self.start_table_row = start.table_row
        if not self.shadow_mode and not self.action_client.server_is_ready():
            return False, "fr3_policy_control action server is not ready"

        self.goal = goal_base
        self.goal_marker_publisher.publish(
            self._box_marker("goal", self.goal[:3], self.goal[3:7], (0.1, 0.9, 0.2, 0.35))
        )
        self.previous_action.fill(0.0)
        self.step_index = 0
        self.horizon_warned = False
        self.final_box_pose = None
        self.result_recorded = False
        self.feedback_max_joint_error = float("nan")
        self.feedback_command_age = float("nan")
        self.end_reason = ""
        self.session_start_ns = time.monotonic_ns()
        self.last_tick_ns = 0
        self._open_log()

        row = "" if self.start_table_row is None else f", pre-grasp table row {self.start_table_row}"
        row += (f", goal {np.round(self.goal[:3], 3).tolist()} m "
                f"({goal_reference} + offset {goal_xy_offset.tolist()}, z {goal_z})")
        if self.shadow_mode:
            self.state = "RUNNING"
            return True, f"shadow session started{row} (inference and logging only, no commands)"

        goal = RunPolicyControl.Goal()
        goal.robot_name = "dual"
        goal.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        goal.max_duration_s = float(self.get_parameter("max_duration_s").value)
        goal.max_policy_target_delta_rad = self.max_policy_target_delta
        goal.max_actuator_step_rad = _LEGACY_MAX_ACTUATOR_STEP_RAD
        goal.joint_velocity_scale = _LEGACY_JOINT_VELOCITY_SCALE
        goal.joint_acceleration_scale = _LEGACY_JOINT_ACCELERATION_SCALE
        goal.isaac_relative_control = True
        goal.relative_target_refresh_hz = _LEGACY_RELATIVE_TARGET_REFRESH_HZ
        goal.control_gripper = False
        self.state = "PENDING"
        future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        return True, f"policy control goal requested{row}"

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
        self.get_logger().info(
            f"Box-lift policy RUNNING: goal {np.round(self.goal[:3], 3).tolist()} (base). "
            "Stopping releases the squeeze and drops the box."
        )

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
        self.get_logger().error(f"Box-lift fault: {message}")
        self.end_reason = f"fault: {message}"
        self._stop_controller_or_end()

    def _stop_controller_or_end(self) -> None:
        # the box as the policy last saw it, before the controller releases the squeeze (a held box drops then)
        if self.final_box_pose is None and self.box_position_b is not None:
            self.final_box_pose = (self.box_position_b.copy(), self.box_quat_b.copy())
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
            pose = self.final_box_pose or (
                None if self.box_position_b is None else (self.box_position_b, self.box_quat_b))
            if pose is not None and self.goal is not None:
                if self.result_recorded:  # row already written at the trained episode end; keep the stop pose too
                    self.logger_.metadata["box_vs_goal_at_session_end"] = box_goal_error(
                        *pose, self.goal, self.logger_.metadata.get("box_start_quat_base_raw"))
                else:  # ended before the trained episode length
                    self._record_result_safely(reason, *pose)
            run_dir = self.logger_.finalize(reason)
            self.get_logger().info(f"Session ended ({reason}); {self.logger_.row_count} steps logged to {run_dir}")
            self.logger_ = None
        self.final_box_pose = None

    def _record_result_safely(self, reason: str, box_position, box_quat) -> None:
        self.result_recorded = True
        try:
            self._record_result(reason, box_position, box_quat)
        except Exception as error:  # never lose the trace/metadata over the summary
            self.get_logger().error(f"Failed to record the session result: {error!r}")

    def _record_result(self, reason: str, box_position, box_quat) -> None:
        """Final box vs goal: into metadata.json and one row of <result_root>/<task>/results.csv."""
        metadata = self.logger_.metadata
        final = box_goal_error(box_position, box_quat, self.goal, metadata.get("box_start_quat_base_raw"))
        metadata["final_box_vs_goal"] = final
        e, rpy = final["error_xyz_m"], final["rpy_error_deg"]
        self.get_logger().info(
            f"Final box vs goal: error x {e[0]:+.3f} y {e[1]:+.3f} z {e[2]:+.3f} m (|xyz| {final['error_m']:.3f}), "
            f"rpy error {rpy[0]:+.1f} {rpy[1]:+.1f} {rpy[2]:+.1f} deg, tilt {final['tilt_deg']:.1f} deg")
        start = np.asarray(metadata["box_start_position_base"], dtype=np.float64)
        offset = self.get_parameter("goal_xy_offset_m").value
        row = {
            "session": self.logger_.run_dir.name,
            "model": Path(str(self.actor.model_path)).name,
            "mode": metadata["mode"],
            "end_reason": reason,
            "steps": self.logger_.row_count,
            "duration_s": round(self.logger_.row_count * self.manifest.policy_dt, 2),
            "goal_xy_reference": str(self.get_parameter("goal_xy_reference").value),
            "goal_xy_offset_x": float(offset[0]),
            "goal_xy_offset_y": float(offset[1]),
            **{f"start_{a}": round(float(v), 4) for a, v in zip("xyz", start)},
            **{f"goal_{a}": v for a, v in zip("xyz", final["goal_position_base"])},
            **{f"final_{a}": v for a, v in zip("xyz", final["box_position_base"])},
            **{f"error_{a}": v for a, v in zip("xyz", e)},
            "error_xy_norm": final["error_xy_m"],
            "error_xyz_norm": final["error_m"],
            **{f"final_{a}_deg": v for a, v in zip(("roll", "pitch", "yaw"), final["box_rpy_deg"])},
            **{f"error_{a}_deg": v for a, v in zip(("roll", "pitch", "yaw"), rpy)},
            **{f"change_{a}_deg": v for a, v in zip(("roll", "pitch", "yaw"), final.get("rpy_change_deg", [None] * 3))},
            "tilt_deg": final["tilt_deg"],
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
        parameters = {
            name: self.get_parameter(name).value
            for name in (
                "box_size_m", "box_size_tolerance_m", "joint_states_topic", "box_pose_topic", "box_position_offset_m", "base_frame",
                "goal_z_m", "goal_xy_offset_m", "goal_xy_reference", "joint_state_timeout_s", "box_pose_timeout_s", "command_timeout_s", "max_duration_s",
                "max_policy_target_delta_rad", "ready_tolerance_rad", "start_position_margin_m",
                "start_yaw_margin_rad", "rest_z_tolerance_m", "pregrasp_joint_margin_rad", "shadow_mode",
            )
        }
        try:
            self.logger_ = BoxLiftRunLogger(
                log_root,
                str(self.get_parameter("log_task_name").value),
                {
                    "model_path": str(self.actor.model_path),
                    "model_source_sha256": self.actor.source_sha256,
                    "mode": "shadow" if self.shadow_mode else "command",
                    "goal_base": self.goal,
                    "box_start_position_base": self.box_position_b,
                    "box_start_quat_base_raw": self.box_quat_b,
                    "box_start_yaw": float(yaw_from_quat(canonicalize_box_quat(self.box_quat_b))),
                    "start_mode": self.manifest.start_mode,
                    "start_table_row": self.start_table_row,
                    # copy: the logger rewrites metadata at finalize, after joint_position was updated in place
                    "joint_start_position": self.joint_position.copy(),
                    "joint_names": list(DUAL_ARM_JOINT_NAMES),
                    "parameters": parameters,
                    "manifest": self.manifest.raw,
                },
            )
            self.get_logger().info(f"Logging to {self.logger_.run_dir}")
        except (OSError, ValueError, TypeError) as error:
            self.logger_ = None
            self.get_logger().error(f"Failed to start box-lift logging: {error}")

    # ------------------------------------------------------------------ policy tick
    def _box_marker(self, ns: str, position, quat, rgba) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = ns, 0, Marker.CUBE, Marker.ADD
        p, o = marker.pose.position, marker.pose.orientation
        p.x, p.y, p.z = (float(v) for v in position)
        o.x, o.y, o.z, o.w = (float(v) for v in quat)
        marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in self.box_size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _start_range_markers(self) -> MarkerArray:
        """Trained box start centers (pre-grasp table rows) and their x/y bounds, drawn at the box top face."""
        z = self.manifest.rest_center_z + 0.5 * float(self.box_size[2]) + 0.002
        markers = []
        if self.manifest.start_mode == START_PREGRASP_TABLE:
            xy = self.manifest.pregrasp_box_pose[:, :2]
            noise = self.manifest.start_box_pos_noise
            low, high = xy.min(axis=0) - noise, xy.max(axis=0) + noise
            centers = Marker()
            centers.header.frame_id = self.base_frame
            centers.ns, centers.id, centers.type, centers.action = "start_centers", 0, Marker.POINTS, Marker.ADD
            centers.pose.orientation.w = 1.0
            centers.scale.x = centers.scale.y = 0.004
            centers.color.r, centers.color.g, centers.color.b, centers.color.a = 0.2, 0.6, 1.0, 0.8
            centers.points = [Point(x=float(px), y=float(py), z=z) for px, py in xy]
            markers.append(centers)
        else:
            low = np.array([self.manifest.start_x_range[0], self.manifest.start_y_range[0]])
            high = np.array([self.manifest.start_x_range[1], self.manifest.start_y_range[1]])
        bounds = Marker()
        bounds.header.frame_id = self.base_frame
        bounds.ns, bounds.id, bounds.type, bounds.action = "start_bounds", 0, Marker.LINE_STRIP, Marker.ADD
        bounds.pose.orientation.w = 1.0
        bounds.scale.x = 0.003
        bounds.color.r, bounds.color.g, bounds.color.b, bounds.color.a = 0.2, 0.6, 1.0, 1.0
        bounds.points = [Point(x=float(px), y=float(py), z=z) for px, py in
                         ((low[0], low[1]), (high[0], low[1]), (high[0], high[1]), (low[0], high[1]),
                          (low[0], low[1]))]
        markers.append(bounds)
        return MarkerArray(markers=markers)

    def _tick(self) -> None:
        if self.box_position_b is not None:
            self.box_marker_publisher.publish(
                self._box_marker("policy_box", self.box_position_b, self.box_quat_b, (1.0, 0.55, 0.0, 0.5))
            )
            if self.state == "IDLE":
                # Preview: the goal start_policy would latch for the box where it is now
                # (red: outside the trained goal range for this box position, start_policy refuses it).
                goal_z, goal_xy_offset, goal_reference = self._goal_settings()
                rgba = (0.1, 0.9, 0.2, 0.35)
                try:
                    make_goal(self.box_position_b[:2], goal_z, self.manifest, goal_xy_offset, goal_reference)
                    goal_xy = goal_xy_from_settings(self.box_position_b[:2], goal_xy_offset, self.manifest,
                                                    goal_reference)
                except ValueError as error:
                    rgba = (0.9, 0.1, 0.1, 0.35)
                    self._log_status(f"idle: {error}")
                    goal_xy = self.box_position_b[:2] + goal_xy_offset
                self.goal_marker_publisher.publish(self._box_marker(
                    "goal", np.append(goal_xy, goal_z), GOAL_QUAT_XYZW, rgba))
        if self.state != "RUNNING":
            if self.state == "IDLE":
                ready, message = self._inputs_ready()
                if not ready:
                    self._log_status(f"idle: {message}")
            return
        ready, message = self._inputs_ready()
        if not ready:
            self._fault(message)
            return

        tick_ns = time.monotonic_ns()
        now_ns = self._now_ns()
        q = self.joint_position.copy()
        try:
            observation = build_observation(
                q, self.joint_velocity, self.box_position_b, self.box_quat_b,
                self.goal, self.previous_action, self.manifest.default_position,
            )
            action = np.asarray(self.actor(observation), dtype=np.float64)
            target, delta = process_action(action, q, self.manifest)
        except (ValueError, FloatingPointError) as error:
            self._fault(f"inference failed: {error}")
            return
        inference_ms = (time.monotonic_ns() - tick_ns) * 1e-6
        max_delta = float(np.max(np.abs(delta)))
        # The action term bounds |delta| by the clip; anything else is a bug, never a policy choice.
        if not np.all(np.isfinite(target)) or max_delta > float(self.manifest.action_clip[1]) + 1e-9:
            self._fault(f"invalid joint target (max |delta| {max_delta:.4f} rad)")
            return

        if not self.shadow_mode:
            self.sequence += 1
            command = PolicyJointCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self.base_frame
            command.sequence = self.sequence
            command.robot_name = "dual"
            # policy_step as trained: an absolute target the controller holds until the next command.
            command.target_positions = target.tolist()
            command.relative_position_offsets = False
            command.gripper_action = 0.0
            self.command_publisher.publish(command)
        self.previous_action = action

        if self.logger_ is not None:
            try:
                self.logger_.append(
                    {
                        "elapsed_s": (tick_ns - self.session_start_ns) * 1e-9,
                        "ros_time_ns": now_ns,
                        "step": self.step_index,
                        "tick_period_ms": (tick_ns - self.last_tick_ns) * 1e-6 if self.last_tick_ns else float("nan"),
                        "joint_state_age_ms": (now_ns - int(self.joint_update_ns.min())) * 1e-6,
                        "box_pose_age_ms": (now_ns - self.box_update_ns) * 1e-6,
                        "box_stamp_age_ms": (now_ns - self.box_stamp_ns) * 1e-6 if self.box_stamp_ns else float("nan"),
                        "inference_ms": inference_ms,
                        "max_abs_delta_rad": max_delta,
                        "feedback_max_joint_error_rad": self.feedback_max_joint_error,
                        "feedback_command_age_s": self.feedback_command_age,
                    },
                    observation, action, target, self.box_quat_b,
                )
            except (OSError, ValueError) as error:
                self._log_status(f"Skipping log row: {error}")
        self.last_tick_ns = tick_ns
        self.step_index += 1
        if not self.horizon_warned and self.step_index * self.manifest.policy_dt > self.manifest.episode_length_s:
            self.horizon_warned = True
            self.get_logger().warn(
                f"Past the trained episode length ({self.manifest.episode_length_s:.1f} s); "
                "the policy never saw this phase in training."
            )
            # the result row is the box at the trained episode end; the policy keeps running (and holding)
            if self.logger_ is not None and not self.result_recorded:
                self._record_result_safely(
                    f"trained episode end ({self.manifest.episode_length_s:.1f} s)",
                    self.box_position_b, self.box_quat_b)


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = PPOBoxLiftPolicyNode()
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
        node._end_session(node.end_reason or "node shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
