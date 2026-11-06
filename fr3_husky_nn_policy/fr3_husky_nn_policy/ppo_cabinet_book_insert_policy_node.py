"""Inference node for the dual-FR3 cabinet book-insertion policy.

The node consumes joint states plus book pose and positive-open door angle. The latter
two inputs are deliberately separate: an FK estimator can publish them once the measured
UMI-to-object and handle-to-door transforms have been calibrated.
"""

from __future__ import annotations

import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from fr3_husky_msgs.action import RunPolicyControl
from fr3_husky_msgs.msg import PolicyJointCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .boxlift_core import transform_pose
from .cabinet_book_insert_core import (
    ACTION_DIM, JOINT_NAMES, build_observation, load_cabinet_policy, nearest_start_row, normalize_quat_xyzw,
    process_action,
)

# fr3_husky_controller policy_control_action_server: joint speed limit = these x joint_velocity_scale, capped at
# max_actuator_step_rad per 1 kHz control period; the goal is rejected unless 0 < max_actuator_step_rad <= 0.01.
FR3_MAX_JOINT_VELOCITY = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
CONTROL_PERIOD_S = 0.001


class PPOCabinetBookInsertPolicyNode(Node):
    def __init__(self):
        super().__init__("ppo_cabinet_book_insert_policy_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_states_topic", "/dual_fr3/joint_states")
        self.declare_parameter("book_pose_topic", "/cabinet_book_insert/book_pose")
        self.declare_parameter("door_angle_topic", "/cabinet_book_insert/door_angle")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("command_topic", "/policy_joint_command")
        self.declare_parameter("control_action", "/fr3_policy_control")
        self.declare_parameter("joint_state_timeout_s", 0.15)
        self.declare_parameter("book_pose_timeout_s", 0.25)
        self.declare_parameter("door_angle_timeout_s", 0.25)
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("max_policy_target_delta_rad", 0.20)
        self.declare_parameter("start_door_angle_tolerance_rad", 0.20)
        self.declare_parameter("start_joint_tolerance_rad", 0.10)
        # controller goal (boxlift values); joint_velocity_scale 0.19 = 0.5 rad/s on j1-4 (Slow checkpoints)
        self.declare_parameter("max_actuator_step_rad", 0.001)
        self.declare_parameter("joint_velocity_scale", 0.10)
        self.declare_parameter("joint_acceleration_scale", 0.20)
        self.declare_parameter("relative_target_refresh_hz", 1000.0)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("log_root", "")

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            raise ValueError("model_path is required (NPZ from export_cabinet_book_insert_actor)")
        self.actor, self.manifest = load_cabinet_policy(model_path)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.max_policy_target_delta = float(self.get_parameter("max_policy_target_delta_rad").value)
        if not (self.manifest.action_clip[1] < self.max_policy_target_delta <= 1.0):
            raise ValueError("max_policy_target_delta_rad must exceed trained action clip and be <= 1.0")
        self.max_actuator_step = float(self.get_parameter("max_actuator_step_rad").value)
        self.joint_velocity_scale = float(self.get_parameter("joint_velocity_scale").value)
        self.joint_acceleration_scale = float(self.get_parameter("joint_acceleration_scale").value)
        if not 0.0 < self.max_actuator_step <= 0.01:
            raise ValueError("max_actuator_step_rad must be in (0, 0.01] (the controller rejects the goal otherwise)")
        if not (0.0 < self.joint_velocity_scale <= 1.0 and 0.0 < self.joint_acceleration_scale <= 1.0):
            raise ValueError("joint_velocity_scale and joint_acceleration_scale must be in (0, 1]")
        real_speed = np.minimum(FR3_MAX_JOINT_VELOCITY * self.joint_velocity_scale,
                                self.max_actuator_step / CONTROL_PERIOD_S)
        trained_speed = self.manifest.action_clip[1] / self.manifest.policy_dt

        self.q = np.full(ACTION_DIM, np.nan, dtype=np.float64)
        self.qd = np.full(ACTION_DIM, np.nan, dtype=np.float64)
        self.q_time_ns = np.zeros(ACTION_DIM, dtype=np.int64)
        self.book_position = None
        self.book_quaternion = None
        self.book_time_ns = 0
        self.door_angle = float("nan")
        self.door_time_ns = 0
        self.previous_action = np.zeros(ACTION_DIM, dtype=np.float64)
        self.step_index = 0
        self.sequence = 0
        self.session_start_ns = 0
        self.last_tick_ns = 0
        self.state = "IDLE"
        self.goal_handle = None
        self.end_reason = ""
        self.trace_file = None
        self.trace_writer = None
        self.last_status_log_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value,
                                 self._joint_state_callback, 10)
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, self.get_parameter("book_pose_topic").value,
                                 self._book_pose_callback, pose_qos)
        self.create_subscription(Float64, self.get_parameter("door_angle_topic").value,
                                 self._door_angle_callback, 10)
        self.command_publisher = self.create_publisher(
            PolicyJointCommand, self.get_parameter("command_topic").value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.action_client = ActionClient(self, RunPolicyControl, self.get_parameter("control_action").value)
        self.create_service(Trigger, "~/start_policy", self._start_service)
        self.create_service(Trigger, "~/stop_policy", self._stop_service)
        self.timer = self.create_timer(self.manifest.policy_dt, self._tick)

        provenance = self.manifest.raw["provenance"]
        self.get_logger().info(
            f"Loaded {model_path}: {provenance.get('experiment_name')}/{provenance.get('run_name')} "
            f"iter {provenance.get('iteration')} ({self.actor.input_dim}->{self.actor.output_dim}), "
            f"{1.0 / self.manifest.policy_dt:.1f} Hz, {self.manifest.episode_length_s:.1f} s horizon, "
            f"mode {'SHADOW' if self.shadow_mode else 'COMMAND'}")
        self.get_logger().info(
            "Inputs: JointState + book-center PoseStamped (TF to base) + Float64 door angle "
            "(positive=open, rad). Start is operator-triggered.")
        self.get_logger().info(
            f"Joint speed: trained up to {trained_speed:.2f} rad/s per joint, controller limit "
            f"{np.round(real_speed, 2).tolist()} rad/s (j1-j7)")
        if real_speed.min() < 0.9 * trained_speed:
            self.get_logger().warn(
                "The controller is slower than the policy was trained for; the arms lag the policy's targets "
                "(states it did not see in training).")
        if self.manifest.start_joint_position is None:
            self.get_logger().warn("NPZ has no start rows: start_policy cannot check the start pose (re-export it)")
        else:
            self.get_logger().info(
                f"{len(self.manifest.start_joint_position)} start rows; start_policy needs every joint within "
                f"{float(self.get_parameter('start_joint_tolerance_rad').value):.2f} rad of one of them")

    def _now_ns(self):
        return time.monotonic_ns()

    def _status(self, message):
        now = self._now_ns()
        if now - self.last_status_log_ns > 1_000_000_000:
            self.get_logger().warn(message)
            self.last_status_log_ns = now

    def _joint_state_callback(self, msg: JointState):
        now = self._now_ns()
        indices = {name: i for i, name in enumerate(msg.name)}
        for out, name in enumerate(JOINT_NAMES):
            index = indices.get(name)
            if index is None or index >= len(msg.position) or not np.isfinite(msg.position[index]):
                continue
            old_q, old_ns = self.q[out], self.q_time_ns[out]
            self.q[out] = float(msg.position[index])
            if index < len(msg.velocity) and np.isfinite(msg.velocity[index]):
                self.qd[out] = float(msg.velocity[index])
            elif np.isfinite(old_q) and old_ns and now > old_ns:
                self.qd[out] = (self.q[out] - old_q) / ((now - old_ns) * 1.0e-9)
            self.q_time_ns[out] = now

    def _book_pose_callback(self, msg: PoseStamped):
        position = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        orientation = msg.pose.orientation
        try:
            quat = normalize_quat_xyzw([orientation.x, orientation.y, orientation.z, orientation.w])
            if not np.all(np.isfinite(position)):
                raise ValueError("book position contains NaN/Inf")
            frame = msg.header.frame_id or self.base_frame
            if frame != self.base_frame:
                transform = self.tf_buffer.lookup_transform(self.base_frame, frame, RosTime()).transform
                position, quat = transform_pose(
                    [transform.translation.x, transform.translation.y, transform.translation.z],
                    [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
                    position, quat)
            self.book_position = np.asarray(position, dtype=np.float64)
            self.book_quaternion = normalize_quat_xyzw(quat)
            self.book_time_ns = self._now_ns()
        except (ValueError, TransformException) as error:
            self._status(f"Ignoring book pose: {error}")

    def _door_angle_callback(self, msg: Float64):
        angle = float(msg.data)
        if np.isfinite(angle):
            self.door_angle = angle
            self.door_time_ns = self._now_ns()

    def _inputs_ready(self):
        now = self._now_ns()
        if not (np.all(np.isfinite(self.q)) and np.all(np.isfinite(self.qd))):
            return False, "waiting for positions and velocities of all 14 arm joints"
        if np.any(now - self.q_time_ns > float(self.get_parameter("joint_state_timeout_s").value) * 1e9):
            return False, "joint state is stale"
        if self.book_position is None or now - self.book_time_ns > float(self.get_parameter("book_pose_timeout_s").value) * 1e9:
            return False, "book pose missing or stale"
        if not np.isfinite(self.door_angle) or now - self.door_time_ns > float(self.get_parameter("door_angle_timeout_s").value) * 1e9:
            return False, "door angle missing or stale"
        return True, "ready"

    def _start_service(self, _request, response):
        if self.state != "IDLE":
            response.success, response.message = False, f"policy session is {self.state}"
            return response
        ready, reason = self._inputs_ready()
        if not ready:
            response.success, response.message = False, reason
            return response
        expected_angle = float(self.manifest.raw.get("start", {}).get("door_angle_rad", 0.0))
        angle_tolerance = float(self.get_parameter("start_door_angle_tolerance_rad").value)
        if abs(self.door_angle - expected_angle) > angle_tolerance:
            response.success = False
            response.message = (f"door angle {self.door_angle:.3f} rad is outside checkpoint start "
                                f"{expected_angle:.3f} +/- {angle_tolerance:.3f} rad")
            return response
        if self.manifest.start_joint_position is None:
            if not self.shadow_mode:
                response.success = False
                response.message = "NPZ has no start rows; re-export it before command mode"
                return response
        else:
            row, joint_error = nearest_start_row(self.q, self.manifest)
            joint_tolerance = float(self.get_parameter("start_joint_tolerance_rad").value)
            row_q = self.manifest.start_joint_position[row]
            book = self.manifest.start_book_position_base[row]
            self.get_logger().info(
                f"Nearest start row {row}: largest joint error {joint_error:.3f} rad, row book centre (base) "
                f"{np.round(book, 3).tolist()}, measured {np.round(self.book_position, 3).tolist()}")
            if joint_error > joint_tolerance:
                self.get_logger().warn(
                    "Move to that row with: ros2 run fr3_husky_task_manager move_to_joint --left-target-positions "
                    + " ".join(f"{v:.4f}" for v in row_q[:7]) + " --right-target-positions "
                    + " ".join(f"{v:.4f}" for v in row_q[7:]))
                response.success = False
                response.message = (f"joints are {joint_error:.3f} rad from the nearest training start row {row} "
                                    f"(tolerance {joint_tolerance:.3f})")
                return response
        if not self.shadow_mode and not self.action_client.server_is_ready():
            response.success, response.message = False, "fr3_policy_control action server is not ready"
            return response

        self.previous_action.fill(0.0)
        self.step_index = 0
        self.sequence = 0
        self.session_start_ns = self._now_ns()
        self.last_tick_ns = 0
        self.end_reason = ""
        self._open_trace()
        if self.shadow_mode:
            self.state = "RUNNING"
            response.success, response.message = True, "shadow inference started; no commands will be published"
            return response

        goal = RunPolicyControl.Goal()
        goal.robot_name = "dual"
        goal.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        goal.max_duration_s = 0.0  # the node itself stops at the checkpoint's trained episode horizon
        goal.max_policy_target_delta_rad = self.max_policy_target_delta
        goal.max_actuator_step_rad = self.max_actuator_step
        goal.joint_velocity_scale = self.joint_velocity_scale
        goal.joint_acceleration_scale = self.joint_acceleration_scale
        goal.isaac_relative_control = True
        goal.relative_target_refresh_hz = float(self.get_parameter("relative_target_refresh_hz").value)
        goal.control_gripper = False
        self.state = "PENDING"
        future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        response.success, response.message = True, "controller policy goal requested"
        return response

    def _goal_response_callback(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._end_session("controller rejected the goal")
            return
        if self.state != "PENDING":
            handle.cancel_goal_async()
            return
        self.goal_handle = handle
        self.state = "RUNNING"
        handle.get_result_async().add_done_callback(self._result_callback)
        self.get_logger().info("Cabinet book-insertion policy is RUNNING")

    def _feedback_callback(self, _feedback):
        return

    def _result_callback(self, future):
        wrapped = future.result()
        result = None if wrapped is None else wrapped.result
        message = "no result" if result is None else f"success={result.success}, {result.message!r}"
        self.goal_handle = None
        self._end_session(self.end_reason or f"controller ended: {message}")
        self.get_logger().info(f"policy control finished: {message}")

    def _stop_service(self, _request, response):
        if self.state == "IDLE":
            response.success, response.message = False, "no active session"
            return response
        self.end_reason = "operator stop"
        if self.goal_handle is not None:
            self.state = "STOPPING"
            self.goal_handle.cancel_goal_async()
        else:
            self._end_session(self.end_reason)
        response.success, response.message = True, "stop requested"
        return response

    def _open_trace(self):
        configured = str(self.get_parameter("log_root").value).strip()
        root = Path(configured).expanduser() if configured else Path(__file__).resolve().parents[2] / "data"
        run_dir = root / "cabinet_book_insert" / datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%Z")
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "metadata.json").write_text(json.dumps({
            "model_path": str(self.actor.model_path),
            "checkpoint_sha256": self.manifest.checkpoint_sha256,
            "manifest": self.manifest.raw,
            "mode": "shadow" if self.shadow_mode else "command",
            "joint_names": list(JOINT_NAMES),
        }, indent=2), encoding="utf-8")
        self.trace_file = (run_dir / "trace.csv").open("w", newline="", encoding="utf-8")
        self.trace_writer = csv.writer(self.trace_file)
        self.trace_writer.writerow(
            ["elapsed_s", "step", "tick_period_ms", "inference_ms", "door_angle_rad"]
            + [f"obs_{i}" for i in range(50)]
            + [f"action_{i}" for i in range(14)]
            + [f"q_target_{i}" for i in range(14)])
        self.trace_file.flush()

    def _end_session(self, reason):
        self.state = "IDLE"
        self.goal_handle = None
        if self.trace_file is not None:
            self.trace_file.flush()
            self.trace_file.close()
            self.trace_file = None
            self.trace_writer = None
        self.get_logger().info(f"Session ended: {reason}")

    def _tick(self):
        if self.state != "RUNNING":
            return
        ready, reason = self._inputs_ready()
        if not ready:
            self._fault(reason)
            return
        now = self._now_ns()
        elapsed_s = (now - self.session_start_ns) * 1.0e-9
        if elapsed_s > self.manifest.episode_length_s:
            self.end_reason = "trained episode horizon reached"
            self._stop_controller_or_end()
            return
        q = self.q.copy()
        try:
            t0 = self._now_ns()
            observation = build_observation(
                q, self.qd, self.door_angle, self.book_position, self.book_quaternion,
                self.previous_action, self.manifest)
            action = np.asarray(self.actor(observation), dtype=np.float64)
            target, delta = process_action(action, q, self.manifest)
            inference_ms = (self._now_ns() - t0) * 1.0e-6
        except (ValueError, FloatingPointError) as error:
            self._fault(f"inference failed: {error}")
            return
        max_delta = float(np.max(np.abs(delta)))
        if not np.all(np.isfinite(target)) or max_delta > self.max_policy_target_delta:
            self._fault(f"unsafe/non-finite target; max joint delta {max_delta:.4f} rad")
            return
        if not self.shadow_mode:
            self.sequence += 1
            command = PolicyJointCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self.base_frame
            command.sequence = self.sequence
            command.robot_name = "dual"
            command.target_positions = target.tolist()
            command.relative_position_offsets = False
            command.gripper_action = 0.0
            self.command_publisher.publish(command)
        if self.trace_writer is not None:
            period_ms = (now - self.last_tick_ns) * 1.0e-6 if self.last_tick_ns else float("nan")
            self.trace_writer.writerow([elapsed_s, self.step_index, period_ms, inference_ms, self.door_angle]
                                       + observation.tolist() + action.tolist() + target.tolist())
            self.trace_file.flush()
        self.previous_action = action
        self.last_tick_ns = now
        self.step_index += 1

    def _fault(self, message):
        self.get_logger().error(f"Cabinet insertion policy fault: {message}")
        self.end_reason = f"fault: {message}"
        self._stop_controller_or_end()

    def _stop_controller_or_end(self):
        if self.goal_handle is not None:
            self.state = "STOPPING"
            self.goal_handle.cancel_goal_async()
        else:
            self._end_session(self.end_reason or "stopped")


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = PPOCabinetBookInsertPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node._end_session(node.end_reason or "node shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
