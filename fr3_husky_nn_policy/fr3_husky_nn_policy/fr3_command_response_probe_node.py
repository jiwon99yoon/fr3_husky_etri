"""FR3 command-response probe: absolute 20 Hz targets on the effort or position interface.

Purpose (KRoC 11-16 plan, ``dual_fr3_lab/docs/todo_1116.md``): measure the response of
the *box-lift policy command path* to identical excitations on both FR3 command
interfaces, so torque mode and position mode can be compared on the same day,
pose and settings.

Each command is an absolute joint target (``relative_position_offsets=False``)
that PolicyControl holds until the next one (zero-order hold):

* ``command_interface:=effort``   -> Isaac PD (Kp 80, Kd 4) + 1000 Nm/s, the v2 path
* ``command_interface:=position`` -> 1 kHz jerk-limited tracker -> Franka joint impedance

The older ``fr3_consistency_probe_node`` / ``fr3_sysid_node`` send measured-relative
offsets (refreshed at 100 Hz) and are kept unchanged so the 9/29 data stay reproducible.

Run sequence: ``start_probe`` -> wait until the arm is stationary -> latch q0 ->
check every trial against the joint limits -> one PolicyControl goal for the whole
run -> for each trial: profile, then hold q0 until position and velocity settle.
``stop_probe`` cancels at any time.  ``dry_run`` (default true) runs the clock and
logs the measured state without sending a goal or commands.

Supports a single-arm launch (``robot:=left|right``, 7 joints) and the dual launch
(``robot:=dual``, 14 joints; ``excite_arm`` selects the moving arm, the other is held).

Logs: ``<log_root>/<stamp>_<label>/metadata.json`` + one CSV per trial at the
joint-state rate.  Record a rosbag alongside (joint_states, robot_state,
policy_joint_command, rosout) for latency and Franka error analysis.
"""

from __future__ import annotations

import csv
import json
import math
import socket
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from fr3_husky_msgs.action import RunPolicyControl
from fr3_husky_msgs.msg import PolicyJointCommand
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .command_response_profiles import (
    ALL_TESTS,
    ProfileConfig,
    Trial,
    build_trials,
    load_replay,
)

# FR3 joint limits used by PolicyControl (policy_control_action_server.hpp).
FR3_LOWER = np.asarray([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
FR3_UPPER = np.asarray([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
JOINT_LIMIT_MARGIN_RAD = 0.02  # PolicyControl kJointLimitMargin

# Feedback status -> code for the CSV.
STATUS_CODES = {
    "waiting for first command": 0,
    "tracking-policy-step-reference": 1,
    "tracking-policy-step-position": 2,
    "tracking-policy-step-position (acceleration limited)": 3,
    "tracking-physics-step-reference": 4,
}

IDLE, SETTLING, STARTING, RUNNING, RETURNING, DONE = range(6)


def _text(value) -> str:
    """String parameter; launch placeholders auto / none / - mean empty."""
    text = str(value).strip()
    return "" if text.lower() in ("auto", "none", "-") else text


def _float_list(value, name: str) -> tuple[float, ...]:
    values = np.asarray(value, dtype=np.float64)
    if values.ndim != 1 or values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be a non-empty list of finite numbers")
    return tuple(float(v) for v in values)


class FR3CommandResponseProbe(Node):
    def __init__(self):
        super().__init__("fr3_command_response_probe")
        p = self.declare_parameter
        # Lists of numbers accept ints or floats ("[1, 2]" or "[1.0, 2.0]").
        numbers = ParameterDescriptor(dynamic_typing=True)

        # --- robot / controller ---
        p("robot", "left")                 # left | right | dual (= controller robot_side)
        p("excite_arm", "")                # dual only: left | right (default: left)
        p("command_interface", "effort")   # expected controller interface: effort | position
        p("joint_states_topic", "")        # default: /<robot>_fr3/joint_states or /dual_fr3/joint_states
        p("command_topic", "/policy_joint_command")
        p("control_action", "/fr3_policy_control")
        p("command_timeout_s", 0.15)
        p("max_policy_target_delta_rad", 0.20)

        # --- run plan ---
        p("tests", ["step", "sine", "chirp"])  # step|sine|chirp|press per joint, hold|replay whole arm
        p("joints", [1, 2, 3, 4, 5, 6, 7])      # 1-based joint numbers of the excited arm
        p("target_rate_hz", 20.0)               # command rate (policy: 20 Hz ZOH)
        p("pre_s", 1.0)
        p("wrist_scale", 1.0)
        p("step_amplitudes_rad", [0.02, 0.05, 0.10], numbers)
        p("step_hold_s", 1.5)
        p("step_rest_s", 1.5)
        p("sine_frequencies_hz", [0.5, 1.0, 2.0, 3.0], numbers)
        p("sine_amplitude_rad", 0.05)
        p("sine_max_speed_rad_s", 0.5)
        p("sine_steady_cycles", 4)
        p("sine_ramp_cycles", 1)
        p("sine_rest_s", 1.0)
        p("chirp_f0_hz", 0.2)
        p("chirp_f1_hz", 5.0)
        p("chirp_duration_s", 30.0)
        p("chirp_amplitude_rad", 0.03)
        p("chirp_max_speed_rad_s", 0.5)
        p("chirp_taper_s", 1.0)
        p("chirp_rest_s", 1.0)
        p("press_levels_rad", [0.002, 0.005, 0.01], numbers)
        p("press_direction", 1.0)
        p("press_hold_s", 2.0)
        p("press_rest_s", 1.5)
        p("press_max_dtau_nm", 10.0)            # torque guard vs. the trial-start effort
        p("hold_duration_s", 3.0)
        p("replay_file", "")
        p("replay_start_tolerance_rad", 0.05)

        # --- settling / safety ---
        p("initial_settle_s", 0.75)
        p("home_position_tolerance_rad", 0.01)
        p("home_velocity_tolerance_rad_s", 0.02)
        p("home_settle_s", 0.75)
        p("home_timeout_s", 30.0)
        p("joint_state_timeout_s", 0.2)
        p("max_live_excursion_rad", 0.12)
        p("max_live_speed_rad_s", 0.8)

        # --- logging ---
        p("log_root", "")
        p("label", "")                     # e.g. torque_default / position_default
        p("notes", "")                     # collision thresholds, EE mass, stiffness, pose ...
        p("dry_run", True)
        p("auto_start", False)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.robot = str(get("robot")).strip().lower()
        if self.robot not in ("left", "right", "dual"):
            raise ValueError("robot must be left, right or dual")
        excite_arm = _text(get("excite_arm")).lower() or (
            "left" if self.robot == "dual" else self.robot)
        if self.robot != "dual" and excite_arm != self.robot:
            raise ValueError("single-arm run: excite_arm must equal robot")
        if excite_arm not in ("left", "right"):
            raise ValueError("excite_arm must be left or right")
        self.excite_arm = excite_arm
        self.command_interface = str(get("command_interface")).strip().lower()
        if self.command_interface not in ("effort", "position"):
            raise ValueError("command_interface must be effort or position")

        sides = ["left", "right"] if self.robot == "dual" else [self.robot]
        self.joint_names = [f"{s}_fr3_joint{i}" for s in sides for i in range(1, 8)]
        self.n = len(self.joint_names)
        self.lower = np.tile(FR3_LOWER, len(sides)) + JOINT_LIMIT_MARGIN_RAD
        self.upper = np.tile(FR3_UPPER, len(sides)) - JOINT_LIMIT_MARGIN_RAD
        arm_offset = 7 if (self.robot == "dual" and excite_arm == "right") else 0
        self.excited = list(range(arm_offset, arm_offset + 7))

        self.max_delta = float(get("max_policy_target_delta_rad"))
        self.rate_hz = float(get("target_rate_hz"))
        if not (0.0 < self.rate_hz <= 200.0):
            raise ValueError("target_rate_hz must be in (0, 200]")
        if 1.0 / self.rate_hz >= float(get("command_timeout_s")):
            raise ValueError("command period must be shorter than command_timeout_s")
        self.dry_run = bool(get("dry_run"))
        self.press_max_dtau = float(get("press_max_dtau_nm"))

        cfg = ProfileConfig(
            pre_s=float(get("pre_s")),
            wrist_scale=float(get("wrist_scale")),
            step_amplitudes=_float_list(get("step_amplitudes_rad"), "step_amplitudes_rad"),
            step_hold_s=float(get("step_hold_s")),
            step_rest_s=float(get("step_rest_s")),
            sine_frequencies=_float_list(get("sine_frequencies_hz"), "sine_frequencies_hz"),
            sine_amplitude=float(get("sine_amplitude_rad")),
            sine_max_speed=float(get("sine_max_speed_rad_s")),
            sine_steady_cycles=int(get("sine_steady_cycles")),
            sine_ramp_cycles=int(get("sine_ramp_cycles")),
            sine_rest_s=float(get("sine_rest_s")),
            chirp_f0=float(get("chirp_f0_hz")),
            chirp_f1=float(get("chirp_f1_hz")),
            chirp_duration_s=float(get("chirp_duration_s")),
            chirp_amplitude=float(get("chirp_amplitude_rad")),
            chirp_max_speed=float(get("chirp_max_speed_rad_s")),
            chirp_taper_s=float(get("chirp_taper_s")),
            chirp_rest_s=float(get("chirp_rest_s")),
            press_levels=_float_list(get("press_levels_rad"), "press_levels_rad"),
            press_direction=float(get("press_direction")),
            press_hold_s=float(get("press_hold_s")),
            press_rest_s=float(get("press_rest_s")),
            hold_duration_s=float(get("hold_duration_s")),
        )
        self.tests = [str(t).strip().lower() for t in get("tests")]
        unknown = [t for t in self.tests if t not in ALL_TESTS]
        if not self.tests or unknown:
            raise ValueError(f"tests must be a non-empty subset of {ALL_TESTS}; got {unknown}")
        joint_numbers = [int(j) for j in get("joints")]
        self.replay = None
        replay_names: list[str] = []
        if "replay" in self.tests:
            path = _text(get("replay_file"))
            if not path:
                raise ValueError("tests contains replay but replay_file is empty")
            self.replay = load_replay(path)
            replay_names = [self.joint_names[j] for j in self.excited]
        self.trials: list[Trial] = build_trials(
            self.tests, joint_numbers, arm_offset, self.n, cfg, self.joint_names,
            self.replay, replay_names,
        )
        self.profile_cfg = cfg
        self._check_static_limits()

        # --- state ---
        self.q = np.full(self.n, np.nan)
        self.qd = np.full(self.n, np.nan)
        self.tau = np.full(self.n, np.nan)
        self.rx_ns = np.zeros(self.n, dtype=np.int64)
        self.stamp_ns = 0
        self.state = IDLE
        self.q0 = np.full(self.n, np.nan)
        self.trial_idx = -1
        self.trial_start_ns = 0
        self.stable_since_ns = 0
        self.return_start_ns = 0
        self.settle_request_ns = 0
        self.sequence = 0
        self.q_cmd = np.full(self.n, np.nan)
        self.cmd_ns = 0
        self.phase = "idle"
        self.tau_base = np.full(self.n, np.nan)
        self.press_tripped = False
        self.goal_handle = None
        self.feedback_status = ""
        self.feedback_age = math.nan
        self.feedback_error = math.nan
        self.mode_checked = False
        self.run_dir: Optional[Path] = None
        self.metadata: dict = {}
        self.trial_results: list[dict] = []
        self._csv_file = None
        self._csv = None
        self._rows = 0

        topic = _text(get("joint_states_topic")) or (
            "/dual_fr3/joint_states" if self.robot == "dual" else f"/{self.robot}_fr3/joint_states")
        self.create_subscription(JointState, topic, self._joint_state_cb, 50)
        self.command_pub = self.create_publisher(
            PolicyJointCommand, str(get("command_topic")),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.action_client = ActionClient(self, RunPolicyControl, str(get("control_action")))
        self.create_service(Trigger, "~/start_probe", self._start_srv)
        self.create_service(Trigger, "~/stop_probe", self._stop_srv)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        total = sum(t.duration for t in self.trials)
        self.get_logger().info(
            f"command-response probe: robot={self.robot} excite={self.excite_arm} "
            f"interface={self.command_interface} tests={self.tests} joints={joint_numbers} "
            f"trials={len(self.trials)} (~{total / 60.0:.1f} min of profile + settling) "
            f"rate={self.rate_hz:.0f} Hz dry_run={self.dry_run} joint_states={topic}")

    # ------------------------------------------------------------ validation
    def _check_static_limits(self):
        """Refuse live trials whose reference could exceed the live guards."""
        if self.dry_run:
            return
        excursion = float(self.get_parameter("max_live_excursion_rad").value)
        speed = float(self.get_parameter("max_live_speed_rad_s").value)
        for trial in self.trials:
            peak = float(np.max(trial.max_abs_delta()))
            if trial.kind != "replay" and peak > excursion + 1e-9:
                raise ValueError(f"{trial.name}: |delta| {peak:.3f} > max_live_excursion_rad {excursion:.3f}")
            if trial.speed_bound() > speed + 1e-9:
                raise ValueError(f"{trial.name}: speed bound {trial.speed_bound():.2f} > max_live_speed_rad_s {speed:.2f}")
            if peak > self.max_delta:
                raise ValueError(f"{trial.name}: |delta| {peak:.3f} > max_policy_target_delta_rad {self.max_delta:.3f}")

    def _check_trials_at_q0(self) -> tuple[bool, str]:
        for trial in self.trials:
            if trial.kind == "replay":
                lo = self.q0 + np.min(trial.replay, axis=0)
                hi = self.q0 + np.max(trial.replay, axis=0)
                # The replayed deltas are relative to the recorded start pose, so
                # the robot must start there (move_to_joint printed by the converter).
                tol = float(self.get_parameter("replay_start_tolerance_rad").value)
                idx = list(trial.replay_joints)
                cols = [self.replay.joint_names.index(self.joint_names[i]) for i in idx]
                err = np.abs(self.q0[idx] - self.replay.q_start[cols])
                if np.any(err > tol) and not self.dry_run:
                    worst = idx[int(np.argmax(err))]
                    return False, (f"replay start pose differs from q0 by {np.max(err):.3f} rad "
                                   f"({self.joint_names[worst]}); move to the replay start pose first")
            else:
                bound = trial.max_abs_delta()
                lo, hi = self.q0 - bound, self.q0 + bound
            bad = np.flatnonzero((lo < self.lower) | (hi > self.upper))
            if bad.size:
                return False, f"{trial.name}: joint {self.joint_names[int(bad[0])]} would leave the safe band"
        return True, "ok"

    # ------------------------------------------------------------------ state
    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _joint_state_cb(self, msg: JointState):
        now = self._now_ns()
        index = {name: i for i, name in enumerate(msg.name)}
        for k, name in enumerate(self.joint_names):
            i = index.get(name)
            if i is None or i >= len(msg.position) or not math.isfinite(msg.position[i]):
                continue
            self.q[k] = msg.position[i]
            if i < len(msg.velocity):
                self.qd[k] = msg.velocity[i]
            if i < len(msg.effort):
                self.tau[k] = msg.effort[i]
            self.rx_ns[k] = now
        self.stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._csv is not None and self.state in (RUNNING, RETURNING):
            self._log_row(now)

    def _inputs_ready(self) -> tuple[bool, str]:
        if not (np.all(np.isfinite(self.q)) and np.all(np.isfinite(self.qd))):
            return False, "waiting for joint positions and velocities"
        timeout = float(self.get_parameter("joint_state_timeout_s").value)
        if np.any((self._now_ns() - self.rx_ns) * 1e-9 > timeout):
            return False, "joint state is stale"
        return True, "ready"

    def _stationary(self, check_position: bool) -> bool:
        vel_ok = np.all(np.abs(self.qd) <= float(self.get_parameter("home_velocity_tolerance_rad_s").value))
        if not check_position:
            return bool(vel_ok)
        pos_ok = np.all(np.abs(self.q - self.q0) <= float(self.get_parameter("home_position_tolerance_rad").value))
        return bool(vel_ok and pos_ok)

    # ------------------------------------------------------------------- loop
    def _tick(self):
        if self.state in (IDLE, DONE):
            if (self.state == IDLE and self.get_parameter("auto_start").value
                    and not getattr(self, "_auto_started", False)):
                self._auto_started = True
                ok, message = self._request_start()
                if not ok:
                    self.get_logger().warn(f"auto_start failed: {message}")
            return
        ready, message = self._inputs_ready()
        if not ready:
            self._abort(f"input fault: {message}")
            return
        now = self._now_ns()
        if self.state == SETTLING:
            self._tick_settling(now)
        elif self.state == STARTING:
            pass  # waiting for the goal response
        elif self.state == RUNNING:
            self._tick_running(now)
        elif self.state == RETURNING:
            self._tick_returning(now)

    def _tick_settling(self, now: int):
        if (now - self.settle_request_ns) * 1e-9 > float(self.get_parameter("home_timeout_s").value):
            self._abort("timed out waiting for a stationary start pose")
            return
        if not self._stationary(check_position=False):
            self.stable_since_ns = 0
            return
        if self.stable_since_ns == 0:
            self.stable_since_ns = now
            return
        if (now - self.stable_since_ns) * 1e-9 < float(self.get_parameter("initial_settle_s").value):
            return
        self.q0 = self.q.copy()
        if np.any((self.q0 < self.lower) | (self.q0 > self.upper)):
            self._abort("start pose is outside the joint safe band")
            return
        ok, message = self._check_trials_at_q0()
        if not ok:
            self._abort(message)
            return
        self._open_run()
        self.get_logger().info("q0 latched: " + np.array2string(self.q0, precision=4, separator=", "))
        if self.dry_run:
            self._begin_trial(0, now)
            return
        if not self.action_client.server_is_ready():
            self._abort("fr3_policy_control action server is not ready")
            return
        goal = RunPolicyControl.Goal()
        goal.robot_name = self.robot
        goal.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        goal.max_duration_s = 0.0
        goal.max_policy_target_delta_rad = self.max_delta
        goal.max_actuator_step_rad = 0.001
        goal.joint_velocity_scale = 0.10
        goal.joint_acceleration_scale = 0.20
        goal.isaac_relative_control = True
        goal.relative_target_refresh_hz = 1000.0
        goal.control_gripper = False
        self.state = STARTING
        future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _begin_trial(self, index: int, now: int):
        self._close_csv()
        if index >= len(self.trials):
            self.get_logger().info("all trials done; q0 is stable")
            self._finish("completed")
            return
        self.trial_idx = index
        self.trial_start_ns = now
        self.tau_base = self.tau.copy()
        self.press_tripped = False
        self.stable_since_ns = 0
        self.phase = "pre"
        self.state = RUNNING
        trial = self.trials[index]
        self._open_trial_csv(trial)
        self.get_logger().info(f"[{index + 1}/{len(self.trials)}] {trial.name}: {trial.duration:.1f} s")

    def _tick_running(self, now: int):
        trial = self.trials[self.trial_idx]
        t = (now - self.trial_start_ns) * 1e-9
        delta, phase = trial.evaluate(t)
        if trial.kind == "press" and trial.joint >= 0 and not self.press_tripped:
            dtau = abs(self.tau[trial.joint] - self.tau_base[trial.joint])
            if math.isfinite(dtau) and dtau > self.press_max_dtau:
                self.press_tripped = True
                self.get_logger().warn(
                    f"press guard: |dtau| {dtau:.1f} Nm > {self.press_max_dtau:.1f} Nm at {phase}; back to q0")
        if self.press_tripped:
            delta, phase = np.zeros(self.n), "press_tripped"
        target = self.q0 + delta
        if not self._send(target, phase, now):
            return
        if t >= trial.duration:
            self.state = RETURNING
            self.return_start_ns = now
            self.stable_since_ns = 0

    def _tick_returning(self, now: int):
        if not self._send(self.q0.copy(), "return", now):
            return
        if self._stationary(check_position=True):
            if self.stable_since_ns == 0:
                self.stable_since_ns = now
            elif (now - self.stable_since_ns) * 1e-9 >= float(self.get_parameter("home_settle_s").value):
                trial = self.trials[self.trial_idx]
                self.trial_results.append({
                    "trial": trial.name, "status": "ok",
                    "press_tripped": self.press_tripped, "rows": self._rows})
                self._begin_trial(self.trial_idx + 1, now)
                return
        else:
            self.stable_since_ns = 0
        if (now - self.return_start_ns) * 1e-9 > float(self.get_parameter("home_timeout_s").value):
            self._abort("timed out waiting for the q0 return to settle")

    def _send(self, target: np.ndarray, phase: str, now: int) -> bool:
        if np.any((target < self.lower) | (target > self.upper)):
            self._abort(f"target outside the safe band in {phase}")
            return False
        gap = np.abs(target - self.q)
        if np.any(gap > self.max_delta):
            worst = int(np.argmax(gap))
            self._abort(f"|target - q| {gap[worst]:.3f} rad > max_policy_target_delta_rad on "
                        f"{self.joint_names[worst]} in {phase}")
            return False
        self.q_cmd = target.copy()
        self.cmd_ns = now
        self.phase = phase
        if self.dry_run:
            return True
        self.sequence += 1
        msg = PolicyJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "probe"
        msg.sequence = self.sequence
        msg.robot_name = self.robot
        msg.target_positions = target.tolist()
        msg.relative_position_offsets = False
        msg.gripper_action = 0.0
        self.command_pub.publish(msg)
        return True

    # ----------------------------------------------------------------- action
    def _start_srv(self, _request, response):
        response.success, response.message = self._request_start()
        return response

    def _request_start(self) -> tuple[bool, str]:
        if self.state not in (IDLE, DONE):
            return False, "probe already running"
        ready, message = self._inputs_ready()
        if not ready:
            return False, message
        self.trial_results = []
        self.sequence = 0
        self.mode_checked = False
        self.feedback_status = ""
        self.state = SETTLING
        self.settle_request_ns = self._now_ns()
        self.stable_since_ns = 0
        return True, "waiting for a stationary start pose, then latching q0"

    def _goal_response_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("fr3_policy_control rejected the probe goal")
            self.state = DONE
            self._write_metadata(status="goal rejected")
            return
        if self.state != STARTING:  # stopped while the goal was in flight
            handle.cancel_goal_async()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)
        self._begin_trial(0, self._now_ns())

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.feedback_status = fb.status
        self.feedback_age = fb.command_age_s
        self.feedback_error = fb.max_joint_error
        if not self.mode_checked and fb.status.startswith("tracking"):
            self.mode_checked = True
            is_position = "position" in fb.status
            expected_position = self.command_interface == "position"
            self.metadata["controller_status"] = fb.status
            if is_position != expected_position or "physics-step" in fb.status:
                self._abort(f"controller reports '{fb.status}' but command_interface={self.command_interface}")

    def _result_cb(self, future):
        wrapped = future.result()
        message = "" if wrapped is None else f"success={wrapped.result.success}, '{wrapped.result.message}'"
        self.goal_handle = None
        if self.state in (RUNNING, RETURNING, STARTING):
            self.get_logger().error(f"controller ended the goal during the run: {message}")
            self.trial_results.append({"trial": self._trial_name(), "status": f"controller ended: {message}"})
            self.state = DONE
            self._close_csv()
            self._write_metadata(status="controller ended the goal")
        else:
            self.get_logger().info(f"controller finished: {message}")
            self.metadata["controller_result"] = message
            self._write_metadata()

    def _stop_srv(self, _request, response):
        if self.state in (IDLE, DONE):
            response.success, response.message = False, "probe is not running"
            return response
        self._abort("stop requested")
        response.success, response.message = True, "probe stopped"
        return response

    def _trial_name(self) -> str:
        return self.trials[self.trial_idx].name if 0 <= self.trial_idx < len(self.trials) else ""

    def _abort(self, reason: str):
        self.get_logger().error(f"probe abort: {reason}")
        if self.state in (RUNNING, RETURNING):
            self.trial_results.append({"trial": self._trial_name(), "status": f"aborted: {reason}"})
        self._finish(f"aborted: {reason}")

    def _finish(self, status: str):
        self.state = DONE
        self._close_csv()
        if self.goal_handle is not None:
            # Position mode: the controller brakes the tracker to rest before it
            # returns the result; effort mode returns at once.
            self.goal_handle.cancel_goal_async()
        self._write_metadata(status=status)

    # ---------------------------------------------------------------- logging
    def _open_run(self):
        root_param = _text(self.get_parameter("log_root").value)
        root = Path(root_param).expanduser() if root_param else (
            Path(__file__).resolve().parents[2] / "data" / "command_response")
        label = _text(self.get_parameter("label").value) or self.command_interface
        stamp = time.strftime("%y%m%d_%H%M%S")
        self.run_dir = root / f"{stamp}_{self.robot}_{label}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        params = {name: self.get_parameter(name).value for name in sorted(self._parameters)
                  if name != "use_sim_time"}
        self.metadata = {
            "schema": "fr3_command_response_probe/1",
            "started_local": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "host": socket.gethostname(),
            "robot": self.robot,
            "excite_arm": self.excite_arm,
            "command_interface": self.command_interface,
            "dry_run": self.dry_run,
            "rate_hz": self.rate_hz,
            "joint_names": self.joint_names,
            "q0": self.q0.tolist(),
            "parameters": {k: (list(v) if isinstance(v, (list, tuple)) else v) for k, v in params.items()},
            "replay_source": self.replay.source if self.replay is not None else "",
            "trials": [
                {"index": i, "name": t.name, "kind": t.kind, "joint": t.joint,
                 "joint_name": self.joint_names[t.joint] if t.joint >= 0 else "",
                 "duration_s": t.duration, "segments": t.segment_table(),
                 "csv": self._trial_csv_name(i, t)}
                for i, t in enumerate(self.trials)
            ],
        }
        self._write_metadata(status="running")
        self.get_logger().info(f"logging to {self.run_dir}")

    def _write_metadata(self, status: Optional[str] = None):
        if self.run_dir is None:
            return
        if status is not None:
            self.metadata["status"] = status
        self.metadata["results"] = self.trial_results
        self.metadata["finished_local"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        (self.run_dir / "metadata.json").write_text(json.dumps(self.metadata, indent=2))

    def _trial_csv_name(self, index: int, trial: Trial) -> str:
        who = self.joint_names[trial.joint] if trial.joint >= 0 else f"{self.excite_arm}_arm"
        return f"trial{index:02d}_{trial.kind}_{who}.csv"

    def _open_trial_csv(self, trial: Trial):
        path = self.run_dir / self._trial_csv_name(self.trial_idx, trial)
        self._csv_file = open(path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        names = self.joint_names
        self._csv.writerow(
            ["t_s", "stamp_t_s", "phase", "cmd_seq", "cmd_t_s", "fb_status", "fb_cmd_age_s",
             "ref_active"]
            + [f"q_cmd_{n}" for n in names] + [f"q_{n}" for n in names]
            + [f"qd_{n}" for n in names] + [f"tau_{n}" for n in names])
        self._rows = 0

    def _log_row(self, now: int):
        trial = self.trials[self.trial_idx]
        t = (now - self.trial_start_ns) * 1e-9
        ref = math.nan
        if trial.joint >= 0 and self.state == RUNNING:
            ref = float(self.q0[trial.joint] + trial.evaluate(t)[0][trial.joint])
        stamp_t = (self.stamp_ns - self.trial_start_ns) * 1e-9 if self.stamp_ns else math.nan
        row = [f"{t:.6f}", f"{stamp_t:.6f}", self.phase, self.sequence,
               f"{(self.cmd_ns - self.trial_start_ns) * 1e-9:.6f}",
               STATUS_CODES.get(self.feedback_status, -1), f"{self.feedback_age:.4f}", f"{ref:.6f}"]
        row += [f"{v:.6f}" for v in self.q_cmd] + [f"{v:.6f}" for v in self.q]
        row += [f"{v:.6f}" for v in self.qd] + [f"{v:.4f}" for v in self.tau]
        self._csv.writerow(row)
        self._rows += 1

    def _close_csv(self):
        if self._csv_file is not None:
            self._csv_file.close()
        self._csv_file = None
        self._csv = None


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = FR3CommandResponseProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node._close_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
