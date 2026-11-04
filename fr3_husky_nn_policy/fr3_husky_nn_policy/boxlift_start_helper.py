"""Pick the pre-grasp start row for wherever the box is, and preview the node's start check.

Run it next to a running ppo_boxlift_policy_node (real robot or MuJoCo)::

    ros2 run fr3_husky_nn_policy boxlift_start_helper          # box placed: which row, which arm command
    ros2 run fr3_husky_nn_policy boxlift_start_helper --move   # ... and send the arms there (box removed!)

1. Averages the box pose on the node's ``box_pose_topic`` for ``--seconds`` and applies the node's
   ``box_position_offset_m``, so it sees exactly what the policy would see.
2. Picks the table row (embedded in the node's NPZ) whose box pose is closest to it and prints the
   ``move_to_joint`` command for that row. MoveIt does not know the box: take the box out before moving
   the arms, then put it back on the same spot.
3. With the arms anywhere, runs the node's own start check (``boxlift_core.check_start_conditions`` with
   the node's margins) on the current joints and box, i.e. what ``start_policy`` would answer now.

Model path, box size and margins are read from the running node; ``--node`` picks another node name.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .boxlift_core import (
    canonicalize_box_quat,
    check_start_conditions,
    load_boxlift_policy,
    normalize_quat,
    yaw_from_quat,
)

PARAMETERS = (
    "model_path", "box_size_m", "box_size_tolerance_m", "box_pose_topic", "box_position_offset_m",
    "joint_states_topic", "base_frame", "ready_tolerance_rad", "start_position_margin_m",
    "start_yaw_margin_rad", "rest_z_tolerance_m", "pregrasp_joint_margin_rad", "shadow_mode",
    "goal_z_m", "goal_xy_offset_m",
)


def _value(v):
    """rcl_interfaces ParameterValue -> python value."""
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value,
            8: list(v.double_array_value)}.get(v.type)


def _node_parameters(node, target: str) -> dict:
    client = node.create_client(GetParameters, f"{target}/get_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"{target} is not running (start the box-lift launch first)")
    future = client.call_async(GetParameters.Request(names=list(PARAMETERS)))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        raise RuntimeError(f"no answer from {target}/get_parameters")
    return {name: _value(v) for name, v in zip(PARAMETERS, future.result().values)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--node", default="/ppo_boxlift_policy_node")
    parser.add_argument("--seconds", type=float, default=3.0, help="box pose averaging window")
    parser.add_argument("--move", action="store_true", help="send the arms to the picked row (asks first)")
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("boxlift_start_helper")
    try:
        p = _node_parameters(node, args.node)
        _, manifest = load_boxlift_policy(p["model_path"], p["box_size_m"], p["box_size_tolerance_m"])
        if manifest.pregrasp_joint_position is None:
            print("this checkpoint has no pre-grasp table (start.mode ready_pose); use the ready pose")
            return 1
        offset = np.asarray(p["box_position_offset_m"], dtype=np.float64)

        boxes, stamps, joints = [], [], {}

        def on_box(msg: PoseStamped) -> None:
            if msg.header.frame_id not in ("", p["base_frame"]):
                print(f"box pose is in '{msg.header.frame_id}', expected '{p['base_frame']}'; ignoring")
                return
            q = msg.pose.orientation
            boxes.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          *normalize_quat([q.x, q.y, q.z, q.w])))
            stamps.append(time.monotonic())

        node.create_subscription(PoseStamped, p["box_pose_topic"], on_box,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(JointState, p["joint_states_topic"], lambda m: joints.__setitem__("msg", m), 10)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

        if not boxes:
            print(f"no box pose on {p['box_pose_topic']} in {args.seconds:.0f} s (vision off or marker hidden)")
            return 1
        a = np.asarray(boxes)
        rate = len(a) / args.seconds
        gap_ms = 1000.0 * float(np.max(np.diff(stamps))) if len(stamps) > 1 else float("nan")
        position = a[:, :3].mean(axis=0) + offset
        quat = canonicalize_box_quat(normalize_quat(a[-1, 3:]))
        yaw = float(yaw_from_quat(quat))
        spread = 100.0 * np.ptp(a[:, :2], axis=0)
        print(f"model   {p['model_path'].rsplit('/', 1)[-1]}  ({'SHADOW' if p['shadow_mode'] else 'COMMAND'} mode)")
        print(f"box     {p['box_pose_topic']} {rate:.1f} Hz, longest gap {gap_ms:.0f} ms, x/y spread "
              f"{spread[0]:.1f}/{spread[1]:.1f} cm")
        print(f"        policy input x {position[0]:.3f}  y {position[1]:+.3f}  z {position[2]:+.3f}  "
              f"yaw {math.degrees(yaw):+.1f} deg  (offset {offset.tolist()})")
        goal_xy = position[:2] + np.asarray(p["goal_xy_offset_m"], dtype=np.float64)
        print(f"goal    x {goal_xy[0]:.3f}  y {goal_xy[1]:+.3f}  z {p['goal_z_m']:+.3f}  "
              f"(start + offset {p['goal_xy_offset_m']}; RViz 'Box-lift goal' after start_policy)")
        if rate < 20.0:
            print("        WARNING: below the 20 Hz policy rate; the node faults after "
                  "box_pose_timeout_s without a pose")

        table_q, table_box = manifest.pregrasp_joint_position, manifest.pregrasp_box_pose
        position_tolerance = manifest.start_box_pos_noise + p["start_position_margin_m"]
        yaw_tolerance = manifest.start_box_yaw_noise + p["start_yaw_margin_rad"]
        table_yaw = yaw_from_quat(canonicalize_box_quat(table_box[:, 3:7]))
        position_error = np.max(np.abs(table_box[:, :2] - position[:2]), axis=1)
        yaw_error = np.abs((table_yaw - yaw + np.pi) % (2.0 * np.pi) - np.pi)
        score = np.maximum(position_error / position_tolerance, yaw_error / yaw_tolerance)
        row = int(np.argmin(score))
        print(f"row     {row}: box x {table_box[row, 0]:.3f}  y {table_box[row, 1]:+.3f}  yaw "
              f"{math.degrees(table_yaw[row]):+.1f} deg -> off {100 * position_error[row]:.1f} cm "
              f"(allowed {100 * position_tolerance:.1f}), {math.degrees(yaw_error[row]):.1f} deg "
              f"(allowed {math.degrees(yaw_tolerance):.1f})")
        if score[row] > 1.0:
            print("        WARNING: no table row matches this box; move the box into the trained range "
                  f"x {table_box[:, 0].min():.2f}..{table_box[:, 0].max():.2f}, "
                  f"y {table_box[:, 1].min():+.2f}..{table_box[:, 1].max():+.2f}")
        q = table_q[row]
        command = ["ros2", "run", "fr3_husky_task_manager", "move_to_joint",
                   "--left-target-positions", *[f"{v:.4f}" for v in q[:7]],
                   "--right-target-positions", *[f"{v:.4f}" for v in q[7:]]]
        print("arms    " + " ".join(command))

        if "msg" in joints:
            positions = dict(zip(joints["msg"].name, joints["msg"].position))
            missing = [n for n in manifest.joint_names if n not in positions]
            if missing:
                print(f"start   joint states lack {missing}")
            else:
                current = np.array([positions[n] for n in manifest.joint_names])
                print(f"        arms are {np.max(np.abs(current - q)):.3f} rad from row {row}")
                start = check_start_conditions(
                    current, position, quat, manifest,
                    ready_tolerance_rad=p["ready_tolerance_rad"], start_margin_m=p["start_position_margin_m"],
                    yaw_margin_rad=p["start_yaw_margin_rad"], rest_z_tolerance_m=p["rest_z_tolerance_m"],
                    pregrasp_joint_margin_rad=p["pregrasp_joint_margin_rad"],
                )
                if start.ok:
                    print(f"start   PASS now (row {start.table_row}): start_policy would be accepted")
                else:
                    print("start   FAIL now: " + "; ".join(start.reasons))
        else:
            print(f"start   no joint states on {p['joint_states_topic']}")

        if args.move:
            answer = input("Box OUT of the way and area clear? Send the arms to this row [y/N] ")
            if answer.strip().lower() == "y":
                return subprocess.call(command)
            print("not moved")
        return 0
    except RuntimeError as error:
        print(error)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
