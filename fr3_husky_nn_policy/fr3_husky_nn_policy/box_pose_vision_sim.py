"""Vision stand-in for MuJoCo sim-to-sim: ground-truth box pose -> noisy, delayed, dropped box pose.

MuJoCo publishes the exact box pose (``mujoco_object_pose_topic``, here ``/box_pose_gt``). This node
re-publishes it the way the Azure Kinect ArUco node (``/box_tf`` on the real robot) delivers it, so the
policy node sees vision-like input::

    /box_pose_gt (MuJoCo, 30 Hz) -> + bias + white noise -> held for delay_s -> /box_pose (policy)

Defaults come from the real ``/box_tf`` with the box at rest (2026-11-04, 4 s, 120 samples): position std
x 3.4 mm, y 0.7 mm, z 1.9 mm; rotation std about 0.3 deg; capture-to-receive delay about 30 ms.
``header.stamp`` keeps the ground-truth stamp (= capture time), so the policy log's ``box_stamp_age_ms``
shows the injected delay. A dropped frame is not published at all, as the vision node does on a
missed detection; the policy node faults after ``box_pose_timeout_s`` without a pose.
"""

from __future__ import annotations

import collections
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


class BoxPoseVisionSim(Node):
    def __init__(self) -> None:
        super().__init__("box_pose_vision_sim")
        self.declare_parameter("input_topic", "/box_pose_gt")
        self.declare_parameter("output_topic", "/box_pose")
        self.declare_parameter("delay_s", 0.03)
        self.declare_parameter("position_bias_m", [0.0, 0.0, 0.0])
        self.declare_parameter("position_noise_std_m", [0.0034, 0.0007, 0.0019])
        self.declare_parameter("rotation_noise_std_deg", 0.3)
        self.declare_parameter("drop_probability", 0.0)
        self.declare_parameter("seed", 0)

        self.delay_ns = int(float(self.get_parameter("delay_s").value) * 1e9)
        self.bias = np.asarray(self.get_parameter("position_bias_m").value, dtype=np.float64)
        self.pos_std = np.asarray(self.get_parameter("position_noise_std_m").value, dtype=np.float64)
        self.rot_std = math.radians(float(self.get_parameter("rotation_noise_std_deg").value))
        self.drop_p = float(self.get_parameter("drop_probability").value)
        if self.bias.shape != (3,) or self.pos_std.shape != (3,):
            raise ValueError("position_bias_m and position_noise_std_m must have 3 values (x, y, z)")
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))

        self.queue: collections.deque = collections.deque()
        self.received = 0
        self.dropped = 0
        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("input_topic").value,
            self._on_pose,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(0.001, self._release)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value}: "
            f"delay {self.delay_ns * 1e-6:.0f} ms, bias {self.bias.tolist()} m, "
            f"std {self.pos_std.tolist()} m / {math.degrees(self.rot_std):.2f} deg, drop {self.drop_p:.2f}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.received += 1
        if self.rng.random() < self.drop_p:
            self.dropped += 1
            return
        p = msg.pose.position
        position = np.array([p.x, p.y, p.z]) + self.bias + self.rng.normal(0.0, 1.0, 3) * self.pos_std
        q = msg.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        if self.rot_std > 0.0:
            # small rotation about a random axis, applied in the box frame
            angle = self.rng.normal(0.0, self.rot_std)
            axis = self.rng.normal(0.0, 1.0, 3)
            axis /= np.linalg.norm(axis)
            dq = np.concatenate([axis * math.sin(angle / 2.0), [math.cos(angle / 2.0)]])
            quat = _quat_mul(quat, dq)
            quat /= np.linalg.norm(quat)
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x, out.pose.position.y, out.pose.position.z = position.tolist()
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = quat.tolist()
        self.queue.append((time.monotonic_ns() + self.delay_ns, out))

    def _release(self) -> None:
        now = time.monotonic_ns()
        while self.queue and self.queue[0][0] <= now:
            self.publisher.publish(self.queue.popleft()[1])

    def _report(self) -> None:
        if self.received:
            self.get_logger().info(f"received {self.received}, dropped {self.dropped}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxPoseVisionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
