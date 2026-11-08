"""Estimate cabinet task book pose and door angle from the arm joint states (forward kinematics).

Book pose is reconstructed from right link7 -> UMI TCP -> book fixed grasp transform.
Door angle is inferred from the left UMI TCP orbiting a calibrated vertical hinge.
No camera is used. Door angle is withheld until the hinge and closed TCP calibration
are provided, so an uncalibrated value cannot silently reach the policy.

link7 poses are computed from ``/dual_fr3/joint_states`` (ros2_control, 1 kHz) with the FR3 DH model; only the
fixed base -> link0 mounts are read from TF (URDF, once). The robot_state_publisher TF of the moving links is
only ~15 Hz on the real robot (2026-11-07), which made the door angle lag ~4 deg at policy speed. The FK result
is compared with that TF once while the arms are still (``fk_tf_check_tolerance_m``).
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


def _qnormalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or n < 1e-9:
        raise ValueError("quaternion must be a finite nonzero xyzw vector")
    return q / n


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _qrotate(q, v):
    q = _qnormalize(q)
    vq = np.array([v[0], v[1], v[2], 0.0])
    qi = np.array([-q[0], -q[1], -q[2], q[3]])
    return _qmul(_qmul(q, vq), qi)[:3]


# FR3 modified DH (Craig): a, d, alpha per joint (franka FR3 kinematics; link7 frame = DH frame 7)
_FR3_DH = ((0.0, 0.333, 0.0), (0.0, 0.0, -math.pi / 2), (0.0, 0.316, math.pi / 2), (0.0825, 0.0, math.pi / 2),
           (-0.0825, 0.384, -math.pi / 2), (0.0, 0.0, math.pi / 2), (0.088, 0.0, math.pi / 2))


def fr3_link7_in_link0(q):
    """4x4 pose of link7 in link0 for the seven FR3 joint angles."""
    T = np.eye(4)
    for (a, d, alpha), theta in zip(_FR3_DH, q):
        ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
        T = T @ np.array([[ct, -st, 0.0, a], [st * ca, ct * ca, -sa, -d * sa], [st * sa, ct * sa, ca, d * ca],
                          [0.0, 0.0, 0.0, 1.0]])
    return T


def _quat_from_matrix(R):
    """xyzw quaternion of a rotation matrix (Shepperd: pivot on the largest component, stable near 180 deg)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > max(R[0, 0], R[1, 1], R[2, 2]):
        s = 2.0 * math.sqrt(1.0 + trace)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] >= R[2, 2]:
        s = 2.0 * math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    return _qnormalize(q)


def _matrix_from_quat(q):
    x, y, z, w = _qnormalize(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _inverse_transform(p, q):
    qi = np.array([-q[0], -q[1], -q[2], q[3]])
    return _qrotate(qi, -np.asarray(p, dtype=float)), qi


class CabinetBookInsertFkNode(Node):
    def __init__(self):
        super().__init__("cabinet_book_insert_fk_node")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("left_link7_frame", "left_fr3_link7")
        self.declare_parameter("right_link7_frame", "right_fr3_link7")
        self.declare_parameter("book_pose_topic", "/cabinet_book_insert/book_pose")
        self.declare_parameter("door_angle_topic", "/cabinet_book_insert/door_angle")
        self.declare_parameter("left_tcp_pose_topic", "/cabinet_book_insert/left_tcp_pose")
        # Isaac cabinet task's link7 -> UMI fingertip-centre TCP transform (xyzw).
        self.declare_parameter("umi_tcp_offset_position", [0.0, 0.0, 0.2844])
        self.declare_parameter("umi_tcp_offset_quaternion_xyzw", [0.0, 0.0, -0.3826834, 0.9238795])
        # Training's right-hand grasp: TCP pose expressed in the book frame (xyzw).
        self.declare_parameter("right_tcp_in_book_position", [-0.0608578644, 0.0, -0.0141421356])
        self.declare_parameter("right_tcp_in_book_quaternion_xyzw", [0.9238795, 0.0, 0.3826834, 0.0])
        # Required cabinet calibration in base: hinge xyz and left TCP xyz with door closed. NaN = not set
        # (an empty-list default would declare a byte array and reject the xyz override).
        self.declare_parameter("hinge_position_base", [math.nan] * 3)
        self.declare_parameter("left_tcp_closed_position_base", [math.nan] * 3)
        self.declare_parameter("door_angle_sign", -1.0)  # trained layout: the door opens clockwise from above
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("joint_states_topic", "/dual_fr3/joint_states")
        self.declare_parameter("left_link0_frame", "left_fr3_link0")
        self.declare_parameter("right_link0_frame", "right_fr3_link0")
        self.declare_parameter("joint_state_timeout_s", 0.1)  # no publish from older joints (policy then faults)
        self.declare_parameter("fk_tf_check_tolerance_m", 0.002)

        self.base_frame = str(self.get_parameter("base_frame").value).strip("/")
        self.left_frame = str(self.get_parameter("left_link7_frame").value).strip("/")
        self.right_frame = str(self.get_parameter("right_link7_frame").value).strip("/")
        self.tcp_p = np.asarray(self.get_parameter("umi_tcp_offset_position").value, dtype=float)
        self.tcp_q = _qnormalize(self.get_parameter("umi_tcp_offset_quaternion_xyzw").value)
        self.book_tcp_p = np.asarray(self.get_parameter("right_tcp_in_book_position").value, dtype=float)
        self.book_tcp_q = _qnormalize(self.get_parameter("right_tcp_in_book_quaternion_xyzw").value)
        if self.tcp_p.shape != (3,) or self.book_tcp_p.shape != (3,):
            raise ValueError("tool and grasp translations must be 3-vectors")
        self.hinge_p = self._optional_xyz("hinge_position_base")
        self.closed_tcp_p = self._optional_xyz("left_tcp_closed_position_base")
        self.angle_sign = float(self.get_parameter("door_angle_sign").value)
        if self.angle_sign not in (-1.0, 1.0):
            raise ValueError("door_angle_sign must be +1 or -1")
        if (self.hinge_p is None) != (self.closed_tcp_p is None):
            raise ValueError("set both hinge_position_base and left_tcp_closed_position_base")
        if self.hinge_p is not None and np.linalg.norm((self.closed_tcp_p - self.hinge_p)[:2]) < 0.02:
            raise ValueError("closed TCP calibration is too close to the hinge")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.left_link0_frame = str(self.get_parameter("left_link0_frame").value).strip("/")
        self.right_link0_frame = str(self.get_parameter("right_link0_frame").value).strip("/")
        self.mount = {}  # link7 frame -> 4x4 base <- link0 (fixed, from TF once)
        self.joint_index = {self.left_frame: [f"left_fr3_joint{i}" for i in range(1, 8)],
                            self.right_frame: [f"right_fr3_joint{i}" for i in range(1, 8)]}
        self.q = {frame: np.full(7, np.nan) for frame in self.joint_index}
        self.qd_max = math.inf
        self.joint_time_ns = 0
        self.tf_checked = False
        self.create_subscription(JointState, str(self.get_parameter("joint_states_topic").value),
                                 self._joint_state_callback, 10)
        self.book_pub = self.create_publisher(PoseStamped, str(self.get_parameter("book_pose_topic").value), 10)
        self.door_pub = self.create_publisher(Float64, str(self.get_parameter("door_angle_topic").value), 10)
        self.left_tcp_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("left_tcp_pose_topic").value), 10
        )
        hz = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("publish_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self._last_warn_ns = 0
        if self.hinge_p is None:
            self.get_logger().warning(
                "Door angle disabled: calibrate hinge_position_base and "
                "left_tcp_closed_position_base before using the policy."
            )
        self.get_logger().info(
            f"FK estimator ready: FR3 FK from {self.get_parameter('joint_states_topic').value} "
            f"(mounts from TF {self.base_frame} <- {self.left_link0_frame}, {self.right_link0_frame}), "
            f"{hz:.0f} Hz; publishing book pose and calibrated door angle."
        )

    def _optional_xyz(self, name):
        xyz = np.asarray(self.get_parameter(name).value, dtype=float)
        if xyz.shape == (3,) and np.isnan(xyz).all():
            return None
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError(f"{name} must be unset or a finite xyz vector")
        return xyz

    def _joint_state_callback(self, msg: JointState):
        index = {name: i for i, name in enumerate(msg.name)}
        updated, speed = False, 0.0
        for frame, names in self.joint_index.items():
            if not all(name in index and index[name] < len(msg.position) for name in names):
                continue
            q = np.array([msg.position[index[name]] for name in names], dtype=float)
            if np.isfinite(q).all():
                self.q[frame] = q
                updated = True
            if len(msg.velocity) == len(msg.position):
                speed = max(speed, max(abs(msg.velocity[index[name]]) for name in names))
        if updated:
            self.qd_max = speed
            self.joint_time_ns = self.get_clock().now().nanoseconds

    def _transform(self, frame):
        msg = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        t, q = msg.transform.translation, msg.transform.rotation
        return np.array([t.x, t.y, t.z]), _qnormalize([q.x, q.y, q.z, q.w])

    def _link7_pose(self, frame):
        """base <- link7 from the latest joint state (FK) and the fixed link0 mount."""
        if frame not in self.mount:
            link0 = self.left_link0_frame if frame == self.left_frame else self.right_link0_frame
            p, q = self._transform(link0)
            M = np.eye(4)
            M[:3, :3], M[:3, 3] = _matrix_from_quat(q), p
            self.mount[frame] = M
        if not np.isfinite(self.q[frame]).all():
            raise ValueError(f"waiting for {self.joint_index[frame][0][:-1]}1-7 on the joint state topic")
        T = self.mount[frame] @ fr3_link7_in_link0(self.q[frame])
        return T[:3, 3].copy(), _quat_from_matrix(T[:3, :3])

    def _check_against_tf(self):
        """Once, with the arms still: FK link7 vs robot_state_publisher TF (catches a URDF/DH mismatch)."""
        tolerance = float(self.get_parameter("fk_tf_check_tolerance_m").value)
        errors = [float(np.linalg.norm(self._link7_pose(frame)[0] - self._transform(frame)[0]))
                  for frame in (self.left_frame, self.right_frame)]
        self.tf_checked = True
        message = f"FK vs TF link7 position: left {1e3 * errors[0]:.2f} mm, right {1e3 * errors[1]:.2f} mm"
        if max(errors) > tolerance:
            self.get_logger().error(message + f" (> {1e3 * tolerance:.1f} mm: FK model does not match the URDF)")
        else:
            self.get_logger().info(message)

    def _tcp_pose(self, frame):
        link_p, link_q = self._link7_pose(frame)
        tcp_p = link_p + _qrotate(link_q, self.tcp_p)
        tcp_q = _qnormalize(_qmul(link_q, self.tcp_q))
        return tcp_p, tcp_q

    def _tick(self):
        age_s = (self.get_clock().now().nanoseconds - self.joint_time_ns) * 1e-9
        if age_s > float(self.get_parameter("joint_state_timeout_s").value):
            now = self.get_clock().now().nanoseconds
            if now - self._last_warn_ns > 2_000_000_000:
                self.get_logger().warning(f"Waiting for fresh joint states ({age_s:.2f} s old); nothing published")
                self._last_warn_ns = now
            return
        try:
            if not self.tf_checked and self.qd_max < 0.01:
                self._check_against_tf()
            right_p, right_q = self._tcp_pose(self.right_frame)
            # Invert the training's book->TCP grasp transform to recover book pose.
            tcp_to_book_p, tcp_to_book_q = _inverse_transform(self.book_tcp_p, self.book_tcp_q)
            book_p = right_p + _qrotate(right_q, tcp_to_book_p)
            book_q = _qnormalize(_qmul(right_q, tcp_to_book_q))
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, book_p)
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, book_q)
            self.book_pub.publish(msg)

            left_p, left_q = self._tcp_pose(self.left_frame)
            left_msg = PoseStamped()
            left_msg.header = msg.header
            left_msg.pose.position.x, left_msg.pose.position.y, left_msg.pose.position.z = map(float, left_p)
            left_msg.pose.orientation.x, left_msg.pose.orientation.y, left_msg.pose.orientation.z, left_msg.pose.orientation.w = map(float, left_q)
            self.left_tcp_pub.publish(left_msg)
            if self.hinge_p is not None:
                v0 = self.closed_tcp_p[:2] - self.hinge_p[:2]
                v = left_p[:2] - self.hinge_p[:2]
                angle = self.angle_sign * math.atan2(v0[0] * v[1] - v0[1] * v[0], float(np.dot(v0, v)))
                self.door_pub.publish(Float64(data=float(angle)))
        except (TransformException, ValueError) as exc:
            now = self.get_clock().now().nanoseconds
            if now - self._last_warn_ns > 2_000_000_000:
                self.get_logger().warning(f"Waiting for valid FK transform: {exc}")
                self._last_warn_ns = now


def main(args=None):
    rclpy.init(args=args)
    node = CabinetBookInsertFkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
