"""Estimate cabinet task book pose and door angle from robot_state_publisher TF.

Book pose is reconstructed from right link7 -> UMI TCP -> book fixed grasp transform.
Door angle is inferred from the left UMI TCP orbiting a calibrated vertical hinge.
No camera is used. Door angle is withheld until the hinge and closed TCP calibration
are provided, so an uncalibrated value cannot silently reach the policy.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
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
        self.declare_parameter("publish_rate_hz", 50.0)

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
            f"FK estimator ready: TF {self.base_frame} <- ({self.left_frame}, {self.right_frame}); "
            "publishing book pose and calibrated door angle."
        )

    def _optional_xyz(self, name):
        xyz = np.asarray(self.get_parameter(name).value, dtype=float)
        if xyz.shape == (3,) and np.isnan(xyz).all():
            return None
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError(f"{name} must be unset or a finite xyz vector")
        return xyz

    def _transform(self, frame):
        msg = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        t, q = msg.transform.translation, msg.transform.rotation
        return np.array([t.x, t.y, t.z]), _qnormalize([q.x, q.y, q.z, q.w])

    def _tcp_pose(self, frame):
        link_p, link_q = self._transform(frame)
        tcp_p = link_p + _qrotate(link_q, self.tcp_p)
        tcp_q = _qnormalize(_qmul(link_q, self.tcp_q))
        return tcp_p, tcp_q

    def _tick(self):
        try:
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
