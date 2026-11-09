"""FR3 forward kinematics (ROS-free), shared by the cabinet FK estimator and the cube pick-and-place node."""

from __future__ import annotations

import math

import numpy as np

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
