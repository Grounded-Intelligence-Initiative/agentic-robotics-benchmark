"""Kinematics of the palletizing arm (the numbers of task_card.md as code).

fk(q) gives the pad-face centre [x, y, z] in the world frame and the tool yaw for the joint
angles q = [q1, q2, q3, q4]; ik(xyz, yaw) is its inverse (elbow up). The observation's `tcp` is
fk(q) of the joint readings, with the cup's spring extended. The readings are off by each
joint's zero error (see the instruction), so for the readings fk gives where the drawing puts
the pad, not where it is; for the true joint angles it is exact.
"""

import math

A0 = 0.25        # base axis -> shoulder axis (m)
H0 = 0.80        # shoulder axis height (m)
L1 = 1.20        # upper arm (m)
L2 = 1.30        # forearm (m)
A3 = 0.20        # wrist pivot -> tool axis (m)
D_TOOL = 0.55    # wrist pivot -> pad face, cup spring extended (m)
Q_MIN = (-0.80, -0.90, -1.30, -4.20)
Q_MAX = (3.95, 1.45, 2.30, 4.20)


def fk(q):
    q1, q2, q3, q4 = (float(v) for v in q[:4])
    phi = q2 + q3
    r = A0 + L1 * math.sin(q2) + L2 * math.cos(phi) + A3
    z = H0 + L1 * math.cos(q2) - L2 * math.sin(phi) - D_TOOL
    return [r * math.cos(q1), r * math.sin(q1), z], q1 + q4


def ik(xyz, yaw):
    """Joint angles, or None when the point is out of reach. Joint limits are not checked."""
    x, y, z = (float(v) for v in xyz)
    q1 = math.atan2(y, x)
    if q1 < Q_MIN[0]:
        q1 += 2.0 * math.pi
    u = math.hypot(x, y) - A3 - A0
    v = z + D_TOOL - H0
    d2 = u * u + v * v
    d = math.sqrt(d2)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    beta = math.acos(max(-1.0, min(1.0, (L1 * L1 + d2 - L2 * L2) / (2.0 * L1 * d))))
    alpha = math.atan2(v, u) + beta
    fu, fv = u - L1 * math.cos(alpha), v - L1 * math.sin(alpha)
    phi = -math.atan2(fv, fu)
    q2 = math.pi / 2.0 - alpha
    return [q1, q2, phi - q2, yaw - q1]
