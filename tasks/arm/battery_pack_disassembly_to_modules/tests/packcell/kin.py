"""Kinematics of the cell's robot (Universal Robots UR20) and the small rotation toolbox the cell uses.

NumPy only. The joint frames are the ones of the robot's USD description (Isaac Sim asset library,
`Isaac/Robots/UniversalRobots/ur20/ur20.usd`): every joint turns about the z axis of its own frame, and `JOINTS`
gives that frame in the parent link. `flange(q)` is the pose of `wrist_3_link` (the tool flange, z out of the
flange) in the robot's base frame, which is the cell's world frame. The cell's controller turns Cartesian targets
into joint targets with `ik_step` (damped least squares on the geometric Jacobian), on the commanded joint state.
"""

import math

import numpy as np

# (translation, quaternion w x y z) of each joint frame in its parent link, as authored in the USD file
JOINTS = (
    ((0.0, 0.0, 0.2363), (0.0, 0.0, 0.0, 1.0)),                       # shoulder_pan
    ((0.0, 0.0, 0.0), (0.70710678, 0.70710678, 0.0, 0.0)),            # shoulder_lift
    ((-0.862, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),                       # elbow
    ((-0.7287, 0.0, 0.201), (1.0, 0.0, 0.0, 0.0)),                    # wrist_1
    ((0.0, -0.1593, 0.0), (0.70710678, 0.70710678, 0.0, 0.0)),        # wrist_2
    ((0.0, 0.1543, 0.0), (0.70710678, -0.70710678, 0.0, 0.0)),        # wrist_3
)
JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
               "wrist_3_joint")
Q_LIMITS = np.array([[-2 * math.pi, 2 * math.pi]] * 2 + [[-math.pi, math.pi]] + [[-2 * math.pi, 2 * math.pi]] * 3)


def quat_R(q):
    """Rotation matrix of a quaternion (w, x, y, z)."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    """Quaternion (w, x, y, z) of a rotation matrix."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    return np.array(q)


def rotvec_R(v):
    """Rotation matrix of a rotation vector (axis times angle, rad)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def R_rotvec(R):
    """Rotation vector (axis times angle, angle in [0, pi]) of a rotation matrix."""
    c = max(-1.0, min(1.0, (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if math.pi - a < 1e-6:                      # near pi: the axis from the symmetric part
        M = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        k = M[:, i] / math.sqrt(max(M[i, i], 1e-12))
        return k * a
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a))
    return k * a


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def inv(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


_JT = tuple(pose(quat_R(q), np.array(p)) for p, q in JOINTS)


def link_frames(q):
    """The six link frames (4x4, base frame) for joint angles q (rad)."""
    T = np.eye(4)
    out = []
    for i in range(6):
        T = T @ _JT[i] @ pose(Rz(float(q[i])), np.zeros(3))
        out.append(T)
    return out


def flange(q):
    """Pose of the tool flange (wrist_3_link) in the base frame."""
    return link_frames(q)[5]


def jacobian(q, tool=None):
    """Geometric Jacobian (6x6, base frame: linear rows, then angular) of the point `tool` (a 4x4 pose in the
    flange frame, default the flange origin)."""
    frames = link_frames(q)
    T = frames[5] if tool is None else frames[5] @ tool
    p = T[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = frames[i][:3, 2]
        J[:3, i] = np.cross(z, p - frames[i][:3, 3])
        J[3:, i] = z
    return J


def pose_error(T_target, T):
    """6-vector taking T to T_target: translation difference and the rotation vector of R_target R^T (base frame)."""
    return np.concatenate([T_target[:3, 3] - T[:3, 3], R_rotvec(T_target[:3, :3] @ T[:3, :3].T)])


def ik_step(q, T_target, tool=None, damping=0.02, max_dq=0.35, iters=4, w_rot=0.3):
    """A few damped-least-squares steps from q toward the tool pose T_target; returns (q_new, residual 6-vector).
    `w_rot` weighs rotation error (rad) against position error (m)."""
    q = np.array(q, dtype=float)
    W = np.diag([1.0, 1.0, 1.0, w_rot, w_rot, w_rot])
    err = np.zeros(6)
    for _ in range(iters):
        T = flange(q) if tool is None else flange(q) @ tool
        err = pose_error(T_target, T)
        if np.linalg.norm(err[:3]) < 1e-6 and np.linalg.norm(err[3:]) < 1e-6:
            break
        J = W @ jacobian(q, tool)
        dq = J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(6), W @ err)
        n = float(np.max(np.abs(dq)))
        if n > max_dq:
            dq *= max_dq / n
        q = np.clip(q + dq, Q_LIMITS[:, 0], Q_LIMITS[:, 1])
    T = flange(q) if tool is None else flange(q) @ tool
    return q, pose_error(T_target, T)
