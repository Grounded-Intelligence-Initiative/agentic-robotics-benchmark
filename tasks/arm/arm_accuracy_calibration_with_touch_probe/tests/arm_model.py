"""arm_model.py: the manufacturer's first-order accuracy model of the arm.

A real unit follows this model to first order; defects.md describes the defects a unit may have
beyond it. The submission is predictions (submission_format.py; cell_data.py, "acceptance"), not
this model's parameters.

The tool tip of the arm, in the cell's datum frame, as a function of what the controller knows:
the encoder readings `q`, the direction each joint last moved in `s` (+1 / -1), the link
temperatures `temp` and the mounted tool. Batched over N conditions with NumPy.

Chain: for joint j = 0..5

    T_j = Trans(xyz_j * (1 + kappa_j (temp_j - T0))) Rx(rpy_j[0]) Ry(rpy_j[1]) Rz(rpy_j[2]) Rz(theta_j)

then the flange (Trans z = FLANGE_Z) and the tool's tip. The joint angle is

    theta_j = q_j + b_j s_j / 2
                  + h_j[0] sin q_j + h_j[1] cos q_j + h_j[2] sin 2 q_j + h_j[3] cos 2 q_j
                  + c_j tau_j

with tau_j the gravity torque about joint j, computed with the NOMINAL geometry at the encoder
readings (links' masses below, the tool's mass and centre of gravity from its certificate).

The wrist camera is fixed to the flange. Its frame (OpenCV convention: z along the optical axis,
x to the right of the image, y down the image) in the flange frame is

    Trans(camera xyz) Rx(camera rpy[0]) Ry(camera rpy[1]) Rz(camera rpy[2])

(the same rotation convention as the joints). `camera_pose` gives it in the datum frame.

Parameter vector (N_PARAM = 84): the 78 parameters of the arm (N_KINEMATIC), then the camera's
xyz (3) and rpy (3).
"""
import numpy as np

G = 9.80665
T0 = 20.0
FLANGE_Z = 0.08
D = np.pi / 180.0

NOMINAL_XYZ = np.array([
    [0.00, 0.0, 0.00],
    [0.05, 0.0, 0.38],
    [0.40, 0.0, 0.00],
    [0.08, 0.0, 0.00],
    [0.00, 0.0, 0.30],
    [0.00, 0.0, 0.00],
])
NOMINAL_RPY = np.array([
    [0.0, 0.0, 0.0],
    [-90.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 90.0, 0.0],
    [0.0, -90.0, 0.0],
    [0.0, 90.0, 0.0],
]) * D
# mass (kg) and centre of gravity of the link that turns with joint j, in joint j's frame
LINK_MASS = np.array([8.0, 6.0, 2.5, 3.0, 1.0, 0.5])
LINK_COM = np.array([
    [0.02, 0.0, 0.25],
    [0.20, 0.0, 0.00],
    [0.04, 0.0, 0.00],
    [0.00, 0.0, 0.15],
    [0.00, 0.0, 0.00],
    [0.00, 0.0, 0.04],
])
Q_MIN = np.array([-170, -190, -80, -180, -115, -180]) * D
Q_MAX = np.array([170, 10, 150, 180, 115, 180]) * D

# the camera frame in the flange frame, nominal: 60 mm along the flange's +y, 20 mm along its +z;
# optical axis (camera z) = flange +y, image x = flange +x, image y (down) = flange -z
NOMINAL_CAMERA_XYZ = np.array([0.0, 0.06, 0.02])
NOMINAL_CAMERA_RPY = np.array([-90.0, 0.0, 0.0]) * D

N_KINEMATIC = 78
N_PARAM = 84
SLICES = {"xyz": slice(0, 18), "rpy": slice(18, 36), "b": slice(36, 42), "h": slice(42, 66),
          "c": slice(66, 72), "kappa": slice(72, 78), "camera": slice(78, 84),
          "camera_xyz": slice(78, 81), "camera_rpy": slice(81, 84)}


def nominal():
    p = np.zeros(N_PARAM)
    p[SLICES["xyz"]] = NOMINAL_XYZ.ravel()
    p[SLICES["rpy"]] = NOMINAL_RPY.ravel()
    p[SLICES["camera_xyz"]] = NOMINAL_CAMERA_XYZ
    p[SLICES["camera_rpy"]] = NOMINAL_CAMERA_RPY
    return p


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rz_batch(a):
    c, s = np.cos(a), np.sin(a)
    out = np.zeros(a.shape + (3, 3))
    out[..., 0, 0], out[..., 0, 1], out[..., 1, 0], out[..., 1, 1], out[..., 2, 2] = c, -s, s, c, 1.0
    return out


def _chain(xyz, rpy, theta, scale=None):
    """Frames of the six joints (after their own rotation). xyz, rpy: (6, 3); theta: (N, 6);
    scale: (N, 6) or None. Returns R (N, 6, 3, 3) and o (N, 6, 3)."""
    n = theta.shape[0]
    R = np.broadcast_to(np.eye(3), (n, 3, 3))
    o = np.zeros((n, 3))
    Rs, os_ = [], []
    for j in range(6):
        t = xyz[j][None, :] * (scale[:, j:j + 1] if scale is not None else 1.0)
        o = o + np.einsum("nij,nj->ni", R, t)
        fixed = _rx(rpy[j, 0]) @ _ry(rpy[j, 1]) @ _rz(rpy[j, 2])
        R = R @ fixed @ _rz_batch(theta[:, j])
        Rs.append(R)
        os_.append(o)
    return np.stack(Rs, axis=1), np.stack(os_, axis=1)


def gravity_torques(q, tool_mass, tool_com):
    """Gravity torque about each joint, nominal geometry, at the encoder readings. q: (N, 6)."""
    R, o = _chain(NOMINAL_XYZ, NOMINAL_RPY, q)
    com = o + np.einsum("njab,jb->nja", R, LINK_COM)                    # (N, 6, 3)
    flange = o[:, 5] + R[:, 5, :, 2] * FLANGE_Z
    tool = flange + np.einsum("nab,b->na", R[:, 5], np.asarray(tool_com, float))
    pos = np.concatenate([com, tool[:, None, :]], axis=1)               # (N, 7, 3)
    mass = np.concatenate([LINK_MASS, [tool_mass]])
    force = np.zeros((7, 3))
    force[:, 2] = -mass * G
    tau = np.zeros_like(q)
    for j in range(6):
        arm = pos[:, j:, :] - o[:, j:j + 1, :]
        moment = np.cross(arm, force[None, j:, :]).sum(axis=1)
        tau[:, j] = np.einsum("na,na->n", moment, R[:, j, :, 2])
    return tau


def joint_angles(p, q, s, tau):
    b, h, c = p[SLICES["b"]], p[SLICES["h"]].reshape(6, 4), p[SLICES["c"]]
    return (q + 0.5 * b * s + h[:, 0] * np.sin(q) + h[:, 1] * np.cos(q)
            + h[:, 2] * np.sin(2 * q) + h[:, 3] * np.cos(2 * q) + c * tau)


def tcp(p, q, s, temp, tool, tau=None):
    """Tool tip in the datum frame. q, s, temp: (N, 6); tool: dict(mass, com, tip)."""
    q = np.atleast_2d(q)
    s = np.atleast_2d(s)
    temp = np.atleast_2d(temp)
    if tau is None:
        tau = gravity_torques(q, tool["mass"], tool["com"])
    theta = joint_angles(p, q, s, tau)
    scale = 1.0 + p[SLICES["kappa"]] * (temp - T0)
    R, o = _chain(p[SLICES["xyz"]].reshape(6, 3), p[SLICES["rpy"]].reshape(6, 3), theta, scale)
    tip = np.asarray(tool["tip"], float) + np.array([0.0, 0.0, FLANGE_Z])
    return o[:, 5] + np.einsum("nab,b->na", R[:, 5], tip)


def camera_rotation(rpy):
    """Rotation of the camera frame in the flange frame, Rx(rpy[0]) Ry(rpy[1]) Rz(rpy[2])."""
    return _rx(rpy[0]) @ _ry(rpy[1]) @ _rz(rpy[2])


def camera_pose(p, q, s, temp, tool, tau=None):
    """The camera frame in the datum frame: R (N, 3, 3), whose columns are the camera's x, y, z
    axes, and the camera centre o (N, 3). A point P given in the camera frame is R P + o in the
    datum frame. q, s, temp: (N, 6); tool: dict(mass, com, tip) of the mounted tool (its weight
    bends the arm)."""
    p = np.asarray(p, float)
    if p.shape != (N_PARAM,):
        raise ValueError("camera_pose needs all {} parameters".format(N_PARAM))
    q = np.atleast_2d(q)
    s = np.atleast_2d(s)
    temp = np.atleast_2d(temp)
    if tau is None:
        tau = gravity_torques(q, tool["mass"], tool["com"])
    theta = joint_angles(p, q, s, tau)
    scale = 1.0 + p[SLICES["kappa"]] * (temp - T0)
    R, o = _chain(p[SLICES["xyz"]].reshape(6, 3), p[SLICES["rpy"]].reshape(6, 3), theta, scale)
    cam = p[SLICES["camera_xyz"]] + np.array([0.0, 0.0, FLANGE_Z])
    Rc = np.einsum("nab,bc->nac", R[:, 5], camera_rotation(p[SLICES["camera_rpy"]]))
    return Rc, o[:, 5] + np.einsum("nab,b->na", R[:, 5], cam)
