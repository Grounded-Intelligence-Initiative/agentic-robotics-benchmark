"""Small numerical helpers of the grader rd04 (the log band, rotations, angles: band, rotx / roty / rotz, rotvec,
axis_rot, angle_between); a non-finite error scores 0 in the band."""
from __future__ import annotations

import math

import mujoco
import numpy as np

DEG = math.pi / 180.0


def band(e, full, zero):
    """L(e) = clip(ln(zero / e) / ln(zero / full), 0, 1): 1 at or below full, 0 at or beyond zero; a non-finite e
    (NaN, inf) scores 0. Works elementwise on arrays."""
    e = np.asarray(e, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.log(zero / np.maximum(e, 1e-300)) / math.log(zero / full)
    s = np.clip(s, 0.0, 1.0)
    return np.where(np.isfinite(e), s, 0.0)


def word(e, full, zero):
    """the band word of an error: 'ok' within full, 'missed' at or beyond zero (or non-finite), else 'partial'"""
    e = float(e)
    if not math.isfinite(e) or e >= zero:
        return "missed"
    if e <= full:
        return "ok"
    return "partial"


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def rotvec(R):
    """the rotation vector of R (MuJoCo's mat2Quat then quat2Vel over unit time)"""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(np.asarray(R, float)).flatten())
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def axis_rot(a, th):
    a = np.asarray(a, float) / np.linalg.norm(a)
    Kx = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * Kx @ Kx


def angle_between(u, v):
    u = np.asarray(u, float); v = np.asarray(v, float)
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    s = np.linalg.norm(np.cross(u, v)) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.atan2(s, c)


def mat2quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(np.asarray(R, float)).flatten())
    return q


def finite(*xs) -> bool:
    for x in xs:
        if not np.all(np.isfinite(np.asarray(x, float))):
            return False
    return True
