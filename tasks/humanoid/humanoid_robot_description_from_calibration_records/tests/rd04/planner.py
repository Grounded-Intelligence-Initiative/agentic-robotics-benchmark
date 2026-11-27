"""The grader-side planner of D: a posture planned on the submitted files.

Plan on the given tree (the files' compiled model, whose frames the gate fixes) with the files' soles: each sole
box's -z face flat on a level plate, the feet at set positions with yaw 0 (the foot link's x axis along the plate's
x), the pelvis at the posture's drop, pitch and shift; leg IK per foot by least-squares Newton on the leg's six
joints from a fixed start, at most 60 iterations to 1e-12; a residual above 1e-9 m or rad fails the posture (it
scores 0). A sole box whose normal is not the foot link's z axis is set flat by turning the foot (for an
axis-aligned box the target attitude is the identity exactly)."""
from __future__ import annotations

import json
import math
import os

import numpy as np

from .model import FOOT, LEG_SLOTS, SIDES, SLOT_JOINT, TORSO
from .util import axis_rot, rotvec, roty, rotz

HERE = os.path.dirname(os.path.abspath(__file__))
IK_MAXIT = 60                                      # at most 60 iterations (printed in the acceptance sheet)
IK_TOL = 1e-12                                     # the convergence test, m or rad
IK_FD = 1e-7                                       # the forward-difference step, rad
IK_RESID_MAX = 1e-9                                # a residual above 1e-9 m or rad: the posture scores 0
IK_START = (-0.2, 0.0, 0.0, 0.4, -0.2, 0.0)       # the fixed start (left leg; the right leg's negated)


def load_postures(path=None):
    """the posture file: {"frame": {...}, "postures": [...]} (data/postures.json beside this module by default)"""
    p = path or os.path.join(HERE, "data", "postures.json")
    with open(p, encoding="utf-8") as f:
        return json.load(f)


class PlanFail(Exception):
    def __init__(self, code, detail=""):
        super().__init__(f"{code}: {detail}")
        self.code = code


def foot_target(n):
    """the foot link's attitude that sets a sole box with normal n (in the foot frame) flat on the plate, the foot's
    x axis along the plate's x: the identity when n is the foot's z axis"""
    n = np.asarray(n, float)
    z = np.array([0.0, 0.0, 1.0])
    if n[0] == 0.0 and n[1] == 0.0 and n[2] > 0.0:
        return np.eye(3)
    v = np.cross(n, z); s = float(np.linalg.norm(v)); c = float(n @ z)
    if s < 1e-15:
        Ra = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        Ra = axis_rot(v / s, math.atan2(s, c))
    xh = Ra @ np.array([1.0, 0.0, 0.0])
    return rotz(-math.atan2(xh[1], xh[0])) @ Ra


class Plan:
    __slots__ = ("name", "th", "Rp", "tp", "centres", "torso_z", "foot_x", "residual")


def plan(files, spec, frame):
    """the posture spec planned on the files -> Plan (th: slot angles, Rp / tp: the pelvis pose, centres: the planned
    tread centres (world), torso_z, foot_x: the planned feet's x axes); PlanFail('ik') by the residual rule"""
    k = files.kin
    th = np.zeros(25)
    slot = {n: i for i, n in enumerate(SLOT_JOINT)}
    for jn, v in spec["arms"].items():
        th[slot[jn]] = float(v)
    th[slot["waist_yaw_joint"]] = float(spec["waist"])
    Rf = {s: foot_target(files.sole[s].n) for s in SIDES}
    zf = {s: -float((Rf[s] @ files.sole[s].c)[2]) for s in SIDES}
    HIP_MID = np.array(frame["hip_mid_m"], float)
    Rp = roty(float(spec["pitch"]))
    z_pel_straight = 0.5 * (zf["left"] + zf["right"]) + float(frame["leg_drop0_m"])
    hip_world = np.array([HIP_MID[0], float(spec["shift"]), z_pel_straight + HIP_MID[2] - float(spec["drop"])])
    tp = hip_world - Rp @ HIP_MID
    targets = {}
    for s in SIDES:
        t = np.array(frame["nom_foot_m"][s], float).copy(); t[0] += float(spec["feet"].get(s, 0.0)); t[2] = zf[s]
        targets[s] = t
    resid = 0.0
    for s in SIDES:
        idx = list(LEG_SLOTS[s])
        x = np.array(IK_START) * (1 if s == "left" else -1)
        x[2] = 0.0
        fb = k.foot[s]
        for it in range(IK_MAXIT):
            th[idx] = x
            k.fk(th, Rp, tp)
            xp, xm = k.xpos(fb), k.xmat(fb)
            e = np.concatenate([targets[s] - xp, rotvec(Rf[s] @ xm.T)])
            if np.max(np.abs(e)) < IK_TOL:
                break
            J = np.zeros((6, 6))
            for c in range(6):
                th2 = th.copy(); th2[idx[c]] += IK_FD
                k.fk(th2, Rp, tp)
                J[:3, c] = (k.xpos(fb) - xp) / IK_FD
                J[3:, c] = rotvec(k.xmat(fb) @ xm.T) / IK_FD
            x = x + np.linalg.lstsq(J, e, rcond=None)[0]
            if not np.all(np.isfinite(x)):
                raise PlanFail("ik", f"{s} leg: a non-finite step")
        th[idx] = x
        k.fk(th, Rp, tp)
        e = np.concatenate([targets[s] - k.xpos(fb), rotvec(Rf[s] @ k.xmat(fb).T)])
        r = float(np.max(np.abs(e)))
        if not math.isfinite(r) or r > IK_RESID_MAX:
            raise PlanFail("ik", f"{s} leg residual {r:.3e}")
        resid = max(resid, r)
    k.fk(th, Rp, tp)
    P = Plan()
    P.name = spec["name"]; P.th = th.copy(); P.Rp = Rp; P.tp = tp
    P.centres = {s: k.xpos(k.foot[s]) + k.xmat(k.foot[s]) @ files.sole[s].c for s in SIDES}
    P.torso_z = k.xmat(k.torso)[:, 2].copy()
    P.foot_x = {s: k.xmat(k.foot[s])[:, 0].copy() for s in SIDES}
    P.residual = resid
    return P
