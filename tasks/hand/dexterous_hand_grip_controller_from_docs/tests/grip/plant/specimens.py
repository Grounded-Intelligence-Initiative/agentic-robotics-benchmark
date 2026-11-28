"""The instrumented specimens: two half-shells on a calibrated spring along the squeeze axis.

Half A (the thumb's side) carries a free joint; half B (the fingers' side) rides on a slide joint along the squeeze
axis with the class's spring. The slide's lower limit is the rest stop (no preload), its upper limit the travel end.
Squeeze force = k x (compression), travel = the compression; both are read from the slide joint. Specimen frame:
x = the squeeze axis (A to B), y = lateral (along the line of the index and middle pads), z = up at the pick.
Classes: brittle shells and full-load blocks have their class's spring and travel end; a soft pack has its own spring
(the job's), and its travel end lies a fixed distance past the printed travel limit, with the gap between the halves
just wider than that travel. Masses are split evenly between the halves.
"""
import numpy as np
import mujoco

from contract import constants as C
from contract import names as N
from plant import TRUTH

_S = TRUTH["specimens"]
_CON = TRUTH["contact"]
BODY_A, BODY_B = "specimen_a", "specimen_b"
GEOM_A, GEOM_B = "specimen_a_box", "specimen_b_box"
FREE, SPRING = "specimen_free", "specimen_spring"
LATERAL_M = C.SPECIMENS["lateral_mm"] / 1000.0
HEIGHT_M = C.SPECIMENS["height_mm"] / 1000.0


def geometry(job, soft_limit_mm=None):
    """The specimen's build numbers for one job spec (scenario format: class, code, mass_kg, mu, k_n_per_mm, w_mm,
    dx_mm, dy_mm, yaw_deg)."""
    kind = job["class"]
    if kind not in N.CLASSES:
        raise ValueError("not a class: %r" % (kind,))
    if job["code"] not in N.CODES[kind]:
        raise ValueError("code %r is not of class %r" % (job["code"], kind))
    w = float(job["w_mm"]) / 1000.0
    if kind == "soft":
        k = float(job["k_n_per_mm"]) * 1000.0
        travel = (float(soft_limit_mm) / 1000.0 + _S["soft_travel_end_past_limit_m"] if soft_limit_mm is not None
                  else _S["soft_travel_end_default_m"])
        gap = travel + _S["soft_gap_past_travel_end_m"]
    else:
        k = _S["spring_n_per_m"][kind]
        if job.get("k_n_per_mm") is not None and abs(float(job["k_n_per_mm"]) * 1000.0 - k) > 1e-9 * k:
            raise ValueError("the job's spring constant is not its class's")
        travel = _S["travel_end_m"][kind]
        gap = _S["gap_m"]
    t = (w - gap) / 2.0
    if t <= 0.0:
        raise ValueError("the halves would have no thickness (width %.4f m, gap %.4f m)" % (w, gap))
    return {"class": kind, "code": job["code"], "w_m": w, "L_m": LATERAL_M, "H_m": HEIGHT_M, "gap_m": gap,
            "t_m": t, "mass_kg": float(job["mass_kg"]), "mu": float(job["mu"]), "k_n_per_m": k, "travel_m": travel,
            "damping": _S["slide_damping_n_s_per_m"][kind]}


def box_inertia(m, sx, sy, sz):
    return [m * (sy * sy + sz * sz) / 12.0, m * (sx * sx + sz * sz) / 12.0, m * (sx * sx + sy * sy) / 12.0]


def placed_pose(job, nest_pos, nest_R):
    """The specimen's centre and orientation (H) placed in a nest whose pose in H is (nest_pos, nest_R)."""
    off = np.array([float(job.get("dx_mm", 0.0)) / 1000.0, float(job.get("dy_mm", 0.0)) / 1000.0, _S["rest_lift_m"]])
    yaw = np.radians(float(job.get("yaw_deg", 0.0)))
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return nest_pos + nest_R @ off, nest_R @ Rz


def half_offsets(geo):
    """The geometric centres of the two halves along the squeeze axis, in the specimen frame."""
    a = -(geo["gap_m"] / 2.0 + geo["t_m"] / 2.0)
    return a, -a


def add(spec, geo, pos, R):
    """Add the specimen to an MjSpec at its placed pose (pos, R in H)."""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, np.float64).ravel())
    t, L, H = geo["t_m"], geo["L_m"], geo["H_m"]
    xa, xb = half_offsets(geo)
    mh = geo["mass_kg"] / 2.0
    A = spec.worldbody.add_body()
    A.name = BODY_A
    A.pos = list(pos)
    A.quat = list(q)
    fj = A.add_freejoint()
    fj.name = FREE
    A.ipos = [xa, 0.0, 0.0]
    A.mass = mh
    A.inertia = box_inertia(mh, t, L, H)
    A.explicitinertial = True
    B = A.add_body()
    B.name = BODY_B
    B.pos = [0.0, 0.0, 0.0]
    B.ipos = [xb, 0.0, 0.0]
    B.mass = mh
    B.inertia = box_inertia(mh, t, L, H)
    B.explicitinertial = True
    j = B.add_joint()
    j.name = SPRING
    j.type = mujoco.mjtJoint.mjJNT_SLIDE
    j.axis = [-1.0, 0.0, 0.0]                         # positive = half B moves towards half A (compression)
    j.stiffness = [geo["k_n_per_m"], 0.0, 0.0]
    j.springref = 0.0
    j.damping = [geo["damping"], 0.0, 0.0]
    j.limited = True
    j.range = [0.0, geo["travel_m"]]
    j.solref_limit = list(_S["spring_limit_solref"])
    j.solimp_limit = list(_S["spring_limit_solimp"])
    j.armature = 0.0
    for body, name, x in ((A, GEOM_A, xa), (B, GEOM_B, xb)):
        g = body.add_geom()
        g.name = name
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [t / 2.0, L / 2.0, H / 2.0]
        g.pos = [x, 0.0, 0.0]
        g.friction = [geo["mu"], _CON["torsional_m"], _CON["rolling"]]
        g.condim = int(_CON["specimen_condim"])
        g.solref = list(_CON["pad_solref"])
        g.solimp = list(_CON["pad_solimp"])
        g.contype = 1
        g.conaffinity = 1
        g.mass = 0.0
    return A, B


def corners(geo, pos, R):
    """The 16 corners (H) of the two halves at rest (no compression) for a specimen pose."""
    t, L, H = geo["t_m"], geo["L_m"], geo["H_m"]
    out = []
    for x in half_offsets(geo):
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    out.append(pos + R @ np.array([x + sx * t / 2.0, sy * L / 2.0, sz * H / 2.0]))
    return np.array(out)
