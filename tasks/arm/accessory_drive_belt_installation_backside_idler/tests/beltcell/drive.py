"""The engine's accessory drive and the poly-V belt (cell internals).

Engine frame E (all drive geometry is given in it): origin on the crank axis in the plane of the accessory face;
x to the right and z up as seen from the robots; y into the engine (right-handed), so everything in front of the face
has y < 0. "Depth" d = -y is the distance in front of the face. Pulley axes are parallel to y.

The belt is a closed chain of rigid links (boxes: link length x width x thickness) on two-hinge joints: flatwise
bending (about the link's width axis, soft) and edgewise bending (about its thickness axis, stiffer); there is no
twist DOF. The chain is a tree rooted at link 0 (free joint) with a forward branch and a backward branch; the two
branch ends are closed by two point constraints on the closing joint's flatwise axis (a flatwise hinge). Link frame:
origin at the link's leading joint point (in the belt's running direction), z along the link, x across the belt
(the width; parallel to the pulley axes when the belt runs on the drive), y = z cross x, which on the drive points
to the rib side (the loop's inside); the back (print) face is -y.

Contacts: the belt's links touch the grooved pulleys' floors, shoulders and chamfers and the tensioner pulley's face and
flanges with friction (priority 2, condim 3); everything else they touch frictionlessly (the belt has priority 1,
condim 1); the belt never touches itself.

Runtime dependencies: mujoco + numpy.
"""

import math

import mujoco
import numpy as np

__all__ = ["BELT", "VARIANTS", "wrap_sense", "opening_sense", "Circle", "loop_path", "resample_closed", "add_drive", "add_belt", "Drive", "Belt"]

# ------------------------------------------------------------------------------------------------ belt constants
BELT = {
    "width": 0.020, "thick": 0.0045, "link": 0.010,
    "mass_per_m": 0.24,                 # kg/m (documented 0.20-0.28 class, per batch +-10 %)
    "bend_EI": 0.0120,                  # N m^2 flatwise (documented droop class: 200 mm free end droops 32-71 mm)
    "edge_ratio": 8.0,                  # edgewise / flatwise joint stiffness
    "bend_damp_s": 0.004,               # flatwise damping = stiffness x this (s)
    "armature": 2.0e-5,                 # kg m^2 per hinge DOF: slows the tensioned chain's link-scale modes below the 2 ms step
    "drag": 0.02,                       # N s/m linear drag per link (a hanging span stops swinging)
    "solref": (0.004, 1.0), "solimp": (0.95, 0.99, 0.0005, 0.5, 2.0), "margin": 0.0005,
    "close_dx": 0.008,                  # the closing joint's two point constraints, +-dx along the width
}
BITS = {"belt": (2, 1), "other": (1, 3)}   # (contype, conaffinity): belt-belt never collide

# grooved pulley axial layout (depth d in front of the face, metres): back shoulder, groove zone, front shoulder,
# 45 deg chamfer; shoulders stand `shoulder_h` proud of the groove floor
GROOVE = {"d_back": 0.010, "d_zone0": 0.016, "d_zone1": 0.038, "d_front1": 0.041, "chamfer": 0.0025, "shoulder_h": 0.0025}
BELT_PLANE_D = 0.5 * (GROOVE["d_zone0"] + GROOVE["d_zone1"])        # 0.027
# the tensioner pulley: a smooth face between two flanges
TPULLEY = {"d_back": 0.012, "d_face0": 0.015, "d_face1": 0.040, "d_front1": 0.043, "flange_h": 0.004, "chamfer": 0.002}

# ------------------------------------------------------------------------------------------------ drive variants
# pulleys: name -> centre (E x, E z), groove-floor radius r (grooved) or face radius (tensioner), kind
# tensioner: pivot, arm length to the pulley centre, angles (rad, about E -y as seen from the front: positive turns
#   counter-clockwise in the (x, z) drawing), closed / open stops, the window, spring (rate, torque at the closed stop),
#   damper, the service lever (radius from the pivot, angle offset from the pulley arm), the pointer
VARIANTS = {
    "A": {
        "pulleys": {
            "C": {"c": (0.0, 0.0), "r": 0.070, "kind": "crank"},
            "WP": {"c": (-0.120, 0.136), "r": 0.050, "kind": "grooved"},
            "ALT": {"c": (0.084, 0.228), "r": 0.045, "kind": "grooved"},
        },
        "route": ["C", "T", "ALT", "WP"],          # counter-clockwise as drawn (seen from the front); T on the back
        "tensioner": {"pivot": (0.140, 0.160), "arm": 0.070, "r": 0.045,
                      "theta_closed": math.radians(-152.0), "theta_open": math.radians(-72.0),
                      "window": (math.radians(-149.0), math.radians(-133.0)),
                      "rate": 0.83, "tau_closed": 1.84, "damper": 1.0, "friction": 0.08,
                      "lever_r": 0.150, "lever_off": math.radians(123.0), "lever_len": 0.085, "lever_radius": 0.008,
                      "lever_d0": 0.004},
        "belt_links": 105,                         # 1.050 m nominal (Plan S, 2026-11-09: +10 mm installation slack)
    },
    # five-pulley drive with a smooth backside idler I on the top run: the belt's back wraps I and T (two reverse bends);
    # the crank turns clockwise as drawn (crank_sense -1): the belt runs C -> T -> WP -> I -> ALT, T on the slack side;
    # the tensioner opens clockwise (theta_open < theta_closed)
    "B": {
        "pulleys": {
            "C": {"c": (0.0, 0.0), "r": 0.070, "kind": "crank"},
            "ALT": {"c": (0.165, 0.225), "r": 0.040, "kind": "grooved"},
            "I": {"c": (0.035, 0.285), "r": 0.035, "kind": "idler", "s": -1},
            "WP": {"c": (-0.095, 0.245), "r": 0.050, "kind": "grooved"},
        },
        "route": ["C", "ALT", "I", "WP", "T"],     # counter-clockwise as drawn; I and T on the back
        "crank_sense": -1,
        "tensioner": {"pivot": (-0.140, 0.160), "arm": 0.070, "r": 0.045,
                      "theta_closed": math.radians(-24.0), "theta_open": math.radians(-114.0),
                      "window": (math.radians(-47.0), math.radians(-28.0)),   # correct fits rest at -42.8..-31.5 deg (observability.md)
                      "rate": 0.83, "tau_closed": 1.84, "damper": 1.0, "friction": 0.08,
                      "lever_r": 0.157, "lever_off": math.radians(230.0), "lever_len": 0.085, "lever_radius": 0.008,
                      "lever_d0": 0.004},
        "belt_links": 121,                         # 1.210 m nominal
    },
}


def wrap_sense(V, k):
    """+1: the belt's rib side wraps pulley k (inside the loop); -1: its back (T, a backside idler)."""
    if k == "T":
        return -1
    return int(V["pulleys"][k].get("s", 1))


def opening_sense(T):
    """+1 if the tensioner opens counter-clockwise as drawn (theta_open > theta_closed), else -1."""
    return 1 if T["theta_open"] > T["theta_closed"] else -1


CELL_VARIANT = "B"                                 # the drive this cell builds


# ------------------------------------------------------------------------------------------------ 2-D path geometry
class Circle(object):
    """A pulley in the belt plane: centre (2,), the radius of the belt's neutral line at the joint points, wrap sense
    s (+1: the belt turns counter-clockwise round it, the rib side touching; -1: clockwise, the back touching)."""

    def __init__(self, name, c, rho, s):
        self.name = name
        self.c = np.asarray(c, float)
        self.rho = float(rho)
        self.s = int(s)


def _tangent(c1, c2):
    D = c2.c - c1.c
    dist = float(np.linalg.norm(D))
    k = c2.s * c2.rho - c1.s * c1.rho
    if abs(k) >= dist:
        raise ValueError("pulleys %s and %s overlap" % (c1.name, c2.name))
    phi = math.atan2(D[1], D[0]) - math.asin(k / dist)
    d = np.array([math.cos(phi), math.sin(phi)])
    nR = np.array([d[1], -d[0]])
    return c1.c + c1.s * c1.rho * nR, c2.c + c2.s * c2.rho * nR


def loop_path(circles, step=0.0005):
    """The closed belt line round `circles` (in running order): (points (n, 2) densely sampled, total length,
    per-pulley wrap {name: (in angle, out angle, wrap angle)})."""
    n = len(circles)
    tang = [_tangent(circles[i], circles[(i + 1) % n]) for i in range(n)]
    pts = []
    total = 0.0
    wraps = {}
    for i in range(n):
        ci = circles[i]
        p_in = tang[i - 1][1]
        p_out = tang[i][0]
        a_in = math.atan2(*(p_in - ci.c)[::-1])
        a_out = math.atan2(*(p_out - ci.c)[::-1])
        if ci.s > 0:
            sweep = (a_out - a_in) % (2 * math.pi)
        else:
            sweep = (a_in - a_out) % (2 * math.pi)
        wraps[ci.name] = (a_in, a_out, sweep)
        m = max(2, int(math.ceil(ci.rho * sweep / step)))
        for k in range(m):
            a = a_in + ci.s * sweep * k / m
            pts.append(ci.c + ci.rho * np.array([math.cos(a), math.sin(a)]))
        total += ci.rho * sweep
        q0, q1 = tang[i]
        L = float(np.linalg.norm(q1 - q0))
        m = max(1, int(math.ceil(L / step)))
        for k in range(m):
            pts.append(q0 + (q1 - q0) * k / m)
        total += L
    return np.asarray(pts), total, wraps


def resample_closed(path, n_links, link_len, start_index=0):
    """Joint points of a closed chain of n_links chords of exactly link_len along the closed polyline `path`, from
    path[start_index]; returns (points (n_links, 2), closure gap vector: last chord's end minus the first point)."""
    P = np.roll(np.asarray(path, float), -int(start_index), axis=0)
    P = np.vstack([P, P[:1], P[1:2]])
    pts = [P[0].copy()]
    cur = P[0].copy()
    seg, s_cur = 0, 0.0
    for _ in range(n_links):
        placed = False
        j = seg
        while j < len(P) - 1:
            a, b = P[j], P[j + 1]
            dd = b - a
            A = float(dd @ dd)
            if A < 1e-16:
                j += 1
                continue
            f = a - cur
            B = 2.0 * float(f @ dd)
            C = float(f @ f) - link_len * link_len
            disc = B * B - 4 * A * C
            lo = s_cur if j == seg else 0.0
            if disc >= 0:
                r = math.sqrt(disc)
                cands = [s for s in ((-B - r) / (2 * A), (-B + r) / (2 * A)) if lo - 1e-12 <= s <= 1.0 + 1e-12]
                if cands:
                    s = min(1.0, max(0.0, max(cands)))
                    cur = a + s * dd
                    pts.append(cur.copy())
                    seg, s_cur = j, s
                    placed = True
                    break
            j += 1
        if not placed:
            raise ValueError("path too short for the chain")
    pts = np.asarray(pts)
    return pts[:-1], pts[-1] - pts[0]


def joint_radius(rho, link_len):
    """Radius of the circle through the joint points of rigid links whose middles touch a circle of radius rho."""
    return math.sqrt(rho * rho + 0.25 * link_len * link_len)


# ------------------------------------------------------------------------------------------------ builders
def _q_from_R(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def _cyl_y(body, name, radius, d0, d1, **kw):
    """A cylinder whose axis is the engine's y axis, spanning depth d0..d1 (y = -d1..-d0) in the body frame (the body
    frame's y is E y); the local origin of the body is on the axis."""
    q = _q_from_R(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]))   # cylinder z -> body y
    return body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[radius, 0.5 * (d1 - d0)],
                         pos=[0.0, -0.5 * (d0 + d1), 0.0], quat=q.tolist(), **kw)


def _frustum_mesh(spec, name, r0, r1, length, n=48):
    """A convex frustum mesh along local +z from radius r0 at z=0 to r1 at z=length."""
    verts = []
    for z, r in ((0.0, r0), (length, r1)):
        for k in range(n):
            a = 2 * math.pi * k / n
            verts += [r * math.cos(a), r * math.sin(a), z]
    spec.add_mesh(name=name, uservert=verts)
    return name


def _shoulder_mesh(spec, name, r_top, r_floor, w_top, chamfer, n=64):
    """The front shoulder as one convex mesh along local +z: a ring top of radius r_top over w_top, then a 45 deg
    chamfer down to r_floor over `chamfer` (the solid includes the core below the top)."""
    verts = []
    for z, r in ((0.0, r_top), (w_top, r_top), (w_top + chamfer, r_floor)):
        for k in range(n):
            a = 2 * math.pi * k / n
            verts += [r * math.cos(a), r * math.sin(a), z]
    spec.add_mesh(name=name, uservert=verts)
    return name


def add_drive(spec, parent, variant, params, materials, prefix="eng/"):
    """Build the drive (pulleys on hinges, the tensioner) under `parent` (the engine body, whose frame is E).
    Returns a descriptor dict."""
    V = VARIANTS[variant]
    g = GROOVE
    desc = {"pulleys": {}, "variant": variant}
    mu = float(params["mu"])
    fr = [mu, 0.005, 0.0001]
    belt_kw = dict(priority=2, condim=3, friction=fr, solref=list(params.get("contact_solref", BELT["solref"])),
                   solimp=list(BELT["solimp"]))
    for name, P in V["pulleys"].items():
        cx, cz = P["c"]
        r = float(P["r"])
        b = parent.add_body(name=prefix + name, pos=[cx, 0.0, cz])
        jn = b.add_joint(name=prefix + name + "/spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0.0, -1.0, 0.0],
                         damping=0.002, frictionloss=float(params.get("crank_hold", 5.0) if P["kind"] == "crank" else
                                            params.get("idler_bearing", params.get("bearing", 0.01)) if P["kind"] == "idler" else params.get("bearing", 0.01)),
                         armature=0.0005)
        geoms = {}
        if P["kind"] == "idler":
            # a smooth flanged idler (the tensioner pulley's section): face between two flanges, the belt's back on it
            t = TPULLEY
            geoms["face"] = _cyl_y(b, prefix + name + "/face", r, t["d_back"], t["d_front1"], material=materials["steel"], mass=0.15, **belt_kw)
            geoms["back"] = _cyl_y(b, prefix + name + "/back", r + t["flange_h"], t["d_back"], t["d_face0"], material=materials["steel"], mass=0.02, **belt_kw)
            geoms["front"] = _cyl_y(b, prefix + name + "/front", r + t["flange_h"], t["d_face1"], t["d_front1"], material=materials["steel"], mass=0.02, **belt_kw)
            _cyl_y(b, prefix + name + "/hub", 0.6 * r, 0.0, t["d_back"], material=materials["dark"], mass=0.03, contype=0, conaffinity=0, group=1)
            b.add_geom(name=prefix + name + "/mark", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.004, 0.0004, 0.4 * r],
                       pos=[0.0, -t["d_front1"] - 0.0003, 0.5 * r], material=materials["mark"], contype=0, conaffinity=0,
                       group=1, mass=0.0)
            desc["pulleys"][name] = {"body": prefix + name, "joint": prefix + name + "/spin", "r": r, "kind": P["kind"],
                                     "c": (cx, cz), "geoms": {k: prefix + name + "/" + k for k in geoms}}
            continue
        # the groove floor runs the pulley's full width (its end caps lie under the shoulders' outer faces, so a link
        # pressed into the rim is always pushed out radially); the back shoulder is a plain ring; the front shoulder
        # and its 45 deg chamfer are one convex mesh
        d_end = g["d_front1"] + g["chamfer"]
        geoms["floor"] = _cyl_y(b, prefix + name + "/floor", r, g["d_back"], d_end, material=materials["steel"], mass=0.3,
                                **belt_kw)
        geoms["back"] = _cyl_y(b, prefix + name + "/back", r + g["shoulder_h"], g["d_back"], g["d_zone0"],
                               material=materials["steel"], mass=0.05, **belt_kw)
        mesh = _shoulder_mesh(spec, prefix + name + "/front_mesh", r + g["shoulder_h"], r, g["d_front1"] - g["d_zone1"], g["chamfer"])
        qz = _q_from_R(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]))
        geoms["front"] = b.add_geom(name=prefix + name + "/front", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh,
                                    pos=[0.0, -g["d_zone1"], 0.0], quat=qz.tolist(), material=materials["steel"], mass=0.04,
                                    **belt_kw)
        # hub (visual + structure, behind the back shoulder) and a spoke mark so that turning is visible
        _cyl_y(b, prefix + name + "/hub", 0.6 * r, 0.0, g["d_back"], material=materials["dark"], mass=0.05,
               contype=0, conaffinity=0, group=1)
        b.add_geom(name=prefix + name + "/mark", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.004, 0.0004, 0.4 * r],
                   pos=[0.0, -g["d_front1"] - 0.0003, 0.5 * r], material=materials["mark"], contype=0, conaffinity=0,
                   group=1, mass=0.0)
        desc["pulleys"][name] = {"body": prefix + name, "joint": prefix + name + "/spin", "r": r, "kind": P["kind"],
                                 "c": (cx, cz), "geoms": {k: prefix + name + "/" + k for k in geoms}}
    # ---------------------------------------------------------------- the automatic tensioner
    T = V["tensioner"]
    px, pz = T["pivot"]
    t = TPULLEY
    arm = parent.add_body(name=prefix + "T_arm", pos=[px, 0.0, pz])
    th_c, th_o = float(T["theta_closed"]), float(T["theta_open"])
    sense = opening_sense(T)
    rate = float(params.get("t_rate", T["rate"]))
    tau_c = float(params.get("t_tau_closed", T["tau_closed"]))
    # joint angle q = sense x (theta - theta_closed) (0 at the closed stop, positive = opening; the hinge axis is E -y
    # for a counter-clockwise opening, E +y for a clockwise one); the spring torque is tau_c + rate * q towards closing:
    # MuJoCo's spring -k (q - q_ref) with q_ref = -tau_c / rate
    jt = arm.add_joint(name=prefix + "T_arm/pivot", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0.0, -1.0 * sense, 0.0],
                       range=[0.0, sense * (th_o - th_c)], limited=True, stiffness=rate, springref=-tau_c / rate,
                       damping=float(params.get("t_damper", T["damper"])), frictionloss=float(params.get("t_friction", T["friction"])),
                       armature=0.0005, solref_limit=[0.01, 0.7], solimp_limit=[0.9, 0.95, 0.002, 0.5, 2.0])
    # the arm geometry is drawn at the closed stop (q = 0): pulley-arm direction theta_closed
    ua = np.array([math.cos(th_c), math.sin(th_c)])
    a = float(T["arm"])
    pc = ua * a
    # the arm: a flat bar from the pivot to the pulley hub, behind the belt (depth 0.002-0.008)
    L = a
    ang = math.atan2(ua[1], ua[0])
    Rbar = np.array([[math.cos(ang), 0.0, -math.sin(ang)], [0.0, 1.0, 0.0], [math.sin(ang), 0.0, math.cos(ang)]])
    arm.add_geom(name=prefix + "T_arm/bar", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5 * L, 0.003, 0.012],
                 pos=[0.5 * pc[0], -0.005, 0.5 * pc[1]], quat=_q_from_R(Rbar).tolist(), material=materials["arm"], mass=0.12)
    _cyl_y(arm, prefix + "T_arm/boss", 0.016, 0.0, 0.010, material=materials["arm"], mass=0.03)
    # the lever: a round bar parallel to the pulley axes, at lever_r from the pivot, at angle theta + lever_off
    lo = th_c + float(T["lever_off"])
    pl = np.array([math.cos(lo), math.sin(lo)]) * float(T["lever_r"])
    Rl = np.array([[math.cos(lo), 0.0, -math.sin(lo)], [0.0, 1.0, 0.0], [math.sin(lo), 0.0, math.cos(lo)]])
    arm.add_geom(name=prefix + "T_arm/branch", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5 * float(T["lever_r"]), 0.003, 0.010],
                 pos=[0.5 * pl[0], -0.005, 0.5 * pl[1]], quat=_q_from_R(Rl).tolist(), material=materials["arm"], mass=0.08)
    lever_d0 = float(T["lever_d0"])
    lever_d1 = lever_d0 + float(T["lever_len"])
    lev = _cyl_y(arm, prefix + "T_arm/lever", float(T["lever_radius"]), lever_d0, lever_d1, material=materials["lever"], mass=0.04,
                 friction=[0.8, 0.005, 0.0001])
    lev.pos = [pl[0], -0.5 * (lever_d0 + lever_d1), pl[1]]
    arm.add_site(name=prefix + "T_arm/lever_axis", pos=[pl[0], -lever_d1, pl[1]], size=[0.002] * 3, group=4)
    # the pointer: a thin white blade on the boss, pointing away from the pulley, reading against the window marks
    pp = -ua * 0.038
    arm.add_geom(name=prefix + "T_arm/pointer", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.008, 0.0004, 0.0012],
                 pos=[pp[0], -0.0085, pp[1]], quat=_q_from_R(Rbar).tolist(), material=materials["mark"], contype=0,
                 conaffinity=0, group=1, mass=0.0)
    # the pulley on its own spin hinge at the arm's end
    tp = arm.add_body(name=prefix + "T", pos=[pc[0], 0.0, pc[1]])
    tp.add_joint(name=prefix + "T/spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0.0, -1.0, 0.0], damping=0.002,
                 frictionloss=float(params.get("bearing", 0.01)), armature=0.0003)
    rT = float(T["r"])
    tg = {}
    tg["face"] = _cyl_y(tp, prefix + "T/face", rT, t["d_back"], t["d_front1"], material=materials["steel"], mass=0.20, **belt_kw)
    tg["back"] = _cyl_y(tp, prefix + "T/back", rT + t["flange_h"], t["d_back"], t["d_face0"], material=materials["steel"], mass=0.03, **belt_kw)
    tg["front"] = _cyl_y(tp, prefix + "T/front", rT + t["flange_h"], t["d_face1"], t["d_front1"], material=materials["steel"], mass=0.03, **belt_kw)
    _cyl_y(tp, prefix + "T/hub", 0.6 * rT, 0.006, t["d_back"], material=materials["dark"], mass=0.03, contype=0, conaffinity=0, group=1)
    tp.add_geom(name=prefix + "T/mark", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.004, 0.0004, 0.4 * rT],
                pos=[0.0, -t["d_front1"] - 0.0003, 0.5 * rT], material=materials["mark"], contype=0, conaffinity=0, group=1, mass=0.0)
    desc["tensioner"] = {"arm_body": prefix + "T_arm", "pivot_joint": prefix + "T_arm/pivot", "pulley_body": prefix + "T",
                         "spin_joint": prefix + "T/spin", "lever_geom": prefix + "T_arm/lever", "lever_site": prefix + "T_arm/lever_axis",
                         "geoms": {k: prefix + "T/" + k for k in tg}, "theta_closed": th_c, "theta_open": th_o,
                         "window": tuple(T["window"]), "arm": a, "r": rT, "pivot": (px, pz), "lever_r": float(T["lever_r"]),
                         "lever_off": float(T["lever_off"]), "lever_d": (lever_d0, lever_d1), "lever_radius": float(T["lever_radius"]),
                         "rate": rate, "tau_closed": tau_c, "sense": sense}
    # window marks on the face (two short white lines at the window's ends, at the pointer's radius)
    for k, th in enumerate(T["window"]):
        u = np.array([math.cos(th + math.pi), math.sin(th + math.pi)])
        q = th + math.pi
        Rm = np.array([[math.cos(q), 0.0, -math.sin(q)], [0.0, 1.0, 0.0], [math.sin(q), 0.0, math.cos(q)]])
        parent.add_geom(name=prefix + "window%d" % k, type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.006, 0.0003, 0.0010],
                        pos=[px + u[0] * 0.046, -0.0002, pz + u[1] * 0.046], quat=_q_from_R(Rm).tolist(),
                        material=materials["mark"], contype=0, conaffinity=0, group=1)
    return desc


def add_belt(spec, parent, name, n_links, params, material):
    """The belt chain (built straight along world +x at a parking spot; posed later by Belt.set_points).
    Root = link 0 (free joint); forward branch 1..K, backward branch n-1..K+1 (built as children going backwards).
    Returns a descriptor."""
    l = float(params.get("link", BELT["link"]))
    hw, ht = 0.5 * BELT["width"], 0.5 * BELT["thick"]
    m_link = float(params.get("mass_per_m", BELT["mass_per_m"])) * l
    EI = float(params.get("bend_EI", BELT["bend_EI"]))
    kf = EI / l
    ke = kf * float(params.get("edge_ratio", BELT["edge_ratio"]))
    cf = kf * float(BELT["bend_damp_s"])
    ce = ke * float(BELT["bend_damp_s"])
    arm_ = float(params.get("armature", BELT["armature"]))
    K = n_links // 2
    collider = params.get("collider", BELT.get("collider", "spheres"))
    ckw = dict(mass=0.0, priority=1, condim=1, friction=[0.0, 0.0, 0.0], solref=list(params.get("contact_solref", BELT["solref"])),
               solimp=list(BELT["solimp"]), margin=BELT["margin"], contype=BITS["belt"][0], conaffinity=BITS["belt"][1])
    vkw = dict(type=mujoco.mjtGeom.mjGEOM_BOX, size=[hw, ht, 0.5 * l], material=material, mass=m_link)

    def _link_geoms(body, i):
        """The link's visual box (carries the mass; no contacts) and its colliders."""
        if collider == "box":
            body.add_geom(name="%s/g%03d" % (name, i), pos=[0.0, 0.0, 0.5 * l], type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[hw, ht, 0.5 * l], material=material, mass=m_link, **{k: v for k, v in ckw.items() if k != "mass"})
            return
        body.add_geom(name="%s/g%03d" % (name, i), pos=[0.0, 0.0, 0.5 * l], contype=0, conaffinity=0, **vkw)
        rs = ht
        ns = int(params.get("n_spheres", BELT.get("n_spheres", 4)))
        for k in range(ns):
            sx = -(hw - rs) + 2 * (hw - rs) * k / (ns - 1)
            body.add_geom(name="%s/c%03d%s" % (name, i, "abcdef"[k]), type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[rs, 0, 0],
                          pos=[sx, 0.0, 0.5 * l], group=3, rgba=[0.2, 0.2, 0.2, 1.0], **ckw)

    bodies = [None] * n_links
    hinges = {}

    def _hinges(body, i, at_end):
        jp = [0.0, 0.0, l] if at_end else [0.0, 0.0, 0.0]
        jx = body.add_joint(name="%s/j%03dx" % (name, i), type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1.0, 0.0, 0.0], pos=jp,
                            stiffness=kf, damping=cf, armature=arm_)
        jy = body.add_joint(name="%s/j%03dy" % (name, i), type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0.0, 1.0, 0.0], pos=jp,
                            stiffness=ke, damping=ce, armature=arm_)
        hinges[i] = (jx.name, jy.name)

    n_sub = int(params.get("n_sub", 0))
    roots = []
    closures = []          # (body1 index, body2 index): body1's end meets body2's start
    if n_sub <= 1:
        root = parent.add_body(name="%s/l%03d" % (name, 0), pos=[0.0, 0.0, -0.5])
        root.add_freejoint(name="%s/free" % name)
        _link_geoms(root, 0)
        bodies[0] = root
        roots.append(0)
        prev = root
        for i in range(1, K + 1):
            b = prev.add_body(name="%s/l%03d" % (name, i), pos=[0.0, 0.0, l])
            _hinges(b, i, at_end=False)
            _link_geoms(b, i)
            bodies[i] = b
            prev = b
        prev = root
        for i in range(n_links - 1, K, -1):
            b = prev.add_body(name="%s/l%03d" % (name, i), pos=[0.0, 0.0, -l])
            _hinges(b, i, at_end=True)
            _link_geoms(b, i)
            bodies[i] = b
            prev = b
        closures.append((K, K + 1))
    else:
        bounds = [int(round(j * n_links / n_sub)) for j in range(n_sub + 1)]
        for j in range(n_sub):
            a0, a1 = bounds[j], bounds[j + 1]
            root = parent.add_body(name="%s/l%03d" % (name, a0), pos=[0.1 * j, 0.0, -0.5])
            root.add_freejoint(name="%s/free%02d" % (name, j))
            _link_geoms(root, a0)
            bodies[a0] = root
            roots.append(a0)
            prev = root
            for i in range(a0 + 1, a1):
                b = prev.add_body(name="%s/l%03d" % (name, i), pos=[0.0, 0.0, l])
                _hinges(b, i, at_end=False)
                _link_geoms(b, i)
                bodies[i] = b
                prev = b
            closures.append((a1 - 1, a1 % n_links))
    # print marks (white, every 5th link, on the back face) for honest segmentation
    for i in range(0, n_links, 5):
        bodies[i].add_geom(name="%s/print%03d" % (name, i), type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.006, 0.0002, 0.0015],
                           pos=[0.0, -(ht + 0.0002), 0.5 * l], material=params["print_material"], contype=0, conaffinity=0,
                           group=1, mass=0.0)
    dx = float(BELT["close_dx"])
    eqs = []
    for c, (i1, i2) in enumerate(closures):
        for k, sx in enumerate((-dx, dx)):
            e = spec.add_equality(name="%s/close%d%s" % (name, c, "ab"[k]), type=mujoco.mjtEq.mjEQ_CONNECT,
                                  objtype=mujoco.mjtObj.mjOBJ_BODY, name1="%s/l%03d" % (name, i1), name2="%s/l%03d" % (name, i2),
                                  solref=[0.004, 1.0], solimp=[0.95, 0.99, 0.001, 0.5, 2.0])
            e.data[0:6] = [sx, 0.0, l, sx, 0.0, 0.0]
            eqs.append(e.name)
    colliders = (["%s/g%03d" % (name, i) for i in range(n_links)] if collider == "box" else
                 ["%s/c%03d%s" % (name, i, k) for i in range(n_links) for k in "abcdef"[:int(params.get("n_spheres", BELT.get("n_spheres", 4)))]])
    return {"name": name, "n": n_links, "K": K, "link": l, "bodies": ["%s/l%03d" % (name, i) for i in range(n_links)],
            "geoms": ["%s/g%03d" % (name, i) for i in range(n_links)], "colliders": colliders, "roots": roots, "hinges": hinges,
            "close": eqs, "closures": closures, "kf": kf, "ke": ke, "cf": cf, "ce": ce, "mass_link": m_link}


# ------------------------------------------------------------------------------------------------ runtime helpers
class Drive(object):
    """Runtime view of the drive: ids, pulley angles, the tensioner angle, the belt-plane frame."""

    def __init__(self, model, desc, engine_body):
        m = model
        self.m = m
        self.desc = desc
        self.engine = m.body(engine_body).id
        self.p = {}
        for k, P in desc["pulleys"].items():
            j = m.joint(P["joint"]).id
            self.p[k] = {"body": m.body(P["body"]).id, "jnt": j, "qadr": m.jnt_qposadr[j], "dof": m.jnt_dofadr[j],
                         "r": P["r"], "kind": P["kind"], "geoms": {g: m.geom(n).id for g, n in P["geoms"].items()}}
        T = desc["tensioner"]
        j = m.joint(T["pivot_joint"]).id
        self.t_jnt = j
        self.t_qadr = m.jnt_qposadr[j]
        self.t_dof = m.jnt_dofadr[j]
        js = m.joint(T["spin_joint"]).id
        self.p["T"] = {"body": m.body(T["pulley_body"]).id, "jnt": js, "qadr": m.jnt_qposadr[js], "dof": m.jnt_dofadr[js],
                       "r": T["r"], "kind": "tensioner", "geoms": {g: m.geom(n).id for g, n in T["geoms"].items()}}
        self.lever_geom = m.geom(T["lever_geom"]).id
        self.lever_site = m.site(T["lever_site"]).id
        self.arm_body = m.body(T["arm_body"]).id
        self.T = T
        self.grooved = [k for k, P in desc["pulleys"].items()]
        self.pulley_geoms = set()
        for P in self.p.values():
            self.pulley_geoms |= set(P["geoms"].values())

    def theta(self, d):
        """Tensioner arm angle (rad, drawing convention)."""
        return self.T["theta_closed"] + self.T.get("sense", 1) * float(d.qpos[self.t_qadr])

    def omega(self, d):
        """Tensioner arm angular velocity (rad/s, drawing convention)."""
        return self.T.get("sense", 1) * float(d.qvel[self.t_dof])

    def E(self, d):
        """(origin, R) of the engine frame in the world."""
        return d.xpos[self.engine].copy(), d.xmat[self.engine].reshape(3, 3).copy()

    def circles(self, theta_T=None, link=BELT["link"]):
        """The route's circles (joint-point radii) for a tensioner angle (default: from the drawing's window centre)."""
        V = VARIANTS[self.desc["variant"]]
        T = V["tensioner"]
        th = theta_T if theta_T is not None else 0.5 * (T["window"][0] + T["window"][1])
        tc = np.asarray(T["pivot"]) + float(T["arm"]) * np.array([math.cos(th), math.sin(th)])
        out = []
        t = BELT["thick"]
        for k in V["route"]:
            if k == "T":
                out.append(Circle("T", tc, joint_radius(T["r"] + 0.5 * t, link), -1))
            else:
                P = V["pulleys"][k]
                out.append(Circle(k, P["c"], joint_radius(P["r"] + 0.5 * t, link), wrap_sense(V, k)))
        return out


class Belt(object):
    """Runtime view of the belt chain: pose it on a 3-D closed polyline of joint points, read its joint points."""

    def __init__(self, model, desc):
        m = model
        self.m = m
        self.desc = desc
        self.n = desc["n"]
        self.K = desc["K"]
        self.l = desc["link"]
        self.body = np.array([m.body(b).id for b in desc["bodies"]])
        self.geom = np.array([m.geom(g).id for g in desc["geoms"]])
        self.roots = list(desc["roots"])
        self.free_qadr = {}
        self.free_dof = {}
        for r in self.roots:
            b = m.body(desc["bodies"][r]).id
            j = int(m.body_jntadr[b])
            self.free_qadr[r] = m.jnt_qposadr[j]
            self.free_dof[r] = m.jnt_dofadr[j]
        self.parent_idx = {}
        for i in range(self.n):
            pb = int(m.body_parentid[self.body[i]])
            self.parent_idx[i] = int(np.nonzero(self.body == pb)[0][0]) if pb in set(int(x) for x in self.body) else None
        self.hq = {}
        self.hdof = {}
        for i, (jx, jy) in desc["hinges"].items():
            self.hq[i] = (m.jnt_qposadr[m.joint(jx).id], m.jnt_qposadr[m.joint(jy).id])
            self.hdof[i] = (m.jnt_dofadr[m.joint(jx).id], m.jnt_dofadr[m.joint(jy).id])
        self.dofs = np.array([self.free_dof[r] + k for r in self.roots for k in range(6)] +
                             [d for i in sorted(self.hdof) for d in self.hdof[i]])
        self.close_eq = [m.equality(e).id for e in desc["close"]]
        self.closures = list(desc["closures"])
        self.colliders = np.array([m.geom(g).id for g in desc["colliders"]])
        self.geom_set = set(int(g) for g in self.colliders) | set(int(g) for g in self.geom)
        self.body_set = set(int(b) for b in self.body)

    def setup_junctions(self, kf, cf):
        """The sub-chain junctions are flatwise hinges (two point constraints); give them the joints' flatwise spring and
        damper as explicit equal-and-opposite torques, applied before every physics step (apply_junction_torques).
        Also put the joints' armature on the roots' rotational DOFs so the junction mode is as slow as a joint's."""
        self.j_kf, self.j_cf = float(kf), float(cf)
        self.junctions = [(int(self.body[i1]), int(self.body[i2])) for (i1, i2) in self.closures]
        self._jb1 = np.array([j[0] for j in self.junctions], dtype=int)
        self._jb2 = np.array([j[1] for j in self.junctions], dtype=int)

    def apply_junction_torques(self, d):
        if not getattr(self, "junctions", None):
            return
        R1 = d.xmat[self._jb1].reshape(-1, 3, 3)
        R2 = d.xmat[self._jb2].reshape(-1, 3, 3)
        x1 = R1[:, :, 0]
        # flatwise angle of body2 relative to body1 about the shared x axis: Q = R1^T R2, a = atan2(Q21, Q11)
        y1, z1 = R1[:, :, 1], R1[:, :, 2]
        y2 = R2[:, :, 1]
        a = np.arctan2(np.einsum("ij,ij->i", z1, y2), np.einsum("ij,ij->i", y1, y2))
        w1 = d.cvel[self._jb1, 0:3]
        w2 = d.cvel[self._jb2, 0:3]
        adot = np.einsum("ij,ij->i", w2 - w1, x1)
        tau = -(self.j_kf * a + self.j_cf * adot)
        T = tau[:, None] * x1
        d.xfrc_applied[self._jb2, 3:6] = T
        d.xfrc_applied[self._jb1, 3:6] = -T

    def frames_from_points(self, P, nvec):
        """Link rotation matrices for joint points P (n, 3) (closed: link i runs P[i] -> P[i+1 mod n]) and a per-link
        width direction nvec (n, 3) (projected orthogonal to the link)."""
        n = self.n
        Rs = []
        for i in range(n):
            z = P[(i + 1) % n] - P[i]
            z = z / np.linalg.norm(z)
            x = nvec[i] - (nvec[i] @ z) * z
            x = x / np.linalg.norm(x)
            y = np.cross(z, x)
            Rs.append(np.column_stack([x, y, z]))
        return Rs

    def set_points(self, d, P, nvec):
        """Pose the chain so that its joint points are P (n, 3, closed) with link i from P[i] to P[i+1]; nvec = the
        width direction per link. Forward kinematics then reproduces P up to the hinge decomposition."""
        m = self.m
        Rs = self.frames_from_points(P, nvec)
        d.qvel[self.dofs] = 0.0

        def angles(Rp, Rc):
            Q = Rp.T @ Rc
            b = math.atan2(Q[0, 2], Q[0, 0])
            a_ = math.atan2(Q[2, 1], Q[1, 1])
            return a_, b

        for r in self.roots:
            a = self.free_qadr[r]
            d.qpos[a:a + 3] = P[r]
            d.qpos[a + 3:a + 7] = _q_from_R(Rs[r])
        for i in self.hq:
            p = self.parent_idx[i]
            ax, by = angles(Rs[p], Rs[i])
            d.qpos[self.hq[i][0]] = ax
            d.qpos[self.hq[i][1]] = by
        mujoco.mj_kinematics(m, d)

    def points(self, d):
        """Joint points (n, 3): the origin of every link body (its leading joint point)."""
        return d.xpos[self.body].copy()

    def frames(self, d):
        return d.xmat[self.body].reshape(-1, 3, 3).copy()

    def centres(self, d):
        """Link centres (n, 3)."""
        return d.xpos[self.body] + d.xmat[self.body].reshape(-1, 3, 3)[:, :, 2] * (0.5 * self.l)


# ------------------------------------------------------------------------------------------------ acceptance geometry
ACCEPT = {
    "sector_trim_deg": 12.0,       # wrap sector trimmed at both ends (entry / exit transitions are not judged)
    "near_mm": 6.0,                # a link centre within (r + t/2 + near) of the axis counts as on the pulley
    "min_wrap_frac": 0.6,          # at least this share of the drawing's wrap must be covered by links on the pulley
    "lift_mm": 1.5,                # gross lift (link-centre radius - floor radius - half the belt's thickness; a seated link
                                   # reads ~0.5 mm, the colliders' contact margin): midpoint of the measured gap between
                                   # seated (max 1.06 mm) and shoulder-riding (min 1.90 mm) belts, 564 samples
    "axial_mm": 3.2,               # |belt-centre depth - groove-zone centre|: midpoint of the gap between seated (max
                                   # 0.86 mm) and riding (min 5.62 mm) belts; see tests/review/observability.md
    "t_axial_mm": 3.0,             # on the tensioner pulley's 25 mm face (and the backside idler's)
    "i_sector_trim_deg": 4.0,      # the backside idler I wraps only ~34 deg: its sector is trimmed 4 deg at both ends
    "i_min_wrap_frac": 0.5,        # ... and at least half of the trimmed sector's links must be on it
}


LINK_MAX_SCALE = 1.005             # the longest belt's element (layout.HIDDEN_RANGES["len_scale"])
I_SECTOR_MIN_LINKS = 1.5           # a judged idler arc must hold >= 1.5 element lengths: any arc that long holds at least
                                   # one element centre, so a correct back wrap never reads 0 elements by phase alone


def idler_sector_links(variant):
    """{idler: the trimmed judged arc of its drawn back wrap, in element lengths of the longest belt}. The wrap of an
    idler between two fixed pulleys does not depend on the tensioner."""
    V = VARIANTS[variant]
    T = V["tensioner"]
    th = 0.5 * (T["window"][0] + T["window"][1])
    tc = np.asarray(T["pivot"]) + float(T["arm"]) * np.array([math.cos(th), math.sin(th)])
    l = BELT["link"] * LINK_MAX_SCALE
    circ = []
    for k in V["route"]:
        if k == "T":
            circ.append(Circle("T", tc, joint_radius(T["r"] + 0.5 * BELT["thick"], l), -1))
        else:
            circ.append(Circle(k, V["pulleys"][k]["c"], joint_radius(V["pulleys"][k]["r"] + 0.5 * BELT["thick"], l), wrap_sense(V, k)))
    _, _, wraps = loop_path(circ, step=0.001)
    out = {}
    for k, P in V["pulleys"].items():
        if P["kind"] == "idler":
            arc = wraps[k][2] - 2.0 * math.radians(ACCEPT["i_sector_trim_deg"])
            out[k] = arc * (P["r"] + 0.5 * BELT["thick"]) / l
    return out


# the coupling of the idler's acceptance to the element pitch, checked when the cell is imported
assert all(v >= I_SECTOR_MIN_LINKS for v in idler_sector_links(CELL_VARIANT).values()), idler_sector_links(CELL_VARIANT)


def _wind(xz, c):
    a = np.arctan2(xz[:, 1] - c[1], xz[:, 0] - c[0])
    da = np.diff(np.concatenate([a, a[:1]]))
    da = (da + math.pi) % (2 * math.pi) - math.pi
    return float(da.sum() / (2 * math.pi))


def inspect_drive(model, data, drive, belt, E_pos, R_E, variant):
    """Per pulley: winding of the belt's joint polygon round the axis, links on the pulley inside the drawing's wrap
    sector (trimmed), their radial lift and axial offset; pass flags per the ACCEPT constants."""
    d = data
    V = VARIANTS[variant]
    t = BELT["thick"]
    C = belt.centres(d)
    loc = (C - E_pos) @ R_E
    pts = (belt.points(d) - E_pos) @ R_E
    xz = pts[:, [0, 2]]
    th = drive.theta(d)
    try:
        _, _, wraps = loop_path(drive.circles(th, belt.l), step=0.002)
    except Exception:
        wraps = {}
    out = {}
    for k in V["route"]:
        P = drive.p[k]
        c = ((d.xpos[P["body"]] - E_pos) @ R_E)[[0, 2]]
        r = P["r"]
        rho = r + 0.5 * t
        w = abs(_wind(xz, c))            # the belt may run either way round; |w| = 1: inside the loop = rib side
        sk = wrap_sense(V, k)            # -1: the back wraps it (T, a backside idler): outside the loop, |w| = 0
        want_w = 0.0 if sk < 0 else 1.0
        rad = np.linalg.norm(loc[:, [0, 2]] - c, axis=1)
        ang = np.arctan2(loc[:, 2] - c[1], loc[:, 0] - c[0])
        depth = -loc[:, 1]
        smooth = k == "T" or V["pulleys"].get(k, {}).get("kind") == "idler"
        zc = 0.5 * (TPULLEY["d_face0"] + TPULLEY["d_face1"]) if smooth else BELT_PLANE_D
        near = (rad < rho + ACCEPT["near_mm"] * 1e-3) & (np.abs(depth - zc) < 0.020)
        res = {"wind": round(w, 3)}
        if k in wraps:
            a_in, a_out, sweep = wraps[k]
            idler = k != "T" and sk < 0
            trim = math.radians(ACCEPT["i_sector_trim_deg"] if idler else ACCEPT["sector_trim_deg"])
            s = sk
            rel = ((ang - a_in) * s) % (2 * math.pi)
            inside = (rel > trim) & (rel < sweep - trim)
            sel = near & inside
            n_exp = max(1.0, (sweep - 2 * trim) * rho / belt.l)
            cover = float(sel.sum()) / n_exp
            min_frac = ACCEPT["i_min_wrap_frac"] if idler else ACCEPT["min_wrap_frac"]
            lift = (rad[sel] - rho) * 1000.0
            axial = np.abs(depth[sel] - zc) * 1000.0
            res.update({"sweep_deg": round(math.degrees(sweep), 1), "n_on": int(sel.sum()), "cover": round(cover, 2),
                        "lift_max_mm": round(float(lift.max()), 2) if sel.any() else None,
                        "axial_max_mm": round(float(axial.max()), 2) if sel.any() else None})
            ok = (abs(w - want_w) < 0.5 and cover >= min_frac and sel.any()
                  and float(lift.max()) <= ACCEPT["lift_mm"]
                  and float(axial.max()) <= (ACCEPT["t_axial_mm"] if smooth else ACCEPT["axial_mm"]))
        else:
            ok = False
        res["ok"] = bool(ok)
        out[k] = res
    W = V["tensioner"]["window"]
    out["tensioner"] = {"theta_deg": round(math.degrees(th), 2), "in_window": bool(W[0] <= th <= W[1]),
                        "omega_dps": round(math.degrees(drive.omega(d)), 2)}
    out["belt_min_z"] = float(np.min(belt.points(d)[:, 2]))
    return out
