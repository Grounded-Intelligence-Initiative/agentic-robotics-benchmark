"""The standard simulation of a twin, its convex pieces and its frames.

One module (stdlib, numpy, mujoco; scipy only for the polyhedra of round primitives) with every number in
twin_sim_config.json next to it. The same code runs in check_twin.py and in the grader, so the physics and the
geometry they see are identical. Nothing here reads a file other than the config, writes a file, reads a clock or
draws a random number.

Roles in a twin (the gate builds them from objects.json):
- the support: the static top-level body named "support"; its collision geoms are boxes;
- the things: top-level bodies with a free joint, each mapped to one thing key; a thing's geoms are the geoms of its
  body and of the bodies below it.
A geom takes part in contacts when its contype or conaffinity is non-zero in the delivered model; visual geoms
(contype = conaffinity = 0) stay inert.

Main entry points:
    roles = Roles(model, {body name: key})
    out = settle(model, roles, held=(), config=None)        # the standard settle (a copy of the model is simulated)
    data = data_at(model, out["qpos_end"])                   # the settled state
    pieces = pieces_from_model(model, data, roles)           # {key: [V (n, 3) float64 mm, world frame], ...}
    top_T = support_frame(model, data, roles)                # 4x4 world_from_top (m)
    T, warning = rehearsal_frame(model, data, top_T)         # top_T turned so that x follows photo_cam's line of sight
    xml, bodies = planning_xml(model, data, roles, T)        # the settled twin as a self-contained MJCF in frame T

The settle. The model is copied, then standardised: mjOption is reset to MuJoCo's defaults and set from the config;
gravity points along -z of the support's top frame; every colliding geom gets the standard contact parameters; every
force that does not come from a contact is cleared (joint stiffness and damping, friction loss, armature, gravity
compensation, surface velocity, adhesion, fluid coefficients). Held things (named by key) are kinematic: a large
armature on their dofs (followed by mj_setConst) and their qpos / qvel written before every step, so contacts never
move them. The run starts at qpos0 with zero velocity and lasts settle.seconds. A thing's displacement is the
displacement of a weighted mean of its collision geoms' frame origins (geom_xpos), each weighted by the volume of the
geom's bounding box (from geom_aabb's half sizes; the box's centre offset is not used; equal weights when no box has a
volume; a thing without a collision geom: its body's centre of mass), its rotation the angle of its body's rotation
relative to the start; both are sampled at every step.
blowup = the state becomes non-finite, MuJoCo reports any warning, or inside settle.explode_window_s a thing moves
faster than explode_speed_m_s or further than explode_disp_mm.

The pieces. Collision geoms only. A mesh geom is the set of vertices of the convex hull MuJoCo compiled for it (the
mesh's scale, pos and quat as MuJoCo compiled them), placed at the geom's pose; a box is its eight corners; a sphere,
capsule, cylinder or ellipsoid is a circumscribed polyhedron (the primitive lies inside it; the largest distance
between the two surfaces is at most pieces.primitive_max_gap_mm). The support's geoms are not returned.

The support's top frame: among the support's box geoms, the one whose upward face (the face whose outward normal has
the largest world z) has the largest area (ties: the lower geom id); origin = that face's centre, z = its outward
normal, x = world x projected on the face (world y when world x lies within 10 deg of the normal), y = z cross x.
The rehearsal frame turns the top frame about its z so that x = photo_cam's optical axis projected on the top plane,
pointing away from the camera. Without photo_cam, or when photo_cam looks straight down, the top frame is returned
unchanged with a warning.
"""
import copy
import json
import math
import os

import mujoco
import numpy as np

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "twin_sim_config.json")
SUPPORT = "support"
CAMERA = "photo_cam"
NO_CAM_WARNING = "no photo_cam: directions are measured from the twin's own x"
DOWN_CAM_WARNING = "photo_cam looks straight down: directions are measured from the twin's own x"
_CONFIG = None

_INTEGRATORS = {"euler": mujoco.mjtIntegrator.mjINT_EULER, "rk4": mujoco.mjtIntegrator.mjINT_RK4,
                "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
                "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST}
_CONES = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL, "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}
_SOLVERS = {"pgs": mujoco.mjtSolver.mjSOL_PGS, "cg": mujoco.mjtSolver.mjSOL_CG, "newton": mujoco.mjtSolver.mjSOL_NEWTON}

# model elements that can hold or move a thing without a contact and that cannot be switched off field by field
FORBIDDEN = (("npair", "explicit contact pairs (<contact><pair>)"),
             ("nexclude", "contact exclusions (<contact><exclude>)"),
             ("neq", "equality constraints"), ("nu", "actuators"), ("ntendon", "tendons"),
             ("nflex", "flex objects"), ("nplugin", "plugins"))


class TwinSimError(ValueError):
    """The model contains something the standard simulation does not accept."""


def load_config(path=None):
    """The parameters (cached for the default path)."""
    global _CONFIG
    if path is None:
        if _CONFIG is None:
            with open(CONFIG_PATH, encoding="utf-8") as f:
                _CONFIG = json.load(f)
        return _CONFIG
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _flag(kind, name):
    enum = mujoco.mjtDisableBit if kind == "disable" else mujoco.mjtEnableBit
    prefix = "mjDSBL_" if kind == "disable" else "mjENBL_"
    return int(getattr(enum, prefix + name.upper()))


def is_collision_geom(model, g):
    """A geom collides when (contype, conaffinity) != (0, 0)."""
    return int(model.geom_contype[g]) != 0 or int(model.geom_conaffinity[g]) != 0


# ---------------------------------------------------------------------------------------------------------------
# roles

class Roles:
    """Which bodies are the support and the things (body <-> key), with the ids of a compiled model.

    body_to_key {body name: key}, key_to_body {key: body name}, support_body (name), support_id (body id),
    item_ids {key: body id}, keys (sorted), item_qadr / item_dadr {key: qpos / dof address of the free joint},
    geom_key [key or None per geom] (collision geoms of a thing), support_geoms [geom ids] (collision geoms whose
    root body is the support)."""

    def __init__(self, model, bodies, support=SUPPORT):
        m = model
        self.model = m
        self.body_to_key = dict(bodies)
        self.key_to_body = {k: b for b, k in self.body_to_key.items()}
        if len(self.key_to_body) != len(self.body_to_key):
            raise TwinSimError("two bodies are mapped to the same key")
        self.support_body = support
        self.support_id = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, support))
        self.item_ids, self.item_qadr, self.item_dadr = {}, {}, {}
        for b, k in sorted(self.body_to_key.items(), key=lambda kv: kv[1]):
            bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b))
            if bid < 0:
                raise TwinSimError("no body named %r" % b)
            if int(m.body_parentid[bid]) != 0 or int(m.body_jntnum[bid]) != 1 or \
                    int(m.jnt_type[int(m.body_jntadr[bid])]) != int(mujoco.mjtJoint.mjJNT_FREE):
                raise TwinSimError("body %r is not a top-level body with a free joint" % b)
            j = int(m.body_jntadr[bid])
            self.item_ids[k] = bid
            self.item_qadr[k] = int(m.jnt_qposadr[j])
            self.item_dadr[k] = int(m.jnt_dofadr[j])
        self.keys = sorted(self.item_ids)
        root = {b: k for k, b in self.item_ids.items()}
        self.geom_key = [None] * int(m.ngeom)
        self.support_geoms = []
        for g in range(m.ngeom):
            if not is_collision_geom(m, g):
                continue
            r = int(m.body_rootid[int(m.geom_bodyid[g])])
            if r in root:
                self.geom_key[g] = root[r]
            elif self.support_id >= 0 and r == self.support_id:
                self.support_geoms.append(g)

    def thing_geoms(self, key):
        return [g for g, k in enumerate(self.geom_key) if k == key]


# ---------------------------------------------------------------------------------------------------------------
# frames

def _pose(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def support_top(model, data, roles):
    """(world_from_top 4x4 m, geom id, face area m2) of the support's top (the module docstring's rule)."""
    best = None
    for g in roles.support_geoms:
        if int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            continue
        R = np.asarray(data.geom_xmat[g], float).reshape(3, 3)
        s = np.asarray(model.geom_size[g], float)
        faces = []
        for i in range(3):
            for sign in (1.0, -1.0):
                n = sign * R[:, i]
                j, k = [a for a in range(3) if a != i]
                faces.append((float(n[2]), 4.0 * float(s[j] * s[k]), n, float(s[i])))
        up = max(faces, key=lambda f: f[0])
        if best is None or up[1] > best[1]:
            best = up + (g,)
    if best is None:
        raise TwinSimError("the support has no box collision geom")
    _nz, area, n, half, g = best
    n = n / np.linalg.norm(n)
    origin = np.asarray(data.geom_xpos[g], float) + n * half
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, n))) > math.cos(math.radians(10.0)):
        ref = np.array([0.0, 1.0, 0.0])
    x = ref - np.dot(ref, n) * n
    x /= np.linalg.norm(x)
    return _pose(np.column_stack([x, np.cross(n, x), n]), origin), int(g), float(area)


def support_frame(model, data, roles):
    """4x4 world_from_top (m): the support's top frame."""
    return support_top(model, data, roles)[0]


def rehearsal_frame(model, data, top_T):
    """(top_T turned about its z so that x = photo_cam's optical axis projected on the top plane, pointing away from
    the camera, warning or None). Without photo_cam (or when it looks straight down) top_T unchanged and a warning."""
    top_T = np.asarray(top_T, dtype=np.float64)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA)
    if cid < 0:
        return top_T.copy(), NO_CAM_WARNING
    axis = -np.asarray(data.cam_xmat[cid], float).reshape(3, 3)[:, 2]          # a MuJoCo camera looks along its -z
    n = top_T[:3, 2]
    x = axis - np.dot(axis, n) * n
    if np.linalg.norm(x) < 1e-9:
        return top_T.copy(), DOWN_CAM_WARNING
    x /= np.linalg.norm(x)
    T = top_T.copy()
    T[:3, :3] = np.column_stack([x, np.cross(n, x), n])
    return T, None


def inv_T(T):
    T = np.asarray(T, float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def to_frame_mm(pieces, world_from_frame):
    """The pieces (mm, world) expressed in a frame (world_from_frame 4x4, metres)."""
    Ti = inv_T(world_from_frame)
    R, t = Ti[:3, :3], 1000.0 * Ti[:3, 3]
    return {k: [np.asarray(V, float) @ R.T + t for V in vs] for k, vs in pieces.items()}


# ---------------------------------------------------------------------------------------------------------------
# the standard simulation

def standardize(model, roles, held=(), config=None):
    """Apply the standard simulation to a compiled model in place (see the module docstring). held: thing keys.
    Raises TwinSimError for elements that cannot be neutralised."""
    cfg = config or load_config()
    m = model
    bad = ["%s (%d)" % (what, int(getattr(m, n))) for n, what in FORBIDDEN if int(getattr(m, n)) > 0]
    if bad:
        raise TwinSimError("the standard simulation does not accept " + ", ".join(bad))
    d0 = mujoco.MjData(m)
    mujoco.mj_forward(m, d0)
    up = support_frame(m, d0, roles)[:3, 2]
    default = mujoco.MjOption()
    for k in ("timestep", "impratio", "tolerance", "ls_tolerance", "noslip_tolerance", "ccd_tolerance",
              "density", "viscosity", "o_margin", "integrator", "cone", "jacobian", "solver", "iterations",
              "ls_iterations", "noslip_iterations", "ccd_iterations", "disableflags", "enableflags",
              "disableactuator", "sdf_initpoints", "sdf_iterations", "sleep_tolerance"):
        if hasattr(default, k):
            setattr(m.opt, k, getattr(default, k))
    for k in ("gravity", "wind", "magnetic", "o_solref", "o_solimp", "o_friction"):
        getattr(m.opt, k)[:] = getattr(default, k)
    s = cfg["sim"]
    m.opt.timestep = float(s["timestep"])
    m.opt.integrator = int(_INTEGRATORS[s["integrator"].lower()])
    m.opt.cone = int(_CONES[s["cone"].lower()])
    m.opt.impratio = float(s["impratio"])
    m.opt.solver = int(_SOLVERS[s["solver"].lower()])
    m.opt.iterations = int(s["iterations"])
    m.opt.ls_iterations = int(s["ls_iterations"])
    m.opt.tolerance = float(s["tolerance"])
    m.opt.noslip_iterations = int(s["noslip_iterations"])
    if s["gravity_frame"] != "support_top":
        raise TwinSimError("unknown gravity_frame %r" % s["gravity_frame"])
    m.opt.gravity[:] = -float(s["gravity_m_s2"]) * up
    for f in s["disable_flags"]:
        m.opt.disableflags |= _flag("disable", f)
    for f in s["enable_flags"]:
        m.opt.enableflags |= _flag("enable", f)
    c = cfg["contact"]
    for g in range(m.ngeom):
        if not is_collision_geom(m, g):
            continue
        m.geom_condim[g] = int(c["condim"])
        m.geom_solref[g] = c["solref"]
        m.geom_solimp[g] = c["solimp"]
        m.geom_margin[g] = float(c["margin_m"])
        m.geom_gap[g] = float(c["gap_m"])
        m.geom_priority[g] = int(c["priority"])
        m.geom_solmix[g] = float(c["solmix"])
        if c["override_friction"]:
            m.geom_friction[g] = c["friction"]
        m.geom_contype[g] = int(c["contype"])
        m.geom_conaffinity[g] = int(c["conaffinity"])
    m.jnt_stiffness[:] = 0.0
    if hasattr(m, "jnt_stiffnesspoly"):
        m.jnt_stiffnesspoly[:] = 0.0
    m.dof_damping[:] = 0.0
    if hasattr(m, "dof_dampingpoly"):
        m.dof_dampingpoly[:] = 0.0
    m.dof_frictionloss[:] = 0.0
    m.dof_armature[:] = 0.0
    m.body_gravcomp[:] = 0.0
    m.geom_surfacevel[:] = 0.0
    m.geom_adhesion[:] = 0.0
    m.geom_fluid[:] = 0.0
    kin = float(cfg["kinematic"]["armature"])
    for k in held:
        if k in roles.item_dadr:
            a = roles.item_dadr[k]
            m.dof_armature[a:a + 6] = kin
    d = mujoco.MjData(m)
    mujoco.mj_setConst(m, d)
    return m


def _measure_weights(model, roles):
    """(keys, geoms, W): W (n_keys, n_geoms) weights of each thing's collision geoms (bounding-box volume)."""
    keys = list(roles.keys)
    geoms, rows, cols, w = [], [], [], []
    for i, k in enumerate(keys):
        gs = roles.thing_geoms(k)
        if not gs:
            continue
        vol = np.array([8.0 * float(np.prod(np.maximum(model.geom_aabb[g, 3:6], 0.0))) for g in gs])
        vol = np.where(np.isfinite(vol) & (vol > 0), vol, 0.0)
        if vol.sum() <= 0:
            vol = np.ones(len(gs))
        for g, v in zip(gs, vol / vol.sum()):
            rows.append(i)
            cols.append(len(geoms))
            w.append(v)
            geoms.append(g)
    W = np.zeros((len(keys), len(geoms)))
    W[rows, cols] = w
    return keys, np.array(geoms, int), W


def _quat_angles_deg(qa, qb):
    dot = np.abs(np.sum(qa * qb, axis=1))
    return np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0)))


def data_at(model, qpos=None):
    """MjData at qpos (default qpos0) with zero velocity, after mj_forward."""
    d = mujoco.MjData(model)
    d.qpos[:] = model.qpos0 if qpos is None else np.asarray(qpos, float)
    d.qvel[:] = 0.0
    mujoco.mj_forward(model, d)
    return d


def settle(model, roles, held=(), config=None):
    """The standard settle of a copy of the model from qpos0 (held: thing keys, kept kinematic).

    Returns {"poses": {key: 4x4 world_from_body (m)}, "disp_mm", "rot_deg" (at the end), "max_disp_mm",
    "max_rot_deg" (largest over every step), "blowup", "finite", "exploded", "warnings", "held", "seconds", "steps",
    "qpos_end"}."""
    cfg = config or load_config()
    held = [k for k in held if k in roles.item_ids]
    m = standardize(copy.deepcopy(model), roles, held, cfg)
    st = cfg["settle"]
    dt = float(m.opt.timestep)
    n = int(round(float(st["seconds"]) / dt))
    nexp = int(round(float(st["explode_window_s"]) / dt))
    vmax = float(st["explode_speed_m_s"])
    dmax = float(st["explode_disp_mm"])
    d = data_at(m)
    keys, geoms, W = _measure_weights(m, roles)
    ids = np.array([roles.item_ids[k] for k in keys], int)
    dadr = np.array([roles.item_dadr[k] for k in keys], int)
    has = W.sum(axis=1) > 0

    def points():
        P = W @ d.geom_xpos[geoms] if len(geoms) else np.zeros((len(keys), 3))
        if not has.all():
            P[~has] = d.xipos[ids[~has]]
        return P

    hold = [(roles.item_qadr[k], roles.item_dadr[k], d.qpos[roles.item_qadr[k]:roles.item_qadr[k] + 7].copy())
            for k in held]
    P0, quat0 = points(), d.xquat[ids].copy()
    nk = len(keys)
    maxd, maxr, dr, rr = np.zeros(nk), np.zeros(nk), np.zeros(nk), np.zeros(nk)
    finite, exploded = True, False
    for k in range(n):
        for qa, da, q in hold:
            d.qpos[qa:qa + 7] = q
            d.qvel[da:da + 6] = 0.0
        mujoco.mj_step(m, d)
        if not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))):
            finite = False
            break
        dr = 1000.0 * np.linalg.norm(points() - P0, axis=1)
        rr = _quat_angles_deg(quat0, d.xquat[ids])
        np.maximum(maxd, dr, out=maxd)
        np.maximum(maxr, rr, out=maxr)
        if k < nexp and nk and not exploded:
            v = np.linalg.norm(d.qvel[dadr[:, None] + np.arange(3)[None, :]], axis=1)
            if (dr > dmax).any() or (v > vmax).any():
                exploded = True
    warn = {str(mujoco.mjtWarning(i)).split(".")[-1]: int(d.warning[i].number) for i in range(len(d.warning))
            if d.warning[i].number}
    if finite:
        for qa, da, q in hold:                       # the held things end exactly where they started
            d.qpos[qa:qa + 7] = q
            d.qvel[da:da + 6] = 0.0
        mujoco.mj_forward(m, d)
    out = {"poses": {}, "disp_mm": {}, "rot_deg": {}, "max_disp_mm": {}, "max_rot_deg": {}}
    for i, key in enumerate(keys):
        if finite:
            out["poses"][key] = _pose(np.asarray(d.xmat[ids[i]], float).reshape(3, 3),
                                      np.asarray(d.xpos[ids[i]], float)).tolist()
            vals = (float(dr[i]), float(rr[i]), float(maxd[i]), float(maxr[i]))
        else:
            out["poses"][key] = None
            vals = (float("inf"),) * 4
        for name, v in zip(("disp_mm", "rot_deg", "max_disp_mm", "max_rot_deg"), vals):
            out[name][key] = round(v, 6) if math.isfinite(v) else v
    out.update({"blowup": (not finite) or exploded or bool(warn), "finite": finite, "exploded": exploded,
                "warnings": warn, "held": sorted(held), "seconds": n * dt, "steps": n,
                "qpos_end": [float(v) for v in d.qpos]})
    return out


# ---------------------------------------------------------------------------------------------------------------
# convex pieces

_UNIT_SPHERE = {}


def _fibonacci(n):
    k = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * k / n)
    th = math.pi * (1.0 + 5 ** 0.5) * k
    return np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]


def unit_sphere_polytope(eps):
    """Vertices (n, 3) of a polyhedron that contains the unit ball and whose vertices lie at most eps outside it: n
    spread points on the sphere scaled by 1 / (the smallest distance of a hull facet from the centre), n doubled from
    32 until the scale is at most 1 + eps."""
    from scipy.spatial import ConvexHull
    if not eps > 0:
        raise ValueError("eps must be positive")
    n = 32
    while True:
        if n in _UNIT_SPHERE:
            U, s = _UNIT_SPHERE[n]
        else:
            U = _fibonacci(n)
            h = ConvexHull(U)
            s = 1.0 / float(np.min(-h.equations[:, 3]))
            _UNIT_SPHERE[n] = (U, s)
        if s - 1.0 <= eps:
            return U * s
        if n >= 1 << 20:
            raise ValueError("no polyhedron of at most %d vertices within %g of the unit sphere" % (n, eps))
        n *= 2


def primitive_vertices(kind, size, max_gap):
    """Local vertices (n, 3) of the circumscribed polyhedron of a primitive (the geom frame; lengths in the units of
    size and max_gap)."""
    s = np.asarray(size, float)
    if kind == "box":
        return np.array([[a * s[0], b * s[1], c * s[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    if kind == "sphere":
        r = float(s[0])
        return r * unit_sphere_polytope(max_gap / r)
    if kind == "ellipsoid":
        rad = s[:3]
        return unit_sphere_polytope(max_gap / float(np.max(rad))) * rad
    if kind == "capsule":
        r, h = float(s[0]), float(s[1])
        U = r * unit_sphere_polytope(max_gap / r)
        return np.vstack([U + [0.0, 0.0, h], U - [0.0, 0.0, h]])
    if kind == "cylinder":
        r, h = float(s[0]), float(s[1])
        k = 8
        while r / math.cos(math.pi / k) - r > max_gap:
            k *= 2
        a = 2.0 * math.pi * (np.arange(k) + 0.5) / k
        R = r / math.cos(math.pi / k)
        ring = np.c_[R * np.cos(a), R * np.sin(a)]
        return np.vstack([np.c_[ring, np.full(k, h)], np.c_[ring, np.full(k, -h)]])
    raise ValueError("not a primitive: %r" % kind)


_KINDS = {int(mujoco.mjtGeom.mjGEOM_BOX): "box", int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
          int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule", int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
          int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid"}


def mesh_hull_local(model, mesh_id):
    """The vertices (n, 3) of the convex hull MuJoCo compiled for a mesh, in the geom frame (all the mesh's vertices
    when MuJoCo has no hull for it)."""
    va, vn = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
    V = np.asarray(model.mesh_vert[va:va + vn], float)
    adr = int(model.mesh_graphadr[mesh_id])
    if adr < 0:
        return V
    g = model.mesh_graph[adr:]
    nv, nf = int(g[0]), int(g[1])
    if nv < 4 or nf < 4:
        return V
    off = 2 + 2 * nv + (nv + 3 * nf)
    faces = np.asarray(g[off:off + 3 * nf], np.int64)
    return V[np.unique(faces)]


def geom_vertices_mm(model, data, g, max_gap_mm=None):
    """World vertices (mm) of the convex piece of collision geom g (None for a plane or another type)."""
    gap = float(load_config()["pieces"]["primitive_max_gap_mm"] if max_gap_mm is None else max_gap_mm)
    t = int(model.geom_type[g])
    R = np.asarray(data.geom_xmat[g], float).reshape(3, 3)
    p = np.asarray(data.geom_xpos[g], float)
    if t == int(mujoco.mjtGeom.mjGEOM_MESH):
        V = mesh_hull_local(model, int(model.geom_dataid[g]))
    elif t in _KINDS:
        V = primitive_vertices(_KINDS[t], model.geom_size[g], gap / 1000.0)
    else:
        return None
    return (V @ R.T + p) * 1000.0


def pieces_from_model(model, data, roles):
    """{key: [V (n, 3) float64 mm, world frame]} of every thing's collision geoms (geom id order; keys sorted)."""
    out = {}
    for k in roles.keys:
        vs = []
        for g in roles.thing_geoms(k):
            V = geom_vertices_mm(model, data, g)
            if V is not None:
                vs.append(np.ascontiguousarray(V, dtype=np.float64))
        out[k] = vs
    return out


# ---------------------------------------------------------------------------------------------------------------
# the settled twin as a self-contained MJCF (for lift rehearsals on the twin)

def _f(v):
    return " ".join("%.10g" % float(x) for x in np.ravel(v))


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def composite_inertia(model, data, body):
    """(mass, centre of mass (3,), inertia about it (3, 3)) of a body and every body below it, world frame."""
    bodies = [i for i in range(1, model.nbody) if int(model.body_rootid[i]) == int(model.body_rootid[body])
              and _below(model, i, body)]
    M = float(sum(float(model.body_mass[i]) for i in bodies))
    if not M > 0:
        return M, np.asarray(data.xipos[body], float).copy(), np.zeros((3, 3))
    C = sum(float(model.body_mass[i]) * np.asarray(data.xipos[i], float) for i in bodies) / M
    I = np.zeros((3, 3))
    for i in bodies:
        Ri = np.asarray(data.ximat[i], float).reshape(3, 3)
        r = np.asarray(data.xipos[i], float) - C
        I += Ri @ np.diag(np.asarray(model.body_inertia[i], float)) @ Ri.T + float(model.body_mass[i]) * (
            (r @ r) * np.eye(3) - np.outer(r, r))
    return M, C, 0.5 * (I + I.T)


def _below(model, i, body):
    while i > 0:
        if i == body:
            return True
        i = int(model.body_parentid[i])
    return False


def planning_xml(model, data, roles, frame_T, held=(), config=None):
    """(MJCF text, {key: body name}) of the twin at data's state expressed in the frame frame_T (world_from_frame):
    world = that frame, a plane geom "plane" at z = 0 (the support's top), one body per thing named by its key (a free
    joint, none for a held thing) with its own mass, centre of mass and inertia (its body and the bodies below it)
    as an explicit <inertial>, its convex pieces as inline meshes, and the standard simulation's options (gravity along
    -z)."""
    cfg = config or load_config()
    s, c = cfg["sim"], cfg["contact"]
    Ti = inv_T(frame_T)
    Rf, tf = Ti[:3, :3], Ti[:3, 3]
    assets, bodies = [], []
    for k in roles.keys:
        b = roles.item_ids[k]
        Rb = Rf @ np.asarray(data.xmat[b], float).reshape(3, 3)
        pb = Rf @ np.asarray(data.xpos[b], float) + tf
        Rbi = np.asarray(data.xmat[b], float).reshape(3, 3).T
        mass, C, I_world = composite_inertia(model, data, b)
        ipos = Rbi @ (C - np.asarray(data.xpos[b], float))
        I_local = Rbi @ I_world @ Rbi.T
        geoms = []
        for j, g in enumerate(roles.thing_geoms(k)):
            V = geom_vertices_mm(model, data, g)
            if V is None:
                continue
            Vl = (V / 1000.0 - np.asarray(data.xpos[b], float)) @ Rbi.T
            nm = "%s__%02d" % (k, j)
            assets.append('<mesh name="%s" vertex="%s"/>' % (nm, _f(Vl)))
            geoms.append('<geom name="%s" type="mesh" mesh="%s" mass="0"/>' % (nm, nm))
        full = [I_local[0, 0], I_local[1, 1], I_local[2, 2], I_local[0, 1], I_local[0, 2], I_local[1, 2]]
        inertial = '<inertial pos="%s" mass="%.10g" fullinertia="%s"/>' % (_f(ipos), mass, _f(full))
        joint = "" if k in held else '<freejoint name="%s__free"/>' % k
        bodies.append('<body name="%s" pos="%s" quat="%s">%s%s%s</body>' % (k, _f(pb), _f(_quat(Rb)), joint,
                                                                            inertial, "".join(geoms)))
    flags = "".join(' %s="disable"' % f for f in s["disable_flags"])
    xml = ('<mujoco model="twin in the planning frame"><compiler angle="radian" inertiafromgeom="auto"/>'
           '<option timestep="%.10g" integrator="%s" cone="%s" impratio="%.10g" solver="%s" iterations="%d" '
           'ls_iterations="%d" tolerance="%.10g" noslip_iterations="%d" gravity="0 0 %.10g">'
           '<flag%s/></option>'
           '<default><geom condim="%d" solref="%s" solimp="%s" friction="%s" margin="%.10g" gap="%.10g" '
           'contype="%d" conaffinity="%d"/></default>'
           '<asset>%s</asset><worldbody><geom name="plane" type="plane" size="5 5 0.1"/>%s</worldbody></mujoco>') % (
        float(s["timestep"]), s["integrator"], s["cone"], float(s["impratio"]), s["solver"], int(s["iterations"]),
        int(s["ls_iterations"]), float(s["tolerance"]), int(s["noslip_iterations"]), -float(s["gravity_m_s2"]),
        flags, int(c["condim"]), _f(c["solref"]), _f(c["solimp"]), _f(c["friction"]), float(c["margin_m"]),
        float(c["gap_m"]), int(c["contype"]), int(c["conaffinity"]), "".join(assets), "".join(bodies))
    return xml, {k: k for k in roles.keys}
