"""calib_cell.py: the calibration cell. One unit of the arm, its datum spheres, its touch probe.

HIDDEN: the agent does not have this file.

The lab service drives a `Cell` for the agent; the grader uses `draw_unit`, `write_conditions`,
`acceptance_errors_from_predictions` and `run_job` on the same units. Time is virtual: nothing here
waits.

World: the cell's datum frame, z up, the table at z = 0. Twelve datum spheres (radius
R_SPHERE) stand on vertical posts. A unit is
  * one draw of the 84 parameters of arm_model.py (the manufacturer's first-order model, public),
  * plus 2 to 4 of the six defect classes of defects.md (`DEFECT_CLASSES`; which ones, on which
    joints and how large is hidden and drawn per unit), implemented in `true_frames`,
  * plus its cell: knocked datum spheres, false-trigger rate, heating, cooling, repeatability
    (`CELL`-ranges below; hidden per unit, the documents give only ranges or a description),
  * plus its gauge plate (`draw_plate`: the pose inside the documented ranges, its own stream),
    which the stylus must never touch (a collision) and which the jobs' pin enters (`run_job`),
  * plus its warm job's start temperatures (`warm_start`, its own stream; published in the conditions file),
  * plus its service (`draw_service`, its own stream; hidden): after the measuring session one joint's
    drive is replaced, which changes that joint's zero offset and backlash (`serviced`: the unit after the
    service, measured in its re-qualification session and running the serviced job).

Images are drawn by cell_camera.py (hidden): a deterministic NumPy ray caster with per-unit
optics (focal length, principal point, radial distortion, focus, blur), lamps, materials and
sensor (vignetting, automatic exposure, noise, hot and dead pixels); camera.md describes the
camera in words only.

The figures the documents state (spheres as surveyed, tools, speeds, ranges, tolerances, budget,
acceptance limits) are in cell_data.py, which the agent has.
"""
import hashlib

import numpy as np

import arm_model as am
import cell_camera

from cell_data import *  # noqa: F401,F403 - the documented figures
from cell_data import D


def _rng(*key):
    digest = hashlib.blake2b(repr(key).encode(), digest_size=8, person=b"calibcell").digest()
    return np.random.default_rng(int.from_bytes(digest, "big"))


# ---- defects beyond the first-order model (defects.md states the forms and the bounds) ----------
DEFECT_CLASSES = ("coupling", "ratio", "deflection", "cyclic", "thermal", "pedestal")
# Upper bounds (public, in defects.md). A present class draws each magnitude uniform in
# [DEFECT_LOW, 1] times its bound (hidden), with a random sign where the sign is free.
COUPLING_MAX = 8.0e-4                    # rad per rad: k45, k56
RATIO_JOINTS = (0, 3, 5)                 # the joints that turn far enough for a ratio error to show
RATIO_MAX = 1.2e-3                       # rad per rad, e_j
DEFLECT_MAX = np.array([3.0e-4, 3.0e-4, 3.0e-4, 3.0e-4])   # rad / (N m): upper arm torsion, bending; forearm torsion, bending
CYCLIC_HO_MAX = 1.0e-3                   # rad: amplitude of the higher harmonic
CYCLIC_HO_JOINTS = (0, 1, 2)
CYCLIC_HO_ORDERS = (3, 4, 5, 6, 7, 8)
THERMAL_ZERO_JOINTS = (1, 2, 4)
THERMAL_ZERO_MAX = np.array([0.0, 5.0e-5, 6.0e-5, 0.0, 1.0e-4, 0.0])    # rad / K, per joint
PEDESTAL_MAX = 5.0e-5                    # rad / (N m), about each horizontal axis
DEFECT_LOW = 0.4                         # hidden: a present defect is at least this fraction of its bound
N_CLASSES = (2, 4)                       # hidden: how many classes a unit has

# ---- the cell around a unit (hidden per unit; the documents give the ranges of cell_data.py) ----
NEIGHBOUR_HEAT = (0.05, 0.30)            # hidden: a link's warming by each neighbouring joint, relative to its own joint's


def zero_defects():
    return {"classes": [], "k45": 0.0, "k56": 0.0, "ratio": np.zeros(6), "deflect": np.zeros(4),
            "cyc_joint": 0, "cyc_order": 3, "cyc_ab": np.zeros(2), "thermal": np.zeros(6),
            "pedestal": np.zeros(2)}


def draw_defects(seed, force_classes=None):
    """The unit's defects. force_classes (studies only): these classes instead of the drawn ones."""
    rng = _rng("defects", seed)
    d = zero_defects()
    n = int(rng.integers(N_CLASSES[0], N_CLASSES[1] + 1))
    classes = sorted(rng.choice(len(DEFECT_CLASSES), size=n, replace=False))
    d["classes"] = [DEFECT_CLASSES[i] for i in classes] if force_classes is None else list(force_classes)

    def mag(size=None):
        return rng.uniform(DEFECT_LOW, 1.0, size)

    def sign(size=None):
        return rng.choice([-1.0, 1.0], size)
    for c in d["classes"]:
        if c == "coupling":
            d["k45"], d["k56"] = mag(2) * sign(2) * COUPLING_MAX
        elif c == "ratio":
            js = rng.choice(RATIO_JOINTS, size=int(rng.integers(1, 3)), replace=False)
            for j in js:
                d["ratio"][j] = mag() * sign() * RATIO_MAX
        elif c == "deflection":
            d["deflect"] = mag(4) * DEFLECT_MAX               # compliances: positive
        elif c == "cyclic":
            d["cyc_joint"] = int(rng.choice(CYCLIC_HO_JOINTS))
            d["cyc_order"] = int(rng.choice(CYCLIC_HO_ORDERS))
            a, ph = mag() * CYCLIC_HO_MAX, rng.uniform(0, 2 * np.pi)
            d["cyc_ab"] = np.array([a * np.cos(ph), a * np.sin(ph)])     # a sin(n q + ph) = A sin nq + B cos nq
        elif c == "thermal":
            js = rng.choice(THERMAL_ZERO_JOINTS, size=int(rng.integers(1, 3)), replace=False)
            for j in js:
                d["thermal"][j] = mag() * sign() * THERMAL_ZERO_MAX[j]
        elif c == "pedestal":
            d["pedestal"] = mag(2) * PEDESTAL_MAX             # compliances: positive
    return d


def draw_cell(seed):
    """The cell around the unit: knocked spheres, false triggers, heating, cooling, repeatability."""
    rng = _rng("cell", seed)
    centres = surveyed_spheres()
    knocked = rng.choice(12, size=int(rng.integers(KNOCKED_SPHERES[0], KNOCKED_SPHERES[1] + 1)), replace=False)
    for k in knocked:
        v = rng.normal(size=3)
        centres[k] += v / np.linalg.norm(v) * rng.uniform(KNOCK_MIN, KNOCK_MAX)
    heat = HEAT_NOMINAL * rng.uniform(*HEAT_FACTOR_RANGE, 6)
    # heat[i, j]: K per radian joint j turns, for link i (own joint on the diagonal, neighbours beside)
    H = np.diag(heat)
    for i in range(6):
        for j in (i - 1, i + 1):
            if 0 <= j < 6:
                H[i, j] = rng.uniform(*NEIGHBOUR_HEAT) * heat[j]
    return {"centres": centres, "knocked": sorted(int(k) for k in knocked),
            "false_trigger": float(rng.uniform(*FALSE_TRIGGER_RANGE)), "heat": H,
            "tau_cool": float(rng.uniform(*TAU_COOL_RANGE)), "sigma_repeat": float(rng.uniform(*SIGMA_REPEAT_RANGE))}


def draw_plate(seed):
    """The gauge plate's pose in the unit's cell (its own stream: nothing else of the unit changes with it)."""
    rng = _rng("plate", seed)
    x = PLATE_XY_NOMINAL[0] + rng.uniform(-PLATE_XY_TOL, PLATE_XY_TOL)
    y = PLATE_XY_NOMINAL[1] + rng.uniform(-PLATE_XY_TOL, PLATE_XY_TOL)
    yaw = PLATE_YAW_NOMINAL + rng.uniform(-PLATE_YAW_TOL, PLATE_YAW_TOL)
    tx, ty = rng.uniform(-PLATE_TILT_MAX, PLATE_TILT_MAX, 2)
    lift = rng.uniform(*PLATE_LIFT)
    R, O = plate_frame(x, y, lift, yaw, tx, ty)
    # each shim: a vertical cylinder from the table up to the plate's bottom face at its axis
    feet = O[None] + np.column_stack([SHIMS, np.full(len(SHIMS), -PLATE_SIZE[2])]) @ R.T
    return {"R": R, "O": O, "pose": (float(x), float(y), float(lift), float(yaw), float(tx), float(ty)),
            "shims": feet}


def warm_start(seed):
    """The warm job's start (per unit, published in its conditions file): six link temperatures, each uniform
    in JOB_WARM_TEMP (its own stream: nothing else of the unit changes with it)."""
    return _rng("warm start", seed).uniform(JOB_WARM_TEMP[0], JOB_WARM_TEMP[1], 6)


def draw_service(seed):
    """The unit's service (its own stream: nothing else of the unit changes with it): the serviced joint (one of
    SERVICE_JOINTS), the change of its zero offset (magnitude uniform in SERVICE_ZERO, either sign) and its new
    backlash (uniform in the documented range of section 3, as a new unit's)."""
    rng = _rng("service", seed)
    j = int(rng.choice(SERVICE_JOINTS))
    zero = float(rng.uniform(*SERVICE_ZERO) * rng.choice([-1.0, 1.0]))
    return {"joint": j, "zero": zero, "backlash": float(rng.uniform(0.2, 1.0) * BACKLASH_MAX[j])}


def serviced(unit):
    """The unit after its service: the serviced joint's zero offset (the third rpy angle of its frame, which acts
    about the joint's axis) moved by the service's change, its backlash replaced; everything else (the chain's
    geometry, the other first-order parameters, the defects, the camera, the cell, the plate) unchanged."""
    sv = draw_service(unit["seed"])
    p = unit["params"].copy()
    j = sv["joint"]
    p[am.SLICES["rpy"].start + 3 * j + 2] += sv["zero"]
    p[am.SLICES["b"].start + j] = sv["backlash"]
    return dict(unit, params=p, service=sv)


def draw_unit(seed):
    """A unit: the true parameters of arm_model, its defects, and its cell (sphere centres, ...)."""
    rng = _rng("unit", seed)
    p = am.nominal()
    xyz = rng.uniform(-TOL_XYZ, TOL_XYZ, (6, 3))
    xyz[0] = rng.uniform(-TOL_BASE_XYZ, TOL_BASE_XYZ, 3)
    rpy = rng.uniform(-1.0, 1.0, (6, 3)) * TOL_RPY
    rpy[0] = rng.uniform(-TOL_BASE_RPY, TOL_BASE_RPY, 3)
    p[am.SLICES["xyz"]] += xyz.ravel()
    p[am.SLICES["rpy"]] += rpy.ravel()
    p[am.SLICES["b"]] = rng.uniform(0.2, 1.0, 6) * BACKLASH_MAX
    amp = rng.uniform(0.3, 1.0, (6, 2)) * CYCLIC_MAX[:, None]
    ph = rng.uniform(0, 2 * np.pi, (6, 2))
    p[am.SLICES["h"]] = np.stack([amp[:, 0] * np.cos(ph[:, 0]), amp[:, 0] * np.sin(ph[:, 0]),
                                  amp[:, 1] * np.cos(ph[:, 1]), amp[:, 1] * np.sin(ph[:, 1])], axis=1).ravel()
    p[am.SLICES["c"]] = rng.uniform(0.3, 1.0, 6) * COMPLIANCE_MAX
    p[am.SLICES["kappa"]] = rng.uniform(KAPPA_RANGE[0], KAPPA_RANGE[1], 6)
    cam = _rng("camera", seed)            # its own stream: the arm's draw is as it was before the camera
    p[am.SLICES["camera_xyz"]] += cam.uniform(-TOL_CAMERA_XYZ, TOL_CAMERA_XYZ, 3)
    p[am.SLICES["camera_rpy"]] += cam.uniform(-TOL_CAMERA_RPY, TOL_CAMERA_RPY, 3)
    cell = draw_cell(seed)
    return {"seed": seed, "params": p, "defects": draw_defects(seed), "cell": cell,
            "centres": cell["centres"], "knocked": cell["knocked"], "camera": cell_camera.draw_camera(seed),
            "plate": draw_plate(seed), "warm_start": warm_start(seed)}


# ---- the true kinematics: the first-order model plus the unit's defects -------------------------
def _rx_batch(a):
    c, s = np.cos(a), np.sin(a)
    out = np.zeros(a.shape + (3, 3))
    out[..., 0, 0], out[..., 1, 1], out[..., 1, 2], out[..., 2, 1], out[..., 2, 2] = 1.0, c, -s, s, c
    return out


def _ry_batch(a):
    c, s = np.cos(a), np.sin(a)
    out = np.zeros(a.shape + (3, 3))
    out[..., 0, 0], out[..., 0, 2], out[..., 1, 1], out[..., 2, 0], out[..., 2, 2] = c, s, 1.0, -s, c
    return out


def gravity_moments(q, tool):
    """Moments of the weights (nominal chain at the encoder readings, as in am.gravity_torques):
    about the datum origin, of links 0..5 and the tool (N, 3); about frame 1's origin, of links
    1..5 and the tool, on frame 1's x and y axes (N, 2); about frame 2's origin, of links 2..5 and
    the tool, on frame 2's x and y axes (N, 2)."""
    R, o = am._chain(am.NOMINAL_XYZ, am.NOMINAL_RPY, q)
    com = o + np.einsum("njab,jb->nja", R, am.LINK_COM)
    flange = o[:, 5] + R[:, 5, :, 2] * am.FLANGE_Z
    toolc = flange + np.einsum("nab,b->na", R[:, 5], np.asarray(tool["com"], float))
    pos = np.concatenate([com, toolc[:, None, :]], axis=1)                 # (N, 7, 3)
    w = -np.concatenate([am.LINK_MASS, [tool["mass"]]]) * am.G               # force z of each body
    # r x (0, 0, w) = (r_y w, -r_x w, 0)

    def moment(first, origin):
        r = pos[:, first:, :] - origin[:, None, :]
        return np.stack([(r[..., 1] * w[first:]).sum(1), -(r[..., 0] * w[first:]).sum(1), np.zeros(len(q))], axis=1)
    m0 = moment(0, np.zeros((len(q), 3)))
    m1 = moment(1, o[:, 1])
    m2 = moment(2, o[:, 2])
    up = np.stack([np.einsum("na,na->n", m1, R[:, 1, :, 0]), np.einsum("na,na->n", m1, R[:, 1, :, 1])], axis=1)
    fo = np.stack([np.einsum("na,na->n", m2, R[:, 2, :, 0]), np.einsum("na,na->n", m2, R[:, 2, :, 1])], axis=1)
    return m0, up, fo


def defect_angles(d, q, temp):
    """What the defects add to the joint angles theta (N, 6)."""
    extra = q * d["ratio"][None, :]
    extra[:, 4] += d["k45"] * q[:, 3]
    extra[:, 5] += d["k56"] * q[:, 4]
    j, n = d["cyc_joint"], d["cyc_order"]
    if np.any(d["cyc_ab"]):
        extra[:, j] += d["cyc_ab"][0] * np.sin(n * q[:, j]) + d["cyc_ab"][1] * np.cos(n * q[:, j])
    extra += d["thermal"][None, :] * (temp - am.T0)
    return extra


def true_frames(unit, q, s, temp, tool):
    """The true frame 5 of the unit: rotation (N, 3, 3) and origin (N, 3), datum frame."""
    p, d = unit["params"], unit["defects"]
    q = np.atleast_2d(np.asarray(q, float))
    s = np.broadcast_to(s, q.shape)
    temp = np.broadcast_to(temp, q.shape)
    tau = am.gravity_torques(q, tool["mass"], tool["com"])
    theta = am.joint_angles(p, q, s, tau) + defect_angles(d, q, temp)
    scale = 1.0 + p[am.SLICES["kappa"]] * (temp - am.T0)
    xyz, rpy = p[am.SLICES["xyz"]].reshape(6, 3), p[am.SLICES["rpy"]].reshape(6, 3)
    n = len(q)
    if np.any(d["pedestal"]) or np.any(d["deflect"]):
        m0, up, fo = gravity_moments(q, tool)
    if np.any(d["pedestal"]):
        R = _rx_batch(d["pedestal"][0] * m0[:, 0]) @ _ry_batch(d["pedestal"][1] * m0[:, 1])
    else:
        R = np.broadcast_to(np.eye(3), (n, 3, 3))
    o = np.zeros((n, 3))
    R2 = None
    for j in range(6):
        o = o + np.einsum("nij,nj->ni", R, xyz[j][None, :] * scale[:, j:j + 1])
        if j == 2 and np.any(d["deflect"][:2]):            # at the upper arm's end, in frame 1's axes
            R = R @ _rx_batch(d["deflect"][0] * up[:, 0]) @ _ry_batch(d["deflect"][1] * up[:, 1])
        if j == 4 and np.any(d["deflect"][2:]):            # at the forearm's end (the wrist), in frame 2's axes
            Df = _rx_batch(d["deflect"][2] * fo[:, 0]) @ _ry_batch(d["deflect"][3] * fo[:, 1])
            R = R2 @ Df @ np.swapaxes(R2, 1, 2) @ R
        fixed = am._rx(rpy[j, 0]) @ am._ry(rpy[j, 1]) @ am._rz(rpy[j, 2])
        R = R @ fixed @ am._rz_batch(theta[:, j])
        if j == 2:
            R2 = R
    return R, o


def true_tip(unit, q, s, temp, tool):
    R, o = true_frames(unit, q, s, temp, tool)
    return o + np.einsum("nab,b->na", R, np.asarray(tool["tip"], float) + np.array([0.0, 0.0, am.FLANGE_Z]))


def true_camera(unit, q, s, temp, tool):
    """The true camera frame (axes Rc (N, 3, 3) as columns, centre (N, 3)), datum frame."""
    p = unit["params"]
    R, o = true_frames(unit, q, s, temp, tool)
    cam = p[am.SLICES["camera_xyz"]] + np.array([0.0, 0.0, am.FLANGE_Z])
    Rc = np.einsum("nab,bc->nac", R, am.camera_rotation(p[am.SLICES["camera_rpy"]]))
    return Rc, o + np.einsum("nab,b->na", R, cam)


def _segment_distance(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


# ---- the gauge plate's geometry ---------------------------------------------------------------------
PLATE_PARTS = ("top face", "side face", "bore wall", "bore bottom", "bottom face")


def plate_distance(plate, P, parts=False):
    """Distance (m) from points P (N, 3) to the plate's solid (the block less its bores), 0 inside it; with
    `parts`, also the index into PLATE_PARTS of the nearest surface."""
    a, b, T = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2, PLATE_SIZE[2]
    p = (np.asarray(P, float) - plate["O"]) @ plate["R"]
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    ex, ey = np.maximum(np.abs(x) - a, 0.0), np.maximum(np.abs(y) - b, 0.0)
    rect = np.hypot(ex, ey)                       # in the plate's plane, to its outline (0 inside)
    inside = rect == 0.0
    rho = np.hypot(x[:, None] - BORES[None, :, 0], y[:, None] - BORES[None, :, 1]).min(axis=1)
    disc = inside & (rho < BORE_RADIUS)
    wall = BORE_RADIUS - rho
    above = np.hypot(z, np.where(disc, wall, rect))
    hole = disc & (z >= -BORE_DEPTH)
    slab = np.where(inside, np.where(hole, np.minimum(wall, z + BORE_DEPTH), 0.0), rect)
    below = np.hypot(-T - z, rect)
    d = np.where(z > 0.0, above, np.where(z >= -T, slab, below))
    if not parts:
        return d
    slab_z = z >= -T
    part = np.select([(z > 0.0) & inside, z > 0.0, slab_z & hole & (wall <= z + BORE_DEPTH), slab_z & hole,
                      slab_z & inside, slab_z], [0, 1, 2, 3, 0, 1], default=4)
    return d, part


def shim_distance(plate, P):
    """Distance (m) from points P (N, 3) to the plate's shims (vertical cylinders, table to plate)."""
    out = np.full(len(P), np.inf)
    for f in plate["shims"]:
        out = np.minimum(out, _segment_distance(P, np.array([f[0], f[1], 0.0]), f) - SHIM_RADIUS)
    return out


def _seg_point(E, B, C):
    """Distance from segments E-B (N, 3) to points C (3,) or (N, 3)."""
    d = B - E
    t = np.clip(np.einsum("na,na->n", C - E, d) / np.maximum(np.einsum("na,na->n", d, d), 1e-300), 0.0, 1.0)
    return np.linalg.norm(C - (E + t[:, None] * d), axis=1)


def _seg_seg(E, B, A0, A1):
    """Distance from segments E-B (N, 3) to one segment A0-A1 (closest points, clamped)."""
    d1, d2 = B - E, (A1 - A0)[None]
    r = E - A0[None]
    a = np.einsum("na,na->n", d1, d1)
    e = float(d2[0] @ d2[0])
    f = r @ d2[0]
    c = np.einsum("na,na->n", d1, r)
    bb = d1 @ d2[0]
    den = a * e - bb * bb
    s = np.where(den > 1e-300, np.clip((bb * f - c * e) / np.where(den > 1e-300, den, 1.0), 0.0, 1.0), 0.0)
    t = (bb * s + f) / e
    s = np.where(t < 0.0, np.clip(-c / a, 0.0, 1.0), np.where(t > 1.0, np.clip((bb - c) / a, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    return np.linalg.norm(E + s[:, None] * d1 - (A0[None] + t[:, None] * d2), axis=1)


# ---- the camera's image: cell_camera.py (hidden) -------------------------------------------


class CellError(RuntimeError):
    """A request the cell refuses (nothing happens, nothing is charged)."""


class Cell:
    def __init__(self, unit, session=0, budget=None):
        """A session on the unit (`serviced(unit)` for a re-qualification session), with its own noise streams
        and budget (default BUDGET; a re-qualification session: BUDGET_SERVICE)."""
        self.unit = unit
        self.budget = dict(BUDGET if budget is None else budget)
        self.cellp = unit["cell"]
        self.rng = _rng("session", unit["seed"], session)
        self.rng_temp = _rng("temperature readings", unit["seed"], session)     # its own stream
        self.q = Q_HOME.copy()
        self.s = np.ones(6)
        self.temp = np.full(6, T_AMBIENT)
        self.time = 0.0
        self.probes = 0
        self.tool = "probe_a"
        self.damaged = False
        self.offset = np.zeros(3)           # this arrival's repeatability error
        self.session = session
        self.images = 0
        self._grid = None                   # the camera's pixel rays, made at the first image
        self._vig = None
        self.image_truth = []               # the true camera pose of every image (evaluation only)

    # ---- what the agent is told ---------------------------------------------------------
    def observation(self, **extra):
        out = {"q": [float(v) for v in np.round(self.q / ENCODER_STEP) * ENCODER_STEP],
               "temp": [float(v) for v in self.temp + self.rng_temp.normal(0, SIGMA_TEMP, 6)],
               "tool": self.tool, "time_s": round(float(self.time), 3), "probes_used": self.probes,
               "time_left_s": round(float(self.budget["time_s"] - self.time), 3),
               "probes_left": self.budget["probes"] - self.probes, "images_used": self.images,
               "images_left": self.budget["images"] - self.images, "damaged": self.damaged}
        out.update(extra)
        return out

    # ---- physics --------------------------------------------------------------------------
    def _tip(self, q, s, temp=None):
        temp = self.temp if temp is None else temp
        q = np.atleast_2d(q)
        return true_tip(self.unit, q, np.broadcast_to(s, q.shape), np.broadcast_to(temp, q.shape),
                        TOOLS[self.tool]) + self.offset

    def _gap(self, tip):
        """Clearance of the stylus ball to each sphere (N, 12) and to the posts and the table (N,)."""
        c = self.unit["centres"]
        spheres = np.linalg.norm(tip[:, None, :] - c[None], axis=2) - (R_SPHERE + R_STYLUS)
        posts = np.full(len(tip), np.inf)
        for k in range(len(c)):
            foot = np.array([c[k, 0], c[k, 1], 0.0])
            posts = np.minimum(posts, _segment_distance(tip, foot, c[k]) - (R_POST + R_STYLUS))
        return spheres, np.minimum(posts, tip[:, 2] - R_STYLUS)

    def _plate_gap(self, tip):
        """Clearance of the stylus ball to the gauge plate and its shims (N,): any contact damages the probe."""
        plate = self.unit["plate"]
        return np.minimum(plate_distance(plate, tip), shim_distance(plate, tip)) - R_STYLUS

    def _advance(self, dq, duration):
        tau = self.cellp["tau_cool"]
        rate = self.cellp["heat"] @ np.abs(dq) / duration
        level = T_AMBIENT + rate * tau
        self.temp = level + (self.temp - level) * np.exp(-duration / tau)
        self.time += duration

    def _arrive(self):
        self.offset = self.rng.normal(0, self.cellp["sigma_repeat"], 3)

    def _check(self, q_target):
        if self.damaged:
            raise CellError("the probe is damaged: this session is over")
        q_target = np.asarray(q_target, float)
        if q_target.shape != (6,) or not np.all(np.isfinite(q_target)):
            raise CellError("q must be six finite numbers")
        if np.any(q_target < am.Q_MIN) or np.any(q_target > am.Q_MAX):
            raise CellError("q outside the joint limits")
        if self.time >= self.budget["time_s"]:
            raise CellError("the session's time is used up")
        return q_target

    def _path(self, q_target, step):
        dq = q_target - self.q
        s = np.where(np.abs(dq) > S_THRESHOLD, np.sign(dq), self.s)
        reach = np.abs(dq).max()
        n = int(max(2, np.ceil(reach / step) + 1))
        lam = np.linspace(0.0, 1.0, n)
        return dq, s, lam, self.q[None] + lam[:, None] * dq[None]

    def move(self, q_target, speed=MOVE_SPEED_MAX):
        """Unguarded joint move. Touching anything on the way damages the probe."""
        q_target = self._check(q_target)
        speed = float(speed)
        if not 0.0 < speed <= MOVE_SPEED_MAX:
            raise CellError("speed must be in (0, {}] rad/s".format(MOVE_SPEED_MAX))
        dq, s, lam, q = self._path(q_target, 0.002)
        self._arrive()
        tips = self._tip(q, s)
        spheres, rest = self._gap(tips)
        duration = np.abs(dq).max() / speed + SETTLE
        touching = (spheres.min(axis=1) <= 0) | (rest <= 0) | (self._plate_gap(tips) <= 0)
        free = np.nonzero(~touching)[0]     # a move may start in contact (after a probe): it must leave it
        hit = np.nonzero(touching[free[0]:])[0] + free[0] if len(free) else np.arange(1, len(lam))
        if len(hit):
            frac = lam[hit[0]]
            self._advance(dq * frac, duration * frac + 1e-6)
            self.q, self.s, self.damaged = self.q + dq * frac, s, True
            return self.observation(collided=True)
        self._advance(dq, duration)
        self.q, self.s = q_target, s
        return self.observation(collided=False)

    def probe(self, q_target):
        """Guarded joint move at probing speed: stops when the stylus touches something."""
        q_target = self._check(q_target)
        if self.probes >= self.budget["probes"]:
            raise CellError("no probes left")
        dq, s, lam, q = self._path(q_target, 0.0005)
        self.probes += 1
        self._arrive()
        tips = self._tip(q, s)
        spheres, rest = self._gap(tips)
        gap = np.minimum(spheres.min(axis=1), rest)
        hit = np.nonzero(gap <= 0)[0]
        on_plate = np.nonzero(self._plate_gap(tips) <= 0)[0]    # the gauge plate is never probed: a touch damages
        false = self.rng.random() < self.cellp["false_trigger"]
        where = self.rng.random()
        # The bisection runs on every probe, wanted or not, so that the time a reply takes does not
        # tell a touch from a path that touched nothing.
        first = hit[0] if len(hit) and hit[0] > 0 else len(lam) - 1
        lo, hi = lam[first - 1], lam[first]
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            sp, rs = self._gap(self._tip(self.q + mid * dq, s))
            lo, hi = (lo, mid) if min(sp.min(), rs.min()) <= 0 else (mid, hi)
        if len(hit) == 0:
            frac = 1.0 if not false else where
            triggered = false
        elif hit[0] == 0:
            frac, triggered = 0.0, True
        else:
            frac, triggered = hi, True
            if false:
                frac = where * frac
        if len(on_plate) and lam[on_plate[0]] <= frac:
            # the stylus reached the plate before the probe stopped: a collision, as in a move
            frac = lam[on_plate[0]]
            duration = np.abs(dq).max() * frac / PROBE_SPEED + SETTLE
            self._advance(dq * frac, duration)
            self.q, self.s, self.damaged = self.q + dq * frac, s, True
            return self.observation(triggered=False, collided=True)
        duration = np.abs(dq).max() * frac / PROBE_SPEED + SETTLE
        self._advance(dq * frac, duration)
        self.q, self.s = self.q + dq * frac, s
        return self.observation(triggered=bool(triggered))

    def change_tool(self, name):
        if self.damaged:
            raise CellError("the probe is damaged: this session is over")
        if name not in MOUNTABLE:
            raise CellError("no such tool to mount")
        if np.abs(self.q - Q_HOME).max() > 1e-6:
            raise CellError("tools are changed at the home pose")
        self._advance(np.zeros(6), TOOL_CHANGE)
        self.tool = name
        self._arrive()
        return self.observation()

    def _camera_pose(self):
        """The camera's true pose now: axes Rc (3, 3) and centre (3,) in the datum frame."""
        Rc, oc = true_camera(self.unit, self.q[None], self.s[None], self.temp[None], TOOLS[self.tool])
        return Rc[0], oc[0] + self.offset

    def image(self):
        """One grey-level image (uint8, IMAGE_H x IMAGE_W) from the wrist camera, the arm at rest."""
        if self.damaged:
            raise CellError("the probe is damaged: this session is over")
        if self.time >= self.budget["time_s"]:
            raise CellError("the session's time is used up")
        if self.images >= self.budget["images"]:
            raise CellError("no images left")
        Rc, oc = self._camera_pose()
        cam = self.unit["camera"]
        if self._grid is None:
            self._grid = cell_camera.Grid(cam["optics"])
            self._vig = cell_camera.vignetting(cam, self._grid)
        img = cell_camera.render(cam, self._grid, Rc, oc, self.unit["centres"],
                                 _rng("image", self.unit["seed"], self.session, self.images), self._vig,
                                 plate=self.unit["plate"])
        self.images += 1
        self.image_truth.append((Rc, oc))
        self._advance(np.zeros(6), IMAGE_TIME)
        return self.observation(image=img)

    def wait(self, seconds):
        seconds = float(seconds)
        if not 0.0 < seconds <= 600.0:
            raise CellError("seconds must be in (0, 600]")
        self._advance(np.zeros(6), seconds)
        return self.observation()


# ---- acceptance ---------------------------------------------------------------------------
def acceptance_conditions(seed, n=N_ACCEPT):
    """The unit's acceptance conditions (deterministic per unit): q, s, temp (N, 6) and the tool."""
    rng = _rng("accept", seed)
    nom = am.nominal()
    tool = TOOLS[ACCEPT_TOOL]
    qs = []
    while len(qs) < n:
        q = rng.uniform(am.Q_MIN, am.Q_MAX, (4000, 6))
        tip = am.tcp(nom, q, np.zeros_like(q), np.full_like(q, am.T0), tool)
        r = np.hypot(tip[:, 0], tip[:, 1])
        az = np.arctan2(tip[:, 1], tip[:, 0])
        reg = ACCEPT_REGION
        ok = ((r > reg["radius"][0]) & (r < reg["radius"][1]) & (tip[:, 2] > reg["z"][0]) & (tip[:, 2] < reg["z"][1])
              & (np.abs(az) < reg["azimuth"]))
        qs.extend(q[ok])
    q = np.array(qs[:n])
    s = rng.choice([-1.0, 1.0], (n, 6))
    cold = (np.arange(n) % 2 == 0)[:, None]
    temp = np.where(cold, rng.uniform(*ACCEPT_TEMP[0], (n, 6)), rng.uniform(*ACCEPT_TEMP[1], (n, 6)))
    return q, s, temp, tool


def write_conditions(unit):
    """What goes into the unit's public conditions file (JSON-ready): the N_ACCEPT conditions, the
    camera-test points and the warm job's start temperatures. q in rad, s +1 / -1, temp and job_warm_temp in
    degrees C, cam_points in m (camera frame)."""
    q, s, temp, _ = acceptance_conditions(unit["seed"])
    return {"tool": ACCEPT_TOOL, "n": int(len(q)), "q": q.tolist(), "s": s.astype(int).tolist(), "temp": temp.tolist(),
            "cam_points": ACCEPT_CAM_POINTS.tolist(), "job_warm_temp": warm_start(unit["seed"]).tolist()}


def truth_on_conditions(unit):
    """The true tip (N, 3) and the true datum-frame camera points (N, 5, 3) on the unit's conditions."""
    q, s, temp, tool = acceptance_conditions(unit["seed"])
    tip = true_tip(unit, q, s, temp, tool)
    Rc, oc = true_camera(unit, q, s, temp, tool)
    return tip, np.einsum("nab,kb->nka", Rc, ACCEPT_CAM_POINTS) + oc[:, None, :]


def acceptance_errors_from_predictions(pred_tip, pred_cam, unit):
    """Errors (m) of the predictions on the unit's acceptance conditions. pred_tip: (N_ACCEPT, 3),
    pred_cam: (N_ACCEPT, len(ACCEPT_CAM_POINTS), 3), metres, datum frame. Returns tip_err (N,) and
    cam_err (N, 5). Raises ValueError on a wrong shape or a non-finite value."""
    pred_tip = np.asarray(pred_tip, dtype=float)
    pred_cam = np.asarray(pred_cam, dtype=float)
    if pred_tip.shape != (N_ACCEPT, 3) or pred_cam.shape != (N_ACCEPT, len(ACCEPT_CAM_POINTS), 3):
        raise ValueError("predictions must be ({0}, 3) and ({0}, {1}, 3)".format(N_ACCEPT, len(ACCEPT_CAM_POINTS)))
    if not (np.all(np.isfinite(pred_tip)) and np.all(np.isfinite(pred_cam))):
        raise ValueError("predictions must be finite")
    tip, cam = truth_on_conditions(unit)
    return np.linalg.norm(pred_tip - tip, axis=1), np.linalg.norm(pred_cam - cam, axis=2)


def predict_first_order(p, conditions=None, seed=None):
    """Predictions of the first-order model (84 parameters) on conditions (q, s, temp, tool)."""
    q, s, temp, tool = conditions if conditions is not None else acceptance_conditions(seed)
    tau = am.gravity_torques(q, tool["mass"], tool["com"])
    tip = am.tcp(p, q, s, temp, tool, tau)
    Rc, oc = am.camera_pose(p, q, s, temp, tool, tau)
    return tip, np.einsum("nab,kb->nka", Rc, ACCEPT_CAM_POINTS) + oc[:, None, :]


def acceptance_errors(params, unit):
    """Errors of the first-order model with these 84 parameters: its predictions graded as above."""
    p = np.asarray(params, float)
    if p.shape != (am.N_PARAM,):
        raise ValueError("the first-order model has {} parameters".format(am.N_PARAM))
    return acceptance_errors_from_predictions(*predict_first_order(p, seed=unit["seed"]), unit)


def accepted(tip_errors, cam_errors):
    tip_errors, cam_errors = np.ravel(tip_errors), np.ravel(cam_errors)
    return bool(np.percentile(tip_errors, 95) <= ACCEPT_P95 and tip_errors.max() <= ACCEPT_MAX
                and np.percentile(cam_errors, 95) <= ACCEPT_CAM_P95 and cam_errors.max() <= ACCEPT_CAM_MAX)


# ---- the job: the submission's program run once on the true unit, process_c mounted ---------------------
JOB_STEP = 0.002                # rad: the first samples of a row's path (the joint that moves farthest)
POSE_BATCH = 512                # poses examined per batch (bounded memory)
REFINE_NEAR = 1.0e-3            # m: the pin is tested finely along its axis only when it may be this near the plate
FINE_POINTS = 100000            # points of the pin tested per batch near the plate (bounded memory)
H_FINE = JOB_RESOLUTION          # m: spacing of the points tested along the pin's axis near the plate
NEAR_PLATE = 4.0e-3             # m: a pin nearer the plate's block than this is tested at the fine spacing
DISP_MIN = 1.0e-6               # m: an interval of a path whose pin points move less than this is not split
_AXIS_PAD = 1.02                # a pin point's path between two examined poses is at most this times their distance


class JobTooComplex(RuntimeError):
    """The program's contact check needed more than JOB_MAX_POSES poses."""


def _box_lower(plate, E, B):
    """Lower bound (m) of the distance from segments E-B (N, 3) to the plate's block: golden-section search of
    the (convex) distance to the box along each segment."""
    a, b, T = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2, PLATE_SIZE[2]
    e, d = (E - plate["O"]) @ plate["R"], (B - E) @ plate["R"]
    half = np.array([a, b, T / 2])
    mid = np.array([0.0, 0.0, -T / 2])

    def dist(t):
        p = e + t[:, None] * d - mid
        return np.linalg.norm(np.maximum(np.abs(p) - half, 0.0), axis=1)
    lo, hi = np.zeros(len(E)), np.ones(len(E))
    g = 0.5 * (np.sqrt(5.0) - 1.0)
    for _ in range(40):
        m1, m2 = hi - g * (hi - lo), lo + g * (hi - lo)
        left = dist(m1) <= dist(m2)
        hi, lo = np.where(left, m2, hi), np.where(left, lo, m1)
    return np.minimum(np.minimum(dist(lo), dist(np.zeros(len(E)))), dist(np.ones(len(E)))) - 1e-9


def pin_axis(unit, q, s, temp, offset, radius=PIN_RADIUS):
    """The pin's axis segment ends (N, 3): the centre of its hemispherical end and the far end, datum frame;
    and its apex (the tool tip) (N, 3). s and offset: one row or one per pose."""
    tool = TOOLS[JOB_TOOL]
    R, o = true_frames(unit, q, s, temp, tool)
    apex = o + np.einsum("nab,b->na", R, np.asarray(tool["tip"], float) + np.array([0.0, 0.0, am.FLANGE_Z])) + offset
    a = R[:, :, 2]
    return apex - radius * a, apex - (PIN_LENGTH - radius) * a, apex


def pin_clearance(unit, E, B, radius=PIN_RADIUS):
    """Per pose (segments E-B, N): (lower, upper) bounds of the pin's clearance to everything (m; the pin's
    radius taken off), and the index of the nearest object (0 plate, 1 shim, 2 table, 3 + k sphere k,
    15 + k post k). The spheres, posts, shims and the table are exact; the plate is tested at points along the
    axis JOB_RESOLUTION apart when the pin is near its block (lower bound: their least distance less half the
    spacing), its block's distance otherwise."""
    n = len(E)
    c = unit["centres"]
    plate = unit["plate"]
    low = np.empty((n, 27))
    low[:, 2] = np.minimum(E[:, 2], B[:, 2])
    for k in range(len(c)):
        low[:, 3 + k] = _seg_point(E, B, c[k]) - R_SPHERE
        low[:, 15 + k] = _seg_seg(E, B, np.array([c[k, 0], c[k, 1], 0.0]), c[k]) - R_POST
    low[:, 1] = np.min([_seg_seg(E, B, np.array([f[0], f[1], 0.0]), f) for f in plate["shims"]], axis=0) - SHIM_RADIUS
    up = low.copy()
    box = _box_lower(plate, E, B)
    low[:, 0], up[:, 0] = box, np.inf
    near = np.nonzero(box <= NEAR_PLATE + radius)[0]
    if len(near):
        dp_up, dp_low = _plate_segment(plate, E[near], B[near], near=radius + REFINE_NEAR)
        up[near, 0], low[near, 0] = dp_up, np.maximum(dp_low, box[near])
    return low.min(axis=1) - radius, up.min(axis=1) - radius, np.argmin(low, axis=1)


def _plate_segment(plate, E, B, coarse=50, fine=50, near=None):
    """Least distance from segments E-B (N, 3) to the plate's solid: an upper bound (the least at the points
    tested) and a lower bound within H_FINE / 2 of it. The distance is 1-Lipschitz along a segment: it is
    tested at coarse + 1 points, then finely (spacing at most H_FINE) in every coarse interval that may hold a
    point nearer than the nearest coarse point, for the segments whose coarse lower bound is below `near` (the
    others keep the coarse bound, already far from deciding a contact)."""
    n = len(E)
    L = np.linalg.norm(B - E, axis=1)
    uc = np.linspace(0.0, 1.0, coarse + 1)
    P = E[:, None, :] + uc[None, :, None] * (B - E)[:, None, :]
    dc = plate_distance(plate, P.reshape(-1, 3)).reshape(n, coarse + 1)
    best = dc.min(axis=1)
    hc = L / coarse
    # intervals [k, k + 1] whose lower bound (the smaller end less half the step) is below the best
    lowk = np.minimum(dc[:, :-1], dc[:, 1:]) - 0.5 * hc[:, None]
    close = (best - 0.5 * hc < near) if near is not None else np.ones(n, bool)
    cand = np.nonzero((lowk < best[:, None]) & close[:, None])
    m = max(fine, int(np.ceil(hc.max() / H_FINE)))
    pi_all, ki_all = cand
    df = np.empty(len(pi_all))
    step = max(1, FINE_POINTS // (m + 1))         # candidate intervals per batch: bounded memory
    for a in range(0, len(pi_all), step):
        pi, ki = pi_all[a:a + step], ki_all[a:a + step]
        uf = (ki[:, None] + np.linspace(0.0, 1.0, m + 1)[None, :]) / coarse
        Pf = E[pi, None, :] + uf[:, :, None] * (B[pi] - E[pi])[:, None, :]
        df[a:a + step] = plate_distance(plate, Pf.reshape(-1, 3)).reshape(len(pi), m + 1).min(axis=1)
    pi = pi_all
    up = best.copy()
    np.minimum.at(up, pi, df)
    lo_f = np.full(n, np.inf)
    np.minimum.at(lo_f, pi, df - 0.5 * hc[pi] / m)
    # intervals not refined: their lower bound is at least the best coarse value; segments not refined at all:
    # the coarse bound (the best coarse value less half the coarse step)
    lo = np.minimum(lo_f, best)
    lo = np.where(close, lo, best - 0.5 * hc)
    return up, lo


def row_poses(unit, q0, q1, s0, s1, temp, off0, off1, lam, radius=PIN_RADIUS):
    """The pin along a job row at path fractions lam: the joints on the straight line q0 -> q1, each direction
    sign going linearly from its old value s0 to its new one s1 (the backlash taken up over the row), the
    repeatability error going linearly from the last arrival's off0 to this arrival's off1."""
    lam = np.asarray(lam, float)[:, None]
    return pin_axis(unit, q0[None] + lam * (q1 - q0)[None], s0[None] + lam * (s1 - s0)[None], temp,
                    off0[None] + lam * (off1 - off0)[None], radius)


def _row_contact(unit, q0, q1, s0, s1, temp, off0, off1, budget, radius=PIN_RADIUS):
    """The first contact (path fraction, object index) along the job row q0 -> q1, or None; and the number of
    poses examined. A contact is found wherever it is; a pin within JOB_RESOLUTION of a surface may count as
    one."""
    dq = q1 - q0
    n = int(max(2, np.ceil(np.abs(dq).max() / JOB_STEP) + 1))
    if n > budget:                       # every pose counts, before it is computed
        raise JobTooComplex()
    lam = list(np.linspace(0.0, 1.0, n))
    poses = {}
    for a in range(0, n, POSE_BATCH):
        part = lam[a:a + POSE_BATCH]
        E, B, _ = row_poses(unit, q0, q1, s0, s1, temp, off0, off1, part, radius)
        lo, up, obj = pin_clearance(unit, E, B, radius)
        poses.update({l: (E[i], B[i], lo[i], up[i], obj[i]) for i, l in enumerate(part)})
    used = n
    while True:
        keys = sorted(poses)
        first = next((k for k in keys if poses[k][2] <= 0.0), None)
        todo = []
        for a, b in zip(keys[:-1], keys[1:]):
            if first is not None and a >= first:
                break
            Ea, Ba, ga = poses[a][:3]
            Eb, Bb, gb = poses[b][:3]
            disp = max(np.linalg.norm(Ea - Eb), np.linalg.norm(Ba - Bb)) * _AXIS_PAD + 1e-12
            if ga + gb > disp:
                continue
            if disp < DISP_MIN:
                first = a if first is None else min(first, a)
                break
            todo.append(0.5 * (a + b))
        if not todo:
            if first is None:
                return None, used
            return (float(first), int(poses[first][4])), used
        used += len(todo)
        if used > budget:
            raise JobTooComplex()
        for a in range(0, len(todo), POSE_BATCH):
            part = todo[a:a + POSE_BATCH]
            E, B, _ = row_poses(unit, q0, q1, s0, s1, temp, off0, off1, part, radius)
            lo, up, obj = pin_clearance(unit, E, B, radius)
            for i, l in enumerate(part):
                poses[l] = (E[i], B[i], lo[i], up[i], obj[i])


def object_name(index, unit=None, where=None):
    if index == 0:
        return "the plate"
    if index == 1:
        return "a shim"
    if index == 2:
        return "the table"
    return ("datum sphere {}" if index < 15 else "the post of datum sphere {}").format((index - 3) % 12)


def run_job(unit, rows, stream=0, warm=False, trace=False, radius=PIN_RADIUS, temp0=None):
    """Run a job program (rows: (m, 6) joint targets, finite and inside the joint limits) once on the true
    unit: cold (every link at T_AMBIENT at the start) or, with `warm`, from the unit's published warm start
    temperatures (warm_start), each with its own stream of repeatability errors; on `serviced(unit)` (the serviced
    job, cold) with a stream of its own again. Returns a dict: done (bool per
    bore), bores_done, jammed, jam_row, jam_object, rows_run, poses (the check's poses examined); with `trace`,
    the arrival of every row run (q, s, temp, offset). `radius`, `temp0` (studies only): another pin radius,
    other start temperatures."""
    rows = np.asarray(rows, float)
    if rows.ndim != 2 or rows.shape[1] != 6 or len(rows) > JOB_MAX_ROWS:
        raise ValueError("a job program is at most {} rows of 6 joint targets".format(JOB_MAX_ROWS))
    if not np.all(np.isfinite(rows)) or np.any(rows < am.Q_MIN) or np.any(rows > am.Q_MAX):
        raise ValueError("job targets must be finite and inside the joint limits")
    rng = _rng("job serviced" if "service" in unit else "job warm" if warm else "job", unit["seed"], stream)
    cellp, plate = unit["cell"], unit["plate"]
    if temp0 is None:
        temp0 = warm_start(unit["seed"]) if warm else np.full(6, T_AMBIENT)
    q, s, temp, offset = Q_HOME.copy(), np.ones(6), np.array(temp0, float), np.zeros(3)
    done = np.zeros(N_BORES, bool)
    out = dict(jammed=False, jam_row=-1, jam_object="", rows_run=0, poses=0, arrivals=[])
    budget = JOB_MAX_POSES
    for i, target in enumerate(rows):
        dq = target - q
        s_new = np.where(np.abs(dq) > S_THRESHOLD, np.sign(dq), s)
        off_new = rng.normal(0.0, cellp["sigma_repeat"], 3)
        hit, used = _row_contact(unit, q, target, s, s_new, temp, offset, off_new, budget, radius)
        budget -= used
        out["poses"] = JOB_MAX_POSES - budget
        if hit is not None:
            out.update(jammed=True, jam_row=i, jam_object=object_name(hit[1]), rows_run=i)
            break
        duration = np.abs(dq).max() / JOB_SPEED + SETTLE
        offset = off_new
        E_end, B_end, apex = pin_axis(unit, target[None], s_new, temp, offset)
        if trace:
            out["arrivals"].append(dict(q=target.copy(), s=s_new.copy(), temp=temp.copy(), offset=offset))
        done |= bores_entered(plate, apex[0], E_end[0], B_end[0])
        tau = cellp["tau_cool"]
        level = T_AMBIENT + cellp["heat"] @ np.abs(dq) / duration * tau
        temp = level + (temp - level) * np.exp(-duration / tau)
        q, s = target.copy(), s_new
        out["rows_run"] = i + 1
    out.update(done=done, bores_done=int(done.sum()))
    return out


def bores_entered(plate, apex, E, B):
    """Which bores the pin is in (N_BORES bools): its apex inside the bore's hole, between JOB_DEPTH below the
    mouth and the bottom, less than BORE_RADIUS from the axis, and its axis leaving through the mouth (it
    crosses the top face's plane less than BORE_RADIUS from the bore's axis, the far end above the plane). A pin
    that touches nothing can be there only by having come in through the mouth."""
    p, e, b = ((x - plate["O"]) @ plate["R"] for x in (apex, E, B))
    rho = np.hypot(p[0] - BORES[:, 0], p[1] - BORES[:, 1])
    deep = (p[2] <= -JOB_DEPTH) & (p[2] >= -BORE_DEPTH) & (rho < BORE_RADIUS)
    if b[2] <= 0.0 or e[2] >= 0.0:
        return np.zeros(N_BORES, bool)
    t = e[2] / (e[2] - b[2])
    cx, cy = e[0] + t * (b[0] - e[0]), e[1] + t * (b[1] - e[1])
    return deep & (np.hypot(cx - BORES[:, 0], cy - BORES[:, 1]) < BORE_RADIUS)


def job_passed(result):
    return bool(result["bores_done"] == N_BORES)
