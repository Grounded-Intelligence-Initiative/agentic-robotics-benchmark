"""reference.py: reference solution for the accuracy calibration of one arm unit.

    calibrate(cell, conditions) -> (pred_tip (N, 3), pred_cam (N, 5, 3))
    calibrate(cell, conditions, return_info=True) -> (..., info): info["job"], info["job_warm"] the job programs

`conditions` is the unit's public conditions file as a dict (/home/user/conditions/<unit>.json): the
N acceptance conditions q, s, temp, the tool (process_c), the camera-frame points and the warm job's
start temperatures (job_warm_temp). The result
is the predicted tool tip for every condition and the predicted datum-frame position of every
camera point for every condition (metres).

What the solution may know (and uses):
  * arm_model.py: the manufacturer's first-order model, its nominal values, joint limits;
  * cell_data.py: the survey, radii, tool certificates, Q_HOME, the budget, speeds, timings, the
    tolerances of the first-order parameters, the camera's figures (resolution, nominal focal
    length and its tolerance, the bounds of the principal point and of the distortion), and the
    RANGES the documents give for each cell (repeatability, false triggers, knocked spheres,
    heating); nothing is assumed about where in a range a unit lies;
  * camera.md: what the images look like, in words (no image formation law is known: the
    vision measures OUTLINES, never shading);
  * defects.md: the six defect classes, their exact forms and bounds, transcribed below as a
    candidate library (`LIB`); a unit has some of them, which ones is decided from the data;
  * the observations returned by the cell's five methods move / probe / change_tool / wait / image;
  * the unit's conditions (where the predictions are wanted; also used to weight the design).
It never touches the cell's state: `Robot` wraps the cell and keeps only its five bound methods.

The parameter vector `p` has NX = 140 entries: the 84 of arm_model (78 arm, 6 camera mounting),
the 50 coefficients of the defect library (each zero when its defect is absent), the camera's
intrinsics (focal length, principal point, radial distortion k1, k2) and one outline offset.

Phases in one session: the touch-probe campaign (designed greedily, I-optimal for the unit's
acceptance conditions, with the whole defect library in the model so that the plan and the safety
margins account for every candidate defect); the robust probe-only fit with a parsimonious model
(every class but the higher harmonic): which datum spheres are knocked and which harmonic (joint,
order) the arm has are decided together by a greedy structure search on a truncated loss
(redescending weights, score tests to rank the moves; started from no harmonic and from each of
the leading candidate harmonics, the cheapest end kept), outlying touches set aside by their
leverage-corrected residuals, the unit's repeatability estimated robustly from the residuals;
the wrist-camera views (spheres all over the image, so that the intrinsics are calibrated too)
and the image measurements: each sphere found by a Hough search near its predicted place, its
outline located to a fraction of a pixel along ~100 profiles (a blurred step-ramp-square-root
model of the limb, profiles that meet other objects, saturation or too little contrast left
out), the cone of rays tangent to it fitted (robustly) with the current intrinsics: its centre in
the camera frame and how that centre moves with the intrinsics; the camera's mounting and
intrinsics fitted with the arm held, measurements that disagree set aside, all measured again
from the refined poses; the structure (knocked datum spheres, higher harmonic) and the repeatability
decided again on every touch and every image measurement (an image measures a sphere's centre
directly), searched from the probe fit's structure and from none; the joint fit of
touches and images, intrinsics included (touches the
images show to be wrong set aside, the measurements recomputed whenever the intrinsics move);
last a Bayesian model selection over the defect classes (linearised Laplace evidence of every
subset of classes with none or one of the 18 candidate harmonics), whose best models are refitted
and averaged with their posterior weights for the predictions.

The job (plate_job.py): the probing keeps the stylus clear of the region the gauge plate may occupy
(it is never touched); before the spheres' views, N_PLATE_VIEWS views of the plate from several
directions; after the final fit, the plate located in 6 degrees of freedom from all its views with
the most probable model's camera poses and intrinsics, and the two job programs planned with that model
(`job_program`): the cold one from every link at T_AMBIENT, the warm one from the published warm start
temperatures, the link temperatures along each predicted with the cell's heating (own joint and neighbours)
and cooling time constant fitted to every temperature reading of the session (`thermal_fit`: the session
starts at T_AMBIENT, and the campaign's warm-up and the cooling after it show both).
calibrate(..., return_info=True) returns them as info["job"] and info["job_warm"].
"""
import sys

import numpy as np

import arm_model as am
import plate_job as pj
# documented figures only (cell_data.py is what the agent has; the cell's simulator is not imported)
from cell_data import (BACKLASH_MAX, CYCLIC_MAX, COMPLIANCE_MAX, KAPPA_RANGE, ACCEPT_TEMP, surveyed_spheres, R_SPHERE,
                       R_STYLUS, R_POST, TOOLS, Q_HOME, MOVE_SPEED_MAX, PROBE_SPEED, SETTLE, S_THRESHOLD,
                       SIGMA_REPEAT_RANGE, KNOCK_MIN, KNOCK_MAX, KNOCKED_SPHERES, T_AMBIENT, TAU_COOL_RANGE)
from cell_data import (IMAGE_W, IMAGE_H, F_PX_NOMINAL, CX_NOMINAL, CY_NOMINAL, F_TOL, PP_TOL, DISTORTION_MAX_PX,
                       IMAGE_TIME, BUDGET, TOL_CAMERA_XYZ, TOL_CAMERA_RPY)
from scipy import ndimage, special

D = am.D
NP = am.N_KINEMATIC                         # 78: the arm's first-order parameters
NC = am.N_PARAM                             # 84: ... and the camera's 6
SL = am.SLICES
CAM = SL["camera"]
R_TOUCH = R_SPHERE + R_STYLUS
SPHERES = surveyed_spheres()
SIGMA_PLAN = SIGMA_REPEAT_RANGE[1]          # m: the repeatability assumed until the data say otherwise
DISPLACED_MAX = KNOCK_MAX                   # m, documented: a datum sphere can be knocked this far
TOOL_A, TOOL_B = "probe_a", "probe_b"

# ---- the defect library (defects.md, transcribed) --------------------------------------------
LIB_CLASSES = ("coupling", "ratio", "deflection", "cyclic", "thermal", "pedestal")
COUPLING_BOUND = 8.0e-4                     # |k45|, |k56|
RATIO_JOINTS, RATIO_BOUND = (0, 3, 5), 1.2e-3
DEFLECT_BOUND = 3.0e-4                      # kt_u, kb_u, kt_f, kb_f in [0, bound]
CYC_JOINTS, CYC_ORDERS, CYC_BOUND = (0, 1, 2), (3, 4, 5, 6, 7, 8), 1.0e-3
THERMAL_JOINTS = (1, 2, 4)
THERMAL_BOUND = np.array([5.0e-5, 6.0e-5, 1.0e-4])
PEDESTAL_BOUND = 5.0e-5                     # kx, ky in [0, bound]

LIB = {"coupling": [84, 85], "ratio": [86, 87, 88], "deflection": [89, 90, 91, 92], "thermal": [93, 94, 95],
       "pedestal": [96, 97], "cyclic": list(range(98, 134))}
# the camera's own unknowns (camera.md): focal length, principal point, radial distortion k1, k2
# (x_d = x (1 + k1 r^2 + k2 r^4) in normalised coordinates about the principal point), and the edge
# offset (px) common to every outline measurement (what is left of the outline detector's bias)
INTR = slice(134, 139)
EDGE = 139
VIS = slice(134, 140)
NX = 140
CYC_PAIRS = [(j, n) for j in CYC_JOINTS for n in CYC_ORDERS]          # 18 candidates


def cyc_cols(j, n):
    """The two coefficients (of sin n q_j and cos n q_j) of one candidate higher harmonic."""
    i = 98 + 2 * (CYC_PAIRS.index((j, n)))
    return [i, i + 1]


# ---- priors ---------------------------------------------------------------------------------------
def prior():
    """Mean and standard deviation of the 78 first-order arm parameters from the documented
    tolerances (uniform distributions)."""
    mean = am.nominal()[:NP]
    sd = np.zeros(NP)
    u = 1.0 / np.sqrt(3.0)                   # sd of U(-1, 1)
    xyz = np.full((6, 3), 1e-3 * u)
    xyz[0] = 5e-3 * u
    rpy = np.tile(np.array([0.1, 0.1, 0.3]) * D * u, (6, 1))
    rpy[0] = 0.5 * D * u
    sd[SL["xyz"]], sd[SL["rpy"]] = xyz.ravel(), rpy.ravel()
    mean[SL["b"]] = 0.6 * BACKLASH_MAX                              # U(0.2, 1.0) * max
    sd[SL["b"]] = 0.4 * u * BACKLASH_MAX
    e_a2 = (1.0 - 0.3 ** 3) / (3 * 0.7)                              # E[a^2], a ~ U(0.3, 1)
    sd[SL["h"]] = np.repeat(CYCLIC_MAX * np.sqrt(e_a2 / 2), 4)         # a cos(phase), a sin(phase)
    mean[SL["c"]] = 0.65 * COMPLIANCE_MAX                            # U(0.3, 1.0) * max
    sd[SL["c"]] = 0.35 * u * COMPLIANCE_MAX
    mean[SL["kappa"]] = 0.5 * (KAPPA_RANGE[0] + KAPPA_RANGE[1])
    sd[SL["kappa"]] = 0.5 * (KAPPA_RANGE[1] - KAPPA_RANGE[0]) * u
    return mean, sd


def camera_prior():
    """Mean and sd of the camera's 6 parameters (documented: uniform within the tolerances)."""
    u = 1.0 / np.sqrt(3.0)
    mu = am.nominal()[CAM]
    sd = np.concatenate([np.full(3, TOL_CAMERA_XYZ * u), np.full(3, TOL_CAMERA_RPY * u)])
    return mu, sd


def class_prior(name):
    """Prior (mean, sd) of a class's coefficients given that the class is present: the documents give
    only the bound, so uniform within it (either sign, or [0, bound] for a compliance)."""
    u = 1.0 / np.sqrt(3.0)
    if name == "coupling":
        return np.zeros(2), np.full(2, COUPLING_BOUND * u)
    if name == "ratio":
        return np.zeros(3), np.full(3, RATIO_BOUND * u)
    if name == "deflection":
        return np.full(4, 0.5 * DEFLECT_BOUND), np.full(4, 0.5 * DEFLECT_BOUND * u)
    if name == "thermal":
        return np.zeros(3), THERMAL_BOUND * u
    if name == "pedestal":
        return np.full(2, 0.5 * PEDESTAL_BOUND), np.full(2, 0.5 * PEDESTAL_BOUND * u)
    if name == "cyclic":                     # one pair: amplitude uniform in [0, bound], any phase
        return np.zeros(2), np.full(2, CYC_BOUND / np.sqrt(6.0))
    raise KeyError(name)


CYC_SPREAD = 1.0 / np.sqrt(6.0)          # the planning prior spreads the one harmonic over the orders


def full_prior():
    """Mean and sd of all NX parameters with every defect class present ('fit all candidates'):
    the planning prior of the campaign. The single higher harmonic is not known: every one of the
    18 candidate pairs gets the pair prior shrunk by CYC_SPREAD."""
    mu, sd = np.zeros(NX), np.zeros(NX)
    mu[:NP], sd[:NP] = prior()
    mu[CAM], sd[CAM] = camera_prior()
    mu[VIS], sd[VIS] = vision_prior()
    for name in LIB_CLASSES:
        m, s = class_prior(name)
        if name == "cyclic":
            mu[LIB[name]] = 0.0
            sd[LIB[name]] = s[0] * CYC_SPREAD
        else:
            mu[LIB[name]], sd[LIB[name]] = m, s
    return mu, sd


def model_prior(classes, cyc=None):
    """Mean and sd for one candidate model: the first-order parameters, the classes in `classes`
    (for "cyclic": the pair cyc = (j, n) only), every other defect coefficient fixed at zero
    (sd = 0). Returns mu, sd, active (bool)."""
    mu, sd = np.zeros(NX), np.zeros(NX)
    mu[:NP], sd[:NP] = prior()
    mu[CAM], sd[CAM] = camera_prior()
    mu[VIS], sd[VIS] = vision_prior()
    for name in classes:
        m, s = class_prior(name)
        cols = cyc_cols(*cyc) if name == "cyclic" else LIB[name]
        mu[cols], sd[cols] = m, s
    return mu, sd, sd > 0


def vision_prior():
    """Mean and sd of the camera's own unknowns (f, cx, cy, k1, k2, edge offset) from camera.md:
    uniform within the stated bounds; k1 and k2 wide enough for any smooth radial displacement that
    keeps its sign, grows with the radius and stays within the documented bound at the corners
    (the two terms can partly cancel: k2 r^2 / k1 down to -0.6 at the corner)."""
    u = 1.0 / np.sqrt(3.0)
    rc = np.hypot(CX_NOMINAL + PP_TOL, CY_NOMINAL + PP_TOL) / F_PX_NOMINAL
    k1_max = DISTORTION_MAX_PX / (F_PX_NOMINAL * rc ** 3 * 0.4)
    mu = np.array([F_PX_NOMINAL, CX_NOMINAL, CY_NOMINAL, 0.0, 0.0, 0.0])
    sd = np.array([F_PX_NOMINAL * F_TOL * u, PP_TOL * u, PP_TOL * u, 0.5 * k1_max, 0.5 * 0.6 * k1_max / rc ** 2,
                   EDGE_OFFSET_SD])
    return mu, sd


EDGE_OFFSET_SD = 0.05           # px: prior sd of the common outline offset


def nominal_x():
    x = np.zeros(NX)
    x[:NC] = am.nominal()
    x[VIS] = vision_prior()[0]
    return x


# ---- the extended kinematics: arm_model + the defect library ---------------------------------------
def tip_vector(tool):
    return np.asarray(TOOLS[tool]["tip"], float) + np.array([0.0, 0.0, am.FLANGE_Z])


def gravity_load(q, tool):
    """What the nominal chain says the weights do at the encoder readings (independent of the
    parameters): the gravity torques tau (N, 6) and the moments of defects.md (pedestal (N, 2),
    upper arm (N, 2), forearm (N, 2))."""
    q = np.atleast_2d(np.asarray(q, float))
    T = TOOLS[tool]
    R, o = am._chain(am.NOMINAL_XYZ, am.NOMINAL_RPY, q)
    com = o + np.einsum("njab,jb->nja", R, am.LINK_COM)
    toolc = o[:, 5] + R[:, 5, :, 2] * am.FLANGE_Z + np.einsum("nab,b->na", R[:, 5], np.asarray(T["com"], float))
    pos = np.concatenate([com, toolc[:, None, :]], axis=1)
    w = -np.concatenate([am.LINK_MASS, [T["mass"]]]) * am.G

    def moment(first, origin):
        r = pos[:, first:, :] - origin[:, None, :]
        return np.stack([(r[..., 1] * w[first:]).sum(1), -(r[..., 0] * w[first:]).sum(1), np.zeros(len(q))], axis=1)
    m0, m1, m2 = moment(0, np.zeros((len(q), 3))), moment(1, o[:, 1]), moment(2, o[:, 2])
    up = np.stack([np.einsum("na,na->n", m1, R[:, 1, :, 0]), np.einsum("na,na->n", m1, R[:, 1, :, 1])], axis=1)
    fo = np.stack([np.einsum("na,na->n", m2, R[:, 2, :, 0]), np.einsum("na,na->n", m2, R[:, 2, :, 1])], axis=1)
    tau = am.gravity_torques(q, T["mass"], T["com"])
    return dict(tau=tau, ped=m0[:, :2], up=up, fo=fo)


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    out = np.zeros(a.shape + (3, 3))
    out[..., 0, 0], out[..., 1, 1], out[..., 1, 2], out[..., 2, 1], out[..., 2, 2] = 1.0, c, -s, s, c
    return out


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    out = np.zeros(a.shape + (3, 3))
    out[..., 0, 0], out[..., 0, 2], out[..., 1, 1], out[..., 2, 0], out[..., 2, 2] = c, s, 1.0, -s, c
    return out


def _fixed(rpy):
    """Rx(r) Ry(p) Rz(y) for a batch of angle triples (..., 3) -> (..., 3, 3)."""
    cr, sr = np.cos(rpy[..., 0]), np.sin(rpy[..., 0])
    cp, sp = np.cos(rpy[..., 1]), np.sin(rpy[..., 1])
    cy, sy = np.cos(rpy[..., 2]), np.sin(rpy[..., 2])
    out = np.empty(rpy.shape[:-1] + (3, 3))
    out[..., 0, 0], out[..., 0, 1], out[..., 0, 2] = cp * cy, -cp * sy, sp
    out[..., 1, 0] = sr * sp * cy + cr * sy
    out[..., 1, 1] = -sr * sp * sy + cr * cy
    out[..., 1, 2] = -sr * cp
    out[..., 2, 0] = -cr * sp * cy + sr * sy
    out[..., 2, 1] = cr * sp * sy + sr * cy
    out[..., 2, 2] = cr * cp
    return out


def fk(X, q, s, temp, load):
    """Frame 5 under parameters X (B, NX) at N conditions: R (B, N, 3, 3), o (B, N, 3)."""
    X = np.atleast_2d(X)
    B, N = len(X), len(q)
    xyz, rpy = X[:, 0:18].reshape(B, 6, 3), X[:, 18:36].reshape(B, 6, 3)
    b, h, c, kap = X[:, 36:42], X[:, 42:66].reshape(B, 6, 4), X[:, 66:72], X[:, 72:78]
    sq, cq, s2, c2 = np.sin(q), np.cos(q), np.sin(2 * q), np.cos(2 * q)
    th = (q[None] + 0.5 * b[:, None, :] * s[None] + h[:, None, :, 0] * sq[None] + h[:, None, :, 1] * cq[None]
          + h[:, None, :, 2] * s2[None] + h[:, None, :, 3] * c2[None] + c[:, None, :] * load["tau"][None])
    th[:, :, 4] += X[:, 84, None] * q[None, :, 3]
    th[:, :, 5] += X[:, 85, None] * q[None, :, 4]
    for i, j in enumerate(RATIO_JOINTS):
        th[:, :, j] += X[:, 86 + i, None] * q[None, :, j]
    for i, j in enumerate(THERMAL_JOINTS):
        th[:, :, j] += X[:, 93 + i, None] * (temp[None, :, j] - am.T0)
    cyc = X[:, 98:134]
    if np.any(cyc):
        for i, (j, n) in enumerate(CYC_PAIRS):
            a, bb = cyc[:, 2 * i], cyc[:, 2 * i + 1]
            if np.any(a) or np.any(bb):
                th[:, :, j] += a[:, None] * np.sin(n * q[None, :, j]) + bb[:, None] * np.cos(n * q[None, :, j])
    scale = 1.0 + kap[:, None, :] * (temp[None] - am.T0)
    ped, dfl = X[:, 96:98], X[:, 89:93]
    if np.any(ped):
        R = _rot_x(ped[:, 0, None] * load["ped"][None, :, 0]) @ _rot_y(ped[:, 1, None] * load["ped"][None, :, 1])
    else:
        R = np.broadcast_to(np.eye(3), (B, N, 3, 3))
    o = np.zeros((B, N, 3))
    fixed = _fixed(rpy)                                                  # (B, 6, 3, 3)
    R2 = None
    for j in range(6):
        t = xyz[:, None, j, :] * scale[:, :, j, None]
        o = o + np.einsum("bnij,bnj->bni", R, t)
        if j == 2 and np.any(dfl[:, :2]):
            R = R @ (_rot_x(dfl[:, 0, None] * load["up"][None, :, 0]) @ _rot_y(dfl[:, 1, None] * load["up"][None, :, 1]))
        if j == 4 and np.any(dfl[:, 2:]):
            Df = _rot_x(dfl[:, 2, None] * load["fo"][None, :, 0]) @ _rot_y(dfl[:, 3, None] * load["fo"][None, :, 1])
            R = R2 @ Df @ np.swapaxes(R2, -1, -2) @ R
        R = R @ fixed[:, None, j]
        ct, st = np.cos(th[:, :, j])[..., None], np.sin(th[:, :, j])[..., None]
        R0, R1 = R[..., :, 0], R[..., :, 1]
        R = np.stack([ct * R0 + st * R1, -st * R0 + ct * R1, R[..., :, 2]], axis=-1)
        if j == 2:
            R2 = R
    return R, o


def _prep(q, s, temp, tool, load):
    q = np.atleast_2d(np.asarray(q, float))
    s = np.broadcast_to(np.asarray(s, float), q.shape)
    temp = np.broadcast_to(np.asarray(temp, float), q.shape)
    if load is None:
        load = gravity_load(q, tool)
    return q, s, temp, load


def pose(p, q, s, temp, tool, load=None):
    """Tool tip (N, 3) and flange rotation (N, 3, 3) under parameters p (NX)."""
    q, s, temp, load = _prep(q, s, temp, tool, load)
    R, o = fk(p, q, s, temp, load)
    return o[0] + np.einsum("nab,b->na", R[0], tip_vector(tool)), R[0]


def tip(p, q, s, temp, tool, load=None):
    return pose(p, q, s, temp, tool, load)[0]


def _cam_vec(x):
    return x[:3] + np.array([0.0, 0.0, am.FLANGE_Z])


def camera_pose(p, q, s, temp, tool, load=None):
    """Camera axes Rc (N, 3, 3) (columns, datum frame) and centre (N, 3) under parameters p."""
    q, s, temp, load = _prep(q, s, temp, tool, load)
    R, o = fk(p, q, s, temp, load)
    Rc = np.einsum("nab,bc->nac", R[0], am.camera_rotation(p[81:84]))
    return Rc, o[0] + np.einsum("nab,b->na", R[0], _cam_vec(p[78:84]))


_FD_STEP = np.full(NX, 1e-7)
_FD_STEP[SL["c"]] = 2e-9                    # tau is tens of N m
_FD_STEP[SL["kappa"]] = 1e-6
_FD_STEP[LIB["thermal"]] = 1e-8             # per K, tens of K
_FD_STEP[LIB["deflection"]] = 2e-8          # per N m, a few N m
_FD_STEP[LIB["pedestal"]] = 2e-9            # per N m, tens of N m
_CHUNK = 40000                              # batch rows (parameters x conditions) per kinematics call


def jacobian(p, q, s, temp, tool, cols, load=None, what="tip", points=None):
    """Forward-difference sensitivities to the parameters `cols`, batched. what = "tip": (N, 3, m);
    "camera": of the camera-frame `points` (k, 3) placed in the datum frame, (N, k, 3, m).
    Returns (value, J)."""
    q, s, temp, load = _prep(q, s, temp, tool, load)
    cols = np.asarray(cols, int)
    N = len(q)

    def evaluate(X):
        R, o = fk(X, q, s, temp, load)
        if what == "tip":
            return o + np.einsum("bnac,c->bna", R, tip_vector(tool))
        Rc = np.einsum("bnac,bcd->bnad", R, np.stack([am.camera_rotation(x[81:84]) for x in X]))
        oc = o + np.einsum("bnac,bc->bna", R, np.stack([_cam_vec(x[78:84]) for x in X]))
        return np.einsum("bnac,kc->bnka", Rc, points) + oc[:, :, None, :]
    v0 = evaluate(p[None])[0]
    out = np.empty(v0.shape + (len(cols),))
    per = max(1, _CHUNK // max(N, 1))
    for a in range(0, len(cols), per):
        cc = cols[a:a + per]
        X = np.repeat(p[None], len(cc), axis=0)
        X[np.arange(len(cc)), cc] += _FD_STEP[cc]
        v = evaluate(X)
        out[..., a:a + len(cc)] = np.moveaxis((v - v0[None]) / _FD_STEP[cc].reshape((-1,) + (1,) * v0.ndim), 0, -1)
    return v0, out


def tip_jacobian(p, q, s, temp, tool, load=None, cols=None):
    cols = np.arange(NX) if cols is None else cols
    return jacobian(p, q, s, temp, tool, cols, load)[1]


def joint_jacobian(p, q, s, temp, tool, rot=True, h=1e-6):
    """d(tip, flange rotation vector) / dq, (N, 6, 6) (or (N, 3, 6) without rotation). The gravity
    load varies slowly with q: held at q."""
    q, s, temp, load = _prep(q, s, temp, tool, None)
    N = len(q)
    qq = np.concatenate([q] + [q + h * np.eye(6)[j] for j in range(6)])
    rep = lambda a: np.concatenate([a] * 7)
    R, o = fk(p, qq, rep(s), rep(temp), {k: rep(v) for k, v in load.items()})
    t = o[0] + np.einsum("nab,b->na", R[0], tip_vector(tool))
    t0, R0 = t[:N], R[0, :N]
    J = np.empty((N, 6 if rot else 3, 6))
    for j in range(6):
        sl = slice((j + 1) * N, (j + 2) * N)
        J[:, :3, j] = (t[sl] - t0) / h
        if rot:
            J[:, 3:, j] = _rot_err(R0, R[0, sl]) / h
    return J, t0, R0

def _rot_err(R, Rd):
    """Rotation vector (small-angle) taking R to Rd, (N, 3)."""
    return 0.5 * (np.cross(R[:, :, 0], Rd[:, :, 0]) + np.cross(R[:, :, 1], Rd[:, :, 1])
                  + np.cross(R[:, :, 2], Rd[:, :, 2]))


LIM_MARGIN = 3.0 * D
Q_LO, Q_HI = am.Q_MIN + LIM_MARGIN, am.Q_MAX - LIM_MARGIN


def ik_pose(p, target, Rd, seed, s, temp, tool, iters=60):
    """Batched damped least squares IK on the full flange pose (rows that converge or clearly
    fail are dropped from the iteration). Returns q (N, 6), ok (N,)."""
    q = np.array(seed, float)
    target, Rd = np.atleast_2d(target), np.asarray(Rd).reshape(-1, 3, 3)
    act = np.arange(len(q))
    lam = 1e-2
    for it in range(iters):
        J, t, R = joint_jacobian(p, q[act], s, temp, tool)
        e = np.concatenate([target[act] - t, _rot_err(R, Rd[act]) * 0.3], axis=1)
        err = np.abs(e).max(axis=1)
        keep = (err > 1e-11) & ~((it >= 25) & (err > 1e-3))
        if not keep.any():
            break
        act, J, e = act[keep], J[keep], e[keep]
        J[:, 3:] *= 0.3
        A = J @ np.swapaxes(J, 1, 2) + lam ** 2 * np.eye(6)
        dq = np.einsum("nji,nj->ni", J, np.linalg.solve(A, e[..., None])[..., 0])
        step = np.abs(dq).max(axis=1, keepdims=True)
        dq *= np.minimum(1.0, 0.4 / np.maximum(step, 1e-12))
        qa = q[act] + dq
        q[act] = np.where(qa > np.pi, qa - 2 * np.pi, np.where(qa < -np.pi, qa + 2 * np.pi, qa))
    t, R = pose(p, q, s, temp, tool)
    ok = ((np.linalg.norm(t - target, axis=1) < 1e-7) & (np.linalg.norm(_rot_err(R, Rd), axis=1) < 1e-6)
          & np.all(q > Q_LO, axis=1) & np.all(q < Q_HI, axis=1))
    return q, ok


def ik_position(p, target, q0, s, temp, tool, iters=4):
    """Minimal joint motion from q0 to put the tip at target (position only). (N, 6)."""
    q = np.atleast_2d(np.array(q0, float))
    for _ in range(iters):
        J, t, _ = joint_jacobian(p, q, s, temp, tool, rot=False)
        e = np.atleast_2d(target) - t
        A = J @ np.swapaxes(J, 1, 2) + 1e-10 * np.eye(3)
        q = q + np.einsum("nji,nj->ni", J, np.linalg.solve(A, e[..., None])[..., 0])
    return q


def frame_pointing(a, spin, tool):
    """Flange rotation that points the tool's tip vector along world direction a, spun by `spin`
    about it. a: (N, 3), spin: (N,)."""
    v = tip_vector(tool)
    v = v / np.linalg.norm(v)
    a = a / np.linalg.norm(a, axis=1, keepdims=True)
    # rotation taking v to a (Rodrigues), then spin about a
    w = np.cross(v[None], a)
    c = a @ v
    sn = np.linalg.norm(w, axis=1)
    k = w / np.maximum(sn, 1e-12)[:, None]
    ang = np.arctan2(sn, c)
    Ra = _rodrigues(k, ang)
    Rs = _rodrigues(a, spin)
    return Rs @ Ra


def _rodrigues(k, ang):
    K = np.zeros((len(k), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -k[:, 2], k[:, 1], -k[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = k[:, 2], -k[:, 1], k[:, 0]
    a = ang[:, None, None]
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


# ---- obstacle clearance (the stylus ball against spheres, posts and the table) ----------------
def clearance_cols(tips):
    """Clearance (m) of the stylus ball at each tip position to each surveyed sphere, each post,
    the table and the region where the gauge plate may stand (cell_data.plate_envelope: the plate is
    never touched): (N, 26) = 12 spheres, 12 posts, table, plate."""
    c = SPHERES
    out = np.empty((len(tips), 2 * len(c) + 2))
    out[:, :len(c)] = np.linalg.norm(tips[:, None, :] - c[None], axis=2) - R_TOUCH
    for k in range(len(c)):
        a = np.array([c[k, 0], c[k, 1], 0.0])
        ab = c[k] - a
        t = np.clip(((tips - a) @ ab) / (ab @ ab), 0.0, 1.0)
        out[:, len(c) + k] = np.linalg.norm(tips - (a + t[:, None] * ab), axis=1) - (R_POST + R_STYLUS)
    out[:, -2] = tips[:, 2] - R_STYLUS
    out[:, -1] = pj.envelope_clearance(tips) - R_STYLUS
    return out


def clearance(tips, skip_sphere=None):
    """Smallest clearance (N,), leaving out sphere `skip_sphere` and its post."""
    cols = clearance_cols(tips)
    if skip_sphere is not None:
        cols[:, [skip_sphere, len(SPHERES) + skip_sphere]] = np.inf
    return cols.min(axis=1)


def path_samples(qa, qb, step=0.002):
    reach = np.abs(qb - qa).max()
    n = int(max(2, np.ceil(reach / step) + 1))
    lam = np.linspace(0.0, 1.0, n)
    return qa[None] + lam[:, None] * (qb - qa)[None]


def motion_dirs(qa, qb, s_prev):
    dq = qb - qa
    return np.where(np.abs(dq) > S_THRESHOLD, np.sign(dq), s_prev)


# ---- the agent's handle on the cell ------------------------------------------------------------
class Robot:
    """Everything the calibration may do with the cell: its five commands. Keeps only the bound
    methods (never the cell, so none of the cell's hidden state is reachable from here) and what
    the observations say, plus the joint directions it tracks from its own commands."""

    def __init__(self, cell):
        self._move, self._probe = cell.move, cell.probe
        self._change_tool, self._wait, self._image = cell.change_tool, cell.wait, cell.image
        self.s = np.full(6, np.nan)                 # unknown until each joint has been commanded
        self.n_moves = 0
        # every reply's time, temperature readings and the joints' travel since the last reply (encoder
        # readings): the heating and cooling are estimated from it (thermal_history)
        self.history = []
        self.q = np.array(Q_HOME, float)
        self._observe(self._wait(0.01), np.zeros(6))

    def _observe(self, obs, dq):
        self.s = motion_dirs(np.zeros(6), dq, self.s)
        q = np.array(obs["q"], float)
        self.history.append((float(obs["time_s"]), np.array(obs["temp"], float), np.abs(q - self.q)))
        self.q = q
        self.temp = np.array(obs["temp"], float)
        self.tool = obs["tool"]
        self.time_left = obs["time_left_s"]
        self.probes_left = obs["probes_left"]
        self.images_left = obs["images_left"]
        self.damaged = obs["damaged"]
        return obs

    def sanitize(self, q_target):
        """Joints that would move by an amount whose direction is ambiguous for the backlash
        bookkeeping (near the 1e-4 rad threshold) are not moved at all."""
        q_target = np.array(q_target, float)
        dq = q_target - self.q
        hold = np.abs(dq) < 2.5e-4
        q_target[hold] = self.q[hold]
        return np.clip(q_target, am.Q_MIN, am.Q_MAX)

    def move(self, q_target, speed=MOVE_SPEED_MAX):
        q_target = self.sanitize(q_target)
        dq = q_target - self.q
        obs = self._observe(self._move(q_target, speed), dq)
        self.n_moves += 1
        if obs["collided"]:
            raise RuntimeError("collision: the planner failed")
        return obs

    def probe(self, q_target):
        q_target = self.sanitize(q_target)
        dq = q_target - self.q
        return self._observe(self._probe(q_target), dq)

    def change_tool(self, name):
        return self._observe(self._change_tool(name), np.zeros(6))

    def wait(self, seconds):
        return self._observe(self._wait(seconds), np.zeros(6))

    def image(self):
        return self._observe(self._image(), np.zeros(6))



# ---- design: candidate touches and their information -------------------------------------------
TOUCH_COLS = np.r_[0:NP, NC:INTR.start]     # the parameters a touch can inform (not the camera's)
_DESIGN = {}


def conditions_arrays(conditions):
    q = np.asarray(conditions["q"], float)
    return q, np.asarray(conditions["s"], float), np.asarray(conditions["temp"], float), conditions["tool"]


def acceptance_weights(conditions, p=None):
    """W_tip (NX x NX): mean of G^T G with G the sensitivity of the acceptance tool's tip to the
    parameters on the unit's own conditions; W_cam: the same for the camera-test points."""
    p = nominal_x() if p is None else p
    q, s, temp, tool = conditions_arrays(conditions)
    pts = np.asarray(conditions["cam_points"], float)
    load = gravity_load(q, tool)
    G = jacobian(p, q, s, temp, tool, np.arange(NX), load)[1]
    Gc = jacobian(p, q, s, temp, tool, np.arange(NX), load, what="camera", points=pts)[1].reshape(-1, 3, NX)
    return np.einsum("nap,naq->pq", G, G) / len(G), np.einsum("nap,naq->pq", Gc, Gc) / len(Gc)


def make_candidates(tool, n_try, rng):
    """Touch candidates for one tool: a sphere, a contact normal, a flange orientation and the
    nominal joint solution at a 5 mm standoff; the probe's joint direction and the joints it
    hardly moves (those are preloaded in a designed direction)."""
    nom = nominal_x()
    k = rng.integers(len(SPHERES), size=n_try)
    n = rng.normal(size=(n_try, 3))
    n[:, 2] = np.abs(n[:, 2]) - 0.4                    # from above, the side, a little below
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    t = rng.normal(size=(n_try, 3))
    t -= (t * n).sum(1, keepdims=True) * n
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    tilt = rng.uniform(0, 80, n_try) * D               # stylus within 80 deg of the normal
    a = -n * np.cos(tilt)[:, None] + t * np.sin(tilt)[:, None]
    Rd = frame_pointing(a, rng.uniform(-np.pi, np.pi, n_try), tool)
    seed = rng.uniform(am.Q_MIN * 0.9, am.Q_MAX * 0.9, (n_try, 6))
    target = SPHERES[k] + n * (R_TOUCH + 0.005)
    seed[:, 0] = np.arctan2(target[:, 1], target[:, 0]) + rng.normal(0, 0.3, n_try)
    zero, cold = np.zeros(6), np.full(6, am.T0)
    qs, ok = ik_pose(nom, target, Rd, seed, zero, cold, tool)
    far = SPHERES[k] + n * (R_TOUCH + 0.080)
    qf, okf = ik_pose(nom, far, Rd, qs, zero, cold, tool, iters=20)
    ok &= okf & (np.abs(qf - qs).max(axis=1) < 0.8)
    qs, k, n, Rd = qs[ok], k[ok], n[ok], Rd[ok]
    qc = ik_position(nom, SPHERES[k] + n * R_TOUCH, qs, zero, cold, tool)
    dq = qc - qs
    small = np.abs(dq) * (8.0 / 5.0) < 2.5e-4          # at a typical 8 mm probe
    pre = rng.choice([-1.0, 1.0], (len(qs), 6))
    return dict(tool=tool, qs=qs, qc=qc, k=k, n=n, Rd=Rd, dq=dq, small=small, pre=pre)


def candidate_rows(p, cand, temp):
    """Rows n . d tip / d p (touch columns) of each candidate at link temperature `temp`, both
    preload variants. The kappa and thermal-drift columns scale with (temp - T0); the backlash
    column of a preloaded joint flips sign with the preload: computed once at 22 C and scaled."""
    s = np.where(cand["small"], cand["pre"], np.sign(cand["dq"]))
    load = gravity_load(cand["qc"], cand["tool"])
    J = jacobian(p, cand["qc"], s, np.full(6, 22.0), cand["tool"], TOUCH_COLS, load)[1]
    rows = np.zeros((len(J), NX))
    rows[:, TOUCH_COLS] = np.einsum("na,nap->np", cand["n"], J)
    out = {}
    for name, T in temp.items():
        r = rows.copy()
        f = (T - am.T0) / (22.0 - am.T0)
        r[:, SL["kappa"]] *= f
        r[:, LIB["thermal"]] *= f
        r2 = r.copy()
        flip = cand["small"]                      # these joints take the preload's direction
        bcols = np.arange(SL["b"].start, SL["b"].stop)
        r2[:, bcols] = np.where(flip, -r[:, bcols], r[:, bcols])
        out[name] = np.concatenate([r, r2])
    return out


def design_pool():
    """Candidate pool; deterministic, the same for every unit (the nominal geometry)."""
    if not _DESIGN:
        rng = np.random.default_rng(20240611)
        nom = nominal_x()
        for tool, n_try in ((TOOL_A, 8000), (TOOL_B, 9000)):
            c = make_candidates(tool, n_try, rng)
            rows = candidate_rows(nom, c, {"cold": 22.0, "warm": 0.5 * (ACCEPT_TEMP[1][0] + ACCEPT_TEMP[1][1])})
            c2 = {key: (np.concatenate([v, v]) if isinstance(v, np.ndarray) else v) for key, v in c.items()}
            c2["pre"] = np.concatenate([c["pre"], -c["pre"]])
            for name, r in rows.items():
                c2["rows_" + name] = r
            _DESIGN[tool] = c2
    return _DESIGN


def greedy(Cov, W, groups, quotas, sphere_cap=None, exclude_spheres=(), masks=None, counts0=None,
           min_per_sphere=0):
    """I-optimal greedy selection: each step adds the candidate touch that most reduces the
    expected mean squared process-tool tip error tr(W Cov). groups: name -> (rows / sigma, sphere
    index). Spheres with fewer than min_per_sphere touches (counts0 already taken) get their
    candidates' gain doubled, so every sphere is touched often enough to tell whether it has been
    knocked. Returns name -> list of chosen indices (in order of choice)."""
    Cov = Cov.copy()
    st = {}
    for name, (C, ks) in groups.items():
        A = C @ Cov
        st[name] = dict(C=C, k=ks, A=A, num=np.einsum("np,pq,nq->n", A, W, A),
                        den=1.0 + np.einsum("np,np->n", A, C), mask=~np.isin(ks, exclude_spheres))
        if masks is not None and name in masks:
            st[name]["mask"] &= masks[name]
    chosen = {name: [] for name in groups}
    per_sphere = {name: np.zeros(len(SPHERES), int) for name in groups}
    total = np.zeros(len(SPHERES), int) if counts0 is None else np.array(counts0, int)
    while True:
        best = None
        for name, g in st.items():
            if len(chosen[name]) >= quotas.get(name, 0):
                continue
            gain = np.where(g["mask"], g["num"] / g["den"], -np.inf)
            gain = gain * np.where(total[g["k"]] < min_per_sphere, 2.0, 1.0)
            if sphere_cap is not None:
                gain = np.where(per_sphere[name][g["k"]] >= sphere_cap, -np.inf, gain)
            i = int(np.argmax(gain))
            if np.isfinite(gain[i]) and (best is None or gain[i] > best[0]):
                best = (gain[i], name, i)
        if best is None:
            break
        _, name, i = best
        u = st[name]["A"][i].copy()
        d0 = st[name]["den"][i]
        Wu = W @ u
        uWu = u @ Wu
        for g in st.values():
            a = g["C"] @ u
            AWu = g["A"] @ Wu
            g["num"] = g["num"] - 2 * a * AWu / d0 + a ** 2 * uWu / d0 ** 2
            g["den"] = g["den"] - a ** 2 / d0
            g["A"] -= np.outer(a, u) / d0
        Cov -= np.outer(u, u) / d0
        chosen[name].append(i)
        per_sphere[name][st[name]["k"][i]] += 1
        total[st[name]["k"][i]] += 1
    return chosen, Cov


# ---- fitting ---------------------------------------------------------------------------------
SIGMA = SIGMA_PLAN                          # m per touch: set from the data by `calibrate`


def touch_free(sd):
    """The parameters a touch fit adjusts: those with a prior (sd > 0), not the camera's."""
    m = sd > 0
    m[CAM] = False
    m[VIS] = False
    return m


def stack(data):
    """Touch records grouped by tool: arrays q, s, temp, k, load."""
    out = {}
    for tool in (TOOL_A, TOOL_B):
        sel = [d for d in data if d["tool"] == tool]
        if sel:
            q = np.array([d["q"] for d in sel])
            out[tool] = dict(q=q, s=np.array([d["s"] for d in sel]), temp=np.array([d["temp"] for d in sel]),
                             k=np.array([d["k"] for d in sel]), load=gravity_load(q, tool),
                             idx=np.array([i for i, d in enumerate(data) if d["tool"] == tool]))
    return out


def residuals(p, data, centres=None, groups=None):
    """Touch residuals (m): distance of the modelled stylus centre to the sphere centre minus the
    contact radius; and the unit normals."""
    centres = SPHERES if centres is None else centres
    groups = stack(data) if groups is None else groups
    r, n = np.zeros(len(data)), np.zeros((len(data), 3))
    for tool, g in groups.items():
        v = tip(p, g["q"], g["s"], g["temp"], tool, g["load"]) - centres[g["k"]]
        dist = np.linalg.norm(v, axis=1)
        r[g["idx"]], n[g["idx"]] = dist - R_TOUCH, v / dist[:, None]
    return r, n


GEOMETRY = np.zeros(NX, bool)
GEOMETRY[SL["xyz"]] = GEOMETRY[SL["rpy"]] = True


def sphere_centre(tips, c0, iters=6):
    """The centre that puts the stylus centres `tips` (m, 3) at R_TOUCH from it (Gauss-Newton from
    c0, with a weak pull to c0 so that one or two touches cannot run away; Huber-like weights)."""
    c = np.array(c0, float)
    if len(tips) == 0:
        return c
    for _ in range(iters):
        v = tips - c
        d = np.linalg.norm(v, axis=1)
        r = d - R_TOUCH
        J = -v / d[:, None]
        a = np.abs(r - np.median(r)) / 1e-4
        w = np.where(a <= 1, 1.0, 1.0 / np.maximum(a, 1e-12))
        H = (J.T * w) @ J + np.eye(3) * 1e-2
        c = c - np.linalg.solve(H, (J.T * w) @ r + 1e-2 * (c - c0))
    return c


def fit(data, p0, mu, sd, free=None, free_spheres=(), robust=True, iters=8, sigma_c=2e-3, sigma=None):
    """MAP fit of the parameters (and of the centres of `free_spheres`) to touch records.
    Gaussian prior (mu, sd; parameters with sd = 0 are held at mu); Huber weights when robust.
    Returns p, centres, Cov (NX x NX, zero rows for parameters not fitted), residuals (m), lin."""
    sigma = SIGMA if sigma is None else sigma
    free = touch_free(sd) if free is None else (free & touch_free(sd))
    idx = np.nonzero(free)[0]
    free_spheres = list(free_spheres)
    nc = 3 * len(free_spheres)
    p = np.array(p0, float)
    p[sd == 0] = mu[sd == 0]
    centres = SPHERES.copy()
    groups = stack(data)
    m = len(idx) + nc
    P = np.concatenate([1.0 / sd[idx] ** 2, np.full(nc, 1.0 / sigma_c ** 2)])
    sel_k = [np.array([d["k"] == ks for d in data]) for ks in free_spheres]
    if free_spheres:                       # start each free centre where its touches put it
        tips = np.zeros((len(data), 3))
        for tool, g in groups.items():
            tips[g["idx"]] = tip(p, g["q"], g["s"], g["temp"], tool, g["load"])
        for ks, sel in zip(free_spheres, sel_k):
            centres[ks] = sphere_centre(tips[sel], SPHERES[ks])
    for it in range(iters):
        r, n = residuals(p, data, centres, groups)
        J = np.zeros((len(data), m))
        for tool, g in groups.items():
            Jt = tip_jacobian(p, g["q"], g["s"], g["temp"], tool, g["load"], cols=idx)
            J[g["idx"], :len(idx)] = np.einsum("na,nap->np", n[g["idx"]], Jt)
        for j, sel in enumerate(sel_k):
            J[sel, len(idx) + 3 * j:len(idx) + 3 * j + 3] = -n[sel]
        if robust:
            a = np.abs(r) / (3 * sigma)
            w = np.where(a <= 1, 1.0, 1.0 / np.maximum(a, 1e-12))         # Huber
            w_score = w
            if robust == "redescend":           # and no pull at all from touches far off (> 15 sigma),
                on_free = np.zeros(len(r), bool)   # except on the spheres being located
                for sel in sel_k:
                    on_free |= sel
                w = np.where(on_free, w, w * np.clip(1.0 - (a - 3.0) / 2.0, 0.0, 1.0))
        else:
            w = w_score = np.ones(len(r))
        x = np.concatenate([p[idx] - mu[idx], (centres[free_spheres] - SPHERES[free_spheres]).ravel()])
        H = (J.T * w) @ J / sigma ** 2 + np.diag(P)
        gvec = (J.T * w) @ r / sigma ** 2 + P * x
        step = -np.linalg.solve(H, gvec)
        p[idx] += step[:len(idx)]
        if nc:
            centres[free_spheres] += step[len(idx):].reshape(-1, 3)
        if np.abs(step[:len(idx)] / sd[idx]).max() < 1e-4 and (not nc or np.abs(step[len(idx):]).max() < 1e-7):
            break
    r, n = residuals(p, data, centres, groups)
    Cov = np.zeros((NX, NX))
    Cfull = np.linalg.inv(H)
    Cov[np.ix_(idx, idx)] = Cfull[:len(idx), :len(idx)]
    prior_cost = float(np.sum(((p[idx] - mu[idx]) / sd[idx]) ** 2))
    lev = np.einsum("np,pq,nq->n", J, Cfull, J) * w / sigma ** 2      # leverage of each touch
    lin = dict(J=J, w=w, w_score=w_score, Hinv=Cfull, r=r, n=n, prior_cost=prior_cost, idx=idx, lev=lev)
    return p, centres, Cov, r, lin


# Every way a probe can go wrong (a false trigger, a post or the table in the way) stops it
# early: the touch then looks OUTSIDE the sphere (positive residual). A touch far INSIDE can only
# mean a misplaced sphere or a wrong model. So the loss and the trimming are one-sided.
TRIM = 4.0                      # sigma: a touch further outside than this is an outlier
TRIM_IN = 6.0                   # sigma: ... further inside than this
SPHERE_PENALTY = 25.0           # sigma^2: cost of declaring a datum sphere displaced
SPHERE_MIN_SHIFT = 0.4 * KNOCK_MIN          # m: a knocked sphere has moved by KNOCK_MIN or more (documented)
SPHERE_MAX_SHIFT = KNOCK_MAX + 0.4e-3       # m: ... and by KNOCK_MAX at most
SPHERE_MIN_TOUCHES = 5
MAX_FREE = KNOCKED_SPHERES[1]


def _truncated(r, sigma=None):
    z = r / (SIGMA if sigma is None else sigma)
    return np.where(z > 0, np.minimum(z ** 2, TRIM ** 2), np.minimum(z ** 2, 64.0))


def sphere_scores(data, lin, free_spheres):
    """Score test (linearised chi-square drop, Huber weights) for setting each sphere's centre
    free, at a fitted solution."""
    J, w, Hinv, r, n = lin["J"], lin["w_score"], lin["Hinv"], lin["r"], lin["n"]
    ks_all = np.array([d["k"] for d in data])
    out = {}
    for ks in range(len(SPHERES)):
        sel = ks_all == ks
        if ks in free_spheres or sel.sum() < SPHERE_MIN_TOUCHES:
            continue
        A = np.zeros((len(data), 3))
        A[sel] = -n[sel]
        u = A.T @ (w * r) / SIGMA ** 2
        B = (A.T * w) @ J / SIGMA ** 2
        info0 = (A.T * w) @ A / SIGMA ** 2
        info = info0 - B @ Hinv @ B.T
        if np.linalg.eigvalsh(info).min() <= 1e-6 * np.trace(info0):
            info = info0                     # the fit gave these touches no weight: uncorrected score
        out[ks] = float(u @ np.linalg.solve(info + 1e-9 * np.eye(3), u))
    return out


def truncated_cost(r, lin):
    return float(_truncated(r).sum() + lin["prior_cost"])


def standardized(r, lin):
    """Externally standardised residuals: r / (sigma sqrt(1 - leverage)). A flexible model can
    bend to one wrong touch (its residual then looks small); its leverage tells."""
    return r / (SIGMA * np.sqrt(np.maximum(1.0 - lin["lev"], 1e-3)))


def _try_set(data, p, mu, sd, free, spheres):
    trial = fit(data, p, mu, sd, free=free, free_spheres=list(spheres), robust=True)
    for ks in spheres:
        if not SPHERE_MIN_SHIFT < np.linalg.norm(trial[1][ks] - SPHERES[ks]) < SPHERE_MAX_SHIFT:
            return None
    return truncated_cost(trial[3], trial[4]), trial


def find_displaced(data, p, mu, sd, free_spheres, max_free=MAX_FREE, log=None, free=None, penalty=SPHERE_PENALTY,
                   swaps=True):
    """Model selection for the knocked spheres: greedily set free the centre of the sphere that
    most lowers the truncated MAP cost (outliers cost at most TRIM^2) while the drop beats
    `penalty`; then try swapping each chosen sphere for the best-scoring others, and dropping it."""
    free_spheres = list(free_spheres)
    p, centres, _, r, lin = fit(data, p, mu, sd, free=free, free_spheres=free_spheres, robust=True)
    base = truncated_cost(r, lin)
    cur = (p, centres, None, r, lin)
    while len(free_spheres) < max_free:
        scores = sphere_scores(data, lin, free_spheres)
        if not scores:
            break
        best = None
        for ks in sorted(scores, key=scores.get, reverse=True)[:5]:
            got = _try_set(data, p, mu, sd, free, free_spheres + [ks])
            if got is not None and (best is None or got[0] < best[0]):
                best = (got[0], ks, got[1])
        if best is None:
            break
        if log is not None:
            log.append((dict((k, round(v)) for k, v in scores.items()), best[1], round(base - best[0])))
        if base - best[0] < penalty:
            break
        base = best[0]
        free_spheres.append(best[1])
        cur = best[2]
        p, centres, _, r, lin = cur
    if swaps and free_spheres:
        improved = True
        while improved:
            improved = False
            for f in list(free_spheres):
                rest = [k for k in free_spheres if k != f]
                got = _try_set(data, p, mu, sd, free, rest)
                if got is None:
                    continue
                if got[0] + penalty * len(rest) < base + penalty * len(free_spheres) - 1e-9:   # f was not worth it
                    base, cur, free_spheres, improved = got[0], got[1], rest, True
                    p = cur[0]
                    break
                scores = sphere_scores(data, got[1][4], rest)
                for ks in sorted(scores, key=scores.get, reverse=True)[:3]:
                    if ks == f:
                        continue
                    g2 = _try_set(data, p, mu, sd, free, rest + [ks])
                    if g2 is not None and g2[0] < base - 1.0:
                        base, cur, free_spheres, improved = g2[0], g2[1], rest + [ks], True
                        p = cur[0]
                        if log is not None:
                            log.append(("swap", f, ks))
                        break
                if improved:
                    break
    return free_spheres, cur[0]


def trim_outliers(keep, p, mu, sd, free, free_spheres, trimmed, max_trim=12, outside_only=False, limit=1.0,
                  robust=True):
    """Set aside, one at a time, the touch whose standardised residual is furthest beyond the
    one-sided limits (limit x TRIM outside, limit x TRIM_IN inside)."""
    for _ in range(max_trim):
        p, centres, Cov, r, lin = fit(keep, p, mu, sd, free=free, free_spheres=free_spheres, robust=robust)
        t = standardized(r, lin)
        z = np.where(t > 0, t / TRIM, -t / TRIM_IN if not outside_only else 0.0) / limit
        if not np.any(z > 1):
            break
        worst = int(np.argmax(z))
        trimmed.append(keep[worst])
        keep = keep[:worst] + keep[worst + 1:]
    return keep, p


def robust_calibration(data, mu, sd, free=None, p0=None):
    """Probe-only fit of one model (mu, sd): staged (geometry, then everything) and robust; touches
    far outside their sphere (false triggers that got through) set aside first; the displaced
    datum spheres found by model selection on a truncated loss; then outlying touches set aside by
    their standardised residuals, the spheres decided again, and touches set aside that the final
    fit explains after all taken back; a last plain least-squares fit on what is kept."""
    p0 = mu.copy() if p0 is None else p0
    full = touch_free(sd) if free is None else free
    keep = list(data)
    p, *_ = fit(keep, p0, mu, sd, free=full & GEOMETRY, robust=True)
    p, *_ = fit(keep, p, mu, sd, free=full, robust=True)
    log, trimmed = [], []
    # a knocked sphere's touches all look outside (or inside) by up to KNOCK_MAX: the spheres
    # are decided first (the truncated loss caps what a wrong touch can cost), then the touches
    free_spheres, p = find_displaced(keep, p, mu, sd, [], log=log, free=full)
    keep, p = trim_outliers(keep, p, mu, sd, full, free_spheres, trimmed)
    free_spheres, p = find_displaced(keep, p, mu, sd, [], log=log, free=full)
    for _ in range(2):
        p, centres, Cov, r, _l = fit(keep, p, mu, sd, free=full, free_spheres=free_spheres, robust=True)
        if trimmed:                          # take back what the present model explains
            rt, _ = residuals(p, trimmed, centres)
            z = np.where(rt > 0, rt / (TRIM * SIGMA), -rt / (TRIM_IN * SIGMA))
            keep = keep + [d for d, zi in zip(trimmed, z) if zi <= 0.8]
            trimmed = [d for d, zi in zip(trimmed, z) if zi > 0.8]
        keep, p = trim_outliers(keep, p, mu, sd, full, free_spheres, trimmed)
    p, centres, Cov, r, _l = fit(keep, p, mu, sd, free=full, free_spheres=free_spheres, robust=False)
    return p, dict(free_spheres=sorted(free_spheres), centres=centres, kept=len(keep), trimmed=trimmed,
                   residuals=r, Cov=Cov, sphere_log=log, keep=keep)


NONCYCLIC = tuple(c for c in LIB_CLASSES if c != "cyclic")
CYC_PENALTY = 2 * np.log(len(CYC_PAIRS)) + 10.0     # sigma^2: what a higher harmonic must buy


def probe_model(pair):
    """The probe-fit model: the first-order parameters, every class but the higher harmonic, and
    the harmonic `pair` = (j, n) if any (all 36 candidate coefficients together would let the
    model bend to single touches)."""
    mu, sd, _ = model_prior(NONCYCLIC + (("cyclic",) if pair is not None else ()), pair)
    return mu, sd


def cyclic_scores(data, p, centres, lin):
    """Score test (linearised chi-square drop, Huber weights) of adding each candidate higher
    harmonic to the model of a fit (its prior included)."""
    J, w, Hinv, r = lin["J"], lin["w_score"], lin["Hinv"], lin["r"]
    groups = stack(data)
    _, n = residuals(p, data, centres, groups)
    A_all = np.zeros((len(data), 36))
    for tool, g in groups.items():
        Jt = tip_jacobian(p, g["q"], g["s"], g["temp"], tool, g["load"], cols=LIB["cyclic"])
        A_all[g["idx"]] = np.einsum("na,nap->np", n[g["idx"]], Jt)
    s2 = class_prior("cyclic")[1] ** 2
    out = {}
    for i, jn in enumerate(CYC_PAIRS):
        A = A_all[:, 2 * i:2 * i + 2]
        u = A.T @ (w * r) / SIGMA ** 2
        B = (A.T * w) @ J / SIGMA ** 2
        info0 = (A.T * w) @ A / SIGMA ** 2 + np.diag(1.0 / s2)
        info = info0 - B @ Hinv @ B.T
        if np.linalg.eigvalsh(info).min() <= 1e-6 * np.trace(info0):
            info = info0
        out[jn] = float(u @ np.linalg.solve(info, u))
    return out


def _state_fit(data, p, spheres, pair):
    """Fit of one structure (knocked spheres, harmonic); None if a freed sphere moves outside the
    documented knock range. Returns (cost with the structure's price, fit tuple)."""
    mu, sd = probe_model(pair)
    trial = fit(data, p, mu, sd, free_spheres=list(spheres), robust="redescend")
    for ks in spheres:
        if not SPHERE_MIN_SHIFT < np.linalg.norm(trial[1][ks] - SPHERES[ks]) < SPHERE_MAX_SHIFT:
            return None
    price = SPHERE_PENALTY * len(spheres) + (CYC_PENALTY if pair is not None else 0.0)
    return truncated_cost(trial[3], trial[4]) + price, trial


STRUCTURE_STARTS = 3            # the greedy search also starts from each of this many leading harmonics


def structure_search(data, p, log=None):
    """Which datum spheres are knocked and which higher harmonic (if any) the arm has, decided
    together: the greedy search below started from no harmonic and from each of the
    STRUCTURE_STARTS harmonics that the score test ranks first on the plain fit (a harmonic and a
    knocked sphere can mimic each other well enough that the search from nothing takes the wrong
    first step and cannot undo it); the start whose final structure costs least wins."""
    base = _state_fit(data, p, [], None)
    _cost, (pp, centres, _, _r, lin) = base
    cs = cyclic_scores(data, pp, centres, lin)
    starts = [None] + sorted(cs, key=cs.get, reverse=True)[:STRUCTURE_STARTS]
    best = None
    for pair0 in starts:
        sub = []
        res = _structure_search_from(data, p, sub, pair0)
        if res is None:
            continue
        if best is None or res[3] < best[0][3] - 1e-9:
            best = (res, sub, pair0)
    if log is not None:
        log.extend(best[1])
        if best[2] is not None:
            log.append(("start", best[2]))
    return best[0][:3]


def _structure_search_from(data, p, log, pair0):
    """The greedy search from no knocked sphere and the harmonic pair0 (or none): greedy moves (free
    one more sphere, or add a harmonic), each step taking the move that most lowers the truncated
    MAP cost plus the structure's price (outliers cost at most TRIM^2); candidates ranked by score
    tests; then single swaps and drops while they help. Returns (spheres, pair, p, cost) or None."""
    spheres, pair = [], pair0
    best = _state_fit(data, p, spheres, pair)
    if best is None:
        return None
    while True:
        cost, (pp, centres, _, r, lin) = best
        moves = []
        if len(spheres) < MAX_FREE:
            sc = sphere_scores(data, lin, spheres)
            moves += [(spheres + [k], pair) for k in sorted(sc, key=sc.get, reverse=True)[:4]]
        if pair is None:
            cs = cyclic_scores(data, pp, centres, lin)
            moves += [(spheres, jn) for jn in sorted(cs, key=cs.get, reverse=True)[:3]]
        cand = [(m, _state_fit(data, pp, *m)) for m in moves]
        cand = [(m, g) for m, g in cand if g is not None]
        if not cand:
            break
        m, g = min(cand, key=lambda x: x[1][0])
        if g[0] >= cost:
            break
        if log is not None:
            log.append(("add", m[0], m[1], round(cost - g[0])))
        spheres, pair, best = list(m[0]), m[1], g
    # refinement: drop or swap one element while it helps
    improved = True
    while improved:
        improved = False
        cost, (pp, centres, _, r, lin) = best
        alts = [([k for k in spheres if k != f], pair) for f in spheres]
        if pair is not None:
            alts.append((spheres, None))
            cs = cyclic_scores(data, pp, centres, fit(data, pp, *probe_model(None), free_spheres=spheres, robust="redescend")[4])
            alts += [(spheres, jn) for jn in sorted(cs, key=cs.get, reverse=True)[:3] if jn != pair]
        for f in spheres:
            rest = [k for k in spheres if k != f]
            sc = sphere_scores(data, fit(data, pp, *probe_model(pair), free_spheres=rest, robust="redescend")[4], rest)
            alts += [(rest + [k], pair) for k in sorted(sc, key=sc.get, reverse=True)[:2] if k != f]
        for m in alts:
            g = _state_fit(data, pp, *m)
            if g is not None and g[0] < cost - 1.0:
                if log is not None:
                    log.append(("swap", m[0], m[1], round(cost - g[0])))
                spheres, pair, best, improved = list(m[0]), m[1], g, True
                break
    return sorted(spheres), pair, best[1][0], best[0]


def robust_probe_fit(good, p0, log=None):
    """The probe-only fit: the structure (knocked spheres, higher harmonic) decided on all the
    touches; outlying touches set aside by their standardised residuals; the repeatability
    estimated from the residuals; the structure decided again with it; touches set aside that the
    model explains after all taken back; a last plain least-squares fit on what is kept."""
    global SIGMA
    log = [] if log is None else log
    keep, trimmed = list(good), []
    mu, sd = probe_model(None)
    p, *_ = fit(keep, p0, mu, sd, free=touch_free(sd) & GEOMETRY, robust="redescend")
    p, *_ = fit(keep, p, mu, sd, robust="redescend")
    spheres, pair, p = structure_search(keep, p, log)
    mu, sd = probe_model(pair)
    keep, p = trim_outliers(keep, p, mu, sd, touch_free(sd), spheres, trimmed, robust="redescend")
    _p, _c, _C, r, lin = fit(keep, p, mu, sd, free_spheres=spheres, robust=False)
    SIGMA = estimate_sigma(r, lin)
    log.append(("sigma", round(SIGMA * 1e6, 1)))
    keep, trimmed = list(good), []           # decided again on every touch, with that sigma
    spheres, pair, p = structure_search(keep, p, log)
    mu, sd = probe_model(pair)
    keep, p = trim_outliers(keep, p, mu, sd, touch_free(sd), spheres, trimmed, robust="redescend")
    mu, sd = probe_model(pair)
    for _ in range(2):
        p, centres, Cov, r, _l = fit(keep, p, mu, sd, free_spheres=spheres, robust="redescend")
        if trimmed:                          # take back what the present model explains
            rt, _ = residuals(p, trimmed, centres)
            z = np.where(rt > 0, rt / (TRIM * SIGMA), -rt / (TRIM_IN * SIGMA))
            keep = keep + [d for d, zi in zip(trimmed, z) if zi <= 0.8]
            trimmed = [d for d, zi in zip(trimmed, z) if zi > 0.8]
        keep, p = trim_outliers(keep, p, mu, sd, touch_free(sd), spheres, trimmed, robust="redescend")
    p, centres, Cov, r, lin = fit(keep, p, mu, sd, free_spheres=spheres, robust=False)
    SIGMA = estimate_sigma(r, lin)
    return p, dict(free_spheres=spheres, centres=centres, kept=len(keep), trimmed=trimmed, residuals=r, Cov=Cov,
                   sphere_log=log, keep=keep, probe_model=(NONCYCLIC, pair), mu=mu, sd=sd)


def estimate_sigma(r, lin):
    """Repeatability from the residuals of a fit, robustly: 1.4826 x the median of
    |r| / sqrt(1 - leverage) (a few touches the model misses, a sphere knocked too little to have
    been found, do not inflate it), within the documented range (a little wider)."""
    t = np.abs(r) / np.sqrt(np.maximum(1.0 - lin["lev"], 1e-3))
    s = 1.4826 * float(np.median(t))
    return float(np.clip(s, 0.8 * SIGMA_REPEAT_RANGE[0], 1.2 * SIGMA_REPEAT_RANGE[1]))


# ---- the campaign --------------------------------------------------------------------------------
PRELOAD = 6e-4                  # rad: last joint move before each probe, in a designed direction
WARM_TARGET = 0.5 * (ACCEPT_TEMP[1][0] + ACCEPT_TEMP[1][1]) + 3.0     # degrees C: links 1..4 at the start of a warm block
VERIFY_SIGMA = 0.8e-3           # m: while the model is rougher than this, every touch is re-probed
VERIFY_BACK = 0.4e-3            # m: back-off before a re-probe
MIN_PER_SPHERE = 7              # touches per datum sphere the design aims for


class Campaign:
    def __init__(self, robot, W, verbose=False):
        self.rb = robot
        self.verbose = verbose
        self.mu, self.sd = full_prior()
        self.p = self.mu.copy()
        self.Cov = np.diag(self.sd ** 2)
        self.Cov[CAM, CAM] = 0.0                # touches see nothing of the camera
        self.Cov[VIS, VIS] = 0.0
        self.pool = design_pool()
        self.W = W
        self.data = []              # every probe that triggered; ok = believed a true sphere touch
        self.events = []            # every probe; kind: touch / false (trigger) / post / miss
        self.free_spheres = []
        self.since_fit = 0
        self.sigma_tip = 5e-3
        self.centres = SPHERES.copy()
        self._last_check = 0
        self._sig_cache = {}
        self.failed = {t: np.zeros(len(self.pool[t]["k"]), bool) for t in (TOOL_A, TOOL_B)}

    def say(self, *a):
        if self.verbose:
            print(*a, flush=True)

    # -- model state --
    def refit(self):
        good = [d for d in self.data if d["ok"]]
        if not good:
            return
        p, centres, Cov, r, _ = fit(good, self.p, self.mu, self.sd, free_spheres=self.free_spheres, robust=True, iters=6)
        self.p, self.Cov, self.centres = p, Cov, centres
        self.since_fit = 0

    def tip_sigma(self, q, s, tool):
        J = tip_jacobian(self.p, q, s, self.rb.temp, tool)
        C = np.einsum("nap,pq,nbq->nab", J, self.Cov, J)
        return float(np.sqrt(np.linalg.eigvalsh(C)[:, -1].max()))

    # -- motion --
    def path_clear(self, qa, qb, margin, skip=None, margin_skip=None, relax=False):
        """Is the joint-linear move qa -> qb clear of everything by `margin` under the current
        model? skip: the target sphere (its post is shielded by it), kept at margin_skip.
        relax: objects already nearer than the margin at qa must not get nearer (moving away)."""
        s_prev = self.rb.s
        variants = [np.nan_to_num(s_prev, nan=1.0)]
        if not np.all(np.isfinite(s_prev)):
            variants.append(np.nan_to_num(s_prev, nan=-1.0))
        qq = path_samples(qa, qb)
        nk = len(SPHERES)
        for sp in variants:
            t = tip(self.p, qq, motion_dirs(qa, qb, sp), self.rb.temp, self.rb.tool)
            cols = clearance_cols(t)
            req = np.full(cols.shape[1], margin)
            if skip is not None:
                req[skip] = -np.inf if margin_skip is None else margin_skip
                req[nk + skip] = 1.0e-3
            if relax:
                req = np.minimum(req, 0.9 * cols[0])
            if np.any(cols < req[None]):
                return False
        return True

    def sigma_at(self, q):
        key = tuple(np.round(q, 6))
        if key not in self._sig_cache:
            self._sig_cache[key] = self.tip_sigma(np.atleast_2d(q), np.nan_to_num(self.rb.s, nan=1.0), self.rb.tool)
        return self._sig_cache[key]

    def travel_margin(self, qa=None, qb=None):
        if qa is None:
            sig = self.sigma_tip
        else:
            sig = 1.25 * max(self.sigma_at(qa), self.sigma_at(qb))
        return DISPLACED_MAX + 3.0e-3 + 4.0 * sig

    def travel(self, q_goal):
        """Collision-free sequence of joint moves to q_goal (direct, or via waypoints)."""
        q_goal = np.array(q_goal, float)
        q0 = self.rb.q.copy()
        self._sig_cache = {}
        routes = [[q_goal], [Q_HOME, q_goal]]
        for hub in self.hubs():
            routes.append([hub, q_goal])
        for hub in self.hubs():
            routes.append([Q_HOME, hub, q_goal])
            routes.append([hub, Q_HOME, q_goal])
        for h1 in self.hubs():
            for h2 in self.hubs():
                if h1 is not h2:
                    routes.append([h1, h2, q_goal])
        for route in routes:
            pts = [q0] + route
            if all(self.path_clear(a, b, self.travel_margin(a, b), relax=(j == 0))
                   for j, (a, b) in enumerate(zip(pts[:-1], pts[1:]))):
                for q in route:
                    self.rb.move(q)
                return True
        if self.verbose:
            for b in [q_goal] + list(self.hubs()[:2]):
                t = tip(self.p, path_samples(q0, b), motion_dirs(q0, b, np.nan_to_num(self.rb.s, nan=1.0)),
                        self.rb.temp, self.rb.tool)
                cols = clearance_cols(t)
                j = np.unravel_index(np.argmin(cols), cols.shape)
                self.say("    travel fail: margin {:.1f} mm, min clr {:.1f} mm obj {} at {:.2f}, start clr {:.1f} end clr {:.1f}".format(
                    self.travel_margin(q0, b) * 1e3, cols.min() * 1e3, j[1], j[0] / len(cols), cols[0].min() * 1e3, cols[-1].min() * 1e3))
        return False

    def hubs(self):
        """Waypoints with the stylus high above the middle of the cell, at a few azimuths."""
        if not hasattr(self, "_hubs"):
            az = np.array([-100.0, -50.0, 0.0, 50.0, 100.0]) * D
            target = np.stack([0.30 * np.cos(az), 0.30 * np.sin(az), np.full(5, 0.85)], axis=1)
            seed = np.tile(np.array([0.0, -100.0, 60.0, 0.0, 40.0, 0.0]) * D, (5, 1))
            seed[:, 0] = az
            out, self._back = [], {}
            for tool in (TOOL_A, TOOL_B):
                q = ik_position(nominal_x(), target, seed, np.zeros(6), np.full(6, am.T0), tool, iters=8)
                ok = np.all((q > Q_LO) & (q < Q_HI), axis=1)
                out.extend(q[ok])
                self._back[tool] = q[2] if ok[2] else None      # the one at azimuth 0
            self._hubs = out
        return self._hubs

    def change_tool(self, name):
        if self.rb.tool == name:
            return
        if not self.travel(Q_HOME):
            raise RuntimeError("no safe way home")
        self.rb.change_tool(name)
        self.update_sigma(name)

    def warm_up(self, target, t_max):
        """Swing joints 1..4 in the empty sector behind the cell (the spheres stand within
        +-125 deg of azimuth) until links 1..4 are warm. Reached by turning joint 0 with the
        stylus high above the middle of the cell."""
        rb = self.rb
        self.hubs()
        front = self._back[rb.tool]
        if front is None:
            self.say("  warm-up: no hub")
            return
        back = front.copy()
        back[0] = 160 * D
        if not self.travel(front) or not self.path_clear(rb.q, back, 0.05):
            self.say("  warm-up: no way to the back sector")
            return
        rb.move(back)
        rng = np.random.default_rng(7)
        span = np.array([0.0, 60.0, 70.0, 150.0, 100.0, 0.0]) * D
        best = None
        for _ in range(300):
            qa = np.clip(back + rng.uniform(-1, 1, 6) * span, Q_LO, Q_HI)
            qb = np.clip(back + rng.uniform(-1, 1, 6) * span, Q_LO, Q_HI)
            score = np.abs(qa - qb)[1:5].sum() / (np.abs(qa - qb).max() / MOVE_SPEED_MAX + SETTLE)
            if best is not None and score <= best[0]:
                continue
            if all(self.path_clear(a, b, 0.05) for a, b in ((back, qa), (qa, qb), (qb, qa), (qa, back))):
                best = (score, qa, qb)
        if best is None:
            self.say("  warm-up: no clear swing found")
            return
        _, qa, qb = best
        rb.move(qa)
        t_end = rb.time_left - t_max
        while rb.temp[1:5].mean() < target and rb.time_left > t_end:
            rb.move(qb if np.abs(rb.q - qa).max() < 1e-6 else qa)
        if np.abs(rb.q - qa).max() > 1e-6:
            rb.move(qa)
        rb.move(back)
        rb.move(front)
        self.say("  warm-up done: temp", np.round(rb.temp, 1), "time left", round(rb.time_left))

    # -- one touch --
    def touch(self, tool, i, rows_name):
        c = self.pool[tool]
        k, n, Rd, pre = int(c["k"][i]), c["n"][i], c["Rd"][i], c["pre"][i]
        g = c["rows_" + rows_name][i]
        sig_n = float(np.sqrt(g @ self.Cov @ g + SIGMA ** 2))
        # standoff and depth: a knocked sphere (DISPLACED_MAX), the preload step, 4 sigma
        d_s = DISPLACED_MAX + 2.5e-3 + 4.0 * sig_n
        d_t = DISPLACED_MAX + 1.0e-3 + 4.0 * sig_n
        d_f = max(d_s + 10e-3, self.travel_margin() + 5e-3)
        ctr = SPHERES[k]
        T = self.rb.temp
        qS, ok = ik_pose(self.p, (ctr + n * (R_TOUCH + d_s))[None], Rd[None], c["qs"][i][None], pre, T, tool)
        if not ok[0]:
            return "unreachable"
        qS = qS[0]
        qF, ok = ik_pose(self.p, (ctr + n * (R_TOUCH + d_f))[None], Rd[None], qS[None], pre, T, tool, iters=20)
        if not ok[0] or np.abs(qF[0] - qS).max() > 0.6:
            return "unreachable"
        qF = qF[0]
        d_e = d_f + 40e-3
        qE, ok = ik_pose(self.p, (ctr + n * (R_TOUCH + d_e))[None], Rd[None], qF[None], pre, T, tool, iters=20)
        if not ok[0] or np.abs(qE[0] - qF).max() > 0.8:
            return "unreachable"
        qE = qE[0]
        qT = ik_position(self.p, (ctr + n * (R_TOUCH - d_t))[None], qS, pre, T, tool)[0]
        hold = np.abs(qT - qS) < 2.5e-4
        qT[hold] = qS[hold]
        qP = qS - pre * PRELOAD
        if np.any(qT < am.Q_MIN) or np.any(qT > am.Q_MAX) or np.any(qP < am.Q_MIN) or np.any(qP > am.Q_MAX):
            return "unreachable"
        m_k = max(1.0e-3, d_s - 2.5e-3 - DISPLACED_MAX)   # predicted clearance kept to the target
        self._sig_cache = {}
        m_app = self.travel_margin(qE, qS)
        if not (self.path_clear(qE, qF, m_app, skip=k, margin_skip=0.6 * d_f)
                and self.path_clear(qF, qP, m_app, skip=k, margin_skip=m_k)
                and self.path_clear(qP, qS, m_app, skip=k, margin_skip=m_k)):
            return "unsafe approach"
        need_t = (np.abs(qT - qS).max() / PROBE_SPEED + 10.0)
        if self.rb.time_left < need_t + 60 or self.rb.probes_left < 1:
            return "budget"
        if not self.travel(qE):
            return "unsafe travel"
        self.rb.move(qF)
        self.rb.move(qP)
        self.rb.move(qS)
        obs = self.rb.probe(qT)
        kinds = []
        rec = self.record(obs, tool, k, rows_name)
        rec["sig_n"] = sig_n
        kinds.append(rec["kind"])
        # A touch that stopped outside where the model expects the surface (or any touch while the
        # model is still rough) is checked: back off a little along the path and probe again. A
        # genuine contact is found again where it was (and the repeat is a measurement too); after
        # a false trigger the probe goes on to the surface.
        tries = 0
        step = (qT - qS) * (VERIFY_BACK / (d_s + d_t))
        while (rec["kind"] == "touch" and tries < 3 and self.rb.probes_left > 0
               and (sig_n > VERIFY_SIGMA or rec["r"] > max(3.0 * sig_n, 0.08e-3))):
            tries += 1
            here = tip(self.p, self.rb.q, self.rb.s, self.rb.temp, tool)[0]
            r_here = np.linalg.norm(here - ctr) - R_TOUCH
            for grow in (1.0, 1.5, 2.25, 3.4, 5.0):         # clear of the backlash jump
                qb = self.rb.sanitize(self.rb.q - step * grow)
                tb = tip(self.p, qb, motion_dirs(self.rb.q, qb, self.rb.s), self.rb.temp, tool)[0]
                if np.linalg.norm(tb - ctr) - R_TOUCH - r_here > 0.25e-3:
                    break
            self.rb.move(qb)
            obs = self.rb.probe(qT)
            if not obs["triggered"]:            # nothing there: the first stop was false
                rec["kind"], rec["ok"] = "false", False
                kinds.append("false+miss")
                break
            new = self.record(obs, tool, k, rows_name)
            if rec["r"] - new["r"] > 0.12e-3:   # the earlier stop was well outside the surface
                rec["kind"], rec["ok"] = "false", False
                rec = new
                kinds.append("false+" + new["kind"])
            elif new["r"] - rec["r"] > 0.12e-3:  # the re-probe stopped early: it is the false one
                new["kind"], new["ok"] = "false", False
                kinds.append("again-false")
            else:
                kinds.append("again")
                break
        # back out along the approach
        if not self.path_clear(self.rb.q, qF, m_app, skip=k, relax=True):
            self.say("  warning: retract path tight")
        self.rb.move(qF)
        self.rb.move(qE)
        self.since_fit += 1
        return ",".join(kinds)

    def record(self, obs, tool, k, rows_name):
        rec = dict(q=self.rb.q.copy(), s=self.rb.s.copy(), temp=self.rb.temp.copy(), tool=tool, k=k,
                   block=rows_name, kind="miss", ok=False, r=np.nan)
        if obs["triggered"]:
            t = tip(self.p, rec["q"], rec["s"], rec["temp"], tool)[0]
            r = float(np.linalg.norm(t - self.centres[k]) - R_TOUCH)
            gap_post = clearance(t[None])[0]
            rec["r"] = r
            rec["kind"], rec["ok"] = "touch", True
            dk = np.linalg.norm(t - SPHERES[k]) - R_TOUCH
            if gap_post < dk - 1.0e-3 and gap_post < 0.5e-3:
                rec["kind"], rec["ok"] = "post", False
            self.data.append(rec)
        self.events.append(rec)
        return rec

    # -- the plan --
    def design(self, blocks, Cov):
        """Greedy I-optimal choice of touches for the remaining blocks given the current
        information. blocks: list of (tool, rows_name, quota)."""
        groups, quotas, masks = {}, {}, {}
        for b, (tool, rows, quota) in enumerate(blocks):
            c = self.pool[tool]
            groups[b] = (c["rows_" + rows] / SIGMA, c["k"])
            quotas[b] = quota
            masks[b] = ~self.failed[tool]
        counts = np.bincount([d["k"] for d in self.data if d["ok"]], minlength=len(SPHERES))
        chosen, _ = greedy(Cov, self.W, groups, quotas, exclude_spheres=self.free_spheres, masks=masks,
                           counts0=counts, min_per_sphere=MIN_PER_SPHERE)
        return chosen

    def order(self, tool, idx):
        """Nearest-neighbour tour (max joint motion) from the present pose."""
        c = self.pool[tool]
        left = list(idx)
        out, q = [], self.rb.q
        while left:
            j = min(left, key=lambda i: np.abs(c["qs"][i] - q).max())
            out.append(j)
            left.remove(j)
            q = c["qs"][j]
        return out

    def run_block(self, tool, rows_name, picks):
        """Take the touches `picks` (in a short tour); returns how many used the probe."""
        used = 0
        for i in self.order(tool, picks):
            if self.rb.probes_left < 1 or self.rb.time_left < 60:
                break
            n0 = self.rb.probes_left
            res = self.touch(tool, i, rows_name)
            if self.rb.probes_left == n0:
                self.failed[tool][i] = True
            else:
                used += 1
            self.say("  touch {} sphere {} -> {}  (probes left {}, time left {:.0f}, sigma_tip {:.3f} mm)".format(
                rows_name, self.pool[tool]["k"][i], res, self.rb.probes_left, self.rb.time_left, self.sigma_tip * 1e3))
            if self.sigma_tip > 0.3e-3 or self.since_fit >= 4:
                self.refit()
                self.update_sigma(tool)
            n_ok = sum(d["ok"] for d in self.data)
            if n_ok >= 50 and n_ok - self._last_check >= 15:
                self.check_spheres()
        self.refit()
        self.update_sigma(tool)
        return used

    def update_sigma(self, tool):
        c = self.pool[tool]
        sel = np.arange(0, len(c["qs"]), 37)
        J = tip_jacobian(self.p, c["qs"][sel], c["pre"][sel], self.rb.temp, tool)
        C = np.einsum("nap,pq,nbq->nab", J, self.Cov, J)
        self.sigma_tip = float(np.sqrt(np.linalg.eigvalsh(C)[:, -1].max()))

    def run(self):
        rb = self.rb
        self.update_sigma(TOOL_A)
        plan = [(TOOL_A, "cold", 26), (TOOL_B, "cold", 44), (TOOL_B, "warm", 30)]
        reserve = 4
        # bootstrap: one touch per sphere from above with probe_a, long standoffs, every touch
        # re-probed while the model is rough
        c = self.pool[TOOL_A]
        boot = {0: (c["rows_cold"] / SIGMA, c["k"])}
        chosen, _ = greedy(self.Cov, self.W, boot, {0: 12}, sphere_cap=1, masks={0: c["n"][:, 2] > 0.5})
        picks = chosen[0]
        picks = [picks[j] for j in np.argsort(np.abs(np.arctan2(SPHERES[c["k"][picks], 1],
                                                              SPHERES[c["k"][picks], 0])))]
        self.say("bootstrap")
        done0 = 0
        for i in picks:
            n0 = rb.probes_left
            res = self.touch(TOOL_A, i, "cold")
            done0 += rb.probes_left < n0
            self.say("  boot sphere {} -> {} (probes left {}, time left {:.0f}, sigma_tip {:.3f} mm)".format(
                c["k"][i], res, rb.probes_left, rb.time_left, self.sigma_tip * 1e3))
            self.refit()
            self.update_sigma(TOOL_A)
        for b in range(len(plan)):
            tool, rows_name, _ = plan[b]
            self.change_tool(tool)
            if rows_name == "warm" and rb.temp[1:5].mean() < WARM_TARGET - 4:
                self.warm_up(WARM_TARGET, t_max=300)
            self.check_spheres()
            # quotas for what is left, scaled to the probes left
            last = b == len(plan) - 1
            avail = rb.probes_left - (0 if last else reserve)
            f = avail / max(sum(x[2] for x in plan[b:]) - (done0 if b == 0 else 0), 1)
            target = int(round((plan[b][2] - (done0 if b == 0 else 0)) * f)) if not last else rb.probes_left
            done = 0
            self.say("block", b, tool, rows_name, "target", target)
            while done < target and rb.probes_left > 0 and rb.time_left > 60:
                blocks = [(tool, rows_name, target - done)] + [(t, r, int(round(qn * f))) for t, r, qn in plan[b + 1:]]
                picks = self.design(blocks, self.Cov_design())[0]
                if not picks:
                    break
                got = self.run_block(tool, rows_name, picks)
                done += got
                if got == 0:
                    break

    def Cov_design(self):
        return self.Cov + 1e-30 * np.eye(NX)

    def check_spheres(self):
        """Set free the centre of a datum sphere whose touches say it has been knocked."""
        good = [d for d in self.data if d["ok"]]
        if len(good) < 50:              # too little redundancy before that
            return
        self._last_check = len(good)
        before = list(self.free_spheres)
        # decided afresh each time, so an early wrong call is undone when more touches are in
        self.free_spheres, _ = find_displaced(good, self.p, self.mu, self.sd, [], penalty=2 * SPHERE_PENALTY)
        if self.free_spheres != before:
            self.say("  displaced spheres now", self.free_spheres)
            self.refit()


# ==== the wrist camera ============================================================================
# camera.md: 1280 x 960 square pixels; focal length 1400 px nominal (+-1 %); principal point within
# 8 px of the image centre and radial distortion of at most 4 px at the corners, neither
# calibrated; finite depth of field; fixed lamps, semi-gloss white spheres, dark posts, a grey
# table with a faint grain, walls beyond; automatic exposure per image; a few hot and dead pixels.
# What is measured: the OUTLINE of each datum sphere against whatever is behind it (never its
# shading, which depends on lamps the documents do not give). The rays tangent to a sphere form a
# cone around the direction of its centre, of half-angle asin(R_SPHERE / distance): the cone of an
# outline gives the centre in the camera frame, given the intrinsics; the intrinsics are unknowns
# of the fits, alongside the camera's mounting and the arm.
F_PX, CX, CY = F_PX_NOMINAL, CX_NOMINAL, CY_NOMINAL     # nominal intrinsics: planning only
SIGMA_CAM_LAT = 14e-6                       # m at 0.35 m: a sphere centre across the line of sight (planning)
SIGMA_CAM_DEPTH = 300e-6                    # m at 0.35 m: ... along it (planning)
LAT_SYS_PX = 0.05                           # px: floor of a measured centre's error across the line of sight
DEPTH_SYS = 300e-6                          # m at 0.35 m: floor of its error along it (the outline's size
                                            # depends on the shading near the limb: the depth comes mostly
                                            # from the directions seen from several poses)
N_VIEWS = BUDGET["images"] - 2 - pj.N_PLATE_VIEWS     # two images kept in hand; the plate's views are taken first
VIEW_TIME = 6.0                             # s: a typical view (travel, preload, image), for budgeting


def spheres_in_camera(p, q, s, temp, tool, centres, load=None):
    """Centres (N, 12, 3) in the camera frame, and the camera poses."""
    Rc, oc = camera_pose(p, q, s, temp, tool, load)
    return np.einsum("nab,nka->nkb", Rc, centres[None] - oc[:, None]), Rc, oc


def to_pixels(P):
    """Nominal pinhole (planning)."""
    return CX + F_PX * P[..., 0] / P[..., 2], CY + F_PX * P[..., 1] / P[..., 2]


def outline_box(P):
    """Image box (u0, u1, v0, v1) of the outline of a sphere centred at P (camera frame), nominal
    pinhole, from the dual of its outline conic; None unless the outline is an ellipse in front of
    the camera."""
    d2 = float(P @ P)
    if P[2] <= 0 or d2 <= R_SPHERE ** 2:
        return None
    sin_b = R_SPHERE / np.sqrt(d2)
    cos_a = P[2] / np.sqrt(d2)
    if np.arccos(min(cos_a, 1.0)) + np.arcsin(sin_b) > 80 * D:
        return None
    Cs = np.linalg.inv(np.outer(P, P) - (d2 - R_SPHERE ** 2) * np.eye(3))
    out = []
    for a, cen in ((0, CX), (1, CY)):
        r = np.sqrt(max(Cs[a, 2] ** 2 - Cs[2, 2] * Cs[a, a], 0.0))
        lo, hi = sorted([(Cs[a, 2] - r) / Cs[2, 2], (Cs[a, 2] + r) / Cs[2, 2]])
        out += [cen + F_PX * lo, cen + F_PX * hi]
    return np.array(out)


def radius_px(P, f=F_PX):
    return f * R_SPHERE / np.sqrt(np.maximum((P * P).sum(-1) - R_SPHERE ** 2, 1e-12))


# ---- the camera's geometry with intrinsics th = (f, cx, cy, k1, k2) --------------------------------
def undistort_px(u, v, th, iters=8):
    """Ideal normalised coordinates of pixel points (inverse of the radial distortion)."""
    f, cx, cy, k1, k2 = th[:5]
    xd, yd = (u - cx) / f, (v - cy) / f
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        sc = 1.0 + r2 * (k1 + k2 * r2)
        x, y = xd / sc, yd / sc
    return x, y


def rays_px(u, v, th):
    """Unit rays (camera frame) through pixel points u, v."""
    x, y = undistort_px(u, v, th)
    n = 1.0 / np.sqrt(x * x + y * y + 1.0)
    return np.stack([x * n, y * n, n], axis=-1)


def project_px(P, th):
    """Pixel of camera-frame points P (..., 3)."""
    f, cx, cy, k1, k2 = th[:5]
    x, y = P[..., 0] / P[..., 2], P[..., 1] / P[..., 2]
    r2 = x * x + y * y
    sc = 1.0 + r2 * (k1 + k2 * r2)
    return cx + f * x * sc, cy + f * y * sc


def _perp_basis(a):
    t = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(a, t)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(a, e1)


def outline_px(P, th, n):
    """n points of the outline of the sphere centred at P (camera frame), in pixels, and their
    outward unit normals, and their angles around the cone."""
    d = np.linalg.norm(P)
    a = P / d
    beta = np.arcsin(min(R_SPHERE / d, 0.999))
    e1, e2 = _perp_basis(a)
    phi = 2 * np.pi * np.arange(n) / n
    dirs = np.cos(beta) * a[None] + np.sin(beta) * (np.cos(phi)[:, None] * e1[None] + np.sin(phi)[:, None] * e2[None])
    u, v = project_px(dirs, th)
    pts = np.stack([u, v], axis=1)
    tang = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    nrm = np.stack([tang[:, 1], -tang[:, 0]], axis=1)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    cen = np.array(project_px(a, th))
    nrm[((pts - cen) * nrm).sum(1) < 0] *= -1
    return pts, nrm, phi


def despeckle(img):
    """Hot and dead pixels: a pixel brighter than all eight of its neighbours by more than 25 grey
    levels, or darker than all of them by as much, is replaced by their median (a real edge pixel
    always lies between its neighbours)."""
    img = img.astype(float)
    ring = np.ones((3, 3), bool)
    ring[1, 1] = False
    hi = ndimage.maximum_filter(img, footprint=ring, mode="nearest")
    lo = ndimage.minimum_filter(img, footprint=ring, mode="nearest")
    bad = (img > hi + 25.0) | (img < lo - 25.0)
    if np.any(bad):
        med = ndimage.median_filter(img, footprint=ring, mode="nearest")
        img[bad] = med[bad]
    return img


# ---- an outline's position along one intensity profile ----------------------------------------
def _gauss(z):
    return np.exp(-0.5 * z * z) / np.sqrt(2 * np.pi)


def _Gauss(z):
    return 0.5 * (1.0 + special.erf(z / np.sqrt(2.0)))


def _sqrt_table():
    """F(z) = integral over x > 0 of sqrt(x) phi(x - z) dx: sqrt(max(x, 0)) blurred by N(0, 1)."""
    z = np.arange(-9.0, 60.0 + 1e-9, 0.01)
    x = np.linspace(0.0, 75.0, 30001)
    out = np.empty(len(z))
    for a in range(0, len(z), 200):
        g = np.sqrt(x)[None] * _gauss(x[None] - z[a:a + 200, None])
        out[a:a + 200] = (g.sum(1) - 0.5 * (g[:, 0] + g[:, -1])) * (x[1] - x[0])
    return z, out


_SQRT_TAB = []


def _blurred_sqrt(z):
    if not _SQRT_TAB:
        _SQRT_TAB.extend(_sqrt_table())
    return np.interp(z, _SQRT_TAB[0], _SQRT_TAB[1])


def _profile_basis(s, s0, w):
    """The profile model at offsets s (M, K) (outward), for an outline at s0 (M,) blurred by a
    Gaussian of sd w (M,); z = (s0 - s) / w. Whatever lies behind the sphere: a line across the
    whole profile (1, s). The sphere: from its outline inwards a step, a ramp, and the square root
    of the depth inside the outline (how the radiance of a lit sphere starts at its limb: the
    surface normal turns as the square root of the distance from the outline), all blurred:
    Phi(z), (s - s0) Phi(z) - w phi(z), sqrt(w) F(z)."""
    z = (s0[:, None] - s) / w[:, None]
    Pz = _Gauss(z)
    return np.stack([np.ones_like(s), s, Pz, (s - s0[:, None]) * Pz - w[:, None] * _gauss(z),
                     np.sqrt(w)[:, None] * _blurred_sqrt(z)], axis=-1)


def _project_out(B, I):
    A = np.einsum("mki,mkj->mij", B, B) + 1e-9 * np.eye(B.shape[-1])
    c = np.linalg.solve(A, np.einsum("mki,mk->mi", B, I)[..., None])[..., 0]
    return I - np.einsum("mki,mi->mk", B, c), c


def fit_profiles(s, I, s0, w, fit_w=True, iters=8):
    """Outline position s0 (and blur w) of each profile: the linear coefficients eliminated
    (variable projection), Gauss-Newton on the rest. Returns s0, w, sd of s0, coefficients, rms."""
    s0, w = s0.copy(), w.copy()
    h = 1e-3
    for _ in range(iters):
        r, _ = _project_out(_profile_basis(s, s0, w), I)
        J0 = (_project_out(_profile_basis(s, s0 + h, w), I)[0] - r) / h
        if fit_w:
            J1 = (_project_out(_profile_basis(s, s0, w + h), I)[0] - r) / h
            J = np.stack([J0, J1], axis=-1)
            A = np.einsum("mki,mkj->mij", J, J) + 1e-9 * np.eye(2)
            st = np.clip(-np.linalg.solve(A, np.einsum("mki,mk->mi", J, r)[..., None])[..., 0], -1.0, 1.0)
            s0 = s0 + st[:, 0]
            w = np.clip(w + st[:, 1], 0.4, 6.0)
        else:
            s0 = s0 - np.clip((J0 * r).sum(1) / np.maximum((J0 * J0).sum(1), 1e-12), -1.0, 1.0)
    r, c = _project_out(_profile_basis(s, s0, w), I)
    J0 = (_project_out(_profile_basis(s, s0 + h, w), I)[0] - r) / h
    s2 = (r * r).sum(1) / (s.shape[1] - (7 if fit_w else 6))
    return s0, w, np.sqrt(s2 / np.maximum((J0 * J0).sum(1), 1e-12)), c, np.sqrt(s2)


def edge_points(img, P, th, valid_fn=None, w0=None, group=4, half=None):
    """Sub-pixel points of the outline of the sphere expected near P (camera frame; intrinsics th):
    intensity profiles along the normals of the expected outline, one per pixel of arc, averaged in
    groups of `group` neighbours, each fitted with the blurred profile model; profiles that leave
    the image, touch a saturated or black pixel, meet another object (valid_fn) or show too little
    contrast are left out. Returns dict(uv (m, 2), sig (m,) px, blur px, ...) or None."""
    rpx = th[0] * R_SPHERE / np.sqrt(max(P @ P - R_SPHERE ** 2, 1e-9))
    n = int(np.clip(2 * np.pi * rpx / group, 90, 400))
    pts, nrm, phi = outline_px(P, th, n * group)
    w_init = 1.5 if w0 is None else w0
    half = float(np.clip(3.5 * w_init + 3.0, 6.0, 14.0)) if half is None else half
    s = np.arange(-half, half + 1e-9, 0.5)
    U = pts[:, 0:1] + s[None] * nrm[:, 0:1]
    V = pts[:, 1:2] + s[None] * nrm[:, 1:2]
    inside = (U.min(1) >= 1) & (U.max(1) <= IMAGE_W - 2) & (V.min(1) >= 1) & (V.max(1) <= IMAGE_H - 2)
    # Profiles that leave the image are dropped below (`inside`); their coordinates can be enormous (a rough
    # camera estimate, a sphere almost in the camera's plane), and the interpolation must never see them.
    U = np.clip(np.nan_to_num(U, nan=0.0, posinf=IMAGE_W, neginf=-1.0), -1.0, float(IMAGE_W))
    V = np.clip(np.nan_to_num(V, nan=0.0, posinf=IMAGE_H, neginf=-1.0), -1.0, float(IMAGE_H))
    I = ndimage.map_coordinates(img, [V.ravel(), U.ravel()], order=1, mode="nearest").reshape(U.shape)
    # no sample may draw on a saturated (or black) pixel
    clip = ((img >= 254.5) | (img <= 0.5)).astype(float)
    hit = ndimage.map_coordinates(clip, [V.ravel(), U.ravel()], order=1, mode="nearest").reshape(U.shape)
    ok = inside & (hit.max(1) == 0.0)
    if valid_fn is not None:
        ok &= valid_fn(U, V)
    ok_g = ok.reshape(n, group).all(1)
    if ok_g.sum() < 15:
        return None
    I = I.reshape(n, group, -1).mean(1)[ok_g]
    pts_k = pts.reshape(n, group, 2).mean(1)[ok_g]
    nrm_k = nrm.reshape(n, group, 2).mean(1)[ok_g]
    nrm_k /= np.linalg.norm(nrm_k, axis=1, keepdims=True)
    phi_k = phi.reshape(n, group).mean(1)[ok_g]
    g = np.abs(ndimage.gaussian_filter1d(I, 2.0, axis=1, order=1))
    g[:, :3] = g[:, -3:] = 0
    S = np.broadcast_to(s, I.shape)
    s0, w, sig, c, rms = fit_profiles(S, I, s[np.argmax(g, axis=1)], np.full(len(I), w_init), fit_w=True)
    good = (np.abs(s0) < half - 2.5 * w) & (np.abs(c[:, 2]) > np.maximum(4.0 * rms, 3.0)) & np.isfinite(sig)
    if good.sum() < 15:
        return None
    wm = float(np.median(w[good]))
    # the blur is one per sphere (one depth): refit the positions with it
    s0, _, sig, c, rms = fit_profiles(S, I, s0, np.full(len(I), wm), fit_w=False)
    good = ((np.abs(s0) < half - 2.5 * wm) & (np.abs(c[:, 2]) > np.maximum(4.0 * rms, 3.0)) & np.isfinite(sig)
            & (sig < 0.5))
    # a blurred curved edge: the profile's step lies inside the outline by about w^2 / (2 radius)
    s0 = s0 + wm * wm / (2.0 * rpx)
    uv = pts_k + s0[:, None] * nrm_k
    return dict(uv=uv[good], sig=np.maximum(sig[good], 0.01), blur=wm, phi=phi_k[good], n_try=n)


# ---- the cone of an outline: the sphere's centre ------------------------------------------------
def _cone_res(d, a, beta):
    return np.arctan2(np.linalg.norm(np.cross(d, a[None]), axis=1), d @ a) - beta


def _cone_params(x, a0, e1, e2):
    a = a0 + x[0] * e1 + x[1] * e2
    return a / np.linalg.norm(a), x[2]


def cone_fit(uv, sig, th, P0, w_fixed=None, iters=8, x0=None):
    """The centre P (camera frame) of the sphere whose outline passes through the pixel points uv
    (sd sig px), intrinsics th: the cone of rays tangent to the sphere (axis a, half-angle beta),
    residuals in pixels across the outline, Tukey weights (or w_fixed). Returns P, cov(P), the
    weights, the rms of the normalised residuals and the number of points kept."""
    d = rays_px(uv[:, 0], uv[:, 1], th)
    a0 = P0 / np.linalg.norm(P0)
    e1, e2 = _perp_basis(a0)
    # px per radian of cone angle at each point: 1 / |gradient of the angle field|
    gu = (_cone_res(rays_px(uv[:, 0] + 0.5, uv[:, 1], th), a0, 0) - _cone_res(rays_px(uv[:, 0] - 0.5, uv[:, 1], th), a0, 0))
    gv = (_cone_res(rays_px(uv[:, 0], uv[:, 1] + 0.5, th), a0, 0) - _cone_res(rays_px(uv[:, 0], uv[:, 1] - 0.5, th), a0, 0))
    k = 1.0 / np.hypot(gu, gv) / sig
    x = np.array([0.0, 0.0, np.arcsin(min(R_SPHERE / np.linalg.norm(P0), 0.999))]) if x0 is None else x0.copy()
    wts = np.ones(len(d)) if w_fixed is None else w_fixed
    J = np.zeros((len(d), 3))
    for it in range(iters):
        a, b = _cone_params(x, a0, e1, e2)
        r = _cone_res(d, a, b) * k
        if w_fixed is None and it >= 2:
            scale = max(1.4826 * np.median(np.abs(r[wts > 0])), 0.7)
            t = r / (4.685 * scale)
            wts = np.where(np.abs(t) < 1, (1 - t * t) ** 2, 0.0)
        for j in range(3):
            e = np.zeros(3)
            e[j] = 1e-7
            a2, b2 = _cone_params(x + e, a0, e1, e2)
            J[:, j] = (_cone_res(d, a2, b2) * k - r) / 1e-7
        Jw = J * wts[:, None]
        try:
            st = -np.linalg.solve(Jw.T @ J + 1e-12 * np.eye(3), Jw.T @ r)
        except np.linalg.LinAlgError:       # a degenerate outline (every point weighed out): least squares
            st = -np.linalg.lstsq(Jw.T @ J + 1e-12 * np.eye(3), Jw.T @ r, rcond=None)[0]
        x = x + st
        if np.abs(st).max() < 1e-11:
            break
    a, b = _cone_params(x, a0, e1, e2)
    r = _cone_res(d, a, b) * k
    chi2 = float((wts * r * r).sum() / max(wts.sum() - 3, 1))
    try:
        Cx = np.linalg.inv((J * wts[:, None]).T @ J + 1e-18 * np.eye(3)) * max(chi2, 1.0)
    except np.linalg.LinAlgError:
        Cx = np.linalg.pinv((J * wts[:, None]).T @ J) * max(chi2, 1.0)
    P = a * R_SPHERE / np.sin(b)
    Jp = np.empty((3, 3))
    for j in range(3):
        e = np.zeros(3)
        e[j] = 1e-7
        a2, b2 = _cone_params(x + e, a0, e1, e2)
        Jp[:, j] = (a2 * R_SPHERE / np.sin(b2) - P) / 1e-7
    return P, Jp @ Cx @ Jp.T, wts, np.sqrt(chi2), int((wts > 0).sum())


TH_STEP = np.array([0.5, 0.2, 0.2, 2e-4, 6e-4])


def sphere_measurement(uv, sig, th, P0, w_fixed=None):
    """A sphere's centre from its outline points at intrinsics th, its covariance (with the floors
    of systematic error), and its sensitivity G (3 x 6) to the intrinsics and the edge offset."""
    P, C, wts, rms, n = cone_fit(uv, sig, th, P0, w_fixed=w_fixed)
    G = np.empty((3, 6))
    for j in range(5):
        t = np.array(th[:5], float)
        t[j] += TH_STEP[j]
        G[:, j] = (cone_fit(uv, sig, t, P, w_fixed=wts, iters=3)[0] - P) / TH_STEP[j]
    # an outline offset outwards by delta px reads the sphere nearer by |P| delta / radius_px
    d = np.linalg.norm(P)
    G[:, 5] = -P * d / (th[0] * R_SPHERE)
    u = P / d
    C = C + (LAT_SYS_PX * d / th[0]) ** 2 * (np.eye(3) - np.outer(u, u)) + (DEPTH_SYS * (d / 0.35) ** 2) ** 2 * np.outer(u, u)
    return dict(P=P, cov=C, G=G, wts=wts, rms=rms, n_used=n, th0=np.array(th[:5], float))


# ---- finding a sphere near where it is expected ---------------------------------------------------
def detect_sphere(img, P_pred, th, search_px):
    """Hough search (votes along the intensity gradient, either polarity) for the outline of the
    sphere expected at P_pred: its centre pixel and radius, or None."""
    rpx = radius_px(P_pred, th[0])
    uc, vc = project_px(P_pred, th)
    m = rpx * 1.15 + search_px + 6
    i0, i1 = int(max(0, vc - m)), int(min(IMAGE_H, vc + m + 1))
    j0, j1 = int(max(0, uc - m)), int(min(IMAGE_W, uc + m + 1))
    if i1 - i0 < 20 or j1 - j0 < 20:
        return None
    sm = ndimage.gaussian_filter(img[i0:i1, j0:j1], 1.5)
    gy, gx = np.gradient(sm)
    mag = np.hypot(gx, gy)
    ii, jj = np.nonzero(mag > max(np.percentile(mag, 90), 1.5))
    if len(ii) < 30:
        return None
    wgt = np.sqrt(mag[ii, jj])
    nx, ny = gx[ii, jj] / mag[ii, jj], gy[ii, jj] / mag[ii, jj]
    H, W = sm.shape
    acc = np.zeros(H * W)
    for r in np.linspace(0.88, 1.12, 13) * rpx:
        for sgn in (-1.0, 1.0):
            ci = np.round(ii + sgn * r * ny).astype(int)
            cj = np.round(jj + sgn * r * nx).astype(int)
            ok = (ci >= 0) & (ci < H) & (cj >= 0) & (cj < W)
            acc += np.bincount(ci[ok] * W + cj[ok], weights=wgt[ok], minlength=H * W)
    acc = ndimage.gaussian_filter(acc.reshape(H, W), 2.0)
    yy, xx = np.mgrid[0:H, 0:W]
    dist2 = (yy + i0 - vc) ** 2 + (xx + j0 - uc) ** 2
    score = acc * np.exp(-0.5 * dist2 / search_px ** 2)
    score[dist2 > (1.3 * search_px) ** 2] = 0
    bi, bj = divmod(int(np.argmax(score)), W)
    if score[bi, bj] <= 0:
        return None
    dd = np.hypot(ii - bi, jj - bj)
    sel = (dd > 0.85 * rpx) & (dd < 1.15 * rpx)
    if sel.sum() < 20:
        return None
    hist, edges = np.histogram(dd[sel], bins=60, weights=wgt[sel])
    b = int(np.argmax(ndimage.gaussian_filter1d(hist, 1.0)))
    return bj + j0, bi + i0, 0.5 * (edges[b] + edges[b + 1])


def P_from_circle(u, v, rpx, th):
    """A sphere centre whose outline is (about) a circle of radius rpx about pixel (u, v)."""
    ray = rays_px(np.array([u]), np.array([v]), th)[0]
    return ray * R_SPHERE / np.sin(rpx / th[0])


# ---- what else is in the picture -------------------------------------------------------------------
def _near_post_rays(d, Rc, oc, c, margin_px, f, P=None):
    """Rays d (..., 3; camera frame) within margin_px of the post under the sphere centred at c
    (datum frame), where the post is not hidden behind the sphere P (camera frame)."""
    dd = d @ Rc.T
    wx, wy = oc[0] - c[0], oc[1] - c[1]
    a2 = np.maximum(dd[..., 0] ** 2 + dd[..., 1] ** 2, 1e-300)
    h = np.abs(wx * dd[..., 1] - wy * dd[..., 0]) / np.sqrt(a2)
    ts = -(wx * dd[..., 0] + wy * dd[..., 1]) / a2
    z = oc[2] + ts * dd[..., 2]
    near = (ts > 0) & (h < R_POST + margin_px * ts / f) & (z > -0.01) & (z < c[2] + 0.01)
    if P is None:
        return near
    rad = R_POST + margin_px * np.maximum(ts, 0.0) / f
    b2 = wx * dd[..., 0] + wy * dd[..., 1]
    disc = b2 * b2 - a2 * (wx * wx + wy * wy - rad * rad)
    t_post = (-b2 - np.sqrt(np.maximum(disc, 0.0))) / a2
    b = d @ P
    disc_s = b * b - (P @ P - R_SPHERE ** 2)
    t_sph = np.where(disc_s > 0, b - np.sqrt(np.maximum(disc_s, 0.0)), np.inf)
    return near & (t_post < t_sph)


def make_valid(k, Pall, Rc, oc, centres, th, margin_px, own_post_px):
    """A test of profile samples (pixel arrays U, V (M, K)): True for the profiles clear by
    margin_px of every other sphere's outline (and of the inside of the nearer ones) and of every
    post (own_post_px for the sphere's own post, which is placed with the sphere)."""
    f = th[0]
    dk = np.linalg.norm(Pall[k])
    others = []
    for j in range(len(Pall)):
        if j != k and Pall[j][2] > 0.02:
            dj = np.linalg.norm(Pall[j])
            others.append((Pall[j] / dj, np.arcsin(min(R_SPHERE / dj, 0.999)), dj < dk))

    def valid(U, V):
        d = rays_px(U, V, th)
        bad = np.zeros(U.shape, bool)
        for aj, bj, nearer in others:
            ang = np.arccos(np.clip(d @ aj, -1, 1)) - bj
            bad |= np.abs(ang) * f < margin_px
            if nearer:
                bad |= ang < 0
        for j in range(len(centres)):
            bad |= _near_post_rays(d, Rc, oc, centres[j], own_post_px if j == k else margin_px, f, Pall[k])
        return ~bad.any(axis=1)
    return valid


def _small_rotation(A, B):
    """Rotation taking the unit vectors A (n, 3) onto B (n, 3) best (Kabsch)."""
    U, _, Vt = np.linalg.svd(B.T @ A)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def measure_spheres(img, rec, p, centres, margin_px, prev=None, search_px=70.0):
    """The datum spheres in one image (despeckled, float): list of measurement dicts (k, P, cov,
    G, uv, sig, ...). The poses of everything come from p (all NX: the arm, the camera's mounting
    and intrinsics); where `prev` (this image's earlier measurements) has a sphere, its outline is
    looked for there, otherwise it is searched for within search_px of the prediction."""
    th = np.array(p[INTR], float)
    Pall, Rc, oc = spheres_in_camera(p, rec["q"], rec["s"], rec["temp"], rec["tool"], centres)
    Pall, Rc, oc = Pall[0], Rc[0], oc[0]
    prev = {m["k"]: m for m in (prev or [])}
    cand = []
    for k in range(len(centres)):
        P = Pall[k]
        if P[2] <= 0.05 or not 0.2 < np.linalg.norm(P) < 0.7:
            continue
        u, v = project_px(P, th)
        r = radius_px(P, th[0])
        # at least most of the outline inside the image
        if u < -0.3 * r or u > IMAGE_W - 1 + 0.3 * r or v < -0.3 * r or v > IMAGE_H - 1 + 0.3 * r:
            continue
        cand.append(k)
    # where each sphere is: the earlier measurement, or a search around the prediction
    start, wide = {}, set()
    for k in cand:
        if k in prev:
            start[k] = prev[k]["P"]
        else:
            det = detect_sphere(img, Pall[k], th, search_px)
            if det is not None:
                start[k] = P_from_circle(det[0], det[1], det[2], th)
                wide.add(k)
    if not start:
        return []
    # the other objects follow the spheres found (a pose error is mostly a common rotation)
    ks = list(start)
    A = np.array([Pall[k] / np.linalg.norm(Pall[k]) for k in ks])
    B = np.array([start[k] / np.linalg.norm(start[k]) for k in ks])
    Rcor = _small_rotation(A, B) if len(ks) >= 2 else _small_rotation(np.vstack([A, [[0, 0, 1.0]]]), np.vstack([B, [[0, 0, 1.0]]]))
    if np.arccos(np.clip((np.trace(Rcor) - 1) / 2, -1, 1)) > 0.05:
        Rcor = np.eye(3)
    Pc = Pall @ Rcor.T
    Rc_c = Rc @ Rcor.T
    out = []
    for k in ks:
        P0 = start[k]
        Pc_k = Pc.copy()
        Pc_k[k] = P0
        # the sphere's own post placed with the sphere: the camera centre moved by the difference
        oc_k = oc - Rc_c @ (P0 - Pc[k])
        valid = make_valid(k, Pc_k, Rc_c, oc_k, centres, th, margin_px, max(4.0, 0.25 * margin_px))
        e, m = None, None
        rounds = 3 if k in wide else 2
        for it in range(rounds):
            # a detection's radius is rough: the first window is wide
            # (its blur unknown yet: up to ~3 px for a sphere far from the focus)
            half = 0.12 * radius_px(P0, th[0]) + 4.0 if (k in wide and it == 0) else None
            w0 = None if (e is None or half) else e["blur"]
            e = edge_points(img, P0, th, valid, w0=w0, half=half)
            if e is None:
                break
            m = sphere_measurement(e["uv"], e["sig"], th, P0) if it == rounds - 1 else None
            if it < rounds - 1:
                P0 = cone_fit(e["uv"], e["sig"], th, P0)[0]
        if e is None or m is None:
            continue
        arc = m["n_used"] / e["n_try"]
        if arc < 0.35 or m["rms"] > 4.0 or not 0.2 < np.linalg.norm(m["P"]) < 0.7:
            continue
        m.update(k=k, uv=e["uv"], sig=e["sig"], blur=e["blur"], arc=arc)
        out.append(m)
    return out


def relinearize(obs, th):
    """Every sphere measurement recomputed from its outline points at the intrinsics th."""
    out = []
    for rec in obs:
        r = dict(rec)
        r["spheres"] = []
        for m in rec["spheres"]:
            if "uv" not in m:
                r["spheres"].append(m)
                continue
            m2 = dict(m)
            m2.update(sphere_measurement(m["uv"], m["sig"], th, m["P"]))
            r["spheres"].append(m2)
        out.append(r)
    return out


# ---- the camera fit ----------------------------------------------------------------------------
def _obs_cov(P):
    """Planning: measurement covariance of a sphere centre (camera frame), across / along the line
    of sight."""
    d = np.linalg.norm(P)
    u = P / d
    lat, dep = SIGMA_CAM_LAT * d / 0.35, SIGMA_CAM_DEPTH * (d / 0.35) ** 2
    return lat ** 2 * np.eye(3) + (dep ** 2 - lat ** 2) * np.outer(u, u)


def _image_blocks(obs, centres):
    """Group the sphere measurements per image: measured centres y0 at the intrinsics th0 they were
    computed with, their sensitivity G to (intrinsics, edge offset), and whitening matrices
    (measurement covariance and the arrival's repeatability, common to all the spheres of an
    image)."""
    blocks = []
    for rec in obs:
        if not rec["spheres"]:
            continue
        ks = np.array([m["k"] for m in rec["spheres"]])
        y = np.concatenate([m["P"] for m in rec["spheres"]])
        n = len(ks)
        S = np.kron(np.ones((n, n)), np.eye(3)) * SIGMA ** 2
        G = np.zeros((3 * n, 6))
        v0 = vision_prior()[0]
        for i, m in enumerate(rec["spheres"]):
            S[3 * i:3 * i + 3, 3 * i:3 * i + 3] += m["cov"] if "G" in m else _obs_cov(m["P"])
            if "G" in m:
                G[3 * i:3 * i + 3] = m["G"]
                v0 = np.concatenate([m["th0"], [0.0]])
        Lw = np.linalg.cholesky(np.linalg.inv(S)).T       # Lw^T Lw = S^-1
        blocks.append(dict(rec=rec, ks=ks, y=y, G=G, v0=v0, Lw=Lw))
    return blocks


def _measured(b, p):
    """The block's measured centres at the parameters' intrinsics and edge offset (linearised)."""
    return b["y"] + b["G"] @ (p[VIS] - b["v0"])


def image_residuals(p, blocks, centres):
    r = []
    for b in blocks:
        rec = b["rec"]
        P = spheres_in_camera(p, rec["q"], rec["s"], rec["temp"], rec["tool"], centres[b["ks"]])[0][0]
        r.append(b["Lw"] @ (_measured(b, p) - P.ravel()))
    return np.concatenate(r) if r else np.zeros(0)


def _camera_groups(blocks):
    by_tool = {}
    for n, b in enumerate(blocks):
        by_tool.setdefault(b["rec"]["tool"], []).append(n)
    cams = {}
    for tl, ns in by_tool.items():
        q = np.array([blocks[n]["rec"]["q"] for n in ns])
        cams[tl] = dict(ns=ns, q=q, s=np.array([blocks[n]["rec"]["s"] for n in ns]),
                        temp=np.array([blocks[n]["rec"]["temp"] for n in ns]), load=gravity_load(q, tl))
    return cams


def _image_res_jac(p, blocks, cams, centres, cols, free_spheres):
    """Whitened image residuals (list per block) and their Jacobian (rows, len(cols) + 3 per free
    sphere), the camera poses of all the images for all perturbed parameter vectors at once; the
    intrinsics and the edge offset enter through the measurements (analytic columns)."""
    cols = np.asarray(cols, int)
    pose_cols = cols[(cols < VIS.start) | (cols >= VIS.stop)]
    vis_pos = [(i, c - VIS.start) for i, c in enumerate(cols) if VIS.start <= c < VIS.stop]
    pose_pos = np.array([i for i, c in enumerate(cols) if not VIS.start <= c < VIS.stop], int)
    out_r = [None] * len(blocks)
    out_J = [None] * len(blocks)
    for tl, g in cams.items():
        X = np.repeat(p[None], 1 + len(pose_cols), axis=0)
        X[1 + np.arange(len(pose_cols)), pose_cols] += _FD_STEP[pose_cols]
        R, o = fk(X, g["q"], g["s"], g["temp"], g["load"])
        Rc = np.einsum("bnac,bcd->bnad", R, np.stack([am.camera_rotation(x[81:84]) for x in X]))
        oc = o + np.einsum("bnac,bc->bna", R, np.stack([_cam_vec(x[78:84]) for x in X]))
        for i, n in enumerate(g["ns"]):
            b = blocks[n]
            ks = b["ks"]
            P = np.einsum("bac,bkc->bka", np.swapaxes(Rc[:, i], -1, -2), centres[ks][None] - oc[:, i, None, :])
            res = b["Lw"] @ (_measured(b, p)[None] - P.reshape(len(X), -1)).T            # (rows, B)
            out_r[n] = res[:, 0]
            Jp = np.zeros((len(res), len(cols)))
            Jp[:, pose_pos] = (res[:, 1:] - res[:, :1]) / _FD_STEP[pose_cols][None]
            for ic, jv in vis_pos:
                Jp[:, ic] = b["Lw"] @ b["G"][:, jv]
            Js = np.zeros((len(res), 3 * len(free_spheres)))
            for jj, ksp in enumerate(free_spheres):
                hit = np.nonzero(ks == ksp)[0]
                if len(hit):                       # d P / d centre = Rc^T for that sphere
                    blk = np.zeros((3 * len(ks), 3))
                    blk[3 * hit[0]:3 * hit[0] + 3] = Rc[0, i].T
                    Js[:, 3 * jj:3 * jj + 3] = -(b["Lw"] @ blk)
            out_J[n] = np.concatenate([Jp, Js], axis=1)
    return out_r, out_J


CAMERA_COLS = np.r_[CAM.start:CAM.stop, VIS.start:VIS.stop]


def fit_camera(p, obs, centres, iters=10):
    """The camera's mounting (6), intrinsics (5) and edge offset by weighted least squares (MAP with
    the documented tolerances), the arm held. Returns p (NX), covariance of those 12, whitened
    residuals."""
    p = np.array(p, float)
    mu, sd = np.zeros(NX), np.zeros(NX)
    mu[CAM], sd[CAM] = camera_prior()
    mu[VIS], sd[VIS] = vision_prior()
    cols = CAMERA_COLS
    blocks = _image_blocks(obs, centres)
    cams = _camera_groups(blocks)
    if not blocks:
        return p, np.diag(sd[cols] ** 2), np.zeros(0)
    # measurements that do not depend on the intrinsics (no G) leave them at the prior
    for it in range(iters):
        rl, Jl = _image_res_jac(p, blocks, cams, centres, cols, [])
        r, J = np.concatenate(rl), np.concatenate(Jl)
        H = J.T @ J + np.diag(1 / sd[cols] ** 2)
        g = -J.T @ r - (p[cols] - mu[cols]) / sd[cols] ** 2
        step = np.linalg.solve(H, g)
        p[cols] += step
        if np.abs(step / sd[cols]).max() < 1e-6:
            break
    return p, np.linalg.inv(H), image_residuals(p, blocks, centres)


def fit_joint(data, obs, p0, mu, sd, free_spheres, centres0, iters=8, sigma_c=2e-3, blocks=None):
    """MAP fit of the parameters of a model (mu, sd: sd = 0 holds a parameter at mu) and of the
    centres of the displaced datum spheres to the kept touches and the sphere measurements of
    the images together. Returns p, centres, touch residuals (m), whitened image residuals per
    image block, the posterior covariance and the linearisation (for the model evidence)."""
    free = sd > 0
    idx = np.nonzero(free)[0]
    free_spheres = list(free_spheres)
    nc = 3 * len(free_spheres)
    p = np.array(p0, float)
    p[~free] = mu[~free]
    centres = SPHERES.copy()
    centres[free_spheres] = centres0[free_spheres]
    groups = stack(data)
    blocks = _image_blocks(obs, centres) if blocks is None else blocks
    cams = _camera_groups(blocks)
    m = len(idx) + nc
    P_prior = np.concatenate([1.0 / sd[idx] ** 2, np.full(nc, 1.0 / sigma_c ** 2)])
    cam_like = set(range(CAM.start, CAM.stop)) | set(range(VIS.start, VIS.stop))
    tidx = np.array([k for k, i in enumerate(idx) if i not in cam_like], int)
    sel_k = [np.array([d["k"] == ks for d in data]) for ks in free_spheres]
    for it in range(iters):
        r_t, n_t = residuals(p, data, centres, groups)
        J_t = np.zeros((len(data), m))
        for tl, g in groups.items():
            Jt = tip_jacobian(p, g["q"], g["s"], g["temp"], tl, g["load"], cols=idx[tidx])
            J_t[np.ix_(g["idx"], tidx)] = np.einsum("na,nap->np", n_t[g["idx"]], Jt)
        for j, sel in enumerate(sel_k):
            J_t[sel, len(idx) + 3 * j:len(idx) + 3 * j + 3] = -n_t[sel]
        rl, Jl = _image_res_jac(p, blocks, cams, centres, idx, free_spheres)
        r_i = np.concatenate(rl) if rl else np.zeros(0)
        J_i = np.concatenate(Jl) if Jl else np.zeros((0, m))
        x = np.concatenate([p[idx] - mu[idx], (centres[free_spheres] - SPHERES[free_spheres]).ravel()])
        H = J_t.T @ J_t / SIGMA ** 2 + J_i.T @ J_i + np.diag(P_prior)
        gv = J_t.T @ r_t / SIGMA ** 2 + J_i.T @ r_i + P_prior * x
        step = -np.linalg.solve(H, gv)
        p[idx] += step[:len(idx)]
        if nc:
            centres[free_spheres] += step[len(idx):].reshape(-1, 3)
        if np.abs(step[:len(idx)] / sd[idx]).max() < 1e-4:
            break
    r_t, n_t = residuals(p, data, centres, groups)
    rl, Jl = _image_res_jac(p, blocks, cams, centres, idx, free_spheres)
    Cov = np.zeros((NX, NX))
    Hinv = np.linalg.inv(H)
    Cov[np.ix_(idx, idx)] = Hinv[:len(idx), :len(idx)]
    lev = np.einsum("np,pq,nq->n", J_t, Hinv, J_t) / SIGMA ** 2        # leverage of each touch
    return p, centres, r_t, rl, Cov, dict(idx=idx, H=H, lev=lev)


def joint_calibration(data, obs, p0, mu, sd, free_spheres, centres0, max_trim=4, trim=3.0):
    """fit_joint, then images whose sphere measurements disagree with the rest (rms of the
    whitened residuals above `trim`) set aside one at a time, and the fit repeated; when the
    intrinsics have moved, the measurements are recomputed at the new ones first."""
    obs = [o for o in obs if o["spheres"]]
    dropped = []
    for _ in range(max_trim + 1):
        p, centres, r_t, r_i, Cov, lin = fit_joint(data, obs, p0, mu, sd, free_spheres, centres0)
        if obs and _needs_relinearize(obs, p):
            obs = relinearize(obs, p[INTR])
            p, centres, r_t, r_i, Cov, lin = fit_joint(data, obs, p, mu, sd, free_spheres, centres0)
        z = np.array([np.sqrt(np.mean(r ** 2)) for r in r_i])
        lev = lin["lev"]
        if len(z) == 0 or z.max() <= trim:
            break
        worst = int(np.argmax(z))
        dropped.append(obs[worst])
        obs = obs[:worst] + obs[worst + 1:]
    return p, dict(centres=centres, touch_residuals=r_t, image_rms=z, dropped=len(dropped), Cov=Cov, obs=obs, lev=lev)


def _needs_relinearize(obs, p, tol=0.05):
    """Have the intrinsics moved from those the measurements were made at by more than tol px
    (anywhere in the image)?"""
    rc = np.hypot(CX_NOMINAL, CY_NOMINAL) / F_PX_NOMINAL
    px_per_unit = np.array([rc, 1.0, 1.0, F_PX_NOMINAL * rc ** 3, F_PX_NOMINAL * rc ** 5])
    for rec in obs:
        for m in rec["spheres"]:
            if "th0" in m:
                return bool((np.abs(np.asarray(p[INTR]) - m["th0"]) * px_per_unit).max() > tol)
    return False


STAGE_ARM_SD = 0.15e-3          # m: what the probe-fitted arm alone may be off by at a camera pose


def sphere_residuals(p, obs, centres, extra=0.0):
    """Per sphere measurement: its residual (measured minus predicted centre) normalised by its own
    covariance plus `extra` (m, isotropic sd) (chi with 3 degrees of freedom), as a list per image."""
    out = []
    for rec in obs:
        z = []
        if rec["spheres"]:
            P = spheres_in_camera(p, rec["q"], rec["s"], rec["temp"], rec["tool"], centres)[0][0]
            for m in rec["spheres"]:
                y = m["P"] + (m["G"] @ (p[VIS] - np.concatenate([m["th0"], [0.0]])) if "G" in m else 0.0)
                C = (m["cov"] if "G" in m else _obs_cov(m["P"])) + (SIGMA ** 2 + extra ** 2) * np.eye(3)
                e = y - P[m["k"]]
                z.append(float(np.sqrt(e @ np.linalg.solve(C, e) / 3.0)))
        out.append(z)
    return out


def robust_camera_fit(p, obs, centres, rounds=8, cut=8.0):
    """fit_camera, the measurements recomputed at the new intrinsics, and the sphere measurements
    that disagree most (normalised residual above `cut`, worst half of them per round) set aside. The
    arm is held at its probe fit: its error is allowed for (STAGE_ARM_SD)."""
    for _ in range(rounds):
        p, _, _ = fit_camera(p, obs, centres)
        obs = relinearize(obs, p[INTR])
        z = sphere_residuals(p, obs, centres, STAGE_ARM_SD)
        flat = [v for zz in z for v in zz]
        # relative to the bulk: when the arm held at its probe fit is off, every residual is large
        c = max(cut, 3.0 * float(np.median(flat))) if flat else cut
        if not flat or max(flat) <= c:
            break
        lim = max(c, np.percentile([v for v in flat if v > c], 50))
        obs = [dict(rec, spheres=[m for m, v in zip(rec["spheres"], zz) if v < lim]) for rec, zz in zip(obs, z)]
    p, _, _ = fit_camera(p, obs, centres)
    return p, relinearize(obs, p[INTR])


# ---- planning the views ------------------------------------------------------------------------
def _frame_from_axis(v, roll):
    """Camera axes (columns) with optical axis v (N, 3), image x horizontal before the roll."""
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(v, up)
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    y = np.cross(v, x)
    c, s = np.cos(roll)[:, None], np.sin(roll)[:, None]
    return np.stack([c * x + s * y, -s * x + c * y, v], axis=2)


def view_candidates(p, tool, temp, rng, n_try=3000):
    """Camera poses looking at the datum spheres from the middle of the cell, their joint
    solutions (nominal camera on the flange, current kinematics), the spheres each would see
    cleanly, and the rows (sensitivity of those sphere measurements to the 84 parameters)."""
    k = rng.integers(len(SPHERES), size=n_try)
    az = np.arctan2(SPHERES[k, 1], SPHERES[k, 0]) + rng.uniform(-45, 45, n_try) * D
    el = rng.uniform(-20, 45, n_try) * D                     # looking down by el
    v = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), -np.sin(el)], axis=1)
    dist = rng.uniform(0.27, 0.48, n_try)
    Rv = _frame_from_axis(v, rng.uniform(-np.pi, np.pi, n_try))
    # the sphere off the image centre (anywhere in the image, to the corners: the intrinsics are
    # calibrated too): tilt the axis by up to ~ +-22 / 17 degrees
    a, b = rng.uniform(-0.38, 0.38, n_try), rng.uniform(-0.29, 0.29, n_try)
    tilt = np.stack([np.stack([np.cos(a), np.zeros(n_try), np.sin(a)], 1),
                     np.stack([np.sin(a) * np.sin(b), np.cos(b), -np.cos(a) * np.sin(b)], 1),
                     np.stack([-np.sin(a) * np.cos(b), np.sin(b), np.cos(a) * np.cos(b)], 1)], axis=1)
    Rc = Rv @ tilt
    oc = SPHERES[k] - dist[:, None] * v
    # and views of two neighbouring spheres at once, across the image (their angular separation
    # ties the focal length, hence the camera's place along its axis, and the principal point)
    Rp, op_, kp = pair_views(rng, n_try)
    Rc, oc, k = np.concatenate([Rc, Rp]), np.concatenate([oc, op_]), np.concatenate([k, kp])
    n_try = len(k)
    xc = p[CAM]
    R6 = Rc @ am.camera_rotation(xc[3:]).T
    target = oc + np.einsum("nab,b->na", R6, tip_vector(tool) - _cam_vec(xc))
    pre = rng.choice([-1.0, 1.0], (n_try, 6))
    seed = rng.uniform(am.Q_MIN * 0.9, am.Q_MAX * 0.9, (n_try, 6))
    seed[:, 0] = np.arctan2(oc[:, 1], oc[:, 0]) + rng.normal(0, 0.3, n_try)
    q, ok = ik_pose(p, target, R6, seed, np.zeros(6), temp, tool)
    # camera body and stylus clear of everything; the stylus far from the table
    ok &= oc[:, 2] > 0.08
    ok &= np.linalg.norm(oc[:, None, :] - SPHERES[None], axis=2).min(axis=1) > 0.12
    ok &= np.hypot(oc[:, None, 0] - SPHERES[None, :, 0], oc[:, None, 1] - SPHERES[None, :, 1]).min(axis=1) > 0.06
    tips = tip(p, q, pre, temp, tool)
    ok &= clearance(tips) > 0.02
    out = []
    for n in np.nonzero(ok)[0]:
        P = spheres_in_camera(p, q[n], pre[n], temp, tool, SPHERES)[0][0]
        good = clean_spheres(P)
        if good:
            out.append(dict(q=q[n], pre=pre[n], ks=good, k=int(k[n])))
    return out


def pair_views(rng, n):
    """Camera poses (axes Rc (n, 3, 3), centres (n, 3)) that see two neighbouring datum spheres
    near opposite sides (or corners) of the image, from 0.4 to 0.65 m; and the first sphere's index."""
    pairs = [(i, j) for i in range(len(SPHERES)) for j in range(i + 1, len(SPHERES))
             if np.linalg.norm(SPHERES[i] - SPHERES[j]) < 0.5]
    pick = rng.integers(len(pairs), size=n)
    I = np.array([pairs[t][0] for t in pick])
    J = np.array([pairs[t][1] for t in pick])
    m = 0.5 * (SPHERES[I] + SPHERES[J])
    e = SPHERES[J] - SPHERES[I]
    sep = np.linalg.norm(e, axis=1)
    e /= sep[:, None]
    # look outwards from the cell's middle, from above or below, square to the pair
    v = np.stack([m[:, 0], m[:, 1], np.zeros(n)], axis=1)
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v = v + rng.uniform(-0.7, 0.7, (n, 1)) * np.stack([-v[:, 1], v[:, 0], np.zeros(n)], axis=1)
    v[:, 2] = -np.tan(rng.uniform(-20, 40, n) * D)
    v -= (v * e).sum(1, keepdims=True) * e
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    half = rng.uniform(0.27, 0.40, n)                       # half the angle between the two
    dist = np.clip(0.5 * sep / np.tan(half), 0.0, 0.65)
    oc = m - dist[:, None] * v
    x = e - (e * v).sum(1, keepdims=True) * v
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    y = np.cross(v, x)
    roll = rng.uniform(-0.65, 0.65, n)                      # along the image's width or its diagonals
    c, s_ = np.cos(roll)[:, None], np.sin(roll)[:, None]
    Rc = np.stack([c * x + s_ * y, -s_ * x + c * y, v], axis=2)
    return Rc, oc, I


def clean_spheres(P, margin=40.0):
    """Spheres (indices) fully in the image (with a margin) whose outline is clear of every other
    sphere's and of every post's predicted outline by `margin` px."""
    boxes = [outline_box(P[j]) for j in range(len(P))]
    rpx = radius_px(P)
    u, v = to_pixels(P)
    good = []
    for j, bx in enumerate(boxes):
        if bx is None or bx[0] < 20 or bx[1] > IMAGE_W - 21 or bx[2] < 20 or bx[3] > IMAGE_H - 21:
            continue
        if not 0.24 < np.linalg.norm(P[j]) < 0.65:
            continue
        clear = True
        for i in range(len(P)):
            if i != j and P[i, 2] > 0.02 and np.hypot(u[i] - u[j], v[i] - v[j]) < rpx[i] + rpx[j] + margin:
                clear = False
        if clear:
            good.append(j)
    return good


def plan_G(P):
    """Planning: how a sphere centre measured from its outline depends on the intrinsics and the
    edge offset, (..., 3, 6) (the outline held where the nominal camera puts it)."""
    th0 = vision_prior()[0][:5]
    u0, v0 = project_px(P, th0)
    d0 = np.linalg.norm(P, axis=-1)
    beta0 = np.arcsin(np.minimum(R_SPHERE / d0, 0.999))
    G = np.zeros(P.shape + (6,))
    for j in range(5):
        th = th0.copy()
        th[j] += TH_STEP[j]
        Pj = rays_px(u0, v0, th) * (R_SPHERE / np.sin(beta0 * th0[0] / th[0]))[..., None]
        G[..., j] = (Pj - P) / TH_STEP[j]
    G[..., 5] = -P * (d0 / (th0[0] * R_SPHERE))[..., None]
    return G


def view_rows(p, views, temp, tool, cols):
    """For each view: sensitivity (3 m x NX, nonzero in `cols`) of its predicted sphere centres
    (camera frame) to the parameters, and their measurement covariance (3 m x 3 m)."""
    q = np.array([v["q"] for v in views])
    pre = np.array([v["pre"] for v in views])
    temp = np.broadcast_to(temp, q.shape)
    load = gravity_load(q, tool)
    cols = np.asarray(cols, int)
    J = np.zeros((len(q), len(SPHERES), 3, NX))
    per = max(1, _CHUNK // max(len(q), 1))
    P0 = None
    for a in range(0, len(cols), per):
        cc = cols[a:a + per]
        X = np.repeat(p[None], 1 + len(cc), axis=0)
        X[1 + np.arange(len(cc)), cc] += _FD_STEP[cc]
        R, o = fk(X, q, pre, temp, load)
        Rc = np.einsum("bnac,bcd->bnad", R, np.stack([am.camera_rotation(x[81:84]) for x in X]))
        oc = o + np.einsum("bnac,bc->bna", R, np.stack([_cam_vec(x[78:84]) for x in X]))
        P = np.einsum("bnca,bnkc->bnka", Rc, SPHERES[None, None] - oc[:, :, None, :])
        P0 = P[0]
        J[..., cc] = np.moveaxis((P[1:] - P[:1]) / _FD_STEP[cc][:, None, None, None], 0, -1)
    if P0 is not None and np.any((cols >= VIS.start) & (cols < VIS.stop)):
        with np.errstate(all="ignore"):
            J[..., VIS] = np.nan_to_num(-plan_G(P0))     # the measurements' dependence on the intrinsics
    out = []
    for n, v in enumerate(views):
        ks = v["ks"]
        m = len(ks)
        S = np.kron(np.ones((m, m)), np.eye(3)) * SIGMA ** 2
        for i, k in enumerate(ks):
            S[3 * i:3 * i + 3, 3 * i:3 * i + 3] += _obs_cov(P0[n, k])
        out.append((J[n, ks].reshape(3 * m, NX), S))
    return out


def choose_views(cands, rows, Cov, W, n):
    """Greedy: each step adds the view that most reduces tr(W Cov), the expected mean squared
    camera-test error (block update for the spheres a view sees)."""
    Cov = Cov.copy()
    chosen = []
    for _ in range(min(n, len(cands))):
        best = None
        for i, (J, S) in enumerate(rows):
            if i in chosen:
                continue
            A = J @ Cov
            M = S + A @ J.T
            gain = np.trace(np.linalg.solve(M, A @ W @ A.T))
            if best is None or gain > best[0]:
                best = (gain, i)
        if best is None:
            break
        J, S = rows[best[1]]
        A = J @ Cov
        Cov = Cov - A.T @ np.linalg.solve(S + A @ J.T, A)
        chosen.append(best[1])
    return chosen, Cov


PRELOAD_CAM = 6e-4              # rad: last move before each image, every joint, in a designed direction


class CameraCampaign:
    """Images of the datum spheres from poses planned with the probe-fitted kinematics."""

    def __init__(self, camp, p, info, W_cam, verbose=False):
        self.camp, self.rb = camp, camp.rb
        self.verbose = verbose
        mu_c, sd_c = camera_prior()
        mu_v, sd_v = vision_prior()
        self.p = p.copy()
        self.p[CAM], self.p[VIS] = mu_c, mu_v
        self.centres = info["centres"]
        self.Cov = info["Cov"].copy()
        self.Cov[CAM, CAM] = np.diag(sd_c ** 2)
        self.Cov[VIS, VIS] = np.diag(sd_v ** 2)
        self.W = W_cam
        self.records = []

    def say(self, *a):
        if self.verbose:
            print(*a, flush=True)

    def plan(self):
        """Candidate views ranked by the greedy design: the first n to take, the rest spares (for
        views the planner cannot reach safely)."""
        rb = self.rb
        tool, temp = rb.tool, rb.temp.copy()
        rng = np.random.default_rng(20240613)
        cands = view_candidates(self.p, tool, temp, rng)
        self.say("camera: {} candidate views".format(len(cands)))
        cols = np.nonzero(np.diag(self.Cov) > 0)[0]
        rows = view_rows(self.p, cands, temp, tool, cols)
        n = int(min(N_VIEWS, rb.images_left, max(0, (rb.time_left - 20) // VIEW_TIME)))
        chosen, _ = choose_views(cands, rows, self.Cov, self.W, n + 15)
        return [cands[i] for i in chosen], n

    def run(self):
        rb = self.rb
        if rb.damaged or rb.images_left < 1:
            return
        views, n = self.plan()
        pending, spares = list(range(min(n, len(views)))), list(range(min(n, len(views)), len(views)))
        while pending and len(self.records) < n:
            if rb.images_left < 1 or rb.time_left < IMAGE_TIME + 10:
                break
            j = min(pending, key=lambda i: np.abs(views[i]["q"] - rb.q).max())     # a short tour
            pending.remove(j)
            v = views[j]
            qa = v["q"] - v["pre"] * PRELOAD_CAM
            if not self.camp.path_clear(qa, v["q"], 0.01) or not self.camp.travel(qa):
                self.say("  view unreachable")
                if spares:
                    pending.append(spares.pop(0))
                continue
            rb.move(v["q"])
            temp = rb.temp.copy()
            obs = rb.image()
            self.records.append(dict(q=rb.q.copy(), s=rb.s.copy(), temp=temp, tool=rb.tool, image=obs["image"],
                                     ks=v["ks"], clean=despeckle(obs["image"]).astype(np.uint8), n=len(self.records)))
            self.say("  image {} spheres {} (images left {}, time left {:.0f})".format(
                len(self.records), v["ks"], rb.images_left, rb.time_left))

    def measure(self, p, margin_px, prev=None, search_px=70.0):
        out = []
        for n, rec in enumerate(self.records):
            r = dict(rec)
            r["spheres"] = measure_spheres(rec["clean"].astype(float), rec, p, self.centres, margin_px,
                                           prev[n]["spheres"] if prev is not None else None, search_px)
            out.append(r)
        return out

    def measure_all(self, data, free_spheres):
        """Two passes of sphere measurement: searched for around the predictions (probe-fitted arm,
        nominal camera) with wide masks around the other objects, the camera's mounting and
        intrinsics fitted (the arm held at its probe fit; the measurements recomputed at the new
        intrinsics, measurements that disagree set aside); then all measured again from there,
        with masks from the refined poses, and the camera fitted again."""
        p = self.p
        obs = self.measure(p, 30.0)
        p, obs = robust_camera_fit(p, obs, self.centres)
        obs = self.measure(p, 6.0, prev=obs, search_px=15.0)
        self.obs_all = obs                  # before any is set aside: the final stage weighs them all
        p_staged, obs = robust_camera_fit(p, obs, self.centres)
        self.obs = obs
        return p_staged, obs


# ==== which defects: Bayesian model selection over the classes ====================================
SELECT_MASS = 0.99              # the best models up to this posterior mass are refitted and averaged
SELECT_MAX = 4                  # ... at most this many


def linearise(data, obs, p, centres, free_spheres):
    """Whitened residuals r0 and Jacobian J (all NX parameters and the free spheres' centres) of the
    touches and the image measurements at p: the data seen by a linear model around p."""
    groups = stack(data)
    r_t, n_t = residuals(p, data, centres, groups)
    nf = 3 * len(free_spheres)
    J_t = np.zeros((len(data), NX + nf))
    for tl, g in groups.items():
        Jt = tip_jacobian(p, g["q"], g["s"], g["temp"], tl, g["load"], cols=TOUCH_COLS)
        J_t[np.ix_(g["idx"], TOUCH_COLS)] = np.einsum("na,nap->np", n_t[g["idx"]], Jt)
    for j, ks in enumerate(free_spheres):
        sel = np.array([d["k"] == ks for d in data])
        J_t[sel, NX + 3 * j:NX + 3 * j + 3] = -n_t[sel]
    r = [r_t / SIGMA]
    J = [J_t / SIGMA]
    if obs:
        blocks = _image_blocks(obs, centres)
        rl, Jl = _image_res_jac(p, blocks, _camera_groups(blocks), centres, np.arange(NX), free_spheres)
        r += rl
        J += Jl
    return np.concatenate(r), np.concatenate(J)


def candidate_models():
    """Every subset of the five other classes, without or with one of the 18 higher harmonics."""
    others = [c for c in LIB_CLASSES if c != "cyclic"]
    out = []
    for mask in range(2 ** len(others)):
        base = [c for i, c in enumerate(others) if mask >> i & 1]
        out.append((tuple(base), None))
        for jn in CYC_PAIRS:
            out.append((tuple(base) + ("cyclic",), jn))
    return out


def model_evidence(r0, J, x0, free_spheres, models, sigma_c=2e-3):
    """Log evidence (Laplace, linearised at x0) of each model, plus its log prior (classes present
    or not with equal odds; the harmonic's joint and order uniform over the 18 candidates)."""
    G = J.T @ J
    g = J.T @ r0
    rr = float(r0 @ r0)
    nf = 3 * len(free_spheres)
    out = []
    for classes, jn in models:
        mu, sd, act = model_prior(classes, jn)
        a = np.concatenate([np.nonzero(act)[0], NX + np.arange(nf)])
        v = np.concatenate([-x0, np.zeros(nf)])          # inactive: held at zero
        v[:NX][act] = mu[act] - x0[act]                    # active: at the prior mean
        v[NX:] = 0.0                                       # free centres: the fit's value is the prior's
        s2 = np.concatenate([sd[act] ** 2, np.full(nf, sigma_c ** 2)])
        Gv = G @ v
        uu = rr + 2 * g @ v + v @ Gv
        Ju = g[a] + Gv[a]
        A = G[np.ix_(a, a)] + np.diag(1.0 / s2)
        L = np.linalg.cholesky(A)
        y = np.linalg.solve(L, Ju)
        logdet = 2 * np.log(np.diag(L)).sum() + np.log(s2).sum()
        logz = -0.5 * (uu - y @ y) - 0.5 * logdet
        out.append(logz + (-np.log(len(CYC_PAIRS)) if jn is not None else 0.0))
    return np.array(out)


def select_models(data, obs, p_full, centres, free_spheres, log=None):
    """Posterior over the candidate models; the best ones (mass SELECT_MASS, at most SELECT_MAX)."""
    r0, J = linearise(data, obs, p_full, centres, free_spheres)
    # the free centres enter as deviations from the fitted ones: their prior is about those
    models = candidate_models()
    lz = model_evidence(r0, J, p_full, free_spheres, models)
    w = np.exp(lz - lz.max())
    w /= w.sum()
    order = np.argsort(-w)
    best, mass = [], 0.0
    for i in order:
        best.append((models[i], float(w[i]), float(lz[i])))
        mass += w[i]
        if mass >= SELECT_MASS or len(best) >= SELECT_MAX:
            break
    if log is not None:
        log.extend(best)
    return best


# ==== the knocked spheres decided again, with the images =======================================
REVISE_CUT = 6.0                # a sphere measurement further out than this (chi, 3 dof) is set aside in a trial
REVISE_CAP = 36.0               # ... and costs at most this (per degree of freedom) in the comparison


def _revision_trial(good, keep0, obs_all, p0, mu, sd, spheres, centres0, _depth=0):
    """The joint fit of touches and images with the datum spheres `spheres` free; touches and image
    measurements that the fit puts far off set aside and the fit repeated. Starts from the touches
    keep0 and every touch of a sphere set free. Returns (cost, p, centres, info): the truncated
    cost of every touch, the capped cost of every image measurement, the spheres' price."""
    spheres = sorted(spheres)
    c0 = SPHERES.copy()
    c0[spheres] = centres0[spheres]
    ids = {id(d) for d in keep0}
    touches = [d for d in good if id(d) in ids or d["k"] in spheres]
    obs = [o for o in obs_all if o["spheres"]]
    p, centres = p0, c0
    for it in range(2):
        p, centres, r_t, r_i, Cov, lin = fit_joint(touches, obs, p, mu, sd, spheres, centres, iters=6)
        r_all, _ = residuals(p, good, centres)
        z = sphere_residuals(p, obs_all, centres)
        if it == 0:
            zt = np.where(r_all > 0, r_all / (TRIM * SIGMA), -r_all / (TRIM_IN * SIGMA))
            touches = [d for d, v in zip(good, zt) if v <= 1.0]
            obs = [dict(rec, spheres=[m for m, v in zip(rec["spheres"], zz) if v < REVISE_CUT])
                   for rec, zz in zip(obs_all, z)]
            obs = [o for o in obs if o["spheres"]]
    shift = {ks: np.linalg.norm(centres[ks] - SPHERES[ks]) for ks in spheres}
    if any(v >= SPHERE_MAX_SHIFT for v in shift.values()):
        return None
    small = [ks for ks in spheres if shift[ks] <= SPHERE_MIN_SHIFT]
    if small:                    # set free but not displaced: not knocked after all
        if _depth > 0:
            return None
        return _revision_trial(good, keep0, obs_all, p, mu, sd, [k for k in spheres if k not in small], centres,
                               _depth=1)
    cost_t = float(_truncated(r_all).sum())
    cost_i = float(sum(3.0 * min(v * v, REVISE_CAP) for zz in z for v in zz))
    return cost_t + cost_i + SPHERE_PENALTY * len(spheres), p, centres, dict(r_t=r_t, lin=lin, z=z, r_all=r_all,
                                                                            touches=touches, spheres=spheres)


def _revise_from(good, keep0, obs_all, p0, spheres, pair, centres0, log=None, rounds=6):
    """The structure (knocked datum spheres, higher harmonic) decided again on every touch AND every
    image measurement (an image measures a sphere's centre directly: a knocked sphere the touches
    alone could not single out stands out in the images), from the state (spheres, pair). Greedy:
    add, drop or swap one sphere, or change the harmonic, while the cost falls; the spheres to add
    ranked by how far off their touches and image measurements are, the harmonics by their score
    tests. The repeatability is estimated again from the touches of the result (and the search
    repeated once when it moved). Returns spheres, pair, p, centres, the touches kept, the cost."""
    global SIGMA
    log = [] if log is None else log
    ks_all = np.array([d["k"] for d in good])

    def trial(sp, pr, p, keep, c0):
        g = _revision_trial(good, keep, obs_all, p, *probe_model(pr), sp, c0)
        if g is None:
            return None
        return (g[0] + (CYC_PENALTY if pr is not None else 0.0),) + tuple(g[1:])
    for attempt in range(2):
        best = trial(spheres, pair, p0, keep0, centres0)
        if best is None:
            spheres = []
            best = trial(spheres, pair, p0, keep0, centres0)
        for _ in range(rounds):
            cost, p, centres, fi = best
            spheres = list(fi["spheres"])
            badness = {}
            tc = _truncated(fi["r_all"])
            for k in range(len(SPHERES)):
                if k in spheres:
                    continue
                sel = ks_all == k
                img = [v * v for rec, zz in zip(obs_all, fi["z"]) for m, v in zip(rec["spheres"], zz) if m["k"] == k]
                badness[k] = (tc[sel].mean() if sel.any() else 0.0) + (np.mean(img) if img else 0.0)
            adds = sorted(badness, key=badness.get, reverse=True)[:3]
            moves = [(sorted(spheres + [k]), pair) for k in adds if len(spheres) < MAX_FREE]
            moves += [(sorted(k for k in spheres if k != f), pair) for f in spheres]
            moves += [(sorted([k for k in spheres if k != f] + [a]), pair) for f in spheres for a in adds[:2]]
            # the harmonic: none, or the best-scoring candidates on the touches at this fit
            lin = fit(fi["touches"], p, *probe_model(None), free_spheres=spheres, robust="redescend")[4]
            cs = cyclic_scores(fi["touches"], p, centres, lin)
            moves += [(spheres, jn) for jn in sorted(cs, key=cs.get, reverse=True)[:3] if jn != pair]
            if pair is not None:
                moves.append((spheres, None))
            trials = []
            for sp, pr in moves:
                g = trial(sp, pr, p, fi["touches"], centres)
                if g is not None:
                    trials.append((g[0], (g[3]["spheres"], pr), g))
            if not trials:
                break
            c_new, (sp, pr), g = min(trials, key=lambda t: t[0])
            if c_new >= cost - 1.0:
                break
            log.append(("revise", list(sp), pr, round(cost - c_new)))
            spheres, pair, best = list(sp), pr, g
        cost, p, centres, fi = best
        spheres = list(fi["spheres"])
        old = SIGMA
        SIGMA = estimate_sigma(fi["r_t"], fi["lin"])
        log.append(("sigma", round(SIGMA * 1e6, 1)))
        p0, centres0, keep0 = p, centres, fi["touches"]
        if abs(SIGMA / old - 1.0) < 0.1:
            break
    return sorted(spheres), pair, p, centres, fi["touches"], cost


def revise_structure(good, keep0, obs_all, p0, spheres, pair, centres0, log=None):
    """The structure decided again on every touch and every image measurement (see _revise_from),
    searched from the probe fit's structure and from none at all (a structure that went wrong
    early may need several changes at once); the cheaper result, compared at one repeatability,
    is kept. Returns spheres, pair, p, centres, the touches kept."""
    global SIGMA
    log = [] if log is None else log
    sigma0 = SIGMA
    runs = []
    for start in ((list(spheres), pair), ([], None)):
        if runs and start == runs[0][3]:
            continue
        SIGMA = sigma0
        sub = []
        out = _revise_from(good, keep0, obs_all, p0, start[0], start[1], centres0, sub)
        runs.append((out, SIGMA, sub, start))
    SIGMA = min(r[1] for r in runs)
    costs = []
    for (sp, pr, p, c, kept, _cost), sig, sub, start in runs:
        g = _revision_trial(good, kept, obs_all, p, *probe_model(pr), sp, c)
        costs.append(np.inf if g is None else g[0] + (CYC_PENALTY if pr is not None else 0.0))
    i = int(np.argmin(costs))
    (sp, pr, p, c, kept, _cost), sig, sub, start = runs[i]
    SIGMA = sig
    log.extend(sub + [("chosen start", start, [round(v) for v in costs])])
    return sp, pr, p, c, kept


def predict(p, conditions, image_centre_frame=False):
    """Predicted tool tip (N, 3) and camera points (N, k, 3) on the conditions, under p. With
    image_centre_frame (a diagnostic, not the submission): the points taken in the frame whose z
    axis is the ray imaged at the image centre (see image_centre_rotation)."""
    q, s, temp, tool = conditions_arrays(conditions)
    load = gravity_load(q, tool)
    t = tip(p, q, s, temp, tool, load)
    Rc, oc = camera_pose(p, q, s, temp, tool, load)
    pts = np.asarray(conditions["cam_points"], float)
    if image_centre_frame:
        pts = pts @ image_centre_rotation(p[INTR]).T
    return t, np.einsum("nab,kb->nka", Rc, pts) + oc[:, None, :]


def image_centre_rotation(th):
    """The rotation (camera frame <- image-centre frame) taking the optical axis to the ray imaged
    at the image centre (the smallest such rotation)."""
    x, y = undistort_px(np.array(CX_NOMINAL), np.array(CY_NOMINAL), th)
    d = np.array([float(x), float(y), 1.0])
    d /= np.linalg.norm(d)
    ax = np.cross([0.0, 0.0, 1.0], d)
    sn = np.linalg.norm(ax)
    if sn < 1e-15:
        return np.eye(3)
    k = ax / sn
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + sn * K + (1 - d[2]) * K @ K


def final_fit(data, obs, p0, centres0, free_spheres, conditions, mu=None, sd=None, select=True, trim=True):
    """The joint fit of touches and images with the model (mu, sd) (default: every candidate
    defect, 'full'); then, with `select`, the model selection around that fit and the
    evidence-weighted average of the best models' refitted predictions. Returns pred_tip,
    pred_cam and a dict of diagnostics."""
    if mu is None:
        mu, sd = full_prior()
    # touches set aside that the images show to be wrong: with the images in, a touch is checked by
    # much more than the other touches (its leverage drops), so its standardised residual tells
    data = list(data)
    dropped = []
    for _ in range(6 if trim else 1):
        p_base, jinfo = joint_calibration(data, obs, p0, mu, sd, free_spheres, centres0)
        t = jinfo["touch_residuals"] / (SIGMA * np.sqrt(np.maximum(1.0 - jinfo["lev"], 1e-3)))
        z = np.where(t > 0, t / TRIM, -t / TRIM_IN)
        if not trim or not len(z) or z.max() <= 1.0:
            break
        worst = int(np.argmax(z))
        dropped.append(data[worst])
        data = data[:worst] + data[worst + 1:]
        p0 = p_base
    out = dict(p_base=p_base, joint=jinfo, pred_base=predict(p_base, conditions), final_dropped=len(dropped), data=data)
    if not select:
        return out["pred_base"][0], out["pred_base"][1], out
    obs_kept = jinfo["obs"]
    best = select_models(data, obs_kept, p_base, jinfo["centres"], free_spheres)
    tips, cams, cams_ic, ws, fits = [], [], [], [], []
    for (classes, jn), w, lz in best:
        mu_m, sd_m, _ = model_prior(classes, jn)
        p_m, c_m, r_t, r_i, Cov, _l = fit_joint(data, obs_kept, p_base, mu_m, sd_m, free_spheres, jinfo["centres"])
        t, c = predict(p_m, conditions)
        tips.append(t)
        cams.append(c)
        cams_ic.append(predict(p_m, conditions, image_centre_frame=True)[1])
        ws.append(w)
        fits.append(dict(classes=classes, cyc=jn, weight=w, logz=lz, p=p_m))
    ws = np.array(ws) / np.sum(ws)
    pred_tip = np.einsum("m,mna->na", ws, np.array(tips))
    pred_cam = np.einsum("m,mnka->nka", ws, np.array(cams))
    out.update(models=fits, pred_map=(tips[0], cams[0]),
               pred_cam_ic=np.einsum("m,mnka->nka", ws, np.array(cams_ic)))
    return pred_tip, pred_cam, out


def measure_session(cell, conditions, verbose=False):
    """The session: the touch campaign, the robust probe-only fit, the views and the image
    measurements. Returns everything the final fit needs (a dict)."""
    global SIGMA
    import time as _time
    t0 = _time.time()
    SIGMA = SIGMA_PLAN
    W_tip, W_cam = acceptance_weights(conditions)
    camp = Campaign(Robot(cell), W_tip, verbose=verbose)
    try:
        camp.run()
    except RuntimeError as err:          # should not happen; keep what was measured
        camp.say("campaign stopped:", err)
    t1 = _time.time()
    good = [d for d in camp.data if d["ok"]]
    plog = []
    p, info = robust_probe_fit(good, camp.p, plog)
    info["probe_log"] = plog
    info.update(data=camp.data, events=camp.events, probes_left=camp.rb.probes_left,
                time_left_probe=camp.rb.time_left, p_probe=p, sigma=SIGMA)
    t2 = _time.time()
    cam = CameraCampaign(camp, p, info, W_cam, verbose=verbose)
    # the plate's views first (the job needs them; the spheres' views fill what time and images are left)
    plate = pj.PlateViews(camp, cam.p, say=camp.say)
    try:
        plate.run(min(pj.N_PLATE_VIEWS, camp.rb.images_left))
    except RuntimeError as err:          # should not happen; keep what was taken
        camp.say("plate views stopped:", err)
    info["plate_images"] = plate.records
    try:
        cam.run()
    except RuntimeError as err:          # should not happen; keep what was taken
        cam.say("camera campaign stopped:", err)
    info["images"] = cam.records
    info["camera_obs"] = []
    info["p_staged"] = cam.p
    if cam.records:
        info["p_staged"], info["camera_obs"] = cam.measure_all(info["keep"], info["free_spheres"])
        info["camera_obs_all"] = relinearize(cam.obs_all, info["p_staged"][INTR])
    info["time_left"] = camp.rb.time_left
    info["thermal_history"] = list(camp.rb.history)
    info["timing"] = dict(campaign=t1 - t0, probe_fit=t2 - t1, camera=_time.time() - t2)
    return info


def finish(info, conditions, select=True):
    """The final fit and the predictions from a session's measurements."""
    global SIGMA
    import time as _time
    t0 = _time.time()
    SIGMA = info["sigma"]
    # the knocked spheres (and the repeatability) decided again with every touch and every image
    good = [d for d in info["data"] if d["ok"]]
    obs_all = info.get("camera_obs_all", info["camera_obs"])
    log = []
    spheres, pair, p, centres, keep = revise_structure(good, info["keep"], obs_all, info["p_staged"],
                                                       info["free_spheres"], info["probe_model"][1], info["centres"], log)
    info["mu"], info["sd"] = probe_model(pair)
    info["probe_model"] = (NONCYCLIC, pair)
    z = sphere_residuals(p, obs_all, centres)
    obs = [dict(rec, spheres=[m for m, v in zip(rec["spheres"], zz) if v < REVISE_CUT]) for rec, zz in zip(obs_all, z)]
    info.update(probe_free_spheres=info["free_spheres"], free_spheres=spheres, centres=centres, keep=keep,
                camera_obs=obs, p_staged=p, sigma=SIGMA, revise_log=log)
    pred_tip, pred_cam, fin = final_fit(info["keep"], info["camera_obs"], info["p_staged"], info["centres"],
                                        info["free_spheres"], conditions, info["mu"], info["sd"], select=select)
    info.update(fin)
    info.setdefault("timing", {})["final"] = _time.time() - t0
    t1 = _time.time()
    info["thermal"] = thermal_fit(info.get("thermal_history") or [])
    info["job"], info["plate_pose"] = job_program(info)
    info["job_warm"] = None
    warm = conditions.get("job_warm_temp")
    if info["plate_pose"] is not None and warm is not None:
        info["job_warm"], _ = job_program(info, warm=np.asarray(warm, float), plate=info["plate_pose"])
    info["timing"]["job"] = _time.time() - t1
    return pred_tip, pred_cam


def model_kin(p):
    """The pin under the parameter vector p, process_c mounted (plate_job.Kin)."""
    return pj.Kin(lambda q, s, temp: pose(p, q, s, temp, pj.JOB_TOOL))


# ---- the cell's heating and cooling, from the session's temperature readings ---------------------------
def _heat_inputs(d, travel, tau):
    """For cooling time constants tau (m,): per reply k and joint j, the link temperature rise (K) that one K
    per radian of heating by joint j would have caused by then (cell.md, section 7: during a command of
    duration d with travel |dq|, a link tends to T_AMBIENT + tau H |dq| / d and relaxes towards it with tau).
    Returns (m, n, 6)."""
    tau = np.atleast_1d(np.asarray(tau, float))
    e = np.exp(-d[None, :] / tau[:, None])                       # (m, n)
    drive = tau[:, None, None] * (travel / d[:, None])[None] * (1.0 - e)[:, :, None]
    out = np.empty((len(tau), len(d), 6))
    acc = np.zeros((len(tau), 6))
    for k in range(len(d)):
        acc = drive[:, k] + acc * e[:, k, None]
        out[:, k] = acc
    return out


def thermal_fit(history):
    """The unit's cooling time constant and heating (K per radian, link i by joint j: own joint and its
    neighbours, non-negative) fitted to every temperature reading of the session (least squares, the
    session starting with every link at T_AMBIENT): profile over tau, the heating linear given tau.
    Returns dict(tau, heat (6, 6), rms (K), n)."""
    from scipy.optimize import nnls, minimize_scalar
    # one record per reply: the time since the last kept reply, the travel since then, the readings (replies
    # less than half a millisecond after the last kept one are merged into the next)
    d, travel, temp = [], [], []
    t_last, acc = 0.0, np.zeros(6)
    for t, reading, tr in history:
        acc = acc + tr
        if t - t_last > 5e-4:
            d.append(t - t_last)
            travel.append(acc)
            temp.append(reading)
            t_last, acc = t, np.zeros(6)
    if len(d) < 20:
        return None
    d, travel, y = np.array(d), np.array(travel), np.array(temp) - T_AMBIENT

    def solve(tau):
        G = _heat_inputs(d, travel, tau)[0]
        H, cost = np.zeros((6, 6)), 0.0
        for i in range(6):
            cols = [j for j in (i - 1, i, i + 1) if 0 <= j < 6]
            x, r = nnls(G[:, cols], y[:, i])
            H[i, cols] = x
            cost += r ** 2
        return cost, H
    lo, hi = 0.5 * TAU_COOL_RANGE[0], 2.0 * TAU_COOL_RANGE[1]
    grid = np.geomspace(lo, hi, 25)
    costs = [solve(x)[0] for x in grid]
    i = int(np.argmin(costs))
    a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    res = minimize_scalar(lambda x: solve(x)[0], bounds=(a, b), method="bounded", options=dict(xatol=0.5))
    tau = float(res.x)
    cost, H = solve(tau)
    return dict(tau=tau, heat=H, rms=float(np.sqrt(cost / y.size)), n=int(len(d)))


NOMINAL_THERMAL = "nominal"         # job_program(thermal=...): plan with the documented nominal heating and cooling


def job_program(info, p=None, free=(0, 1, 2, 3, 4, 5), use=None, plan_p=None, warm=None, thermal=None, plate=None):
    """The plate located in its images under the model (the most probable model of the selection unless p
    is given; or `plate`, a pose already located) and the job program planned with it (or with plan_p:
    studies): cold, or from `warm` (the six published start temperatures of the warm job); the link
    temperatures along it predicted with the session's thermal fit (`thermal`: default info["thermal"];
    NOMINAL_THERMAL or no fit: the documented nominal figures). Returns (rows or None, plate pose or None)."""
    if p is None:
        p = info["models"][0]["p"] if info.get("models") else info["p_base"]
    log = info.setdefault("job_log", [])
    recs = info.get("plate_images") or []
    if plate is None and not recs:
        log.append("no plate images")
        return None, None
    th = info.get("thermal") if thermal is None else thermal
    heat, tau = (th["heat"], th["tau"]) if isinstance(th, dict) else (None, None)
    try:
        if plate is None:
            plate, pinfo = pj.locate_plate(sys.modules[__name__], p, recs, free=free, log=log, use=use)
            if plate is None:
                return None, None
        planner = pj.Planner(model_kin(p if plan_p is None else plan_p), plate, temp0=warm, heat=heat, tau=tau,
                             log=log)
        rows = planner.plan()
    except (ValueError, np.linalg.LinAlgError, FloatingPointError) as err:
        log.append("job: {}".format(err))
        return None, None
    return rows, plate


def calibrate(cell, conditions, return_info=False, verbose=False, select=True):
    """Run the session on the cell and return the predictions on the unit's conditions:
    pred_tip (N, 3) and pred_cam (N, k, 3). With return_info, also the touch records, the images
    and diagnostics (for tests/review/reference_check.py)."""
    info = measure_session(cell, conditions, verbose=verbose)
    pred_tip, pred_cam = finish(info, conditions, select=select)
    return (pred_tip, pred_cam, info) if return_info else (pred_tip, pred_cam)
