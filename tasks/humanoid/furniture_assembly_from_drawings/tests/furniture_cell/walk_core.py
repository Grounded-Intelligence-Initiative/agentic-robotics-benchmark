"""Walking legs for the furniture cell's G1 robots, drawn only: an online, causal leg animator.

The cell's locomotion abstraction carries each robot's pelvis on four base joints (base_x, base_y, base_yaw, base_z)
and poses its legs from the stand height, so a robot that travels glides over the floor. The cameras draw the legs
walking instead: one `LegWalker` per robot, stepped at the cell's control rate with the simulated time and the robot's
base pose (the four base joints), keeps its own footsteps and returns the 12 leg joint angles to draw. Nothing else
changes: the pelvis, the torso, the arms and the head (with the head camera) stay where the physics has them, and the
physics keeps its posed legs.

The gait (game-style root motion, no foot sliding):
* the base's velocity and yaw rate are estimated from its poses (causal, smoothed over 0.08 s);
* while the base moves, the feet step alternately; each step lands where its stance spot under the hip is predicted
  (from the current velocity) to be half a stance after the landing, so the hip passes over the foot at mid-stance.
  Cadence 1.5-2.2 steps/s rising with speed (a 0.3 m stride at 0.3 m/s), pure turns make short steps on the spot (the
  turning side first), side-steps (up to 2.3 steps/s) widen the stance (the leading foot first), a landing keeps
  0.17 m across and 2.5 cm between the soles from the other foot; a swing's landing spot follows the base until 65 %
  of the swing; when the base stops (0.6 s), settle steps put the feet back into the stance under the hips;
* a stance foot stays where it landed, flat on its support (the floor, or the top of a drawn support such as the
  rug); a swing foot follows a lifted arc (minimum jerk along the ground, sin^2 lift of 3-6.5 cm by step length);
* legs only: the pelvis keeps its height, and the cell's robots stand with nearly straight legs, so a stance foot
  that a leg cannot reach flat rolls over its heel (the foot ahead of the hip) or its toe (behind), that edge staying
  on the support; a pelvis above the straight-leg height leaves the feet hovering as the posed legs do;
* a swing knee bends from the lift-off on (at least 0.05 + 0.35 sin(pi u) rad over the swing's share u), the foot's
  lift-off roll unrolls over the first 40 % of the swing; in a deep squat (where the ankle's dorsiflexion limit would
  tip a lifted flat foot's toe into the floor) a swing foot rolls over its toe;
* each leg's 6 joints and its foot's roll by damped least squares on the robot's own leg chain (copied from the cell
  model at construction; Levenberg-Marquardt damping for a target out of reach), within the joint limits (the knee
  bent at least 0.05 rad): one tracking step per tick from the previous tick's solution, converged by `pose_at`.

`step` is meant to be called every control tick whatever the cameras do (the legs then depend on the motion only);
`pose_at` gives the converged legs at the current time without changing anything. Deterministic (the same base poses
at the same times give the same angles), numpy + mujoco only, no file I/O.
"""

from __future__ import annotations

import bisect
import collections
import math

import mujoco
import numpy as np

SIDES = ("left", "right")
LEG_JOINTS = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
BASE_JOINTS = ("base_x", "base_y", "base_yaw", "base_z")
HEEL, TOE = 1, -1                  # a stance foot's pivot (the edge it rolls over when it cannot be flat)


class WalkParams:
    """The gait's numbers (SI units: m, s, rad)."""

    tick = 0.01                    # the planner's longest internal step (base poses between calls are interpolated)
    gap_reset = 1.0                # a longer gap between calls, time running backwards or a jump of the base ...
    jump_xy, jump_yaw, jump_z = 0.25, 0.6, 0.08     # ... beyond these in one call re-plants the feet (a reset)
    tau_v = 0.08                   # smoothing of the base velocity estimate
    f_min, f_max, v_ref = 1.5, 1.9, 0.30            # cadence (steps/s) from the effective speed (m/s) ...
    f_fast, v_fast = 2.2, 0.50     # ... and on to f_fast at v_fast
    f_lat, tsw_lat = 2.3, 0.30     # side-stepping: higher cadence, shorter swing ...
    widen_max = 0.06               # ... and a stance widened by up to this per foot, so a closing step keeps w_min
    f_turn = 2.2                   # turning on the spot: higher cadence (smaller foot rotation per step)
    r_turn = 0.25                  # a yaw rate counts as speed r_turn * |w| for the cadence
    sw_frac = 0.74                 # swing share of a step period
    tsw_min, tds_min = 0.34, 0.10  # least swing and double-support times
    v_on, w_on = 0.02, 0.05        # the base counts as moving (it stops counting below half of these) ...
    motion_window = 0.2            # ... by its travel over this long
    d_walk, d_settle = 0.030, 0.020                 # step thresholds (footstep deviation) while moving / when still
    d_creep = 0.06                 # a stance foot this far from its spot under the hip steps even when not moving
    d_strain = 0.14                # ... and this far steps next, whichever foot stepped last
    k_yaw = 0.15                   # m per rad in the footstep deviation
    settle_wait = 0.6              # still this long before settle steps
    restart_s = 1.2                # without a step this long, the next first foot is chosen by need
    w_min, c_min = 0.17, 0.025     # lateral foot distance (pelvis frame) and sole clearance of a landing
    h_min, h_max, h_base, h_gain = 0.030, 0.065, 0.020, 0.20       # swing lift from the step length
    u_freeze = 0.65                # a swing's landing spot follows the base until this share of the swing ...
    retarget_v, retarget_w, retarget_z = 0.6, 1.5, 0.2   # ... moving at most this fast (m/s, rad/s, m/s up / down)
    pred_max, pred_yaw_max = 0.30, 0.8              # predicted base travel is clipped to this
    reach_fwd, reach_back = 0.17, 0.17              # a landing at most this far ahead of / behind its hip's spot
    reach_in, reach_out = 0.10, 0.10                # ... and this far inwards / outwards of it
    yaw_rel_max = 0.6              # foot yaw relative to the pelvis at a landing
    knee_min = 0.05                # least knee bend of the drawn legs
    theta_max = 0.6                # largest roll of a foot over its heel or toe
    unroll = 0.4                   # share of a swing over which the lift-off roll unrolls
    knee_swing = 0.35              # a swing knee bends at least knee_min + knee_swing * sin(pi u) (u: swing share)
    w_rot, w_theta = 0.2, 0.003    # IK weights: rotation rows (m per rad), the roll drawn to 0
    ik_iters, ik_iters_eval = 4, 8                  # residual checks at most per tick / per pose_at and reset
    ik_tol, ik_tol_eval = 2e-5, 1e-7                # m: residual counted as converged per tick / per pose_at
    ik_track = 2e-2                # m: below this residual one step per tick tracks the moving targets
    ik_step = 0.4                  # largest joint change per IK iteration
    ik_lm = 0.05                   # IK damping per squared residual (m^2)
    pivot_band = 0.005             # hysteresis of the heel / toe choice (foot position against the hip)
    rug_along, rug_across = 0.05, 0.08              # a landing straddling a support's edge moves this far at most
    still_tol = 1e-5               # base change below which an idle robot's legs are not solved again


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _minjerk(u: float) -> float:
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _hull(points) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2D points."""
    pts = sorted(set((round(float(x), 9), round(float(y), 9)) for x, y in points))
    if len(pts) < 3:
        return np.array(pts, float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return np.array(lo[:-1] + hi[:-1], float)


def support_from_geom(m: mujoco.MjModel, geom: int):
    """A flat support (its top's outline as a convex polygon, counter-clockwise, and its height) from a box or mesh geom
    of a static body: e.g. a drawn rug, so that feet stand on it rather than in it. The geom's pose is the model's
    default (qpos0) kinematics."""
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    R = d.geom_xmat[geom].reshape(3, 3)
    pos = d.geom_xpos[geom]
    if m.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(m.geom_dataid[geom])
        a, n = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
        V = m.mesh_vert[a:a + n].astype(np.float64) @ R.T + pos
    elif m.geom_type[geom] == mujoco.mjtGeom.mjGEOM_BOX:
        sx, sy, sz = m.geom_size[geom]
        V = np.array([(i * sx, j * sy, k * sz) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) @ R.T + pos
    else:
        raise ValueError("a support must be a box or a mesh geom")
    top = float(V[:, 2].max())
    upper = V[V[:, 2] > top - 0.5 * (top - float(V[:, 2].min())) - 1e-9]
    return _hull(upper[:, :2]), top


def _leg_model(m: mujoco.MjModel, prefix: str) -> mujoco.MjModel:
    """The robot's two leg chains in its pelvis frame (the pelvis is this model's world): the cell model's own body
    frames, joint axes and ranges."""
    B = mujoco.mjtObj.mjOBJ_BODY
    s = mujoco.MjSpec()
    s.modelname = "legs"
    s.compiler.degree = False                           # the ranges below are the compiled model's radians
    s.option.gravity = [0.0, 0.0, 0.0]
    for side in SIDES:
        parent = s.worldbody
        for w in LEG_JOINTS:
            bid = mujoco.mj_name2id(m, B, f"{prefix}{side}_{w}_link")
            if bid < 0:
                raise KeyError(f"{prefix}{side}_{w}_link")
            b = parent.add_body(name=f"{side}_{w}_link", pos=m.body_pos[bid].tolist(), quat=m.body_quat[bid].tolist())
            b.explicitinertial = True
            b.mass = 1.0
            b.inertia = [1e-3, 1e-3, 1e-3]
            jid = int(m.body_jntadr[bid])
            b.add_joint(name=f"{side}_{w}_joint", type=mujoco.mjtJoint.mjJNT_HINGE, axis=m.jnt_axis[jid].tolist(),
                        pos=m.jnt_pos[jid].tolist(), range=m.jnt_range[jid].tolist(),
                        limited=mujoco.mjtLimited.mjLIMITED_TRUE)
            parent = b
    return s.compile()


class _Supports:
    """Support height under a sole: the floor (z = 0) or the top of a flat support (convex polygon, height)."""

    def __init__(self, supports):
        self.polys = []
        for poly, top in supports or ():
            P = np.asarray(poly, np.float64).reshape(-1, 2)
            if len(P) >= 3:
                # edge normals (outward) and offsets of the counter-clockwise polygon
                E = np.roll(P, -1, axis=0) - P
                N = np.stack([E[:, 1], -E[:, 0]], axis=1)
                N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
                self.polys.append((N, np.sum(N * P, axis=1), float(top)))

    def top_of(self, pts: np.ndarray) -> np.ndarray:
        """Support height at each 2D point."""
        z = np.zeros(len(pts))
        for N, c, top in self.polys:
            inside = np.all(pts @ N.T <= c + 1e-9, axis=1)
            z = np.where(inside, np.maximum(z, top), z)
        return z

    def under(self, pts: np.ndarray):
        """(the height a rigid flat sole rests at: the highest support under any of its points, whether the sole is
        fully on that support)."""
        if not self.polys:
            return 0.0, True
        z = self.top_of(pts)
        top = float(z.max())
        return top, bool(np.all(z == top))

    def uniform(self, pts: np.ndarray) -> np.ndarray:
        """For point sets (n, k, 2): whether each lies on one support only (fully on or fully off)."""
        if not self.polys:
            return np.ones(pts.shape[0], bool)
        z = self.top_of(pts.reshape(-1, 2)).reshape(pts.shape[:2])
        return np.all(z == z[:, :1], axis=1)


class LegWalker:
    """One robot's walking legs (drawn only). `step(t, base)` every control tick (base = the robot's base_x, base_y,
    base_yaw, base_z joint values at sim time t) returns the 12 leg joint angles to draw: left then right, each
    (hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll). `qpos_adr` are those joints' qpos addresses in the
    cell model, `base_adr` the base joints'."""

    def __init__(self, m: mujoco.MjModel, prefix: str, *, supports=(), params: WalkParams | None = None):
        P = self.P = params or WalkParams()
        J, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
        self.prefix = prefix
        self.qpos_adr = np.array([int(m.jnt_qposadr[mujoco.mj_name2id(m, J, f"{prefix}{s}_{w}_joint")])
                                  for s in SIDES for w in LEG_JOINTS])
        self.base_adr = np.array([int(m.jnt_qposadr[mujoco.mj_name2id(m, J, prefix + n)]) for n in BASE_JOINTS])
        pelvis = mujoco.mj_name2id(m, B, prefix + "pelvis")
        if pelvis < 0:
            raise KeyError(prefix + "pelvis")
        self.z0 = float(m.body_pos[pelvis][2])          # pelvis height at base_z = 0 (world-aligned base joints)
        self.lm = _leg_model(m, prefix)
        self.ld = mujoco.MjData(self.lm)
        lm = self.lm
        self.foot_body = [mujoco.mj_name2id(lm, B, f"{s}_ankle_roll_link") for s in SIDES]
        hips = [mujoco.mj_name2id(lm, B, f"{s}_hip_pitch_link") for s in SIDES]
        self.hip_xy = [(float(lm.body_pos[b][0]), float(lm.body_pos[b][1])) for b in hips]   # base frame
        lo = lm.jnt_range[:, 0] + 1e-4
        hi = lm.jnt_range[:, 1] - 1e-4
        lo[[3, 9]] = np.maximum(lo[[3, 9]], P.knee_min)
        self.lo, self.hi = lo, hi
        self.supports = _Supports(supports)
        da, dc = np.meshgrid(np.arange(-P.rug_along, P.rug_along + 1e-9, 0.01),
                             np.arange(-P.rug_across, P.rug_across + 1e-9, 0.01), indexing="ij")
        cost = 2.0 * np.abs(da) + np.abs(dc)
        order = np.argsort(cost.ravel(), kind="stable")
        order = order[cost.ravel()[order] > 0.0]
        self._shift_grid = (da.ravel()[order], dc.ravel()[order], cost.ravel()[order])
        self._sole(m, prefix)
        self._stance_table()
        self.jacp = np.zeros((2, 3, lm.nv))
        self.jacr = np.zeros((2, 3, lm.nv))
        self._J7 = np.zeros((2, 7, 7))
        self._lo7 = np.concatenate([self.lo.reshape(2, 6), np.zeros((2, 1))], axis=1)
        self._hi7 = np.concatenate([self.hi.reshape(2, 6), np.full((2, 1), P.theta_max)], axis=1)
        self._eye7 = np.broadcast_to(np.eye(7), (2, 7, 7)).copy()
        self._eye3 = np.broadcast_to(np.eye(3), (2, 3, 3)).copy()
        self._e7 = np.zeros((2, 7))
        self._J7[:, 6, 6] = P.w_theta
        self._fixed = np.zeros((2, 7))
        self._X = np.zeros((2, 7))
        self.theta = np.zeros(2)
        self.t = None
        self.reset_count = 0

    # ------------------------------------------------------------------------------------------- the model's numbers
    def _sole(self, m, prefix):
        """The sole in the ankle_roll_link frame, from the foot's drawn mesh (its lowest 1 mm): heel and toe x,
        half width, centre y, sole plane z."""
        B = mujoco.mjtObj.mjOBJ_BODY
        b = mujoco.mj_name2id(m, B, f"{prefix}left_ankle_roll_link")
        geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        if geoms:
            g = geoms[0]
            mid = int(m.geom_dataid[g])
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(np.float64)
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            V = V @ R.reshape(3, 3).T + m.geom_pos[g]
            band = V[V[:, 2] < V[:, 2].min() + 0.001]
            self.x_heel, self.x_toe = float(band[:, 0].min()), float(band[:, 0].max())
            self.half_w = float(0.5 * (band[:, 1].max() - band[:, 1].min()))
        else:                                           # Menagerie's G1 foot
            self.x_heel, self.x_toe, self.half_w = -0.0658, 0.1424, 0.0378
        # sample points of each sole (foot frame x, y; the right sole mirrors the left's y)
        xs = np.linspace(self.x_heel, self.x_toe, 4)
        ys = np.linspace(-self.half_w, self.half_w, 3)
        self.sole_pts = np.array([(x, y) for x in xs for y in ys])
        self._rect_xy = ((self.x_heel, -self.half_w), (self.x_toe, -self.half_w), (self.x_toe, self.half_w),
                         (self.x_heel, self.half_w))

    def _fk_ankles(self, q6: np.ndarray):
        """Ankle-roll frames of both legs (pelvis frame) for the same 6 angles on both legs (a symmetric pose)."""
        ld = self.ld
        ld.qpos[:6] = q6
        ld.qpos[6:] = q6
        mujoco.mj_kinematics(self.lm, ld)
        return [ld.xpos[b].copy() for b in self.foot_body], [ld.xmat[b].reshape(3, 3).copy() for b in self.foot_body]

    def _stance_table(self):
        """The stance under the hips per stand height (the cell's posed squat: hip pitch -a, knee 2a, ankle pitch -a):
        the ankle-roll origin in the base frame, and the angles. Beyond the ankle's dorsiflexion limit a flat foot sits
        further forward (ankle at its limit, hip pitch th, knee lim - th). Above the straight-leg height the posed feet
        hover."""
        P = self.P
        p0, _ = self._fk_ankles(np.zeros(6))
        self.ankle_h0 = self.z0 + float(p0[0][2])      # ankle-roll origin above the support in the posed stance
        lim = -float(self.lo[4]) - 0.01                 # ankle dorsiflexion used at most in the stance
        a_min = 0.5 * P.knee_min
        rows = []
        for a in np.concatenate([[a_min], np.linspace(0.04, 1.25, 122)]):
            if a <= lim:
                q = np.array([-a, 0.0, 0.0, 2 * a, -a, 0.0])
            else:
                lo_, hi_ = -2.0, -lim                  # hip pitch th: deeper squat <-> more hip flexion
                pa, _ = self._fk_ankles(np.array([-a, 0.0, 0.0, 2 * a, -a, 0.0]))
                z_t = float(pa[0][2])
                for _ in range(50):
                    th = 0.5 * (lo_ + hi_)
                    pp, _ = self._fk_ankles(np.array([th, 0.0, 0.0, lim - th, -lim, 0.0]))
                    if pp[0][2] > z_t:
                        lo_ = th
                    else:
                        hi_ = th
                q = np.array([th, 0.0, 0.0, lim - th, -lim, 0.0])
            pa, _ = self._fk_ankles(q)
            h = self.ankle_h0 - float(pa[0][2])
            rows.append((h, float(pa[0][0]), float(pa[0][1]), float(pa[1][1]), q))
        rows.sort(key=lambda r: r[0])
        self.tab_h = [r[0] for r in rows]
        self.tab_ax = [r[1] for r in rows]
        self.tab_ay = {"left": [r[2] for r in rows], "right": [r[3] for r in rows]}
        self.tab_q = np.array([r[4] for r in rows])
        self.h_top = self.tab_h[-1]                     # highest pelvis with the feet flat on the support
        # below this pelvis height (above a lifted foot) the posed squat's ankle passes its dorsiflexion limit
        self.h_squat = min(r[0] for r in rows if r[4][4] > -lim + 1e-9)

    def _interp(self, arr, h):
        hs = self.tab_h
        if h <= hs[0]:
            return arr[0]
        if h >= hs[-1]:
            return arr[-1]
        i = bisect.bisect_right(hs, h) - 1
        u = (h - hs[i]) / (hs[i + 1] - hs[i])
        return arr[i] + (arr[i + 1] - arr[i]) * u

    def home(self, side: str, h: float):
        """The stance spot of a foot under its hip (ankle-roll origin, base frame x, y) for pelvis height h above the
        support."""
        return self._interp(self.tab_ax, h), self._interp(self.tab_ay[side], h)

    def posed(self, h: float) -> np.ndarray:
        """The stance's 12 angles for pelvis height h (both legs alike)."""
        q6 = self._interp(self.tab_q, h)
        return np.concatenate([q6, q6])

    # ------------------------------------------------------------------------------------------- public
    def reset(self, t: float, base) -> np.ndarray:
        """Both feet planted in the stance under the hips (a start or a teleport)."""
        x, y, yaw, bz = (float(v) for v in base)
        h = self.z0 + bz
        self.t = float(t)
        self.base = (x, y, yaw, bz)
        self.v = [0.0, 0.0]
        self.hist = collections.deque([(self.t, x, y, yaw)])
        self.w = 0.0
        self.moving = False
        self.still_since = self.t
        self.swing = None
        self.last_land = self.t - 10.0
        self.last_side = None
        self.plant = {}
        for s in SIDES:
            self.plant[s] = self._settled(self._home_world(s, x, y, yaw, h))
        self.pivot = {s: HEEL for s in SIDES}
        self.q = self.posed(h - self.plant["left"][2])
        self.solved_for = None
        self.converged = False
        self._fixed[:] = 0.0
        self.theta = np.zeros(2)
        self.reset_count += 1
        self._solve(self.t, self.base, self.P.ik_iters_eval, commit=True)
        return self.q.copy()

    def step(self, t: float, base) -> np.ndarray:
        """Advance to sim time t with the base pose `base` (base_x, base_y, base_yaw, base_z); returns the 12 angles
        (one tracking step of the IK). A call with the time running backwards, after a gap of more than gap_reset or
        with the base jumped away re-plants the feet (a reset); base poses between calls are interpolated."""
        P = self.P
        t = float(t)
        x, y, yaw, bz = (float(v) for v in base)
        if not all(math.isfinite(v) for v in (t, x, y, yaw, bz)):
            raise ValueError("non-finite time or base pose")
        if self.t is None:
            return self.reset(t, (x, y, yaw, bz))
        dt = t - self.t
        x0, y0, yaw0, bz0 = self.base
        dyaw = _wrap(yaw - yaw0)
        if dt < -1e-9 or dt > P.gap_reset or math.hypot(x - x0, y - y0) > P.jump_xy + 0.6 * max(dt, 0.0) \
                or abs(dyaw) > P.jump_yaw + 1.0 * max(dt, 0.0) or abs(bz - bz0) > P.jump_z + 0.2 * max(dt, 0.0):
            return self.reset(t, (x, y, yaw, bz))
        if dt > 1e-12:
            t_prev = self.t
            n = max(1, int(math.ceil(dt / P.tick - 1e-9)))
            for i in range(1, n + 1):
                u = i / n
                self._advance(t_prev + dt * u if i < n else t,
                              (x0 + (x - x0) * u, y0 + (y - y0) * u, yaw0 + dyaw * u, bz0 + (bz - bz0) * u), dt / n)
            self.t = t
        self.base = (x, y, yaw, bz)
        self._solve(t, self.base, P.ik_iters, commit=True)
        return self.q.copy()

    def pose_at(self, t: float, base) -> np.ndarray:
        """The 12 angles at a time between two steps (t not before the last step, base the pose then), with the
        footsteps as of the last step; nothing changes."""
        return self._solve(float(t), tuple(float(v) for v in base), self.P.ik_iters_eval, commit=False)

    def feet(self, t: float | None = None) -> dict:
        """Per side: whether the foot stands, and its target pose on the ground (x, y, support z, yaw)."""
        t = self.t if t is None else float(t)
        out = {}
        for s in SIDES:
            sw = self.swing
            if sw is not None and sw["side"] == s and t < sw["t1"]:
                pose, u = self._swing_pose(t)
                out[s] = {"stance": False, "pose": pose, "u": u}
            else:
                pose = sw["P1"] if sw is not None and sw["side"] == s else self.plant[s]   # landed by then
                out[s] = {"stance": True, "pose": tuple(pose), "u": -1.0}
        return out

    # ------------------------------------------------------------------------------------------- planner
    def _home_world(self, side, x, y, yaw, h, widen=0.0):
        ax, ay = self.home(side, h)
        ay += widen if side == "left" else -widen
        c, s = math.cos(yaw), math.sin(yaw)
        return [x + c * ax - s * ay, y + s * ax + c * ay, 0.0, yaw]

    def _sole_world(self, pose, side):
        c, s = math.cos(pose[3]), math.sin(pose[3])
        L = self.sole_pts if side == "left" else self.sole_pts * np.array([1.0, -1.0])
        return np.stack([pose[0] + c * L[:, 0] - s * L[:, 1], pose[1] + s * L[:, 0] + c * L[:, 1]], axis=1)

    def _settled(self, pose, side=None):
        """The pose with its support height."""
        z, _ = self.supports.under(self._sole_world(pose, side or "left"))
        return [pose[0], pose[1], z, pose[3]]

    def _timing(self):
        P = self.P
        x, y, yaw, _ = self.base
        v = math.hypot(self.v[0], self.v[1])
        c, s = math.cos(yaw), math.sin(yaw)
        vlat = abs(-s * self.v[0] + c * self.v[1])
        wt = P.r_turn * abs(self.w)
        veff = math.hypot(v, wt)
        lat = min(1.0, vlat / (v + 0.02))
        turn = min(1.0, wt / (v + wt + 0.02))
        fmax = max(P.f_max + (P.f_lat - P.f_max) * lat, P.f_max + (P.f_turn - P.f_max) * turn)
        if veff <= P.v_ref:
            f = P.f_min + (fmax - P.f_min) * veff / P.v_ref
        else:
            f = fmax + (max(P.f_fast, fmax) - fmax) * min(1.0, (veff - P.v_ref) / (P.v_fast - P.v_ref))
        tstep = 1.0 / f
        tsw = max(P.tsw_min + (P.tsw_lat - P.tsw_min) * lat, P.sw_frac * tstep)
        tds = max(P.tds_min, tstep - tsw)
        return tsw, tds, tsw + 2.0 * tds, lat

    def _predict(self, horizon):
        """The base pose `horizon` seconds ahead at the current velocity and yaw rate (clipped)."""
        P = self.P
        x, y, yaw, bz = self.base
        dyaw = max(-P.pred_yaw_max, min(P.pred_yaw_max, self.w * horizon))
        c, s = math.cos(0.5 * dyaw), math.sin(0.5 * dyaw)
        dx = (c * self.v[0] - s * self.v[1]) * horizon
        dy = (s * self.v[0] + c * self.v[1]) * horizon
        n = math.hypot(dx, dy)
        if n > P.pred_max:
            dx, dy = dx * P.pred_max / n, dy * P.pred_max / n
        return x + dx, y + dy, yaw + dyaw, bz

    def _dev(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) + self.P.k_yaw * abs(_wrap(a[3] - b[3]))

    def _target(self, side, t_land, horizon, lat, tstep):
        """The landing pose of `side`: its stance spot under the hip at the predicted base pose, within reach of the hip
        at the landing, apart from the other foot, fully on or off a support."""
        P = self.P
        other = self.plant["right" if side == "left" else "left"]
        xp, yp, yawp, bz = self._predict(horizon)
        h = self.z0 + bz - other[2]
        # side-stepping: both feet land wider, so that the closing step still keeps w_min to the other foot
        vlat = -math.sin(yawp) * self.v[0] + math.cos(yawp) * self.v[1]
        w0 = abs(self.home("left", h)[1] - self.home("right", h)[1])
        widen = min(P.widen_max, max(0.0, (P.w_min + 0.01 - w0 + abs(vlat) * tstep) / 2.0)) if lat > 0.0 else 0.0
        tgt = self._home_world(side, xp, yp, yawp, h, widen)
        # within reach of the hip at the landing
        xl, yl, yawl, _ = self._predict(t_land - self.t if self.t is not None else 0.0)
        hx, hy = self._home_world(side, xl, yl, yawl, h)[:2]
        c, s = math.cos(yawl), math.sin(yawl)
        dx, dy = tgt[0] - hx, tgt[1] - hy
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        sg = 1.0 if side == "left" else -1.0
        fx = max(-P.reach_back, min(P.reach_fwd, fx))
        fy = sg * max(-P.reach_in, min(P.reach_out, sg * fy))
        tgt[0], tgt[1] = hx + c * fx - s * fy, hy + s * fx + c * fy
        dyl = _wrap(tgt[3] - yawl)
        if abs(dyl) > P.yaw_rel_max:
            tgt[3] = yawl + math.copysign(P.yaw_rel_max, dyl)
        if abs(_wrap(tgt[3] - other[3])) > 2.0 * P.yaw_rel_max:
            tgt[3] = other[3] + math.copysign(2.0 * P.yaw_rel_max, _wrap(tgt[3] - other[3]))
        tgt = self._apart(tgt, other, yawp, side)
        return self._on_support(tgt, other, yawp, side)

    def _apart(self, p, other, yaw, side):
        P = self.P
        nx, ny = -math.sin(yaw), math.cos(yaw)
        sg = 1.0 if side == "left" else -1.0
        lat = ((p[0] - other[0]) * nx + (p[1] - other[1]) * ny) * sg
        if lat < P.w_min:
            p[0] += nx * sg * (P.w_min - lat)
            p[1] += ny * sg * (P.w_min - lat)
        for _ in range(12):
            c = self._clearance(p, other, side)
            if c >= P.c_min:
                break
            p[0] += nx * sg * (P.c_min - c + 0.002)
            p[1] += ny * sg * (P.c_min - c + 0.002)
        return p

    def _corners(self, pose, side):
        c, s = math.cos(pose[3]), math.sin(pose[3])
        sg = 1.0 if side == "left" else -1.0
        return [(pose[0] + c * x - s * y * sg, pose[1] + s * x + c * y * sg) for x, y in self._rect_xy]

    def _clearance(self, p, other, side):
        """Gap between the two soles' rectangles along the best separating axis (a lower bound of their distance;
        negative when they overlap)."""
        A = self._corners(p, side)
        Bc = self._corners(other, "right" if side == "left" else "left")
        best = -1e9
        for yaw in (p[3], other[3]):
            for ax, ay in ((math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))):
                pa = [ax * u + ay * v for u, v in A]
                pb = [ax * u + ay * v for u, v in Bc]
                best = max(best, min(pb) - max(pa), min(pa) - max(pb))
        return best

    def _on_support(self, p, other, yaw, side):
        """Move a landing that straddles a support's edge so its sole is fully on or off it (the cheapest shift within
        rug_along along the step and rug_across across it, along counting double), keeping it apart from the other
        foot; unchanged when none works (the sole then rests on the support's top, its edge under the sole)."""
        P = self.P
        if not self.supports.polys:
            return [p[0], p[1], 0.0, p[3]]
        z, full = self.supports.under(self._sole_world(p, side))
        if full:
            return [p[0], p[1], z, p[3]]
        dx, dy = p[0] - self.plant[side][0], p[1] - self.plant[side][1]
        n = math.hypot(dx, dy)
        ux, uy = (dx / n, dy / n) if n > 0.02 else (math.cos(p[3]), math.sin(p[3]))
        da, dc, cost = self._shift_grid
        cx = p[0] + da * ux - dc * uy
        cy = p[1] + da * uy + dc * ux
        c, s = math.cos(p[3]), math.sin(p[3])
        L = self.sole_pts if side == "left" else self.sole_pts * np.array([1.0, -1.0])
        pts = np.stack([cx[:, None] + c * L[None, :, 0] - s * L[None, :, 1],
                        cy[:, None] + s * L[None, :, 0] + c * L[None, :, 1]], axis=2)
        ok = self.supports.uniform(pts)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        sg = 1.0 if side == "left" else -1.0
        ok &= ((cx - other[0]) * nx + (cy - other[1]) * ny) * sg >= P.w_min - 1e-6
        for i in np.flatnonzero(ok):                   # the grid is sorted by cost
            q = [float(cx[i]), float(cy[i]), 0.0, p[3]]
            if self._clearance(q, other, side) >= P.c_min - 1e-6:
                q[2] = self.supports.under(pts[i])[0]
                return q
        return [p[0], p[1], z, p[3]]

    def _advance(self, t, base, dt):
        """One planner tick at time t (base pose there, dt since the previous tick)."""
        P = self.P
        x, y, yaw, bz = base
        x0, y0, yaw0, _ = self.base
        if dt > 1e-12:
            a = 1.0 - math.exp(-dt / P.tau_v)
            self.v[0] += a * ((x - x0) / dt - self.v[0])
            self.v[1] += a * ((y - y0) / dt - self.v[1])
            self.w += a * (_wrap(yaw - yaw0) / dt - self.w)
        self.base = (x, y, yaw, bz)
        self.t = t
        # moving: the base's travel over the last motion_window s (jitter averages out over the window)
        hist = self.hist
        hist.append((t, x, y, yaw))
        while len(hist) > 2 and hist[1][0] <= t - P.motion_window + 1e-9:
            hist.popleft()
        th, xh, yh, yawh = hist[0]
        span = t - th
        if span > 1e-9:
            v = math.hypot(x - xh, y - yh) / span
            wv = abs(_wrap(yaw - yawh)) / span
            if v > P.v_on or wv > P.w_on:
                self.moving = True
            elif self.moving and v < 0.5 * P.v_on and wv < 0.5 * P.w_on:
                self.moving = False
                self.still_since = t
        tsw, tds, tst, lat = self._timing()
        sw = self.swing
        if sw is not None:
            if t >= sw["t1"] - 1e-9:
                self.plant[sw["side"]] = list(sw["P1"])
                self.pivot[sw["side"]] = sw["c1"]
                self.last_land, self.last_side = sw["t1"], sw["side"]
                self.swing = None
            else:
                u = (t - sw["t0"]) / (sw["t1"] - sw["t0"])
                if u < P.u_freeze and self.moving:
                    tgt = self._target(sw["side"], sw["t1"], (sw["t1"] - t) + 0.5 * tst, lat, sw["t1"] - sw["t0"] + tds)
                    p1 = sw["P1"]
                    dx, dy = tgt[0] - p1[0], tgt[1] - p1[1]
                    n = math.hypot(dx, dy)
                    lim = P.retarget_v * dt
                    if n > lim:
                        dx, dy = dx * lim / n, dy * lim / n
                    dyaw = _wrap(tgt[3] - p1[3])
                    dyaw = max(-P.retarget_w * dt, min(P.retarget_w * dt, dyaw))
                    dz = max(-P.retarget_z * dt, min(P.retarget_z * dt, tgt[2] - p1[2]))
                    z1 = p1[2] + dz
                    sw["P1"] = [p1[0] + dx, p1[1] + dy, z1, p1[3] + dyaw]
                return
        h = self.z0 + bz
        # a stance foot the base has left far from its spot under the hip steps next, without the double support
        strain = {s: self._dev(self.plant[s], self._home_world(s, x, y, yaw, h - self.plant[s][2])) for s in SIDES}
        far = max(strain.values())
        urgent = max(SIDES, key=lambda s: strain[s]) if far > P.d_strain else None
        if t - self.last_land < (tds if urgent is None else min(tds, P.tds_min * 0.5)):
            return
        if self.moving:
            thr, horizon = P.d_walk, tsw + 0.5 * tst
        else:
            # not moving: settle steps once still for a while, or a step for a stance foot left far behind (creeping)
            if t - self.still_since >= P.settle_wait:
                thr = P.d_settle
            elif far > P.d_creep:
                thr = P.d_creep
            else:
                return
            horizon = 0.0
        xp, yp, yawp, _ = self._predict(horizon)
        dev = {}
        for s in SIDES:
            dev[s] = self._dev(self.plant[s], self._home_world(s, xp, yp, yawp, h - self.plant[s][2]))
        restart = self.last_side is None or (t - self.last_land) > P.restart_s
        pick = None
        if restart:
            need = [s for s in SIDES if dev[s] > thr]
            if need:
                def score(s):
                    tg = self._home_world(s, xp, yp, yawp, h - self.plant[s][2])
                    sg = 1.0 if s == "left" else -1.0
                    nx, ny = -math.sin(yawp) * sg, math.cos(yawp) * sg
                    out = (tg[0] - self.plant[s][0]) * nx + (tg[1] - self.plant[s][1]) * ny
                    return dev[s] + 0.5 * max(0.0, out) + 0.05 * max(0.0, sg * _wrap(tg[3] - self.plant[s][3]))
                pick = max(need, key=lambda s: (score(s), s == "left"))
        else:
            o = "right" if self.last_side == "left" else "left"
            if dev[o] > thr:
                pick = o
            elif dev[self.last_side] > max(thr, 2.0 * P.d_walk):
                pick = self.last_side
        if urgent is not None and dev[urgent] > thr:
            pick = urgent
        if pick is None:
            return
        tgt = self._target(pick, t + tsw, horizon, lat, tsw + tds)
        if not self.moving and self._dev(self.plant[pick], tgt) <= 0.5 * thr:
            return                                      # the apart / support rules leave nothing worth a step
        D = math.hypot(tgt[0] - self.plant[pick][0], tgt[1] - self.plant[pick][1])
        lift = min(P.h_max, max(P.h_min, P.h_base + P.h_gain * D))
        self.swing = {"side": pick, "t0": t, "t1": t + tsw, "P0": list(self.plant[pick]), "P1": tgt, "h": lift,
                      "c0": self.pivot[pick], "c1": self.pivot[pick], "th0": float(self.theta[SIDES.index(pick)])}
        sw = self.swing
        # a deep squat: the swing foot rolls over its toe throughout (decided once, at the lift-off)
        sw["squat"] = bool(h - max(sw["P0"][2], tgt[2]) - lift < self.h_squat and
                           (sw["c0"] == TOE or sw["th0"] < 0.005))

    # ------------------------------------------------------------------------------------------- foot targets, IK
    def _swing_pose(self, t):
        sw = self.swing
        u = min(1.0, max(0.0, (t - sw["t0"]) / (sw["t1"] - sw["t0"])))
        s = _minjerk(u)
        P0, P1 = sw["P0"], sw["P1"]
        pose = (P0[0] + (P1[0] - P0[0]) * s, P0[1] + (P1[1] - P0[1]) * s,
                P0[2] + (P1[2] - P0[2]) * s + sw["h"] * math.sin(math.pi * u) ** 2,
                P0[3] + _wrap(P1[3] - P0[3]) * s)
        return pose, u

    def _targets(self, t, base):
        """Per leg, in the pelvis frame: the flat foot (ankle-roll origin, yaw), the edge it may roll over (a point in
        the foot frame and the roll axis' sign: +1 the toe, the heel lifting; -1 the heel, the toe lifting), the pivot
        chosen, the roll's bounds and the knee's lower bound. A stance foot rolls over the edge towards its hip; a swing
        foot keeps its lift-off edge for the first half of the swing (its lift-off roll unrolling, the knee bending
        from the start: a straight leg would have to snap) and its landing edge for the second; in a deep squat (the
        ankle's dorsiflexion limit would tip a lifted flat foot's toe into the floor) a swing foot rolls over its
        toe."""
        P = self.P
        x, y, yaw, bz = base
        hz = self.z0 + bz
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        sw = self.swing
        for k, side in enumerate(SIDES):
            hx, hy = self.hip_xy[k]
            hip = (x + c * hx - s * hy, y + s * hx + c * hy)
            th_lo, th_hi, knee_lo = 0.0, P.theta_max, P.knee_min
            if sw is not None and sw["side"] == side and t < sw["t1"]:
                pose, u = self._swing_pose(t)
                ground = sw["P0"][2] + (sw["P1"][2] - sw["P0"][2]) * _minjerk(u)
                knee_lo = P.knee_min + P.knee_swing * math.sin(math.pi * u)
                if sw["squat"]:
                    land = piv = TOE
                elif u < 0.5:
                    land = self._pivot(side, sw["P1"], hip, hz - sw["P1"][2], sw["c1"], 0.0)
                    piv = sw["c0"]
                    th_lo = th_hi = sw["th0"] * (1.0 - _minjerk(min(1.0, u / P.unroll)))
                else:
                    land = piv = self._pivot(side, sw["P1"], hip, hz - sw["P1"][2], sw["c1"], 0.0)
            else:
                landed = sw is not None and sw["side"] == side      # a swing past its landing time (pose_at)
                pose = sw["P1"] if landed else self.plant[side]
                cur = sw["c1"] if landed else self.pivot[side]
                land = piv = self._pivot(side, pose, hip, hz - pose[2], cur, float(self.theta[k]))
                ground = pose[2]
            hov = max(0.0, hz - ground - self.h_top)          # above the straight-leg height: hover as posed
            dx, dy = pose[0] - x, pose[1] - y
            out.append(((c * dx + s * dy, -s * dx + c * dy, pose[2] + hov + self.ankle_h0 - hz), pose[3] - yaw,
                        self._edge(piv), -1.0 if piv == HEEL else 1.0, land, th_lo, th_hi, knee_lo))
        return out

    def _pivot(self, side, pose, hip, h_rel, current, theta):
        """Heel or toe: the foot's edge towards the hip, switching where the foot passes its stance spot under the hip
        (with hysteresis), and only while the foot is flat (or the hip is well past)."""
        c, s = math.cos(pose[3]), math.sin(pose[3])
        fx = (hip[0] - pose[0]) * c + (hip[1] - pose[1]) * s    # the hip ahead of the ankle, along the foot
        ref = -self.home(side, h_rel)[0]
        band = self.P.pivot_band
        want = current
        if current == HEEL and fx > ref + band:
            want = TOE
        elif current == TOE and fx < ref - band:
            want = HEEL
        if want != current and theta > 0.01 and abs(fx - ref) < 0.05:
            return current
        return want

    def _edge(self, piv):
        return self.x_heel if piv == HEEL else self.x_toe

    def _solve(self, t, base, iters, commit):
        """The legs for the targets at time t (warm start: the last solution); commit keeps it and the pivots."""
        if commit and self.swing is None and self.solved_for is not None and self.converged:
            b0 = self.solved_for
            if max(abs(base[0] - b0[0]), abs(base[1] - b0[1]), abs(_wrap(base[2] - b0[2])),
                   abs(base[3] - b0[3])) < self.P.still_tol and self.solved_plant == self._plant_key():
                return self.q
        tg = self._targets(t, base)
        # (pose_at works on a copy of the active set: nothing it does reaches the next step)
        q, th, conv = self._ik(self.q.copy(), self.theta.copy(), tg, iters,
                               self.P.ik_tol if commit else self.P.ik_tol_eval,
                               self._fixed if commit else self._fixed.copy())
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(th))):
            q, th, conv = self.posed(self.z0 + base[3]), np.zeros(2), False
        if commit:
            self.q, self.theta = q, th
            self.converged = conv
            for k, side in enumerate(SIDES):
                if self.swing is not None and self.swing["side"] == side and t < self.swing["t1"]:
                    self.swing["c1"] = tg[k][4]
                else:
                    self.pivot[side] = tg[k][4]
            self.solved_for = tuple(base)
            self.solved_plant = self._plant_key()
        return q

    def _plant_key(self):
        return tuple(tuple(self.plant[s]) for s in SIDES) + (self.swing is not None,)

    def _ik(self, q, th, tg, iters, tol, fixed):
        """Damped least squares on both legs at once, 7 unknowns per leg: its 6 joints and the foot's roll theta >= 0
        over its pivot edge. The ankle-roll frame must match the flat foot rolled by theta (6 rows), theta is drawn to
        0 (a small weight), joints and theta held at their limits (a primal active set, kept between calls). Up to
        `iters` checks of the residual, each followed by a step unless it is below `tol` (m); after a step from a
        residual under `track` m no further check is made (the next call starts from it). Returns (q, theta,
        converged: the last check passed)."""
        P, lm, ld = self.P, self.lm, self.ld
        # per leg, in one array: the flat foot's ankle-roll origin pF and yaw frame RF (pelvis frame), the roll axis
        # (+-the foot's y), the pivot point on it, the axis' cross-product matrix K and K @ K
        h0 = self.ankle_h0
        vals = []
        for (px, py, pz), yw, xc, sg, _, _, _, _ in tg:
            c, s = math.cos(yw), math.sin(yw)
            ax, ay = -s * sg, c * sg
            vals += (px, py, pz, c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0, ax, ay, 0.0, px + c * xc, py + s * xc, pz - h0,
                     0.0, 0.0, ay, 0.0, 0.0, -ax, -ay, ax, 0.0,
                     ax * ax - 1.0, ax * ay, 0.0, ax * ay, ay * ay - 1.0, 0.0, 0.0, 0.0, -1.0)
        V = np.array(vals).reshape(2, 36)
        pF, RF, axis, pivot = V[:, 0:3], V[:, 3:12].reshape(2, 3, 3), V[:, 12:15], V[:, 15:18]
        K, KK = V[:, 18:27].reshape(2, 3, 3), V[:, 27:36].reshape(2, 3, 3)
        r0 = (pF - pivot)[:, :, None]
        lo, hi, eye7 = self._lo7.copy(), self._hi7.copy(), self._eye7
        lo[:, 6] = (tg[0][5], tg[1][5])
        hi[:, 6] = (tg[0][6], tg[1][6])
        lo[:, 3] = np.maximum(lo[:, 3], (tg[0][7], tg[1][7]))
        X = self._X
        X[:, :6] = q.reshape(2, 6)
        X[:, 6] = np.clip(th, lo[:, 6], hi[:, 6])
        X[:, 3] = np.maximum(X[:, 3], lo[:, 3])                    # a swing knee's rising bound
        pinned = hi - lo < 1e-12                                   # a prescribed roll: held, never released
        fixed[pinned] = 1.0
        jacp, jacr, fb, J, e = self.jacp, self.jacr, self.foot_body, self._J7, self._e7
        wr, wt = P.w_rot, P.w_theta
        J[:, 3:6, 6] = -wr * axis
        conv = False
        for _ in range(iters):
            ld.qpos[:] = X[:, :6].ravel()
            mujoco.mj_kinematics(lm, ld)
            th_ = X[:, 6]
            Ra = self._eye3 + np.sin(th_)[:, None, None] * K + (1.0 - np.cos(th_))[:, None, None] * KK
            arm = (Ra @ r0)[:, :, 0]
            Mf = ((Ra @ RF) @ ld.xmat[fb].reshape(2, 3, 3).transpose(0, 2, 1)).reshape(2, 9)
            e[:, :3] = (pivot + arm) - ld.xpos[fb]
            e[:, 3:6] = (0.5 * wr) * (Mf[:, (7, 2, 3)] - Mf[:, (5, 6, 1)])
            res = float(np.abs(e[:, :6]).max())
            if res < tol:
                conv = True
                break
            e[:, 6] = -wt * th_
            mujoco.mj_comPos(lm, ld)
            mujoco.mj_jacBody(lm, ld, jacp[0], jacr[0], fb[0])
            mujoco.mj_jacBody(lm, ld, jacp[1], jacr[1], fb[1])
            J[0, :3, :6] = jacp[0, :, :6]
            J[1, :3, :6] = jacp[1, :, 6:]
            J[0, 3:6, :6] = jacr[0, :, :6]
            J[1, 3:6, :6] = jacr[1, :, 6:]
            J[:, 3:6, :6] *= wr
            J[:, :3, 6] = -(K @ arm[:, :, None])[:, :, 0]           # the target moves about the pivot's axis
            JT = J.transpose(0, 2, 1)
            # Levenberg-Marquardt damping: negligible while tracking, large for a target out of reach (no flailing)
            A = JT @ J + self._eye7 * (1e-10 + P.ik_lm * (e[:, :6] ** 2).sum(axis=1))[:, None, None]
            g = (JT @ e[:, :, None])[:, :, 0]
            # primal active set (kept between calls): unknowns held at a limit. A held one no longer at its limit (the
            # limit moved) or whose multiplier says the objective gains inwards is released; a free one at its limit
            # the step pushes outwards is held; then the remaining free ones are solved again
            at_lo, at_hi = X <= lo + 1e-12, X >= hi - 1e-12
            fixed[(fixed > 0.0) & ~(at_lo | at_hi)] = 0.0
            fixed[pinned] = 1.0
            released = np.zeros_like(pinned)
            for _ in range(4):
                keep = 1.0 - fixed
                A2 = A * (keep[:, :, None] * keep[:, None, :]) + eye7 * fixed[:, :, None]
                dx = np.linalg.solve(A2, (g * keep)[:, :, None])[:, :, 0] * keep
                push = (keep > 0.0) & ~released & ((at_lo & (dx < 0.0)) | (at_hi & (dx > 0.0)))
                if push.any():
                    fixed[push] = 1.0
                    continue
                if not fixed.any():
                    break
                r = (A @ dx[:, :, None])[:, :, 0] - g
                release = (fixed > 0.0) & ~pinned & ((at_lo & (r < -1e-12)) | (at_hi & (r > 1e-12)))
                if not release.any():
                    break
                fixed[release] = 0.0
                released |= release
            Xn = X + dx
            mx = float(np.abs(dx).max())
            if mx > P.ik_step or ((Xn < lo) | (Xn > hi)).any():
                with np.errstate(divide="ignore", invalid="ignore"):
                    room = np.where(dx < 0.0, (lo - X) / dx, np.where(dx > 0.0, (hi - X) / dx, np.inf))
                room = np.where(fixed > 0.0, np.inf, np.maximum(room, 0.0))
                alpha = np.minimum(np.minimum(1.0, room.min(axis=1)),
                                   P.ik_step / np.maximum(np.abs(dx).max(axis=1), P.ik_step))
                fixed[(room <= alpha[:, None] + 1e-12) & (room < 1.0 + 1e-12)] = 1.0
                Xn = np.clip(X + alpha[:, None] * dx, lo, hi)
                full = False                        # a limit cut the step short: check and step again
            else:
                full = True
            X[:] = Xn
            if full and res < P.ik_track:
                break
        return X[:, :6].ravel(), X[:, 6].copy(), conv

