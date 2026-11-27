"""plate_job.py: the reference's gauge plate and job program (part of the reference solution).

What it uses is public: the plate's certificate, the ranges of its pose and the job's rules (cell_data.py), the
pin's certificate (cell_data.TOOLS, PIN_RADIUS, PIN_LENGTH), arm_model.py, and the images and the calibrated
model of reference.py. Nothing here reads the cell.

  * where the plate may be: `envelope` (cell_data.plate_envelope), kept clear by the probing campaign;
  * the plate's views: planned around its nominal pose, taken in the session (`PlateViews`);
  * the plate located in 6 degrees of freedom from all its views together (`locate_plate`): the bores' dark
    mouths found in the first view (blobs, matched to the certificate's pattern), then the mouth outlines
    measured to a fraction of a pixel along profiles normal to their predicted ellipses in every view, the pose
    fitted to all of them with the calibrated camera poses and intrinsics (Gauss-Newton, outliers set aside,
    measured again from the refined pose);
  * the job programs (`Planner`, cold or from given start temperatures, the link temperatures along the program
    predicted with given heating and cooling figures): for each bore, a pose above its mouth with the pin along the bore's axis, a
    descent in short rows along the axis to JOB_AIM below the mouth and a retract along it, every row's target
    solved with the direction signs the row will set (cell_data: a row's path takes the backlash up linearly),
    rows split until the model's path stays within DEV_TOL of the axis, the pin turned a little about its own
    axis at every row so that the joints that serve the turn move clearly one way, transits above the plate
    checked with the model against the located plate, the spheres and posts, the joint targets inside the
    joint limits.
"""
import numpy as np

import arm_model as am
from cell_data import (PLATE_SIZE, N_BORES, BORE_RADIUS, BORE_DEPTH, BORES, PLATE_XY_NOMINAL,
                       PLATE_XY_TOL, PLATE_YAW_NOMINAL, PLATE_YAW_TOL, PLATE_TILT_MAX, PLATE_LIFT, plate_frame,
                       plate_envelope, PIN_RADIUS, PIN_LENGTH, JOB_TOOL, JOB_MAX_ROWS, JOB_SPEED,
                       Q_HOME, S_THRESHOLD, SETTLE, T_AMBIENT, HEAT_NOMINAL, TAU_COOL_RANGE, TOOLS,
                       surveyed_spheres, R_SPHERE, R_POST, KNOCK_MAX, IMAGE_W, IMAGE_H, IMAGE_TIME)

D = am.D
SPHERES = surveyed_spheres()
ENV_RADIUS, ENV_TOP = plate_envelope()
TIP_C = np.asarray(TOOLS[JOB_TOOL]["tip"], float) + np.array([0.0, 0.0, am.FLANGE_Z])


# ---- the plate's geometry (certificate) -----------------------------------------------------------------
def nominal_pose():
    """(x, y, lift, yaw, tilt_x, tilt_y) at the middle of the documented ranges."""
    return np.array([PLATE_XY_NOMINAL[0], PLATE_XY_NOMINAL[1], np.mean(PLATE_LIFT), PLATE_YAW_NOMINAL, 0.0, 0.0])


def frame_of(pose):
    """Rotation R (datum <- plate) and origin O (top face centre) of a pose vector."""
    x, y, lift, yaw, tx, ty = pose
    return plate_frame(x, y, lift, yaw, tx, ty)


def mouths(pose):
    """Datum-frame centres of the bores' mouths (N_BORES, 3) and the axis pointing into the plate (3,)."""
    R, O = frame_of(pose)
    return O[None] + np.column_stack([BORES, np.zeros(N_BORES)]) @ R.T, -R[:, 2]


def plate_distance(R, O, P):
    """Distance (m) from points P (N, 3) to the block less its bores (0 inside), the plate at (R, O)."""
    a, b, T = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2, PLATE_SIZE[2]
    p = (np.asarray(P, float) - O) @ R
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    rect = np.hypot(np.maximum(np.abs(x) - a, 0.0), np.maximum(np.abs(y) - b, 0.0))
    inside = rect == 0.0
    rho = np.hypot(x[:, None] - BORES[None, :, 0], y[:, None] - BORES[None, :, 1]).min(axis=1)
    disc = inside & (rho < BORE_RADIUS)
    wall = BORE_RADIUS - rho
    above = np.hypot(z, np.where(disc, wall, rect))
    hole = disc & (z >= -BORE_DEPTH)
    slab = np.where(inside, np.where(hole, np.minimum(wall, z + BORE_DEPTH), 0.0), rect)
    return np.where(z > 0.0, above, np.where(z >= -T, slab, np.hypot(-T - z, rect)))


def envelope_clearance(points):
    """Clearance (m) of points to the documented envelope of the plate (a vertical cylinder from the table)."""
    r = np.hypot(points[:, 0] - PLATE_XY_NOMINAL[0], points[:, 1] - PLATE_XY_NOMINAL[1]) - ENV_RADIUS
    h = points[:, 2] - ENV_TOP
    return np.where(r > 0, np.where(h > 0, np.hypot(r, h), r), np.where(h > 0, h, np.maximum(r, h)))


# ---- kinematics helpers ----------------------------------------------------------------------------------
def _rot_err(R, Rd):
    return 0.5 * (np.cross(R[..., :, 0], Rd[..., :, 0]) + np.cross(R[..., :, 1], Rd[..., :, 1])
                  + np.cross(R[..., :, 2], Rd[..., :, 2]))


def axis_frame(a, spin):
    """Flange rotation whose z axis is a (the pin's axis, unit), spun by `spin` about it."""
    a = a / np.linalg.norm(a)
    ref = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = ref - (ref @ a) * a
    x /= np.linalg.norm(x)
    y = np.cross(a, x)
    c, s = np.cos(spin), np.sin(spin)
    return np.stack([c * x + s * y, -s * x + c * y, a], axis=1)


class Kin(object):
    """The pin's apex and the flange's rotation as functions of (q, s, temp), process_c mounted: under a
    parameter vector of reference.py (or any callable with the same signature, for studies)."""

    def __init__(self, fn):
        self.fn = fn

    def __call__(self, q, s, temp):
        q = np.atleast_2d(np.asarray(q, float))
        return self.fn(q, np.broadcast_to(s, q.shape), np.broadcast_to(temp, q.shape))

    def pin(self, q, s, temp):
        """Axis segment ends E (hemisphere centre), B (far end), apex: (N, 3) each."""
        t, R = self(q, s, temp)
        a = R[:, :, 2]
        return t - PIN_RADIUS * a, t - (PIN_LENGTH - PIN_RADIUS) * a, t

    def ik(self, target, Rd, q0, s, temp, iters=40, tol=1e-9):
        """Damped least squares on the full pose. Returns q, ok."""
        q = np.array(q0, float)
        h = 1e-7
        for _ in range(iters):
            qq = np.vstack([q] + [q + h * np.eye(6)[j] for j in range(6)])
            t, R = self(qq, s, temp)
            e = np.concatenate([target - t[0], _rot_err(R[0], Rd)])
            if np.abs(e).max() < tol:
                break
            J = np.empty((6, 6))
            for j in range(6):
                J[:3, j] = (t[j + 1] - t[0]) / h
                J[3:, j] = _rot_err(R[0], R[j + 1]) / h
            dq = np.linalg.solve(J.T @ J + 1e-12 * np.eye(6), J.T @ e)
            step = np.abs(dq).max()
            q = q + dq * min(1.0, 0.3 / max(step, 1e-12))
        t, R = self(q, s, temp)
        err = max(np.linalg.norm(t[0] - target), np.linalg.norm(_rot_err(R[0], Rd)) * 0.3)
        ok = err < 1e-8 and np.all(q > am.Q_MIN + 1e-3) and np.all(q < am.Q_MAX - 1e-3)
        return q, ok


def ik5(kin, target, axis, q0, s, temp, hold=(), iters=40):
    """The apex at `target` and the pin along `axis`, the turn about the axis free, the joints in `hold` kept
    where they are (at most one). Returns q, ok."""
    free = [j for j in range(6) if j not in hold]
    q = np.array(q0, float)
    h = 1e-7
    for _ in range(iters):
        qq = np.vstack([q] + [q + h * np.eye(6)[j] for j in free])
        t, R = kin(qq, s, temp)
        ax = R[:, :, 2]
        e = np.concatenate([target - t[0], np.cross(axis, ax[0])])
        J = np.empty((6, len(free)))
        for c in range(len(free)):
            J[:3, c] = (t[c + 1] - t[0]) / h
            J[3:, c] = (np.cross(ax[c + 1], axis) - np.cross(ax[0], axis)) / h
        dq = np.linalg.lstsq(J, e, rcond=1e-10)[0]
        step = np.abs(dq).max()
        q[free] = q[free] + dq * min(1.0, 0.3 / max(step, 1e-12))
        if step < 1e-12:
            break
    t, R = kin(q[None], s, temp)
    ok = (np.linalg.norm(t[0] - target) < 1e-8 and np.linalg.norm(np.cross(R[0, :, 2], axis)) < 1e-7
          and float(R[0, :, 2] @ axis) > 0 and np.all(q > am.Q_MIN + 1e-3) and np.all(q < am.Q_MAX - 1e-3))
    return q, ok


def spin_of(R, a):
    """The turn about the axis a of a flange rotation R whose z is a (inverse of axis_frame)."""
    x0 = axis_frame(a, 0.0)[:, 0]
    y0 = np.cross(a, x0)
    return float(np.arctan2(R[:, 0] @ y0, R[:, 0] @ x0))


def new_signs(q_prev, q, s_prev):
    dq = q - q_prev
    return np.where(np.abs(dq) > S_THRESHOLD, np.sign(dq), s_prev)


# ---- the job program -------------------------------------------------------------------------------------
JOB_APPROACH = 0.006        # m above the mouth (along the axis): the pose before entering a bore
JOB_AIM = 0.010             # m below the mouth aimed at (done at JOB_DEPTH, the bottom at BORE_DEPTH)
STEP_IN = 0.004             # m: a row inside a bore at most this long
DEV_TOL = 12e-6             # m: a row inside a bore leaves its line by at most this (else it is split)
DEV_FREE = 0.3e-3           # m: ... a row above the mouth
TRANSIT_UP = 0.030          # m: transits between bores with the apex this far above the top face
CLEAR_TRANSIT = 0.008       # m: the pin's clearance to the located plate along transits (model)
CLEAR_OBJECTS = 0.015       # m: ... to the surveyed spheres and posts (a knocked sphere included) and the table
HUB_HEIGHT = 0.20           # m: the apex above the plate's centre at the hub pose
SPINS = np.arange(-180.0, 180.0, 15.0) * D
SPIN_STEP = 2.0 * D          # rad: the pin's turn about its own axis per row near a bore


class Planner(object):
    """The job program for a plate pose under a kinematic model (Kin). Every row's target is solved with the
    direction signs the row will set; the pin is turned about its own axis by SPIN_STEP at every row in and
    near a bore (the pin is round: harmless), so that every joint that serves the turn moves clearly one way.
    A target whose solution would set a sign that is not its own (a joint asked to move by less than its
    backlash) is turned a little more or less, the pin's axis tipped by under a milliradian, the target moved
    along the axis by under half a millimetre, or one joint held."""

    def __init__(self, kin, pose, temp0=None, heat=None, tau=None, log=None):
        self.kin = kin
        self.R, self.O = frame_of(pose)
        self.pose = np.array(pose, float)
        self.mouth, self.a = mouths(pose)
        self.temp = np.full(6, T_AMBIENT) if temp0 is None else np.array(temp0, float)
        # heating: K per radian, link i by joint j (6 x 6; a number: the documented nominal, own joint only)
        self.heat = np.eye(6) * (HEAT_NOMINAL if heat is None else heat) if np.ndim(heat) < 2 else np.array(heat, float)
        self.tau = np.mean(TAU_COOL_RANGE) if tau is None else float(tau)
        self.rows = []
        self.q, self.s = Q_HOME.copy(), np.ones(6)
        self.spin = 0.0
        self.log = log if log is not None else []

    # -- prediction of the link temperatures along the program (the heating and cooling given, by default the
    # documented nominal figures; cell.md, section 7: the same law as in a session) --
    def _advance(self, dq):
        duration = np.abs(dq).max() / JOB_SPEED + SETTLE
        level = T_AMBIENT + self.heat @ np.abs(dq) / duration * self.tau
        self.temp = level + (self.temp - level) * np.exp(-duration / self.tau)

    def push(self, q, spin):
        q = np.array(q, float)
        s = new_signs(self.q, q, self.s)
        self._advance(q - self.q)
        self.rows.append(q.copy())
        self.q, self.s, self.spin = q, s, spin

    def _state(self):
        return (self.q.copy(), self.s.copy(), self.temp.copy(), len(self.rows), self.spin)

    def _restore(self, st):
        self.q, self.s, self.temp, self.spin = st[0].copy(), st[1].copy(), st[2].copy(), st[4]
        del self.rows[st[3]:]

    def _apex(self, s=None):
        return self.kin.pin(self.q[None], self.s if s is None else s, self.temp)[2][0]

    # -- one row's target --
    def solve(self, target, spin, q_seed=None):
        """Joint target for the apex at `target`, the pin along the bore axis turned by `spin`, consistent with
        the signs the row sets. When no such solution exists (a joint asked to move by less than its
        backlash), the turn is nudged and the target moved a little along the axis; last, one joint is held
        where it is with the turn left free. Returns q, s, spin (None when unreachable)."""
        q_prev, s_prev = self.q, self.s
        q0 = np.array(q_prev if q_seed is None else q_seed, float)

        def consistent(fn):
            q, s = q0.copy(), s_prev.copy()
            for _ in range(5):
                q, ok = fn(q, s)
                if not ok:
                    return None, None
                s_new = new_signs(q_prev, q, s_prev)
                if np.array_equal(s_new, s):
                    return (q, s) if np.all(np.abs(np.abs(q - q_prev) - S_THRESHOLD) > 1e-9) else (None, None)
                s = s_new
            return None, None
        e1 = axis_frame(self.a, 0.0)[:, 0]
        e2 = np.cross(self.a, e1)
        tilts = [np.zeros(3)] + [m * e for m in (3e-4, 6e-4, 1e-3) for e in (e1, -e1, e2, -e2)]
        for tilt in tilts:
            axis = self.a + tilt
            axis /= np.linalg.norm(axis)
            for shift in ((0.0, 3e-4, -3e-4) if not np.any(tilt) else (0.0,)):
                t = target + shift * self.a
                for nudge in (0.0, 0.7, -0.7, 1.5, -1.5, 3.0, -3.0):
                    Rd = axis_frame(axis, spin + nudge * D)
                    q, s = consistent(lambda qq, ss: self.kin.ik(t, Rd, qq, ss, self.temp))
                    if q is not None:
                        return q, s, spin + nudge * D
        for j in range(6):
            q, s = consistent(lambda qq, ss: ik5(self.kin, target, self.a, qq, ss, self.temp, hold=(j,)))
            if q is not None:
                _, R = self.kin(q[None], s, self.temp)
                return q, s, spin_of(R[0], self.a)
        return None, None, None

    def row_pin(self, q0, q1, s0, s1, lam):
        """The pin along a row (the job's rule: joints on the straight line, each direction sign going linearly
        from its old to its new value): E, B, apex at the path fractions lam."""
        lam = np.asarray(lam, float)[:, None]
        return self.kin.pin(q0[None] + lam * (q1 - q0)[None], s0[None] + lam * (s1 - s0)[None], self.temp)

    def path_clearance(self, q0, q1, s0, s1):
        """Least model clearance of the pin along the row to the located plate less CLEAR_TRANSIT and to the
        surveyed spheres and posts (a knocked sphere's worst case) and the table less CLEAR_OBJECTS."""
        n = int(max(2, np.ceil(np.abs(q1 - q0).max() / 0.002) + 1))
        E, B, _ = self.row_pin(q0, q1, s0, s1, np.linspace(0.0, 1.0, n))
        u = np.linspace(0.0, 1.0, 41)
        P = (E[:, None, :] + u[None, :, None] * (B - E)[:, None, :]).reshape(-1, 3)
        d = plate_distance(self.R, self.O, P).min() - PIN_RADIUS
        sp = np.linalg.norm(P[:, None, :] - SPHERES[None], axis=2).min() - R_SPHERE - KNOCK_MAX
        po = np.inf
        for c in SPHERES:
            t = np.clip(P[:, 2] / c[2], 0.0, 1.0)
            po = min(po, float(np.min(np.hypot(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]), P[:, 2] - t * c[2]))) - R_POST - KNOCK_MAX)
        other = min(sp, po, float(P[:, 2].min())) - PIN_RADIUS
        return float(min(d - CLEAR_TRANSIT, other - CLEAR_OBJECTS))

    def lateral_dev(self, q0, q1, s0, s1, p0, p1, n=24):
        """Largest distance of the apex from the line p0 -> p1 along the row (model)."""
        _, _, apex = self.row_pin(q0, q1, s0, s1, np.linspace(0.0, 1.0, n))
        d = p1 - p0
        L = np.linalg.norm(d)
        u = d / L if L > 0 else self.a
        w = apex - p0
        return float(np.linalg.norm(w - (w @ u)[:, None] * u, axis=1).max())

    def _line(self, start, end, step):
        n = max(1, int(np.ceil(np.linalg.norm(end - start) / step)))
        return [start + (end - start) * (k + 1) / n for k in range(n)]

    def go_line(self, start, end, turn, step=STEP_IN, tol=DEV_TOL):
        """Rows along the line start -> end (the apex at `start` now), the pin turned by `turn` per row; a row
        whose apex would leave its line by more than `tol` is split."""
        for p in self._line(start, end, step):
            q, s, sp = self.solve(p, self.spin + turn)
            if q is None:
                return False
            p0 = self._apex()
            if step > 0.6e-3 and self.lateral_dev(self.q, q, self.s, s, p0, p) > tol:
                if not self.go_line(p0, p, turn * 0.5, step / 2, tol):
                    return False
                continue
            self.push(q, sp)
        return True

    def transit(self, target, spin):
        """Rows to (target, spin) clear of everything (model): direct, or by way of points above."""
        home = np.abs(self.q - Q_HOME).max() < 1e-9
        seed = self._far_seed(target, spin) if home else None
        q, s, sp = self.solve(target, spin, q_seed=seed)
        if q is not None and self.path_clearance(self.q, q, self.s, s) > 0:
            self.push(q, sp)
            return True
        here = self._apex()
        up = max(here[2], target[2]) + 0.04
        hub = self.O + np.array([0.0, 0.0, HUB_HEIGHT])
        for via in ([np.array([here[0], here[1], up]), np.array([target[0], target[1], up])], [hub],
                    [np.array([here[0], here[1], hub[2]]), hub]):
            st = self._state()
            good = True
            for v in list(via) + [target]:
                home = np.abs(self.q - Q_HOME).max() < 1e-9
                qv, sv, spv = self.solve(v, spin, q_seed=self._far_seed(v, spin) if home else None)
                if qv is None or self.path_clearance(self.q, qv, self.s, sv) <= 0:
                    good = False
                    break
                self.push(qv, spv)
            if good:
                return True
            self._restore(st)
        return False

    def _seed_for(self, point):
        """A joint seed for the pin pointing down near `point` (elbow up, wrist bent down)."""
        az = np.arctan2(point[1], point[0])
        return np.array([az, -60.0 * D, 120.0 * D, 0.0, 40.0 * D, 0.0])

    def _far_seed(self, target, spin):
        q, ok = self.kin.ik(target, axis_frame(self.a, spin), self._seed_for(target), self.s, self.temp, iters=80)
        return q if ok else self._seed_for(target)

    def choose_spin(self):
        """The turn of the pin about its axis at which every bore's bottom pose is reachable with the widest
        joint margins."""
        best = None
        for spin in SPINS:
            worst = np.inf
            for i in range(N_BORES):
                bottom = self.mouth[i] + JOB_AIM * self.a
                qb, ok = self.kin.ik(bottom, axis_frame(self.a, spin), self._seed_for(bottom), np.ones(6), self.temp)
                if not ok:
                    worst = -np.inf
                    break
                worst = min(worst, np.min(np.minimum(qb - am.Q_MIN, am.Q_MAX - qb)))
            if best is None or worst > best[0]:
                best = (worst, spin)
        return None if best is None or not np.isfinite(best[0]) else best

    def bore(self, i, turn):
        """Rows into bore i along its axis to JOB_AIM below the mouth and out again, from above it."""
        m, a = self.mouth[i], self.a
        start = m - JOB_APPROACH * a
        if not self.go_line(self._apex(), start, turn, step=0.012, tol=DEV_FREE):
            self.log.append("bore {}: approach failed".format(i))
            return False
        if not self.go_line(start, m + JOB_AIM * a, turn):
            self.log.append("bore {}: descent failed".format(i))
            return False
        if not self.go_line(self._apex(), start, turn):
            self.log.append("bore {}: retract failed".format(i))
            return False
        return True

    def plan(self, order=None):
        best = self.choose_spin()
        if best is None:
            self.log.append("no turn of the pin reaches every bore")
            return None
        spin0 = best[1]
        self.log.append("turn {:.0f} deg, least joint margin {:.0f} deg".format(spin0 / D, best[0] / D))
        for n, i in enumerate(order if order is not None else self._order()):
            turn = SPIN_STEP if n % 2 == 0 else -SPIN_STEP
            start_spin = spin0 - turn * 8                  # so that the turns of a bore are centred on spin0
            if not self.transit(self.mouth[i] - TRANSIT_UP * self.a, start_spin):
                self.log.append("bore {}: no way above it".format(i))
                return None
            if not self.bore(i, turn):
                return None
            q, s, sp = self.solve(self.mouth[i] - TRANSIT_UP * self.a, self.spin)
            if q is None:
                return None
            self.push(q, sp)
        if len(self.rows) > JOB_MAX_ROWS:
            self.log.append("{} rows: too many".format(len(self.rows)))
            return None
        return np.array(self.rows)

    def _order(self):
        """Bores in a short tour starting from the one nearest the base."""
        left = list(range(N_BORES))
        r = np.hypot(self.mouth[:, 0], self.mouth[:, 1])
        cur = int(np.argmin(r))
        out = [cur]
        left.remove(cur)
        while left:
            nxt = min(left, key=lambda k: np.linalg.norm(self.mouth[k] - self.mouth[cur]))
            out.append(nxt)
            left.remove(nxt)
            cur = nxt
        return out


# ---- the plate's views ----------------------------------------------------------------------------------
N_PLATE_VIEWS = 6
PLATE_TOP_NOMINAL = np.mean(PLATE_LIFT) + PLATE_SIZE[2]       # m: the top face's height at the nominal pose
PRELOAD_CAM = 6e-4


def plate_points(pose, grow=0.0, n=(9, 6)):
    """Datum-frame points on the plate's top face (a grid), its outline grown by `grow`."""
    R, O = frame_of(pose)
    xs = np.linspace(-PLATE_SIZE[0] / 2 - grow, PLATE_SIZE[0] / 2 + grow, n[0])
    ys = np.linspace(-PLATE_SIZE[1] / 2 - grow, PLATE_SIZE[1] / 2 + grow, n[1])
    g = np.array([[x, y, 0.0] for x in xs for y in ys])
    return O[None] + g @ R.T


def _occluded(oc, P):
    """Which of the points P (datum frame) a datum sphere or a post (surveyed, with margins) hides from oc."""
    d = P - oc
    L = np.linalg.norm(d, axis=1)
    u = d / L[:, None]
    hid = np.zeros(len(P), bool)
    for c in SPHERES:
        t = (c - oc) @ u.T
        dist = np.linalg.norm(oc + t[:, None] * u - c, axis=1)
        hid |= (t > 0) & (t < L) & (dist < R_SPHERE + KNOCK_MAX + 0.006)
        # the post: closest approach of the ray to the vertical segment under the sphere
        for z in np.linspace(0.0, c[2], 12):
            pc = np.array([c[0], c[1], z])
            t = (pc - oc) @ u.T
            dist = np.linalg.norm(oc + t[:, None] * u - pc, axis=1)
            hid |= (t > 0) & (t < L) & (dist < R_POST + KNOCK_MAX + 0.006 + c[2] / 22.0)
    return hid


def view_candidates(ref, p, tool, temp, rng, n_try=800):
    """Camera poses looking at the plate's nominal place from 0.34 to 0.42 m, 45 to 72 degrees down, from the
    side of the arm's base, the whole range of the plate's place in the image and nothing in front of it;
    their joint solutions (current model, the documented camera mounting)."""
    C0 = np.array([PLATE_XY_NOMINAL[0], PLATE_XY_NOMINAL[1], PLATE_TOP_NOMINAL])
    base = np.arctan2(-C0[1], -C0[0])
    alpha = base + rng.uniform(-75, 75, n_try) * D
    el = rng.uniform(45, 72, n_try) * D
    dist = rng.uniform(0.38, 0.48, n_try)
    w = np.stack([np.cos(el) * np.cos(alpha), np.cos(el) * np.sin(alpha), np.sin(el)], axis=1)
    oc = C0[None] + dist[:, None] * w
    v = -w
    # the image's rows along the plate's length
    lx = np.array([np.cos(PLATE_YAW_NOMINAL), np.sin(PLATE_YAW_NOMINAL), 0.0])
    x = lx[None] - (v @ lx)[:, None] * v
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    y = np.cross(v, x)
    roll = rng.uniform(-0.35, 0.35, n_try)
    c, s_ = np.cos(roll)[:, None], np.sin(roll)[:, None]
    Rc = np.stack([c * x + s_ * y, -s_ * x + c * y, v], axis=2)
    xc = p[ref.CAM]
    R6 = Rc @ am.camera_rotation(xc[3:]).T
    target = oc + np.einsum("nab,b->na", R6, ref.tip_vector(tool) - ref._cam_vec(xc))
    seed = np.tile(np.array([0.0, -80.0, 100.0, 0.0, 60.0, 0.0]) * D, (n_try, 1)) + rng.normal(0, 0.3, (n_try, 6))
    seed[:, 0] = np.arctan2(oc[:, 1], oc[:, 0]) + rng.normal(0, 0.2, n_try)
    q, ok = ref.ik_pose(p, target, R6, seed, np.zeros(6), temp, tool)
    pre = rng.choice([-1.0, 1.0], (n_try, 6))
    tips = ref.tip(p, q, pre, temp, tool)
    ok &= ref.clearance(tips) > 0.03
    ok &= oc[:, 2] > 0.15
    grid = plate_points(nominal_pose(), grow=0.025)
    out = []
    th = ref.vision_prior()[0][:5]
    for k in np.nonzero(ok)[0]:
        P = (grid - oc[k]) @ Rc[k]
        if np.any(P[:, 2] < 0.1):
            continue
        u, vv = ref.project_px(P, th)
        if u.min() < 10 or u.max() > IMAGE_W - 11 or vv.min() < 10 or vv.max() > IMAGE_H - 11:
            continue
        if _occluded(oc[k], plate_points(nominal_pose(), grow=0.03)).mean() > 0.0:
            continue
        out.append(dict(q=q[k], pre=pre[k], alpha=float(alpha[k]), el=float(el[k]), oc=oc[k]))
    return out


def spread(cands, n):
    """n views far apart in the direction they look from (greedy farthest point on the unit sphere)."""
    if not cands:
        return []
    dirs = np.array([[np.cos(c["el"]) * np.cos(c["alpha"]), np.cos(c["el"]) * np.sin(c["alpha"]), np.sin(c["el"])]
                     for c in cands])
    chosen = [int(np.argmin(dirs[:, 2]))]
    while len(chosen) < min(n, len(cands)):
        d = np.min(np.linalg.norm(dirs[:, None, :] - dirs[None, chosen, :], axis=2), axis=1)
        d[chosen] = -1
        chosen.append(int(np.argmax(d)))
    return [cands[i] for i in chosen]



class PlateViews(object):
    """The plate's images in the session (after the touches, before the spheres' views): planned with the probe-fitted model."""

    def __init__(self, camp, p, say=None):
        self.camp, self.rb, self.p = camp, camp.rb, p
        self.records = []
        self.say = say or (lambda *a: None)
        self.escaped = False

    def _lift(self, ref):
        """Move the stylus 0.12 m straight up (model), if that path is clear."""
        self.escaped = True
        rb = self.rb
        s = np.nan_to_num(rb.s, nan=1.0)
        t = ref.tip(self.camp.p, rb.q, s, rb.temp, rb.tool)[0]
        q_up = ref.ik_position(self.camp.p, (t + np.array([0.0, 0.0, 0.12]))[None], rb.q[None], s, rb.temp, rb.tool)[0]
        if np.any(q_up < am.Q_MIN) or np.any(q_up > am.Q_MAX) or not self.camp.path_clear(rb.q, q_up, 0.01, relax=True):
            return False
        rb.move(q_up)
        self.say("  plate: stylus lifted")
        return True

    def run(self, n):
        import reference as ref
        rb = self.rb
        if rb.damaged or rb.images_left < 1 or n < 1:
            return self.records
        rng = np.random.default_rng(31337)
        cands = view_candidates(ref, self.p, rb.tool, rb.temp.copy(), rng)
        views = spread(cands, n + 6)
        self.say("plate: {} candidate views".format(len(cands)))
        pending = list(range(min(n, len(views))))
        spares = list(range(min(n, len(views)), len(views)))
        while pending and len(self.records) < n:
            if rb.images_left < 1 or rb.time_left < IMAGE_TIME + 10:
                break
            j = min(pending, key=lambda i: np.abs(views[i]["q"] - rb.q).max())
            pending.remove(j)
            v = views[j]
            qa = v["q"] - v["pre"] * PRELOAD_CAM
            if not self.camp.path_clear(qa, v["q"], 0.01):
                if spares:
                    pending.append(spares.pop(0))
                continue
            if not self.camp.travel(qa):
                # no way from here (the stylus low after the last view): lift it straight up once and try again
                if not self.escaped and self._lift(ref):
                    pending.append(j)
                    continue
                if spares:
                    pending.append(spares.pop(0))
                continue
            rb.move(v["q"])
            temp = rb.temp.copy()
            obs = rb.image()
            self.records.append(dict(q=rb.q.copy(), s=rb.s.copy(), temp=temp, tool=rb.tool, image=obs["image"]))
            self.say("  plate image {} (images left {}, time left {:.0f})".format(len(self.records), rb.images_left,
                                                                                   rb.time_left))
        return self.records



# ---- locating the plate in its images -------------------------------------------------------------------
N_RIM = 48                  # points per bore's mouth outline
PROFILE_OUT = 6.0           # px: a profile reaches this far outside the predicted outline ...
PROFILE_STEP = 0.25         # px
MIN_CONTRAST = 25.0         # grey levels between the bore's inside and the top face
POSE_STEP = np.array([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6])
POSE_PRIOR_SD = np.array([PLATE_XY_TOL, PLATE_XY_TOL, (PLATE_LIFT[1] - PLATE_LIFT[0]) / 2, PLATE_YAW_TOL,
                          PLATE_TILT_MAX, PLATE_TILT_MAX]) / np.sqrt(3.0)


def rim_points(pose, n=N_RIM, radius=BORE_RADIUS):
    """Datum-frame points of every bore's mouth outline (N_BORES, n, 3) and the mouths' centres."""
    R, O = frame_of(pose)
    phi = 2 * np.pi * np.arange(n) / n
    loc = np.zeros((N_BORES, n, 3))
    loc[..., 0] = BORES[:, 0, None] + radius * np.cos(phi)[None]
    loc[..., 1] = BORES[:, 1, None] + radius * np.sin(phi)[None]
    cen = np.column_stack([BORES, np.zeros(N_BORES)])
    return O + loc @ R.T, O + cen @ R.T


def _project(ref, P, Rc, oc, th):
    C = (P - oc) @ Rc
    with np.errstate(all="ignore"):
        u, v = ref.project_px(C, th)
    return np.stack([u, v], axis=-1), C[..., 2]


def detect_bores(img, ref, Rc, oc, th):
    """Dark blobs of a bore's size inside the image region where the plate can be: their datum-frame places on
    the plane of the top face's nominal height."""
    from scipy import ndimage
    grid = plate_points(nominal_pose(), grow=0.045, n=(13, 9))
    uv, z = _project(ref, grid, Rc, oc, th)
    if np.any(z < 0.05) or not np.all(np.isfinite(uv)):
        return np.zeros((0, 2))
    j0, j1 = int(max(0, uv[:, 0].min())), int(min(IMAGE_W, uv[:, 0].max() + 1))
    i0, i1 = int(max(0, uv[:, 1].min())), int(min(IMAGE_H, uv[:, 1].max() + 1))
    if j1 - j0 < 20 or i1 - i0 < 20:
        return np.zeros((0, 2))
    sub = ndimage.gaussian_filter(img[i0:i1, j0:j1].astype(float), 1.5)
    bg = ndimage.maximum_filter(ndimage.uniform_filter(sub, 9), 41)
    dist = float(np.median(z))
    r_px = th[0] * BORE_RADIUS / dist
    area = np.pi * r_px ** 2
    lab, n = ndimage.label((sub < 0.45 * bg) & (bg > 30.0))
    if n == 0:
        return np.zeros((0, 2))
    idx = np.arange(1, n + 1)
    sizes = ndimage.sum(np.ones_like(sub), lab, idx)
    cents = np.array(ndimage.center_of_mass(np.ones_like(sub), lab, idx)).reshape(-1, 2)
    keep = (sizes > 0.2 * area) & (sizes < 1.6 * area)
    out = []
    for (ci, cj) in cents[keep]:
        ray = ref.rays_px(np.array(cj + j0), np.array(ci + i0), th)
        d = Rc @ ray
        if d[2] > -1e-3:
            continue
        t = (PLATE_TOP_NOMINAL - oc[2]) / d[2]
        out.append((oc + t * d)[:2])
    return np.array(out).reshape(-1, 2)


def match_pattern(pts):
    """The plate's (x, y, yaw) whose bore pattern best fits the detected points (datum xy): pairs of points
    matched to pairs of bores, the hypothesis with the most bores within 2.5 mm; None without three."""
    if len(pts) < 3:
        return None
    B = BORES
    best = None
    for i in range(len(pts)):
        for j in range(len(pts)):
            if i == j:
                continue
            dij = pts[j] - pts[i]
            L = np.linalg.norm(dij)
            for k in range(N_BORES):
                for l in range(N_BORES):
                    if k == l or abs(np.linalg.norm(B[l] - B[k]) - L) > 3e-3:
                        continue
                    yaw = np.arctan2(dij[1], dij[0]) - np.arctan2(*(B[l] - B[k])[::-1])
                    yaw = (yaw - PLATE_YAW_NOMINAL + np.pi) % (2 * np.pi) - np.pi + PLATE_YAW_NOMINAL
                    if abs(yaw - PLATE_YAW_NOMINAL) > PLATE_YAW_TOL + 8 * D:
                        continue
                    c, s = np.cos(yaw), np.sin(yaw)
                    Rz = np.array([[c, -s], [s, c]])
                    t = pts[i] - Rz @ B[k]
                    mapped = B @ Rz.T + t
                    dd = np.linalg.norm(mapped[:, None, :] - pts[None], axis=2).min(axis=1)
                    score = (dd < 2.5e-3).sum() - 1e-3 * dd[dd < 2.5e-3].sum() / 2.5e-3
                    if best is None or score > best[0]:
                        best = (score, yaw, t)
    if best is None or best[0] < 3:
        return None
    _, yaw, t = best
    return np.array([t[0], t[1], np.mean(PLATE_LIFT), yaw, 0.0, 0.0])


def measure_rims(img, ref, pose, Rc, oc, th):
    """Edge points of every visible bore mouth: for each outline point, its predicted pixel, the outward
    normal of the predicted outline, and the offset (px) along it of the 50 % level between the bore's dark
    inside and the bright top face. Returns arrays (k,), (k, 2), (k, 2), (k,) and the bore index (k,)."""
    from scipy import ndimage
    P, cen = rim_points(pose)
    uv, z = _project(ref, P, Rc, oc, th)
    uvc, _ = _project(ref, cen, Rc, oc, th)
    if not np.all(np.isfinite(uv)) or np.any(z < 0.05):
        return None
    hidden = _occluded(oc, P.reshape(-1, 3)).reshape(N_BORES, N_RIM)
    tang = np.roll(uv, -1, axis=1) - np.roll(uv, 1, axis=1)
    nrm = np.stack([tang[..., 1], -tang[..., 0]], axis=-1)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=-1, keepdims=True), 1e-12)
    flip = ((uv - uvc[:, None, :]) * nrm).sum(-1) < 0
    nrm[flip] *= -1
    rad = np.linalg.norm(uv - uvc[:, None, :], axis=-1)
    t = np.arange(-PROFILE_OUT, PROFILE_OUT + 1e-9, PROFILE_STEP)
    out_k, out_uv, out_n, out_s, out_b = [], [], [], [], []
    for b in range(N_BORES):
        for k in range(N_RIM):
            if hidden[b, k]:
                continue
            inner = min(PROFILE_OUT, 0.6 * rad[b, k])
            pts = uv[b, k][None] + t[:, None] * nrm[b, k][None]
            if (pts[:, 0].min() < 3 or pts[:, 0].max() > IMAGE_W - 4 or pts[:, 1].min() < 3
                    or pts[:, 1].max() > IMAGE_H - 4):
                continue
            prof = ndimage.map_coordinates(img, [pts[:, 1], pts[:, 0]], order=1, mode="nearest")
            sel = t >= -inner
            tt, pp = t[sel], prof[sel]
            lo = np.median(pp[tt <= -inner + 1.5])
            hi = np.median(pp[tt >= PROFILE_OUT - 1.5])
            if hi - lo < MIN_CONTRAST or hi - lo < 0.4 * hi:
                continue
            mid = 0.5 * (lo + hi)
            above = pp >= mid
            cross = np.nonzero(~above[:-1] & above[1:])[0]
            if len(cross) != 1:
                continue
            c = cross[0]
            s0 = tt[c] + (mid - pp[c]) / (pp[c + 1] - pp[c]) * PROFILE_STEP
            out_k.append(k)
            out_uv.append(uv[b, k])
            out_n.append(nrm[b, k])
            out_s.append(s0)
            out_b.append(b)
    if not out_k:
        return None
    return (np.array(out_k), np.array(out_uv), np.array(out_n), np.array(out_s), np.array(out_b))


def _rim_pixels(ref, pose, Rc, oc, th, b, k):
    P, _ = rim_points(pose)
    uv, _ = _project(ref, P[b, k], Rc, oc, th)
    return uv


def fit_pose(ref, views, pose0, free=(0, 1, 2, 3, 4, 5), iters=8, edge=True):
    """Gauss-Newton fit of the plate's pose (and a common edge offset) to the edge measurements of every
    view: views = [(Rc, oc, th, meas)]; robust (Huber at 2.5 sigma). Returns pose, edge offset, rms (px), n."""
    pose = np.array(pose0, float)
    e = 0.0
    free = list(free)
    for _ in range(iters):
        r_all, J_all = [], []
        for Rc, oc, th, m in views:
            if m is None:
                continue
            kk, uv0, nrm, s0, bb = m
            u = _rim_pixels(ref, pose, Rc, oc, th, bb, kk)
            res = s0 - ((u - uv0) * nrm).sum(1) - e
            J = np.zeros((len(res), len(free) + 1))
            for c, j in enumerate(free):
                dp = pose.copy()
                dp[j] += POSE_STEP[j]
                du = _rim_pixels(ref, dp, Rc, oc, th, bb, kk) - u
                J[:, c] = (du * nrm).sum(1) / POSE_STEP[j]
            J[:, -1] = 1.0 if edge else 0.0
            r_all.append(res)
            J_all.append(J)
        if not r_all:
            return None
        r, J = np.concatenate(r_all), np.concatenate(J_all)
        sig = max(1.4826 * np.median(np.abs(r - np.median(r))), 0.03)
        w = np.minimum(1.0, 2.5 * sig / np.maximum(np.abs(r), 1e-12))
        sw = np.sqrt(w)
        # weak priors from the documented ranges
        Jp = np.zeros((len(free), len(free) + 1))
        rp = np.zeros(len(free))
        nom = nominal_pose()
        for c, j in enumerate(free):
            Jp[c, c] = 1.0 / POSE_PRIOR_SD[j]
            rp[c] = (nom[j] - pose[j]) / POSE_PRIOR_SD[j]
        A = np.vstack([J * sw[:, None] / sig, Jp])
        y = np.concatenate([r * sw / sig, rp])
        step = np.linalg.lstsq(A, y, rcond=None)[0]
        if not np.all(np.isfinite(step)):
            return None
        for c, j in enumerate(free):
            pose[j] += step[c]
        e += step[-1] if edge else 0.0
        if np.abs(step[:len(free)]).max() < 1e-8:
            break
    return pose, e, float(np.sqrt(np.mean((r * sw) ** 2))), int(len(r))


def locate_plate(ref, p, records, free=(0, 1, 2, 3, 4, 5), log=None, use=None):
    """The plate's pose from its images under the model p (camera poses and intrinsics). `use`: indices of the
    records to use (default all). Returns pose (6,) or None, and a dict of diagnostics."""
    log = log if log is not None else []
    th = np.array(p[ref.INTR], float)
    recs = [records[i] for i in (use if use is not None else range(len(records)))]
    cams = []
    for rec in recs:
        Rc, oc = ref.camera_pose(p, rec["q"], rec["s"], rec["temp"], rec["tool"])
        img = rec.get("clean")
        if img is None:
            img = ref.despeckle(rec["image"])
            rec["clean"] = img
        cams.append((Rc[0], oc[0], img))
    pts = [detect_bores(img, ref, Rc, oc, th) for Rc, oc, img in cams]
    pose = None
    for k in np.argsort([-len(x) for x in pts]):
        pose = match_pattern(pts[k])
        if pose is not None:
            break
    if pose is None:
        log.append("plate: no bore pattern found")
        return None, dict(log=log)
    info = dict(coarse=pose.copy(), log=log)
    for rnd in range(3):
        views = [(Rc, oc, th, measure_rims(img, ref, pose, Rc, oc, th)) for Rc, oc, img in cams]
        n = sum(len(v[3][0]) for v in views if v[3] is not None)
        if n < 40:
            log.append("plate: {} edge points only".format(n))
            return None, info
        res = fit_pose(ref, views, pose, free=free)
        if res is None:
            return None, info
        pose, e, rms, n = res
        log.append("plate round {}: {} edge points, rms {:.3f} px, edge offset {:+.3f} px".format(rnd, n, rms, e))
    info.update(edge=e, rms=rms, n=n)
    return pose, info
