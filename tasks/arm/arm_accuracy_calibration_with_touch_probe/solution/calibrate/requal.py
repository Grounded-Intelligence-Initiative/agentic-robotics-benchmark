"""requal.py: the reference's re-qualification session after a unit's service (part of the reference solution).

What it uses is public: cell_data.py (the service's documented set of joints and bound of the zero offset's change,
the backlash's range, the re-qualification budget), the model and the plate that reference.py made of the unit in
its measuring session (carried over in memory), and the five commands of the session. Nothing here reads the cell.

The service changed one joint's zero offset (by at most SERVICE_ZERO[1]) and its backlash (anywhere in its
documented range); which joint is not told. So:
  * every hypothesis (a joint of SERVICE_JOINTS, a zero offset change and a backlash in their ranges) is a model:
    the measured model with that joint's third rpy angle moved and its backlash replaced;
  * safety: every move and every probe path is checked against every corner of the hypotheses still alive (the
    extreme zero offset changes and backlashes of every joint not yet ruled out): clear of every sphere, post, the
    table and the plate's documented region by the measuring campaign's margins, the touched sphere approached to a
    standoff larger than the largest displacement any corner predicts there, the probe aimed as far beyond the
    surface, so that the stylus meets nothing before the probe and the probe finds the sphere;
  * the touches: chosen one at a time from the measuring campaign's candidate pool (probe_a, designed directions)
    for what they tell about the zero offset and the backlash of every joint still alive, near where the arm is;
  * after every touch, each joint's (zero offset, backlash) fitted to the touches (Gauss-Newton, weak priors from
    the documented ranges, a touch that stopped well outside the surface, a false trigger, set aside), the joints
    whose fit is much worse than the best ruled out, the hypotheses' ranges narrowed to 4 standard deviations;
  * the serviced job: the best hypothesis's model, the plate as located in the measuring session, the job planned
    cold by plate_job's planner with the session's thermal fit.
"""
import numpy as np

import arm_model as am
import reference as ref
from cell_data import (SERVICE_JOINTS, SERVICE_ZERO, BACKLASH_MAX, R_POST, R_STYLUS, Q_HOME, PROBE_SPEED,
                       SETTLE, MOVE_SPEED_MAX)

SL = am.SLICES
TOOL = ref.TOOL_A
SIGMA_TOUCH = 40e-6             # m: a touch's residual under the right hypothesis (repeatability, the model's error)
MARGIN = ref.DISPLACED_MAX + 2.5e-3     # m: clearance kept under every corner model to everything but the target
MARGIN_TARGET = 1.0e-3          # m: ... to the touched sphere (its centre as the measuring session estimated it)
STANDOFF = 1.5e-3               # m: standoff beyond the largest predicted displacement
DEPTH = 1.0e-3                  # m: the probe aimed this far inside the surface beyond the largest displacement
RULE_OUT = 25.0                 # chi-square excess over the best joint above which a joint is ruled out
FALSE_CUT = 4.0                 # a touch whose residual is this many sigma outside the surface is set aside
MAX_FALSE = 6                   # ... at most this many per fit (only touches outside the surface: a wrong joint
#                                 cannot fit its way out by setting aside the touches it puts inside)
MIN_KEEP = 4                    # ... keeping at least this many touches
MIN_RULE_OUT = 6                # touches before any joint is ruled out
MIN_NARROW = 5                  # touches a joint's fit must use before its ranges are narrowed for the safety bound
TOUCHES = 12                    # touches planned at most (every probe of the budget)


def zero_index(j):
    return SL["rpy"].start + 3 * j + 2


def back_index(j):
    return SL["b"].start + j


def hypothesis(p, j, dz, b):
    q = np.array(p, float)
    q[zero_index(j)] += dz
    q[back_index(j)] = b
    return q


class Hypotheses(object):
    """For every joint of SERVICE_JOINTS still alive: the range of its zero offset change and of its backlash."""

    def __init__(self, p):
        self.p = np.array(p, float)
        self.alive = list(SERVICE_JOINTS)
        self.zr = {j: (-SERVICE_ZERO[1], SERVICE_ZERO[1]) for j in SERVICE_JOINTS}
        self.br = {j: (0.2 * BACKLASH_MAX[j], 1.0 * BACKLASH_MAX[j]) for j in SERVICE_JOINTS}
        self.fit = {}

    def corners(self):
        out = []
        for j in self.alive:
            for dz in self.zr[j]:
                for b in self.br[j]:
                    out.append(hypothesis(self.p, j, dz, b))
        return out


def clearance_all(tips, centres, k=None):
    """Clearance (m) of the stylus at tips to everything less the margins: the surveyed spheres and posts, the
    table and the plate's region (MARGIN); with a target sphere k, its estimated centre (MARGIN_TARGET) in place
    of its survey, its post at MARGIN_TARGET."""
    cols = ref.clearance_cols(tips)
    n = len(ref.SPHERES)
    req = np.full(cols.shape[1], MARGIN)
    if k is not None:
        cols[:, k] = np.linalg.norm(tips - centres[k], axis=1) - ref.R_TOUCH
        req[k] = MARGIN_TARGET
        c = centres[k]
        a = np.array([c[0], c[1], 0.0])
        ab = c - a
        t = np.clip(((tips - a) @ ab) / (ab @ ab), 0.0, 1.0)
        cols[:, n + k] = np.linalg.norm(tips - (a + t[:, None] * ab), axis=1) - (R_POST + R_STYLUS)
        req[n + k] = MARGIN_TARGET
    return (cols - req[None]).min(axis=1)


class Requal(object):
    def __init__(self, cell, info, say=None):
        self.rb = ref.Robot(cell)
        self.info = info
        self.p0 = np.array(info["models"][0]["p"] if info.get("models") else info["p_base"], float)
        self.centres = np.array(info["centres"], float)
        self.hyp = Hypotheses(self.p0)
        self.data = []
        self.say = say or (lambda *a: None)
        self.log = []
        pool = ref.design_pool()[TOOL]
        keep = (pool["n"][:, 2] > -0.1)
        self.pool = {key: (v[keep] if isinstance(v, np.ndarray) and len(v) == len(keep) else v) for key, v in pool.items()}
        self.used = np.zeros(len(self.pool["k"]), bool)
        h = type("H", (), {})()
        self.hubs = ref.Campaign.hubs(h)

    # -- safety under every corner model --
    def path_ok(self, qa, qb, k=None, relax=False):
        """The joint-linear move qa -> qb clear under every corner model (k: the target sphere)."""
        qq = ref.path_samples(qa, qb)
        s_prev = np.nan_to_num(self.rb.s, nan=1.0)
        s = ref.motion_dirs(qa, qb, s_prev)
        for p in self.hyp.corners():
            c = clearance_all(ref.tip(p, qq, s, self.rb.temp, TOOL), self.centres, k)
            if relax:
                c = c[np.argmax(c > 0):] if np.any(c > 0) else c
                if np.any(np.diff(c) < 0) and c.min() < 0:
                    return False
            elif c.min() < 0:
                return False
        return True

    def travel(self, q_goal):
        q0 = self.rb.q.copy()
        routes = [[q_goal], [Q_HOME, q_goal]] + [[h, q_goal] for h in self.hubs] + [[Q_HOME, h, q_goal] for h in self.hubs]
        for route in routes:
            pts = [q0] + route
            if all(self.path_ok(a, b) for a, b in zip(pts[:-1], pts[1:])):
                for q in route:
                    self.rb.move(q)
                return True
        return False

    def bound(self, q, s):
        """The largest displacement (m) of the stylus at q any corner model predicts against the measured model."""
        t0 = ref.tip(self.p0, q[None], s, self.rb.temp, TOOL)[0]
        return max(np.linalg.norm(ref.tip(p, q[None], s, self.rb.temp, TOOL)[0] - t0) for p in self.hyp.corners())

    # -- one touch --
    def plan_touch(self, i):
        c = self.pool
        k, n, Rd, pre = int(c["k"][i]), c["n"][i], c["Rd"][i], c["pre"][i]
        ctr = self.centres[k]
        T = self.rb.temp
        q5, ok = ref.ik_pose(self.p0, (ctr + n * (ref.R_TOUCH + 0.005))[None], Rd[None], c["qs"][i][None], pre, T, TOOL)
        if not ok[0]:
            return None
        E = self.bound(q5[0], pre)
        d_s, d_t = E + STANDOFF, E + DEPTH
        qS, ok = ref.ik_pose(self.p0, (ctr + n * (ref.R_TOUCH + d_s))[None], Rd[None], q5, pre, T, TOOL, iters=20)
        if not ok[0]:
            return None
        qS = qS[0]
        d_f = d_s + 10e-3
        qF, ok = ref.ik_pose(self.p0, (ctr + n * (ref.R_TOUCH + d_f))[None], Rd[None], qS[None], pre, T, TOOL, iters=20)
        if not ok[0] or np.abs(qF[0] - qS).max() > 0.6:
            return None
        qF = qF[0]
        qE, ok = ref.ik_pose(self.p0, (ctr + n * (ref.R_TOUCH + d_f + 40e-3))[None], Rd[None], qF[None], pre, T, TOOL,
                             iters=20)
        if not ok[0] or np.abs(qE[0] - qF).max() > 0.8:
            return None
        qE = qE[0]
        qT = ref.ik_position(self.p0, (ctr + n * (ref.R_TOUCH - d_t))[None], qS, pre, T, TOOL)[0]
        hold = np.abs(qT - qS) < 2.5e-4
        qT[hold] = qS[hold]
        qP = qS - pre * ref.PRELOAD
        if np.any(qT < am.Q_MIN) or np.any(qT > am.Q_MAX) or np.any(qP < am.Q_MIN) or np.any(qP > am.Q_MAX):
            return None
        return dict(i=i, k=k, n=n, qE=qE, qF=qF, qP=qP, qS=qS, qT=qT, pre=pre, E=E,
                    s=ref.motion_dirs(qS, qT, pre))

    def probe_clear(self, t):
        """The probe's path qS -> qT under every corner model: nothing but the target sphere within the margins
        (its post at MARGIN_TARGET) up to where the sphere is met."""
        qq = ref.path_samples(t["qS"], t["qT"], 0.0005)
        for p in self.hyp.corners():
            tips = ref.tip(p, qq, t["s"], self.rb.temp, TOOL)
            hit = np.linalg.norm(tips - self.centres[t["k"]], axis=1) - ref.R_TOUCH <= 0
            if not hit.any():
                return False                       # this corner would miss the sphere: aim deeper
            upto = tips[:np.argmax(hit)]
            if len(upto):
                cols = ref.clearance_cols(upto)
                cols[:, t["k"]] = np.inf
                req = np.full(cols.shape[1], MARGIN)
                req[len(ref.SPHERES) + t["k"]] = MARGIN_TARGET
                if np.any(cols < req[None]):
                    return False
        return True

    def touch(self, t):
        rb = self.rb
        if not (self.path_ok(t["qE"], t["qF"], k=t["k"]) and self.path_ok(t["qF"], t["qP"], k=t["k"])
                and self.path_ok(t["qP"], t["qS"], k=t["k"]) and self.probe_clear(t)):
            return "unsafe"
        need = np.abs(t["qT"] - t["qS"]).max() / PROBE_SPEED + 8.0
        if rb.time_left < need + 10 or rb.probes_left < 1:
            return "budget"
        if not self.travel(t["qE"]):
            return "no way"
        rb.move(t["qF"])
        rb.move(t["qP"])
        rb.move(t["qS"])
        obs = rb.probe(t["qT"])
        rec = dict(q=rb.q.copy(), s=rb.s.copy(), temp=rb.temp.copy(), k=t["k"], triggered=bool(obs["triggered"]))
        self.data.append(rec)
        rb.move(t["qF"])
        rb.move(t["qE"])
        return "touch" if rec["triggered"] else "miss"

    # -- estimation --
    def residuals(self, p, recs):
        q = np.array([r["q"] for r in recs])
        s = np.array([r["s"] for r in recs])
        T = np.array([r["temp"] for r in recs])
        tips = ref.tip(p, q, s, T, TOOL)
        return np.linalg.norm(tips - self.centres[[r["k"] for r in recs]], axis=1) - ref.R_TOUCH

    def fit_joint(self, j, recs):
        """(zero offset change, backlash) of joint j fitted to the touches: Gauss-Newton with weak priors (the
        documented ranges as Gaussians). Returns x (2,), its covariance, chi-square, the residuals (m)."""
        mu = np.array([0.0, 0.6 * BACKLASH_MAX[j]])
        sd = np.array([SERVICE_ZERO[1], 0.4 * BACKLASH_MAX[j] / np.sqrt(3.0)])
        lo = np.array([-SERVICE_ZERO[1], 0.2 * BACKLASH_MAX[j]])     # the documented ranges, held as bounds
        hi = np.array([SERVICE_ZERO[1], 1.0 * BACKLASH_MAX[j]])
        x = mu.copy()
        h = np.array([1e-6, 1e-6])
        for _ in range(8):
            r = self.residuals(hypothesis(self.p0, j, *x), recs)
            J = np.empty((len(r), 2))
            for c in range(2):
                dx = x.copy()
                dx[c] += h[c]
                J[:, c] = (self.residuals(hypothesis(self.p0, j, *dx), recs) - r) / h[c]
            A = np.vstack([J / SIGMA_TOUCH, np.diag(1.0 / sd)])
            y = np.concatenate([-r / SIGMA_TOUCH, (mu - x) / sd])
            step = np.linalg.lstsq(A, y, rcond=None)[0]
            x_new = np.clip(x + step, lo, hi)
            pinned = (x_new != x + step)
            if pinned.any() and not pinned.all():
                # a parameter at its bound: the other fitted with it held there
                c = int(np.nonzero(~pinned)[0][0])
                st = np.linalg.lstsq(A[:, [c]], y - A[:, [1 - c]] @ (x_new - x)[[1 - c]], rcond=None)[0][0]
                x_new[c] = np.clip(x[c] + st, lo[c], hi[c])
            step = x_new - x
            x = x_new
            if np.abs(step[0]) < 1e-9 and np.abs(step[1]) < 1e-9:
                break
        r = self.residuals(hypothesis(self.p0, j, *x), recs)
        Cov = np.linalg.inv(A.T @ A)
        chi2 = float(np.sum((r / SIGMA_TOUCH) ** 2) + np.sum(((x - mu) / sd) ** 2))
        return x, Cov, chi2, r

    def update(self):
        recs = [r for r in self.data if r["triggered"] and not r.get("false")]
        if not recs:
            return
        fits = {}
        for j in SERVICE_JOINTS:
            # a false trigger stops the probe short of the surface: under the right joint's fit its touch lies
            # outside; the touch farthest outside is set aside while it is more than FALSE_CUT sigma out (at most
            # MAX_FALSE, keeping MIN_KEEP), each at the cost FALSE_CUT^2 in the comparison of the joints
            use = list(recs)
            while True:
                x, Cov, chi2, r = self.fit_joint(j, use)
                z = r / SIGMA_TOUCH
                if len(use) <= MIN_KEEP or len(recs) - len(use) >= MAX_FALSE or z.max() < FALSE_CUT:
                    break
                worst = int(np.argmax(z))
                use = use[:worst] + use[worst + 1:]
            fits[j] = (x, Cov, chi2 + FALSE_CUT ** 2 * (len(recs) - len(use)), len(use),
                       float(np.abs(z).max()) if len(z) else 0.0)
        best = min(fits.values(), key=lambda f: f[2])[2]
        # a joint is ruled out only once enough touches make its fit's failure plain (a false trigger among the
        # first touches can make the right joint look wrong until it is set aside)
        if len(recs) >= MIN_RULE_OUT:
            self.hyp.alive = [j for j in SERVICE_JOINTS if fits[j][2] - best < RULE_OUT]
        for j in SERVICE_JOINTS:
            x, Cov, chi2, n_used, zmax = fits[j]
            if j not in self.hyp.alive or n_used < MIN_NARROW or zmax > FALSE_CUT:
                # too few touches to have found a false trigger, or a fit the touches do not bear out: the
                # documented ranges stay the safety bound's
                self.hyp.zr[j] = (-SERVICE_ZERO[1], SERVICE_ZERO[1])
                self.hyp.br[j] = (0.2 * BACKLASH_MAX[j], 1.0 * BACKLASH_MAX[j])
                continue
            sz, sb = np.sqrt(np.diag(Cov))
            zr = (max(-SERVICE_ZERO[1], x[0] - 4 * sz), min(SERVICE_ZERO[1], x[0] + 4 * sz))
            br = (max(0.2 * BACKLASH_MAX[j], x[1] - 4 * sb), min(1.0 * BACKLASH_MAX[j], x[1] + 4 * sb))
            if zr[0] < zr[1]:
                self.hyp.zr[j] = zr
            if br[0] < br[1]:
                self.hyp.br[j] = br
        self.hyp.fit = fits

    # -- the choice of the next touch --
    def score(self, t):
        """What the touch tells: for every joint alive, the gain in the log determinant of its (zero, backlash)
        information; less a little for the joint travel to reach it."""
        g = 0.0
        q, s = t["qS"], t["s"]
        for j in self.hyp.alive:
            x = self.hyp.fit[j][0] if j in self.hyp.fit else np.array([0.0, 0.6 * BACKLASH_MAX[j]])
            p = hypothesis(self.p0, j, *x)
            rec = dict(q=q, s=s, temp=self.rb.temp, k=t["k"])
            r0 = self.residuals(p, [rec])[0]
            row = []
            for c, hh in ((0, 1e-6), (1, 1e-6)):
                dx = x.copy()
                dx[c] += hh
                row.append((self.residuals(hypothesis(self.p0, j, *dx), [rec])[0] - r0) / hh)
            row = np.array(row) / SIGMA_TOUCH
            if j in self.hyp.fit:
                M = np.linalg.inv(self.hyp.fit[j][1])
            else:
                M = np.diag(1.0 / np.array([SERVICE_ZERO[1], 0.4 * BACKLASH_MAX[j] / np.sqrt(3.0)]) ** 2)
            g += np.linalg.slogdet(M + np.outer(row, row))[1] - np.linalg.slogdet(M)[1]
        return g - 0.3 * np.abs(t["qS"] - self.rb.q).max()

    def next_touch(self, n_eval=60, rng=None):
        c = self.pool
        free = np.nonzero(~self.used)[0]
        here = self.rb.q
        # candidates near the arm's present pose first
        dist = np.abs(c["qs"][free] - here).max(axis=1)
        order = free[np.argsort(dist)][:n_eval * 3]
        best = None
        tried = 0
        for i in order:
            if tried >= n_eval:
                break
            t = self.plan_touch(int(i))
            if t is None:
                self.used[i] = True
                continue
            tried += 1
            sc = self.score(t)
            if best is None or sc > best[0]:
                best = (sc, t)
        return None if best is None else best[1]

    def run(self):
        while len(self.data) < TOUCHES and self.rb.probes_left > 0 and not self.rb.damaged:
            t = self.next_touch()
            if t is None:
                self.log.append("no touch left")
                break
            self.used[t["i"]] = True
            what = self.touch(t)
            self.log.append("touch sphere {} ({}): {}; bound {:.2f} mm; alive {}".format(
                t["k"], t["i"], what, t["E"] * 1e3, self.hyp.alive))
            self.say(self.log[-1])
            if what in ("budget",):
                break
            if what in ("touch", "miss"):
                self.update()
        return self.result()

    def result(self):
        if not self.hyp.fit:
            return None
        j = min(self.hyp.alive, key=lambda jj: self.hyp.fit[jj][2])
        x = self.hyp.fit[j][0]
        return dict(joint=j, zero=float(x[0]), backlash=float(x[1]), p=hypothesis(self.p0, j, *x),
                    alive=list(self.hyp.alive), fits={jj: (self.hyp.fit[jj][0].tolist(), self.hyp.fit[jj][2])
                                                     for jj in self.hyp.fit}, touches=len(self.data))


def requalify(cell, info, say=None):
    """The re-qualification session on `cell` (the serviced unit) with the measuring session's `info`; the serviced
    job planned. Returns (rows or None, estimate dict or None, log)."""
    rq = Requal(cell, info, say)
    est = None
    try:
        est = rq.run()
    except RuntimeError as err:          # should not happen (a collision raises in ref.Robot.move)
        rq.log.append("re-qualification stopped: {}".format(err))
        est = rq.result()
    info["requal"] = dict(log=rq.log, data=rq.data, estimate=est, time_left=rq.rb.time_left,
                          probes_left=rq.rb.probes_left, damaged=rq.rb.damaged)
    if est is None or info.get("plate_pose") is None:
        return None, est, rq.log
    rows, _ = ref.job_program(info, p=est["p"], plate=info["plate_pose"])
    return rows, est, rq.log
