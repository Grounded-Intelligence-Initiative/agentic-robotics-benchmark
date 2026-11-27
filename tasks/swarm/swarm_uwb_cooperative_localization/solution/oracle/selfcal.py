"""Calibrate the swarm's hardware from ordinary flights, without any ground truth.

  python3 selfcal.py <calib.json> <flight_dir> [<flight_dir> ...]

Nothing ever measured where the drones were, so the calibration is self-consistent:

1. On the pads (the seconds before take-off, while the flow is still blind) every drone's
   position is the surveyed pad. The line-of-sight ranges there give the antenna delays of all
   fourteen radios, the blocked ones a first non-line-of-sight model, and the ToF its offset.
   (A board still warm from the charger adds its own offset there; the joint fit below
   separates the per-flight drift from the fixed delays.)
2. Then alternate: localize every flight with the current calibration (localize.Problem), and
   re-fit the calibration against those estimates, pooled over the flights: on line-of-sight
   ranges, range - estimated distance - each drone's estimated thermal drift = delay_a +
   delay_b + pattern_a(direction in a's body frame) + pattern_b(direction in b's body frame)
   + power(distance), one robust (Huber IRLS) linear least squares over 14 delays, 8 x 8
   pattern coefficients (degree-1 and degree-2 spherical harmonics) and the chipset's
   received-power curve; the blocked ranges' excess as a robust line in the length of path
   inside the racks; each drone's flow scale against its estimated body-frame velocity. The
   hardware itself comes from a joint fit of all flights at once (Joint), which keeps an
   antenna offset that turns with the drone from being absorbed into the positions.
"""

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

import common as C

PAD_SECONDS = 1.5
L_KNOT_S = 10.0      # localize.KNOT_S
# The localizer's strong smoothing rounds off the turns, which would bias the flow scales
# high; the calibration uses a looser prior.
CAL_ACC_SIGMA = 3.0


def _huber_lstsq(A, y, iters=8):
    w = np.ones(len(y))
    for _ in range(iters):
        sw = np.sqrt(w)
        x, *_ = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)
        e = y - A @ x
        s = 1.4826 * np.median(np.abs(e))
        w = np.where(np.abs(e) < 2.0 * s, 1.0, 2.0 * s / np.maximum(np.abs(e), 1e-9))
        w = np.where(np.abs(e) > 8.0 * s, 0.0, w)
    return x, e, s


def _positions(site, node, P_at):
    """Positions of `node` (drone ids < D, beacon ids >= D) at the ranges' instants."""
    D = site.n_drones
    out = np.empty((len(node), 3))
    for nd in np.unique(node):
        m = node == nd
        out[m] = P_at[nd][m] if nd < D else site.anchors[nd]
    return out


def _nlos_fit(site, pa, pb, resid, default=(0.15, 0.3, 0.1)):
    L = site.inside_length(pa, pb)
    nl = L > 0.0
    if nl.sum() < 30:
        return dict(nlos_base=default[0], nlos_per_m=default[1], nlos_fit_sigma=default[2])
    A = np.column_stack([np.ones(nl.sum()), L[nl]])
    coef, _, s = _huber_lstsq(A, resid[nl], iters=10)
    return dict(nlos_base=float(coef[0]), nlos_per_m=float(coef[1]), nlos_fit_sigma=float(s))


def _nlos_fit_racks(Ls, resids):
    """Blocked ranges' excess = base + sum over racks of slope[flight, rack] * length inside the
    rack: one base for all flights, a slope per rack and flight (what a rack holds changes
    between flights). The calibration keeps the base, the median slope (the localizer's prior)
    and the spread about the fit."""
    F, R = len(Ls), Ls[0].shape[1]
    rows, ys = [], []
    for f, (L, e) in enumerate(zip(Ls, resids)):
        nl = L.sum(axis=1) > 0.0
        A = np.zeros((int(nl.sum()), 1 + F * R))
        A[:, 0] = 1.0
        A[:, 1 + f * R: 1 + (f + 1) * R] = L[nl]
        rows.append(A)
        ys.append(e[nl])
    A, y = np.concatenate(rows), np.concatenate(ys)
    used = np.abs(A).sum(axis=0) > 0
    coef = np.zeros(A.shape[1])
    coef[used], _, s = _huber_lstsq(A[:, used], y, iters=10)
    slopes = coef[1:][used[1:]]
    return dict(nlos_base=float(coef[0]), nlos_per_m=float(np.median(slopes)),
                nlos_fit_sigma=float(s))


def pad_calibration(flights):
    """First calibration from the pads: delays, a first NLOS model and the ToF offsets."""
    site = flights[0][0]
    D = site.n_drones
    n_nodes = D + len(site.anchors)
    rows_A, rows_y, pas, pbs, tofs = [], [], [], [], [[] for _ in range(D)]
    for site, meta, uwb, drones, _ in flights:
        # a drone is still on its pad until its flow first reports
        t_off = []
        for i in range(D):
            d = drones[i]
            fv = np.isfinite(d[:, 1])
            t_off.append(min(d[fv, 0].min() if fv.any() else PAD_SECONDS, PAD_SECONDS))
            on = d[:, 0] < t_off[-1] - 0.2
            tofs[i].append(d[on & np.isfinite(d[:, 3]), 3])
        t, a, b, r = uwb[:, 0], uwb[:, 1].astype(int), uwb[:, 2].astype(int), uwb[:, 3]
        lim = np.array(t_off + [np.inf] * len(site.anchors))
        sel = (t < lim[a] - 0.2) & (t < lim[b] - 0.2)
        a, b, r = a[sel], b[sel], r[sel]
        pa = np.array([site.pads[n] if n < D else site.anchors[n] for n in a])
        pb = np.array([site.pads[n] if n < D else site.anchors[n] for n in b])
        A = np.zeros((len(r), n_nodes))
        A[np.arange(len(r)), a] += 1.0
        A[np.arange(len(r)), b] += 1.0
        rows_A.append(A)
        rows_y.append(r - np.linalg.norm(pb - pa, axis=1))
        pas.append(pa)
        pbs.append(pb)
    A, y = np.concatenate(rows_A), np.concatenate(rows_y)
    pa, pb = np.concatenate(pas), np.concatenate(pbs)
    los = ~site.blocked(pa, pb)
    delays, e, s = _huber_lstsq(A[los], y[los])
    # a beacon with no clear line to any pad: start it at the typical delay
    seen = A[los].sum(axis=0) > 0
    delays = np.where(seen, delays, np.median(delays[seen]))
    calib = dict(delays=delays.tolist(), pattern=np.zeros((D, C.N_PATTERN)).tolist(),
                 power=[0.0] * C.N_POWER,
                 los_sigma=float(max(s, 0.05)), flow_scale=[1.0] * D,
                 tof_offset=[float(np.median(np.concatenate(v))) for v in tofs])
    calib.update(_nlos_fit(site, pa, pb, y - A @ delays))
    calib["nlos_fit_sigma"] = max(calib["nlos_fit_sigma"], 0.1)
    return calib


def refit(flights, estimates, calib):
    """Re-fit delays, antenna patterns, the power curve, the NLOS model and the flow scales
    against the current trajectory estimates (positions, yaw and the radios' drift knots)."""
    site = flights[0][0]
    D = site.n_drones
    n_nodes = D + len(site.anchors)
    K = C.N_PATTERN
    rows_A, rows_y, pas, pbs = [], [], [], []
    flow_num, flow_den = np.zeros(D), np.zeros(D)
    for (site, meta, uwb, drones, _), (tn, X, Y, Wk, knot_s) in zip(flights, estimates):
        t, a, b, r = uwb[:, 0], uwb[:, 1].astype(int), uwb[:, 2].astype(int), uwb[:, 3]
        P_at = [np.column_stack([np.interp(t, tn, X[i, :, c]) for c in range(3)])
                for i in range(D)]
        Y_at = [np.interp(t, tn, Y[i]) for i in range(D)]
        pa, pb = _positions(site, a, P_at), _positions(site, b, P_at)
        d = pb - pa
        n = len(t)
        A = np.zeros((n, n_nodes + K * D + C.N_POWER))
        A[np.arange(n), a] += 1.0
        A[np.arange(n), b] += 1.0
        tk = np.arange(Wk.shape[1]) * knot_s
        drift = np.zeros(n)
        for node, vec in ((a, d), (b, -d)):
            dr = node < D
            yaw = np.where(dr, np.array(Y_at)[np.minimum(node, D - 1), np.arange(n)], 0.0)
            basis = C.pattern_basis(vec, yaw)
            for k in range(K):
                A[np.where(dr)[0], n_nodes + K * node[dr] + k] += basis[dr, k]
            for i in range(D):
                m = node == i
                drift[m] += np.interp(t[m], tk, Wk[i])
        A[:, n_nodes + K * D:] = C.power_basis(np.linalg.norm(d, axis=1))
        rows_A.append(A)
        rows_y.append(r - drift - np.linalg.norm(d, axis=1))
        pas.append(pa)
        pbs.append(pb)
        # flow scale: flow = scale * body-frame velocity, over the floor while moving
        for i in range(D):
            rows = drones[i]
            tt = rows[:, 0]
            v = np.gradient(X[i], tn, axis=0)
            vx, vy = np.interp(tt, tn, v[:, 0]), np.interp(tt, tn, v[:, 1])
            yw = np.interp(tt, tn, Y[i])
            c, s = np.cos(yw), np.sin(yw)
            vb = np.column_stack([c * vx + s * vy, -s * vx + c * vy])
            f = rows[:, 1:3]
            ok = np.all(np.isfinite(f), axis=1) & (np.linalg.norm(vb, axis=1) > 0.3)
            flow_num[i] += np.sum(f[ok] * vb[ok])
            flow_den[i] += np.sum(vb[ok] * vb[ok])
    A, y = np.concatenate(rows_A), np.concatenate(rows_y)
    pa, pb = np.concatenate(pas), np.concatenate(pbs)
    los = ~site.blocked(pa, pb)
    x, _, s = _huber_lstsq(A[los], y[los])
    out = dict(calib)
    out.update(delays=x[:n_nodes].tolist(), pattern=x[n_nodes:n_nodes + K * D].reshape(D, K).tolist(),
               power=x[n_nodes + K * D:].tolist(), los_sigma=float(s),
               flow_scale=(flow_num / flow_den).tolist())
    e = y - A @ x
    cuts = np.cumsum([len(v) for v in rows_y])[:-1]
    Ls = [site.inside_lengths(a_, b_) for a_, b_ in zip(pas, pbs)]
    out.update(_nlos_fit_racks(Ls, np.split(e, cuts)))
    return out


def _localize(job):
    folder, calib = job
    import localize as L
    prob = L.Problem(folder, calib)
    prob.acc_sigma = CAL_ACC_SIGMA
    prob.solve()
    return prob.x_final


class Joint(object):
    """All flights' trajectories (with each flight's heading parameters and drift knots) and
    the shared hardware (14 delays, 8 x 8 pattern coefficients, the power curve, 8 flow gains)
    in one robust least squares. The pattern's first harmonic acts like an antenna
    offset that turns with the drone, which a per-flight localizer absorbs into the position;
    only the flow and the gyro, jointly with every flight, tell the two apart."""

    def __init__(self, folders, calib, xs):
        import localize as L
        self.probs = [L.Problem(f, calib) for f in folders]
        for p in self.probs:
            p.acc_sigma = CAL_ACC_SIGMA
        self.xs = list(xs)
        p0 = self.probs[0]
        self.D, self.n_nodes = p0.D, p0.D + len(p0.site.anchors)
        self.K = C.N_PATTERN
        self.ipw = self.n_nodes + self.K * self.D     # power-curve coefficients
        self.np_ = self.ipw + C.N_POWER               # flow gains (1 / scale) after these
        self.G = self.np_ + self.D
        self.nf = p0.NG                               # per-flight parameters
        scale = np.asarray(calib["flow_scale"], dtype=float)
        for p in self.probs:
            fi = p.flow[:, 0].astype(int)
            p.flow_raw = p.flow[:, 2:4] * scale[fi, None]
        self.g = np.concatenate([np.asarray(calib["delays"]), np.asarray(calib["pattern"]).ravel(),
                                 np.asarray(calib.get("power", [0.0] * C.N_POWER)), 1.0 / scale])
        self.g0 = self.g.copy()
        self.prior_sigma = np.concatenate([np.full(self.n_nodes, 0.5), np.full(self.K * self.D, 0.3),
                                           np.full(C.N_POWER, 0.3), np.full(self.D, 0.2)])
        self._apply(self.g)

    def _apply(self, g):
        for p in self.probs:
            p.set_hardware(g[: self.n_nodes], g[self.n_nodes: self.ipw].reshape(self.D, self.K),
                           g[self.ipw: self.np_])
            fi = p.flow[:, 0].astype(int)
            p.flow[:, 2:4] = p.flow_raw * g[self.np_ + fi, None]

    def hardware_jacobian(self, p, x, n_rows):
        """d residual / d (delays, pattern, power curve, flow gains) for one flight: the range
        rows and the flow rows."""
        from scipy.sparse import coo_matrix
        n = len(p.r_t)
        sig = p.range_sigma()
        rows, cols, vals = [], [], []
        ridx = np.arange(n)
        for node, basis in ((p.r_a, p.r_basis[0]), (p.r_b, p.r_basis[1])):
            rows.append(ridx); cols.append(node); vals.append(-1.0 / sig)
            dr = node < self.D
            for k in range(self.K):
                rows.append(ridx[dr]); cols.append(self.n_nodes + self.K * node[dr] + k)
                vals.append(-basis[dr, k] / sig[dr])
        for k in range(C.N_POWER):
            rows.append(ridx); cols.append(np.full(n, self.ipw + k)); vals.append(-p.r_pbasis[:, k] / sig)
        X = p.unpack(x)[0]
        fi = p.flow[:, 0].astype(int)
        fk = p.flow[:, 1].astype(int)
        fs = 0.045 * (1 + 0.3 * np.maximum(X[fi, fk, 2], 0.3)) / np.sqrt(p.flow[:, 4])
        nf = len(fi)
        for comp in range(2):
            rows.append(n + comp * nf + np.arange(nf)); cols.append(self.np_ + fi)
            vals.append(-p.flow_raw[:, comp] / fs)
        return coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                          shape=(n_rows, self.G)).tocsr()

    def cost(self, rs, g, scale):
        c = sum(float(np.sum(scale ** 2 * np.log1p((r / scale) ** 2))) for r in rs)
        return c + float(np.sum(((g - self.g0) / self.prior_sigma) ** 2))

    def step(self, ws, rs, lam):
        from scipy.sparse import diags
        from scipy.linalg import cho_solve_banded, cholesky_banded
        F, D, G, NF = len(self.probs), self.D, self.G, self.nf
        Z = F * NF + G
        E = np.zeros((Z, Z))
        gz = np.zeros(Z)
        gi = slice(F * NF, Z)
        E[gi, gi] += np.diag(1.0 / self.prior_sigma ** 2)
        gz[gi] += (self.g - self.g0) / self.prior_sigma ** 2
        per = []
        for f, (p, x, w, r) in enumerate(zip(self.probs, self.xs, ws, rs)):
            J = p.jacobian(x)
            Jg = self.hardware_jacobian(p, x, J.shape[0])
            W = diags(w)
            perm = p._perm()
            n = D * p.M * 3
            JW = (J.T @ W).tocsr()
            A = (JW @ J).tocsr()[perm][:, perm].tocsc()
            Ag = (JW @ Jg).toarray()[perm]
            gx = (JW @ r)[perm]
            idx = np.concatenate([np.arange(f * NF, (f + 1) * NF), np.arange(F * NF, Z)])
            E[gi, gi] += (Jg.T @ W @ Jg).toarray()
            gz[gi] += Jg.T @ (w * r)
            Ef = A[n:, n:].toarray()
            E[np.ix_(idx[:NF], idx[:NF])] += Ef
            E[np.ix_(idx[:NF], idx[NF:])] += Ag[n:]
            E[np.ix_(idx[NF:], idx[:NF])] += Ag[n:].T
            gz[idx[:NF]] += gx[n:]
            Bm = A[:n, :n]
            Bm = (Bm + diags(lam * Bm.diagonal() + 1e-9)).tocoo()
            low = Bm.row >= Bm.col
            u = int(np.max(Bm.row[low] - Bm.col[low]))
            ab = np.zeros((u + 1, n))
            ab[Bm.row[low] - Bm.col[low], Bm.col[low]] = Bm.data[low]
            cb = cholesky_banded(ab, lower=True)
            Cf = np.hstack([A[:n, n:].toarray(), Ag[:n]])
            BiC = cho_solve_banded((cb, True), Cf)
            Big = cho_solve_banded((cb, True), gx[:n])
            E[np.ix_(idx, idx)] -= Cf.T @ BiC
            gz[idx] -= Cf.T @ Big
            per.append((perm, n, idx, BiC, Big))
        E += np.diag(lam * np.maximum(np.diag(E), 1e-9))
        dz = np.linalg.solve(E, -gz)
        steps = []
        for (perm, n, idx, BiC, Big) in per:
            dx = -(Big + BiC @ dz[idx])
            out = np.empty(len(perm))
            out[perm] = np.concatenate([dx, dz[idx[:NF]]])
            steps.append(out)
        return steps, dz[gi]

    def irls(self, iters, scale=2.0):
        lam = 1e-3
        rs = [p.residuals(x) for p, x in zip(self.probs, self.xs)]
        cost = self.cost(rs, self.g, scale)
        for _ in range(iters):
            ws = [1.0 / (1.0 + (r / scale) ** 2) for r in rs]
            steps, dg = self.step(ws, rs, lam)
            xs = [x + s for x, s in zip(self.xs, steps)]
            g = self.g + dg
            self._apply(g)
            rn = [p.residuals(x) for p, x in zip(self.probs, xs)]
            cn = self.cost(rn, g, scale)
            if cn < cost:
                self.xs, self.g, rs, cost = xs, g, rn, cn
                lam = max(lam / 3.0, 1e-7)
                if max(np.max(np.abs(s[: self.D * p.M * 3])) for s, p in zip(steps, self.probs)) < 1e-4:
                    break
            else:
                self._apply(self.g)
                lam *= 5.0
        return cost

    def solve(self, rounds=4, iters=12):
        for _ in range(rounds):
            for p, x in zip(self.probs, self.xs):
                p.linearisation_terms(x)
            self._apply(self.g)
            cost = self.irls(iters)
            pat = self.g[self.n_nodes: self.ipw].reshape(self.D, self.K)
            print("[selfcal]   joint cost %.1f" % cost, "delays", np.round(self.g[: self.n_nodes], 3),
                  "pattern", np.round(pat, 3).tolist(), "power", np.round(self.g[self.ipw: self.np_], 4),
                  "flow", np.round(1.0 / self.g[self.np_:], 3), flush=True)

    def estimates(self):
        out = []
        for p, x in zip(self.probs, self.xs):
            X, psi, bg, Wk, _ = p.unpack(x)
            out.append((p.tn, X, p.yaw(psi, bg), Wk, L_KNOT_S))
        return out


def main():
    out, folders = sys.argv[1], sys.argv[2:]
    flights = [C.load_flight(f) for f in folders]
    calib = pad_calibration(flights)
    print("[selfcal] pads: delays", np.round(calib["delays"], 3), "tof",
          np.round(calib["tof_offset"], 3), flush=True)
    workers = int(os.environ.get("SELFCAL_WORKERS", "2"))
    with Pool(workers) as pool:
        xs = pool.map(_localize, [(f, calib) for f in folders])
    for rnd in range(2):
        joint = Joint(folders, calib, xs)
        joint.solve()
        xs = joint.xs
        g = joint.g
        calib = refit(flights, joint.estimates(), calib)
        # keep the joint estimate of the hardware; take the non-line-of-sight model and the
        # noise levels from the re-fit
        calib.update(delays=g[: joint.n_nodes].tolist(),
                     pattern=g[joint.n_nodes: joint.ipw].reshape(joint.D, joint.K).tolist(),
                     power=g[joint.ipw: joint.np_].tolist(),
                     flow_scale=(1.0 / g[joint.np_:]).tolist())
        print("[selfcal] pass", rnd + 1, "nlos %.3f + %.3f/m" % (calib["nlos_base"], calib["nlos_per_m"]),
              "flow", np.round(calib["flow_scale"], 3), "los sigma %.3f" % calib["los_sigma"], flush=True)
    with open(out, "w") as f:
        json.dump(calib, f, indent=1)


if __name__ == "__main__":
    main()
