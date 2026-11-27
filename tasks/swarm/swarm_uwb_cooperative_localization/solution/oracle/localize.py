"""Localize every drone of one flight; the reference method.

  python3 localize.py <flight_dir> <out_dir>     (calib.json beside this file)

1. Per drone, a robust smoothing fit to the anchor ranges and the ToF height (the start pad
   pins the first seconds) gives a first trajectory.
2. Per drone, the heading offset and gyro bias that best rotate the scale-corrected optical
   flow onto that trajectory's velocity.
3. One joint robust least squares over all drones' 20 Hz positions, their heading parameters
   and each drone radio's slowly varying delay drift in this flight (the board temperature:
   piecewise linear on a 10 s grid, small at the start, smooth) and each rack's excess range
   per metre of path in this flight (what it holds changes from flight to flight): every range (delays, the 3-D antenna patterns and the
   received-power bias removed, line of sight re-predicted from the current estimate; ranges
   predicted through a rack carry the calibrated NLOS excess and a wide sigma), the flow, the
   ToF over the surface below, a constant-velocity smoothness prior and the pads.
   Re-linearised three times.
"""

import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix, diags, lil_matrix
from scipy.linalg import cho_solve_banded, cholesky_banded

import common as C

RATE = 20.0          # state grid (Hz); the output is every second state, at 10 Hz
ACC_SIGMA = 0.5      # smoothness prior on the acceleration (m/s^2 per state)
TOF_SIGMA = 0.01     # ToF noise of one state's median (m)
KNOT_S = 10.0        # delay-drift knots (s)
DRIFT_START = 0.06   # prior on the drift at t = 0, relative to the calibrated delays (m)
DRIFT_STEP = 0.10    # prior on the change between knots (m)
DRIFT_BEND = 0.03    # prior on the second difference of the knots (m)
STOCK_SIGMA = 0.3    # prior on a rack's excess per metre around the calibrated mean (m/m)


def load_calib():
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")) as f:
        return json.load(f)


class Problem(object):
    def __init__(self, folder, calib):
        self.site, self.meta, uwb, self.drones, _ = C.load_flight(folder)
        self.cal = calib
        self.acc_sigma = ACC_SIGMA
        self.D = self.site.n_drones
        self.T = float(self.meta["duration_s"])
        self.M = int(round(self.T * RATE))
        self.tn = np.arange(self.M) / RATE
        self.uwb = uwb
        self.delays = np.array(calib["delays"])
        self.pattern = np.array(calib["pattern"])
        self.power = np.array(calib.get("power", [0.0] * C.N_POWER))
        self.NK = int(math.ceil(self.T / KNOT_S - 1e-9)) + 1
        self.R = len(self.site.racks)
        # per drone: heading offset, gyro bias, drift knots; then each rack's excess per metre
        self.NG = 2 * self.D + self.D * self.NK + self.R
        self.G =np.array([C.integrated_gyro(self.drones[i], self.tn) for i in range(self.D)])
        self.prep_flow()
        self.prep_tof()
        self.prep_ranges()
        self.prep_pads()

    # -- measurement bookkeeping ------------------------------------------------------- #
    def prep_flow(self):
        """Mean scale-corrected body-frame flow over each 10 Hz interval with valid samples."""
        rows = []
        for i in range(self.D):
            d = self.drones[i]
            k = np.floor(d[:, 0] * RATE + 1e-9).astype(int)
            f = d[:, 1:3] / self.cal["flow_scale"][i]
            ok = np.all(np.isfinite(f), axis=1) & (k < self.M - 1)
            for kk in np.unique(k[ok]):
                m = ok & (k == kk)
                rows.append((i, kk, f[m, 0].mean(), f[m, 1].mean(), m.sum()))
        self.flow = np.array(rows)

    def prep_pads(self):
        """The states while each drone still sits on its surveyed pad: before its ToF first
        reads clearly above the pad (take-off), with a margin."""
        ii, kk = [], []
        for i in range(self.D):
            d = self.drones[i]
            h = d[:, 3] - self.cal["tof_offset"][i]
            up = np.nonzero(np.isfinite(h) & (h > 0.06))[0]
            t_up = d[up[0], 0] if len(up) else 0.0
            n = max(1, int(np.floor((t_up - 0.3) * RATE)) + 1)
            ii.append(np.full(n, i))
            kk.append(np.arange(n))
        self.pad_i, self.pad_k = np.concatenate(ii), np.concatenate(kk)

    def prep_tof(self):
        """Median ToF per 10 Hz node, and whether the flow was valid there (flow works only
        over the textured floor, so a valid flow means the ToF sees the floor)."""
        rows = []
        for i in range(self.D):
            d = self.drones[i]
            k = np.round(d[:, 0] * RATE).astype(int)
            ok = np.isfinite(d[:, 3]) & (k < self.M)
            fv = np.isfinite(d[:, 1])
            for kk in np.unique(k[ok]):
                m = ok & (k == kk)
                rows.append((i, kk, np.median(d[m, 3]) - self.cal["tof_offset"][i],
                             float(np.mean(fv[m]) > 0.5)))
        self.tof = np.array(rows)

    def prep_ranges(self):
        t, a, b, r = self.uwb[:, 0], self.uwb[:, 1].astype(int), self.uwb[:, 2].astype(int), \
            self.uwb[:, 3]
        self.r_t, self.r_a, self.r_b = t, a, b
        self.r_meas = r - self.delays[a] - self.delays[b]
        u = t * RATE
        self.r_k = np.minimum(np.floor(u).astype(int), self.M - 2)
        self.r_f = np.clip(u - self.r_k, 0.0, 1.0)
        v = t / KNOT_S
        self.r_j = np.minimum(np.floor(v).astype(int), self.NK - 2)
        self.r_g = np.clip(v - self.r_j, 0.0, 1.0)

    # -- state helpers ---------------------------------------------------------------- #
    def unpack(self, x):
        n = self.D * self.M * 3
        X = x[:n].reshape(self.D, self.M, 3)
        psi = x[n: n + self.D]
        bg = x[n + self.D: n + 2 * self.D]
        ns = n + 2 * self.D + self.D * self.NK
        Wk = x[n + 2 * self.D: ns].reshape(self.D, self.NK)
        return X, psi, bg, Wk, x[ns: n + self.NG]

    def drift(self, Wk, node):
        """Delay drift of each range's node in this flight (beacons: none)."""
        dr = node < self.D
        nd = np.minimum(node, self.D - 1)
        v = Wk[nd, self.r_j] * (1 - self.r_g) + Wk[nd, self.r_j + 1] * self.r_g
        return np.where(dr, v, 0.0)

    def yaw(self, psi, bg):
        return psi[:, None] + self.G - bg[:, None] * self.tn[None, :]

    def node_positions(self, X, node, k, f):
        out = np.empty((len(node), 3))
        dr = node < self.D
        nd = node[dr]
        out[dr] = X[nd, k[dr]] * (1 - f[dr, None]) + X[nd, k[dr] + 1] * f[dr, None]
        for an, p in self.site.anchors.items():
            out[node == an] = p
        return out

    def linearisation_terms(self, x):
        """Pattern corrections, line-of-sight flags and surfaces from the current estimate."""
        X, psi, bg, _, stock = self.unpack(x)
        yaw = self.yaw(psi, bg)
        pa = self.node_positions(X, self.r_a, self.r_k, self.r_f)
        pb = self.node_positions(X, self.r_b, self.r_k, self.r_f)
        d = pb - pa
        self.r_basis = []
        for node, vec in ((self.r_a, d), (self.r_b, -d)):
            dr = node < self.D
            nd = np.minimum(node, self.D - 1)
            yk = yaw[nd, self.r_k] * (1 - self.r_f) + yaw[nd, self.r_k + 1] * self.r_f
            self.r_basis.append(np.where(dr[:, None], C.pattern_basis(vec, yk), 0.0))
        self.r_pbasis = C.power_basis(np.linalg.norm(d, axis=1))
        self.r_corr = self.pattern_corr()
        self.r_L = self.site.inside_lengths(pa, pb)
        self.r_inside = self.r_L.sum(axis=1)
        self.r_blocked = self.r_inside > 0.0
        # (the excess at the last linearisation sets the blocked ranges' sigma)
        self.r_excess = self.excess(stock)
        ti = self.tof[:, 0].astype(int)
        tk = self.tof[:, 1].astype(int)
        # The ToF sees the floor or a rack top. The flow works only over the textured floor,
        # so an airborne drone with a valid flow has the floor below it, and one without has
        # a rack top below it: the top of the rack whose footprint is nearest the estimate.
        x, y = X[ti, tk, 0], X[ti, tk, 1]
        best = self.rack_top_below(x, y)
        best = np.where(self.tof[:, 3] > 0.5, 0.0, best)
        best = np.where(self.tof[:, 2] < 0.3, 0.0, best)
        self.tof_surface = best

    def excess(self, stock):
        """Non-line-of-sight excess of every range for the racks' excess per metre `stock`."""
        return np.where(self.r_blocked, self.cal["nlos_base"] + self.r_L @ stock, 0.0)

    def pattern_corr(self):
        """Antenna-pattern and received-power part of every range, from the directions and
        distances of the last linearisation."""
        na = np.minimum(self.r_a, self.D - 1)
        nb = np.minimum(self.r_b, self.D - 1)
        return (np.sum(self.r_basis[0] * self.pattern[na], axis=1)
                + np.sum(self.r_basis[1] * self.pattern[nb], axis=1)
                + self.r_pbasis @ self.power)

    def set_hardware(self, delays, pattern, power):
        """New antenna delays, patterns and power bias (the directions stay those of the last
        linearisation)."""
        self.delays = np.asarray(delays, dtype=float)
        self.pattern = np.asarray(pattern, dtype=float)
        self.power = np.asarray(power, dtype=float)
        self.r_meas = self.uwb[:, 3] - self.delays[self.r_a] - self.delays[self.r_b]
        if hasattr(self, "r_basis"):
            self.r_corr = self.pattern_corr()

    def range_sigma(self):
        """Blocked ranges: the NLOS fit's spread plus the 20 % variability of the excess."""
        nl = np.sqrt(self.cal["nlos_fit_sigma"] ** 2 + (0.2 * self.r_excess) ** 2)
        return np.where(self.r_blocked, np.maximum(nl, 0.06), max(self.cal["los_sigma"], 0.03))

    def rack_top_below(self, x, y):
        """Top of the rack whose footprint is nearest to each (x, y)."""
        best_d = np.full(len(x), np.inf)
        top = np.zeros(len(x))
        for b in self.site.racks:
            dx = np.maximum(np.maximum(b[0] - x, x - b[1]), 0.0)
            dy = np.maximum(np.maximum(b[2] - y, y - b[3]), 0.0)
            d = np.hypot(dx, dy)
            take = d < best_d
            best_d = np.where(take, d, best_d)
            top = np.where(take, b[5], top)
        return top

    # -- residuals -------------------------------------------------------------------- #
    def residuals(self, x):
        X, psi, bg, Wk, stock = self.unpack(x)
        out = []
        pa = self.node_positions(X, self.r_a, self.r_k, self.r_f)
        pb = self.node_positions(X, self.r_b, self.r_k, self.r_f)
        dist = np.linalg.norm(pb - pa, axis=1)
        expect = (self.r_meas - self.r_corr - self.excess(stock)
                  - self.drift(Wk, self.r_a) - self.drift(Wk, self.r_b))
        sig = self.range_sigma()
        out.append((expect - dist) / sig)
        # flow
        fi = self.flow[:, 0].astype(int)
        fk = self.flow[:, 1].astype(int)
        v = (X[fi, fk + 1] - X[fi, fk]) * RATE
        yaw = self.yaw(psi, bg)
        ym = 0.5 * (yaw[fi, fk] + yaw[fi, fk + 1])
        c, s = np.cos(ym), np.sin(ym)
        vbx = c * v[:, 0] + s * v[:, 1]
        vby = -s * v[:, 0] + c * v[:, 1]
        h = np.maximum(X[fi, fk, 2], 0.3)
        fs = 0.045 * (1 + 0.3 * h) / np.sqrt(self.flow[:, 4])
        out.append((vbx - self.flow[:, 2]) / fs)
        out.append((vby - self.flow[:, 3]) / fs)
        # ToF
        ti = self.tof[:, 0].astype(int)
        tk = self.tof[:, 1].astype(int)
        out.append((X[ti, tk, 2] - self.tof_surface - self.tof[:, 2]) / TOF_SIGMA)
        # smoothness (acceleration) and pads
        acc = (X[:, 2:] - 2 * X[:, 1:-1] + X[:, :-2]) * RATE * RATE
        out.append(acc.ravel() / self.acc_sigma)
        out.append((X[self.pad_i, self.pad_k, :2] - self.site.pads[self.pad_i, :2]).ravel() / 0.05)
        # delay drift: small at the start, smooth changes
        out.append(Wk[:, 0] / DRIFT_START)
        out.append(np.diff(Wk, axis=1).ravel() / DRIFT_STEP)
        out.append(np.diff(Wk, 2, axis=1).ravel() / DRIFT_BEND)
        out.append((stock - self.cal["nlos_per_m"]) / STOCK_SIGMA)
        return np.concatenate(out)

    # -- initialisation ---------------------------------------------------------------- #
    def init_drone(self, i):
        """Robust per-drone fit to anchor ranges + ToF + pad + smoothness."""
        M = self.M
        sel = ((self.r_a == i) & (self.r_b >= self.D)) | ((self.r_b == i) & (self.r_a >= self.D))
        anc = np.where(self.r_a[sel] == i, self.r_b[sel], self.r_a[sel])
        ap = np.array([self.site.anchors[a] for a in anc])
        k, f = self.r_k[sel], self.r_f[sel]
        meas = self.r_meas[sel]
        tsel = (self.tof[:, 0] == i) & ((self.tof[:, 3] > 0.5) | (self.tof[:, 2] < 0.3))
        tk, tv = self.tof[tsel, 1].astype(int), self.tof[tsel, 2]
        over = (self.tof[:, 0] == i) & (self.tof[:, 3] < 0.5) & (self.tof[:, 2] >= 0.3)
        ok_, ov_ = self.tof[over, 1].astype(int), self.tof[over, 2]
        pad = self.site.pads[i]

        def res(z):
            X = z.reshape(M, 3)
            p = X[k] * (1 - f[:, None]) + X[k + 1] * f[:, None]
            r1 = (np.linalg.norm(p - ap, axis=1) - meas) / 0.1
            r2 = (X[tk, 2] - tv) / 0.3
            r3 = ((X[2:] - 2 * X[1:-1] + X[:-2]) * RATE * RATE).ravel() / 2.0
            r4 = (X[0, :2] - pad[:2]) / 0.05
            return np.concatenate([r1, r2, r3, r4])

        J = lil_matrix((len(k) + len(tk) + (M - 2) * 3 + 2, M * 3), dtype=np.int8)
        row = 0
        for j in range(len(k)):
            for kk in (k[j], k[j] + 1):
                J[row, kk * 3:kk * 3 + 3] = 1
            row += 1
        for j in range(len(tk)):
            J[row, tk[j] * 3 + 2] = 1
            row += 1
        for kk in range(1, M - 1):
            for a in range(3):
                for k3 in (kk - 1, kk, kk + 1):
                    J[row, k3 * 3 + a] = 1
                row += 1
        J[row, 0] = 1
        J[row + 1, 1] = 1
        # start in the middle of the hall: a plain least-squares pass finds the basin, the
        # robust passes then shed the multipath and non-line-of-sight ranges
        z0 = np.tile(np.array([20.0, 12.0, 3.0]), (M, 1)).ravel()
        Js = J.tocsr()
        sol = least_squares(res, z0, jac_sparsity=Js, loss="linear", method="trf",
                            tr_solver="lsmr", max_nfev=50)
        for scale in (3.0, 1.0):
            sol = least_squares(res, sol.x, jac_sparsity=Js, loss="cauchy", f_scale=scale,
                                method="trf", tr_solver="lsmr", max_nfev=50)
        if len(ok_):
            # second pass: the ToF over rack tops, referenced to the nearest rack's top
            X1 = sol.x.reshape(M, 3)
            top = self.rack_top_below(X1[ok_, 0], X1[ok_, 1])
            tk = np.concatenate([tk, ok_])
            tv = np.concatenate([tv, ov_ + top])
            J = lil_matrix((len(k) + len(tk) + (M - 2) * 3 + 2, M * 3), dtype=np.int8)
            row = 0
            for j in range(len(k)):
                for kk in (k[j], k[j] + 1):
                    J[row, kk * 3:kk * 3 + 3] = 1
                row += 1
            for j in range(len(tk)):
                J[row, tk[j] * 3 + 2] = 1
                row += 1
            for kk in range(1, M - 1):
                for a in range(3):
                    for k3 in (kk - 1, kk, kk + 1):
                        J[row, k3 * 3 + a] = 1
                    row += 1
            J[row, 0] = 1
            J[row + 1, 1] = 1
            sol = least_squares(res, sol.x, jac_sparsity=J.tocsr(), loss="cauchy", f_scale=1.0,
                                method="trf", tr_solver="lsmr", max_nfev=50)
        return sol.x.reshape(M, 3)

    def init_heading(self, i, X):
        fi = self.flow[:, 0] == i
        fk = self.flow[fi, 1].astype(int)
        fb = self.flow[fi, 2:4]
        v = (X[fk + 1] - X[fk]) * RATE
        sp = np.linalg.norm(v[:, :2], axis=1)
        ok = sp > 0.3
        best = (np.inf, 0.0, 0.0)
        for bg in np.linspace(-0.03, 0.03, 61):
            yb = self.G[i, fk] - bg * self.tn[fk]
            # rotate flow by yb into a frame offset by psi from world: world = R(psi) R(yb) fb
            c, s = np.cos(yb), np.sin(yb)
            wx = c * fb[:, 0] - s * fb[:, 1]
            wy = s * fb[:, 0] + c * fb[:, 1]
            # best psi in closed form
            num = np.sum((wx * v[:, 1] - wy * v[:, 0])[ok])
            den = np.sum((wx * v[:, 0] + wy * v[:, 1])[ok])
            psi = math.atan2(num, den)
            cp, sp_ = math.cos(psi), math.sin(psi)
            e = np.sum(((cp * wx - sp_ * wy - v[:, 0]) ** 2 + (sp_ * wx + cp * wy - v[:, 1]) ** 2)[ok])
            if e < best[0]:
                best = (e, psi, bg)
        return best[1], best[2]

    def jacobian(self, x):
        """Analytic sparse Jacobian of residuals(x)."""
        X, psi, bg, _, _ = self.unpack(x)
        D, M, NK = self.D, self.M, self.NK
        nx = D * M * 3
        nw = nx + 2 * D                       # first drift knot
        ns = nw + D * NK                      # first rack excess per metre
        rows, cols, vals = [], [], []
        row0 = 0

        def xi(i, k, a):
            return (i * M + k) * 3 + a

        # ranges
        n = len(self.r_t)
        pa = self.node_positions(X, self.r_a, self.r_k, self.r_f)
        pb = self.node_positions(X, self.r_b, self.r_k, self.r_f)
        dv = pb - pa
        dist = np.maximum(np.linalg.norm(dv, axis=1), 1e-6)
        sig = self.range_sigma()
        u = dv / dist[:, None] / sig[:, None]     # d(-dist)/d pb = -u*sig..., residual = (e - dist)/sig
        ridx = np.arange(n) + row0
        for node, sign in ((self.r_a, 1.0), (self.r_b, -1.0)):
            dr = node < D
            for kk, wgt in ((self.r_k, 1 - self.r_f), (self.r_k + 1, self.r_f)):
                for a in range(3):
                    rows.append(ridx[dr])
                    cols.append(xi(node[dr], kk[dr], a))
                    vals.append(sign * u[dr, a] * wgt[dr])
            # the residual is (measured - ... - drift - dist) / sigma
            for jj, wgt in ((self.r_j, 1 - self.r_g), (self.r_j + 1, self.r_g)):
                rows.append(ridx[dr])
                cols.append(nw + node[dr] * NK + jj[dr])
                vals.append(-wgt[dr] / sig[dr])
        bl = np.nonzero(self.r_blocked)[0]
        for r in range(self.R):
            rows.append(ridx[bl]); cols.append(np.full(len(bl), ns + r)); vals.append(-self.r_L[bl, r] / sig[bl])
        row0 += n
        # flow
        fi = self.flow[:, 0].astype(int)
        fk = self.flow[:, 1].astype(int)
        v = (X[fi, fk + 1] - X[fi, fk]) * RATE
        yaw = self.yaw(psi, bg)
        ym = 0.5 * (yaw[fi, fk] + yaw[fi, fk + 1])
        tm = 0.5 * (self.tn[fk] + self.tn[fk + 1])
        c, s_ = np.cos(ym), np.sin(ym)
        h = np.maximum(X[fi, fk, 2], 0.3)
        fs = 0.045 * (1 + 0.3 * h) / np.sqrt(self.flow[:, 4])
        nf = len(fi)
        # vbx = c vx + s vy ; vby = -s vx + c vy
        dvbx_dyaw = -s_ * v[:, 0] + c * v[:, 1]
        dvby_dyaw = -c * v[:, 0] - s_ * v[:, 1]
        for comp, (cx, cy, dyaw) in enumerate(((c, s_, dvbx_dyaw), (-s_, c, dvby_dyaw))):
            ridx = np.arange(nf) + row0
            for kk, sgn in ((fk + 1, 1.0), (fk, -1.0)):
                rows.append(ridx); cols.append(xi(fi, kk, 0)); vals.append(sgn * RATE * cx / fs)
                rows.append(ridx); cols.append(xi(fi, kk, 1)); vals.append(sgn * RATE * cy / fs)
            rows.append(ridx); cols.append(nx + fi); vals.append(dyaw / fs)
            rows.append(ridx); cols.append(nx + D + fi); vals.append(-tm * dyaw / fs)
            row0 += nf
        # ToF
        ti = self.tof[:, 0].astype(int)
        tk = self.tof[:, 1].astype(int)
        ridx = np.arange(len(ti)) + row0
        rows.append(ridx); cols.append(xi(ti, tk, 2)); vals.append(np.full(len(ti), 1 / TOF_SIGMA))
        row0 += len(ti)
        # smoothness
        ii, kk, aa = np.meshgrid(np.arange(D), np.arange(1, M - 1), np.arange(3), indexing="ij")
        ii, kk, aa = ii.ravel(), kk.ravel(), aa.ravel()
        ridx = np.arange(len(ii)) + row0
        for dk, w in ((-1, 1.0), (0, -2.0), (1, 1.0)):
            rows.append(ridx); cols.append(xi(ii, kk + dk, aa)); vals.append(np.full(len(ii), w * RATE * RATE / self.acc_sigma))
        row0 += len(ii)
        # pads (every state before take-off; x then y, as residuals() ravels them)
        npad = len(self.pad_i)
        for a in range(2):
            rows.append(row0 + 2 * np.arange(npad) + a); cols.append(xi(self.pad_i, self.pad_k, a))
            vals.append(np.full(npad, 1 / 0.05))
        row0 += 2 * npad
        # drift priors
        kn = nw + np.arange(D) * NK
        rows.append(row0 + np.arange(D)); cols.append(kn); vals.append(np.full(D, 1 / DRIFT_START))
        row0 += D
        ii, jj = np.meshgrid(np.arange(D), np.arange(NK - 1), indexing="ij")
        base = (nw + ii * NK + jj).ravel()
        ridx = row0 + np.arange(base.size)
        rows += [ridx, ridx]; cols += [base, base + 1]
        vals += [np.full(base.size, -1 / DRIFT_STEP), np.full(base.size, 1 / DRIFT_STEP)]
        row0 += base.size
        ii, jj = np.meshgrid(np.arange(D), np.arange(NK - 2), indexing="ij")
        base = (nw + ii * NK + jj).ravel()
        ridx = row0 + np.arange(base.size)
        for dk, w in ((0, 1.0), (1, -2.0), (2, 1.0)):
            rows.append(ridx); cols.append(base + dk); vals.append(np.full(base.size, w / DRIFT_BEND))
        row0 += base.size
        rows.append(row0 + np.arange(self.R)); cols.append(ns + np.arange(self.R))
        vals.append(np.full(self.R, 1 / STOCK_SIGMA))
        row0 += self.R
        J = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(row0, nx + self.NG)).tocsr()
        return J

    def _perm(self):
        """Time-major order of the position unknowns (then the flight's parameters: headings,
        gyro biases, drift knots, rack excess), which makes the normal matrix banded apart from
        those columns."""
        D, M = self.D, self.M
        k, i, a = np.meshgrid(np.arange(M), np.arange(D), np.arange(3), indexing="ij")
        return np.concatenate([((i * M + k) * 3 + a).ravel(), D * M * 3 + np.arange(self.NG)])

    def _normal_solve(self, A, g, lam):
        """Solve (A + lam diag A) dx = -g by banded Cholesky with a Schur complement for the
        per-drone parameters."""
        perm = self._perm()
        Ap = A[perm][:, perm].tocsc()
        gp = g[perm]
        n = self.D * self.M * 3
        d = Ap.diagonal()
        Ap = Ap + diags(lam * d + 1e-9)
        B = Ap[:n, :n].tocoo()
        Cm = Ap[:n, n:].toarray()
        E = Ap[n:, n:].toarray()
        low = B.row >= B.col
        u = int(np.max(B.row[low] - B.col[low]))
        ab = np.zeros((u + 1, n))
        ab[B.row[low] - B.col[low], B.col[low]] = B.data[low]
        cb = cholesky_banded(ab, lower=True)
        BiC = cho_solve_banded((cb, True), Cm)
        Big = cho_solve_banded((cb, True), gp[:n])
        S = E - Cm.T @ BiC
        dz = np.linalg.solve(S, -(gp[n:] - Cm.T @ Big))
        dx = -(Big + BiC @ dz)
        out = np.empty_like(g)
        out[perm] = np.concatenate([dx, dz])
        return out

    def irls(self, x, iters, scale=2.0):
        """Levenberg-Marquardt on Cauchy-reweighted residuals."""
        lam = 1e-3
        r = self.residuals(x)
        w = 1.0 / (1.0 + (r / scale) ** 2)
        cost = float(np.sum(scale ** 2 * np.log1p((r / scale) ** 2)))
        for _ in range(iters):
            J = self.jacobian(x)
            A = (J.T @ diags(w) @ J).tocsr()
            g = J.T @ (w * r)
            step = self._normal_solve(A, g, lam)
            xn = x + step
            rn = self.residuals(xn)
            cn = float(np.sum(scale ** 2 * np.log1p((rn / scale) ** 2)))
            if cn < cost:
                x, r, cost = xn, rn, cn
                w = 1.0 / (1.0 + (r / scale) ** 2)
                lam = max(lam / 3.0, 1e-7)
                if np.max(np.abs(step[: self.D * self.M * 3])) < 1e-4:
                    break
            else:
                lam *= 5.0
        return x

    def solve(self):
        Xs, psis, bgs = [], [], []
        for i in range(self.D):
            X = self.init_drone(i)
            psi, bg = self.init_heading(i, X)
            Xs.append(X)
            psis.append(psi)
            bgs.append(bg)
        x = np.concatenate([np.array(Xs).ravel(), psis, bgs, np.zeros(self.D * self.NK),
                            np.full(self.R, self.cal["nlos_per_m"])])
        for it in range(4):
            self.linearisation_terms(x)
            x = self.irls(x, 12)
        self.x_final = x
        return self.unpack(x)[0]


def main():
    folder, out = sys.argv[1], sys.argv[2]
    prob = Problem(folder, load_calib())
    X = prob.solve()
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "trajectories.csv"), "w") as f:
        f.write("t_s,drone,x_m,y_m,z_m\n")
        step = int(round(RATE / float(prob.meta.get("output_rate_hz", 10.0))))
        for k in range(0, prob.M, step):
            t = k / RATE
            for i in range(prob.D):
                f.write("{:.2f},{:d},{:.4f},{:.4f},{:.4f}\n".format(t, i, *X[i, k]))


if __name__ == "__main__":
    main()
