"""The program's eyes: RGB-D frames from the cell (published calibration, T_world_cam per frame), the engine's pose from
the front camera, the belt's visible centreline, local re-aims from a wrist camera, the tensioner's pointer against its
window marks, and each pulley's wrap from a wrist depth frame. Public inputs only (describe(), the drawing, frames)."""
import base64
import io
import math

import numpy as np
from PIL import Image

import hk

DARK = 40           # the belt is black (15, 15, 15) under a gain of 0.94-1.06 and brightness 0.92-1.08; the plate > 95
BELT_W = 0.020


def decode(reply):
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))
    d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["depth_u16_b64"]))), dtype=np.float64)
    z = d * float(reply.get("depth_unit_m", 0.001))
    z[d == float(reply.get("depth_invalid_value", 0))] = np.nan
    return rgb, z


class Eyes(object):
    def __init__(self, rig):
        self.r = rig
        self.cams = rig.info["cameras"]
        self.frames = 0
        self.edge_centre = False

    def frame(self, name):
        rep = self.r.cell.get_camera(name, True)
        self.frames += 1
        rgb, z = decode(rep)
        c = self.cams[name]
        return {"rgb": rgb, "z": z, "K": {k: float(c[k]) for k in ("fx", "fy", "cx", "cy")},
                "T": np.asarray(rep["T_world_cam"], float), "name": name}

    # ------------------------------------------------------------------ geometry helpers
    @staticmethod
    def backproject(fr, mask=None):
        z = fr["z"]
        ok = np.isfinite(z) & (z > 0)
        if mask is not None:
            ok &= mask
        v, u = np.nonzero(ok)
        zz = z[v, u]
        K = fr["K"]
        x = (u - K["cx"]) * zz / K["fx"]
        y = (v - K["cy"]) * zz / K["fy"]
        P = np.stack([x, y, zz], axis=-1) @ fr["T"][:3, :3].T + fr["T"][:3, 3]
        return P, v, u

    def to_E(self, Pw, E=None):
        E_pos, R_E = (self.r.E_pos, self.r.R_E) if E is None else E
        q = (np.asarray(Pw, float) - E_pos) @ R_E
        return np.stack([q[..., 0], q[..., 2], -q[..., 1]], axis=-1)

    def from_E(self, X):
        X = np.asarray(X, float)
        q = np.stack([X[..., 0], -X[..., 2], X[..., 1]], axis=-1)
        return q @ self.r.R_E.T + self.r.E_pos

    def project(self, fr, Pw):
        T = fr["T"]
        q = (np.asarray(Pw, float) - T[:3, 3]) @ T[:3, :3]
        K = fr["K"]
        return K["cx"] + K["fx"] * q[..., 0] / q[..., 2], K["cy"] + K["fy"] * q[..., 1] / q[..., 2], q[..., 2]

    # ------------------------------------------------------------------ the engine
    def survey_frame(self, given_dir, name="no_belt_front.png"):
        """a given survey frame (nominal engine pose: survey.json) as a frame dict"""
        import json
        import os
        sv = json.load(open(os.path.join(given_dir, "survey", "survey.json")))
        f = [x for x in sv["frames"] if x["file"] == name][0]
        rgb = np.asarray(Image.open(os.path.join(given_dir, "survey", f["file"])).convert("RGB"))
        d = np.asarray(Image.open(os.path.join(given_dir, "survey", f["depth_file"])), dtype=np.float64)
        z = d * float(f["depth_unit_m"])
        z[d == 0] = np.nan
        K = {k: float(f["intrinsics"][k]) for k in ("fx", "fy", "cx", "cy")}
        return {"rgb": rgb, "z": z, "K": K, "T": np.asarray(f["T_world_cam"], float), "name": "survey"}

    def engine_pose_calibrated(self, fr, given_dir):
        """engine_pose, less the estimator's own bias measured on the given survey frame of an engine at its nominal
        pose (the in-plane offset of the estimate there, in E)"""
        E = self.engine_pose(fr)
        try:
            Es = self.engine_pose(self.survey_frame(given_dir))
            f = self.r.info["frames"]["engine_nominal_pose"]
            En = np.asarray(f["origin_world"], float)
            Rn = np.column_stack([f["x_E_world"], f["y_E_world"], f["z_E_world"]]).astype(float)
            b = Rn.T @ (Es[0] - En)
            b[1] = 0.0                                      # the face plane's depth is measured directly
            self.survey_bias = b
            E = (E[0] - E[1] @ b, E[1])
        except Exception:  # noqa: BLE001 - without the survey the raw estimate stands
            self.survey_bias = None
        return E

    def engine_pose(self, fr, iters=3):
        dw = self.r.dw
        f = self.r.info["frames"]["engine_nominal_pose"]
        E = (np.asarray(f["origin_world"], float), np.column_stack([f["x_E_world"], f["y_E_world"], f["z_E_world"]]).astype(float))
        P, _, _ = self.backproject(fr)
        for it in range(iters):
            X = self.to_E(P, E)
            win = 0.012 if it == 0 else 0.006
            sel = (np.abs(X[:, 2]) < win) & (X[:, 0] > -0.24) & (X[:, 0] < 0.30) & (X[:, 1] > -0.12) & (X[:, 1] < 0.34)
            for k, p in dw.pul.items():
                sel &= np.hypot(X[:, 0] - p["c"][0], X[:, 1] - p["c"][1]) > p["r"] + 0.012
            sel &= np.hypot(X[:, 0] - dw.pivot[0], X[:, 1] - dw.pivot[1]) > 0.21
            sel &= np.hypot(X[:, 0] - dw.peg[0], X[:, 1] - dw.peg[1]) > 0.03
            Q = P[sel]
            if len(Q) < 500:
                break
            c0 = Q.mean(axis=0)
            _, _, Vt = np.linalg.svd(Q - c0, full_matrices=False)
            n = Vt[2]
            E_pos, R_E = E
            if n @ R_E[:, 1] < 0:
                n = -n
            ax = np.cross(R_E[:, 1], n)
            s = np.linalg.norm(ax)
            if s > 1e-9:
                R_E = hk.rot_axis(ax, math.asin(min(1.0, s))) @ R_E
            E_pos = E_pos + n * ((c0 - E_pos) @ n)
            E = (E_pos, R_E)
            X = self.to_E(P, E)
            cs, ms = [], []
            d_face = dw.front_sh[1] + dw.chamfer
            for k in [k_ for k_ in dw.pul if k_ not in dw.idlers]:
                c = dw.pul[k]["c"]
                rad = np.hypot(X[:, 0] - c[0], X[:, 1] - c[1])
                disk = (np.abs(X[:, 2] - d_face) < 0.003) & (rad < dw.pul[k]["r"] + 0.008)
                if disk.sum() < 200:
                    continue
                xs, zs = X[disk, 0], X[disk, 1]
                x_lo, x_hi = np.percentile(xs, 0.5), np.percentile(xs, 99.5)
                # a disk partly hidden (an arm in front of it in the frame) reads narrow: its centre is not used
                # (the front cap reads 1-4 mm under 2 r between those percentiles)
                if (x_hi - x_lo) < 2.0 * dw.pul[k]["r"] - 0.005:
                    continue
                cx = 0.5 * (x_lo + x_hi)
                # the centre's height from the disk's chord widths (w/2)^2 + z^2 = r^2 - cz^2 + 2 cz z: the sensor
                # drops the pixels on depth edges, so an edge-based estimate (the lowest pixel + r) sits ~2 mm high
                rr = dw.pul[k]["r"]
                zb = np.floor(zs / 0.001).astype(int)
                rows = []
                for b_ in np.unique(zb):
                    sel_ = zb == b_
                    if sel_.sum() < 8:
                        continue
                    w = float(xs[sel_].max() - xs[sel_].min())
                    if w > 0.8 * rr:
                        rows.append(((b_ + 0.5) * 0.001, w))
                if len(rows) >= 10:
                    zr = np.array([r_[0] for r_ in rows])
                    yv = np.array([r_[1] for r_ in rows]) ** 2 / 4.0 + zr ** 2
                    A = np.column_stack([np.ones_like(zr), 2 * zr])
                    sol, *_ = np.linalg.lstsq(A, yv, rcond=None)
                    cz = float(sol[1])
                else:
                    cz = np.percentile(zs, 0.3) + rr
                cs.append(np.array([cx, cz]))
                ms.append(c)
            if len(cs) >= 2:
                # an in-plane rigid fit (turn about the face normal + offset) of the drawn centres to the seen ones
                A_, B_ = np.asarray(ms), np.asarray(cs)
                ca, cb = A_.mean(axis=0), B_.mean(axis=0)
                H = (A_ - ca).T @ (B_ - cb)
                ang = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
                if abs(ang) < math.radians(2.0):
                    c_, s_ = math.cos(ang), math.sin(ang)
                    Rot = np.array([[c_, -s_], [s_, c_]])
                    off = cb - Rot @ ca
                    if np.linalg.norm(off) < 0.015:
                        E_pos, R_E = E
                        # x_seen = Rot x_drawn + off (in E's x-z plane): the new frame's axes turn by ang about -y_E
                        # (x towards z is counter-clockwise in the drawing)
                        R_new = hk.rot_axis(-R_E[:, 1], ang) @ R_E
                        E = (E_pos + R_E @ np.array([off[0], 0.0, off[1]]), R_new)
            elif cs:
                off = np.median(np.asarray(cs) - np.asarray(ms), axis=0)
                if np.linalg.norm(off) < 0.012:
                    E_pos, R_E = E
                    E = (E_pos + R_E @ np.array([off[0], 0.0, off[1]]), R_E)
        return E

    # ------------------------------------------------------------------ the belt
    def arm_keepout(self, radius=0.075):
        kin = self.r.kin
        out = []
        for s in ("L", "R"):
            for b in kin.pts_b[s]:
                out.append((kin.d.xpos[b].copy(), radius))
            hb = kin.hand[s]
            Rh = kin.d.xmat[hb].reshape(3, 3)
            for z in (0.06, 0.10):
                out.append((kin.d.xpos[hb] + Rh[:, 2] * z, 0.045))
        return out

    def belt_E(self, fr, d_range=(0.012, 0.20), keep=True, extra_keep=()):
        """the belt's visible surface in E (dark pixels with depth, in front of the face, away from the arms and the
        peg's post)"""
        rgb = fr["rgb"]
        m = rgb.max(axis=2) < DARK
        P, v, u = self.backproject(fr, m)
        if not len(P):
            return np.zeros((0, 3))
        ok = np.ones(len(P), bool)
        for c, rad in (self.arm_keepout() if keep else []) + list(extra_keep):
            ok &= np.linalg.norm(P - np.asarray(c, float), axis=1) > rad
        X = self.to_E(P[ok])
        dw = self.r.dw
        sel = (X[:, 2] > d_range[0]) & (X[:, 2] < d_range[1])
        px, pz = dw.peg
        sel &= ~((X[:, 0] > px - 0.035) & (X[:, 0] < px + 0.035) & (X[:, 1] > pz - 0.40) & (X[:, 1] < pz + 0.04) & (X[:, 2] < 0.012 + 0.013))
        return X[sel]

    @staticmethod
    def loop(X, n_bins=180, centre=None):
        """ordered closed centreline (x, z, d) by polar angle about the centroid; d = the band's front edge less half
        the belt's width"""
        if len(X) < 20:
            return np.zeros((0, 3)), 0.0
        c = np.median(X[:, :2], axis=0) if centre is None else np.asarray(centre, float)
        a = np.arctan2(X[:, 1] - c[1], X[:, 0] - c[0])
        b = np.floor((a + math.pi) / (2 * math.pi) * n_bins).astype(int) % n_bins
        out = []
        for k in range(n_bins):
            s = b == k
            if s.sum() < 3:
                continue
            rad = np.hypot(X[s, 0] - c[0], X[s, 1] - c[1])
            ang = -math.pi + (k + 0.5) * 2 * math.pi / n_bins
            rm = float(np.median(rad))
            d_c = float(np.percentile(X[s, 2], 92)) - 0.5 * BELT_W + 0.001
            out.append((c[0] + rm * math.cos(ang), c[1] + rm * math.sin(ang), d_c))
        return np.asarray(out), len(out) / float(n_bins)

    @staticmethod
    def resample(L, step=0.010, closed=True):
        if len(L) < 2:
            return L
        P = np.vstack([L, L[:1]]) if closed else L
        seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        n = max(2, int(s[-1] / step))
        t = np.linspace(0.0, s[-1], n, endpoint=not closed)
        return np.column_stack([np.interp(t, s, P[:, i]) for i in range(3)])

    def front_loop(self):
        """(Q, coverage, n points): the belt's centreline from one front frame, 10 mm apart, in angle order"""
        fr = self.frame("front")
        X = self.belt_E(fr)
        L, cov = self.loop(X)
        return self.resample(L), cov, len(X)

    def local(self, fr, guessE, radius=0.025, d_range=(0.0, 0.20), hand=None):
        """re-aim on the belt near guessE (x, z, d) from a (wrist) frame: the dark points within radius in the face
        plane, their line (PCA), the point on it nearest the guess, the depth from the band's front edge.
        Returns (centre E, in-plane normal (nx, nz), n points) or None."""
        X = self.belt_E(fr, d_range=d_range, keep=False, extra_keep=self._finger_keep(hand))
        if len(X) < 15:
            return None
        g = np.asarray(guessE, float)
        near = np.hypot(X[:, 0] - g[0], X[:, 1] - g[1]) < radius
        if near.sum() < 15:
            return None
        Y = X[near]
        c0 = Y[:, :2].mean(axis=0)
        _, _, Vt = np.linalg.svd(Y[:, :2] - c0, full_matrices=False)
        t = Vt[0]
        # the tangent where the hand will close: from the points within 12 mm along the belt of the guess
        s_ = (Y[:, :2] - g[:2]) @ t
        loc_ = np.abs(s_) < 0.012
        if loc_.sum() >= 15:
            cl_ = Y[loc_, :2].mean(axis=0)
            _, _, Vt2 = np.linalg.svd(Y[loc_, :2] - cl_, full_matrices=False)
            t = Vt2[0] if float(Vt2[0] @ t) >= 0 else -Vt2[0]
        nrm = np.array([-t[1], t[0]])
        nrm = np.array([-t[1], t[0]])
        off = (Y[:, :2] - c0) @ nrm
        # across the belt: the band's front edge (the points within 3 mm of the front-most depth) spans the whole
        # thickness symmetrically, whereas a broad face seen obliquely sits on one side: centre on the edge points
        d_front = float(np.percentile(Y[:, 2], 92))
        edge = Y[:, 2] > d_front - 0.003
        c1 = c0 + nrm * float(np.median(off[edge] if (self.edge_centre and edge.sum() >= 10) else off))
        p = c1 + t * float((g[:2] - c1) @ t)
        s_near = np.hypot(Y[:, 0] - p[0], Y[:, 1] - p[1]) < 0.6 * radius
        dd = Y[s_near, 2] if s_near.sum() >= 5 else Y[:, 2]
        d_c = float(np.percentile(dd, 92)) - 0.5 * BELT_W + 0.001
        return np.array([p[0], p[1], d_c]), nrm, int(near.sum())

    def _finger_keep(self, hand):
        if hand is None:
            return []
        kin = self.r.kin
        hb = kin.hand[hand]
        Rh = kin.d.xmat[hb].reshape(3, 3)
        return [(kin.d.xpos[hb] + Rh[:, 2] * z, 0.03) for z in (0.03, 0.06, 0.09, 0.115)]

    def lever_angle(self):
        """the tensioner's angle from the lever hand's wrist camera: the orange service lever's axis (drawing: a round
        bar at lever_r from the pivot, at theta + lever_off), from its pixels' median position in the face plane"""
        dw = self.r.dw
        fr = self.frame("left_wrist" if self.r.TS == "L" else "right_wrist")
        rgb = fr["rgb"].astype(np.int32)
        m = (rgb[..., 0] > 150) & (rgb[..., 1] > 80) & (rgb[..., 1] < 200) & (rgb[..., 2] < 90) & (rgb[..., 0] - rgb[..., 2] > 100)
        P, _, _ = self.backproject(fr, m)
        if len(P) < 30:
            return None
        X = self.to_E(P)
        rad = np.hypot(X[:, 0] - dw.pivot[0], X[:, 1] - dw.pivot[1])
        sel = (X[:, 2] > dw.lever_d[0]) & (X[:, 2] < dw.lever_d[1] + 0.003) & (np.abs(rad - dw.lever_r) < 0.03)
        if sel.sum() < 30:
            return None
        # the bar's visible surface lies about its radius in front of / beside the axis: the angle of its pixels' mean
        ang = np.arctan2(X[sel, 1] - dw.pivot[1], X[sel, 0] - dw.pivot[0])
        a = math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))
        th = a - dw.lever_off
        return (th + math.pi) % (2 * math.pi) - math.pi

    # ------------------------------------------------------------------ the tensioner's pointer (front camera)
    def pointer(self, fr=None, th_prior=None, tol_deg=30.0):
        from scipy import ndimage
        dw = self.r.dw
        fr = self.frame("front") if fr is None else fr
        rgb, z, Tw, K = fr["rgb"], fr["z"], fr["T"], fr["K"]
        piv = dw.pivot
        u0, v0, _ = self.project(fr, self.r.W(piv[0], piv[1], 0.0085))
        h, w = z.shape
        u_lo, u_hi = max(0, int(u0) - 90), min(w, int(u0) + 91)
        v_lo, v_hi = max(0, int(v0) - 90), min(h, int(v0) + 91)
        sub = rgb[v_lo:v_hi, u_lo:u_hi].astype(np.int32)
        bright = sub.min(axis=2) > 190
        Vv, Ut = np.mgrid[v_lo:v_hi, u_lo:u_hi]
        R_E, E_pos = self.r.R_E, self.r.E_pos

        def on_plane(us, vs, dd):
            dirs = np.stack([(us - K["cx"]) / K["fx"], (vs - K["cy"]) / K["fy"], np.ones_like(us, float)], axis=-1) @ Tw[:3, :3].T
            o = Tw[:3, 3]
            nrm = -R_E[:, 1]
            p0 = E_pos + R_E @ np.array([0.0, -dd, 0.0])
            t = ((p0 - o) @ nrm) / (dirs @ nrm)
            X = self.to_E(o + dirs * t[..., None])
            return np.hypot(X[..., 0] - piv[0], X[..., 1] - piv[1]), np.degrees(np.arctan2(X[..., 1] - piv[1], X[..., 0] - piv[0]))

        # the blade (30-46 mm from the pivot) can touch a window mark (40-52 mm) end to end when the tensioner rests
        # near a window end: label only the bright pixels inside the marks' inner radius
        rad_all, _ = on_plane(Ut.astype(float), Vv.astype(float), 0.0085)
        bright &= rad_all < 0.0395
        lab, n = ndimage.label(bright, structure=np.ones((3, 3), int))

        best = None
        for c in range(1, n + 1):
            m = lab == c
            area = int(m.sum())
            if area < 6 or area > 600:
                continue
            rad, ang = on_plane(Ut[m].astype(float), Vv[m].astype(float), 0.0085)
            r_med = float(np.median(rad))
            if th_prior is not None:
                # the blade points away from the pulley: theta + 180 deg; only near where the tensioner was let go
                a_mean = math.degrees(math.atan2(float(np.sin(np.radians(ang)).mean()), float(np.cos(np.radians(ang)).mean())))
                if abs((a_mean - (math.degrees(th_prior) + 180.0) + 180.0) % 360.0 - 180.0) > tol_deg:
                    continue
            if 0.029 <= r_med <= 0.046 and float(rad.max() - rad.min()) >= 0.006 and float(rad.min()) <= 0.036 \
                    and float(rad.max()) <= 0.049:
                sel = (rad >= 0.029) & (rad <= 0.040)
                aa = np.radians(ang[sel])
                R_ = math.hypot(float(np.cos(aa).mean()), float(np.sin(aa).mean())) if sel.any() else 0.0
                spread = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(1e-9, R_)))))
                if 5 <= sel.sum() <= 250 and spread <= 6.0 and (best is None or sel.sum() > best[0]):
                    best = (int(sel.sum()), ang[sel])
        res = {"n_blobs": int(n)}
        if best is None:
            res.update({"ok": False, "why": "pointer not seen"})
            return res
        a = np.radians(best[1])
        phi = math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))
        th = (phi - 180.0 + 180.0) % 360.0 - 180.0
        win = [math.degrees(v) for v in dw.window]
        res.update({"theta_deg": round(th, 2), "n_pointer_px": best[0], "ok": bool(win[0] <= th <= win[1])})
        return res

    # ------------------------------------------------------------------ the wraps (wrist depth)
    def wraps(self, th):
        """the belt path's wrap sectors from the drawing: {k: (angle in, direction s, sweep)}; the route counter-
        clockwise as drawn, ribs (inside the loop) on the grooved pulleys, the back (outside) on T and the idlers"""
        dw = self.r.dw
        half = 0.5 * dw.belt_thick
        circ = []
        for k in dw.route:
            if k == "T":
                circ.append(("T", dw.T_centre(th), dw.T_r + half))
            elif k in dw.idlers:
                circ.append((k, dw.pul[k]["c"], dw.pul[k]["r"] + half))
            else:
                circ.append((k, dw.pul[k]["c"], -(dw.pul[k]["r"] + half)))
        nc = len(circ)
        tang = []
        for i in range(nc):
            k1, c1, a1 = circ[i]
            k2, c2, a2 = circ[(i + 1) % nc]
            D = c2 - c1
            Lh = float(np.hypot(*D))
            phi = math.atan2(D[1], D[0])
            al = phi - math.asin((a1 - a2) / Lh)
            n = np.array([-math.sin(al), math.cos(al)])
            tang.append((c1 + a1 * n, c2 + a2 * n))
        out = {}
        for i in range(nc):
            k, c, a = circ[i]
            p_in = tang[i - 1][1]
            p_out = tang[i][0]
            a_in = math.atan2(p_in[1] - c[1], p_in[0] - c[0])
            a_out = math.atan2(p_out[1] - c[1], p_out[0] - c[0])
            s = 1 if a < 0 else -1
            sweep = ((a_out - a_in) * s) % (2 * math.pi)
            out[k] = (a_in, s, sweep, c)
        return out

    # (pulley, the hand whose wrist camera looks, park that hand after): drive B
    CHECK_VIEWS = (("C", "R", False), ("ALT", "R", False), ("I", "R", True), ("WP", "L", False), ("T", "L", True))
    EDGE_D_MAX = 0.0390
    T_EDGE_D_MAX = 0.0395
    COVER_MIN = 0.6
    TRIM = math.radians(8.0)
    I_TRIM = math.radians(4.0)          # a backside idler wraps only ~34 deg

    def view(self, k, side, th):
        r = self.r
        dw = r.dw
        a_in, s, sweep, c = self.wraps(th)[k]
        rr = dw.T_r if k == "T" else dw.pul[k]["r"]
        smooth = k == "T" or k in dw.idlers
        mid = a_in + s * 0.5 * sweep
        u = np.array([math.cos(mid), math.sin(mid)])
        aim = c + u * (rr + 0.012)
        radial = r.R_E @ np.array([u[0], 0.0, u[1]])
        xh = -radial
        z = r.into
        xh = xh - (xh @ z) * z
        xh /= np.linalg.norm(xh)
        R = np.column_stack([xh, np.cross(z, xh), z])
        tcp = r.W(aim[0], aim[1], 0.037) - r.into * 0.15
        r.goto_joint(side, tcp, R)
        r.wait(0.3)
        fr = self.frame("left_wrist" if side == "L" else "right_wrist")
        P, _, _ = self.backproject(fr)
        E = self.to_E(P)
        rad = np.hypot(E[:, 0] - c[0], E[:, 1] - c[1])
        ang = np.arctan2(E[:, 1] - c[1], E[:, 0] - c[0])
        rel = ((ang - a_in) * s) % (2 * math.pi)
        trim = self.I_TRIM if k in dw.idlers else self.TRIM
        insec = (rel > trim) & (rel < sweep - trim)
        if smooth:
            edge = insec & (rad >= rr + 0.0046) & (rad <= rr + 0.0056) & (E[:, 2] > 0.012) & (E[:, 2] < 0.055)
            band = insec & (rad >= rr + 0.0040) & (rad <= rr + 0.0130) & (E[:, 2] > 0.012) & (E[:, 2] < 0.055)
            lim = rr + 0.0072
        else:
            edge = insec & (rad >= rr + 0.0028) & (rad <= rr + 0.0042) & (E[:, 2] > 0.025) & (E[:, 2] < 0.060)
            band = insec & (rad >= rr + 0.0028) & (rad <= rr + 0.0120) & (E[:, 2] > 0.012) & (E[:, 2] < 0.055)
            lim = rr + 0.0063
        nb = 12
        relt = (rel - trim) / (sweep - 2 * trim + 1e-9)
        bidx = np.clip(np.floor(relt * nb).astype(int), 0, nb - 1)
        seen, outer = [], []
        for b in range(nb):
            eb = edge & (bidx == b)
            bb = band & (bidx == b)
            if eb.sum() >= 5:
                seen.append(float(np.percentile(E[eb, 2], 90)) if smooth else float(np.median(E[eb, 2])))
            if bb.sum() >= 20:
                outer.append(float(np.percentile(rad[bb], 90)))
        cover = len(seen) / float(nb)
        d_edge = max(seen) if seen else None
        r_out = max(outer) if outer else None
        lifted = r_out is not None and r_out > lim
        if smooth:
            ok = (not lifted) and (d_edge is None or d_edge <= self.T_EDGE_D_MAX)
            if k != "T":
                # a backside idler: the belt must be seen on its face (the sector is short: >= 1/3 of the bins)
                ok = ok and cover >= 1.0 / 3.0
            else:
                cover = None
        else:
            ok = d_edge is not None and d_edge <= self.EDGE_D_MAX and not lifted and cover >= self.COVER_MIN
        return {"ok": bool(ok), "d_edge_mm": None if d_edge is None else round(1000 * d_edge, 2),
                "outer_mm": None if r_out is None else round(1000 * (r_out - rr), 2), "lifted": bool(lifted), "cover": cover}

    POINTER_TOL_DEG = 8.0

    def check(self, park=True, th_prior=None):
        po = self.pointer(th_prior=th_prior, tol_deg=self.POINTER_TOL_DEG)
        if po.get("theta_deg") is not None:
            th = math.radians(po["theta_deg"])
        elif th_prior is not None:
            th = th_prior                   # (the blade not made out: the angle where the lever hand left the lever)
        else:
            th = 0.5 * sum(self.r.dw.window)
        out = {"pointer": po}
        for k, side, park_after in self.CHECK_VIEWS:
            out[k] = self.view(k, side, th)
            if park and park_after:
                self.r.park(side)
        return out
