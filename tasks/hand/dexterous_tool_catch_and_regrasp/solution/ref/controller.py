import sys, json, base64, os, pickle, math
import numpy as np
import vision


def _env(name, default=None):
    return os.environ.get("DEXC_" + name, default)


LOGDIR = _env("LOG", "")
G = 9.81
GVEC = np.array([0, 0, -G])

# tool data: handle range, com, handle colour-region centre (x along tool), zones
CELL_ZONE_FRAC = float(_env("CELL_ZONE_FRAC", "0.30"))   # grip-zone width as a fraction of the handle (tools.md)
LAT_S = 0.02 * int(_env("CELL_LATENCY", "4"))         # sensing/actuation latency (interface.md)
TOOLS = {
    "hammer": dict(handle=(0.030, 0.200), com=0.245, hc=0.130, end=(0.030, 0.107), choke=(0.124, 0.200), m=0.57,
                   r=0.016, head=0.295),
    "screwdriver": dict(handle=(0.030, 0.110), com=0.114, hc=0.070, end=(0.030, 0.066), choke=(0.074, 0.110), m=0.22,
                        r=0.018, head=0.20),
    "wrench": dict(handle=(0.010, 0.130), com=0.186, hc=0.065, end=(0.010, 0.064), choke=(0.076, 0.130), m=0.35,
                   r=0.008, head=0.268),
    "spatula": dict(handle=(0.030, 0.170), com=0.175, hc=0.100, end=(0.030, 0.093), choke=(0.107, 0.170), m=0.23,
                    r=0.014, head=0.29),
}
for _t in TOOLS.values():
    _h0, _h1 = _t["handle"]
    _w = CELL_ZONE_FRAC * (_h1 - _h0)
    _t["end"], _t["choke"] = (_h0, _h0 + _w), (_h1 - _w, _h1)

P = dict(
    T_LEAD=float(_env("T_LEAD", "0.11")),   # close command this long before predicted arrival
    CATCH_X=float(_env("CATCH_X", "0.05")),
    AIM=(float(_env("AIM_X", "0.0")), 0.0, float(_env("AIM_Z", "-0.01"))),   # aim point in H for handle centre (X offset added: palm + radius)
    TAU=float(_env("TAU", "0.12")),
    PALM_UP=float(_env("PALM_UP", "-0.4")),
    WIN_X=float(_env("WIN_X", "0.0")),
    WIN_T=float(_env("WIN_T", "0.15")),
    W_T=float(_env("W_T", "3.0")),
    MATCH_D=float(_env("MATCH_D", "0.08")),
    L_END=float(_env("L_END", "0.30")),
)

OPEN = np.array([0, 0.25, 0.4, 0.3, 0, 0.25, 0.4, 0.3, 0, 0.25, 0.4, 0.3, 1.2, 0.2, 0.2, 0.2])
_oc = float(_env("OPEN_K", "1.0"))
if _env("OPEN_TH"):
    OPEN[12:16] = [float(x) for x in _env("OPEN_TH").split(",")]
OPEN[[1, 2, 3, 5, 6, 7, 9, 10, 11]] *= _oc
CLOSE = np.array([0, 1.61, 1.709, 1.618, 0, 1.61, 1.709, 1.618, 0, 1.61, 1.709, 1.618, 1.396, 1.0, 1.4, 1.5])
if _env("TH_MAX", "0") == "1":
    CLOSE[12:16] = [1.396, 1.163, 1.644, 1.719]
LOOSE = np.array([0, 0.75, 0.9, 0.8, 0, 0.75, 0.9, 0.8, 0, 0.75, 0.9, 0.8, 1.3, 0.6, 0.6, 0.6])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_rotvec(R):
    c = (np.trace(R) - 1) / 2
    c = min(1.0, max(-1.0, c))
    th = math.acos(c)
    if th < 1e-6:
        return np.zeros(3)
    if th > math.pi - 1e-4:
        # 180 deg: axis from R + I
        M = (R + np.eye(3)) / 2
        i = int(np.argmax(np.diag(M)))
        a = M[:, i] / math.sqrt(max(M[i, i], 1e-12))
        return a * th
    a = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    return a * th


def rotvec_to_R(rv):
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def frame_from_XY(X, Y):
    X = X / np.linalg.norm(X)
    Y = Y - X * np.dot(X, Y)
    Y = Y / np.linalg.norm(Y)
    Z = np.cross(X, Y)
    return np.column_stack([X, Y, Z])


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def dec(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


R_HOME = np.column_stack([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])
# ready pose: palm facing up/forward, fingers pointing +y (so Y_H along x initially)
R_READY = frame_from_XY(np.array([0.5, 0, 0.87]), np.array([-0.87, 0, 0.5]))
HOME = np.array([0, 0, 1.0])
BOX_LO = np.array([-0.45, -0.45, 0.70])
BOX_HI = np.array([0.45, 0.45, 1.45])


class Ctl(object):
    def __init__(self):
        self.cams = vision.load_cams()
        self.logbuf = []
        self.nlog = 0
        self.reset_throw(-1)

    def reset_throw(self, k):
        self.throw = k
        self.obs_track = []   # (t, h, axis(unsigned) or None, weight)
        self.rest = []
        self.p_init = None
        self.flying = False
        self.t_launch = None
        self.phase = "wait"
        self.target_p = HOME.copy()
        self.target_R = R_HOME.copy()
        self.fingers = OPEN.copy()
        self.pred = None
        self.t_close = None
        self.ft_base = []
        self.base_F = None
        self.base_T = None
        self.meas = []
        self.est = None
        self.zone = None
        self.plan = None
        self.slide_steps = 0
        self.t_phase = 0.0
        self.last_axis = None
        self.go = None
        self.R_frozen = None
        self.we_H = 0.0
        self.cmd_p = HOME.copy()
        self.cmd_R = R_HOME.copy()
        self.base_p = HOME.copy()
        self.base_R = R_HOME.copy()
        self.last_t = None
        self.n_slides = 0
        self.q_hold = None
        self.meas_R = None
        self.vert_R = None
        self.aH = None
        self.sgn0 = None
        self.slide_k = SLIDE_K
        self.n_retry = 0
        self.beyond = False
        self.est_stale = False

    def flush(self):
        if LOGDIR and self.logbuf:
            os.makedirs(LOGDIR, exist_ok=True)
            with open(os.path.join(LOGDIR, "t%03d.pkl" % self.nlog), "wb") as f:
                pickle.dump(self.logbuf, f)
            self.nlog += 1
            self.logbuf = []

    # ---------------- perception ----------------
    def see(self, o, tool):
        ims = dec(o["img_side"]); imh = dec(o["img_head"])
        bs = vision.blob(vision.handle_mask(ims, tool))
        bh = vision.blob(vision.handle_mask(imh, tool))
        if bs is None or bh is None:
            return None, None
        p, res = vision.triangulate_blob(self.cams, bs, bh)
        if res > 0.04:
            return None, None
        ax = None
        if bs["len"] > 2.5 * max(bs["wid"], 0.8) and bh["len"] > 2.5 * max(bh["wid"], 0.8):
            ax = vision.axis_from_blobs(self.cams, bs, bh, p)
        return p, ax

    def predict(self, tool):
        tr = self.obs_track[-25:]
        if len(tr) < 4:
            return None
        T = np.array([a[0] for a in tr])
        H = np.array([a[1] for a in tr])
        # axis: sign-align by continuity
        axs = []
        prev = None
        for a in tr:
            ax = a[2]
            if ax is None:
                axs.append(None); continue
            if prev is not None and np.dot(ax, prev) < 0:
                ax = -ax
            axs.append(ax); prev = ax
        idx = [i for i, a in enumerate(axs) if a is not None]
        d = TOOLS[tool]["com"] - TOOLS[tool]["hc"]
        rot = None
        if len(idx) >= 4:
            A = np.array([axs[i] for i in idx]); Ta = T[idx]
            # rotation plane via SVD
            U, S, Vt = np.linalg.svd(A - 0 * A.mean(0), full_matrices=False)
            n = Vt[2]
            e1 = Vt[0]; e2 = np.cross(n, e1)
            th = np.unwrap(np.arctan2(A @ e2, A @ e1))
            Am = np.column_stack([np.ones_like(Ta), Ta])
            c, *_ = np.linalg.lstsq(Am, th, rcond=None)
            if S[1] < 0.15 * S[0]:  # hardly rotating
                c[1] = 0.0
                c[0] = math.atan2(np.mean(A @ e2), np.mean(A @ e1))
            rot = (e1, e2, c[0], c[1])
        def axis_at(t):
            if rot is None:
                return None
            e1, e2, a0, w = rot
            th = a0 + w * t
            return math.cos(th) * e1 + math.sin(th) * e2
        best = None
        Hc = H - 0.5 * GVEC * (T ** 2)[:, None]
        Am = np.column_stack([np.ones_like(T), T])
        for s in ([1.0, -1.0] if rot is not None else [0.0]):
            off = np.array([s * d * axis_at(t) for t in T]) if rot is not None else np.zeros_like(H)
            Y = Hc + off  # com positions (gravity-compensated)
            w = np.ones(len(T), bool)
            for it in range(3):
                c, *_ = np.linalg.lstsq(Am[w], Y[w], rcond=None)
                res = np.linalg.norm(Am @ c - Y, axis=1)
                thr = max(0.012, 3.0 * float(np.median(res[w])))
                w2 = res < thr
                if w2.sum() < 4 or np.array_equal(w2, w):
                    break
                w = w2
            r = np.sum(res[w] ** 2) / max(w.sum(), 1) * len(T)
            if best is None or r < best[0]:
                best = (r, s, c)
        r, s, c = best
        com0, v = c[0], c[1]

        def com_at(t):
            return com0 + v * t + 0.5 * GVEC * t * t

        def hc_at(t):
            a = axis_at(t)
            if a is None:
                return com_at(t)
            return com_at(t) - s * d * a
        return dict(com_at=com_at, hc_at=hc_at, axis_at=axis_at, v=v, s=s, rot=rot, rms=math.sqrt(r / len(T)))

    # ---------------- main ----------------
    def act(self, o):
        if o["throw"] != self.throw:
            self.flush()
            self.reset_throw(o["throw"])
        t = o["t"]
        tool = o["tool"]
        TD = TOOLS[tool]
        if LOGDIR:
            self.logbuf.append(o)
        if o.get("zone"):
            self.zone = o["zone"]
        wp = np.array(o["wrist_pos"]); Rw = quat_to_R(o["wrist_quat"])
        self.wp = wp
        key = (o.get("throw"), o.get("step"))
        if key != self.last_t:
            self.last_t = key
            self.base_p = self.cmd_p.copy(); self.base_R = self.cmd_R.copy()
        else:
            # repeated query of the same step: restart the rate limiter from the same base
            self.cmd_p = self.base_p.copy(); self.cmd_R = self.base_R.copy()
        touch = np.array(o["touch"])

        if self.phase in ("wait", "track"):
            try:
                p, ax = self.see(o, tool)
                if p is not None and not np.all(np.isfinite(p)):
                    p, ax = None, None
            except Exception:
                p, ax = None, None
            # closed-fist F/T baseline at the home orientation at the very start
            if t < 0.06:
                self.fingers = CLOSE.copy()
            elif t > 0.20 and t < 0.26:
                self.ft_base.append((np.array(o["ft_force"]), np.array(o["ft_torque"])))
                self.fingers = OPEN.copy()
            if p is not None:
                if not self.flying:
                    self.rest.append(p)
                    if len(self.rest) > 12:
                        self.rest.pop(0)
                    p0 = np.median(np.array(self.rest), axis=0)
                    if len(self.rest) >= 3 and np.linalg.norm(p - p0) > LAUNCH_D:
                        self.flying = True
                        self.t_launch = t
                        self.phase = "track"
                        self.p_init = p0
                if self.flying:
                    self.obs_track.append((t, p, ax))
            if t >= 0.26:
                self.target_R = R_READY
                self.target_p = np.array([0.0, 0.0, 1.1])
            if self.phase == "track":
                try:
                    pr = self.predict(tool)
                    if pr is not None:
                        self.pred = pr
                        self.plan_catch(pr, t, tool)
                except Exception:
                    pass
            if self.phase == "track" and self.t_close is not None and t >= self.t_close:
                self.phase = "caught"; self.t_phase = t
                if self.pred is not None:
                    sys.stderr.write("weH %d %.1f %s rms %.4f\n" % (o["throw"], self.we_H, self.pred["s"], self.pred["rms"]))
            if t > 0.3 and touch.sum() > 1.0 and self.phase in ("wait", "track"):
                self.phase = "caught"; self.t_phase = t
            if self.phase == "caught":
                self.fingers = CLOSE.copy()
        elif self.phase == "caught":
            self.fingers = CLOSE.copy()
            if t - self.t_phase > float(_env("T_CAUGHT","0.3")):
                self.phase = "measure"; self.t_phase = t
                self.cmd_p = wp.copy(); self.cmd_R = Rw.copy()
        else:
            self.manipulate(o, t, tool, wp, Rw, touch)
        if self.phase != getattr(self, "_lastph", None):
            sys.stderr.write("  [%d %.2f] %s\n" % (o["throw"], t, self.phase))
            self._lastph = self.phase
        self.target_p = np.clip(self.target_p, BOX_LO, BOX_HI)
        if self.phase in ("wait", "track", "caught"):
            self.cmd_p = self.target_p.copy(); self.cmd_R = self.target_R.copy()
        else:
            dp = self.target_p - self.cmd_p
            n = np.linalg.norm(dp)
            vmax = VMAX * 0.02
            self.cmd_p = self.cmd_p + (dp if n < vmax else dp * vmax / n)
            rv = R_to_rotvec(self.target_R @ self.cmd_R.T)
            n = np.linalg.norm(rv)
            wmax = WMAX * 0.02
            if n > wmax:
                rv = rv * wmax / n
                self.cmd_R = rotvec_to_R(rv) @ self.cmd_R
            else:
                self.cmd_R = self.target_R.copy()
        if not (np.all(np.isfinite(self.cmd_p)) and np.all(np.isfinite(self.cmd_R))):
            self.cmd_p = self.base_p.copy(); self.cmd_R = self.base_R.copy()
            self.target_p = self.cmd_p.copy(); self.target_R = self.cmd_R.copy()
        return {"wrist_pos": [float(x) for x in self.cmd_p],
                "wrist_rotvec": [float(x) for x in R_to_rotvec(self.cmd_R)],
                "fingers": [float(x) for x in self.fingers]}

    def plan_catch(self, pr, t, tool):
        TD = TOOLS[tool]
        # choose catch time: when handle centre crosses x = CATCH_X (or inside box)
        ts = np.arange(t + 0.02, t + 1.0, 0.01)
        best = None
        t_first = None
        bestc = 1e9
        for tt in ts:
            h = pr["hc_at"](tt)
            if h[2] < 0.80:
                break
            inside = np.all(h >= BOX_LO + [0, 0.0, 0.05]) and np.all(h <= BOX_HI - [0.0, 0.0, 0.02])
            if inside and h[0] <= P["CATCH_X"] + P["WIN_X"]:
                if t_first is None:
                    t_first = tt
                if h[0] < -0.30 or tt - t_first > P["WIN_T"]:
                    break
                a = pr["axis_at"](tt)
                vv = unit(pr["v"] + GVEC * tt)
                c = (abs(np.dot(a, vv)) if a is not None else 0.0) + P["W_T"] * abs(h[0] - P["CATCH_X"])
                if c < bestc:
                    bestc = c; best = tt
        if best is None:
            # fallback: point nearest to box
            for tt in ts:
                h = pr["hc_at"](tt)
                if h[0] <= P["CATCH_X"] or h[2] < 0.9:
                    best = tt
                    break
        if best is None:
            return
        tc = best
        h = pr["hc_at"](tc)
        a0 = pr["axis_at"](tc)
        if pr["s"] != 0 and a0 is not None:
            lo, hi = TD["handle"]
            xt = lo + AIM_FRACS.get(tool, AIM_FRAC) * (hi - lo)
            h = h + (xt - TD["hc"]) * pr["s"] * unit(a0)
        v = pr["v"] + GVEC * tc
        a = pr["axis_at"](tc)
        X = unit(unit(-v) + P["PALM_UP"] * np.array([0, 0, 1.0]))
        if a is None:
            a = np.cross(X, [0, 1.0, 0]) if abs(X[1]) < 0.9 else np.array([1.0, 0, 0])
        a = unit(a)
        Xp = X - a * np.dot(a, X)
        if np.linalg.norm(Xp) < 0.3:
            Xp = np.array([0, 0, 1.0]) - a * a[2]
        Xp = unit(Xp)
        # bias palm to face up a bit
        Y = a
        Rn = frame_from_XY(Xp, Y)
        if SIGN_MODE == 1:
            # choose the sign needing less rotation from the current command, unless fingers point well down
            R2 = frame_from_XY(Xp, -Y)
            c1 = np.linalg.norm(R_to_rotvec(Rn @ self.cmd_R.T)) + (3.0 if Rn[2, 2] < -0.3 else 0.0)
            c2 = np.linalg.norm(R_to_rotvec(R2 @ self.cmd_R.T)) + (3.0 if R2[2, 2] < -0.3 else 0.0)
            if c2 < c1:
                Rn = R2
        elif Rn[2, 2] < 0:
            # choose sign of Y so fingers (Z) point upward-ish
            Rn = frame_from_XY(Xp, -Y)
        if self.R_frozen is not None:
            Rn = self.R_frozen
        elif tc - t < ORI_FREEZE:
            self.R_frozen = Rn
        aim = np.array([0.011 + TOOLS[tool]["r"] + P["AIM"][0], P["AIM"][1], P["AIM"][2]])
        if not (np.all(np.isfinite(Rn)) and np.all(np.isfinite(h)) and np.all(np.isfinite(v))):
            return
        self.target_R = Rn
        pc = h - Rn @ aim
        vh = unit(v)
        self.t_catch = tc
        self.t_close = tc - P["T_LEAD"]
        self.t_go = tc - LAT_S - P["TAU"]
        self.p_catch = pc
        self.v_hat = vh
        if pr["s"] != 0 and pr["rot"] is not None:
            self.we_H = float(np.sign(np.dot(Rn[:, 1], pr["s"] * a)))
        else:
            self.we_H = 0.0
        if self.go is None:
            self.target_p = pc - P["MATCH_D"] * vh
            if t >= self.t_go:
                self.go = pc + P["L_END"] * vh
                # if hand far from start, skip matching (static catch)
                if np.linalg.norm(self.wp - (pc - P["MATCH_D"] * vh)) > 0.08:
                    self.go = pc
        if self.go is not None:
            self.target_p = self.go

    # ---------------- after the catch ----------------
    def manipulate(self, o, t, tool, wp, Rw, touch):
        TD = TOOLS[tool]
        if t > T_FORCE and self.phase != "present":
            if self.phase == "slide":
                self.fingers = CLOSE.copy()
            self.phase = "present"; self.vert_R = None
            if self.est is None and self.meas:
                self.estimate(tool)
            if self.est is None:
                self.aH = None
        F = np.array(o["ft_force"]); T = np.array(o["ft_torque"])
        holding = (touch > 0.2).sum() >= 2
        if self.phase == "measure":
            # palm up, keep yaw, hold still, measure F/T
            if self.meas_R is None:
                Y = self.cmd_R[:, 1].copy(); Y[2] = 0
                if np.linalg.norm(Y) < 0.2:
                    Y = self.cmd_R[:, 2].copy(); Y[2] = 0
                if np.linalg.norm(Y) < 0.2:
                    Y = np.array([1.0, 0, 0])
                self.meas_R = frame_from_XY(np.array([0, 0, 1.0]), unit(Y))
            self.target_R = self.meas_R
            self.target_p = np.array([0.0, 0.0, 1.15])
            self.fingers = CLOSE.copy()
            arrived = np.linalg.norm(self.cmd_p - self.target_p) < 1e-6 and np.allclose(self.cmd_R, self.target_R)
            if not arrived:
                self.t_phase = t
            dt = t - self.t_phase
            if dt > M_SETTLE:
                self.meas.append((F, T))
            if dt > M_SETTLE + 0.12:
                ok = self.estimate(tool)
                if not ok and self.n_retry < 2 and t < T_LAST:
                    self.n_retry += 1
                    self.meas = []
                    self.t_phase = t
                else:
                    self.est_stale = not ok
                    self.phase = "plan"; self.t_phase = t
                    self.meas_R = None
        elif self.phase == "plan":
            self.fingers = CLOSE.copy()
            if self.zone is None:
                return
            e = self.est
            if e is None or self.est_stale:
                self.phase = "present"; self.t_phase = t
                self.plan = None
                return
            lo, hi = TD[self.zone]
            x_t = lo + ZONE_FRAC * (hi - lo)
            delta = x_t - e["xg"]
            # learn slide gain from the previous slide
            if self.plan is not None and "xg0" in self.plan and self.plan.get("steps"):
                moved = (e["xg"] - self.plan["xg0"]) * np.sign(self.plan["delta"])
                if moved < -0.012 and not self.beyond and TD["com"] - TD["handle"][1] < 0.03:
                    # slid the wrong way: the grip must be beyond the centre of mass
                    self.beyond = True
                    sys.stderr.write("flip beyond\n")
                    self.estimate(tool)
                    e = self.est
                    self.plan = None
                    x_t = lo + ZONE_FRAC * (hi - lo)
                    delta = x_t - e["xg"]
            if self.plan is not None and "xg0" in self.plan and self.plan.get("steps"):
                pred = 0.011 * self.slide_k * (self.plan["steps"] - 0.8)
                if pred > 0.003:
                    r = moved / pred
                    self.slide_k = float(np.clip(self.slide_k * np.clip(r, 0.4, 2.5), 0.3, 3.0))
            inside = (lo + ZONE_MARGIN) <= e["xg"] <= (hi - 2 * ZONE_MARGIN)
            if inside or abs(delta) < 0.008 or t > T_LAST or self.n_slides >= 4:
                self.phase = "present"; self.t_phase = t
                return
            self.plan = dict(delta=delta, xg0=e["xg"])
            self.phase = "slide_orient"; self.t_phase = t
        elif self.phase == "slide_orient":
            self.fingers = CLOSE.copy()
            e = self.est
            # working end direction in H: +Y if e['sgn']>0
            we_up = self.plan["delta"] > 0  # slide toward working end -> working end up
            if self.vert_R is None:
                self.vert_R = self.R_vertical(self.cmd_R, 1.0 if we_up else -1.0)
            self.target_R = self.vert_R
            self.target_p = np.array([0.0, 0.0, 1.15])
            arrived = np.linalg.norm(self.cmd_p - self.target_p) < 1e-6 and np.allclose(self.cmd_R, self.target_R, atol=1e-3)
            if not arrived:
                self.t_phase = t
            if t - self.t_phase > O_SETTLE:
                self.phase = "slide"; self.t_phase = t
                self.vert_R = None
                d = abs(self.plan["delta"])
                self.slide_steps = int(min(MAXSTEP, max(1, round(d / (0.011 * self.slide_k) + 0.8))))
                self.plan["steps"] = self.slide_steps
                self.q_hold = np.array(o["finger_q"])
        elif self.phase == "slide":
            k = int(round((t - self.t_phase) / 0.02))
            rel = CLOSE * RELAXS.get(tool, RELAX)
            rel[12] = self.q_hold[12]
            self.fingers = rel if k < self.slide_steps else CLOSE.copy()
            if k > self.slide_steps + SLIDE_WAIT:
                self.phase = "measure" if t < T_LAST - 0.1 else "present"; self.t_phase = t
                self.meas = []
                self.n_slides += 1
        elif self.phase == "present":
            self.fingers = CLOSE.copy()
            e = self.est
            if e is None:
                self.aH = np.array([0.0, self.we_H if self.we_H != 0 else 1.0, 0.0])
            if self.vert_R is None:
                self.vert_R = self.R_vertical(self.cmd_R, 1.0)
            self.target_R = self.vert_R
            self.target_p = np.array([0.0, 0.0, 1.15])

    def R_vertical(self, Rw, sgn):
        # tool axis (toward working end, aH in H) -> sgn*up; X_H horizontal (keep current X heading)
        aH = getattr(self, "aH", None)
        if aH is None:
            aH = np.array([0.0, 1.0, 0.0])
        X = Rw[:, 0].copy(); X[2] = 0
        if np.linalg.norm(X) < 0.2:
            X = np.array([1.0, 0, 0])
        h = unit(X)
        up = np.array([0, 0, 1.0])
        Bw = np.column_stack([h, sgn * up, np.cross(h, sgn * up)])
        ex = np.array([1.0, 0, 0])
        Bh = np.column_stack([ex, aH, np.cross(ex, aH)])
        return Bw @ Bh.T

    def estimate(self, tool):
        TD = TOOLS[tool]
        if not self.meas or not self.ft_base:
            return False
        F = np.mean([m[0] for m in self.meas], 0); T = np.mean([m[1] for m in self.meas], 0)
        bF = np.mean([m[0] for m in self.ft_base], 0); bT = np.mean([m[1] for m in self.ft_base], 0)
        dF = F - bF; dT = T - bT
        m = abs(dF[0]) / G
        # reported force opposes gravity (+X when palm up); torque = r x W with W = -dF
        W = -dF
        if W.dot(W) < 1e-6:
            return False
        if not (0.5 * TD["m"] < m < 1.8 * TD["m"]):
            sys.stderr.write("est %s bad mass %.3f\n" % (tool, m))
            self.bad_meas = getattr(self, "bad_meas", 0) + 1
            return False
        r = np.cross(W, dT) / W.dot(W) * SGN_FT
        g = np.array([0.0, 0.0, -0.01])
        rv = r - g
        rv[0] = 0.0
        L = float(np.linalg.norm(rv))
        y = float(r[1])
        # mass-corrected com
        mt = TD["m"]
        com = TD["com"]
        if m > 0.3 * mt:
            dm = max(-0.3 * mt, min(0.5 * mt, m - mt))
            com = (mt * TD["com"] + dm * TD["head"]) / max(mt + dm, 1e-3)
        sgn = 1.0 if y > 0 else -1.0
        # sign bookkeeping: a weak estimate never overrides an earlier decision
        if self.sgn0 is None:
            if L >= 0.03 or self.we_H == 0:
                self.sgn0 = (sgn, L)
            else:
                self.sgn0 = (self.we_H, 0.0)
        elif L >= 0.04 and L > self.sgn0[1]:
            self.sgn0 = (sgn, L)
        sgn = self.sgn0[0]
        if L > 0.05:
            aH = rv / L
        else:
            aH = np.array([0.0, sgn, 0.0])
        if np.dot(aH, [0, sgn, 0]) < 0:
            aH = -aH
        # clamp tilt to 40 deg
        if abs(aH[2]) > 0.64:
            aH = np.array([0.0, sgn * 0.77, np.sign(aH[2]) * 0.64])
        if self.beyond:
            sgn = -sgn
            aH = -aH
            xg = com + L
        else:
            xg = com - L
        self.aH = aH
        self.est = dict(m=m, y=y, sgn=sgn, xg=xg, com=com, ay=aH[1], az=aH[2])
        sys.stderr.write("est %s %s\n" % (tool, json.dumps({k: round(float(v), 4) for k, v in self.est.items()})))
        return True


SLIDE_K = float(_env("SLIDE_K", "1.0"))
RELAX = float(_env("RELAX", "0.6"))
AIM_FRAC = float(_env("AIM_FRAC", "0.8"))
RELAXS = {}
if _env("RELAX_W"):
    RELAXS["wrench"] = float(_env("RELAX_W"))
AIM_FRACS = dict(hammer=0.95, wrench=0.95, spatula=0.65, screwdriver=0.55)
if _env("AIM_FRACS"):
    AIM_FRACS = dict(zip(["hammer", "wrench", "spatula", "screwdriver"], [float(x) for x in _env("AIM_FRACS").split(",")]))
VMAX = float(_env("VMAX", "1.0"))
WMAX = float(_env("WMAX", "5.0"))
M_SETTLE = float(_env("M_SETTLE", "0.2"))
O_SETTLE = float(_env("O_SETTLE", "0.15"))
T_LAST = float(_env("T_LAST", "5.4"))
T_FORCE = float(_env("T_FORCE", "6.0"))
MAXSTEP = int(_env("MAXSTEP", "7"))
SIGN_MODE = int(_env("SIGN_MODE", "0"))
ORI_FREEZE = float(_env("ORI_FREEZE", "0.0"))
LAUNCH_D = float(_env("LAUNCH_D", "0.07"))
ZONE_FRAC = float(_env("ZONE_FRAC", "0.4"))
ZONE_MARGIN = float(_env("ZONE_MARGIN", "0.006"))
SLIDE_WAIT = int(_env("SLIDE_WAIT", "20"))
SGN_FT = float(_env("SGN_FT", "-1.0"))


def main():
    ctl = Ctl()
    out = sys.stdout
    last = None
    out.write("READY\n"); out.flush()
    for line in sys.stdin:
        if not line.strip():
            continue
        m = json.loads(line)
        ty = m.get("type")
        if ty == "reset":
            ctl.flush(); ctl.reset_throw(-1)
            out.write('{"ok": true}\n')
        elif ty == "act":
            try:
                a = ctl.act(m["obs"])
                for k, n in (("wrist_pos", 3), ("wrist_rotvec", 3), ("fingers", 16)):
                    if len(a[k]) != n or not all(math.isfinite(x) for x in a[k]):
                        raise ValueError("bad action")
                last = a
            except Exception as e:
                import traceback
                traceback.print_exc(file=sys.stderr)
                a = last if last is not None else {"wrist_pos": [0, 0, 1.0], "wrist_rotvec": [float(x) for x in R_to_rotvec(R_HOME)],
                     "fingers": [float(x) for x in OPEN]}
            out.write(json.dumps({"action": a}) + "\n")
        elif ty == "close":
            ctl.flush()
            out.write('{"ok": true}\n'); out.flush()
            break
        out.flush()


if __name__ == "__main__":
    main()
