"""The reference program: harvests every ready truss of a row section through the cell's public actions.

It is a privileged reference: the grader adds `truth` to its observations (where every stem, leaf, peduncle and fruit
is, which trusses are ready, each peduncle's cutting force, the cart's true place) because the submission carries the
reference's token. With that it skips the perception a submission has to do, and nothing else: every motion of the
arm and the cart, every cut and every truss laid in a crate goes through the same actions, rules and physics as any
program's. Without `truth` it ends the row at once.

What it does for a ready truss: drives the cart until the plant is in front of the robot; picks, among the three
stated approach directions, one along which the tool reaches the cut point without touching a stem, a fruit, another
peduncle or a leaf; goes there in a straight line, following the peduncle as it moves; closes the jaws gently (the
opening then reads the peduncle's diameter), then with 1.15 times the peduncle's cutting force; draws the truss out
the way the tool came, swings it over a crate by the base joint with the tool as it was when it cut (the truss hangs
as it hung on the plant), turns the tool upright with its x axis along a free lane, lowers the truss until its lowest
fruit is just over the crate's floor, lays it down along the lane on an arc about that fruit, and opens the jaws.

Structure: `Reference.act(t, obs)` answers one request. A step may be asked twice with the same `t` and slightly
different readings; the program advances once per `t` and builds each answer from the observation it came with: the
plan (a generator) yields a function of an observation, and a motion's displacement is always the difference between
a planned waypoint and that observation's commanded tool pose or joint angles.
"""

import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "site.json")) as _f:
    SITE = json.load(_f)
TICK = SITE["tick_s"]
MAXT = SITE["max_ticks_per_action"]
TOOL = SITE["tool"]
OPEN = TOOL["open_max"]
TCP_Z = TOOL["tcp_z"]
FZ = SITE["cart"]["crate_floor_z"]
HOME_Q = SITE["cart"]["home_q"]
ALPHAS = (0.0,)
DEBUG = os.environ.get("REFERENCE_DEBUG") == "1"
BACK = 0.30
UP = np.array([0.0, 0.0, 1.0])


def rotvec_R(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def R_rotvec(R):
    """Rotation vector (axis times angle, angle in [0, pi]) of a rotation matrix, accurate at every angle."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = 0.5 * float(np.linalg.norm(w))
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    a = math.atan2(s, c)
    if a < 1e-9:
        return 0.5 * w
    if a < math.pi - 1e-3:
        return w / (2.0 * s) * a
    B = 0.5 * (R + R.T) - c * np.eye(3)
    i = int(np.argmax(np.diag(B)))
    k = B[:, i] / math.sqrt(max(B[i, i], 1e-300))
    if float(np.dot(k, w)) < 0.0:
        k = -k
    return k / np.linalg.norm(k) * a


def T_of(p):
    T = np.eye(4)
    T[:3, :3] = rotvec_R(p[3:6])
    T[:3, 3] = p[:3]
    return T


def mk(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def err6(T_goal, T):
    """The displacement that takes T to T_goal, as the wire's `tcp` move reads it."""
    return np.concatenate([T_goal[:3, 3] - T[:3, 3], R_rotvec(T_goal[:3, :3] @ T[:3, :3].T)])


def between(T0, T1, s):
    e = err6(T1, T0)
    return mk(rotvec_R(s * e[3:]) @ T0[:3, :3], T0[:3, 3] + s * e[:3])


def seg_dist(pts, a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ab = b - a
    t = np.clip((pts - a) @ ab / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def _box(x, y, z, n=(3, 3, 5)):
    return [(a, b, c) for a in np.linspace(x[0], x[1], n[0]) for b in np.linspace(y[0], y[1], n[1]) for c in np.linspace(z[0], z[1], n[2])]


def tool_points():
    """Sample points of the tool, the wrist camera's housing and the robot's wrist behind it, in the tool frame."""
    j, nk, bd = TOOL["jaw"], TOOL["neck"], TOOL["body"]
    hw = OPEN / 2 + j["t"]
    pts = _box(j["x"], (-hw, hw), (j["z"][0] - TCP_Z, j["z"][1] - TCP_Z), (3, 3, 3))
    pts += _box((-nk["size"][0] / 2, nk["size"][0] / 2), (-nk["size"][1] / 2, nk["size"][1] / 2), (nk["z"][0] - TCP_Z, nk["z"][1] - TCP_Z), (3, 3, 5))
    pts += _box((-bd["size"][0] / 2, bd["size"][0] / 2), (-bd["size"][1] / 2, bd["size"][1] / 2), (bd["z"][0] - TCP_Z, bd["z"][1] - TCP_Z), (3, 3, 5))
    cx, _cy, cz = SITE["cameras"]["wrist"]["pos"]
    pts += _box((cx - 0.020, cx + 0.020), (-0.022, 0.022), (cz - 0.025 - TCP_Z, cz + 0.025 - TCP_Z), (2, 2, 3))
    for zl in np.linspace(-TCP_Z - 0.32, -TCP_Z, 7):          # the robot's wrist: a generous cylinder about the tool axis
        pts += [(0.12 * math.cos(a), 0.12 * math.sin(a), zl) for a in np.linspace(0.0, 2 * math.pi, 10, endpoint=False)]
    return np.array(pts)


TOOL_PTS = tool_points()


def cut_frame(tr, alpha_deg):
    """The tool frame for a cut of the truss `tr` (truth), and the cut point: x along the peduncle toward the stem,
    z the approach direction (alpha 0: in the peduncle's vertical plane, from above and in front)."""
    a, b = np.array(tr["peduncle"][0]), np.array(tr["peduncle"][1])
    L = float(np.linalg.norm(b - a))
    lo, hi = 0.024, L - 0.010
    s = min(hi, max(lo, 0.5 * (lo + hi)))
    x = unit(a - b)
    m = unit(np.cross(x, UP))
    n1 = unit(np.cross(m, x))
    if n1[2] > 0:
        n1 = -n1
    al = math.radians(alpha_deg)
    z = math.cos(al) * n1 + math.sin(al) * np.cross(x, n1)
    return np.stack([x, np.cross(z, x), z], axis=1), a + s * unit(b - a)


def gap(truth, pts, tid, own_fruits=True, leaves=True):
    """The least distance (m, beyond each member's own radius) between the points `pts` and what stands at the row:
    stems, the fruits of the trusses on their plants (truss `tid`'s own only with `own_fruits`), the other trusses'
    peduncles and (with `leaves`) the leaves."""
    best = 1.0
    y = float(np.mean(pts[:, 1]))
    for pid, pl in truth["plants"].items():
        st = pl["stem"]
        if abs(st[0][1] - y) > 0.9:
            continue
        for s0, s1 in zip(st[:-1], st[1:]):
            best = min(best, float(seg_dist(pts, s0, s1).min()) - pl["stem_r"])
        if leaves:
            for lf in pl["leaves"]:
                best = min(best, float(seg_dist(pts, lf["petiole"][0], lf["petiole"][1]).min()) - 0.004)
                best = min(best, float(seg_dist(pts, lf["blade"][0], lf["blade"][1]).min()) - lf["width"] / 2)
    for oid, o in truth["trusses"].items():
        if o["status"] != "on_plant" or (oid == tid and not own_fruits):
            continue
        for f in o["fruits"]:
            best = min(best, float(np.linalg.norm(pts - np.array(f[:3]), axis=1).min()) - f[3])
        if oid != tid:
            best = min(best, float(seg_dist(pts, o["peduncle"][0], o["peduncle"][1]).min()) - o["r"])
    return best


def swept(R, p0, p1, n=5):
    """The tool's sample points along the straight move of its centre point from p0 to p1 (orientation R)."""
    return np.concatenate([(p0 + (p1 - p0) * s) + TOOL_PTS @ R.T for s in np.linspace(0.0, 1.0, n)])


def clearance(truth, tid, alpha_deg):
    """The least distance between the tool on its straight approach to truss `tid` and everything at the row."""
    R, p = cut_frame(truth["trusses"][tid], alpha_deg)
    return gap(truth, swept(R, p - BACK * R[:, 2], p, 7), tid)


class Reference(object):
    def __init__(self, log=None):
        self.log = log or (lambda *a: None)
        self.reset()

    def reset(self):
        self.gen, self.fn, self.t = None, None, None
        self.lanes = None

    # ------------------------------------------------------------------ the wire's side
    def act(self, t, obs):
        if "truth" not in obs:
            return {"cmd": "done"}                       # not a token run: nothing to do without the privileged view
        if t != self.t:
            self.t = t
            try:
                self.fn = next(self.gen) if self.gen is None and self._start(obs) else self.gen.send(obs)
            except StopIteration:
                self.fn = lambda o: {"cmd": "done"}
        return self.fn(obs)

    def _start(self, obs):
        self.gen = self.plan(obs)
        return True

    # ------------------------------------------------------------------ primitives (generators: `obs = yield fn`)
    def hold(self, ticks=1, grip=None, cart=0.0):
        left = int(ticks)
        obs = None
        while left > 0:
            n = min(MAXT, left)
            left -= n
            a = {"move": {"type": "hold", "ticks": n}}
            if grip is not None:
                a["gripper"] = {"opening": float(grip[0]), "force": float(grip[1])}
            if cart:
                a["cart"] = float(cart)
            obs = yield (lambda o, a=a: a)
        return obs

    def move(self, obs, T_goal, v=0.30, w=0.9, guard=None):
        """The commanded tool pose to T_goal on a straight line, in pieces of at most MAXT ticks; with `guard` (N)
        a piece ends where the wrench's force has changed by that much since the piece began."""
        T0 = T_of(obs["truth"]["tcp_target"])
        e = err6(T_goal, T0)
        n = max(2, int(math.ceil(1.5 * max(np.linalg.norm(e[:3]) / v, np.linalg.norm(e[3:]) / w) / TICK)))
        pieces = int(math.ceil(n / float(MAXT)))
        ticks = int(math.ceil(n / float(pieces)))
        for k in range(1, pieces + 1):
            Tk = between(T0, T_goal, k / float(pieces))
            obs = yield (lambda o, Tk=Tk, ticks=ticks: {"move": dict({"type": "tcp", "ticks": ticks,
                         "delta": [float(x) for x in err6(Tk, T_of(o["tcp_target"]))]},
                         **({"guard_force": float(guard)} if guard else {}))})
            if guard and obs["status"]["guard"]:
                break
        return obs

    def joints(self, obs, q_goal, speed=0.5):
        q0 = np.array(obs["truth"]["q_target"], dtype=float)
        q_goal = np.array(q_goal, dtype=float)
        n = max(2, int(math.ceil(1.5 * float(np.max(np.abs(q_goal - q0))) / speed / TICK)))
        pieces = int(math.ceil(n / float(MAXT)))
        ticks = int(math.ceil(n / float(pieces)))
        for k in range(1, pieces + 1):
            qk = q0 + (q_goal - q0) * k / float(pieces)
            obs = yield (lambda o, qk=qk, ticks=ticks: {"move": {"type": "joint", "ticks": ticks,
                         "delta": [float(x) for x in qk - np.array(o["q_target"], dtype=float)]}})
        return obs

    def cart_to(self, obs, y_goal):
        """Drive the cart until its base is at y_goal along the rail (the true place and the slip are in `truth`)."""
        for _ in range(12):
            tr = obs["truth"]
            d = y_goal - tr["cart_y"]
            if abs(d) < 0.008:
                break
            step = max(-0.30, min(0.30, d / (1.0 - tr["slip"])))
            n = max(8, int(math.ceil(1.5 * abs(step) / SITE["cart"]["speed_max"] / TICK)) + 1)
            obs = yield from self.hold(min(MAXT, n), cart=step)
        obs = yield from self.hold(6)
        return obs

    # ------------------------------------------------------------------ the plan
    def plan(self, obs):
        truth = obs["truth"]
        self.lanes = [dict(bay, used=False) for bay in SITE["cart"]["bays"]]         # a bay takes one truss
        todo = [tid for tid, tr in truth["trusses"].items() if tr["ready"]]
        # plant by plant along the rail, the lowest truss first
        todo.sort(key=lambda tid: (truth["trusses"][tid]["plant"], truth["trusses"][tid]["peduncle"][0][2]))
        self.log("ready trusses:", todo)
        for tid in todo:
            obs = yield from self.harvest(obs, tid)
        obs = yield from self.joints(obs, HOME_Q)
        yield (lambda o: {"cmd": "done"})

    def harvest(self, obs, tid):
        truth = obs["truth"]
        tr = truth["trusses"][tid]
        if tr["status"] != "on_plant" or truth["plants"][tr["plant"]]["lost"]:
            return obs
        t0 = truth["time"]
        # the cart: the truss's node in front of the robot
        obs = yield from self.cart_to(obs, tr["peduncle"][0][1] + truth["cart_y"])
        truth = obs["truth"]
        # the approach: the clearest of the stated directions
        scored = [(clearance(truth, tid, al), -abs(al), al) for al in ALPHAS]
        clear, _, alpha = max(scored)
        R, p = cut_frame(truth["trusses"][tid], alpha)
        T_far = mk(R, p - BACK * R[:, 2])
        # turn the tool in the aisle, then go to the start of the straight approach
        T_aisle = mk(R, np.array([min(T_far[0, 3], 0.80), T_far[1, 3], T_far[2, 3]]))
        obs = yield from self.move(obs, T_aisle)
        obs = yield from self.move(obs, T_far, v=0.20)
        # in, following the peduncle
        for back in (0.20, 0.12, 0.06, 0.02, 0.0, 0.0):
            R, p = cut_frame(obs["truth"]["trusses"][tid], alpha)
            obs = yield from self.move(obs, mk(R, p - back * R[:, 2]), v=0.10, w=0.5)
        # the caliper, then the cut
        obs = yield from self.hold(14, grip=(0.0, 4.0))
        d_read = obs["gripper"]["opening"]
        need = obs["truth"]["trusses"][tid]["need"]
        obs = yield from self.hold(10, grip=(0.0, 1.15 * need))
        truth = obs["truth"]
        self.log("%s: approach %+.0f (clear %.3f m), caliper %.2f mm, cut at %.0f N -> %s" % (
            tid, alpha, clear, d_read * 1000, 1.15 * need, truth["trusses"][tid]["status"]))
        T_now = T_of(truth["tcp_target"])
        T_out = mk(T_now[:3, :3], T_now[:3, 3] - BACK * T_now[:3, 2])
        T_back = mk(T_now[:3, :3], np.array([min(T_out[0, 3], 0.80), T_out[1, 3], T_out[2, 3]]))
        if truth["trusses"][tid]["status"] != "held":
            obs = yield from self.hold(16, grip=(OPEN, 0.0))
            obs = yield from self.move(obs, T_out, v=0.15)
            obs = yield from self.move(obs, T_back)
            obs = yield from self.joints(obs, HOME_Q)
            return obs
        # the way out: first away from the stem along the peduncle's own direction (the truss hangs beside the stem:
        # drawn off sideways, a fruit can catch behind it), as far as leaves the tool and the hanging truss clear on
        # the way back out, parallel to the way the tool came
        h = np.array([-T_now[0, 0], -T_now[1, 0], 0.0])
        h = h / np.linalg.norm(h)
        Rn, pn = T_now[:3, :3], T_now[:3, 3]
        fr = np.array([f[:3] for f in truth["trusses"][tid]["fruits"]])
        rf = max(f[3] for f in truth["trusses"][tid]["fruits"])
        best = None
        for shift in (0.10, 0.07, 0.04, 0.02, 0.0):
            a, b = pn + shift * h, pn + shift * h - BACK * Rn[:, 2]
            tool_gap = min(gap(truth, swept(Rn, pn, a, 3), tid, own_fruits=False), gap(truth, swept(Rn, a, b, 7), tid, own_fruits=False))
            hang = np.concatenate([fr + (a - pn) * s for s in np.linspace(0, 1, 3)] + [fr + (a - pn) + (b - a) * s for s in np.linspace(0, 1, 7)])
            truss_gap = gap(truth, hang, tid, own_fruits=False, leaves=False) - rf
            score = min(tool_gap - 0.004, truss_gap)
            if best is None or score > best[0] + (0.0 if best[0] < 0.006 else 1.0):
                best = (score, shift, tool_gap, truss_gap)
        _score, shift, tool_gap, truss_gap = best
        self.log("%s: out by %.2f m along the peduncle (tool clear %.3f m, truss clear %.3f m)" % (tid, shift, tool_gap, truss_gap))
        if shift > 0:
            obs = yield from self.move(obs, mk(Rn, pn + shift * h), v=0.06)
        T_out = mk(Rn, pn + shift * h - BACK * Rn[:, 2])
        T_back = mk(Rn, np.array([min(T_out[0, 3], 0.80), T_out[1, 3], T_out[2, 3]]))
        obs = yield from self.move(obs, T_out, v=0.10)
        obs = yield from self.move(obs, T_back, v=0.20)
        obs = yield from self.move(obs, mk(T_now[:3, :3], np.array([0.80, 0.0, 0.40])), v=0.25)
        # over a crate by the base joint, the tool as it is: the truss hangs as it hung
        xt = T_now[:3, 0]
        psi = math.atan2(xt[1], xt[0])
        free = [ln for ln in self.lanes if not ln["used"]]
        if not free:
            self.log("no lane left for", tid)
            return obs
        sides = sorted({1.0 if ln["y0"] > 0 else -1.0 for ln in free},
                       key=lambda s: abs((s * math.pi / 2 - psi + math.pi) % (2 * math.pi) - math.pi))
        sgn = sides[0]
        lane = [ln for ln in free if (ln["y0"] > 0) == (sgn > 0)][0]
        lane["used"] = True
        dpsi = (sgn * math.pi / 2 - psi + math.pi) % (2 * math.pi) - math.pi
        q = np.array(obs["truth"]["q_target"], dtype=float)
        q[0] += dpsi
        obs = yield from self.joints(obs, q, speed=0.45)
        # upright over the lane, x along it: the truss trails behind the clamp
        u = np.array([0.0, sgn, 0.0])
        Rv = np.stack([u, np.cross(-UP, u), -UP], axis=1)
        y_far = lane["y1"] if sgn > 0 else lane["y0"]
        y_near = lane["y0"] if sgn > 0 else lane["y1"]
        # the peduncle's stub stands out ahead of the clamp: the clamp stops that much short of the lane's far wall
        Tc = T_of(obs["truth"]["tool"])
        stub = float(np.linalg.norm(np.array(obs["truth"]["trusses"][tid]["peduncle"][0]) - Tc[:3, 3]))
        y_c = y_far - sgn * (stub + 0.030)
        T_sw = T_of(obs["truth"]["tcp_target"])
        obs = yield from self.move(obs, mk(T_sw[:3, :3], np.array([lane["x"], y_c - sgn * 0.20, 0.25])), v=0.25)
        obs = yield from self.move(obs, mk(Rv, np.array([lane["x"], y_c - sgn * 0.20, 0.25])), w=0.6)
        obs = yield from self.hold(16)
        # how the truss hangs now: its lowest fruit, and how far it trails behind the clamp
        truth = obs["truth"]
        fr = truth["trusses"][tid]["fruits"]
        low = min(fr, key=lambda f: f[2] - f[3])
        tcp = T_of(truth["tcp_target"])[:3, 3]
        hang = tcp[2] - (low[2] - low[3])
        lag = max(0.0, sgn * (tcp[1] - low[1]))
        # laid by feel, like a stick let down about its foot: the lowest fruit comes down on the floor, and the clamp
        # goes along the bay on an arc about it. A truss is a chain of fat links: let down, it now and then stands on
        # another fruit and pushes back. The wrench tells: while the truss pushes, the clamp only goes on along the bay
        # (and a little up), which draws the truss out, and the arc is taken up again from there, wider.
        z_end = FZ + 0.100
        L = math.hypot(lag, hang + 0.010)
        phi1 = math.acos(min(1.0, (z_end - FZ) / L))
        y_piv = y_c - sgn * L * math.sin(phi1)
        if sgn * (y_piv - y_near) < 0.045:               # a long truss: its foot stays clear of the bay's near wall
            y_piv = y_near + sgn * 0.045
        y, z = y_piv + sgn * lag, FZ + hang + 0.010
        D = abs(y_c - y)
        obs = yield from self.move(obs, mk(Rv, np.array([lane["x"], y, 0.25])), v=0.2)
        obs = yield from self.move(obs, mk(Rv, np.array([lane["x"], y, z])), v=0.15)
        obs = yield from self.hold(6)
        phi = math.atan2(lag, hang + 0.010)
        pushed = 0
        for _k in range(160):
            if z <= z_end + 1e-4:
                break
            push = obs["wrench"][2] > 1.5 or obs["status"]["guard"]
            left = sgn * (y_c - y)
            if push:
                pushed += 1
                if left < 0.004:
                    break                                 # at the bay's end and still pushed: let go here
                y, z = y + sgn * min(left, 0.012), z + 0.002
                L = math.hypot(y - y_piv, z - FZ)
                phi = math.atan2(sgn * (y - y_piv), z - FZ)
            else:
                phi += 0.008 / L
                y_n, z_n = y_piv + sgn * L * math.sin(phi), FZ + L * math.cos(phi)
                if sgn * (y_n - y_c) > 0:                 # at the bay's end: straight down from here
                    y_n, z_n = y_c, z - 0.004
                    L = math.hypot(y_n - y_piv, z_n - FZ)
                    phi = math.atan2(sgn * (y_n - y_piv), z_n - FZ)
                y, z = y_n, max(z_end, z_n)
            obs = yield from self.move(obs, mk(Rv, np.array([lane["x"], y, z])), v=0.03, guard=2.5)
            if DEBUG:
                self.log("   lay: tcp y %.3f z %.3f fz %.2f%s" % (y, z - FZ, obs["wrench"][2], " pushed" if push else ""))
            if obs["status"]["stopped"]:
                break
        # let go there, a hand's breadth over the floor: the first fruits lie under the clamp, and the clamp brought
        # lower would press them into the floor
        obs = yield from self.hold(12)
        z = T_of(obs["truth"]["tcp_target"])[2, 3]
        obs = yield from self.hold(18, grip=(OPEN, 0.0))
        T_end = T_of(obs["truth"]["tcp_target"])
        obs = yield from self.move(obs, mk(Rv, np.array([T_end[0, 3], T_end[1, 3], 0.25])), v=0.25)
        st = obs["truth"]["trusses"][tid]
        self.log("%s: laid in the %s crate, lane x %.2f (hang %.3f m, drawn out %.2f m, let go %.3f m over the floor), %s%s, %.1f s" % (
            tid, lane["crate"], lane["x"], hang, D, z - FZ, st["status"], ", DAMAGED" if st["damaged"] else "",
            obs["truth"]["time"] - t0))
        obs = yield from self.joints(obs, HOME_Q)
        return obs
