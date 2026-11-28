"""Reference controller of spindle_ring_sorting: re-sorts the rings of the fixture using only what the cell reports
(the cameras' images, the hand's pose, the jaw opening and the wrist force/torque sensor) and the cell's documents.

Per episode:
  1. Survey: the wrist camera looks down on every spindle from above (camera over the drawing's spindle position);
     the top ring's disc (or the bare tip) gives the spindle's position; the overview image gives each stack's
     colours (four bands per full stack), the wrist image the top ring's colour.
  2. Plan: weighted A* over arrangements (any ring may go on any spindle with room).
  3. Every move: look at the source from above (check the top ring's colour, refine the position), grasp the ring
     with the jaws across the fixture, lift it off the spindle's tip, look at the target from above, carry the ring
     there and thread it: descend gently; if the ring stops on the tip (the hand stalls above the tip's height, the
     wrist force rises), slide it in a spiral on the tip with a light push until it drops in; then set it down on
     the stack and let go. The offset where it went in corrects the camera's aim for the rest of the episode.
  4. A mismatch between the belief and what the wrist camera sees re-plans from the corrected belief.

Speaks the grader's stdio wire (run.sh). Answers a repeated step t (the grader's probe) from the observation it
carries, without advancing.
"""
import base64
import heapq
import json
import math
import sys

import numpy as np

# ------------------------------------------------------------------------------------------ the cell's documents
PLATE_TOP = 0.812
SP_LEN = 0.074
TIP_Z = PLATE_TOP + SP_LEN
RING_H = 0.016
LEVEL = 0.0162                      # ring pitch in a stack (16 mm ring on its 0.2 mm feet)
R_OUT = 0.020
NSP = 7
CAP = 4
NOMINAL = [(0.07, (k - 3) * 0.068) for k in range(NSP)]
PAD = 0.0085                        # pad centre above the ring's bottom face when grasping
CAM_IN_TCP = np.array([0.0, 0.05, -0.1604])
R_CAM_IN_TCP = np.diag([-1.0, 1.0, -1.0])
WRIST_W, WRIST_H, WRIST_FOVY = 128, 96, 75.0
OV_W, OV_H, OV_FOVY = 240, 180, 40.0
OV_POS = np.array([0.52, -0.42, 1.22])
OV_TGT = np.array([0.06, 0.0, 0.84])
Z_SAFE = 0.92                       # TCP height that clears every full stack with open jaws
Z_CAM_LOOK = 1.08                   # wrist camera height when looking down on a spindle
PARK = np.array([-0.15, 0.0, 1.00])  # out of the overview camera's view
Z_LOW = 0.900                        # TCP height with the jaws 8 mm above the tips of full stacks
SHIFT_MM = 0.004                     # a look that finds a spindle this far off means the fixture moved
# overview chroma (r, g) of the six variants (from practice images)
CHROMA = np.array([[0.093, 0.211], [0.780, 0.108], [0.159, 0.587]])   # blue, red, green (overview chroma r, g)
HEAVY_N = 0.76                      # weighing threshold: plain ring 0.44 N, steel-cored ring 1.08 N


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def look_at(pos, target, up=(0, 0, 1)):
    f = np.asarray(target, float) - np.asarray(pos, float)
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, float), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


R_OV = look_at(OV_POS, OV_TGT)
F_OV = (OV_H / 2) / math.tan(math.radians(OV_FOVY) / 2)
F_WR = (WRIST_H / 2) / math.tan(math.radians(WRIST_FOVY) / 2)


def project(P, pc, Rc, f, w, h):
    q = Rc.T @ (np.asarray(P, float) - pc)
    return np.array([w / 2 + f * q[0] / -q[2], h / 2 - f * q[1] / -q[2]])


def backproject(uv, pc, Rc, f, w, h, z):
    d = Rc @ np.array([(uv[0] - w / 2) / f, -(uv[1] - h / 2) / f, -1.0])
    s = (z - pc[2]) / d[2]
    return pc + d * s


def decode(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def chroma(rgb):
    rgb = np.asarray(rgb, float)
    return rgb[..., :2] / np.maximum(rgb.sum(-1, keepdims=True), 1.0)


def classify(rgb):
    c = chroma(rgb)
    return int(np.argmin(((CHROMA - c) ** 2).sum(1)))


def saturated(img):
    f = img.astype(np.int32)
    mx, mn = f.max(-1), f.min(-1)
    return (mx - mn > 30) & ((mx - mn) > 0.3 * mx)


def components(mask):
    """Connected components (4-neighbour) of a boolean image: list of (N x 2 array of (row, col))."""
    h, w = mask.shape
    seen = np.zeros_like(mask, bool)
    out = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack = [(r0, c0)]
        seen[r0, c0] = True
        pts = []
        while stack:
            r, c = stack.pop()
            pts.append((r, c))
            for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
        out.append(np.array(pts))
    return out


# ------------------------------------------------------------------------------------------ planning
def is_pure(s):
    return len(s) > 0 and all(v[0] == s[0][0] for v in s)


def solved(st, counts):
    """Every variant on a spindle of its own, complete, and every ring's variant verified."""
    seen = set()
    for s in st:
        if not s:
            continue
        v = s[0][0]
        if any(r[0] != v or not r[1] for r in s) or len(s) != counts[v] or v in seen:
            return False
        seen.add(v)
    return True


def heuristic(st):
    h = 0
    for s in st:
        if not s:
            continue
        k = 0
        while k < len(s) and s[k][0] == s[0][0]:
            k += 1
        h += len(s) - k
        for j in range(k):                  # an unverified ring in the pure part must still come off (and all above)
            if not s[j][1]:
                h += k - j
                break
    return h


def plan(state, weight=2.0, max_expand=120000, forbid=None):
    """Weighted A* over arrangements of (variant, verified) rings; moving a ring verifies it (it is weighed)."""
    counts = {}
    for s in state:
        for r in s:
            counts[r[0]] = counts.get(r[0], 0) + 1
    start = tuple(tuple((int(r[0]), bool(r[1])) for r in s) for s in state)
    key = lambda st: tuple(sorted(st))
    openq = [(weight * heuristic(start), 0, 0, start)]
    parent = {key(start): (None, None)}
    gbest = {key(start): 0}
    n = 0
    while openq:
        _f, g, _n, st = heapq.heappop(openq)
        if solved(st, counts):
            path, node = [], st
            while True:
                prev, mv = parent[key(node)]
                if prev is None:
                    break
                path.append(mv)
                node = prev
            return path[::-1]
        n += 1
        if n > max_expand:
            break
        for a in range(len(st)):
            if not st[a]:
                continue
            for b in range(len(st)):
                if a == b or len(st[b]) >= CAP or (g == 0 and (a, b) == forbid):
                    continue
                ns = [list(s) for s in st]
                r = ns[a].pop()
                ns[b].append((r[0], True))
                nt = tuple(tuple(s) for s in ns)
                k = key(nt)
                if k in gbest and gbest[k] <= g + 1:
                    continue
                gbest[k] = g + 1
                parent[k] = (st, (a, b))
                heapq.heappush(openq, (g + 1 + weight * heuristic(nt), g + 1, n, nt))
    return None


def guess(belief):
    """Fill in the unverified rings. Known counts above four (a doubtful weighing) are undone first; rings settled by
    elimination (four of a variant already verified) become verified; the other unverified rings keep the guess they
    had, and only as many guesses are flipped as the counts (four of every variant) require, so that the plan does not
    churn from one move to the next."""
    out = [[list(r) + [0.0] * (3 - len(r)) for r in s] for s in belief]
    for col in range(3):
        for var in (2 * col, 2 * col + 1):
            known = sorted([(out[k][j][2], k, j) for k, s in enumerate(out) for j, r in enumerate(s)
                            if r[1] and r[2] > 0 and r[0] == var])
            while sum(1 for s in out for r in s if r[1] and r[0] == var) > 4 and known:
                _m, k, j = known.pop(0)
                out[k][j][1] = False
        kh = sum(1 for s in out for r in s if r[1] and r[0] == 2 * col + 1)
        kl = sum(1 for s in out for r in s if r[1] and r[0] == 2 * col)
        unk = [(k, j) for k, s in enumerate(out) for j, r in enumerate(s) if not r[1] and r[0] // 2 == col]
        need_h = max(0, 4 - kh)
        need_l = max(0, 4 - kl)
        if need_h == 0 or need_l == 0:
            for k, j in unk:
                out[k][j] = [2 * col + (1 if need_h > 0 else 0), True, 0.0]
            continue
        guessed_h = [(k, j) for k, j in unk if out[k][j][0] == 2 * col + 1]
        guessed_l = [(k, j) for k, j in unk if out[k][j][0] == 2 * col]
        # flip the fewest guesses: the lowest rings first (they are the last to be seen)
        while len(guessed_h) > need_h:
            k, j = max(guessed_h, key=lambda kj: -kj[1])
            guessed_h.remove((k, j))
            out[k][j][0] = 2 * col
            guessed_l.append((k, j))
        while len(guessed_h) < need_h:
            k, j = min(guessed_l, key=lambda kj: kj[1])
            guessed_l.remove((k, j))
            out[k][j][0] = 2 * col + 1
            guessed_h.append((k, j))
    return out


# ------------------------------------------------------------------------------------------ the controller
class Cmd(object):
    __slots__ = ("p", "rel", "yaw", "grip", "cams")

    def __init__(self, p=None, rel=None, yaw=None, grip=-1.0, cams=0):
        self.p, self.rel, self.yaw, self.grip, self.cams = p, rel, yaw, grip, cams


class Controller(object):
    def __init__(self, log=None):
        self.log = log or (lambda *a: None)
        self.reset()

    def reset(self):
        self.gen = self.program()
        self.cmd = None
        self.last_t = None
        self.obs = None
        self.started = False

    # ---------------------------------------------------------------- wire
    def act(self, t, obs):
        if t != self.last_t:
            self.last_t = t
            self.obs = obs
            if not self.started:
                self.started = True
                self.cmd = next(self.gen)
            else:
                self.cmd = self.gen.send(obs)
        return self.to_action(self.cmd, obs)

    def to_action(self, c, obs):
        ee = np.asarray(obs["ee_pos"], float)
        if c.rel is not None:
            dp = np.asarray(c.rel, float)
        elif c.p is not None:
            dp = np.asarray(c.p, float) - ee
        else:
            dp = np.zeros(3)
        n = float(np.linalg.norm(dp))
        if n > 0.02:
            dp = dp * (0.02 / n)
        dyaw = 0.0 if c.yaw is None else float(np.clip(wrap(c.yaw - float(obs["ee_yaw"])), -0.15, 0.15))
        return [round(float(dp[0]), 6), round(float(dp[1]), 6), round(float(dp[2]), 6), round(dyaw, 6),
                float(c.grip), int(c.cams)]

    # ---------------------------------------------------------------- helpers (generators)
    def ee(self):
        return np.asarray(self.obs["ee_pos"], float)

    def wrench(self):
        return np.asarray(self.obs["wrench"], float)

    def step(self, p=None, rel=None, cams=0):
        yield Cmd(p=p, rel=rel, yaw=self.yaw, grip=self.grip, cams=cams)

    def goto(self, target, v=0.018, tol=0.0015, max_steps=150, settle=0):
        target = np.asarray(target, float)
        for _ in range(max_steps):
            e = target - self.ee()
            n = float(np.linalg.norm(e))
            yaw_ok = abs(wrap(self.yaw - float(self.obs["ee_yaw"]))) < 0.01
            if n < tol and yaw_ok:
                break
            yield Cmd(rel=e * min(1.0, v / max(n, 1e-9)), yaw=self.yaw, grip=self.grip)
        for _ in range(settle):
            yield Cmd(p=target, yaw=self.yaw, grip=self.grip)

    def hold(self, n, cams=0):
        for k in range(n):
            yield Cmd(rel=np.zeros(3), yaw=self.yaw, grip=self.grip, cams=cams if k == n - 1 else 0)

    def wrist_cam(self):
        R = quat_mat(np.asarray(self.obs["ee_quat"], float))
        return self.ee() + R @ CAM_IN_TCP, R @ R_CAM_IN_TCP

    def tcp_for_cam_over(self, xy, z_cam=Z_CAM_LOOK):
        """The TCP position that puts the wrist camera right above xy at height z_cam (current hand yaw)."""
        R = quat_mat(np.asarray(self.obs["ee_quat"], float))
        off = R @ CAM_IN_TCP
        return np.array([xy[0] - off[0], xy[1] - off[1], z_cam - off[2]])

    def snap_wrist(self):
        """Hold still and return the wrist image of the next observation with its camera pose."""
        yield from self.hold(3, cams=2)
        img = decode(self.obs["wrist"])
        pc, Rc = self.wrist_cam()
        return img, pc, Rc

    # ---------------------------------------------------------------- perception
    def detect_top(self, img, pc, Rc, xy_guess, n_rings):
        """Position (x, y) and colour class of the top of spindle at xy_guess holding n_rings (0: the bare tip)."""
        if n_rings > 0:
            z = PLATE_TOP + (n_rings - 1) * LEVEL + RING_H + 0.0004
        else:
            z = TIP_Z
        uv0 = project([xy_guess[0], xy_guess[1], z], pc, Rc, F_WR, WRIST_W, WRIST_H)
        if n_rings > 0:
            mask = saturated(img)
        else:
            g = img.astype(np.int32)
            mask = (g.min(-1) > 200) & ((g.max(-1) - g.min(-1)) < 40)
        best, bd = None, 1e9
        for comp in components(mask):
            if len(comp) < (6 if n_rings > 0 else 2):
                continue
            c = comp.mean(0)[::-1] + 0.5
            dd = float(np.linalg.norm(c - uv0))
            if dd < bd:
                best, bd = comp, dd
        self.last_bd = bd
        if best is None or bd > (14 if n_rings > 0 else 7):
            return None, None
        if n_rings > 0:
            # the disc's outline: fill the tip's hole by taking the extent-weighted centre
            rows, cols = best[:, 0], best[:, 1]
            c = np.array([(cols.min() + cols.max() + 1) / 2.0, (rows.min() + rows.max() + 1) / 2.0])
            c = 0.5 * c + 0.5 * (best.mean(0)[::-1] + 0.5)
            pix = img[rows, cols].astype(float)
            keep = pix.max(1) < 250
            col = (pix[keep] if keep.sum() >= 3 else pix).mean(0)
            cls = classify(col)
        else:
            c = best.mean(0)[::-1] + 0.5
            cls = None
        P = backproject(c, pc, Rc, F_WR, WRIST_W, WRIST_H, z)
        return P[:2], cls

    def read_overview(self, img, counts):
        """Colour distances (n x 6 per stack, bottom to top) of the rings of every stack, from the overview image;
        counts[k] rings on spindle k. The projection of the drawing's camera is registered per stack on the
        column's bottom edge (the real camera sits a little differently)."""
        sat = saturated(img)
        out = []
        for k in range(NSP):
            n = counts[k]
            if n == 0:
                out.append(np.zeros((0, 6)))
                continue
            x, y = self.sp[k]
            d = OV_POS[:2] - np.array([x, y])
            d = d / np.linalg.norm(d)
            front = np.array([x, y]) + d * (R_OUT * 0.85)
            bot = project([front[0], front[1], PLATE_TOP + 0.0004], OV_POS, R_OV, F_OV, OV_W, OV_H)
            centres = [project([front[0], front[1], PLATE_TOP + 0.0002 + j * LEVEL + RING_H / 2], OV_POS, R_OV, F_OV,
                               OV_W, OV_H) for j in range(n)]
            top = project([front[0], front[1], PLATE_TOP + n * LEVEL], OV_POS, R_OV, F_OV, OV_W, OV_H)
            # the column: the pixel column (near the prediction) with the most saturated pixels in the stack's range
            best = None
            for du in range(-4, 5):
                u = int(round(bot[0])) + du
                if u < 2 or u >= OV_W - 2:
                    continue
                v0, v1 = int(top[1]) - 3, int(bot[1]) + 4
                cnt = int(sat[max(0, v0):min(OV_H, v1), u].sum())
                if best is None or cnt > best[0]:
                    best = (cnt, u)
            u = best[1]
            du = u - bot[0]
            # the bottom edge: the lowest saturated pixel of the column near the predicted bottom
            vs = [v for v in range(int(bot[1]) - 6, min(OV_H, int(bot[1]) + 7)) if sat[v, u]]
            dv = (max(vs) + 1 - bot[1]) if vs else 0.0
            rows = []
            for c in centres:
                vc, uc = c[1] + dv, c[0] + du
                pix = []
                for r in (int(round(vc - 0.5)) + o for o in (-1, 0, 1)):
                    for cc in (int(round(uc - 0.5)) + o for o in (-2, -1, 0, 1, 2)):
                        if 0 <= r < OV_H and 0 <= cc < OV_W and sat[r, cc]:
                            pix.append(img[r, cc])
                if len(pix) < 3:
                    rows.append(np.ones(3))
                    continue
                ch = chroma(np.mean(np.asarray(pix, float), 0))
                rows.append(((CHROMA - ch) ** 2).sum(1))
            out.append(np.asarray(rows))
        return out

    def assign(self, dists, counts):
        """Colour group of every ring from the colour distances, with exactly eight rings of every colour (the
        balanced assignment of least total distance). Returns a belief of unverified rings."""
        from scipy.optimize import linear_sum_assignment
        items = [(k, j) for k in range(NSP) for j in range(counts[k])]
        if not items:
            return [[] for _ in range(NSP)]
        cost = np.array([dists[k][j] for k, j in items])                 # n x 3
        per = int(math.ceil(len(items) / 3.0))
        big = np.repeat(cost, per, axis=1)
        r, cc = linear_sum_assignment(big)
        belief = [[None] * counts[k] for k in range(NSP)]
        for i, col in zip(r, cc):
            k, j = items[i]
            belief[k][j] = [2 * int(col // per), False]
        return guess(belief)

    # ---------------------------------------------------------------- manipulation
    def pick(self, k):
        """Grasp the top ring of spindle k and lift it off the tip. Returns True when the jaws hold a ring."""
        n = len(self.belief[k])
        x, y = self.sp[k]
        z_ring = PLATE_TOP + (n - 1) * LEVEL + 0.0004
        self.grip = -1.0
        if self.ee()[2] < Z_LOW - 0.003:
            yield from self.goto([self.ee()[0], self.ee()[1], Z_LOW], tol=0.003)
        # stay low (jaws just above the tips) with the jaws open: the pallet is never bumped then, so the look that
        # came just before is still true when the jaws close
        yield from self.goto([x, y, Z_LOW], tol=0.002)
        # tare the wrist sensor with the hand still and empty
        tare = []
        for _ in range(9):
            yield Cmd(p=[x, y, Z_LOW], yaw=self.yaw, grip=self.grip)
            tare.append(self.wrench()[2])
        tare = float(np.mean(tare[3:]))
        yield from self.goto([x, y, z_ring + PAD], v=0.012, tol=0.001, settle=2)
        self.grip = 1.0
        yield from self.hold(6)
        if float(self.obs["gripper"]) < 0.030:
            self.log("pick %d: no ring between the jaws (%.3f)" % (k, float(self.obs["gripper"])))
            self.grip = -1.0
            yield from self.hold(3)
            yield from self.goto([self.ee()[0], self.ee()[1], Z_LOW], tol=0.004)
            return False
        # the jaws centre the ring across the fixture: the spindle's x is the hand's x
        self.sp[k] = np.array([0.7 * (self.ee()[0] - self.bias[0]) + 0.3 * x, y])
        lift_to = TIP_Z + 0.012 + PAD
        f0 = self.wrench()
        while self.ee()[2] < lift_to - 0.001:
            w = self.wrench()
            lat = np.clip(-0.0003 * (w[:2] - f0[:2]), -0.001, 0.001)
            yield Cmd(rel=[lat[0], lat[1], min(0.010, lift_to - self.ee()[2] + 0.002)], yaw=self.yaw, grip=self.grip)
        yield from self.goto([self.ee()[0], self.ee()[1], max(Z_SAFE, lift_to)], tol=0.003)
        if float(self.obs["gripper"]) < 0.030:
            self.log("pick %d: ring lost on the lift" % k)
            return False
        # weigh it: hold still and compare with the empty hand's reading
        hold_p = self.ee().copy()
        wz = []
        for _ in range(13):
            yield Cmd(p=hold_p, yaw=self.yaw, grip=self.grip)
            wz.append(self.wrench()[2])
        load = float(np.mean(wz[3:])) - tare
        self.last_heavy = load > HEAVY_N
        self.last_margin = abs(load - HEAVY_N)
        self.log("pick %d: load %.2f N -> %s" % (k, load, "steel-cored" if self.last_heavy else "plain"))
        return True

    def thread(self, k):
        """Thread the held ring onto spindle k. Returns "ok" or "no_entry"."""
        n = len(self.belief[k])
        aim = np.asarray(self.sp[k], float) + self.bias
        for attempt in range(2):
            max_r = 0.0045 if attempt == 0 else 0.0075
            z_hover = TIP_Z + 0.004 + PAD
            yield from self.goto([aim[0], aim[1], max(self.ee()[2], z_hover)], tol=0.002)
            yield from self.goto([aim[0], aim[1], z_hover], v=0.01, tol=0.0008, settle=2)
            f0 = np.zeros(6)
            for _ in range(3):
                yield Cmd(rel=np.zeros(3), yaw=self.yaw, grip=self.grip)
                f0 += self.wrench() / 3.0
            rb = lambda: self.ee()[2] - PAD
            entered, z_c = False, None
            n_in, n_st = 0, 0                 # decisions need two readings in a row (a probe copy is one reading)
            for _ in range(22):
                z0 = self.ee()[2]
                xy = aim - self.ee()[:2]
                yield Cmd(rel=[xy[0], xy[1], -0.002], yaw=self.yaw, grip=self.grip)
                fz = f0[2] - self.wrench()[2]
                n_in = n_in + 1 if rb() < TIP_Z - 0.004 else 0
                if n_in >= 2:
                    entered = True
                    break
                n_st = n_st + 1 if ((z0 - self.ee()[2] < 0.0005 and rb() < TIP_Z + 0.002) or fz > 6.0) else 0
                if n_st >= 2:
                    z_c = self.ee()[2]
                    break
            if not entered:
                if z_c is None:
                    z_c = self.ee()[2]
                th, pitch, arc = 0.0, 0.0006, 0.0007
                while True:
                    r = pitch * th / (2 * math.pi)
                    if r > max_r:
                        break
                    th += arc / max(r, 0.0006)
                    r = pitch * th / (2 * math.pi)
                    p = aim + r * np.array([math.cos(th), math.sin(th)])
                    cur = self.ee()
                    yield Cmd(rel=[p[0] - cur[0], p[1] - cur[1], -0.0015], yaw=self.yaw, grip=self.grip)
                    if self.wrench()[2] < f0[2] - 12.0:
                        yield Cmd(rel=[0, 0, 0.002], yaw=self.yaw, grip=self.grip)
                    if self.ee()[2] < z_c - 0.004:
                        yield Cmd(rel=[0, 0, -0.0015], yaw=self.yaw, grip=self.grip)
                        if self.ee()[2] < z_c - 0.004:
                            entered = True
                            break
                    if self.ee()[2] < z_c - 0.0008:
                        # the ring's edge caught in the bore's mouth: stop sliding, let the lateral force centre it
                        for _ in range(8):
                            w = self.wrench()
                            lat = np.clip(-0.0004 * (w[:2] - f0[:2]), -0.0008, 0.0008)
                            yield Cmd(rel=[lat[0], lat[1], -0.0015], yaw=self.yaw, grip=self.grip)
                            if self.ee()[2] < z_c - 0.004:
                                yield Cmd(rel=[0, 0, -0.0015], yaw=self.yaw, grip=self.grip)
                                if self.ee()[2] < z_c - 0.004:
                                    entered = True
                                    break
                        if entered:
                            break
                        cur = self.ee()
                        yield Cmd(rel=[0, 0, z_c + 0.0005 - cur[2]], yaw=self.yaw, grip=self.grip)
            if entered:
                found = self.ee()[:2].copy()
                # the hand's xy where the ring went in = the spindle's axis (ring centred in the jaws)
                self.bias = 0.5 * self.bias + 0.5 * (found - np.asarray(self.sp[k], float))
                self.sp[k] = np.asarray(self.sp[k], float) * 0.3 + (found - self.bias) * 0.7
                self.confirmed[k] = True
                break
            self.log("thread %d: no entry (attempt %d)" % (k, attempt))
            yield from self.goto(self.ee() + np.array([0, 0, 0.008]), tol=0.002)
        if not entered:
            return "no_entry"
        goal = PLATE_TOP + n * LEVEL + 0.0006
        fxy0 = f0[:2]
        down = False
        for tries in range(3):
            stall = 0
            for _ in range(60):
                z0 = self.ee()[2]
                w = self.wrench()
                lat = np.clip(-0.0004 * (w[:2] - fxy0), -0.001, 0.001)
                tgt = max(goal, rb() - 0.003)
                yield Cmd(rel=[lat[0], lat[1], tgt - rb()], yaw=self.yaw, grip=self.grip)
                if rb() < goal + 0.0015:
                    yield Cmd(rel=[0, 0, goal - rb()], yaw=self.yaw, grip=self.grip)
                    if rb() < goal + 0.0015:
                        down = True
                        break
                if z0 - self.ee()[2] < 0.0003:
                    stall += 1
                    if stall >= 4:
                        break
                else:
                    stall = 0
            if down:
                break
            # jammed on the way down (the ring cocked in the tight bore): back off a little and let it straighten
            self.log("thread %d: jam %.1f mm above the stack, backing off" % (k, 1000 * (rb() - goal)))
            for _ in range(3):
                w = self.wrench()
                lat = np.clip(-0.0006 * (w[:2] - fxy0), -0.0015, 0.0015)
                yield Cmd(rel=[lat[0], lat[1], 0.0015], yaw=self.yaw, grip=self.grip)
        if not down:
            self.log("thread %d: ring stuck %.1f mm above the stack; taking it back off" % (k, 1000 * (rb() - goal)))
            while rb() < TIP_Z + 0.008:
                w = self.wrench()
                lat = np.clip(-0.0004 * (w[:2] - fxy0), -0.001, 0.001)
                yield Cmd(rel=[lat[0], lat[1], 0.006], yaw=self.yaw, grip=self.grip)
            yield from self.goto([self.ee()[0], self.ee()[1], Z_SAFE], tol=0.004)
            return "no_entry"
        yield from self.hold(2)
        self.grip = -1.0
        yield from self.hold(4)
        # back up only to just above the tips: an empty hand kept this low never lets the pallet be bumped
        yield from self.goto([self.ee()[0], self.ee()[1], Z_LOW], tol=0.003)
        return "ok"

    def look(self, k, n=None, low=False):
        """Wrist view of spindle k from above (camera centred over it): refines its position, returns the top ring's
        colour class (None for a bare spindle or when nothing was found). low: with the TCP at Z_LOW."""
        n = len(self.belief[k]) if n is None else n
        z_tcp = Z_LOW if low else Z_SAFE
        if abs(self.ee()[2] - z_tcp) > 0.004:
            yield from self.goto([self.ee()[0], self.ee()[1], z_tcp], tol=0.003)
        xy, cls = None, None
        for it in range(2):
            guess = self.sp[k] if xy is None else xy
            yield from self.goto(self.tcp_for_cam_over(guess, z_cam=z_tcp + 0.1604), tol=0.001, settle=1)
            img, pc, Rc = yield from self.snap_wrist()
            xy, cls = self.detect_top(img, pc, Rc, guess, n)
            if xy is None or np.linalg.norm(xy - guess) < 0.004:
                break
        if xy is not None:
            delta = xy - np.asarray(self.sp[k], float)
            if self.seen[k] and np.linalg.norm(delta) > SHIFT_MM:
                # the fixture moved (the pallet was bumped): every spindle moved with it
                self.log("fixture shift %.1f mm seen at spindle %d" % (1000 * np.linalg.norm(delta), k))
                for j in range(NSP):
                    if j != k:
                        self.sp[j] = np.asarray(self.sp[j], float) + delta
                self.sp[k] = xy
            else:
                self.sp[k] = 0.4 * np.asarray(self.sp[k], float) + 0.6 * xy if self.seen[k] else xy
            self.seen[k] = True
        return xy, cls

    # ---------------------------------------------------------------- the episode
    def count_stacks(self, img):
        """How many rings stand on every spindle, from the overview image: the saturated bands above the plate."""
        sat = saturated(img)
        counts = []
        for k in range(NSP):
            x, y = self.sp[k]
            d = OV_POS[:2] - np.array([x, y])
            d = d / np.linalg.norm(d)
            front = np.array([x, y]) + d * (R_OUT * 0.85)
            n = 0
            for j in range(CAP):
                c = project([front[0], front[1], PLATE_TOP + 0.0002 + j * LEVEL + RING_H / 2], OV_POS, R_OV, F_OV,
                            OV_W, OV_H)
                u0, v0 = int(round(c[0] - 0.5)), int(round(c[1] - 0.5))
                win = sat[max(0, v0 - 1):v0 + 2, max(0, u0 - 3):u0 + 4]
                if win.size and win.mean() > 0.4:
                    n = j + 1
                else:
                    break
            counts.append(n)
        return counts

    def recount(self):
        self.dirty = True
        """Read the fixture afresh (how many rings on every spindle and their colours) from the overview camera."""
        yield from self.goto([self.ee()[0], self.ee()[1], max(self.ee()[2], Z_SAFE)], tol=0.004)
        yield from self.goto(PARK, tol=0.004)
        yield from self.hold(3, cams=1)
        img = decode(self.obs["overview"])
        counts = self.count_stacks(img)
        self.log("recount %s (believed %s)" % (counts, [len(b) for b in self.belief]))
        self.belief = self.assign(self.read_overview(img, counts), counts)

    def overview_read(self):
        """Park the hand out of the overview camera's view, take an overview image and read every stack."""
        self.dirty = True
        yield from self.goto([self.ee()[0], self.ee()[1], max(self.ee()[2], Z_SAFE)], tol=0.004)
        yield from self.goto(PARK, tol=0.004)
        yield from self.hold(3, cams=1)
        img = decode(self.obs["overview"])
        counts = [len(b) for b in self.belief]
        return self.assign(self.read_overview(img, counts), counts)

    def program(self):
        yield Cmd(rel=np.zeros(3), yaw=None, grip=-1.0, cams=0)
        self.yaw = 0.0
        self.grip = -1.0
        self.bias = np.zeros(2)
        self.sp = [np.array(p, float) for p in NOMINAL]
        self.seen = [False] * NSP
        self.confirmed = [False] * NSP            # a ring went onto it: its position is known by touch
        self.dirty = True
        # survey: where every spindle is and which one is empty (wrist camera from above)
        self.belief = [[[0, False]] * CAP for _ in range(NSP)]
        yield from self.goto([self.ee()[0], self.ee()[1], max(self.ee()[2], Z_SAFE)], tol=0.004)
        miss = []
        for k in range(NSP):
            self.last_bd = 1e9
            xy, _cls = yield from self.look(k, CAP)
            miss.append(1e9 if xy is None else self.last_bd)
        # exactly one spindle starts empty: the one where no ring's disc was found where a full stack's top would be
        empty = int(np.argmax(miss))
        self.belief[empty] = []
        self.seen[empty] = False
        self.sp[empty] = np.array(NOMINAL[empty], float) + np.mean(
            [self.sp[k] - np.array(NOMINAL[k]) for k in range(NSP) if k != empty], axis=0)
        yield from self.look(empty, 0)
        # the stacks' colours (overview camera)
        self.belief = yield from self.overview_read()
        self.log("belief %s" % self.belief)
        moves, checks = 0, 0
        fails = {}
        last_move = None
        while True:
            mv = plan(self.belief, weight=2.5, max_expand=25000)
            if mv and last_move is not None and mv[0] == (last_move[1], last_move[0]):
                alt = plan(self.belief, weight=2.5, max_expand=25000, forbid=(last_move[1], last_move[0]))
                if alt:
                    mv = alt
            if mv is None:
                mv = plan(self.belief, weight=6.0, max_expand=25000)
            if mv is None:
                self.log("no plan for %s" % self.belief)
            if not mv:
                if checks >= 3:
                    break
                checks += 1
                seen = yield from self.overview_read()
                cols = [[r[0] // 2 for r in s] for s in seen]
                if cols == [[r[0] // 2 for r in s] for s in self.belief]:
                    break
                self.log("final check: read colours %s, believed %s" % (cols, self.belief))
                for k in range(NSP):
                    if [r[0] // 2 for r in seen[k]] != [r[0] // 2 for r in self.belief[k]]:
                        self.belief[k] = [[2 * cc, False] for cc in cols[k]]
                self.belief = guess(self.belief)
                continue
            a, b = mv[0]
            if self.dirty:
                # the hand was up high with open jaws (survey, overview, a failure): the pallet may have moved
                yield from self.look(a, low=True)
                self.dirty = False
            ok = yield from self.pick(a)
            if not ok:
                self.dirty = True
                fails[a] = fails.get(a, 0) + 1
                if fails[a] >= 2:
                    # the belief is wrong here (a ring went missing): read the fixture again
                    fails[a] = 0
                    yield from self.recount()
                yield from self.look(a)
                continue
            ring = self.belief[a].pop()
            ring = [2 * (ring[0] // 2) + (1 if self.last_heavy else 0), True, self.last_margin]
            self.belief[a].append(ring)
            self.belief = guess(self.belief)
            ring = self.belief[a].pop()
            r = "no_entry"
            for attempt in range(2):
                if attempt > 0 or not self.confirmed[b]:
                    yield from self.look(b)
                r = yield from self.thread(b)
                if r == "ok":
                    break
            if r != "ok":
                self.dirty = True
                yield from self.look(a)
                r2 = yield from self.thread(a)
                if r2 == "ok":
                    self.belief[a].append(ring)
                else:
                    self.log("could not place the ring; leaving it on the table")
                    yield from self.goto([-0.05, 0.30, Z_SAFE], tol=0.004)
                    self.grip = -1.0
                    yield from self.hold(4)
                continue
            self.belief[b].append(ring)
            last_move = (a, b)
            moves += 1
            if moves > 120:
                break
        # done: park the hand away from the fixture
        self.grip = -1.0
        while True:
            yield from self.goto(PARK, tol=0.003)
            yield from self.hold(10)


# ------------------------------------------------------------------------------------------ stdio
def main():
    ctl = Controller(log=lambda m: sys.stderr.write(m + "\n"))
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        typ = msg.get("type")
        if typ == "reset":
            ctl.reset()
            out.write('{"ok": true}\n')
        elif typ == "act":
            a = ctl.act(int(msg["t"]), msg["obs"])
            out.write(json.dumps({"action": a}) + "\n")
        elif typ == "close":
            out.write('{"ok": true}\n')
            out.flush()
            break
        out.flush()


if __name__ == "__main__":
    main()
