"""The reference controller: only the observation and the data sheet. Wrist camera watches the belt (detect,
classify, pose, speed), tracks and grasps a needed part, the turnover station rights an upside-down part, the
inspection camera measures the part in the hand, the kit is located from its fiducials, and the part is inserted
by a force-guided peck search with a height check; a failed insertion is retried, a missed pick re-tried.

Knobs (cfg): force (peck search, default on), verify (retry when the height check fails), comp (latency
compensation), inspect (in-hand offset from the inspection camera), learn (per-shift insertion bias learning).
"""
import math

import numpy as np
import cv2

import percept as P
import skills as S
from ctl import Intent, Runner, wrap, HOME_YAW

# ---- data sheet --------------------------------------------------------------------------------------------------
TABLE_Z = 0.80
BELT_X, BELT_Z = 0.10, 0.84
PICK_ZONE = (0.40, -0.40)
KIT_NOMINAL = (-0.155, -0.27, math.pi / 2)
KIT_FLOOR = TABLE_Z + 0.010
POCKETS = (("A1", -0.075, 0.040, 0.0), ("B2", 0.000, 0.040, 0.0), ("C1", 0.075, 0.040, math.pi / 2),
           ("A2", -0.075, -0.040, math.pi), ("B1", 0.000, -0.040, 0.0), ("C2", 0.075, -0.040, math.pi))
FIDUCIALS = np.array([(-0.110, 0.068), (0.110, 0.068), (-0.110, -0.068)])
STATION = (-0.12, 0.28)
NEST_OFF = 0.046
NEST_FLOOR = TABLE_Z + 0.010
H = 0.020
CAM_COST = 0.05

WATCH_Y = 0.27
WATCH = np.array([BELT_X - 0.09, WATCH_Y, BELT_Z + 0.10])
WATCH_DOWN = np.array([BELT_X - 0.09, 0.05, BELT_Z + 0.10])    # first look after a cycle (parts that came meanwhile)
INSPECT_AT = np.array([-0.10, 0.04 - 0.0, 0.0])           # the hand holds the part over the inspection camera
INSPECT_GAP = 0.10                                        # part bottom above the inspection camera


def pocket_world(kit, k):
    kx, ky, kyaw = kit
    _v, px, py, pyaw = POCKETS[k]
    c, s = math.cos(kyaw), math.sin(kyaw)
    return kx + c * px - s * py, ky + s * px + c * py, wrap(kyaw + pyaw)


class Track(object):
    def __init__(self, det, t):
        self.obs = [(t, det["x"], det["y"], det["yaw"])]
        self.votes = {}
        self.add_vote(det)
        self.picked = False
        self.failed = 0
        self.v = None

    def add_vote(self, det):
        key = (det["v"], det["flipped"])
        self.votes[key] = self.votes.get(key, 0.0) + det["iou"]

    @property
    def label(self):
        return max(self.votes.items(), key=lambda kv: kv[1])[0]

    def update(self, det, t):
        self.obs.append((t, det["x"], det["y"], det["yaw"]))
        self.obs = self.obs[-12:]
        self.add_vote(det)

    def predict(self, t, v):
        """Position at time t: from a line fitted to the recent sightings when there are enough, else from the last
        sighting and the belt speed v."""
        t0, x0, y0, yaw0 = self.obs[-1]
        ob = [o for o in self.obs if o[0] >= t0 - 3.0]
        if len(ob) >= 4 and ob[-1][0] - ob[0][0] >= 0.8:
            ts = np.array([o[0] for o in ob])
            ys = np.array([o[2] for o in ob])
            xs = np.array([o[1] for o in ob])
            b, a = np.polyfit(ts - t0, ys, 1)
            if 0.01 < -b < 0.1:
                return float(xs.mean()), float(a + b * (t - t0)), yaw0
        return x0, y0 - v * (t - t0), yaw0

    def slope(self, span=3.0):
        """The belt speed from this part's own recent sightings (least squares), or None."""
        ob = [o for o in self.obs if o[0] >= self.obs[-1][0] - span]
        if len(ob) < 4 or ob[-1][0] - ob[0][0] < 0.8:
            return None
        ts = np.array([o[0] for o in ob])
        ys = np.array([o[2] for o in ob])
        b = np.polyfit(ts, ys, 1)[0]
        return float(-b)


class World(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.obs = None
        self.R = None
        self.hold = None
        self.log = []
        self.tracks = []
        self.speed = None
        self.speed_samples = []
        self.kit = None
        self.filled = set()
        self.bias = np.zeros(2)
        self.nbias = 0
        self.found = None
        self.inhand_h = S.GRIP_H
        self.came_back = False

    def hold_goal(self):
        return self.hold if self.hold is not None else np.asarray(self.obs["ee_pos"], float)

    # ---- belt perception
    def belt_detections(self, img):
        cam = P.wrist_camera(self.obs["ee_pos"], self.obs["ee_yaw"])
        out = []
        h, w = img.shape[:2]
        for fam, m, st in P.blobs(img):
            if st[0] <= 6 or st[1] <= 6 or st[0] + st[2] >= w - 6 or st[1] + st[3] >= 0.70 * h:
                continue
            if m.sum() < 25:
                continue
            r = P.fit_part(cam, m, fam, BELT_Z + H)
            if r["iou"] < 0.72:
                continue
            if abs(r["x"] - BELT_X) > 0.07:
                continue
            out.append(r)
        return out

    def update_tracks(self, dets, t):
        v = self.speed if self.speed is not None else 0.045
        for d in dets:
            best = None
            for tr in self.tracks:
                if tr.picked:
                    continue
                x, y, _ = tr.predict(t, v)
                dist = math.hypot(d["x"] - x, d["y"] - y)
                if dist < 0.025 and (best is None or dist < best[0]):
                    best = (dist, tr)
            if best is None:
                self.tracks.append(Track(d, t))
            else:
                tr = best[1]
                tr.update(d, t)
                s = tr.slope()
                if s is not None and 0.01 < s < 0.1:
                    self.speed_samples.append((t, s))
                    self.speed_samples = [x for x in self.speed_samples if x[0] > t - 4.0]
                    self.speed = float(np.median([x[1] for x in self.speed_samples]))
                    tr.v = s
        # forget tracks that left the pick zone
        v = self.speed if self.speed is not None else 0.045
        keep = []
        for tr in self.tracks:
            if tr.predict(t, v)[1] > PICK_ZONE[1] - 0.05 and not tr.picked:
                keep.append(tr)
            elif not tr.picked:
                self.log.append(("track-gone", tr.label, len(tr.obs), round(t, 1)))
                if getattr(self, "dbg_gone", None) is not None:
                    self.dbg_gone(self, tr)
        self.tracks = keep


def look(w, cams):
    """One step that takes images; returns the observation carrying them."""
    obs = yield Intent(goal=w.hold_goal(), cams=cams, grip=None)
    return w.obs


def locate_kit(w):
    """Kit pose from its three fiducials (dark dots on the light plate), from two close looks."""
    kx, ky, kyaw = KIT_NOMINAL
    pts = []
    looks_by = {}
    # the tray is set within +-12 mm and +-5 deg: aim each look where the fiducials found so far put the next one
    # (one found: shift by it; two or more: a rigid fit), so far dots stay in view and the gate can stay tight
    found = {}

    def nominal(fi):
        fx0, fy0 = FIDUCIALS[fi]
        return np.array([kx + math.cos(kyaw) * fx0 - math.sin(kyaw) * fy0, ky + math.sin(kyaw) * fx0 + math.cos(kyaw) * fy0])

    def predict(fi):
        keys = sorted(found)
        if not keys:
            return nominal(fi), 0.030
        if len(keys) == 1:
            k = keys[0]
            return nominal(fi) + (np.mean(found[k], axis=0) - nominal(k)), 0.030
        src = np.array([nominal(k) for k in keys])
        dst = np.array([np.mean(found[k], axis=0) for k in keys])
        sc, dc = src.mean(0), dst.mean(0)
        U, _s, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
        Rm = Vt.T @ U.T
        if np.linalg.det(Rm) < 0:
            Vt[1] *= -1
            Rm = Vt.T @ U.T
        return Rm @ (nominal(fi) - sc) + dc, 0.008

    for LY, order in ((0.0, (0, 1, 2)), (math.pi, (2, 1, 0))):   # all looks at one yaw, then all at the other
      for fi in order:
        fx0, fy0 = FIDUCIALS[fi]
        pred, gate = predict(fi)
        fx, fy = pred[0] - kx, pred[1] - ky
        looks = looks_by.setdefault(fi, [])
        if True:
            # put the wrist camera (offset from the grip axis) above the nominal fiducial; a second look with the hand
            # turned half a turn sees the same dot from the other side, so the camera's mounting error cancels
            z = KIT_FLOOR + 0.045
            Rs = P.site_R(LY)
            cam_off = (Rs @ P.WRIST_POS)[:2]
            goal = np.array([kx + fx - cam_off[0], ky + fy - cam_off[1], z])
            yield from S.move_to(w, goal, yaw=LY, grip=-0.2, tol=0.002, yaw_tol=0.02, max_steps=60, settle=1)
            yield Intent(goal=goal, yaw=LY, grip=-0.2)
            yield Intent(goal=goal, yaw=LY, grip=-0.2, cams=2)
            obs = w.obs
            img = obs["wrist"]
            cam = P.wrist_camera(obs["ee_pos"], obs["ee_yaw"])
            val = img.max(axis=2).astype(float)
            dark = (val < 0.7 * cv2.blur(val, (21, 21))).astype(np.uint8)
            n, lab, stats, cents = cv2.connectedComponentsWithStats(dark, 8)
            cand = []
            for k in range(1, n):
                a_ = stats[k, cv2.CC_STAT_AREA]
                bw, bh = stats[k, cv2.CC_STAT_WIDTH], stats[k, cv2.CC_STAT_HEIGHT]
                if a_ < 8 or max(bw, bh) > 1.8 * min(bw, bh) or a_ < 0.5 * bw * bh:
                    continue
                vs, us = np.nonzero(lab == k)
                Pw = cam.to_plane(us, vs, KIT_FLOOR)
                ext = max(np.ptp(Pw[:, 0]), np.ptp(Pw[:, 1]))
                if not (0.008 < ext < 0.017):          # a 12 mm dot, not a pocket corner, a boss or a wall shadow
                    continue
                c = (Pw[:, 0].mean(), Pw[:, 1].mean())
                d = math.hypot(c[0] - kx - fx, c[1] - ky - fy)
                if d < gate:
                    cand.append((d, c))
            w.log.append(("fid-look", len(cand), round(float(np.percentile(val, 75)), 1)))
            if cand:
                c = min(cand)[1]
                looks.append(c)
                found.setdefault(fi, []).append(np.array(c))
            w.fid_img = img
    for fi in range(len(FIDUCIALS)):
        looks = looks_by.get(fi, [])
        pts.append(tuple(np.mean(looks, axis=0)) if looks else None)
    if any(p is None for p in pts):
        w.log.append(("kit", "fallback-nominal"))
        w.kit = KIT_NOMINAL
        return
    matched = pts
    A = np.array(matched)
    # rigid 2D fit (Kabsch) of the drawing's fiducials onto the measured ones
    src = FIDUCIALS
    sc, dc = src.mean(0), A.mean(0)
    Hm = (src - sc).T @ (A - dc)
    U, _s, Vt = np.linalg.svd(Hm)
    Rm = Vt.T @ U.T
    if np.linalg.det(Rm) < 0:
        Vt[1] *= -1
        Rm = Vt.T @ U.T
    yaw = math.atan2(Rm[1, 0], Rm[0, 0])
    t = dc - Rm @ sc
    w.kit = (float(t[0]), float(t[1]), yaw)
    w.log.append(("kit", [round(t[0], 4), round(t[1], 4), round(math.degrees(yaw), 2)]))


def overview_scan(w):
    """Coarse positions (y on the belt) of the coloured parts the fixed overview camera sees on the belt now. Its mount
    drifts by a centimetre or two from the drawing, which is plenty to know where to look with the wrist camera."""
    yield Intent(goal=w.hold_goal(), grip=-0.2, cams=1)
    img = w.obs["overview"]
    out = []
    if img.size == 0:
        return out
    pos, tgt = np.array(P.OVERVIEW_POS), np.array(P.OVERVIEW_TGT)
    cam = P.Camera(pos, P.look_at(pos, tgt), P.OVERVIEW_FOVY, P.OVERVIEW_WH)
    for fam, m in P.colour_masks(img).items():
        n, lab, stats, cents = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < 3:
                continue
            u, v = cents[k]
            Pw = cam.to_plane(np.array([u - 0.5]), np.array([v - 0.5]), BELT_Z + H)[0]
            if abs(Pw[0] - BELT_X) < 0.12 and PICK_ZONE[1] < Pw[1] < 0.42:
                out.append((float(Pw[1]), fam))
    return out


def watch(w):
    """Watch the belt with the wrist camera (an image every step) and return a track to pick: a needed variant that
    can still be caught. After coming back from a cycle, first ask the overview camera where parts are and look at
    the most downstream one that could still be caught (parts that came while the robot was busy)."""
    looks = [WATCH]
    if w.came_back:
        w.came_back = False
        cands = yield from overview_scan(w)
        v = w.speed if w.speed is not None else 0.045
        ys = [y for y, fam in cands if y - v * (6.0 + 1.5 * S.lx(w)) > PICK_ZONE[1] + 0.03 + v and y < WATCH_Y + 0.05]
        if ys:
            looks = [np.array([BELT_X - 0.09, max(PICK_ZONE[1] + 0.12, min(ys) - v * 1.5), BELT_Z + 0.10]), WATCH]
    for li, pose in enumerate(looks):
        yield from S.move_to(w, pose, yaw=HOME_YAW, grip=-0.2, tol=0.004, yaw_tol=0.05, max_steps=60)
        w.hold = pose.copy()
        n = 0
        while True:
            yield Intent(goal=pose, yaw=HOME_YAW, grip=-0.2, cams=2)
            n += 1
            obs = w.obs
            if obs["wrist"].size == 0:
                continue
            t = obs["time"]
            w.update_tracks(w.belt_detections(obs["wrist"]), t)
            best = None
            for tr in w.tracks:
                if tr.obs[-1][0] < t - 0.3:
                    continue
                if tr.slope() is None and (len(tr.obs) < 2 or w.speed is None):
                    continue
                (name, flipped) = tr.label
                if name in w.filled:
                    continue
                v = tr.v if tr.v is not None else w.speed
                x, y, yaw = tr.predict(t, v)
                if y - v * (6.0 + 1.5 * S.lx(w)) < PICK_ZONE[1] + 0.03 + v or y > WATCH_Y + 0.02:
                    continue
                if best is None or y < best[1]:
                    best = (tr, y)
            if best is not None:
                return best[0]
            if li < len(looks) - 1 and n >= 8:
                break


def refresh(w, tr):
    """A fresh look at the chosen part just ahead of where it will be picked (short extrapolation)."""
    got = False
    for _ in range(4):
        if got and tr.slope() is not None:
            return True
        x, y, _ = tr.predict(w.obs["time"] + 1.0, w.speed)
        goal = np.array([BELT_X - 0.09, max(PICK_ZONE[1] + 0.02, min(WATCH_Y, y)), BELT_Z + 0.10])
        yield from S.move_to(w, goal, yaw=HOME_YAW, grip=-0.2, tol=0.006, yaw_tol=0.05, max_steps=25)
        yield Intent(goal=goal, yaw=HOME_YAW, grip=-0.2, cams=2)
        obs = w.obs
        if obs["wrist"].size == 0:
            continue
        n0 = len(tr.obs)
        w.update_tracks(w.belt_detections(obs["wrist"]), obs["time"])
        if len(tr.obs) > n0:
            got = True
    return got


def pick_track(w, tr, psi, width):
    """Track the part on the belt from its estimate and grasp it."""
    grasp_z = BELT_Z + S.GRIP_H
    g_open = S.g_open(width)
    for phase, z_off, steps, ms in (("approach", 0.045, 50, 0.03), ("descend", 0.0, 40, 0.03)):
        for _ in range(steps):
            obs = w.obs
            v = tr.v if tr.v is not None else w.speed
            x, y, _ = tr.predict(obs["time"], v)
            dt = 0.1
            goal = np.array([x, y, grasp_z + z_off])
            vel = np.array([0.0, -v * dt, 0.0])
            ee = np.asarray(obs["ee_pos"], float)
            if y < PICK_ZONE[1] + 0.02 + v * (1.3 + 0.1 * S.lx(w)):   # it would leave the zone while closing
                return False
            tol = 0.002 + 0.001 * S.lx(w)          # a longer delay settles slower: start down a little earlier
            if np.linalg.norm(ee[:2] - goal[:2]) < tol and abs(ee[2] - goal[2]) < tol and \
                    abs(w.R._yaw(obs) - psi) < 0.03:
                break
            if phase == "descend":
                if abs(w.R._yaw(obs) - psi) > 0.05 or np.linalg.norm(ee[:2] - goal[:2] - vel[:2]) > 0.009:
                    goal[2] = grasp_z + 0.045     # not lined up yet: the fingers would land on the part
                else:
                    goal[2] = max(grasp_z, ee[2] - 0.010)     # down at about 6 mm per step (the belt keeps the
                                                               # full step for tracking)
            yield Intent(goal=goal, vel=vel, yaw=psi, grip=g_open, max_step=ms)
    if w.obs["ee_pos"][2] > grasp_z + 0.005:
        obs = w.obs
        v = tr.v if tr.v is not None else w.speed
        x, y, _ = tr.predict(obs["time"], v)
        w.log.append(("pick-abort", round(w.obs["time"], 1), "yaw err", round(w.R._yaw(obs) - psi, 3),
                      "xy err mm", [round(1000 * (a - b), 1) for a, b in zip(obs["ee_pos"][:2], (x, y - v * 0.1))],
                      "z above", round(1000 * (obs["ee_pos"][2] - grasp_z), 1)))
        ee = np.asarray(w.obs["ee_pos"], float)
        yield from S.move_to(w, [ee[0], ee[1], S.SAFE_Z], grip=-0.2, tol=0.01, max_steps=20)
        return False
    zs = []
    e = S.lx(w)
    prev_g = 1.0
    for k in range(10 + e):
        obs = w.obs
        v = tr.v if tr.v is not None else w.speed
        x, y, _ = tr.predict(obs["time"], v)
        z = grasp_z if k < 6 + e else grasp_z + 0.004 * (k - 5 - e)
        if 3 + e <= k <= 7 + e:
            zs.append(obs["ee_pos"][2])
            if getattr(w, "dbg_close", None) is not None:
                try:
                    w.dbg_close(w, tr)
                except Exception:  # noqa: BLE001  (dev harness only)
                    pass
        if k >= 4 + e and len(zs) >= 2 and obs["gripper"] < 0.075 and abs(obs["gripper"] - prev_g) < 0.0005:
            break                                   # the fingers have closed on it: lift now, the belt keeps moving
        prev_g = obs["gripper"]
        yield Intent(goal=np.array([x, y, z]), vel=np.array([0.0, -v * 0.1, 0.0]), yaw=psi, grip=1.0,
                     max_step=0.012)
    w.inhand_h = float(min(zs) - BELT_Z) if zs else S.GRIP_H
    ee = np.asarray(w.obs["ee_pos"], float)
    yield from S.move_to(w, [ee[0], ee[1], S.SAFE_Z], grip=1.0, tol=0.01, max_steps=30)
    if getattr(w, "dbg", None) is not None:
        try:
            w.dbg_pick(w, tr)
        except Exception:  # noqa: BLE001  (dev harness only)
            pass
    return w.obs["gripper"] > width - 0.005


def inspect_once(w, name, yaw, delta, flipped_view=False):
    """One look at the held part from below with the hand at `yaw`: the part's centre (world) and yaw, or None."""
    z = P.INSPECT_POS[2] + INSPECT_GAP + w.inhand_h
    goal = np.array([P.INSPECT_POS[0], P.INSPECT_POS[1], z])
    yield from S.move_to(w, goal, yaw=yaw, grip=1.0, tol=0.0015, yaw_tol=0.02, max_steps=60, settle=1)
    for _ in range(2):
        yield Intent(goal=goal, yaw=yaw, grip=1.0)
    yield Intent(goal=goal, yaw=yaw, grip=1.0, cams=4)
    obs = w.obs
    img = obs["inspect"]
    if img.size == 0:
        return None
    cam = P.inspect_camera()
    fam = P.VARIANTS[name][4]
    m = P.colour_masks(img)[fam]
    n, lab, stats, _c = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mk = lab == k
    z_bottom = obs["ee_pos"][2] - w.inhand_h
    # the outline is fitted in world coordinates on the bottom-face plane (the camera model handles the view)
    best = None
    ee = np.asarray(obs["ee_pos"], float)
    psi = w.R._yaw(obs)
    base = psi - math.pi / 2 + delta     # the hand's forward direction plus the grasp's relative yaw (0 or pi)
    for b in (base, base + math.pi):
        r = fit_known(cam, mk, name, flipped_view, z_bottom, b, ee[:2], fit_yaw=True)
        if r is not None:
            r["iou_adj"] = r["iou"] + (0.03 if b == base else 0.0)   # prefer the grasp's own relative yaw
            if best is None or r["iou_adj"] > best["iou_adj"]:
                best = r
    if best is None or best["iou"] < 0.7:
        return None
    return {"c": np.array([best["x"], best["y"]]), "ee": ee[:2].copy(), "psi": psi, "yaw": best["yaw"],
            "iou": best["iou"]}


def measure_inhand(w, name, flipped_view=False):
    """The held part's centre relative to the grip site (world, at the hand's current yaw) and its yaw, from two looks
    with the hand half a turn apart: the part's offset turns with the hand, the camera's own mounting error does not,
    so the difference of the two looks measures the offset free of it."""
    psi1 = w.R._yaw(w.obs)
    m1 = yield from inspect_once(w, name, psi1, w.delta, flipped_view)
    if m1 is None:
        return None
    win = S.yaw_window((P.INSPECT_POS[0], P.INSPECT_POS[1]))
    psi2 = psi1 + math.pi if psi1 + math.pi <= win[1] else psi1 - math.pi
    rel1 = wrap(m1["yaw"] - (m1["psi"] - math.pi / 2))          # the part's yaw relative to the hand
    m2 = yield from inspect_once(w, name, psi2, rel1)
    if m2 is None:
        return {"off": m1["c"] - m1["ee"], "yaw": m1["yaw"], "iou": m1["iou"]}
    # o(psi1) = ((c1 - ee1) - (c2 - ee2)) / 2 ; now the hand is at psi2: o(psi2) = -o(psi1)
    o1 = 0.5 * ((m1["c"] - m1["ee"]) - (m2["c"] - m2["ee"]))
    rel2 = wrap(m2["yaw"] - (m2["psi"] - math.pi / 2))
    rel = rel1 + 0.5 * wrap(rel2 - rel1)
    prior = w.delta                       # the pads square the part up: its yaw relative to the hand is 0 or pi
    if abs(wrap(rel - prior)) < math.radians(2.5):
        rel = prior
    elif abs(wrap(rel - prior - math.pi)) < math.radians(2.5):
        rel = wrap(prior + math.pi)
    return {"off": -o1, "yaw": wrap(m2["psi"] - math.pi / 2 + rel), "iou": min(m1["iou"], m2["iou"])}


def fit_known(cam, mask, name, flipped, z, yaw0, xy0, fit_yaw=True):
    """Fine fit of a known variant near (xy0, yaw0) by IoU (coordinate descent)."""
    vs, us = np.nonzero(mask)
    if len(vs) < 20:
        return None
    y0_, y1_ = max(0, vs.min() - 4), min(mask.shape[0], vs.max() + 5)
    x0_, x1_ = max(0, us.min() - 4), min(mask.shape[1], us.max() + 5)
    sub = mask[y0_:y1_, x0_:x1_]
    Pm = cam.to_plane(us, vs, z)
    poly = np.array(P.outline(name))
    if flipped:
        poly = poly * np.array([1.0, -1.0])
    mm = cv2.moments(poly.astype(np.float32) * 1000.0)
    ox, oy = mm["m10"] / mm["m00"] / 1000.0, mm["m01"] / mm["m00"] / 1000.0
    c, s = math.cos(yaw0), math.sin(yaw0)
    cur = [Pm[:, 0].mean() - (c * ox - s * oy), Pm[:, 1].mean() - (s * ox + c * oy), yaw0]

    def iou(p):
        R = P.render_mask(cam, P.polygon_world(name, p[0], p[1], p[2], flipped, z), mask.shape)[y0_:y1_, x0_:x1_]
        return np.logical_and(R, sub).sum() / max(1, np.logical_or(R, sub).sum())
    best = iou(cur)
    if fit_yaw:
        # coarse yaw scan (the part can sit a few degrees turned in the grasp), re-centring the outline each time
        for dy in np.radians(np.arange(-8.0, 8.01, 0.5)):
            yy = yaw0 + dy
            c2, s2 = math.cos(yy), math.sin(yy)
            trial = [Pm[:, 0].mean() - (c2 * ox - s2 * oy), Pm[:, 1].mean() - (s2 * ox + c2 * oy), yy]
            v = iou(trial)
            if v > best:
                best, cur = v, trial
    step = [0.0008, 0.0008, math.radians(0.5)]
    for _ in range(5):
        improved = True
        while improved:
            improved = False
            for k in range(3 if fit_yaw else 2):
                for sg in (-1, 1):
                    trial = list(cur)
                    trial[k] += sg * step[k]
                    v = iou(trial)
                    if v > best + 1e-9:
                        best, cur, improved = v, trial, True
        step = [x * 0.5 for x in step]
    return {"x": cur[0], "y": cur[1], "yaw": wrap(cur[2]), "iou": best}


def nest_part(w, name):
    """Look at the output nest; fit the part lying there (upright after the turnover)."""
    sx, sy = STATION
    ly = w.R._yaw(w.obs)                      # look with the hand's current yaw: put the camera above the nest
    cam_off = (P.site_R(ly) @ P.WRIST_POS)[:2]
    goal = np.array([sx - cam_off[0], sy - NEST_OFF - cam_off[1], NEST_FLOOR + 0.07])
    yield from S.move_to(w, goal, yaw=ly, grip=-0.2, tol=0.005, yaw_tol=0.05, max_steps=50)
    yield Intent(goal=goal, yaw=ly, grip=-0.2, cams=2)
    obs = w.obs
    cam = P.wrist_camera(obs["ee_pos"], obs["ee_yaw"])
    fam = P.VARIANTS[name][4]
    best = None
    for f, m, st in P.blobs(obs["wrist"]):
        if f != fam:
            continue
        r = P.fit_part(cam, m, fam, NEST_FLOOR + H)
        if best is None or r["iou"] > best["iou"]:
            best = r
    return best


def reference_script(w):
    def script(R):
        w.R = R
        obs = yield Intent(grip=-1.0)
        w.obs = obs
        gen = main(w)
        intent = next(gen)
        while True:
            obs = yield intent
            w.obs = obs
            try:
                intent = gen.send(obs)
            except StopIteration:
                intent = Intent(goal=w.hold_goal())
    return script


def identify(w):
    """The shift's own arm: command latency and per-step response differ from shift to shift (data sheet), so
    measure them first. A short excitation with known raw commands (about 4 s, before the first part arrives), then a
    least-squares fit of the latency and the response in the Runner."""
    seq = [(0, 0, 0)] * 4 + [(0.012, 0, 0)] * 5 + [(0, 0, 0)] * 3 + [(-0.012, 0, 0)] * 5 + [(0, 0, 0)] * 3 + \
          [(0, 0.012, 0.008)] * 4 + [(0, 0, 0)] * 3 + [(0, -0.012, -0.008)] * 4 + [(0, 0, 0)] * 8
    t0 = int(w.obs["t"])
    for u in seq:
        uu = np.array(u, float)
        w.R.integ[:] = 0.0
        yield Intent(fn=lambda obs, R, uu=uu: uu, grip=-0.2)
    fit = w.R.identify(t0, int(w.obs["t"]))
    # the drift under a zero command (gravity compensation error): with the target re-anchored at every command,
    # a steady load moves the hand at a steady rate; hold zero commands and measure it
    ees = []
    for _ in range(8 + 2 * S.lx(w)):
        w.R.integ[:] = 0.0
        yield Intent(fn=lambda obs, R: R.drift / R.arm_b, grip=-0.2)   # cancels the runner's compensation: raw 0
        ees.append(np.asarray(w.obs["ee_pos"], float))
    d = np.diff(np.array(ees[-6:]), axis=0).mean(axis=0)
    w.R.drift = np.clip(d * (1.0 - w.R.arm_a), -0.001, 0.001)
    w.log.append(("identify", None if fit is None else [round(fit[0] * 1000, 3), fit[1], round(fit[2], 3),
                                                        round(fit[3], 3)], "drift mm", [round(1000 * x, 3) for x in w.R.drift]))


def main(w):
    yield from S.move_to(w, [-0.15, -0.1, S.SAFE_Z], yaw=HOME_YAW, grip=-0.2, tol=0.01, max_steps=40)
    yield from identify(w)
    yield from S.move_to(w, [-0.15, -0.1, S.SAFE_Z], yaw=HOME_YAW, grip=-0.2, tol=0.005, max_steps=40)
    yield from locate_kit(w)
    while True:
        tr = yield from watch(w)
        name, flipped = tr.label
        L, W = P.VARIANTS[name][0], P.VARIANTS[name][1]
        k = [p[0] for p in POCKETS].index(name)
        px, py, pyaw = pocket_world(w.kit, k)
        w.pocket_xy = (px, py)
        x, y, yaw = tr.predict(w.obs["time"], w.speed)
        goal_theta = pyaw if not flipped else S.nest_yaw(pyaw)
        place_xy = (px, py) if not flipped else (STATION[0], STATION[1] + NEST_OFF)
        psi_pick, psi_place, delta = S.grasp_plan(yaw, goal_theta, (x, y), place_xy)
        w.delta = delta
        fresh = yield from refresh(w, tr)
        if not fresh:
            w.log.append(("refresh-miss", name, round(w.obs["time"], 1)))
        x, y, yaw = tr.predict(w.obs["time"], w.speed)
        ok = yield from pick_track(w, tr, psi_pick, W)
        w.log.append(("pick", name, flipped, bool(ok), round(w.obs["time"], 1)))
        if not ok:
            # a missed grasp leaves the part on the belt: keep its track (it can be retried while still in the zone)
            tr.failed += 1
            if tr.failed >= 2 or w.obs["gripper"] > 0.002:
                tr.picked = True          # something went wrong in the hand (or twice): give it up
            if w.obs["gripper"] > 0.002:
                yield from dump(w)
            w.came_back = True
            continue
        tr.picked = True
        if flipped:
            ok = yield from turnover_ref(w, name, yaw, psi_pick, W, pyaw)
            if not ok:
                w.came_back = True
                continue
        # the part in the hand, measured
        w.log.append(("t-inspect", round(w.obs["time"], 1)))
        m = yield from measure_inhand(w, name)
        if getattr(w, "dbg", None) is not None and m is not None:
            w.dbg(w, "inhand", m)
        if m is None:
            w.log.append(("inspect-fail", name))
            yield from dump(w)
            w.came_back = True
            continue
        psi = w.R._yaw(w.obs)
        delta = wrap(m["yaw"] - (psi - math.pi / 2))
        psi_place = S.yaw_place(pyaw - delta + math.pi / 2, (px, py), psi)
        rot = psi_place - psi
        c_, s_ = math.cos(rot), math.sin(rot)
        off = m["off"]
        off_place = np.array([c_ * off[0] - s_ * off[1], s_ * off[0] + c_ * off[1]])
        aim = np.array([px, py]) - off_place + w.bias
        w.dbg_aim = (aim.copy(), off_place.copy(), (px, py, pyaw), psi, psi_place, off.copy())
        w.log.append(("t-transit", round(w.obs["time"], 1)))
        yield from S.transit(w, aim, yaw=psi_place, grip=1.0)
        w.log.append(("t-insert0", round(w.obs["time"], 1)))
        z_seat = KIT_FLOOR + w.inhand_h
        res = "stuck"
        tries = 2 if w.cfg.get("verify", True) else 1
        t_end = w.obs["time"] + 25.0                  # one stall must not cost the next parts
        for attempt in range(tries):
            w.found = None
            res = yield from S.insert_force(w, aim, psi_place, z_seat, t_end=t_end)
            if res == "seated" or not w.cfg.get("verify", True) or w.obs["time"] > t_end:
                break
            yield from S.move_to(w, [aim[0], aim[1], z_seat + 0.015], yaw=psi_place, grip=1.0, tol=0.002,
                                 max_steps=20)
        if res == "seated" and w.found is not None and w.cfg.get("learn", False):
            w.bias = (w.bias * w.nbias + (w.bias + w.found)) / (w.nbias + 1)
            w.nbias += 1
        w.log.append(("insert", name, res, attempt, round(w.obs["time"], 1), "psi_place", round(math.degrees(psi_place), 1),
                      None if w.found is None else [round(1000 * x, 1) for x in w.found], [round(1000 * x, 2) for x in w.bias],
                      [round(1000 * x, 2) for x in m["off"]], round(m["iou"], 2), round(w.inhand_h * 1000, 1)))
        if res != "seated" and w.cfg.get("verify", True):
            # not in: do not leave an unseated part in the pocket (it would block a spare); take it out
            ee = np.asarray(w.obs["ee_pos"], float)
            yield from S.move_to(w, [ee[0], ee[1], S.SAFE_Z], grip=1.0, tol=0.01, max_steps=30)
            yield from dump(w)
            w.came_back = True
            continue
        # let go where it is (a raw zero command holds the hand; the opening lags by the command latency), then lift:
        # lifting while the fingers still close would pull the part out of its pocket
        w.R.integ[:] = 0.0
        for _ in range(5 + S.lx(w)):
            yield Intent(fn=lambda obs, R: np.zeros(3), grip=-0.2)
        ee = np.asarray(w.obs["ee_pos"], float)
        yield from S.move_to(w, [ee[0], ee[1], ee[2] + 0.015], grip=-0.2, tol=0.003, max_steps=15, max_step=0.004)
        yield from S.move_to(w, [ee[0], ee[1], S.SAFE_Z], grip=-0.2, tol=0.01, max_steps=30)
        w.filled.add(name)
        w.came_back = True


def dump(w):
    """Put a part we cannot use on the table beside the kit (out of the way)."""
    yield from S.transit(w, (-0.30, 0.10), grip=1.0)
    yield from S.move_to(w, [-0.30, 0.10, TABLE_Z + 0.03], grip=1.0, tol=0.003, max_steps=40)
    for _ in range(4 + S.lx(w)):
        yield Intent(goal=w.hold_goal(), grip=-0.2)
    yield from S.move_to(w, [-0.30, 0.10, S.SAFE_Z], grip=-0.2, tol=0.01, max_steps=30)


def turnover_ref(w, name, yaw_belt, psi_pick, width, pyaw=0.0):
    """Place the held (upside-down) part in the input nest along x, cycle the station, pick it from the output nest."""
    sx, sy = STATION
    nin = np.array([sx, sy + NEST_OFF])
    psi = w.R._yaw(w.obs)
    # the part's yaw relative to the hand (from the belt estimate)
    delta = wrap(yaw_belt - (psi_pick - math.pi / 2))
    psi_place = S.yaw_place(S.nest_yaw(pyaw) - delta + math.pi / 2, nin, psi)
    yield from S.transit(w, nin, yaw=psi_place, grip=1.0)
    w.log.append(("t-nest", round(w.obs["time"], 1)))
    yield from S.place_release(w, nin, NEST_FLOOR + w.inhand_h + 0.001, psi_place)
    w.log.append(("t-turn", round(w.obs["time"], 1)))
    c0 = w.obs["station_cycles"]
    started = False
    for _ in range(8):
        yield Intent(goal=w.hold_goal(), grip=-0.2, turn=True)
        if w.obs["station_busy"]:
            started = True
            break
    for _ in range(60):
        if not w.obs["station_busy"] and w.obs["station_cycles"] > c0:
            break
        yield Intent(goal=w.hold_goal(), grip=-0.2)
    w.log.append(("turn", name, started, w.obs["station_cycles"] > c0, round(w.obs["time"], 1)))
    w.log.append(("t-look", round(w.obs["time"], 1)))
    det = yield from nest_part(w, name)
    w.log.append(("t-repick", round(w.obs["time"], 1)))
    if det is None:
        w.log.append(("nest-miss", name))
        return False
    psi2, _pp, w.delta = S.grasp_plan(det["yaw"], pyaw, (det["x"], det["y"]), w.pocket_xy)
    z = NEST_FLOOR + S.GRIP_H
    xy = np.array([det["x"], det["y"]])
    yield from S.move_to(w, [xy[0], xy[1], z + 0.03], yaw=psi2, grip=S.g_open(width), tol=0.0015, yaw_tol=0.03,
                         max_steps=60)
    yield from S.move_to(w, [xy[0], xy[1], z], yaw=psi2, grip=S.g_open(width), tol=0.001, max_steps=40,
                         max_step=0.008)
    zs = []
    for _ in range(6 + S.lx(w)):
        yield Intent(goal=np.array([xy[0], xy[1], z]), yaw=psi2, grip=1.0)
        zs.append(w.obs["ee_pos"][2])
    w.inhand_h = float(min(zs) - NEST_FLOOR)
    yield from S.move_to(w, [xy[0], xy[1], S.SAFE_Z], grip=1.0, tol=0.01, max_steps=40)
    ok = w.obs["gripper"] > width - 0.005
    w.log.append(("repick", name, bool(ok), round(w.obs["time"], 1)))
    return ok


class Reference(object):
    def __init__(self, cfg=None, latency=2):
        self.cfg = dict(cfg or {})
        self.latency = latency
        self.reset()

    def reset(self):
        self.w = World(self.cfg)
        self.runner = Runner(reference_script(self.w), latency=self.latency, comp=self.cfg.get("comp", True))

    def act(self, obs):
        return self.runner.act(obs)
