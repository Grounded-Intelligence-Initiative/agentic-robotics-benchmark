"""Reference controller for toy_car_assembly_from_manual.

It reads nothing hidden: the cell's public numbers (the instruction: the hand, the wrist camera bolted to it, the
stand's nominal place) and the episode's car, read off the picture manual that arrives with the observation at t = 0
(read_manual.py beside this file: the ruler gives the scale, the orthographic views the sizes). The overview camera is re-seated every episode and its pose is not known, so this
controller does not use it: it finds the parts by scanning the table with the wrist camera, whose mount and lens are
known, and localises each part, and the stand, precisely from a close look with the wrist camera before it acts.
Wire: one JSON object per line on stdin, one reply per line on stdout.
"""

import base64
import copy
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

# ------------------------------------------------------------------ the cell (instruction, "The cell")
TABLE_Z = 0.80
STAND_NOM = np.array([0.06, 0.0])               # the stand's nominal place; each episode moves it a little
STAND_TOP_Z = 0.90
C0 = np.array([1.0, 0.0, 0.0])                 # finger-closing axis at yaw 0, tilt 0
A0 = np.array([0.0, 0.0, -1.0])                # approach (pointing direction) at tilt 0
Q7_LIMIT = 2.75
PARK = np.array([-0.40, 0.0, 1.10])
SAFE_Z = TABLE_Z + 0.15
# the wrist camera, bolted to the hand: position in the hand frame, axes (image right = hand -x, image up = hand +y,
# looking along hand +z), pinhole, vertical field of view 75 deg, 96 x 96
WR_MOUNT = np.array([0.0, 0.050, -0.097])
WR_AXES = np.diag([-1.0, 1.0, -1.0])
WR_N = 96
WR_F = (WR_N / 2.0) / math.tan(math.radians(75.0) / 2)
WR_VALID_ROWS = 70                              # the hand's own fingers fill the rows below
SCAN_CAM_H = 0.27                               # camera height over the table for the scan
# where the scan puts the wrist camera: with the free rows of each image, the six views cover the parts' region
SCAN_XY = ((-0.11, -0.28), (-0.11, -0.03), (-0.11, 0.22), (0.13, 0.22), (0.13, -0.03), (0.13, -0.28))
LOOK_CAM_H = 0.17                               # ... and for a close look at a part
LOOK_H = {"chassis": 0.25, "axle": 0.21, "wheel": 0.17, "cabin": 0.17}   # (long parts from higher up, wholly in view)
LOOK_ROW = 35.0                                 # a close look shows its target here: the middle of the free rows

# ------------------------------------------------------------------ the episode's kit, as read off its manual
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_manual  # noqa: E402

KIT = {}
TOP_H = {}
AXLE_DIA = 0.010                                # the family's axles are all 10 mm thick (instruction)


def set_kit(kit):
    """Make `kit` (as read off the manual) the one the controller builds."""
    KIT.clear()
    KIT.update(copy.deepcopy(kit))
    KIT["axle"]["dia"] = AXLE_DIA
    TOP_H.clear()
    TOP_H.update({"chassis": KIT["chassis"]["height"], "cabin": KIT["cabin"]["height"] + 0.001,
                  "wheel": KIT["wheel"]["thick"], "axle": KIT["axle"]["dia"]})
MATES = ("stand", "front_notch", "rear_notch", "front_left", "front_right", "rear_left", "rear_right", "cabin")
MATE_TYPE = {"stand": "chassis", "front_notch": "axle", "rear_notch": "axle", "front_left": "wheel",
             "front_right": "wheel", "rear_left": "wheel", "rear_right": "wheel", "cabin": "cabin"}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_axis(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def hand_axes(yaw, tilt):
    """(closing axis, approach, third axis) of the hand in the world at (yaw, tilt)."""
    c = rot_z(yaw) @ C0
    a = rot_z(yaw) @ rot_axis(C0, tilt) @ A0
    return c, a, np.cross(a, c)


def quat2mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def decode(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def yaw_step(target, yaw, q7):
    d = wrap(target - yaw)
    alt = d - 2 * math.pi * math.copysign(1.0, d)
    if abs(q7 - 0.9 * d) > Q7_LIMIT and abs(q7 - 0.9 * alt) < abs(q7 - 0.9 * d):
        return alt
    return d


def best_yaw(cands, yaw, q7):
    return min(cands, key=lambda c: abs(q7 - 0.9 * yaw_step(c, yaw, q7)))


def mod_pi_near(a, ref):
    return a + math.pi * round((ref - a) / math.pi)


# ------------------------------------------------------------------ the wrist camera
def wrist_pose(obs):
    Rh = quat2mat(obs["ee_quat"])
    return np.asarray(obs["ee_pos"], float) + Rh @ WR_MOUNT, Rh @ WR_AXES


def back_project(rows, cols, cpos, Rc, z):
    d = np.stack([(cols + 0.5 - WR_N / 2.0) / WR_F, -(rows + 0.5 - WR_N / 2.0) / WR_F, -np.ones(len(rows))], 1)
    d = d @ Rc.T
    s = (z - cpos[2]) / d[:, 2]
    return cpos[None, :2] + d[:, :2] * s[:, None]


def masks(img):
    im = img.astype(np.int16)
    R, G, B = im[..., 0], im[..., 1], im[..., 2]
    mx, mn = im.max(axis=2), im.min(axis=2)
    return {
        "chassis": (R > 140) & (R > 1.7 * G) & (R > 1.7 * B),
        "cabin": (B > 120) & (B > R + 40) & (B > G + 15),      # blue sides and rim; its white roof, see detect()
        "white": (mn > 185) & (mx - mn < 30),
        "wheel": (mx < 115) & (mx - mn < 24),
        "axle": (R > 165) & (G > 135) & (B > 95) & (R - B > 35) & (B < G),
        "plate": (mn > 150) & (mx - mn < 24),                     # the stand's top plate, seen from above
    }


MIN_PX = {"chassis": 30, "cabin": 15, "wheel": 10, "axle": 6, "plate": 30}


def min_rect(q):
    best = None
    for ang in np.radians(np.arange(0.0, 90.0, 1.0)):
        u1 = q @ np.array([math.cos(ang), math.sin(ang)])
        u2 = q @ np.array([-math.sin(ang), math.cos(ang)])
        e1, e2 = u1.max() - u1.min(), u2.max() - u2.min()
        if best is None or e1 * e2 < best[0]:
            best = (e1 * e2, ang, e1, e2, (u1.max() + u1.min()) / 2, (u2.max() + u2.min()) / 2)
    _a, ang, e1, e2, m1, m2 = best
    centre_shift = m1 * np.array([math.cos(ang), math.sin(ang)]) + m2 * np.array([-math.sin(ang), math.cos(ang)])
    if e2 > e1:
        ang, e1, e2 = ang + math.pi / 2, e2, e1
    return wrap(ang), e1, e2, centre_shift


def detect(obs, types, top_z=None):
    """Parts the wrist camera sees: dicts (type, x, y, theta, length, width, dia, whole, dist_px)."""
    img = decode(obs["wrist"]) if isinstance(obs["wrist"], dict) else np.asarray(obs["wrist"])
    cpos, Rc = wrist_pose(obs)
    out = []
    ms = masks(img)
    for typ in types:
        m = ms[typ].copy()
        if typ == "cabin":                                        # the blue box and its white roof inside the blue
            m = m | (ms["white"] & ndimage.binary_dilation(m, iterations=6))
        m[WR_VALID_ROWS:] = False
        m = ndimage.binary_fill_holes(ndimage.binary_closing(m, np.ones((3, 3), bool)))
        lab, n = ndimage.label(m)
        z = (top_z if top_z is not None else TABLE_Z + TOP_H.get(typ, 0.0))
        for k in range(1, n + 1):
            rows, cols = np.nonzero(lab == k)
            if len(rows) < MIN_PX[typ]:
                continue
            whole = rows.min() > 0 and cols.min() > 0 and cols.max() < WR_N - 1 and rows.max() < WR_VALID_ROWS - 1
            pts = back_project(rows, cols, cpos, Rc, z)
            c = pts.mean(axis=0)
            q = pts - c
            pix = (cpos[2] - z) / WR_F
            if typ in ("cabin", "plate"):
                theta, e1, e2, shift = min_rect(q)
                c = c + shift
                length, width = e1 + pix, e2 + pix
            else:
                w, V = np.linalg.eigh(q.T @ q / len(q))
                ax = V[:, 1]
                theta = math.atan2(ax[1], ax[0])
                a1, a2 = q @ ax, q @ V[:, 0]
                length, width = a1.max() - a1.min() + pix, a2.max() - a2.min() + pix
            out.append({"type": typ, "x": float(c[0]), "y": float(c[1]), "theta": float(theta),
                        "length": float(length), "width": float(width),
                        "dia": float(2 * math.sqrt(len(rows) * pix * pix / math.pi)), "whole": bool(whole),
                        "dist_px": float(math.hypot(rows.mean() - (WR_N / 2 - 0.5), cols.mean() - (WR_N / 2 - 0.5)))})
    return out


def size_err(det):
    """How far a part's measured size is from the kit's (m); the close-look measurement reads a little large."""
    t = det["type"]
    if t == "axle":
        return abs(det["length"] - KIT["axle"]["length"] - LOOK_BIAS["axle"])
    if t == "wheel":
        return abs(det["dia"] - KIT["wheel"]["dia"] - LOOK_BIAS["wheel"])
    if t == "cabin":
        return abs(det["length"] - KIT["cabin"]["length"] - LOOK_BIAS["cabin"])
    return 0.0


LOOK_BIAS = {"axle": 0.001, "wheel": 0.0, "cabin": 0.0015}
KIT_TOL = {"axle": 0.007, "wheel": 0.004, "cabin": 0.006}
SCAN_TOL = {"axle": 0.012, "wheel": 0.008, "cabin": 0.009}


# ------------------------------------------------------------------ the controller
PRESS_LIMIT = 20.0       # N: the most the reference pushes with (the parts break at 30 N)
PRESS_MAX_STEP = 0.014   # m per control step
PRESS_TARGET = 16.0      # N
PRESS_HOLD = 13.0        # N


def press_wp(pos, yaw, tilt, direction, travel, maxn=30, retry=()):
    """A force-guided insertion. When the push held but the hand did not go in by about the detent's depth (the part
    met the seat's shoulder off its axis), back off and try again from the next offset in `retry`."""
    w = wp(pos, yaw, tilt, 1.0, maxn=maxn)
    w["press"] = [float(v) for v in direction]
    w["travel"] = float(travel)
    w["retry"] = [list(map(float, r)) for r in retry]
    w["phase"] = "push"
    w["pos0"] = list(w["pos"])
    return w


def wp(pos, yaw, tilt, grip, tol=0.004, dwell=0, maxn=120, ang_tol=0.05, tag=None):
    return {"pos": [float(v) for v in pos], "yaw": float(yaw), "tilt": float(tilt), "grip": float(grip),
            "tol": tol, "dwell": dwell, "maxn": maxn, "n": 0, "ok": 0, "ang_tol": ang_tol, "tag": tag}


class Oracle(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.s = {"phase": "start", "wps": [], "done": [], "fails": {}, "job": None, "rec": None, "scan": [],
                  "cands": [], "stand": None, "Rh0": None, "seen": None, "queue": list(MATES), "rescans": 0,
                  "kit": None}
        self._snap = None
        self._t = None

    def act(self, t, obs):
        if t == self._t:
            self.s = copy.deepcopy(self._snap)
        else:
            self._snap = copy.deepcopy(self.s)
            self._t = t
        if self.s["kit"] is None and obs.get("manual"):
            # the episode's manual comes with the first observation only: read the car off it now
            self.s["kit"] = read_manual.read_images(read_manual.decode_manual(obs["manual"]))
        if self.s["kit"] is not None:
            set_kit(self.s["kit"])
        if not KIT:
            return self._hold(obs)
        return self._act(obs)

    # -- motion
    def _follow(self, obs):
        s = self.s
        ee = np.asarray(obs["ee_pos"], float)
        q7 = float(obs["q"][6])
        while s["wps"]:
            w = s["wps"][0]
            if "press" in w:
                a = self._press(w, obs, ee)
                if a is None:
                    s["wps"].pop(0)
                    continue
                return a
            dp = np.asarray(w["pos"]) - ee
            dyaw = yaw_step(w["yaw"], obs["ee_yaw"], q7)
            dtilt = w["tilt"] - obs["ee_tilt"]
            close = np.linalg.norm(dp) < w["tol"] and abs(dyaw) < w["ang_tol"] and abs(dtilt) < w["ang_tol"]
            if close:
                w["ok"] += 1
            if (close and w["ok"] > w["dwell"]) or w["n"] >= w["maxn"]:
                self._on_reached(w, obs)
                s["wps"].pop(0)
                continue
            w["n"] += 1
            if w.get("integ"):
                # integral action near the goal: the arm's controller leaves a steady error of up to ~1 cm in some
                # tilted postures (gravity and its posture term), too much for the press fits' lead-ins
                if np.linalg.norm(dp) < 0.03:
                    s["int"] = np.clip(np.asarray(s.get("int", [0.0, 0.0, 0.0])) + 0.3 * dp, -0.015, 0.015).tolist()
                dp = dp + np.asarray(s.get("int", [0.0, 0.0, 0.0]))
            else:
                s["int"] = [0.0, 0.0, 0.0]
            n = np.linalg.norm(dp)
            vmax = w.get("vmax", 0.02)
            if n > vmax:
                dp = dp * (vmax / n)
            tr = w.get("tilt_rate", 0.15)
            return [float(dp[0]), float(dp[1]), float(dp[2]), float(np.clip(dyaw, -0.15, 0.15)),
                    float(np.clip(dtilt, -tr, tr)), w["grip"]]
        return None

    def _press(self, w, obs, ee):
        """Force-guided insertion: push along w["press"] (a unit vector into the seat), with the step size regulated
        so that the wrist sensor's push settles near PRESS_TARGET (never above PRESS_LIMIT); a push is over once it
        has held at least PRESS_HOLD for a few steps (far longer than the detent needs to give). It worked when the
        hand then went in by about the detent's depth; if not, retry from the next offset."""
        d = np.asarray(w["press"])
        q7 = float(obs["q"][6])
        dyaw = yaw_step(w["yaw"], obs["ee_yaw"], q7)
        dtilt = float(np.clip(w["tilt"] - obs["ee_tilt"], -0.05, 0.05))
        if w["phase"] in ("back", "approach"):
            tgt = np.asarray(w["pos"]) - (d * 0.006 if w["phase"] == "back" else 0.0)
            dp = tgt - ee
            integ = np.clip(np.asarray(self.s.get("int", [0.0, 0.0, 0.0])) + 0.3 * dp, -0.015, 0.015)
            self.s["int"] = integ.tolist()
            dp = dp + integ
            w["m"] = w.get("m", 0) + 1
            if np.linalg.norm(dp) < 0.0015 or w["m"] > 15:
                w["m"] = 0
                if w["phase"] == "back":
                    w["phase"] = "approach"
                else:
                    w["phase"], w["n"], w["ok"] = "push", 0, 0
                return self._press(w, obs, ee)
            n = np.linalg.norm(dp)
            if n > 0.004:
                dp = dp * (0.004 / n)
            return [float(dp[0]), float(dp[1]), float(dp[2]), float(np.clip(dyaw, -0.15, 0.15)), dtilt, w["grip"]]
        f = np.asarray(obs["wrench"][:3], float)
        if w["n"] == 0:
            w["base"] = f.tolist()
            w["start"] = ee.tolist()
            w["step"] = 0.002
        push = float((f - np.asarray(w["base"])) @ d)
        if push >= PRESS_HOLD:
            w["ok"] += 1
        if w["ok"] >= 4 or w["n"] >= w["maxn"]:
            prog = float((ee - np.asarray(w["start"])) @ d)
            if prog < 0.6 * w["travel"] and w["retry"]:
                off = np.asarray(w["retry"].pop(0))
                w["pos"] = (np.asarray(w["pos0"]) + off).tolist()
                w["phase"], w["m"] = "back", 0
                return self._press(w, obs, ee)
            return None
        w["n"] += 1
        if push > PRESS_LIMIT:
            step = 0.0
        else:
            step = float(np.clip(w["step"] + 0.0004 * (PRESS_TARGET - push), 0.0, PRESS_MAX_STEP))
        w["step"] = step
        across = np.asarray(w["pos"]) - ee
        across = across - float(across @ d) * d
        integ = np.asarray(self.s.get("int", [0.0, 0.0, 0.0]))
        integ = np.clip(integ + 0.3 * across, -0.015, 0.015)
        integ = integ - float(integ @ d) * d
        self.s["int"] = integ.tolist()
        dp = d * step + np.clip(across, -0.003, 0.003) + integ
        return [float(dp[0]), float(dp[1]), float(dp[2]), float(np.clip(dyaw, -0.15, 0.15)), dtilt, w["grip"]]

    def _on_reached(self, w, obs):
        s = self.s
        tag = w["tag"]
        if tag == "grasped":
            s["rec"] = {"ee": list(map(float, obs["ee_pos"])), "yaw": float(obs["ee_yaw"]),
                        "width": float(obs["gripper"])}
        elif tag == "scan":
            for d in detect(obs, ("chassis", "cabin", "wheel", "axle")):
                s["scan"].append(d)
        elif tag == "look":
            s["seen"] = detect(obs, (s["job"]["type"],)) if s["job"] else []
        elif tag == "standlook":
            s["seen"] = detect(obs, ("plate",), top_z=STAND_TOP_Z)
        elif tag == "chassislook":
            s["seen"] = detect(obs, ("chassis",), top_z=STAND_TOP_Z + KIT["chassis"]["height"])

    def _hold(self, obs):
        ee = np.asarray(obs["ee_pos"], float)
        dp = PARK - ee
        n = np.linalg.norm(dp)
        if n > 0.02:
            dp = dp * (0.02 / n)
        return [float(dp[0]), float(dp[1]), float(dp[2]), 0.0, float(np.clip(-obs["ee_tilt"], -0.15, 0.15)), -1.0]

    def _grip_for_cam(self, xy, cam_h, yaw=0.0, row=None):
        """Grip-point position that puts the wrist camera cam_h above the table and shows the table point xy at
        image row `row` (default: the centre) of the middle column, the hand pointing down at yaw."""
        Rh = rot_z(yaw) @ np.asarray(self.s["Rh0"])
        off = Rh @ WR_MOUNT
        cam_xy = np.asarray(xy, float)
        if row is not None:
            up = (Rh @ WR_AXES)[:2, 1]                            # image up, in the world
            cam_xy = cam_xy - up / np.linalg.norm(up) * ((WR_N / 2 - 0.5 - row) * cam_h / WR_F)
        return np.array([cam_xy[0] - off[0], cam_xy[1] - off[1], TABLE_Z + cam_h - off[2]])

    def _look_wps(self, xy, cam_h, tag):
        g = self._grip_for_cam(xy, cam_h, row=LOOK_ROW)
        return [wp([g[0], g[1], max(g[2], SAFE_Z)], 0.0, 0.0, -1.0, tol=0.015, maxn=60, ang_tol=0.1),
                wp(g, 0.0, 0.0, -1.0, tol=0.003, dwell=2, maxn=40, ang_tol=0.02, tag=tag)]

    # -- the plan
    def _act(self, obs):
        s = self.s
        for _ in range(10):
            a = self._follow(obs)
            if a is not None:
                return a
            ph = s["phase"]
            if ph == "start":
                # the hand frame at yaw 0 (the wrist camera's mount turns with it)
                s["Rh0"] = (rot_z(-float(obs["ee_yaw"])) @ quat2mat(obs["ee_quat"])).tolist()
                s["wps"] = self._scan_wps(obs)
                s["phase"] = "merge"
            elif ph == "merge":
                self._merge_scan()
                s["wps"] = self._look_wps(STAND_NOM, 0.14 + (STAND_TOP_Z - TABLE_Z), "standlook")
                s["phase"] = "stand"
            elif ph == "remerge":
                self._merge_scan()
                s["phase"] = "next"
            elif ph == "stand":
                plates = [d for d in (s["seen"] or []) if d["length"] > 0.04]
                if plates:
                    p = min(plates, key=lambda d: math.hypot(d["x"] - STAND_NOM[0], d["y"] - STAND_NOM[1]))
                    s["stand"] = [p["x"], p["y"], wrap(mod_pi_near(p["theta"], 0.0))]
                else:
                    s["stand"] = [float(STAND_NOM[0]), float(STAND_NOM[1]), 0.0]
                s["phase"] = "next"
            elif ph == "next":
                if not self._next_job():
                    s["phase"] = "idle"
            elif ph == "job":
                self._job_stage(obs)
            else:
                return self._hold(obs)
        return self._hold(obs)

    def _scan_wps(self, obs):
        ws = [wp([obs["ee_pos"][0], obs["ee_pos"][1], max(obs["ee_pos"][2], SAFE_Z)], 0.0, 0.0, -1.0, tol=0.02,
                 maxn=30, ang_tol=0.3)]
        for xy in SCAN_XY:
            g = self._grip_for_cam(xy, SCAN_CAM_H)
            ws.append(wp(g, 0.0, 0.0, -1.0, tol=0.006, dwell=1, maxn=60, ang_tol=0.03, tag="scan"))
        self.s["scan"] = []
        return ws

    def _merge_scan(self):
        s = self.s
        old = s["cands"]
        cands = []
        for d in s["scan"]:
            if math.hypot(d["x"] - STAND_NOM[0], d["y"] - STAND_NOM[1]) < 0.085:
                continue                                          # the stand and what will stand on it
            same = [c for c in cands if c["type"] == d["type"] and math.hypot(c["x"] - d["x"], c["y"] - d["y"]) < 0.035]
            if same:
                c = same[0]
                better = (d["whole"] and not c["whole"]) or (d["whole"] == c["whole"] and d["dist_px"] < c["dist_px"])
                if better:
                    c.update(d)
                continue
            cands.append(dict(d, used=False, sibling=False))
        for c in cands:                                           # what an earlier look found out stays known
            for e in old:
                if e["type"] == c["type"] and math.hypot(e["x"] - c["x"], e["y"] - c["y"]) < 0.02:
                    c["sibling"] = c["sibling"] or e["sibling"]
        s["cands"] = cands

    def _next_job(self):
        s = self.s
        typ_needed = None
        while s["queue"]:
            mate = s["queue"][0]
            if s["fails"].get(mate, 0) >= 3:
                s["queue"].pop(0)
                continue
            typ = MATE_TYPE[mate]
            pool = [c for c in s["cands"] if c["type"] == typ and not c["used"] and not c["sibling"]]
            if typ != "chassis":
                pool = [c for c in pool if size_err(c) < SCAN_TOL[typ] + 0.004] or pool
            if not pool:
                if s["rescans"] < 2:                              # look over the table again before giving up
                    s["rescans"] += 1
                    s["wps"] = self._scan_wps({"ee_pos": [0.0, 0.0, SAFE_Z]})
                    s["phase"] = "remerge"
                    return True
                s["queue"].pop(0)
                continue
            if typ == "chassis":
                pick = pool[0]
            else:
                pick = min(pool, key=lambda c: (size_err(c), -self._clearance(c)))
            s["job"] = {"mate": mate, "type": typ, "cand": s["cands"].index(pick), "stage": "look"}
            s["phase"] = "job"
            return True
        return False

    def _clearance(self, c):
        others = [math.hypot(c["x"] - e["x"], c["y"] - e["y"]) for e in self.s["cands"] if e is not c and not e["used"]]
        return min(others) if others else 1.0

    def _fail(self, mate):
        s = self.s
        s["fails"][mate] = s["fails"].get(mate, 0) + 1
        s["phase"] = "next"
        s["job"] = None

    def _job_stage(self, obs):
        s = self.s
        job = s["job"]
        typ, stage, mate = job["type"], job["stage"], job["mate"]
        cand = s["cands"][job["cand"]]
        yaw, q7 = float(obs["ee_yaw"]), float(obs["q"][6])
        if stage == "look":
            s["seen"] = None
            s["wps"] = self._look_wps((cand["x"], cand["y"]), LOOK_H[typ], "look")
            job["stage"] = "classify"
            return
        if stage == "classify":
            near = [d for d in (s["seen"] or []) if math.hypot(d["x"] - cand["x"], d["y"] - cand["y"]) < 0.06]
            if not near:
                cand["used"] = True                               # gone: take another
                s["phase"], s["job"] = "next", None
                return
            d = min(near, key=lambda e: math.hypot(e["x"] - cand["x"], e["y"] - cand["y"]))
            if not d["whole"] and not job.get("relooked"):          # cut by the image's edge: centre it and look again
                job["relooked"] = True
                cand["x"], cand["y"] = d["x"], d["y"]
                job["stage"] = "look"
                return
            if typ != "chassis" and size_err(d) > KIT_TOL[typ]:
                cand["sibling"] = True
                s["phase"], s["job"] = "next", None
                return
            job["det"] = {"x": d["x"], "y": d["y"], "theta": d["theta"]}
            job["stage"] = "grasp"
        det = job.get("det")
        if job["stage"] == "grasp":
            x, y = det["x"], det["y"]
            # grip low on a wheel: thin wheels slip out of a high grasp when the hand tilts them upright
            gz = {"chassis": TABLE_Z + 0.010, "axle": TABLE_Z + 0.006, "wheel": TABLE_Z + 0.006,
                  "cabin": TABLE_Z + 0.014}[typ]
            if typ == "wheel":
                gyaw = 0.0
            else:
                g0 = wrap(det["theta"] + math.pi / 2)
                gyaw = best_yaw([g0, wrap(g0 + math.pi)], yaw, q7)
            s["wps"] = [wp([x, y, max(obs["ee_pos"][2], gz + 0.04)], gyaw, 0.0, -1.0, tol=0.006, maxn=40, ang_tol=0.06),
                        wp([x, y, gz + 0.03], gyaw, 0.0, -1.0, tol=0.004, maxn=40),
                        wp([x, y, gz], gyaw, 0.0, -1.0, tol=0.0015, dwell=1, maxn=30),
                        wp([x, y, gz], gyaw, 0.0, 1.0, tol=0.004, dwell=5, maxn=12, tag="grasped"),
                        wp([x, y, SAFE_Z], gyaw, 0.0, 1.0, tol=0.015, maxn=40)]
            job["stage"] = "to_place"
            return
        if job["stage"] == "to_place":
            if s["rec"] is None or s["rec"]["width"] < 0.003:   # the fingers closed on nothing
                cand["x"], cand["y"] = det["x"], det["y"]
                job["stage"] = "look"
                s["fails"][mate] = s["fails"].get(mate, 0) + 1
                if s["fails"][mate] >= 3:
                    cand["used"] = True
                    s["phase"], s["job"] = "next", None
                return
            cand["used"] = True
            tgt, pyaw_cands, tilt = self._place_target(job, s["rec"])
            job["tgt"] = tgt.tolist()
            if typ == "wheel":
                st = s["stand"]
                side = 1.0 if mate.endswith("left") else -1.0
                axle_dir = rot_z(st[2]) @ np.array([0.0, 1.0, 0.0])
                hi = tgt + axle_dir * (side * 0.09) + np.array([0.0, 0.0, 0.10])
                job["pyaw"], job["ptilt"] = pyaw_cands[0], tilt
                s["wps"] = [wp([obs["ee_pos"][0], obs["ee_pos"][1], SAFE_Z + 0.05], job["pyaw"], 0.0, 1.0,
                               tol=0.02, maxn=40, ang_tol=0.1),
                            dict(wp(hi, job["pyaw"], tilt, 1.0, tol=0.012, maxn=70, ang_tol=0.06), tilt_rate=0.07)]
            else:
                s["wps"] = [wp([tgt[0], tgt[1], SAFE_Z], yaw, 0.0, 1.0, tol=0.015, maxn=80, ang_tol=0.5)]
                job["pyaw_cands"] = pyaw_cands
            job["stage"] = "place"
            return
        if job["stage"] == "place":
            tgt = np.asarray(job["tgt"])
            if typ == "wheel":
                st = s["stand"]
                side = 1.0 if mate.endswith("left") else -1.0
                axle_dir = rot_z(st[2]) @ np.array([0.0, 1.0, 0.0])
                pyaw, pt = job["pyaw"], job["ptilt"]
                pre = tgt + axle_dir * (side * 0.045)
                out = tgt + axle_dir * (side * 0.07)
                near = tgt + axle_dir * (side * 0.008)
                s["wps"] = [dict(wp(pre, pyaw, pt, 1.0, tol=0.0015, dwell=1, maxn=50, ang_tol=0.03), integ=True),
                            dict(wp(near, pyaw, pt, 1.0, tol=0.0015, dwell=1, maxn=20, ang_tol=0.03), vmax=0.006,
                                 integ=True),
                            dict(wp(tgt, pyaw, pt, 1.0, tol=0.0015, dwell=1, maxn=20, ang_tol=0.03), vmax=0.0025,
                                 integ=True),
                            press_wp(tgt, pyaw, pt, -axle_dir * side, 0.0022, retry=[[0.0, 0.0, -0.006],
                                                                                  [0.0, 0.0, 0.004]]),
                            wp(tgt, pyaw, pt, -1.0, tol=0.004, dwell=3, maxn=8),
                            wp(out, pyaw, pt, -1.0, tol=0.01, maxn=20),
                            wp(out + [0, 0, 0.09], pyaw, pt, -1.0, tol=0.015, maxn=20),
                            wp(out + [0, 0, 0.10], pyaw, 0.0, -1.0, tol=0.03, maxn=30, ang_tol=0.3)]
            else:
                pyaw = best_yaw(job["pyaw_cands"], yaw, q7)
                tgt = self._place_target(job, s["rec"], pyaw)[0]
                grip_open = -0.4 if typ == "cabin" else -1.0
                s["wps"] = [wp(tgt + [0, 0, 0.03], pyaw, 0.0, 1.0, tol=0.003, maxn=40, ang_tol=0.03),
                            dict(wp(tgt, pyaw, 0.0, 1.0, tol=0.0015, dwell=1, maxn=(18 if typ == "axle" else 30),
                                    ang_tol=0.03), vmax=(0.006 if typ == "axle" else 0.02), integ=(typ == "axle"))]
                if typ == "axle":
                    s["wps"].append(press_wp(tgt, pyaw, 0.0, [0.0, 0.0, -1.0], 0.0030))
                s["wps"] += [wp(tgt, pyaw, 0.0, grip_open, tol=0.004, dwell=3, maxn=8),
                            wp(tgt + [0, 0, 0.07], pyaw, 0.0, grip_open, tol=0.015, maxn=25)]
            job["stage"] = "placed"
            return
        if job["stage"] == "placed":
            s["done"].append(mate)
            s["queue"].pop(0)
            if typ == "chassis":
                # the chassis sits exactly where the stand is: measure it to set the frame of every later mate
                st = s["stand"]
                s["wps"] = self._look_wps(st[:2], 0.14 + (STAND_TOP_Z - TABLE_Z) + KIT["chassis"]["height"],
                                          "chassislook")
                job["stage"] = "chassis_check"
                return
            s["phase"], s["job"] = "next", None
            return
        if job["stage"] == "chassis_check":
            st = s["stand"]
            reds = [d for d in (s["seen"] or []) if math.hypot(d["x"] - st[0], d["y"] - st[1]) < 0.04
                    and d["length"] > 0.6 * KIT["chassis"]["length"]]
            if reds:
                d = min(reds, key=lambda e: math.hypot(e["x"] - st[0], e["y"] - st[1]))
                s["stand"] = [d["x"], d["y"], wrap(mod_pi_near(d["theta"], st[2]))]
            s["phase"], s["job"] = "next", None

    def _place_target(self, job, rec, pyaw=None):
        """Grip-point target for placing, from the grasp as it happened, and the hand yaws that seat the part."""
        typ, det, mate = job["type"], job["det"], job["mate"]
        c = KIT["chassis"]
        st = self.s["stand"]
        Rs = rot_z(st[2])
        base = np.array([st[0], st[1], STAND_TOP_Z])
        ee = np.asarray(rec["ee"])
        ref_z = {"chassis": TABLE_Z, "cabin": TABLE_Z, "axle": TABLE_Z + KIT["axle"]["dia"] / 2,
                 "wheel": TABLE_Z + KIT["wheel"]["thick"] / 2}[typ]
        off = np.array([det["x"], det["y"], ref_z]) - ee
        cg, ag, bg = hand_axes(rec["yaw"], 0.0)
        k_a, k_b = float(off @ ag), float(off @ bg)
        if typ == "wheel":
            # the pads centre a wheel across them: its offset in the hand's y comes from the detection's error
            # (a few mm), not from the grasp; only the depth (k_b, from the grasp height) is real
            k_a = 0.0
        zc = c.get("axle_height", c["base_t"] + KIT["axle"]["dia"] / 2)
        if typ == "wheel":
            sx = c["notch_x"] if mate.startswith("front") else -c["notch_x"]
            side = 1.0 if mate.endswith("left") else -1.0
            centre = base + Rs @ np.array([sx, side * (KIT["axle"]["length"] / 2 - KIT["wheel"]["thick"] / 2), zc])
            mtilt = -side * math.pi / 2
            cp, ap, bp = hand_axes(st[2], mtilt)
            return centre - (k_a * ap + k_b * bp), [st[2]], mtilt
        if typ == "chassis":
            ref = base
            yaw_c = [wrap(st[2] + math.pi / 2), wrap(st[2] - math.pi / 2)]
        elif typ == "cabin":
            ref = base + np.array([0.0, 0.0, c["height"]])
            yaw_c = [wrap(st[2] + math.pi / 2), wrap(st[2] - math.pi / 2)]
        else:
            sx = c["notch_x"] if mate == "front_notch" else -c["notch_x"]
            ref = base + Rs @ np.array([sx, 0.0, zc])
            yaw_c = [wrap(st[2]), wrap(st[2] + math.pi)]
        if pyaw is None:
            return ref, yaw_c, 0.0
        cp, ap, bp = hand_axes(pyaw, 0.0)
        tgt = ref - (k_a * ap + k_b * bp) + np.array([0.0, 0.0, 0.003])
        return tgt, [pyaw], 0.0


def main():
    ctrl = Oracle()
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
            ctrl.reset()
            out.write(json.dumps({"ok": True}) + "\n")
        elif typ == "act":
            a = ctrl.act(int(msg["t"]), msg["obs"])
            out.write(json.dumps({"action": a}) + "\n")
        elif typ == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            break
        else:
            out.write(json.dumps({"ok": False}) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
