"""Reference supervisor for driving_safety_supervisor.

Built only from what the agent also gets: the front RGB camera, the ego speed, the next
route target and command, and the nominal planner's control.

* Perception, every step: torchvision's COCO Faster R-CNN for road users and OWLv2 (plus
  an orange-paint mask) for props on the road. A detection's ground contact gives its
  distance and lateral extent through the flat-road geometry of the level camera.
* Tracking: detections are associated over steps; a least-squares fit over the recent
  history gives each track its closing and lateral rates.
* Braking: tracks inside the corridor along the planner's curvature (widened by their
  predicted lateral motion) set an allowed speed from a comfortable-deceleration envelope;
  above it the supervisor brakes, otherwise the planner's control passes through.
* Yielding: on a left turn it waits for oncoming vehicles.
* Unblocking: stopped behind a static blocker away from a junction, with no pedestrian
  about and the left side clear, it shifts left by a lane past the blocker and hands back.

A repeated query for the same `t` is answered from that observation, but only objects both
queries agree on are committed to the tracker.
"""
import base64
import copy
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import torch

W, H = 512, 256
F = W / 2.0               # focal length, px (90 deg HFOV)
HOR = H / 2.0             # horizon row (camera is level)
CAM_Z = 1.6
BUMPER = 0.8              # camera (1.5 m ahead of centre) to front bumper, m
DT = 0.1
DEBUG = os.environ.get("SUP_DEBUG", "") != ""


def log(*a):
    sys.stderr.write(" ".join(str(x) for x in a) + "\n")
    sys.stderr.flush()


# ---------------------------------------------------------------- detection
COCO = {1: "person", 2: "bicycle", 3: "car", 4: "motorcycle", 6: "bus", 8: "truck"}
OWL_QUERIES = ["a traffic cone", "a road barrier", "a construction warning sign trailer",
               "a cardboard box on the road", "a trash bin on the road"]
OWL_THRESH = [0.14, 0.2, 0.2, 0.25, 0.3]


class Detector(object):
    def __init__(self):
        from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2
        dev = "cuda" if torch.cuda.is_available() else "cpu"
        self.dev = dev
        self.half = dev == "cuda"
        m = fasterrcnn_resnet50_fpn_v2(weights=None, weights_backbone=None, num_classes=91,
                                       min_size=512, max_size=1024, box_score_thresh=0.3)
        m.load_state_dict(torch.load("/opt/models/fasterrcnn_resnet50_fpn_v2_coco.pth",
                                     map_location="cpu"))
        m.eval().to(dev)
        if self.half:
            m.half()
        self.frcnn = m
        self.owl = None
        try:
            from transformers import Owlv2ForObjectDetection, Owlv2Processor
            path = "/opt/models/owlv2-base-patch16-ensemble"
            proc = Owlv2Processor.from_pretrained(path)
            om = Owlv2ForObjectDetection.from_pretrained(path).eval().to(dev)
            if self.half:
                om.half()
            tok = proc.tokenizer(OWL_QUERIES, padding="max_length", max_length=16,
                                 return_tensors="pt")
            self.owl_ids = tok["input_ids"].to(dev)
            self.owl_mask = tok["attention_mask"].to(dev)
            self.owl = om
            self.owl_size = 960
            mean = torch.tensor([0.48145466, 0.4578275, 0.40821073], device=dev).view(3, 1, 1)
            std = torch.tensor([0.26862954, 0.26130258, 0.27577711], device=dev).view(3, 1, 1)
            self.owl_mean, self.owl_std = mean, std
        except Exception as exc:  # noqa: BLE001
            log("owl load failed", exc)

    @torch.no_grad()
    def __call__(self, rgb):
        dets = []
        x = torch.from_numpy(rgb).to(self.dev).permute(2, 0, 1).float() / 255.0
        xi = x.half() if self.half else x
        out = self.frcnn([xi])[0]
        boxes = out["boxes"].float().cpu().numpy()
        labels = out["labels"].cpu().numpy()
        scores = out["scores"].float().cpu().numpy()
        for b, l, s in zip(boxes, labels, scores):
            name = COCO.get(int(l))
            if name is None:
                continue
            thr = 0.45 if name == "person" else 0.5
            if s >= thr:
                dets.append([name, float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s)])
        if self.owl is not None:
            S = self.owl_size
            scale = S / float(W)
            img = torch.nn.functional.interpolate(x[None], size=(int(H * scale), S),
                                                  mode="bilinear", align_corners=False)[0]
            pad = torch.full((3, S, S), 0.5, device=self.dev)
            pad[:, :img.shape[1], :] = img
            pv = ((pad - self.owl_mean) / self.owl_std)[None]
            if self.half:
                pv = pv.half()
            o = self.owl(input_ids=self.owl_ids, attention_mask=self.owl_mask, pixel_values=pv)
            logits = torch.sigmoid(o.logits[0].float()).cpu().numpy()   # (N, Q)
            pb = o.pred_boxes[0].float().cpu().numpy()                    # cx, cy, w, h (0..1)
            best = logits.argmax(1)
            sc = logits[np.arange(len(best)), best]
            keep = [i for i in range(len(best)) if sc[i] >= OWL_THRESH[best[i]]]
            cand = []
            for i in keep:
                cx, cy, bw, bh = pb[i] * S / scale
                cand.append([float(sc[i]), "prop", cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2])
            cand.sort(reverse=True)
            taken = []
            for c in cand:
                bx = c[2:]
                if (bx[2] - bx[0]) > 0.6 * W or (bx[3] - bx[1]) > 0.8 * H:
                    continue
                if any(_iou(bx, t) > 0.4 for t in taken):
                    continue
                # props overlapping a road-user box are that road user
                if any(_iou(bx, d[1:5]) > 0.4 for d in dets):
                    continue
                taken.append(bx)
                dets.append(["prop"] + [float(v) for v in bx] + [c[0]])
        dets.extend(orange_props(rgb, dets))
        return dets


def orange_props(rgb, dets):
    """Traffic cones / construction props by their saturated orange paint."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    lo = int(HOR) + 2
    sub = hsv[lo:]
    mask = ((sub[:, :, 0] >= 4) & (sub[:, :, 0] <= 20) & (sub[:, :, 1] >= 160) &
            (sub[:, :, 2] >= 110)).astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < 8 or h < 3:
            continue
        box = [float(x), float(y + lo), float(x + w), float(y + lo + h)]
        if any(_iou(box, d[1:5]) > 0.1 or (d[0] != "prop" and d[1] <= box[0] and
                                           box[2] <= d[3] and d[2] <= box[1] and
                                           box[3] <= d[4]) for d in dets):
            continue
        out.append(["prop"] + box + [0.5])
    return out


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def ground(det):
    """Detection -> (Z forward from camera, x_left, x_right lateral, m) or None."""
    name, x1, y1, x2, y2 = det[:5]
    if y2 < HOR + 4:
        return None
    z = F * CAM_Z / (y2 - HOR)
    if z > 60:
        return None
    return z, (x1 - F) * z / F, (x2 - F) * z / F


# ---------------------------------------------------------------- tracking
class Track(object):
    def __init__(self, tid, kind, z, xl, xr, t):
        self.id, self.kind = tid, kind
        self.hist = [(t, z, xl, xr)]
        self.miss = 0
        self.ghost = False      # a misdetection: ignored for good
        self.late = False       # first seen close while moving: needs more hits

    @property
    def z(self):
        return self.hist[-1][1]

    @property
    def xl(self):
        return self.hist[-1][2]

    @property
    def xr(self):
        return self.hist[-1][3]

    def rates(self):
        """(dz/dt, dx/dt) by least squares over the recent history; None if too short."""
        h = self.hist[-8:]
        if len(h) < 4:
            return None
        t = np.array([p[0] for p in h], float)
        t = t - t.mean()
        if (t * t).sum() <= 0:
            return None
        z = np.array([p[1] for p in h])
        x = np.array([(p[2] + p[3]) / 2 for p in h])
        return float((t * (z - z.mean())).sum() / (t * t).sum()), \
            float((t * (x - x.mean())).sum() / (t * t).sum())


def group(kind):
    if kind in ("car", "truck", "bus"):
        return "veh"
    if kind in ("bicycle", "motorcycle"):
        return "bike"
    return kind


class Tracker(object):
    def __init__(self):
        self.tracks = []
        self.next_id = 0

    def update(self, objs, t):
        """objs: list of (kind, z, xl, xr)."""
        used = set()
        for o in objs:
            kind, z, xl, xr = o
            cx = (xl + xr) / 2
            best, bd = None, 1e9
            for tr in self.tracks:
                if tr.id in used or group(tr.kind) != group(kind):
                    continue
                gate = 2.0 + 0.15 * z
                d = abs(tr.z - z) + abs((tr.xl + tr.xr) / 2 - cx)
                if d < gate and d < bd:
                    best, bd = tr, d
            if best is None:
                best = Track(self.next_id, kind, z, xl, xr, t)
                self.next_id += 1
                self.tracks.append(best)
            else:
                best.hist.append((t, z, xl, xr))
                best.hist = best.hist[-20:]
                best.miss = 0
            used.add(best.id)
        for tr in self.tracks:
            if tr.id not in used:
                tr.miss += 1
        self.tracks = [tr for tr in self.tracks if tr.miss <= 3]


# ---------------------------------------------------------------- supervisor
class Supervisor(object):
    def __init__(self, det):
        self.det = det
        self.reset()

    def reset(self):
        self.tracker = Tracker()
        self.last_t = -1
        self.snapshot = None       # state before the current t was applied
        self.first_objs = None
        self.odo = 0.0
        self.still_block = 0.0     # time stopped behind the same static blocker
        self.bypass = None         # active bypass manoeuvre
        self.yield_t = 0.0
        self.hold_t = 0.0
        self.closed = False        # barriers seen across the passing lane: wait it out

    def _state(self):
        return (copy.deepcopy(self.tracker), self.odo, self.still_block,
                copy.deepcopy(self.bypass), self.yield_t, self.hold_t, self.closed)

    def _restore(self, s):
        (self.tracker, self.odo, self.still_block, self.bypass, self.yield_t, self.hold_t,
         self.closed) = (copy.deepcopy(s[0]), s[1], s[2], copy.deepcopy(s[3]), s[4], s[5], s[6])

    def act(self, t, obs):
        img = base64.b64decode(obs["image"])
        bgr = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        dets = self.det(rgb)
        objs = []
        for d in dets:
            g = ground(d)
            if g is not None and d[0] == "prop":
                hpx, wpx = d[4] - d[2], d[3] - d[1]
                if hpx < F * 0.35 / g[0] or wpx > F * 4.0 / g[0]:
                    continue        # flat (road marking) or too wide to be a prop
            if g is not None:
                objs.append((d[0], g[0], g[1], g[2]))
        if t == self.last_t and self.snapshot is not None:
            # repeated query: answer from this observation; commit only objects both agree on
            common = [o for o in objs if any(o[0] == p[0] and abs(o[1] - p[1]) < 1.0 and
                                             abs(o[2] - p[2]) < 1.0 for p in self.first_objs)]
            self._restore(self.snapshot)
            action = self._decide(t, obs, objs)
            self._restore(self.snapshot)
            self._decide(t, obs, common)
            return action
        self.snapshot = self._state()
        self.first_objs = objs
        self.last_t = t
        return self._decide(t, obs, objs)

    # -- path geometry
    def _curv(self, obs):
        fwd, right = obs["target"]
        if fwd < 3.0:
            return 0.0
        k = 2.0 * right / (fwd * fwd + right * right)
        return max(-0.12, min(0.12, k))

    def _decide(self, t, obs, objs):
        v = float(obs["speed"])
        nom = [float(a) for a in obs["nominal"]]
        cmd = obs["command"]
        tfwd, tright = obs["target"]
        n_before = self.tracker.next_id
        self.tracker.update(objs, t * DT)
        for tr in self.tracker.tracks:
            # static props are usually seen from afar first; one popping up close is
            # suspect
            if tr.id >= n_before and tr.kind == "prop":
                if t < 30 and tr.z < 6.0:
                    tr.ghost = True     # under the bonnet at the start: road texture
                elif v > 2.0 and tr.z < 4.0 + 1.5 * v:
                    tr.late = True      # unless it persists, it is road texture or shadow
        self.odo += v * DT
        k = self._curv(obs)
        offset = 0.0
        if self.bypass is not None:
            offset = self.bypass["offset"]

        # ---------------- hazards in the path
        v_allow = 99.0
        in_path = []
        oncoming = False
        for tr in self.tracker.tracks:
            if tr.miss > 0:
                continue
            z = tr.z
            d = z - BUMPER
            centre = k * z * z / 2.0 - offset
            r = tr.rates()
            vobj_fwd = 0.0      # object's own forward speed (ego frame)
            vx = 0.0
            if r is not None:
                vobj_fwd = v + r[0]
                vx = r[1]
            hw = 1.25
            if tr.kind == "person":
                hw = 1.6
            elif tr.kind == "bike":
                hw = 1.4
            xl, xr = tr.xl, tr.xr
            # predict lateral motion over the time it takes us to get there
            closing = max(0.5, v - min(vobj_fwd, v))
            tt = min(3.0, d / max(closing, 1.0))
            if abs(vx) > 0.6:
                pxl, pxr = xl + vx * tt, xr + vx * tt
                xl, xr = min(xl, pxl), max(xr, pxr)
            if tr.kind == "prop" and (len(tr.hist) < 3 or tr.ghost or
                                      (tr.late and len(tr.hist) < 5)):
                continue
            if self.bypass is not None:
                # bypassing: the blocker and props sit to our right; watch a corridor
                # slightly left for road users only
                if tr.kind == "prop" or (tr.id == self.bypass["blocker"] and
                                         tr.kind != "person"):
                    continue
                if tr.kind != "person" and (r is None or vobj_fwd > -3.0):
                    continue
                centre = k * z * z / 2.0 - 0.3
            overl = xr > centre - hw and xl < centre + hw
            if overl and d < 40:
                moving_away = r is not None and vobj_fwd > 1.5 and tr.kind != "prop"
                in_path.append((d, tr, vobj_fwd if moving_away else 0.0))
            # oncoming vehicle for a left turn
            if group(tr.kind) in ("veh", "bike") and r is not None and d < 45:
                cx = (tr.xl + tr.xr) / 2
                if r[0] < -(v + 2.0) and -14.0 < cx < 1.0:
                    oncoming = True

        a_comf = 3.0
        worst = None
        for d, tr, vf in in_path:
            margin = 3.5
            if vf <= 0.0 and len(tr.hist) >= 3:
                margin = 8.5 if tr.kind == "prop" else 5.5
            room = max(0.0, d - margin)
            va = math.sqrt(2 * a_comf * room) + max(0.0, vf)
            if va < v_allow:
                v_allow, worst = va, (d, tr, vf)

        # ---------------- yield for oncoming traffic on a left turn
        yielding = False
        if cmd == "left" and oncoming and tfwd < 25.0 and self.bypass is None:
            yielding = True

        # ---------------- static blocker: bypass on the left
        walker_ahead = False
        if worst is not None and worst[1].kind == "person" and worst[0] < 15:
            r = worst[1].rates()
            walker_ahead = r is not None and 0.4 < v + r[0] < 2.5 and v < 2.5
        if worst is not None and v < 0.3 and worst[0] < 14 and worst[2] <= 0.0 and \
                worst[1].kind != "person":
            self.still_block += DT
        elif walker_ahead:
            # following someone walking along the lane: pass them like a slow blocker
            self.still_block += DT
        elif v > 0.5:
            self.still_block = 0.0
        need = 3.0 if worst is not None and worst[1].kind == "prop" else 6.0
        near_junction = cmd != "lanefollow" and tfwd < (worst[0] + 12 if worst else 0)
        if self.bypass is None and self.still_block > need and not near_junction and \
                worst is not None:
            bl = worst[1]
            left_busy = False
            for tr in self.tracker.tracks:
                if tr is bl or tr.miss > 0:
                    continue
                if tr.kind == "prop" and not tr.ghost and len(tr.hist) >= 3:
                    # a barrier in the lane we would pass in: the road is closed, wait
                    if abs(tr.z - bl.z) < 10.0 and -6.0 < (tr.xl + tr.xr) / 2 < -2.3:
                        self.closed = True
                    continue
                if group(tr.kind) not in ("veh", "bike", "person"):
                    continue
                if tr.kind == "person":
                    # never swerve towards a pedestrian in or stepping into the passing lane
                    cx = (tr.xl + tr.xr) / 2
                    r = tr.rates()
                    toward = r is not None and r[1] > 0.3
                    if tr.z < 30 and (-6.0 < cx < 1.5 or (toward and -10.0 < cx < 1.5)):
                        left_busy = True
                    continue
                if tr.xr > bl.xl - 0.3 or (tr.xl + tr.xr) / 2 < -7.0:
                    continue
                r = tr.rates()
                moving = r is not None and abs(v + r[0]) > 1.0
                if (moving and tr.z < 40) or tr.z < bl.z + 12:
                    left_busy = True
            if DEBUG and int(self.still_block * 10) % 20 == 0:
                log("  tracks", [(tr.kind, round(tr.z, 1), round(tr.xl, 1), round(tr.xr, 1),
                                  len(tr.hist), tr.miss, tr.ghost, tr.late)
                                 for tr in self.tracker.tracks])
                log("t=%d blocked %.1fs by %s, left_busy=%s" % (t, self.still_block, bl.kind,
                                                                  left_busy))
            if not left_busy and not self.closed:
                length = {"prop": 10.0, "person": 25.0}.get(worst[1].kind, 9.0)
                self.bypass = {"start": self.odo, "end": self.odo + worst[0] + length + 9.0,
                               "offset": 0.0, "blocker": worst[1].id, "y": 0.0}
                log("bypass start t=%d d=%.1f kind=%s" % (t, worst[0], worst[1].kind))
        if self.bypass is not None:
            return self._bypass_control(t, obs, v, v_allow, in_path)

        if yielding:
            if DEBUG:
                log("t=%d yield" % t)
            return [1, 0.0, nom[1], 0.6 if v > 1.0 else 1.0]

        if worst is None or worst[0] > 25.0:
            self.hold_t = 0.0
            self.closed = False
            return [0]
        # held still behind a vehicle against the planner's wish for long: the planner
        # tracks vehicles itself, so defer to it rather than block forever
        if v < 0.3 and nom[0] > 0.1 and group(worst[1].kind) in ("veh", "bike") and \
                self.bypass is None:
            self.hold_t += DT
        elif v >= 0.3:
            self.hold_t = 0.0
        if self.hold_t > 12.0 and worst[0] > 2.0:
            return [1, min(nom[0], 0.4), nom[1], nom[2]]
        # something in our path within 25 m: take over, never faster than allowed
        steer = nom[1]
        if v > v_allow:
            excess = v - v_allow
            brake = min(0.6, 0.2 + 0.2 * excess)
            room = max(0.1, worst[0] - 1.5)
            need_a = v * v / (2 * room)
            if need_a > 5.5:
                brake = 1.0
            elif need_a > 4.0:
                brake = max(brake, 0.75)
            act = [1, 0.0, steer, brake]
        elif v_allow < 0.3:
            act = [1, 0.0, steer, 1.0]
        else:
            thr = nom[0]
            if v > v_allow - 1.0:
                thr = min(thr, 0.3)
            act = [1, thr, steer, nom[2]]
        if DEBUG:
            log("t=%d v=%.1f d=%.1f %s va=%.1f nom=%s act=%s" % (
                t, v, worst[0], worst[1].kind, v_allow, nom, act))
        return act

    def _bypass_control(self, t, obs, v, v_allow, in_path):
        """Shift left by ~3.2 m past the blocker, then hand back to the planner.
        y: estimated offset left of the route; psi: heading left of the route, read from
        the bearing of the route target corrected for our own offset."""
        b = self.bypass
        done = self.odo - b["start"]
        remaining = b["end"] - self.odo
        tfwd, tright = obs["target"]
        fwd = max(tfwd, 6.0)
        if "k0" not in b:
            # stopped behind the blocker we are aligned with the lane: the target's bearing
            # is all road curvature
            b["k0"] = max(-0.1, min(0.1, 2.0 * math.atan2(tright, fwd) / fwd))
        psi = (tright - b["y"]) / fwd - b["k0"] * fwd / 2.0
        b["y"] += v * DT * math.sin(max(-0.6, min(0.6, psi)))
        y_d = min(3.3, 1.0 + done * 0.6)
        if remaining < 8.0:
            y_d = min(y_d, max(0.0, 3.2 * remaining / 8.0))
        if remaining < 1.5 or done > 80.0:
            log("bypass end t=%d y=%.1f" % (t, b["y"]))
            self.bypass = None
            return [0]
        psi_d = max(-0.4, min(0.4, 0.4 * (y_d - b["y"])))
        steer = max(-0.6, min(0.6, -1.8 * (psi_d - psi)))
        vt = 3.5
        if v_allow < vt:
            vt = v_allow
        if v < vt - 0.3:
            thr, brk = min(0.6, 0.35 + 0.2 * (vt - v)), 0.0
        elif v > vt + 0.5:
            thr, brk = 0.0, min(0.6, 0.2 + 0.2 * (v - vt))
        else:
            thr, brk = 0.25, 0.0
        if vt < 0.3:
            thr, brk = 0.0, 1.0
        if DEBUG and in_path:
            log("  in path:", [(round(d, 1), tr.kind, round(vf, 1)) for d, tr, vf in in_path])
        if DEBUG:
            log("t=%d bypass y=%.2f yd=%.1f psi=%.2f rem=%.1f steer=%.2f v=%.1f tgt=%s" % (
                t, b["y"], y_d, psi, remaining, steer, v, obs["target"]))
        return [1, thr, steer, brk]


def main():
    t0 = time.time()
    det = Detector()
    dummy = np.zeros((H, W, 3), np.uint8)
    for _ in range(2):
        det(dummy)
    sup = Supervisor(det)
    log("loaded in %.1fs" % (time.time() - t0))
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        typ = msg.get("type")
        if typ == "reset":
            sup.reset()
            reply = {"ok": True}
        elif typ == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            break
        elif typ == "act":
            try:
                action = sup.act(int(msg.get("t", 0)), msg["obs"])
                action = [float(a) for a in action]
                if not all(math.isfinite(a) for a in action):
                    action = [0]
                if action[0] == 0:
                    action = [0]
                else:
                    action = [1, min(1.0, max(0.0, action[1])),
                              min(1.0, max(-1.0, action[2])), min(1.0, max(0.0, action[3]))]
            except Exception as exc:  # noqa: BLE001
                log("act error", repr(exc))
                action = [0]
            reply = {"action": action}
        else:
            reply = {"ok": True}
        out.write(json.dumps(reply) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
