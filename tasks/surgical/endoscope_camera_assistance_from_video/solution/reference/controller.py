"""The reference camera-holding controller.

It does not look at the picture. The grader recognises the reference submission by its token (.oracle_token) and adds
`obs["privileged"]` to every observation: what a perfect perception system would have read from the same delayed
frame (the scope fitted, the camera head's angle on the eyepiece, the video latency, whether the lens is smeared, each
instrument's tip in the camera frame). Everything else is
what any submission has: the joint readings, the published holder model (holder_model.py) and the published view
rules. It is not told which instruments count, when the surgeon does fine work, where the tissue is or what comes
next, and it moves the holder only by the rate commands of the public wire.

What it does each step:

* rebuilds the tips in the patient frame from the delayed frame's camera pose (its own joint history);
* tells an exchange from the motion (one hand leaves along its shaft while the other stays) and keeps the view
  where the work is until that hand is back at its place;
* aims at the middle of the work at a working distance set by how far apart the tips are (resolved-rate control with
  the model's Jacobian, the tips' velocity fed forward, the frame's latency predicted away);
* keeps the horizon level (level_roll, with the scope and the camera head's angle);
* stands still while the tips do, and moves the image slowly when it has to move during fine work;
* withdraws into the cannula when the lens is smeared, waits until it is clean, and comes back.

State is keyed by the step index: a step asked twice is answered twice from the observation each request carries.
Both observations of such a step are possible (the copy shows the instruments as they would be had they moved
differently over the frame's last step), so each request's memory is kept, and when the next step arrives the one
whose tips the new frame continues becomes the controller's memory.
"""

import json
import math
import sys

import numpy as np

import holder_model as model

HANDS = ("left", "right")
NOMINAL_TROCAR = {"left": np.array([-0.075, -0.045, -0.035]), "right": np.array([0.075, -0.045, -0.035])}
TARGET_PX = 31.0            # the apparent shaft diameter aimed for (the zoom band is 22-44 px)
COMFORT_U, COMFORT_V = 70.0, 50.0   # the working point may sit this far from the image centre before the view moves
FINE_SAMPLES = 9            # frames (0.8 s) over which the tips must stay put to count as fine work (the rule: 1.0 s)
CALM_IMAGE_SPEED = 36.0     # px/s the view may move the working point during fine work (limit 48)


class State(object):
    """The controller's memory after a step (immutable by convention: `advance` returns a new one)."""

    def __init__(self):
        self.q_hist = []              # the joint readings of the last steps (the delayed frame's pose is among them)
        self.tips = {}                # hand -> world tip of the latest frame
        self.vel = {h: np.zeros(3) for h in HANDS}
        self.away = {h: False for h in HANDS}
        self.work = {}                # hand -> where it last worked (kept while it is away)
        self.out_count = {h: 0 for h in HANDS}
        self.history = {h: [] for h in HANDS}
        self.cleaning = False
        self.calm = False

    def copy(self):
        s = State()
        s.q_hist = [v.copy() for v in self.q_hist]
        s.tips = {h: v.copy() for h, v in self.tips.items()}
        s.vel = {h: v.copy() for h, v in self.vel.items()}
        s.away = dict(self.away)
        s.work = {h: v.copy() for h, v in self.work.items()}
        s.out_count = dict(self.out_count)
        s.history = {h: list(v) for h, v in self.history.items()}
        s.cleaning = self.cleaning
        s.calm = self.calm
        return s


def advance(state, q, priv):
    """(new state, action) from the state before this step and this step's observation."""
    s = state.copy()
    scope, head, lat = priv["scope"], math.radians(priv["camera_head_deg"]), int(priv["latency_steps"])
    q = np.asarray(q, dtype=float)
    q_frame = frame_joints(s.q_hist, q, lat)               # the frame is `lat` steps old: so is its camera pose
    r_wc, p = model.camera_pose(q_frame, scope, head)
    items = {i["role"]: i for i in priv["instruments"]}

    # 1. the tips in the patient frame, their velocity
    for h in HANDS:
        it = items[h]
        if not it["present"]:
            s.away[h] = True
            s.vel[h] = np.zeros(3)
            s.tips.pop(h, None)
            s.history[h] = []
            continue
        world = p + r_wc @ np.asarray(it["tip_cam"], dtype=float)
        if h in s.tips:
            v = (world - s.tips[h]) / model.DT
            s.vel[h] = 0.5 * s.vel[h] + 0.5 * v
        s.tips[h] = world
        s.history[h] = (s.history[h] + [world])[-FINE_SAMPLES:]

    # 2. exchanges: a hand that leaves along its shaft while the other stays is away until it is back at its place
    for h in HANDS:
        o = "left" if h == "right" else "right"
        if h not in s.tips:
            continue
        if not s.away[h]:
            outward = NOMINAL_TROCAR[h] - s.tips[h]
            outward /= np.linalg.norm(outward)
            leaving = float(s.vel[h] @ outward) > 0.045 and float(np.linalg.norm(s.vel[o])) < 0.025 and not s.away[o]
            s.out_count[h] = s.out_count[h] + 1 if leaving else 0
            if s.out_count[h] >= 2:
                s.away[h] = True
                s.work[h] = s.history[h][-min(6, len(s.history[h]))].copy()   # where it was before it started to leave
                s.out_count[h] = 0
            else:
                if float(np.linalg.norm(s.vel[h])) < 0.03:
                    s.work[h] = s.tips[h].copy()
        else:
            back = h in s.work and float(np.linalg.norm(s.tips[h] - s.work[h])) < 0.012
            together = o in s.tips and float(np.linalg.norm(s.vel[o])) > 0.035 and float(np.linalg.norm(s.tips[h] - s.tips[o])) < 0.08
            fresh = h not in s.work                           # it was away from the first frame on
            if back or together or (fresh and float(np.linalg.norm(s.vel[h])) < 0.02 and o in s.tips
                                    and float(np.linalg.norm(s.tips[h] - s.tips[o])) < 0.07):
                s.away[h] = False
    s.q_hist = (s.q_hist + [q.copy()])[-4:]

    # 3. the lens
    if priv["lens_smeared"]:
        s.cleaning = True
    elif s.cleaning:
        s.cleaning = False
    if s.cleaning:
        rate = np.zeros(4)
        rate[2] = -model.RATE_MAX[2] if q[2] > model.CLEAN_INSERTION - 0.004 else 0.0
        rate[3] = _roll_rate(q, scope, head)
        return s, rate

    # 4. what to look at: each hand's tip, or the place it left while it is away. Fine work is told from the tips
    #    themselves (the published rule: every counted tip stayed within FINE_RADIUS of its own mean): then the aim is
    #    the tips' mean place; otherwise where they will be when the command has acted (the frame is one step old,
    #    the command takes another).
    fine = True
    for h in HANDS:
        if s.away[h] or h not in s.tips:
            continue
        hist = np.array(s.history[h])
        if len(hist) < FINE_SAMPLES or np.linalg.norm(hist - hist.mean(0), axis=1).max() > model.FINE_RADIUS:
            fine = False
    pts, vels = [], []
    for h in HANDS:
        if not s.away[h] and h in s.tips:
            if fine:
                pts.append(np.array(s.history[h]).mean(0))
                vels.append(np.zeros(3))
            else:
                pts.append(s.tips[h] + s.vel[h] * ((lat + 1.0) * model.DT))
                vels.append(s.vel[h])
        elif h in s.work:
            pts.append(s.work[h])
            vels.append(np.zeros(3))
    if not pts:
        return s, np.array([0.0, 0.0, 0.0, _roll_rate(q, scope, head)])
    pts = np.array(pts)
    centre, v_centre = pts.mean(0), np.mean(vels, axis=0)
    spread = float(np.linalg.norm(pts[0] - pts[-1])) if len(pts) > 1 else 0.0
    k = model.SCOPES[scope]
    want_depth = min(max(k["fx"] * model.SHAFT_DIAMETER / TARGET_PX, 0.022 + 1.22 * spread), k["fx"] * model.SHAFT_DIAMETER / 25.0)

    # 5. resolved-rate aiming with the model's Jacobian (pan, tilt, insertion; the roll follows the horizon)
    def view(qq, point):
        qq = np.array([qq[0], qq[1], qq[2], q[3]])
        return model.project(point[None, :], model.camera_pose(qq, scope, head), scope)[0]

    now = view(q[:3], centre)
    err = np.array([k["cx"] - now[0], k["cy"] - now[1], want_depth - now[2]])
    jac = np.zeros((3, 3))
    for i, eps in enumerate((1e-4, 1e-4, 1e-5)):
        dq = np.zeros(3)
        dq[i] = eps
        jac[:, i] = (view(q[:3] + dq, centre) - view(q[:3] - dq, centre)) / (2.0 * eps)
    drift = (view(q[:3], centre + v_centre * 0.05) - view(q[:3], centre - v_centre * 0.05)) / 0.1   # the work's own image motion
    # how the counted tips sit in the view right now (their latest positions, not the aim)
    live = np.array([s.tips[h] if (not s.away[h] and h in s.tips) else s.work[h] for h in HANDS if (h in s.tips and not s.away[h]) or h in s.work])
    tips_now = model.project(live, model.camera_pose(q, scope, head), scope)
    margin_u = model.SAFE_ZONE * model.IMAGE_W - np.abs(tips_now[:, 0] - k["cx"]).max()
    margin_v = model.SAFE_ZONE * model.IMAGE_H - np.abs(tips_now[:, 1] - k["cy"]).max()
    sizes = k["fx"] * model.SHAFT_DIAMETER / np.clip(tips_now[:, 2], 1e-3, None)
    band_lo, band_hi = sizes.min() - model.ZOOM_BAND_PX[0], model.ZOOM_BAND_PX[1] - sizes.max()
    usable = margin_u > 0.0 and margin_v > 0.0 and band_lo > 0.0 and band_hi > 0.0 and tips_now[:, 2].min() > 0.0
    comfortable = margin_u > 30.0 and margin_v > 24.0 and band_lo > 2.5 and band_hi > 3.5
    if fine and comfortable:
        s.calm = True
    elif not fine or margin_u < 12.0 or margin_v < 10.0 or band_lo < 1.0 or band_hi < 1.5:
        s.calm = False
    if s.calm:
        rate3 = np.zeros(3)
    else:
        gain = np.array([3.0, 3.0, 2.5])
        try:
            rate3 = np.linalg.solve(jac, gain * err - drift)
        except np.linalg.LinAlgError:
            rate3 = np.zeros(3)
    rate = np.concatenate([rate3, [_roll_rate(q, scope, head)]])
    rate = np.clip(rate, -model.RATE_MAX, model.RATE_MAX)
    if q[2] <= model.Q_MIN[2] + 1e-4 and rate[2] < 0:
        rate[2] = 0.0
    if fine and usable:
        # fine work in a usable view: whatever the view does, the working point's image must move slowly
        work = live.mean(0)
        a = model.project(work[None, :], model.camera_pose(q, scope, head), scope)[0]
        b = model.project(work[None, :], model.camera_pose(q + rate * model.DT, scope, head), scope)[0]
        px_s = math.hypot(b[0] - a[0], b[1] - a[1]) / model.DT
        if px_s > CALM_IMAGE_SPEED:
            rate = rate * (CALM_IMAGE_SPEED / px_s)
    return s, rate


def frame_joints(q_hist, q, lat):
    """The joint readings of the step the frame shows: `lat` steps before this one (the start, early on)."""
    if not q_hist:
        return q
    return q_hist[-lat] if len(q_hist) >= lat else q_hist[0]


def _roll_rate(q, scope, head):
    err = model.level_roll(q[0], q[1], scope, head) - q[3]
    return float(np.clip(4.0 * err, -model.RATE_MAX[3], model.RATE_MAX[3])) if abs(err) > math.radians(1.5) else 0.0


class Controller(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.before = {}              # t -> the state before step t
        self.after = []               # the states the requests of the latest step left, one per request
        self.state = State()

    def act(self, t, obs):
        if t not in self.before:
            self.state = self._continued(obs)
            self.before = {t: self.state}
            self.after = []
        q = np.asarray(obs["joints"], dtype=float)
        new, rate = advance(self.before[t], q, obs["privileged"])
        self.after.append(new)
        return [float(x) for x in rate]

    def _continued(self, obs):
        """Of the states the requests of the last step left, the one whose tips this step's frame continues best
        (the tips it held, moved on by their velocity, against the tips this frame shows)."""
        if not self.after:
            return self.state
        if len(self.after) == 1:
            return self.after[0]
        priv = obs["privileged"]
        items = {i["role"]: i for i in priv["instruments"]}
        costs = []
        for s in self.after:
            lat = int(priv["latency_steps"])
            r_wc, p = model.camera_pose(frame_joints(s.q_hist, None, lat), priv["scope"], math.radians(priv["camera_head_deg"]))
            cost = 0.0
            for h in HANDS:
                if h in s.tips and items[h]["present"]:
                    seen = p + r_wc @ np.asarray(items[h]["tip_cam"], dtype=float)
                    cost += float(np.linalg.norm(seen - (s.tips[h] + s.vel[h] * model.DT)))
            costs.append(cost)
        return self.after[int(np.argmin(costs))]


def main():
    ctl = Controller()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "reset":
            ctl.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            if "privileged" not in msg["obs"]:
                print(json.dumps({"action": [0.0, 0.0, 0.0, 0.0]}), flush=True)   # without its eyes it stands still
                continue
            print(json.dumps({"action": ctl.act(int(msg["t"]), msg["obs"])}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        else:
            print(json.dumps({"error": "unknown request"}), flush=True)


if __name__ == "__main__":
    main()
