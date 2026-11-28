"""The pixel baseline: a controller that uses only what every submission gets, the frame and the joint readings.

Perception is the small net of net.py, trained on the development set (train.py): the left and right tips in the
picture and their apparent size, and whether the lens is smeared. From those it builds what the reference controller
is handed as `privileged` (each tip in the camera frame, from its pixel and the depth its apparent size implies), and
lets the reference's own control law (reference_controller.py, a copy of solution/reference/controller.py) do the
rest. Which scope is fitted it works out from the picture's motion: under the right scope, the tips it rebuilds in the
patient frame stay put while the camera moves; under the wrong one they drift.

It exists to show that the task can be done from the public interface, and how far a first learned detector gets
(tests/README.md). It is not the reference and it is not tuned.
"""

import base64
import copy
import json
import os
import sys

import cv2
import numpy as np
import torch

import holder_model as model
import reference_controller as ref
from net import TipNet, decode, prepare

HANDS = ("left", "right")
CONF = 0.45
SEARCH_POSE = np.array([0.0, 0.75, 0.07])      # where to look from when no instrument has been seen for a while


class Eyes(object):
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.net = TipNet()
        self.net.load_state_dict(torch.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.pt"), map_location="cpu"))
        self.net.to(self.device).eval()

    def __call__(self, obs):
        data = np.frombuffer(base64.b64decode(obs["image"]), dtype=np.uint8)
        frame = cv2.cvtColor(cv2.imdecode(data, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        with torch.no_grad():
            out, smear = self.net(prepare(frame).to(self.device))
        out = out[0].float().cpu()
        return [decode(out, k) for k in (0, 1)], float(torch.sigmoid(smear[0]))


class Memory(object):
    def __init__(self):
        self.control = ref.State()
        self.size = {}                       # hand -> smoothed apparent size
        self.last_world = {"0": {}, "30": {}}
        self.q_frame = None
        self.drift = {"0": 0.0, "30": 0.0}
        self.samples = 0
        self.scope = "0"
        self.smear_run = 0
        self.smeared = False
        self.unseen = 0


def step(mem, q, dets, smear_p):
    """(new memory, action) from the memory before this step, the joint readings and what the net saw."""
    m = copy.deepcopy(mem)
    q = np.asarray(q, dtype=float)
    q_frame = q if m.control.q_prev is None else m.control.q_prev        # the frame is one step old
    moved = m.q_frame is not None and (np.abs(q_frame[:2] - m.q_frame[:2]).max() > 0.006 or abs(q_frame[2] - m.q_frame[2]) > 0.0008)
    items, seen_any = [], False
    for k, hand in enumerate(HANDS):
        u, v, size, conf = dets[k]
        present = conf > CONF and 6.0 < size < 120.0
        if present:
            seen_any = True
            m.size[hand] = size if hand not in m.size else 0.65 * m.size[hand] + 0.35 * size
        else:
            m.size.pop(hand, None)
        tip_cam = [0.0, 0.0, -0.1]
        for scope in ("0", "30"):                                          # the tip in the patient frame under each scope
            kk = model.SCOPES[scope]
            if not present:
                m.last_world[scope].pop(hand, None)
                continue
            d = kk["fx"] * model.SHAFT_DIAMETER / m.size[hand]
            cam = np.array([(u - kk["cx"]) / kk["fx"] * d, -(v - kk["cy"]) / kk["fy"] * d, -d])
            r_wc, p = model.camera_pose(q_frame, scope)
            world = p + r_wc @ cam
            if moved and hand in m.last_world[scope]:
                m.drift[scope] += float(np.linalg.norm(world - m.last_world[scope][hand]))
                m.samples += 1
            m.last_world[scope][hand] = world
            if scope == m.scope:
                tip_cam = [float(x) for x in cam]
        items.append({"role": hand, "present": bool(present), "tip_cam": tip_cam})
    m.q_frame = q_frame.copy()
    if m.samples >= 40:
        lo, hi = sorted(m.drift.values())
        if hi > 1.12 * lo:
            m.scope = min(m.drift, key=m.drift.get)
    m.smear_run = m.smear_run + 1 if smear_p > 0.5 else 0
    if m.smear_run >= 2:
        m.smeared = True
    elif smear_p < 0.3:
        m.smeared = False
    m.unseen = 0 if seen_any else m.unseen + 1
    priv = {"scope": m.scope, "lens_smeared": m.smeared, "instruments": items}
    m.control, rate = ref.advance(m.control, q, priv)
    if m.unseen > 8 and not m.smeared:
        # nothing in sight: go and look from the overview pose
        want = np.concatenate([SEARCH_POSE, [model.level_roll(q[0], q[1])]])
        rate = np.clip(1.5 * (want - q), -model.RATE_MAX, model.RATE_MAX)
    return m, [float(x) for x in rate]


def continued(memories, dets):
    """Of the memories the requests of the last step left, the one whose tips this step's detections continue best
    (a repeated step's copy shows the instruments moved differently over the frame's last step)."""
    if len(memories) == 1:
        return memories[0]
    costs = []
    for m in memories:
        c, kk = m.control, model.SCOPES[m.scope]
        cost = 0.0
        if c.q_prev is not None:
            r_wc, p = model.camera_pose(c.q_prev, m.scope)                # this frame was taken at the last step's joints
            for k, hand in enumerate(HANDS):
                u, v, size, conf = dets[k]
                if hand in c.tips and conf > CONF and 6.0 < size < 120.0:
                    d = kk["fx"] * model.SHAFT_DIAMETER / size
                    cam = np.array([(u - kk["cx"]) / kk["fx"] * d, -(v - kk["cy"]) / kk["fy"] * d, -d])
                    cost += float(np.linalg.norm(p + r_wc @ cam - (c.tips[hand] + c.vel[hand] * model.DT)))
        costs.append(cost)
    return memories[int(np.argmin(costs))]


def main():
    eyes = Eyes()
    memory, before, after = Memory(), {}, []
    print("READY", flush=True)
    for line in sys.stdin:
        if not line.strip():
            continue
        msg = json.loads(line)
        if msg["type"] == "reset":
            memory, before, after = Memory(), {}, []
            print(json.dumps({"ok": True}), flush=True)
        elif msg["type"] == "act":
            t, obs = int(msg["t"]), msg["obs"]
            q = np.asarray(obs["joints"], dtype=float)
            dets, smear_p = eyes(obs)
            if t not in before:
                if after:
                    memory = continued(after, dets)
                before, after = {t: memory}, []
            new, action = step(before[t], q, dets, smear_p)
            after.append(new)
            print(json.dumps({"action": action}), flush=True)
        elif msg["type"] == "close":
            print(json.dumps({"ok": True}), flush=True)
            return


if __name__ == "__main__":
    main()
