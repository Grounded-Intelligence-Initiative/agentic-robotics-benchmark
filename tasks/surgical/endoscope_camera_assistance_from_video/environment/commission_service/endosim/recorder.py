"""The development set: recorded procedures as endoscope video with labels.

A recording is a development procedure (never a graded one) played with a scripted camera assistant at the holder,
`Assistant` below, standing in for the person who held the camera when the procedure was recorded. It is deliberately
ordinary: it reacts late, it lets the work drift off centre before it corrects, its horizon is only roughly level,
and in some recordings it runs the lens into tissue and has to withdraw and clean. The labels are the simulator's
truth for the frame they belong to (episode.Episode.annotation): every instrument's tip in the image, which
instruments count, whether the surgeon is doing fine work, whether the lens is smeared, and how the view rules judge
the frame.

Used by the commissioning service's set-up (environment/commission_service/record_dev.py); the grader never records.
"""

import json
import math
import os

import numpy as np

from . import model
from .episode import Episode
from .procedure import Rng


class Assistant(object):
    """The scripted camera assistant of the development recordings (reads the simulator's truth, as a person in the
    room sees the instruments; it is not the reference controller and not a target to imitate)."""

    def __init__(self, seed, episode, touch=False):
        rng = Rng(seed, "assistant-operator")
        self.ep = episode
        self.gain = rng.uniform(1.2, 2.2)
        self.lag = rng.integers(2, 6)                      # steps between seeing and moving
        self.dead_px = rng.uniform(55.0, 105.0)
        self.want_px = rng.uniform(27.0, 37.0)             # the apparent size it likes
        self.roll_band = math.radians(rng.uniform(4.0, 9.0))
        self.seen = []
        self.moving = False
        self.touch_at = rng.integers(int(0.25 * episode.steps), int(0.75 * episode.steps)) if touch else -1
        self.mode, self.wait = "follow", 0

    def act(self):
        ep, scope = self.ep, self.ep.scope
        q = ep.holder.q
        k = model.SCOPES[scope]
        roll_err = model.level_roll(q[0], q[1]) - q[3]
        roll = float(np.clip(1.5 * roll_err, -model.RATE_MAX[3], model.RATE_MAX[3])) if abs(roll_err) > self.roll_band else 0.0
        # the lens: an accident, some time to notice it, the withdrawal, the wait in the cannula
        if ep.t == self.touch_at and self.mode == "follow":
            self.mode = "dive"
        if self.mode == "dive":
            if ep.smeared:
                self.mode, self.wait = "unaware", int(Rng(ep.t, "notice").uniform(8, 22))
            elif q[2] >= model.Q_MAX[2] - 1e-3 or ep.t > self.touch_at + 60:
                self.mode = "follow"                         # nothing in the way this time
            return np.array([0.0, 0.0, model.RATE_MAX[2], roll])
        if ep.smeared and self.mode == "follow":
            self.mode, self.wait = "unaware", int(Rng(ep.t, "notice").uniform(8, 22))
        if self.mode == "unaware":
            self.wait -= 1
            if self.wait <= 0:
                self.mode = "clean"
        if self.mode == "clean":
            if not ep.smeared:
                self.mode = "follow"
            else:
                return np.array([0.0, 0.0, -model.RATE_MAX[2] if q[2] > model.CLEAN_INSERTION - 0.004 else 0.0, roll])
        tips = ep.active_tips()
        if not tips:
            return np.array([0.0, 0.0, 0.0, roll])
        self.seen = (self.seen + [np.mean(np.array(tips), axis=0)])[-(self.lag + 1):]
        target = self.seen[0]
        spread = float(np.linalg.norm(tips[0] - tips[-1])) if len(tips) > 1 else 0.0
        want_depth = max(k["fx"] * model.SHAFT_DIAMETER / self.want_px, 0.02 + 1.2 * spread)

        def view(qq):
            return model.project(target[None, :], model.camera_pose(np.array([qq[0], qq[1], qq[2], q[3]]), scope), scope)[0]

        now = view(q[:3])
        err = np.array([k["cx"] - now[0], k["cy"] - now[1], want_depth - now[2]])
        off = math.hypot(err[0], err[1])
        if self.moving and off < 0.35 * self.dead_px and abs(err[2]) < 0.006:
            self.moving = False
        elif not self.moving and (off > self.dead_px or abs(err[2]) > 0.016):
            self.moving = True
        if not self.moving:
            return np.array([0.0, 0.0, 0.0, roll])
        jac = np.zeros((3, 3))
        for i, eps in enumerate((1e-4, 1e-4, 1e-5)):
            d = np.zeros(3)
            d[i] = eps
            jac[:, i] = (view(q[:3] + d) - view(q[:3] - d)) / (2.0 * eps)
        try:
            rate3 = np.linalg.solve(jac, self.gain * err)
        except np.linalg.LinAlgError:
            rate3 = np.zeros(3)
        return np.clip(np.concatenate([rate3, [roll]]), -model.RATE_MAX, model.RATE_MAX)


def record(procedure, renderer, out_dir, name, every=2, operator_seed=0, touch=False, log=None):
    """Play `procedure` with the scripted assistant and write <out_dir>/<name>/: frames/<step>.jpg for every
    `every`-th step, labels.jsonl (one line per frame), meta.json. Returns the recording's summary."""
    root = os.path.join(out_dir, name)
    os.makedirs(os.path.join(root, "frames"), exist_ok=True)
    renderer.load(procedure.scene_spec())
    ep = Episode(procedure)
    op = Assistant(operator_seed, ep, touch=touch)
    serial = 0
    with open(os.path.join(root, "labels.jsonl"), "w") as labels:
        while True:
            if ep.t % every == 0:
                serial += 1
                jpeg = renderer.render(ep.render_state(serial))
                fname = "%06d.jpg" % ep.t
                with open(os.path.join(root, "frames", fname), "wb") as f:
                    f.write(jpeg)
                lab = ep.annotation()
                lab["frame"] = "frames/" + fname
                labels.write(json.dumps(lab) + "\n")
            if ep.done:
                break
            ep.step(model.clip_command(op.act()))
    k = model.SCOPES[procedure.scope]
    summary = ep.summary()
    meta = {
        "name": name, "scope": procedure.scope, "view_deg": k["view_deg"], "intrinsics": {"fx": k["fx"], "fy": k["fy"], "cx": k["cx"], "cy": k["cy"]},
        "image": [model.IMAGE_W, model.IMAGE_H], "step_s": model.DT, "steps": procedure.steps, "recorded_every": every,
        "procedure_kind": procedure.kind,
        "camera": "held by a scripted assistant; its view was usable on {:.0%} of the steps".format(summary["useful_view_fraction"]),
        "labels": "labels.jsonl: one JSON object per recorded frame, for the state the frame shows (holder_manual.md, section 8)",
    }
    with open(os.path.join(root, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    if log:
        log("recorded {}: {} frames, the assistant's useful_view_fraction {:.3f}, lens contacts {}".format(
            name, serial, summary["useful_view_fraction"], summary["lens_contacts"]))
    return summary
