"""A procedure being run for a controller: the episode, the renderer and the video latency joined into observations.

The grader (tests/verify.py), the commissioning service and the development-set recorder all drive a `Session`, so the
controller meets the same observations everywhere:

    obs = {"image": <base64 JPEG, 640 x 480>, "joints": [pan, tilt, insertion, roll]}

The image is `frame_delay` control steps old (the video chain's latency: the procedure's own, 1 to 3 steps, unless a
caller sets it); the joint readings are current. A session opened with `privileged=True` (the grader does this for the
reference submission only) adds `obs["privileged"]`: what a perfect perception system would have read from that same
delayed frame, and the latency.

`probe_copy` gives the match lock its second observation of a step, built the way an observation is built: the frame
the observation shows, rendered again at the same moment from the same camera pose (the joint readings are the same),
with each instrument where it would be had it moved over that frame's last step as it moved over another step of the
procedure. The holder's joints follow from the public holder model and the commands, so they are the same in both;
what differs is what nobody but the surgeon decides.
"""

import base64
import collections
import copy

import numpy as np

from . import model
from .episode import Episode, tip_view
from .procedure import HANDS, ROLES

PROBE_TRIES = 64              # draws of another step before a probe copy is given up for this step


class NullRenderer(object):
    """Stands in for the renderer where no frame is needed (maintainers' fast runs of a state-only controller)."""

    def load(self, spec):
        pass

    def render(self, state):
        return b""

    def close(self):
        pass


class Session(object):
    def __init__(self, procedure, renderer, frame_delay=None, privileged=False):
        self.episode = Episode(procedure)
        self.renderer = renderer
        self.delay = int(getattr(procedure, "latency", 1) if frame_delay is None else frame_delay)
        self.with_privileged = bool(privileged)
        self._serial = 0
        self._frames = collections.deque(maxlen=self.delay + 1)   # (serial, jpeg, privileged, state, step) per step
        self._rendered_step = -1
        renderer.load(procedure.scene_spec())

    @property
    def done(self):
        return self.episode.done

    @property
    def t(self):
        return self.episode.t

    def _capture(self):
        ep = self.episode
        if self._rendered_step == ep.t:
            return
        self._serial += 1
        state = ep.render_state(self._serial)
        jpeg = self.renderer.render(state)
        self._frames.append((self._serial, jpeg, ep.privileged() if self.with_privileged else None, state, ep.t))
        self._rendered_step = ep.t

    def observation(self):
        """The observation of the current step (rendering it if that has not happened yet)."""
        self._capture()
        _, jpeg, priv, _, _ = self._frames[0]   # the oldest kept frame: `delay` steps old once the run is under way
        obs = {"image": base64.b64encode(jpeg).decode("ascii"), "joints": self.episode.joints()}
        if priv is not None:
            obs["privileged"] = dict(priv, latency_steps=self.delay)
        return obs

    def shown_serial(self):
        """The renderer's serial of the frame the current observation shows."""
        self._capture()
        return self._frames[0][0]

    def probe_copy(self, obs, rng):
        """The match lock's copy of the current observation `obs`, or None when this step cannot have one.

        The frame `obs` shows is that of step f. For another step k of the procedure, drawn with `rng` among the steps
        with the same instruments in at k - 1 and k as at f - 1 and f, the same fine work and the same counted hands,
        every instrument that is in at f - 1 and f is put at its place of f - 1 plus its motion from k - 1 to k
        (a draw that would put a tip into the tissue is drawn again). The frame is rendered again with the
        instruments there and everything else as it was: the camera pose, the tissue's breathing, the smoke, the lens.
        The joint readings are those of `obs`; a privileged observation reads the moved tips from the same pose."""
        self._capture()
        _, _, priv, state, f = self._frames[0]
        proc = self.episode.proc
        if f < 1:
            return None
        moving = [r for r in ROLES if proc.present[r][f] and proc.present[r][f - 1]]
        if not moving:
            return None
        time_s = self.episode.time_s(f)
        tissue = proc.tissue
        tips = None
        for _ in range(PROBE_TRIES):
            k = 1 + int(rng.random() * proc.steps)
            if k == f or bool(proc.fine[k]) != bool(proc.fine[f]):
                continue
            if any(bool(proc.present[r][k]) != bool(proc.present[r][f]) or bool(proc.present[r][k - 1]) != bool(proc.present[r][f - 1])
                   for r in ROLES):
                continue
            if any(bool(proc.active[h][k]) != bool(proc.active[h][f]) for h in HANDS):
                continue
            cand = {r: np.asarray(proc.tip[r][f - 1], dtype=float) + (np.asarray(proc.tip[r][k], dtype=float) - np.asarray(proc.tip[r][k - 1], dtype=float))
                    for r in moving}
            if any(tissue.clearance(cand[r], time_s) < min(0.0, tissue.clearance(proc.tip[r][f], time_s)) for r in moving):
                continue
            tips = cand
            break
        if tips is None:
            return None
        scope = self.episode.scope
        pose = (np.array(state["cam"]["r"], dtype=float), np.array(state["cam"]["p"], dtype=float))
        moved = copy.deepcopy(state)
        for r in moving:
            moved["inst"][r]["tip"] = [float(v) for v in tips[r]]
        if "right" in tips:
            src = model.project(tips["right"][None, :], pose, scope)[0]
            moved["smoke_px"] = [float(src[0]), float(src[1])] if src[2] > 0 else None
        self._serial += 1
        moved["serial"] = int(self._serial)
        out = dict(obs)
        out["image"] = base64.b64encode(self.renderer.render(moved)).decode("ascii")
        if priv is not None:
            p = copy.deepcopy(obs.get("privileged", priv))
            for item in p["instruments"]:
                if item["role"] in tips:
                    item.update(tip_view(tips[item["role"]], pose, scope))
            out["privileged"] = p
        return out

    def step(self, action):
        """Apply one action (anything `model.clip_command` accepts; ValueError otherwise)."""
        self._capture()                       # the frame of this step exists even if nobody asked for it
        return self.episode.step(model.clip_command(action))

    def finish_idle(self):
        self.episode.run_idle()
