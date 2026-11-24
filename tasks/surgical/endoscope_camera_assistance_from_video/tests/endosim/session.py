"""A procedure being run for a controller: the episode, the renderer and the video latency joined into observations.

The grader (tests/verify.py), the commissioning service and the development-set recorder all drive a `Session`, so the
controller meets the same observations everywhere:

    obs = {"image": <base64 JPEG, 640 x 480>, "joints": [pan, tilt, insertion, roll]}

The image is `frame_delay` control steps old (the video chain's latency); the joint readings are current. A session
opened with `privileged=True` (the grader does this for the reference submission only) adds `obs["privileged"]`: what a
perfect perception system would have read from that same delayed frame.
"""

import base64
import collections

from . import model
from .episode import Episode


class NullRenderer(object):
    """Stands in for the renderer where no frame is needed (maintainers' fast runs of a state-only controller)."""

    def load(self, spec):
        pass

    def render(self, state):
        return b""

    def variant(self, serial, shift):
        return b""

    def close(self):
        pass


class Session(object):
    def __init__(self, procedure, renderer, frame_delay=1, privileged=False):
        self.episode = Episode(procedure)
        self.renderer = renderer
        self.delay = int(frame_delay)
        self.with_privileged = bool(privileged)
        self._serial = 0
        self._frames = collections.deque(maxlen=self.delay + 1)   # (serial, jpeg, privileged) of the latest steps
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
        jpeg = self.renderer.render(ep.render_state(self._serial))
        self._frames.append((self._serial, jpeg, ep.privileged() if self.with_privileged else None))
        self._rendered_step = ep.t

    def observation(self):
        """The observation of the current step (rendering it if that has not happened yet)."""
        self._capture()
        _, jpeg, priv = self._frames[0]         # the oldest kept frame: `delay` steps old once the run is under way
        obs = {"image": base64.b64encode(jpeg).decode("ascii"), "joints": self.episode.joints()}
        if priv is not None:
            obs["privileged"] = priv
        return obs

    def shown_serial(self):
        """The renderer's serial of the frame the current observation shows."""
        self._capture()
        return self._frames[0][0]

    def probe_copy(self, obs, shift, joint_offset):
        """A perturbed copy of the current observation for the match lock: the same frame with its picture shifted by
        `shift` pixels inside the endoscope's disk, and the pan, tilt and roll readings offset."""
        jpeg = self.renderer.variant(self.shown_serial(), shift)
        out = dict(obs)
        out["image"] = base64.b64encode(jpeg).decode("ascii")
        j = list(obs["joints"])
        for i, d in zip((0, 1, 3), joint_offset):
            j[i] = round(j[i] + float(d), 5)
        out["joints"] = j
        if "privileged" in obs:
            # the reference reads its tips from here: shift them as the picture is shifted
            import copy
            p = copy.deepcopy(obs["privileged"])
            for item in p["instruments"]:
                item["tip_px"] = [item["tip_px"][0] + shift[0], item["tip_px"][1] + shift[1]]
                k = model.SCOPES[p["scope"]]
                d = max(item["depth"], 1e-6)
                item["tip_cam"] = [item["tip_cam"][0] + shift[0] * d / k["fx"], item["tip_cam"][1] - shift[1] * d / k["fy"],
                                   item["tip_cam"][2]]
            out["privileged"] = p
        return out

    def step(self, action):
        """Apply one action (anything `model.clip_command` accepts; ValueError otherwise)."""
        self._capture()                       # the frame of this step exists even if nobody asked for it
        return self.episode.step(model.clip_command(action))

    def finish_idle(self):
        self.episode.run_idle()
