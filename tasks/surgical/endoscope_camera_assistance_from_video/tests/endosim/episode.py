"""One run of a procedure with the camera holder under rate commands: the holder's motion, the lens, the view rules.

The episode owns the truth. Step t = 0 is the start (the holder at home, withdrawn); an action moves the holder over
one control step, and the view at the step it leads to is judged from the simulator's state alone, never from pixels:

    usable(t) = lens clean  and  every active tip inside the central safe zone  and  every active instrument's
                apparent size inside the zoom band  and  |horizon error| within its limit  and
                (not fine work at t, or the camera moved the working point's image no faster than the limit)

`useful_view_fraction` = usable steps / steps (steps 1 .. steps). There is no other term: a lens that touches tissue is
smeared, which makes every step unusable until the scope has been held withdrawn in the cannula for the cleaning time.

Pure numpy; no renderer here (session.py joins the two).
"""

import math

import numpy as np

from . import model
from .procedure import HANDS, ROLES

CRITERIA = ("lens", "zone", "zoom", "horizon", "motion")


class Episode(object):
    def __init__(self, procedure):
        self.proc = procedure
        self.scope = procedure.scope
        self.holder = model.Holder()
        self.t = 0
        self.smeared = False
        self.clean_s = 0.0
        self.contacts = 0
        self.cleanings = 0
        self.usable = 0
        self.judged = 0
        self.failed = {c: 0 for c in CRITERIA}
        self.first_usable = None
        self.smear_serial = 0
        self.trace = []                       # per judged step: (usable, failed criteria bitmask)
        self._pose = model.camera_pose(self.holder.q, self.scope)
        self._prev_pose = self._pose

    # -- state ----------------------------------------------------------------------------------------------------
    @property
    def steps(self):
        return self.proc.steps

    @property
    def done(self):
        return self.t >= self.proc.steps

    def time_s(self, t=None):
        return (self.t if t is None else t) * model.DT

    def joints(self):
        return [round(float(v), 5) for v in self.holder.q]

    def pose(self):
        return self._pose

    def active_tips(self, t=None):
        t = self.t if t is None else t
        return [self.proc.tip[h][t] for h in HANDS if self.proc.active[h][t]]

    # -- one control step ------------------------------------------------------------------------------------------
    def step(self, action):
        """Apply a rate command over one control step, then judge the view at the step reached. `action` is a
        clipped command (model.clip_command); returns whether the new step's view is usable."""
        if self.done:
            raise RuntimeError("the procedure is over")
        command = np.asarray(action, dtype=float)
        h = model.DT / model.SUBSTEPS
        prev_pose = self._pose
        tissue = self.proc.tissue
        for k in range(model.SUBSTEPS):
            q, v = self.holder.tick(command, h)
            now = self.time_s() + (k + 1) * h
            if tissue.clearance(model.lens_position(q), now) < model.LENS_CLEARANCE:
                # the lens meets tissue: the scope stops where it was, and the lens is smeared
                if tissue.clearance(model.lens_position(self.holder.q), now) < model.LENS_CLEARANCE:
                    # the breathing tissue rose onto a lens held too close: back the scope off along its own axis
                    q = self.holder.q.copy()
                    q[2] = max(model.Q_MIN[2], q[2] - 0.002)
                    self.holder.q = q
                self.holder.v = np.zeros(4)
                if not self.smeared:
                    self.smear_serial += 1
                self.smeared = True
                self.contacts += 1
                self.clean_s = 0.0
                continue
            self.holder.q, self.holder.v = q, v
        self.t += 1
        if self.smeared:
            if self.holder.q[2] <= model.CLEAN_INSERTION:
                self.clean_s += model.DT
                if self.clean_s >= model.CLEAN_TIME_S - 1e-9:
                    self.smeared, self.clean_s = False, 0.0
                    self.cleanings += 1
            else:
                self.clean_s = 0.0
        self._pose = model.camera_pose(self.holder.q, self.scope)
        self._prev_pose = prev_pose
        return self._judge(prev_pose)

    def _judge(self, prev_pose):
        t, scope = self.t, self.scope
        tips = self.active_tips()
        fail = set()
        if self.smeared:
            fail.add("lens")
        if tips:
            uvd = model.project(np.array(tips), self._pose, scope)
            for u, v, depth in uvd:
                if depth <= 0.0 or not model.in_safe_zone(u, v, scope):
                    fail.add("zone")
                size = model.apparent_diameter(depth, scope) if depth > 0.0 else float("inf")
                if not model.ZOOM_BAND_PX[0] <= size <= model.ZOOM_BAND_PX[1]:
                    fail.add("zoom")
            if self.proc.fine[t]:
                work = np.mean(np.array(tips), axis=0)[None, :]
                a = model.project(work, prev_pose, scope)[0]
                b = model.project(work, self._pose, scope)[0]
                speed = math.hypot(b[0] - a[0], b[1] - a[1]) / model.DT if a[2] > 0 and b[2] > 0 else float("inf")
                if speed > model.IMAGE_SPEED_LIMIT:
                    fail.add("motion")
        else:
            fail.add("zone")
        if abs(model.horizon_error(self.holder.q, scope)) > math.radians(model.HORIZON_LIMIT_DEG):
            fail.add("horizon")
        ok = not fail
        self.judged += 1
        self.usable += int(ok)
        for c in fail:
            self.failed[c] += 1
        if ok and self.first_usable is None:
            self.first_usable = t
        self.trace.append(sorted(fail))
        return ok

    def run_idle(self):
        """The rest of the procedure with the holder commanded to stand still (an aborted run's honest cost)."""
        zero = np.zeros(4)
        while not self.done:
            self.step(zero)

    # -- what the renderer and the reference get --------------------------------------------------------------------
    def render_state(self, serial):
        """Everything the renderer needs for the frame of the current step (no future, no flags)."""
        t = self.t
        r_wc, p = self._pose
        tip = self.proc.tip["right"][t]
        src = model.project(tip[None, :], self._pose, self.scope)[0]
        # how far the camera's own motion moved the picture during the last control step (for the motion blur): the
        # image displacement of a point 8 cm in front of the lens
        ahead = (p + r_wc @ np.array([0.0, 0.0, -0.08]))[None, :]
        was = model.project(ahead, self._prev_pose, self.scope)[0]
        k = model.SCOPES[self.scope]
        flow = [float(k["cx"] - was[0]), float(k["cy"] - was[1])] if was[2] > 0 else [0.0, 0.0]
        return {
            "flow_px": flow,
            "serial": int(serial), "step": int(t),
            "cam": {"r": [[float(x) for x in row] for row in r_wc], "p": [float(x) for x in p]},
            "inst": self.proc.roles_state(t),
            "breath": float(self.proc.tissue.breathing(self.time_s())),
            "smoke": float(self.proc.smoke[t]), "smoke_px": [float(src[0]), float(src[1])] if src[2] > 0 else None,
            "smear": int(self.smear_serial) if self.smeared else 0,
        }

    def privileged(self):
        """What a perfect perception system would report at the current step: the scope fitted, whether the lens is
        smeared, and each instrument's tip in the camera frame and in the image. Not the future, not which
        instruments count, not whether the surgeon is doing fine work."""
        t = self.t
        items = []
        for r in ROLES:
            tip = self.proc.tip[r][t]
            uvd = model.project(tip[None, :], self._pose, self.scope)[0]
            pc = model.to_camera(tip[None, :], self._pose)[0]
            items.append({"role": r, "present": bool(self.proc.present[r][t]), "tip_px": [float(uvd[0]), float(uvd[1])],
                          "depth": float(uvd[2]), "tip_cam": [float(x) for x in pc]})
        return {"scope": self.scope, "lens_smeared": bool(self.smeared), "instruments": items}

    def annotation(self):
        """A development recording's label for the current step (the dev set only; never on a graded wire)."""
        t = self.t
        out = self.privileged()
        for item in out["instruments"]:
            r = item["role"]
            u, v = item["tip_px"]
            item["in_image"] = bool(item["present"] and item["depth"] > 0 and 0 <= u < model.IMAGE_W and 0 <= v < model.IMAGE_H)
            item["apparent_px"] = float(model.apparent_diameter(item["depth"], self.scope)) if item["depth"] > 0 else None
            item["active"] = bool(self.proc.active[r][t])
            del item["tip_cam"]
        out.update(step=int(t), joints=self.joints(), fine_work=bool(self.proc.fine[t]),
                   horizon_error_deg=math.degrees(model.horizon_error(self.holder.q, self.scope)),
                   usable=None if t == 0 else not self.trace[-1], failed=[] if t == 0 else list(self.trace[-1]))
        return out

    def summary(self):
        n = max(1, self.judged)
        out = {"steps": int(self.judged), "usable_steps": int(self.usable), "useful_view_fraction": self.usable / n,
               "lens_contacts": int(self.contacts), "lens_cleanings": int(self.cleanings),
               "first_usable_step": self.first_usable}
        out.update({"failed_" + c: self.failed[c] / n for c in CRITERIA})
        return out
