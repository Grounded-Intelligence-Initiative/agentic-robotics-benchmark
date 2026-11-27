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

CRITERIA = ("lens", "zone", "zoom", "horizon", "motion", "takeover")
TAKEOVERS = ("lens_injury", "second_contact", "critical_unseen")
DISK_RADIUS_PX = 305.0        # a tip an annotator can mark lies inside the image disk (post.DISK_RADIUS 310, its edge)
SMOKE_HIDES = 0.6             # the smoke density above which the plume's core hides a tip in it ...
PLUME_HIDES_PX = 50.0         # ... within this distance of the plume's centre (PLUME_RISE_PX above the source)
PLUME_RISE_PX = 15.0          # post.PLUME_RISE_PX (post.py imports the renderer's stack; this file must not)
ANNOTATION_SD_PX = 1.5        # a hand-marked tip's error (px), per coordinate ...
ANNOTATION_MAX_PX = 4.0       # ... at most this


def tip_view(tip, pose, scope):
    """An instrument tip as a perfect perception system reads it from a frame taken from `pose`: in the image, its
    depth, and in the camera frame (the privileged observation's fields)."""
    tip = np.asarray(tip, dtype=float)
    uvd = model.project(tip[None, :], pose, scope)[0]
    pc = model.to_camera(tip[None, :], pose)[0]
    return {"tip_px": [float(uvd[0]), float(uvd[1])], "depth": float(uvd[2]), "tip_cam": [float(x) for x in pc]}


class Episode(object):
    def __init__(self, procedure, takeovers=True):
        self.proc = procedure
        self.takeovers = bool(takeovers)      # False for a development recording: nobody took the camera from its assistant
        self.scope = procedure.scope
        self.head_at = getattr(procedure, "head_at", None)      # the camera head's angle on the eyepiece per step (rad)
        if self.head_at is None:
            self.head_at = np.full(procedure.steps + 1, float(getattr(procedure, "head", 0.0)))
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
        self.takeover = None                  # why the surgeon took the camera back, once it happened (version 0.3)
        self.touches = 0                      # separate contacts of the lens with the tissue
        self._touching = False
        self.takeover_step = None
        self.critical = list(getattr(procedure, "critical", None) or [])   # [start, end, kind] of each critical action
        self.critical_seen = []               # per critical action judged: the share of its steps with a usable view
        self.trace = []                       # per judged step: (usable, failed criteria bitmask)
        self._pose = model.camera_pose(self.holder.q, self.scope, self.head)
        self._prev_pose = self._pose

    # -- state ----------------------------------------------------------------------------------------------------
    @property
    def head(self):
        """The camera head's angle at the current step (rad): it changes when the head is knocked (version 0.3)."""
        return float(self.head_at[min(self.t, len(self.head_at) - 1)])

    @property
    def steps(self):
        return self.proc.steps

    @property
    def done(self):
        return self.t >= self.proc.steps

    def time_s(self, t=None):
        return (self.t if t is None else t) * model.DT

    @property
    def taken_over(self):
        return self.takeover is not None

    def _take_over(self, why):
        if self.takeovers and self.takeover is None:
            self.takeover, self.takeover_step = why, self.t

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
        grips = self.proc.grips(self.t)
        for k in range(model.SUBSTEPS):
            q, v = self.holder.tick(command, h)
            now = self.time_s() + (k + 1) * h
            new_clear = tissue.clearance(model.lens_position(q), now, grips)
            touching = new_clear < model.LENS_CLEARANCE
            if touching and not self._touching:
                self.touches += 1                    # a new contact: the lens was clear of the tissue before
            self._touching = touching
            if touching:
                # the lens meets tissue: the scope stops where it was, and the lens is smeared. Driven into it (closing
                # faster than LENS_INJURY_SPEED), or touching it a second time, it ends the holder's procedure
                old_clear = tissue.clearance(model.lens_position(self.holder.q), now - h, grips)
                closing = (old_clear - new_clear) / h
                if closing > model.LENS_INJURY_SPEED and old_clear >= model.LENS_CLEARANCE:
                    self._take_over("lens_injury")
                elif self.touches > model.LENS_CONTACTS_TOLERATED:
                    self._take_over("second_contact")
                if tissue.clearance(model.lens_position(self.holder.q), now, grips) < model.LENS_CLEARANCE:
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
        self._pose = model.camera_pose(self.holder.q, self.scope, self.head)
        self._prev_pose = prev_pose
        return self._judge(prev_pose)

    def _judge(self, prev_pose):
        t, scope = self.t, self.scope
        tips = self.active_tips()
        fail = set()
        if self.taken_over:
            fail.add("takeover")
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
        if abs(model.horizon_error(self.holder.q, scope, self.head)) > math.radians(model.HORIZON_LIMIT_DEG):
            fail.add("horizon")
        ok = not fail
        self.judged += 1
        self.usable += int(ok)
        for c in fail:
            self.failed[c] += 1
        if ok and self.first_usable is None:
            self.first_usable = t
        self.trace.append(sorted(fail))
        # a critical action ends at this step: was it seen? (the trace holds step k at index k - 1)
        for start, end, kind in self.critical:
            if int(end) == t and not self.taken_over:
                seen = float(np.mean([not f for f in self.trace[int(start) - 1:int(end)]]))
                self.critical_seen.append(seen)
                if seen < model.CRITICAL_MIN_USABLE:
                    self._take_over("critical_unseen")
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
            "time_s": float(self.time_s()),
            "smoke": float(self.proc.smoke[t]), "smoke_px": [float(src[0]), float(src[1])] if src[2] > 0 else None,
            "smear": int(self.smear_serial) if self.smeared else 0,
        }

    def privileged(self):
        """What a perfect perception system would report at the current step: the scope fitted, the camera head's angle,
        whether the lens is smeared, and each instrument's tip in the camera frame and in the image. Not the future,
        not which instruments count, not whether the surgeon is doing fine work."""
        t = self.t
        items = []
        for r in ROLES:
            item = {"role": r, "present": bool(self.proc.present[r][t])}
            item.update(tip_view(self.proc.tip[r][t], self._pose, self.scope))
            items.append(item)
        return {"scope": self.scope, "camera_head_deg": math.degrees(self.head), "lens_smeared": bool(self.smeared), "instruments": items}

    def visible(self, role, t=None):
        """Whether an annotator sees the tip of `role` in the frame of step t: the instrument in, its tip in front of
        the lens and inside the image disk, above the moving tissue with nothing of the tissue between it and the lens,
        and not inside the core of a dense plume (density above SMOKE_HIDES, within PLUME_HIDES_PX of the plume's
        centre, which rises from the right-hand instrument's tip)."""
        t = self.t if t is None else t
        proc = self.proc
        if not proc.present[role][t]:
            return False
        tip = np.asarray(proc.tip[role][t], dtype=float)
        u, v, depth = model.project(tip[None, :], self._pose, self.scope)[0]
        if depth <= 0.002 or math.hypot(u - 0.5 * (model.IMAGE_W - 1), v - 0.5 * (model.IMAGE_H - 1)) > DISK_RADIUS_PX:
            return False
        time_s, grips = self.time_s(t), proc.grips(t)
        tissue = proc.tissue
        if tissue.clearance(tip, time_s, grips) < 0.0005:
            return False
        lens = self._pose[1]
        s = np.linspace(0.05, 1.0 - 0.0015 / max(float(np.linalg.norm(tip - lens)), 1e-6), 48)
        pts = lens[None, :] + s[:, None] * (tip - lens)[None, :]
        if np.any(pts[:, 2] < tissue.surface_z(pts[:, 0], pts[:, 1], time_s, grips)):
            return False
        if float(proc.smoke[t]) > SMOKE_HIDES:
            src = model.project(np.asarray(proc.tip["right"][t], dtype=float)[None, :], self._pose, self.scope)[0]
            if src[2] > 0 and math.hypot(u - src[0], v - (src[1] - PLUME_RISE_PX)) < PLUME_HIDES_PX:
                return False
        return True

    def annotation(self, rng):
        """A development recording's label for the current step (the dev set only; never on a graded wire), as an
        annotation team makes it (version 0.3): the tips it can see, marked by hand (`rng` draws the marking's error,
        ANNOTATION_SD_PX, at most ANNOTATION_MAX_PX), which instruments are in and count and their catalogue size, the
        holder's joints, the scope, the lab's camera head angle, a smeared lens. No depth, no judgement of the view."""
        t = self.t
        items = []
        for r in ROLES:
            present = bool(self.proc.present[r][t])
            item = {"role": r, "present": present, "active": bool(self.proc.active[r][t]),
                    "shaft_mm": int(self.proc.looks[int(self.proc.look[r][t])]["shaft_mm"]) if present else None,
                    "visible": False, "tip_px": None}
            if present and self.visible(r, t):
                u, v, _ = model.project(np.asarray(self.proc.tip[r][t], dtype=float)[None, :], self._pose, self.scope)[0]
                du, dv = [max(-ANNOTATION_MAX_PX, min(ANNOTATION_MAX_PX, ANNOTATION_SD_PX * rng.normal())) for _ in range(2)]
                item.update(visible=True, tip_px=[round(float(u + du), 1), round(float(v + dv), 1)])
            items.append(item)
        return {"step": int(t), "joints": self.joints(), "scope": self.scope, "camera_head_deg": round(math.degrees(self.head), 3),
                "lens_smeared": bool(self.smeared), "instruments": items}

    def summary(self):
        """The procedure's record. A procedure the surgeon took the camera back in counts no usable step at all (version
        0.3: a holder that had to be taken over did not assist that procedure); `usable_before_takeover` keeps the
        steps that were usable before it, for the record only."""
        n = max(1, self.judged)
        usable = 0 if self.taken_over else int(self.usable)
        out = {"steps": int(self.judged), "usable_steps": usable, "useful_view_fraction": usable / n,
               "usable_before_takeover": int(self.usable) if self.taken_over else None,
               "lens_contacts": int(self.contacts), "lens_touches": int(self.touches), "lens_smears": int(self.smear_serial),
               "lens_cleanings": int(self.cleanings),
               "first_usable_step": self.first_usable, "takeover": self.takeover, "takeover_step": self.takeover_step,
               "critical_actions": len(self.critical_seen),
               "critical_min_seen": round(min(self.critical_seen), 4) if self.critical_seen else None}
        out.update({"failed_" + c: self.failed[c] / n for c in CRITERIA})
        return out
