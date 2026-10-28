"""Reference supervisor (privileged expert) for driving_safety_supervisor.

Submitted with the oracle token, so every observation carries the grader's ground-truth
block (`obs["privileged"]`): every road user and prop near the car with its true position,
velocity and size, and the route's lane geometry ahead, all in the car's frame (x forward,
y right, metres, from the car's centre). It does not look at the camera.

Each 0.1 s step it:

* projects every actor onto the route (station along it, offset across it) and predicts
  moving ones at constant velocity over the next few seconds;
* finds the nearest point on the path it is about to drive where something is, or will be
  by the time the car gets there, inside the car's corridor, and caps the speed so the car
  can stop short of it with a comfortable deceleration;
* when its own lane is blocked by something that stays put (props, a car that has stood
  still away from junctions and lights, a pedestrian walking slowly in the lane) and the
  lane to the left is clear for the whole manoeuvre, drives around it on a path offset into
  that lane and back; when the left lane is closed or busy it waits;
* otherwise lets the planner's control through.

Wire: stdio JSON lines, READY first; each `act` gets `{"action": [0]}` or
`{"action": [1, throttle, steer, brake]}`. A repeated `t` (the probe lock) is answered from
the state the first ask of that `t` started from.
"""

import copy
import json
import math
import os
import sys

DT = 0.1
HALF_L, HALF_W = 2.45, 1.05      # the ego car (Lincoln MKZ), from its centre
WHEELBASE = 2.9
MAX_STEER = 1.22                 # rad at steer = 1
DECEL = 4.0                      # m/s^2 for stopping envelopes
LEAD_DECEL = 30.0                 # a lead car may brake this hard, without warning
BRAKE_PER_MS2 = 1.0 / 8.5        # brake command per m/s^2 of deceleration
# Below about 5.5 m/s any brake command locks this car's wheels (well over 7 m/s^2, a harsh
# brake); coasting slows it by roughly 3-6 m/s^2 there, so gentle stops coast.
LOCK_SPEED, COAST_DECEL = 5.5, 4.5
GAP = {"vehicle": 3.5, "prop": 3.0, "walker": 4.5}     # bumper gap kept when stopped
HEADWAY_S = 1.5                  # extra following gap per m/s of the car's speed
GAP_STATIC = 9.0                 # ... behind something that stays put: room to pull out
CREEP_AFTER_S = 8.0              # waited this long and still no way round: close up
SIDE = {"vehicle": 0.35, "prop": 0.45, "walker": 0.9}  # lateral clearance
PROP_MIN = 0.6                   # some props' boxes are far smaller than their meshes
HORIZON = 4.0                    # s of constant-velocity prediction
V_PASS = 7.0                     # m/s while driving around a blocker
V_CRUISE = 12.0
V_NEAR_WALKER = 7.0
RAMP = 12.0                      # m to move into (or back from) the passing lane
RAMP_MIN, RAMP_PER_MS = 6.5, 1.8   # shortest move out: at a standstill, and per m/s
BEYOND_CENTRE = 0.9              # m past the passing lane's centre the car may go
CLEAR = {"vehicle": 0.3, "prop": 0.3, "walker": 0.7}   # extra room when passing
TIGHT = 0.25                     # side room for a slow, tight pass of props or a parked car
V_TIGHT = 4.0
RAMP_SHORT = 4.0                 # a move out from a standstill close behind the blocker
RAMP_BACK_MIN = 4.5              # shortest way back in, before a junction
LAT_ACCEL = 3.0                  # m/s^2 in curves when the expert sets the speed
NUDGE_SIDE = 0.2                 # room left to an oncoming car over the centre line
V_NUDGE = 5.0
STEER_TRUST = 0.35               # planner steer this far off the route's is not followed
STILL_CAR_S = 2.5                # a stopped car is passed only after waiting behind it this long
DEBUG = bool(os.environ.get("SUP_DEBUG"))


def log(*parts):
    sys.stderr.write("[expert] " + " ".join(str(p) for p in parts) + "\n")
    sys.stderr.flush()


class Path(object):
    """The route polyline in the car's frame, with stations and per-point lane data."""

    def __init__(self, route):
        start = 0
        for i, p in enumerate(route):
            if p["x"] < 0.0:
                start = i
        pts = route[start:]
        if len(pts) < 2:
            pts = [{"x": -1.0, "y": 0.0, "width": 3.5, "junction": False, "left": None},
                   {"x": 50.0, "y": 0.0, "width": 3.5, "junction": False, "left": None}]
        self.pts = pts
        self.s = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.s.append(self.s[-1] + math.hypot(b["x"] - a["x"], b["y"] - a["y"]))
        self.s0, self.d0 = self.project(0.0, 0.0)
        self._kerbs = {}

    def project(self, x, y):
        """(station, offset to the right) of a point; extrapolated past either end."""
        best = (float("inf"), 0.0, 0.0)
        last = len(self.pts) - 2
        for i in range(last + 1):
            a, b = self.pts[i], self.pts[i + 1]
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            seg = math.hypot(dx, dy)
            if seg < 1e-6:
                continue
            ux, uy = dx / seg, dy / seg
            u = (x - a["x"]) * ux + (y - a["y"]) * uy
            uc = min(seg, max(0.0, u))
            d2 = (x - a["x"] - ux * uc) ** 2 + (y - a["y"] - uy * uc) ** 2
            if d2 < best[0]:
                off = -(x - a["x"]) * uy + (y - a["y"]) * ux
                extra = u - uc if (i == 0 and u < 0) or (i == last and u > seg) else 0.0
                best = (d2, self.s[i] + uc + extra, off)
        return best[1], best[2]

    def _seg(self, station):
        for i in range(len(self.s) - 1):
            if self.s[i + 1] >= station:
                return i
        return len(self.s) - 2

    def heading(self, station):
        i = self._seg(station)
        a, b = self.pts[i], self.pts[i + 1]
        return math.atan2(b["y"] - a["y"], b["x"] - a["x"])

    def point(self, station, offset=0.0):
        i = self._seg(station)
        a, b = self.pts[i], self.pts[i + 1]
        u = (station - self.s[i]) / max(1e-6, self.s[i + 1] - self.s[i])
        x, y = a["x"] + (b["x"] - a["x"]) * u, a["y"] + (b["y"] - a["y"]) * u
        h = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        return x - math.sin(h) * offset, y + math.cos(h) * offset

    def lane(self, station):
        return self.pts[self._seg(station)]

    def left_offset(self, station, k=0):
        """Offset (negative: to the left) of the centre of the k-th driving lane to the left,
        or None."""
        lefts = self.lane(station).get("lefts")
        if lefts is None:
            lefts = [self.lane(station)["left"]] if self.lane(station).get("left") else []
        if k >= len(lefts):
            return None
        return self.project(lefts[k]["x"], lefts[k]["y"])[1]

    def right_offset(self, station, k=0):
        """Offset (positive: to the right) of the centre of the k-th lane to the right
        (driving, shoulder, parking or bike lane), or None."""
        rights = self.lane(station).get("rights") or []
        if k >= len(rights):
            return None
        return self.project(rights[k]["x"], rights[k]["y"])[1]

    def side_offset(self, station, k, side):
        return self.left_offset(station, k) if side < 0 else self.right_offset(station, k)

    def kerbs(self, station):
        """(left edge, right edge) of the drivable road at a station, as offsets."""
        key = int(station)
        if key not in self._kerbs:
            p = self.lane(station)
            half = p.get("width", 3.5) / 2.0
            left, right = -half, half
            lefts, rights = p.get("lefts") or [], p.get("rights") or []
            if lefts:
                left = self.project(lefts[-1]["x"], lefts[-1]["y"])[1] - lefts[-1]["width"] / 2.0
            if rights:
                right = self.project(rights[-1]["x"], rights[-1]["y"])[1] + rights[-1]["width"] / 2.0
            self._kerbs[key] = (left, right)
        return self._kerbs[key]

    def junction_from(self, a, horizon=60.0):
        """Station of the first junction point at or after `a`, or None."""
        s = a
        while s <= a + horizon and s <= self.s[-1]:
            if self.lane(s).get("junction"):
                return s
            s += 2.0
        return None

    def junction_between(self, a, b):
        s = a
        while s <= b:
            if self.lane(s).get("junction"):
                return True
            s += 2.0
        return False


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class Expert(object):
    def __init__(self):
        self.reset()

    def reset(self):
        # pass plans are kept in odometer metres, since the path is rebuilt every step
        self.mem = {"still": {}, "pass": None, "odo": 0.0, "behind": 0.0, "recover": False,
                    "gap_static": GAP_STATIC, "nudge": 0.0}
        self.s0, self.red, self.ahead, self.path = 0.0, False, 0, None
        self.last_t, self.before = None, None

    def act(self, t, obs):
        if t == self.last_t and self.before is not None:
            self.mem = copy.deepcopy(self.before)
        else:
            self.before = copy.deepcopy(self.mem)
            self.last_t = t
        priv = obs.get("privileged")
        if not priv:
            return [0]
        return self._decide(obs, priv)

    # -- perception from ground truth ----------------------------------------------
    def _actors(self, path, priv):
        out = []
        for a in priv["actors"]:
            if a["type"] == "prop":
                if a.get("height", 1.0) < 0.15:
                    continue
                a = dict(a, length=max(a["length"], PROP_MIN), width=max(a["width"], PROP_MIN))
            # the box's four corners on the route: its true span along and across it
            yaw = math.radians(a["yaw"])
            cy, sy = math.cos(yaw), math.sin(yaw)
            spans = [path.project(a["x"] + cy * dl - sy * dw, a["y"] + sy * dl + cy * dw)
                     for dl in (-a["length"] / 2, a["length"] / 2)
                     for dw in (-a["width"] / 2, a["width"] / 2)]
            s_lo, s_hi = min(p[0] for p in spans), max(p[0] for p in spans)
            d_lo, d_hi = min(p[1] for p in spans), max(p[1] for p in spans)
            s, d = (s_lo + s_hi) / 2.0, (d_lo + d_hi) / 2.0
            h = path.heading(max(0.0, s))
            vs = a["vx"] * math.cos(h) + a["vy"] * math.sin(h)
            vd = -a["vx"] * math.sin(h) + a["vy"] * math.cos(h)
            speed = math.hypot(a["vx"], a["vy"])
            still = self.mem["still"].get(a["id"], 0.0)
            still = still + DT if speed < 0.3 else 0.0
            self.mem["still"][a["id"]] = still
            ha, hl = (d_hi - d_lo) / 2.0, (s_hi - s_lo) / 2.0
            out.append(dict(a, s=s, d=d, vs=vs, vd=vd, speed=speed, still=still, ha=ha, hl=hl))
        return out

    # -- the planned lateral offset (passing) ------------------------------------------
    def _update_nudge(self, path, actors):
        """Move toward the kerb, within the lane, for an oncoming car that comes over the
        centre line into the car's lane; drift back once it has gone by."""
        lane_w = path.lane(path.s0).get("width", 3.5)
        room = lane_w / 2.0 - HALF_W - 0.05
        want = 0.0
        for a in actors:
            if a["type"] != "vehicle" or a["vs"] > -1.0 or a["d"] > 0.0:
                continue
            if a["s"] + a["hl"] < path.s0 - HALF_L or a["s"] - path.s0 > 60.0:
                continue
            want = max(want, a["d"] + a["ha"] + HALF_W + NUDGE_SIDE + 0.05)
        want = min(max(0.0, want), room)
        now = self.mem["nudge"]
        self.mem["nudge"] = now + max(-0.1, min(0.15, want - now))
        if self.mem["nudge"] < 0.02:
            self.mem["nudge"] = 0.0

    def _odo(self, station):
        return station - self.s0 + self.mem["odo"]

    def _offset_at(self, station):
        p = self.mem["pass"]
        if p is None:
            return self.mem["nudge"]
        station = self._odo(station)
        if station <= p["a"] or station >= p["e"]:
            return 0.0
        if station < p["b"]:
            return p["off"] * (station - p["a"]) / (p["b"] - p["a"])
        if station <= p["c"]:
            return p["off"]
        return p["off"] * (p["e"] - station) / (p["e"] - p["c"])

    def _in_corridor(self, a, s, d, pad=0.0):
        p = self.mem["pass"]
        side = TIGHT if p is not None and p.get("tight") and a["id"] in p["ids"] else SIDE[a["type"]]
        if self.mem["nudge"] > 0.0 and a["type"] == "vehicle" and a["vs"] < -1.0:
            side = NUDGE_SIDE
        if a["type"] == "walker" and self.path is not None:
            left, right = self.path.kerbs(max(0.0, s))
            if d - a["ha"] > right or d + a["ha"] < left:
                side = SIDE["vehicle"]       # on the pavement, beyond the kerb
        return abs(d - self._offset_at(s)) < HALF_W + a["ha"] + side + pad

    def _limit(self, path, actors, v):
        """Speed cap from everything on, or coming onto, the path ahead, with the actor
        that sets it and the distance left to stop in (its stopping room counted)."""
        v_cap, worst, room = V_CRUISE, None, 100.0
        s0 = path.s0
        for a in actors:
            gap0 = GAP[a["type"]]
            if a["speed"] < 0.3 and a["type"] != "walker" and a["id"] != -1 and not (
                    a["type"] == "vehicle" and self.mem["behind"] >= CREEP_AFTER_S):
                gap0 = self.mem["gap_static"]
            elif a["type"] == "vehicle" and a["speed"] < 3.0 and self.mem["behind"] < CREEP_AFTER_S:
                # slowing to a stop: open up to the standing gap in good time
                gap0 += (self.mem["gap_static"] - gap0) * (1.0 - a["speed"] / 3.0)
            if a["s"] > s0 and self._in_corridor(a, a["s"], a["d"]):
                # only a vehicle is trusted to keep moving; a sliding prop or a person may stop
                lead = max(0.0, a["vs"]) if a["type"] == "vehicle" else 0.0
                if a["type"] == "vehicle" and lead > 0.3:
                    gap0 += HEADWAY_S * v
                if a["vs"] < -0.3 and a["type"] == "vehicle":
                    gap0 += 4.0 + 2.0 * -a["vs"]        # backing (or driving) toward the car
                gap = a["s"] - a["hl"] - s0 - HALF_L - gap0 + lead * lead / (2 * LEAD_DECEL)
                cap = math.sqrt(2 * DECEL * gap) if gap > 0 else 0.0
                if cap < v_cap:
                    v_cap, worst, room = cap, a, gap
                continue
            if a["speed"] < 0.4:
                continue
            if a["s"] < s0 - HALF_L and a["vs"] > 0.0:
                continue                 # behind, going the same way: not ahead of the car
            if a["type"] == "vehicle" and abs(a["vd"]) < 0.5:
                continue                 # driving along its lane: a straight-line guess would cut bends
            if a["type"] == "walker" and abs(a["vd"]) < 0.4 and abs(a["vs"]) > 0.5:
                continue                 # walking along the road, swaying: not crossing
            tau = 0.25
            while tau <= HORIZON:
                s, d = path.project(a["x"] + a["vx"] * tau, a["y"] + a["vy"] * tau)
                closing = abs(d - self._offset_at(s)) < abs(a["d"] - self._offset_at(a["s"])) - 0.05
                if s > s0 + HALF_L and closing and self._in_corridor(a, s, d, pad=0.3):
                    reach = (s - a["hl"] - s0 - HALF_L) / max(v, 0.5)
                    clear = (s + a["hl"] - s0 + HALF_L) / max(v, 0.5)
                    if tau < clear + 1.0 and reach < tau + 2.0:
                        gap = s - a["hl"] - s0 - HALF_L - gap0
                        cap = math.sqrt(2 * DECEL * gap) if gap > 0 else 0.0
                        if cap < v_cap:
                            v_cap, worst, room = cap, a, gap
                    break
                tau += 0.25
        return v_cap, worst, room

    # -- passing -------------------------------------------------------------------
    def _blocker(self, path, actors, s_from):
        """The nearest group of things that stay put in the car's own lane ahead:
        (start station, end station, members)."""
        members = []
        for a in actors:
            if a["s"] <= s_from or a["s"] > path.s0 + 45.0 or a["id"] == -1:
                continue
            if abs(a["d"]) > HALF_W + a["ha"] + SIDE[a["type"]] + 0.1:   # all the corridor stops for
                continue
            if a["type"] == "walker":
                if a["vs"] > 2.5 or abs(a["vd"]) > 0.8:
                    continue
            elif a["speed"] > 0.3 and not self._backing(a):
                continue
            members.append(a)
        if not members:
            return None
        members.sort(key=lambda a: a["s"])
        start, end = members[0]["s"] - members[0]["hl"], members[0]["s"] + members[0]["hl"]
        group = [members[0]]
        for a in members[1:]:
            if a["s"] - a["hl"] > end + 10.0:
                break
            end = max(end, a["s"] + a["hl"])
            group.append(a)
        return start, end, group

    @staticmethod
    def _backing(a):
        """A vehicle in the lane coming back toward the car (reversing): get out of its way."""
        return a["type"] == "vehicle" and a["vs"] < -0.3 and abs(a["vd"]) < 0.5

    def _left_clear(self, path, actors, a_s, e_s, off, v, skip):
        """Nothing in the passing lane over [a_s, e_s] for as long as the manoeuvre takes."""
        duration = (e_s - path.s0) / max(V_PASS * 0.8, v, 1.0) + 2.0
        s0 = path.s0
        for act in actors:
            if act["id"] in skip:
                continue
            if act["vs"] > max(3.0, v) and act["s"] - act["hl"] > s0 + HALF_L + 3.0:
                continue                 # ahead and pulling away
            if act["type"] == "vehicle" and act["vs"] > 0.5 and act["s"] < s0 and \
                    s0 - act["s"] > act["vs"] * 3.0 + 8.0:
                continue                 # coming up behind, but far enough back
            s_then = act["s"] + act["vs"] * duration
            lo = min(act["s"], s_then) - act["hl"]
            hi = max(act["s"], s_then) + act["hl"]
            if hi < a_s - 6.0 or lo > e_s + 6.0:
                continue
            d_then = act["d"] + act["vd"] * min(duration, 2.0)
            for d in (act["d"], d_then):
                if off - HALF_W - act["ha"] - SIDE[act["type"]] < d < off + HALF_W + act["ha"] + SIDE[act["type"]]:
                    return False, act
        return True, None

    def _plan_pass(self, path, actors, blocker, v, red_light):
        start, end, group = blocker
        s0 = path.s0
        kinds = set(a["type"] for a in group)
        if start - s0 - HALF_L > 40.0:
            return None
        if kinds == {"vehicle"}:
            if self.mem["behind"] < STILL_CAR_S and not any(self._backing(a) for a in group):
                return None
            # a car queued at a light or a junction is not passed; one parked askew or off
            # the lane centre is
            parked = any(abs(a["d"]) > 0.5 or abs(_wrap_deg(a["yaw"] - math.degrees(
                path.heading(max(0.0, a["s"]))))) > 15.0 for a in group)
            backing = any(self._backing(a) for a in group)
            if not parked and not backing and (red_light or path.junction_between(start - 5.0, end + 8.0)):
                if DEBUG:
                    log("no pass: stopped car at a light or junction")
                return None
        # the left first (lane by lane, against the traffic if need be), then the right:
        # a shoulder or a parking lane when the left is closed
        for side in (-1, 1):
            k = 0
            while True:
                offs = [path.side_offset(s, k, side) for s in (start - 10.0, start, end, end + 6.0)]
                if any(o is None for o in offs):
                    if DEBUG:
                        log("no pass: no lane %d on the %s" % (k, "left" if side < 0 else "right"))
                    break
                plan, who = self._try_lane(path, actors, group, kinds, start, end, v, k, offs, side)
                if plan is not None:
                    return plan
                if who is not None and (who["speed"] > 0.3 or who["type"] == "walker"):
                    return None              # traffic or a person there: wait for it
                if who is None:
                    break                    # no room on this side as things stand
                # something that stays put in that lane too: look one lane further
                k += 1
                further = [path.side_offset(s, k, side) for s in (start, end)]
                if all(o is not None for o in further):
                    # stop far enough back to swing out that far
                    self.mem["gap_static"] = max(self.mem["gap_static"], GAP_STATIC * max(
                        1.0, max(abs(o) for o in further) / 3.5))
        return None

    def _try_lane(self, path, actors, group, kinds, start, end, v, k, offs, side):
        """A pass into the k-th lane on one side (-1 left, +1 right), or (None, what is in
        the way). Worked out as if to the left (offsets negative), then mirrored."""
        s0 = path.s0
        offs = [-side * o for o in offs]
        need = -side * self._need(group, side=side)
        # on the right that lane is often a shoulder against a kerb or a rail: keep clear of
        # its outer edge
        limit = max(offs) - (BEYOND_CENTRE if side < 0 else 0.3)
        if side > 0:
            outer = min(path.kerbs(s)[1] for s in (start - 10.0, start, end, end + 6.0))
            limit = max(limit, -(outer - HALF_W - 0.8))
        tight = False
        settled = all(a["still"] >= 3.0 for a in group)    # nothing still sliding or rolling
        if k == 0 and need < limit and kinds == {"prop"} and settled and \
                -side * self._need(group, True, side) >= limit:
            tight, need = True, -side * self._need(group, True, side)   # slowly, with less room
        off = max(limit, min(-0.5, need))
        if k > 0:
            off = max(limit, min(need, max(offs)))          # that lane's centre, at least
        if off > need + 1e-6:
            if DEBUG:
                log("no pass: needs %.2f, %s lane %d allows %.2f" % (
                    need, "left" if side < 0 else "right", k, limit))
            return None, None
        off, limit = -side * off, -side * limit
        scale = max(0.45, abs(off) / 3.5)   # the ramp scales with how far the car moves over
        margin = 6.0 if "walker" in kinds else 4.0
        # fully out before the front bumper reaches the first thing passed; the move out
        # needs more room the faster the car goes: too close, keep slowing and look again
        b_s = start - HALF_L - 1.0
        a_s = b_s - scale * max(RAMP, RAMP_MIN, RAMP_PER_MS * v)
        if a_s < s0 + 0.3:
            a_s = b_s - scale * max(RAMP_MIN, RAMP_PER_MS * v)
        if a_s < s0 + 0.3 and v < 0.5 and self.mem["behind"] >= 2.0:
            # stopped close behind it: a short, slow move out
            b_s = start - HALF_L - 0.3
            a_s = s0 + 0.3
            tight = tight or b_s - a_s < scale * RAMP_MIN
            if b_s - a_s < scale * RAMP_SHORT:
                a_s = b_s + 1.0      # flagged below
        if any(self._backing(a) for a in group):
            # it is coming back at the car: out at once, on the shortest move
            a_s = s0 + 0.3
            b_s = a_s + scale * max(RAMP_SHORT + 1.0, 1.2 * v)
        if a_s < s0 + 0.3 or a_s > b_s:
            if DEBUG:
                log("no pass: too close (%.1f m to go at %.1f m/s)" % (b_s - s0, v))
            return None, None
        c_s = end + margin
        e_s = c_s + scale * RAMP
        junction = path.junction_from(start)
        if junction is not None:
            e_s = min(e_s, junction - 1.0)
            if e_s - c_s < scale * RAMP_BACK_MIN:
                if DEBUG:
                    log("no pass: junction %.1f m after the blocker" % (junction - end))
                return None, None
        ids = [a["id"] for a in group]
        clear, who = self._left_clear(path, actors, a_s, e_s, off, v, set(ids))
        if not clear:
            if DEBUG:
                log("no pass: %s#%s in %s lane %d (v %.1f, d %.1f)" % (
                    who["type"], who["id"], "left" if side < 0 else "right", k, who["speed"], who["d"]))
            return None, who
        log("pass", sorted(kinds), "in %.1f m, %s lane %d, offset %.2f%s" % (
            start - s0 - HALF_L, "left" if side < 0 else "right", k, off, ", tight" if tight else ""))
        return {"a": self._odo(a_s), "b": self._odo(b_s), "c": self._odo(c_s),
                "e": self._odo(e_s), "off": off, "limit": limit, "ids": ids, "margin": margin,
                "tight": tight, "ramp": scale * RAMP, "side": side}, None

    @staticmethod
    def _need(group, tight=False, side=-1):
        """Offset that clears every member on one side (negative: left) with room to spare
        (or, tight, with just enough)."""
        room = lambda a: HALF_W + (TIGHT + 0.05 if tight else SIDE[a["type"]] + CLEAR[a["type"]])  # noqa: E731
        if side < 0:
            return min(a["d"] - a["ha"] - room(a) for a in group)
        return max(a["d"] + a["ha"] + room(a) for a in group)

    def _track_pass(self, path, actors):
        """Follow the blocker while it stays put; if it drives off (or disappears), drop
        the plan before pulling out, or head back to the lane at once after."""
        p = self.mem["pass"]
        odo = self.mem["odo"]
        if p["ids"]:
            here = [a for a in actors if a["id"] in p["ids"]]
            if here and all(a["speed"] < 1.0 or a["type"] == "walker" or self._backing(a)
                            for a in here):
                start = min(a["s"] - a["hl"] for a in here)
                end = max(a["s"] + a["hl"] for a in here)
                more = self._blocker(path, actors, end - 0.1)
                if more is not None and more[0] < end + 10.0:
                    p["ids"] += [a["id"] for a in more[2] if a["id"] not in p["ids"]]
                    here += [a for a in more[2] if a not in here]
                    end = max(end, more[1])
                if p.get("side", -1) < 0:
                    p["off"] = min(p["off"], max(p["limit"], self._need(here, p.get("tight"))))
                else:
                    p["off"] = max(p["off"], min(p["limit"], self._need(here, p.get("tight"), 1)))
                # the move out is never pushed later once it has begun; the way back
                # follows the end of the blocker
                b = self._odo(start - HALF_L - 1.0)
                if any(self._backing(a) for a in here):
                    b = p["b"]
                if odo < p["a"]:
                    p["a"] = max(odo, min(p["a"], b - RAMP_MIN))
                    p["b"] = min(p["b"], b)
                else:
                    p["b"] = min(p["b"], max(b, odo + 3.0))
                p["b"] = max(p["b"], p["a"] + 2.0)
                p["c"] = max(p["b"], self._odo(end + p["margin"]))
                p["e"] = p["c"] + p.get("ramp", RAMP)
            elif odo < p["a"]:
                self.mem["pass"] = None
                return
            else:
                # it drove off (or went) while the car is pulling out: hold the offset
                p["ids"] = []
                p["hold"] = True
        if p.get("hold"):
            if self._lane_clear_back(path, actors):
                p["hold"] = False
                p["c"] = max(p["b"], odo)
                p["e"] = p["c"] + p.get("ramp", RAMP)
            else:
                p["c"] = max(p["c"], odo + 5.0)
                p["e"] = p["c"] + p.get("ramp", RAMP)
        if odo > p["e"]:
            self.mem["pass"] = None

    def _lane_clear_back(self, path, actors):
        """Room to move back into the car's own lane: nothing there from just behind the
        car to well past where the move ends, counting a lead car's stopping room."""
        s0 = path.s0
        for a in actors:
            if abs(a["d"]) > HALF_W + a["ha"] + SIDE[a["type"]]:
                continue
            ahead = a["s"] - a["hl"] - s0 - HALF_L
            behind = s0 - HALF_L - (a["s"] + a["hl"])
            if behind > 4.0:
                continue
            if ahead < RAMP + 8.0 + max(0.0, -a["vs"]) * 3.0:
                return False
        return True

    # -- decision --------------------------------------------------------------------
    def _decide(self, obs, priv):
        v = float(obs["speed"])
        nominal = obs["nominal"]
        path = self.path = Path(priv["route"])
        s0 = self.s0 = path.s0
        self.red = bool(priv.get("red_light"))
        self.mem["odo"] += v * DT
        actors = self._actors(path, priv)

        self._update_nudge(path, actors)
        blocker = self._blocker(path, actors, s0)
        near = blocker is not None and blocker[0] - s0 - HALF_L < 15.0
        self.mem["behind"] = self.mem["behind"] + DT if near and v < 0.5 else 0.0
        if self.mem["pass"] is not None:
            self._track_pass(path, actors)
        if self.mem["pass"] is None and blocker is not None:
            self.mem["pass"] = self._plan_pass(path, actors, blocker, v, priv.get("red_light"))

        v_cap, worst, room = self._limit(path, actors, v)
        self.ahead = sum(1 for a in actors if 0.0 < a["s"] - s0 < 25.0 and
                         self._in_corridor(a, a["s"], a["d"]))
        passing = self.mem["pass"] is not None
        cruise = V_PASS if passing else V_CRUISE
        if passing and self.mem["pass"].get("tight"):
            cruise = V_TIGHT
        if self.mem["nudge"] > 0.0:
            cruise = min(cruise, V_NUDGE)
        for a in actors:
            if a["type"] == "walker" and a["id"] != -1 and 0 < a["s"] - s0 < 30 and abs(a["d"]) < 4.5:
                cruise = min(cruise, V_NEAR_WALKER)
        target = min(cruise, v_cap)
        if DEBUG:
            log("t=%s v=%.2f cap=%.2f target=%.2f room=%.1f worst=%s pass=%s nom=%s" % (
                obs.get("t"), v, v_cap, target, room,
                None if worst is None else "%s#%s s=%.1f d=%.1f v=%.1f" % (
                    worst["type"], worst["id"], worst["s"] - s0, worst["d"], worst["speed"]),
                None if not passing else "%.1f/%.1f/%.1f/%.1f off %.2f" % tuple(
                    [self.mem["pass"][k] - self.mem["odo"] for k in "abce"] + [self.mem["pass"]["off"]]),
                nominal) + " d0=%.2f red=%d" % (path.d0, self.red))
        # back in the lane before the planner gets the wheel again
        # at a red light the planner's own braking stops at the line (while the expert
        # steers, the light is still the planner's call)
        planner_stop = self.red
        if passing or self.mem["nudge"] > 0.0:
            self.mem["recover"] = False
        elif abs(path.d0) > 0.8 and not path.junction_between(s0 - 4.0, s0 + 8.0):
            self.mem["recover"] = True
        elif abs(path.d0) < 0.3:
            self.mem["recover"] = False
        own = passing or self.mem["recover"] or self.mem["nudge"] > 0.0   # steering and speed
        steer_only = not own and abs(nominal[1] - self._steer(path, v)) > STEER_TRUST
        if not own and not steer_only and target >= v + 0.5 and (worst is None or room > 20.0):
            return [0]
        if self.mem["recover"]:
            target = min(target, V_PASS)
        if own and planner_stop:
            return [1, 0.0, self._steer(path, v), max(nominal[2], 0.3)]
        if own:
            target = min(target, self._curve_speed(path))
        action = self._drive(path, v, target, room, nominal, own=own)
        if steer_only:
            action[2] = self._steer(path, v)
        return action

    def _curve_speed(self, path):
        """Speed that keeps the lateral acceleration in the coming bends below LAT_ACCEL."""
        s0, best = path.s0, V_CRUISE
        for ahead in (4.0, 8.0, 12.0, 16.0, 20.0):
            h1, h2 = path.heading(s0 + ahead), path.heading(s0 + ahead + 4.0)
            turn = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
            if turn > 1e-3:
                radius = 4.0 / turn
                best = min(best, math.sqrt(LAT_ACCEL * radius + 2 * DECEL * max(0.0, ahead - 4.0)))
        return best

    def _steer(self, path, v):
        """Pure pursuit on the route, shifted by the planned offset."""
        look = max(5.0, 0.9 * v + 3.0)
        s = path.s0 + look
        x, y = path.point(s, self._offset_at(s))
        delta = math.atan2(2.0 * WHEELBASE * y, max(x * x + y * y, 1.0))
        return max(-1.0, min(1.0, delta / MAX_STEER))

    def _drive(self, path, v, target, room, nominal, own):
        """Steer along the planned offset (own) or with the planner; brake with the
        deceleration the stop needs, otherwise cap the planner's throttle (or, when
        driving around a blocker, use its own)."""
        steer = self._steer(path, v) if own else nominal[1]
        if target < 0.3 and v < 0.3:
            # hold: the more there is in the way just ahead, the firmer
            return [1, 0.0, steer, min(1.0, 0.5 + 0.1 * self.ahead)]
        if v > target + 0.3:
            need = (v * v - target * target) / (2.0 * max(room, 0.5))
            need = max(need, 0.8 * (v - target))
            if v < LOCK_SPEED and need < COAST_DECEL and room > 2.5:
                return [1, 0.0, steer, 0.0]                     # coast
            brake = min(1.0, max(0.05, need * BRAKE_PER_MS2))
            return [1, 0.0, steer, brake]
        throttle = max(0.0, min(0.85, 0.35 + 0.025 * target + 0.3 * (target - v)))
        if own:
            return [1, throttle, steer, 0.0]
        if room < 40.0 and not self.red:
            # following something on the path: the expert's own envelope sets the braking,
            # not the planner's late, hard stop
            return [1, min(throttle, nominal[0]), steer, 0.0]
        return [1, min(throttle, nominal[0]), steer, nominal[2]]


def main():
    sup = Expert()
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
                action = [float(a) for a in sup.act(int(msg.get("t", 0)), msg["obs"])]
                if not all(math.isfinite(a) for a in action) or action[0] == 0:
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
