"""Scripted image-only reference for the delivery bay. Not an LLM result.

It uses only what the agent gets: the status of every reply (compass heading, the active delivery point's
coordinates, the counters), the start pose and the camera frames of `robot_observe`, and the camera model and the bay
description of the instruction. No ground truth, no token. It is the evidence that those observations suffice.

What it does:
- the remote: the two turns show on the compass; forward is the walking button that brings a wall nearer in the
  frame; the forward step is the change of the range to that wall over a run of presses; after the loading stop the
  compass gives the factor by which every gait has slowed;
- perception: in every image column the row where the lit floor ends (at a wall's foot, or at the edge of the
  light) and the row of the wall's top edge, turned into a range with the camera model: a 90 degree fan of ranges per
  frame; the delivery pad from its green foot print;
- position: dead reckoning with the compass and the learned step, corrected by the fence sides and the partition
  faces in view (the partitions' x is estimated together with the robot's), by the pad, and by how far the wall
  feet moved between two frames, which also shows a press that a wall stopped;
- map: an occupancy grid in which the newest frame decides; the fence is drawn in from the instruction;
- driving: travel distance to the delivery point over the map (unseen cells of a partition band are passable, at a
  price), a short search over press sequences, a new plan after every press; a frame only where a wall could be
  ahead that no frame has covered, before and inside an opening, near walls, and every sixth step otherwise.
"""
import base64
import io
import math

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

import nav

# --- from the instruction ---------------------------------------------------------------------------------------------
SIZE = 256
FOCAL = 128.              # pixels: 90 degrees across 256 px
CAMERA_HEIGHT = .25       # m above the floor, on the robot's vertical axis, looking along the heading
LIGHT = 1.0               # m: the floodlight's reach along the floor
WALL_TOP = .724           # m: the height of the fence and partition panels with their cap
PAD_TOP = .04             # m: the delivery pad is a 0.4 m square, 4 cm high
WALL_HALF = .12           # m: half the thickness of a partition
BAND = (.9, 1.5)          # m: a partition's centre line is at |x| in this range
LOADING_STOP = 3
# --- the reference's own settings -----------------------------------------------------------------------------------
MARGIN = .04              # planning clearance beyond the footprint (m)
MAX_BLIND = 6             # at most this many steps forward between two looks
UNKNOWN_BAND_COST = 5.    # price of crossing a not yet seen cell of a partition band, in cell lengths
HIT, MISS, CLAMP = .9, .45, 4.    # map evidence: a cell counts as known after one wall foot or two clear sight lines


def decode(obs):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][0]))).convert("RGB"))


def classify(im):
    """Per-pixel masks: lit floor, wall panel, delivery pad (green), dark."""
    r, g, b = (im[:, :, i].astype(np.int16) for i in range(3))
    lit = np.maximum(np.maximum(r, g), b) > 20
    pad = lit & (g-np.maximum(r, b) > 30)
    fringe = ndimage.binary_dilation(pad, structure=np.ones((3, 3), bool))     # blended pixels around the green
    wall = lit & ~fringe & (b-r > -4)
    floor = lit & ~fringe & ~wall
    return floor, wall, pad, ~lit


def scan(im):
    """A fan of ranges from one image: for every column, where the floor ends.

    Returns (forward, lateral, kind), arrays over the 256 columns. `forward` is the distance along the heading (m) of
    the point where the lit floor ends in that column and `lateral` its offset to the left. `kind` is 1 where the
    floor ends at a wall's foot, 2 where a wall is so near that its foot is below the image (forward is then an upper
    bound, 0.25 m), 0 where the floor ends at the edge of the light (nothing within reach) and -1 where the column
    says nothing.

    Two readings of a wall's distance are combined: the row of its foot (where floor colour gives way to panel
    colour) and the row of its top edge against the dark (a wall is 0.724 m tall). A metal post in front of a panel
    spoils the first, a lit building wall behind the fence the second; where they disagree the usable one is taken.
    """
    floor, wall, pad, dark = classify(im)
    columns = np.arange(SIZE)
    half = SIZE//2
    # --- the foot: ground pixels counted from the bottom row -----------------------------------------------------
    ground = (floor | pad | ndimage.binary_dilation(pad, structure=np.ones((3, 3), bool)))[half:][::-1]
    stop = np.where(ground.all(axis=0), half, np.argmin(ground, axis=0))       # rows 255 .. 128, bottom first
    first = np.clip(SIZE-1-stop, 2, SIZE-1)              # the first row, from the bottom, that is not ground
    at_wall = wall[first, columns] & wall[first-1, columns] & wall[first-2, columns]
    at_dark = dark[first, columns] & dark[first-2, columns]
    seen = (stop > 0) & (stop < half)
    boundary = (first+1).astype(float)                   # continuous row coordinate of the ground's upper edge
    foot = CAMERA_HEIGHT*FOCAL/np.maximum(boundary-half, .5)
    # --- the top edge: the first panel pixels from the top, with dark above them --------------------------------
    panel = wall[:half-2]
    panel = panel & np.roll(panel, -1, axis=0) & np.roll(panel, -2, axis=0)    # three panel pixels in a row
    panel[-2:] = False
    has_top = panel.any(axis=0)
    top_row = np.where(has_top, np.argmax(panel, axis=0), 0)
    lit_above = (~dark[:half-2]).cumsum(axis=0)[np.maximum(top_row-1, 0), columns] > 0
    top_ok = has_top & (top_row >= 2) & ~(lit_above & (top_row > 0))
    top = (WALL_TOP-CAMERA_HEIGHT)*FOCAL/np.maximum(half-top_row, .5)
    # --- combine ---------------------------------------------------------------------------------------------------
    forward = np.full(SIZE, np.inf)
    kind = np.full(SIZE, -1)
    free = seen & at_dark & ~has_top
    kind[free] = 0
    forward[free] = foot[free]
    by_foot = seen & at_wall & (foot < LIGHT+.3)
    agree = by_foot & top_ok & (np.abs(foot-top) < .12*top+.03)
    kind[agree] = 1
    forward[agree] = (foot[agree]+top[agree])/2
    only_foot = by_foot & ~top_ok                        # the top is out of the image, or something lit is behind
    kind[only_foot] = 1
    forward[only_foot] = foot[only_foot]
    only_top = top_ok & ~agree & ~only_foot & (top < LIGHT+.1)    # a post, or the pad, hides the foot
    kind[only_top] = 1
    forward[only_top] = top[only_top]
    near = wall[SIZE-1] & wall[SIZE-3] & ~top_ok         # a wall whose foot is below the image
    kind[near] = 2
    forward[near] = CAMERA_HEIGHT*FOCAL/half
    lateral = forward*(half-(columns+.5))/FOCAL
    return forward, lateral, kind


def pad_fix(im):
    """(forward, lateral, whole) of the delivery pad's centre from its foot print in the image, or None.

    Pixels of the pad's top face (4 cm above the floor) project onto the floor plane 0.25/0.21 too far; the pad's
    near side reaches down to the floor. So the pad spans, along the heading, from the nearest floor-plane distance
    to 0.84 of the farthest. `whole` is False when the image border or the edge of the light cuts the pad."""
    _, _, pad, _ = classify(im)
    rows, cols = np.nonzero(pad[SIZE//2+3:])
    if len(rows) < 25:
        return None
    rows = rows+SIZE//2+3
    plane = CAMERA_HEIGHT*FOCAL/(rows+1-SIZE/2)          # floor-plane distance of each pixel's lower edge
    scale = (CAMERA_HEIGHT-PAD_TOP)/CAMERA_HEIGHT
    far_row = rows.min()
    far = CAMERA_HEIGHT*FOCAL/(far_row-SIZE/2)*scale     # the far edge of the top face
    nearest = plane.min()
    forward = (nearest+far)/2
    top = rows < far_row+max(3, (rows.max()-far_row)//3)
    lateral_all = (SIZE/2-(cols[top]+.5))/FOCAL*(CAMERA_HEIGHT*FOCAL/(rows[top]+.5-SIZE/2))*scale
    lateral = (lateral_all.min()+lateral_all.max())/2
    whole = bool(cols.min() > 1 and cols.max() < SIZE-2 and rows.max() < SIZE-2
                 and math.hypot(far, lateral) < LIGHT-.08 and (far-nearest) > .25)
    return float(forward), float(lateral), whole


class Reference:
    def __init__(self, client):
        self.client = client
        self.X, self.Y = nav.centres()
        self.outside = (np.abs(self.X) > nav.FENCE) | (np.abs(self.Y) > nav.FENCE)
        self.band = ((np.abs(self.X) > BAND[0]-WALL_HALF) & (np.abs(self.X) < BAND[1]+WALL_HALF) & ~self.outside)
        self.logodds = np.zeros((nav.N, nav.N))
        self.version, self.cache = 0, None
        self.x = self.y = self.heading = 0.
        self.P = np.diag([.02**2, .17**2, .17**2])       # variances of [robot x, west partition x, east partition x]
        self.partition = np.array([-(BAND[0]+BAND[1])/2, (BAND[0]+BAND[1])/2])
        self.var_y = .02**2
        self.button = {}
        self.steady = {"F": .25, "B": .15, "L": math.radians(17), "R": math.radians(17)}
        self.first = dict(self.steady)
        self.samples = {k: 0 for k in ("L1", "Ls", "R1", "Rs")}       # turn presses seen, first of a run / later
        self.sums = {k: 0. for k in self.samples}                      # and the angle they turned in all (rad)
        self.last = None                 # the gait of the previous press
        self.since = []                  # presses since the last look
        self.ahead = None                # range straight ahead at the last look (m), None if nothing in reach
        self.feet = np.zeros((0, 2))     # wall feet seen at the last look, relative to the robot, along the bay's axes
        self.fresh = False               # True while no press has followed the last look
        self.obs = None
        self.spin = 0
        self.detours = []
        self.blocked_count = 0
        self.looks = 0
        self.blind = 0                   # steps forward since the last look
        self.why = {}                    # what the looks were taken for (a count per reason)
        self.fixed = []                  # the position fixes of the last look (for a trace)
        self.pad_looked = False          # the one look for the pad on the way in has been taken (per delivery)
        self.travel = {"x": None, "y": None}    # dead-reckoned travel along each axis since its last absolute fix
        self.rejected = {"x": [], "y": []}   # innovations of absolute fixes the gate refused, in a row

    # --- bookkeeping -----------------------------------------------------------------------------------------------
    def _status(self, reply):
        self.obs = reply
        self.heading = math.radians(reply["heading_deg"])
        return reply

    def goal(self):
        return (self.obs["goal"]["x"], self.obs["goal"]["y"])

    # --- perception ------------------------------------------------------------------------------------------------
    def look(self, why=""):
        self.why[why] = self.why.get(why, 0)+1
        if self.obs is not None and self.obs.get("frames_remaining", 1) <= 0:
            self.fresh, self.blind = True, 0             # no frames left: drive on by dead reckoning
            return None
        return self._process(self._status(self.client.observe()))

    def _process(self, reply):
        im = decode(reply)
        self.looks += 1
        forward, lateral, kind = scan(im)
        ahead = self._ahead(forward, kind)
        # Wall feet that face the robot's heading (not those of a wall it runs along): their place along the
        # heading is what a step forward changes. Kept relative to the robot, along the bay's axes.
        w = 4
        cols = np.arange(w, SIZE-w)
        solid = (kind[cols] == 1) & (kind[cols-w] == 1) & (kind[cols+w] == 1)
        with np.errstate(invalid="ignore"):              # columns with nothing in reach hold inf
            tf, tl = forward[cols+w]-forward[cols-w], lateral[cols+w]-lateral[cols-w]
            length = np.hypot(tf, tl)
        facing = solid & (length < .25) & (np.abs(tf) < .7*np.maximum(length, 1e-9))
        use = cols[facing]
        c, s = math.cos(self.heading), math.sin(self.heading)
        feet = np.stack([c*forward[use]-s*lateral[use], s*forward[use]+c*lateral[use]], axis=1)
        self._odometry(ahead, feet)
        self.ahead, self.feet, self.since, self.fresh, self.blind = ahead, feet, [], True, 0
        self.fixed = self._fixes(forward, lateral, kind, im)
        self._map(forward, lateral, kind)
        return im

    @staticmethod
    def _ahead(forward, kind):
        """The range straight ahead (m) from the middle columns of a fan, None if nothing is in reach there."""
        centre = kind[SIZE//2-6:SIZE//2+6]
        ranges = forward[SIZE//2-6:SIZE//2+6]
        if (centre == 1).sum() >= 8:
            return float(np.median(ranges[centre == 1]))
        if (centre == 2).sum() >= 8:
            return .2
        return None

    def _odometry(self, ahead, feet):
        """How far the walking presses since the last look really took the robot, from the two looks.

        Along a straight run the wall feet seen before must reappear shifted back by the distance walked. That
        distance is searched for (the shift under which most of the old feet fall on new ones); where the feet
        say nothing about it (no wall in view twice, or only a wall parallel to the run) the change of the range
        straight ahead is used, if there is one. The measured distance replaces the dead-reckoned one, teaches the
        forward step, and shows a press that a wall stopped."""
        walks = [p for p in self.since if p["gait"] in ("F", "B")]
        if not walks or len(walks) != len(self.since):
            return
        if len({p["gait"] for p in walks}) != 1 or abs(nav.wrap(walks[0]["heading"]-self.heading)) > math.radians(4):
            return
        expected = sum(p["d"] for p in walks)
        measured = None
        c, s = math.cos(self.heading), math.sin(self.heading)
        if len(self.feet) >= 12 and len(feet) >= 12:
            tree = cKDTree(feet)
            sign = 1. if expected >= 0 else -1.
            candidates = sign*np.arange(-.08, max(abs(expected)*1.4, abs(expected)+.15)+.001, .02)
            scores = []
            for d in candidates:
                moved = self.feet-d*np.array([c, s])
                along = moved[:, 0]*c+moved[:, 1]*s
                across = -moved[:, 0]*s+moved[:, 1]*c
                keep = (along > .3) & (np.abs(across) < .9*along) & (np.hypot(along, across) < LIGHT-.08)
                if keep.sum() < 10:
                    scores.append(-1.)
                    continue
                gap, _ = tree.query(moved[keep])
                scores.append(float((gap < .035).mean()))
            scores = np.array(scores)
            usable = scores >= 0
            if usable.sum() >= 5 and scores.max() >= .6 and scores[usable].min() < .6*scores.max():
                top = np.nonzero(scores >= scores.max()-.03)[0]
                measured = float(candidates[top].mean())
        if measured is None and self.ahead is not None and ahead is not None:
            measured = self.ahead-ahead
            low, high = sorted((-.08*(1 if expected >= 0 else -1), expected*1.3+.1*(1 if expected >= 0 else -1)))
            if not low <= measured <= high:
                measured = None          # another surface came into the centre of the view
        if measured is None:
            return
        self.x += (measured-expected)*c
        self.y += (measured-expected)*s
        if walks[0]["gait"] == "F" and expected > .1:
            ratio = measured/expected
            if ratio > .6:
                scale = min(1.12, max(.89, ratio))**.5
                for store in (self.steady, self.first):
                    store["F"] *= scale
                self.blocked_count = 0
            elif ratio < .3:
                self.blocked_count += 1                  # a wall stopped the robot

    def _gate(self, axis, innovation, gate, absolute):
        """True if a fix is to be used. An absolute fix (a fence side, the pad) that is refused three times in a
        row with the same error is believed after all: the dead reckoning has drifted, not the fence."""
        if abs(innovation) <= gate:
            if absolute:
                self.rejected[axis] = []
            return True
        if absolute:
            seen = self.rejected[axis] = (self.rejected[axis]+[innovation])[-3:]
            if len(seen) == 3 and max(seen)-min(seen) < .25:
                self.rejected[axis] = []
                if axis == "x":
                    self.P[0, 0] = max(self.P[0, 0], .5**2)
                else:
                    self.var_y = max(self.var_y, .5**2)
                return True
        return False

    def _learn_scale(self, axis, innovation):
        """An absolute fix after a stretch of dead reckoning says how long the steps really were."""
        travelled = self.travel[axis]
        self.travel[axis] = 0.
        if travelled is None or abs(travelled) < .8:
            return
        ratio = (travelled+innovation)/travelled
        if .5 < ratio < 2.:
            ratio = min(1.25, max(.8, ratio))**min(1., .4+abs(travelled)/3.)
            for store in (self.steady, self.first):
                store["F"] *= ratio

    def _update_x(self, H, innovation, R, gate=.6, absolute=False):
        if not self._gate("x", innovation, gate, absolute):
            return False
        H = np.asarray(H, float)
        S = float(H@self.P@H)+R
        K = self.P@H/S
        delta = K*innovation
        if absolute:
            self._learn_scale("x", delta[0])
        self.x += delta[0]
        self.partition += delta[1:]
        self.P = (np.eye(3)-np.outer(K, H))@self.P
        return True

    def _update_y(self, innovation, R, gate=.6, absolute=True):
        if not self._gate("y", innovation, gate, absolute):
            return False
        K = self.var_y/(self.var_y+R)
        if absolute:
            self._learn_scale("y", K*innovation)
        self.y += K*innovation
        self.var_y *= (1-K)
        return True

    def _fixes(self, forward, lateral, kind, im):
        """Correct the position from what the image shows of surfaces whose place is known."""
        fixed = []
        c, s = math.cos(self.heading), math.sin(self.heading)
        index = np.nonzero(kind == 1)[0]
        if len(index) >= 12:
            wx = self.x+c*forward[index]-s*lateral[index]
            wy = self.y+s*forward[index]+c*lateral[index]
            # Runs of neighbouring columns on one straight face, by the local direction of the foot line.
            runs, begin = [], 0
            for k in range(1, len(index)+1):
                if k == len(index) or index[k]-index[k-1] > 3 or math.hypot(wx[k]-wx[k-1], wy[k]-wy[k-1]) > .12:
                    runs.append((begin, k))
                    begin = k
            for a, b in runs:
                if b-a < 12:
                    continue
                px, py = wx[a:b], wy[a:b]
                w = 4
                dx, dy = np.abs(px[2*w:]-px[:-2*w]), np.abs(py[2*w:]-py[:-2*w])
                orient = np.where(dy > 2.5*dx, 1, np.where(dx > 2.5*dy, 2, 0))       # 1: face along y, 2: along x
                start = 0
                for k in range(1, len(orient)+1):
                    if k == len(orient) or orient[k] != orient[start]:
                        if orient[start] and k-start >= 8:
                            fixed.append(self._face(orient[start], px[start+w:k+w], py[start+w:k+w]))
                        start = k
        found = pad_fix(im)
        if found is not None and found[2]:
            f, l, _ = found
            gx, gy = self.goal()
            R = (.03+.05*math.hypot(f, l))**2
            ox, oy = gx-(c*f-s*l), gy-(s*f+c*l)
            a = self._update_x([1, 0, 0], ox-self.x, R, gate=1.2, absolute=True)
            b = self._update_y(oy-self.y, R, gate=1.2)
            fixed.append("pad" if a and b else "pad?")
        return [f for f in fixed if f]

    def _face(self, orient, px, py):
        """One straight face in view: a fence side (its place is known) or a partition face (its x is learned).

        Which face it is follows from the side it is seen from: a face along x that is north of the robot is the
        north fence; a face along y that is east of the robot is the west face of a partition or the east fence,
        whichever is nearest to where the dead reckoning puts it. A face that is farther off than the gate but has
        no rival is believed too: the robot has been stopped by a wall unnoticed, or has drifted."""
        if orient == 2:
            if px.max()-px.min() < .4:
                return None              # a partition's end face is only 0.24 m wide
            y = float(np.median(py))
            side = 1. if y > self.y else -1.
            innovation = side*nav.FENCE-y
            if .6 < abs(innovation) < 1.5:
                self.var_y = max(self.var_y, .5**2)
            ok = self._update_y(innovation, .03**2, gate=1.5)
            return "fence-y" if ok else "fence-y?"
        if py.max()-py.min() < .3:
            return None
        x = float(np.median(px))
        east_of_robot = x > self.x
        if east_of_robot:
            candidates = [(self.partition[0]-WALL_HALF, 0), (self.partition[1]-WALL_HALF, 1), (nav.FENCE, None)]
        else:
            candidates = [(-nav.FENCE, None), (self.partition[0]+WALL_HALF, 0), (self.partition[1]+WALL_HALF, 1)]
        ranked = sorted(candidates, key=lambda c: abs(c[0]-x))
        (face, k), runner_up = ranked[0], ranked[1]
        innovation = x-face
        if abs(innovation) > .6:
            if abs(innovation) > 1.5 or abs(runner_up[0]-x) < abs(innovation)+.5:
                return "face?"
            self.P[0, 0] = max(self.P[0, 0], .5**2)      # lost: let the robot's x take the correction
        if k is None:
            ok = self._update_x([1, 0, 0], -innovation, .03**2, gate=1.5, absolute=True)
            return "fence-x" if ok else "fence-x?"
        H = [-1, 0, 0]
        H[1+k] = 1
        # measured: face x minus robot x; predicted: the same from the state. A partition whose x is settled is as
        # good as a fence side for the robot's own x.
        ok = self._update_x(H, innovation, .03**2, gate=1.5, absolute=self.P[1+k, 1+k] < .04**2)
        return f"partition{k}" if ok else f"partition{k}?"

    def _map(self, forward, lateral, kind):
        c, s = math.cos(self.heading), math.sin(self.heading)
        free = np.zeros((nav.N, nav.N), bool)
        hit = np.zeros((nav.N, nav.N), bool)
        seen = np.isfinite(forward)
        forward, lateral = np.where(seen, forward, 1.), np.where(seen, lateral, 0.)
        ranges = np.hypot(forward, lateral)
        unit_f, unit_l = forward/np.maximum(ranges, 1e-9), lateral/np.maximum(ranges, 1e-9)
        for code, reach, mark in ((0, lambda r: np.minimum(r, LIGHT)-.08, False), (1, lambda r: r-.12, True)):
            cols = np.nonzero(kind == code)[0]
            if not len(cols):
                continue
            limit = reach(ranges[cols])
            for t in np.arange(.2, LIGHT, nav.CELL*.6):
                use = cols[t < limit]
                if not len(use):
                    break
                fx, fl = unit_f[use]*t, unit_l[use]*t
                wx, wy = self.x+c*fx-s*fl, self.y+s*fx+c*fl
                iy = np.clip(((wy+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1)
                ix = np.clip(((wx+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1)
                free[iy, ix] = True
            if mark:
                r = ranges[cols]+.03
                wx = self.x+c*unit_f[cols]*r-s*unit_l[cols]*r
                wy = self.y+s*unit_f[cols]*r+c*unit_l[cols]*r
                iy = np.clip(((wy+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1)
                ix = np.clip(((wx+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1)
                hit[iy, ix] = True
        cols = np.nonzero(kind == 2)[0]
        if len(cols):
            r = .2/np.maximum(unit_f[cols], .5)
            wx = self.x+c*unit_f[cols]*r-s*unit_l[cols]*r
            wy = self.y+s*unit_f[cols]*r+c*unit_l[cols]*r
            hit[np.clip(((wy+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1),
                np.clip(((wx+nav.EXTENT)/nav.CELL).astype(int), 0, nav.N-1)] = True
        free &= ~hit
        # The newest look decides: an older one was placed with an older, possibly drifted, position estimate.
        self.logodds[free] = np.minimum(self.logodds[free]-MISS, -MISS)
        self.logodds[hit] = np.maximum(self.logodds[hit]+HIT, HIT)
        here = (self.X-self.x)**2+(self.Y-self.y)**2 < .13**2
        self.logodds[here] = np.minimum(self.logodds[here], -1.)        # the robot stands on free floor
        np.clip(self.logodds, -CLAMP, CLAMP, out=self.logodds)
        self.version += 1

    # --- motion ----------------------------------------------------------------------------------------------------
    def press(self, gait):
        before = self.heading
        self._status(self.client.act([self.button[gait]/3]))
        turned = nav.wrap(self.heading-before)
        repeat = self.last == gait
        if self.last in ("F", "B") and not repeat:
            # the base coasts on: what the first press of a run falls short of a steady one arrives now
            coast = (self.steady[self.last]-self.first[self.last])*(1 if self.last == "F" else -1)
            self.x += coast*math.cos(before)
            self.y += coast*math.sin(before)
        if gait in ("L", "R"):
            size = abs(turned)
            if size > math.radians(5) and (turned > 0) == (gait == "L"):
                # The compass reads whole degrees: single presses are 1 degree uncertain, their mean is not.
                key = gait+("s" if repeat else "1")
                self.samples[key] += 1
                self.sums[key] += size
                (self.steady if repeat else self.first)[gait] = self.sums[key]/self.samples[key]
            d = 0.
        else:
            d = (self.steady if repeat else self.first)[gait]*(1 if gait == "F" else -1)
            if gait == "F" and self.ahead is not None and all(p["gait"] == "F" for p in self.since):
                # a wall seen straight ahead at the last look stops the robot's front, whatever the reckoning says
                room = self.ahead-nav.ROBOT_RADIUS-sum(p["d"] for p in self.since)
                d = min(d, max(0., room))
            mid = before+turned/2
            self.x += d*math.cos(mid)
            self.y += d*math.sin(mid)
            q = (.15*abs(d))**2
            self.P[0, 0] += q*math.cos(mid)**2+1e-5
            self.var_y += q*math.sin(mid)**2+1e-5
            self.blind += 1
            limit = nav.FENCE-nav.ROBOT_RADIUS           # the fence stops the robot, whatever the reckoning says
            self.x, self.y = min(limit, max(-limit, self.x)), min(limit, max(-limit, self.y))
            if gait == "F":
                for axis, part in (("x", d*math.cos(mid)), ("y", d*math.sin(mid))):
                    if self.travel[axis] is not None:
                        self.travel[axis] += part
            else:
                self.travel = {"x": None, "y": None}        # a backward step: this stretch says nothing about forward
        self.since.append({"gait": gait, "d": d, "steady": repeat, "heading": self.heading})
        self.last, self.fresh = gait, False

    def steps(self):
        return {"F": self.steady["F"], "L": self.steady["L"], "R": self.steady["R"]}

    # --- learning the remote ---------------------------------------------------------------------------------------
    def _glance(self):
        """A look that only measures the range straight ahead, and whether the wall there is seen square on."""
        self.glanced = self._status(self.client.observe())
        im = decode(self.glanced)
        self.looks += 1
        self.why["remote"] = self.why.get("remote", 0)+1
        forward, _, kind = scan(im)
        ahead = self._ahead(forward, kind)
        wide = kind[96:160] == 1
        square = bool(ahead is not None and wide.sum() >= 48 and np.ptp(forward[96:160][wide]) < .1*ahead+.04)
        return ahead, square

    def _turn_to(self, target):
        """Turn in place to the reachable heading nearest to `target` (rad)."""
        for _ in range(60):
            error = nav.wrap(target-self.heading)
            gait = "L" if error > 0 else "R"
            if abs(error) <= self.steady[gait]/2:
                return
            self.press(gait)

    def _walk_raw(self, button, early):
        self._status(self.client.act([button/3]))
        early.append((button, self.heading))

    def learn(self):
        """Which button is which gait, and how long a forward step is.

        Turns show on the compass. The robot then faces the nearest fence side and watches the range straight
        ahead: the walking button that shortens it is forward, and in a run of forward presses every press after
        the first is one steady step. The walking presses made before all that is known are placed afterwards."""
        reply = self._status(self.client.observe())
        self.looks += 1
        self.x, self.y = reply["start"]["x"], reply["start"]["y"]
        early, turns, walks = [], {}, []
        for b in range(4):
            before = self.heading
            self._status(self.client.act([b/3]))
            turned = nav.wrap(self.heading-before)
            if abs(turned) > math.radians(6):
                turns[b] = turned
            else:
                walks.append(b)
                early.append((b, self.heading))
        left = max(turns, key=turns.get)
        right = min(turns, key=turns.get)
        self.button.update(L=left, R=right)
        self.first["L"] = self.steady["L"] = abs(turns[left])
        self.first["R"] = abs(turns[right])
        # Two left presses in a row: the second is a steady step; the first falls short of it by the base's lag.
        self.last = None
        self.press("L")
        self.press("L")
        lag = min(.95, max(.8, self.first["L"]/self.steady["L"]))
        self.steady["R"] = self.first["R"]/lag
        # Face the nearest fence side (the start pose says where it is) and walk until a wall shows in the middle
        # of the image: the walking button that brings it nearer is forward. A walking button that has been pressed
        # more often than the distance allows without a wall coming into reach was a step back.
        distance, angle = min([(nav.FENCE-self.x, 0.), (nav.FENCE+self.x, math.pi), (nav.FENCE-self.y, math.pi/2),
                               (nav.FENCE+self.y, -math.pi/2)])
        self._turn_to(angle)
        ahead, _ = self._glance()
        a, b = walks
        fwd = back = None
        tried = {a: 0, b: 0}
        current = a
        for _ in range(60):
            if fwd is not None:
                break
            before = ahead
            self._walk_raw(current, early)
            tried[current] += 1
            other = b if current == a else a
            # No frame is spent while the wall cannot have come into reach yet, even with the longest step.
            if before is None and distance+.12*tried[other]-.28*tried[current] > LIGHT+.1:
                continue
            ahead, _ = self._glance()
            if before is not None and ahead is not None and abs(before-ahead) > .04:
                fwd, back = (current, other) if before > ahead else (other, current)
            elif before is None and ahead is not None:
                fwd, back = current, other
            elif before is not None and ahead is None:
                fwd, back = other, current               # the wall went out of reach: that was a step back
            else:
                gone = .12*tried[other]                  # what the other button may have added to the distance
                if tried[current] > (distance+gone-LIGHT+.3)/.19+2:
                    if tried[other] == 0:
                        current = other
                    else:
                        fwd, back = (a, b) if tried[a] <= tried[b] else (b, a)      # (not reached in a fenced bay)
        if fwd is None:
            fwd, back = a, b
        # The forward step: in a run of forward presses every press after the first is a steady step, free of any
        # coast, so the range after the first press minus the range after a later one is that many steady steps;
        # the nearer the wall, the finer the range. Back off first where the wall is too near for a run.
        samples = []
        for attempt in range(3):
            for _ in range(8):
                if ahead is None or ahead >= .8:
                    break
                self._walk_raw(back, early)
                ahead, _ = self._glance()
            ranges = []
            for _ in range(10):
                self._walk_raw(fwd, early)
                ahead, _ = self._glance()
                if ahead is None:
                    ranges = []
                    continue
                ranges.append(ahead)
                if len(ranges) >= 3 or ahead < .5:
                    break
            if len(ranges) >= 2 and ranges[-1] >= .27 and .05 < (ranges[0]-ranges[-1])/(len(ranges)-1) < .45:
                samples.append((ranges[0]-ranges[-1])/(len(ranges)-1))
                break
        steady = sum(samples)/len(samples) if samples else .24
        self.button.update(F=fwd, B=back)
        self.steady["F"], self.first["F"] = steady, steady*lag
        self.steady["B"], self.first["B"] = .65*steady, .65*steady*lag     # roughly: the reference never backs up
        # Every walking press with its coast is one steady step; the last run's coast is still to come (press()).
        signed = {fwd: self.steady["F"], back: -self.steady["B"]}
        for button, heading in early:
            self.x += signed[button]*math.cos(heading)
            self.y += signed[button]*math.sin(heading)
        last = "F" if early[-1][0] == fwd else "B"
        pending = (self.steady[last]-self.first[last])*(1 if last == "F" else -1)
        self.x -= pending*math.cos(self.heading)
        self.y -= pending*math.sin(self.heading)
        self.x = min(nav.FENCE-.16, max(-nav.FENCE+.16, self.x))
        self.y = min(nav.FENCE-.16, max(-nav.FENCE+.16, self.y))
        self.P[0, 0] = self.var_y = .15**2
        self.last = last
        self.since, self.ahead = [], None
        self.travel = {"x": None, "y": None}
        # The last frame, taken square on to the wall just used: a full look at it places the robot against it.
        self.looks -= 1
        self._process(self.glanced)

    def loaded(self):
        """After the loading stop every gait is slower by one factor. Measure it on the compass: five presses of
        each turn, of which the second to fifth are steady steps, against the steady steps learned before."""
        old = dict(self.steady)
        for k in self.samples:
            self.samples[k], self.sums[k] = 0, 0.
        self.last = None
        for gait in ("L", "R"):
            for _ in range(5):
                if self.obs["done"] or self.obs["steps_remaining"] <= 0:
                    return
                self.press(gait)
        factors = [self.steady[k]/old[k] for k in ("L", "R") if self.samples[k+"s"] >= 3 and old[k] > 0]
        factor = sum(factors)/len(factors) if factors else .675
        factor = min(.8, max(.55, factor))
        for store in (self.steady, self.first):
            for gait in ("F", "B"):
                store[gait] *= factor
        self.travel = {"x": None, "y": None}

    # --- driving ---------------------------------------------------------------------------------------------------
    def plan(self, goal):
        key = (self.version, goal)
        if self.cache is None or self.cache[0] != key:
            occupied = (self.logodds > .5) | self.outside
            margin = MARGIN
            while True:
                blocked = nav.inflate(occupied, nav.ROBOT_RADIUS+margin)
                blocked[nav.cell(self.x, self.y)] = False
                unknown = (np.abs(self.logodds) <= .5) & self.band
                dist = nav.distance_field(blocked, goal, 1.+UNKNOWN_BAND_COST*unknown)
                if np.isfinite(dist[nav.cell(self.x, self.y)]) or margin <= 0:
                    break
                margin -= .02
            self.cache = (key, blocked, dist)
        return self.cache[1], self.cache[2]

    def decide(self, goal):
        target = self.detours[0] if self.detours else goal
        if math.hypot(target[0]-self.x, target[1]-self.y) < .12:
            # Standing where the delivery point should be, and it is not reached: look around for the pad.
            if not self.fresh and self.spin == 0:
                return "look"
            if self.spin < 6:
                self.spin += 1
                return "search"
            self.spin = 0
            if self.detours:
                self.detours.pop(0)
            if not self.detours:
                ring = [(.45, 0), (0, .45), (-.45, 0), (0, -.45), (.45, .45), (-.45, .45), (-.45, -.45), (.45, -.45)]
                self.detours = [(goal[0]+dx, goal[1]+dy) for dx, dy in ring
                                if abs(goal[0]+dx) < nav.FENCE-.3 and abs(goal[1]+dy) < nav.FENCE-.3]
            target = self.detours[0]
        blocked, dist = self.plan(target)
        return nav.next_press((self.x, self.y, self.heading), dist, blocked, self.steps(), target, reach=.1)

    def need_look(self, goal):
        """Before a step forward: could there be a wall ahead that no look has covered, or has it been long enough?

        The bay's floor is free except for the fence (known) and the two partitions. A partition stands inside its
        band, and once its x has been measured, inside a 0.24 m strip: only there can unseen floor hide a wall."""
        if self.blind >= MAX_BLIND:
            return "long"
        blocked, _ = self.plan(self.detours[0] if self.detours else goal)
        c, s = math.cos(self.heading), math.sin(self.heading)
        if self.blind >= 1:
            # about to go through a partition: a fresh look at the opening first, and at every step inside it
            for k in (0, 1):
                gap = self.partition[k]-self.x
                if abs(gap) < .75 and gap*c > .3*abs(gap) and abs(c) > .3:
                    return "gate"
        if self.blind >= 2:
            # close to a wall (a corner, the fence): a look every second step
            for d in (.2, .4, .6):
                for l in (-.3, 0., .3):
                    if blocked[nav.cell(self.x+c*d-s*l, self.y+s*d+c*l)]:
                        return "near"
        possible = self.band
        for k in (0, 1):
            if self.P[1+k, 1+k] < .05**2:
                side = (self.X < 0) if k == 0 else (self.X > 0)
                possible = possible & ~(side & (np.abs(self.X-self.partition[k]) > WALL_HALF+.1))
        c, s = math.cos(self.heading), math.sin(self.heading)
        for d in np.arange(.25, self.steady["F"]+.23, .05):
            half = min(.2, .8*d)
            for l in (-half, -half/2, 0., half/2, half):
                iy, ix = nav.cell(self.x+c*d-s*l, self.y+s*d+c*l)
                if abs(self.logodds[iy, ix]) <= .5 and possible[iy, ix] and not blocked[iy, ix]:
                    return "band"
        # coming up to the delivery point: one look for the pad, which fixes the position where it matters
        dx, dy = goal[0]-self.x, goal[1]-self.y
        if (not self.pad_looked and math.hypot(dx, dy) < LIGHT-.2
                and abs(nav.wrap(math.atan2(dy, dx)-self.heading)) < math.radians(35)):
            self.pad_looked = True
            return "pad"
        return ""

    def run(self):
        self.learn()
        while not self.obs["done"] and self.obs["requests_remaining"] > 1 and self.obs["steps_remaining"] > 0:
            if self.obs["goal_reached"]:
                delivery = self.obs["delivery"]
                gx, gy = self.goal()
                off = math.hypot(self.x-gx, self.y-gy)
                if off > .25:                            # reached means within 0.3 m of the pad's centre
                    self.x, self.y = gx+(self.x-gx)*.25/off, gy+(self.y-gy)*.25/off
                self._status(self.client.advance())
                self.spin, self.detours, self.pad_looked = 0, [], False
                if delivery == LOADING_STOP:
                    self.loaded()
                self.fresh = False
                continue
            goal = self.goal()
            press = self.decide(goal)
            if press == "look":
                self.look("search")
                continue
            if press == "search":
                before = (self.x, self.y)
                for _ in range(max(1, int(math.radians(60)/max(self.steady["L"], .1)))):
                    self.press("L")
                self.look("search")
                if math.hypot(self.x-before[0], self.y-before[1]) > .1:
                    self.spin, self.detours = 0, []       # the pad came into view and moved the estimate
                continue
            if press == "F":
                why = "" if self.fresh else self.need_look(goal)
                if why:
                    self.look(why)
                    continue
                if self.blocked_count >= 2:
                    # walking into something the map does not show: mark it, trust the position less, plan around
                    ahead = (self.x+.24*math.cos(self.heading), self.y+.24*math.sin(self.heading))
                    self.logodds[nav.cell(*ahead)] = CLAMP
                    self.version += 1
                    self.blocked_count = 0
                    self.P[0, 0] += .2**2
                    self.var_y += .2**2
                    continue
            self.press(press)
        return self.obs


def solve(client, **options):
    reference = Reference(client, **options)
    try:
        return reference.run()
    except ValueError as exc:            # the service refuses requests once a budget is used up
        if "finished" in str(exc) or "exhausted" in str(exc):
            return reference.obs
        raise
