"""The scout's mission: map the dock, find the pad, and see the whole of a route the carrier can take.

The scout knows neither where it is in the hall nor how it is turned; everything here happens in its own map
frame. It

  1. identifies its four buttons (one press each) and surveys the dock, where the carrier will be set down;
  2. explores until the pad is in the map: it goes where the most floor it has not yet looked at is nearest;
  3. plans the carrier's route from the dock to the pad on the optimistic reading of the map and drives down
     the middle of it wherever the carrier's lane was not yet seen as floor over its full width, until the
     route is certain end to end (or the presses run out).
"""
import numpy as np
from scipy import ndimage

import mapping as M
import planning as G
import vision as V

CARRIER = .65                    # the carrier's circumscribed circle (<= 0.60 m) plus a margin
DOCK = .8                        # the carrier is set down within this distance of the dock point
SCOUT = .36                      # the scout plans this far from what is solid (so: never through a narrow opening)
SCOUT_HARD = .21                 # the scout's own circumscribed circle
SEE = 1.3                        # half-width of what a frame shows of the floor (the pad is looked for in all of it)
TRUST = .6                       # a view is planned to hold what it is for this near its centre (a frame trusts 0.8 m)
ROOM_DISC = .9                   # a disc of this radius passes no opening of the partition
PAD_ROOM = .75                   # no part of the pad is nearer than this to a wall or a crate
SEQUENCES = [(a, b, c) for a in range(4) for b in range(4) for c in range(4)]


class Scout:
    def __init__(self, robot, obs, sense, mosaic, log=lambda *a: None):
        """`sense(obs, stride)` registers the observation and returns the scout's pose in the map."""
        self.robot, self.sense, self.mosaic, self.log = robot, sense, mosaic, log
        self.obs = obs
        self.odometer = np.array(obs["odometer"], float)
        self.pose = np.zeros(3)
        self.dirs = [None]*4                       # each button's direction in the body frame
        self.step = .2                             # metres a press moves the scout
        self.stuck = 0
        self.given_up = []                         # views the scout could not get to

    # --- pressing ------------------------------------------------------------------------- #
    def left(self):
        return self.obs["steps_remaining"]

    def press(self, button):
        before = self.pose.copy()
        self.obs = self.robot.act([button/3])
        odometer = np.array(self.obs["odometer"], float)
        stride, self.odometer = odometer-self.odometer, odometer
        self.pose = self.sense(self.obs, stride)
        moved = V.rot(-before[2])@(self.pose[:2]-before[:2])
        slipped = np.hypot(*stride)-np.hypot(*moved) > .02
        if not slipped and np.hypot(*moved) > .1:
            self.step = .8*self.step+.2*float(np.hypot(*moved))
        return moved, slipped

    def identify(self):
        """One press per button: each moves the scout along one of its own axes."""
        axes = [np.array(a, float) for a in ((1, 0), (-1, 0), (0, 1), (0, -1))]
        for attempt in range(3):
            for b in range(4):
                if self.dirs[b] is None and self.left() > 0:
                    moved, _ = self.press(b)
                    if np.hypot(*moved) > .04:
                        self.dirs[b] = axes[int(np.argmax([a@moved for a in axes]))]
            known = [tuple(d) for d in self.dirs if d is not None]
            if len(known) == 4 and len(set(known)) == 4:
                return
            # Two buttons cannot share a direction: press the doubtful ones again.
            for b in range(4):
                if self.dirs[b] is None or known.count(tuple(self.dirs[b])) > 1:
                    self.dirs[b] = None
        missing = [a for a in axes if not any(d is not None and (d == a).all() for d in self.dirs)]
        for b in range(4):
            if self.dirs[b] is None:
                self.dirs[b] = missing.pop()

    def moves(self):
        """Where each button would take the scout from here (map frame)."""
        return [self.pose[:2]+V.rot(self.pose[2])@d*self.step for d in self.dirs]

    # --- driving -------------------------------------------------------------------------- #
    def toward(self, grid, field):
        """One press down `field` (cost to the target on `grid`): the first press of the best three (the
        buttons drive along the scout's own axes, seldom along the route, so progress takes two of them).
        False when no three presses make progress."""
        here = field[grid.cell(self.pose[:2])]
        steps = [V.rot(self.pose[2])@d*self.step for d in self.dirs]
        best, value = None, np.inf
        for sequence in SEQUENCES:
            p, total = self.pose[:2], 0.
            for b in sequence:
                q = p+steps[b]
                if min(grid.gap[grid.cell(q)], grid.gap[grid.cell((p+q)/2)]) < SCOUT_HARD:
                    total = np.inf
                    break
                p = q
                total += field[grid.cell(p)]            # summed over the three: sooner is better
            if total < value:
                best, value = sequence[0], total
        if best is None or not np.isfinite(value) or (np.isfinite(here) and value >= 3*here-.05):
            return False
        expected = self.step
        moved, slipped = self.press(best)
        if np.hypot(*moved) < .3*expected and not slipped:
            # Something the map does not show is in the way: note it, so the next route avoids it.
            self.stuck += 1
            ahead = self.pose[:2]+V.rot(self.pose[2])@self.dirs[best]*(SCOUT_HARD+.05)
            r, c = (int(round(v)) for v in M.to_cell(ahead[0], ahead[1]))
            self.mosaic.solid[r-4:r+5, c-4:c+5] += 2
        return True

    def field_to(self, grid, target, radius=.12):
        """Cost for the scout to reach `target`: through anything not known to be solid."""
        passable = grid.gap >= SCOUT
        toll = 1+2*np.clip((SCOUT+.2-grid.gap)/.2, 0, 1)
        return G.geodesic(passable, grid.disc(target, radius), grid.res, toll)

    def clear_off(self, grid):
        """The scout has come nearer to something solid than it plans to be (the map did not show it yet):
        step away, to where routes start. The presses made."""
        made = 0
        while made < 6 and self.left() > 0 and grid.gap[grid.cell(self.pose[:2])] < SCOUT+.02:
            here = grid.gap[grid.cell(self.pose[:2])]
            ends = self.moves()
            gaps = [min(grid.gap[grid.cell(p)], grid.gap[grid.cell((p+self.pose[:2])/2)]) for p in ends]
            best = int(np.argmax(gaps))
            if gaps[best] <= here:
                break
            self.press(best)
            made += 1
        return made

    def go(self, target, presses, radius=.2, until=None):
        """Drive toward `target` for at most `presses` presses; the number of presses made. It stops when it
        is within `radius`, and when five presses in a row have brought it no nearer."""
        grid = G.Grid(self.mosaic)
        made = self.clear_off(grid)
        field = self.field_to(grid, target, radius)
        nearest, since = np.inf, 0
        for k in range(presses):
            away = float(np.hypot(*(self.pose[:2]-target)))
            if self.left() <= 0 or away < radius:
                break
            if until is not None and until():
                break
            nearest, since = (away, 0) if away < nearest-.05 else (nearest, since+1)
            if since >= 5 or not self.toward(grid, field):
                break
            made += 1
            if k % 6 == 5:
                grid = G.Grid(self.mosaic)
                field = self.field_to(grid, target, radius)
        return made

    # --- what to look at next ------------------------------------------------------------------- #
    def view_of(self, grid, wanted, half, weight=None):
        """The reachable place from which the most of the `wanted` cells lie in a frame's square of half-width
        `half` (less what the scout itself hides), weighed against the way there; None when there is none."""
        passable = grid.gap >= SCOUT
        for reach in (.15, .3, .45, .6):                   # (from the nearest place routes start from)
            source = grid.disc(self.pose[:2], reach)
            if (source & passable).any():
                break
        cost = G.geodesic(passable, source, grid.res)
        k, hidden = 2*int(half/grid.res)+1, 2*int(.3/grid.res)+1
        w = wanted.astype(np.float32)
        gain = ndimage.uniform_filter(w, size=k, mode="constant")*k*k
        gain -= ndimage.uniform_filter(w, size=hidden, mode="constant")*hidden*hidden
        if weight is not None:
            gain = gain*weight
        reachable = np.isfinite(cost)
        for p in self.given_up:
            reachable &= ~grid.disc(p, .5)
        value = np.where(reachable & (gain > max(3., .02*gain.max())), gain/(cost+.6), -1.)
        if value.max() <= 0:
            return None
        r, c = np.unravel_index(int(np.argmax(value)), value.shape)
        return grid.point(r, c)

    def unexplored(self, grid):
        """While the pad is unknown: where it can still be (floor never seen, far enough from everything solid
        for the pad: its centre is at least 1.0 m from a wall and 1.15 m from a crate), and a weight that
        favours views beyond an opening (the pad is in the other room: the dock's room is what a disc too wide
        for any opening reaches from the dock on the map so far)."""
        labels, _ = ndimage.label(grid.gap >= ROOM_DISC, structure=np.ones((3, 3), bool))
        home = labels == labels[grid.cell((0., 0.))]
        beyond = ~ndimage.binary_dilation(home, iterations=int(ROOM_DISC/grid.res)+2) & grid.seen
        k = 2*int(SEE/grid.res)+1
        weight = np.where(ndimage.uniform_filter(beyond.astype(np.float32), size=k, mode="constant") > .15, 3., 1.)
        return ~grid.seen & (grid.gap > PAD_ROOM), weight

    def route(self, grid, pad, certain, radius=CARRIER):
        """The carrier's route from the dock to the pad as a cost field (and whether it exists)."""
        room = grid.room if certain else np.minimum(grid.gap, 1e3)
        passable = room >= radius
        toll = 1+4*np.clip((radius+.15-room)/.15, 0, 1)
        field = G.geodesic(passable, grid.disc(pad, .15), grid.res, toll)
        return field, bool(np.isfinite(field[grid.cell((0., 0.))]))

    def certain(self, grid, pad):
        """The carrier, set down anywhere at the dock, has a route to the pad over floor that was seen."""
        passable = grid.room >= CARRIER
        if passable[grid.disc((0., 0.), DOCK)].mean() < .98:
            return False
        labels, _ = ndimage.label(passable, structure=np.ones((3, 3), bool))
        home = labels[grid.cell((0., 0.))]
        return home > 0 and bool((labels[grid.disc(pad, .15)] == home).any())

    def unseen_on_route(self, grid, pad):
        """The cells the carrier would cover on the optimistic route (and at the dock) that are neither seen
        floor nor explained by an obstacle's foot; None when there is no route."""
        field, exists = self.route(grid, pad, certain=False)
        if not exists:
            return None
        lane = np.zeros(grid.shape, bool)
        for p in grid.descend(field, (0., 0.), 40.):
            lane[grid.cell(p)] = True
        lane = ndimage.distance_transform_edt(~lane)*grid.res <= CARRIER
        lane |= grid.disc((0., 0.), DOCK+CARRIER)
        return lane & (grid.room < grid.res)

    def drive_route(self, pad):
        """Drive the scout down the middle of the optimistic route wherever the carrier's lane is not yet
        certain: from the middle of the lane everything in it shows as floor, a metre ahead and behind.
        True when the route is certain."""
        for attempt in range(8):
            grid = G.Grid(self.mosaic)
            pad = self.mosaic.pad() if self.mosaic.pad() is not None else pad
            if self.certain(grid, pad):
                return True
            field, exists = self.route(grid, pad, certain=False)
            if not exists or self.left() <= 2:
                return False
            path = grid.descend(field, (0., 0.), 40.)
            stops = path[::8]+[path[-1]]
            todo = [i for i, p in enumerate(stops) if grid.room[grid.cell(p)] < CARRIER]
            if not todo:
                # Every stop is certain but the lane is not, somewhere between or at the dock: look at it.
                wanted = self.unseen_on_route(grid, pad)
                target = self.view_of(grid, wanted, TRUST) if wanted is not None and wanted.any() else None
                if target is None or not self.go(target, 10, .2):
                    return False
                continue
            near_first = np.hypot(*(stops[todo[0]]-self.pose[:2])) <= np.hypot(*(stops[todo[-1]]-self.pose[:2]))
            order = range(todo[0], todo[-1]+1) if near_first else range(todo[-1], todo[0]-1, -1)
            for n, i in enumerate(order):
                if self.left() <= 2:
                    return False
                if n % 3 == 2:
                    grid = G.Grid(self.mosaic)
                if grid.room[grid.cell(stops[i])] >= CARRIER:
                    continue
                self.log("route stop", i, "of", len(stops), np.round(stops[i], 2), "left", self.left())
                if not self.go(stops[i], 16, .2) and np.hypot(*(stops[i]-self.pose[:2])) > .3:
                    break                              # the way is not what the map said: plan again
        return False

    # --- the mission ---------------------------------------------------------------------------- #
    def run(self):
        self.identify()
        for corner in ((.65, .65), (-.65, .65), (-.65, -.65), (.65, -.65)):
            self.go(np.array(corner), 14)
        pad, sure, idle = None, False, 0
        while self.left() > 2 and idle < 12:
            pad = self.mosaic.pad()
            if pad is not None:
                sure = self.drive_route(pad)
                pad = self.mosaic.pad()
                if sure or self.left() <= 2:
                    break
            # The pad, or a way to it, is not on the map yet: look at floor not yet looked at.
            grid = G.Grid(self.mosaic)
            floor, weight = self.unexplored(grid)
            target = self.view_of(grid, floor, SEE, weight)
            self.log("left", self.left(), "at", self.pose[:2].round(2), "pad", pad is not None,
                     "view", None if target is None else np.round(target, 2))
            if target is None:
                break
            made = self.go(target, 8, .25, until=None if pad is not None else lambda: self.mosaic.pad() is not None)
            if not made:
                self.given_up.append(target)
            idle = 0 if made else idle+1
        self.log("scout done: presses left", self.left(), "pad", None if pad is None else pad.round(2), "certain", sure)
        return pad, sure
