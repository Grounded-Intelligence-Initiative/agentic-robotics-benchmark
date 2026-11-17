"""The carrier's drive to the pad on the scout's map.

The carrier is heavy: its velocity follows a button with a lag, and a slipping press delivers only part of the
commanded velocity. The response (one pole and a steady speed, a 20 x 10 ms velocity-tracking model) is fitted
to the displacements the localizer measures, press by press; the presses the odometer shows to have slipped
are left out of the fit and their loss is read off the measured displacement. Control is model-predictive:
before every press, every sequence of the next five presses is rolled through the model, and the first button
of the best one is pressed: the sequence that stays nearest a point 0.4 m down the route, among those whose
every sampled point keeps the carrier's circumscribed circle on floor the scout has seen and whose first
press stays clear even if it slips.
"""
import itertools
import math

import numpy as np

import planning as G
import vision as V

H = .01                          # model step, seconds
SUB = 20                         # steps per press
SAMPLES = np.array([4, 9, 14, 19])
HARD = .63                       # the carrier's circumscribed circle (<= 0.60 m) plus the localization error
ROUTES = (.66, .64)              # the route keeps this far from anything not seen as floor (first that connects)
AHEAD = .4                       # the route point steered for, metres down the route
ROOM = .12                       # clearance below which a point of a sequence is charged
CLOSE = .25                      # the charge at zero clearance, metres per press
HORIZON = 5
SLIP_LOSS = .8                   # the worst slip: this fraction of the commanded velocity is lost
SLIP_TURN = math.radians(6.)     # and the body turns by up to this
AXES = [np.array(a, float) for a in ((1, 0), (-1, 0), (0, 1), (0, -1))]


class Model:
    """v' = a v + (1 - a) w per 10 ms step, w the commanded steady velocity."""

    def __init__(self, a=math.exp(-H/.16), speed=.7):
        self.a, self.speed = a, speed
        powers = a**np.arange(1, SUB+1)
        self.C_all = H*np.cumsum(powers)                 # displacement per unit initial velocity, by substep
        self.D_all = H*np.cumsum(1-powers)               # displacement per unit command
        self.C, self.D = self.C_all[SAMPLES], self.D_all[SAMPLES]
        self.cv, self.dv = float(powers[-1]), float(1-powers[-1])
        self.coast = H*a/(1-a)                            # distance per unit velocity until the base has stopped


def fit(history, dirs):
    """(model, velocity now) from the presses so far: (button, heading before, displacement in the map,
    slipped). The pole and the speed that best reproduce the displacements of the presses that did not slip."""
    poles = np.exp(-H/np.linspace(.08, .32, 25))
    speeds = np.linspace(.45, 1.0, 34)
    a, speed = (v.ravel() for v in np.meshgrid(poles, speeds, indexing="ij"))
    powers = a[:, None]**np.arange(1, SUB+1)[None, :]
    C = H*powers.sum(axis=1)
    D = H*(1-powers).sum(axis=1)
    cv, dv = powers[:, -1], 1-powers[:, -1]
    v = np.zeros((len(a), 2))
    err = np.zeros(len(a))
    for button, heading, moved, slipped in history:
        unit = V.rot(heading)@dirs[button]
        u = speed[:, None]*unit[None, :]
        if slipped:
            along = (moved[None, :]-C[:, None]*v)@unit
            grip = np.clip(along/(D*speed), .05, 1.)
            u = u*grip[:, None]
        else:
            err += ((C[:, None]*v+D[:, None]*u-moved[None, :])**2).sum(axis=1)
        v = cv[:, None]*v+dv[:, None]*u
    clean = sum(1 for h in history if not h[3])
    if clean < 3:
        best = int(np.argmin((a-math.exp(-H/.16))**2+(speed-.7)**2*1e-4+err*10))
    else:
        best = int(np.argmin(err))
    return Model(float(a[best]), float(speed[best])), v[best].copy()


class Driver:
    def __init__(self, robot, obs, locate, mosaic, pad, log=lambda *a: None):
        """`locate(obs)` returns the carrier's pose (x, y, heading) in the scout's map."""
        self.robot, self.obs, self.locate, self.log = robot, obs, locate, log
        self.pad = np.asarray(pad, float)
        self.odometer = np.array(obs["odometer"], float)
        self.pose = np.asarray(self.locate(obs), float)
        self.grid = G.Grid(mosaic)
        self.history = []
        self.dirs = [None]*4
        self.model = Model()
        self.velocity = np.zeros(2)
        self.sequences = np.array(list(itertools.product(range(4), repeat=HORIZON)))
        self._plan()

    def _plan(self):
        grid = self.grid
        start = grid.cell(self.pose[:2])
        for certain in (True, False):
            room = grid.room if certain else np.minimum(grid.gap, 1e3)
            self.clearance = (grid.fine_room if certain else np.minimum(grid.fine_gap, 1e3))-HARD
            for radius in ROUTES:
                passable = room >= radius
                toll = 1+4*np.clip((radius+.15-room)/.15, 0, 1)
                self.field = G.geodesic(passable, grid.disc(self.pad, .12), grid.res, toll)
                near = self.field[max(0, start[0]-12):start[0]+13, max(0, start[1]-12):start[1]+13]
                if np.isfinite(near).any():
                    self.log("route: certain" if certain else "route: optimistic", "radius", radius)
                    return
        self.log("no route to the pad on this map")

    # --- pressing ------------------------------------------------------------------------- #
    def press(self, button):
        before = self.pose.copy()
        self.obs = self.robot.act([button/3])
        odometer = np.array(self.obs["odometer"], float)
        stride, self.odometer = odometer-self.odometer, odometer
        if self.obs["done"]:
            return None
        self.pose = np.asarray(self.locate(self.obs), float)
        moved = self.pose[:2]-before[:2]
        slipped = bool(np.hypot(*stride)-np.hypot(*moved) > .02)
        self.history.append((button, before[2], moved, slipped))
        return moved

    def identify(self):
        """One press per button (more where a press slipped): which of the body's axes each one drives."""
        for attempt in range(4):
            for b in range(4):
                if self.dirs[b] is not None or self.obs["done"]:
                    continue
                before, velocity = self.pose.copy(), self.velocity.copy()
                moved = self.press(b)
                if moved is None:
                    return
                # what the press added to the coasting of the presses before it, in the body's frame
                own = V.rot(-before[2])@(moved-self.model.C_all[-1]*velocity)
                slipped = self.history[-1][3]
                if np.hypot(*own) > .025 and not slipped:
                    self.dirs[b] = AXES[int(np.argmax([a@own for a in AXES]))]
                    self.velocity = self.model.cv*velocity+self.model.dv*self.model.speed*(V.rot(before[2])@self.dirs[b])
                else:
                    self.velocity = self.model.cv*velocity+(moved-self.model.C_all[-1]*velocity)/self.model.D_all[-1]*self.model.dv
            known = [tuple(d) for d in self.dirs if d is not None]
            if len(known) == 4 and len(set(known)) == 4:
                break
            for b in range(4):
                if self.dirs[b] is not None and known.count(tuple(self.dirs[b])) > 1:
                    self.dirs[b] = None
        missing = [a for a in AXES if not any(d is not None and (d == a).all() for d in self.dirs)]
        for b in range(4):
            if self.dirs[b] is None:
                self.dirs[b] = missing.pop()
        self.model, self.velocity = fit(self.history, self.dirs)

    # --- control -------------------------------------------------------------------------- #
    def choose(self):
        model, grid = self.model, self.grid
        p, v, heading = self.pose[:2], self.velocity, self.pose[2]
        commands = model.speed*np.array([V.rot(heading)@d for d in self.dirs])        # (4, 2), map frame
        targets = commands[self.sequences]
        count = len(self.sequences)
        route = grid.descend(self.field, p, AHEAD)
        aim = route[-1]
        if np.hypot(*(aim-self.pad)) < .15:
            aim = self.pad
        pos = np.broadcast_to(p, (count, 2)).copy()
        vel = np.broadcast_to(v, (count, 2)).copy()
        clear = np.full(count, np.inf)
        cost = np.zeros(count)
        arrived = np.zeros(count, bool)
        for k in range(HORIZON):
            u = targets[:, k]
            pts = pos[:, None, :]+model.C[None, :, None]*vel[:, None, :]+model.D[None, :, None]*u[:, None, :]
            r, c = grid.fine_cells(pts[..., 0], pts[..., 1])
            room = self.clearance[r, c].min(axis=1)
            clear = np.where(arrived, clear, np.minimum(clear, room))
            pos = pts[:, -1]
            vel = model.cv*vel+model.dv*u
            left = np.hypot(pos[:, 0]-aim[0], pos[:, 1]-aim[1])+CLOSE*np.clip(1-room/ROOM, 0, 1)
            cost += np.where(arrived, 0., left)
            arrived |= np.hypot(pos[:, 0]-self.pad[0], pos[:, 1]-self.pad[1]) < .18
        rest = pos+vel*model.coast
        r, c = grid.fine_cells(rest[:, 0], rest[:, 1])
        clear = np.where(arrived, clear, np.minimum(clear, self.clearance[r, c]))
        cost = np.where(clear < 0, np.inf, cost)
        safe = np.ones(4, bool)
        for b in range(4):
            for turn in (-SLIP_TURN, 0., SLIP_TURN):
                u = (1-SLIP_LOSS)*(V.rot(turn)@commands[b])
                pts = p+model.C[:, None]*v+model.D[:, None]*u
                end = model.cv*v+model.dv*u
                pts = np.vstack([pts, pts[-1]+end*model.coast])
                r, c = grid.fine_cells(pts[:, 0], pts[:, 1])
                if self.clearance[r, c].min() < -.02:
                    safe[b] = False
        cost = np.where(safe[self.sequences[:, 0]], cost, np.inf)
        if np.isfinite(cost).any():
            return int(self.sequences[int(np.argmin(cost)), 0])
        return int(self.sequences[int(np.argmax(clear)), 0])

    def run(self):
        self.identify()
        k = 0
        while not self.obs["done"]:
            button = self.choose()
            if self.press(button) is None:
                break
            k += 1
            if k <= 30 or k % 5 == 0:
                self.model, self.velocity = fit(self.history, self.dirs)
            else:
                # between fits: the model's velocity, corrected for a slip by what the press actually did
                b, heading, moved, slipped = self.history[-1]
                u = self.model.speed*(V.rot(heading)@self.dirs[b])
                if slipped:
                    unit = u/self.model.speed
                    grip = np.clip(((moved-self.model.C_all[-1]*self.velocity)@unit)/(self.model.D_all[-1]*self.model.speed), .05, 1.)
                    u = u*grip
                self.velocity = self.model.cv*self.velocity+self.model.dv*u
        return self.obs
