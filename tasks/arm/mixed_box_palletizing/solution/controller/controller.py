"""Privileged reference controller for mixed_box_palletizing.

It is the reference the task is scored against: a controller with perfect calibration and perfect
perception. The grader adds `truth` to every observation of a run that presents the reference's
token (tests/verify.py): the hidden joint zero offsets, the pallet's real pose, and the pose, size,
mass, centre of gravity and handling limits of every carton that has arrived. Without `truth` it
holds still. What it does with the truth is ordinary: an online planner (it never knows a carton
before it arrives) and a state machine of motion primitives, closed on the carton's true pose.

Structure: a queue of small primitives (move, descend until contact, wait, ...) run by `act()`,
and a planner (`Packer`) that decides where each carton goes. The whole controller state is one
picklable object that is copied before every step, so a step index asked twice is answered from
the same state both times.
"""

import copy
import json
import math
import sys


import numpy as np


class pc(object):
    """The numbers of the cell this controller uses, from task_card.md."""
    G = 9.81
    CONTROL_DT = 0.1
    STATION = np.array([1.45, 0.0])
    BELT_Z = 0.60
    TABLE_Z = 0.60
    PALLET = np.array([0.0, 1.50])
    PALLET_HALF = np.array([0.60, 0.40])
    DECK_Z = 0.144
    STACK_HEIGHT_MAX = 1.00
    LOAD_UNMARKED_KG = 80.0


# The arm's drawing (task_card.md).
A0, H0, L1, L2, A3, D_TOOL = 0.25, 0.80, 1.20, 1.30, 0.20, 0.55


def fk(q):
    q1, q2, q3, q4 = (float(v) for v in q[:4])
    phi = q2 + q3
    r = A0 + L1 * math.sin(q2) + L2 * math.cos(phi) + A3
    z = H0 + L1 * math.cos(q2) - L2 * math.sin(phi) - D_TOOL
    return [r * math.cos(q1), r * math.sin(q1), z, q1 + q4]


DT = pc.CONTROL_DT
GAP = 0.004                      # m between neighbouring cartons on the pallet
DECK_U, DECK_V = 2 * pc.PALLET_HALF[0], 2 * pc.PALLET_HALF[1]
ORIGIN = np.array([pc.PALLET[0] - pc.PALLET_HALF[0], pc.PALLET[1] - pc.PALLET_HALF[1]])
LOAD_MARGIN = 0.80               # plan with this fraction of a printed stacking limit
SUPPORT_MIN = 0.60               # fraction of a carton's base that must rest on something
SLOTS = [(-1.72, -0.36), (-1.72, 0.36), (-1.17, -0.36), (-1.17, 0.36)]   # buffer table slots
# Weights of the placement score (tuned on random practice orders with an offline simulation).
WEIGHTS = {"z": 1.0, "deck20": 0.7, "deck10": 1.6, "side": 0.5, "edge": 0.25, "flush": 0.06, "support": 0.5,
           "corner": 0.05, "far": 0.15}
WRIST_CLEAR = 0.40               # m: the arm passes this far above the top of the carton it sets down


def log(*args):
    print("[controller]", *args, file=sys.stderr, flush=True)


# --------------------------------------------------------------------------------------------
# Packing
# --------------------------------------------------------------------------------------------

class Packer(object):
    """Where cartons stand on the pallet. Pallet frame: u along world x, v along world y, origin
    at the pallet's corner, z above the deck. Cartons are axis-aligned."""

    def __init__(self):
        self.items = []     # dicts: u0, v0, u1, v1, z0, z1, mass, limit, load, below [(index, share)]

    @staticmethod
    def _overlap(a, u0, v0, u1, v1):
        du = min(a["u1"], u1) - max(a["u0"], u0)
        dv = min(a["v1"], v1) - max(a["v0"], v0)
        return du * dv if du > 0 and dv > 0 else 0.0

    def _added_load(self, supports, mass, acc=None):
        """Extra load on every carton below when `mass` is put on `supports` [(index, share)]."""
        acc = {} if acc is None else acc
        for index, share in supports:
            # A stack does not spread a load by area: a narrow strip under an edge can carry half
            # of it. Plan with 1.5 times the area share, and at least half.
            extra = mass * min(1.0, max(1.5 * share, 0.5))
            acc[index] = acc.get(index, 0.0) + extra
            self._added_load(self.items[index]["below"], extra, acc)
        return acc

    def candidates(self, du, dv, height, mass, com):
        """Feasible resting places for a du x dv footprint: list of (u0, v0, z0, supports)."""
        us = {0.0, DECK_U - du}
        vs = {0.0, DECK_V - dv}
        for it in self.items:
            us.update((it["u1"] + GAP, it["u0"] - GAP - du, it["u0"], it["u1"] - du))
            vs.update((it["v1"] + GAP, it["v0"] - GAP - dv, it["v0"], it["v1"] - dv))
        out = []
        area = du * dv
        for u0 in us:
            if u0 < -1e-9 or u0 + du > DECK_U + 1e-9:
                continue
            for v0 in vs:
                if v0 < -1e-9 or v0 + dv > DECK_V + 1e-9:
                    continue
                u1, v1 = u0 + du, v0 + dv
                # Keep the clearance to every neighbour that reaches above the resting height.
                under = [(i, self._overlap(it, u0 - GAP + 1e-6, v0 - GAP + 1e-6, u1 + GAP - 1e-6, v1 + GAP - 1e-6))
                         for i, it in enumerate(self.items)]
                touching = [(i, a) for i, a in under if a > 0]
                real = [(i, self._overlap(self.items[i], u0, v0, u1, v1)) for i, _ in touching]
                z0 = max([self.items[i]["z1"] for i, a in real if a > 0] or [0.0])
                if any(self.items[i]["z1"] > z0 + 1e-6 for i, _ in touching):
                    continue        # a neighbour inside the clearance stands higher: no room to lower it
                if z0 + height > pc.STACK_HEIGHT_MAX - 0.004:
                    continue
                if z0 <= 1e-9:
                    supports, fraction = [], 1.0
                    su0, sv0, su1, sv1 = u0, v0, u1, v1
                else:
                    sup = [(i, a) for i, a in real if a > 0 and abs(self.items[i]["z1"] - z0) < 0.004]
                    fraction = sum(a for _, a in sup) / area
                    if fraction < SUPPORT_MIN:
                        continue
                    supports = [(i, a / (fraction * area)) for i, a in sup]
                    su0 = min(max(self.items[i]["u0"], u0) for i, _ in sup)
                    su1 = max(min(self.items[i]["u1"], u1) for i, _ in sup)
                    sv0 = min(max(self.items[i]["v0"], v0) for i, _ in sup)
                    sv1 = max(min(self.items[i]["v1"], v1) for i, _ in sup)
                cu, cv = (u0 + u1) / 2 + com[0], (v0 + v1) / 2 + com[1]
                if not self._stands(cu, cv, z0, height, mass, supports, (su0, sv0, su1, sv1), (u0, v0, u1, v1)):
                    continue
                if not self._reachable((u0 + u1) / 2, (v0 + v1) / 2, z0 + height):
                    continue
                extra = self._added_load(supports, mass)
                if any(self.items[i]["load"] + e > LOAD_MARGIN * self.items[i]["limit"] + 1e-9 for i, e in extra.items()):
                    continue
                out.append((u0, v0, z0, supports, fraction))
        return out

    def _stands(self, cu, cv, z0, height, mass, supports, region, foot):
        """Whether a carton with its centre of gravity at (cu, cv), and the column it would stand
        on, stay put when the pallet is tilted: at every interface down the column, the centre of
        gravity of everything above it must lie inside the area it rests on, by a margin that
        grows with its height above that interface."""
        m, mu, mv, mz = mass, mass * cu, mass * cv, mass * (z0 + height / 2)
        level = z0
        while True:
            gu, gv, gz = mu / m, mv / m, mz / m
            margin = 0.035 + 0.20 * (gz - level)
            if not (region[0] + margin <= gu <= region[2] - margin and region[1] + margin <= gv <= region[3] - margin):
                return False
            if len(supports) != 1:
                return True         # on the deck, or spread over several cartons
            it = self.items[supports[0][0]]
            m, mu, mv, mz = m + it["mass"], mu + it["mass"] * it["cu"], mv + it["mass"] * it["cv"], \
                mz + it["mass"] * (it["z0"] + it["z1"]) / 2
            level = it["z0"]
            supports = it["below"]
            if not supports:
                return True
            if len(supports) == 1:
                low = self.items[supports[0][0]]
                region = (max(it["u0"], low["u0"]), max(it["v0"], low["v0"]), min(it["u1"], low["u1"]), min(it["v1"], low["v1"]))
            else:
                region = (it["u0"], it["v0"], it["u1"], it["v1"])

    def _reachable(self, cu, cv, top):
        """Whether the arm can lower a carton to a place whose top ends at `top`: the wrist and
        the forearm come in over everything between the base and the place, WRIST_CLEAR above
        the carton's top."""
        bu, bv = -ORIGIN[0], -ORIGIN[1]            # the arm's base in the pallet frame
        length = math.hypot(cu - bu, cv - bv)
        n = int(length / 0.04) + 1
        for it in self.items:
            if it["z1"] <= top + WRIST_CLEAR:
                continue
            for k in range(n + 1):
                f = k / float(n)
                pu, pv = bu + (cu - bu) * f, bv + (cv - bv) * f
                if it["u0"] - 0.13 <= pu <= it["u1"] + 0.13 and it["v0"] - 0.13 <= pv <= it["v1"] + 0.13:
                    return False
        return True

    def best(self, L, W, height, mass, coms, limit=80.0):
        """The best resting place over both orientations: (score, quarter, u0, v0, z0, supports).
        coms[quarter] is the centre of gravity's offset in the pallet frame for that orientation."""
        best = None
        for quarter in (0, 1):
            du, dv = (L, W) if quarter == 0 else (W, L)
            c = coms[quarter]
            for u0, v0, z0, supports, fraction in self.candidates(du, dv, height, mass, c):
                u1, v1 = u0 + du, v0 + dv
                # Contact with walls of neighbours of at least the same height, and flush tops.
                side, flush = 0.0, 0.0
                for it in self.items:
                    if it["z1"] <= z0 + 0.02 or it["z0"] >= z0 + height:
                        continue
                    if abs(it["u1"] + GAP - u0) < 1e-6 or abs(it["u0"] - GAP - u1) < 1e-6:
                        side += max(0.0, min(it["v1"], v1) - max(it["v0"], v0))
                    if abs(it["v1"] + GAP - v0) < 1e-6 or abs(it["v0"] - GAP - v1) < 1e-6:
                        side += max(0.0, min(it["u1"], u1) - max(it["u0"], u0))
                    if abs(it["z1"] - (z0 + height)) < 0.012:
                        flush += 1.0
                edge = (du if v0 < 1e-6 or v1 > DECK_V - 1e-6 else 0.0) + (dv if u0 < 1e-6 or u1 > DECK_U - 1e-6 else 0.0)
                # The deck is for cartons that can carry a stack: a weak carton on the deck wastes
                # the column above it, so it prefers the top of something that can carry it.
                deck = 0.0
                if z0 <= 1e-9:
                    deck = 0.0 if limit >= 40.0 else (WEIGHTS["deck20"] if limit >= 20.0 else WEIGHTS["deck10"])
                score = -WEIGHTS["z"] * z0 - deck + WEIGHTS["side"] * side + WEIGHTS["edge"] * edge \
                    + WEIGHTS["flush"] * flush + WEIGHTS["support"] * (fraction - 1.0) \
                    - WEIGHTS["corner"] * u0 + WEIGHTS["far"] * v0
                if best is None or score > best[0]:
                    best = (score, quarter, u0, v0, z0, supports)
        return best

    def add(self, du, dv, u0, v0, z0, height, mass, limit, supports, com=(0.0, 0.0)):
        for index, extra in self._added_load(supports, mass).items():
            self.items[index]["load"] += extra
        self.items.append({"u0": u0, "v0": v0, "u1": u0 + du, "v1": v0 + dv, "z0": z0, "z1": z0 + height,
                           "mass": mass, "limit": limit, "load": 0.0, "below": supports,
                           "cu": u0 + du / 2 + com[0], "cv": v0 + dv / 2 + com[1]})

    def top(self):
        return max([it["z1"] for it in self.items] or [0.0])


# --------------------------------------------------------------------------------------------
# The controller
# --------------------------------------------------------------------------------------------

class State(object):
    def __init__(self):
        self.queue = []
        self.cmd = np.zeros(4)          # last velocity command
        self.vacuum = 0
        self.packer = Packer()
        self.carton = None              # the carton being handled
        self.buffer = [None] * len(SLOTS)
        self.fz = []                    # recent Fz readings
        self.tq = []                    # recent (Tx, Ty) readings
        self.tare = None
        self.wait = 0
        self.handled = 0
        self.flush = False
        self.finished = False
        self.counter = 0
        self.discard_z = [0.0, 0.0, 0.0]
        self.discards = 0


def nearest_yaw(yaw, reference):
    """The yaw equal to `yaw` modulo pi that is nearest to `reference` (the pad is symmetric)."""
    return yaw + math.pi * round((reference - yaw) / math.pi)


class Controller(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.s = State()
        self.saved = None
        self.saved_t = None
        self.s.queue = [("wait", 3), ("tare",), ("idle",)]

    # ---- one step ------------------------------------------------------------------------

    def act(self, t, obs):
        if self.saved_t == t and self.saved is not None:
            self.s = copy.deepcopy(self.saved)      # the same step asked again: answer from the same state
        else:
            self.saved = copy.deepcopy(self.s)
            self.saved_t = t
        s = self.s
        truth = obs.get("truth")
        if not truth:               # not a token run: the reference needs the truth
            return [0.0, 0.0, 0.0, 0.0, 0, 0, 0]
        self.truth = truth
        self.pxy = np.array(truth["pallet_xy"])
        self.pyaw = float(truth["pallet_yaw"])
        obs = dict(obs, tcp=fk(np.array(obs["q"]) - np.array(truth["q_off"])))
        s.fz.append(obs["wrench"][2])
        s.fz = s.fz[-8:]
        s.tq.append((obs["wrench"][3], obs["wrench"][4]))
        s.tq = s.tq[-8:]
        vel, cams, done = np.zeros(4), 0, 0
        for _ in range(60):
            if not s.queue:
                s.queue.append(("idle",))
            head = s.queue[0]
            result = getattr(self, "_" + head[0])(obs, *head[1:])
            if result is None:
                s.queue.pop(0)
                continue
            if result == "again":
                continue
            vel, cams, done = result
            break
        return [float(vel[0]), float(vel[1]), float(vel[2]), float(vel[3]), int(s.vacuum), int(cams), int(done)]

    # ---- motion --------------------------------------------------------------------------

    def _velocity(self, obs, goal, yaw, vmax, amax, final):
        """One velocity command toward `goal`. The height is always approached with braking; the
        horizontal motion brakes only at a final point and runs through a via point at speed.
        Every axis changes by at most amax * DT per step."""
        s = self.s
        tcp = np.array(obs["tcp"])
        err = np.array(goal) - tcp[:3]
        dist = float(np.linalg.norm(err))
        horizontal = float(np.linalg.norm(err[:2]))
        brake = lambda e: min(vmax, 2.5 * e, math.sqrt(2.0 * 0.5 * amax * e))
        speed = brake(horizontal) if final else vmax
        v = np.zeros(3)
        if horizontal > 1e-9:
            v[:2] = err[:2] / horizontal * speed
        v[2] = math.copysign(brake(abs(err[2])), err[2])
        step = amax * DT
        out = np.zeros(4)
        out[:3] = s.cmd[:3] + np.clip(v - s.cmd[:3], -step, step)
        yerr = yaw - tcp[3]
        w = max(-1.2, min(1.2, 2.0 * yerr))
        out[3] = s.cmd[3] + max(-0.4, min(0.4, w - s.cmd[3]))
        s.cmd = out
        return dist, abs(yerr)

    def _move(self, obs, goal, yaw, vmax, amax, tol, final):
        dist, yerr = self._velocity(obs, goal, yaw, vmax, amax, final)
        if dist < tol and (not final or (yerr < 0.006 and np.linalg.norm(self.s.cmd[:3]) < 0.03)):
            if final:
                self.s.cmd = np.zeros(4)
            return None
        return self.s.cmd.copy(), 0, 0

    def _wait(self, obs, n):
        s = self.s
        s.cmd = np.zeros(4)
        if s.wait >= n:
            s.wait = 0
            return None
        s.wait += 1
        return np.zeros(4), 0, 0

    def _vac(self, obs, on):
        self.s.vacuum = on
        return None

    def _tare(self, obs):
        self.s.tare = float(np.mean(self.s.fz[-3:]))
        return None

    def path(self, obs_tcp, goal, yaw, z_travel, vmax, amax):
        """Primitives that take the pad from where it is to `goal`: straight up to the travel
        height, around the base at that height, then to a point above the goal."""
        start = np.array(obs_tcp[:3])
        goal = np.array(goal)
        steps = [("move", [start[0], start[1], z_travel], obs_tcp[3], vmax, amax, 0.05, False)]
        a0, a1 = math.atan2(start[1], start[0]), math.atan2(goal[1], goal[0])
        if a0 < -1.0:
            a0 += 2 * math.pi
        if a1 < -1.0:
            a1 += 2 * math.pi
        r0, r1 = math.hypot(start[0], start[1]), math.hypot(goal[0], goal[1])
        n = max(1, int(math.ceil(abs(a1 - a0) / 0.30)))
        for k in range(1, n + 1):
            f = k / float(n)
            a, r = a0 + (a1 - a0) * f, r0 + (r1 - r0) * f
            steps.append(("move", [r * math.cos(a), r * math.sin(a), z_travel], yaw, vmax, amax, 0.12 if k < n else 0.004,
                          k == n))
        return steps

    # ---- the cycle -----------------------------------------------------------------------

    def _idle(self, obs):
        """Nothing in hand: take the next carton from the station, or empty the buffer, or stop."""
        s = self.s
        if obs["infeed_present"]:
            s.queue[0:1] = [("clear_view", obs["tcp"][:]), ("wait", 4), ("look",)]
            return "again"
        if obs["delivered"] >= obs["total"] or s.flush:
            index = self._next_from_buffer(final=obs["delivered"] >= obs["total"])
            if index is not None:
                s.queue[0:1] = self._pick_from_buffer(obs, index)
                return "again"
            if obs["delivered"] >= obs["total"]:
                if not s.finished:
                    s.finished = True
                    home = [1.0, 1.0, 1.9]
                    s.queue[0:1] = [("move", [obs["tcp"][0], obs["tcp"][1], 1.9], obs["tcp"][3], 0.8, 1.5, 0.05, True),
                                    ("wait", 5), ("done",)]
                    return "again"
        s.cmd = np.zeros(4)
        return np.zeros(4), 0, 0

    def _done(self, obs):
        return np.zeros(4), 0, 1

    def _clear_view(self, obs, tcp):
        """Move the arm out of the infeed camera's view if it is in it."""
        r = math.hypot(tcp[0] - pc.STATION[0], tcp[1] - pc.STATION[1])
        if r > 0.75:
            return None
        self.s.queue[0:1] = [("move", [tcp[0], tcp[1], max(tcp[2], 1.5)], tcp[3], 0.8, 2.0, 0.05, False),
                             ("move", [0.95, 0.95, max(tcp[2], 1.5)], tcp[3], 0.8, 2.0, 0.05, True)]
        return "again"

    def _look(self, obs):
        """Ask for the infeed image; the next step reads it."""
        self.s.queue[0:1] = [("read",)]
        return np.zeros(4), 0, 0

    def _read(self, obs):
        s = self.s
        carton = self._station_carton()
        if carton is None:
            s.counter += 1
            s.queue[0:1] = [("wait", 3), ("look",)] if s.counter < 5 else [("wait", 20), ("idle",)]
            return "again"
        s.counter = 0
        # The buffer is a sorting window: when it is full and holds a carton stronger than the
        # one that just arrived, that one goes to the pallet first.
        if all(b is not None for b in s.buffer):
            seen = dict(carton, tool_yaw=carton["yaw"])
            index = self._next_from_buffer(stronger_than=self._strength(seen))
            if index is None and self._plan(seen)[0] is None:
                index = self._next_from_buffer()        # no place for the new one: free a slot for it
            if index is not None:
                s.queue[0:1] = self._pick_from_buffer(obs, index)
                return "again"
        log("carton", s.handled, {k: (np.round(v, 3).tolist() if isinstance(v, np.ndarray) else v) for k, v in carton.items()})
        tcp = obs["tcp"]
        yaw = nearest_yaw(carton["yaw"], math.atan2(carton["pick"][1], carton["pick"][0]))
        carton["tool_yaw"] = yaw
        carton["offset"] = (carton["pick"] - carton["centre"]).tolist()     # pick point from the centre, world
        z_travel = max(tcp[2], carton["top"] + 0.25, pc.DECK_Z + s.packer.top() + 0.15)
        above = [carton["pick"][0], carton["pick"][1], carton["top"] + 0.10]
        s.carton = carton
        s.queue[0:1] = self.path(tcp, [above[0], above[1], z_travel], yaw, z_travel, 1.0, 2.5) + [
            ("move", above, yaw, 0.8, 2.5, 0.006, True),
            ("touch", carton["top"]), ("vac", 1), ("wait", 2), ("sealed",),
            ("move", [above[0], above[1], carton["top"] + 0.07], yaw, 0.05, 0.4, 0.008, True),
            ("wait", 6), ("weigh",), ("balance", 0), ("decide",)]
        return "again"

    def _from_truth(self, c):
        """The carton dict the planner uses, from one entry of the truth."""
        L, W, H = c["size"]
        yaw = c["yaw"]
        com = np.array(c["com"], dtype=float)
        grip = np.array([max(-L / 2 + 0.08, min(L / 2 - 0.08, com[0])), max(-W / 2 + 0.06, min(W / 2 - 0.06, com[1]))])
        ca, sa = math.cos(yaw), math.sin(yaw)
        centre = np.array(c["pos"][:2])
        pick = centre + np.array([ca * grip[0] - sa * grip[1], sa * grip[0] + ca * grip[1]])
        return {"index": c["index"], "centre": centre, "yaw": yaw, "L": L, "W": W, "H": H,
                "top": c["pos"][2] + H / 2, "limit": c["stack_limit"], "fragile": c["fragile"], "pick": pick,
                "com": com, "mass": c["mass"]}

    def _station_carton(self):
        for c in self.truth["cartons"]:
            if not c["held"] and abs(c["pos"][0] - pc.STATION[0]) < 0.45 and abs(c["pos"][1] - pc.STATION[1]) < 0.40 \
                    and c["pos"][2] < pc.BELT_Z + 0.45:
                return self._from_truth(c)
        return None

    def _align(self, obs, centre, yaw_goal):
        """Bring the held carton's true centre over `centre` and its long axis to yaw_goal."""
        s = self.s
        held = [c for c in self.truth["cartons"] if c["held"]]
        if not held:
            s.cmd = np.zeros(4)
            return None
        c = held[0]
        e = np.array(centre) - np.array(c["pos"][:2])
        ey = yaw_goal - c["yaw"]
        ey -= math.pi * round(ey / math.pi)
        if float(np.linalg.norm(e)) < 0.0007 and abs(ey) < 0.0012:
            s.wait += 1
            if s.wait >= 2:
                s.wait = 0
                s.cmd = np.zeros(4)
                return None
        else:
            s.wait = 0
        v = np.clip(2.5 * e, -0.06, 0.06)
        w = max(-0.25, min(0.25, 2.5 * ey))
        s.cmd = np.array([v[0], v[1], 0.0, w])
        return s.cmd.copy(), 0, 0

    def _descend(self, obs, centre, yaw_goal, z_stop):
        """Straight down to z_stop with the held carton's true centre and yaw kept on target."""
        s = self.s
        held = [c for c in self.truth["cartons"] if c["held"]]
        ez = obs["tcp"][2] - z_stop
        if ez < 0.004 or not held:
            s.cmd = np.zeros(4)
            return None
        c = held[0]
        e = np.array(centre) - np.array(c["pos"][:2])
        ey = yaw_goal - c["yaw"]
        ey -= math.pi * round(ey / math.pi)
        v = np.clip(2.5 * e, -0.06, 0.06)
        vz = -min(0.30, 1.5 * ez + 0.01)
        if float(np.linalg.norm(e)) > 0.002:
            vz *= 0.3               # let the lateral error settle before going deeper
        vz = max(vz, s.cmd[2] - 0.08)
        s.cmd = np.array([v[0], v[1], vz, max(-0.25, min(0.25, 2.5 * ey))])
        return s.cmd.copy(), 0, 0

    def _rise(self, obs, z):
        """Straight up to z, without any sideways motion: neighbours stand 4 mm away."""
        s = self.s
        if obs["tcp"][2] >= z - 0.01:
            s.cmd = np.zeros(4)
            return None
        s.cmd = np.array([0.0, 0.0, min(0.5, s.cmd[2] + 0.15), 0.0])
        return s.cmd.copy(), 0, 0

    def _touch(self, obs, top):
        """Lower the pad until the cup's spring is compressed by about 10 mm."""
        s = self.s
        if obs["wrench"][2] - s.tare > 10.0 or obs["tcp"][2] < top - 0.02:
            s.cmd = np.zeros(4)
            return None
        s.cmd = np.array([0.0, 0.0, -0.10 if obs["tcp"][2] > top + 0.03 else -0.05, 0.0])
        return s.cmd.copy(), 0, 0

    def _sealed(self, obs):
        s = self.s
        if obs["sealed"]:
            return None
        s.counter += 1
        if s.counter > 3:
            s.counter = 0
            s.vacuum = 0
            log("no seal: looking again")
            tcp = obs["tcp"]
            s.queue = [("move", [tcp[0], tcp[1], tcp[2] + 0.3], tcp[3], 0.5, 2.0, 0.03, True), ("idle",)]
            return "again"
        s.queue[0:1] = [("vac", 0), ("wait", 1), ("vac", 1), ("wait", 2), ("sealed",)]
        return "again"

    def _weigh(self, obs):
        s = self.s
        fz = float(np.mean(s.fz[-4:]))
        s.carton["hang"] = fz
        log("mass", round(s.carton["mass"], 2))
        return None

    def _balance(self, obs, attempt):
        """Nothing printed says where a carton's centre of gravity is, and the seal holds little
        moment. The torque a hanging carton puts on the wrist tells how far its centre of gravity
        is from the pad: if that is more than 20 mm, set it down and take it there."""
        s = self.s
        carton = s.carton
        weight = carton["mass"] * pc.G
        tx = float(np.mean([t[0] for t in s.tq[-4:]]))
        ty = float(np.mean([t[1] for t in s.tq[-4:]]))
        off = np.array([ty / weight, -tx / weight])          # centre of gravity from the pad centre, world xy
        ca, sa = math.cos(carton["yaw"]), math.sin(carton["yaw"])
        held = np.array(carton["offset"])                    # pad centre from the carton's centre, world xy
        cog = held + off
        local = np.array([ca * cog[0] + sa * cog[1], -sa * cog[0] + ca * cog[1]])
        log("centre of gravity", np.round(local, 3).tolist(), "from the pad by", np.round(off, 3).tolist())
        if True:                    # the grip is already at the true centre of gravity
            return None
        local[0] = max(-carton["L"] / 2 + 0.08, min(carton["L"] / 2 - 0.08, local[0]))
        local[1] = max(-carton["W"] / 2 + 0.06, min(carton["W"] / 2 - 0.06, local[1]))
        new = np.array([ca * local[0] - sa * local[1], sa * local[0] + ca * local[1]])
        carton["offset"] = new.tolist()
        carton["pick"] = np.array(carton["centre"]) + new
        yaw = carton["tool_yaw"]
        above = [carton["pick"][0], carton["pick"][1], carton["top"] + 0.10]
        s.queue[0:1] = [("lower", carton["top"], carton["hang"], carton["mass"], 0.04), ("vac", 0), ("wait", 3),
                        ("move", [obs["tcp"][0], obs["tcp"][1], carton["top"] + 0.10], yaw, 0.4, 2.0, 0.02, True),
                        ("move", above, yaw, 0.4, 2.0, 0.005, True),
                        ("touch", carton["top"]), ("vac", 1), ("wait", 2), ("sealed",),
                        ("move", [above[0], above[1], carton["top"] + 0.07], yaw, 0.05, 0.4, 0.008, True),
                        ("wait", 6), ("weigh",), ("balance", attempt + 1)]
        return "again"

    # ---- decisions -----------------------------------------------------------------------

    @staticmethod
    def _strength(carton):
        """Sorting key: cartons that carry more go to the pallet first, larger ones before smaller."""
        limit = 80.0 if carton["limit"] is None else carton["limit"]
        return limit + carton["L"] * carton["W"]

    @staticmethod
    def _turn(carton, quarter_yaw, reference):
        """How far the tool turns between the pick and a set-down with the carton's long axis at
        quarter_yaw, the tool's yaw staying near `reference`."""
        tool_yaw = nearest_yaw(carton["tool_yaw"] + quarter_yaw - carton["yaw"], reference)
        return tool_yaw - carton["tool_yaw"]

    def _plan(self, carton):
        limit = pc.LOAD_UNMARKED_KG if carton["limit"] is None else max(carton["limit"], 0.0)
        com = carton.get("com", np.zeros(2))
        coms = []
        for quarter in (0, 1):
            a = carton["yaw"] + self._turn(carton, self.pyaw + quarter * math.pi / 2, math.pi / 2) - self.pyaw
            coms.append((math.cos(a) * com[0] - math.sin(a) * com[1], math.sin(a) * com[0] + math.cos(a) * com[1]))
        carton["coms"] = coms
        return self.s.packer.best(carton["L"], carton["W"], carton["H"], carton["mass"], coms, limit), limit

    def _decide(self, obs):
        """A carton hangs on the pad at the station (or at a buffer slot): pallet or buffer?"""
        s = self.s
        carton = s.carton
        if not obs["sealed"]:
            log("lost the carton after the lift")
            s.vacuum = 0
            s.queue = [("idle",)]
            return "again"
        plan, limit = self._plan(carton)
        strong = self._strength(carton)
        free = [i for i, b in enumerate(s.buffer) if b is None]
        from_buffer = carton.get("slot") is not None
        remaining = obs["total"] - obs["delivered"]
        # While stronger cartons may still come, a carton waits on the buffer unless it is of the
        # strongest kind; with the buffer full, the strongest of the window goes to the pallet.
        top_class = strong >= 40.0
        hold_back = (not from_buffer) and free and (plan is None or (remaining > 0 and not top_class))
        if plan is not None and not hold_back:
            s.queue[0:1] = self._place_on_pallet(obs, carton, plan, limit)
        elif from_buffer:
            s.queue[0:1] = self._put_back(obs, carton)
        elif free:
            s.queue[0:1] = self._place_on_buffer(obs, carton, free[0])
        else:
            # No room anywhere: set it down on the floor side of the table's far end.
            log("no place for carton", s.handled)
            s.queue[0:1] = self._discard(obs, carton)
        return "again"

    def _transfer(self, obs, carton, centre, quarter_yaw, z_rest, reference):
        """Primitives that carry the held carton so that its centre lands at `centre` (world xy),
        with its long axis at `quarter_yaw`, resting with its base at z_rest."""
        s = self.s
        tcp = obs["tcp"]
        mass = carton["mass"]
        vmax = 1.0 if mass < 8 else (0.85 if mass < 15 else 0.7)
        amax = 1.8 if mass < 8 else (1.2 if mass < 15 else 0.8)
        turn = self._turn(carton, quarter_yaw, reference)
        tool_yaw = carton["tool_yaw"] + turn
        c, sn = math.cos(turn), math.sin(turn)
        off = carton["offset"]
        pick = [centre[0] + c * off[0] - sn * off[1], centre[1] + sn * off[0] + c * off[1]]
        top = z_rest + carton["H"]
        z_travel = max(pc.DECK_Z + s.packer.top(), pc.TABLE_Z + 0.45, pc.BELT_Z + 0.45, z_rest) + carton["H"] + 0.12
        z_travel = min(max(z_travel, tcp[2]), 2.0)
        steps = self.path(tcp, [pick[0], pick[1], z_travel], tool_yaw, z_travel, vmax, amax)
        target = [float(centre[0]), float(centre[1])]
        steps += [("wait", 2), ("align", target, float(quarter_yaw)),
                  ("descend", target, float(quarter_yaw), top + 0.035),
                  ("align", target, float(quarter_yaw)),
                  ("lower", top, carton["hang"], mass, 0.03 if carton["fragile"] else 0.05),
                  ("vac", 0), ("wait", 2), ("rise", top + 0.12)]
        carton["new_yaw"] = carton["yaw"] + turn
        carton["new_tool_yaw"] = tool_yaw
        carton["new_centre"] = [float(centre[0]), float(centre[1])]
        carton["new_top"] = top
        return steps

    def _lower(self, obs, top, hang, mass, speed):
        """Lower the held carton until most of its weight rests on what is below."""
        s = self.s
        fz = obs["wrench"][2]
        if fz - hang > 0.6 * mass * pc.G + 3.0 or obs["tcp"][2] < top - 0.03 or not obs["sealed"]:
            s.cmd = np.zeros(4)
            return None
        s.cmd = np.array([0.0, 0.0, -speed, 0.0])
        return s.cmd.copy(), 0, 0

    def _place_on_pallet(self, obs, carton, plan, limit):
        s = self.s
        _, quarter, u0, v0, z0, supports = plan
        du, dv = (carton["L"], carton["W"]) if quarter == 0 else (carton["W"], carton["L"])
        local = np.array([u0 + du / 2, v0 + dv / 2]) - pc.PALLET_HALF
        cp, sp = math.cos(self.pyaw), math.sin(self.pyaw)
        centre = self.pxy + np.array([cp * local[0] - sp * local[1], sp * local[0] + cp * local[1]])
        steps = self._transfer(obs, carton, centre, self.pyaw + quarter * math.pi / 2, pc.DECK_Z + z0, math.pi / 2)
        s.packer.add(du, dv, u0, v0, z0, carton["H"], carton["mass"], limit, supports, carton["coms"][quarter])
        if carton.get("slot") is not None:
            s.buffer[carton["slot"]] = None
        s.handled += 1
        log("to pallet", [round(u0, 3), round(v0, 3), round(z0, 3)], "quarter", quarter)
        return steps + [("idle",)]

    def _place_on_buffer(self, obs, carton, slot):
        s = self.s
        centre = np.array(SLOTS[slot])
        steps = self._transfer(obs, carton, centre, math.pi / 2, pc.TABLE_Z, math.pi)
        kept = dict(carton)
        kept["slot"] = slot
        s.buffer[slot] = kept
        s.handled += 1
        log("to buffer slot", slot)
        return steps + [("stored", slot), ("idle",)]

    def _stored(self, obs, slot):
        """Remember where the buffered carton stands now (for the pick that takes it back)."""
        b = self.s.buffer[slot]
        if b is not None:
            turn = b["new_tool_yaw"] - b["tool_yaw"]
            b["yaw"] = b["new_yaw"]
            b["tool_yaw"] = b["new_tool_yaw"]
            b["centre"] = np.array(b["new_centre"])
            c, sn = math.cos(turn), math.sin(turn)
            off = b["offset"]
            b["offset"] = [c * off[0] - sn * off[1], sn * off[0] + c * off[1]]
            b["pick"] = b["centre"] + np.array(b["offset"])
            b["top"] = b["new_top"]
            for c in self.truth["cartons"]:
                if c["index"] == b.get("index"):
                    fresh = self._from_truth(c)
                    fresh["yaw"] = nearest_yaw(fresh["yaw"], b["yaw"])
                    b["centre"], b["pick"], b["top"], b["yaw"] = fresh["centre"], fresh["pick"], fresh["top"], fresh["yaw"]
                    b["offset"] = (fresh["pick"] - fresh["centre"]).tolist()
        return None

    def _put_back(self, obs, carton):
        slot = carton["slot"]
        steps = self._transfer(obs, carton, np.array(SLOTS[slot]), math.pi / 2, pc.TABLE_Z, math.pi)
        carton["tried"] = len(self.s.packer.items)
        self.s.buffer[slot] = dict(carton)
        return steps + [("stored", slot), ("idle",)]

    def _discard(self, obs, carton):
        """Set a carton that has no place down on the floor, away from everything."""
        s = self.s
        s.handled += 1
        spots = [(28.0, 1.95), (45.0, 1.95), (135.0, 1.95)]
        k = s.discards % len(spots)
        angle = math.radians(spots[k][0])
        z = s.discard_z[k]
        s.discard_z[k] += carton["H"] + 0.01
        s.discards += 1
        centre = np.array([spots[k][1] * math.cos(angle), spots[k][1] * math.sin(angle)])
        return self._transfer(obs, carton, centre, angle, z, angle) + [("idle",)]

    def _next_from_buffer(self, final=False, stronger_than=None):
        """The buffered carton to try next: the strongest that has a place on the pallet."""
        s = self.s
        order = sorted([i for i, b in enumerate(s.buffer) if b is not None],
                       key=lambda i: -self._strength(s.buffer[i]))
        for i in order:
            b = s.buffer[i]
            if stronger_than is not None and self._strength(b) <= stronger_than:
                break
            if b.get("tried") == len(s.packer.items):
                continue
            plan, _ = self._plan(b)
            if plan is not None:
                return i
        return None

    def _pick_from_buffer(self, obs, index):
        s = self.s
        b = s.buffer[index]
        s.carton = dict(b)
        carton = s.carton
        tcp = obs["tcp"]
        above = [carton["pick"][0], carton["pick"][1], carton["top"] + 0.10]
        z_travel = max(tcp[2], carton["top"] + 0.25, pc.DECK_Z + s.packer.top() + 0.15)
        return self.path(tcp, [above[0], above[1], z_travel], carton["tool_yaw"], z_travel, 1.0, 2.5) + [
            ("move", above, carton["tool_yaw"], 0.8, 2.5, 0.006, True),
            ("touch", carton["top"]), ("vac", 1), ("wait", 2), ("sealed",),
            ("move", [above[0], above[1], carton["top"] + 0.07], carton["tool_yaw"], 0.05, 0.4, 0.008, True),
            ("wait", 4), ("decide",)]


def main():
    controller = Controller()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "reset":
            controller.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            print(json.dumps({"action": controller.act(msg["t"], msg["obs"])}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            break


if __name__ == "__main__":
    main()
