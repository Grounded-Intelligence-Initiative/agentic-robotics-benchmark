"""The data-hall cell's mechanisms (cell internals): the fixtures' catches and stays, the GPU trays' latch, the hand-over
of a seated part to a free body (and back), the slot rule every field-replaceable part obeys, the GPU modules' screws
and the torque driver, the fibre scope and the cleaner, the tool holders, the cables of unplugged connectors, the LEDs.

The slot rule (parts.py has the numbers). A part in a slot is held by its slot weld to the equipment. Latched, the weld
holds it at the seat. A hand that holds the part and pulls it out along the slot axis harder than the part's release
force (for 50 ms; while the part may be released: a transceiver whose connector is plugged, a power supply whose plug
is in, a GPU module with a screw still in do not let go) unlatches it; then the weld is re-anchored every tick at the
part's own axial position, so it keeps the part on the slot's axis and square to it while the hand slides it, until
the part leaves the slot's mouth and is free. A held part brought to a free slot of its kind with its frame within the
lateral and angular tolerance of the slot's frame at the mouth is taken by the slot and slides; pushed home and further
than its latch force it latches. One brought outside the tolerance (but within three times it) and pushed into the
mouth jams: pushing a jammed part harder than its damage force breaks the slot.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import hall as H, parts as P
from .grips import Grips, feature_score, rel_pose, weld_wrench

TICK = 0.01
PARK_Z = -30.0
LATCH_TICKS = 5                       # ticks of pull past a part's release force before its latch lets go
SERVICE_LOOP = 0.60                   # m of fibre beyond a trunk's drawn run (the slack left at its last tie)
# a C19 power cord and an MPO trunk are stiff enough over their last half metre that a plug / connector let go out
# of its port stays about where it was let go (no drop, no swing)
STIFF_CORD = {"gravcomp": 1.0, "damping": (3.0, 3.0, 3.0, 0.02, 0.02, 0.02), "kinds": ("plug", "mpo")}
SEAT_TICKS = 2                        # ticks of push past its latch force before it clicks home
LED_STATES = ("off", "green", "amber", "blue", "cyan", "red", "white")


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(-1))
    return q


def _mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _rot_angle(R):
    return float(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))


DOOR_ZONE = (1.1, 0.9)               # m: |x - doorway| and |y| within which a door's sensor sees someone
DOOR_HOLD_S, DOOR_SPEED = 2.0, 0.5    # s a door stays open after the zone clears; m/s of its leaves
SEAT_SOLREF = (0.01, 1.0)             # a latched part's weld in its slot
SLIDE_SOLREF = (0.03, 1.0)            # a part sliding in its slot's guides
SEAT_SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)
SLIDE_SOLIMP = (0.30, 0.40, 0.001, 0.5, 2.0)    # low impedance: the hand's force moves the part, the anchor guides it


class Ports:
    """Every port's state and its world frame now."""

    def __init__(self, m, d, meta):
        self.m, self.d, self.meta = m, d, meta
        B = mujoco.mjtObj.mjOBJ_BODY
        self.rec = meta["ports"]
        self.parent = {p: mujoco.mj_name2id(m, B, r["parent"]) for p, r in self.rec.items()}
        self.seat = {p: mujoco.mj_name2id(m, B, r["seat"]) for p, r in self.rec.items()}
        self.seat_pos0 = {p: m.body_pos[b].copy() for p, b in self.seat.items()}
        self.pos = {p: np.asarray(r["pos"], float) for p, r in self.rec.items()}
        self.R = {p: _mat(r["quat"]) for p, r in self.rec.items()}
        self.axis_local = {p: np.asarray(P.KINDS[r["kind"]]["axis"], float) for p, r in self.rec.items()}
        self.occupied = {p: True for p in self.rec}          # a part is in it (seated look or a proxy in the slot)
        self.proxy = {p: None for p in self.rec}             # the proxy in it, if the part is a free body now
        self.broken = {p: False for p in self.rec}
        self.instance = {p: p for p in self.rec}             # which part (an id the cluster model knows) sits there
        self.by_parent = {}
        for p, b in self.parent.items():
            self.by_parent.setdefault(b, []).append(p)

    def world(self, p):
        """(origin, rotation, out axis) of port p in the world now."""
        b = self.parent[p]
        Rb = self.d.xmat[b].reshape(3, 3)
        o = self.d.xpos[b] + Rb @ self.pos[p]
        R = Rb @ self.R[p]
        return o, R, R @ self.axis_local[p]

    def available(self, p):
        """A part may enter p: nothing in it, not broken, and what it depends on is in place."""
        if self.occupied[p] or self.broken[p]:
            return False
        need = self.rec[p].get("needs")
        return need is None or (self.occupied[need] and self.proxy[need] is None)

    def show_seat(self, p, on):
        b = self.seat[p]
        self.m.body_pos[b] = self.seat_pos0[p] if on else self.seat_pos0[p] + np.array([0.0, 0.0, -50.0])


class DCGrips(Grips):
    """The hand grips with the seat hand-over: a hand that closes on a seated part's grasp feature gets a free body of
    that part's kind put in its place (latched in the slot), and holds that."""

    def __init__(self, m, d, robot, meta, ports: Ports, pool):
        super().__init__(m, d, robot, meta)
        self.ports, self.pool = ports, pool
        self.seat_features = {p: P.KINDS[r["kind"]]["features"] for p, r in meta["ports"].items()}
        # a body's colliders are its group-3 geoms whatever their contact bits now (a parked proxy has none on); a
        # loose body let go gets the ordinary part bits back (not the bits it had when the cell was built)
        sub = {n: self._subtree(m, self.body[n]) for n in self.names}
        cols = {n: [g for g in range(m.ngeom) if m.geom_bodyid[g] in sub[n] and m.geom_group[g] == 3] for n in self.names}
        self.part_geoms = cols
        self.loose_geoms = {n: cols[n] for n in self.names if n in self.loose}
        self.saved_bits = {n: (np.full(len(gs), 4, dtype=m.geom_contype.dtype), np.full(len(gs), 1 | 2 | 4 | 8, dtype=m.geom_conaffinity.dtype))
                           for n, gs in self.loose_geoms.items()}

    def candidates(self, side):
        out = super().candidates(side)
        sid = self.r.grasp_site[side]
        p = self.d.site_xpos[sid].copy()
        y = self.d.site_xmat[sid].reshape(3, 3)[:, 1].copy()
        for port, feats in self.seat_features.items():
            if not self.ports.occupied[port] or self.ports.proxy[port] is not None:
                continue
            o, R, _ = self.ports.world(port)
            if float(np.linalg.norm(p - o)) > 0.35:
                continue
            pl, yl = R.T @ (p - o), R.T @ y
            best = None
            for f in feats:
                sc = feature_score(f, pl, yl)
                if sc is not None and (best is None or sc < best):
                    best = sc
            if best is not None:
                out.append((best, "seat:" + port, "cyl"))
        return sorted(out)

    def _engage(self, side, name, t):
        if name.startswith("seat:"):
            port = name[5:]
            px = self.pool.take_for(port)
            if px is None:
                return
            name = px
        super()._engage(side, name, t)


class Pool:
    """The free bodies that stand in for parts out of their slots, and the slot rule."""

    def __init__(self, m, d, meta, ports: Ports, events):
        self.m, self.d, self.meta, self.ports, self.events = m, d, meta, ports, events
        B, J, E = mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_EQUALITY
        self.rec = meta["proxies"]
        self.body = {n: mujoco.mj_name2id(m, B, n) for n in self.rec}
        self.qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, J, n + "_free")] for n in self.rec}
        self.vadr = {n: m.jnt_dofadr[mujoco.mj_name2id(m, J, n + "_free")] for n in self.rec}
        self.seat_eq = {n: mujoco.mj_name2id(m, E, r["seat_eq"]) for n, r in self.rec.items()}
        self.kind = {n: r["kind"] for n, r in self.rec.items()}
        self.state = {n: ("free" if r.get("spare") else "parked") for n, r in self.rec.items()}
        self.port = {n: None for n in self.rec}
        self.s = {n: 0.0 for n in self.rec}
        self.latched = {n: False for n in self.rec}
        self.count = {n: 0 for n in self.rec}
        self.jam = {n: 0 for n in self.rec}
        self.rearm = {n: True for n in self.rec}
        self.instance = {n: ("spare:" + n if r.get("spare") else None) for n, r in self.rec.items()}
        self.grips = None
        self.release_ok = lambda port: True            # the cluster's interlocks (screws, plugs) say whether it may go
        T = mujoco.mjtObj.mjOBJ_TENDON
        self.tendon = {n: mujoco.mj_name2id(m, T, r["tendon"]) for n, r in self.rec.items() if "tendon" in r}
        self.anchor = {n: m.body_mocapid[mujoco.mj_name2id(m, B, r["anchor_body"])] for n, r in self.rec.items() if "anchor_body" in r}
        self.chain = {n: [m.body_mocapid[mujoco.mj_name2id(m, B, c)] for c in r["chain"]] for n, r in self.rec.items() if "chain" in r}
        self.cable_len = {n: 0.0 for n in self.rec}
        self.end_site = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, r["end_site"]) for n, r in self.rec.items() if "end_site" in r}
        for n in self.rec:                     # parked bodies float below the floor: no contact (the floor is a plane)
            if self.state[n] == "parked":
                self._contacts(n, False)

    # ---------------------------------------------------------------- hand-over
    def take_for(self, port):
        """A free body of the port's kind put at the port's seat, latched; the seat's look hidden. Returns its name."""
        kind = self.ports.rec[port]["kind"]
        px = next((n for n in self.rec if self.kind[n] == kind and self.state[n] == "parked"), None)
        if px is None:
            self.events.append(("pool_empty", kind))
            return None
        o, R, _ = self.ports.world(port)
        a = self.qadr[px]
        self.d.qpos[a:a + 3] = o
        self.d.qpos[a + 3:a + 7] = _quat(R)
        self.d.qvel[self.vadr[px]:self.vadr[px] + 6] = 0.0
        self.m.body_gravcomp[self.body[px]] = STIFF_CORD["gravcomp"] if kind in STIFF_CORD["kinds"] else 0.0
        if kind in STIFF_CORD["kinds"]:
            self.m.dof_damping[self.vadr[px]:self.vadr[px] + 6] = STIFF_CORD["damping"]
        self._contacts(px, True)
        self.port[px] = port
        self.state[px] = "slot"
        self.latched[px] = True
        self.s[px] = 0.0
        self.instance[px] = self.ports.instance[port]
        self.ports.proxy[port] = px
        self._weld(px, port, 0.0)
        self.ports.show_seat(port, False)
        if px in self.tendon:
            self._cable_on(px, port)
        mujoco.mj_kinematics(self.m, self.d)
        return px

    def _contacts(self, px, on):
        b = self.body[px]
        for g in range(self.m.ngeom):
            if self.m.geom_bodyid[g] == b and self.m.geom_group[g] == 3:
                self.m.geom_contype[g] = 4 if on else 0
                self.m.geom_conaffinity[g] = (1 | 2 | 4 | 8) if on else 0

    def park(self, px):
        """Back to the pool (a part seated and let go is drawn by its seat again)."""
        eq = self.seat_eq[px]
        self.d.eq_active[eq] = 0
        a = self.qadr[px]
        self.d.qpos[a:a + 3] = self.rec[px]["park"]
        self.d.qpos[a + 3:a + 7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qvel[self.vadr[px]:self.vadr[px] + 6] = 0.0
        self.m.body_gravcomp[self.body[px]] = 1.0
        self._contacts(px, False)
        if px in self.tendon:
            self._cable_off(px)
        self.state[px] = "parked"
        self.port[px] = None
        self.instance[px] = None

    def _weld(self, px, port, s, hard=False):
        """The slot weld: the part held on the slot's axis, at axial position s from its seat. Stiff while latched;
        while the part slides in its guides the weld is soft (the guides' clearance: a hand that guides the part a
        millimetre off its axis meets a few newtons, not the full stiffness of a seated part)."""
        eq = self.seat_eq[px]
        m = self.m
        stiff = bool(self.latched.get(px)) or hard            # latched, or against the slot's end stop
        m.eq_solref[eq] = SEAT_SOLREF if stiff else SLIDE_SOLREF
        m.eq_solimp[eq] = SEAT_SOLIMP if stiff else SLIDE_SOLIMP
        m.eq_obj1id[eq] = self.ports.parent[port]
        rel_p = self.ports.pos[port] + self.ports.R[port] @ (self.ports.axis_local[port] * s)
        q = _quat(self.ports.R[port])
        m.eq_data[eq, 0:3] = 0.0
        m.eq_data[eq, 3:6] = rel_p
        m.eq_data[eq, 6:10] = q
        m.eq_data[eq, 10] = 1.0
        self.d.eq_active[eq] = 1

    # ---------------------------------------------------------------- cables
    def _cable_on(self, px, port):
        anc = np.asarray(self.ports.rec[port]["anchor"], float)
        b = self.ports.parent[port]
        Rb = self.d.xmat[b].reshape(3, 3)
        self.d.mocap_pos[self.anchor[px]] = self.d.xpos[b] + Rb @ anc
        self.cable_len[px] = float(self.ports.rec[port]["cable_len"]) * 1.15
        # a fibre trunk's last tie leaves a service loop: the connector reaches SERVICE_LOOP farther than the cable's
        # drawn run (power cords have none)
        loop = SERVICE_LOOP if self.kind[px] == "mpo" else 0.0
        self.m.tendon_range[self.tendon[px]] = [0.0, self.cable_len[px] + loop]

    def _cable_off(self, px):
        p = self.rec[px]["park"]
        self.d.mocap_pos[self.anchor[px]] = [p[0], p[1], PARK_Z + 1.0]
        self.m.tendon_range[self.tendon[px]] = [0.0, 50.0]
        for i, mid in enumerate(self.chain[px]):
            self.d.mocap_pos[mid] = [p[0], p[1], PARK_Z - 1.0 - 0.1 * i]

    def draw_cables(self):
        """Lay each live cable's chain along a sagging curve from its anchor to its connector (drawn only)."""
        for px, chain in self.chain.items():
            if self.state[px] == "parked":
                continue
            a = self.d.mocap_pos[self.anchor[px]].copy()
            e = self.d.site_xpos[self.end_site[px]].copy()
            L = max(self.cable_len[px], float(np.linalg.norm(e - a)) + 1e-3)
            chord = float(np.linalg.norm(e - a))
            sag = 0.5 * math.sqrt(max(0.0, L * L - chord * chord))
            mid = 0.5 * (a + e) + np.array([0.0, 0.0, -sag])
            ts = np.linspace(0.0, 1.0, 64)
            pts = ((1 - ts) ** 2)[:, None] * a + (2 * (1 - ts) * ts)[:, None] * mid + (ts ** 2)[:, None] * e
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            cum = np.concatenate([[0.0], np.cumsum(seg)])
            step = self.rec[px]["seg"]
            for i, mid_id in enumerate(chain):
                s0 = i * step
                if s0 + step > cum[-1] + 1e-6:
                    self.d.mocap_pos[mid_id] = [0.0, 0.0, PARK_Z - 2.0]
                    continue
                p0 = np.array([np.interp(s0, cum, pts[:, k]) for k in range(3)])
                p1 = np.array([np.interp(s0 + step, cum, pts[:, k]) for k in range(3)])
                c = 0.5 * (p0 + p1)
                z = p1 - p0
                n = float(np.linalg.norm(z))
                if n < 1e-9:
                    continue
                z /= n
                x = np.cross([0.0, 0.0, 1.0], z)
                if np.linalg.norm(x) < 1e-6:
                    x = np.array([1.0, 0.0, 0.0])
                x /= np.linalg.norm(x)
                y = np.cross(z, x)
                self.d.mocap_pos[mid_id] = c
                self.d.mocap_quat[mid_id] = _quat(np.stack([x, y, z], 1))

    # ---------------------------------------------------------------- the slot rule, every tick
    def tick(self, t):
        g = self.grips
        held_by = {}
        for side, st in g.state.items():
            if st["obj"] is not None:
                held_by.setdefault(st["obj"], []).append(side)
        for px in self.rec:
            st = self.state[px]
            if st == "slot":
                self._slot_tick(px, t, held_by.get(px, []))
            elif st == "free" and px in held_by:
                self._capture(px, t, held_by[px])
            elif st == "free" and self.instance[px] is None and px not in held_by:
                pass

    def _axial(self, px, port):
        o, R, ax = self.ports.world(port)
        b = self.body[px]
        return float((self.d.xpos[b] - o) @ ax), o, R, ax

    def _pull(self, px, sides, ax):
        """The hands' pull on the part along the slot's out axis (N; negative = a push into the slot)."""
        f = 0.0
        for side in sides:
            fv, _ = weld_wrench(self.d, self.grips.state[side]["eq"])
            f += float(-fv @ ax)
        return f

    def _slot_tick(self, px, t, sides):
        port = self.port[px]
        kind = self.kind[px]
        K = P.KINDS[kind]
        s, o, R, ax = self._axial(px, port)
        if self.latched[px]:
            if not sides:
                if self.instance[px] is not None and self.ports.proxy[port] == px:
                    self.ports.show_seat(port, True)          # seated and let go: drawn by the seat again
                    self.ports.proxy[port] = None
                    self.ports.instance[port] = self.instance[px]
                    self.park(px)
                return
            pull = self._pull(px, sides, ax)
            if pull > K["release_N"] and self.release_ok(port):
                self.count[px] += 1
                if self.count[px] >= LATCH_TICKS:
                    self.latched[px] = False
                    self.count[px] = 0
                    self.rearm[px] = False          # it latches again only after it has come 2 mm out (hysteresis)
                    self.events.append(("unlatched", port, round(t, 2)))
            else:
                self.count[px] = 0
            return
        # the guide: the weld re-anchored each tick where the part will be one tick on along the axis (its axial
        # velocity), so the slot holds the part's other five degrees of freedom and does not brake its sliding
        b = self.body[px]
        va = self.m.body_dofadr[b]
        v_ax = float(self.d.qvel[va:va + 3] @ ax)
        sc = min(max(s + v_ax * TICK, 0.0), K["travel"])
        if s >= K["travel"]:
            self.d.eq_active[self.seat_eq[px]] = 0
            self.state[px] = "free"
            self.ports.occupied[port] = False
            self.ports.proxy[port] = None
            self.ports.instance[port] = None
            self.events.append(("removed", port, self.instance[px], round(t, 2)))
            return
        pull_now = self._pull(px, sides, ax) if sides else 0.0
        # the seat is a hard stop against a push (a push builds force there); a part being pulled out slides free
        self._weld(px, port, sc, hard=(s <= 0.0008 and pull_now <= 0.5))
        if s >= 0.002:
            self.rearm[px] = True
        if s <= 0.0015 and (self.rearm.get(px, True) or (kind == "gpu" and not sides)):   # a module let go seats
            push = -self._pull(px, sides, ax) if sides else 0.0
            if kind == "gpu" or push > K["latch_N"]:
                self.count[px] += 1                         # a leaky count: the push at a stiff stop chatters
                if self.count[px] >= SEAT_TICKS or kind == "gpu":
                    self.latched[px] = True
                    self.count[px] = 0
                    self._weld(px, port, 0.0)
                    self.events.append(("seated", port, self.instance[px], round(t, 2)))
            else:
                self.count[px] = max(0, self.count[px] - 1)

    def _capture(self, px, t, sides):
        kind = self.kind[px]
        K = P.KINDS[kind]
        lat_tol, ang_tol = K["tol"]
        b = self.body[px]
        pos = self.d.xpos[b]
        Rp = self.d.xmat[b].reshape(3, 3)
        best = None
        for port, rec in self.ports.rec.items():
            if rec["kind"] != kind or not self.ports.available(port):
                continue
            o, R, ax = self.ports.world(port)
            dv = pos - o
            s = float(dv @ ax)
            if s < -0.01 or s > K["travel"] + 0.02:
                continue
            lat = float(np.linalg.norm(dv - s * ax))
            if lat > 3 * lat_tol + 0.004:
                continue
            ang = _rot_angle(R.T @ Rp)
            if best is None or lat < best[1]:
                best = (port, lat, ang, s, ax)
        if best is None:
            self.jam[px] = 0
            return
        port, lat, ang, s, ax = best
        if s > K["travel"] - 0.002:                 # in its guide by 2 mm before it counts as entered (no flicker
            return                                  # at the guide's mouth after it came out)
        if lat <= lat_tol and ang <= ang_tol:
            self.port[px] = port
            self.state[px] = "slot"
            self.rearm[px] = True
            self.latched[px] = False
            self.ports.occupied[port] = True
            self.ports.proxy[port] = px
            self._weld(px, port, min(max(s, 0.0), K["travel"]))
            if px in self.tendon and self.ports.rec[port].get("anchor"):
                pass
            self.events.append(("entered", port, self.instance[px], round(t, 2)))
            return
        if s < K["travel"] - 0.002 and lat <= 3 * lat_tol + 0.004:
            push = -self._pull(px, sides, ax)
            if push > K["damage_N"]:
                self.jam[px] += 1
                if self.jam[px] >= LATCH_TICKS and not self.ports.broken[port]:
                    self.ports.broken[port] = True
                    self.events.append(("damaged", port, "misaligned_insertion", round(t, 2)))


class Fixtures:
    """The trays' latches, the levers' detents, the covers' latch and stay, the rear doors' latches."""

    def __init__(self, m, d, meta, grips):
        self.m, self.d, self.meta, self.g = m, d, meta, grips
        J, E, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_EQUALITY, mujoco.mjtObj.mjOBJ_BODY
        self.jq = lambda jn: m.jnt_qposadr[mujoco.mj_name2id(m, J, jn)]
        self.lock = {}
        for jn in [t["joint"] for t in meta["trays"].values()] + [f["joint"] for f in meta["fixtures"].values()
                                                                  if f["type"] in ("bezel", "cover", "rdoor", "lever")]:
            self.lock[jn] = mujoco.mj_name2id(m, E, "lock_" + jn)
        self.trays = meta["trays"]
        self.fx = meta["fixtures"]
        q = lambda jn: float(d.qpos[self.jq(jn)])
        self.latched = {h: q(t["joint"]) <= 0.002 and max(q(lv) for lv in t["levers"]) <= 0.12 for h, t in self.trays.items()}
        self.service = {h: False for h in self.trays}        # held at the rails' service stop (fully extended)

    def _set_lock(self, jn, on, value=None):
        eq = self.lock[jn]
        if on:
            if not self.d.eq_active[eq]:
                self.m.eq_data[eq, :5] = [float(self.d.qpos[self.jq(jn)]) if value is None else value, 0, 0, 0, 0]
                self.d.eq_active[eq] = 1
        else:
            self.d.eq_active[eq] = 0

    def held(self, body):
        return any(st["obj"] == body for st in self.g.state.values())

    def tick(self, t, events):
        q = lambda jn: float(self.d.qpos[self.jq(jn)])
        for host, tr in self.trays.items():
            levers = [q(lv) for lv in tr["levers"]]
            pos = q(tr["joint"])
            if self.latched[host]:
                if min(levers) >= 0.55:
                    self.latched[host] = False
                    events.append(("tray_unlatched", host, round(t, 2)))
                    self._set_lock(tr["joint"], False)
                else:
                    self._set_lock(tr["joint"], True, 0.0)
            else:
                if pos <= 0.002 and max(levers) <= 0.12:
                    self.latched[host] = True
                    self.service[host] = False
                    events.append(("tray_latched", host, round(t, 2)))
                    self._set_lock(tr["joint"], True, 0.0)
                elif any(self.held(lv) for lv in tr["levers"]):
                    # a lever's release tab held: the rails run free (the tray is pushed home or pulled by its levers)
                    if self.service[host]:
                        self.service[host] = False
                        events.append(("tray_service_stop_released", host, round(t, 2)))
                    self._set_lock(tr["joint"], False)
                elif pos >= H.TRAY_SERVICE_TRAVEL - 0.003:
                    # the rails' service-position stop: a fully extended tray stays put (a bump does not push it in)
                    if not self.service[host]:
                        self.service[host] = True
                        events.append(("tray_service_stop", host, round(t, 2)))
                    self._set_lock(tr["joint"], True, H.TRAY_SERVICE_TRAVEL)
                else:
                    self._set_lock(tr["joint"], False)
        for body, f in self.fx.items():
            if f["type"] not in ("bezel", "cover", "rdoor", "lever"):
                continue
            jn = f["joint"]
            v = q(jn)
            if self.held(body):
                self._set_lock(jn, False)
                continue
            if f["type"] in ("bezel", "lever"):          # a magnetic catch / the lever's detent while closed
                self._set_lock(jn, v <= 0.03, 0.0)
            elif f["type"] == "cover":
                if v <= 0.02:
                    self._set_lock(jn, True, 0.0)
                elif v >= f["stay_rad"] - 0.01:
                    self._set_lock(jn, True, f["stay_rad"])
                else:
                    self._set_lock(jn, False)
            elif f["type"] == "rdoor":
                self._set_lock(jn, v <= 0.02, 0.0)


class Screws:
    """The GPU modules' four captive screws per slot and the torque driver."""

    def __init__(self, ports: Ports, events):
        self.ports, self.events = ports, events
        self.turns = {p: [0.0] * 4 for p, r in ports.rec.items() if r["kind"] == "gpu"}
        self.torque = {p: [P.SCREW["final_Nm"]] * 4 for p in self.turns}
        self.damaged = {p: False for p in self.turns}

    def fastened(self, port):
        return all(t <= 0.01 for t in self.turns[port]) and all(q >= 0.85 for q in self.torque[port])

    def loose(self, port):
        return all(t >= P.SCREW["turns"] - 0.05 for t in self.turns[port])

    def head_world(self, port, k):
        o, R, _ = self.ports.world(port)
        sx, sy = P.GPU_SCREWS[k]
        z = 0.010 + P.SCREW["head_h"] + P.SCREW["rise_per_turn"] * self.turns[port][k]
        return o + R @ np.array([sx, sy, z]), R[:, 2]

    def engaged(self, tip, axis):
        """The (port, screw) whose head the bit sits in, or None (tip and axis: the bit's in the world)."""
        best = None
        for port in self.turns:
            if not self.ports.occupied[port]:
                continue
            o, _, _ = self.ports.world(port)
            if float(np.linalg.norm(tip - o)) > 0.12:
                continue
            for k in range(4):
                h, ax = self.head_world(port, k)
                dv = tip - h
                axial = float(dv @ ax)
                lat = float(np.linalg.norm(dv - axial * ax))
                ang = math.acos(max(-1.0, min(1.0, float(axis @ ax))))      # the driver's axis along the screw's
                if lat <= P.SCREW["engage_lat"] and P.SCREW["engage_ax"][0] <= axial <= P.SCREW["engage_ax"][1] and ang <= P.SCREW["engage_ang"]:
                    if best is None or lat < best[2]:
                        best = (port, k, lat)
        return None if best is None else best[:2]

    def drive(self, port, k, mode, setpoint, dt, t):
        """Turn screw k of `port` for dt seconds; returns "running", "clutch" (setpoint reached) or "out"."""
        rate = P.SCREW["rev_per_s"] * dt
        tr = self.turns[port]
        tq = self.torque[port]
        if mode == "tighten":
            tq[k] = min(tq[k], 0.05) if tr[k] > 0.0 else tq[k]
            if tr[k] > 0.0:
                tr[k] = max(0.0, tr[k] - rate)
                return "running"
            tq[k] = max(tq[k], float(setpoint))
            if tq[k] > 0.5 and any(tr[j] > 0.2 for j in range(4) if j != k) and not self.damaged[port]:
                self.damaged[port] = True
                self.events.append(("damaged", port, "module_tilted_pins_bent", round(t, 2)))
            return "clutch"
        tq[k] = 0.0
        if tr[k] < P.SCREW["turns"]:
            tr[k] = min(P.SCREW["turns"], tr[k] + rate)
            return "running"
        return "out"


class Tools:
    """The torque driver, the fibre scope and the cleaner: their tips, their holders, the driver's runs and the
    cleaner's clicks."""

    def __init__(self, m, d, meta, grips, screws: Screws, events):
        self.m, self.d, self.meta, self.g, self.screws, self.events = m, d, meta, grips, screws, events
        B, S, E = mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_SITE, mujoco.mjtObj.mjOBJ_EQUALITY
        self.rec = meta["tools"]
        self.body = {n: mujoco.mj_name2id(m, B, n) for n in self.rec}
        self.tip = {n: mujoco.mj_name2id(m, S, r["tip_site"]) for n, r in self.rec.items()}
        self.holster = {n: mujoco.mj_name2id(m, E, r["holster_eq"]) for n, r in self.rec.items()}
        self.belt = {n: mujoco.mj_name2id(m, B, r["belt_body"]) for n, r in self.rec.items()}
        self.geoms = {n: [g for g in range(m.ngeom) if m.geom_bodyid[g] == self.body[n]
                          and (m.geom_contype[g] or m.geom_conaffinity[g])] for n in self.rec}
        self.run = None                     # the driver's current run: dict(mode, torque, until, result, screw)
        self.clicked = False
        self.faces = None                   # set by the cell: callable() -> [(face id, centre, normal)]
        grips.on_engage.append(self._taken)
        grips.on_release.append(self._let_go)

    def holster_all(self):
        for n in self.rec:
            self._holster(n)

    def belt_world(self, n):
        """The holster's mouth in the world now: (position, rotation) of the belt point (the tool's origin there)."""
        bb = self.belt[n]
        Rb = self.d.xmat[bb].reshape(3, 3)
        return self.d.xpos[bb] + Rb @ np.asarray(self.rec[n]["belt"], float), Rb

    def _holster(self, n):
        """The tool into its belt holster: welded upright at the holster's mouth, no collider while it hangs there."""
        eq = self.holster[n]
        b = self.body[n]
        p, Rb = self.belt_world(n)
        a = self.m.jnt_qposadr[self.m.body_jntadr[b]]
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, Rb.reshape(-1))
        self.d.qpos[a:a + 3] = p
        self.d.qpos[a + 3:a + 7] = q
        self.d.qvel[self.m.body_dofadr[b]:self.m.body_dofadr[b] + 6] = 0.0
        self.m.eq_data[eq, 0:3] = 0.0
        self.m.eq_data[eq, 3:6] = self.rec[n]["belt"]
        self.m.eq_data[eq, 6:10] = [1.0, 0.0, 0.0, 0.0]
        self.m.eq_data[eq, 10] = 1.0
        self.d.eq_active[eq] = 1
        gs = self.geoms[n]
        self.m.geom_contype[gs] = 0
        self.m.geom_conaffinity[gs] = 0

    def holstered(self, n):
        return bool(self.d.eq_active[self.holster[n]])

    def _taken(self, side, name, t):
        if name in self.holster and self.d.eq_active[self.holster[name]]:
            self.d.eq_active[self.holster[name]] = 0

    def _let_go(self, side, name, t, why):
        """A tool let go upright with its origin within 15 mm of its holster's mouth drops into the holster."""
        if name not in self.holster:
            return
        b = self.body[name]
        hp, Rb = self.belt_world(name)
        R = self.d.xmat[b].reshape(3, 3)
        if float(np.linalg.norm(self.d.xpos[b] - hp)) <= 0.015 and float(R[:, 2] @ Rb[:, 2]) >= math.cos(math.radians(15)):
            self._holster(name)
            self.events.append(("holstered", name, round(t, 2)))

    def held_side(self, name):
        for side, st in self.g.state.items():
            if st["obj"] == name:
                return side
        return None

    def tip_axis(self, name):
        b = self.body[name]
        return self.d.site_xpos[self.tip[name]].copy(), self.d.xmat[b].reshape(3, 3)[:, 2].copy()

    def start_driver(self, mode, torque, seconds, t):
        if self.held_side("tool_driver") is None:
            return {"error": "not_held"}
        if mode not in ("tighten", "loosen"):
            return {"error": "bad_params"}
        self.run = {"mode": mode, "torque": float(torque), "until": t + float(seconds), "result": "running", "screw": None}
        return {"ok": True}

    def tick(self, t):
        if self.run is not None and self.run["result"] == "running":
            if t >= self.run["until"] or self.held_side("tool_driver") is None:
                self.run["result"] = "stopped"
            else:
                tip, ax = self.tip_axis("tool_driver")
                hit = self.screws.engaged(tip, ax)
                if hit is None:
                    self.run["result"] = "no_engagement" if self.run["screw"] is None else "cam_out"
                else:
                    self.run["screw"] = hit
                    res = self.screws.drive(hit[0], hit[1], self.run["mode"], self.run["torque"], TICK, t)
                    if res != "running":
                        self.run["result"] = res
                        self.events.append(("screw", hit[0], hit[1] + 1, self.run["mode"], res, round(t, 2)))
        # the cleaner: one click per push past a face (it must come back out 3 mm before the next)
        if self.held_side("tool_cleaner") is not None and self.faces is not None:
            tip, ax = self.tip_axis("tool_cleaner")
            hit = None
            for fid, c, n in self.faces():
                dv = tip - c
                axial = float(dv @ n)
                lat = float(np.linalg.norm(dv - axial * n))
                ang = math.acos(max(-1.0, min(1.0, float(ax @ n))))
                if lat <= P.CLEANER["lat"] and ang <= P.CLEANER["ang"] and -0.012 <= axial <= 0.003:
                    hit = (fid, axial)
                    break
            if hit is not None and hit[1] <= -P.CLEANER["click_depth"] and not self.clicked:
                self.clicked = True
                self.events.append(("clean", hit[0], round(t, 2)))
            elif hit is None or hit[1] >= 0.001:
                self.clicked = False

    def scope_view(self):
        """(face id, lateral offset m, angle rad, axial m) of the face the scope's tip is at, or None."""
        if self.held_side("tool_scope") is None or self.faces is None:
            return None
        tip, ax = self.tip_axis("tool_scope")
        best = None
        for fid, c, n in self.faces():
            dv = tip - c
            axial = float(dv @ n)
            lat = float(np.linalg.norm(dv - axial * n))
            ang = math.acos(max(-1.0, min(1.0, float(ax @ n))))
            if lat <= 0.004 and P.SCOPE["ax"][0] <= axial <= P.SCOPE["ax"][1] and (best is None or lat < best[1]):
                best = (fid, lat, ang, axial)
        return best


class AisleDoors:
    """The cold aisle's two automatic sliding double doors (a motion sensor over each doorway): a door opens while
    someone is within DOOR_ZONE of its doorway and closes DOOR_HOLD_S after the zone is clear; the leaves move at
    DOOR_SPEED (driven, not pushed)."""

    def __init__(self, m, d):
        self.m, self.d = m, d
        self.doors = {}
        for end, xd in (("w", H.DOOR_X[0]), ("e", H.DOOR_X[1])):
            jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "cdoor_%s%d" % (end, sy)) for sy in (-1, 1)]
            self.doors[end] = {"x": xd, "q": [int(m.jnt_qposadr[j]) for j in jids], "v": [int(m.jnt_dofadr[j]) for j in jids],
                               "hi": [float(m.jnt_range[j][1]) for j in jids], "clear_since": -1e9}

    def tick(self, t, xy):
        for dr in self.doors.values():
            near = abs(xy[0] - dr["x"]) < DOOR_ZONE[0] and abs(xy[1]) < DOOR_ZONE[1]
            if near:
                dr["clear_since"] = None
            elif dr["clear_since"] is None:
                dr["clear_since"] = t
            opening = near or (dr["clear_since"] is not None and t - dr["clear_since"] < DOOR_HOLD_S)
            for qa, va, hi in zip(dr["q"], dr["v"], dr["hi"]):
                tgt = hi if opening else 0.0
                q = float(self.d.qpos[qa])
                self.d.qpos[qa] = q + float(np.clip(tgt - q, -DOOR_SPEED * TICK, DOOR_SPEED * TICK))
                self.d.qvel[va] = 0.0

    def state(self):
        return {end: max(float(self.d.qpos[qa]) for qa in dr["q"]) for end, dr in self.doors.items()}


class Leds:
    """The LEDs: each one geom whose material the cell switches (rgba does not reach a geom with a material)."""

    def __init__(self, m, meta):
        self.m = m
        G, MT = mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_MATERIAL
        self.geom = {n: mujoco.mj_name2id(m, G, g) for n, (g, _) in meta["leds"].items()}
        self.state = {n: s for n, (_, s) in meta["leds"].items()}
        self.mat = {s: mujoco.mj_name2id(m, MT, meta["led_materials"][s]) for s in meta["led_materials"]}

    def set(self, name, state):
        if name in self.geom and self.state.get(name) != state and state in self.mat:
            self.m.geom_matid[self.geom[name]] = self.mat[state]
            self.state[name] = state
