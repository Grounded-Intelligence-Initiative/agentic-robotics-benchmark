"""The kitting cell around the hand: the two nests, the arm's motion as a field, the taught grasp.

Frames. The hand frame H is the model's root frame; the palm is fixed in it. The cell frame is the pick nest's frame at
the pick pose, z up; the arm's motion is given in it (contract section 9). The arm is a field: the hand feels
gravity_in_hand(t) = R(t)^T (g - a_base(t)), exact for translation; a tilt is a slow rotation of that field. The nests
are mocap bodies that the field moves in H, so the hand sees them come and go as the arm carries it.
A nest is a plate under the specimen and four low walls around a pocket that takes the widest specimen at any allowed
placement; the walls' height is the printed one (contract constants), lower than the height a test lift must clear. The taught grasp (the posture family over widths and the ring finger's parking posture) and the pick nest's
pose come from the model's custom numeric fields.
"""
import math

import mujoco
import numpy as np

from contract import constants as C
from contract import names as N
from plant import TRUTH

_NEST = TRUTH["nest"]
_CON = TRUTH["contact"]
NESTS = N.NESTS
PROBE_BODY = "probe"
PROBE_GEOMS = ("probe_a", "probe_b")
HALF_H = C.SPECIMENS["height_mm"] / 2000.0
PLACE_OFFSET_CELL = np.array([C.MOTION["transfer"]["x_m"], C.MOTION["transfer"]["y_m"], 0.0])


def quat_of(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, np.float64).ravel())
    return q


def mat_of(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, np.float64))
    return R.reshape(3, 3)


def rot_axis(axis, ang):
    a = np.asarray(axis, np.float64)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


# ------------------------------------------------------------------------------------------------- taught data
def custom(model, name):
    return np.array(model.numeric(name).data, np.float64)


class Taught:
    """The taught grasp read from a compiled model: the posture family q(w) and the pick nest's pose."""

    def __init__(self, model):
        n = int(custom(model, "taught_rows")[0])
        rows = np.array([custom(model, "taught_%02d" % k) for k in range(n)])
        self.widths_m = rows[:, 0]
        self.q = rows[:, 1:]                                  # rad, servo id order
        self.ring_park = custom(model, "ring_park")
        self.open = custom(model, "taught_open")             # rad, servo id order
        pose = custom(model, "pick_nest_pose")
        self.pick_pos = pose[:3]
        self.pick_quat = pose[3:] / np.linalg.norm(pose[3:])
        self.pick_R = mat_of(self.pick_quat)
        self.width_m = float(custom(model, "taught_width")[0])

    def at_width(self, w_m):
        """The 13 servo angles (rad) of the posture family at width w (m), linear between rows, clipped."""
        w = min(max(float(w_m), self.widths_m[0]), self.widths_m[-1])
        return np.array([np.interp(w, self.widths_m, self.q[:, k]) for k in range(N.N_SERVOS)])

    def preset(self):
        """The cell's taught presets in degrees by joint name: the open pre-shape (clear of the widest specimen at
        any allowed placement), the contact postures at 35, 40 and 45 mm, and the ring finger's parking posture."""
        def by_joint(q):
            return {s.joint: round(math.degrees(float(q[s.id])), 3) for s in N.SERVOS}
        lo, hi = C.SPECIMENS["width_mm"]
        mid = (lo + hi) / 2.0
        return {"open": by_joint(self.open),
                "contact": {"%d" % w: by_joint(self.at_width(w / 1000.0)) for w in (lo, mid, hi)},
                "ring_park": {j: round(math.degrees(float(self.ring_park[k])), 3)
                              for k, j in enumerate(("ring abad", "ring mcp", "ring pip"))}}

    def nest_poses_h(self):
        """Both nests' poses in H with the hand at the pick pose: {nest: (pos (3), quat wxyz (4))}."""
        place = self.pick_pos + self.pick_R @ PLACE_OFFSET_CELL
        return {"pick": (self.pick_pos.copy(), self.pick_quat.copy()), "place": (place, self.pick_quat.copy())}


# ------------------------------------------------------------------------------------------------- nests and probe
def pocket_half():
    return _NEST["pocket_half_x_m"], _NEST["pocket_half_y_m"]


def wall_height_m():
    """The nests' wall height (m): the printed value of the cell's documents."""
    return float(C.NEST_GEOMETRY["wall_height_mm"]) / 1000.0


def add_cell(spec, taught):
    """Add the two nests (mocap) and the clearance probe (mocap, no contacts) to an MjSpec of the hand."""
    px, py = pocket_half()
    pt, wh, wt = _NEST["plate_half_thickness_m"], wall_height_m(), _NEST["wall_thickness_m"]
    poses = taught.nest_poses_h()
    for nest in NESTS:
        pos, quat = poses[nest]
        b = spec.worldbody.add_body()
        b.name = "nest_" + nest
        b.mocap = True
        b.pos = list(pos)
        b.quat = list(quat)
        parts = [((0.0, 0.0, -HALF_H - pt), (px + wt, py + wt, pt)),
                 ((px + wt / 2, 0.0, -HALF_H + wh / 2), (wt / 2, py + wt, wh / 2)),
                 ((-px - wt / 2, 0.0, -HALF_H + wh / 2), (wt / 2, py + wt, wh / 2)),
                 ((0.0, py + wt / 2, -HALF_H + wh / 2), (px, wt / 2, wh / 2)),
                 ((0.0, -py - wt / 2, -HALF_H + wh / 2), (px, wt / 2, wh / 2))]
        for k, (p, sz) in enumerate(parts):
            g = b.add_geom()
            g.name = "nest_%s_%d" % (nest, k)
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = list(sz)
            g.pos = list(p)
            g.friction = [_CON["nest_friction"], _CON["torsional_m"], _CON["rolling"]]
            g.condim = int(_CON["nest_condim"])
            g.contype = 1
            g.conaffinity = 1
            g.group = 4
    pb = spec.worldbody.add_body()
    pb.name = PROBE_BODY
    pb.mocap = True
    pb.pos = list(poses["pick"][0])
    pb.quat = list(poses["pick"][1])
    for k, name in enumerate(PROBE_GEOMS):
        g = pb.add_geom()
        g.name = name
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [0.005, 0.005, 0.005]
        g.pos = [0.01 * (2 * k - 1), 0.0, 0.0]          # resized and moved per query (session.clearance_m)
        g.contype = 0
        g.conaffinity = 0
        g.group = 5
        g.mass = 0.0


# ------------------------------------------------------------------------------------------------- the arm's motion
def _minjerk(t, T):
    if t <= 0.0:
        return 0.0, 0.0
    if t >= T:
        return 1.0, 0.0
    s = t / T
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, (60 * s - 180 * s ** 2 + 120 * s ** 3) / (T * T)


class Segment:
    """One base segment of the arm (cell frame): displacement (m), acceleration (m/s^2) and tilt (rad) over time.
    kind: one of names.SET_BASE_KINDS; start: the displacement in force when the segment starts."""

    def __init__(self, kind, t0_ms, params, start):
        if kind not in N.SET_BASE_KINDS:
            raise ValueError("not a base segment kind: %r" % (kind,))
        self.kind = kind
        self.t0 = int(t0_ms) / 1000.0
        self.scale = float(params.get("track_scale", 1.0))
        self.start = np.zeros(3) if kind in ("rest", "return") else np.asarray(start, np.float64).copy()
        self.moves, self.tilts = [], []
        self.axis = np.array([1.0, 0.0, 0.0])
        mo = C.MOTION
        if kind == "test_lift":
            self.moves.append((0.0, mo["test_lift"]["rise_s"], np.array([0.0, 0.0, mo["test_lift"]["rise_mm"] / 1e3])))
        elif kind in ("lower", "set_down"):
            self.moves.append((0.0, mo[kind]["s"], np.array([0.0, 0.0, -mo[kind]["drop_mm"] / 1e3])))
        elif kind == "transfer":
            prof = params["profile"]
            if prof not in N.PROFILES:
                raise ValueError("not a profile: %r" % (prof,))
            for st in C.transfer_steps(prof):
                if st["kind"] == "move":
                    self.moves.append((st["t0_s"], st["dur_s"], np.asarray(st["delta_m"], np.float64)))
                elif st["kind"] == "tilt":
                    self.axis = np.asarray(st["axis"], np.float64)
                    self.tilts.append((st["t0_s"], st["dur_s"], math.radians(st["deg"])))

    def at(self, t_s):
        p = self.start.copy()
        a = np.zeros(3)
        th = 0.0
        ramp = C.MOTION["tilt_ramp_s"]
        for t0, T, dv in self.moves:
            f, dd = _minjerk(t_s - self.t0 - t0, T)
            p += f * dv
            a += self.scale * dd * dv
        for t0, T, ang in self.tilts:
            s = min(max(t_s - self.t0 - t0, 0.0), T)
            vmax = ang / (T - ramp)
            if s < ramp:
                th += 0.5 * vmax * s * s / ramp
            elif s > T - ramp:
                th += ang - 0.5 * vmax * (T - s) ** 2 / ramp
            else:
                th += 0.5 * vmax * ramp + vmax * (s - ramp)
        return p, a, th


class Field:
    """The arm's motion as seen from the hand: the segments in force and what they give in H."""

    def __init__(self, taught, g_mps2):
        self.R_wh0 = taught.pick_R.T                 # hand -> cell at the pick pose
        self.c_pick = taught.pick_pos
        self.g_w = np.array([0.0, 0.0, -float(g_mps2)])
        self.segments = [Segment("rest", 0, {}, np.zeros(3))]

    def set(self, kind, t0_ms, params, now_ms):
        if int(t0_ms) < int(now_ms):
            raise ValueError("a segment cannot start in the past")
        p, _a, _th = self.at(int(t0_ms) / 1000.0)
        self.segments = [s for s in self.segments if s.t0 < int(t0_ms) / 1000.0] or self.segments[:1]
        self.segments.append(Segment(kind, t0_ms, params, p))

    def active(self, t_s):
        seg = self.segments[0]
        for s in self.segments:
            if s.t0 <= t_s + 1e-12:
                seg = s
        return seg

    def prune(self, t_s):
        while len(self.segments) > 1 and self.segments[1].t0 <= t_s + 1e-12:
            self.segments.pop(0)

    def at(self, t_s):
        seg = self.active(t_s)
        return seg.at(t_s) if seg.t0 <= t_s + 1e-12 else (seg.start, np.zeros(3), 0.0)

    def pose(self, t_s):
        """(R_wh: hand -> cell rotation, p_base: the hand's origin in the cell frame, a: acceleration, th)."""
        seg = self.active(t_s)
        p, a, th = self.at(t_s)
        R_wh = rot_axis(seg.axis, th) @ self.R_wh0 if th != 0.0 else self.R_wh0
        p_base = -self.R_wh0 @ self.c_pick + p
        return R_wh, p_base, a, th

    def gravity_h(self, t_s):
        R_wh, _p, a, _th = self.pose(t_s)
        return R_wh.T @ (self.g_w - a)

    def nest_h(self, nest, t_s):
        """A nest's pose in H at time t: (pos (3), rotation (3 x 3))."""
        R_wh, p_base, _a, _th = self.pose(t_s)
        pn = np.zeros(3) if nest == "pick" else PLACE_OFFSET_CELL
        return R_wh.T @ (pn - p_base), R_wh.T.copy()
