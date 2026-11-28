"""Motion skills of the privileged reference program (grader side; reference.py runs them).

They drive the robot only through the cell's public controller (command / step), its tools (tool) and the site's
services (noc), exactly as a program on the wire does; what is privileged is where they aim: the true poses of the
racks, ports, parts and tools (the model's state), the true scenario, and tables measured offline on the drawing's
nominal pod (stance tables: tests/dc_cell/ref_data/*.json, expressed in the rack's frame so that one table serves
every node of both rows and every pod's installation offsets).
"""

from __future__ import annotations

import copy
import json
import math
import os

import mujoco
import numpy as np

from . import hall as H
from . import parts as P
from .robot import ARM_HOME, PELVIS_HEIGHT

TICK = 0.01
TUCK = {"left": [0.6, 0.3, 0.0, 1.8, 0.0, 0.0, 0.0], "right": [0.6, -0.3, 0.0, 1.8, 0.0, 0.0, 0.0]}
# the arms close to the body for walking (tools/walk_pose.py): the arm colliders within 0.22 m of the base centre, less
# than the navigation stack's clearance (nav.R_HARD 0.24); ARM_HOME's wrists stand out to 0.29 m
WALK_ARMS = {"left": [-0.1, 0.1, 0.3, 1.8, 0.0, 0.0, 0.0], "right": [-0.1, -0.1, -0.3, 1.8, 0.0, 0.0, 0.0]}
TABLE_FRAME = (1.5, 0.6, 0.0)      # the rack frame (A03, nominal pod) the stance tables were measured in
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ref_data")
WALK_STAND = 0.74
CHECK_MARGIN = 0.012          # m: the clearance an arm path keeps from everything (the robot's tracking error)
# the controller state a dry run copies (control.Controller): the arms' targets and smoothing, the stand and the waist
SIM_STATE = ("arm_mode", "arm_cmd", "arm_pose", "arm", "arm_vel", "pose_cur", "q_goal", "seg", "seg_group", "seg_end",
             "arm_ik_err", "fallbacks", "stand", "stand_cmd", "waist", "waist_cmd", "_group")


def quat_from_axes(x_axis, y_axis):
    x = np.asarray(x_axis, float)
    x = x / np.linalg.norm(x)
    y = np.asarray(y_axis, float)
    y = y - (y @ x) * x
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.stack([x, y, z], 1).reshape(-1))
    return q


def mat_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(-1))
    return q


def quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def mulq(a, b):
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, np.asarray(a, float), np.asarray(b, float))
    return q


def rotq(axis, ang):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, float), float(ang))
    return q


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_table(name):
    with open(os.path.join(DATA, name)) as f:
        return json.load(f)


def _slerp(q0, q1, t):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(q0 @ q1)
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


class DryRun:
    """The controller's own arm, waist and stand logic (control.Controller.arm_step and its rates) run kinematically
    on scratch data from the present state: the base stays where it is (or where set_base puts it), the joints follow
    their rate-limited commands exactly, a held free part follows its hand. A move is tried this way before it is
    commanded (the stance studies checked the work poses, not the moves between them); hits() lists robot links (the
    hands carry no contact) and held parts within CHECK_MARGIN of anything."""

    def __init__(self, S, held=True):
        S._check_setup()
        self.S = S
        c = S.c
        sim = copy.copy(c.ctrl)
        for a in SIM_STATE:
            setattr(sim, a, copy.deepcopy(getattr(c.ctrl, a)))
        self.sim = sim
        self.q = c.d.qpos.copy()
        r = c.robot
        self.base = self.q[r.base_q[:3]].copy()
        self.held = {}                  # side -> (body, qpos address, position and rotation in the grasp site's frame)
        m, d = c.m, c.d
        for side in ("left", "right"):
            n = S.holds(side) if held else None
            if not n:
                continue
            if not n.startswith("tool_") and c.pool.state.get(n) != "free":
                continue                # a part in its slot slides in its guide: the slot's contacts are its own
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
            ps, Rs = S.site(side), S.site_R(side)
            self.held[side] = (b, int(m.jnt_qposadr[m.body_jntadr[b]]), Rs.T @ (d.xpos[b] - ps),
                               Rs.T @ d.xmat[b].reshape(3, 3))
        self.t = 0.0
        self.trace = []
        self.active = ("left", "right")     # the arms whose controller logic runs (an idle arm's target stays put)

    def site_pose(self, side):
        """The grasp site's (position, rotation) at the scratch state."""
        S = self.S
        dc = S._dc
        dc.qpos[:] = self.q
        mujoco.mj_kinematics(S._mc, dc)
        sid = S.c.robot.grasp_site[side]
        return dc.site_xpos[sid].copy(), dc.site_xmat[sid].reshape(3, 3).copy()

    def set_base(self, x, y, yaw):
        r = self.S.c.robot
        self.q[r.base_q[0]], self.q[r.base_q[1]], self.q[r.base_q[2]] = x, y, yaw
        self.base = np.array([x, y, yaw], float)

    def command(self, cmd):
        self.sim.command(cmd)

    def _tick(self):
        from .control import STAND_RATE, WAIST_RATE
        S, sim = self.S, self.sim
        r = S.c.robot
        sim.stand += float(np.clip(sim.stand_cmd - sim.stand, -STAND_RATE * TICK, STAND_RATE * TICK))
        self.q[r.base_q[3]] = sim.stand - PELVIS_HEIGHT
        lt = sim.legs.targets(sim.stand)
        for sd in ("left", "right"):
            self.q[r.leg_q[sd]] = lt
        sim.waist = sim.waist + np.clip(sim.waist_cmd - sim.waist, -WAIST_RATE * TICK, WAIST_RATE * TICK)
        self.q[r.waist_q] = sim.waist
        x, y, yaw = self.base
        cz, sz = math.cos(yaw), math.sin(yaw)
        Rb = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        qb = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        dc = S._dc
        dc.qpos[:] = self.q
        mujoco.mj_kinematics(S._mc, dc)
        for sd in self.active:
            sim.arm_step(sd, dc, x, y, Rb, qb)
            self.q[r.arm_q[sd]] = sim.arm[sd]
        self.t += TICK

    def raw(self, sides=("left", "right"), body=False):
        """{(link or part, other): distance} within CHECK_MARGIN at the scratch state: the links of the arms of
        `sides` (every link with body) and the free parts those hands hold (their own wrist excepted)."""
        S = self.S
        mc, dc = S._mc, S._dc
        dc.qpos[:] = self.q
        if self.held:
            mujoco.mj_kinematics(mc, dc)
            for sd, (b, adr, prel, Rrel) in self.held.items():
                sid = S.c.robot.grasp_site[sd]
                ps, Rs = dc.site_xpos[sid].copy(), dc.site_xmat[sid].reshape(3, 3).copy()
                dc.qpos[adr:adr + 3] = ps + Rs @ prel
                dc.qpos[adr + 3:adr + 7] = mat_quat(Rs @ Rrel)
        return S._hits_at(sides, body, {v[0]: sd for sd, v in self.held.items()})

    def extent(self, side):
        """(lateral, forward) reach in the base frame of `side`'s arm links and what its hand holds, at the scratch
        state: how far they stand out of the walking footprint."""
        S = self.S
        mc, dc = S._mc, S._dc
        dc.qpos[:] = self.q
        mujoco.mj_kinematics(mc, dc)
        x, y, yaw = self.base
        Rb = rz(yaw)
        o = np.array([x, y, 0.0])
        pts = [dc.xpos[b] for b, sd in S._arm_of.items() if sd == side]
        if side in self.held:
            b, adr, prel, Rrel = self.held[side]
            sid = S.c.robot.grasp_site[side]
            ps, Rs = dc.site_xpos[sid].copy(), dc.site_xmat[sid].reshape(3, 3).copy()
            pp, Rp = ps + Rs @ prel, Rs @ Rrel
            m = S.c.m
            for g in range(m.ngeom):
                if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                    hs = m.geom_size[g]
                    for sx in (-1, 1):
                        for sy in (-1, 1):
                            for sz in (-1, 1):
                                pts.append(pp + Rp @ (m.geom_pos[g] + quat_mat(m.geom_quat[g]) @
                                                      (hs * np.array([sx, sy, sz]))))
        loc = [Rb.T @ (np.asarray(p_) - o) for p_ in pts]
        return max(abs(v[1]) for v in loc), max(v[0] for v in loc)

    def baseline(self, sides=("left", "right"), body=False):
        """What is already within the margin now: a move may stay that close to it, not come closer."""
        self.base_hits = self.raw(sides, body)
        return self.base_hits

    def hits(self, sides=("left", "right"), body=False):
        """The pairs within the margin that are new, or 3 mm closer than at the baseline and under 4 mm apart."""
        base = getattr(self, "base_hits", {})
        out = []
        for pair, dist in self.raw(sides, body).items():
            b0 = base.get(pair)
            if b0 is None or (dist < b0 - 0.003 and dist < 0.004):
                out.append(pair)
        return out

    def arm_done(self, side):
        sim = self.sim
        if sim.arm_mode[side] == "pose" and sim.arm_pose[side] is not None and sim.arm_pose[side][3] == "line":
            if sim.pose_cur[side] is None or sim.seg[side] is None or sim.seg[side][3] < sim.seg[side][2]:
                return False
        return float(np.max(np.abs(sim.arm[side] - sim.arm_cmd[side]))) < 0.003 and \
            float(np.max(np.abs(sim.arm_vel[side]))) < 0.02

    def posture_done(self):
        sim = self.sim
        return abs(sim.stand - sim.stand_cmd) < 1e-3 and float(np.max(np.abs(sim.waist - sim.waist_cmd))) < 1e-3 and \
            self.arm_done("left") and self.arm_done("right")

    def run(self, until, sides=("left", "right"), t_max=6.0, every=4, body=False):
        """Advance until until(self) (at most t_max s); the first hit found or None."""
        n = max(1, int(round(t_max / TICK)))
        r = self.S.c.robot
        for k in range(n):
            self._tick()
            if getattr(self.S, "shadow", False):
                self.trace.append((round(self.t, 3), {sd: self.q[r.arm_q[sd]].copy() for sd in self.active},
                                   self.q[r.base_q[3]], self.q[r.waist_q].copy()))
            done = until(self)
            if done or k % every == every - 1 or k == n - 1:
                h = self.hits(sides, body)
                if h:
                    return h
            if done:
                return None
        return None


class RefSkills:
    def __init__(self, cell, log=None, background=None, watch_contacts=True):
        self.c = cell
        self.log = log or (lambda *a: None)
        self.background = background            # called about once per simulated second (the reference's watch)
        self._bg_next = 0.0
        self.phase = ""                          # what the reference is doing now (for the contact watch's log)
        self.step = ""                           # the motion primitive under way (ditto)
        self.watch = watch_contacts
        self.checked = True                      # reach() checks arm paths for clearance
        self._seen = set()
        m = cell.m
        self._robot_body = np.array([(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith("g1/")
                                     for i in range(m.nbody)])
        self._f6 = np.zeros(6)

    def _contact_watch(self):
        """Log robot contacts above 60 N (the safety watch's limit is 150 N for 0.2 s) once per pair and phase."""
        c, m, d = self.c, self.c.m, self.c.d
        for i in range(d.ncon):
            con = d.contact[i]
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if self._robot_body[b1] == self._robot_body[b2] or b1 == 0 or b2 == 0:
                continue
            mujoco.mj_contactForce(m, d, i, self._f6)
            f = abs(float(self._f6[0]))
            if f < 60.0:
                continue
            rb, ob = (b1, b2) if self._robot_body[b1] else (b2, b1)
            key = (self.phase, self.step, rb, ob, f >= 150.0)
            if key in self._seen:
                continue
            self._seen.add(key)
            self.log("[ref] contact %.0f N at t %.1f during '%s' / %s: %s <-> %s" % (
                f, c.sim_time, self.phase, self.step, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, rb),
                mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, ob)))

    # ------------------------------------------------------------------------------------------------ time
    def run(self, ticks):
        r = None
        left = int(ticks)
        while left > 0 and not self.c.done:
            n = min(left, 10 if self.watch else 100)
            r = self.c.step(n)
            left -= n
            if self.watch:
                self._contact_watch()
            if self.background is not None and self.c.sim_time >= self._bg_next:
                self._bg_next = self.c.sim_time + 1.0
                self.background()
        return r

    def wait(self, seconds):
        self.run(int(round(seconds / TICK)))

    def noc(self, method, **params):
        r = self.c.noc(method, params)
        if isinstance(r, dict) and r.get("error"):
            self.log("[ref] noc %s %s -> %s" % (method, params, r))
        return r

    # ------------------------------------------------------------------------------------------------ state
    def site(self, side):
        return self.c.d.site_xpos[self.c.robot.grasp_site[side]].copy()

    def site_R(self, side):
        return self.c.d.site_xmat[self.c.robot.grasp_site[side]].reshape(3, 3).copy()

    def base(self):
        return self.c.d.qpos[self.c.robot.base_q[:3]].copy()

    def holds(self, side):
        return self.c.grips.state[side]["obj"]

    HEAVY_KG = 1.0                    # a part this heavy is moved gently: the grip holds 30 N, the part's weight
    # takes much of it. Per kg above: the hand's path (m/s, peak m/s^2), its turn (rad/s, peak rad/s^2), a joint
    # move (rad/s, peak rad/s^2 per joint); the supply (1.1 kg) at the controller's own path limits, the GPU module
    # (1.8 kg) slower
    HEAVY_LIMITS = {"psu": (0.25, 1.0, 0.8, 3.0, 1.0, 3.0), "gpu": (0.18, 0.7, 0.6, 2.0, 0.6, 2.0)}

    def heavy(self, side):
        px = self.holds(side)
        if px is None:
            return False
        return P.KINDS.get(self.c.pool.kind.get(px), {}).get("mass", 0.0) > self.HEAVY_KG

    def very_heavy(self, side):
        """A GPU module (1.8 kg) in the hand: the joint-space moves and holds (a supply, 1.1 kg, keeps pose moves)."""
        px = self.holds(side)
        return px is not None and P.KINDS.get(self.c.pool.kind.get(px), {}).get("mass", 0.0) > 1.5

    def _limits(self, side):
        return self.HEAVY_LIMITS.get(self.c.pool.kind.get(self.holds(side)), self.HEAVY_LIMITS["gpu"])

    def reachable(self, side, pos, quat=None, tol=0.01, why=None):
        """Whether the arm (on its present configuration's branch, the body as it stands) reaches (pos, quat)."""
        c = self.c
        qa = c.d.qpos[c.robot.arm_q[side]].copy()
        _, ep, er = c.ctrl.ik.solve(c.d.qpos.copy(), side, np.asarray(pos, float),
                                    None if quat is None else np.asarray(quat, float), qa, iters=200, w_home=0.0)
        ok = ep <= tol and (quat is None or er <= 0.08)
        if not ok and why is not None:
            self.log("[ref] %s: the %s arm reaches only to %.0f mm / %.2f rad of (%.3f, %.3f, %.3f)" % (
                why, side, 1000 * ep, er, pos[0], pos[1], pos[2]))
        return ok

    def _stream_targets(self, side, p0, q0, pos, quat, v_max=None):
        """The near targets (one per other tick) of a hand moved from (p0, q0) to (pos, quat) along the straight line on
        a minimum-jerk progression bounded by the held part's limits; [] for a short move."""
        pos = np.asarray(pos, float)
        d = float(np.linalg.norm(pos - p0))
        ang = 0.0
        if quat is not None:
            quat = np.asarray(quat, float)
            if float(np.dot(q0, quat)) < 0.0:
                quat = -quat
            ang = 2.0 * math.acos(min(1.0, abs(float(np.dot(q0, quat)))))
        v, a, w_, aw, _qd, _qdd = self._limits(side) if self.heavy(side) else (0.25, 1.0, 1.0, 4.0, 1.5, 6.0)
        if v_max is not None:
            v = min(v, v_max)
        T = max(0.3, 1.875 * d / v, math.sqrt(5.7735 * d / a), 1.875 * ang / w_, math.sqrt(5.7735 * ang / aw))
        n = int(T / (2 * TICK))
        if d < 0.01 and ang < 0.05 or n < 3:
            return []
        out = []
        for k in range(1, n):
            u = k / n
            w = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
            p = p0 + (pos - p0) * w
            if quat is None:
                out.append({"pos": [float(x) for x in p], "frame": "room", "path": "line"})
            else:
                out.append({"pose": [float(x) for x in p] + [float(x) for x in _slerp(q0, quat, w)], "frame": "room",
                            "path": "line"})
        return out

    def _stream_q_targets(self, side, q0, qa):
        qa = np.asarray(qa, float)
        dq = float(np.max(np.abs(qa - q0)))
        _v, _a, _w, _aw, qd, qdd = self._limits(side)
        T = max(0.3, 1.875 * dq / qd, math.sqrt(5.7735 * dq / qdd))
        n = int(T / (2 * TICK))
        if dq < 0.02 or n < 3:
            return []
        return [{"q": [float(x) for x in q0 + (qa - q0) * (u * u * u * (10.0 - 15.0 * u + 6.0 * u * u))]}
                for u in (k / n for k in range(1, n))]

    def _stream_pose(self, side, pos, quat, v_max=None):
        """The hand (holding a heavy part) moved to (pos, quat) along the straight line by a stream of near targets
        (a minimum-jerk progression, one target every other tick): each near target keeps the arm's configuration
        (no swing of the elbow to another branch on the way), and the progression bounds the part's acceleration.
        The dry runs replay the same stream (_try_moves)."""
        for tgt in self._stream_targets(side, self.site(side), mat_quat(self.site_R(side)), pos, quat, v_max):
            self.c.command({side + "_arm": tgt})
            self.run(2)
            if self.c.done:
                return

    def _stream_q(self, side, qa):
        """A joint move of an arm that holds a heavy part: joint targets streamed along a minimum-jerk progression."""
        for tgt in self._stream_q_targets(side, self.c.d.qpos[self.c.robot.arm_q[side]].copy(), qa):
            self.c.command({side + "_arm": tgt})
            self.run(2)
            if self.c.done:
                return

    # ------------------------------------------------------------------------------------------------ frames
    @staticmethod
    def rack_of(device):
        """gpn-a03 -> A03; leaf-a01-2 -> A01; A03 -> A03."""
        if device.startswith("gpn-"):
            return device[4].upper() + device[5:7]
        if device.startswith("leaf-"):
            return device[5].upper() + device[6:8]
        return device

    def rack_pose(self, rack):
        """(origin xy, yaw) of a rack's frame in the hall as installed (this pod's offsets included)."""
        b = mujoco.mj_name2id(self.c.m, mujoco.mjtObj.mjOBJ_BODY, "rack_" + rack)
        R = self.c.d.xmat[b].reshape(3, 3)
        return self.c.d.xpos[b][:2].copy(), math.atan2(R[1, 0], R[0, 0])

    def to_world(self, rack, lx, ly, lyaw=0.0):
        o, yaw = self.rack_pose(rack)
        c, s = math.cos(yaw), math.sin(yaw)
        return o[0] + c * lx - s * ly, o[1] + s * lx + c * ly, wrap(yaw + lyaw)

    def vec_to_world(self, rack, v):
        _, yaw = self.rack_pose(rack)
        return rz(yaw) @ np.asarray(v, float)

    @staticmethod
    def table_to_local(x, y, yaw):
        """A stance measured in the table frame (A03, nominal) -> the rack's own frame."""
        ox, oy, oyaw = TABLE_FRAME
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - oyaw)

    # ------------------------------------------------------------------------------------------------ the body
    def arms(self, q_left=None, q_right=None, settle=1.2, on_hit="up", held=True):
        """Joint targets for the arms. Checked (dry runs): a joint move that would sweep an arm (or what its hand
        holds) through something first takes the hand up / back; with on_hit "skip" an arm with no clear move stays
        where it is. Returns False when an arm had no clear move."""
        targets = {"left": q_left, "right": q_right}
        clear = True
        if self.checked:
            for side in ("left", "right"):
                qt = targets[side]
                if qt is None:
                    continue
                mv = ("arm", side, {"q": [float(v) for v in qt]})
                hit = self.try_moves([mv], (side,), held=held)
                if hit is None:
                    continue
                p, q0 = self.site(side), mat_quat(self.site_R(side))
                b = self.base()
                back = rz(b[2]) @ np.array([-1.0, 0.0, 0.0])
                up = np.array([0.0, 0.0, 1.0])
                done = False
                for vias in ([p + up * 0.12], [p + up * 0.12 + back * 0.10], [p + up * 0.25],
                             [p + up * 0.12, p + up * 0.12 + back * 0.15],
                             [np.array([p[0], p[1], max(p[2] + 0.15, 1.05)]) + back * 0.08]):
                    if self.try_moves([("arm", side, self._tgt(v, q0)) for v in vias] + [mv], (side,), held=held) is None:
                        self.step = "arms %s: up first" % side
                        for v in vias:
                            self.reach(side, v, q0, tol=0.01, timeout=4.0, checked=False)
                        done = True
                        break
                if not done:
                    clear = False
                    if on_hit == "skip":
                        self.log("[ref] arms %s: the joint move would touch %s; the arm stays" % (side, hit[0]))
                        targets[side] = None
                    else:
                        self.log("[ref] arms %s: no clear joint move (%s); moving anyway" % (side, hit[0]))
        cmd = {}
        if targets["left"] is not None:
            cmd["left_arm"] = {"q": list(targets["left"])}
        if targets["right"] is not None:
            cmd["right_arm"] = {"q": list(targets["right"])}
        if cmd:
            self.step = "arms%s%s" % (" left" if "left_arm" in cmd else "", " right" if "right_arm" in cmd else "")
            for sd in ("left", "right"):
                if targets[sd] is not None and self.heavy(sd):
                    self._stream_q(sd, targets[sd])
            self.c.command(cmd)
            self.run(int(settle / TICK))
        return clear

    def tuck(self, sides=("left", "right"), settle=1.2, back=False, walk=False, on_hit="retreat"):
        """Free arms to the home pose (hands in front of the hips: with the opposite row close behind), tucked back
        (elbows bent behind: clear of a tray the torso leans over) or close to the body for walking. A side whose hand
        holds something stays as it is. Checked: an arm that cannot come in where the robot stands (a stance by a
        tray is closer than the arms' home pose allows) waits while the robot steps away (on_hit "retreat"), or stays
        (on_hit "skip"). Returns False when an arm had no clear move."""
        pose = TUCK if back else (WALK_ARMS if walk else ARM_HOME)

        def targets():
            ql = list(pose["left"]) if "left" in sides and self.holds("left") is None else None
            qr = list(pose["right"]) if "right" in sides and self.holds("right") is None else None
            return ql, qr

        ql, qr = targets()
        if on_hit != "retreat":
            return self.arms(ql, qr, settle, on_hit=on_hit)
        if self.arms(ql, qr, settle, on_hit="skip"):
            return True
        self.retreat()
        ql, qr = targets()
        return self.arms(ql, qr, settle, on_hit="up")

    def posture(self, stand=None, waist=None, settle=1.5, steps=1):
        """Stand height and waist. Checked (dry run, every link): when the change would bring an arm (or what its hand
        holds) against something, that hand is first held where it is in the room (the arm follows the torso's move
        by its IK), or taken up first."""
        cmd = {}
        self.step = "posture %s %s" % (stand, waist)
        if stand is not None:
            cmd["stand_height"] = float(stand)
        if waist is not None:
            cmd["waist"] = [float(v) for v in waist]
        if self.checked and cmd:
            hit = self.try_moves([("posture", cmd)], ("left", "right"), body=True)
            if hit is not None:
                holds = {"left": self.holds("left"), "right": self.holds("right")}
                arms_hit = sorted({self._side_of(h[0], holds) for h in hit} - {None})
                fixed = False
                if arms_hit:
                    up = np.array([0.0, 0.0, 1.0])
                    for lift in (0.0, 0.10, 0.20):
                        pre = []
                        for sd in arms_hit:
                            pre.append(("arm", sd, self._tgt(self.site(sd) + up * lift, mat_quat(self.site_R(sd)))))
                        if self.try_moves(pre + [("posture", cmd)], ("left", "right"), body=True) is None:
                            self.step = "posture: hold %s (up %.2f)" % (arms_hit, lift)
                            for mv in pre:
                                self.reach(mv[1], mv[2]["pose"][:3], mv[2]["pose"][3:], tol=0.01, timeout=3.0,
                                           checked=False)
                            fixed = True
                            break
                if not fixed:
                    self.log("[ref] posture %s %s: touches %s; going anyway" % (stand, waist, hit[0]))
            self.step = "posture %s %s" % (stand, waist)
        if steps == 1 and (self.heavy("left") or self.heavy("right")):
            steps = 8
            settle = min(settle, 0.6)           # the stream brings the body there smoothly
        if steps > 1:                           # gently (a heavy part in a hand): a smooth stream of small changes
            ctrl = self.c.ctrl
            s0, w0 = float(ctrl.stand_cmd), np.array(ctrl.waist_cmd, float)
            dw = 0.0 if waist is None else float(np.max(np.abs(np.asarray(waist, float) - w0)))
            ds = 0.0 if stand is None else abs(float(stand) - s0)
            T = max(0.4, 1.875 * dw / 0.35, 1.875 * ds / 0.12, math.sqrt(5.7735 * dw / 0.8),
                    math.sqrt(5.7735 * ds / 0.3))
            n = int(T / (2 * TICK))
            for k in range(1, n):
                u = k / n
                f = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
                sub = {}
                if stand is not None:
                    sub["stand_height"] = s0 + f * (float(stand) - s0)
                if waist is not None:
                    sub["waist"] = [float(v) for v in (w0 + f * (np.asarray(waist, float) - w0))]
                self.c.command(sub)
                self.run(2)
                if self.c.done:
                    break
        self.c.command(cmd)
        self.run(int(settle / TICK))

    def hold_in_base(self, side):
        """The hand (and what it holds) kept where it is relative to the base while the robot walks. A heavy part's
        arm is locked in joint space instead (a pose held by the IK tracking feeds the gait's sway back into the arm
        and it rings: the grip lets go)."""
        if self.very_heavy(side):
            self.c.command({side + "_arm": {"q": [float(v) for v in self.c.ctrl.arm_cmd[side]]}})
            return
        p, Rm = self.site(side), self.site_R(side)
        b = self.base()
        Rb = rz(b[2])
        rel = Rb.T @ (p - np.array([b[0], b[1], 0.0]))
        self.c.command({side + "_arm": {"pose": list(rel) + list(mat_quat(Rb.T @ Rm)), "frame": "base"}})

    def walk(self, x, y, yaw, timeout=120.0, tuck=True, precise=True, carry="close"):
        """To (x, y, yaw) by the navigation stack (free arms in; a held tool or part brought in close, or with carry
        "base" kept where it is relative to the base), then a precise base goal. Returns the position error (m)."""
        if tuck:
            for side in ("left", "right"):
                held = self.holds(side)
                if held is not None and (carry == "base" or not held.startswith("tool_")):
                    self.hold_in_base(side)     # a part (or anything with carry "base") stays where it is held
                elif held is not None:
                    # a held tool or part comes in close too (back first, then the arm's walking pose)
                    b = self.base()
                    self.reach(side, self.site(side) + rz(b[2]) @ np.array([-0.10, 0.0, 0.03]), mat_quat(self.site_R(side)),
                               timeout=2.0)
                    if not self.arms(**{("q_" + side): WALK_ARMS[side]}, settle=0.8, on_hit="skip"):
                        if self.retreat():
                            self.arms(**{("q_" + side): WALK_ARMS[side]}, settle=0.8)
            if not self.tuck(walk=True, on_hit="skip"):
                if self.retreat():
                    self.tuck(walk=True)
            self.posture(stand=WALK_STAND, waist=(0.0, 0.0, 0.0), settle=0.8)
        if self.checked and tuck:
            b0 = self.base()
            far = math.hypot(b0[0] - x, b0[1] - y) > 0.15
            if far and (DryRun(self).raw((), True) or self.floor_clearance(b0[0], b0[1]) < 0.38):
                # the body stands close to something (a work stance by a tray, a rack): a turn on the spot would sweep
                # the hips or the shoulders (or what a hand holds) into it, so a straight step back first
                self.retreat(0.30)
        self.step = "walk to (%.2f, %.2f, %.2f)" % (x, y, yaw)
        r = self.c.command({"nav_goal": [float(x), float(y), float(yaw)]})
        nav = r.get("nav", {})
        if not nav.get("ok"):
            self.log("[ref] nav_goal %s failed: %s; walking straight" % ((round(x, 3), round(y, 3)), nav))
            self.c.command({"base_goal": [float(x), float(y), float(yaw)]})
        t0 = self.c.sim_time
        while self.c.sim_time - t0 < timeout and not self.c.done:
            self.run(25)
            st = self.c.nav.status
            if st != "moving":
                break
        if precise:
            self.step = "base goal (%.2f, %.2f, %.2f)" % (x, y, yaw)
            self.c.command({"base_goal": [float(x), float(y), float(yaw)]})
            t1 = self.c.sim_time
            while self.c.sim_time - t1 < 8.0 and not self.c.done:
                self.run(10)
                b = self.base()
                if math.hypot(b[0] - x, b[1] - y) < 0.004 and abs(wrap(b[2] - yaw)) < 0.008:
                    break
            self.run(20)
        if tuck:
            self.tuck(settle=0.8, on_hit="skip")   # arrived: free arms home (where every stance study kept them)
        b = self.base()
        err = math.hypot(b[0] - x, b[1] - y)
        if err > 0.02:
            self.log("[ref] walk to (%.3f, %.3f, %.2f) ended %.3f m off (nav %s)" % (x, y, yaw, err, self.c.nav.status))
        return err

    def floor_clearance(self, x, y):
        """The distance (m) from (x, y) to the nearest obstacle on the navigation stack's floor map (racks, open
        doors, the cart)."""
        from scipy import ndimage
        from .nav import RES
        nv = self.c.nav
        occ = nv.occupancy()
        dist = ndimage.distance_transform_edt(~occ) * RES
        i, j = nv._cell(x, y)
        if not (0 <= i < dist.shape[0] and 0 <= j < dist.shape[1]):
            return 0.0
        return float(dist[i, j])

    # ------------------------------------------------------------------------------------------------ checked motion
    def _check_setup(self):
        """A private copy of the model whose robot links (hands excepted), loose parts and tools carry a clearance
        margin, with scratch data: moves are dry-run on it (DryRun) before they are commanded."""
        if hasattr(self, "_mc"):
            return
        m = self.c.m
        self._mc = copy.copy(m)
        self._dc = mujoco.MjData(self._mc)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "" for i in range(m.nbody)]
        self._hand_body = np.array([any(k in n for k in ("hand", "palm", "thumb", "index", "middle", "ring", "little"))
                                    for n in names])
        self._names = names
        self._arm_of = {i: ("left" if "/left_" in n else "right") for i, n in enumerate(names)
                        if n.startswith("g1/") and any(k in n for k in ("shoulder", "elbow", "wrist"))}
        loose = set(int(b) for b in self.c.pool.body.values())
        loose |= {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tool_" + t) for t in P.TOOLS}
        for g in range(m.ngeom):
            b = m.geom_bodyid[g]
            if (self._robot_body[b] and not self._hand_body[b]) or int(b) in loose:
                self._mc.geom_margin[g] = CHECK_MARGIN

    def _hits_at(self, sides, body, held_b):
        """Contacts within CHECK_MARGIN on the scratch data (its qpos set): {(link or part, other): distance}."""
        m, mc, dc = self.c.m, self._mc, self._dc
        mc.geom_contype[:] = m.geom_contype
        mc.geom_conaffinity[:] = m.geom_conaffinity
        mujoco.mj_fwdPosition(mc, dc)
        holding = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in (self.holds("left"), self.holds("right"))
                   if n}
        out = {}

        def add(a, b, dist):
            key = (self._names[a], self._names[b])
            if key not in out or dist < out[key]:
                out[key] = dist

        for i in range(dc.ncon):
            con = dc.contact[i]
            if con.dist > CHECK_MARGIN - 1e-4:
                continue
            b1, b2 = int(mc.geom_bodyid[con.geom1]), int(mc.geom_bodyid[con.geom2])
            r1, r2 = self._robot_body[b1], self._robot_body[b2]
            if r1 and r2:
                continue
            if r1 or r2:
                rb, ob, og = (b1, b2, con.geom2) if r1 else (b2, b1, con.geom1)
                if self._hand_body[rb]:
                    continue
                if ob in holding:
                    sd = held_b.get(ob)              # a held free part against the robot's own body
                    if sd is None or sd not in sides or (self._arm_of.get(rb) == sd and "wrist" in self._names[rb]):
                        continue
                    add(ob, rb, float(con.dist))
                    continue
                if ob == 0 and mc.geom_type[og] == mujoco.mjtGeom.mjGEOM_PLANE:
                    continue
                if not body and self._arm_of.get(rb) not in sides:
                    continue
                add(rb, ob, float(con.dist))
            else:
                for pb, ob in ((b1, b2), (b2, b1)):
                    sd = held_b.get(pb)
                    if sd is not None and sd in sides and ob not in holding:
                        add(pb, ob, float(con.dist))
                        break
        return out

    @staticmethod
    def _tgt(pos, quat=None, path="line", frame="room"):
        pos = [float(v) for v in pos]
        if quat is None:
            return {"pos": pos, "frame": frame, "path": path}
        return {"pose": pos + [float(v) for v in quat], "frame": frame, "path": path}

    def try_moves(self, moves, sides, body=False, held=True):
        """Dry-run moves in order: ("arm", side, target) or ("posture", command). The first hit, or None. With held
        False the parts in the hands are not checked (a part that goes into its slot comes close to it on purpose)."""
        import time as _time
        t_wall = _time.time()
        try:
            return self._try_moves(moves, sides, body, held)
        finally:
            self.dry_wall = getattr(self, "dry_wall", 0.0) + _time.time() - t_wall
            self.dry_count = getattr(self, "dry_count", 0) + 1

    def _try_moves(self, moves, sides, body=False, held=True):
        D = DryRun(self, held=held)
        base = D.baseline(sides, body)
        if getattr(self, "debug_dry", False) and base:
            self.log("[dry] baseline %s" % {k: round(1000 * v, 1) for k, v in base.items()})
        moved = set()
        for mv in moves:
            if mv[0] == "arm":
                moved.add(mv[1])
                D.active = tuple(sorted(moved))
                h = None
                if self.heavy(mv[1]):
                    # a heavy part's move is a stream of near targets (_stream_pose / _stream_q): replayed as such
                    tg = mv[2]
                    if "q" in tg:
                        stream = self._stream_q_targets(mv[1], D.q[self.c.robot.arm_q[mv[1]]].copy(), tg["q"])
                    elif tg.get("path", "line") == "line" and tg.get("frame", "base") == "room":
                        p0, R0 = D.site_pose(mv[1])
                        pq = tg.get("pose") or tg.get("pos")
                        stream = self._stream_targets(mv[1], p0, mat_quat(R0), pq[:3],
                                                      pq[3:7] if "pose" in tg else None)
                    else:
                        stream = []
                    for k_, cm in enumerate(stream):
                        D.command({mv[1] + "_arm": cm})
                        D._tick()
                        D._tick()
                        if k_ % 3 == 2 or k_ == len(stream) - 1:      # (checked every 60 ms of the stream)
                            h = D.hits(sides, body)
                            if h:
                                break
                if not h:
                    D.command({mv[1] + "_arm": mv[2]})
                    h = D.run(lambda D_, s_=mv[1]: D_.arm_done(s_), sides, body=body)
            else:
                D.active = ("left", "right")
                D.command(mv[1])
                h = D.run(lambda D_: D_.posture_done(), sides, body=body)
            if h:
                return h
        self._last_dry = D
        return None

    def plan_reach(self, side, pos, quat=None, path="line", held=True):
        """Via points (possibly none) that take the hand to (pos, quat) with the arm and what the hand holds clear of
        everything (dry runs of the controller), or None."""
        tgt = self._tgt(pos, quat, path)
        hit = self.try_moves([("arm", side, tgt)], (side,), held=held)
        if hit is None:
            return []
        self._last_hit = hit[0]
        b = self.base()
        back = rz(b[2]) @ np.array([-1.0, 0.0, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        p0, q0 = self.site(side), mat_quat(self.site_R(side))
        pos = np.asarray(pos, float)
        top = max(p0[2], pos[2]) + 0.12
        cands = [[p0 + up * 0.10],
                 [np.array([p0[0], p0[1], top]), np.array([pos[0], pos[1], top])],
                 [p0 + back * 0.12, np.array([p0[0], p0[1], top]) + back * 0.12, np.array([pos[0], pos[1], top])],
                 [p0 + up * 0.20], [p0 + back * 0.15], [p0 + back * 0.15 + up * 0.15],
                 [p0 + up * 0.10, p0 + up * 0.10 + back * 0.15]]
        for vias in cands:
            moves = [("arm", side, self._tgt(v, q0)) for v in vias] + [("arm", side, tgt)]
            if self.try_moves(moves, (side,), held=held) is None:
                return vias
        return None

    @staticmethod
    def _side_of(name, holds):
        if "/left_" in name:
            return "left"
        if "/right_" in name:
            return "right"
        for sd, n in holds.items():
            if n == name:
                return sd
        return None

    def retreat(self, dist=0.30, dirs=("back", "back_left", "back_right", "left", "right")):
        """A step away (straight back first) with the arms (and what the hands hold) kept where they are relative to
        the base, where the way is clear (dry run along it, every link). True if it stepped."""
        b = self.base()
        Rb = rz(b[2])
        vec = {"back": (-1.0, 0.0), "back_left": (-0.7, 0.7), "back_right": (-0.7, -0.7), "left": (0.0, 1.0),
               "right": (0.0, -1.0)}
        why = {}
        for d_ in (dist, 0.6 * dist, 0.4 * dist):
            for k in dirs:
                v = Rb @ np.array([vec[k][0], vec[k][1], 0.0])
                D = DryRun(self)
                D.baseline(("left", "right"), True)
                ok = True
                for t in np.linspace(0.25, 1.0, 4):
                    D.set_base(b[0] + v[0] * d_ * t, b[1] + v[1] * d_ * t, b[2])
                    h = D.hits(("left", "right"), True)
                    if h:
                        why[(k, round(d_, 2))] = h[0]
                        ok = False
                        break
                if not ok:
                    continue
                for sd in ("left", "right"):
                    if self.c.ctrl.arm_mode[sd] == "pose":
                        self.hold_in_base(sd)
                gx, gy = b[0] + v[0] * d_, b[1] + v[1] * d_
                self.step = "retreat %s %.2f m" % (k, d_)
                self.c.command({"base_goal": [float(gx), float(gy), float(b[2])]})
                t0 = self.c.sim_time
                while self.c.sim_time - t0 < 6.0 and not self.c.done:
                    self.run(10)
                    bb = self.base()
                    if math.hypot(bb[0] - gx, bb[1] - gy) < 0.01:
                        break
                self.run(20)
                return True
        self.log("[ref] retreat: no clear way (%s)" % list(why.items())[:4])
        return False

    # ------------------------------------------------------------------------------------------------ the arms
    def reach(self, side, pos, quat=None, tol=0.004, timeout=8.0, path="line", checked=True, check_held=True,
              optional=False):
        """The hand to pos (and quat). Checked: the straight path is tried on the clearance model first; when it would
        bring an arm link near something, the hand goes by via points (up, back towards the body) instead."""
        if self.very_heavy(side) and path == "line" and \
                (float(np.linalg.norm(self.site(side) - np.asarray(pos, float))) > 0.08 or
                 not self.reachable(side, pos, quat, tol=0.005)):
            # not on the arm's present branch: a straight move would make the controller switch branches on the way
            # (the arm swings, the part flies). A joint move instead, to a configuration that reaches it (global IK),
            # streamed and dry-run with the part; none clear: the hand stays
            c_ = self.c
            qa, ep, er = c_.ctrl.ik.solve_global(c_.d.qpos.copy(), side, np.asarray(pos, float),
                                                 None if quat is None else np.asarray(quat, float),
                                                 c_.d.qpos[c_.robot.arm_q[side]].copy())
            mv = ("arm", side, {"q": [float(v) for v in qa]})
            if ep <= 0.01 and (er <= 0.08 or quat is None) and \
                    (not (checked and self.checked) or self.try_moves([mv], (side,), held=check_held) is None):
                self.step = "reach %s (%.3f, %.3f, %.3f) [joint move]" % (side, pos[0], pos[1], pos[2])
                self._stream_q(side, qa)
                self.c.command({side + "_arm": mv[2]})
                t0 = self.c.sim_time
                while self.c.sim_time - t0 < timeout and not self.c.done:
                    self.run(10)
                    if float(np.max(np.abs(c_.d.qpos[c_.robot.arm_q[side]] - qa))) < 0.03 and \
                            float(np.max(np.abs(c_.ctrl.arm_vel[side]))) < 0.05:
                        break
                return float(np.linalg.norm(self.site(side) - np.asarray(pos, float)))
            self.log("[ref] reach %s (%.3f, %.3f, %.3f): out of the arm's reach (or no clear joint move) with %s in "
                     "hand; the hand stays" % (side, pos[0], pos[1], pos[2], self.holds(side)))
            return float(np.linalg.norm(self.site(side) - np.asarray(pos, float)))
        vias = None
        if checked and self.checked:
            vias = self.plan_reach(side, pos, quat, path, held=check_held)
            if vias is None and optional:
                self.log("[ref] reach %s: no clear path (%s); skipped" % (side, getattr(self, "_last_hit", None)))
                return float(np.linalg.norm(self.site(side) - np.asarray(pos, float)))
            if vias is None:
                self.log("[ref] reach %s: no clear path (%s); going straight" % (side, getattr(self, "_last_hit", None)))
            elif vias:
                q0 = mat_quat(self.site_R(side))
                for v in vias:
                    self.reach(side, v, q0, tol=0.01, timeout=4.0, checked=False)
        tgt = {"pose": list(pos) + list(quat), "frame": "room", "path": path} if quat is not None else \
            {"pos": list(pos), "frame": "room", "path": path}
        self.step = "reach %s (%.3f, %.3f, %.3f)%s" % (side, pos[0], pos[1], pos[2], "" if checked else " [via]")
        shadow = getattr(self, "shadow", False) and checked and self.checked and vias == [] and \
            getattr(self, "_last_dry", None) is not None
        if shadow:
            path_q = np.array([q_[side] for t_, q_, _z, _w in self._last_dry.trace if side in q_])
            worst = (0.0, 0.0)
        v_tol = 0.02
        if self.heavy(side):
            # the arm's servos give a few millimetres under a heavy part: a move at the default tolerance has arrived
            # within that, once still (a precise move asks for its own tolerance and keeps it)
            if tol >= 0.004:
                tol, v_tol = max(tol, 0.008), 0.05
            if path == "line":
                self._stream_pose(side, pos, quat)
                # then held in joint space at the stream's end (a pose target tracked by the IK feeds the torso's
                # sway under a heavy part back into the arm: it rings)
                c_ = self.c
                qa, ep, er = c_.ctrl.ik.solve(c_.d.qpos.copy(), side, np.asarray(pos, float),
                                              None if quat is None else np.asarray(quat, float),
                                              np.asarray(c_.ctrl.arm_cmd[side], float), iters=60, w_home=0.0)
                if ep < 0.01:
                    tgt = {"q": [float(v) for v in qa]}
        # a move that mostly turns the hand has arrived when it has turned too (its position is there from the start)
        turn = quat is not None and 2.0 * math.acos(min(1.0, abs(float(np.dot(mat_quat(self.site_R(side)),
                                                                             np.asarray(quat, float)))))) > 0.2
        self.c.command({side + "_arm": tgt})
        t0 = self.c.sim_time
        err = 1.0
        while self.c.sim_time - t0 < timeout and not self.c.done:
            self.run(10)
            if shadow and len(path_q):
                # how far the arm is from the dry run's path (the nearest configuration on it: a servo that lags
                # behind the path is still on it)
                dev = float(np.min(np.max(np.abs(path_q - self.c.d.qpos[self.c.robot.arm_q[side]]), axis=1)))
                if dev > worst[0]:
                    worst = (dev, self.c.sim_time - t0)
            err = float(np.linalg.norm(self.site(side) - np.asarray(pos)))
            rot_ok = not turn or 2.0 * math.acos(min(1.0, abs(float(np.dot(mat_quat(self.site_R(side)),
                                                                           np.asarray(quat, float)))))) < 0.05
            if err < tol and rot_ok and np.linalg.norm(self.c.ctrl.arm_vel[side]) < v_tol:
                break
        if shadow and worst[0] > 0.08:
            self.log("[shadow] reach %s to %s: the arm left the dry run's path by %.2f rad at %.2f s" % (
                side, np.round(pos, 3), worst[0], worst[1]))
        return err

    def settle(self, side, pos, quat=None, t_min=0.25, t_max=1.5, v_tol=0.004):
        """Command the hand to pos (and quat) and wait until the arm has come to rest (not until it is there: a
        closed loop corrects what the servo leaves)."""
        tgt = {"pose": list(pos) + list(quat), "frame": "room", "path": "line"} if quat is not None else \
            {"pos": list(pos), "frame": "room", "path": "line"}
        self.step = "settle %s" % side
        self.c.command({side + "_arm": tgt})
        t0 = self.c.sim_time
        while self.c.sim_time - t0 < t_max and not self.c.done:
            self.run(10)
            if self.c.sim_time - t0 >= t_min and np.linalg.norm(self.c.ctrl.arm_vel[side]) < v_tol:
                break

    def approach(self, x, y, yaw, work=(), back=0.30, carry="close", posture=None):
        """To a close-work stance (by a tray, a rack's rear, the cart): walk to a point `back` short of it (along its
        heading), raise the working hands there (where there is room) and bring the free ones home, then step in
        with the arms held as they are (the way in dry-run checked; the hands of `work` stay up, above whatever is
        at work height). Returns the position error."""
        f = rz(yaw) @ np.array([1.0, 0.0, 0.0])
        for bk in (back, 0.6 * back, 0.3 * back, 0.0):    # a point short of it where the robot stands clear
            D = DryRun(self)
            D.set_base(x - f[0] * bk, y - f[1] * bk, yaw)
            if not D.raw((), True):
                break
        back = bk
        px_, py_ = x - f[0] * back, y - f[1] * back
        self.walk(px_, py_, yaw, carry=carry)
        for sd in ("left", "right"):
            if sd in work:
                self.raise_hand(sd)
            elif self.holds(sd) is None:
                self.arms(**{"q_" + sd: ARM_HOME[sd]}, settle=0.8)
        if posture is not None:
            # the stance's own posture taken here, short of it, with the free arms home: the stance study checked
            # exactly that body at the stance, and the robot steps in already so (a lean at the stance would sweep
            # the arms through whatever is in front)
            self.posture(stand=posture[0], waist=posture[1])
        D = DryRun(self)
        D.baseline(("left", "right"), True)
        b = self.base()
        hit = None
        for t in np.linspace(0.2, 1.0, 5):
            D.set_base(b[0] + (x - b[0]) * t, b[1] + (y - b[1]) * t, b[2] + wrap(yaw - b[2]) * t)
            h = D.hits(("left", "right"), True)
            if h:
                hit = h[0]
                break
        if hit is not None:
            self.log("[ref] approach (%.2f, %.2f): stepping in touches %s; stepping in anyway" % (x, y, hit))
        for sd in ("left", "right"):
            if self.c.ctrl.arm_mode[sd] == "pose":
                self.hold_in_base(sd)
        self.step = "step in (%.2f, %.2f, %.2f)" % (x, y, yaw)
        self.c.command({"base_goal": [float(x), float(y), float(yaw)]})
        t1 = self.c.sim_time
        while self.c.sim_time - t1 < 8.0 and not self.c.done:
            self.run(10)
            bb = self.base()
            if math.hypot(bb[0] - x, bb[1] - y) < 0.004 and abs(wrap(bb[2] - yaw)) < 0.008:
                break
        self.run(20)
        bb = self.base()
        return math.hypot(bb[0] - x, bb[1] - y)

    def arm_to(self, side, pos, quat, seed=None, check_held=True, tol_ik=0.04):
        """The arm into a stance study's own configuration (its branch: the elbow where the study checked it) with the
        hand at (pos, quat): the IK seeded by and pulled towards `seed` (the table's arm at that point of its chain),
        reached by a joint move whose dry run is clear; when the joint move would sweep the arm through something
        (a cart's riser, a tray's wall), the same branch over the point (12 or 22 cm up) first and then straight down.
        The rest by a short Cartesian move (a local solve keeps the branch). Without a seed: a plain reach."""
        if seed is None:
            return self.reach(side, pos, quat, check_held=check_held)
        sq = np.asarray(seed, float)
        pos = np.asarray(pos, float)
        quat = np.asarray(quat, float)
        up = np.array([0.0, 0.0, 1.0])
        last = None
        for dz in (0.0, 0.12, 0.22):
            tgt = pos + up * dz
            qa, ep, er = self.c.ctrl.ik.solve(self.c.d.qpos.copy(), side, tgt, quat, sq, iters=80, q_rest=sq)
            if ep > tol_ik or er > 0.15:
                last = "the study's branch reaches only to %.1f mm" % (1000 * ep)
                continue
            mv = ("arm", side, {"q": [float(v) for v in qa]})
            hit = self.try_moves([mv], (side,), held=check_held)
            if hit is not None:
                last = "the joint move would touch %s" % (hit[0],)
                continue
            self.step = "arm_to %s (study branch, up %.2f)" % (side, dz)
            if self.heavy(side):
                self._stream_q(side, qa)
            self.c.command({side + "_arm": {"q": [float(v) for v in qa]}})
            t0 = self.c.sim_time
            q_tol = 0.03 if self.heavy(side) else 0.01   # (the servos give a little under a heavy part)
            while self.c.sim_time - t0 < 4.0 and not self.c.done:
                self.run(10)
                if float(np.max(np.abs(self.c.d.qpos[self.c.robot.arm_q[side]] - qa))) < q_tol and \
                        float(np.max(np.abs(self.c.ctrl.arm_vel[side]))) < 0.05:
                    break
            if dz > 0.0 or ep > 0.003 or er > 0.02:
                return self.reach(side, pos, quat, check_held=check_held)
            return float(np.linalg.norm(self.site(side) - pos))
        self.log("[ref] arm_to %s: %s; a plain reach" % (side, last))
        return self.reach(side, pos, quat, check_held=check_held)

    def step_to(self, x, y, yaw, timeout=6.0):
        """A short base move (a few centimetres, a small turn) with the arms held where they are relative to the
        base."""
        for sd in ("left", "right"):
            if self.c.ctrl.arm_mode[sd] == "pose":
                self.hold_in_base(sd)
        self.step = "step to (%.2f, %.2f, %.2f)" % (x, y, yaw)
        self.c.command({"base_goal": [float(x), float(y), float(yaw)]})
        t1 = self.c.sim_time
        while self.c.sim_time - t1 < timeout and not self.c.done:
            self.run(10)
            bb = self.base()
            if math.hypot(bb[0] - x, bb[1] - y) < 0.004 and abs(wrap(bb[2] - yaw)) < 0.008:
                break
        self.run(10)

    def raise_hand(self, side, z=1.22, fwd=0.26, out=0.16):
        """The hand (and what it holds) up in front of its shoulder, above any tray or cover, before the torso leans
        over the work (a hand left low would dip into it). Keeps the hand's orientation."""
        b = self.base()
        sgn = 1.0 if side == "left" else -1.0
        p = np.array([b[0], b[1], 0.0]) + rz(b[2]) @ np.array([fwd, sgn * out, 0.0])
        p[2] = z
        self.reach(side, p, mat_quat(self.site_R(side)), timeout=4.0)

    def best_quat(self, side, pos, candidates):
        ik = self.c.ctrl.ik
        best = None
        for q in candidates:
            qq, ep, er = ik.solve_global(self.c.d.qpos.copy(), side, np.asarray(pos, float), np.asarray(q, float),
                                         self.c.d.qpos[self.c.robot.arm_q[side]].copy())
            sc = ep + 0.05 * er
            if best is None or sc < best[0]:
                best = (sc, np.asarray(q, float))
        return best[1]

    def grip(self, side, close=True, settle=0.3):
        self.step = "grip %s %s" % (side, "close" if close else "open")
        if close and self.c.grips.state[side]["closed"]:
            # a hand left closed (a part that slipped out of it) takes nothing until it has opened again
            self.c.command({side + "_hand": "open"})
            self.run(int(0.2 / TICK))
        self.c.command({side + "_hand": "close" if close else "open"})
        self.run(int(settle / TICK))
        return self.holds(side)

    def feature(self, name, k=0):
        a, b, r = self.c.feature_world(name, k)
        return np.asarray(a), np.asarray(b), r

    # ------------------------------------------------------------------------------------------------ the belt
    def draw_tool(self, side, name, approach="top"):
        """A tool out of its belt holster; returns the hand orientation used (or None when the grip failed)."""
        c = self.c
        a, b, r = self.feature(name)
        hp, Rb = c.tools.belt_world(name)
        out = Rb[:, 1] * (1.0 if side == "right" else -1.0)
        cands = []
        if approach == "top":
            g = a + 0.80 * (b - a)
            down = np.array([0.0, 0.0, -1.0])
            for k in range(4):
                ap = down * math.cos(0.15 * k) + out * math.sin(0.15 * k)
                cands += [quat_from_axes(ap, Rb[:, 0]), quat_from_axes(ap, -Rb[:, 0]), quat_from_axes(ap, Rb[:, 1]),
                          quat_from_axes(ap, -Rb[:, 1])]
        else:
            g = a + 0.55 * (b - a)
            for k in range(3):
                ap = out * math.cos(0.5 * k) + np.array([0.0, 0.0, -math.sin(0.5 * k)])
                cands += [quat_from_axes(ap, Rb[:, 0]), quat_from_axes(ap, -Rb[:, 0])]
        q = self.best_quat(side, g, cands)
        ap = quat_mat(q)[:, 0]
        self.reach(side, g - 0.08 * ap, q)
        self.reach(side, g, q)
        if self.grip(side, True) != name:
            self.grip(side, False)
            self.log("[ref] could not take %s from the belt (the hand %.1f mm from its grip point, holstered %s)" % (
                name, 1000 * float(np.linalg.norm(self.site(side) - g)), c.tools.holstered(name)))
            return None
        self.reach(side, self.site(side) + np.array([0.0, 0.0, 0.16 if approach == "top" else 0.12]), q)
        self.reach(side, self.site(side) + 0.15 * Rb[:, 0] + np.array([0.0, 0.0, 0.06]), q)
        return q

    def stow_tool(self, side, name):
        """The held tool back into its belt holster (upright over the mouth, down, let go)."""
        c = self.c
        if self.holds(side) != name:
            return c.tools.holstered(name)
        self.posture(stand=WALK_STAND, waist=(0.0, 0.0, 0.08), settle=1.0)
        hp, Rb = c.tools.belt_world(name)
        b = c.tools.body[name]
        Rt = c.d.xmat[b].reshape(3, 3)
        Rs = self.site_R(side)
        rel = Rt.T @ (self.site(side) - c.d.xpos[b])
        q = mat_quat(Rb @ Rt.T @ Rs)
        tgt = hp + Rb @ rel
        self.reach(side, tgt + np.array([0.0, 0.0, 0.14]), q)
        self.reach(side, tgt + np.array([0.0, 0.0, 0.01]), q)
        for _ in range(4):
            hp, Rb = c.tools.belt_world(name)
            e = (hp + np.array([0.0, 0.0, 0.004])) - c.d.xpos[b]
            if np.linalg.norm(e) < 0.004:
                break
            self.reach(side, self.site(side) + e, q, tol=0.001, timeout=2.0)
        self.grip(side, False)
        self.run(30)
        out = Rb[:, 1] * (1.0 if side == "right" else -1.0)
        self.reach(side, self.site(side) - 0.07 * out + np.array([0.0, 0.0, 0.03]), q)
        if not c.tools.holstered(name):
            self.log("[ref] %s did not go back into its holster" % name)
        return c.tools.holstered(name)
