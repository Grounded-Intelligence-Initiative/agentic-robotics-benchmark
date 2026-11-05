"""Lacing skills of the reference controller (the oracle): seat a shoe in the cradle, lace it criss-cross through
rows 2..n (the batch has n = 2 in this version: row 1 arrives threaded and tacked) with both lace ends, unseat it.
Privileged: the skills read the exact scene (`rig.sc`) and drive the two arms through the cell's own controllers,
attach rules and physics tick.

Shoe frame (shoecell/shoe.py): x toward the toe, y toward side L, z up, origin at the centre of the sole bottom; row 1
is nearest the toe; the rings sit on the flap tops at z_f; the cavity floor is at z_floor; the throat slot between
the flaps is |y| < g/2.

Strategy
  seat    grasp a shoe handle (heel or toe counter, whichever is nearer the gripper base) from above with the fingers
          closing along the shoe's length, lift, right an on-side shoe in the air (the weld keeps the shoe fixed to
          the hand, so a 90 deg roll of the hand about the shoe's length rights it; a close neighbour shoe is kept
          clear by the other handle and a sideways shift), carry it round the gripper base to the cradle and set it
          down on the seat with a closed-loop xy/yaw correction read from the true shoe pose. The hook arm stays
          parked (far heel side) throughout: the tails trail behind the shoe on the gripper's side of the cradle.
  lace    for each tail in turn (A then B; the sides follow episode._expected_sequence; the throat-return pattern):
          the gripper picks the aglet from the bench (truth pose; beside the seated shoe the hand's long side is kept
          parallel to the shoe and the tcp shifted outward, never onto the upper), or the hook fetches it there and
          hands it over in free space when it lies in the hook arm's zone; the gripper drops it IN through row 2 on
          the side opposite to that tail's row-1 exit (aligned on the hole with bounded corrections); the hook enters
          the cavity through the throat, catches the aglet under the flap, rises out through the throat slot and
          draws the lace until the segment from row 1 is snug (arc <= 1.25 x straight, a 6 mm margin; the scorer
          allows 1.5 x, the cord bent through the ring springs back 0.5-2.5 cm when the tension drops), then lays the
          aglet on the bench beside the shoe. One arm works at the cradle at a time; a lost aglet is re-acquired.
  unseat  release the clamp, grasp the heel (else the toe) and carry the shoe back to its own arrival spot on the
          infeed tray (the delivery spots are clear of each other and of the lid), sole down, toe along +x.
"""

import math

import mujoco
import numpy as np

import box_skills
from primitives import EpisodeOver, down_quat, hook_quat, rot_quat
from shoecell.control import ik_solve_seeds, mat_to_quat, quat_mul, quat_to_mat

__all__ = ["seat_shoe", "lace_shoe", "unseat_shoe"]

Z = np.array([0.0, 0.0, 1.0])
LEFT_BASE = np.array([-0.05, 0.30])
RIGHT_BASE = np.array([-0.55, -0.30])
HANDOVER = np.array([-0.27, -0.06, 0.11])    # free-space handover point (hook tip end): both arms reach it and their
#                                               wrists stay apart (gripper leaning toward +y, hook foot toward +y)
PULL_DIR = np.array([0.8, -0.6]) / math.hypot(0.8, 0.6)   # tails are pulled out toward here (both arms reach the spot)
HOOK_FOOT_FREE = math.pi / 2                  # foot yaw for free-space catches and the handover (no wrist turn in between)
HOOK_WAIT = np.array([0.15, -0.26, 0.05])     # hook tip waits here (relative to the seat; clear of the staging spot) while the shoe is seated: the
#                                               tails trail behind the shoe's last leg and slide on; the hook catches one at once
FOOT, TIP, SHAFT_R = 0.014, 0.012, 0.0025
LEAN = 0.70              # handover: the gripper approach tilted outward (rad); the palm then clears the hook shaft
PHI_SIDE = 1.33          # hook foot angle from the shoe's x toward a side (76 deg): the shaft stays inside the throat and
#                          the wrist (joint 7, q7 ~ -1.3 - phi at the cradle) stays inside its range for both sides
AGLET_GRIP_DZ = 0.0085   # the aglet centre this far beyond the tcp when the gripper closes on it (pinch box: <= 12 mm)
SNUG_RATIO = 1.25
UNSEAT_OFFSET = (0.0, 0.14)     # where an unseated shoe goes, relative to the cradle seat (shoe 1: a further +0.10 along y)


# ---------------------------------------------------------------------------------------------------- small helpers
def _guard(rig):
    """Raise EpisodeOver as soon as the server has ended the episode or the physics diverged (MuJoCo auto-reset)."""
    if rig.s.done or getattr(rig.sc, "physics_fault", False):
        raise EpisodeOver(rig.s.done_reason or "physics_fault")


def _log(rig, msg):
    rig.log("[lace] " + msg)
    _guard(rig)


def _wait(rig, seconds):
    rig.wait(seconds)
    _guard(rig)


def _ensure_hold_width(rig):
    """WORKAROUND for a cell bug: server._grasp_logic / _release_left call `Arm.hold_width(width)` on the left arm,
    which neither physics.Arm nor control.ArmController defines (every successful grasp raises AttributeError inside
    physics_tick). Until the cell defines it, give the arm instance a minimal one: stop the finger ramp at the pinch
    width of the attached handle (None = nothing to do, the release command already re-targets the fingers)."""
    return      # (fixed in the cell: physics.Arm.hold_width exists)


def _sg(side):
    return 1.0 if side == "L" else -1.0


def _other(side):
    return "R" if side == "L" else "L"


def _rot_err(R_goal, R):
    c = (np.trace(R_goal @ R.T) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def _yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def _rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _lean_quat(yaw, sg, a):
    """Gripper closing along the world direction `yaw`, approach tilted by `a` so that the hand body leans toward
    sg * (the left-hand normal of yaw)."""
    ex = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return quat_mul(rot_quat(ex, -sg * a), down_quat(yaw))


class _Geom(object):
    """Static shoe dimensions (shoe frame)."""

    def __init__(self, rig, s):
        d = rig.sc.shoes[s]["dims"]
        self.L, self.W, self.Wc, self.g = d["L"], d["W"], d["Wc"], d["g"]
        self.y_e, self.z_f, self.z_floor = d["eyelet_y"], d["z_f"], d["z_floor"]
        self.x_rows = [float(x) for x in d["x_rows"]]
        self.x_hc, self.x_tc = d["x_hc"], d["x_tc"]
        self.z_hc_top, self.z_tc_top = d["z_hc_top"], d["z_tc_top"]
        self.y_shaft = self.g / 2 - SHAFT_R - 0.002        # |y| limit of the hook shaft centre inside the throat
        self.n_rows = int(d["n_rows"])

    def ring(self, row, side):
        return np.array([self.x_rows[row - 1], _sg(side) * self.y_e, self.z_f])


class _Frame(object):
    """The shoe body pose now."""

    def __init__(self, rig, s):
        p, q = rig.sc.shoe_pose(s)
        self.p = np.asarray(p, float)
        self.R = quat_to_mat(np.asarray(q, float))
        self.yaw = _yaw_of(self.R)
        self.ex, self.ey = self.R[:, 0].copy(), self.R[:, 1].copy()

    def w(self, local):
        return self.p + self.R @ np.asarray(local, float)

    def l(self, world):
        return self.R.T @ (np.asarray(world, float) - self.p)


def _aglet(rig, s, which):
    """(body name, centre, axis pointing out of the lace) of aglet `which` of the pair's lace s."""
    for w, bname, c, ax in rig.sc.aglet_info(s):
        if w == which:
            return bname, np.asarray(c, float), np.asarray(ax, float)
    raise KeyError(which)


def _tails(rig, s):
    """(passes, snug, len_a, len_b, side_a, side_b): the lace passes and the free length of each tail (from its aglet to
    its outermost ring crossing); sides are those of the row-1 passes (None when row 1 is not threaded both ways)."""
    passes, snug = rig.sc.lace_passes(s, s)
    total = float(np.sum(rig.sc.lace_lengths[s]))
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    len_a = passes[0][3] * total if passes else total
    len_b = (1.0 - passes[-1][3]) * total if passes else 0.0
    side_a = passes[r1[0]][1] if len(r1) == 2 else None
    side_b = passes[r1[1]][1] if len(r1) == 2 else None
    return passes, snug, len_a, len_b, side_a, side_b


def _chain(rig, s, which):
    """This tail's passes walked from row 1 outward: [(row, side, direction, t)] with direction +1 = out (inside ->
    outside), -1 = in; empty when row 1 is not threaded."""
    passes, _ = rig.sc.lace_passes(s, s)
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    if len(r1) != 2:
        return []
    if which == "a":
        return [(r, sd, -d, t) for (r, sd, d, t) in reversed(passes[:r1[0]])]
    return list(passes[r1[1] + 1:])


def _my_snug(rig, s, which, geo):
    """{row: slack ratio} of this tail's segments between consecutive rows (arc / straight), rows >= 2."""
    passes, _ = rig.sc.lace_passes(s, s)
    total = float(np.sum(rig.sc.lace_lengths[s]))
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    if len(r1) != 2:
        return {}
    chain = list(reversed(passes[:r1[0] + 1])) if which == "a" else passes[r1[1]:]
    out = {}
    for a, b in zip(chain[:-1], chain[1:]):
        if b[0] == a[0] + 1 and b[0] >= 2:
            arc = abs(b[3] - a[3]) * total
            straight = float(np.linalg.norm(geo.ring(b[0], b[1]) - geo.ring(a[0], a[1])))
            out[b[0]] = arc / max(straight, 1e-6)
    return out


SNUG_THROAT_M = 0.03


def _tail_snug(rig, s, which, geo, row):
    """(ok, arc, allowed) for this tail's segment from row-1 to row (scorer rule: arc <= 1.25 x straight, plus 3 cm
    when the segment dips under a flap and returns through the slot, i.e. from row 2 on); (None, ..) when the segment
    is not threaded."""
    passes, _ = rig.sc.lace_passes(s, s)
    total = float(np.sum(rig.sc.lace_lengths[s]))
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    if len(r1) != 2:
        return None, 0.0, 0.0
    chain = list(reversed(passes[:r1[0] + 1])) if which == "a" else passes[r1[1]:]
    for a, b in zip(chain[:-1], chain[1:]):
        if a[0] == row - 1 and b[0] == row:
            arc = abs(b[3] - a[3]) * total
            straight = float(np.linalg.norm(geo.ring(b[0], b[1]) - geo.ring(a[0], a[1])))
            allowed = SNUG_RATIO * straight + (SNUG_THROAT_M if a[0] >= 2 else 0.0)
            return arc <= allowed - 0.006, arc, allowed
    return None, 0.0, 0.0


def _passes_str(rig, s):
    passes, snug = rig.sc.lace_passes(s, s)
    return _fmt_passes(passes, float(np.sum(rig.sc.lace_lengths[s])))


def _fmt_passes(passes, total):
    return " ".join("%d%s%s@%.3f" % (r, sd, "+" if d > 0 else "-", t * total) for r, sd, d, t in passes)


# ---------------------------------------------------------------------------------------------------- motion
def _loose(bname):
    return bname.startswith(("shoe", "lace", "box", "lid"))


def _ik(rig, name, pos, quat, refs=None, rot_tol=math.radians(1.0)):
    """A joint solution for the tcp pose that is free of contacts with the static cell (contacts with the loose objects,
    e.g. the held shoe, are tolerated), or None."""
    arm = rig.sc.arms[name]
    R = quat_to_mat(np.asarray(quat, float))
    refs = [arm.q(), arm.home_q] + list(refs or [])
    best = None
    for qr in refs:
        q, pe, re, ok, cols = ik_solve_seeds(arm, np.asarray(pos, float), R, q_ref=qr)
        if ok and not cols:
            return q
        if ok and all(_loose(b1) or _loose(b2) for b1, b2, _ in cols) and best is None:
            best = q
    if best is not None:
        return best
    for qr in refs:
        q, pe, re, ok, cols = ik_solve_seeds(arm, np.asarray(pos, float), R, q_ref=qr, collision_free=False)
        if ok and all(_loose(b1) or _loose(b2) for b1, b2, _ in cols):
            return q
    if rot_tol > math.radians(1.0):
        for qr in refs:
            q, pe, re, ok, cols = ik_solve_seeds(arm, np.asarray(pos, float), R, q_ref=qr, rot_tol=rot_tol)
            if ok and all(_loose(b1) or _loose(b2) for b1, b2, _ in cols):
                return q
    return None


def _arm_contact_force(rig, name, thr):
    """The (body, body, force) of the first contact where a link or pad of arm `name` presses on something else
    with more than `thr` N, else None (the position controller winds up force against an obstacle; the cell flags
    > 30 N on a link as a safety violation). The arm's own link-link contacts and its tool are ignored."""
    sc, m, d = rig.sc, rig.m, rig.d
    gc = sc.geom_class
    f6 = np.zeros(6)
    pre = name + "/"
    for i in range(d.ncon):
        con = d.contact[i]
        c1, c2 = gc[con.geom1], gc[con.geom2]
        if c1 in (1, 2, 3) or c2 in (1, 2, 3):
            g = con.geom1 if c1 in (1, 2, 3) else con.geom2
            b1 = m.body(m.geom_bodyid[g]).name
            if not b1.startswith(pre):
                continue
            other = con.geom2 if g == con.geom1 else con.geom1
            b2 = m.body(m.geom_bodyid[other]).name
            if b2.startswith(pre):
                continue                                  # the arm's own geoms (adjacent links, hand/fingers, tool)
            mujoco.mj_contactForce(m, d, i, f6)
            if abs(float(f6[0])) > thr:
                return (b1, b2, float(f6[0]))
    return None


def _cart(rig, name, pos, quat, tol, rot_tol, timeout, stall_s=0.6):
    arm = rig.sc.arms[name]
    if arm.mode != "cartesian":
        arm.set_mode("cartesian")
    arm.command_cartesian(pos, quat)
    Rg = quat_to_mat(quat)
    n = int(round(timeout / rig.s.tick_s))
    best, stalled = None, 0
    for i in range(n):
        rig.tick()
        p, R = arm.tcp_actual()
        ep = float(np.linalg.norm(pos - p))
        er = _rot_err(Rg, R)
        if ep <= tol and er <= rot_tol:
            return True
        if True:
            hit = _arm_contact_force(rig, name, 12.0)
            if hit is not None:
                _log(rig, "%s arm: %s presses %s with %.0f N on the way to %s, backing off" % (name, hit[0], hit[1], hit[2], np.round(pos, 3)))
                break
        m = ep + 0.05 * er
        if best is None or m < best - 0.0003:
            best, stalled = m, 0
        else:
            stalled += 1
        if stalled * rig.s.tick_s > stall_s and i > 20:
            break
    # not reached: re-target the pose the tool actually has, so that a blocked arm stops winding up force
    p, R = arm.tcp_actual()
    arm.command_cartesian(p, mat_to_quat(R))
    return False


def goto(rig, name, pos, quat, tol=0.003, rot_tol=0.03, timeout=4.0, joint_ok=True, settle=0.0):
    """Move `name`'s tcp to (pos, quat): the local Cartesian controller first; when it stalls (wrist flip, joint limit)
    and `joint_ok`, a seeded IK solution is travelled in joint space and the pose finished in Cartesian mode. Joint
    moves sweep the tool along a curved path, so `joint_ok` must be False near the shoe."""
    pos = np.asarray(pos, float)
    quat = np.asarray(quat, float)
    p_start = rig.tcp(name)[0]
    z_start = float(p_start[2])
    ok = _cart(rig, name, pos, quat, tol, rot_tol, timeout)
    allow = (joint_ok is True and pos[2] >= 0.17 and z_start >= 0.17) or joint_ok == "free" or \
            (joint_ok == "descend" and z_start >= 0.2 and pos[2] >= 0.11 and np.linalg.norm(pos[:2] - p_start[:2]) < 0.08)
    if not ok and allow:
        q = _ik(rig, name, pos, quat, rot_tol=max(rot_tol, math.radians(1.0)))
        if q is None:
            _log(rig, "goto %s: no IK solution at %s" % (name, np.round(pos, 3)))
        else:
            rig.move_joint(name, q, timeout=5.0)
            p, R = rig.tcp(name)
            if np.linalg.norm(p - pos) <= 2.0 * tol and _rot_err(quat_to_mat(quat), R) <= 2.0 * rot_tol:
                ok = True
            else:
                ok = _cart(rig, name, pos, quat, tol, rot_tol, 2.5)
    if not ok:
        p, _ = rig.tcp(name)
        _log(rig, "goto %s: not reached %s (err %.1f mm)" % (name, np.round(pos, 3), 1000 * np.linalg.norm(p - pos)))
    if settle > 0:
        rig.wait(settle)
    return ok


def _slerp(q0, q1, u):
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def _slow_goto(rig, name, pos, quat, v=0.2, w=0.9, tol=0.004, rot_tol=0.03):
    """Cartesian move with the target interpolated at <= v m/s and <= w rad/s (the controller's own limits are 0.5 m/s
    and 1.5 rad/s): a shoe in hand drags its frictionless lace, which whips and slips out of the rings when jolted."""
    arm = rig.sc.arms[name]
    if arm.mode != "cartesian":
        arm.set_mode("cartesian")
    p0, R0 = arm.tcp_actual()
    q0 = mat_to_quat(R0)
    pos = np.asarray(pos, float)
    quat = np.asarray(quat, float)
    dist = float(np.linalg.norm(pos - p0))
    ang = _rot_err(quat_to_mat(quat), R0)
    T = max(dist / v, ang / w, 0.05)
    n = int(math.ceil(T / 0.05))
    for k in range(1, n + 1):
        u = k / float(n)
        arm.command_cartesian(p0 + u * (pos - p0), _slerp(q0, quat, u))
        rig.tick(5)
        hit = _arm_contact_force(rig, name, 12.0)
        if hit is not None:
            _log(rig, "%s arm: %s presses %s with %.0f N during a slow move to %s, stopping" % (name, hit[0], hit[1], hit[2], np.round(pos, 3)))
            p, R = arm.tcp_actual()
            arm.command_cartesian(p, mat_to_quat(R))
            return False
    return _cart(rig, name, pos, quat, tol, rot_tol, 1.5)


def _park_hook(rig, near=None):
    """Hook arm out of the way (beyond the cradle's heel, high), in joint space from a cached configuration."""
    if rig.hook_loaded:
        rig.hook(False)
    arm = rig.sc.arms["right"]
    q_park = arm.home_q.copy()
    q_park[0] += 0.9                    # swung toward -x: tool at ~(-0.88, -0.04, 0.33), clear of the cradle and of both lay-down spots
    if float(np.max(np.abs(arm.q() - q_park))) < 0.02:
        return
    p, R = rig.tcp("right")
    if p[2] < 0.18:
        goto(rig, "right", np.array([p[0], p[1], 0.20]), mat_to_quat(R), tol=0.01, rot_tol=0.2, timeout=2.5, joint_ok=False)
    rig.move_joint("right", q_park, timeout=6.0)
    for _ in range(80):                       # settle within 0.02 rad before the gripper is allowed to move
        if float(np.max(np.abs(arm.q() - q_park))) < 0.02:
            break
        rig.tick(10)


def _hook_wait(rig, seat_p):
    """Put the (empty) hook tip low near where the tails will trail when the shoe is seated."""
    if rig.hook_loaded:
        rig.hook(False)
    quat = hook_quat(HOOK_FOOT_FREE)
    tip = seat_p + HOOK_WAIT
    goto(rig, "right", rig.tcp_for_tip(tip + Z * 0.15, quat), quat, tol=0.01, rot_tol=0.05, timeout=5.0)
    goto(rig, "right", rig.tcp_for_tip(tip, quat), quat, tol=0.004, rot_tol=0.04, timeout=4.0, joint_ok="free")


def _gripper_aside(rig, near):
    """Gripper out of the hook arm's way: straight up, then home in joint space."""
    rig.gripper(0.03, wait=False)
    _gripper_home(rig)


def _gripper_home(rig):
    arm = rig.sc.arms["left"]
    p, _ = rig.tcp("left")
    if p[0] > 0.10:
        # out over the infeed tray (a shoe set down or righted there): the joint mode clips each joint's step
        # separately and swept the hand down onto the lid tray on the way home; lift, then a checked straight line
        box_skills._go_home(rig)
    else:
        if p[2] < 0.22:
            rig.lift("left", 0.24 - p[2], timeout=2.0)
        rig.move_joint("left", arm.home_q, timeout=6.0)
    for _ in range(80):                       # settle within 0.02 rad before the hook is allowed to move
        if float(np.max(np.abs(arm.q() - arm.home_q))) < 0.02:
            break
        rig.tick(10)


# ---------------------------------------------------------------------------------------------------- shoe handling
STAGE = (-0.10, 0.0)          # staging spot (shoe origin, toe toward -y) for a re-grasp at the toe / righting; clear of the cradle
TAKEOVER_LEAN = 1.15          # ring-top take-over: hand approach axis leaning this much toward +y (its own base side), closing along x
GRIPPER_WAIT = np.array([-0.30, 0.24, 0.26])   # the gripper waits here (take-over orientation) while the hook works at the cradle
SETDOWN = {"L": (-0.44, 0.21), "R": (-0.46, 0.14)}   # where a finished tail's aglet is laid down (gripper side, -x/+y quadrant)
Q7_DYAW = -0.85               # measured: the left wrist's joint 7 changes by about -0.85 x the tool yaw change (tool down)
Q7_LIM = 2.55                 # keep |q7| below this when planning yaw changes (range +-2.9)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _tool_yaw(rig, name="left"):
    """World yaw of the tool's closing axis (tool +y)."""
    _, R = rig.tcp(name)
    return math.atan2(R[1, 1], R[0, 1])


def _handle_sites(rig, s):
    hs = rig.sc.shoe_handles(s)
    return {k: (np.asarray(v["pos"], float), float(v["pinch_width"])) for k, v in hs.items()}


def _best_handle(rig, s, exclude=None):
    """The shoe handle (heel or toe counter) at the most comfortable reach of the gripper arm (~0.40 m from its base;
    closer than ~0.25 m the arm cannot point the tool down)."""
    hs = _handle_sites(rig, s)
    cands = [k for k in hs if k != exclude] or list(hs)
    return min(cands, key=lambda k: abs(np.linalg.norm(hs[k][0][:2] - LEFT_BASE) - 0.40))


def _fix_held_contacts(rig):
    """WORKAROUND for a cell bug: scene.attach_left -> grasp.attach(exclude_tool_contact=True) drops the held body's
    conaffinity to 0, which is meant for an aglet on the hook but is also applied to a held SHOE: the laces
    (conaffinity 0 themselves) then pass straight through the shoe and fall out of the rings on every carry. Restore
    the shoe's conaffinity (release() restores the originals anyway)."""
    return      # (fixed in the cell: a held body keeps colliding with everything but the hand)


def _close_until(rig, body, timeout=2.0, probe=None):
    """Close the fingers until the gripper holds `body` (or the time is up). `probe` = a world point (the handle)
    whose tcp-frame offset is logged on failure."""
    rig.s.command("left", width=0.0)
    n = int(round(timeout / rig.s.tick_s))
    trace = []
    for i in range(n):
        rig.tick()
        h = rig.holding
        if h is not None:
            _fix_held_contacts(rig)
            return h[0] == body
        if probe is not None and i % 10 == 0:
            pt, Rt = rig.tcp("left")
            trace.append((round(rig.sc.arms["left"].width(), 4), np.round(1000 * (Rt.T @ (probe() - pt)), 1).tolist()))
    if trace:
        _log(rig, "close failed; width / handle offset (mm) trace: %s" % trace[:8])
    return False


def _take_from_hook(rig, body, timeout=1.6):
    """Handover rule of the cell: closing the fingers on the aglet that sits on the hook tip (its centre inside the
    pinch box, fingers <= 9.6 mm) attaches it to the gripper and opens the hook automatically. Returns (True, None)
    when the gripper holds `body`, else (False, offset of the aglet centre in the tcp frame, metres) with the hook
    still holding the aglet (the caller re-aligns and retries)."""
    rig.s.command("left", width=0.0)
    n = int(round(timeout / rig.s.tick_s))
    arm = rig.sc.arms["left"]
    for i in range(n):
        rig.tick()
        if rig.holding is not None:
            break
        if arm.width() <= 0.0070 and i > 30:
            break
    if rig.holding is not None:          # the cell's handover rule fired (event hook_released:handover)
        _wait(rig, 0.05)
        return rig.holding[0] == body, None
    pt, Rt = rig.tcp("left")
    centre = None
    for i in range(2):
        for w, bname, c, ax in rig.sc.aglet_info(i):
            if bname == body:
                centre = np.asarray(c, float)
    off = None if centre is None else Rt.T @ (centre - pt)
    _log(rig, "take from hook: no attach (width %.4f, aglet centre in the tcp frame %s mm, hook loaded %s)" % (
        arm.width(), None if off is None else np.round(1000 * off, 1).tolist(), rig.hook_loaded))
    return False, off


def _plan_yaw(rig, yaw_shoe, delta):
    """Closing-axis yaw variant (yaw_shoe or yaw_shoe + pi) for a grasp from the arm's current wrist state, and the
    signed rotation to apply afterwards (delta, or +-pi chosen by the wrist room when |delta| ~ pi)."""
    q7 = float(rig.sc.arms["left"].q()[6])
    yaw_now = _tool_yaw(rig)
    best = None
    for v in (yaw_shoe, yaw_shoe + math.pi):
        q7_h = q7 + Q7_DYAW * _wrap(v - yaw_now)
        for d in ([delta] if abs(delta) < math.pi - 0.35 else [math.pi, -math.pi]):
            q7_f = q7_h + Q7_DYAW * d
            margin = min(Q7_LIM - abs(q7_h), Q7_LIM - abs(q7_f))
            if best is None or margin > best[0]:
                best = (margin, _wrap(v), d, q7_h, q7_f)
    _log(rig, "yaw plan: closing yaw %.2f, rotation %+.2f (q7 %.2f -> hover %.2f -> final %.2f, margin %.2f)" % (best[1], best[2], q7, best[3], best[4], best[0]))
    return best[1], best[2], best[0]


def _wrist_room(q7, delta):
    """Signed rotation to apply (delta, or +-pi by the wrist room) and the margin of joint 7 for it (dq7/dyaw is
    about -0.85 .. -1.25 for a tool pointing down)."""
    cands = [delta] if abs(delta) < math.pi - 0.3 else [math.pi, -math.pi]
    best = None
    for d in cands:
        q7_f = q7 - 1.25 * d
        margin = min(2.75 - abs(q7), 2.75 - abs(q7_f))
        if best is None or margin > best[0]:
            best = (margin, d)
    return best[1], best[0]


def _grasp_dz(key, upright=True):
    """tcp height relative to the handle site: the heel counter stands 25 mm above the flaps (pads straddle it 4 mm
    down), the toe cap only 15 mm (the back finger hangs over the flap front strip and ring 1: stay at the cap top)."""
    if not upright:
        return 0.008
    return 0.0 if key == "toe_grip" else -0.004


def _grasp_handle(rig, s, key, dz, yaw=None, delta=0.0):
    """Grasp the shoe at handle `key` from above, fingers closing along the shoe's length (closing yaw `yaw` or the
    shoe's heading, or that + pi when the wrist then has no room for the planned rotation `delta` about the vertical);
    the tcp ends `dz` above the site. Returns (attached, signed rotation to apply)."""
    body = rig.sc.shoes[s]["body"]
    F = _Frame(rig, s)
    yaw0 = math.atan2(F.ex[1], F.ex[0]) if yaw is None else yaw
    delta_s = delta
    for attempt in range(3):
        P, pinch = _handle_sites(rig, s)[key]
        quat = down_quat(yaw0 + (0.0 if attempt % 2 == 0 else math.pi))
        rig.gripper(min(0.08, pinch + 0.03), wait=False)
        if not goto(rig, "left", P + Z * 0.10, quat, tol=0.004, timeout=5.0):
            _log(rig, "shoe %d: cannot hover above %s (closing yaw %.2f)" % (s, key, yaw0 + (0.0 if attempt % 2 == 0 else math.pi)))
            continue
        q7 = float(rig.sc.arms["left"].q()[6])
        delta_s, margin = _wrist_room(q7, delta)
        if margin < 0.5 and attempt < 2:
            _log(rig, "shoe %d: wrist q7 %.2f leaves no room for a %.2f rotation (margin %.2f), other closing yaw" % (s, q7, delta, margin))
            continue
        rig.gripper(min(0.08, pinch + 0.03))
        goto(rig, "left", P + Z * (dz + 0.04), quat, tol=0.003, rot_tol=0.03, timeout=3.0, joint_ok=False)
        if not _slow_goto(rig, "left", P + Z * dz, quat, v=0.08, tol=0.002, rot_tol=0.02):
            rig.lift("left", 0.08, timeout=2.0)
            continue
        if _close_until(rig, body, timeout=2.5):
            _log(rig, "shoe %d grasped at %s (t=%.1f, q7 %.2f, planned rotation %+.2f)" % (s, key, rig.time, rig.sc.arms["left"].q()[6], delta_s))
            return True, delta_s
        _log(rig, "shoe %d: grasp at %s failed (holding %s), retrying" % (s, key, rig.holding))
        rig.release(0.3)
        goto(rig, "left", P + Z * 0.10, quat, tol=0.004, timeout=3.0)
    return False, delta_s


def _tcp_for_shoe(rig, s, p_des, R_des):
    """The tcp pose that puts the held shoe at (p_des, R_des), from the current hand/shoe relative pose."""
    F = _Frame(rig, s)
    pt, Rt = rig.tcp("left")
    R_rel = Rt.T @ F.R
    p_rel = Rt.T @ (F.p - pt)
    Rt_des = R_des @ R_rel.T
    pt_des = p_des - Rt_des @ p_rel
    return pt_des, mat_to_quat(Rt_des)


def _move_shoe(rig, s, p_des, R_des, tol=0.004, timeout=5.0, v=0.2):
    """Move the held shoe to (p_des, R_des), slowly, in Cartesian mode (never a joint-space sweep while holding)."""
    pt, qt = _tcp_for_shoe(rig, s, p_des, R_des)
    return _slow_goto(rig, "left", pt, qt, v=v, tol=tol, rot_tol=0.03)


def _lift_shoe(rig, s, dz):
    F = _Frame(rig, s)
    return _move_shoe(rig, s, F.p + Z * dz, F.R, tol=0.005)


def _shoe_low_z(rig, s, R):
    """Lowest point of the shoe's bounding box (relative to its origin) for orientation R."""
    aabb = rig.sc.shoes[s]["aabb"]
    lo, hi = np.asarray(aabb["min"], float), np.asarray(aabb["max"], float)
    zs = []
    for cx in (lo[0], hi[0]):
        for cy in (lo[1], hi[1]):
            for cz in (lo[2], hi[2]):
                zs.append((R @ np.array([cx, cy, cz]))[2])
    return float(min(zs))


def _rotate_held(rig, s, delta, step=math.radians(45)):
    """Rotate the held shoe about the vertical through its origin by `delta` (signed), in steps."""
    if abs(delta) < 0.02:
        return True
    n = int(math.ceil(abs(delta) / step))
    F = _Frame(rig, s)
    p = F.p.copy()
    ok = True
    for k in range(1, n + 1):
        Rk = _rotz(delta * k / n) @ F.R
        ok = _move_shoe(rig, s, p, Rk, tol=0.006, timeout=4.0) and ok
    F2 = _Frame(rig, s)
    _log(rig, "rotated shoe %d by %+.2f: heading now %.2f (q7 %.2f) ok %s" % (s, delta, F2.yaw, rig.sc.arms["left"].q()[6], ok))
    return ok


def _right_in_air(rig, s):
    """Roll the held shoe (lying on its side) upright about its own length in 30 deg steps; returns the sole-normal z."""
    F = _Frame(rig, s)
    axis = np.array([F.ex[0], F.ex[1], 0.0])
    axis /= max(1e-9, np.linalg.norm(axis))
    yaw = math.atan2(axis[1], axis[0])
    R_des = _rotz(yaw)
    R_delta = R_des @ F.R.T
    ang = _rot_err(R_des, F.R)
    rv = np.array([R_delta[2, 1] - R_delta[1, 2], R_delta[0, 2] - R_delta[2, 0], R_delta[1, 0] - R_delta[0, 1]])
    sgn = 1.0 if np.dot(rv, axis) > 0 else -1.0
    p = F.p.copy()
    p[2] = 0.14
    n = max(1, int(math.ceil(ang / math.radians(30))))
    for k in range(1, n + 1):
        Rk = quat_to_mat(rot_quat(axis, sgn * ang * k / n)) @ F.R
        if not _move_shoe(rig, s, p, Rk, tol=0.008, timeout=3.0):
            break
    F2 = _Frame(rig, s)
    _log(rig, "shoe %d rolled in the air: sole normal z %.2f (q7 %.2f)" % (s, F2.R[2, 2], rig.sc.arms["left"].q()[6]))
    return float(F2.R[2, 2])


def _lower_and_release(rig, s, clearance=0.004, max_drop=0.035):
    """Lower the held shoe straight down (orientation kept) until its lowest point is `clearance` above the bench
    and release; never release from higher than `max_drop`. Returns True when released."""
    F = _Frame(rig, s)
    z_low = _shoe_low_z(rig, s, F.R)
    p = F.p.copy()
    p[2] = clearance - z_low
    _move_shoe(rig, s, p, F.R, tol=0.003, timeout=4.0)
    F2 = _Frame(rig, s)
    h = F2.p[2] + _shoe_low_z(rig, s, F2.R)
    if h > max_drop:
        _log(rig, "shoe %d: cannot lower it (lowest point %.0f mm up), keeping hold" % (s, 1000 * h))
        return False
    rig.release(0.5)
    rig.lift("left", 0.10, timeout=2.0)
    rig.wait(0.6)
    return True


def _set_down(rig, s, p_xy, R_des=None, clearance=0.007):
    """Lower the held shoe over p_xy (keeping its orientation unless R_des) until its lowest point is `clearance`
    above the bench, release, retreat."""
    F = _Frame(rig, s)
    R = F.R if R_des is None else R_des
    z0 = clearance - _shoe_low_z(rig, s, R)
    p = np.array([p_xy[0], p_xy[1], z0])
    _move_shoe(rig, s, p + Z * 0.04, R, tol=0.005, timeout=5.0)
    _move_shoe(rig, s, p, R, tol=0.003, timeout=3.0)
    return _lower_and_release(rig, s, clearance)


def _place_shoe(rig, s, p_seat, R_seat, hover=0.04, final=0.006, corrections=3):
    """Lower the held shoe onto (p_seat, R_seat) with a closed-loop correction of the measured pose error at hover
    height, release and retreat."""
    target = p_seat + Z * hover
    corr = np.zeros(3)
    dyaw = 0.0
    _move_shoe(rig, s, p_seat + Z * 0.10, R_seat, tol=0.006, timeout=5.0)
    for k in range(corrections):
        _move_shoe(rig, s, target + corr, _rotz(dyaw) @ R_seat, tol=0.003, timeout=4.0)
        rig.wait(0.2)
        F = _Frame(rig, s)
        e = F.p - target
        ey = _wrap(_yaw_of(R_seat) - F.yaw)
        _log(rig, "shoe %d hover: xy err %.1f mm, yaw err %.1f deg" % (s, 1000 * np.linalg.norm(e[:2]), math.degrees(ey)))
        if np.linalg.norm(e[:2]) < 0.0015 and abs(ey) < math.radians(1.0):
            break
        corr[:2] -= e[:2]
        dyaw += ey
    _move_shoe(rig, s, p_seat + Z * final + corr, _rotz(dyaw) @ R_seat, tol=0.002, timeout=3.0)
    rig.release(0.5)
    rig.lift("left", 0.10, timeout=2.0)


def _settle_seated(rig, s, wait_s=2.0):
    n = int(round(wait_s / rig.s.tick_s))
    for _ in range(n):
        rig.tick()
        if rig.s.clamp_engaged == s:
            return True
    return rig.sc.seated(s)


def _patch_cell_bugs(rig):
    """WORKAROUNDS for cell bugs met while developing the oracle (reported in the oracle's notes):
    1. Arm.hold_width is missing (see _ensure_hold_width);
    2. server._clamp_logic builds the clamp event with "clamped:" + cand where cand is the shoe INDEX (int): TypeError
       inside physics_tick on every clamp engagement, after the clamp is already set. Wrap it so the intended event
       is returned instead of the exception."""
    _guard(rig)      # (both fixed in the cell)


def _carry_to_cradle_and_seat(rig, s, seat_p, seat_R):
    for attempt in range(3):
        _place_shoe(rig, s, seat_p, seat_R)
        if _settle_seated(rig, s):
            _log(rig, "shoe %d seated and clamped (t=%.1f)" % (s, rig.time))
            return True
        F = _Frame(rig, s)
        _log(rig, "shoe %d not seated: pos err %s mm yaw err %.1f deg, tilt z %.3f; retry %d" % (
            s, np.round(1000 * (F.p - seat_p), 1), math.degrees(_wrap(F.yaw - _yaw_of(seat_R))), F.R[2, 2], attempt + 1))
        ok, _ = _grasp_handle(rig, s, "toe_grip", dz=0.0, yaw=_yaw_of(seat_R))
        if not ok:
            break
        _lift_shoe(rig, s, 0.06)
    raise RuntimeError("shoe %d: could not seat it in the cradle" % s)


def _check_world(rig, s):
    p, _ = rig.sc.shoe_pose(s)
    if p[2] < -0.05:
        raise RuntimeError("shoe %d is below the bench (z=%.2f m): the physics blew up and MuJoCo reset the world" % (s, p[2]))


def seat_shoe(rig, s, _retry=0):
    """Shoe s of the current pair: from the infeed tray into the cradle, clamped."""
    sc = rig.sc
    _patch_cell_bugs(rig)
    _log(rig, "seat shoe %d (t=%.1f, try %d)" % (s, rig.time, _retry))
    if _retry > 4:
        raise RuntimeError("shoe %d: giving up seating" % s)
    _park_hook(rig)
    if rig.holding is not None:
        rig.release()
    _gripper_home(rig)
    cr = sc.cradle_truth()
    seat_p = np.asarray(cr["seat_pos"], float)
    seat_R = quat_to_mat(np.asarray(cr["seat_quat"], float))
    yaw_c = _yaw_of(seat_R)
    F = _Frame(rig, s)
    if F.p[2] < -0.05:
        raise RuntimeError("shoe %d is not on the bench (z=%.2f m): the cell's delivery failed (physics blow-up at reset)" % (s, F.p[2]))
    if not hasattr(rig, "_lace_arrival"):
        rig._lace_arrival = {}
    if _retry == 0:
        rig._lace_arrival[s] = np.asarray(sc.shoe_centre(s), float)[:2].copy()
        if not hasattr(rig, "_lace_arrival_yaw"):
            rig._lace_arrival_yaw = {}
        rig._lace_arrival_yaw[s] = math.atan2(F.ex[1], F.ex[0])
    on_side = F.R[2, 2] < 0.9
    yaw_shoe = math.atan2(F.ex[1], F.ex[0])
    at_stage = np.linalg.norm(F.p[:2] - np.array(STAGE)) < 0.06
    if on_side:
        key = _best_handle(rig, s)
        ok, _ = _grasp_handle(rig, s, key, dz=0.008)
        if not ok:
            key2 = _best_handle(rig, s, exclude=key)
            _log(rig, "shoe %d: on its side, %s blocked (neighbour?), trying %s" % (s, key, key2))
            ok, _ = _grasp_handle(rig, s, key2, dz=0.008)
        if not ok:
            raise RuntimeError("shoe %d: cannot grasp it lying on its side" % s)
        R0 = F.R.copy()
        _lift_shoe(rig, s, 0.08)
        nz = _right_in_air(rig, s)
        other = np.asarray(sc.shoe_centre(1 - s), float)
        F1 = _Frame(rig, s)
        away = F1.p[:2] - other[:2]
        if other[2] > -0.1 and np.linalg.norm(away) < 0.32:
            # the neighbour shoe is close: set the righted shoe down a little further away from it (the hand is wide
            # along the shoe's length and would press the neighbour on the way down)
            away = away / max(1e-6, np.linalg.norm(away))
            tgt = F1.p + np.array([away[0], away[1], 0.0]) * 0.07
            _log(rig, "shoe %d: neighbour %.2f m away, shifting the righted shoe by 7 cm before setting it down" % (s, np.linalg.norm(F1.p[:2] - other[:2])))
            _move_shoe(rig, s, tgt, F1.R, tol=0.008, timeout=4.0)
        if nz < math.cos(math.radians(35)):
            # not rolled far enough to fall onto its sole: put it back on its side and try again from scratch
            _log(rig, "shoe %d: roll insufficient (sole normal z %.2f), putting it back" % (s, nz))
            F1 = _Frame(rig, s)
            _move_shoe(rig, s, F1.p, R0, tol=0.01, timeout=4.0)
        if not _lower_and_release(rig, s, clearance=0.004 if nz > 0.95 else 0.010, max_drop=0.05):
            F1 = _Frame(rig, s)
            _move_shoe(rig, s, F1.p, R0, tol=0.01, timeout=4.0)
            _lower_and_release(rig, s, clearance=0.006, max_drop=0.08)
        rig.wait(0.6)
        return seat_shoe(rig, s, _retry + 1)
    key = "toe_grip" if at_stage else _best_handle(rig, s)
    delta = _wrap(yaw_c - yaw_shoe)
    ok, delta_s = _grasp_handle(rig, s, key, dz=_grasp_dz(key), delta=delta)
    if not ok:
        key2 = _best_handle(rig, s, exclude=key)
        ok, delta_s = _grasp_handle(rig, s, key2, dz=_grasp_dz(key2), delta=delta)
        if not ok:
            raise RuntimeError("shoe %d: cannot grasp it" % s)
        key = key2
    _lift_shoe(rig, s, 0.19)
    F = _Frame(rig, s)
    p_carry = F.p.copy()
    p_carry[2] = 0.20                                  # high: the 0.2 m tails hang free of the tray rim
    _move_shoe(rig, s, p_carry, F.R, tol=0.006, timeout=3.0)
    if not at_stage:
        # round the gripper base via a waypoint in front of the tray
        _move_shoe(rig, s, np.array([0.18, 0.10, 0.20]), F.R, tol=0.01, timeout=5.0)
    _rotate_held(rig, s, delta_s)
    if key != "toe_grip":
        # the heel is out of comfortable reach when the shoe sits in the cradle: stage it and re-grasp at the toe
        F = _Frame(rig, s)
        if not _set_down(rig, s, STAGE, _rotz(yaw_c)):
            _lower_and_release(rig, s, clearance=0.007, max_drop=0.08)
        return seat_shoe(rig, s, _retry + 1)
    F = _Frame(rig, s)
    _move_shoe(rig, s, np.array([F.p[0], F.p[1], 0.20]), _rotz(yaw_c), tol=0.006, timeout=3.0)
    _move_shoe(rig, s, seat_p + np.array([0.26, 0.0, 0.20]), seat_R, tol=0.008, timeout=6.0)
    _move_shoe(rig, s, seat_p + Z * 0.20, seat_R, tol=0.008, timeout=6.0, v=0.08)       # the tails hang free, then trail toward +x
    _move_shoe(rig, s, seat_p + Z * 0.11, seat_R, tol=0.008, timeout=4.0, v=0.06)
    return _carry_to_cradle_and_seat(rig, s, seat_p, seat_R)


def unseat_shoe(rig, s):
    """Release the clamp and set the shoe down beside the cradle (sole down, toe along the cradle x)."""
    sc = rig.sc
    _patch_cell_bugs(rig)
    _check_world(rig, s)
    _log(rig, "unseat shoe %d (t=%.1f)" % (s, rig.time))
    if rig.holding is not None:
        rig.release()
    _park_hook(rig)
    _gripper_home(rig)
    cr = sc.cradle_truth()
    seat_p = np.asarray(cr["seat_pos"], float)
    seat_R = quat_to_mat(np.asarray(cr["seat_quat"], float))
    yaw_c = _yaw_of(seat_R)
    arrival = getattr(rig, "_lace_arrival", {}).get(s)
    yaw_t = 0.0                                   # preferred: toe along +x on the tray; any heading is acceptable
    delta = _wrap(yaw_t - yaw_c)
    rig.release_clamp()          # the cradle re-clamps a resting shoe 2 s after a release: grasp within that time
    ok, delta_s = _grasp_handle(rig, s, "heel_grip", dz=_grasp_dz("heel_grip"), delta=delta)
    if not ok:
        rig.release_clamp()
        ok, delta_s = _grasp_handle(rig, s, "heel_grip", dz=_grasp_dz("heel_grip"), delta=0.0)
        if ok:
            delta_s = 0.0
    if not ok:
        rig.release_clamp()
        ok, delta_s = _grasp_handle(rig, s, "toe_grip", dz=0.0, delta=0.0)
        if ok:
            delta_s = 0.0
    if not ok:
        raise RuntimeError("shoe %d: cannot grasp it in the cradle" % s)
    rig.release_clamp()
    rig.wait(0.1)
    _lift_shoe(rig, s, 0.19)
    cl = np.asarray(rig.sc.shoes[s]["centre_local"], float)
    if arrival is not None:
        # back where it arrived: the delivery spots are clear of each other and of the lid (a spot 0.10 m beside shoe 0
        # put shoe 1 on the lid's rim or on shoe 0 in a third of the episodes, tilted, and the packing then failed)
        target = np.asarray(arrival, float)
    else:
        tx, ty = sc.layout["infeed"]["centre"]
        target = np.array([tx - 0.115 + 0.23 * s, ty + 0.10])
    _move_shoe(rig, s, seat_p + np.array([0.0, -0.18, 0.20]), seat_R, tol=0.01, timeout=6.0, v=0.12)     # high, toe first, tails hang free
    _move_shoe(rig, s, np.array([-0.10, seat_p[1] - 0.15, 0.20]), seat_R, tol=0.01, timeout=6.0, v=0.12)
    _move_shoe(rig, s, np.array([0.18, 0.10, 0.20]), seat_R, tol=0.01, timeout=6.0, v=0.12)
    if abs(delta_s) > 0.05:
        _rotate_held(rig, s, delta_s)
    F = _Frame(rig, s)
    R_des = _rotz(F.yaw)                           # whatever heading the shoe has now (the box skills re-grasp)
    p_des = np.array([target[0], target[1], 0.0]) - np.array([(R_des @ cl)[0], (R_des @ cl)[1], 0.0])
    # across at carry height, then straight down: shoe 1's spot is at the far end of the tray, and a slanted move down
    # to it swept the forearm (link5) onto the tray (55-67 N in an engine episode)
    _move_shoe(rig, s, np.array([p_des[0], p_des[1], 0.20]), F.R, tol=0.01, timeout=6.0, v=0.12)
    _move_shoe(rig, s, p_des + Z * 0.10, F.R, tol=0.01, timeout=6.0, v=0.08)
    _log(rig, "shoe %d to the tray spot %s (t=%.1f)" % (s, np.round(p_des[:2], 3), rig.time))
    _place_shoe(rig, s, p_des, R_des, hover=0.03, final=0.007, corrections=2)
    rig.wait(0.3)
    F = _Frame(rig, s)
    ins = [bool(rig.sc.inside_cavity(s, c)) for c in rig.sc.aglet_centres(s)]
    _log(rig, "shoe %d set down at %s upright %s, aglets inside %s (t=%.1f)" % (s, np.round(F.p, 3), F.R[2, 2] > 0.95, ins, rig.time))
    _gripper_home(rig)
    return True


# ---------------------------------------------------------------------------------------------------- lace handling
def _wait_still(rig, s, which, v_max=0.02, t_max=4.0):
    """Wait until aglet `which` has (nearly) stopped sliding; returns its speed."""
    n = int(round(t_max / 0.2))
    _, c0, _ = _aglet(rig, s, which)
    v = 1.0
    for _ in range(n):
        rig.wait(0.2)
        _, c1, _ = _aglet(rig, s, which)
        v = float(np.linalg.norm(c1 - c0)) / 0.2
        c0 = c1
        if v < v_max:
            break
    return v


def _nudge_from_wall(rig, s, which, geo, F):
    """An aglet lying against the shoe (on the cradle plate) cannot be grasped from above: push it outward with the
    closed fingers so that it slides clear (frictionless), then it can be grasped."""
    _, c, _ = _aglet(rig, s, which)
    cl = F.l(c)
    if not (abs(cl[1]) < geo.W / 2 + 0.04 and -geo.L / 2 - 0.03 < cl[0] < geo.L / 2 + 0.03 and cl[2] < 0.03):
        return False
    sg = 1.0 if cl[1] >= 0 else -1.0
    o = sg * F.ey
    yaw = F.yaw                      # fingers closed, palm's long axis along the shoe: it stays clear of the wall
    inb = float(min(0.03, max(0.005, abs(cl[1]) - geo.W / 2 - 0.02)))   # never start over the wall / flap
    start = c - o * inb
    start[2] = 0.015
    end = c + o * 0.07
    end[2] = 0.015
    rig.gripper(0.0, wait=False)
    _log(rig, "aglet %s lies against the shoe (local %s): nudging it outward" % (which, np.round(cl, 3)))
    if not goto(rig, "left", np.array([start[0], start[1], 0.20]), down_quat(yaw), tol=0.006, timeout=4.0):
        return False
    goto(rig, "left", start + Z * 0.05, down_quat(yaw), tol=0.003, timeout=3.0, joint_ok=False)
    goto(rig, "left", start, down_quat(yaw), tol=0.002, timeout=2.0, joint_ok=False)
    _slow_goto(rig, "left", end, down_quat(yaw), v=0.12, tol=0.004)
    rig.lift("left", 0.06, timeout=1.5)
    rig.gripper(0.03, wait=False)
    _wait_still(rig, s, which, t_max=2.0)
    return True


def _aglet_vel(rig, s, which, dt=0.05):
    _, c0, _ = _aglet(rig, s, which)
    rig.wait(dt)
    _, c1, _ = _aglet(rig, s, which)
    return c1, (c1 - c0) / dt


def _hook_catch_free(rig, s, which, geo, F, t_max=8.0):
    """The hook catches aglet `which` lying (and sliding: the lace is frictionless) on the bench: the tip is servoed
    onto the aglet's position, hook(True) every few ticks. Returns True when loaded."""
    if rig.hook_loaded:
        rig.hook(False)
    c, v = _aglet_vel(rig, s, which)
    d_r = float(np.linalg.norm(c[:2] - RIGHT_BASE))
    if not 0.21 < d_r < 0.53:
        _log(rig, "hook catch: aglet %s at %s is out of the hook arm's reach (%.2f m from its base)" % (which, np.round(c, 3), d_r))
        return False
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    arm = rig.sc.arms["right"]
    quat = hook_quat(HOOK_FOOT_FREE)                       # same foot yaw as the handover: no wrist turn afterwards
    shaft_off = np.array([-FOOT * math.cos(HOOK_FOOT_FREE), -FOOT * math.sin(HOOK_FOOT_FREE), 0.0])   # shaft relative to the tip
    def tip_for(cc):
        t = np.array([cc[0], cc[1], 0.0])
        cl = F.l(np.array([cc[0] + shaft_off[0], cc[1] + shaft_off[1], 0.05]))
        if abs(cl[0]) < geo.L / 2 + 0.02 and abs(cl[1]) < geo.W / 2 + 0.012:      # the shaft would stand in the shoe
            t[:2] -= shaft_off[:2] * 0.7
        return t
    tip = tip_for(c)
    tip[2] = 0.18
    p_now, _ = rig.tcp("right")
    if np.linalg.norm(p_now[:2] - rig.tcp_for_tip(tip, quat)[:2]) > 0.05 or p_now[2] < 0.1:
        goto(rig, "right", rig.tcp_for_tip(tip, quat), quat, tol=0.008, rot_tol=0.05, timeout=4.0)
    tip[2] = 0.06
    goto(rig, "right", rig.tcp_for_tip(tip, quat), quat, tol=0.005, rot_tol=0.04, timeout=3.0, joint_ok="free")
    n = int(round(t_max / rig.s.tick_s))
    if arm.mode != "cartesian":
        arm.set_mode("cartesian")
    c_prev, t_prev = c.copy(), rig.time
    for i in range(n):
        if i % 3 == 0:
            _, c, _ = _aglet(rig, s, which)
            dt = max(1e-3, rig.time - t_prev)
            vel = (c - c_prev) / dt if i > 0 else np.zeros(3)
            c_prev, t_prev = c.copy(), rig.time
            lead = np.clip(vel * 0.12, -0.03, 0.03)
            tip = tip_for(c + lead)
            tip[2] = max(c[2] + 0.003 + TIP, 0.0045 + TIP)                        # foot 3 mm above the aglet's height
            d_r = float(np.linalg.norm(tip[:2] - RIGHT_BASE))
            if not 0.19 < d_r < 0.53:
                _log(rig, "hook catch: aglet %s slid out of reach (%.2f m from the base)" % (which, d_r))
                break
            arm.command_cartesian(rig.tcp_for_tip(tip, quat), quat)
        rig.tick()
        if rig.hook(True):
            held = rig.s.hook_loaded
            tipw, _ = rig.tip_pose()
            _log(rig, "hook caught aglet %s (lace %s) at %s after %.1f s" % (which, held, np.round(tipw, 3), i * rig.s.tick_s))
            rig.wait(0.2)
            p, R = rig.tcp("right")
            _slow_goto(rig, "right", np.array([p[0], p[1], 0.13]), quat, v=0.05, w=0.5, tol=0.01, rot_tol=0.1)
            if not rig.hook_loaded:
                _log(rig, "hook lost aglet %s while lifting" % which)
                return False
            return True
    p, _ = rig.tcp("right")
    goto(rig, "right", p + Z * 0.10, quat, tol=0.01, rot_tol=0.1, timeout=2.0)
    return False


def _handover_free(rig, s, which, geo, F):
    """The hook (loaded with aglet `which`) brings it to the free-space handover point; the gripper takes it there
    (approach tilted toward +y, the hook's shaft on the -y side). Returns True when the gripper holds it."""
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    sg = 1.0                                             # the gripper leans toward +y (its base side): the two wrists stay apart
    quat_h = hook_quat(HOOK_FOOT_FREE)                   # foot toward +y: the shaft 14 mm toward -y of the aglet
    pl, _ = rig.tcp("left")
    if pl[2] < 0.2 or np.linalg.norm(pl[:2] - HANDOVER[:2]) < 0.2:
        _gripper_home(rig)
    if not rig.hook_loaded:
        return False
    _slow_goto(rig, "right", rig.tcp_for_tip(HANDOVER + Z * 0.03, quat_h), quat_h, v=0.08, w=0.5, tol=0.006, rot_tol=0.05)
    goto(rig, "right", rig.tcp_for_tip(HANDOVER, quat_h), quat_h, tol=0.002, rot_tol=0.03, timeout=3.0, joint_ok=False)
    rig.wait(0.3)
    _, c, _ = _aglet(rig, s, which)
    tipw, _ = rig.tip_pose()
    _log(rig, "free handover: hook loaded %s, aglet - tip = %s mm" % (rig.hook_loaded, np.round(1000 * (c - tipw), 1)))
    if not rig.hook_loaded or np.linalg.norm(c - tipw - Z * 0.008) > 0.012:
        _log(rig, "free handover: the hook lost aglet %s" % which)
        rig.hook(False)
        return False
    quat = _lean_quat(0.0, sg, 1.35)                      # closing along world x, hand nearly horizontal from +y: the hook's shaft and wrist are on -y / above
    R = quat_to_mat(quat)
    tz = R[:, 2]
    tcp = c - AGLET_GRIP_DZ * tz
    rig.gripper(0.03, wait=False)
    pre = tcp - 0.10 * tz
    if not goto(rig, "left", np.array([pre[0], pre[1] + 0.05, 0.22]), quat, tol=0.01, rot_tol=0.08, timeout=5.0):
        _log(rig, "free handover: the gripper cannot reach the approach point")
        return False
    if not _slow_goto(rig, "left", pre, quat, v=0.12, w=0.8, tol=0.004, rot_tol=0.04):
        _log(rig, "free handover: the gripper cannot reach the pre-grasp point")
        rig.lift("left", 0.08, timeout=1.5)
        return False
    rig.gripper(0.03)
    _, c, _ = _aglet(rig, s, which)
    tcp = c - AGLET_GRIP_DZ * tz
    if not _slow_goto(rig, "left", tcp, quat, v=0.06, w=0.5, tol=0.0012, rot_tol=0.02):
        goto(rig, "left", tcp, quat, tol=0.0012, rot_tol=0.02, timeout=2.0, joint_ok=False)
    got = False
    for k in range(3):
        got, off = _take_from_hook(rig, body)
        if got:
            break
        if off is None or not rig.hook_loaded:
            break
        d = np.array([off[0], 0.0, off[2] - AGLET_GRIP_DZ])
        if np.linalg.norm(d) > 0.008:
            d = d / np.linalg.norm(d) * 0.008
        rig.gripper(0.03)
        pt, Rt = rig.tcp("left")
        goto(rig, "left", pt + Rt @ d, mat_to_quat(Rt), tol=0.001, rot_tol=0.03, timeout=2.0, joint_ok=False)
        _log(rig, "free handover: re-aligned by %s mm (try %d)" % (np.round(1000 * d, 1), k + 1))
    if not got:
        _log(rig, "free handover: the gripper did not take aglet %s (holding %s)" % (which, rig.holding))
        rig.release(0.3)
        rig.lift("left", 0.08, timeout=2.0)
        return False
    goto(rig, "left", tcp - 0.05 * tz, quat, tol=0.004, timeout=2.0, joint_ok=False)
    _log(rig, "free handover: aglet %s in the gripper (t=%.1f)" % (which, rig.time))
    _park_hook(rig, near=F.p[:2])
    return True


def _foot_phi_world(rig):
    _, R = rig.tcp("right")
    return math.atan2(R[1, 0], R[0, 0])


def _acquire_aglet(rig, s, which, geo, F):
    """Get aglet `which` into the gripper: already held, else fetched by the hook and handed over, else (far on the
    gripper's side) grasped from the bench."""
    h = rig.holding
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    if h is not None and h[0] == body:
        return True
    if h is not None:
        rig.release(0.3)
    for attempt in range(8):
        _, c, _ = _aglet(rig, s, which)
        d_r = float(np.linalg.norm(c[:2] - RIGHT_BASE))
        d_l = float(np.linalg.norm(c[:2] - LEFT_BASE))
        if 0.15 < d_l < 0.50 and attempt < 4:
            if d_l >= 0.18 and _grasp_aglet(rig, s, which, geo, F):
                return True
            if d_l < 0.28 and _grasp_aglet_lean(rig, s, which, geo, F):
                return True
        elif 0.21 < d_r < 0.53:
            if _hook_catch_free(rig, s, which, geo, F) and _handover_free(rig, s, which, geo, F):
                return True
            if rig.hook_loaded:
                rig.hook(False)
            _park_hook(rig, near=F.p[:2])
        else:
            _log(rig, "aglet %s at %s is in neither arm's zone (%.2f from the hook base, %.2f from the gripper base); waiting" % (which, np.round(c, 3), d_r, d_l))
            rig.wait(1.5)
    return False


HAND_HALF_T = 0.032      # the gripper hand's half thickness across its closing axis (it reaches down to 37 mm above the tcp)


def _beside_shoe_grasp(rig, s, c, geo, F):
    """A top-down grasp of an aglet lying near the seated shoe whose hand (0.2 m wide along the closing axis) stays off
    the shoe: closing axis parallel to the nearest side (or across an end), the tcp shifted outward by up to 10 mm (the
    pads attach an aglet within 12 mm across them). Returns (yaw, xy shift), None when the aglet lies clear of the
    shoe, or "close" when even the shift would leave the hand on the shoe."""
    cl = F.l(c)
    dx = abs(float(cl[0])) - geo.L / 2
    dy = abs(float(cl[1])) - geo.W / 2
    if dx > 0.07 or dy > 0.07:
        return None
    if dy >= dx:
        need = HAND_HALF_T + 0.006 - dy
        out, yaw = (1.0 if cl[1] >= 0 else -1.0) * F.ey, F.yaw
    else:
        need = HAND_HALF_T + 0.006 - dx
        out, yaw = (1.0 if cl[0] >= 0 else -1.0) * F.ex, F.yaw + math.pi / 2
    if need > 0.010:
        return "close"
    return yaw, out[:2] * max(0.0, need)


def _grasp_aglet(rig, s, which, geo, F):
    """Grasp aglet `which` wherever it lies (fingers across it, from above). Returns True when held."""
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    rig.gripper(0.03, wait=False)
    v = _wait_still(rig, s, which)
    if v >= 0.02:
        _log(rig, "aglet %s still sliding at %.2f m/s" % (which, v))
    _, c, _ = _aglet(rig, s, which)
    if _beside_shoe_grasp(rig, s, c, geo, F) in (None, "close"):
        _nudge_from_wall(rig, s, which, geo, F)
    for attempt in range(3):
        _, c, ax = _aglet(rig, s, which)
        h = np.array([ax[0], ax[1], 0.0])
        to_shoe = F.p[:2] - c[:2]
        near = _beside_shoe_grasp(rig, s, c, geo, F)
        shift = np.zeros(2)
        if near not in (None, "close"):
            # beside the shoe: the hand's long side parallel to the shoe (a palm aimed at the shoe's centre swung its
            # far end 4 cm into the upper), never turned across it on a retry
            yaw, shift = near
            yaw += (0.0, 0.15, -0.15)[attempt]
            if attempt == 0:
                _log(rig, "aglet %s beside the shoe (local %s): hand parallel to it, tcp shifted %s mm" % (
                    which, np.round(F.l(c), 3), np.round(1000 * shift, 1)))
        else:
            if np.linalg.norm(to_shoe) < 0.17:
                yaw = math.atan2(to_shoe[1], to_shoe[0]) + math.pi / 2     # palm tangential to the shoe
            elif np.linalg.norm(h) > 0.5:
                yaw = math.atan2(h[1], h[0]) + math.pi / 2                 # pads across the aglet
            else:
                yaw = F.yaw
            yaw += (math.pi / 2) * attempt
        quat = down_quat(yaw)
        if not 0.17 < np.linalg.norm(c[:2] - LEFT_BASE) < 0.50:
            _log(rig, "aglet %s at %s is outside the gripper's safe bench zone" % (which, np.round(c, 3)))
            return False
        if not goto(rig, "left", np.array([c[0] + shift[0], c[1] + shift[1], 0.20]), quat, tol=0.006, timeout=4.0, joint_ok=(not rig.hook_loaded)):
            continue
        rig.gripper(0.012, wait=False)
        _, c, ax = _aglet(rig, s, which)
        if not goto(rig, "left", c + Z * 0.04 + np.array([shift[0], shift[1], 0.0]), quat, tol=0.002, timeout=4.0, joint_ok=False):
            continue
        rig.gripper(0.012)
        c2 = c
        for k in range(3):
            c, vel = _aglet_vel(rig, s, which)         # it slides (frictionless): aim a little ahead
            z = max(c[2] + AGLET_GRIP_DZ, 0.0115)
            tgt = np.array([c[0] + 0.25 * vel[0] + shift[0], c[1] + 0.25 * vel[1] + shift[1], z])
            goto(rig, "left", np.array([tgt[0], tgt[1], z + 0.03]), quat, tol=0.0015, rot_tol=0.02, timeout=2.0, joint_ok=False)
            if not goto(rig, "left", tgt, quat, tol=0.0015, rot_tol=0.02, timeout=1.5, joint_ok=False):
                break
            _, c2, _ = _aglet(rig, s, which)
            if np.linalg.norm(c2[:2] + shift - tgt[:2]) < 0.004:
                break
        pt, Rt = rig.tcp("left")
        dd = Rt.T @ (c2 - pt)
        if _close_until(rig, body, timeout=1.5, probe=lambda: _aglet(rig, s, which)[1]):
            _log(rig, "aglet %s grasped at %s (tcp-frame offset %s mm, t=%.1f)" % (which, np.round(c2, 3), np.round(1000 * dd, 1), rig.time))
            return True
        _log(rig, "aglet %s: grasp failed (holding %s, offset %s mm, width %.4f), retry %d" % (which, rig.holding, np.round(1000 * dd, 1), rig.sc.arms["left"].width(), attempt + 1))
        rig.release(0.3)
        rig.gripper(0.03, wait=False)
        rig.lift("left", 0.05, timeout=1.5)
    return False


def _grasp_aglet_lean(rig, s, which, geo, F):
    """Grasp an aglet lying close to the gripper's base (< ~0.26 m: the tool cannot point straight down there): the
    hand leans toward the base, the fingers reach outward-down; closing axis across the base direction."""
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    _wait_still(rig, s, which, t_max=2.0)
    for lean in (0.45, 0.7):
        _, c, ax = _aglet(rig, s, which)
        v = LEFT_BASE - c[:2]
        v = v / max(1e-6, np.linalg.norm(v))
        yaw = math.atan2(v[1], v[0]) - math.pi / 2
        quat = _lean_quat(yaw, 1.0, lean)
        R = quat_to_mat(quat)
        tz = R[:, 2]
        tcp = c - 0.006 * tz
        tcp[2] = max(tcp[2], 0.011)
        pre = tcp - 0.06 * tz
        if _ik(rig, "left", pre, quat, rot_tol=0.05) is None or _ik(rig, "left", tcp, quat, rot_tol=0.05) is None:
            _log(rig, "lean grasp of aglet %s (lean %.2f): no IK solution" % (which, lean))
            continue
        rig.gripper(0.03, wait=False)
        if not goto(rig, "left", pre + Z * 0.04, quat, tol=0.006, rot_tol=0.05, timeout=4.0, joint_ok=True):
            continue
        rig.gripper(0.012)
        _, c, ax = _aglet(rig, s, which)
        tcp = c - 0.006 * tz
        tcp[2] = max(tcp[2], 0.011)
        goto(rig, "left", tcp - 0.03 * tz, quat, tol=0.002, rot_tol=0.03, timeout=3.0, joint_ok=False)
        if not _slow_goto(rig, "left", tcp, quat, v=0.08, tol=0.0015, rot_tol=0.03):
            rig.lift("left", 0.05, timeout=1.5)
            continue
        pt, Rt = rig.tcp("left")
        _, c2, _ = _aglet(rig, s, which)
        dd = Rt.T @ (c2 - pt)
        if _close_until(rig, body, timeout=1.5):
            _log(rig, "aglet %s grasped leaning at %s (tcp-frame offset %s mm, lean %.2f, t=%.1f)" % (which, np.round(c2, 3), np.round(1000 * dd, 1), lean, rig.time))
            return True
        _log(rig, "lean grasp of aglet %s failed (offset %s mm)" % (which, np.round(1000 * dd, 1)))
        rig.release(0.3)
        rig.gripper(0.03, wait=False)
        rig.lift("left", 0.05, timeout=1.5)
    return False


def _lift_checked(rig, s, which, z_to, step=0.02):
    """Raise the held aglet to height z_to in steps, watching the weld stretch (the aglet lagging behind its canonical
    place 14 mm beyond the tcp): a lace pinned under the seated shoe or the cradle plate would otherwise be stretched
    until the physics blows up. Returns False (aglet released, hand lifted) when the lace is pinned."""
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    p, R = rig.tcp("left")
    _, c, _ = _aglet(rig, s, which)
    lag0 = float(np.linalg.norm(c - (p + R[:, 2] * 0.014)))     # the soft weld never sits exactly canonical
    while True:
        p, R = rig.tcp("left")
        if p[2] >= z_to - 1e-3:
            return True
        z = min(p[2] + step, z_to)
        _slow_goto(rig, "left", np.array([p[0], p[1], z]), mat_to_quat(R), v=0.1, tol=0.004, rot_tol=0.05)
        _wait(rig, 0.1)
        h = rig.holding
        if h is None or h[0] != body:
            return False
        p, R = rig.tcp("left")
        _, c, _ = _aglet(rig, s, which)
        lag = float(np.linalg.norm(c - (p + R[:, 2] * 0.014)))
        if lag > lag0 + 0.015:
            _log(rig, "aglet %s lags %.1f mm behind the hand at z %.3f: the lace is pinned, releasing it" % (which, 1000 * lag, p[2]))
            rig.release(0.4)
            rig.lift("left", 0.06, timeout=1.5)
            return False


def _in_shoe(rig, s, c, geo, F):
    """The aglet is inside the shoe: within the cavity footprint, below the flap tops (on the floor or hanging)."""
    cl = F.l(c)
    return (geo.x_hc - 0.005 < cl[0] < geo.x_tc + 0.005 and abs(cl[1]) < geo.Wc / 2 + 0.004 and geo.z_floor - 0.004 < cl[2] < geo.z_f + 0.001)


def _drop_in(rig, s, which, row, side, geo, F, ref_row, ref_side):
    """Holding an aglet whose lace last left the shoe through ring (ref_row, ref_side): lift it above that ring (the
    tail's slack hangs as a short bight below the hand), carry it over the shoe to just above ring (row, side) with
    the hand offset outward (the bight hangs down the outside of the shoe, not into the throat), align the aglet on
    the hole with bounded corrections, lower it into the hole and release. Returns True when the aglet is inside the
    shoe afterwards."""
    body = rig.sc.laces[s]["bodies"][0 if which == "a" else -1]
    e = F.w(geo.ring(row, side))
    e_ref = F.w(geo.ring(ref_row, ref_side)) if ref_row is not None else F.w((geo.x_rows[1], 0.0, geo.z_f))   # the slot
    quat = down_quat(F.yaw + math.pi / 2)        # hand long axis along world x: clear of the heel counter / toe cap (row 1 is tacked)
    sg = _sg(side)
    out = sg * F.ey                                        # outward on the target side
    for attempt in range(2):
        passes, snug, la, lb, _, _ = _tails(rig, s)
        tail = la if which == "a" else lb
        h = float(np.clip(tail - 0.08, 0.07, 0.20))        # the bight below the hand stays short
        p_now, _ = rig.tcp("left")
        _, c0, _ = _aglet(rig, s, which)
        _log(rig, "IN drop row %d: aglet %.2f m from the reference point, tail %.3f m; passes %s" % (
            row, float(np.linalg.norm(c0[:2] - e_ref[:2])), tail, _passes_str(rig, s)))
        z_safe = float(F.p[2] + geo.z_hc_top + 0.074)       # aglet tip (24 mm below the tcp) 5 cm above the heel counter top
        z_h = max(z_safe, e_ref[2] + h)
        _slow_goto(rig, "left", np.array([p_now[0], p_now[1], z_safe]), quat, v=0.08, tol=0.005, rot_tol=0.05)   # straight up (row 1 is tacked)
        _wait(rig, 0.2)
        if not _slow_goto(rig, "left", np.array([e_ref[0], e_ref[1], z_h]), quat, v=0.08, w=0.6, tol=0.005, rot_tol=0.05):
            p1, _ = rig.tcp("left")
            if np.linalg.norm(p1 - np.array([e_ref[0], e_ref[1], z_h])) > 0.02:
                _log(rig, "IN drop row %d: cannot reach above the reference point %s" % (row, np.round(e_ref, 3)))
                return False
        _wait(rig, 0.3)
        _log(rig, "IN drop row %d: passes above the reference point: %s" % (row, _passes_str(rig, s)))
        z_hover = max(z_safe, e[2] + h)
        if not goto(rig, "left", e + Z * (z_hover - e[2]), quat, tol=0.004, timeout=4.0, joint_ok=False):
            p_now, _ = rig.tcp("left")
            if np.linalg.norm(p_now - (e + Z * (z_hover - e[2]))) > 0.02:
                _log(rig, "IN drop row %d: cannot reach above ring %d%s" % (row, row, side))
                return False
        # centre the aglet over the ring at hover height before any descent
        corr = np.zeros(3)
        centred = False
        for k in range(4):
            goto(rig, "left", e + Z * (z_hover - e[2]) + corr, quat, tol=0.0015, rot_tol=0.02, timeout=3.0, joint_ok=False)
            _wait(rig, 0.25)
            _, c, ax = _aglet(rig, s, which)
            err = c[:2] - e[:2]
            if np.linalg.norm(err) < 0.002:
                centred = True
                break
            if np.linalg.norm(err) > 0.010:
                err = err / np.linalg.norm(err) * 0.010
            corr[:2] -= err
        _log(rig, "IN drop row %d: centred over the ring at hover %s (corr %s mm); passes %s" % (row, centred, np.round(1000 * corr[:2], 1), _passes_str(rig, s)))
        z_align = 0.030                                    # aglet tip (24 mm beyond the tcp) 6 mm above the ring plane
        corr = np.zeros(3)
        if not _slow_goto(rig, "left", e + Z * z_align, quat, v=0.06, tol=0.002, rot_tol=0.03):
            if _arm_contact_force(rig, "left", 12.0) is not None:
                _log(rig, "IN drop row %d: the vertical descent over the ring stalled on a contact, lifting and retrying" % row)
                rig.lift("left", 0.08, timeout=1.5)
                if attempt == 0:
                    continue
                return False
        lost = False
        aligned = False
        for k in range(5):
            goto(rig, "left", e + Z * z_align + corr, quat, tol=0.0015, rot_tol=0.02, timeout=3.0, joint_ok=False)
            _wait(rig, 0.2)
            if rig.holding is None or rig.holding[0] != body:
                lost = True
                break
            _, c, ax = _aglet(rig, s, which)
            err = c[:2] - e[:2]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(ax[2])))))
            _log(rig, "IN drop row %d align %d: aglet offset %s mm, tilt %.0f deg, corr %s mm" % (row, k, np.round(1000 * err, 1), tilt, np.round(1000 * corr[:2], 1)))
            if np.linalg.norm(err) < 0.0012 and tilt < 55.0:
                aligned = True
                break
            if tilt >= 55.0 and k < 4:
                rig.lift("left", 0.03, timeout=1.5)            # the lace snagged: slacken it and let the aglet hang straight
                _wait(rig, 0.4)
            if np.linalg.norm(err) > 0.010:                    # bounded correction: never chase a runaway reading
                err = err / np.linalg.norm(err) * 0.010
            corr[:2] -= err
        if lost:
            _log(rig, "IN drop row %d: the gripper lost aglet %s on the way, re-acquiring" % (row, which))
            rig.release(0.3)
            rig.lift("left", 0.06, timeout=1.5)
            if attempt == 0 and _acquire_aglet(rig, s, which, geo, F):
                continue
            return False
        if not aligned:
            _log(rig, "IN drop row %d: not aligned after the corrections, lifting and retrying" % row)
            rig.lift("left", 0.06, timeout=1.5)
            _wait(rig, 0.5)
            if attempt == 0:
                continue
            return False
        goto(rig, "left", e + Z * 0.012 + corr, quat, tol=0.0015, rot_tol=0.02, timeout=3.0, joint_ok=False)
        _wait(rig, 0.2)
        _, c, ax = _aglet(rig, s, which)
        cl = F.l(c)
        in_hole = cl[2] < geo.z_f + 0.004
        _log(rig, "IN drop row %d: aglet centre offset from ring %s mm, %.1f mm above the flap, axis %s, in hole %s" % (
            row, np.round(1000 * (c[:2] - e[:2]), 1), 1000 * (cl[2] - geo.z_f), np.round(ax, 2), in_hole))
        if not in_hole and aligned:
            rig.lift("left", 0.02, timeout=1.5)
            _wait(rig, 0.2)
            goto(rig, "left", e + Z * 0.012 + corr, quat, tol=0.0015, rot_tol=0.02, timeout=3.0, joint_ok=False)
            _wait(rig, 0.2)
            _, c, ax = _aglet(rig, s, which)
            cl = F.l(c)
            in_hole = cl[2] < geo.z_f + 0.004
        if not in_hole:
            _log(rig, "IN drop row %d: aglet not in the hole (%.1f mm above the flap), lifting and retrying" % (row, 1000 * (cl[2] - geo.z_f)))
            rig.lift("left", 0.06, timeout=1.5)
            _wait(rig, 0.5)
            if attempt == 0:
                continue
            return False
        rig.s.command("left", width=0.024)             # just lets go (attach released when the width exceeds the hold width)
        _wait(rig, 0.6)
        rig.lift("left", 0.05, timeout=2.0)
        _wait(rig, 0.4)
        _, c, _ = _aglet(rig, s, which)
        if _in_shoe(rig, s, c, geo, F) and F.l(c)[2] > geo.z_f - 0.012:
            _log(rig, "IN drop row %d: aglet hangs in the hole (%.1f mm below the flap top); the hook catches it there" % (row, 1000 * (geo.z_f - F.l(c)[2])))
        rig.lift("left", 0.04, timeout=1.5)
        ok = _in_shoe(rig, s, c, geo, F)
        _log(rig, "IN pass row %d side %s: aglet before release at local %s, after at %s, inside %s (tail %.3f, t=%.1f)" % (
            row, side, np.round(cl, 4), np.round(F.l(c), 3), ok, tail, rig.time))
        if ok:
            return True
        if attempt == 0 and not _acquire_aglet(rig, s, which, geo, F):
            return False
    return False


def _drop_throat(rig, s, which, geo, F):
    """Holding an aglet: drop it into the cavity through the throat slot near row 1."""
    pl = np.array([geo.x_rows[0] - 0.012, 0.0, geo.z_f + 0.014])
    quat = down_quat(F.yaw)
    goto(rig, "left", F.w(pl) + Z * 0.07, quat, tol=0.003, timeout=4.0)
    goto(rig, "left", F.w(pl), quat, tol=0.0012, rot_tol=0.02, timeout=3.0, joint_ok=False)
    rig.release(0.5)
    rig.lift("left", 0.08, timeout=2.0)


def _hook_go(rig, F, xs, ys, zs, phi, tol=0.0015, timeout=3.0, joint_ok=False):
    """Hook shaft end (tcp) to the shoe-local point (xs, ys, zs) with the foot at angle `phi` from the shoe's x."""
    return goto(rig, "right", F.w((xs, ys, zs)), hook_quat(F.yaw + phi), tol=tol, rot_tol=0.04, timeout=timeout, joint_ok=joint_ok)


def _hook_entry_posture(rig, F, xe, geo):
    """Joint configuration of the hook arm for the entry pose above the slot with the wrist (link6) furthest toward
    the toe (the shoe's +x, i.e. away from the gripper's side), or None."""
    arm = rig.sc.arms["right"]
    pos = F.w((xe, 0.0, geo.z_f + 0.08))
    R = quat_to_mat(hook_quat(F.yaw))
    m, sd = rig.m, arm.scratch
    b6 = rig.m.body("right/link6").id
    best = None
    home = arm.home_q
    for off in ((0, 0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0, -1.5), (0, 0, 0, 0, 0, 0, 1.5), (0.5, 0, 0, 0, 0, 0, 0), (-0.5, 0, 0.5, 0, 0, 0, 0),
                (0, 0, 0, 0, 1.0, 0, 0), (0, 0, 0, 0, -1.0, 0, 0), (0, 0, -0.8, 0, 0, 0, 0)):
        q, pe, re, ok, cols = ik_solve_seeds(arm, pos, R, q_ref=home + np.asarray(off, float))
        if not ok or any(not (_loose(b1) or _loose(b2)) for b1, b2, _ in cols):
            continue
        if abs(q[6]) > 2.5:
            continue
        sd.qpos[:] = rig.d.qpos
        sd.qpos[arm.qadr] = q
        mujoco.mj_kinematics(m, sd)
        w6 = sd.xpos[b6].copy()
        score = float((w6 - F.p) @ F.ex)            # toward the toe
        if best is None or score > best[0]:
            best = (score, q, w6)
    if best is None:
        return None
    _log(rig, "hook entry posture: wrist toward the toe by %.3f m (link6 at %s, q7 %.2f)" % (best[0], np.round(best[2], 3), best[1][6]))
    return best[1]


def _hook_fetch(rig, s, which, row, side, geo, F):
    """The hook catches aglet `which` lying in the cavity and brings it up through ring (row, side). On success the
    aglet protrudes above the flap on the hook tip; the hook stays there (the gripper takes over)."""
    body, c, _ = _aglet(rig, s, which)
    cl = F.l(c)
    if not _in_shoe(rig, s, c, geo, F):
        _log(rig, "hook fetch: aglet %s is not in the shoe (local %s)" % (which, np.round(cl, 3)))
        return False
    z_low = geo.z_floor + 0.004
    phi_e = 0.0
    xe = float(np.clip(cl[0], geo.x_hc + 0.02, geo.x_tc - 0.02))
    # entry: foot along the shoe's x through the throat slot, high above, then down to just above the cavity floor
    q_entry = _hook_entry_posture(rig, F, xe, geo)
    if q_entry is not None:
        rig.move_joint("right", q_entry, timeout=5.0)
    _hook_go(rig, F, xe, 0.0, geo.z_f + 0.08, phi_e, tol=0.004, timeout=5.0, joint_ok=(q_entry is None))
    q7 = rig.sc.arms["right"].q()[6]
    if q7 - PHI_SIDE < -2.8 or q7 + PHI_SIDE > 2.8:
        _log(rig, "hook fetch: wrist q7 %.2f leaves no room for the foot sweeps, re-posing" % q7)
        q = _ik(rig, "right", F.w((xe, 0.0, geo.z_f + 0.08)), hook_quat(F.yaw + phi_e),
                refs=[rig.sc.arms["right"].home_q + np.array([0, 0, 0, 0, 0, 0, -1.0])])
        if q is not None:
            rig.move_joint("right", q, timeout=5.0)
            _hook_go(rig, F, xe, 0.0, geo.z_f + 0.08, phi_e, tol=0.004, timeout=3.0)
    _hook_go(rig, F, xe, 0.0, z_low, phi_e, tol=0.0015, timeout=4.0)
    loaded = False
    for attempt in range(3):
        body, c, _ = _aglet(rig, s, which)
        cl = F.l(c)
        if abs(cl[1]) >= 0.006:
            phi_c = math.copysign(PHI_SIDE, cl[1])
        else:
            phi_c = phi_e
        fx, fy = FOOT * math.cos(phi_c), FOOT * math.sin(phi_c)
        xs = float(np.clip(cl[0] - fx, geo.x_hc + 0.006, geo.x_tc - 0.006))
        ys = float(np.clip(cl[1] - fy, -geo.y_shaft, geo.y_shaft))
        z_c = float(np.clip(cl[2] - 0.008, z_low, geo.z_f - 0.018))     # the aglet may hang above the floor
        if phi_c != phi_e:
            _hook_go(rig, F, xe, 0.0, z_low, phi_c, tol=0.003, timeout=3.0)
        _hook_go(rig, F, xs, ys, z_c, phi_c, tol=0.0015, timeout=3.0)
        loaded = rig.hook(True)
        if not loaded:
            for dx, dy, dz in ((0.006, 0.0, 0), (-0.006, 0.0, 0), (0.0, 0.006, 0), (0.0, -0.006, 0), (0, 0, 0.008), (0.009, 0.009, 0), (-0.009, -0.009, 0), (0.009, -0.009, 0), (-0.009, 0.009, 0)):
                _hook_go(rig, F, float(np.clip(xs + dx, geo.x_hc + 0.006, geo.x_tc - 0.006)), float(np.clip(ys + dy, -geo.y_shaft, geo.y_shaft)),
                         float(np.clip(z_c + dz, z_low, geo.z_f - 0.016)), phi_c, tol=0.002, timeout=2.0)
                loaded = rig.hook(True)
                if loaded:
                    break
        tip, _ = rig.tip_pose()
        _log(rig, "hook catch attempt %d: aglet local %s, tip local %s, loaded %s" % (attempt + 1, np.round(cl, 3), np.round(F.l(tip), 3), loaded))
        if loaded:
            p_now = F.l(rig.tcp("right")[0])
            _hook_go(rig, F, p_now[0], p_now[1], z_low, phi_c, tol=0.002, timeout=2.0)      # down to the floor level with the aglet
            break
    if not loaded:
        _hook_go(rig, F, xe, 0.0, z_low, phi_e, tol=0.003, timeout=3.0)
        _hook_go(rig, F, xe, 0.0, geo.z_f + 0.08, phi_e, tol=0.004, timeout=4.0)
        return False
    if row is None:
        return _hook_exit_throat(rig, s, which, geo, F, xe, z_low, phi_c, snug_row=getattr(rig, "_lace_snug_row", 2),
                                 lay=getattr(rig, "_lace_lay", "beyond"), side=getattr(rig, "_lace_lay_side", None))
    # carry the aglet under the target ring: foot toward that side, shaft inside the throat
    sg = _sg(side)
    phi_o = sg * PHI_SIDE
    ring = geo.ring(row, side)
    xs_t = ring[0] - FOOT * math.cos(phi_o)
    ys_t = float(np.clip(ring[1] - FOOT * math.sin(phi_o), -geo.y_shaft, geo.y_shaft))
    p_now = F.l(rig.tcp("right")[0])
    if abs(phi_c - phi_o) > 1e-6:
        if abs(phi_c - phi_o) > math.pi / 2 + 0.1:
            _hook_go(rig, F, p_now[0], p_now[1], z_low, 0.0, tol=0.003, timeout=3.0)
        _hook_go(rig, F, p_now[0], p_now[1], z_low, phi_o, tol=0.003, timeout=3.0)
    _hook_go(rig, F, xs_t, ys_t, z_low, phi_o, tol=0.001, timeout=3.0)
    # rise: the tip (and the aglet on it) go up through the ring hole; the foot stays below the flap. The hook weld
    # on a 0.2 g aglet is soft, so the AGLET (not the tip) is centred under the hole and the rise is slow and watched.
    z_up = geo.z_f - 0.007
    for attempt in range(3):
        _hook_go(rig, F, xs_t, ys_t, z_low + 0.006, phi_o, tol=0.0008, timeout=2.5)
        rig.wait(0.25)
        _, c, _ = _aglet(rig, s, which)
        cl = F.l(c)
        tipl = F.l(rig.tip_pose()[0])
        off = cl[:2] - tipl[:2]
        xs_c = xs_t - off[0]
        ys_c = float(np.clip(ys_t - off[1], -geo.y_shaft, geo.y_shaft))
        stuck = False
        z = z_low + 0.006
        while z < z_up - 1e-6:
            z = min(z + 0.004, z_up)
            _hook_go(rig, F, xs_c, ys_c, z, phi_o, tol=0.001, timeout=1.0)
            _, c, _ = _aglet(rig, s, which)
            cl = F.l(c)
            tipl = F.l(rig.tip_pose()[0])
            if cl[2] < tipl[2] + 0.002:
                stuck = True
                break
        up = (not stuck) and cl[2] > geo.z_f + 0.008 and np.linalg.norm(cl[:2] - ring[:2]) < 0.005
        _log(rig, "hook rise attempt %d: aglet offset from tip %s mm, tip local %s aglet local %s stuck %s up %s" % (
            attempt + 1, np.round(1000 * off, 1), np.round(tipl, 4), np.round(cl, 4), stuck, up))
        if up:
            _hook_tilt(rig, s, which, geo, F, HOOK_TILT)
            return True
        _hook_go(rig, F, xs_c, ys_c, z_low + 0.006, phi_o, tol=0.002, timeout=2.0)
        xs_t += 0.0015 if attempt % 2 == 0 else -0.003
    return False


def _lay_spot(rig, F, side):
    """Bench spot where a tail whose last ring is on `side` is laid: the lace drapes from the slot over the flap and
    the outer wall of that column; -x column -> beside the heel, +x column -> beside the toe (world coordinates)."""
    cx, cy = float(F.p[0]), float(F.p[1])
    col_x = 1.0 if float((_sg(side) * F.ey)[0]) > 0.0 else -1.0
    if col_x > 0.0:
        return np.array([cx + 0.13, cy - 0.07, 0.0])
    return np.array([cx - 0.10, cy + 0.11, 0.0])


SLOT_PULL_STEPS = 24   # 16 mm draws: the first few only take up the tail's own slack, a 0.2 m arc needs about 20


def _hook_exit_throat(rig, s, which, geo, F, xe, z_low, phi_c, snug_row=2, lay="beyond", side=None):
    """Loaded hook at the cavity floor: turn the foot along the slot, rise straight up through the slot (the aglet
    tip-up on the tip) to tip z 0.15, draw the lace toward the toe (-y) in 16 mm steps until this tail's segment
    from row (snug_row-1) to snug_row is snug (scorer rule), then lay the aglet on the bench: lay == "beyond" ->
    9 cm beyond the toe cap (y < cradle_y - 0.12), lay == "beside" -> beside the shoe on the gripper-reachable +x
    side; release it and park. Returns True when the aglet ends outside the cavity."""
    p_now = F.l(rig.tcp("right")[0])
    _hook_go(rig, F, p_now[0], p_now[1], z_low, phi_c, tol=0.003, timeout=2.0)
    _hook_go(rig, F, xe, 0.0, z_low, phi_c, tol=0.003, timeout=3.0)
    if abs(phi_c) > 1e-6:
        _hook_go(rig, F, xe, 0.0, z_low, 0.0, tol=0.003, timeout=3.0)
    z = z_low
    while z < geo.z_f + 0.06 - 1e-6:
        z = min(z + 0.01, geo.z_f + 0.06)
        if not _hook_go(rig, F, xe, 0.0, z, 0.0, tol=0.002, timeout=1.5):
            break
        if not rig.hook_loaded:
            break
    _, c, _ = _aglet(rig, s, which)
    _log(rig, "hook out through the slot: loaded %s, aglet local %s (t=%.1f)" % (rig.hook_loaded, np.round(F.l(c), 3), rig.time))
    if not rig.hook_loaded:
        _hook_go(rig, F, xe, 0.0, geo.z_f + 0.08, 0.0, tol=0.004, timeout=3.0)
        _park_hook(rig, near=F.p[:2])
        return not rig.sc.inside_cavity(s, c)
    quat = hook_quat(F.yaw)
    tip0 = F.w((xe, 0.0, 0.0))
    tip0[2] = 0.15
    goto(rig, "right", rig.tcp_for_tip(tip0, quat), quat, tol=0.004, rot_tol=0.04, timeout=3.0, joint_ok=False)
    # draw the lace toward the lay-down spot until the segment is snug; stop when this tail's pass list changes
    spot = _lay_spot(rig, F, side) if side is not None else F.w((geo.x_tc + 0.09, 0.0, 0.0))
    o = spot - tip0
    o[2] = 0.0
    d_spot = float(np.linalg.norm(o))
    o = o / max(1e-6, d_spot)
    chain0 = [(r, sd) for r, sd, d, t in _chain(rig, s, which)]
    for k in range(SLOT_PULL_STEPS):
        ok, arc, allowed = _tail_snug(rig, s, which, geo, snug_row)
        chain = [(r, sd) for r, sd, d, t in _chain(rig, s, which)]
        _log(rig, "slot pull row %d step %d: arc %.3f allowed %.3f ok %s, chain %s" % (snug_row, k, arc, allowed, ok, chain))
        if ok:
            break
        if ok is None or chain != chain0:
            _log(rig, "slot pull row %d: segment not measurable / pass list changed, stopping" % snug_row)
            break
        if k == SLOT_PULL_STEPS - 1:
            break
        # toward the lay spot as far as it, then straight up: further along the floor the hook arm's wrist reached the
        # waiting gripper (the +x column's spot lies in line with the gripper's home)
        run = 0.016 * (k + 1)
        tip = tip0 + o * min(run, d_spot) + Z * max(0.0, run - d_spot)
        if not goto(rig, "right", rig.tcp_for_tip(tip, quat), quat, tol=0.003, rot_tol=0.04, timeout=2.5, joint_ok=False):
            break
    # lay the aglet down over the side wall of the last ring's column (travel high, descend at the spot: never lower
    # the aglet while the tip is still over the slot; going further out along the pull keeps the tail taut but takes
    # the hook arm's wrist into the waiting gripper's space)
    p = spot.copy()
    p[2] = 0.032
    tipw, _ = rig.tip_pose()
    goto(rig, "right", rig.tcp_for_tip(np.array([p[0], p[1], max(tipw[2], 0.12)]), quat), quat, tol=0.005, rot_tol=0.04, timeout=4.0, joint_ok=False)
    _slow_goto(rig, "right", rig.tcp_for_tip(p, quat), quat, v=0.1, tol=0.004)
    rig.hook(False)
    _wait(rig, 0.5)
    goto(rig, "right", rig.tcp_for_tip(p + Z * 0.10, quat), quat, tol=0.006, rot_tol=0.05, timeout=2.5, joint_ok=False)
    _, c, _ = _aglet(rig, s, which)
    inside = rig.sc.inside_cavity(s, c)
    _log(rig, "aglet %s laid down by the hook (%s) at %s inside cavity %s (t=%.1f)" % (which, lay, np.round(c, 3), inside, rig.time))
    _park_hook(rig, near=F.p[:2])
    return not inside


HOOK_TILT = 0.30      # rad: shaft leaned along the slot toward the toe at the take-over (wrist 6 cm further from the gripper)


def _hook_tilt(rig, s, which, geo, F, theta):
    """Lean the hook's shaft by `theta` about the foot axis through the tip end (the tip stays through the ring,
    the aglet tilts with it): the hook arm's flange and wrist move toward the toe, clear of the gripper's approach."""
    if theta <= 0.0:
        return True
    E, R = rig.tip_pose()
    axis = F.ey.copy()                      # the foot runs along the shoe's y (across the slot)
    for sign in (1.0, -1.0):
        rq = rot_quat(axis, sign * theta)
        top = quat_to_mat(rq) @ np.array([0.0, 0.0, 1.0])
        if float(top @ F.ex) > 0.0:         # the shaft top moves toward the toe
            break
    quat = quat_mul(rq, mat_to_quat(R))
    tcp = rig.tcp_for_tip(E, quat)
    ok = _slow_goto(rig, "right", tcp, quat, v=0.05, w=0.4, tol=0.002, rot_tol=0.03)
    _, c, ax = _aglet(rig, s, which)
    _log(rig, "hook tilted by %.2f rad: ok %s, loaded %s, aglet local %s axis %s" % (sign * theta, ok, rig.hook_loaded, np.round(F.l(c), 4), np.round(ax, 2)))
    return ok and rig.hook_loaded


def _hook_retreat(rig, geo, F):
    """After a handover: the hook (unloaded) leaves the cavity through the throat and parks beside the cradle."""
    p = F.l(rig.tcp("right")[0])
    z_low = geo.z_floor + 0.004
    _hook_go(rig, F, p[0], p[1], z_low + 0.004, _foot_phi(rig, F), tol=0.002, timeout=2.0)
    xe = float(np.clip(p[0], geo.x_hc + 0.02, geo.x_tc - 0.02))
    _hook_go(rig, F, xe, 0.0, z_low + 0.004, 0.0, tol=0.003, timeout=3.0)
    _hook_go(rig, F, xe, 0.0, geo.z_f + 0.08, 0.0, tol=0.004, timeout=4.0)
    _park_hook(rig, near=F.p[:2])


def _foot_phi(rig, F):
    _, R = rig.tcp("right")
    f = R[:, 0]
    return math.atan2(f[1], f[0]) - F.yaw


def _handover(rig, s, which, row, side, geo, F):
    """The gripper takes the aglet from the hook tip above ring (row, side): approach tilted outward (the palm clears
    the hook shaft), fingers closing along the shoe's length, tcp shifted 3 mm toward the toe (the heel-side finger
    then clears the heel counter at the back rows)."""
    body, c, _ = _aglet(rig, s, which)
    sg = _sg(side)
    o = sg * F.ey
    rig.gripper(0.03, wait=False)
    # The gripper's wrist must stay clear of the hook arm's wrist, which stands above the throat in a posture that
    # depends on how the hook got there: pick the hand pose at run time from a candidate set (lean about the closing
    # axis, closing-axis yaw offset, tilt toward the toe, outward tcp offset) with the collision-checking IK against
    # the hook arm's actual configuration, for both the grasp pose and the pose 6 cm back along the approach.
    chosen = None
    far = float(np.dot(o, np.array([0.0, -1.0, 0.0]))) > 0.5
    if far:      # (lean, tilt toward the toe, outward offset): validated by the IK/collision probes at the fetch poses
        cands = ((0.7, 0.3, 0.0), (0.9, 0.3, 0.0), (0.7, 0.3, 0.012), (0.5, 0.0, 0.012), (0.9, 0.0, 0.0), (0.7, 0.0, 0.012))
    else:
        cands = ((0.7, 0.0, 0.0), (0.9, 0.0, 0.0), (0.7, 0.3, 0.0), (0.7, -0.3, 0.0), (0.5, 0.0, 0.006), (0.5, 0.3, 0.006), (0.5, -0.3, 0.006))
    for lean, tilt, off in cands:
        q0 = _lean_quat(F.yaw, sg, lean)
        quat = quat_mul(rot_quat(F.ey, tilt), q0)
        R = quat_to_mat(quat)
        tz = R[:, 2]
        tcp = c - AGLET_GRIP_DZ * tz + 0.003 * F.ex + off * o
        if _ik(rig, "left", tcp, quat, rot_tol=0.08) is not None and _ik(rig, "left", tcp - 0.06 * tz, quat, rot_tol=0.08) is not None:
            chosen = (lean, tilt, 0.0, off, quat, tz, tcp)
            break
    if chosen is None:
        _log(rig, "handover row %d side %s: no collision-free hand pose against the hook arm, trying the default" % (row, side))
        quat = _lean_quat(F.yaw, sg, LEAN)
        R = quat_to_mat(quat)
        tz = R[:, 2]
        tcp = c - AGLET_GRIP_DZ * tz + 0.003 * F.ex
    else:
        lean, tilt, yoff, off, quat, tz, tcp = chosen
        _log(rig, "handover row %d side %s: hand pose lean %.1f tilt %.1f yaw_off %.1f offset %.3f" % (row, side, lean, tilt, yoff, off))
    pre = tcp - 0.06 * tz
    # come in low from the toe side: the hook arm's wrist stands above the throat at z ~ 0.3
    w1 = np.array([tcp[0] + 0.18, tcp[1] + 0.03 * sg, 0.17])
    goto(rig, "left", w1, quat, tol=0.008, rot_tol=0.08, timeout=5.0, joint_ok=True)
    if not _slow_goto(rig, "left", pre, quat, v=0.07, w=0.6, tol=0.003, rot_tol=0.05):
        if not goto(rig, "left", pre, quat, tol=0.003, rot_tol=0.05, timeout=3.0, joint_ok=False):
            _log(rig, "handover row %d side %s: the gripper cannot reach the pre-grasp pose, aborting the handover" % (row, side))
            p_now, _ = rig.tcp("left")
            goto(rig, "left", np.array([p_now[0] + 0.10, p_now[1] + 0.03 * sg, 0.20]), quat, tol=0.01, rot_tol=0.1, timeout=3.0, joint_ok=False)
            return False
    rig.gripper(0.03)
    body, c, _ = _aglet(rig, s, which)
    _, Rt = rig.tcp("left")
    tz = Rt[:, 2]                                                    # the approach axis the hand actually has
    tcp = c - AGLET_GRIP_DZ * tz + 0.003 * F.ex
    if not _slow_goto(rig, "left", tcp, mat_to_quat(Rt), v=0.06, w=0.5, tol=0.0012, rot_tol=0.03):
        goto(rig, "left", tcp, mat_to_quat(Rt), tol=0.0012, rot_tol=0.03, timeout=2.0, joint_ok=False)
    pt, Rt = rig.tcp("left")
    _log(rig, "handover row %d: aglet in the tcp frame %s mm, hook loaded %s" % (row, np.round(1000 * (Rt.T @ (c - pt)), 1), rig.hook_loaded))
    got = False
    for k in range(3):
        got, off = _take_from_hook(rig, body)
        if got:
            break
        if off is None or not rig.hook_loaded:
            break
        # re-align (bounded): put the aglet centre at (0, 0, AGLET_GRIP_DZ) in the tcp frame
        d = np.array([off[0], 0.0, off[2] - AGLET_GRIP_DZ])
        if np.linalg.norm(d) > 0.008:
            d = d / np.linalg.norm(d) * 0.008
        rig.gripper(0.03)
        pt, Rt = rig.tcp("left")
        tcp = pt + Rt @ d
        goto(rig, "left", tcp, mat_to_quat(Rt), tol=0.001, rot_tol=0.03, timeout=2.0, joint_ok=False)
        _log(rig, "handover row %d: re-aligned by %s mm (try %d)" % (row, np.round(1000 * d, 1), k + 1))
    if not got:
        _log(rig, "handover row %d side %s: the gripper did not take aglet %s (holding %s)" % (row, side, which, rig.holding))
        rig.release(0.3)
        return False
    _log(rig, "handover row %d side %s: aglet %s in the gripper (t=%.1f)" % (row, side, which, rig.time))
    # lift the aglet clear of the ring along the approach, then the hook leaves
    goto(rig, "left", tcp - 0.035 * tz, quat, tol=0.003, timeout=2.0, joint_ok=False)
    _hook_retreat(rig, geo, F)
    return True


def _pull_snug(rig, s, which, row, side, geo, F, max_steps=12):
    """Holding the aglet just out of ring (row, side): pull it toward +y (the gripper's side) and up, 16 mm steps,
    until this tail's lace is snug (arc <= 1.22 x straight for rows 2..row), without starving the other tail."""
    pt, Rt = rig.tcp("left")
    quat = mat_to_quat(Rt)
    base = pt.copy()
    o = np.array([0.0, 1.0, 0.0])
    for k in range(max_steps + 1):
        ratios = _my_snug(rig, s, which, geo)
        passes, snug, la, lb, _, _ = _tails(rig, s)
        other_free = lb if which == "a" else la
        worst = max([r for rr, r in ratios.items() if rr <= row] or [9.0])
        _log(rig, "pull row %d step %d: my slack ratios %s, other tail free %.3f m, passes %s" % (row, k, {r: round(v, 3) for r, v in ratios.items()}, other_free, _fmt_passes(passes, float(np.sum(rig.sc.lace_lengths[s])))))
        if ratios and worst <= SNUG_RATIO - 0.03 and all(r in ratios for r in range(2, row + 1)):
            return True
        if 2 not in ratios:
            _log(rig, "pull row %d: the row-2 segment is not measurable, not pulling" % row)
            return False
        if other_free < 0.05:
            _log(rig, "pull row %d: the other tail is running short, stopping" % row)
            return False
        if k == max_steps:
            break
        tgt = base + o * 0.016 * (k + 1) + Z * 0.005 * (k + 1)
        if not 0.15 < np.linalg.norm(tgt[:2] - LEFT_BASE) < 0.50:
            _log(rig, "pull row %d: next pull point out of the gripper's zone, stopping" % row)
            break
        goto(rig, "left", tgt, quat, tol=0.003, timeout=2.5, joint_ok=False)
    return False


def _set_down_tail(rig, s, which, side, geo, F, dist=0.10):
    """Holding the aglet: lay it on the bench at this side's set-down spot (gripper side, -x/+y quadrant, away from
    the slot) and release; the hook is parked, so the travel may be in joint space."""
    spot = SETDOWN[side]
    quat = down_quat(math.pi / 2)
    p = np.array([spot[0], spot[1], 0.032])
    pt, Rt = rig.tcp("left")
    if pt[2] < 0.14:
        goto(rig, "left", np.array([pt[0], pt[1], 0.16]), mat_to_quat(Rt), tol=0.006, rot_tol=0.05, timeout=2.0, joint_ok=False)
    if not goto(rig, "left", p + Z * 0.08, quat, tol=0.006, rot_tol=0.05, timeout=5.0, joint_ok="free"):
        _log(rig, "tail set-down: cannot reach above %s" % (spot,))
    _slow_goto(rig, "left", p, quat, v=0.08, tol=0.003)
    rig.wait(0.5)
    rig.release(0.5)
    rig.lift("left", 0.08, timeout=2.0)
    _, c, _ = _aglet(rig, s, which)
    _log(rig, "aglet %s set down at %s inside cavity %s" % (which, np.round(c, 3), rig.sc.inside_cavity(s, c)))


def _out_pass(rig, s, which, row, side, geo, F):
    """The full OUT pass of aglet `which` through ring (row, side): hook fetch + ring-top take-over. The gripper holds
    the aglet at the end; the hook is parked. Fallback when the take-over fails: the hook takes the aglet out through
    the slot and lays it down outside (row 2 stays credited)."""
    _gripper_wait(rig, F, side)
    if not _hook_fetch(rig, s, which, row, side, geo, F):
        _hook_go(rig, F, float(np.clip(F.l(rig.tcp("right")[0])[0], geo.x_hc + 0.02, geo.x_tc - 0.02)), 0.0, geo.z_f + 0.08, 0.0, tol=0.004, timeout=4.0)
        _park_hook(rig, near=F.p[:2])
        return False
    for attempt in range(2):
        if _takeover(rig, s, which, row, side, geo, F):
            _hook_retreat(rig, geo, F)
            return True
        if not rig.hook_loaded:
            break
    if rig.hook_loaded:
        _log(rig, "tail %s: take-over failed, the hook takes the aglet out through the slot" % which)
        pl = F.l(rig.tcp("right")[0])
        xe = float(np.clip(pl[0], geo.x_hc + 0.02, geo.x_tc - 0.02))
        _hook_exit_throat(rig, s, which, geo, F, xe, geo.z_floor + 0.004, _foot_phi(rig, F), snug_row=2, lay="beyond")
    else:
        _hook_retreat(rig, geo, F)
    return False


def _even_tails(rig, s, geo, F):
    """Pull the short tail's aglet away from the shoe until both tails have about the same free length."""
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    total = float(np.sum(rig.sc.lace_lengths[s]))
    _log(rig, "tails: A %.3f m (side %s), B %.3f m (side %s); passes %s" % (la, side_a, lb, side_b, _fmt_passes(passes, total)))
    return True                          # (no evening: pulling snug draws lace through row 1 as required)
    which = "a" if la < lb else "b"
    side = side_a if which == "a" else side_b
    if not _acquire_aglet(rig, s, which, geo, F):
        _log(rig, "even tails: cannot grasp aglet %s" % which)
        return False
    ring1 = F.w(geo.ring(1, side))
    quat = down_quat(F.yaw)
    p, _ = rig.tcp("left")
    r = max(0.12, float(np.linalg.norm(p[:2] - ring1[:2])))
    goto(rig, "left", np.array([p[0], p[1], 0.15]), quat, tol=0.005, timeout=3.0, joint_ok=False)
    for step in range(7):
        passes, snug, la, lb, _, _ = _tails(rig, s)
        diff = (lb - la) if which == "a" else (la - lb)
        _log(rig, "even tails step %d: A %.3f B %.3f (pull radius %.2f)" % (step, la, lb, r))
        if diff < 0.03:
            break
        r += min(0.05, max(0.015, diff / 2))
        tgt = np.array([ring1[0] + r * PULL_DIR[0], ring1[1] + r * PULL_DIR[1], 0.15])
        if np.linalg.norm(tgt[:2] - LEFT_BASE) > 0.54:
            _log(rig, "even tails: pull target out of reach, stopping")
            break
        goto(rig, "left", tgt, quat, tol=0.005, timeout=4.0, joint_ok=False)
        tgt[2] = 0.045
        goto(rig, "left", tgt, quat, tol=0.004, timeout=3.0, joint_ok=False)
        rig.wait(0.3)
    return True


def _ensure_row1(rig, s, geo, F):
    """Row 1 must be threaded the delivered way (A outside on one side entering, B leaving on the other); when one
    end has slipped out of its ring, drop that aglet into the cavity through the throat and fetch it up through its
    ring with the hook."""
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    if len(r1) == 2 and passes[r1[0]][2] == -1 and passes[r1[1]][2] == 1 and side_a != side_b:
        return True
    total = float(np.sum(rig.sc.lace_lengths[s]))
    _log(rig, "row 1 is not threaded as delivered: %s" % _fmt_passes(passes, total))
    if len(r1) != 1:
        return False
    have = passes[r1[0]]
    which = "b" if have[2] == -1 else "a"
    side = _other(have[1])
    _, c, _ = _aglet(rig, s, which)
    if not rig.sc.inside_cavity(s, c):
        if not _acquire_aglet(rig, s, which, geo, F):
            return False
        _drop_throat(rig, s, which, geo, F)
    if not _out_pass(rig, s, which, 1, side, geo, F):
        return False
    sg = _sg(side)
    o = sg * F.ey
    e = F.w(geo.ring(1, side))
    goto(rig, "left", e + o * 0.05 + Z * 0.08, down_quat(F.yaw), tol=0.004, timeout=3.0, joint_ok=False)
    p = e + o * 0.10
    p[2] = 0.032
    goto(rig, "left", p, down_quat(F.yaw), tol=0.004, timeout=3.0, joint_ok=False)
    rig.release(0.4)
    rig.lift("left", 0.08, timeout=2.0)
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    _log(rig, "row 1 repaired: %s" % _fmt_passes(passes, total))
    r1 = [i for i, p in enumerate(passes) if p[0] == 1]
    return len(r1) == 2 and passes[r1[0]][2] == -1 and passes[r1[1]][2] == 1


def _hook_slot_pass(rig, s, which, geo, F, snug_row, lay, side=None):
    """Hook: fetch the aglet lying under / in the ring it was dropped through, out through the slot, pull the segment
    to row `snug_row` snug, lay it down (`lay`), park. The gripper is parked first."""
    _gripper_home(rig)
    rig._lace_snug_row = snug_row
    rig._lace_lay = lay
    rig._lace_lay_side = side
    ok = _hook_fetch(rig, s, which, None, None, geo, F)
    if not ok and rig.hook_loaded:
        rig.hook(False)
    if rig.sc.arms["right"].q() is not None:
        _park_hook(rig, near=F.p[:2])
    return ok


def _hook_relay(rig, s, which, geo, F, spot):
    """The hook catches aglet `which` on the bench (free-space catch) and lays it down at `spot`; the gripper is
    parked first. Returns True when the aglet lies outside the cavity afterwards."""
    _gripper_home(rig)
    if not _hook_catch_free(rig, s, which, geo, F):
        _park_hook(rig, near=F.p[:2])
        return False
    quat = hook_quat(HOOK_FOOT_FREE)
    p = np.array([spot[0], spot[1], 0.032])
    tipw, _ = rig.tip_pose()
    goto(rig, "right", rig.tcp_for_tip(np.array([p[0], p[1], max(tipw[2], 0.12)]), quat), quat, tol=0.005, rot_tol=0.05, timeout=4.0, joint_ok=False)
    _slow_goto(rig, "right", rig.tcp_for_tip(p, quat), quat, v=0.1, tol=0.004)
    rig.hook(False)
    _wait(rig, 0.4)
    goto(rig, "right", rig.tcp_for_tip(p + Z * 0.10, quat), quat, tol=0.006, rot_tol=0.05, timeout=2.5, joint_ok=False)
    _park_hook(rig, near=F.p[:2])
    _, c, _ = _aglet(rig, s, which)
    _log(rig, "aglet %s re-laid by the hook at %s (t=%.1f)" % (which, np.round(c, 3), rig.time))
    return not rig.sc.inside_cavity(s, c)


def _pick_with_relay(rig, s, which, geo, F):
    """Pick the aglet; when no pick pose exists, have the hook re-lay it 4 cm further from the gripper's base."""
    if _acquire_aglet(rig, s, which, geo, F):
        return True
    _, c, _ = _aglet(rig, s, which)
    away = c[:2] - LEFT_BASE
    away = away / max(1e-6, np.linalg.norm(away))
    spot = np.array([c[0] + 0.04 * away[0], c[1] + 0.04 * away[1], 0.0])
    _log(rig, "tail %s: no pick pose at %s, asking the hook to re-lay it at %s" % (which, np.round(c[:2], 3), np.round(spot[:2], 3)))
    if not _hook_relay(rig, s, which, geo, F, spot):
        return False
    return _acquire_aglet(rig, s, which, geo, F)


def _lace_tail(rig, s, which, side_own, geo, F):
    """One throat-return pass per tail (n_rows = 2): row 2 IN on the side opposite to the tail's row-1 exit, hook out
    through the slot, the row-1 -> row-2 diagonal pulled snug, the aglet laid over the side wall of ring 2's column."""
    other = _other(side_own)
    if not _pick_with_relay(rig, s, which, geo, F):
        _log(rig, "tail %s: no aglet grasp, giving up this tail" % which)
        return False
    if not _drop_in(rig, s, which, 2, other, geo, F, 1, side_own):
        _log(rig, "tail %s: IN pass at row 2 failed" % which)
        h = rig.holding
        if h is not None and h[0] == rig.sc.laces[s]["bodies"][0 if which == "a" else -1]:
            _set_down_tail(rig, s, which, side_own, geo, F)
        return False
    _log(rig, "tail %s after row 2: chain %s (t=%.1f)" % (which, [(r, sd, d) for r, sd, d, t in _chain(rig, s, which)], rig.time))
    if not _hook_slot_pass(rig, s, which, geo, F, snug_row=2, lay="wall", side=other):
        _log(rig, "tail %s: slot fetch after row 2 failed" % which)
        return False
    return True


def lace_shoe(rig, s):
    """Criss-cross lacing of the seated shoe s through rows 2..n with both lace ends, pulled snug."""
    sc = rig.sc
    _patch_cell_bugs(rig)
    _check_world(rig, s)
    geo = _Geom(rig, s)
    if rig.s.clamp_engaged != s:
        _settle_seated(rig, s, wait_s=1.5)
    if rig.s.clamp_engaged != s:
        _log(rig, "warning: shoe %d is not clamped (seated %s)" % (s, sc.seated(s)))
    F = _Frame(rig, s)
    _log(rig, "lace shoe %d: L %.3f W %.3f g %.4f rows %s (t=%.1f)" % (s, geo.L, geo.W, geo.g, np.round(geo.x_rows, 3), rig.time))
    if rig.holding is not None:
        rig.release()
    arm_l = rig.sc.arms["left"]
    pl, _ = rig.tcp("left")
    if pl[2] < 0.22:
        rig.lift("left", 0.24 - pl[2], timeout=1.5)
    arm_l.set_mode("joint")
    arm_l.command_joint(arm_l.home_q)            # the gripper goes home while the hook stops the lace: the tails slide on
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    first = "a" if la <= lb else "b"
    if side_a is not None:
        _acquire_aglet(rig, s, first, geo, F)
    if not _ensure_row1(rig, s, geo, F):
        _log(rig, "shoe %d: row 1 cannot be restored, lacing skipped" % s)
        _gripper_home(rig)
        return False
    _even_tails(rig, s, geo, F)
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    ok = True
    order = (("a", side_a), ("b", side_b)) if (rig.holding is None or rig.holding[0] == rig.sc.laces[s]["bodies"][0]) else (("b", side_b), ("a", side_a))
    for which, side_own in order:
        if side_own is None:
            ok = False
            break
        if not _lace_tail(rig, s, which, side_own, geo, F):
            ok = False
    passes, snug, la, lb, side_a, side_b = _tails(rig, s)
    total = float(np.sum(sc.lace_lengths[s]))
    _log(rig, "shoe %d laced: passes %s snug %s tails A %.3f B %.3f (t=%.1f)" % (s, _fmt_passes(passes, total), snug, la, lb, rig.time))
    _park_hook(rig)
    _gripper_home(rig)
    return ok
