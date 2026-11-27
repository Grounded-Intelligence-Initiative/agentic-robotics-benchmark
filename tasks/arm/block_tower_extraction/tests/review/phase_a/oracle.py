"""Phase-A privileged oracle (turntable cell).

Per block: turn the table so the block lies radially; push it 30 mm out at the back with the pusher rod; turn the
table 180 deg so the protruding end faces the robot; grasp it there with the hand tilted (fingertips toward the
tower); pull it free; set it down on the staging post; regrasp it lengthwise from above; place it on top.
Privileged: block poses and live contact loads from the simulator (a block is 'loose' if nothing presses on it).
"""
import json
import math
import os
import sys
import time

import mujoco
import numpy as np

import cell as C
import tower as T

PANDA = os.environ.get("BT_PANDA_DIR", os.environ.get("CNC_PANDA_DIR", ""))   # Menagerie panda dir (converted meshes)
PROTRUDE = 0.030        # push until the block sticks out this far at the back
GRASP_IN = 0.014        # pad centre this far from the protruding end
GRASP_PITCH = math.radians(30.0)
GRASP_UP = 5e-5         # grip a hair high: too low lifts the inner end ~3x more than too high lifts the edge
PUSH_SPEED = 0.010
LOOSE_N = 0.02
COLLAPSE_M = 0.010      # a block (table frame) more than this from its reference pose: collapsed


def log(*a):
    print("[oracle]", *a, file=sys.stderr, flush=True)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def table_frame(c, p):
    th = c.table_angle()
    ct, st = math.cos(-th), math.sin(-th)
    p = np.asarray(p, float)
    d = p[..., :2] - T.TOWER_XY
    x = ct * d[..., 0] - st * d[..., 1]
    y = st * d[..., 0] + ct * d[..., 1]
    return np.stack([x, y, p[..., 2]], axis=-1)


class Collapse(Exception):
    pass


class Game(object):
    def __init__(self, cell):
        self.c = cell
        self.n = len(cell.ids)
        self.level_of = {i: b["level"] for i, b in enumerate(cell.tower["blocks"])}
        self.slot_of = {i: b["slot"] for i, b in enumerate(cell.tower["blocks"])}
        self.out = set()             # blocks in hand / staging
        self.placed = []
        self.relocated = 0
        self.collapsed = False
        self.ref = table_frame(cell, cell.d.xpos[cell.ids])     # table-frame reference positions

    def members(self, k):
        return [i for i in range(self.n) if i not in self.out and self.level_of[i] == k]

    def top_level(self):
        return max(self.level_of[i] for i in range(self.n) if i not in self.out)

    def legal_max_level(self):
        """Blocks may be taken only from levels below the top two complete levels."""
        comp = [k for k in range(self.top_level() + 1) if len(self.members(k)) == 3]
        if len(comp) < 2:
            return -1
        return sorted(comp)[-2] - 1

    def disturbance(self, exclude=()):
        p = table_frame(self.c, self.c.d.xpos[self.c.ids])
        worst, wj = 0.0, -1
        for j in range(self.n):
            if j in self.out or j in exclude:
                continue
            e = float(np.linalg.norm(p[j] - self.ref[j]))
            if e > worst:
                worst, wj = e, j
        return worst, wj


def top_loads(c):
    """Live per-block load from above. Sleeping islands carry no contact rows: evaluate with sleep off."""
    m = c.m
    fl = m.opt.enableflags
    m.opt.enableflags = fl & ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    mujoco.mj_forward(m, c.d)
    out = T.block_loads(m, c.d, c.ids)
    m.opt.enableflags = fl
    return out


MIN_GAP = 6e-5          # a loose block must clear the level above by this much (m)


def gap_above(c, game, i):
    """Vertical clearance between block i's top and the lowest block resting above it (privileged geometry)."""
    k = game.level_of[i]
    top = c.d.xpos[c.ids[i]][2] + c.tower["blocks"][i]["h"] / 2
    ab = game.members(k + 1)
    if not ab:
        return 1.0
    bot = min(c.d.xpos[c.ids[j]][2] - c.tower["blocks"][j]["h"] / 2 for j in ab)
    return bot - top - 2 * T.MARGIN


def removal_ok(game, i):
    """The level stays a two-point support after removing i (never a lone side or lone middle block)."""
    k = game.level_of[i]
    rest = sorted(game.slot_of[j] for j in game.members(k) if j != i)
    return rest in ([0, 2], [0, 1, 2], [0, 1], [1, 2])


def contacts_above(c, game, i):
    """(total normal force, mean contact position) between block i and blocks resting on it (privileged)."""
    import mujoco as mj
    m, d = c.m, c.d
    b = c.ids[i]
    f6 = np.zeros(6)
    tot = 0.0
    pos = np.zeros(3)
    for k in range(d.ncon):
        con = d.contact[k]
        if con.efc_address < 0:
            continue
        b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
        if b not in (b1, b2):
            continue
        o = b2 if b1 == b else b1
        if o not in c.idset or abs(con.frame[2]) < 0.9 or d.xpos[o][2] < d.xpos[b][2] + 0.005:
            continue
        mj.mj_contactForce(m, d, k, f6)
        tot += abs(f6[0])
        pos += abs(f6[0]) * con.pos
    return tot, (pos / tot if tot > 0 else None)


def force_above(c, game, i):
    return contacts_above(c, game, i)[0]


def relax(c, i, tgt, R, game, a_out, step=2e-5, n=25):
    """Keep the held block clear of the level above while it is still partly inside the tower. The block pivots:
    a grip that is too low lifts the block's inner end (contact deep inside: raise the grip), a grip that is too
    high lifts it at the tower's edge (contact near the face: lower the grip)."""
    for _ in range(n):
        f, p = contacts_above(c, game, i)
        if f < 0.005:
            return tgt
        pb, _ = c.block_pose(i)
        # contact further inside than the block centre -> inner end up -> raise
        inner = float(np.dot(p[:2] - pb[:2], a_out[:2])) < 0.0
        tgt = tgt + np.array([0, 0, step if inner else -step])
        c.move("tcp", tgt, R, speed=0.005)
    return tgt


def block_axis(c, i):
    p, R = c.block_pose(i)
    a = R[:, 0].copy()
    a[2] = 0.0
    return p, a / np.linalg.norm(a)


def safe_z(c, game, margin=0.09):
    zs = c.d.xpos[[c.ids[j] for j in range(game.n) if j not in game.out], 2]
    return float(zs.max()) + margin


def lift(c, game, dz=0.0):
    p, Rc = c.site_pose("tcp")
    if np.linalg.norm(p[:2] - T.TOWER_XY) < 0.08:
        dz = min(dz, -0.04)          # over the tower top: 5 cm above the top block centres (reach limit)
    z = safe_z(c, game) + dz
    if p[2] < z - 0.002:
        c.move("tcp", [p[0], p[1], z], Rc, speed=0.2)


def travel(c, game, xy, R, dz=0.0):
    lift(c, game, dz=dz)
    if np.linalg.norm(np.asarray(xy[:2]) - T.TOWER_XY) < 0.08:
        dz = min(dz, -0.04)
    z = safe_z(c, game) + dz
    c.move("tcp", [xy[0], xy[1], z], R, speed=0.3)


FALL_DZ = 0.005         # collapse: a block (not in hand) more than 5 mm below its reference height ...
FALL_TILT = math.radians(5.0)   # ... or tilted more than 5 deg


def fallen(c, game, exclude=()):
    worst = None
    for j in range(game.n):
        if j in game.out or j in exclude:
            continue
        b = c.ids[j]
        dz = game.ref[j][2] - c.d.xpos[b][2]
        tilt = math.acos(max(-1.0, min(1.0, float(c.d.xmat[b][8]))))
        tilt = min(tilt, math.pi - tilt)
        if dz > FALL_DZ or tilt > FALL_TILT:
            return j, dz, tilt
    return worst


def check(c, game, what, exclude=()):
    """Collapse = a block fell (z drop > 5 mm) or tilted > 5 deg; also returns the largest table-frame shift."""
    w, j = game.disturbance(exclude)
    game.max_shift = max(getattr(game, "max_shift", 0.0), w)
    f = fallen(c, game, exclude)
    if f is not None:
        game.collapsed = True
        raise Collapse("%s: block %d fell/tilted (dz %.1f mm, tilt %.1f deg); max shift %.1f mm (block %d)"
                       % (what, f[0], f[1] * 1000, math.degrees(f[2]), w * 1000, j))
    return w


def make_radial(c, i):
    """Turn the table so block i's axis is along world x."""
    p, a = block_axis(c, i)
    d = wrap(-math.atan2(a[1], a[0]))
    if d > math.pi / 2:
        d -= math.pi
    if d < -math.pi / 2:
        d += math.pi
    c.rotate_table(c.table_cmd + d)


def prepose(c, low_pos, R):
    """Before a long Cartesian descent: pick an arm posture that reaches the LOW goal (IK from the home posture),
    move to the matching posture at the current (high) hand position in joint space, keep it as null-space target."""
    q_low, ep, er = c.solve("tcp", low_pos, R, C.Q_HOME, iters=150)
    if ep > 5e-4 or er > 5e-3:
        return False
    hi, _ = c.site_pose("tcp")
    q_hi, ep, er = c.solve("tcp", hi, R, q_low, iters=150)
    if ep > 5e-4 or er > 5e-3:
        return False
    c.pose_q(q_hi, speed=0.8)
    c.q_nom = q_low.copy()
    return True


def extract(c, game, i):
    make_radial(c, i)
    check(c, game, "turn")
    p, a = block_axis(c, i)
    if a[0] < 0:
        a = -a
    face = p - a * T.L / 2                       # the robot-side end face
    stand = face - a * 0.03
    yaw = math.atan2(a[1], a[0])
    hand_off = a * (C.ROD_X0 + C.ROD_LEN)
    tcp_stand = stand - hand_off - np.array([0, 0, C.ROD_DZ])
    travel(c, game, tcp_stand[:2], yaw)
    c.pusher(True)
    c.move("rod_tip", stand, yaw, speed=0.2)
    c.move("rod_tip", face - a * 0.003, yaw, speed=0.05)
    c.move("rod_tip", face + a * PROTRUDE, yaw, speed=PUSH_SPEED)
    c.move("rod_tip", stand, yaw, speed=0.05)
    lift(c, game)
    c.pusher(False)
    check(c, game, "push", exclude=(i,))
    z0 = p[2]
    # turn the protruding end toward the robot
    c.rotate_table(c.table_cmd + math.pi)
    check(c, game, "turn180", exclude=(i,))
    p2, a2 = block_axis(c, i)
    if a2[0] > 0:
        a2 = -a2                                 # a2 points from the tower toward the robot
    end = p2 + a2 * T.L / 2
    g = end - a2 * GRASP_IN
    g[2] = z0 + GRASP_UP
    hyaw = math.atan2(-a2[1], -a2[0])            # hand x toward the tower; fingertips tilt toward the tower
    Rg = C.hand_mat(hyaw, -GRASP_PITCH)
    c.grip_open(0.045)
    out0 = g + a2 * 0.015
    out1 = g + a2 * 0.045
    travel(c, game, out1[:2], hyaw)
    c.move("tcp", [out1[0], out1[1], c.site_pose("tcp")[0][2]], Rg, speed=0.1)
    prepose(c, out1, Rg)
    c.move("tcp", out1, Rg, speed=0.15)
    c.move("tcp", out0, Rg, speed=0.05)
    c.move("tcp", g, Rg, speed=0.02)
    c.grip_close()
    c.wait(0.4)
    g = relax(c, i, g, Rg, game, a2)
    total = T.L - PROTRUDE + 0.012
    done = 0.0
    while done < total - 1e-9:
        stepd = min(0.005, total - done)
        done += stepd
        g = g + a2 * stepd
        c.move("tcp", g, Rg, speed=0.02)
        g = relax(c, i, g, Rg, game, a2)
    out = g
    game.out.add(i)
    c.move("tcp", [out[0], out[1], safe_z(c, game) - 0.04], Rg, speed=0.08)
    check(c, game, "pull")
    pb, _ = c.block_pose(i)
    return np.linalg.norm(pb - c.site_pose("tcp")[0]) < 0.04


def stage_and_regrasp(c, game, i):
    """Set the block down on the staging post (hand still tilted, block along world x), regrasp lengthwise."""
    Rs = C.hand_mat(0.0, -GRASP_PITCH)          # hand x = +x world, fingertips toward +x
    c.q_nom = C.Q_HOME.copy()
    pb, _ = c.block_pose(i)
    tcp = c.site_pose("tcp")[0]
    off = pb - tcp
    dist = float(np.linalg.norm(off[:2]))
    tgt_c = np.array([C.STAGE_XY[0], C.STAGE_XY[1], C.STAGE_TOP + c.tower["blocks"][i]["h"] / 2 + 0.002])
    tcp_t = tgt_c - np.array([dist, 0.0, off[2]])
    zt = safe_z(c, game) - 0.04
    c.move("tcp", [tcp_t[0], tcp_t[1], zt], Rs, speed=0.3)
    c.move("tcp", tcp_t + np.array([0, 0, 0.03]), Rs, speed=0.15)
    c.move("tcp", tcp_t, Rs, speed=0.03)
    c.grip_open(0.045)
    c.wait(0.4)
    c.move("tcp", tcp_t + np.array([0, 0, 0.06]), Rs, speed=0.1)
    pb, Rb = c.block_pose(i)
    a = Rb[:, 0].copy()
    a[2] = 0
    a /= np.linalg.norm(a)
    yaw = wrap(math.atan2(a[1], a[0]) + math.pi / 2)
    if yaw < -1.0:
        yaw += math.pi
    if yaw > 2.2:
        yaw -= math.pi
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.06], yaw, speed=0.1)
    c.grip_open(0.08)
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.002], yaw, speed=0.03)
    c.grip_close()
    c.wait(0.4)
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.06], yaw, speed=0.05)
    pb2, _ = c.block_pose(i)
    return np.linalg.norm(pb2 - c.site_pose("tcp")[0]) < 0.01


def _oriented_lat(c, game, members, lat):
    """Flip `lat` so that slot index increases along it for these blocks (privileged slot labels)."""
    if len(members) < 2:
        return lat
    pts = np.array([c.d.xpos[c.ids[j]][:2] for j in members])
    sl = np.array([game.slot_of[j] - 1 for j in members], float)
    if np.ptp(sl) == 0:
        return lat
    proj = (pts - pts.mean(0)) @ lat[:2]
    return -lat if np.dot(proj, sl - sl.mean()) < 0 else lat


def target_slot(c, game):
    k = game.top_level()
    have = sorted(game.slot_of[j] for j in game.members(k))
    if len(have) == 3:
        k += 1
        have = []
    slot = [s for s in (1, 0, 2) if s not in have][0]
    below = game.members(k - 1)
    ztop = max(c.d.xpos[c.ids[j]][2] + c.tower["blocks"][j]["h"] / 2 for j in below)
    pts = np.array([c.d.xpos[c.ids[j]] for j in below])
    axb = block_axis(c, below[0])[1]
    lat_b = _oriented_lat(c, game, below, np.array([-axb[1], axb[0], 0.0]))
    sl = np.array([game.slot_of[j] - 1 for j in below], float)
    cen = pts.mean(0) - lat_b * (sl.mean() * T.PITCH)
    ax = lat_b.copy()                               # the new level runs across the level below
    lat = np.array([-ax[1], ax[0], 0.0])
    lat = _oriented_lat(c, game, game.members(k), lat) if game.members(k) else _oriented_lat(c, game, game.members(k - 2), lat)
    pos = cen + lat * (slot - 1) * T.PITCH
    return k, slot, pos, math.atan2(ax[1], ax[0]), ztop


def place(c, game, i):
    k, slot, pos, byaw, ztop = target_slot(c, game)
    h = c.tower["blocks"][i]["h"]
    hyaw = wrap(byaw + math.pi / 2)
    if hyaw < -1.0:
        hyaw += math.pi
    if hyaw > 2.2:
        hyaw -= math.pi
    tgt = np.array([pos[0], pos[1], ztop + h / 2 + 0.0015 + 2 * T.MARGIN])
    travel(c, game, tgt[:2], hyaw, dz=-0.04)       # carrying lengthwise above the top: 5 cm over the top centres
    pb, _ = c.block_pose(i)
    off = pb - c.site_pose("tcp")[0]
    c.move("tcp", tgt - off + np.array([0, 0, 0.03]), hyaw, speed=0.08)
    c.move("tcp", tgt - off, hyaw, speed=0.02)
    c.grip_open(0.08)
    c.wait(0.5)
    c.move("tcp", tgt - off + np.array([0, 0, 0.05]), hyaw, speed=0.05)
    game.level_of[i] = k
    game.slot_of[i] = slot
    game.out.discard(i)
    game.placed.append((i, k, slot))
    c.wait(1.0)
    game.ref[i] = table_frame(c, c.d.xpos[c.ids[i]])
    pb, _ = c.block_pose(i)
    return k, slot, float(np.linalg.norm(pb[:2] - pos[:2]))


def run(seed, max_blocks=60, t_budget=3600.0, tower_kw=None, verbose=True):
    tw = T.gen_tower(seed, **(tower_kw or {}))
    c = C.Cell(tw, PANDA)
    game = Game(c)
    t0 = time.time()
    hist = []
    err = None
    try:
        while game.relocated < max_blocks and c.t < t_budget:
            loads = top_loads(c)
            lmax = game.legal_max_level()
            cands = [i for i in range(game.n) if i not in game.out and game.level_of[i] <= lmax
                     and loads[i] < LOOSE_N and gap_above(c, game, i) > MIN_GAP and removal_ok(game, i)]
            if not cands:
                break
            cands.sort(key=lambda i: (-game.level_of[i], game.slot_of[i] != 1))
            i = cands[0]
            lvl0, ts = game.level_of[i], c.t
            if not extract(c, game, i):
                log("block %d not held after extraction" % i)
                break
            if not stage_and_regrasp(c, game, i):
                log("regrasp failed for %d" % i)
                break
            k, slot, perr = place(c, game, i)
            lift(c, game)
            w = check(c, game, "place")
            game.relocated += 1
            hist.append(dict(block=i, from_level=lvl0, to=[k, slot], sim_s=round(c.t - ts, 1), place_err_mm=round(perr * 1000, 2),
                             disturb_mm=round(w * 1000, 2)))
            if verbose:
                log(json.dumps(hist[-1]), "wall %.0f s sim %.0f s" % (time.time() - t0, c.t))
    except Collapse as e:
        err = str(e)
        log("COLLAPSE", err)
    except C.MoveFailed as e:
        err = "movefailed: " + str(e)
        log(err)
    return dict(seed=seed, relocated=game.relocated, collapsed=game.collapsed, error=err, sim_s=round(c.t, 1),
                max_shift_mm=round(getattr(game, "max_shift", 0.0) * 1000, 1),
                wall_s=round(time.time() - t0, 1), hist=hist)


if __name__ == "__main__":
    seed = int(sys.argv[1])
    mb = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    r = run(seed, max_blocks=mb)
    print(json.dumps(r))
