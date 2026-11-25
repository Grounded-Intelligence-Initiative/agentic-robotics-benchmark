"""Phase-A non-privileged baselines (sanity proxies). Same motion recipe as the oracle, but every block pose comes
from the NOMINAL layout + commanded turntable angle (no block-pose readout, no loads).

  B1 'blind':  push every legal block in a fixed order (top legal level down, middle first), position-controlled,
               no force check.
  B2 'probe':  same order, but watch the wrist F/T along the push: abort the push (retract, skip the block) when the
               push force rises more than F_ABORT above its pre-contact value; stop when no candidate is left.
The grader-side collapse check (table-frame displacement > 10 mm) is the oracle's.
"""
import json
import math
import sys
import time

import numpy as np

import cell as C
import oracle as O
import tower as T

F_ABORT = float(__import__("os").environ.get("F_ABORT", "1.0"))


class Nominal(object):
    """Where the baseline believes each block is (table frame), from the published nominal layout."""

    def __init__(self, c, game):
        self.c = c
        self.game = game
        self.pose = {}
        for i, b in enumerate(c.tower["blocks"]):
            self.pose[i] = self.slot_pose(b["level"], b["slot"])
        self.held = None

    @staticmethod
    def slot_pose(k, s):
        yaw = T.level_yaw(k)
        lat = np.array([-math.sin(yaw), math.cos(yaw)])
        xy = lat * (s - 1) * T.PITCH
        z = T.PED_TOP + k * T.H + T.H / 2 + 2 * T.MARGIN * (k + 1)
        return np.array([xy[0], xy[1], z]), yaw

    def world(self, i):
        (pt, yaw) = self.pose[i]
        th = self.c.table_cmd
        ct, st = math.cos(th), math.sin(th)
        x = T.TOWER_XY[0] + ct * pt[0] - st * pt[1]
        y = T.TOWER_XY[1] + st * pt[0] + ct * pt[1]
        return np.array([x, y, pt[2]]), yaw + th

    def push_out(self, i, d):
        """The block was pushed `d` along +x (world) at the current table angle: shift its table-frame pose."""
        (pt, yaw) = self.pose[i]
        th = self.c.table_cmd
        dx = np.array([math.cos(-th) * d, math.sin(-th) * d, 0.0])
        self.pose[i] = (pt + dx, yaw)


FT_RELAX = bool(int(__import__("os").environ.get("BASE_FT_RELAX", "0")))
FT_NOISE = 0.02          # N, per reading (Gaussian); 10 readings are averaged
FT_BAND = 0.25           # N dead band (the block's own weight, 0.15-0.18 N, moves onto the grip as it comes out)
_rng = np.random.default_rng(0)


def ft_z(c, n=10):
    v = []
    for _ in range(n):
        c.tick()
        v.append(float(c.wrench()[2]) + _rng.normal(0.0, FT_NOISE))
    return float(np.mean(v))


def ft_relax(c, g, R, fz0, weight_ok=False, step=2e-5, n=25):
    for _ in range(n):
        # sensor convention (measured): an upward push on the hand reads as -z
        dfz = ft_z(c) - fz0
        hi = FT_BAND + (0.18 if weight_ok else 0.0)    # the block hanging on the grip reads as up to +0.18 N
        if -FT_BAND <= dfz <= hi:
            return g
        g = g + np.array([0, 0, step if dfz < 0 else -step])
        c.move("tcp", g, R, speed=0.005)
    return g


NOM = {}
TRUE_POSE = bool(int(__import__("os").environ.get("BASE_TRUE_POSE", "0")))   # B2+: perfect localization
_true_axis = O.block_axis


def nominal_block_axis(c, i):
    if TRUE_POSE:
        return _true_axis(c, i)
    p, yaw = NOM["n"].world(i)
    return p, np.array([math.cos(yaw), math.sin(yaw), 0.0])


def push_force(c, a):
    return float(np.dot(c.wrench(), a))


def run(seed, mode, max_blocks=60, t_budget=3600.0):
    tw = T.gen_tower(seed)
    c = C.Cell(tw, O.PANDA)
    game = O.Game(c)
    nom = Nominal(c, game)
    NOM["n"] = nom
    O.block_axis = nominal_block_axis               # no privileged poses from here on
    t0 = time.time()
    tried = set()
    hist = []
    err = None
    probes = 0
    try:
        while game.relocated < max_blocks and c.t < t_budget:
            lmax = game.legal_max_level()
            cands = [i for i in range(game.n) if i not in game.out and i not in tried and game.level_of[i] <= lmax
                     and O.removal_ok(game, i)]
            if not cands:
                break
            cands.sort(key=lambda i: (-game.level_of[i], game.slot_of[i] != 1))
            i = cands[0]
            tried.add(i)
            lvl0, ts = game.level_of[i], c.t
            ok = extract_nominal(c, game, nom, i, mode)
            probes += 1
            if ok == "tight":
                O.check(c, game, "probe", exclude=(i,))
                continue
            if not ok:
                O.check(c, game, "extract")
                hist.append(dict(block=i, result="not_held"))
                continue
            if not (O.stage_and_regrasp(c, game, i) if TRUE_POSE else stage_regrasp_nominal(c, game, nom, i)):
                hist.append(dict(block=i, result="regrasp_fail"))
                continue
            if TRUE_POSE:
                k, slot, _e = O.place(c, game, i)
            else:
                k, slot = place_nominal(c, game, nom, i)
            O.lift(c, game)
            w = O.check(c, game, "place")
            game.relocated += 1
            hist.append(dict(block=i, from_level=lvl0, to=[k, slot], sim_s=round(c.t - ts, 1), disturb_mm=round(w * 1000, 2)))
            O.log("[%s]" % mode, json.dumps(hist[-1]), "wall %.0f sim %.0f" % (time.time() - t0, c.t))
    except O.Collapse as e:
        err = str(e)
        O.log("[%s] COLLAPSE" % mode, err)
    except C.MoveFailed as e:
        err = "movefailed: " + str(e)
        O.log(err)
    return dict(seed=seed, mode=mode, f_abort=F_ABORT, true_pose=TRUE_POSE, ft_relax=FT_RELAX, relocated=game.relocated, collapsed=game.collapsed, error=err, probes=probes,
                max_shift_mm=round(getattr(game, "max_shift", 0.0) * 1000, 1),
                sim_s=round(c.t, 1), wall_s=round(time.time() - t0, 1), hist=hist)


def extract_nominal(c, game, nom, i, mode):
    O.make_radial(c, i)
    O.check(c, game, "turn")
    p, a = nominal_block_axis(c, i)
    if a[0] < 0:
        a = -a
    face = p - a * T.L / 2
    stand = face - a * 0.03
    yaw = math.atan2(a[1], a[0])
    tcp_stand = stand - a * (C.ROD_X0 + C.ROD_LEN) - np.array([0, 0, C.ROD_DZ])
    O.travel(c, game, tcp_stand[:2], yaw)
    c.pusher(True)
    c.move("rod_tip", stand, yaw, speed=0.2)
    c.move("rod_tip", face - a * 0.006, yaw, speed=0.05)
    c.wait(0.3)
    f0 = push_force(c, a)
    stop = None
    if mode == "probe":
        t_start = c.t

        def stop(cc):
            # ignore the first 0.15 s (the hand's own start-up acceleration shows on the sensor)
            return cc.t - t_start > 0.15 and abs(push_force(cc, a) - f0) > F_ABORT
    r = c.move("rod_tip", face + a * O.PROTRUDE, yaw, speed=O.PUSH_SPEED, stop=stop)
    c.move("rod_tip", stand, yaw, speed=0.05)
    O.lift(c, game)
    c.pusher(False)
    if r == "stopped":
        return "tight"
    nom.push_out(i, O.PROTRUDE)
    O.check(c, game, "push", exclude=(i,))
    z0 = p[2]
    c.rotate_table(c.table_cmd + math.pi)
    O.check(c, game, "turn180", exclude=(i,))
    p2, a2 = nominal_block_axis(c, i)
    if a2[0] > 0:
        a2 = -a2
    end = p2 + a2 * T.L / 2
    g = end - a2 * O.GRASP_IN
    g[2] = z0
    hyaw = math.atan2(-a2[1], -a2[0])
    Rg = C.hand_mat(hyaw, -O.GRASP_PITCH)
    c.grip_open(0.045)
    out0 = g + a2 * 0.015
    out1 = g + a2 * 0.045
    O.travel(c, game, out1[:2], hyaw)
    c.move("tcp", [out1[0], out1[1], c.site_pose("tcp")[0][2]], Rg, speed=0.1)
    O.prepose(c, out1, Rg)
    c.move("tcp", out1, Rg, speed=0.15)
    c.move("tcp", out0, Rg, speed=0.05)
    c.move("tcp", g, Rg, speed=0.02)
    if FT_RELAX:
        c.wait(0.3)
        fz0 = ft_z(c)                                  # tare with the fingers open around the block
    c.grip_close()
    c.wait(0.4)
    held = c.opening() > 0.015
    out = g + a2 * (T.L - O.PROTRUDE + 0.012)
    if FT_RELAX:
        # non-privileged: only the wrist F/T. A grip too low levers the block's inner end into the level above,
        # which pushes the hand UP; a grip too high presses the block up at the edge, pushing the hand DOWN.
        # Nudge 20 um against the sign of the vertical force change, pull in 5 mm steps.
        g = ft_relax(c, g, Rg, fz0)
        total = T.L - O.PROTRUDE + 0.012
        done = 0.0
        while done < total - 1e-9:
            st = min(0.005, total - done)
            done += st
            g = g + a2 * st
            c.move("tcp", g, Rg, speed=0.02)
            g = ft_relax(c, g, Rg, fz0, weight_ok=done > 0.01)
        out = g
    else:
        c.move("tcp", out, Rg, speed=0.03)
    game.out.add(i)
    c.move("tcp", [out[0], out[1], O.safe_z(c, game) - 0.04], Rg, speed=0.08)
    O.check(c, game, "pull")
    return held and c.opening() > 0.015


def stage_regrasp_nominal(c, game, nom, i):
    """As the oracle's, but the block's pose on the post is assumed (where it was put), not read."""
    Rs = C.hand_mat(0.0, -O.GRASP_PITCH)
    c.q_nom = C.Q_HOME.copy()
    dist = T.L / 2 - O.GRASP_IN
    tgt_c = np.array([C.STAGE_XY[0], C.STAGE_XY[1], C.STAGE_TOP + T.H / 2 + 0.002])
    tcp_t = tgt_c - np.array([dist, 0.0, 0.0])
    zt = O.safe_z(c, game) - 0.04
    c.move("tcp", [tcp_t[0], tcp_t[1], zt], Rs, speed=0.3)
    c.move("tcp", tcp_t + np.array([0, 0, 0.03]), Rs, speed=0.15)
    c.move("tcp", tcp_t, Rs, speed=0.03)
    c.grip_open(0.045)
    c.wait(0.4)
    c.move("tcp", tcp_t + np.array([0, 0, 0.06]), Rs, speed=0.1)
    pb = tgt_c - np.array([0, 0, 0.002])
    yaw = math.pi / 2
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.06], yaw, speed=0.1)
    c.grip_open(0.08)
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.002], yaw, speed=0.03)
    c.grip_close()
    c.wait(0.4)
    c.move("tcp", [pb[0], pb[1], pb[2] + 0.06], yaw, speed=0.05)
    return c.opening() > 0.06


def place_nominal(c, game, nom, i):
    k = game.top_level()
    have = sorted(game.slot_of[j] for j in game.members(k))
    if len(have) == 3:
        k += 1
        have = []
    slot = [s for s in (1, 0, 2) if s not in have][0]
    nom.pose[i] = Nominal.slot_pose(k, slot)
    pos, byaw = nom.world(i)
    hyaw = O.wrap(byaw + math.pi / 2)
    if hyaw < -1.0:
        hyaw += math.pi
    if hyaw > 2.2:
        hyaw -= math.pi
    tgt = pos + np.array([0, 0, 0.0015])
    O.travel(c, game, tgt[:2], hyaw, dz=-0.04)
    c.move("tcp", tgt + np.array([0, 0, 0.03]), hyaw, speed=0.08)
    c.move("tcp", tgt, hyaw, speed=0.02)
    c.grip_open(0.08)
    c.wait(0.5)
    c.move("tcp", tgt + np.array([0, 0, 0.05]), hyaw, speed=0.05)
    game.level_of[i] = k
    game.slot_of[i] = slot
    game.out.discard(i)
    game.placed.append((i, k, slot))
    c.wait(1.0)
    game.ref[i] = O.table_frame(c, c.d.xpos[c.ids[i]])        # grader bookkeeping only
    return k, slot


if __name__ == "__main__":
    seed = int(sys.argv[1])
    mode = sys.argv[2]
    mb = int(sys.argv[3]) if len(sys.argv) > 3 else 60
    print(json.dumps(run(seed, mode, max_blocks=mb)))
