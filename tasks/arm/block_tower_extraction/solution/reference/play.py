"""The reference program (privileged): relocates loose blocks of the tower to its top until no safe block is left.

With the oracle token the cell answers `get_state_privileged` (every block's true pose, the load pressing on it from
above, its gap to the level above, the referee's level / slot bookkeeping) and `contacts_above(i)` (the contact force
between block i and the blocks above it, and where it acts). The reference uses them to choose blocks (loose: no load
and a gap of more than 60 um; never a level left with a single block), to aim every motion at the true poses, and to
keep a block it pulls clear of the level above (a grip a hair too low levers the block's inner end into that level).

Per block: turn the table so the block lies radially (along +x); push it 30 mm out at the back with the pusher rod;
turn the table 180 deg; grasp the protruding end in front with the hand tilted 30 deg; pull it out in 5 mm steps while
nudging the grip height; set it down on the staging post; regrasp it lengthwise from above; place it in the next slot
on top (middle first). Stops when no candidate is left.
"""

import math
import sys

import numpy as np

from towercell.client import CellError, hand_quat

PROTRUDE = 0.030
GRASP_IN = 0.014
GRASP_PITCH = math.radians(30.0)
GRASP_UP = 5e-5
PUSH_SPEED = 0.010
LOOSE_N = 0.02
MIN_GAP = 6e-5
L = 0.075
H = 0.015
PITCH = 0.0265
ROD_X0, ROD_LEN, ROD_DZ = 0.02, 0.12, 0.014
STAGE_XY = (0.42, 0.28)
STAGE_TOP = 0.115
AXIS_XY = np.array([0.65, 0.0])
HOME = [0.0, -0.5, 0.0, -2.3, 0.0, 1.8, 0.785]
TICK = 0.01


def log(*a):
    print("[ref]", *a, file=sys.stderr, flush=True)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MoveFailed(Exception):
    pass


class Ref(object):
    def __init__(self, cell):
        self.c = cell
        self.st = cell.get_state()
        self.table = 0.0
        self.relocated = 0

    # ---------------------------------------------------------------- motion
    def wait(self, s):
        n = int(round(s / TICK))
        while n > 0 and not self.st["episode_over"]:
            k = min(n, 1000)
            self.st = self.c.step(n=k)
            n -= k

    def move(self, frame, pos, quat, speed=0.15, rot_speed=1.0, max_t=20.0):
        pos = [float(x) for x in pos]
        cur = np.array(self.st["tcp"]["pos"] if frame == "tcp" else self.st["rod_tip"]["pos"])
        dist = float(np.linalg.norm(np.array(pos) - cur))
        first = max(20, min(1000, int(dist / max(speed, 1e-3) / TICK) + 20))
        self.st = self.c.step(target={"pos": pos, "quat": quat}, frame=frame, speed=speed, rot_speed=rot_speed, n=first)
        t0 = self.st["t"]
        while not self.st["motion"]["goal_reached"] and not self.st["episode_over"]:
            if self.st["t"] - t0 > max_t:
                break
            self.st = self.c.step(n=50)
        if self.st["episode_over"]:
            raise CellError("shift_over", "episode over")
        m = self.st["motion"]
        if m["out_of_reach"] or not m["goal_reached"]:
            if m["ik_error_m"] > 1.5e-3:
                raise MoveFailed("%s to %s: ik error %.1f mm" % (frame, np.round(pos, 3), m["ik_error_m"] * 1000))

    def pose_q(self, q, speed=0.8):
        self.st = self.c.step(q_target=list(q), joint_speed=speed, n=50)
        while not self.st["motion"]["goal_reached"] and not self.st["episode_over"]:
            self.st = self.c.step(n=50)
        self.wait(0.2)

    def turn(self, angle):
        self.table = float(angle)
        self.st = self.c.step(turntable=self.table, n=100)
        while self.st["turntable"]["moving"] and not self.st["episode_over"]:
            self.st = self.c.step(n=50)
        self.wait(0.5)

    def grip_open(self, w):
        self.st = self.c.step(grip_open=w, n=0)

    def grip_close(self, f=20.0):
        self.st = self.c.step(grip_close=f, n=0)

    def pusher(self, on):
        self.st = self.c.step(pusher=bool(on), n=0)

    def tcp(self):
        return np.array(self.st["tcp"]["pos"])

    # ---------------------------------------------------------------- truth
    def truth(self):
        self.pv = self.c.get_state_privileged()
        self.blocks = self.pv["blocks"]
        return self.pv

    def bpose(self, i):
        b = self.c.get_state_privileged()["blocks"][i]
        return np.array(b["pos"]), np.array(b["xmat"]).reshape(3, 3)

    def axis(self, i):
        p, R = self.bpose(i)
        a = R[:, 0].copy()
        a[2] = 0.0
        return p, a / np.linalg.norm(a)

    def members(self, k):
        return [b for b in self.blocks if b["state"] == "tower" and b["level"] == k]

    def top_z_all(self):
        zs = [b["pos"][2] for b in self.blocks if b["state"] == "tower"]
        return max(zs)

    def safe_z(self, margin=0.09):
        return self.top_z_all() + margin

    def lift(self, dz=0.0):
        p = self.tcp()
        q = self.st["tcp"]["quat"]
        if np.linalg.norm(p[:2] - AXIS_XY) < 0.08:
            dz = min(dz, -0.04)
        z = self.safe_z() + dz
        if p[2] < z - 0.002:
            self.move("tcp", [p[0], p[1], z], q, speed=0.2)

    def travel(self, xy, quat, dz=0.0):
        self.lift(dz)
        if np.linalg.norm(np.asarray(xy[:2]) - AXIS_XY) < 0.08:
            dz = min(dz, -0.04)
        self.move("tcp", [xy[0], xy[1], self.safe_z() + dz], quat, speed=0.3)

    def prepose(self, low_pos, quat):
        r = self.c.ik(low_pos, quat, seed=HOME)
        if r["pos_error_m"] > 5e-4 or r["rot_error_rad"] > 5e-3:
            return False
        r2 = self.c.ik(list(self.tcp()), quat, seed=r["q"])
        if r2["pos_error_m"] > 5e-4 or r2["rot_error_rad"] > 5e-3:
            return False
        self.pose_q(r2["q"], 0.8)
        self.st = self.c.step(posture=r["q"], n=0)
        return True

    # ---------------------------------------------------------------- choice
    def candidates(self):
        self.truth()
        lb = self.pv["legal_below"]
        out = []
        for b in self.blocks:
            if b["state"] != "tower" or b["level"] >= lb:
                continue
            if b["load_above_N"] >= LOOSE_N or b["gap_above_m"] <= MIN_GAP:
                continue
            rest = sorted(o["slot"] for o in self.members(b["level"]) if o["id"] != b["id"])
            if rest not in ([0, 2], [0, 1, 2], [0, 1], [1, 2]):
                continue
            out.append(b)
        out.sort(key=lambda b: (-b["level"], b["slot"] != 1))
        return out

    # ---------------------------------------------------------------- one block
    def make_radial(self, i):
        p, a = self.axis(i)
        d = wrap(-math.atan2(a[1], a[0]))
        if d > math.pi / 2:
            d -= math.pi
        if d < -math.pi / 2:
            d += math.pi
        self.turn(self.table + d)

    def contacts_ok(self, i, tgt, quat, a_out, step=2e-5, n=25):
        for _ in range(n):
            r = self.c.contacts_above(i)
            if r["force_N"] < 0.005:
                return tgt
            p = np.array(r["pos"])
            pb = np.array(r["block_pos"])
            inner = float(np.dot(p[:2] - pb[:2], a_out[:2])) < 0.0
            tgt = tgt + np.array([0, 0, step if inner else -step])
            self.move("tcp", tgt, quat, speed=0.005)
        return tgt

    def extract(self, i):
        self.make_radial(i)
        p, a = self.axis(i)
        if a[0] < 0:
            a = -a
        face = p - a * L / 2
        stand = face - a * 0.03
        yaw = math.atan2(a[1], a[0])
        qd = hand_quat(yaw)
        tcp_stand = stand - a * (ROD_X0 + ROD_LEN) - np.array([0, 0, ROD_DZ])
        self.travel(tcp_stand[:2], qd)
        self.pusher(True)
        self.move("rod_tip", stand, qd, speed=0.2)
        self.move("rod_tip", face - a * 0.003, qd, speed=0.05)
        self.move("rod_tip", face + a * PROTRUDE, qd, speed=PUSH_SPEED)
        self.move("rod_tip", stand, qd, speed=0.05)
        self.truth()
        self.lift()
        self.pusher(False)
        z0 = p[2]
        self.turn(self.table + math.pi)
        p2, a2 = self.axis(i)
        if a2[0] > 0:
            a2 = -a2
        end = p2 + a2 * L / 2
        g = end - a2 * GRASP_IN
        g[2] = z0 + GRASP_UP
        hyaw = math.atan2(-a2[1], -a2[0])
        qg = hand_quat(hyaw, -GRASP_PITCH)
        self.grip_open(0.045)
        out0 = g + a2 * 0.015
        out1 = g + a2 * 0.045
        self.travel(out1[:2], hand_quat(hyaw))
        self.move("tcp", [out1[0], out1[1], self.tcp()[2]], qg, speed=0.1)
        self.prepose(list(out1), qg)
        self.move("tcp", out1, qg, speed=0.15)
        self.move("tcp", out0, qg, speed=0.05)
        self.move("tcp", g, qg, speed=0.02)
        self.grip_close(20.0)
        self.wait(0.4)
        g = self.contacts_ok(i, g, qg, a2)
        total = L - PROTRUDE + 0.012
        done = 0.0
        while done < total - 1e-9:
            s = min(0.005, total - done)
            done += s
            g = g + a2 * s
            self.move("tcp", g, qg, speed=0.02)
            g = self.contacts_ok(i, g, qg, a2)
        self.truth()
        self.move("tcp", [g[0], g[1], self.safe_z() - 0.04], qg, speed=0.08)
        pb, _ = self.bpose(i)
        return np.linalg.norm(pb - self.tcp()) < 0.04

    def stage_and_regrasp(self, i):
        qs = hand_quat(0.0, -GRASP_PITCH)
        self.st = self.c.step(posture=HOME, n=0)
        pb, _ = self.bpose(i)
        tcp = self.tcp()
        off = pb - tcp
        dist = float(np.linalg.norm(off[:2]))
        h = next(b["h"] for b in self.blocks if b["id"] == i)
        tgt_c = np.array([STAGE_XY[0], STAGE_XY[1], STAGE_TOP + h / 2 + 0.002])
        tcp_t = tgt_c - np.array([dist, 0.0, off[2]])
        zt = self.safe_z() - 0.04
        self.move("tcp", [tcp_t[0], tcp_t[1], zt], qs, speed=0.3)
        self.move("tcp", tcp_t + np.array([0, 0, 0.03]), qs, speed=0.15)
        self.move("tcp", tcp_t, qs, speed=0.03)
        self.grip_open(0.045)
        self.wait(0.4)
        self.move("tcp", tcp_t + np.array([0, 0, 0.06]), qs, speed=0.1)
        pb, Rb = self.bpose(i)
        a = Rb[:, 0].copy()
        a[2] = 0
        a /= np.linalg.norm(a)
        yaw = wrap(math.atan2(a[1], a[0]) + math.pi / 2)
        if yaw < -1.0:
            yaw += math.pi
        if yaw > 2.2:
            yaw -= math.pi
        qd = hand_quat(yaw)
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.06], qd, speed=0.1)
        self.grip_open(0.08)
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.002], qd, speed=0.03)
        self.grip_close(20.0)
        self.wait(0.4)
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.06], qd, speed=0.05)
        pb2, _ = self.bpose(i)
        return np.linalg.norm(pb2 - self.tcp()) < 0.01

    def target_slot(self):
        self.truth()
        tower = [b for b in self.blocks if b["state"] == "tower"]
        top = max(b["level"] for b in tower)
        cur = self.members(top)
        k = top + 1 if len(cur) == 3 else top
        here = self.members(k)
        below = self.members(k - 1)
        pts = np.array([b["pos"] for b in below])
        axes = []
        for b in below:
            a = np.array(b["xmat"]).reshape(3, 3)[:, 0].copy()
            a[2] = 0
            a /= np.linalg.norm(a)
            if axes and np.dot(a, axes[0]) < 0:
                a = -a
            axes.append(a)
        axb = np.mean(axes, axis=0)
        axb /= np.linalg.norm(axb)
        cen = pts.mean(0)
        ztop = max(b["pos"][2] + b["h"] / 2 for b in below)
        cands = {s: cen + axb * s * PITCH for s in (0, -1, 1)}
        for s in (0, -1, 1):
            free = all(np.linalg.norm(np.array(b["pos"][:2]) - cands[s][:2]) > 0.010 for b in here)
            if free:
                return k, cands[s], math.atan2(axb[1], axb[0]), ztop
        raise RuntimeError("no free slot on level %d" % k)

    def place(self, i):
        k, pos, axyaw, ztop = self.target_slot()
        h = next(b["h"] for b in self.blocks if b["id"] == i)
        # the new block lies across the level below (along axb's normal); the fingers close along its length, i.e.
        # the hand's y axis along the block: hand x along axb
        hyaw = wrap(axyaw)
        if hyaw < -1.0:
            hyaw += math.pi
        if hyaw > 2.2:
            hyaw -= math.pi
        qd = hand_quat(hyaw)
        tgt = np.array([pos[0], pos[1], ztop + h / 2 + 0.0015 + 1e-4])
        self.travel(tgt[:2], qd, dz=-0.04)
        pb, _ = self.bpose(i)
        off = pb - self.tcp()
        self.move("tcp", tgt - off + np.array([0, 0, 0.03]), qd, speed=0.08)
        self.move("tcp", tgt - off, qd, speed=0.02)
        self.grip_open(0.08)
        self.wait(0.5)
        self.move("tcp", tgt - off + np.array([0, 0, 0.05]), qd, speed=0.05)
        self.wait(1.0)
        self.truth()
        b = self.blocks[i]
        return b["state"] == "tower" and b["level"] == k

    def run(self):
        while not self.st["episode_over"]:
            cands = self.candidates()
            if self.pv["collapsed"] or not cands:
                break
            b = cands[0]
            i = b["id"]
            t0 = self.st["t"]
            if not self.extract(i):
                log("block %d not held after the pull" % i)
                break
            if not self.stage_and_regrasp(i):
                log("regrasp failed for block %d" % i)
                break
            ok = self.place(i)
            self.lift()
            self.relocated += int(ok)
            log("block %d from level %d -> placed %s (level %s slot %s), %.0f s, total %d" % (
                i, b["level"], ok, self.blocks[i]["level"], self.blocks[i]["slot"], self.st["t"] - t0,
                self.relocated))
            if not ok:
                break


def solve(cell):
    r = Ref(cell)
    try:
        r.run()
    except MoveFailed as e:
        log("move failed: %s" % e)
    except CellError as e:
        if e.code not in ("shift_over", "closed"):
            raise
    try:
        cell.finish()
    except CellError:
        pass
