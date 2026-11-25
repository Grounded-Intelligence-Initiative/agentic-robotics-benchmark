"""A NON-privileged reference program (review evidence that the task can be done from the given sensors): no oracle
token, no privileged call, only the public calls (motion goals, gripper, pusher, turntable, the wrist force/torque
sensor) and the published nominal geometry.

  * calibration by touch: the finger tip's height offset on the staging post (known top), the tower's top height (the
    level pitch follows), the tower's centre and yaw on the turntable from the long side faces of outer blocks of two
    perpendicular levels (rod tip, guarded moves, median of three levels each);
  * per block (levels 13 down to 2, middle first): turn it radial, find its end face with the rod (guarded move),
    push it 30 mm while watching the push force (abort above 0.6 N: a bearing block; it is left where it is), turn the
    table 180 deg, touch the top of the protruding end (its height), grip it with the hand tilted 30 deg, pull it out
    in 5 mm steps while nudging the grip height against the wrist's vertical force, set it on the staging post,
    regrasp it lengthwise, place it on top in the next slot (from its own bookkeeping of the tower);
  * never leaves a level with a single block; stops after 14 blocks or at 23 levels.
"""

import math
import sys

import numpy as np

from towercell.client import CellError, hand_quat

L, W, H = 0.075, 0.025, 0.015
PITCH = 0.0265
AXIS = np.array([0.65, 0.0])
BASE_Z = 0.115
LEVELS = 18
STAGE_XY = (0.42, 0.28)
STAGE_TOP = 0.115
ROD_X0, ROD_LEN, ROD_DZ, ROD_R = 0.02, 0.12, 0.014, 0.003
PROTRUDE = 0.030
GRASP_IN = 0.014
GRASP_PITCH = math.radians(30.0)
GRASP_UP = 5e-5
HOME = [0.0, -0.5, 0.0, -2.3, 0.0, 1.8, 0.785]
TICK = 0.01
F_TOUCH = 0.08          # N: contact detected
F_TOUCH_SIDE = 0.035    # N: contact on a tower side face (calibration)
F_ABORT = 0.6           # N: a block that resists more than this while pushed is bearing
FT_BAND = 0.08
GRACE_S = 0.2
MAX_BLOCKS = 14
MAX_LEVEL = 23
LEVEL_RANGE = range(13, 1, -1)


def log(*a):
    print("[ftref]", *a, file=sys.stderr, flush=True)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def level_yaw(k):
    return 0.0 if k % 2 == 0 else math.pi / 2


class MoveFailed(Exception):
    pass


class FtRef(object):
    def __init__(self, cell):
        self.c = cell
        self.st = cell.get_state()
        self.table = 0.0
        # tower model, turntable frame
        self.cen = np.zeros(2)
        self.psi = 0.0
        self.pitch_z = H
        self.blocks = {}                 # (level, slot) -> "tower" | "out" | "tight"
        for k in range(LEVELS):
            for s in range(3):
                self.blocks[(k, s)] = "tower"
        self.placed = {}                 # (level, slot) -> table-frame centre (x, y) of the blocks we placed
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
        if (m["out_of_reach"] or not m["goal_reached"]) and m["ik_error_m"] > 1.5e-3:
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

    def tcp(self):
        return np.array(self.st["tcp"]["pos"])

    def force(self, n=10):
        f = np.zeros(3)
        for _ in range(n):
            self.st = self.c.step(n=1)
            f += np.array(self.st["wrench"]["force_N"])
        return f / n

    def guarded(self, frame, start, direction, travel, quat, speed=0.003, thresh=F_TOUCH):
        """Move `frame` from `start` along `direction` by at most `travel` until the wrist force along -direction
        rises by `thresh`: returns the frame's position at contact (None: no contact). Stops and backs off 1 mm."""
        direction = np.asarray(direction, float) / np.linalg.norm(direction)
        self.move(frame, start, quat, speed=0.05)
        self.wait(0.3)
        f0 = self.force()
        end = np.asarray(start) + direction * travel
        self.st = self.c.step(target={"pos": list(end), "quat": quat}, frame=frame, speed=speed, n=0)
        hit = None
        t_start = self.st["t"]
        recent = []
        window = 10 if thresh < 0.06 else 3              # a light touch needs a quieter (longer) average
        while not self.st["episode_over"]:
            self.st = self.c.step(n=1 if window > 3 else 2)
            if self.st["t"] - t_start < GRACE_S:      # the hand's own start-up acceleration shows on the sensor
                continue
            recent = (recent + [np.array(self.st["wrench"]["force_N"])])[-window:]
            if len(recent) < window:
                continue
            df = np.mean(recent, axis=0) - f0
            # the object pushes back against the motion; the sensor reads the force on the hand: along -direction
            # for a push, i.e. +direction in the sensor's sign convention (an upward push on the hand reads -z)
            if float(np.dot(df, direction)) > thresh:
                p = np.array(self.st["tcp"]["pos"] if frame == "tcp" else self.st["rod_tip"]["pos"])
                hit = p
                break
            if self.st["motion"]["goal_reached"]:
                break
        cur = np.array(self.st["tcp"]["pos"] if frame == "tcp" else self.st["rod_tip"]["pos"])
        self.move(frame, cur - direction * 0.001, quat, speed=0.01)
        return hit

    # ---------------------------------------------------------------- frames
    def to_world(self, p_tab, th=None):
        th = self.table if th is None else th
        return AXIS + rot(th) @ np.asarray(p_tab[:2], float)

    def to_table(self, p_w, th=None):
        th = self.table if th is None else th
        return rot(-th) @ (np.asarray(p_w[:2], float) - AXIS)

    def level_z(self, k):
        return BASE_Z + (k + 0.5) * self.pitch_z

    def block_tab(self, k, s):
        if (k, s) in self.placed:
            return self.placed[(k, s)]
        y = level_yaw(k) + self.psi
        lat = np.array([-math.sin(y), math.cos(y)])
        return self.cen + lat * (s - 1) * PITCH

    def top_level(self):
        return max(k for (k, s), v in self.blocks.items() if v in ("tower", "tight"))

    def members(self, k):
        return [s for s in range(3) if self.blocks.get((k, s)) in ("tower", "tight")]

    def legal_below(self):
        comp = sorted(k for k in range(self.top_level() + 1) if len(self.members(k)) == 3)
        return comp[-2] if len(comp) >= 2 else -1

    def safe_z(self, margin=0.09):
        return BASE_Z + (self.top_level() + 1) * self.pitch_z + margin - 0.0075

    def lift(self, dz=0.0):
        p = self.tcp()
        q = self.st["tcp"]["quat"]
        if np.linalg.norm(p[:2] - AXIS) < 0.08:
            dz = min(dz, -0.04)
        z = self.safe_z() + dz
        if p[2] < z - 0.002:
            self.move("tcp", [p[0], p[1], z], q, speed=0.2)

    def travel(self, xy, quat, dz=0.0):
        self.lift(dz)
        if np.linalg.norm(np.asarray(xy[:2]) - AXIS) < 0.08:
            dz = min(dz, -0.04)
        self.move("tcp", [xy[0], xy[1], self.safe_z() + dz], quat, speed=0.3)

    def prepose(self, low_pos, quat):
        cur = list(self.st["q"])
        best = None
        for seed in (cur, HOME):
            r = self.c.ik(low_pos, quat, seed=seed)
            if r["pos_error_m"] > 5e-4 or r["rot_error_rad"] > 5e-3:
                continue
            r2 = self.c.ik(list(self.tcp()), quat, seed=r["q"])
            if r2["pos_error_m"] > 5e-4 or r2["rot_error_rad"] > 5e-3:
                continue
            jump = max(abs(a - b) for a, b in zip(r2["q"], cur))
            if jump <= 0.6 and (best is None or jump < best[0]):
                best = (jump, r["q"], r2["q"])
        if best is None:
            return
        self.pose_q(best[2], 0.8)
        self.st = self.c.step(posture=best[1], n=0)

    # ---------------------------------------------------------------- calibration
    def calibrate(self):
        qd = hand_quat(0.0)
        self.st = self.c.step(grip_close=10.0, n=30)
        # finger tip height offset: touch the staging post's top (z = 0.115)
        self.move("tcp", [STAGE_XY[0], STAGE_XY[1], 0.20], qd, speed=0.3)
        hit = self.guarded("tcp", [STAGE_XY[0], STAGE_XY[1], STAGE_TOP + 0.020], [0, 0, -1], 0.02, qd, speed=0.004)
        self.tip_off = hit[2] - STAGE_TOP
        self.move("tcp", [STAGE_XY[0], STAGE_XY[1], 0.25], qd, speed=0.2)
        qt = hand_quat(0.0, -GRASP_PITCH)
        self.move("tcp", [STAGE_XY[0] - 0.005, STAGE_XY[1], 0.25], qt, speed=0.2)
        hit = self.guarded("tcp", [STAGE_XY[0] - 0.005, STAGE_XY[1], STAGE_TOP + 0.030], [0, 0, -1], 0.03, qt,
                           speed=0.004)
        self.tip_off_t = hit[2] - STAGE_TOP
        self.move("tcp", [STAGE_XY[0], STAGE_XY[1], 0.30], qd, speed=0.2)
        # the tower's top (level 17's middle block, at the nominal centre)
        zt_nom = BASE_Z + LEVELS * H
        self.move("tcp", [AXIS[0], AXIS[1], zt_nom + 0.06], qd, speed=0.3)
        hit = self.guarded("tcp", [AXIS[0], AXIS[1], zt_nom + 0.012], [0, 0, -1], 0.025, qd, speed=0.004)
        ztop = hit[2] - self.tip_off
        self.pitch_z = (ztop - BASE_Z) / LEVELS
        self.move("tcp", [AXIS[0], AXIS[1], ztop + 0.08], qd, speed=0.2)
        log("tip offset %.2f mm, top %.4f, level pitch %.4f mm" % (self.tip_off * 1000, ztop, self.pitch_z * 1000))
        self.st = self.c.step(grip_open=0.08, n=30)
        # side faces: at table angle 0 the odd levels' slot-2 blocks have a long face toward the robot (table x =
        # -39 mm); at angle -pi/2 the even levels' slot-0 blocks do (table y = -39 mm)
        xs, ys = [], []
        yaws = []
        for th, levels in ((0.0, (11, 13, 15)), (-math.pi / 2, (10, 12, 14))):
            self.turn(th)
            for k in levels:
                pts = []
                for off in (-0.022, 0.022):
                    z = self.level_z(k)
                    # rod tip along +x (world) toward the face, the hand behind it
                    start = np.array([AXIS[0] - 0.039 - 0.007, AXIS[1] + off, z])
                    self.travel((start - np.array([ROD_X0 + ROD_LEN, 0, 0]))[:2], hand_quat(0.0))
                    self.st = self.c.step(pusher=True, n=0)
                    # a loose outer block slides at about 0.05 N: touch it slowly and lightly
                    hit = self.guarded("rod_tip", start, [1, 0, 0], 0.016, hand_quat(0.0), speed=0.001,
                                       thresh=F_TOUCH_SIDE)
                    self.move("rod_tip", start, hand_quat(0.0), speed=0.05)
                    if hit is not None:
                        pts.append(np.array([hit[0], AXIS[1] + off]))   # rod_tip is the rod's surface tip
                self.lift()
                self.st = self.c.step(pusher=False, n=0)
                if len(pts) == 2:
                    yaws.append(math.atan2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) * (-1.0))
                    face_x = 0.5 * (pts[0][0] + pts[1][0])
                    # world x of the face -> table frame coordinate of the tower centre along the probed direction
                    c_w = face_x + 0.039
                    if th == 0.0:
                        xs.append(c_w - AXIS[0])        # table x
                    else:
                        ys.append(c_w - AXIS[0])        # table y (table +y faces world +x at angle -pi/2)
        self.cen = np.array([float(np.median(xs)) if xs else 0.0, float(np.median(ys)) if ys else 0.0])
        self.psi = float(np.median(yaws)) if yaws else 0.0
        log("tower centre (table) %s mm, yaw %.2f deg" % (np.round(self.cen * 1000, 2), math.degrees(self.psi)))
        self.turn(0.0)

    # ---------------------------------------------------------------- choice
    def candidates(self):
        lb = self.legal_below()
        out = []
        for k in LEVEL_RANGE:
            if k >= lb:
                continue
            for s in (1, 0, 2):
                if self.blocks.get((k, s)) != "tower":
                    continue
                rest = sorted(x for x in self.members(k) if x != s)
                if rest not in ([0, 2], [0, 1], [1, 2], [0, 1, 2]):
                    continue
                out.append((k, s))
        return out

    # ---------------------------------------------------------------- one block
    def ft_relax(self, g, quat, fz0, hang=0.0, step=2e-5, n=30):
        """Keep the vertical wrist force at its tare: a block pressed against the level above pushes the hand up
        (grip too low: inner end levered up) or down (grip too high: pressed up at the tower's edge). `hang` is the
        share of the block's weight (0.16 N) already on the grip, which reads as a downward force."""
        for _ in range(n):
            dfz = float(self.force()[2]) - fz0 - 0.16 * hang
            if abs(dfz) <= FT_BAND:
                return g
            g = g + np.array([0, 0, step if dfz < 0 else -step])
            self.move("tcp", g, quat, speed=0.005)
        return g

    def extract(self, k, s):
        # radial: the block's axis along world x
        yaw_tab = level_yaw(k) + self.psi
        d = wrap(-(yaw_tab + self.table))
        if d > math.pi / 2:
            d -= math.pi
        if d < -math.pi / 2:
            d += math.pi
        self.turn(self.table + d)
        a = np.array([math.cos(yaw_tab + self.table), math.sin(yaw_tab + self.table), 0.0])
        if a[0] < 0:
            a = -a
        cw = self.to_world(self.block_tab(k, s))
        z = self.level_z(k)
        center = np.array([cw[0], cw[1], z])
        face_nom = center - a * L / 2
        start = face_nom - a * 0.010
        qd = hand_quat(math.atan2(a[1], a[0]))
        self.travel((start - a * (ROD_X0 + ROD_LEN))[:2], qd)
        self.st = self.c.step(pusher=True, n=0)
        hit = self.guarded("rod_tip", start, a, 0.016, qd, speed=0.003)
        if hit is None:
            self.lift()
            self.st = self.c.step(pusher=False, n=0)
            log("no end face found for (%d, %d)" % (k, s))
            self.blocks[(k, s)] = "tight"
            return "skip"
        face = hit
        # the push, watching the force: tare in free air 0.5 mm short of the face, creep onto it at 1 mm/s (a small
        # impact), then 4 mm/s; a block that resists more than F_ABORT is bearing and is left where it is
        self.move("rod_tip", face - a * 0.0005, qd, speed=0.01)
        self.wait(0.3)
        f0 = self.force()
        tight = False
        for seg_end, v in ((face + a * 0.002, 0.001), (face + a * PROTRUDE, 0.004)):
            self.st = self.c.step(target={"pos": list(seg_end), "quat": qd}, frame="rod_tip", speed=v, n=0)
            t_start = self.st["t"]
            while not self.st["episode_over"]:
                self.st = self.c.step(n=3)
                if self.st["t"] - t_start < GRACE_S:
                    continue
                if float(np.dot(np.array(self.st["wrench"]["force_N"]) - f0, a)) > F_ABORT:
                    tight = True
                    break
                if self.st["motion"]["goal_reached"]:
                    break
            if tight:
                break
        cur = np.array(self.st["rod_tip"]["pos"])
        self.move("rod_tip", cur - a * 0.002, qd, speed=0.01)
        self.move("rod_tip", face - a * 0.030, qd, speed=0.05)
        self.lift()
        self.st = self.c.step(pusher=False, n=0)
        pushed = float(np.dot(cur - face, a))
        if tight:
            log("(%d, %d) bearing (aborted after %.1f mm)" % (k, s, pushed * 1000))
            self.blocks[(k, s)] = "tight"
            return "tight"
        # the protruding end, after the 180 deg turn, in the turntable frame
        back_w = face + a * (pushed + L)
        back_tab = self.to_table(back_w)
        a_tab = rot(-self.table) @ a[:2]
        self.turn(self.table + math.pi)
        end_w = np.append(self.to_world(back_tab), z)
        a2 = np.append(rot(self.table) @ a_tab, 0.0)            # from the tower toward the robot now
        # the block's height: touch the top of its protruding end with the closed fingers, the hand tilted as for
        # the grip (the lowest finger corner is about 5 mm robot-side of the tcp: aim it 8 mm in from the end)
        hyaw = math.atan2(-a2[1], -a2[0])
        qg = hand_quat(hyaw, -GRASP_PITCH)
        self.st = self.c.step(grip_close=10.0, n=30)
        p_touch = end_w - a2 * 0.0126
        self.travel(p_touch[:2], hand_quat(hyaw))
        self.move("tcp", [p_touch[0], p_touch[1], self.tcp()[2]], qg, speed=0.1)
        top_nom = z + 0.0075
        hit = self.guarded("tcp", [p_touch[0], p_touch[1], top_nom + self.tip_off_t + 0.008], [0, 0, -1], 0.014, qg,
                           speed=0.003)
        self.move("tcp", [p_touch[0], p_touch[1], top_nom + self.tip_off_t + 0.03], qg, speed=0.05)
        if hit is not None:
            z = hit[2] - self.tip_off_t - 0.0075
        else:
            log("(%d, %d): no top found at the protruding end" % (k, s))
        # grip the protruding end with the hand tilted
        g = end_w - a2 * GRASP_IN
        g[2] = z + GRASP_UP
        self.st = self.c.step(grip_open=0.045, n=20)
        out0 = g + a2 * 0.015
        out1 = g + a2 * 0.045
        self.travel(out1[:2], hand_quat(hyaw))
        self.move("tcp", [out1[0], out1[1], self.tcp()[2]], qg, speed=0.1)
        self.prepose(list(out1), qg)
        self.move("tcp", out1, qg, speed=0.15)
        self.move("tcp", out0, qg, speed=0.05)
        self.move("tcp", g, qg, speed=0.02)
        self.wait(0.3)
        fz0 = float(self.force()[2])
        self.st = self.c.step(grip_close=20.0, n=40)
        g = self.ft_relax(g, qg, fz0)
        total = L - PROTRUDE + 0.012
        done = 0.0
        while done < total - 1e-9:
            st = min(0.005, total - done)
            done += st
            g = g + a2 * st
            self.move("tcp", g, qg, speed=0.02)
            g = self.ft_relax(g, qg, fz0, hang=min(1.0, max(0.0, (done - 0.0075) / 0.04)))
        self.blocks[(k, s)] = "out"
        self.move("tcp", [g[0], g[1], self.safe_z() - 0.04], qg, speed=0.08)
        if self.st["gripper"]["opening_m"] < 0.015:
            log("(%d, %d): nothing in the hand after the pull" % (k, s))
            return "lost"
        return "held"

    def stage_and_regrasp(self):
        qs = hand_quat(0.0, -GRASP_PITCH)
        self.st = self.c.step(posture=HOME, n=0)
        dist = L / 2 - GRASP_IN
        tgt_c = np.array([STAGE_XY[0], STAGE_XY[1], STAGE_TOP + H / 2 + 0.002])
        tcp_t = tgt_c - np.array([dist, 0.0, 0.0])
        zt = self.safe_z() - 0.04
        self.move("tcp", [tcp_t[0], tcp_t[1], zt], qs, speed=0.3)
        self.move("tcp", tcp_t + np.array([0, 0, 0.03]), qs, speed=0.15)
        self.move("tcp", tcp_t, qs, speed=0.03)
        self.st = self.c.step(grip_open=0.045, n=40)
        self.move("tcp", tcp_t + np.array([0, 0, 0.06]), qs, speed=0.1)
        # where along x does the block lie on the post (it may have slipped in the grip by a few mm)? Touch its top
        # with the closed fingers near each end and bisect the edges: the centre is the mean of the two transitions
        q0 = hand_quat(0.0)
        self.st = self.c.step(grip_close=10.0, n=30)
        ztop = STAGE_TOP + H
        y = STAGE_XY[1]

        def on_block(x):
            self.move("tcp", [x, y, ztop + self.tip_off + 0.010], q0, speed=0.1)
            hit = self.guarded("tcp", [x, y, ztop + self.tip_off + 0.004], [0, 0, -1], 0.010, q0, speed=0.004)
            self.move("tcp", [x, y, ztop + self.tip_off + 0.010], q0, speed=0.05)
            return hit is not None

        ends = []
        for sgn in (-1.0, 1.0):
            inside, outside = tgt_c[0] + sgn * (L / 2 - 0.012), tgt_c[0] + sgn * (L / 2 + 0.012)
            for _ in range(5):
                mid = 0.5 * (inside + outside)
                if on_block(mid):
                    inside = mid
                else:
                    outside = mid
            ends.append(0.5 * (inside + outside))
        xc = 0.5 * (ends[0] + ends[1])
        self.move("tcp", [xc, y, 0.20], q0, speed=0.1)
        self.st = self.c.step(grip_open=0.08, n=30)
        qd = hand_quat(math.pi / 2)
        pb = np.array([xc, y, tgt_c[2] - 0.002])
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.06], qd, speed=0.1)
        self.st = self.c.step(grip_open=0.08, n=30)
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.002], qd, speed=0.03)
        self.st = self.c.step(grip_close=20.0, n=40)
        self.move("tcp", [pb[0], pb[1], pb[2] + 0.06], qd, speed=0.05)
        return 0.070 < self.st["gripper"]["opening_m"] < 0.080

    def place(self):
        top = self.top_level()
        k = top + 1 if len(self.members(top)) == 3 else top
        free = [s for s in (1, 0, 2) if s not in self.members(k)]
        s = free[0]
        # the new level's slot centres: the tower's centre, the slot axis = the level below's block axis
        y_below = level_yaw(k - 1) + self.psi
        ax_below = np.array([math.cos(y_below), math.sin(y_below)])
        sgn = 1.0 if k % 2 == 0 else -1.0       # keep the published slot convention: slot s at (s-1) * lat_k
        y_k = level_yaw(k) + self.psi
        lat_k = np.array([-math.sin(y_k), math.cos(y_k)])
        pos_tab = self.cen + lat_k * (s - 1) * PITCH
        del ax_below, sgn
        ztop = BASE_Z + k * self.pitch_z
        pw = self.to_world(pos_tab)
        y_w = level_yaw(k - 1) + self.psi + self.table     # the hand's x along the level below's axis
        hyaw = wrap(y_w)
        if hyaw < -1.0:
            hyaw += math.pi
        if hyaw > 2.2:
            hyaw -= math.pi
        qd = hand_quat(hyaw)
        tgt = np.array([pw[0], pw[1], ztop + H / 2 + 0.0015 + 1e-4])
        off = np.array([0.0, 0.0, -0.002])
        self.travel(tgt[:2], qd, dz=-0.04)
        self.move("tcp", tgt - off + np.array([0, 0, 0.03]), qd, speed=0.08)
        self.move("tcp", tgt - off, qd, speed=0.02)
        self.st = self.c.step(grip_open=0.08, n=50)
        self.move("tcp", tgt - off + np.array([0, 0, 0.05]), qd, speed=0.05)
        self.wait(1.0)
        self.blocks[(k, s)] = "tower"
        self.placed[(k, s)] = pos_tab
        return k, s

    def run(self):
        self.calibrate()
        while not self.st["episode_over"] and self.relocated < MAX_BLOCKS:
            if self.top_level() >= MAX_LEVEL:
                break
            cands = self.candidates()
            if not cands:
                break
            k, s = cands[0]
            t0 = self.st["t"]
            r = self.extract(k, s)
            if r in ("skip", "tight"):
                continue
            if r == "lost":
                break
            if not self.stage_and_regrasp():
                log("regrasp failed: the block stays on the post; stopping")
                self.st = self.c.step(grip_open=0.08, n=40)
                self.lift()
                break
            kk, ss = self.place()
            self.lift()
            self.relocated += 1
            log("(%d, %d) -> (%d, %d), %.0f s, total %d" % (k, s, kk, ss, self.st["t"] - t0, self.relocated))


def solve(cell):
    r = FtRef(cell)
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
