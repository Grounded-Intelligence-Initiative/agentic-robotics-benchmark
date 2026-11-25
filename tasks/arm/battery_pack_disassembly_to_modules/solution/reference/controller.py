"""The reference program: takes a pack apart to module level through the cell's public actions.

It is a privileged reference: the grader adds `truth` to its observations (where every part and fastener is, which
head a fastener has) because the submission carries the reference's token. With that it skips the perception a
submission has to do, and nothing else: every motion, every tool and bit exchange, every fastener and every part goes
through the same actions, rules and physics as any program's, and what it decides about a fastener while it turns
(stalled, free, out) it decides from the nutrunner's torque and angle. Without `truth` it ends the pack at once.

Structure: `Reference.act(t, obs)` answers one request. A step may be asked twice with the same `t` and slightly
different readings; the program advances once per `t` and builds each answer from the observation it came with: the
plan (a generator) yields a function of an observation, and a motion's displacement is always the difference between
a planned waypoint and that observation's commanded tool pose.
"""

import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "cell.json")) as _f:
    CELL = json.load(_f)
TICK = CELL["tick_s"]
MAXT = CELL["max_ticks_per_action"]
TZ = CELL["table_z"]
# the service manual's torque table
THREADS = {"M6": {"tighten": 10.0, "head_limit": 22.0, "turns": 7.0}, "M8": {"tighten": 23.0, "head_limit": 45.0, "turns": 9.0}}
SPRING = CELL["tools"]["nutrunner"]["spring_N_per_m"]
OPEN = CELL["tools"]["gripper"]["open_max"]
HOLD = {"move": {"type": "hold", "ticks": 1}}


def rotvec_R(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def R_rotvec(R):
    """Rotation vector (axis times angle, angle in [0, pi]) of a rotation matrix, accurate at every angle: the angle
    from atan2 of the antisymmetric part's norm and the trace; within 1e-3 rad of a half turn the axis from the
    symmetric part, signed by the antisymmetric one (acos and a division by sin lose up to 2e-3 there)."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = 0.5 * float(np.linalg.norm(w))
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    a = math.atan2(s, c)
    if a < 1e-9:
        return 0.5 * w
    if a < math.pi - 1e-3:
        return w / (2.0 * s) * a
    B = 0.5 * (R + R.T) - c * np.eye(3)              # (1 - cos a) k k^T
    i = int(np.argmax(np.diag(B)))
    k = B[:, i] / math.sqrt(max(B[i, i], 1e-300))
    if float(np.dot(k, w)) < 0.0:
        k = -k
    return k / np.linalg.norm(k) * a


def T_of(p):
    T = np.eye(4)
    T[:3, :3] = rotvec_R(p[3:6])
    T[:3, 3] = p[:3]
    return T


def mk(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def delta(goal, cur):
    return [float(v) for v in goal[:3, 3] - cur[:3, 3]] + [float(v) for v in R_rotvec(goal[:3, :3] @ cur[:3, :3].T)]


def down(c):
    """Tool axis straight down, the tool's y axis (the gripper's closing axis) along the horizontal direction c."""
    c = np.array([c[0], c[1], 0.0])
    c /= np.linalg.norm(c)
    return np.stack([[-c[1], c[0], 0.0], c, [0.0, 0.0, -1.0]], axis=1)


def split(n, far):
    """A motion that needs n ticks, as equal actions of at most MAXT ticks: (the fraction to cover in the next
    action, its ticks). No action is shorter than 4 ticks unless the motion is tiny: a short action has no room to
    start and stop smoothly, and the jerk of a sudden one overloads the arm when it carries a module."""
    k = int(math.ceil(n / float(MAXT)))
    ticks = int(math.ceil(n / float(k)))
    return 1.0 / k, max(ticks, 4 if far else 1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(R):
    """The direction of the tool's y axis in the horizontal plane (the tool pointing down)."""
    return math.atan2(R[1, 1], R[0, 1])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOCK_R = down([0.0, -1.0])       # the tool's x axis along the world's x axis, as the docks ask


class Reference(object):
    def __init__(self, log=None):
        self.log = log or (lambda *a: None)
        self.reset()

    def reset(self):
        self.gen, self.t, self.maker, self.o, self.tr = None, None, None, None, None
        self.dropped = {}

    # ------------------------------------------------------------------ the wire
    def act(self, t, obs):
        if not isinstance(obs, dict) or obs.get("truth") is None:
            return {"cmd": "done"}              # not a reference run: nothing to work from
        if t != self.t:
            self.t, self.o, self.tr = t, obs, obs["truth"]
            try:
                if self.gen is None:
                    self.gen = self.program()
                    self.maker = next(self.gen)
                else:
                    self.maker = self.gen.send(obs)
            except StopIteration:
                self.maker = lambda ob: {"cmd": "done"}
        return self.maker(obs)

    def do(self, maker):
        o = yield maker
        self.o, self.tr = o, o["truth"]
        return o

    # ------------------------------------------------------------------ motion
    def wait(self, seconds, extra=None):
        n = int(math.ceil(max(seconds, 0.0) / TICK))
        o = self.o
        while n > 0:
            k = min(n, MAXT)
            a = dict({"move": {"type": "hold", "ticks": k}}, **(extra or {}))
            o = yield from self.do(lambda ob, a=a: a)
            n -= k
        return o

    def settle(self, o):
        st = o["status"]
        if st["stopped"] or st["locked_s"] > 0:
            self.log("protective stop; waiting", st)
            yield from self.wait(st["locked_s"] + 0.3)
        elif st["busy"] or st["busy_s"] > 0:
            yield from self.wait(st["busy_s"] + 0.1)

    def move(self, goal, speed=0.45, rot_speed=1.2, guard_stroke=None, guard_force=None, extra=None):
        """Bring the commanded tool pose to `goal` on a straight line, in as many actions as it takes."""
        stops = 0
        for _ in range(40):
            cur = T_of(self.tr["tcp_target"])
            d = delta(goal, cur)
            dist, ang = float(np.linalg.norm(d[:3])), float(np.linalg.norm(d[3:]))
            if dist < 3e-4 and ang < 2e-3:
                return "ok"
            n = max(1, int(math.ceil(1.5 * dist / (speed * TICK))), int(math.ceil(1.5 * ang / (rot_speed * TICK))))
            frac, n = split(n, dist > 0.004 or ang > 0.02)
            way = mk(rotvec_R(frac * np.array(d[3:])) @ cur[:3, :3], cur[:3, 3] + frac * np.array(d[:3]))

            def maker(ob, way=way, n=n):
                m = {"type": "tcp", "delta": delta(way, T_of(ob["tcp_target"])), "ticks": n}
                if guard_stroke is not None:
                    m["guard_stroke_mm"] = guard_stroke
                if guard_force is not None:
                    m["guard_force"] = guard_force
                return dict({"move": m}, **(extra or {}))
            o = yield from self.do(maker)
            if o["status"]["guard"]:
                return "guard"
            if o["status"]["stopped"]:
                stops += 1
            yield from self.settle(o)
            if stops >= 2:
                return "stopped"
        return "fail"

    def move_joints(self, q_goal, speed=0.9, extra=None):
        q_goal = np.array(q_goal, dtype=float)
        for _ in range(40):
            cur = np.array(self.tr["q_target"])
            d = q_goal - cur
            m = float(np.max(np.abs(d)))
            if m < 1e-4:
                return "ok"
            n = max(1, int(math.ceil(1.5 * m / (speed * TICK))))
            frac, n = split(n, m > 0.01)
            way = cur + frac * d
            o = yield from self.do(lambda ob, way=way, n=n: dict({"move": {"type": "joint", "delta": [float(v) for v in way - np.array(ob["q_target"])], "ticks": n}}, **(extra or {})))
            yield from self.settle(o)
        return "fail"

    def go(self, goal, z_safe, speed=0.5, final_speed=None, extra=None, yaw="goal"):
        """Up to z_safe, round the base to the goal's direction (a turn of the first joint: a straight line that
        passes near the base's axis would run the arm through its shoulder singularity), across, down to `goal`.

        The tool points down throughout. Its turn about its own axis is the last joint's alone, and that joint
        must not wind up over a pack's many transfers: `yaw` says how free the turn is. "goal": the goal's
        orientation exactly (the docks). "symmetric": the goal's or the one half a turn from it (a parallel gripper
        on a part). "free": whatever the first joint's turn leaves (the nutrunner, a part over a bin). Of the
        turns that are allowed, the one that leaves the last joint nearest its middle is taken, together with the
        first joint's turn."""
        cur = T_of(self.tr["tcp_target"])
        z = max(z_safe, cur[2, 3])
        R_goal = goal[:3, :3]
        if np.linalg.norm(cur[:2, 3] - goal[:2, 3]) > 2e-3 or np.linalg.norm(R_rotvec(goal[:3, :3] @ cur[:3, :3].T)) > 5e-3:
            if cur[2, 3] < z_safe - 1e-3:
                yield from self.move(mk(cur[:3, :3], [cur[0, 3], cur[1, 3], z]), speed, extra=extra)
            q = np.array(self.tr["q_target"])
            da = wrap(math.atan2(goal[1, 3], goal[0, 3]) - math.atan2(cur[1, 3], cur[0, 3]))
            if abs(da) <= 0.3:
                da = 0.0
            elif abs(q[0] + da) > 3.4:                  # stay well inside the first joint's range
                da -= math.copysign(2 * math.pi, da)
            after = yaw_of(cur[:3, :3]) + da            # the first joint turns the tool with it
            if yaw == "free":
                theta, R_goal = 0.0, rot_z(da) @ cur[:3, :3]
            else:
                dpsi = wrap(yaw_of(R_goal) - after)
                steps = (-2, -1, 0, 1, 2) if yaw == "symmetric" else (-2, 0, 2)
                k = min(steps, key=lambda k: abs(q[5] - (dpsi + k * math.pi)))
                theta = -(dpsi + k * math.pi)           # the last joint turns the tool the other way round
                if k % 2:
                    R_goal = rot_z(math.pi) @ R_goal
            if abs(da) > 1e-4 or abs(theta) > 1e-4:
                yield from self.move_joints(q + [da, 0, 0, 0, 0, theta], speed=1.0, extra=extra)
            yield from self.move(mk(R_goal, [goal[0, 3], goal[1, 3], z]), speed, extra=extra)
        return (yield from self.move(mk(R_goal, goal[:3, 3]), final_speed or speed, extra=extra))

    def lift(self, z, speed=0.4, extra=None):
        cur = T_of(self.tr["tcp_target"])
        if cur[2, 3] < z:
            yield from self.move(mk(cur[:3, :3], [cur[0, 3], cur[1, 3], z]), speed, extra=extra)

    def command(self, cmd, tries=3):
        for _ in range(tries):
            o = yield from self.do(lambda ob: {"cmd": cmd})
            ok = any(e.startswith(cmd) and ": ok" in e for e in o["events"])
            yield from self.settle(o)
            if ok:
                return True
            self.log("command refused", cmd, o["events"])
            yield from self.wait(0.5)
        return False

    # ------------------------------------------------------------------ tools and bits
    def change_tool(self, to):
        """Park the mounted tool in its dock and take `to` from its dock. Returns True when `to` is mounted."""
        for _ in range(3):
            if self.o["tool"] == to:
                return True
            if self.o["tool"] != "none":
                name = self.o["tool"]
                z = CELL["tools"][name]["tcp_z"]
                dock = np.array(CELL["docks"][name])
                yield from self.go(mk(DOCK_R, dock - [0, 0, z]), dock[2] - z + 0.14, final_speed=0.12)
                yield from self.wait(0.5)
                if not (yield from self.command("tool_release")):
                    yield from self.lift(dock[2] - z + 0.14)
                    continue
            dock = np.array(CELL["docks"][to])
            yield from self.go(mk(DOCK_R, dock), dock[2] + 0.14, final_speed=0.12)
            yield from self.wait(0.5)
            if (yield from self.command("tool_lock")):
                z = CELL["tools"][to]["tcp_z"]
                yield from self.move(mk(DOCK_R, dock - [0, 0, z] + [0, 0, 0.14]), 0.3)
            else:
                yield from self.lift(dock[2] + 0.14)
        return self.o["tool"] == to

    def change_bit(self, bit):
        st = np.array(CELL["bit_station"])
        for _ in range(3):
            if self.o["bit"] == bit:
                return True
            yield from self.go(mk(DOCK_R, st), st[2] + 0.12, final_speed=0.12)
            yield from self.wait(0.5)
            yield from self.command("bit:" + bit)
            yield from self.lift(st[2] + 0.12)
        return self.o["bit"] == bit

    # ------------------------------------------------------------------ fasteners
    def unscrew(self, fid, z_safe=-0.10):
        f = self.tr["fasteners"][fid]
        th = THREADS[f["thread"]]
        top = np.array(f["top"])
        torx = f["head"].startswith("T")
        def press(limit):
            return (10.0 + 1.5 * limit + 8.0) if torx else 30.0

        limit = 1.3 * th["tighten"]
        stroke = press(limit) / SPRING
        yield from self.go(mk(DOCK_R, top + [0, 0, 0.012]), z_safe, yaw="free")
        R = T_of(self.tr["tcp_target"])[:3, :3]
        yield from self.move(mk(R, top - [0, 0, stroke]), 0.03)
        start = self.o["spindle"]["angle_deg"]
        stalls = 0
        for _ in range(40):
            o = yield from self.wait(0.5, extra={"spindle": {"speed_rpm": -240.0, "torque_limit": limit}})
            tr = o["trace"]
            turned = abs(tr["angle_deg"][-1] - tr["angle_deg"][0])
            torque = float(np.mean(tr["torque"]))
            if self.tr["captured"] == fid:
                break
            if turned < 20.0 and torque > 0.85 * limit:
                # stalled at the limit: a corroded fastener; raise the limit as far as the head allows, press harder
                stalls += 1
                limit = min(2.0 * th["tighten"], th["head_limit"] - 1.0)
                stroke = press(limit) / SPRING
                yield from self.move(mk(R, top - [0, 0, stroke]), 0.03)
                if stalls > 3:
                    self.log("fastener stays stalled", fid)
                    break
            elif torque < 0.5 and abs(tr["angle_deg"][-1] - start) > th["turns"] * 360.0 + 720.0:
                self.log("fastener turned free without coming out", fid)
                break
        ok = self.tr["captured"] == fid
        yield from self.lift(z_safe)
        return ok

    def eject(self, z_safe=-0.10):
        """Let the fastener on the bit fall into the nearer fasteners bin: the bit goes down between the bin's
        walls first (a fastener dropped from above the walls can bounce off the pile and out), and successive
        fasteners go to different spots."""
        cur = T_of(self.tr["tcp_target"])
        best = None
        for name in ("fasteners_left", "fasteners_right"):
            b = CELL["bins"][name]
            lo = np.array(b["center"]) - np.array(b["size"]) / 2 + 0.07
            hi = np.array(b["center"]) + np.array(b["size"]) / 2 - 0.07
            p = np.minimum(np.maximum(cur[:2, 3], lo), hi)
            d = float(np.linalg.norm(p - cur[:2, 3]))
            if best is None or d < best[0]:
                best = (d, p, b, name, lo, hi)
        _, p, b, name, lo, hi = best
        k = self.dropped.get(name, 0)
        self.dropped[name] = k + 1
        p = np.minimum(np.maximum(p + [0.025 * (k % 3 - 1), 0.025 * ((k // 3) % 3 - 1)], lo), hi)
        yield from self.go(mk(cur[:3, :3], [p[0], p[1], TZ + 0.07]), z_safe, speed=0.55, yaw="free")
        yield from self.command("eject", tries=1)
        yield from self.wait(0.15)
        yield from self.lift(TZ + b["wall_h"] + 0.05)

    def fasteners(self, group_filter):
        """Take out every seated fastener that `group_filter` accepts, bit by bit, nearest first."""
        todo = [fid for fid, f in self.tr["fasteners"].items() if f["status"] == "seated" and not f["stripped"] and group_filter(f)]
        heads = sorted(set(self.tr["fasteners"][fid]["head"] for fid in todo), key=lambda h: (h != self.o["bit"], h))
        for head in heads:
            ids = [fid for fid in todo if self.tr["fasteners"][fid]["head"] == head]
            if not ids:
                continue
            if not (yield from self.change_bit(head)):
                self.log("bit not mounted", head)
                continue
            while ids:
                cur = T_of(self.tr["tcp_target"])[:2, 3]
                ids.sort(key=lambda fid: float(np.linalg.norm(np.array(self.tr["fasteners"][fid]["top"][:2]) - cur)))
                fid = ids.pop(0)
                ok = yield from self.unscrew(fid)
                if not ok:
                    self.log("retrying fastener", fid)
                    ok = yield from self.unscrew(fid)
                if self.tr["captured"] is not None:
                    yield from self.eject()

    # ------------------------------------------------------------------ parts
    def close_on(self, pid, timeout=3.0):
        n = int(timeout / TICK)
        while n > 0:
            o = yield from self.wait(0.15, extra={"gripper": 0.0})
            if o["gripper"]["holding"]:
                return True
            n -= 3
        self.log("gripper did not close on", pid)
        return False

    def open_wait(self, width=OPEN, seconds=0.5):
        yield from self.wait(seconds, extra={"gripper": width})

    def grasp_pose(self, pid, which=0):
        p = self.tr["parts"][pid]
        Rp, pos = np.array(p["R"]), np.array(p["pos"])
        if p["kind"] == "harness":
            # the longest level stretch of the cable nearest to its middle
            pts = [np.array(q) for q in p["path"]]
            c = np.mean(pts, axis=0)
            segs = [(a, b) for a, b in zip(pts[:-1], pts[1:]) if abs(a[2] - b[2]) < 1e-3 and np.linalg.norm(b - a) > 0.02]
            a, b = min(segs or list(zip(pts[:-1], pts[1:])), key=lambda s: float(np.linalg.norm((s[0] + s[1]) / 2 - c)))
            seg = (b - a) / np.linalg.norm(b - a)
            return (a + b) / 2, np.cross(seg, [0.0, 0.0, 1.0]), 0.012
        g = p["grasps"][which]
        return pos + Rp @ np.array(g["pos"]), Rp @ np.array(g["close"]), g["width"]

    def pick(self, pid, z_safe, which=0, pull=False):
        gp, c, width = self.grasp_pose(pid, which)
        goal = mk(down(c), gp)
        w = min(OPEN, width + 0.03)
        if (yield from self.go(goal, z_safe, final_speed=0.15, extra={"gripper": w}, yaw="symmetric")) != "ok":
            self.log("could not reach", pid)
            yield from self.lift(z_safe)
            return False
        yield from self.wait(0.2, extra={"gripper": w})
        held = yield from self.close_on(pid)
        if not held:
            yield from self.open_wait()
            yield from self.lift(z_safe)
            return False
        if pull:                                    # a plug: pull it straight out of its latch, slowly
            R = T_of(self.tr["tcp_target"])[:3, :3]
            yield from self.move(mk(R, gp + [0, 0, 0.004]), 0.004)
            yield from self.move(mk(R, gp + [0, 0, 0.04]), 0.04)
        yield from self.lift(z_safe, speed=0.25)
        return bool(self.o["gripper"]["holding"])

    def drop_in_bin(self, name, z_safe, offset=None, R=None):
        """Let the held part fall into bin `name`. Successive parts go to different spots of the bin, so that they
        do not pile up under the gripper."""
        b = CELL["bins"][name]
        if offset is None:
            k = self.dropped.get(name, 0)
            self.dropped[name] = k + 1
            offset = (0.055 * (k % 3 - 1), 0.055 * ((k // 3) % 3 - 1))
        cur = T_of(self.tr["tcp_target"])
        goal = mk(cur[:3, :3] if R is None else R, [b["center"][0] + offset[0], b["center"][1] + offset[1], TZ + b["wall_h"] + 0.06])
        yield from self.go(goal, z_safe, speed=0.5, yaw="free" if R is None else "symmetric")
        yield from self.open_wait()
        yield from self.lift(z_safe)

    # ------------------------------------------------------------------ the job
    def program(self):
        yield from self.job()
        yield from self.do(lambda ob: {"cmd": "done"})

    def job(self):
        tr = self.tr
        self.log("pack", tr["variant"], "fasteners", len(tr["fasteners"]))
        harness = tr["parts"].get("harness")
        Z_PART, Z_MODULE, Z_LID = -0.10, 0.02, 0.17
        # 1. a harness remnant on the lid goes first (it covers lid bolts)
        if harness is not None and "lid" in [tr["fasteners"][f]["group"] for f in harness["covers"]]:
            if not (yield from self.change_tool("gripper")):
                return
            yield from self.remove_harness(Z_PART)
        # 2. the lid's bolts, then the lid to its stand
        if not (yield from self.change_tool("nutrunner")):
            return
        yield from self.fasteners(lambda f: f["group"] == "lid")
        if self.tr["parts"]["lid"]["locked"] or not (yield from self.change_tool("gripper")):
            self.log("the lid stays on")
            return
        yield from self.remove_lid(Z_LID)
        if not self.tr["parts"]["lid"]["left_seat"]:
            return
        # 3. isolate: the service disconnect out, then the low-voltage plugs while the pack discharges
        for pid in ("msd", "J1", "J2"):
            if (yield from self.pick(pid, Z_PART, pull=True)):
                yield from self.drop_in_bin("electronics", Z_PART, offset={"msd": (-0.10, -0.06), "J1": (0.10, -0.06), "J2": (0.04, -0.06)}[pid])
        while not self.tr["isolated"]:
            if self.tr["msd_out_at"] is None:
                self.log("the service disconnect is still in: stopping before the live parts")
                return
            yield from self.wait(1.0)
        if "harness" in self.tr["parts"] and not self.tr["parts"]["harness"]["left_seat"]:
            yield from self.remove_harness(Z_PART)
        # 4. busbar and bracket fasteners
        if not (yield from self.change_tool("nutrunner")):
            return
        yield from self.fasteners(lambda f: f["group"] in ("busbar", "bracket"))
        # 5. busbars, brackets, modules
        if not (yield from self.change_tool("gripper")):
            return
        for kind, where in (("busbar", "copper"), ("bracket", "steel")):
            for pid in sorted(p for p, v in self.tr["parts"].items() if v["kind"] == kind):
                # a bracket is taken by its long side: the gripper's body then clears the tray's wall
                if not self.tr["parts"][pid]["locked"] and (yield from self.pick(pid, Z_PART, which=1 if kind == "bracket" else 0)):
                    yield from self.drop_in_bin(where, Z_PART)
        bay = 0
        for pid in sorted(p for p, v in self.tr["parts"].items() if v["kind"] == "module"):
            if self.tr["parts"][pid]["locked"]:
                self.log("module still held", pid)
                continue
            if (yield from self.pick(pid, Z_MODULE)):
                yield from self.rack_module(bay, Z_MODULE)
                bay += 1

    def remove_harness(self, z_safe):
        if (yield from self.pick("harness", z_safe)):
            # lay it along the bin's long side
            yield from self.drop_in_bin("electronics", z_safe, offset=(0.0, 0.05), R=down([0.0, -1.0]))

    def remove_lid(self, z_lid):
        if not (yield from self.pick("lid", z_lid)):
            return
        q = np.array(self.tr["q_target"])
        yield from self.move_joints(q + [math.pi, 0, 0, 0, 0, 0], speed=0.7)
        cur = T_of(self.tr["tcp_target"])
        c = CELL["lid_stand"]["center"]
        yield from self.move(mk(cur[:3, :3], [c[0], c[1], z_lid]), 0.3)
        yield from self.move(mk(cur[:3, :3], [c[0], c[1], TZ + 0.004 + 0.038 + 0.004]), 0.15)
        yield from self.open_wait()
        yield from self.lift(z_lid)
        q = np.array(self.tr["q_target"])
        yield from self.move_joints(q - [math.pi, 0, 0, 0, 0, 0], speed=0.9)

    def rack_module(self, bay, z_safe):
        c = CELL["rack"]["bay_centers"][bay]
        goal = mk(down([1.0, 0.0]), [c[0], c[1], TZ + 0.055 + 0.077 + 0.003])
        yield from self.go(goal, z_safe, speed=0.35, final_speed=0.10, yaw="symmetric")
        yield from self.open_wait()
        yield from self.lift(z_safe)
