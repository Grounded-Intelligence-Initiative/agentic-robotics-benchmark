"""A perception-based program for the disassembly cell: takes a pack apart to module level from what the cell's
interface gives (the two RGB-D cameras with their nominal mounting and data sheets, the calibration plates, the
robot's and the tools' readings, the cell's messages) and the pack series' manual. Nothing on the wire says where a
part is; every place, the variant, the pack's placement, each fastener's position, head type, presence, and the
harness remnant come from the frames (`vision.py`), seen through this cell's own camera calibration, identified at
the start of every pack (`calib.py`), and whether a fastener turns, stalls, slips or is out comes from the
nutrunner's torque and angle.

The motion skills (`move`, `move_joints`, `go`, `lift`, `command`, `change_tool`, `change_bit`, `eject`, `pick`,
`drop_in_bin`, `remove_lid`, `rack_module`) and the order of the job are adapted from the task's privileged reference
(`solution/reference/controller.py`); every value that the reference takes from its `truth` comes here from
perception, from the manual or from this program's own bookkeeping instead.

Structure: `Baseline.act(t, obs)` answers one request. A step may be asked twice with the same `t` and slightly
different robot readings: the program advances once per `t`, and every displacement is a planned waypoint minus the
commanded pose of the observation being answered.
"""

import json
import math
import os
import time

import numpy as np

import calib as CB
import vision as V

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "cell.json")) as _f:
    CELL = json.load(_f)
with open(os.path.join(HERE, "manual.json")) as _f:
    MANUAL = json.load(_f)
TICK = CELL["tick_s"]
MAXT = CELL["max_ticks_per_action"]
TZ = CELL["table_z"]
SPRING = CELL["tools"]["nutrunner"]["spring_N_per_m"]
OPEN = CELL["tools"]["gripper"]["open_max"]
TORQUE = MANUAL["torque"]
HEADS = MANUAL["heads"]
ISOLATION_S = CELL["isolation_wait_s"]
# a whole head's outline in a wrist frame over its nominal area (hexagon across flats, pan head's circle): the
# outline's hull is a little larger for a hexagon and smaller for a pan head (measured on development frames)
HULL_OVER_AREA = {"hex": 1.10, "torx": 0.87}
# the heads' shapes (service manual, section 5): a hexagon across flats, a pan head's diameter (m)
HEAD_SHAPE = {"S10": ("hex", 0.010), "S13": ("hex", 0.013), "T30": ("torx", 0.012), "T40": ("torx", 0.016)}
# the height (of the tool's centre point) at which tool and bit exchanges cross the cell: above everything a pack
# carries (the lid's handle at -0.194 m, a harness on the lid), found when a swing toward the tool stand dragged a lid
# freed of its last bolt
Z_TRANSFER = -0.10
# overhead frames averaged per look (the depth's noise is independent from frame to frame)
N_OVERHEAD = 6
# heights in the pack frame (manual, section 3)
Z_LID_TOP, Z_BUSBAR_TOP, Z_BRACKET_TOP = 0.164, 0.136, 0.040


def rotvec_R(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def R_rotvec(R):
    """Rotation vector of a rotation matrix, accurate at every angle (as the client library's)."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = 0.5 * float(np.linalg.norm(w))
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    a = math.atan2(s, c)
    if a < 1e-9:
        return 0.5 * w
    if a < math.pi - 1e-3:
        return w / (2.0 * s) * a
    B = 0.5 * (R + R.T) - c * np.eye(3)
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
    k = int(math.ceil(n / float(MAXT)))
    ticks = int(math.ceil(n / float(k)))
    return 1.0 / k, max(ticks, 4 if far else 1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(R):
    return math.atan2(R[1, 1], R[0, 1])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOCK_R = down([0.0, -1.0])       # the tool's x axis along the world's x axis, as the docks ask


def nominal(name):
    """A camera's nominal pose (overhead: in the base frame; wrist: on the flange), from cell.json."""
    c = CELL["cameras"][name]
    T = np.eye(4)
    T[:3, :3] = c["R"]
    T[:3, 3] = c["pos"]
    return T


OVH_NOM, MOUNT_NOM = nominal("overhead"), nominal("wrist")
PLATES = CELL["calibration_plates"]
# this cell's cameras, identified at the start of every pack (`Baseline.calibrate`)
CAL = {"overhead": None, "mount": None}


def overhead_camera():
    return V.Camera(CELL["cameras"]["overhead"], CAL["overhead"] if CAL["overhead"] is not None else OVH_NOM)


def wrist_camera(pose):
    """The wrist camera of a frame whose `pose` is the nominal one (flange times the nominal mounting)."""
    flange = T_of(pose) @ np.linalg.inv(MOUNT_NOM)
    return V.Camera(CELL["cameras"]["wrist"], flange @ (CAL["mount"] if CAL["mount"] is not None else MOUNT_NOM))


def tilt_R(ax, ay):
    """A rotation by ax degrees about the base x axis, then ay about y."""
    a, b = math.radians(ax), math.radians(ay)
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    return Ry @ Rx


class GiveUp(Exception):
    pass


class Baseline(object):
    def __init__(self, log=None, debug_dir=None):
        self.log = log or (lambda *a: None)
        self.debug_dir = debug_dir
        self.reset()

    def reset(self):
        self.gen, self.t, self.maker, self.o = None, None, None, None
        self.alt = None              # the other copy of a repeated step (same t), if any
        self.prev_alt = None         # the same for the step before
        self.angle0 = 0.0
        self.repeated = set()        # the steps that were asked twice
        self.dropped = {}
        self.cmd = None              # this program's belief of the commanded tool pose (4x4) or None: read it
        self.pack = None             # what is known of the pack
        self.cpu = 0.0
        self.dbg = {"frames": [], "fasteners": {}, "notes": []}
        CAL["overhead"], CAL["mount"] = None, None
        V.WB = np.ones(3)

    # ------------------------------------------------------------------ the wire
    def act(self, t, obs):
        c0 = time.process_time()
        if t != self.t:
            self.prev_alt = self.alt if t == (self.t if self.t is not None else -2) + 1 else None
            self.t, self.o, self.alt = t, obs, None
            try:
                if self.gen is None:
                    self.gen = self.program()
                    self.maker = next(self.gen)
                else:
                    self.maker = self.gen.send(obs)
            except (StopIteration, GiveUp) as exc:
                self.note("the program ends the pack:", repr(exc))
                self.maker = lambda ob: {"cmd": "done"}
        else:
            self.alt = obs
            self.repeated.add(t)
        a = self.maker(obs)
        self.cpu += time.process_time() - c0
        return a

    def do(self, maker):
        o = yield maker
        self.o = o
        if o.get("ended"):
            raise GiveUp("pack ended: %s" % o["ended"])
        return o

    def note(self, *a):
        self.log(*a)
        self.dbg["notes"].append(" ".join(str(x) for x in a))

    # ------------------------------------------------------------------ motion
    def cur_T(self):
        """The commanded tool pose: this program's own record when it has one (it is exact after a completed
        move), else the observation's."""
        if self.cmd is not None:
            return self.cmd.copy()
        return T_of(self.o["tcp_target"])

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
            self.note("protective stop; waiting", st)
            self.cmd = None
            yield from self.wait(st["locked_s"] + 0.3)
        elif st["busy"] or st["busy_s"] > 0:
            yield from self.wait(st["busy_s"] + 0.1)

    def move(self, goal, speed=0.45, rot_speed=1.2, guard_stroke=None, guard_force=None, extra=None):
        """Bring the commanded tool pose to `goal` on a straight line, in as many actions as it takes."""
        stops = 0
        for _ in range(40):
            cur = self.cur_T()
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
            st = o["status"]
            if st["guard"] or st["stopped"] or st["clipped"]:
                self.cmd = None
            else:
                self.cmd = way
            if st["guard"]:
                return "guard"
            if st["stopped"]:
                stops += 1
            yield from self.settle(o)
            if stops >= 2:
                return "stopped"
        return "fail"

    def move_joints(self, q_goal, speed=0.9, extra=None):
        q_goal = np.array(q_goal, dtype=float)
        self.cmd = None
        for _ in range(40):
            cur = np.array(self.o["q_target"])
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

    def go(self, goal, z_safe, speed=0.5, final_speed=None, extra=None, yaw="goal", direct=0.0):
        """Up to z_safe, round the base to the goal's direction (a turn of the first joint), across, down to `goal`
        (the reference's transfer; see its docstring for the choice of the last joint's turn)."""
        cur = self.cur_T()
        z = max(z_safe, cur[2, 3])
        R_goal = goal[:3, :3]
        if direct > 0 and np.linalg.norm(cur[:2, 3] - goal[:2, 3]) < direct and np.linalg.norm(R_rotvec(goal[:3, :3] @ cur[:3, :3].T)) < 5e-3:
            return (yield from self.move(goal, final_speed or speed, extra=extra))
        if np.linalg.norm(cur[:2, 3] - goal[:2, 3]) > 2e-3 or np.linalg.norm(R_rotvec(goal[:3, :3] @ cur[:3, :3].T)) > 5e-3:
            if cur[2, 3] < z_safe - 1e-3:
                yield from self.move(mk(cur[:3, :3], [cur[0, 3], cur[1, 3], z]), speed, extra=extra)
                cur = self.cur_T()
            q = np.array(self.o["q_target"])
            da = wrap(math.atan2(goal[1, 3], goal[0, 3]) - math.atan2(cur[1, 3], cur[0, 3]))
            if abs(da) <= 0.3:
                da = 0.0
            elif abs(q[0] + da) > 3.4:
                da -= math.copysign(2 * math.pi, da)
            after = yaw_of(cur[:3, :3]) + da
            if yaw == "free":
                theta, R_goal = 0.0, rot_z(da) @ cur[:3, :3]
            else:
                dpsi = wrap(yaw_of(R_goal) - after)
                steps = (-2, -1, 0, 1, 2) if yaw == "symmetric" else (-2, 0, 2)
                k = min(steps, key=lambda k: abs(q[5] - (dpsi + k * math.pi)))
                theta = -(dpsi + k * math.pi)
                if k % 2:
                    R_goal = rot_z(math.pi) @ R_goal
            if abs(da) > 1e-4 or abs(theta) > 1e-4:
                yield from self.move_joints(q + [da, 0, 0, 0, 0, theta], speed=1.0, extra=extra)
            yield from self.move(mk(R_goal, [goal[0, 3], goal[1, 3], z]), speed, extra=extra)
        return (yield from self.move(mk(R_goal, goal[:3, 3]), final_speed or speed, extra=extra))

    def lift(self, z, speed=0.4, extra=None):
        cur = self.cur_T()
        if cur[2, 3] < z:
            yield from self.move(mk(cur[:3, :3], [cur[0, 3], cur[1, 3], z]), speed, extra=extra)

    def command(self, cmd, tries=3):
        for _ in range(tries):
            o = yield from self.do(lambda ob: {"cmd": cmd})
            ok = any(e.startswith(cmd) and ": ok" in e for e in o["events"])
            yield from self.settle(o)
            if ok:
                return True
            self.note("command refused", cmd, o["events"])
            if any("refused" in e for e in o["events"]) and cmd == "eject":
                return False
            yield from self.wait(0.5)
        return False

    # ------------------------------------------------------------------ tools and bits
    def change_tool(self, to):
        for _ in range(3):
            if self.o["tool"] == to:
                return True
            if self.o["tool"] != "none":
                name = self.o["tool"]
                z = CELL["tools"][name]["tcp_z"]
                dock = np.array(CELL["docks"][name])
                # across the cell high enough to clear the pack (the lid's handle stands 0.206 m above the table):
                # the dock's own approach height would sweep the tool through a freed lid's handle
                yield from self.go(mk(DOCK_R, dock - [0, 0, z]), max(dock[2] - z + 0.14, Z_TRANSFER), final_speed=0.12)
                yield from self.wait(0.5)
                if not (yield from self.command("tool_release")):
                    yield from self.lift(dock[2] - z + 0.14)
                    continue
                self.cmd = None
            dock = np.array(CELL["docks"][to])
            yield from self.go(mk(DOCK_R, dock), dock[2] + 0.14, final_speed=0.12)
            yield from self.wait(0.5)
            if (yield from self.command("tool_lock")):
                self.cmd = None
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
            yield from self.go(mk(DOCK_R, st), max(st[2] + 0.12, Z_TRANSFER), final_speed=0.12)
            yield from self.wait(0.5)
            yield from self.command("bit:" + bit)
            yield from self.lift(st[2] + 0.12)
        return self.o["bit"] == bit

    # ------------------------------------------------------------------ looking
    def frame_pose_ok(self, o_frame, cam):
        """The wrist frame's pose to use: the one that agrees with this program's record of the commanded pose
        (a repeated step's copy carries shifted readings), or None when that cannot be told."""
        if self.cmd is None:
            return None if self.t - 1 in self.repeated else o_frame["images"][cam]["pose"]
        tool = np.eye(4)
        tool[2, 3] = CELL["tools"].get(self.o["tool"], {}).get("tcp_z", 0.0)
        mount = np.eye(4)
        mount[:3, :3] = CELL["cameras"]["wrist"]["R"]
        mount[:3, 3] = CELL["cameras"]["wrist"]["pos"]
        want = self.cmd @ np.linalg.inv(tool) @ mount
        for cand in [o_frame] + ([self.prev_alt] if self.prev_alt is not None else []):
            im = (cand.get("images") or {}).get(cam)
            if not im:
                continue
            got = T_of(im["pose"])
            if np.linalg.norm(got[:3, 3] - want[:3, 3]) < 0.0015 and np.linalg.norm(R_rotvec(got[:3, :3] @ want[:3, :3].T)) < 0.004:
                return im["pose"]
        return None

    def look_overhead(self, tag, n=None):
        """Overhead frames with the arm out of the camera's view (up, then the first joint turned so the arm points
        away from the fixture). Returns (rgb, depth): the first frame's colour and the mean of the `n` frames' valid
        depth readings (0 where none had one)."""
        n = N_OVERHEAD if n is None else n
        yield from self.lift(-0.05)
        cur = self.cur_T()
        ang = math.atan2(cur[1, 3], cur[0, 3])
        target = 2.0 if ang >= 0 else -2.0
        if abs(wrap(target - ang)) > 0.25 and np.hypot(cur[0, 3], cur[1, 3]) > 0.05:
            q = np.array(self.o["q_target"])
            yield from self.move_joints(q + [wrap(target - ang), 0, 0, 0, 0, 0], speed=1.2)
        rgb, acc, cnt = None, None, None
        for _ in range(n + 2):
            o = yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 2}, "cams": ["overhead"]})
            im = (o.get("images") or {}).get("overhead")
            if not im:
                continue
            c, dep = V.decode(im)
            if rgb is None:
                rgb, acc, cnt = c, np.zeros(dep.shape, np.float64), np.zeros(dep.shape, np.float64)
                self.dbg["frames"].append({"tag": tag, "cam": "overhead", "t": o["time"], "n": n})
                if self.debug_dir:
                    import cv2
                    cv2.imwrite(os.path.join(self.debug_dir, "%s_overhead.png" % tag), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            acc += dep
            cnt += dep > 0
            n -= 1
            if n <= 0:
                break
        if rgb is None:
            raise GiveUp("no overhead frame")
        return rgb, np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0).astype(np.float32)

    def look_wrist(self, p, z_top, cam_dir, z_safe=-0.10, tag=""):
        """A wrist frame with the head at (p, z_top) on the camera's optical axis: tool axis down, the camera (on the
        flange's x axis) on the side `cam_dir`, the bit's tip 0.105 m above the head. Returns (camera, depth) or None."""
        c = np.array([cam_dir[0], cam_dir[1]]) / np.linalg.norm(cam_dir)
        R = down([c[1], -c[0]])                     # tool x axis along cam_dir
        goal = mk(R, [p[0], p[1], z_top + 0.105])
        for attempt in range(3):
            if (yield from self.go(goal, z_safe, yaw="goal", final_speed=0.25)) != "ok":
                return None
            yield from self.wait(0.25)
            o = yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 2}, "cams": ["wrist"]})
            yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 1}})
            if "wrist" not in (o.get("images") or {}):
                return None
            pose = self.frame_pose_ok(o, "wrist")
            if pose is None:
                self.note("wrist frame pose ambiguous, again", tag)
                continue
            rgb, dep = V.decode(o["images"]["wrist"])
            self.dbg["frames"].append({"tag": tag, "cam": "wrist", "t": o["time"]})
            return wrist_camera(pose), dep, rgb
        return None

    # ------------------------------------------------------------------ the cell's own cameras
    def calibrate(self):
        """Identify this cell's cameras: the overhead camera from both plates in an overhead frame, the wrist
        camera's place on the flange from whole-board views of a plate (the bit's tip 0.10 m above its centre, two
        yaws, square and tilted 8 degrees), the flange's pose from each frame's nominal `pose`."""
        for attempt in range(6):
            rgb, dep = yield from self.look_overhead("calib%d" % attempt, n=1)
            T, rms, found = CB.overhead(rgb, CELL["cameras"]["overhead"]["K"], OVH_NOM, PLATES)
            self.note("overhead calibration: plates %s, rms %.2f px" % (found, rms))
            if T is not None and len(found) == 2 and rms < 0.6:
                CAL["overhead"] = T
                # the hall's light: white-balance every later frame on the plates' white
                white = CB.plate_white(rgb, CELL["cameras"]["overhead"]["K"], T, PLATES)
                if white is not None and np.all(white > 20):
                    V.WB = np.clip(V.WHITE_REF / white, 0.4, 3.0)
                self.note("white balance from the plates:", np.round(V.WB, 3).tolist(), "(plate white %s)" % (None if white is None else np.round(white, 1).tolist()))
                break
        if CAL["overhead"] is None:
            raise GiveUp("the overhead camera could not be calibrated")
        self.dbg["cal_overhead"] = CAL["overhead"].tolist()
        mounts = []
        for name in ("near", "far"):
            p = PLATES[name]
            c = np.array(p["origin"]) + [p["squares"][0] * p["square"] / 2, p["squares"][1] * p["square"] / 2, 0.0]
            for yaw in (0.0, math.pi):
                for tilt in ((0, 0), (8, 0), (-8, 0), (0, 8), (0, -8)):
                    if len(mounts) >= 6:
                        break
                    goal = mk(tilt_R(*tilt) @ rot_z(yaw) @ DOCK_R, c + [0.0, 0.0, 0.10])
                    if (yield from self.go(goal, -0.05, yaw="goal", final_speed=0.25)) != "ok":
                        continue
                    yield from self.wait(0.25)
                    o = yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 2}, "cams": ["wrist"]})
                    yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 1}})
                    pose = self.frame_pose_ok(o, "wrist") if "wrist" in (o.get("images") or {}) else None
                    if pose is None:
                        continue
                    rgb, _ = V.decode(o["images"]["wrist"])
                    M, rms = CB.wrist_view(rgb, CELL["cameras"]["wrist"]["K"], T_of(pose) @ np.linalg.inv(MOUNT_NOM), MOUNT_NOM, p)
                    if M is not None and rms < 0.6:
                        mounts.append(M)
            if len(mounts) >= 4:
                break
        yield from self.lift(-0.05)
        if len(mounts) < 2:
            raise GiveUp("the wrist camera could not be calibrated (%d views)" % len(mounts))
        CAL["mount"], good = CB.robust_average(mounts)
        spread = max(CB.pose_err(M, CAL["mount"])[0] for M in good)
        self.note("wrist calibration: %d of %d views, spread %.2f mm, off nominal %.1f mm" % (
            len(good), len(mounts), 1e3 * spread, 1e3 * CB.pose_err(CAL["mount"], MOUNT_NOM)[0]))
        self.dbg["cal_mount"] = CAL["mount"].tolist()

    # ------------------------------------------------------------------ the pack, seen from above
    def pack_frame(self, dep, top_z):
        r = V.pack_pose(overhead_camera(), dep, TZ, top_z)
        if r is None:
            raise GiveUp("no pack found in the overhead frame")
        x, y, yaw, info = r
        self.note("pack at %.4f %.4f yaw %.3f deg" % (x, y, math.degrees(yaw)), info)
        return (x, y, yaw)

    def survey_lid(self, rgb, dep):
        pose = self.pack_frame(dep, Z_LID_TOP)
        view = V.PackView(overhead_camera(), rgb, dep, pose, TZ)
        scores, raised = V.rib_scores(view, MANUAL, Z_LID_TOP)
        variant = max(scores, key=scores.get)
        self.note("variant from the lid's ribs:", variant, {k: round(v, 2) for k, v in scores.items()})
        self.pack = {"pose": pose, "variant": variant, "fast": {}, "isolated_at": None, "msd_out_at": None}
        self.read_group(view, "lid")
        region = view.band(-0.30, 0.30, -0.49, 0.49) & (np.nan_to_num(view.pz, nan=0) > Z_LID_TOP + 0.003)
        self.pack["harness_lid"] = V.harness_grasp(view, V.blue_mask(view, region))
        self.note("harness on the lid:", self.pack["harness_lid"])
        return view

    def read_group(self, view, group, only_unknown=False):
        """Each fastener of the group (manual's places for the variant) as the overhead frame shows it."""
        var = MANUAL["variants"][self.pack["variant"]]
        for f in var["fasteners"]:
            g = "lid" if f["holds"] == "lid" else ("busbar" if f["holds"].startswith("B") else "bracket")
            if g != group and not (group == "inside" and g != "lid"):
                continue
            rec = self.pack["fast"].get(f["id"])
            if rec is None:
                rec = {"id": f["id"], "thread": f["thread"], "group": g, "holds": f["holds"], "nominal": (f["x"], f["y"], f["seat_z"]),
                       "state": "unknown", "kind": None, "est": None, "out": False, "tries": 0}
                self.pack["fast"][f["id"]] = rec
            if only_unknown and rec["state"] not in ("unknown", "covered"):
                continue
            r = V.head_at(view, f["x"], f["y"], f["seat_z"])
            if r[0] == "head":
                rec["state"] = "seated"
                rec["kind"] = V.head_kind(f["thread"], r[3])
                rec["est"] = [float(v) for v in view.to_base(r[1], r[2], r[4])]
                rec["est_pack"] = [r[1], r[2]]
            elif r[0] == "hole":
                rec["state"] = "missing"
            elif r[0] == "covered":
                rec["state"] = "covered"
            else:
                rec["state"] = "unknown"
            self.dbg["fasteners"].setdefault(f["id"], {})["overhead"] = {"state": rec["state"], "kind": rec["kind"], "est": rec["est"]}

    # ------------------------------------------------------------------ fasteners
    def to_base_dir(self, d):
        c, s = math.cos(self.pack["pose"][2]), math.sin(self.pack["pose"][2])
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])

    def cam_side(self, rec):
        """From which side the wrist camera looks at a fastener: for the brackets' bolts deep in the tray along the
        bracket (toward the pack's middle in y), so that neither a module nor a busbar is in the way; else from
        the side of the pack's middle."""
        x, y, _ = rec["nominal"]
        if rec["group"] == "bracket":
            return self.to_base_dir([0.0, -1.0 if y > 0 else 1.0])
        if rec["group"] == "busbar":
            return self.to_base_dir([1.0, 0.0])
        return self.to_base_dir([-1.0 if x > 0 else 1.0, 0.0]) if abs(x) > 0.2 else self.to_base_dir([0.0, -1.0 if y > 0 else 1.0])

    def seat_z(self, rec):
        return TZ + rec["nominal"][2]

    def refine(self, rec, z_safe):
        """A wrist frame over the fastener: its head's centre and height, or an empty hole."""
        est = rec["est"]
        if est is None:
            est = self.nominal_base(rec)
        side = np.asarray(self.cam_side(rec), dtype=float)
        sides = [side, -side, np.array([-side[1], side[0]])]
        h, best = None, None
        for look in range(3):
            side = sides[look]
            res = yield from self.look_wrist(est[:2], est[2], side, z_safe=z_safe, tag=rec["id"])
            if res is None:
                return False
            cam, dep, rgb = res
            if look == 0 and rec["group"] in ("lid", "busbar"):
                frac, n = V.pad_at(cam, dep, self.nominal_base(rec)[:2], self.seat_z(rec))
                if frac > 0.6:
                    self.note(rec["id"], "under a foam pad (%.2f of %d ring readings above 9 mm): by touch" % (frac, n))
                    return (yield from self.touch(rec, z_safe))
            h = V.wrist_head(cam, dep, est, self.seat_z(rec), rgb=rgb)
            if self.debug_dir:
                import cv2
                cv2.imwrite(os.path.join(self.debug_dir, "w_%s_%d_%d.png" % (rec["id"], rec["tries"], look)), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            d = self.dbg["fasteners"].setdefault(rec["id"], {})
            d.setdefault("wrist", []).append(h)
            if h is None or h["hole"]:
                break
            # a head partly hidden (by a busbar, a module's edge) shows less of its top face, and its centre moves:
            # look again from the other side
            # the head's outline fitted to its rim (sealant or a neighbour may hide part of it), else the hull
            hf = V.wrist_head_fit(cam, dep, est, self.seat_z(rec), rec["thread"])
            shape, size = HEAD_SHAPE[V.head_kind(rec["thread"], h["height"])]
            area = (math.sqrt(3) / 2 * size ** 2) if shape == "hex" else math.pi * (size / 2) ** 2
            h["whole"] = h["n"] / (area * h["px_per_m"] ** 2) / HULL_OVER_AREA[shape]
            if hf is not None:
                h.update(x=hf["x"], y=hf["y"], z=hf["z"], height=hf["height"], fit=True, cover=hf["cover"])
                h["score"], good = 1.0 + hf["cover"], hf["cover"] >= 0.45
            else:
                # no rim to fit: the hull is trusted only when its area is a bare head's (sealant makes it larger,
                # a neighbour in the way smaller)
                h["score"], good = 1.0 - abs(h["whole"] - 1.0), 0.88 <= h["whole"] <= 1.08
            d.setdefault("wrist_fit", []).append(hf)
            off = math.hypot(h["x"] - est[0], h["y"] - est[1]) if rec.get("est") else 0.0
            if best is None or h["score"] > best["score"]:
                best = h
            if good and off < 3.0e-3:
                break
            self.note(rec["id"], "partly hidden, sealed or far from the overhead estimate (fit %s, %.2f of a whole head, %.2f mm): "
                      "from another side" % ("cover %.2f" % hf["cover"] if hf else "none", h["whole"], 1000 * off))
            h = best
        if h is None:
            return False
        if h["hole"]:
            rec["state"] = "missing"
            self.note(rec["id"], "an empty hole")
            return True
        rec["state"] = "seated"
        kind = V.head_kind(rec["thread"], h["height"])
        if kind != rec["kind"]:
            self.note(rec["id"], "head seen as", kind, "from close by (from above:", rec["kind"], ")")
        rec["kind"] = kind
        rec["est"] = [h["x"], h["y"], h["z"]]
        rec["refined"] = True
        return True

    def probe(self, x, y, seat, z_safe):
        """Lower the bit straight down at (x, y) until the spindle's spring gives 1 mm; the height above `seat` the
        bit's tip rests at (the tool centre point plus the compression), or None when it met nothing above the seat."""
        R = self.cur_T()[:3, :3]
        yield from self.go(mk(R, [x, y, seat + 0.020]), z_safe, yaw="free", final_speed=0.15, direct=0.03)
        R = self.cur_T()[:3, :3]
        res = yield from self.move(mk(R, [x, y, seat - 0.004]), 0.01, guard_stroke=1.0)
        o = self.o
        tip = T_of(o["tcp"])[2, 3] + o["spindle"]["stroke_mm"] / 1000.0
        yield from self.move(mk(R, [x, y, seat + 0.020]), 0.05)
        if res != "guard":
            return None
        return tip - seat

    def touch(self, rec, z_safe):
        """A fastener under a foam pad (manual, section 7), by touch: the bit's tip at its nominal place rests on its
        head (the head's height: its type) or on the seat (missing); the head's centre is midway between the two
        places on either side, along x and along y, where the tip comes off the head's top."""
        x0, y0, _ = self.nominal_base(rec)
        seat = self.seat_z(rec)
        h0 = yield from self.probe(x0, y0, seat, z_safe)
        self.dbg["fasteners"].setdefault(rec["id"], {})["touch"] = {"h0": h0}
        if h0 is None or h0 < 0.0025:
            rec["state"] = "missing"
            self.note(rec["id"], "by touch: no head (%s)" % (None if h0 is None else "%.1f mm" % (1000 * h0)))
            return True
        c = [x0, y0]
        for axis in (0, 1):
            edges = []
            for sgn in (1.0, -1.0):
                lo, hi = 0.0035, 0.0135                   # the tip is on the top at lo, off it at hi
                for _ in range(5):
                    mid = 0.5 * (lo + hi)
                    p = list(c)
                    p[axis] += sgn * mid
                    h = yield from self.probe(p[0], p[1], seat, z_safe)
                    if h is not None and h > h0 - 0.0005:
                        lo = mid
                    else:
                        hi = mid
                edges.append(0.5 * (lo + hi))
            c[axis] += 0.5 * (edges[0] - edges[1])
        rec["state"], rec["kind"] = "seated", V.head_kind(rec["thread"], h0)
        rec["est"], rec["refined"] = [c[0], c[1], seat + h0], True
        self.note(rec["id"], "by touch: %s, %.1f mm high, %.2f mm from its nominal place" % (rec["kind"], 1000 * h0, 1000 * math.hypot(c[0] - x0, c[1] - y0)))
        self.dbg["fasteners"][rec["id"]]["touch"].update(c=c, kind=rec["kind"])
        return True

    def nominal_base(self, rec):
        x, y, s = rec["nominal"]
        c, sn = math.cos(self.pack["pose"][2]), math.sin(self.pack["pose"][2])
        X, Y, _ = self.pack["pose"]
        h = HEADS[rec["kind"]]["height"] if rec["kind"] else 0.005
        return [X + c * x - sn * y, Y + sn * x + c * y, TZ + s + h]

    def spin(self, seconds, rpm, limit):
        o = yield from self.wait(seconds, extra={"spindle": {"speed_rpm": rpm, "torque_limit": limit}})
        tr = o["trace"]
        a = [self.angle0] + list(tr["angle_deg"])
        self.angle0 = tr["angle_deg"][-1] if tr["angle_deg"] else self.angle0
        turned = abs(a[-1] - a[0])
        tq = list(tr["torque"]) or [0.0]
        return turned, float(np.max(tq)), float(np.mean(tq)), tq

    def unscrew(self, rec, z_safe=-0.10):
        """Press the bit on the head and loosen it; decide from the torque and the angle whether it drives, stalls
        (a corroded fastener: raise the limit within the thread's limit), slips (not square: stop at once) or turns
        free, and when it is out (the turns of the manual's table, then a free-running spindle)."""
        th = TORQUE[rec["thread"]]
        torx = rec["kind"].startswith("T")
        top = np.array(rec["est"])

        def press(limit):
            return (10.0 + 1.5 * limit + 8.0) if torx else 30.0
        limit = 1.3 * th["assembly"]
        stroke = press(limit) / SPRING
        cur = self.cur_T()
        if np.linalg.norm(cur[:2, 3] - top[:2]) < 0.01 and cur[2, 3] > top[2]:
            # straight down from the wrist frame's pose above the head
            yield from self.go(mk(cur[:3, :3], top + [0, 0, 0.012]), z_safe, direct=0.01, final_speed=0.25)
        else:
            yield from self.go(mk(DOCK_R, top + [0, 0, 0.012]), z_safe, yaw="free")
        R = self.cur_T()[:3, :3]
        yield from self.move(mk(R, top - [0, 0, stroke]), 0.03)
        yield from self.wait(0.1)
        self.angle0 = self.o["spindle"]["angle_deg"]
        log = []
        # a first quarter turn, slowly: does the bit drive the head?
        broke, stalls, total, cycles = False, 0, 0.0, 0
        # the highest limit that harms nothing: below the shear torque and the head's limit (manual, section 5)
        safe = min(0.975 * th["shear"], th["never_exceed"] - 1.0)
        result = "fail"
        for k in range(60):
            first = not broke
            turned, tmax, tmean, tq = yield from self.spin(0.25 if first else 0.5, -120.0 if first else -240.0, limit)
            log.append((round(turned, 1), round(tmax, 2), round(tmean, 2)))
            if first:
                if tmax >= 0.85 * limit and turned < 15.0:
                    stalls += 1
                    if limit < safe - 1e-6:
                        # stalled: a corroded fastener; raise the limit as far as is safe, press harder
                        limit = safe
                        stroke = press(limit) / SPRING
                        yield from self.move(mk(R, top - [0, 0, stroke]), 0.03)
                        continue
                    # stalled at the safe limit: seized; work it (a loosening stall at 90 % of the shear torque, a
                    # tightening stall at 110 % of the assembly torque), then try again at the safe limit
                    if cycles >= 8:
                        result = "stalled"
                        break
                    t1, _, _, _ = yield from self.spin(0.6, -120.0, 0.9 * th["shear"])
                    if t1 > 15.0:
                        broke, total = True, t1          # it gave way while being loaded
                        continue
                    yield from self.spin(0.35, 120.0, 1.1 * th["assembly"])
                    cycles += 1
                    continue
                if tmax >= 0.7 * min(limit, th["breakaway"]):
                    broke = True
                    total = turned
                    continue
                if tmean < 0.35:
                    result = "free"            # the bit is not on the head (or does not fit it): nothing happened
                    break
                result = "slip"                # on the head but not driving it: the head wears; stop
                break
            total += turned
            if total >= th["turns"] * 360.0 + 60.0 and tmean < 0.5:
                result = "out"
                break
            if total > th["turns"] * 360.0 + 1440.0:
                result = "free_running"
                break
        self.dbg["fasteners"].setdefault(rec["id"], {}).setdefault("unscrew", []).append({"result": result, "log": log, "est": rec["est"], "kind": rec["kind"]})
        self.note("unscrew", rec["id"], rec["kind"], result, "turned %.0f" % total, "worked %d" % cycles, log[:3], log[-2:])
        yield from self.lift(z_safe)
        return result

    def eject(self, z_safe=-0.10):
        """Let the fastener on the bit fall into the nearer fasteners bin (the reference's spots). Returns whether
        the cell says something was on the bit."""
        cur = self.cur_T()
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
        o = yield from self.do(lambda ob: {"cmd": "eject"})
        ok = any(e == "eject: ok" for e in o["events"])
        yield from self.wait(0.15)
        yield from self.lift(TZ + b["wall_h"] + 0.05)
        return ok

    def take_out(self, recs, z_safe=-0.10):
        """Every seated fastener of `recs`, bit by bit, nearest first: a wrist frame over it, then out."""
        todo = [r for r in recs if r["state"] in ("seated", "unknown", "covered") and not r["out"]]
        for rnd in range(3):
            if not todo:
                break
            heads = sorted(set(r["kind"] or "S10" for r in todo), key=lambda h: (h != self.o["bit"], h))
            retry = []
            for head in heads:
                ids = [r for r in todo if (r["kind"] or "S10") == head]
                if not ids:
                    continue
                if not (yield from self.change_bit(head)):
                    self.note("bit not mounted", head)
                    retry += ids
                    continue
                while ids:
                    cur = self.cur_T()[:2, 3]
                    ids.sort(key=lambda r: float(np.linalg.norm(np.array((r["est"] or self.nominal_base(r))[:2]) - cur)))
                    rec = ids.pop(0)
                    rec["tries"] += 1
                    if not rec.get("refined"):
                        ok = yield from self.refine(rec, z_safe)
                        if not ok:
                            self.note("no clear view of", rec["id"])
                    if rec["state"] == "missing":
                        continue
                    if rec["kind"] != head:
                        retry.append(rec)
                        continue
                    if rec["est"] is None:
                        retry.append(rec)
                        continue
                    res = yield from self.unscrew(rec, z_safe)
                    if res in ("out", "free_running"):
                        if (yield from self.eject(z_safe)):
                            rec["out"] = True
                            continue
                        self.note(rec["id"], "was not on the bit")
                    if res in ("free", "slip") or not rec["out"]:
                        rec["refined"] = False
                        if rec["tries"] < 3:
                            retry.append(rec)
            todo = retry
        return [r["id"] for r in recs if r["state"] == "seated" and not r["out"]]

    # ------------------------------------------------------------------ parts
    def part(self, pid):
        for p in MANUAL["variants"][self.pack["variant"]]["parts"]:
            if p["id"] == pid:
                return p
        return None

    def grasp_of(self, pid):
        """A part's grip point (base frame), closing direction (base frame) and width, from the manual (section 4)
        and the pack's placement."""
        p = self.part(pid)
        kind = p["kind"]
        if kind == "lid":
            c, w = [0.0, 1.0], 0.030
        elif kind == "module":
            c, w = [0.0, 1.0], 0.030
        elif kind == "msd":
            c, w = [0.0, 1.0], 0.020
        elif kind == "connector":
            c, w = [1.0, 0.0], 0.022
        elif kind == "busbar":
            ends = [f for f in MANUAL["variants"][self.pack["variant"]]["fasteners"] if f["holds"] == pid]
            along_y = abs(ends[0]["y"] - ends[1]["y"]) > abs(ends[0]["x"] - ends[1]["x"])
            c, w = ([1.0, 0.0] if along_y else [0.0, 1.0]), 0.026
        elif kind == "bracket":
            c, w = [0.0, 1.0], 0.060         # by its long side: the gripper's body then clears the tray's wall
        else:
            raise ValueError(kind)
        X, Y, yaw = self.pack["pose"]
        cs, sn = math.cos(yaw), math.sin(yaw)
        gp = np.array([X + cs * p["x"] - sn * p["y"], Y + sn * p["x"] + cs * p["y"], TZ + p["z"]])
        return gp, self.to_base_dir(c), w

    def close_on(self, timeout=3.0):
        n = int(timeout / TICK)
        while n > 0:
            o = yield from self.wait(0.15, extra={"gripper": 0.0})
            if o["gripper"]["holding"]:
                return True
            n -= 3
        return False

    def open_wait(self, width=OPEN, seconds=0.5):
        yield from self.wait(seconds, extra={"gripper": width})

    def pick_at(self, gp, c, width, z_safe, pull=False, name=""):
        goal = mk(down(c), gp)
        w = min(OPEN, width + 0.03)
        if (yield from self.go(goal, z_safe, final_speed=0.15, extra={"gripper": w}, yaw="symmetric")) != "ok":
            self.note("could not reach", name)
            yield from self.lift(z_safe)
            return False
        yield from self.wait(0.2, extra={"gripper": w})
        held = yield from self.close_on()
        if not held:
            cur = T_of(self.o["tcp"])
            self.note("the gripper did not close on", name, "(opening %s, tcp %.1f mm from the grip point, tool %s)" % (
                self.o["gripper"]["width"], 1000 * float(np.linalg.norm(cur[:3, 3] - np.asarray(gp))), self.o["tool"]))
            yield from self.open_wait()
            yield from self.lift(z_safe)
            return False
        stops = self.o["status"]["protective_stops"]
        if pull:                                    # a plug or the service disconnect: straight out, slowly at first
            R = self.cur_T()[:3, :3]
            yield from self.move(mk(R, gp + [0, 0, 0.004]), 0.004)
            yield from self.move(mk(R, gp + [0, 0, 0.04]), 0.04)
        r = yield from self.lift(z_safe, speed=0.25)
        if self.o["status"]["protective_stops"] > stops:
            # still held by something: let go
            self.note("protective stop lifting", name, "- letting go")
            yield from self.open_wait()
            yield from self.lift(z_safe)
            return False
        return bool(self.o["gripper"]["holding"])

    def pick(self, pid, z_safe, pull=False, tries=2):
        gp, c, w = self.grasp_of(pid)
        for _ in range(tries):
            if (yield from self.pick_at(gp, c, w, z_safe, pull=pull, name=pid)):
                return True
        return False

    def drop_in_bin(self, name, z_safe, offset=None, R=None):
        b = CELL["bins"][name]
        if offset is None:
            k = self.dropped.get(name, 0)
            self.dropped[name] = k + 1
            offset = (0.055 * (k % 3 - 1), 0.055 * ((k // 3) % 3 - 1))
        cur = self.cur_T()
        goal = mk(cur[:3, :3] if R is None else R, [b["center"][0] + offset[0], b["center"][1] + offset[1], TZ + b["wall_h"] + 0.06])
        yield from self.go(goal, z_safe, speed=0.5, yaw="free" if R is None else "symmetric")
        yield from self.open_wait()
        yield from self.lift(z_safe)

    def remove_harness(self, g, z_safe):
        """Take the harness by the grasp seen from above (pack frame), or by one of the other places along it that
        were seen, and lay it in the electronics bin."""
        X, Y, yaw = self.pack["pose"]
        cs, sn = math.cos(yaw), math.sin(yaw)
        for k, h in enumerate([g] + list(g.get("alternatives") or [])):
            gp = np.array([X + cs * h["x"] - sn * h["y"], Y + sn * h["x"] + cs * h["y"], TZ + h["z"]])
            d = self.to_base_dir(h["dir"])
            c = np.array([-d[1], d[0]])                 # across the cable
            if (yield from self.pick_at(gp, c, 0.012, z_safe, name="harness")):
                yield from self.drop_in_bin("electronics", z_safe, offset=(0.0, 0.05), R=down([0.0, -1.0]))
                return True
            self.note("the harness was not taken at grasp", k)
        return False

    def remove_lid(self, z_lid):
        if not (yield from self.pick("lid", z_lid)):
            return False
        q = np.array(self.o["q_target"])
        yield from self.move_joints(q + [math.pi, 0, 0, 0, 0, 0], speed=0.7)
        cur = self.cur_T()
        c = CELL["lid_stand"]["center"]
        yield from self.move(mk(cur[:3, :3], [c[0], c[1], z_lid]), 0.3)
        yield from self.move(mk(cur[:3, :3], [c[0], c[1], TZ + 0.004 + 0.038 + 0.004]), 0.15)
        yield from self.open_wait()
        yield from self.lift(z_lid)
        q = np.array(self.o["q_target"])
        yield from self.move_joints(q - [math.pi, 0, 0, 0, 0, 0], speed=0.9)
        return True

    def rack_module(self, bay, z_safe):
        c = CELL["rack"]["bay_centers"][bay]
        goal = mk(down([1.0, 0.0]), [c[0], c[1], TZ + 0.055 + 0.077 + 0.003])
        yield from self.go(goal, z_safe, speed=0.35, final_speed=0.10, yaw="symmetric")
        yield from self.open_wait()
        yield from self.lift(z_safe)

    def measure_modules(self, z_safe):
        """Each module's top-plate bump from two wrist frames (the camera beside it along the pack's x and y); the one
        whose bump stands the manual's criterion or more above the plate's rim is swollen (at most one a pack: the
        highest)."""
        sw = MANUAL["swollen"]
        var = MANUAL["variants"][self.pack["variant"]]
        X, Y, yaw = self.pack["pose"]
        c, s = math.cos(yaw), math.sin(yaw)
        top = TZ + 0.124
        out = {}
        for p in sorted((p for p in var["parts"] if p["kind"] == "module"), key=lambda p: p["id"]):
            px, py = X + c * p["x"] - s * p["y"], Y + s * p["x"] + c * p["y"]
            cams = []
            for d in ([1.0, 0.0], [0.0, 1.0]):
                res = yield from self.look_wrist([px, py], top, self.to_base_dir(d), z_safe=z_safe, tag=p["id"] + "_top")
                if res is not None:
                    cams.append((res[0], res[1]))
            r = V.module_bump(cams, self.pack["pose"], (p["x"], p["y"]), top, plate=sw["plate"], rim=sw["rim"], a=sw["a"],
                              b=sw["b"], dx=sw["dx"]) if cams else None
            out[p["id"]] = None if r is None else r[0]
            self.dbg.setdefault("modules", {})[p["id"]] = r
        self.note("module bumps (mm):", {k: None if v is None else round(1000 * v, 3) for k, v in out.items()})
        seen = {k: v for k, v in out.items() if v is not None and v >= sw["criterion"]}
        return set([max(seen, key=seen.get)]) if seen else set()

    # ------------------------------------------------------------------ the pack, open
    def modules_seen(self, view):
        """Which variants' module handles stand where the manual puts them (the handle bar's top, 150 mm)."""
        out = {}
        for v, d in MANUAL["variants"].items():
            hits = []
            for p in d["parts"]:
                if p["kind"] != "module":
                    continue
                sel = view.disc(p["x"], p["y"], 0.008)
                hz = view.pz[sel]
                hz = hz[np.isfinite(hz)]
                hits.append(bool(hz.size) and abs(float(np.median(hz)) - 0.150) < 0.006)
            out[v] = (sum(hits), len(hits))
        return out

    def survey_open(self, rgb, dep, tag):
        pose = self.pack_frame(dep, 0.160)
        self.pack["pose"] = pose
        view = V.PackView(overhead_camera(), rgb, dep, pose, TZ)
        seen = self.modules_seen(view)
        full = [v for v, (k, n) in seen.items() if k == n]
        if full:
            v = max(full, key=lambda v: seen[v][1])
            if v != self.pack["variant"]:
                self.note("the modules say", v, "not", self.pack["variant"], seen)
                self.pack["variant"] = v
                self.pack["fast"] = {k: r for k, r in self.pack["fast"].items() if r["group"] == "lid"}
        self.note("modules seen", seen)
        inner = view.band(-0.28, 0.28, -0.48, 0.48) & (np.nan_to_num(view.pz, nan=0) > 0.05)
        self.pack["harness_in"] = V.harness_grasp(view, V.blue_mask(view, inner))
        self.note("harness inside:", self.pack["harness_in"])
        return view

    # ------------------------------------------------------------------ the job
    def program(self):
        try:
            yield from self.job()
        except GiveUp as exc:
            self.note("giving up:", exc)
        if self.debug_dir:
            with open(os.path.join(self.debug_dir, "estimates.json"), "w") as f:
                json.dump({"pack": self.pack, "dbg": self.dbg, "cpu_s": self.cpu}, f, default=str)
        yield from self.do(lambda ob: {"cmd": "done"})

    def isolated(self):
        return self.pack["msd_out_at"] is not None and self.o["time"] >= self.pack["msd_out_at"] + ISOLATION_S + 3.0

    def job(self):
        Z_PART, Z_MODULE, Z_LID = -0.10, 0.02, 0.17
        # 0. this cell's cameras
        yield from self.calibrate()
        # 1. the lid from above: placement, variant, its fasteners, a harness on it
        rgb, dep = yield from self.look_overhead("lid")
        self.survey_lid(rgb, dep)
        lid = [r for r in self.pack["fast"].values() if r["group"] == "lid"]
        self.note("lid fasteners:", {r["id"]: (r["state"], r["kind"]) for r in lid})
        if self.pack["harness_lid"] is not None:
            if (yield from self.change_tool("gripper")):
                yield from self.remove_harness(self.pack["harness_lid"], Z_PART)
            rgb, dep = yield from self.look_overhead("lid_clear")
            view = V.PackView(overhead_camera(), rgb, dep, self.pack["pose"], TZ)
            self.read_group(view, "lid", only_unknown=True)
            self.note("lid fasteners after the harness:", {r["id"]: (r["state"], r["kind"]) for r in lid})
        # 2. the lid's bolts, then the lid to its stand
        if not (yield from self.change_tool("nutrunner")):
            return
        left = yield from self.take_out(lid)
        if left:
            self.note("lid fasteners not out:", left)
        for attempt in range(3):
            if not (yield from self.change_tool("gripper")):
                return
            if (yield from self.remove_lid(Z_LID)):
                break
            self.note("the lid stays on: looking for what still holds it")
            if attempt == 2:
                return
            rgb, dep = yield from self.look_overhead("lid_again%d" % attempt)
            view = V.PackView(overhead_camera(), rgb, dep, self.pack["pose"], TZ)
            for r in lid:
                if r["out"]:
                    continue
                r["state"], r["refined"], r["tries"] = "unknown", False, 0
            self.read_group(view, "lid", only_unknown=True)
            if not (yield from self.change_tool("nutrunner")):
                return
            yield from self.take_out(lid)
        # 3. the pack open: the variant again from its modules, a harness on the busbars
        rgb, dep = yield from self.look_overhead("open")
        self.survey_open(rgb, dep, "open")
        # 4. isolate: the service disconnect out, then the low-voltage plugs while the pack discharges
        for pid, off in (("msd", (-0.10, -0.06)), ("J1", (0.10, -0.06)), ("J2", (0.04, -0.06))):
            ok = yield from self.pick(pid, Z_PART, pull=True)
            if ok and pid == "msd":
                self.pack["msd_out_at"] = self.o["time"]
            if ok:
                yield from self.drop_in_bin("electronics", Z_PART, offset=off)
        if self.pack["msd_out_at"] is None:
            self.note("the service disconnect is still in: stopping before the live parts")
            return
        rgb, dep = yield from self.look_overhead("isolated")
        view = self.survey_open(rgb, dep, "isolated")
        while not self.isolated():
            yield from self.wait(1.0)
        if self.pack.get("harness_in") is not None:
            if (yield from self.change_tool("gripper")):
                yield from self.remove_harness(self.pack["harness_in"], Z_PART)
            rgb, dep = yield from self.look_overhead("inside_clear")
            view = V.PackView(overhead_camera(), rgb, dep, self.pack["pose"], TZ)
        self.read_group(view, "inside")
        inner = [r for r in self.pack["fast"].values() if r["group"] != "lid"]
        self.note("inner fasteners:", {r["id"]: (r["state"], r["kind"]) for r in inner})
        # 5. busbar and bracket fasteners
        if not (yield from self.change_tool("nutrunner")):
            return
        left = yield from self.take_out(inner)
        if left:
            self.note("inner fasteners not out:", left)
        # 5b. the modules' tops, close by: a swollen one stays in its seat (manual, section 7)
        swollen = yield from self.measure_modules(Z_PART)
        # 6. busbars, brackets, modules
        if not (yield from self.change_tool("gripper")):
            return
        var = MANUAL["variants"][self.pack["variant"]]
        stuck = set(r["holds"] for r in self.pack["fast"].values() if r["state"] == "seated" and not r["out"])
        for kind, where in (("busbar", "copper"), ("bracket", "steel")):
            for p in sorted((p for p in var["parts"] if p["kind"] == kind), key=lambda p: p["id"]):
                if p["id"] in stuck:
                    continue
                if (yield from self.pick(p["id"], Z_PART)):
                    yield from self.drop_in_bin(where, Z_PART)
                else:
                    stuck.add(p["id"])
        bay = 0
        for p in sorted((p for p in var["parts"] if p["kind"] == "module"), key=lambda p: p["id"]):
            n = p["id"][1:]
            holders = {"R%sa" % n, "R%sb" % n}
            if holders & stuck:
                self.note("module still held", p["id"])
                continue
            if p["id"] in swollen:
                self.note("swollen module left in its seat", p["id"])
                continue
            if (yield from self.pick(p["id"], Z_MODULE)):
                yield from self.rack_module(bay, Z_MODULE)
                bay += 1
