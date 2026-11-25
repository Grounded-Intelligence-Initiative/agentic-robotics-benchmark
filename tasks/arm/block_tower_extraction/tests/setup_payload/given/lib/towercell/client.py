"""The program's side of the cell's wire: the ``Cell`` proxy that ``solve(cell)`` receives, the launcher, the
commissioning connection and frame decoding (the one module of the cell a program gets; the cell itself runs in the
grader and in the commissioning service).

    def solve(cell):                                   # in play.py
        info = cell.describe()                         # static facts: nominal tower, cameras, budgets
        st = cell.get_state()                          # the robot's own state, wrench, turntable, budgets
        st = cell.step(target={"pos": [0.45, 0.0, 0.40], "quat": hand_quat(0.0)}, speed=0.1, n=100)
        st = cell.step(grip_close=20.0, n=40)          # squeeze; cell.step(grip_open=0.05, n=30) opens
        st = cell.step(pusher=True, turntable=1.5708, n=300)
        f = cell.get_camera("left", depth=True)        # a frame (one of the episode's frame budget)
        rgb, depth_m = decode_frame(f)
        cell.finish()

Every call is one JSON-lines request on this process's original stdout, answered on its stdin. The launcher moves the
wire away from the standard streams first: ordinary prints, and native libraries writing to file descriptor 1, go to
stderr, never into the wire. A reply carrying "error" raises CellError (``.code``, ``.detail``).

Commissioning (development only): ``connect_commissioning(site)`` opens one episode of the commissioning service: the
same calls, plus ``reset(site)`` which starts it. ``~/commission/run_tower.py`` runs a whole program the way the
grader runs it.

Runtime dependencies: the Python standard library; numpy and Pillow for ``decode_frame`` only.
"""

import base64
import io
import json
import math
import os
import sys
import traceback

__all__ = ["Cell", "CellError", "decode_frame", "decode_rgb", "decode_depth", "connect_stdio",
           "connect_commissioning", "main", "hand_quat", "quat_mul", "launcher_argv", "BOOTSTRAP",
           "COMMISSION_SOCKET", "COMMISSION_STATUS", "CAMERAS"]

PROGRAM = "play"
BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from towercell.client import main; "
             "sys.exit(main(sys.argv[2:]))")
PYTHON_FLAGS = ["-B", "-E", "-s"]
COMMISSION_SOCKET = "/run/towercell/commission.sock"
COMMISSION_STATUS = "/run/towercell/commission_status.json"
CAMERAS = ("left", "right", "wrist")


class CellError(RuntimeError):
    def __init__(self, code, detail=None):
        RuntimeError.__init__(self, code if not detail else "%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def hand_quat(yaw=0.0, tilt=0.0):
    """Orientation (w, x, y, z) of the hand pointing straight down (its z axis = world -z) with its x axis along
    (cos yaw, sin yaw, 0) (the fingers close along the hand's y axis, (sin yaw, -cos yaw, 0)), then turned by `tilt`
    about the hand's own y axis: tilt > 0 swings the fingertips toward the hand's -x side."""
    base = [0.0, 1.0, 0.0, 0.0]                     # 180 degrees about x: z down, y -> -y
    rz = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    ry = [math.cos(-tilt / 2), 0.0, math.sin(-tilt / 2), 0.0]
    return quat_mul(quat_mul(rz, base), ry)


class Cell(object):
    """The proxy. Units: metres, radians, seconds; world frame: the robot's base at the origin, z up, the tower in
    front of the robot along +x."""

    def __init__(self, send, recv):
        self._send = send
        self._recv = recv
        self._id = 0

    def _call(self, method, **params):
        self._id += 1
        self._send(json.dumps({"id": self._id, "method": method, "params": params}) + "\n")
        line = self._recv()
        if not line:
            raise CellError("closed", "the cell closed the connection")
        reply = json.loads(line)
        if reply.get("event") == "shift_over":
            raise CellError("shift_over", "the commissioning episode is over (%s)" % reply.get("reason"))
        if reply.get("id") != self._id:
            raise CellError("protocol", "reply id mismatch")
        if "error" in reply:
            raise CellError(reply["error"], reply.get("detail"))
        return reply.get("result")

    # ---- information
    def describe(self):
        """Static facts: the cell's configuration (nominal tower, turntable, staging post, robot tools, sensors,
        budgets, score rules and the hidden-value ranges), the cameras' intrinsics and fixed poses, the episode's
        duration and N_cap."""
        return self._call("describe")

    def get_state(self):
        """t, q, dq, q_cmd, tcp {pos, quat}, rod_tip {pos}, motion {goal_reached, out_of_reach, ik_error_m,
        ik_error_rad}, gripper {mode, setpoint, opening_m}, wrench {force_N, torque_Nm} (wrist sensor, world frame,
        noisy), pusher_deployed, turntable {angle_rad, target_rad, moving}, frames_left, program_time_left_s,
        episode_over, over_reason."""
        return self._call("get_state")

    def get_camera(self, name, depth=False):
        """A frame of camera `name` (left, right, wrist) at the current time: PNG colour (+ 16-bit depth when
        depth=True), stamp, T_world_cam (OpenCV convention) and intrinsics. One frame of the budget; asking the same
        camera again at the same time is free."""
        return self._call("get_camera", name=str(name), depth=bool(depth))

    # ---- motion and tools
    def step(self, n=1, target=None, frame="tcp", speed=0.1, rot_speed=1.0, q_target=None, joint_speed=0.8,
             posture=None, grip_open=None, grip_close=None, pusher=None, turntable=None):
        """Set new commands (each optional) and advance n ticks (0 .. 1000, 10 ms each).
        target {"pos": [x, y, z], "quat": [w, x, y, z]}: a goal pose of `frame` ("tcp", the point between the finger
        pads, or "rod_tip", the pusher's tip) in the world frame; the cell moves the frame there along a straight line
        at `speed` m/s (<= 0.5), turning at `rot_speed` rad/s, and holds it (motion.goal_reached).
        q_target: 7 joint goals (rad) at `joint_speed` rad/s instead. posture: 7 joint values the redundancy
        resolution leans toward (kept until changed). grip_open: drive the fingers to this opening (m, <= 0.08);
        grip_close: squeeze with this force (N, <= 40). pusher: deploy (True) / stow (False) the pusher rod.
        turntable: the turntable's goal angle (rad, absolute). Returns get_state() plus `ticks`."""
        params = {"n": int(n)}
        if target is not None:
            params["target"] = {"pos": [float(x) for x in target["pos"]], "quat": [float(x) for x in target["quat"]]}
            params["frame"] = str(frame)
            params["speed"] = float(speed)
            params["rot_speed"] = float(rot_speed)
        if q_target is not None:
            params["q_target"] = [float(x) for x in q_target]
            params["joint_speed"] = float(joint_speed)
        if posture is not None:
            params["posture"] = [float(x) for x in posture]
        if grip_open is not None:
            params["grip_open"] = float(grip_open)
        if grip_close is not None:
            params["grip_close"] = float(grip_close)
        if pusher is not None:
            params["pusher"] = bool(pusher)
        if turntable is not None:
            params["turntable"] = float(turntable)
        return self._call("step", **params)

    def ik(self, pos, quat, frame="tcp", seed=None):
        """Inverse kinematics of the arm for a pose of `frame` (no motion): {q, pos_error_m, rot_error_rad}; the
        solver starts from `seed` (7 values; default the home posture) and leans toward it."""
        p = {"pos": [float(x) for x in pos], "quat": [float(x) for x in quat], "frame": str(frame)}
        if seed is not None:
            p["seed"] = [float(x) for x in seed]
        return self._call("ik", **p)

    def finish(self):
        """End the episode now (the tower is then settled and scored as it stands)."""
        return self._call("finish")

    # ---- privileged (the reference program's token only)
    def get_state_privileged(self):
        return self._call("get_state_privileged")

    def contacts_above(self, i):
        return self._call("contacts_above", i=int(i))

    # ---- the commissioning service only (the graded cell answers {"error": "forbidden"})
    def reset(self, site):
        """Start this connection's commissioning episode on practice tower `site`."""
        return self._call("reset", site=int(site))

    def close(self):
        closer = getattr(self, "_close", None)
        if closer is not None:
            closer()


def decode_rgb(reply):
    """uint8 H x W x 3 of a frame reply."""
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))


def decode_depth(reply):
    """float32 H x W depth in metres along the optical axis, NaN where the sensor had no reading; None without depth."""
    import numpy as np
    from PIL import Image
    if not reply.get("depth_png_b64"):
        return None
    d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["depth_png_b64"]))), dtype=np.float32)
    out = d * float(reply.get("depth_unit_m", 0.0001))
    out[d == float(reply.get("depth_invalid", 0))] = np.nan
    return out


def decode_frame(reply):
    return decode_rgb(reply), decode_depth(reply)


def connect_stdio():
    """Bind the wire to the original stdin / stdout; afterwards file descriptor 1 and sys.stdout both point at
    stderr (prints and native writes never reach the wire)."""
    wire_out = os.dup(1)
    os.dup2(2, 1)
    wire_in = os.dup(0)
    out = os.fdopen(wire_out, "w", buffering=1)
    inp = os.fdopen(wire_in, "r", buffering=1)
    sys.stdout = sys.stderr

    def send(text):
        out.write(text)
        out.flush()

    def recv():
        return inp.readline()
    return Cell(send, recv)


def connect_commissioning(site=None, path=None):
    """A connection to the commissioning service (a Cell): one episode, started by `reset(site)` (done here when
    `site` is given)."""
    import socket
    path = path or COMMISSION_SOCKET
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(path)
    except OSError as e:
        sock.close()
        raise CellError("unreachable", "the commissioning service is not reachable at %s (%s)" % (path, e))
    rf = sock.makefile("r", encoding="utf-8", newline="\n")
    wf = sock.makefile("w", encoding="utf-8", newline="\n")

    def send(text):
        try:
            wf.write(text)
            wf.flush()
        except OSError as e:
            raise CellError("closed", "the commissioning service closed the connection (%s)" % e)

    def recv():
        try:
            return rf.readline()
        except OSError:
            return ""

    def close():
        for f in (wf, rf):
            try:
                f.close()
            except OSError:
                pass
        sock.close()
    cell = Cell(send, recv)
    cell._close = close
    if site is not None:
        cell.reset(site)
    return cell


def launcher_argv(python, lib_dir, program_dir, module=PROGRAM):
    """argv of the program as the grader starts it: `bash run.sh` when the folder ships one, else the client launcher
    `python -B -E -s -c <bootstrap> <lib_dir> <module>`."""
    run_sh = os.path.join(program_dir, "run.sh")
    if os.path.isfile(run_sh) and not os.path.islink(run_sh):
        return ["bash", "run.sh"]
    return [python] + PYTHON_FLAGS + ["-c", BOOTSTRAP, lib_dir, module]


def main(argv):
    """The launcher: import the program module (default `play`) from the current directory and call solve(cell) on
    the stdio wire. Exit 0 after solve returns (the episode is then finished), 1 on an exception."""
    module = argv[0] if argv else PROGRAM
    cell = connect_stdio()
    sys.path.insert(0, os.getcwd())
    try:
        mod = __import__(module)
        mod.solve(cell)
        try:
            cell.finish()
        except CellError:
            pass
        return 0
    except CellError as e:
        if e.code in ("shift_over", "closed"):
            return 0
        traceback.print_exc()
        return 1
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        return 1
