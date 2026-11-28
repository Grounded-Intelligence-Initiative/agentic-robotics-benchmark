"""Program-side proxy of the data-hall cell, and the launcher that runs a submission against it.

    from dc_client import decode_frame, decode_scope

    def solve(cell):                                    # in /home/user/submission/shift.py
        info = cell.describe()                          # robot, control limits, grip, cameras, budgets
        tickets = cell.noc("noc.tickets")               # the site's services (cell_api.md, site/services_api.md)
        cell.command(nav_goal=[1.40, -0.30, 1.5708])    # the navigation stack walks the base round obstacles
        while cell.get_state()["robot"]["nav"]["status"] == "moving":
            cell.step(50)                               # 50 x 10 ms of physics; r["sim_time"], r["done"], r["events"]
        f = cell.get_camera("wrist_right")              # one RGB-D frame (counts against the frame budget)
        rgb, depth_m = decode_frame(f)                  # uint8 H x W x 3, float32 H x W metres (NaN = no reading)
        cell.command(right_arm={"pos": [0.40, -0.15, 0.95], "frame": "base"}, right_hand="open")
        cell.tool("driver", mode="loosen", torque=0.9, seconds=2.5)
        pic = decode_scope(cell.tool("scope"))          # the fibre probe's picture of the end face at its tip
        cell.finish()                                   # ends the shift (the launcher also calls it)

Every method is one JSON-lines request ({"id": n, "method": ..., "params": {...}}) answered by one reply line
({"id": n, "result": ...} or {"id": n, "error": code, "detail": text}). A reply carrying an error raises CellError
(its `code` is e.g. "bad_params", "forbidden", "camera_budget"; a "forbidden" call is counted against the program).

The transport: connect_stdio(), the graded run. The launcher (`main`) starts the program with the cell on its own
stdin / stdout; it moves this process's stdout (Python's and the C-level file descriptor 1) to stderr first, so prints
never corrupt the wire. There is no practice cell: Cell(send, recv) takes any pair of line functions (a stand-in of
your own for testing), and load_frame() reads the survey photos (~/given/cell/survey/) as get_camera replies.

numpy-typed arguments are accepted (sent as plain numbers / lists).
"""

import base64
import io
import json
import os
import sys
import traceback

__all__ = ["Cell", "CellError", "decode_frame", "decode_scope", "load_frame", "connect_stdio", "given_dir", "main",
           "BOOTSTRAP", "PUBLIC_CAMERAS"]

# the launcher the graded run starts: `python -B -E -s -c BOOTSTRAP <library dir> shift`
BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from dc_client.client import main; "
             "sys.exit(main(sys.argv[2:]))")
PUBLIC_CAMERAS = ("head", "wrist_left", "wrist_right")


class CellError(RuntimeError):
    """An error reply of the cell (or a broken connection). `code`: the error code; `detail`: the cell's explanation."""

    def __init__(self, code, detail=None):
        self.code = str(code)
        self.detail = detail
        RuntimeError.__init__(self, self.code if not detail else "%s: %s" % (self.code, detail))


def _jsonable(obj):
    """json.dumps fallback for numpy scalars and arrays."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    raise TypeError("%r is not JSON serialisable" % (obj,))


class Cell(object):
    """The proxy. Units: metres, radians, seconds, newton metres; quaternions w, x, y, z; the hall frame of the cell
    API document."""

    def __init__(self, send, recv, close=None):
        self._send = send
        self._recv = recv
        self._close = close
        self._id = 0

    def _call(self, _method, /, **params):
        """One request; returns the reply's result or raises CellError."""
        self._id += 1
        req = {"id": self._id, "method": str(_method), "params": params}
        try:
            self._send(json.dumps(req, default=_jsonable) + "\n")
            line = self._recv()
        except OSError as e:                     # e.g. BrokenPipeError: the cell ended the episode and hung up
            raise CellError("closed", "the connection to the cell broke (%s)" % e.__class__.__name__)
        if not line:
            raise CellError("closed", "the cell closed the connection")
        try:
            reply = json.loads(line)
        except ValueError:
            raise CellError("bad_reply", line[:120])
        if not isinstance(reply, dict) or reply.get("id") != self._id:
            raise CellError("bad_reply", "reply id mismatch")
        if "error" in reply:
            raise CellError(reply["error"], reply.get("detail"))
        return reply.get("result")

    # ---- information
    def describe(self):
        """The cell's published facts: the robot, control limits, the grasp sites, cameras (intrinsics, mounts),
        budgets."""
        return self._call("describe")

    def get_state(self):
        """sim_time, done, and the robot's base_pose [x, y, yaw] (localised, noisy), stand_height, base_moving,
        hands {closed, tactile_N, ik_residual}, nav {status, goal, remaining_m}."""
        return self._call("get_state")

    def get_camera(self, name, depth=True):
        """One frame of `name` (head, wrist_left, wrist_right): PNG rgb + 16-bit PNG depth in mm (0 = no reading),
        intrinsics, the camera's pose in the hall at capture time. Use decode_frame() on the reply."""
        return self._call("get_camera", name=str(name), depth=bool(depth))

    # ---- the tools and the site's services
    def tool(self, name, **params):
        """The smart tools: tool("driver", mode="tighten" | "loosen", torque=N m, seconds=s) starts the torque driver
        in the hand (it runs while the sim steps), tool("driver", action="status") reads its run; tool("scope") takes
        one picture of the end face in front of the fibre probe's tip (decode_scope() on the reply)."""
        return self._call("tool", name=str(name), **params)

    def noc(self, method, **params):
        """One call of the site's services (site/services_api.md): noc("noc.tickets"), noc("bmc.power", host=...,
        action=...), ... The call takes simulated time (the cell steps while it runs)."""
        return self._call("noc", method=str(method), params=params)

    # ---- motion
    def command(self, **targets):
        """Set targets of the robot (held until changed): base_velocity [vx, vy, wz], base_goal [x, y, yaw] (None
        stops a goal) or nav_goal [x, y, yaw] (the navigation stack plans and walks a path; None stops it),
        stand_height, waist [yaw, roll, pitch], left_arm / right_arm {"q": [7]} or
        {"pose": [x, y, z, qw, qx, qy, qz], "frame": "base" | "room", "path": "line" | "joint"} or
        {"pos": [x, y, z], "frame": ..., "path": ...} (a position-only target: the wrist's orientation is free),
        left_hand / right_hand "open" | "close". Returns {"ok": True, "notes": [...]} (notes name clamped values)."""
        return self._call("command", **targets)

    def step(self, n_ticks=1):
        """Advance the physics n_ticks x 10 ms (1 .. max_ticks_per_step); returns {sim_time, done, events}."""
        return self._call("step", n_ticks=int(n_ticks))

    def finish(self):
        """End the shift (the work is judged as it stands)."""
        return self._call("finish")

    def close(self):
        """Close the connection (a transport given a close function); the graded run's launcher handles its own."""
        if self._close is not None:
            try:
                self._close()
            finally:
                self._close = None


def decode_frame(reply):
    """(rgb uint8 H x W x 3, depth float32 H x W in metres with NaN where the sensor gave no reading) of a get_camera
    reply; depth is None when the frame was taken without depth."""
    import numpy as np
    from PIL import Image
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))
    depth = None
    if reply.get("sensor_depth_u16_mm_b64"):
        d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["sensor_depth_u16_mm_b64"]))), dtype=np.float32)
        depth = d * float(reply.get("depth_unit_m", 0.001))
        depth[d == float(reply.get("depth_invalid_value", 0))] = np.nan
    return rgb, depth


def decode_scope(reply):
    """The fibre probe's picture (uint8 H x W x 3) of a tool("scope") reply."""
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["image_png_b64"]))).convert("RGB"))


def given_dir():
    """The folder of the task's given material: $ARB_GIVEN_DIR in the graded run (a read-only copy), else ~/given."""
    return os.environ.get("ARB_GIVEN_DIR") or os.path.join(os.path.expanduser("~"), "given")


def connect_stdio():
    """The cell on this process's original stdin / stdout. From then on file descriptor 1 (and Python's sys.stdout)
    write to stderr and descriptor 0 reads nothing, so prints, C extensions and child processes cannot corrupt the
    wire."""
    try:
        sys.stdout.flush()
    except Exception:  # noqa: BLE001
        pass
    out = os.fdopen(os.dup(1), "w", buffering=1, encoding="utf-8")
    inp = os.fdopen(os.dup(0), "r", encoding="utf-8")
    os.dup2(2, 1)
    null = os.open(os.devnull, os.O_RDONLY)
    os.dup2(null, 0)
    os.close(null)
    sys.stdout = sys.stderr

    def send(text):
        out.write(text)
        out.flush()

    def recv():
        return inp.readline()

    return Cell(send, recv)


def load_frame(path):
    """A survey photo (~/given/cell/survey/<pod>/<view>.json with its two PNG files beside it) as the reply
    get_camera gives for such a frame, so decode_frame() reads it: the same PNG colour, the same 16-bit depth in mm."""
    folder = os.path.dirname(os.path.abspath(path))
    with open(path) as f:
        meta = json.load(f)
    reply = {k: v for k, v in meta.items() if k != "files"}
    with open(os.path.join(folder, meta["files"]["rgb"]), "rb") as f:
        reply["rgb_png_b64"] = base64.b64encode(f.read()).decode("ascii")
    if meta["files"].get("depth_mm"):
        with open(os.path.join(folder, meta["files"]["depth_mm"]), "rb") as f:
            reply["sensor_depth_u16_mm_b64"] = base64.b64encode(f.read()).decode("ascii")
    return reply


def main(argv=None):
    """Launcher of the graded run: import `shift` (or argv[0]) from the current directory, call its solve(cell) with
    the cell on the wire, and always end the episode (finish) when solve returns or raises. Returns the exit code:
    0 ok, 1 an exception in the program, 2 a cell error."""
    argv = list(sys.argv[1:] if argv is None else argv)
    module = argv[0] if argv else "shift"
    cell = connect_stdio()
    sys.path.insert(0, os.getcwd())
    code = 0
    try:
        mod = __import__(module)
        solve = getattr(mod, "solve")
        solve(cell)
    except CellError as e:
        print("[shift] cell error: %s" % e, file=sys.stderr)
        code = 2
    except Exception:  # noqa: BLE001
        traceback.print_exc(file=sys.stderr)
        code = 1
    finally:
        try:
            cell.finish()
        except Exception:  # noqa: BLE001
            pass
        try:
            sys.stderr.flush()
        except Exception:  # noqa: BLE001
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
