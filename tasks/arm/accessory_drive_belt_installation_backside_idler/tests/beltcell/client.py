"""Agent-side proxy of the belt station + the program launcher (the one module of the cell the agent gets; the cell
itself runs in the grader, out of the agent's sight).

    def solve(cell):                                   # in /home/user/submission/belt.py
        info = cell.describe()                         # robots, hands, sensors, cameras, the drive drawing, budgets
        wo = cell.get_work_order()                     # engine id, variant, belt part number, decal
        f = cell.get_camera("front")                   # one frame: PNG rgb + 16-bit PNG depth + the camera pose
        rgb, depth_m = decode_frame(f)                 # uint8 H x W x 3, float32 H x W metres (NaN where invalid)
        st = cell.get_state()                          # both arms: joints, tcp, gripper, F/T (tare-relative)
        cell.set_control("right", "cartesian", 1.0)
        cell.command(right={"tcp_pos": [...], "tcp_quat": [w, x, y, z], "width": 0.05})
        cell.tare("right")                             # zero the right arm's F/T at its current reading
        r = cell.step(50)                              # 50 x 10 ms of simulation; r["done"], r["stopped"]
        cell.finish()

Every method is one JSON-lines request on this process's original stdout, answered on stdin by the grader's
physics server. The launcher rebinds `sys.stdout` to stderr first, so ordinary prints never corrupt the wire.
A reply carrying "error" raises CellError (a `forbidden` error is counted by the grader as an illegal call).
"""

import base64
import io
import json
import os
import sys
import traceback

__all__ = ["Cell", "CellError", "decode_frame", "connect_stdio", "main", "BOOTSTRAP", "launcher_argv"]

BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from beltcell.client import main; "
             "sys.exit(main(sys.argv[2:]))")
PYTHON_FLAGS = ["-B", "-E", "-s"]


class CellError(RuntimeError):
    pass


class Cell(object):
    """The proxy. Units: metres, radians, quaternions wxyz, world frame (cell_manual.md). Arms: "left", "right"."""

    def __init__(self, send, recv):
        self._send = send
        self._recv = recv
        self._id = 0

    def _call(self, method, **params):
        self._id += 1
        req = {"id": self._id, "method": method, "params": params}
        self._send(json.dumps(req) + "\n")
        line = self._recv()
        if not line:
            raise CellError("the cell closed the connection")
        reply = json.loads(line)
        if reply.get("id") != self._id:
            raise CellError("reply id mismatch")
        if "error" in reply:
            raise CellError(reply["error"])
        return reply.get("result")

    # ---- information
    def describe(self):
        return self._call("describe")

    def get_work_order(self):
        return self._call("get_work_order")

    def get_state(self):
        return self._call("get_state")

    def get_camera(self, name="front", depth=True):
        """name: "front", "left_wrist" or "right_wrist"."""
        return self._call("get_camera", name=str(name), depth=bool(depth))

    # ---- motion
    def set_control(self, arm, mode="joint", stiffness_scale=1.0):
        return self._call("set_control", arm=str(arm), mode=str(mode), stiffness_scale=float(stiffness_scale))

    def command(self, left=None, right=None):
        """Per arm a dict: {"q": [7]} (joint mode) or {"tcp_pos": [3], "tcp_quat": [4]} (cartesian mode), and/or
        {"width": metres}. Both arms in one call take effect on the same tick."""
        params = {}
        for k, spec in (("left", left), ("right", right)):
            if spec is None:
                continue
            p = {}
            for key in ("q", "tcp_pos", "tcp_quat"):
                if spec.get(key) is not None:
                    p[key] = [float(v) for v in spec[key]]
            if spec.get("width") is not None:
                p["width"] = float(spec["width"])
            params[k] = p
        return self._call("command", **params)

    def tare(self, arm):
        """Zero that arm's wrist force/torque sensor at its current reading."""
        return self._call("tare", arm=str(arm))

    def step(self, n_ticks=1):
        return self._call("step", n_ticks=int(n_ticks))

    def finish(self):
        return self._call("finish")


def decode_frame(reply):
    """(rgb uint8 H x W x 3, depth float32 H x W in metres with NaN where the sensor reported no reading / None when
    absent)."""
    import numpy as np
    from PIL import Image
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))
    depth = None
    if reply.get("depth_u16_b64"):
        d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["depth_u16_b64"]))), dtype=np.float32)
        depth = d * float(reply.get("depth_unit_m", 0.001))
        depth[d == float(reply.get("depth_invalid_value", 0))] = np.nan
    return rgb, depth


def connect_stdio():
    """Bind the wire to the original stdin/stdout and send prints to stderr."""
    out = os.fdopen(os.dup(1), "w", buffering=1)
    inp = os.fdopen(os.dup(0), "r", buffering=1)
    sys.stdout = sys.stderr

    def send(text):
        out.write(text)
        out.flush()

    def recv():
        return inp.readline()

    return Cell(send, recv)


def launcher_argv(python, lib_dir, program_dir, module="belt"):
    """argv of the program: `bash run.sh` when the submission ships one (regular file), else the client launcher
    `python -B -E -s -c <bootstrap> <lib_dir> <module>`."""
    run_sh = os.path.join(program_dir, "run.sh")
    if os.path.isfile(run_sh) and not os.path.islink(run_sh):
        return ["bash", "run.sh"]
    return [python] + PYTHON_FLAGS + ["-c", BOOTSTRAP, lib_dir, module]


def main(argv=None):
    """Launcher: import belt.py from the current directory and run solve(cell)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    module = argv[0] if argv else "belt"
    cell = connect_stdio()
    sys.path.insert(0, os.getcwd())
    code = 0
    try:
        mod = __import__(module)
        getattr(mod, "solve")(cell)
    except CellError as e:
        print("[belt] cell error: %s" % e, file=sys.stderr)
        code = 2
    except Exception:  # noqa: BLE001
        traceback.print_exc(file=sys.stderr)
        code = 1
    finally:
        try:
            cell.finish()
        except Exception:  # noqa: BLE001
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
