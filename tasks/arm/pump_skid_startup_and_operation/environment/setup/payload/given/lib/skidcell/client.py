"""Agent-side proxy of the pump skid cell + the program launcher + the commissioning and graded-episode connections (the
one module of the cell the agent gets; the cell itself runs in the grader, in the commissioning service and in the
graded episode's service, out of the agent's sight).

    def solve(cell):                                   # in /home/user/submission/operate.py
        info = cell.describe()
        f = cell.get_camera("overview")                # one frame: PNG rgb + 16-bit PNG depth (mm, 0 = no reading)
        rgb, depth_m = decode_frame(f)                 # uint8 H x W x 3, float32 H x W metres (NaN where invalid)
        st = cell.get_state()                          # the robot's own state, the rail, the horn; nothing of the skid
        cell.command_rail(x=1.6, lift=0.2)             # the carriage along the rail and the lift's height
        cell.set_control("cartesian", 0.8)
        cell.command(tcp_pos=[...], tcp_quat=[w, x, y, z], width=0.035)
        r = cell.step(50)                              # 50 x 10 ms of simulation; r["events"], r["done"]
        cell.log_reading("PI-2342", 2.35, "bar")       # an entry in the operator's log sheet (a site tag)
        cell.finish()

Every method is one JSON-lines request on this process's original stdout, answered on stdin by the grader's
physics server. The launcher rebinds `sys.stdout` to stderr first, so ordinary prints never corrupt the wire.
A reply carrying "error" raises CellError (a `forbidden` error is counted by the grader as an illegal call).

Commissioning (development only): `connect_commissioning(site)` opens a shift on one of the series' development sites
(the sister sites the commissioning service runs; never a graded site): the same cell and calls, plus `reset(site)`,
which starts the shift. One connection is one shift, under the graded episode's budgets; the commissioning window has
a fixed number of shifts (`cell_manual.md` section 8). `~/commission/run_shift.py` runs a whole program in a shift the
way the grader runs it.

The graded episode: `connect_graded()` connects to the graded episode's service (a hidden site of the series): the
same cell and calls, plus `start()` (once per session: it starts the shift) and `status()`. The episode lives in the
service, not in the connection: closing the connection (or the script ending) leaves it where it is, and the next
`connect_graded()` resumes it; one connection at a time (a second one raises `CellError("busy")`). `finish()` ends it.

    cell = connect_graded()                            # resumes the episode if it has started
    cell.status()                                      # started, ended, steps, sim_time_s, frames, windows' time left
    cell.start()                                       # ONCE per session; charged at once, never refunded
    ...                                                # describe / get_state / command / step / get_camera / ...
    cell.close()                                       # the episode stays where it is
"""

import base64
import io
import json
import os
import sys
import traceback

__all__ = ["Cell", "CellError", "decode_frame", "connect_stdio", "connect_commissioning", "main", "BOOTSTRAP", "launcher_argv",
           "COMMISSION_SOCKET", "COMMISSION_STATUS", "connect_graded", "GRADED_SOCKET"]

BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from skidcell.client import main; "
             "sys.exit(main(sys.argv[2:]))")
PYTHON_FLAGS = ["-B", "-E", "-s"]
COMMISSION_SOCKET = "/run/skidcell/commission.sock"
COMMISSION_STATUS = "/run/skidcell/commission_status.json"
GRADED_SOCKET = "/run/skidcell-graded/graded.sock"


class CellError(RuntimeError):
    pass


class Cell(object):
    """The proxy. Units: metres, radians, quaternions wxyz, world frame = the skid frame (cell_manual.md)."""

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
        if reply.get("event") == "shift_over":
            raise CellError("the commissioning shift is over (%s)" % reply.get("reason"))
        if reply.get("id") != self._id:
            raise CellError("reply id mismatch")
        if "error" in reply:
            raise CellError(reply["error"])
        return reply.get("result")

    # ---- information
    def describe(self):
        return self._call("describe")

    def get_state(self):
        return self._call("get_state")

    def get_camera(self, name="overview", depth=True):
        return self._call("get_camera", name=str(name), depth=bool(depth))

    # ---- motion
    def set_control(self, mode="joint", stiffness_scale=1.0):
        return self._call("set_control", mode=str(mode), stiffness_scale=float(stiffness_scale))

    def command(self, q=None, tcp_pos=None, tcp_quat=None, width=None):
        params = {}
        if q is not None:
            params["q"] = [float(v) for v in q]
        if tcp_pos is not None:
            params["tcp_pos"] = [float(v) for v in tcp_pos]
        if tcp_quat is not None:
            params["tcp_quat"] = [float(v) for v in tcp_quat]
        if width is not None:
            params["width"] = float(width)
        return self._call("command", **params)

    def command_rail(self, x=None, lift=None):
        params = {}
        if x is not None:
            params["x"] = float(x)
        if lift is not None:
            params["lift"] = float(lift)
        return self._call("command_rail", **params)

    def step(self, n_ticks=1):
        return self._call("step", n_ticks=int(n_ticks))

    # ---- the operator's log sheet
    def log_reading(self, tag, value, unit):
        return self._call("log_reading", tag=str(tag), value=float(value), unit=str(unit))

    def finish(self):
        return self._call("finish")

    # ---- privileged (the reference program's token only)
    def get_state_privileged(self):
        return self._call("get_state_privileged")

    def reference_run(self):
        return self._call("reference_run")

    # ---- the commissioning service only (the graded cell answers {"error": "forbidden"})
    def reset(self, site):
        """Start this connection's commissioning shift on development site `site` (1 .. the number of sites): the skid
        as the previous shift left it, at sim_time 0. Once per connection; it uses one shift of the window."""
        return self._call("reset", site=int(site))

    # ---- the graded episode's service only (the other cells answer {"error": "forbidden"})
    def start(self, token=None):
        """Start the graded episode: the skid as the previous shift left it, at sim_time 0. Once per session and only
        within its start window; the start is charged at once and never refunded."""
        params = {} if token is None else {"token": str(token)}
        return self._call("start", **params)

    def status(self):
        """The graded episode's state: started, ended, end, steps, sim_time_s, camera_frames_used,
        camera_frames_left, prep_window_left_s, graded_window_left_s (nothing else: no score)."""
        return self._call("status")

    def close(self):
        closer = getattr(self, "_close", None)
        if closer is not None:
            closer()


def decode_frame(reply):
    """(rgb uint8 H x W x 3, depth float32 H x W in metres with NaN where the sensor reported no reading / None when absent)."""
    import numpy as np
    from PIL import Image
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))
    depth = None
    if reply.get("depth_u16_mm_b64"):
        d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["depth_u16_mm_b64"]))), dtype=np.float32)
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


def connect_commissioning(site=None, path=None):
    """A connection to the commissioning service (a Cell): one shift, started by `reset(site)` (done here when `site`
    is given). The service takes a few connections at a time; the shifts' budget is in COMMISSION_STATUS."""
    import socket
    path = path or COMMISSION_SOCKET
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(path)
    except OSError as e:
        sock.close()
        raise CellError("the commissioning service is not reachable at %s (%s)" % (path, e))
    rf = sock.makefile("r", encoding="utf-8", newline="\n")
    wf = sock.makefile("w", encoding="utf-8", newline="\n")

    def send(text):
        try:
            wf.write(text)
            wf.flush()
        except OSError as e:
            raise CellError("the commissioning service closed the connection (%s)" % e)

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


def connect_graded(start=False, path=None):
    """A connection to the graded episode's service (a Cell: the cell's calls, plus `start()` and `status()`). The
    episode lives in the service: closing this connection leaves it where it is, and the next connect_graded() resumes
    it. One connection at a time: while another holds the episode this raises CellError("busy"). `start=True` also
    starts the episode (once per session)."""
    import socket
    path = path or GRADED_SOCKET
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(path)
    except OSError as e:
        sock.close()
        raise CellError("the graded episode's service is not reachable at %s (%s)" % (path, e))
    rf = sock.makefile("r", encoding="utf-8", newline="\n")
    wf = sock.makefile("w", encoding="utf-8", newline="\n")

    def send(text):
        try:
            wf.write(text)
            wf.flush()
        except OSError as e:
            raise CellError("the graded episode's service closed the connection (%s)" % e)

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
    try:                                       # the first reply: the episode is ours, or another connection holds it
        cell._id += 1
        send(json.dumps({"id": cell._id, "method": "status", "params": {}}) + "\n")
        line = recv()
        reply = json.loads(line) if line else {}
        if "error" in reply or reply.get("id") != cell._id or "result" not in reply:
            raise CellError(str(reply.get("error") or "the graded episode's service closed the connection"))
    except (CellError, ValueError) as e:
        close()
        raise CellError(str(e))
    if start:
        cell.start()
    return cell


def launcher_argv(python, lib_dir, program_dir, module="operate"):
    """argv of the program: `bash run.sh` when the submission ships one (regular file), else the client launcher
    `python -B -E -s -c <bootstrap> <lib_dir> <module>`."""
    run_sh = os.path.join(program_dir, "run.sh")
    if os.path.isfile(run_sh) and not os.path.islink(run_sh):
        return ["bash", "run.sh"]
    return [python] + PYTHON_FLAGS + ["-c", BOOTSTRAP, lib_dir, module]


def main(argv=None):
    """Launcher: import operate.py from the current directory and run solve(cell)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    module = argv[0] if argv else "operate"
    cell = connect_stdio()
    sys.path.insert(0, os.getcwd())
    code = 0
    try:
        mod = __import__(module)
        getattr(mod, "solve")(cell)
    except CellError as e:
        print("[operate] cell error: %s" % e, file=sys.stderr)
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
