"""Agent-side proxy of the information-tier cell + its launcher (PUBLIC CODE; the wire of `arm.assembly_cell`).

    def solve(cell):                          # in /home/user/submission/assemble.py
        info = cell.describe()
        f = cell.get_camera("top")            # one frame: PNG rgb + 16-bit PNG sensor depth (mm, 0 = invalid)
        rgb, depth_m = decode_frame(f)        # uint8 H x W x 3, float32 H x W metres (NaN where invalid)
        st = cell.get_state()                 # robot state only (noisy wrench)
        cell.set_control("cartesian", 0.35)
        cell.command(tcp_pos=[...], tcp_quat=[w, x, y, z], width=0.08)
        r = cell.step(20)                     # r["events"]: only "safety:..." and "episode_over"
        cell.finish()

The graded episode (dual mode): connect_graded() reaches the graded episode's service over its Unix socket and
returns a GradedCell, the same calls plus status(), start() and close():

    from arm.assembly_info.client import connect_graded
    cell = connect_graded()                   # starts nothing
    print(cell.status())
    cell.start()                              # freezes the submission, then starts THE graded episode (once)
    solve(cell)                               # any solve(cell) runs on it unchanged
    cell.close()                              # this connection only: the episode stays where it is
"""

import base64
import json
import io
import os
import sys
import traceback

from ..assembly_cell.client import Cell, CellError, connect_stdio as _connect_stdio

__all__ = ["InfoCell", "CellError", "GradedCell", "decode_frame", "connect_stdio", "connect_graded", "main", "BOOTSTRAP",
           "launcher_argv", "GRADED_SOCKET"]

BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from arm.assembly_info.client import main; "
             "sys.exit(main(sys.argv[2:]))")
# the graded episode's service (dual mode); $ARB_GRADED_SOCKET overrides it
GRADED_SOCKET = "/run/arb-graded/graded.sock"


class InfoCell(Cell):
    def get_camera(self, name="top", depth=True):
        return self._call("get_camera", name=str(name), depth=bool(depth))

    def get_state_privileged(self):
        return self._call("get_state_privileged")

    def describe_privileged(self):
        return self._call("describe_privileged")

    def truth(self):
        return self._call("truth")


def decode_frame(reply):
    """(rgb uint8 H x W x 3, depth float32 H x W in metres with NaN where the sensor reported 0 / None when absent)."""
    import numpy as np
    from PIL import Image
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["rgb_png_b64"]))).convert("RGB"))
    depth = None
    if reply.get("sensor_depth_u16_mm_b64"):
        d = np.asarray(Image.open(io.BytesIO(base64.b64decode(reply["sensor_depth_u16_mm_b64"]))), dtype=np.float32)
        depth = d * float(reply.get("depth_unit_m", 0.001))
        depth[d == float(reply.get("depth_invalid_value", 0))] = np.nan
    return rgb, depth


def connect_stdio():
    c = _connect_stdio()
    cell = InfoCell(c._send, c._recv)
    return cell


class GradedCell(InfoCell):
    """The graded episode's cell (connect_graded()): the same calls as the cell a program gets (describe, get_state,
    set_control, command, step, get_camera, finish), plus status(), start() and close(). The episode lives in the graded
    episode's service, not in this connection: close() (or the end of this process) leaves it exactly where it is and
    the next connect_graded() resumes it; finish() ends it for good. An error reply raises CellError("<code>") as the
    program's cell does, with ": <detail>" appended when the service explains it (not_started, episode_over, ...)."""

    def __init__(self, send, recv, close):
        InfoCell.__init__(self, send, recv)
        self._close = close

    def _call(self, method, **params):
        self._id += 1
        self._send(json.dumps({"id": self._id, "method": method, "params": params}) + "\n")
        line = self._recv()
        if not line:
            raise CellError("closed: the graded episode's service closed the connection")
        reply = json.loads(line)
        if reply.get("id") != self._id:
            raise CellError("reply id mismatch")
        if "error" in reply:
            detail = reply.get("detail")
            raise CellError("%s: %s" % (reply["error"], detail) if detail else reply["error"])
        return reply.get("result")

    def status(self):
        """Where the graded episode stands, at any time: started, ended, end (how it ended), steps (step calls),
        sim_time, sim_time_max, camera_frames_used, camera_frames_max, prep_window_left_s, graded_window_left_s
        (seconds; None where a window does not apply) and submission_frozen ("ok" | "too_large" | None)."""
        return self._call("status")

    def start(self):
        """Freeze the submission folder, then start the graded episode: once per session, never undone (a second
        start answers "already_started"; after the preparation window, "prep_window_closed"). Returns status()."""
        return self._call("start")

    def close(self):
        """Close this connection only; the episode stays exactly where it is."""
        self._close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def connect_graded(path=None, timeout=None):
    """The graded episode's cell over its Unix stream socket: `path` (default $ARB_GRADED_SOCKET or GRADED_SOCKET).
    Connecting starts nothing (see GradedCell.start). `timeout` (s, optional) bounds every socket operation. One
    connection at a time: while another one is open this raises CellError("busy: ..."). Returns a GradedCell; close()
    it when the script is done with it."""
    import socket
    if path is None:
        path = os.environ.get("ARB_GRADED_SOCKET", GRADED_SOCKET)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        if timeout is not None:
            s.settimeout(float(timeout))
        s.connect(path)
    except OSError as e:
        s.close()
        raise CellError("unreachable: cannot reach the graded episode's service at %s (%s)" % (path, e))
    rf = s.makefile("r", encoding="utf-8", newline="\n")
    wf = s.makefile("w", encoding="utf-8", newline="\n")

    def send(text):
        wf.write(text)
        wf.flush()

    def recv():
        return rf.readline()

    def close():
        for f in (wf, rf):
            try:
                f.close()
            except Exception:  # noqa: BLE001
                pass
        s.close()

    # the service answers a connection it refuses (busy, forbidden) with one error line: a status request first
    try:
        send(json.dumps({"id": 0, "method": "status", "params": {}}) + "\n")
        line = recv()
    except OSError as e:
        close()
        raise CellError("closed: the connection to the graded episode's service broke (%s)" % type(e).__name__)
    try:
        reply = json.loads(line) if line else None
    except ValueError:
        reply = None
    if not isinstance(reply, dict) or "error" in reply or reply.get("id") != 0:
        close()
        if isinstance(reply, dict) and "error" in reply:
            detail = reply.get("detail")
            raise CellError("%s: %s" % (reply["error"], detail) if detail else reply["error"])
        raise CellError("closed: the graded episode's service did not answer")
    return GradedCell(send, recv, close)


def launcher_argv(python, lib_dir, program_dir, module="assemble"):
    run_sh = os.path.join(program_dir, "run.sh")
    if os.path.isfile(run_sh) and not os.path.islink(run_sh):
        return ["bash", "run.sh"]
    from ..assembly_cell.runner import PYTHON_FLAGS
    return [python] + PYTHON_FLAGS + ["-c", BOOTSTRAP, lib_dir, module]


def main(argv=None):
    """Launcher: import assemble.py from the current directory and run solve(cell)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    module = argv[0] if argv else "assemble"
    cell = connect_stdio()
    sys.path.insert(0, os.getcwd())
    code = 0
    try:
        mod = __import__(module)
        getattr(mod, "solve")(cell)
    except CellError as e:
        print("[assemble] cell error: %s" % e, file=sys.stderr)
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
