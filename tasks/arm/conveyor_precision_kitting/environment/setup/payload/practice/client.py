"""Client of the practice cell (/run/kitcell/practice.sock). One connection = one practice shift.

    from client import PracticeCell, decode
    cell = PracticeCell()
    r = cell.call("reset")                      # spends one of the session's practice shifts
    obs = r["obs"]
    r = cell.call("step", action=[0, 0, 0, 0, -1, 2, 0])   # -> {"obs", "done", "info"}
    img = decode(r["obs"]["wrist"])             # numpy uint8 array (H, W, 3), or shape (0, 0, 3) if not taken
    res = cell.call("close")                    # -> {"kit": [...per pocket...], "lost": n, "t": ...}

The cell does not wait for you, exactly as in grading: from the moment it sends an observation, each whole 0.1 s
until your next `step` arrives runs one hold step first (no motion, the gripper as last commanded, no images, no
turnover request; `info["hold_steps"]` says how many). A `step` later than 30 s after the previous reply ends the
shift, and so does the shift's wall-clock cap (`info["ended"]` gives the reason). Stepping by hand therefore watches
the parts go by. See ~/cell/datasheet.md for the action and observation.
"""
import base64
import json
import socket

import numpy as np

SOCKET = "/run/kitcell/practice.sock"


class PracticeCell(object):
    def __init__(self, path=SOCKET):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.connect(path)
        self.rf = self.s.makefile("rb")
        self.wf = self.s.makefile("wb")

    def call(self, method, **params):
        self.wf.write((json.dumps({"method": method, "params": params}) + "\n").encode())
        self.wf.flush()
        line = self.rf.readline()
        if not line:
            raise RuntimeError("the practice cell closed the connection")
        rep = json.loads(line)
        if "error" in rep:
            raise RuntimeError(rep["error"])
        return rep["result"]


def decode(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])
