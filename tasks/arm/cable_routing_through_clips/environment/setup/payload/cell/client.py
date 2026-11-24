"""Client of the practice cell (the service behind /run/cable_cell/cell.sock).

    from client import Cell, load_frames
    cell = Cell("a")                       # a session name; up to 4 sessions run side by side
    obs = cell.reset(seed=3)               # a new practice episode (any seed 0 <= n < 2**63)
    obs, done = cell.act({"pos": [x, y, z], "yaw": a, "grip": g})
    frames = load_frames(obs)              # {"overhead": (rgb, depth), "wrist": (rgb, depth)}
    result = cell.finish()                 # the cell lets go, settles and checks the route

Observations are the ones the grader sends over the wire (instruction.md), frames included.
"""
import json
import os
import socket

import numpy as np

SOCKET = os.environ.get("CABLE_CELL_SOCKET", "/run/cable_cell/cell.sock")


class CellError(RuntimeError):
    pass


class Cell(object):
    def __init__(self, session="a", path=None):
        self.session = session
        self.path = path or SOCKET

    def request(self, method, **params):
        params["session"] = self.session
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(120)
            sock.connect(self.path)
            sock.sendall((json.dumps({"method": method, "params": params}) + "\n").encode())
            with sock.makefile("rb") as stream:
                out = json.loads(stream.readline(4000000))
        if "error" in out:
            raise CellError(out["error"])
        return out

    def reset(self, seed):
        """Start a new episode; returns its first observation."""
        return self.request("reset", seed=int(seed))["obs"]

    def act(self, action):
        """Apply an action for 0.1 s; returns (observation, done). The observation is None if the episode ended in
        an unstable state."""
        out = self.request("act", action=action)
        return out["obs"], out["done"]

    def observe(self, shift=(0.0, 0.0)):
        """The current observation again (no time passes); `shift` gives the observation of the scene moved by
        (dx, dy), the way the grader's repeated-t copies are made."""
        return self.request("observe", shift=list(shift))["obs"]

    def finish(self):
        """End the episode the way the grader does and return the cell's check of the route."""
        return self.request("finish")["result"]


def load_frames(obs):
    """The frames an observation names: {camera: (rgb uint8 HxWx3, depth float32 HxW in metres)}."""
    return {cam: (np.load(p["rgb"]), np.load(p["depth"])) for cam, p in obs["frames"].items()}
