"""Client of the commissioning service (holder_manual.md, section 9).

    from endocam_client import Commissioning, decode_image
    c = Commissioning()
    print(c.status())                   # which development procedures, how many sessions are left
    obs = c.start("dev03")              # one session = one development procedure, from its first step
    while obs is not None:
        frame = decode_image(obs)       # RGB uint8 (480, 640, 3), one control step old
        obs = c.step([0.0, 0.0, 0.0, 0.0])
    c.close()

A session answers observations and nothing else: no labels, no ground truth, no judgement of the view. Sessions are
counted when they start, run one at a time, and end with the procedure, with `close()`, or after two minutes without
a request. `~/commission/run_session.py` drives a whole controller (a folder with run.sh) through a session the way
the grader will.
"""

import base64
import json
import os
import socket

SOCKET = os.environ.get("ENDOCAM_SOCKET", "/run/endocam/commission.sock")


class CommissioningError(Exception):
    pass


class Commissioning(object):
    def __init__(self, path=SOCKET, timeout=300.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(path)
        self.rfile = self.sock.makefile("r", encoding="utf-8")
        self.steps = None
        self.t = None

    def _call(self, message):
        self.sock.sendall((json.dumps(message) + "\n").encode("utf-8"))
        line = self.rfile.readline()
        if not line:
            raise CommissioningError("the service closed the connection")
        reply = json.loads(line)
        if not reply.get("ok"):
            raise CommissioningError(reply.get("error", "refused"))
        return reply

    def status(self):
        return self._call({"cmd": "status"})

    def start(self, procedure):
        """Start a session on a development procedure ("dev01" ...); returns the first observation."""
        reply = self._call({"cmd": "start", "procedure": procedure})
        self.steps, self.t = reply["steps"], reply["t"]
        return reply["obs"]

    def step(self, action):
        """Apply rates [pan, tilt, insertion, roll] for one control step; the next observation, or None at the end."""
        reply = self._call({"cmd": "step", "action": [float(a) for a in action]})
        self.t = reply["t"]
        return None if reply["done"] else reply["obs"]

    def probe_copy(self, shift, joint_offset):
        """A perturbed copy of the current observation, as the grader makes them for its repeated requests: the
        picture shifted by `shift` = (dx, dy) pixels, the pan, tilt and roll readings offset by `joint_offset`."""
        return self._call({"cmd": "probe_copy", "shift": list(shift), "joint_offset": list(joint_offset)})["obs"]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def decode_image(obs):
    """An observation's frame as an RGB uint8 array (480, 640, 3)."""
    import cv2
    import numpy as np
    data = np.frombuffer(base64.b64decode(obs["image"]), dtype=np.uint8)
    return cv2.cvtColor(cv2.imdecode(data, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
