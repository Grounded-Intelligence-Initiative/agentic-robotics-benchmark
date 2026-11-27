"""Client of the graded episode's service (the task's second scoring item; see the task instruction): one episode of
the hidden set, on a scored unit, that you play yourself, live, step by step.

    import sys; sys.path.insert(0, "/home/user/graded")
    from client import connect_graded, decode_image

    g = connect_graded()                  # resumes the episode if it has started
    print(g.status())                     # started, ended, steps, images, the windows' time left
    obs = g.start()                       # ONCE per session: your controller is frozen; the first observation
    while not g.done:
        obs = g.step({"l": [0, 0, 0, 0, 0, 0, -1], "r": [0, 0, 0, 0, 0, 0, -1], "cams": 0})
    g.close()                             # or g.finish() to end the graded episode now

The episode lives in the service, not in the connection: closing the connection (or a script ending) leaves it where
it is, and the next `connect_graded()` resumes it; one connection at a time (a second one raises GradedError("busy")).
Observations are in the wire form of the task instruction (`decode_image` turns an image into a NumPy array).

Standard library only (NumPy only inside `decode_image`).
"""

import json
import socket

GRADED_SOCKET = "/run/jarcell-graded/graded.sock"


class GradedError(Exception):
    """The graded episode's service refused a request: `code` is its error word (busy, not_started, already_started,
    prep_window_closed, episode_over, bad_action, bad_params, forbidden, internal, closed), `detail` its sentence."""

    def __init__(self, code, detail=""):
        Exception.__init__(self, "%s: %s" % (code, detail) if detail else str(code))
        self.code = code
        self.detail = detail


def decode_image(d):
    """A wire image {"dtype": "uint8", "shape": [H, W, 3], "b64": ...} as a NumPy array (H, W, 3), RGB."""
    import base64

    import numpy as np
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


class GradedEpisode(object):
    """A connection to the graded episode's service (connect_graded()). `obs` is the latest observation it returned
    (wire form), `done` whether the episode is over."""

    def __init__(self, socket_path=GRADED_SOCKET, timeout=600.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        try:
            self.sock.connect(socket_path)
        except OSError as e:
            self.sock.close()
            raise GradedError("closed", "the graded episode's service is not reachable at %s (%s)" % (socket_path, e))
        self.file = self.sock.makefile("rwb")
        self.n = 0
        self.obs = None
        self.done = False
        try:
            st = self.status()                 # the episode is ours, or another connection holds it (busy)
        except GradedError:
            self.close()
            raise
        self.done = bool(st.get("ended"))

    def _call(self, method, params=None):
        self.n += 1
        try:
            self.file.write((json.dumps({"id": self.n, "method": method, "params": params or {}}) + "\n").encode("utf-8"))
            self.file.flush()
        except OSError:
            pass                               # the service closed the connection: its last word, if any, follows
        line = self.file.readline()
        if not line:
            raise GradedError("closed", "the graded episode's service closed the connection")
        reply = json.loads(line.decode("utf-8"))
        if "error" in reply:
            if reply["error"] == "episode_over":
                self.done = True
            raise GradedError(reply["error"], reply.get("detail", ""))
        return reply["result"]

    def status(self):
        """{"started", "ended", "end", "steps", "max_steps", "images_requested", "prep_window_left_s",
        "graded_window_left_s", "program_frozen"}: no score."""
        return self._call("status")

    def start(self):
        """Start the graded episode: ONCE per session, within its start window. It freezes the submission folder (your
        controller) first, then is charged at once and never refunded. Returns the first observation (all five
        images)."""
        res = self._call("start")
        self.obs, self.done = res["obs"], False
        return self.obs

    def step(self, action):
        """Apply one action ({"l": [...], "r": [...], "cams": mask}, the format of the task instruction); returns the
        next observation (None after a simulator fatal error, which ends the episode). Sets `done` once the episode is
        over. A malformed action raises GradedError("bad_action") and simulates nothing."""
        res = self._call("step", {"action": action})
        self.obs = res["obs"]
        if res["done"]:
            self.done = True
        return self.obs

    def observe(self):
        """The latest observation again, as the last step (or the start) returned it, images included; nothing is
        simulated."""
        res = self._call("observe")
        self.obs, self.done = res["obs"], bool(res["done"])
        return self.obs

    def finish(self):
        """End the graded episode now: it is credited as it stands, nothing is simulated afterwards."""
        res = self._call("finish")
        self.done = True
        return res

    def close(self):
        """Close this connection; the episode stays where it is."""
        try:
            self.file.close()
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def connect_graded(socket_path=GRADED_SOCKET):
    """A connection to the graded episode's service (GradedEpisode); raises GradedError("busy") while another
    connection holds the episode."""
    return GradedEpisode(socket_path)
