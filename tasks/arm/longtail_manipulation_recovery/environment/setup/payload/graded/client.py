"""Client of the graded episode's service: the one graded episode you play yourself, live, during this session.

    import sys; sys.path.insert(0, "/home/user")
    from graded.client import GradedEpisode
    g = GradedEpisode()               # one connection at a time; once the episode has started, this resumes it
    st = g.status()                   # started, ended, end, ended_by, steps, sim_time_s, steps_left, camera_frames_left,
                                      # prep_window_left_s, graded_window_left_s, program_frozen
    r = g.start()                     # ONCE per session (it builds the kitchen first, about a minute): r["obs"] is the
                                      # first observation (t = 0, the card, all three RGB-D views)
    r = g.step(action)                # r["obs"], r["done"], r["ended"] (how the episode ended), r["steps_left"]
    r = g.observe()                   # the last observation again (images included); nothing is simulated
    g.finish()                        # ends the graded episode: it is credited as it stands
    g.close()                         # the episode stays where it is; the next GradedEpisode() resumes it

The observations and actions are the ones your controller gets and gives when it is graded (the instruction's "Wire
protocol": 12 numbers, or 13 with the camera mask; the observation's proprio, wrist_ft, camera_pose_world, images and,
at t = 0, the card). The service tells nothing else: no object pose, no fixture state, no success flag. `GradedError.code`
is one of: busy, bad_request, bad_params, unknown_method, not_started, already_started, prep_window_closed,
episode_over, malformed_action, kitchen_failed, snapshot_failed, internal, closed.
"""

import base64
import json
import socket

SOCK = "/run/longgrade/graded.sock"


class GradedError(Exception):
    def __init__(self, code, detail=""):
        Exception.__init__(self, "{}: {}".format(code, detail))
        self.code, self.detail = code, detail


def decode_image(d):
    """A wire image {"dtype", "shape", "b64"} as bytes, its dtype and its shape; with NumPy:
    np.frombuffer(raw, dtype).reshape(shape) (uint8 H x W x 3 for rgb, uint16 H x W millimetres for depth_mm)."""
    return base64.b64decode(d["b64"]), d["dtype"], tuple(d["shape"])


class GradedEpisode(object):
    def __init__(self, path=SOCK, timeout=1200.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        try:
            self.sock.connect(path)
        except OSError as exc:
            self.sock.close()
            raise GradedError("closed", "the graded episode's service is not reachable at {} ({})".format(path, exc))
        self.f = self.sock.makefile("rwb")
        self.n = 0
        try:
            self.status()                 # the first reply says whether this connection holds the episode
        except GradedError:
            self.close()
            raise

    def call(self, method, **params):
        self.n += 1
        try:
            self.f.write((json.dumps({"id": self.n, "method": method, "params": params}) + "\n").encode("utf-8"))
            self.f.flush()
        except (BrokenPipeError, ConnectionError):
            pass                          # a refused connection (busy) has its answer waiting to be read
        try:
            line = self.f.readline()
        except (BrokenPipeError, ConnectionError):
            line = b""
        if not line:
            raise GradedError("closed", "the service closed the connection (600 s of silence close it)")
        msg = json.loads(line.decode("utf-8"))
        if "error" in msg:
            raise GradedError(msg["error"], msg.get("detail") or "")
        return msg["result"]

    def status(self):
        return self.call("status")

    def start(self):
        return self.call("start")

    def step(self, action):
        return self.call("step", action=[float(x) for x in action])

    def observe(self):
        return self.call("observe")

    def finish(self):
        return self.call("finish")

    def close(self):
        try:
            self.call("close")
        except (GradedError, OSError, ValueError):
            pass
        try:
            self.sock.close()
        except OSError:
            pass
