"""Client of the commissioning service: the cell on its development kits (data sheet, section 10).

    import sys; sys.path.insert(0, "/home/user/commission")
    from client import Episode, status

    print(status())                       # the window: episodes used and left
    ep = Episode(kit=3)                   # one connection = one episode; it counts from here
    obs = ep.obs                          # the first observation (all three images)
    while not ep.done:
        action = [0, 0, 0, 0, 0, 0, -1, 0]            # the public action format
        obs = ep.step(action)                          # the next observation (None once a budget ended the episode)
    print(ep.close(images=True)["ended"])  # how it ended; with images=True also the final camera images

The service answers what the cell gives a controller (observations in the wire form, see `decode_image`) and how an
episode ended. It answers no object pose, no groove, no score and no completion verdict. A step reply may carry
`probe_obs` (in `ep.probe_obs`): the perturbed copy of the observation's readings that the grader would send with a
repeated step index; `run_kit.py` uses it to query a controller the way the grader does.

The graded episode (the task's second scoring item; see the task instruction): `connect_graded()` connects to the
graded episode's service, a hidden kit of the graded set that you assemble yourself, live. The episode lives in the
service, not in the connection: closing the connection (or a script ending) leaves it where it is, and the next
`connect_graded()` resumes it; one connection at a time (a second one raises GradedError("busy")).

    from client import connect_graded
    g = connect_graded()                  # resumes the episode if it has started
    print(g.status())                     # started, ended, steps, image charges, the windows' time left
    obs = g.start()                       # ONCE per session: your controller is frozen, commissioning ends
    while not g.done:
        obs = g.step([0, 0, 0, 0, 0, 0, -1, 0])        # the public action format; obs is the next observation
    g.close()                             # or g.finish() to end the graded episode now

Standard library only (NumPy only inside `decode_image`).
"""

import json
import socket

SOCKET = "/run/keyedcell/commission.sock"
STATUS = "/run/keyedcell/commission_status.json"
GRADED_SOCKET = "/run/keyedcell-graded/graded.sock"


class CommissioningError(Exception):
    """The service refused a request: `code` is its error word (busy, commissioning_window_used, bad_params,
    bad_action, episode_over, no_episode, cell_fault, ...), `detail` its sentence."""

    def __init__(self, code, detail=""):
        Exception.__init__(self, "%s: %s" % (code, detail) if detail else str(code))
        self.code = code
        self.detail = detail


def status():
    """The commissioning window: {"commissioning_episodes", "episodes_used", "episodes_left", "development_kits",
    "open_at_a_time"}."""
    with open(STATUS) as f:
        return json.load(f)


def decode_image(d):
    """A wire image {"dtype": "uint8", "shape": [H, W, 3], "b64": ...} as a NumPy array (H, W, 3), RGB."""
    import base64

    import numpy as np
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


class Episode(object):
    """One commissioning episode on development kit `kit` (1 .. the number of kits)."""

    def __init__(self, kit, socket_path=SOCKET, timeout=180.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(socket_path)
        self.file = self.sock.makefile("rwb")
        self.n = 0
        self.done = False
        self.ended = None            # how the episode ended, once the service said so
        self.probe_obs = None        # the last step's perturbed copy, if the service sent one
        self.first = self._call("reset", {"kit": kit})
        self.obs = self.first["obs"]
        self.kit, self.episode = self.first["kit"], self.first["episode"]
        self.episodes_left, self.max_steps = self.first["episodes_left"], self.first["max_steps"]

    def _call(self, method, params=None):
        self.n += 1
        try:
            self.file.write((json.dumps({"id": self.n, "method": method, "params": params or {}}) + "\n").encode("utf-8"))
            self.file.flush()
        except OSError:
            pass                      # the service closed the episode: its last word, if any, follows
        while True:
            line = self.file.readline()
            if not line:
                self.done = True
                raise CommissioningError("closed", self.ended or "the service closed the connection")
            reply = json.loads(line.decode("utf-8"))
            if reply.get("event") == "episode_over":       # a limit ended the episode while the service waited
                self.done, self.ended = True, reply.get("reason")
                continue
            if "error" in reply:
                if reply["error"] in ("episode_over", "cell_fault"):
                    self.done = True
                raise CommissioningError(reply["error"], reply.get("detail", ""))
            return reply["result"]

    def step(self, action):
        """Apply one action; returns the next observation (wire form). Sets `done`, `ended`, `probe_obs`."""
        res = self._call("step", {"action": [float(a) for a in action]})
        self.obs = res["obs"]
        self.probe_obs = res.get("probe_obs")
        if res["done"]:
            self.done, self.ended = True, res.get("ended")
        return self.obs

    def close(self, images=False):
        """End the episode. Returns {"ended", "steps", "controller_s", "images_requested", ...}, with
        "final_images" (left, right, wrist in the wire form) when `images` is true."""
        try:
            res = self._call("close", {"images": bool(images)})
        finally:
            self.done = True
            try:
                self.file.close()
                self.sock.close()
            except OSError:
                pass
        self.ended = res.get("ended")
        return res


class GradedError(Exception):
    """The graded episode's service refused a request: `code` is its error word (busy, not_started, already_started,
    prep_window_closed, episode_over, bad_action, bad_params, forbidden, internal, closed), `detail` its sentence."""

    def __init__(self, code, detail=""):
        Exception.__init__(self, "%s: %s" % (code, detail) if detail else str(code))
        self.code = code
        self.detail = detail


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
        """{"started", "ended", "end", "steps", "max_steps", "images_requested", "image_charge_used_s",
        "image_charge_left_s", "prep_window_left_s", "graded_window_left_s", "program_frozen"}: no score."""
        return self._call("status")

    def start(self):
        """Start the graded episode: ONCE per session, within its start window. It freezes the submission folder (your
        controller) and ends commissioning first, then is charged at once and never refunded. Returns the first
        observation (all three images)."""
        res = self._call("start")
        self.obs, self.done = res["obs"], False
        return self.obs

    def step(self, action):
        """Apply one action (8 numbers, the public format); returns the next observation, or None when the step's
        images would take the image charges past the kit's budget (the step is not applied and the episode is over).
        Sets `done` once the episode is over. A malformed action raises GradedError("bad_action") and simulates nothing."""
        res = self._call("step", {"action": [float(a) for a in action]})
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
        """End the graded episode now: it is judged as it stands, nothing is simulated afterwards."""
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
