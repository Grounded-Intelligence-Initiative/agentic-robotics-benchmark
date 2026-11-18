"""graded_episode — play the graded episode: start it, step it, check it, finish it.

The graded episode runs in a service of its own on this machine (its own account; you can neither read nor
change it). You talk to it through a unix socket, one client at a time; this module wraps that socket.
Stdlib-only, 3.8-compatible syntax: import it from any Python (/opt/venv/bin/python3 is 3.12 with NumPy,
Pillow and SciPy).

    import sys; sys.path.insert(0, "/home/user")
    from graded_episode import GradedEpisode
    ep = GradedEpisode()                 # connects; raises GradedEpisodeError("busy") if another client is
    print(ep.status())                   # {"started": false, "ended": false, "steps": 0, "horizon": 20000, ...}
    obs = ep.start()                     # ONCE per session, charged at once: the observation at t = 0;
                                         # it also freezes /home/user/submission for the program's grading
    obs = ep.step([0, 0, -0.02, 0, 0, 0, 0, 0])   # one control step -> the next observation (ep.done at the end)
    obs = ep.observe()                   # the latest observation again; nothing is simulated
    ep.finish()                          # ends the episode now (it is then scored as it stands)
    ep.close()                           # closes the connection only; the episode stays where it is

The episode lives in the service, not in your connection: close it, crash, or run the next script, and a new
GradedEpisode() continues from the same step (observe() returns the observation you have not acted on yet).
The simulation advances only inside step(): between calls the world waits.

From a shell:  /opt/venv/bin/python3 /home/user/graded_episode.py status
               /opt/venv/bin/python3 /home/user/graded_episode.py start      (once; prints t and the request)
               /opt/venv/bin/python3 /home/user/graded_episode.py finish --yes
"""

import json
import socket
import sys

SOCKET = "/run/arb-online/online.sock"


class GradedEpisodeError(Exception):
    """A refused request: `code` (e.g. "busy", "already_started", "bad_action", "episode_over",
    "simulator_unavailable") and the service's fixed message."""

    def __init__(self, code, message=""):
        Exception.__init__(self, "%s: %s" % (code, message))
        self.code, self.message = code, message


class GradedEpisode(object):
    """One connection to the graded episode service. `timeout` bounds every call except start(). `done` is
    true once a reply has said the episode is over (a step, observe(), status() or finish())."""

    def __init__(self, path=SOCKET, timeout=300.0):
        self.path, self.timeout = path, float(timeout)
        self.done = False
        self.t = None
        self.program_snapshot = None
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect(path)
        self._rd = self._sock.makefile("rb")

    def _call(self, msg, timeout=None):
        self._sock.settimeout(self.timeout if timeout is None else float(timeout))
        self._sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
        line = self._rd.readline()
        if not line:
            raise GradedEpisodeError("disconnected", "the service closed the connection")
        reply = json.loads(line.decode("utf-8"))
        if not reply.get("ok"):
            raise GradedEpisodeError(reply.get("error", "error"), reply.get("message", ""))
        if "done" in reply:
            self.done = bool(reply["done"])
        if "t" in reply:
            self.t = reply["t"]
        return reply

    def status(self):
        """started, ended, steps (the episode's step index t), horizon, sim_time_s, prep_left_s (time left to
        start the episode), graded_left_s (time left in the graded window once started)."""
        r = self._call({"op": "status"})
        self.done = bool(r["ended"])
        return {k: r[k] for k in ("started", "ended", "steps", "horizon", "sim_time_s", "prep_left_s",
                                  "graded_left_s")}

    def start(self, token=None, timeout=4800.0):
        """Start the graded episode and return its first observation (t = 0). Once per session: the start is
        charged before anything is simulated and never refunded. It launches the episode's simulator, which
        takes from seconds to a few minutes; if the simulator cannot start, nothing is charged
        (GradedEpisodeError "simulator_unavailable"). Just before the charge it freezes /home/user/submission:
        that copy is the program graded on the hidden episodes; `program_snapshot` then holds its status
        ("ok", or "too_large": nothing kept), files, bytes and skipped entries (links, other accounts' files).
        If the copy fails, nothing is charged (GradedEpisodeError "snapshot_failed"). If this call is cut
        off, the start still completes in the service: check status() and continue with observe(). `token`
        is for the task's maintainers; leave it out."""
        msg = {"op": "start"}
        if token is not None:
            msg["token"] = str(token)
        reply = self._call(msg, timeout=timeout)
        self.program_snapshot = reply.get("program_snapshot")
        return reply["obs"]

    def step(self, action):
        """Apply one action ([dx, dy, dz, droll, dpitch, dyaw, grip, mark], plus an optional ninth number: the
        hi-res camera bitmask) for one control step; return the next observation. A malformed action is
        refused (GradedEpisodeError "bad_action") and nothing is simulated."""
        return self._call({"op": "step", "action": [float(v) for v in action]})["obs"]

    def observe(self):
        """The latest observation again (the one the next step acts on). Nothing is simulated."""
        return self._call({"op": "observe"})["obs"]

    def finish(self):
        """End the episode now. Nothing is simulated afterwards; it is scored on the state it reached (scaled
        by the fraction of the 20000 steps that ran, if it ends early). Final."""
        r = self._call({"op": "finish"})
        self.done = True
        return {"done": True, "steps": r.get("steps")}

    def close(self):
        try:
            self._rd.close()
        finally:
            self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _main(argv):
    cmd = argv[1] if len(argv) > 1 else "status"
    with GradedEpisode() as ep:
        if cmd == "status":
            print(json.dumps(ep.status()))
        elif cmd == "start":
            obs = ep.start()
            print(json.dumps({"t": obs.get("t"), "request": obs.get("request"), "ee_pose": obs.get("ee_pose"),
                              "program_snapshot": ep.program_snapshot}))
        elif cmd == "finish":
            if "--yes" not in argv:
                print("finish ends the graded episode for good; run it again with --yes", file=sys.stderr)
                return 2
            print(json.dumps(ep.finish()))
        else:
            print("usage: graded_episode.py status | start | finish --yes", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    try:
        sys.exit(_main(sys.argv))
    except GradedEpisodeError as exc:
        print("graded episode: %s" % exc, file=sys.stderr)
        sys.exit(1)
    except (OSError, socket.error) as exc:
        print("graded episode service unreachable at %s: %s" % (SOCKET, exc), file=sys.stderr)
        sys.exit(1)
