"""The practice cell: a root-only service that runs cable_env episodes for the agent's development.

Started by setup_runtime.py before the agent's session, from a folder only root can read, so the agent works against
the cell the way it would against a real one: it sends actions and gets observations and camera frames back, and at
the end of an episode the cell's own check of the route. It never hands out the cable's true state.

One request per connection on the Unix socket, one JSON line each way: {"method": m, "params": {...}}. Methods:

  reset   {"session": s, "seed": n}                 a new episode in session s (seed: 0 <= n < 2**63)
  act     {"session": s, "action": {...}}           0.1 s of the cell; returns the next observation and `done`
  observe {"session": s, "shift": [dx, dy]}         the current observation again (optionally of a shifted scene);
                                                    no time passes
  finish  {"session": s}                            end the episode: the cell lets go, settles and checks the route

Each observation names fresh frame files under /run/cable_cell/frames/<session>/ (the previous ones are removed).
At most MAX_SESSIONS sessions; each runs in its own worker thread (a renderer belongs to the thread that made it).
"""

import json
import math
import os
import queue
import re
import shutil
import socketserver
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cable_env as E  # noqa: E402

RUN = "/run/cable_cell"
SOCKET = os.path.join(RUN, "cell.sock")
FRAMES = os.path.join(RUN, "frames")
MAX_SESSIONS = 4
NAME = re.compile(r"^[a-z0-9_]{1,16}$")


class CellError(Exception):
    pass


def check_action(a):
    if not isinstance(a, dict):
        raise CellError("action must be an object")
    try:
        pos = [float(v) for v in a["pos"]]
        yaw = float(a["yaw"])
        grip = float(a["grip"])
    except (KeyError, TypeError, ValueError, OverflowError):
        raise CellError("action needs pos [x, y, z], yaw and grip")
    if len(pos) != 3 or not all(math.isfinite(v) for v in pos + [yaw, grip]):
        raise CellError("action needs pos [x, y, z], yaw and grip as finite numbers")
    done = a.get("done", False)
    return {"pos": pos, "yaw": yaw, "grip": grip, "done": done is True}


class Session(object):
    def __init__(self, name):
        self.name = name
        self.dir = os.path.join(FRAMES, name)
        os.makedirs(self.dir, mode=0o755, exist_ok=True)
        os.chmod(self.dir, 0o755)
        self.env = E.CableRouting()
        self.active = False
        self.done = False
        self.paths = None
        self.jobs = queue.Queue()
        threading.Thread(target=self._work, daemon=True).start()

    def call(self, method, params):
        reply = queue.Queue()
        self.jobs.put((method, params, reply))
        ok, value = reply.get()
        if not ok:
            raise CellError(value)
        return value

    def _work(self):
        while True:
            method, params, reply = self.jobs.get()
            try:
                reply.put((True, getattr(self, "_" + method)(params)))
            except CellError as exc:
                reply.put((False, str(exc)))
            except Exception as exc:  # noqa: BLE001 - the cell must keep serving
                reply.put((False, "cell error: %s" % type(exc).__name__))

    def _frames(self, obs, shift=(0.0, 0.0)):
        if self.paths is not None:
            E.remove_frames(self.paths)
        self.paths, _ = E.write_frames(self.env, self.dir, shift=shift)
        out = E.shift_obs(obs, shift[0], shift[1]) if shift != (0.0, 0.0) else dict(obs)
        out["frames"] = self.paths
        return out

    def _reset(self, p):
        seed = p.get("seed")
        if not isinstance(seed, int) or isinstance(seed, bool) or not 0 <= seed < 2 ** 63:
            raise CellError("seed must be an integer in [0, 2**63)")
        obs = self.env.reset(seed)
        self.active, self.done = True, False
        return {"obs": self._frames(obs)}

    def _act(self, p):
        if not self.active:
            raise CellError("no episode in this session: reset first")
        if self.done:
            raise CellError("the episode is over: finish it")
        obs, done = self.env.step(check_action(p.get("action")))
        if obs is None:
            self.done = True
            return {"obs": None, "done": True}
        self.done = done
        return {"obs": self._frames(obs), "done": done}

    def _observe(self, p):
        if not self.active or self.done:
            raise CellError("no running episode in this session")
        shift = p.get("shift", [0.0, 0.0])
        try:
            dx, dy = float(shift[0]), float(shift[1])
        except (TypeError, ValueError, IndexError):
            raise CellError("shift must be [dx, dy]")
        if not (math.isfinite(dx) and math.isfinite(dy)) or abs(dx) > 0.05 or abs(dy) > 0.05:
            raise CellError("shift must be within 0.05 m")
        return {"obs": self._frames(self.env.observe(), (dx, dy))}

    def _finish(self, p):
        if not self.active:
            raise CellError("no episode in this session")
        res = self.env.finish()
        self.active = False
        if self.paths is not None:
            E.remove_frames(self.paths)
            self.paths = None
        return {"result": {"credit": res["credit"], "routed": res["routed"], "fixtures": res["fixtures"],
                           "mated": res["mated"], "damaged": res["damaged"], "unstable": res["unstable"],
                           "peak_connector_force": round(res["peak_connector_force"], 2),
                           "checks": res["checks"], "steps": self.env.steps}}


class Cell(object):
    def __init__(self):
        self.sessions = {}
        self.lock = threading.Lock()

    def session(self, name):
        if not isinstance(name, str) or not NAME.match(name):
            raise CellError("session must match [a-z0-9_]{1,16}")
        with self.lock:
            if name not in self.sessions:
                if len(self.sessions) >= MAX_SESSIONS:
                    raise CellError("at most %d sessions: %s" % (MAX_SESSIONS, ", ".join(sorted(self.sessions))))
                self.sessions[name] = Session(name)
            return self.sessions[name]


CELL = Cell()


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            req = json.loads(self.rfile.readline(100000))
            method = req.get("method")
            params = req.get("params") or {}
            if method not in ("reset", "act", "observe", "finish") or not isinstance(params, dict):
                raise CellError("unknown method")
            out = CELL.session(params.get("session", "a")).call(method, params)
        except CellError as exc:
            out = {"error": str(exc)}
        except (ValueError, AttributeError):
            out = {"error": "send one JSON object per line"}
        try:
            self.wfile.write((json.dumps(out) + "\n").encode())
        except OSError:
            pass


class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


def main():
    os.makedirs(RUN, mode=0o755, exist_ok=True)
    os.chmod(RUN, 0o755)
    if os.path.isdir(FRAMES):
        shutil.rmtree(FRAMES)
    os.makedirs(FRAMES, mode=0o755)
    os.chmod(FRAMES, 0o755)
    if os.path.exists(SOCKET):
        os.unlink(SOCKET)
    server = Server(SOCKET, Handler)
    os.chmod(SOCKET, 0o666)
    print("cell ready", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
