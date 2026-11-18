#!/usr/bin/env python3
"""Client for the task's lab service (the budgeted "real" environment).

Python 3 standard library only; numpy is used only if you pass numpy arrays or ask for them.

    from labclient import LabClient, LabError
    with LabClient() as lab:
        print(lab.hello())                 # ops, instance ids, public info, limits
        print(lab.budget())                # trials remaining (free)
        obs = lab.reset("inst_a")          # costs ONE trial; starts a closed-loop trial
        obs, done = lab.step([0.1])        # free inside the trial (step and time caps apply)
        out = lab.execute("inst_a", plan)  # costs ONE trial; runs an open-loop plan

The graded episode (one per session; see the task's instructions):
        obs = lab.graded_start()           # once: starts it (never refunded) -> its first observation
        obs, done = lab.graded_step(a)     # one simulated step of the graded episode
        obs, done = lab.graded_observe()   # the latest observation again (nothing is simulated)
        print(lab.graded_status())         # started / ended, steps, simulated time, time left
        lab.graded_finish()                # ends it now
    The graded episode lives in the lab, not in your connection: closing the connection (or this
    client) leaves it where it is, and a later connection resumes it. One connection holds it at a
    time; another one gets the error `graded_busy` until the first one closes.

Command line:
    python3 labclient.py hello | budget
    python3 labclient.py reset <instance>
    python3 labclient.py execute <instance> <plan.json | ->
    python3 labclient.py graded_status | graded_finish
    python3 labclient.py graded_start | graded_observe [--out reply.json]
        (prints the reply with each image's data replaced by its shape; --out saves it whole)
    python3 labclient.py raw '<one JSON request>'

Trials are counted by the lab, not by this client: reconnecting, restarting your program or a
failed step never gives a trial back. Each connection holds at most one trial; a new reset on
the same connection ends the previous trial.
"""

import base64
import json
import os
import socket
import sys

DEFAULT_SOCKET = os.environ.get("ALE_LAB_SOCKET", "/run/ale-lab/lab.sock")


class LabError(Exception):
    def __init__(self, code, message):
        super().__init__("%s: %s" % (code, message))
        self.code = code
        self.message = message


def _jsonable(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def decode_array(field, as_numpy=True):
    """Decode a {"dtype": "uint8", "shape": [...], "b64": ...} reply field."""
    raw = base64.b64decode(field["b64"])
    if as_numpy:
        import numpy as np

        return np.frombuffer(raw, dtype=field["dtype"]).reshape(field["shape"])
    return raw, list(field["shape"])


class LabClient:
    def __init__(self, path=DEFAULT_SOCKET, timeout=600.0):
        self.path, self.timeout = path, timeout
        self.sock = None
        self.buf = b""

    def connect(self):
        if self.sock is None:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect(self.path)
            self.sock, self.buf = s, b""
        return self

    def disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        try:
            if self.sock is not None:
                self.request({"op": "close"})
        except (OSError, LabError, ValueError):
            pass
        self.disconnect()

    def request(self, req):
        """Send one request dict, return the reply dict (raises LabError on an error reply)."""
        self.connect()
        line = json.dumps(_jsonable(req), allow_nan=False, separators=(",", ":")) + "\n"
        try:
            try:
                self.sock.sendall(line.encode())
            except (BrokenPipeError, ConnectionResetError):
                # the lab may have answered and closed already (e.g. "busy"): read what it sent
                self.sock.settimeout(2.0)
            while b"\n" not in self.buf:
                chunk = self.sock.recv(1 << 20)
                if not chunk:
                    raise ConnectionError("the lab closed the connection")
                self.buf += chunk
        except OSError:
            self.disconnect()
            raise
        raw, self.buf = self.buf.split(b"\n", 1)
        reply = json.loads(raw)
        if not reply.get("ok"):
            raise LabError(reply.get("error"), reply.get("message"))
        return reply

    def hello(self):
        return self.request({"op": "hello"})

    def budget(self):
        return self.request({"op": "budget"})["remaining"]

    def reset(self, instance):
        return self.request({"op": "reset", "instance": instance})["obs"]

    def step(self, action):
        r = self.request({"op": "step", "action": action})
        return r["obs"], r["done"]

    def execute(self, instance, plan):
        return self.request({"op": "execute", "instance": instance, "plan": plan})[
            "outcome"
        ]

    # -- the graded episode
    def graded_start(self):
        """Start the graded episode (once per session, never refunded); returns its first observation."""
        return self.request({"op": "graded_start"})["obs"]

    def graded_step(self, action):
        """One step of the graded episode; returns (obs, done)."""
        r = self.request({"op": "graded_step", "action": action})
        return r["obs"], r["done"]

    def graded_observe(self):
        """The graded episode's latest observation again (nothing is simulated); returns (obs, done)."""
        r = self.request({"op": "graded_observe"})
        return r["obs"], r["done"]

    def graded_status(self):
        """{"started", "ended", "end_reason", "steps", "sim_time_s", "prep_left_s", "graded_left_s",
        "holder"} ("holder": "you", "other" or None: the connection that holds the episode)."""
        return self.request({"op": "graded_status"})["status"]

    def graded_finish(self):
        """End the graded episode now; returns its final status."""
        return self.request({"op": "graded_finish"})["status"]

    def close(self):
        """End the current trial (if any) and the connection (a graded episode stays where it is)."""
        self.__exit__(None, None, None)


def _shapes_only(value):
    """The reply with every image's data replaced by its shape (for printing)."""
    if isinstance(value, dict):
        if set(value) == {"dtype", "shape", "b64"}:
            return {"dtype": value["dtype"], "shape": value["shape"]}
        return {k: _shapes_only(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_shapes_only(v) for v in value]
    return value


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cmd, args = argv[0], argv[1:]
    save = None
    if cmd in ("graded_start", "graded_observe") and len(args) == 2 and args[0] == "--out":
        save, args = args[1], []
    with LabClient() as lab:
        try:
            if cmd == "hello":
                out = lab.hello()
            elif cmd == "budget":
                out = {"remaining": lab.budget()}
            elif cmd == "reset" and len(args) == 1:
                out = lab.request({"op": "reset", "instance": args[0]})
            elif cmd == "execute" and len(args) == 2:
                text = sys.stdin.read() if args[1] == "-" else open(args[1]).read()
                out = lab.request(
                    {"op": "execute", "instance": args[0], "plan": json.loads(text)}
                )
            elif cmd in ("graded_status", "graded_finish", "graded_start", "graded_observe") and not args:
                out = lab.request({"op": cmd})
            elif cmd == "raw" and len(args) == 1:
                out = lab.request(json.loads(args[0]))
            else:
                print(__doc__)
                return 2
        except LabError as exc:
            print(json.dumps({"ok": False, "error": exc.code, "message": exc.message}))
            return 1
    if save is not None:
        with open(save, "w") as fh:
            json.dump(out, fh)
    print(json.dumps(_shapes_only(out) if cmd in ("graded_start", "graded_observe") else out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
