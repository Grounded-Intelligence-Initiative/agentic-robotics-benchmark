#!/usr/bin/env python3
"""Client for the task's lab service (the budgeted "real" environment).

Python 3 standard library only; numpy is used only if you pass numpy arrays or ask for them.

    from labclient import LabClient, LabError
    with LabClient() as lab:
        print(lab.hello())                 # ops, instance ids, public info, limits
        print(lab.budget())                # trials remaining (free)
        obs = lab.reset("p0")              # costs ONE trial; starts the order
        obs, done = lab.step(action)       # free inside the trial (step and time caps apply)

Command line:
    python3 labclient.py hello | budget
    python3 labclient.py reset <instance>     (spends a trial and ends it: see below)
    python3 labclient.py raw '<one JSON request>'

Trials are counted by the lab, not by this client: reconnecting, restarting your program or a
failed step never gives a trial back. Each connection holds at most one trial; a new reset on
the same connection ends the previous trial. A trial ends for good when its connection closes:
every command-line call above is one connection, so `reset` from the command line spends a trial
that nothing can continue. Drive an order from one process that keeps its LabClient open.
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

    def close(self):
        """End the current trial (if any) and the connection."""
        self.__exit__(None, None, None)


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cmd, args = argv[0], argv[1:]
    if (cmd, len(args)) not in (("hello", 0), ("budget", 0), ("reset", 1), ("raw", 1)):
        print(__doc__)                     # --help and anything unknown: no connection is opened
        return 2
    with LabClient() as lab:
        try:
            if cmd == "hello":
                out = lab.hello()
            elif cmd == "budget":
                out = {"remaining": lab.budget()}
            elif cmd == "reset" and len(args) == 1:
                out = lab.request({"op": "reset", "instance": args[0]})
            elif cmd == "raw" and len(args) == 1:
                out = lab.request(json.loads(args[0]))
            else:
                print(__doc__)
                return 2
        except LabError as exc:
            print(json.dumps({"ok": False, "error": exc.code, "message": exc.message}))
            return 1
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
