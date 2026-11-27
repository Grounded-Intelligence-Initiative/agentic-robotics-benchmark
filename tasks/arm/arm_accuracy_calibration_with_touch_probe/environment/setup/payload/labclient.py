#!/usr/bin/env python3
"""Client for the task's lab service (the budgeted "real" environment).

Python 3 standard library only; numpy is used only if you pass numpy arrays or ask for them.

    from labclient import LabClient, LabError
    with LabClient() as lab:
        print(lab.hello())                 # ops, unit ids, a session's budget, limits
        print(lab.budget())                # trials remaining (free)
        obs = lab.reset("p1")              # costs ONE trial; opens a measuring session on the unit
        obs, _ = lab.step({"do": "move", "q": [...], "speed": 1.0})
        obs, _ = lab.step({"do": "probe", "q": [...]})
        obs, _ = lab.step({"do": "image"}); picture = decode_image(obs["image"])

Command line:
    python3 labclient.py hello | budget
    python3 labclient.py raw '<one JSON request>'

Trials are counted by the lab, not by this client: reconnecting, restarting your program or a
failed command never gives a trial back. Each connection holds at most one session; a new reset
on the same connection ends the previous one. A session ends for good when its connection
closes: drive a session from one process that keeps its LabClient open.
"""

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


def decode_image(field):
    """The numpy array of a {"dtype": "uint8", "shape": [...], "b64": ...} reply field (obs["image"])."""
    import base64

    import numpy as np

    # a copy: an array of its own, writable (some SciPy routines crash on a read-only buffer view)
    return np.frombuffer(base64.b64decode(field["b64"]), dtype=field["dtype"]).reshape(field["shape"]).copy()


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
    if (cmd, len(args)) not in (("hello", 0), ("budget", 0), ("raw", 1)):
        print(__doc__)                     # --help and anything unknown: no connection is opened
        return 2
    with LabClient() as lab:
        try:
            if cmd == "hello":
                out = lab.hello()
            elif cmd == "budget":
                out = {"remaining": lab.budget()}
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
