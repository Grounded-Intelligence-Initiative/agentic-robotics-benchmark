"""Start the cell's process (`server.py`) and talk to it, with a deadline on every reply. Standard library only.

Used by the grader (as root: the cell then runs as its own unprivileged account), by the commissioning service
(already that account) and by maintainers' tools (as themselves).
"""

import json
import os
import select
import signal
import subprocess
import sys
import time


class CellError(Exception):
    """The cell's process failed or did not answer in time: the cell's fault, never the program's under test."""


class CellProcess(object):
    def __init__(self, grader=False, env=None, python=None, user=None, cwd=None, stderr=None, start_timeout=900.0):
        here = os.path.dirname(os.path.abspath(__file__))
        cmd = [python or sys.executable, "-I", "-s", os.path.join(here, "server.py")] + (["--grader"] if grader else [])
        if user:
            cmd = ["/usr/sbin/runuser", "-u", user, "--"] + cmd
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr, env=env, cwd=cwd,
                                     start_new_session=True)
        self.buf = b""
        hello = self._read(start_timeout)
        if not hello.get("ready"):
            self.kill()
            raise CellError("the cell did not start: %s" % str(hello)[:1500])
        self.hello = hello

    def _read(self, timeout):
        deadline = time.monotonic() + timeout
        fd = self.proc.stdout.fileno()
        while b"\n" not in self.buf:
            left = deadline - time.monotonic()
            if left <= 0:
                raise CellError("the cell did not answer within %.0f s" % timeout)
            ready, _, _ = select.select([fd], [], [], min(left, 5.0))
            if ready:
                chunk = os.read(fd, 1 << 20)
                if not chunk:
                    raise CellError("the cell's process ended (exit %s)" % self.proc.poll())
                self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def request(self, msg, timeout=600.0):
        try:
            self.proc.stdin.write((json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8"))
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise CellError("the cell's process is gone: %s" % exc)
        reply = self._read(timeout)
        if "fatal" in reply:
            raise CellError("the cell failed: %s" % reply["fatal"])
        return reply

    def close(self):
        try:
            self.request({"cmd": "close"}, timeout=30.0)
        except Exception:  # noqa: BLE001
            pass
        self.kill()

    def kill(self):
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass
