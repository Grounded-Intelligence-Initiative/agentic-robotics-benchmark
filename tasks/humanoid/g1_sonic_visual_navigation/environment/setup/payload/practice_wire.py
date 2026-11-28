"""Bounded, line-oriented process interface for a public practice controller."""
from contextlib import contextmanager, suppress
import json
import os
import select
import signal
import subprocess
import time


class WireError(Exception):
    """A solver missed its deadline or returned an unusable reply."""


class Wire:
    def __init__(self, process):
        self.input = process.stdin.fileno()
        self.output = process.stdout.fileno()
        os.set_blocking(self.input, False)
        os.set_blocking(self.output, False)
        self.buffer = bytearray()

    @staticmethod
    def wait(read, write, deadline):
        remaining = deadline - time.monotonic()
        if remaining > 0:
            readable, writable, _ = select.select(read, write, [], remaining)
            if readable or writable:
                return
        raise WireError("solver exceeded its deadline")

    def line(self, deadline):
        while b"\n" not in self.buffer:
            self.wait([self.output], [], deadline)
            data = os.read(self.output, 65536)
            if not data:
                raise WireError("solver closed stdout")
            self.buffer.extend(data)
            if b"\n" not in self.buffer and len(self.buffer) > 4 * 65536:
                raise WireError("solver reply exceeds 65,536 characters")
        line, _, self.buffer = self.buffer.partition(b"\n")
        try:
            decoded = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise WireError("invalid UTF-8 on solver stdout") from exc
        if len(decoded) + 1 > 65536:
            raise WireError("solver reply exceeds 65,536 characters")
        return line.strip()

    def ready(self):
        deadline = time.monotonic() + 60
        for _ in range(200):
            if self.line(deadline) == b"READY":
                return
        raise WireError("solver never printed READY")

    def request(self, message):
        deadline = time.monotonic() + 15
        payload = json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n"
        remaining = memoryview(payload.encode("utf-8"))
        try:
            while remaining:
                self.wait([], [self.input], deadline)
                remaining = remaining[os.write(self.input, remaining):]
        except BrokenPipeError as exc:
            raise WireError("solver closed stdin") from exc
        for _ in range(1000):
            line = self.line(deadline)
            if line:
                try:
                    reply = json.loads(line)
                except (ValueError, RecursionError) as exc:
                    raise WireError("invalid solver JSON on stdout") from exc
                if isinstance(reply, dict):
                    return reply
                raise WireError("solver reply must be a JSON object")
        raise WireError("solver produced too many blank lines")


@contextmanager
def solver(directory):
    directory = directory.resolve()
    if not (directory / "run.sh").is_file():
        raise ValueError(f"missing submission entry point {directory / 'run.sh'}")
    with subprocess.Popen(
        ["bash", "run.sh"], cwd=directory, stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, bufsize=0, start_new_session=True,
    ) as process:
        try:
            wire = Wire(process)
            wire.ready()
            ack = wire.request({"type": "reset"})
            if set(ack) != {"ok"} or ack["ok"] is not True:
                raise WireError("reset requires a true acknowledgment")
            yield wire
            wire.request({"type": "close"})
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=1)
        finally:
            # A launcher can exit before its helpers, including on a broken reply.
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
