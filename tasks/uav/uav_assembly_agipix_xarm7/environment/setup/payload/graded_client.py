"""The graded workcell you drive yourself in this session: `connect_graded()` connects to the cell service that holds it
(a unix socket, GRADED_SOCKET) and returns a `GradedCellClient`, the workcell connection of cell_client.py (describe,
get_state, set_control, step, get_camera, finish, exactly as the instruction documents them) with start(), status() and
close() besides. A program's solve(cell) runs on it as solve(connect_graded()).

    import sys; sys.path.insert(0, "/home/user")
    from graded_client import connect_graded

    cell = connect_graded()
    cell.status()             # started / ended, steps, sim_time, camera_frames, the time left in each window
    cell.start()              # once per session: freezes your program (the instruction), then starts THE graded episode
    frame = cell.get_camera("top")
    cell.set_control(q=[...], gripper_width_m=0.06)
    cell.step(ticks=100)
    cell.close()              # disconnects; the episode stays exactly where it is
    ...                       # a later script: cell = connect_graded() and go on
    cell.finish()             # ends the graded episode: the workcell is scored as it stands

One client at a time: a second connection raises RuntimeError("busy: ..."). The episode lives in the service, not in
the connection: close() (or the end of your script) leaves it where it is. A connection that sends nothing for 15
minutes is closed by the service; the client then connects again by itself on its next call. Before start() the
workcell's calls raise RuntimeError("not_started: ..."), after the episode's end RuntimeError("episode_over: ...")
(finish() then answers {"done": True}).

This module only reaches the graded episode's service (it opens a unix socket): your program's graded runs on fresh
workcells hand solve(cell) its own connection and have no such service, so do not call connect_graded() from solve().
"""

import json
import socket

from cell_client import CellClient

__all__ = ["GradedCellClient", "connect_graded", "GRADED_SOCKET"]

# the graded episode's cell service (its directory and the socket belong to the service; you can connect, nothing more)
GRADED_SOCKET = "/run/arb-online/cell.sock"


class _Lines:
    """The writer / reader pair CellClient expects, over the service's socket; it connects again by itself when the
    service has closed an idle connection."""

    def __init__(self, owner):
        self.owner = owner

    def write(self, text):
        self.owner._send_text(text)

    def flush(self):
        pass

    def readline(self):
        return self.owner._read_line()


class GradedCellClient(CellClient):
    """This session's graded workcell, through its cell service (connect_graded()). The calls of CellClient go to the
    graded episode's workcell; besides them:

      start()   starts the graded episode: once per session, never undone (later calls raise `already_started`);
      status()  {started, ended, end_reason, steps, sim_time, camera_frames, start_window_left_s,
                 graded_window_left_s, program_frozen, ...}: never the episode's score;
      close()   disconnects; the episode stays where it is (connect_graded() again to go on)."""

    def __init__(self, path=GRADED_SOCKET):
        self._path = str(path)
        self._sock = None
        self._rfile = None
        self.service = None
        lines = _Lines(self)
        CellClient.__init__(self, lines, lines)
        self._connect()

    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._path)
        except OSError as e:
            s.close()
            raise RuntimeError("unavailable: cannot connect to the graded episode's cell service at %s (%s)" % (
                self._path, e.__class__.__name__))
        self._sock = s
        self._rfile = s.makefile("r", encoding="utf-8", newline="\n")
        line = self._rfile.readline()
        try:
            hello = json.loads(line) if line else None
        except ValueError:
            hello = None
        if not isinstance(hello, dict) or "error" in hello or "service" not in hello:
            self._disconnect()
            if isinstance(hello, dict) and "error" in hello:
                raise RuntimeError("%s: %s" % (hello["error"], hello.get("detail") or ""))
            raise RuntimeError("bad_reply: the service did not greet")
        self.service = hello["service"]

    def _closed_by_peer(self):
        """True when the service has closed this connection (an idle one): a pending end of file."""
        try:
            return self._sock.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT) == b""
        except (BlockingIOError, InterruptedError):
            return False
        except OSError:
            return True

    def _send_text(self, text):
        if self._sock is not None and self._closed_by_peer():
            self._disconnect()
        if self._sock is None:
            self._connect()
        self._sock.sendall(text.encode("utf-8"))

    def _read_line(self):
        line = self._rfile.readline() if self._rfile is not None else ""
        if not line:
            self._disconnect()
        return line

    def _disconnect(self):
        for f in (self._rfile, self._sock):
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass
        self._rfile = self._sock = None

    def start(self):
        """Start the graded episode (once per session; before the start is charged the program in /home/user/submission
        is frozen: that copy is the program graded, see the instruction)."""
        return self._call("start")

    def status(self):
        """started / ended (and why), steps, sim_time, camera_frames, the time left in each window (seconds)."""
        return self._call("status")

    def close(self):
        """Disconnect; the graded episode stays where it is."""
        self._disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def connect_graded(path=GRADED_SOCKET):
    """The graded episode's workcell (a GradedCellClient), connected to its service. Raises RuntimeError `busy` while
    another client is connected, `unavailable` when the service cannot be reached."""
    return GradedCellClient(path)
