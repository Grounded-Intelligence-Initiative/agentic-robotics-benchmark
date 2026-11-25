"""The graded episode's client (dual mode): one hidden episode of the hand cell, played live from your session.

The graded episode's service runs the grader's own hand cell for one hidden episode, drawn like the hidden episodes of
the program's grading; it serves a unix socket (/run/arb-online/cell.sock) to one client at a time. `connect_graded()`
returns the same proxy that `solve(cell)` gets (arm.assembly_info.client.InfoCell: the calls of
~/given/cell/cell_api.md) plus `status()`, `start()`, `finish()` and `close()`:

    import os, sys
    sys.path.insert(0, os.path.expanduser("~/given/lib"))
    from graded_episode import connect_graded
    from arm.assembly_info.client import decode_frame

    cell = connect_graded()
    print(cell.status())            # started False; start_window_left_s: the time left to start
    cell.start()                    # ONCE per session: freezes /home/user/submission, then the episode begins
    f = cell.get_camera("top")      # the cell's calls, exactly as cell_api.md documents them
    rgb, depth = decode_frame(f)
    cell.set_control("cartesian", 0.35)
    cell.command(tcp_pos=[...], tcp_quat=[1.0, 0.0, 0.0, 0.0], width=0.08)
    cell.step(50)
    cell.close()                    # the episode stays where it is; connect again later to go on
    ...
    cell = connect_graded()
    cell.finish()                   # ends the graded episode: scored as it stands

The episode lives in the service, not in your connection: a script that ends or crashes leaves the episode where it
was, and the next connect_graded() resumes it. One connection at a time: while one is open, another connect_graded()
raises CellError("busy: ..."); a connection that sends nothing for 900 s is closed by the service. Nothing moves in
the cell outside `step` (simulated time only advances there), so your own time between calls costs no simulated time.

A reply carrying an error raises CellError("<code>: <detail>"): the cell's own errors (cell_api.md) and the service's:
not_started (a cell call before start()), already_started, start_window_closed, episode_over (the episode has ended:
by finish(), at 1200 s of simulated time or at the end of its graded window), bad_params, bad_request, line_too_long,
internal_error.

`status()` returns: service, started, ended, end_reason, steps, sim_time, camera_frames, start_window_s,
graded_window_s, start_window_closed, start_window_left_s (before the start), graded_window_left_s (while the episode
runs) and program_frozen (after the freeze: status, files, bytes, at). It never tells the score.

This module is for your session only: the program that the grader runs (assemble.py) talks to its own cell through
`solve(cell)` and cannot use it (the program's run has no such socket, and the grader's network audit counts the
socket import of a submitted copy of this file).
"""

import json
import socket

from arm.assembly_cell.client import CellError
from arm.assembly_info.client import InfoCell

__all__ = ["GradedCell", "CellError", "connect_graded", "SOCKET"]

SOCKET = "/run/arb-online/cell.sock"


class GradedCell(InfoCell):
    """InfoCell over the graded episode's socket, with the service's own calls."""

    def __init__(self, sock, rfile):
        self._sock = sock
        self._rfile = rfile                   # one buffered reader for the whole connection
        InfoCell.__init__(self, self._send_line, self._recv_line)

    def _send_line(self, line):
        self._sock.sendall(line.encode("utf-8"))

    def _recv_line(self):
        return self._rfile.readline().decode("utf-8")

    def _call(self, method, **params):
        self._id += 1
        req = {"id": self._id, "method": method, "params": params}
        self._send(json.dumps(req) + "\n")
        line = self._recv()
        if not line:
            raise CellError("the graded episode's service closed the connection")
        reply = json.loads(line)
        if "error" in reply and reply.get("id") in (None, self._id):   # id None: the request line itself was refused
            detail = reply.get("detail")
            raise CellError("%s: %s" % (reply["error"], detail) if detail else str(reply["error"]))
        if reply.get("id") != self._id:
            raise CellError("reply id mismatch")
        return reply.get("result")

    def status(self):
        """the episode's and the windows' state (never the score)"""
        return self._call("status")

    def start(self):
        """start THE graded episode (once per session; the program in /home/user/submission is frozen first)"""
        return self._call("start")

    def close(self):
        """close this connection; the episode stays where it is"""
        try:
            self._rfile.close()
        finally:
            self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def connect_graded(path=SOCKET, timeout=None):
    """A connection to the graded episode (see the module's text). `timeout` (s) bounds every read and write on the
    socket (None: wait as long as a call takes; a 500-tick step takes some seconds, the episode's first camera frame up
    to a few minutes)."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(timeout)
        sock.connect(path)
        rfile = sock.makefile("rb")
        greeting = rfile.readline()
    except OSError as exc:
        sock.close()
        raise CellError("the graded episode's service is not reachable at %s (%s)" % (path, exc))
    try:
        hello = json.loads(greeting.decode("utf-8"))
    except ValueError:
        sock.close()
        raise CellError("the graded episode's service sent no greeting")
    if not isinstance(hello, dict):
        sock.close()
        raise CellError("the graded episode's service sent no greeting")
    if hello.get("error"):
        sock.close()
        raise CellError("%s: %s" % (hello["error"], hello.get("detail", "")))
    return GradedCell(sock, rfile)
