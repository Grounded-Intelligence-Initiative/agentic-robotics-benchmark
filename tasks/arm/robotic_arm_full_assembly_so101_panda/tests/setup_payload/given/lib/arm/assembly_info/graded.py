"""The graded episode you drive yourself in this session (PUBLIC CODE): its cell through the graded episode's cell
service (a unix socket, GRADED_SOCKET), as the same `InfoCell` proxy your program's solve(cell) gets, plus start(),
status() and close().

    import sys
    sys.path.insert(0, "/home/user/given/lib")
    from arm.assembly_info.graded import connect_graded
    from arm.assembly_info.client import decode_frame

    cell = connect_graded()          # the graded episode's cell: the calls of cell_api.md, plus start / status / close
    print(cell.status())             # started, ended, steps, sim_time, camera_frames, the time left in each window
    cell.start()                     # ONCE per session: freezes your program, then starts THE graded episode
    rgb, depth = decode_frame(cell.get_camera("top"))
    cell.set_control("cartesian", 0.35)
    cell.command(tcp_pos=[...], tcp_quat=[w, x, y, z], width=0.08)
    r = cell.step(20)                # 0.2 s of simulated time; r["sim_time"], r["done"], r["events"]
    cell.close()                     # disconnects: the episode stays exactly where it is
    ...
    cell = connect_graded()          # a later script goes on from there
    cell.finish()                    # ends the graded episode: scored as it stands

One client at a time: a second connection raises CellError("busy: ..."). The episode lives in the service: close() (or
the end or the crash of your script) leaves it where it is, and a later connect_graded() resumes it. A connection that
sends nothing for 15 minutes is closed by the service; this proxy notices that before its next request and connects
again by itself. The service's own errors carry a detail, CellError("<code>: <detail>") (not_started, already_started,
start_window_closed, episode_over, busy, ...); the cell's own replies and errors are those of cell_api.md.
"""
import json
import select
import socket

from .client import CellError, InfoCell, decode_frame

__all__ = ["GradedInfoCell", "connect_graded", "GRADED_SOCKET", "CellError", "decode_frame"]

# the graded episode's cell service (its directory and the socket belong to the service; you can connect, nothing more)
GRADED_SOCKET = "/run/arb-online/cell.sock"


class GradedInfoCell(InfoCell):
    """This session's graded episode, through its cell service (connect_graded()). The calls of InfoCell (describe,
    get_state, set_control, command, step, get_camera, finish) go to the graded episode's cell; besides them:

      start()   starts the graded episode: once per session, never undone (later calls answer `already_started`);
                before the start is charged, the program in /home/user/submission is frozen (the instruction);
      status()  {started, ended, end_reason, steps, sim_time, camera_frames, start_window_left_s,
                 graded_window_left_s, program_frozen, ...}: never the episode's score;
      close()   disconnects; the episode stays where it is (connect_graded() again to go on).

    Before start() the cell's calls answer `not_started`, after the episode's end `episode_over` (finish() then just
    answers {"ok": true})."""

    def __init__(self, path=GRADED_SOCKET):
        self._path = str(path)
        self._sock = None
        self._rfile = None
        self.service = None
        InfoCell.__init__(self, self._send_line, self._recv_line)
        self._connect()

    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._path)
        except OSError as e:
            s.close()
            raise CellError("unavailable: cannot connect to the graded episode's cell service at %s (%s)" % (
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
                raise CellError("%s: %s" % (hello["error"], hello.get("detail", "")))
            raise CellError("bad_reply: the service did not greet")
        self.service = hello["service"]

    def _peer_closed(self):
        """True when the service has closed this connection (an idle connection: the service read nothing of it since)."""
        try:
            readable, _, _ = select.select([self._sock], [], [], 0)
            if readable:
                return self._sock.recv(1, socket.MSG_PEEK) == b""
        except OSError:
            return True
        return False

    def _send_line(self, text):
        if self._sock is not None and self._peer_closed():
            self._disconnect()
        if self._sock is None:
            self._connect()
        self._sock.sendall(text.encode("utf-8"))

    def _recv_line(self):
        return self._rfile.readline() if self._rfile is not None else ""

    def _disconnect(self):
        for f in (self._rfile, self._sock):
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass
        self._rfile = self._sock = None

    def _call(self, method, **params):
        self._id += 1
        req = {"id": self._id, "method": method, "params": params}
        self._send(json.dumps(req) + "\n")
        line = self._recv()
        if not line:
            self._disconnect()
            raise CellError("the cell service closed the connection")
        reply = json.loads(line)
        if reply.get("id") != self._id:
            raise CellError("reply id mismatch")
        if "error" in reply:
            detail = reply.get("detail")
            raise CellError("%s: %s" % (reply["error"], detail) if detail else reply["error"])
        return reply.get("result")

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
    """The graded episode's cell (a GradedInfoCell), connected to its service. Raises CellError `busy` while another
    client is connected, `unavailable` when the service cannot be reached."""
    return GradedInfoCell(path)
