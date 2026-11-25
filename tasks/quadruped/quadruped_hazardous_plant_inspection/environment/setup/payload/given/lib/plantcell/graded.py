"""The graded episode's connection: this session's one graded mission, which you drive yourself, live, through the
same robot calls a program makes (robot_api.md, section 12).

    import sys
    sys.path.insert(0, "/home/user/given/lib")
    from plantcell.graded import connect_graded

    robot = connect_graded()            # a GradedRobot: the plantcell.client.Robot proxy on the graded mission
    print(robot.status())               # started / ended, steps, sim_time, frames_used, the time left in each window
    robot.start()                       # ONCE per session: freezes /home/user/submission, then the mission begins
    docs = robot.mission()
    robot.set_mode("walk"); robot.walk(0.3, 0.0, 0.0)
    r = robot.step(100)                 # 1 s of simulated time; r["sim_time"], r["done"], r["events"]
    robot.close()                       # disconnects: the mission stays exactly where it is

A later script connects again with connect_graded() and goes on from there; robot.finish() ends the graded mission.
A program's solve(robot) runs on it unchanged: solve(connect_graded()) (after start()). Mind that a program which
calls finish() at its end ends the graded mission there.

The service holds the mission, not the connection: a closed connection, or a script that ends or crashes, leaves it
where it is, and simulated time passes only inside step(). One client at a time: a second connection gets
CellError `busy`. A connection that sends nothing for 15 minutes is closed by the service; the next call on this
proxy then connects again by itself.
"""

import json
import socket

from plantcell.client import CellError, Robot

__all__ = ["GradedRobot", "connect_graded", "GRADED_SOCKET", "CellError"]

# the graded episode's service (its directory and the socket belong to the service; you can connect, nothing more)
GRADED_SOCKET = "/run/plantgrade/graded.sock"


class GradedRobot(Robot):
    """This session's graded mission, through its service (connect_graded()). The calls of Robot (describe, mission,
    get_state, the sensors, the motion calls, step, the reports, finish) go to the graded mission; besides them:

      start()   starts the graded mission: once per session, never undone (a second call answers `already_started`);
                the program in /home/user/submission is frozen first (the instruction says what that means);
      status()  {started, ended, end_reason, steps, sim_time, frames_used, start_window_left_s,
                 graded_window_left_s, program_frozen, ...}: never the mission's score;
      close()   disconnects; the mission stays where it is (connect_graded() again to go on).

    Before start() the robot's calls answer `not_started`, after the mission's end `episode_over` (finish() then just
    answers {"ok": true, "ended": true})."""

    def __init__(self, path=GRADED_SOCKET):
        self._path = str(path)
        self._sock = None
        self._rfile = None
        self._reconnect_error = None
        self.service = None
        Robot.__init__(self, self._send_line, self._recv_line)
        self._close = self._disconnect
        self._connect()

    def _call(self, method, **params):
        if self._sock is None:
            self._connect()                 # raises `busy` / `unreachable` as they are
        self._reconnect_error = None
        try:
            return Robot._call(self, method, **params)
        except CellError as e:
            err, self._reconnect_error = self._reconnect_error, None
            if err is not None and e.code == "closed":
                raise err                   # the connection was gone and could not be made again (`busy`, ...)
            raise

    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._path)
        except OSError as e:
            s.close()
            raise CellError("unreachable", "the graded episode's service is not reachable at %s (%s)" % (
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
                raise CellError(str(hello["error"]), str(hello.get("detail") or ""))
            raise CellError("protocol", "the graded episode's service did not greet")
        self.service = hello["service"]

    def _send_line(self, text):
        data = text.encode("utf-8")
        if self._sock is None:
            self._connect()
        try:
            self._sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError):
            # the service had closed this connection (idle): the request never reached it; connect again and send it
            self._disconnect()
            try:
                self._connect()
            except CellError as e:
                self._reconnect_error = e
                raise
            self._sock.sendall(data)

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

    def start(self):
        """Start the graded mission (once per session). Before the start is charged the program in
        /home/user/submission is frozen: that copy is the program graded on the hidden sites."""
        return self._call("start")

    def status(self):
        """started / ended (and why), steps, sim_time, frames_used, the time left in each window (seconds), and what
        was frozen (`program_frozen`)."""
        return self._call("status")

    def close(self):
        """Disconnect; the graded mission stays where it is."""
        self._disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def connect_graded(path=GRADED_SOCKET):
    """The graded mission's robot (a GradedRobot), connected to its service. Raises CellError `busy` while another
    client is connected, `unreachable` when the service cannot be reached."""
    return GradedRobot(path)
