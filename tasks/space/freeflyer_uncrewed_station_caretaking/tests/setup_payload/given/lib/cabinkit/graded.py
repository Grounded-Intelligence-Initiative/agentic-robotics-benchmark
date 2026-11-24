"""cabinkit.graded -- the robot of the graded episode, which you fly yourself during your session (the task
instruction, "The graded episode").

    import os, sys
    sys.path.insert(0, os.path.expanduser("~/given/lib"))
    from cabinkit.graded import connect_graded

    robot = connect_graded()           # connects to the graded episode's service; starts nothing
    robot.graded_status()              # where the episode stands (any time)
    robot.graded_start()               # freezes your program, then starts THE graded episode: once per session
    m = robot.mission()                # from here on, the calls of robot_api.md, as a program makes them
    robot.undock()
    robot.step(5.0)
    robot.close()                      # closes this connection only: the episode stays exactly where it is
    ...
    robot = connect_graded()           # a later script resumes the same episode
    robot.graded_finish()              # ends the episode for good (the run's end of robot_api.md, section 6)

The robot is ``cabinkit.client.Robot`` with three more calls, so a program's ``solve(robot)`` runs on it unchanged:
``solve(connect_graded())``.  A reply that carries an error raises ``cabinkit.client.CellError`` (``.code``: e.g.
``not_started``, ``already_started``, ``prep_window_closed``, ``episode_over``, ``busy``, ``forbidden``).
"""

import json
import os

from .client import CellError, Robot

__all__ = ["GradedRobot", "connect_graded", "GRADED_SOCKET"]

GRADED_SOCKET = "/run/arb-graded/graded.sock"


class GradedRobot(Robot):
    """The graded episode's robot: the robot API's calls plus graded_status(), graded_start() and graded_finish().
    The episode lives in the graded episode's service, not in this connection: close() (or the end of this process)
    leaves it exactly where it is and the next connect_graded() resumes it."""

    def graded_status(self):
        """Where the graded episode stands, at any time: started, ended, end (how it ended), sim_time and
        sim_time_max (mission seconds), steps, colour_frames_used / _max, depth_frames_used / _max, captures_used /
        _max, prep_window_left_s, graded_window_left_s (seconds; None where a window does not apply) and
        submission_frozen ("ok", "too_large" or None)."""
        return self._call("graded_status")

    def graded_start(self):
        """Freeze the program (the copy of /home/user/submission/), then start the graded episode: once per session,
        never undone. Returns graded_status()."""
        return self._call("graded_start")

    def graded_finish(self):
        """End the graded episode for good, as a run ends when solve returns: the firmware stops the robot and holds
        it (a docked robot stays docked), the station runs on for 60 s (about a minute of wall clock before this call
        returns), then the episode is judged. Returns graded_status()."""
        return self._call("graded_finish")


def connect_graded(path=None, timeout=None):
    """The graded episode's robot over its Unix stream socket: `path` (default $ARB_GRADED_SOCKET or
    /run/arb-graded/graded.sock). Connecting starts nothing (graded_start does). `timeout` (s, optional) bounds every
    socket operation; leave it unset for calls that let much simulated time pass and for graded_finish. One connection
    at a time: while another one is open this raises CellError("busy"). Returns a GradedRobot; close() it when the
    script is done with it."""
    import socket
    if path is None:
        path = os.environ.get("ARB_GRADED_SOCKET", GRADED_SOCKET)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        if timeout is not None:
            s.settimeout(float(timeout))
        s.connect(path)
    except OSError as e:
        s.close()
        raise CellError("unreachable", "cannot reach the graded episode's service at %s: %s" % (path, e))
    rf = s.makefile("r", encoding="utf-8", newline="\n")
    wf = s.makefile("w", encoding="utf-8", newline="\n")

    def send(text):
        wf.write(text)
        wf.flush()

    def recv():
        return rf.readline()

    def close():
        for f in (wf, rf):
            try:
                f.close()
            except Exception:  # noqa: BLE001
                pass
        s.close()

    # the service answers a connection it refuses (busy, forbidden) with one error line: a status request first
    try:
        send(json.dumps({"id": 0, "method": "graded_status", "params": {}}) + "\n")
        line = recv()
    except OSError as e:
        close()
        raise CellError("closed", "the connection to the graded episode's service broke (%s)" % e.__class__.__name__)
    try:
        reply = json.loads(line) if line else None
    except ValueError:
        reply = None
    if not isinstance(reply, dict) or "error" in reply or reply.get("id") != 0:
        close()
        if isinstance(reply, dict) and "error" in reply:
            raise CellError(str(reply["error"]), str(reply.get("detail") or ""))
        raise CellError("closed" if not line else "bad_reply", "the graded episode's service did not answer")
    robot = GradedRobot(send, recv)
    robot._close = close
    return robot
