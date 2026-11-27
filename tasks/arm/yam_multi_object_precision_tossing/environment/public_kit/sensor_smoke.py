"""Check controller stdio against one prerecorded RGB-D observation."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import queue
import select
import subprocess
import threading
import time

from rgbd_client import decode_camera

STEP_TIMEOUT_S = 15
READY_TIMEOUT_S = 4 * STEP_TIMEOUT_S
MAX_PRE_READY_LINES = 200
MAX_BLANK_LINES = 1000


class WireError(RuntimeError):
    """Your solver broke the protocol (deadline, non-JSON, no action)."""


class Wire(object):
    """Your `run.sh`, spoken to over stdio exactly as the real grader does."""

    def __init__(self, submission):
        self.proc = subprocess.Popen(["bash", str(submission)], cwd=submission.parent,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1, start_new_session=True)
        self._lines = queue.Queue()
        self._closed = False
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()

    def _pump(self):
        for line in self.proc.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _readline(self, deadline, budget):
        remaining = deadline - time.monotonic()
        try:
            line = self._lines.get(timeout=max(0.0, remaining))
        except queue.Empty:
            raise WireError("no reply within {:.0f}s — the real grader aborts the episode "
                            "here".format(budget))
        if line is None:
            raise WireError("your solver closed its stdout (did it crash? check stderr)")
        return line

    def await_ready(self):
        deadline = time.monotonic() + READY_TIMEOUT_S
        for _ in range(MAX_PRE_READY_LINES):
            if self._readline(deadline, READY_TIMEOUT_S).strip() == "READY":
                return
        raise WireError("your solver never printed READY ({} lines of other output "
                        "seen)".format(MAX_PRE_READY_LINES))

    def _send(self, text, deadline, budget):
        # The same deadline covers writing the request and receiving the complete reply.
        try:
            fd = self.proc.stdin.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except (IOError, OSError, ValueError):
            raise WireError("your solver's stdin is closed")
        view = memoryview(text.encode("utf-8"))
        try:
            while len(view):
                try:
                    view = view[os.write(fd, view):]
                    continue
                except BlockingIOError:
                    pass
                except (IOError, OSError):
                    raise WireError("your solver's stdin is closed")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise WireError("your solver did not read the request within {:.0f}s".format(budget))
                try:
                    select.select([], [fd], [], min(remaining, 1.0))
                except (IOError, OSError, ValueError):
                    raise WireError("your solver's stdin is closed")
        finally:
            try:
                fcntl.fcntl(fd, fcntl.F_SETFL, flags)
            except (IOError, OSError):
                pass

    def request(self, message):
        text = json.dumps(message) + "\n"
        deadline = time.monotonic() + STEP_TIMEOUT_S
        self._send(text, deadline, STEP_TIMEOUT_S)
        for _ in range(MAX_BLANK_LINES):
            line = self._readline(deadline, STEP_TIMEOUT_S).strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except (ValueError, RecursionError):
                raise WireError("non-JSON on stdout: {!r} — stdout is the wire; send "
                                "diagnostics to stderr".format(line[:120]))
            if not isinstance(reply, dict):
                raise WireError("reply is not a JSON object: {!r}".format(line[:120]))
            return reply
        raise WireError("your solver wrote {} blank lines without a reply".format(MAX_BLANK_LINES))

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("solver error at t={}: {}".format(t, reply.get("error", reply)))
        return reply["action"]

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.proc.stdin.close()
        except (OSError, ValueError):
            pass
        try:
            self.proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(self.proc.pid, 9)
        except ProcessLookupError:
            pass
        self.proc.wait()
        self._reader.join(timeout=1)
        self.proc.stdout.close()


def validate_action(action):
    """Check the documented action schema using only fixed robot limits."""
    import math
    if action == {'finish': True}:
        return
    if not isinstance(action, dict) or action.get('arm') not in ('left', 'right'):
        raise WireError('Action must select left or right arm')
    left = {f'left_item{i}' for i in range(6)}
    right = {'right_wood_play_piece'} | {f'right_{kind}_{size}_play_piece' for kind in ('ring', 'slot') for size in ('small', 'medium', 'large')}
    if action.get('item') not in (left if action['arm'] == 'left' else right):
        raise WireError('Action item must use the static ID vocabulary for its arm')
    rows = action.get('waypoints')
    if not isinstance(rows, list) or not 2 <= len(rows) <= 60:
        raise WireError('Expected 2–60 waypoint rows')
    robot = json.loads(Path(__file__).with_name('robot_kinematics.json').read_text())
    previous = -1.
    for index, row in enumerate(rows):
        if not isinstance(row, list) or len(row) != 8 or any(not isinstance(x, (int,float)) or not math.isfinite(x) for x in row):
            raise WireError('Each waypoint needs eight finite numbers')
        if (index == 0 and row[0] != 0) or row[0] <= previous or row[0] > 6:
            raise WireError('Waypoint times must increase from zero through at most six seconds')
        previous = row[0]
        for angle, link in zip(row[1:7], robot['links']):
            if not link['limits'][0]-1e-5 <= angle <= link['limits'][1]+1e-5:
                raise WireError('Joint command exceeds a fixed robot limit')
        if not 0 <= row[7] <= .04695:
            raise WireError('Jaw half-gap is outside [0,0.04695] metres')


def run_smoke(controller, sample):
    observation = json.loads(Path(sample).read_text())
    if set(observation) != {'time', 'arms', 'cameras'}:
        raise ValueError('Sample must contain only time, arms and cameras')
    arm_fields = {'joints','joint_velocity','finger_positions','finger_velocity','reference','jaw_reference'}
    if set(observation['arms']) != {'left','right'} or any(set(arm) != arm_fields for arm in observation['arms'].values()):
        raise ValueError('Sample arm fields do not match the proprioception contract')
    for camera in observation['cameras'].values():
        decode_camera(camera)
    wire = Wire(Path(controller).resolve())
    try:
        wire.await_ready()
        if wire.request({'type':'reset'}).get('ok') is not True:
            raise WireError('Reset requires {"ok":true}')
        action = wire.act(0, observation)
        validate_action(action)
        if wire.request({'type':'close'}).get('ok') is not True:
            raise WireError('Close requires {"ok":true}')
    finally:
        wire.close()
    return dict(protocol='pass', requests=['reset','act','close'], action=action,
                physical_success=None, description='Protocol smoke using one prerecorded observation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', type=Path, default=Path.home()/'submission/run.sh')
    parser.add_argument('--sample', type=Path, default=Path.home()/'sensor_sample.json')
    args = parser.parse_args()
    print(json.dumps(run_smoke(args.controller,args.sample), indent=2))


if __name__ == '__main__':
    main()
