"""Run a controller through one commissioning session, the way the grader will run it.

    /opt/venv/bin/python3 ~/commission/run_session.py dev03 [--controller /home/user/submission]
                          [--save DIR] [--every 10] [--probe-rate 0.06] [--stop-after N]

It starts `bash run.sh` in the controller folder, waits for READY (40 s), sends `reset`, then for every control step
sends `{"type": "act", "t": t, "obs": obs}` and applies the action of the reply (10 s per request). On about
--probe-rate of the steps it asks the step twice with the same t, once with the true observation and once with a
copy the service builds as the grader does (the same frame rendered again with the instruments moved differently over
its last step, the joints unchanged), in a random order, and applies only the true reply: exactly what the grader does.

It uses one of the service's sessions (see `--status`). It prints how the session ended, the controller's reply times
and how many of the repeated requests changed the action, counted by the grader's own rule (`differ`). It does not,
and cannot, tell you how good the view was:
the service gives no ground truth. --save keeps every --every-th frame and a log of joints and actions to look at.
"""

import argparse
import base64
import json
import os
import queue
import select
import random
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "given"))
from endocam_client import Commissioning, CommissioningError  # noqa: E402
from holder_model import clip_command  # noqa: E402

STEP_TIMEOUT_S = 10.0
READY_TIMEOUT_S = 40.0
BUDGET_S_PER_STEP = 0.25


class Wire(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1, start_new_session=True)
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        deadline = time.monotonic() + READY_TIMEOUT_S
        while True:
            line = self._line(deadline, "READY")
            if line.strip() == "READY":
                break

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def _line(self, deadline, what):
        try:
            line = self.lines.get(timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise RuntimeError("the controller missed its {} deadline".format(what))
        if line is None:
            raise RuntimeError("the controller closed its stdout")
        return line

    def _send(self, data, deadline):
        """Write one request without blocking, under the request's own deadline (as the grader does: an observation
        is larger than a pipe holds, and a controller that stops reading must not hang the launcher)."""
        fd = self.proc.stdin.fileno()
        os.set_blocking(fd, False)
        view = memoryview(data)
        while view:
            left = deadline - time.monotonic()
            if left <= 0:
                raise RuntimeError("the controller missed its reply deadline (it did not read its request)")
            _, ready, _ = select.select([], [fd], [], min(left, 1.0))
            if not ready:
                continue
            try:
                view = view[os.write(fd, view):]
            except BlockingIOError:
                continue
            except OSError as exc:
                raise RuntimeError("the controller closed its stdin: {}".format(exc))

    def ask(self, message):
        deadline = time.monotonic() + STEP_TIMEOUT_S        # one deadline for the write and the reply
        self._send((json.dumps(message) + "\n").encode("utf-8"), deadline)
        while True:
            line = self._line(deadline, "reply").strip()
            if line:
                reply = json.loads(line)
                if not isinstance(reply, dict):
                    raise RuntimeError("a reply must be a JSON object")
                return reply

    def close(self):
        try:
            self.ask({"type": "close"})
        except Exception:  # noqa: BLE001
            pass
        try:
            os.killpg(self.proc.pid, 9)
        except OSError:
            pass


def action_of(reply):
    a = reply.get("action")
    if not isinstance(a, list) or len(a) != 4:
        raise RuntimeError("a reply needs \"action\": four numbers, got {!r}".format(reply)[:200])
    return [float(x) for x in a]


def differ(first, second, tolerance=1e-4):
    """The grader's rule for the two replies to one repeated step: True when the two commands, clipped to the
    holder's rate limits as the grader clips them, hold the same count of numbers and at least one differs by more
    than `tolerance`. Two replies that differ only beyond a limit command the same motion and do not differ."""
    a = [float(x) for x in clip_command(first)]
    b = [float(x) for x in clip_command(second)]
    return len(a) == len(b) and max([abs(x - y) for x, y in zip(a, b)] or [0.0]) > tolerance


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("procedure", nargs="?")
    ap.add_argument("--controller", default=os.path.join(os.path.expanduser("~"), "submission"))
    ap.add_argument("--save")
    ap.add_argument("--every", type=int, default=10)
    ap.add_argument("--probe-rate", type=float, default=0.06)
    ap.add_argument("--stop-after", type=int)
    ap.add_argument("--status", action="store_true")
    a = ap.parse_args()
    service = Commissioning()
    if a.status or not a.procedure:
        print(json.dumps(service.status(), indent=1))
        return
    rng = random.Random()
    wire = Wire(a.controller)
    wire.ask({"type": "reset"})
    obs = service.start(a.procedure)
    print("session on {}: {} steps".format(a.procedure, service.steps), flush=True)
    if a.save:
        os.makedirs(a.save, exist_ok=True)
        log = open(os.path.join(a.save, "log.jsonl"), "w")
    times, probes, changed, t, ended = [], 0, 0, 0, "the procedure is complete"
    try:
        while obs is not None:
            copy = service.probe_copy() if rng.random() < a.probe_rate else None
            t0 = time.monotonic()                     # the controller's time only (the copy's render is the service's)
            if copy is not None:
                if rng.random() < 0.5:
                    other = action_of(wire.ask({"type": "act", "t": t, "obs": copy}))
                    action = action_of(wire.ask({"type": "act", "t": t, "obs": obs}))
                else:
                    action = action_of(wire.ask({"type": "act", "t": t, "obs": obs}))
                    other = action_of(wire.ask({"type": "act", "t": t, "obs": copy}))
                probes += 1
                changed += int(differ(action, other))
            else:
                action = action_of(wire.ask({"type": "act", "t": t, "obs": obs}))
            times.append(time.monotonic() - t0)
            if a.save:
                log.write(json.dumps({"t": t, "joints": obs["joints"], "action": action}) + "\n")
                if t % a.every == 0:
                    with open(os.path.join(a.save, "%06d.jpg" % t), "wb") as f:
                        f.write(base64.b64decode(obs["image"]))
            obs = service.step(action)
            t += 1
            if a.stop_after and t >= a.stop_after:
                ended = "stopped after {} steps as asked".format(t)
                break
    except (RuntimeError, ValueError, CommissioningError, OSError) as exc:
        ended = "ended at step {}: {}".format(t, exc)
    finally:
        wire.close()
        service.close()
    n = max(1, len(times))
    print("session over: {}".format(ended))
    print("controller time: {:.3f} s a step on average, {:.3f} s at most; the grader allows {:.2f} s a step on average over the "
          "whole evaluation and {:.0f} s for any one request".format(sum(times) / n, max(times or [0.0]), BUDGET_S_PER_STEP, STEP_TIMEOUT_S))
    print("repeated requests: {} asked, {} changed the action (the grader wants at least a tenth to, over the whole "
          "evaluation)".format(probes, changed))


if __name__ == "__main__":
    main()
