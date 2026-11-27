#!/usr/bin/env python3
"""Run a controller in one commissioning session, the way it is run when it is graded.

    /opt/venv/bin/python3 ~/commission/run_session.py --row 1 [--submission ~/submission] [--max-steps N]

It starts `bash run.sh` in the submission folder, waits for READY (120 s), sends {"type": "reset"}, then one
{"type": "act", "t", "obs"} per step and applies the reply to the cell, as grading does: one deadline per request
(30 s), fixed before the request is written and covering its writing and the reply; the program's own time summed
against the budget (1200 s, checked before each request); the step limit; and the same repeated steps: now and then
a step is asked twice with the same `t`, once with the observation and once with a copy whose robot readings are
shifted, in a random order, and only the reply to the true one is applied. It ends when the row section ends or the
controller fails, and prints what a commissioning engineer has: why the row section ended, the simulated time, the steps,
the program's own time, how many repeated pairs differed under the grader's rule (`rowcell_client.replies_differ`),
the cell's messages and stops as the controller saw them. It prints no yield and nothing of where anything is: there is none on
this side. A session is counted when it opens (see the interface document).
"""

import argparse
import fcntl
import json
import os
import queue
import random
import select
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "given", "lib"))
import rowcell_client as rc  # noqa: E402

STEP_TIMEOUT = 30.0
READY_TIMEOUT = 120.0
REPEAT_RATE = 0.02
BUDGET_S = 1200.0


class Controller(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1, start_new_session=True)
        self.lines = queue.Queue()
        self.spent = 0.0                 # the program's own time: what this launcher waited for it
        threading.Thread(target=self._pump, daemon=True).start()
        deadline = time.monotonic() + READY_TIMEOUT
        for _ in range(200):
            if self._line(deadline).strip() == "READY":
                return
        raise RuntimeError("no READY from the controller")

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def _line(self, deadline):
        try:
            line = self.lines.get(timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise RuntimeError("the controller missed its deadline")
        if line is None:
            raise RuntimeError("the controller closed its output")
        return line

    def _write(self, text, deadline):
        """As the grader writes a request: without blocking, a chunk at a time as the pipe drains. A request the
        program has not read by the deadline is a missed deadline."""
        data = memoryview(text.encode("utf-8"))
        fd = self.proc.stdin.fileno()
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        try:
            while data:
                left = deadline - time.monotonic()
                if left <= 0 or not select.select([], [fd], [], left)[1]:
                    raise RuntimeError("the controller missed its deadline (it did not read the request)")
                try:
                    data = data[os.write(fd, data[:65536]):]
                except BlockingIOError:
                    continue
                except BrokenPipeError:
                    raise RuntimeError("the controller closed its input")
        finally:
            fcntl.fcntl(fd, fcntl.F_SETFL, flags)

    def ask(self, msg):
        # one deadline for the request, fixed before it is written: reading it counts, as when graded
        t0 = time.monotonic()
        deadline = t0 + STEP_TIMEOUT
        try:
            self._write(json.dumps(msg) + "\n", deadline)
            while True:
                line = self._line(deadline).strip()
                if line:
                    reply = json.loads(line)
                    if not isinstance(reply, dict):
                        raise ValueError("the controller's reply is not a JSON object")
                    return reply
        finally:
            self.spent += time.monotonic() - t0

    def close(self):
        try:
            self._write(json.dumps({"type": "close"}) + "\n", time.monotonic() + 1.0)
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass
        try:
            os.killpg(self.proc.pid, 9)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--row", type=int, required=True)
    ap.add_argument("--submission", default=os.path.join(os.path.expanduser("~"), "submission"))
    ap.add_argument("--max-steps", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=None, help="seed of the repeated steps (default: random)")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    ctl = Controller(a.submission)
    seen = {"events": [], "stops": 0, "repeats": 0, "compared": 0, "differed": 0, "frames": 0}
    t0 = time.time()
    with rc.Session(a.row) as s:
        print("[commission] session %s on practice row %d; %s left" % (s.number, a.row, s.sessions_left), flush=True)
        obs, t, why = s.obs, 0, None
        try:
            ctl.ask({"type": "reset"})
            while not obs["ended"]:
                if ctl.spent > BUDGET_S:
                    why = "the program's own time budget of the row section used up (%.0f s)" % BUDGET_S
                    break
                if t >= a.max_steps:
                    why = "step limit of the row section reached (%d)" % a.max_steps
                    break
                if rng.random() < REPEAT_RATE:
                    other = rc.perturbed(obs, rng)
                    pair = [("true", obs), ("other", other)]
                    rng.shuffle(pair)
                    replies = {name: ctl.ask({"type": "act", "t": t, "obs": o}) for name, o in pair}
                    reply = replies["true"]
                    seen["repeats"] += 1
                    if any("action" not in r for r in replies.values()):
                        why = "the controller's reply has no action: %s" % str(replies)[:200]
                        break
                    if not (rc.is_idle(replies["true"]["action"]) and rc.is_idle(replies["other"]["action"])):
                        try:
                            seen["differed"] += int(rc.replies_differ(replies["true"]["action"], replies["other"]["action"]))
                        except ValueError as exc:
                            why = "an answer that is not a valid action: %s" % exc
                            break
                        seen["compared"] += 1
                else:
                    reply = ctl.ask({"type": "act", "t": t, "obs": obs})
                if "action" not in reply:
                    why = "the controller's reply has no action: %s" % str(reply)[:200]
                    break
                try:
                    obs = s.step(reply["action"])
                except rc.CommissioningError as exc:
                    why = "the cell refused the action: %s" % exc
                    break
                seen["events"] += obs["events"]
                seen["stops"] = obs["status"]["protective_stops"]
                seen["frames"] += len(obs["images"])
                t += 1
        except (RuntimeError, ValueError, OSError) as exc:
            why = str(exc)
        finally:
            ctl.close()
    print("[commission] ended: %s" % (why or obs["ended"]))
    print("[commission] simulated time %.1f s, %d steps, %d frames, %d protective stops, the program's own time %.1f s "
          "of %.0f, wall %.0f s" % (obs["time"], t, seen["frames"], seen["stops"], ctl.spent, BUDGET_S, time.time() - t0))
    print("[commission] %d steps repeated: %d pairs compared (a pair of two idle replies is not), %d of them differed "
          "(when graded, fewer than %g of the compared pairs differing over all the row sections sets the reward to 0)"
          % (seen["repeats"], seen["compared"], seen["differed"], rc.PAIR_PASS_FRACTION))
    refused = [e for e in seen["events"] if "refused" in e]
    print("[commission] the cell's messages: %d, of which refused: %d" % (len(seen["events"]), len(refused)))
    for e in refused[:20]:
        print("   ", e)


if __name__ == "__main__":
    main()
