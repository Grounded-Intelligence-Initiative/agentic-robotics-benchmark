#!/usr/bin/env python3
"""Run a controller in one commissioning session, the way it is run when it is graded.

    /opt/venv/bin/python3 ~/commission/run_session.py --pack 1 [--submission ~/submission] [--max-steps N]

It starts `bash run.sh` in the submission folder, waits for READY (120 s), sends {"type": "reset"}, then one
{"type": "act", "t", "obs"} per step and applies the reply to the cell, with the same deadline per request (30 s) and
the same repeated steps: now and then a step is asked twice with the same `t`, once with the observation and once
with a copy whose robot readings are shifted, in a random order, and only the reply to the true one is applied. It
ends when the pack ends or the controller fails, and prints what a commissioning engineer has: why the pack ended,
the simulated time, the steps, the cell's messages and stops as the controller saw them. It prints no credit and no
ground truth: there is none on this side. A session is counted when it opens (see the cell's interface document).
"""

import argparse
import json
import os
import queue
import random
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "given", "lib"))
import packcell_client as pc  # noqa: E402

STEP_TIMEOUT = 30.0
READY_TIMEOUT = 120.0
REPEAT_RATE = 0.02


class Controller(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1, start_new_session=True)
        self.lines = queue.Queue()
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

    def ask(self, msg):
        deadline = time.monotonic() + STEP_TIMEOUT
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        while True:
            line = self._line(deadline).strip()
            if line:
                return json.loads(line)

    def close(self):
        try:
            self.proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass
        try:
            os.killpg(self.proc.pid, 9)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", type=int, required=True)
    ap.add_argument("--submission", default=os.path.join(os.path.expanduser("~"), "submission"))
    ap.add_argument("--max-steps", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=None, help="seed of the repeated steps (default: random)")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    ctl = Controller(a.submission)
    seen = {"events": [], "stops": 0, "repeats": 0, "frames": 0}
    t0 = time.time()
    with pc.Session(a.pack) as s:
        print("[commission] session %s on development pack %d; %s left" % (s.number, a.pack, s.sessions_left), flush=True)
        obs, t, why = s.obs, 0, None
        try:
            ctl.ask({"type": "reset"})
            while not obs["ended"] and t < a.max_steps:
                if rng.random() < REPEAT_RATE:
                    other = pc.perturbed(obs, rng)
                    pair = [("true", obs), ("other", other)]
                    rng.shuffle(pair)
                    replies = {name: ctl.ask({"type": "act", "t": t, "obs": o}) for name, o in pair}
                    reply = replies["true"]
                    seen["repeats"] += 1
                else:
                    reply = ctl.ask({"type": "act", "t": t, "obs": obs})
                if "action" not in reply:
                    why = "the controller's reply has no action: %s" % str(reply)[:200]
                    break
                try:
                    obs = s.step(reply["action"])
                except pc.CommissioningError as exc:
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
    print("[commission] ended: %s" % (why or obs["ended"] or "step limit of this launcher"))
    print("[commission] simulated time %.1f s, %d steps, %d repeated, %d frames, %d protective stops, wall %.0f s"
          % (obs["time"], t, seen["repeats"], seen["frames"], seen["stops"], time.time() - t0))
    refused = [e for e in seen["events"] if "refused" in e]
    print("[commission] the cell's messages: %d, of which refused: %d" % (len(seen["events"]), len(refused)))
    for e in refused[:20]:
        print("   ", e)


if __name__ == "__main__":
    main()
