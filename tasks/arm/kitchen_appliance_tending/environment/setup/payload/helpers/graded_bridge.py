"""Run a wire-protocol controller in the GRADED episode (optional boilerplate).

    /opt/venv/bin/python3 /home/user/helpers/graded_bridge.py --submission /home/user/submission \
        --log /home/user/graded.jsonl [--resume] [--steps N] [--finish]

It starts `bash run.sh` in the submission directory as the practice grader does (READY, reset, act, close; the
same observation dict), then starts the graded episode (graded_start: once per session, never refunded) or, with
--resume, continues the one already started from its latest observation (graded_observe; the controller then gets
a reset and its first `act` carries that observation's `t`). It feeds the controller the episode's observations
step by step until the episode ends or --steps steps were sent, and writes one JSON line per step
({"t", "obs", "action"}) to --log. If it stops before the episode's end (--steps, the controller failed), the
episode stays where it is, to be continued; with --finish it is ended then (graded_finish). It reports no score.
Unlike the practice grader it sends every step once (no repeated-t queries) and does not apply the socket
restriction.
"""

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # the agent home
from labclient import LabClient  # noqa: E402

STEP_TIMEOUT = 30.0
READY_TIMEOUT = 120.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default="/home/user/submission")
    ap.add_argument("--log", required=True)
    ap.add_argument("--resume", action="store_true", help="continue the graded episode already started")
    ap.add_argument("--steps", type=int, default=None, help="send at most this many steps")
    ap.add_argument("--finish", action="store_true", help="end the episode if this run stops before its end")
    a = ap.parse_args()
    proc = subprocess.Popen(["bash", "run.sh"], cwd=a.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            universal_newlines=True, bufsize=1, start_new_session=True)
    lines = queue.Queue()

    def pump():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)
    threading.Thread(target=pump, daemon=True).start()

    def read(timeout):
        while True:
            line = lines.get(timeout=timeout)
            if line is None:
                raise RuntimeError("controller closed its stdout")
            line = line.strip()
            if line:
                return line

    def request(msg):
        proc.stdin.write(json.dumps(msg) + "\n")
        proc.stdin.flush()
        return json.loads(read(STEP_TIMEOUT))

    n, done, status = 0, False, None
    with LabClient() as lab:
        try:
            if read(READY_TIMEOUT) != "READY":
                raise RuntimeError("controller did not print READY first")
            request({"type": "reset"})
            with open(a.log, "a") as log:
                if a.resume:
                    obs, done = lab.graded_observe()
                else:
                    obs = lab.graded_start()
                while not done and (a.steps is None or n < a.steps):
                    t = int(obs["t"])
                    reply = request({"type": "act", "t": t, "obs": obs})
                    if "action" not in reply:
                        raise RuntimeError("controller error: {}".format(reply.get("error", "no action")))
                    action = reply["action"]
                    log.write(json.dumps({"t": t, "obs": obs, "action": action}) + "\n")
                    obs, done = lab.graded_step(action)
                    n += 1
                log.write(json.dumps({"t": int(obs["t"]), "obs": obs, "action": None}) + "\n")
            request({"type": "close"})
        finally:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
            try:
                status = lab.graded_status()
                if a.finish and status["started"] and not status["ended"]:
                    status = lab.graded_finish()
            except Exception as exc:  # noqa: BLE001 - report what is known
                print("graded_bridge: status: {}".format(exc), file=sys.stderr)
    print(json.dumps({"steps_sent": n, "done": done, "status": status, "log": a.log}))


if __name__ == "__main__":
    main()
