"""The privileged reference plays the graded episode (the online variant's oracle path; solution/ only).

    /opt/venv/bin/python3 graded_run.py --submission DIR --token-file oracle_token --log FILE

Starts the reference controller (`bash run.sh` in DIR: READY, reset, act, close, the wire the offline grader
speaks), starts the graded episode through the agent-side client (the agent home's labclient.py) with the oracle
token, so that every graded observation carries the unit's `truth` as the offline grader gives a tokened
submission, feeds the controller every observation and applies its actions until the episode ends (its third
request closed), then ends it with graded_finish if the controller stopped before that. One JSON line per step
({"t", "action"}) goes to --log. Prints the final graded status.
"""

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading

HOME = os.environ.get("ALE_HOME", "/home/user")
sys.path.insert(0, HOME)
from labclient import LabClient  # noqa: E402

STEP_TIMEOUT = 30.0
READY_TIMEOUT = 120.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", required=True)
    ap.add_argument("--token-file", required=True)
    ap.add_argument("--log", required=True)
    a = ap.parse_args()
    with open(a.token_file) as fh:
        token = fh.read().strip()
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

    n, done, status, rc = 0, False, None, 0
    with LabClient() as lab:
        try:
            if read(READY_TIMEOUT) != "READY":
                raise RuntimeError("controller did not print READY first")
            request({"type": "reset"})
            obs = lab.request({"op": "graded_start", "token": token})["obs"]
            with open(a.log, "w") as log:
                while not done:
                    t = int(obs["t"])
                    reply = request({"type": "act", "t": t, "obs": obs})
                    if "action" not in reply:
                        raise RuntimeError("controller error: {}".format(reply.get("error", "no action")))
                    log.write(json.dumps({"t": t, "action": reply["action"]}) + "\n")
                    obs, done = lab.graded_step(reply["action"])
                    n += 1
            request({"type": "close"})
        except Exception as exc:  # noqa: BLE001 - reported; the episode is ended below
            print("graded_run: {}: {}".format(type(exc).__name__, exc), file=sys.stderr)
            rc = 1
        finally:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
            status = lab.graded_status()
            if status["started"] and not status["ended"]:
                status = lab.graded_finish()
    print(json.dumps({"steps_sent": n, "done": done, "status": status}))
    return rc


if __name__ == "__main__":
    sys.exit(main())
