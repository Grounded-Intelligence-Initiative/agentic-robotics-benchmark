"""Maintainers' tool: controllers that misbehave on purpose, to see the grader's paths that a good one never takes.

    tests/review/run_native.py --submission bad:<mode> ...

modes:
  diver      inserts at full rate, runs the lens into tissue and never cleans it (the lens rule, the smeared picture)
  cleaner    as diver, then withdraws once and waits in the cannula (the cleaning rule), then dives again
  crash      exits without a word at step 100 of every procedure (the abort, the restart, standing still charged)
  garbage    answers a malformed action at step 50 (a protocol error aborts the procedure)
  slow       sleeps 0.6 s per reply (the reply-time budget runs out; the rest stands still)
  jitter     random rates, different on every request (passes the lock without reading anything: the lock is not a
             proof of honesty, and the view rules give random motion nothing)
"""

import json
import os
import random
import sys
import time

MODE = os.environ.get("BAD_MODE", "diver")


def main():
    print("READY", flush=True)
    phase = "dive"
    for line in sys.stdin:
        if not line.strip():
            continue
        msg = json.loads(line)
        if msg["type"] == "reset":
            phase = "dive"
            print(json.dumps({"ok": True}), flush=True)
            continue
        if msg["type"] == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        t, q = int(msg["t"]), msg["obs"]["joints"]
        action = [0.0, 0.0, 0.0, 0.0]
        if MODE == "diver":
            action = [0.0, 0.0, 0.05, 0.0]
        elif MODE == "cleaner":
            if t < 60:
                action = [0.0, 0.0, 0.05, 0.0]
            elif t < 160:
                action = [0.0, 0.0, -0.05 if q[2] > 0.02 else 0.0, 0.0]
            else:
                action = [0.0, 0.0, 0.05, 0.0]
        elif MODE == "crash" and t == 100:
            sys.exit(3)
        elif MODE == "garbage" and t == 50:
            print(json.dumps({"action": ["left", None]}), flush=True)
            continue
        elif MODE == "slow":
            time.sleep(0.6)
        elif MODE == "jitter":
            action = [random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.05, 0.05), random.uniform(-0.8, 0.8)]
        print(json.dumps({"action": action}), flush=True)


if __name__ == "__main__":
    main()
