"""Minimal JSONL submission. Copy into your submission and launch with exec python3 -u controller.py."""

import json
import sys


def act(obs):
    # Tick is the physical 50 Hz clock. A chunk can run up to the next RGB frame.
    length = 5 - obs["tick"] % 5
    return {"rows": [{"codes": [16] * 64, "hand": [0, 0]} for _ in range(length)]}


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        kind = request["type"]
        if kind in ("reset", "close"):
            reply = {"ok": True}
        elif kind == "act":
            reply = {"action": act(request["obs"])}
        else:
            reply = {"error": "unknown request type"}
        print(json.dumps(reply), flush=True)
        if kind == "close":
            return


if __name__ == "__main__":
    main()
