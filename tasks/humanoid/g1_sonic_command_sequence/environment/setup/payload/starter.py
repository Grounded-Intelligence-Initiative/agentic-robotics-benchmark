"""Neutral JSONL controller. Install next to run.sh as controller.py."""

import json
import sys


def act(obs):
    count = min(5 - obs["tick"] % 5, obs["horizon"] - obs["tick"])
    return {"rows": [{"codes": [16] * 64} for _ in range(count)]}


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        kind = request["type"]
        if kind == "act":
            reply = {"action": act(request["obs"])}
        elif kind in ("reset", "close"):
            reply = {"ok": True}
        else:
            reply = {"error": "unknown request type"}
        print(json.dumps(reply), flush=True)
        if kind == "close":
            return


if __name__ == "__main__":
    main()
