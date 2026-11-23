"""A controller that only hovers above the cable's free end: it shows the wire, not a strategy."""

import json
import sys


def act(t, obs):
    end = obs["cable"][-1]
    return {"pos": [end[0], end[1], 0.08], "yaw": 0.0, "grip": 0.0, "done": t >= 30}


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        if req.get("type") == "reset":
            print(json.dumps({"ok": True}), flush=True)
        elif req.get("type") == "act":
            print(json.dumps({"action": act(req["t"], req["obs"])}), flush=True)
        elif req.get("type") == "close":
            print(json.dumps({"ok": True}), flush=True)
            return


if __name__ == "__main__":
    main()
