"""A controller that hovers above the socket and looks at its frames: it shows the wire, not a strategy."""

import json
import sys

import numpy as np


def act(t, obs):
    rgb = np.load(obs["frames"]["overhead"]["rgb"])          # (480, 640, 3) uint8
    depth = np.load(obs["frames"]["overhead"]["depth"])      # (480, 640) float32, metres along the optical axis
    if t == 0:
        print("overhead frame %s, depth %.3f-%.3f m" % (rgb.shape, depth.min(), depth.max()), file=sys.stderr)
    s = obs["socket"]["pos"]
    return {"pos": [s[0], s[1], 0.08], "yaw": 0.0, "grip": 0.0, "done": t >= 30}


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
