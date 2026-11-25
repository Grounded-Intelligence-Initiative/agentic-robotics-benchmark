"""A minimal controller for the grading wire: it holds the arm still and looks around once.

It shows the wire (READY, reset, act, close), how a repeated step index is answered without
advancing anything, and how the images of an observation are decoded. It retrieves nothing.

    run.sh:   exec /opt/venv/bin/python3 controller.py
"""

import base64
import json
import sys

import numpy as np


def decode(item):
    return np.frombuffer(base64.b64decode(item["b64"]), dtype=item["dtype"]).reshape(item["shape"])


class Controller(object):
    def __init__(self):
        self.answered = {}          # step index -> the action given for it

    def act(self, t, obs):
        if t in self.answered:      # a step asked again: the same answer, nothing advances
            return self.answered[t]
        for name, image in obs["images"].items():
            rgb, depth = decode(image["rgb"]), decode(image["depth"])
            print("t={} {}: rgb {} depth {} mm (median {})".format(
                t, name, rgb.shape, depth.shape, int(np.median(depth))), file=sys.stderr)
        cameras = 3 if t == 20 else 0                   # ask for both cameras once
        action = list(obs["q"]) + [0.08, cameras]       # hold the joints where they are, hand open
        self.answered = {t: action}
        return action


def main():
    controller = Controller()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        request = json.loads(line)
        if request["type"] == "reset":
            controller = Controller()
            reply = {"ok": True}
        elif request["type"] == "act":
            reply = {"action": controller.act(request["t"], request["obs"])}
        else:
            print(json.dumps({"ok": True}), flush=True)
            return
        print(json.dumps(reply), flush=True)


if __name__ == "__main__":
    main()
