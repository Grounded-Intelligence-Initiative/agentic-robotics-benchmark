#!/usr/bin/env python3
"""Optional boilerplate for the grading wire: READY, reset, act, close, repeated step indices.

Copy it into your submission and fill in `Controller`. A step index may be asked twice (see the
instruction): answer every request from the observation it carries. Here the controller's state
is copied before a step and restored when the same step index comes again, so both requests are
answered from the same state.

Keep stdout for the protocol; write logs to stderr.
"""

import copy
import json
import sys


class Controller(object):
    def __init__(self):
        self.state = {}

    def reset(self):
        """A new order starts."""
        self.state = {}

    def act(self, obs):
        """One action for this observation: [vx, vy, vz, wyaw, vacuum, cams, done]."""
        return [0.0, 0.0, 0.0, 0.0, 0, 0, 0]


def main():
    controller = Controller()
    saved, saved_t = None, None
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        message = json.loads(line)
        kind = message.get("type")
        if kind == "reset":
            controller.reset()
            saved, saved_t = None, None
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            if message["t"] == saved_t:
                controller.state = copy.deepcopy(saved)
            else:
                saved, saved_t = copy.deepcopy(controller.state), message["t"]
            print(json.dumps({"action": controller.act(message["obs"])}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            break


if __name__ == "__main__":
    main()
