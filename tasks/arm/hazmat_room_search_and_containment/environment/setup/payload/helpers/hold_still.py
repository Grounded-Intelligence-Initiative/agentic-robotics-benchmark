"""A controller that only speaks the wire: it holds the cart and the arm where they start,
never closes the latch and asks for a picture twice a second (and shows how one is decoded).
Copy it into your submission as a starting point (it has no control logic).

A step index `t` can be asked twice in a row with two observations; the reply to each must
come from the observation it carries, and whatever your controller remembers must advance
once per `t`, not once per request.
"""

import base64
import json
import sys
import zlib

import numpy as np


def decode(d):
    """A picture from an observation: `rgb` is (rows, columns, 3) uint8, `depth_mm` is
    (rows, columns) uint16."""
    return np.frombuffer(zlib.decompress(base64.b64decode(d["zlib_b64"])),
                         dtype=d["dtype"]).reshape(d["shape"])


class Controller(object):
    def reset(self):
        self.hold = None
        self.stay = None
        self.last_t = None

    def act(self, t, obs):
        if t != self.last_t:              # a new step: advance the controller's memory here
            self.last_t = t
            if self.hold is None:
                self.hold = list(obs["q"])
                self.stay = list(obs["base"])
            if obs["camera"] is not None:
                depth = decode(obs["camera"]["depth_mm"])
                print("picture at {} s: nearest thing {} mm along the camera's axis".format(
                    obs["camera"]["stamp_s"], int(depth[depth > 0].min())), file=sys.stderr)
        # the reply depends on the observation: a small pull back to the held poses (the
        # base's is a velocity in the cart's own frame; the cart has not turned, so the
        # world's axes will do here)
        q = [h + 0.2 * (h - m) for h, m in zip(self.hold, obs["q"])]
        base = [2.0 * (s - m) for s, m in zip(self.stay, obs["base"])]
        return {"base": base, "q": q, "latch": 0, "look": int(t % 25 == 0)}


def main():
    ctl = Controller()
    ctl.reset()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        kind = req.get("type")
        if kind == "reset":
            ctl.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            print(json.dumps({"action": ctl.act(req["t"], req["obs"])}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return


if __name__ == "__main__":
    main()
