"""A minimal controller for the harvesting cart: it speaks the wire, looks at the row once and finishes.

It earns nothing; it is the starting point. Copy both files to /home/user/submission/ and build on them. Everything
this process writes to stdout is the wire: log to stderr.
"""

import json
import sys


class Controller(object):
    def reset(self):
        self.t = None
        self.plan = None

    def act(self, t, obs):
        """One action for step `t`. A step may be asked twice with the same `t` and slightly different robot
        readings: advance your own state once per `t`, and build each answer from the observation it came with."""
        if t != self.t:
            self.t = t
            if t == 0:
                self.plan = "look"
            elif "overview" in obs["images"]:
                print("[example] got the overview frame at t = %d" % t, file=sys.stderr, flush=True)
                self.plan = "finish"
        if self.plan == "look":
            return {"move": {"type": "hold", "ticks": 1}, "cams": ["overview"]}
        return {"cmd": "done"}


def main():
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    ctl = Controller()
    ctl.reset()
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "act":
            reply = {"action": ctl.act(int(msg["t"]), msg["obs"])}
        elif msg["type"] == "reset":
            ctl.reset()
            reply = {"ok": True}
        elif msg["type"] == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            return
        else:
            reply = {"error": "unknown message"}
        out.write(json.dumps(reply) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
