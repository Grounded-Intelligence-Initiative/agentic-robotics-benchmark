"""The reference controller on the grading wire: one JSON object per line on stdin and stdout.

The behaviour (reference.py) is a generator that is sent observations and yields actions. A step
index `t` that is asked a second time is answered with the action already given for it: the
behaviour advances once per step, not once per request.
"""

import json
import sys

from reference import Reference


def main():
    out = sys.stdout
    print("READY", flush=True)
    behaviour = None
    last_t = None
    last_action = None
    controller = None
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
        except ValueError:
            out.write(json.dumps({"error": "bad json"}) + "\n")
            out.flush()
            continue
        kind = request.get("type")
        if kind == "reset":
            controller = Reference()
            behaviour = None
            last_t = None
            reply = {"ok": True}
        elif kind == "act":
            t = request.get("t")
            if t == last_t and last_action is not None:
                reply = {"action": last_action}
            else:
                try:
                    if behaviour is None:
                        behaviour = controller.run(request["obs"])
                        action = next(behaviour)
                    else:
                        action = behaviour.send(request["obs"])
                except StopIteration:
                    action = controller.action()          # the behaviour is over: hold still
                    behaviour = _hold(controller)
                    next(behaviour)
                last_t, last_action = t, action
                reply = {"action": action}
        elif kind == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            return
        else:
            reply = {"error": "unknown request"}
        out.write(json.dumps(reply) + "\n")
        out.flush()


def _hold(controller):
    while True:
        controller.obs = yield controller.action()


if __name__ == "__main__":
    main()
