"""The reference as a closed_loop submission (`run.sh`: `python3 -I main.py`).

Speaks the grader's wire (one JSON object per line on stdin, one reply per line on stdout; logs on stderr): READY,
then {"type": "reset"} -> {"ok": true}, {"type": "act", "t", "obs"} -> {"action": {...}}, {"type": "close"} ->
{"ok": true} and exit. Without `truth` in the observation (not a token run) it ends every pack at once. See `controller.py`.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller import Reference  # noqa: E402


def main():
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    ref = Reference(log=lambda *a: print("[reference]", *a, file=sys.stderr, flush=True))
    for line in sys.stdin:
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "act":
            reply = {"action": ref.act(int(msg["t"]), msg["obs"])}
        elif kind == "reset":
            ref.reset()
            reply = {"ok": True}
        elif kind == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            return
        else:
            reply = {"error": "unknown message"}
        out.write(json.dumps(reply) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
