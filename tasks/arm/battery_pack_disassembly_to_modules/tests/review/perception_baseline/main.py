"""The baseline behind the grader's wire (`run.sh`: `python3 -I main.py`): READY, then {"type": "reset"} ->
{"ok": true}, {"type": "act", "t", "obs"} -> {"action": {...}}, {"type": "close"} -> {"ok": true} and exit. Logs on
stderr. See `controller.py`."""

import json
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller import Baseline  # noqa: E402


def main():
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    debug = os.environ.get("PB_DEBUG_DIR") or None
    ctl = Baseline(log=lambda *a: print("[baseline]", *a, file=sys.stderr, flush=True), debug_dir=debug)
    for line in sys.stdin:
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "act":
            try:
                action = ctl.act(int(msg["t"]), msg["obs"])
            except Exception:  # noqa: BLE001
                print("[baseline] error:", traceback.format_exc(), file=sys.stderr, flush=True)
                action = {"cmd": "done"}
            reply = {"action": action}
        elif kind == "reset":
            ctl.reset()
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
