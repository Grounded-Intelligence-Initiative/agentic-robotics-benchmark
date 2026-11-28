"""The reference controller on the grading wire: READY, then one JSON request per line on stdin, one reply per line
on stdout."""
import base64
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from reference import Reference  # noqa: E402

CAMS = ("overview", "wrist", "inspect")


def decode(obs):
    out = dict(obs)
    for k in CAMS:
        d = obs.get(k)
        if isinstance(d, dict):
            out[k] = np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])
    for k in ("q", "dq", "ee_pos", "ee_quat", "wrench"):
        if k in out:
            out[k] = np.asarray(out[k], float)
    return out


def main():
    ref = None
    out = sys.stdout
    out.write("READY\n")
    out.flush()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        typ = req.get("type")
        if typ == "reset":
            ref = Reference({}, latency=int(os.environ.get("CPK_LATENCY", "2")))
            rep = {"ok": True}
        elif typ == "act":
            if ref is None:
                ref = Reference({})
            a = ref.act(decode(req["obs"]))
            rep = {"action": [float(x) for x in a]}
        elif typ == "close":
            out.write(json.dumps({"ok": True}) + "\n")
            out.flush()
            break
        else:
            rep = {"error": "unknown request"}
        out.write(json.dumps(rep) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
