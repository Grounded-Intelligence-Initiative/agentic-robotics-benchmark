"""The reference's wire loop: READY, then reset / act / close (one JSON object per line). Each `act` carries the
token truth (tests/verify.py adds it only for the reference's token). The script (oracle.py) plans from the truth
alone, so it runs the same whichever copy of a repeated step (a probe pair) comes first; a reply's Cartesian steps
are measured from that copy's observed tool position. A repeated step index is answered from the step's plan without
advancing the script."""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from oracle import Oracle  # noqa: E402


def main():
    out = sys.stdout
    print("READY", file=out, flush=True)
    orc, last_t, last = None, None, None
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        kind = req.get("type")
        if kind == "reset":
            orc, last_t, last = Oracle(), None, None
            print(json.dumps({"ok": True}), file=out, flush=True)
        elif kind == "act":
            t, obs = int(req["t"]), req["obs"]
            if t == last_t and last is not None:
                act = json.loads(json.dumps(last))
                for s in ("l", "r"):        # the same step, measured from this copy's observed tool position
                    if len(act[s]) == 7 and any(abs(x) > 1e-9 for x in act[s][:6]):
                        for i in range(3):
                            act[s][i] += obs["truth"]["tools"][s]["pos"][i] - obs[s]["ee_pos"][i]
            else:
                act = orc.act(obs)
                base = json.loads(json.dumps(act))
                for s in ("l", "r"):
                    if len(base[s]) == 7 and any(abs(x) > 1e-9 for x in base[s][:6]):
                        for i in range(3):
                            base[s][i] -= obs["truth"]["tools"][s]["pos"][i] - obs[s]["ee_pos"][i]
                last_t, last = t, base
            print(json.dumps({"action": act}), file=out, flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), file=out, flush=True)
            return


if __name__ == "__main__":
    main()
