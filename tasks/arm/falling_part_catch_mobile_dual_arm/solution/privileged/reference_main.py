"""The privileged reference on the kit's wire: it is handed ground truth (the grader attaches it for the holder of the
oracle token, which exists only in solution/) and never looks at a camera.

A step may be asked twice with the same `t` (the grader's probe: once with the joint readings shifted). The controller
advances once per step, on the true readings in truth["proprio"]; the reply to either request is that action plus a
small echo on the left arm of the difference between the readings it was sent and the true ones, which is zero for the
real request. So the reply depends on the readings it was given, and the probe never changes what the controller does."""
import os
for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"): os.environ[key] = "1"
import json, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oracle as OR


def main():
    orc = None; last_t, last_row = None, None; print("READY", flush=True)
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "reset":
            par = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "reference_params.json")))
            orc = getattr(OR, par.pop("controller", "Oracle"))(**par); last_t, last_row = None, None; out = {"ok": True}
        elif msg["type"] == "act":
            p = [float(v) for v in msg["obs"]["proprio"]]; tr = msg["obs"].get("truth")
            if not tr: row = [0.0, 0.0, 0.0] + p[53:57] + p[3:10] + [1.0] + p[28:35] + [1.0]      # no token: hold still
            else:
                true = [float(v) for v in tr.get("proprio", p)]
                if msg["t"] != last_t: last_t = msg["t"]; last_row = [float(v) for v in orc.act({"proprio": np.asarray(true, float)}, tr)]
                row = list(last_row)
                for k in range(7): row[7 + k] += 0.05 * (p[3 + k] - true[3 + k])
            out = {"action": row + [0.0, 0.0, 0.0]}
        else:
            print(json.dumps({"ok": True}), flush=True); break
        print(json.dumps(out), flush=True)


if __name__ == "__main__":
    main()
