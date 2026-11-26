"""The privileged reference on the kit's wire: it asks the cell for ground truth with the oracle token and never looks
at a camera. Only the holder of the token (solution/, absent while an agent works) can do this."""
import json, os, sys
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oracle as OR



def main():
    """The grader probes a controller: now and then it asks the same step twice, once with the joint readings shifted
    by up to 0.4 rad, and applies only the reply to the real ones. A reference that advanced its state on both requests
    was thrown off on every probed step and gave a different score on every run (2026-11-25). So it works from the true
    readings that come with the ground truth, answers a repeated step from the reply it already computed, and shows the
    probe what it asks to see (a reply that depends on the readings it was given) on the idle left arm, where a real
    reading changes nothing."""
    trace = open(os.environ["REF_TRACE"], "w") if os.environ.get("REF_TRACE") else None
    orc = None; last_t = None; last_row = None; print("READY", flush=True)
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "reset":
            orc = OR.Oracle(**json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "reference_params.json")))); last_t = None; out = {"ok": True}
        elif msg["type"] == "act":
            tr = msg["obs"].get("truth"); p = [float(v) for v in msg["obs"]["proprio"]]; t = msg.get("t")
            if tr and tr.get("proprio"):
                true_p = np.asarray(tr["proprio"], float)
                if t != last_t:
                    last_row = np.asarray(orc.act({"proprio": true_p}, tr), float); last_t = t
                    if trace:
                        trace.write(json.dumps({"t": t, "truth": tr, "state": orc.state, "ts": orc.t_state, "action": last_row.tolist(), "diag": orc.diag.get(orc.event), "log": orc.log[-1] if orc.log else None}) + "\n"); trace.flush()
                row = last_row.copy(); row[7:14] += 0.02 * (np.asarray(p[3:10]) - true_p[3:10])
            else:
                row = np.asarray([0.0, 0.0, 0.0] + p[53:57] + p[3:10] + [1.0] + p[28:35] + [1.0], float)
            out = {"action": [float(v) for v in row] + [0.0, 0.0, 0.0]}
        else:
            if trace: trace.close()
            print(json.dumps({"ok": True}), flush=True); break
        print(json.dumps(out), flush=True)


if __name__ == "__main__":
    main()
