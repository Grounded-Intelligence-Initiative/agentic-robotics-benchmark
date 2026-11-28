"""The score of the acceptance (CONTRACT.md section 11) and the verifier's public metrics.

    finalize_job(record)                 q of a carry job (None for a decline job) and its band losses
    shift_scores(records, events)        per shift: S, J, the carry mean and the event counts
    score(units, gates)                  the deterministic score dict (contract/schemas/score.json)
    reward(raw, gates_ok, anchor, ...)   the gated reward, as the grader kit computes it
    public_metrics(...)                  the verifier's own metrics: exactly contract.names.METRIC_KEYS together with
                                         the keys the grader kit adds itself ("ratio", "anchor_recorded", the metric
                                         named in anchor.json, "reward_exact")
    value_hits(metrics, values)          metric values equal, to three significant digits, to one of the values

A carry job: q = carried x L(slip) x L(rotation) x released, L = 1 at or below the band's first end, 0 at or above its
second, linear between; any job loss or event makes q = 0. A shift: S = (mean q over its carry jobs) x (0.7 + 0.3 J),
J = max(0, TPR - FPR). raw = the mean of S over the shifts. Every number must be finite: a non-finite one is a task
error (RuntimeError), never a score.
"""
import math

from contract import constants as C
from contract import names as N

SC = C.SCORE
BANDS = {"slip_mm": tuple(SC["slip_band_mm"]), "rot_deg": tuple(SC["rot_band_deg"])}
LOST = set(N.EVENT_KINDS) | {"not_presented", "not_grasped", "not_lifted", "not_seated", "release_late",
                             "release_push", "decline_missed", "decline_wrong", "voided"}
KIT_KEYS = ("ratio", "anchor_recorded", "reward_exact", "acceptance_score")    # the anchor's metric name
OWN_KEYS = tuple(k for k in N.METRIC_KEYS if k not in KIT_KEYS)
FAMILY = {k: N.OBS_EVENT_OF[k] for k in N.EVENT_KINDS}


def finite(x, what):
    x = float(x)
    if not math.isfinite(x):
        raise RuntimeError("task error: %s is not finite (%r)" % (what, x))
    return x


def band(x, ends):
    full, zero = ends
    x = finite(x, "a band input")
    if x <= full:
        return 1.0
    if x >= zero:
        return 0.0
    return (zero - x) / (zero - full)


def finalize_job(rec):
    """Set rec["q"] (carry jobs) and add the band losses; returns q (None for a decline job)."""
    if rec["label"] != "carry":
        rec["q"] = None
        return None
    if LOST & set(rec["losses"]):
        rec["q"] = 0.0
        return 0.0
    if rec["slip_mm"] is None or rec["rot_deg"] is None:
        rec["q"] = 0.0
        return 0.0
    ls = band(rec["slip_mm"], BANDS["slip_mm"])
    lr = band(rec["rot_deg"], BANDS["rot_deg"])
    for kind, val in (("band_slip", ls), ("band_rot", lr)):
        if val < 1.0 and kind not in rec["losses"]:
            rec["losses"].append(kind)
    q = (1.0 if rec["carried"] else 0.0) * ls * lr * (1.0 if rec["released"] else 0.0)
    rec["q"] = finite(q, "q")
    return rec["q"]


def shift_scores(records, events, shifts=N.SHIFTS):
    """[{"shift", "S", "J", "carry_mean", "events"}] for each shift of one unit's records and unit-level events."""
    out = []
    for s in shifts:
        recs = [r for r in records if r["shift"] == s]
        carry = [r for r in recs if r["label"] == "carry"]
        dec = [r for r in recs if r["label"] == "decline"]
        qs = [finite(r["q"] if r["q"] is not None else 0.0, "q") for r in carry]
        carry_mean = sum(qs) / len(qs) if qs else 0.0
        tpr = sum(1 for r in dec if r["declined"]) / len(dec) if dec else 0.0     # no decline job: no J earned
        fpr = sum(1 for r in carry if r["declined"]) / len(carry) if carry else 0.0
        j = max(0.0, tpr - fpr)
        S = carry_mean * (SC["shift_base_weight"] + SC["shift_j_weight"] * j)
        counts = {"crush": 0, "drop": 0, "shutdown": 0}
        for e in events:
            if e["shift"] == s:
                counts[FAMILY[e["kind"]]] += 1
        out.append({"shift": s, "S": finite(S, "S"), "J": finite(j, "J"), "carry_mean": finite(carry_mean, "mean"),
                    "events": counts})
    return out


def reward(raw, gates_ok, anchor=1.0, full_at=1.0, cap=1.5):
    """clamp(clamp(raw / anchor, 0, cap) / full_at, 0, 1), and 0 when a pass condition fails."""
    raw = finite(raw, "raw")
    if not gates_ok:
        return 0.0
    ratio = max(0.0, min(raw / float(anchor), float(cap)))
    return max(0.0, min(ratio / float(full_at), 1.0))


def score(units, gates, anchor=1.0, full_at=1.0):
    """units: [{"unit", "records", "events"}] in run order (records already finalised). The score dict."""
    shifts = []
    for u in units:
        for s in shift_scores(u["records"], u["events"]):
            shifts.append(dict({"unit": int(u["unit"])}, **s))
    raw = sum(s["S"] for s in shifts) / len(shifts) if shifts else 0.0
    raw = finite(raw, "raw")
    g = {k: bool(gates.get(k, False)) for k in N.GATES}
    return {"reward": reward(raw, all(g.values()), anchor, full_at), "raw": raw, "gates": g,
            "shifts": [{"unit": s["unit"], "shift": s["shift"], "S": s["S"], "J": s["J"],
                        "carry_mean": s["carry_mean"], "events": s["events"]} for s in shifts]}


def public_metrics(sc, unit_stats, ops):
    """The verifier's own metrics (OWN_KEYS, every one present, all finite floats).

    sc: the score dict; unit_stats: {unit: stats of acceptance.run_unit}; ops: the operational counters by name."""
    out = {k: 0.0 for k in OWN_KEYS}
    out["contract_ok"] = 1.0 if all(sc["gates"].values()) else 0.0
    for g, v in sc["gates"].items():
        out["gate_" + g] = 1.0 if v else 0.0
    for s in sc["shifts"]:
        p = "u%ds%d_" % (s["unit"], s["shift"])
        out[p + "S"] = s["S"]
        out[p + "J"] = s["J"]
        out[p + "carry_mean"] = s["carry_mean"]
        for k in ("crush", "drop", "shutdown"):
            out[p + "events_" + k] = float(s["events"][k])
    for u, st in unit_stats.items():
        p = "u%d_" % int(u)
        out[p + "on_time_fraction"] = float(st.get("on_time_fraction", 0.0))
        out[p + "late_cycles"] = float(st.get("late_cycles", 0))
        out[p + "controller_ended"] = 1.0 if st.get("controller_ended") else 0.0
        out[p + "wall_s"] = float(round(float(st.get("isolation", {}).get("wall_s", 0.0))))   # whole seconds
    for k, v in ops.items():
        if k in out:
            out[k] = float(v)
    for k, v in out.items():
        finite(v, k)
    if set(out) != set(OWN_KEYS):
        raise AssertionError("public metrics keys differ from the whitelist")
    return out


def _sig(x, n=3):
    return "%.*g" % (n, float(x))


def value_hits(metrics, values, n=3):
    """[(key, value, hidden)] for every metric value that is not a whole number and equals one of `values` to n
    significant digits (whole numbers are counts, flags and the ends of the [0, 1] scale)."""
    hidden = [float(v) for v in values if isinstance(v, (int, float)) and not isinstance(v, bool)
              and math.isfinite(float(v)) and float(v) != 0.0]
    sigs = {}
    for h in hidden:
        sigs.setdefault(_sig(h, n), h)
    out = []
    for k, v in sorted(metrics.items()):
        v = float(v)
        if not math.isfinite(v) or v == round(v):
            continue
        h = sigs.get(_sig(v, n))
        if h is not None:
            out.append((k, v, h))
    return out
