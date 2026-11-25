"""The grade dump: the whole grade of one submission, for maintainers (opt-in: verify.py writes it only when
ARB_HAND_TWIN_GRADE_PATH is set; it never changes the verdict).

    d = build(submission_result, selftest_summary, verdict)
    write(path, d)                    # JSON, keys sorted, one space indent
    strip_cpu(d)                      # the same without "cpu_s", the one block that differs between runs

Format hand-twin-grade/1:
  verdict        {"reward", "twin_score", "metrics": {...}}  (the verdict as written)
  gate           the gate's report
  settle         {key: {"disp_mm", "rot_deg"}}
  registration   the fit: s, psi_deg, t_mm, used, dropped, resid_mm, n_within, ok, reason, twin_points_mm, present
  requests       {id: {"planned", "declined_by", "blocker", "plan", "outcomes": [9], "by": [9], "hit": [9],
                       "lift": [9 x null | {"lifted", "trials"}], "error"}}
  units          {"<key>|<rule>": {"done", "stop", "declined", "w", "kind", "n_creditable", "n_scored"}}
  weights        {"w0", "things", "c", "thing_cap"}
  selftest       the self-test's summary
  stages         {name: {"ok"[, "error"]}};  diagnostics {name: text}
  cpu_s          {stage: CPU seconds}
No wall-clock value is written.
"""
import json

from . import FORMAT_GRADE

KEEP_SELFTEST = ("ok", "manifest_files", "geometric_replayed", "lifts_replayed", "constants", "reason")


def _lift_lists(sub):
    n = len(next(iter(sub.get("requests", {}).values()), {}).get("outcomes", [])) if sub.get("requests") else 0
    out = {rid: [None] * n for rid in sub.get("requests", {})}
    for key, res in (sub.get("lifts") or {}).items():
        rid, i = key.rsplit("|", 1)
        if rid in out and 0 <= int(i) < n:
            out[rid][int(i)] = res
    return out


def build(sub, selftest, verdict):
    sub = sub if isinstance(sub, dict) else {}
    lifts = _lift_lists(sub)
    plans = sub.get("plans") or {}
    reqs = {}
    for rid, row in sorted((sub.get("requests") or {}).items()):
        reqs[rid] = {"planned": row.get("planned"), "declined_by": row.get("declined_by"),
                     "blocker": row.get("blocker"), "plan": plans.get(rid), "outcomes": row.get("outcomes"),
                     "by": row.get("by"), "hit": row.get("hit"), "lift": lifts.get(rid), "error": row.get("error")}
    st = {k: v for k, v in (selftest or {}).items() if k in KEEP_SELFTEST}
    cpu = dict(sub.get("cpu_s") or {})
    if isinstance((selftest or {}).get("cpu_s"), (int, float)):
        cpu["selftest"] = float(selftest["cpu_s"])
    v = verdict or {}
    return {"format": FORMAT_GRADE,
            "verdict": {"reward": (v.get("rewards") or {}).get("reward"), "twin_score": v.get("twin_score"),
                        "metrics": dict(v.get("metrics") or {})},
            "gate": sub.get("gate") or {}, "settle": sub.get("settle") or {},
            "registration": sub.get("registration") or {}, "requests": reqs, "units": sub.get("units") or {},
            "weights": sub.get("weights") or {}, "selftest": st, "stages": sub.get("stages") or {},
            "diagnostics": sub.get("diagnostics") or {}, "stats_requests": sub.get("stats_requests") or {},
            "cpu_s": cpu}


def strip_cpu(d):
    return {k: v for k, v in d.items() if k != "cpu_s"}


def dumps(d):
    return json.dumps(d, indent=1, sort_keys=True, allow_nan=False)


def write(path, d):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(dumps(d) + "\n")
