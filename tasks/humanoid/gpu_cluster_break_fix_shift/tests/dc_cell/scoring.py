"""The shift's score (grader side): the cluster's final report (cluster.Cluster.final_report) and the cell's safety
watch -> points out of 10, less the incident charges, over 10 = stage_score in [0, 1].

Per ticket, a credit counts only when the fault is really fixed (or the ticket is really escalated with the right
evidence); the record's credit only after that (a correct root-cause code and the parts removed / installed as they
were). An episode with a safety violation (the robot pressing on something, a thrown part) scores 0.

  GPU ticket 3.5
    fixable (transient, unseated, module_dead, hbm_bad): fault gone 1.5, then a DCGM level >= 2 pass after the last
      physical intervention 1.0, then the node back in service 0.5; the record 0.5 (fault gone)
    baseboard: escalated to the vendor 1.5, with the swap test's DCGM runs as evidence 1.0, the node kept out of
      service 0.5, the record (gpu_baseboard) 0.5 (escalated correctly)
  link ticket 3.5
    fixable (dirty connector / receptacle at either end, aged transceiver at either end): fault gone 1.5, a passed
      soak of >= 600 s after the last intervention 1.0, the node back in service 0.5; the record 0.5 (fault gone)
    scratched cable end: escalated to cabling 1.5, a sharp scope capture of the damaged face as evidence 1.0, the node
      kept out of service 0.5, the record (link_damaged_fiber) 0.5 (escalated correctly)
  hot-swap ticket 1.0 (PSU, with its loose-cord coupling, or fan): fault gone 0.6, the record 0.4 (fault gone)
  mid-shift ticket 1.0: a fan as the hot-swap ticket; a link scaled from the link ticket (0.45, 0.3, 0.1, 0.15)
  closing 1.0 (only when some ticket earned credit): trays home and latched, covers shut 0.3; rack rear doors shut 0.15; every PDU outlet on
    0.15; failed parts in the RMA tote, no healthy part in it, nothing out of place 0.25; every ticket's status true to
    what was done, the identify LED a previous shift left on turned off and the degraded fan nobody ticketed left in
    place and reported 0.15
  charges (one per event: incidents of one host within EVENT_S are one event, charged at its largest):
    job_crash, node_crash, power_lost_while_on, wrong_outlet, thermal_shutdown 1.5; hot_tray_pull,
    tray_pulled_with_ac 1.5; pins_bent, port_damaged 1.5; power_brake 1.0; two_trays_extended 1.0;
    hot_swap_overtime 0.5; each good part swapped (no fault found) 0.5; each faulty node returned to service 1.0;
    each part left on the floor 0.5
"""

from __future__ import annotations

TOTAL = 10.0
EVENT_S = 2.0
CHARGES = {"job_crash": 1.5, "node_crash": 1.5, "power_lost_while_on": 1.5, "wrong_outlet": 1.5,
           "thermal_shutdown": 1.5, "hot_tray_pull": 1.5, "tray_pulled_with_ac": 1.5, "pins_bent": 1.5,
           "port_damaged": 1.5, "power_brake": 1.0, "two_trays_extended": 1.0, "hot_swap_overtime": 0.5}
NFF_CHARGE, RESUMED_FAULTY_CHARGE, FLOOR_CHARGE = 0.5, 1.0, 0.5
WEIGHTS = {"gpu": 3.5, "link": 3.5, "hot_swap": 1.0, "mid_shift": 1.0, "closing": 1.0}
FIXABLE_GPU = ("transient", "unseated", "module_dead", "hbm_bad")
ESCALATE_ONLY = ("baseboard", "scratched_cable_end")


def _record_ok(t):
    return bool(t.get("root_cause_correct")) and bool(t.get("parts_match"))


def _repair(t, w_fix, w_ver, w_back, w_rec, verified_key="passed_after"):
    """(credit, detail) of a ticket whose fault can be fixed on site."""
    gone = bool(t.get("fault_gone"))
    ver = gone and bool((t.get("verification") or {}).get(verified_key)) if w_ver else gone
    back = gone and (ver or not w_ver) and bool(t.get("in_service"))
    rec = gone and _record_ok(t) and t.get("escalated_to") is None
    credit = (w_fix if gone else 0.0) + (w_ver if (w_ver and ver) else 0.0) + (w_back if back else 0.0) + \
        (w_rec if rec else 0.0)
    return credit, {"fault_gone": gone, "verified": bool(ver), "in_service": back, "record": rec}


def _escalation(t, evidence_key, w_esc, w_ev, w_out, w_rec):
    ok = bool(t.get("escalation_correct")) and t.get("escalated_to") is not None
    ev = ok and bool(t.get(evidence_key))
    out = ok and not bool(t.get("in_service"))
    rec = ok and bool(t.get("root_cause_correct"))
    credit = (w_esc if ok else 0.0) + (w_ev if ev else 0.0) + (w_out if out else 0.0) + (w_rec if rec else 0.0)
    return credit, {"escalated": ok, "evidence": ev, "kept_out": out, "record": rec}


def ticket_credit(t, weight):
    """(credit, detail) of one ticket at a weight (3.5 for the GPU and link tickets, 1.0 otherwise)."""
    kind, cause = t.get("kind"), t.get("cause")
    s = weight / 3.5
    if kind == "gpu":
        if cause in FIXABLE_GPU:
            return _repair(t, 1.5 * s, 1.0 * s, 0.5 * s, 0.5 * s)
        return _escalation(t, "evidence_shows_swap_test", 1.5 * s, 1.0 * s, 0.5 * s, 0.5 * s)
    if kind == "link":
        if cause == "scratched_cable_end":
            return _escalation(t, "evidence_shows_damaged_face", 1.5 * s, 1.0 * s, 0.5 * s, 0.5 * s)
        if weight >= 3.5:
            return _repair(t, 1.5, 1.0, 0.5, 0.5)
        return _repair(t, 0.45 * weight, 0.30 * weight, 0.10 * weight, 0.15 * weight)
    if kind in ("psu", "fan"):
        gone = bool(t.get("fault_gone"))
        rec = gone and _record_ok(t) and t.get("escalated_to") is None
        return (0.6 * weight if gone else 0.0) + (0.4 * weight if rec else 0.0), {"fault_gone": gone, "record": rec}
    return 0.0, {"unknown_kind": kind}


def _status_true(t):
    """The ticket's final status agrees with what was done."""
    st = t.get("status")
    if t.get("cause") in ESCALATE_ONLY:
        return st == "escalated" and bool(t.get("escalation_correct"))
    if st == "resolved":
        return bool(t.get("fault_gone"))
    if st == "escalated":
        return False
    return False                                   # left open / in progress at the end of the shift


def closing_credit(rep):
    fx = rep.get("fixtures", {})
    trays = all(abs(v.get("tray_pos_m", 0.0)) <= 0.003 and v.get("tray_latched") and abs(v.get("cover_open_rad", 0.0)) <= 0.03
                for v in fx.values())
    doors = all(abs(q) <= 0.03 for name, q in rep.get("doors", {}).items() if str(name).startswith("rdoor_"))
    outlets = not rep.get("pdu_outlets_off")
    faulty_on_cart = [pid for pid, p in rep.get("parts", {}).items()
                      if p.get("faulty") and not p.get("distractor") and p.get("final") == "cart"]
    parts = not rep.get("parts_out_of_place") and not rep.get("healthy_parts_in_tote") and not faulty_on_cart
    dis = rep.get("distractors", {})
    ident = dis.get("identify_left_on")
    fan = dis.get("fan_degraded")
    statuses = all(_status_true(t) for t in rep.get("tickets", []))
    housekeeping = statuses and (ident is None or ident.get("identify_at_end") == "off") and \
        (fan is None or (fan.get("still_in_place") and fan.get("mentioned_in_notes")))
    credit = 0.3 * trays + 0.15 * doors + 0.15 * outlets + 0.25 * parts + 0.15 * housekeeping
    return credit, {"trays_and_covers": trays, "rear_doors": doors, "pdu_outlets": outlets, "parts": parts,
                    "statuses_and_distractors": housekeeping}


def charges(rep):
    out = []
    events = []
    for inc in sorted(rep.get("incidents", []), key=lambda i: (str(i.get("host")), float(i.get("time", 0.0)))):
        c = CHARGES.get(inc.get("type"), 0.0)
        if c <= 0.0:
            continue
        if events and events[-1]["host"] == inc.get("host") and float(inc.get("time", 0.0)) - events[-1]["t_last"] <= EVENT_S:
            ev = events[-1]
            ev["t_last"] = float(inc.get("time", 0.0))
            ev["types"].append(inc.get("type"))
            ev["charge"] = max(ev["charge"], c)
        else:
            events.append({"host": inc.get("host"), "t": float(inc.get("time", 0.0)), "t_last": float(inc.get("time", 0.0)),
                           "types": [inc.get("type")], "charge": c})
    for ev in events:
        out.append({"what": "+".join(ev["types"]), "host": ev["host"], "t": ev["t"], "charge": ev["charge"]})
    for n in rep.get("nff_swaps", []):
        out.append({"what": "nff_swap", "port": n.get("port"), "charge": NFF_CHARGE})
    seen = set()
    for r in rep.get("resumed_while_faulty", []):
        key = (r.get("node") or r.get("host"), round(float(r.get("time", 0.0)), 0))
        if key in seen:
            continue
        seen.add(key)
        out.append({"what": "resumed_while_faulty", "host": key[0], "charge": RESUMED_FAULTY_CHARGE})
    for pid, p in rep.get("parts", {}).items():
        if p.get("final") == "floor":
            out.append({"what": "part_on_floor", "part": pid, "charge": FLOOR_CHARGE})
    return out


def score(rep, safety_ok=True, violations=()):
    """The shift's record: stage_score in [0, 1] and how it came about."""
    credit_by_part = {k: 0.0 for k in WEIGHTS}
    detail = {}
    for t in rep.get("tickets", []):
        if t.get("mid_shift"):
            part, w = "mid_shift", WEIGHTS["mid_shift"]
        elif t.get("kind") == "gpu":
            part, w = "gpu", WEIGHTS["gpu"]
        elif t.get("kind") == "link":
            part, w = "link", WEIGHTS["link"]
        else:
            part, w = "hot_swap", WEIGHTS["hot_swap"]
        c, d = ticket_credit(t, w)
        credit_by_part[part] += c
        detail[t.get("id")] = dict(d, part=part, kind=t.get("kind"), cause=t.get("cause"), credit=round(c, 4))
    c, d = closing_credit(rep)
    if sum(credit_by_part.values()) <= 0.0:          # closing duties count for work done: an untouched pod earns none
        c, d = 0.0, dict(d, gated="no ticket earned credit")
    credit_by_part["closing"] = c
    detail["closing"] = d
    ch = charges(rep)
    points = sum(credit_by_part.values()) - sum(x["charge"] for x in ch)
    stage = 0.0 if not safety_ok else max(0.0, points) / TOTAL
    return {"stage_score": round(float(stage), 6), "points": round(float(points), 4), "credit": round(sum(credit_by_part.values()), 4),
            "credit_by_part": {k: round(v, 4) for k, v in credit_by_part.items()}, "charges": ch,
            "ticket_detail": detail, "safety_ok": bool(safety_ok), "violations": list(violations)[:20]}
