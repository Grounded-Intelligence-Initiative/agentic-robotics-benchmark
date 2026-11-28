"""grade: the hand description graded against the photographed unit (grader only; deterministic).

    gt = load_gt(gt_dir)                                  # tests/gt: robot.xml + assets, unit_state.json, hidden_set.json
    rewards, stats, details = grade(sub_dir, gt_dir, cfg, gate=gate.gate_model_loads(sub_dir), gt=gt)
    score = description_score(rewards, cfg)

description_score = model_loads x (0.55 pinch + 0.15 single + 0.10 pose + 0.10 travrej + 0.10 force_load) (the
weights of grader_config.json). Items:
  pinch       mean over the pinch requests of score.score_request
  single      mean over the single-pad requests
  pose        mean over the pose readings x 4 digits of o x (0.7 L(pad point) + 0.3 L(pad normal))
  travrej     0.6 travel + 0.4 reject; travel = mean over the 26 driven and 6 DIP ends of L(end error); reject =
              (impossible requests rejected / their number) x (single-pad requests accepted / their number)
  force_load  0.5 force + 0.5 load; force = mean over the digits of C_d x mean over the force readings of o x
              L(|f - f_true| / f_true); load = mean over the groups (4 digits, the hand) of C_g x mean over the load
              readings of o x sqrt(L(mass) x L(centre of mass)); C_thumb = mean of every pinch score and the thumb's
              single-pad scores, C_f = mean of finger f's pinch and single-pad scores, C_hand = mean of the four
The submitted DIP's sense is read from the model (kin.dip_sense): its range ends are compared in the true sense.
The range check (before the hand is used, each servo-driven hinge is driven alone to both ends of the model's range):
for each of the 13 servo-driven hinges the model's two range ends are converted to the unit's true angles through the
submitted calibration (hidden_unit.driven_end_errors, the travel item's conversion: d_low = converted low end - true
low end, d_high = converted high end - true high end, in degrees). ONE common whole number of turns per hinge,
k = round((d_low + d_high) / 720 deg) (Python's round), is taken off both of its ends (a whole-turn calibration error
moves both ends alike and is the hard stop's to judge; any other difference is judged as stated), and each end's
over-statement is taken: low end = -(d_low - 360 k), high end = d_high - 360 k (positive beyond the true travel,
negative inside it). A hinge with an over-statement above zero_rules.range_check_deg (or one that is not finite) is
locked out, and with it its digit: every pinch and single-pad request of a digit with a locked-out hinge (a pinch
involves the thumb and its finger) scores 0 with the zero reason "locked_out" added to its own reasons. Nothing else
reads the lock: the requests' IK, the rejection formula (it counts the single-pad requests the IK accepts), the
readings, the travel item and the format gate are as without it; the factors C follow the request scores.
Indicators (stats): ind_L1_pose_err_mm (median pad error over the (pose reading, digit) pairs with o = 1; -1 when there
is no such pair),
ind_L2_turn_misses (readings at which a servo is mapped more than 180 deg outside the model's range),
ind_L3_travel_mod_turn (travel with every driven end's error taken modulo one turn),
n_locked_hinges (hinges locked out by the range check), n_locked_out (pinch and single-pad requests that carry the zero
reason "locked_out").
"""
import json
import math
import os

import numpy as np

try:
    from . import contract as C
    from . import hidden_unit as HU
    from . import kin as K
    from . import score as S
except ImportError:  # loaded as plain modules beside each other
    import contract as C
    import hidden_unit as HU
    import kin as K
    import score as S

GRADER_VERSION = "hand-description-grade/1"
GATES = ("model_loads",)
ITEMS = ("pinch", "single", "pose", "travrej", "force_load")
SUB_SCORES = ("travel", "reject", "force", "load")
SET_GROUPS = ("pose", "force", "load", "pinch", "single", "impossible")
LOAD_GROUPS = C.DIGITS + ("hand",)


class GT(object):
    """The grader's side of the task: the unit's model, the unit and the frozen hidden set."""

    def __init__(self, truth, unit, hidden_set):
        self.truth = truth
        self.unit = unit
        self.set = hidden_set


def load_config(path):
    with open(path) as fh:
        return json.load(fh)


def load_gt(gt_dir):
    truth = K.HandModel.load(os.path.join(gt_dir, "robot.xml"))
    with open(os.path.join(gt_dir, "unit_state.json")) as fh:
        unit_state = json.load(fh)
    with open(os.path.join(gt_dir, "hidden_set.json")) as fh:
        hidden_set = json.load(fh)
    return GT(truth, HU.HiddenUnit(truth, unit_state), hidden_set)


def requests_of(hidden_set, group):
    return [dict(r, group=group) for r in hidden_set.get(group, [])]


def _mean(xs):
    xs = list(xs)
    return float(sum(xs) / len(xs)) if xs else 0.0


def description_score(rewards, cfg):
    w = cfg["weights"]
    gate = 1.0
    for g in GATES:
        gate *= float(rewards.get(g, 0.0))
    return gate * sum(float(w[i]) * float(rewards[i]) for i in ITEMS) / sum(float(w[i]) for i in ITEMS)


def empty_stats():
    st = {k: 0.0 for k in SUB_SCORES}
    st.update({"consumer_" + d: 0.0 for d in C.DIGITS})
    st.update({"ind_L1_pose_err_mm": -1.0, "ind_L2_turn_misses": 0.0, "ind_L3_travel_mod_turn": 0.0,
               "n_ik_fail": 0.0, "n_hard_stop": 0.0, "n_interpen": 0.0, "n_out_of_range": 0.0, "n_loop_fail": 0.0,
               "n_locked_out": 0.0, "n_locked_hinges": 0.0})
    return st


def wrap_turn_deg(x):
    """x (degrees) modulo one turn, into (-180, 180]."""
    return float(x - 360.0 * math.ceil((x - 180.0) / 360.0))


def range_check(unit, calib, ranges, check_deg):
    """{servo-driven hinge: (low-end over-statement, high-end over-statement, locked)} in degrees: the model's range
    ends converted to the unit's true angles through the calibration (unit.driven_end_errors: d_low, d_high = the
    converted end - the true end, in degrees), one common whole number of turns k = round((d_low + d_high) / 720 deg)
    taken off both ends, the low end's over-statement = -(d_low - 360 k), the high end's = d_high - 360 k; locked when
    either is above check_deg or not finite."""
    out = {}
    for j, (dl, dh) in unit.driven_end_errors(calib, ranges).items():
        if math.isfinite(dl) and math.isfinite(dh):
            k = round((math.degrees(dl) + math.degrees(dh)) / 720.0)     # one common whole turn for both ends
            lo, hi = -(math.degrees(dl) - 360.0 * k), math.degrees(dh) - 360.0 * k
        else:
            lo = hi = float("nan")
        locked = not (math.isfinite(lo) and math.isfinite(hi)) or lo > check_deg or hi > check_deg
        out[j] = (lo, hi, bool(locked))
    return out


def digits_of_request(r, group):
    """The digits a request commands: a pinch the thumb and its finger, a single-pad request its digit."""
    return ("thumb", r["finger"]) if group == "pinch" else (r["digit"],)


def grade(sub_dir, gt_dir, cfg, *, gate, gt=None, log=None, debug=None):
    """(rewards, stats, details). gate = gate.gate_model_loads(sub_dir); gt = load_gt(gt_dir) when already loaded;
    debug = a dict that receives per-request and per-reading values (a maintainer's debug file only)."""
    log = log or (lambda s: None)
    rewards = {k: 0.0 for k in GATES + ITEMS}
    stats = empty_stats()
    details = {}
    ok, det, models = gate
    details["model_loads"] = list(det.get("violations", []))
    if not ok:
        return rewards, stats, details
    rewards["model_loads"] = 1.0
    gt = gt or load_gt(gt_dir)
    unit, hs = gt.unit, gt.set
    calib = models["calib"]
    try:
        hm = K.HandModel(models["robot"], loop_tol=float(cfg["zero_rules"]["loop_residual_m"]))
        rg = hm.ranges()
        sense = {f: hm.dip_sense(f) for f in C.FINGERS}
    except Exception as exc:  # noqa: BLE001 - a model the grader's kinematics cannot read scores 0 on every item
        details["model"] = ["the grader's kinematics cannot use this model: %s" % str(exc)[:300]]
        log("kinematics refused the model: %s" % exc)
        return rewards, stats, details
    dbg = debug if debug is not None else None

    # ---------------------------------------------------------------- the range check
    try:
        rc = range_check(unit, calib, rg, float(cfg["zero_rules"]["range_check_deg"]))
    except Exception as exc:  # noqa: BLE001 - a range the check cannot convert locks its hinge out
        log("range check raised: %s" % exc)
        rc = {j: (float("nan"), float("nan"), True) for j in C.MOTOR_ORDER}
    locked = [j for j in C.MOTOR_ORDER if rc[j][2]]
    locked_digits = {C.DIGIT_OF_JOINT[j] for j in locked}
    stats["n_locked_hinges"] = float(len(locked))
    if dbg is not None:
        dbg["range_check_over_deg"] = {j: {"low": rc[j][0], "high": rc[j][1], "locked": rc[j][2]}
                                       for j in C.MOTOR_ORDER}

    # ---------------------------------------------------------------- requests
    res = {}
    n_locked_out = 0
    for group in ("pinch", "single", "impossible"):
        out = []
        for r in requests_of(hs, group):
            try:
                rr = S.score_request(hm, calib, unit, r, cfg)
            except Exception as exc:  # noqa: BLE001 - a request that raises on this model scores 0
                log("request %s raised: %s" % (r.get("id"), exc))
                rr = S.RequestResult(0.0, ("error",), False, float("inf"), {})
            if group in S.FEASIBLE and locked_digits.intersection(digits_of_request(r, group)):
                rr = S.RequestResult(0.0, tuple(rr.zero_reasons) + ("locked_out",), rr.ik_ok, rr.residual_mm,
                                     rr.errors_mm, rr.loop_failures, rr.max_penetration_mm, rr.overshoot_deg)
                n_locked_out += 1
            out.append((r, rr))
        res[group] = out
        if dbg is not None:
            dbg[group] = [{"id": r.get("id"), "score": rr.score, "zero_reasons": list(rr.zero_reasons),
                           "ik_ok": rr.ik_ok, "residual_mm": rr.residual_mm, "errors_mm": rr.errors_mm,
                           "max_penetration_mm": rr.max_penetration_mm, "overshoot_deg": rr.overshoot_deg,
                           "loop_failures": rr.loop_failures} for r, rr in out]
    pinch = [rr.score for _, rr in res["pinch"]]
    single = [rr.score for _, rr in res["single"]]
    rewards["pinch"] = _mean(pinch)
    rewards["single"] = _mean(single)
    feas = res["pinch"] + res["single"]
    stats["n_ik_fail"] = float(sum(1 for _, rr in feas if "ik" in rr.zero_reasons or "error" in rr.zero_reasons))
    stats["n_hard_stop"] = float(sum(1 for _, rr in feas if "hard_stop" in rr.zero_reasons))
    stats["n_interpen"] = float(sum(1 for _, rr in feas if "interpen" in rr.zero_reasons))
    stats["n_locked_out"] = float(n_locked_out)
    n_loop = sum(1 for _, rr in res["pinch"] + res["single"] + res["impossible"] if rr.loop_failures)
    consumer = {}
    for d in C.DIGITS:
        if d == "thumb":
            vals = pinch + [rr.score for r, rr in res["single"] if r["digit"] == "thumb"]
        else:
            vals = ([rr.score for r, rr in res["pinch"] if r["finger"] == d]
                    + [rr.score for r, rr in res["single"] if r["digit"] == d])
        consumer[d] = _mean(vals)
        stats["consumer_" + d] = consumer[d]
    consumer["hand"] = _mean(consumer[d] for d in C.DIGITS)
    n_imp = len(res["impossible"])
    rejected = sum(1 for _, rr in res["impossible"] if rr.score == 1.0)       # an error is not a rejection
    accepted = sum(1 for _, rr in res["single"] if rr.ik_ok)
    reject = (rejected / n_imp if n_imp else 0.0) * (accepted / len(res["single"]) if res["single"] else 0.0)

    # ---------------------------------------------------------------- travel ends
    try:
        ends = unit.travel_ends_error(calib, rg, dip_sense=sense)
        ends_mod = unit.travel_ends_error(calib, rg, dip_sense=sense, modulo_turn=True)
        flat = [e for j in C.MOTOR_ORDER + C.DIP_JOINTS for e in ends[j]]
        flat_mod = [e for j in C.MOTOR_ORDER + C.DIP_JOINTS for e in ends_mod[j]]
        travel = _mean(S.band(cfg, "end", e) for e in flat)
        stats["ind_L3_travel_mod_turn"] = _mean(S.band(cfg, "end", e) for e in flat_mod)
        if dbg is not None:
            dbg["travel_end_err_deg"] = {j: list(ends[j]) for j in ends}
    except Exception as exc:  # noqa: BLE001
        log("travel ends raised: %s" % exc)
        travel = 0.0
    sw = cfg["sub_weights"]
    stats["travel"], stats["reject"] = travel, reject
    rewards["travrej"] = float(sw["travel"]) * travel + float(sw["reject"]) * reject

    # ---------------------------------------------------------------- readings
    n_oor = {g: 0 for g in ("pose", "force", "load")}      # out-of-range (reading, digit) pairs, per reading group
    n_lf = {g: 0 for g in ("pose", "force", "load")}       # pairs whose closed chain does not close, per group
    n_turn = 0
    pose_vals, pose_err = [], []
    force_acc = {d: [] for d in C.DIGITS}
    load_acc = {g: [] for g in LOAD_GROUPS}
    rd_dbg = {"pose": [], "force": [], "load": []}
    for group, what in (("pose", ("pose",)), ("force", ("force",)), ("load", ("load",))):
        for rec in hs.get(group, []):
            try:
                rr = S.score_reading(hm, calib, unit, rec["k"], cfg, what=what)
            except Exception as exc:  # noqa: BLE001
                log("reading %s raised: %s" % (rec.get("id"), exc))
                rr = None
            if rr is None:
                if group == "pose":
                    pose_vals += [0.0] * len(C.DIGITS)
                elif group == "force":
                    for d in C.DIGITS:
                        force_acc[d].append(0.0)
                else:
                    for g in LOAD_GROUPS:
                        load_acc[g].append(0.0)
                continue
            n_oor[group] += sum(1 for d in C.DIGITS if rr.out_of_range[d])
            n_lf[group] += sum(1 for d in C.DIGITS if rr.loop_fail[d])
            n_turn += 1 if rr.turn_miss else 0
            if group == "pose":
                for d in C.DIGITS:
                    pose_vals.append(S.pose_item(cfg, rr, d))
                    if rr.o[d]:
                        pose_err.append(rr.pos_err_mm[d])
            elif group == "force":
                for d in C.DIGITS:
                    force_acc[d].append(S.force_item(cfg, rr, d))
            else:
                for g in LOAD_GROUPS:
                    load_acc[g].append(S.load_item(cfg, rr, g))
            if dbg is not None:
                rd_dbg[group].append({"id": rec.get("id"), "o": dict(rr.o), "pos_err_mm": dict(rr.pos_err_mm),
                                      "nrm_err_deg": dict(rr.nrm_err_deg), "force": dict(rr.force),
                                      "force_true": dict(rr.force_true), "turn_miss": rr.turn_miss})
    if dbg is not None:
        dbg["readings"] = rd_dbg
    rewards["pose"] = _mean(pose_vals)
    force = _mean(consumer[d] * _mean(force_acc[d]) for d in C.DIGITS)
    load = _mean(consumer[g] * _mean(load_acc[g]) for g in LOAD_GROUPS)
    stats["force"], stats["load"] = force, load
    rewards["force_load"] = float(sw["force"]) * force + float(sw["load"]) * load
    stats["ind_L1_pose_err_mm"] = float(np.median(pose_err)) if pose_err else -1.0     # -1: no pair with o = 1
    stats["ind_L2_turn_misses"] = float(n_turn)
    stats["n_out_of_range"] = float(sum(n_oor.values()))
    stats["n_loop_fail"] = float(sum(n_lf.values()) + n_loop)
    for k in ITEMS:
        rewards[k] = float(min(1.0, max(0.0, rewards[k])))
    req_line = "%d requests; %d without an IK solution, %d hard stops, %d interpenetrations, %d locked out"
    details["pinch"] = [req_line % (
        len(pinch), sum(1 for _, rr in res["pinch"] if not rr.ik_ok),
        sum(1 for _, rr in res["pinch"] if "hard_stop" in rr.zero_reasons),
        sum(1 for _, rr in res["pinch"] if "interpen" in rr.zero_reasons),
        sum(1 for _, rr in res["pinch"] if "locked_out" in rr.zero_reasons))]
    details["single"] = [req_line % (
        len(single), sum(1 for _, rr in res["single"] if not rr.ik_ok),
        sum(1 for _, rr in res["single"] if "hard_stop" in rr.zero_reasons),
        sum(1 for _, rr in res["single"] if "interpen" in rr.zero_reasons),
        sum(1 for _, rr in res["single"] if "locked_out" in rr.zero_reasons))]
    details["travrej"] = ["%d of %d impossible requests rejected, %d of %d single-pad requests accepted; range check: "
                          "%d hinges locked out%s" % (rejected, n_imp, accepted, len(res["single"]), len(locked),
                                                      (" (%s)" % ", ".join(locked)) if locked else "")]
    details["pose"] = ["%d (reading, digit) pairs of the pose readings outside the model's range, %d with an open "
                       "closed chain" % (n_oor["pose"], n_lf["pose"])]
    details["force_load"] = ["force readings: %d (reading, digit) pairs outside the model's range, %d with an open "
                             "closed chain; load readings: %d and %d" % (n_oor["force"], n_lf["force"], n_oor["load"],
                                                                         n_lf["load"])]
    return rewards, stats, details
