"""The use of the pick table: what the twin's planned picks achieve in the true scene, with protective stops charged.

    res = use(per_request, requests_json, cfg)
        per_request: {request id: {"done": share, "stop": share[, "declined": share]}}   (shares of the placements)
        requests_json: tests/gt/requests.json (only its "requests", the scored ones, are read)
        cfg: {"c": stop charge, "thing_cap": the cap's floor value (see Weights)}
    -> {"use", "done_share", "stop_share", "declined_share", "units": {...}, "weights": {...}}

Units are (thing, rule) pairs with at least one scored request. A unit with a creditable request is a credit unit
(U+), the others are charge-only units (U0: all their requests are stop-only).
    done_u = the mean done share over u's creditable requests;
    stop_u = the mean stop share over ALL of u's scored requests (stops count on creditable and stop-only requests
             alike).
Weights: w0 = 1 / |U+|; a thing's weight W_t = n_t w0 (n_t = its credit units), capped at max(thing_cap, 1 / n) (n =
the number of things with a credit unit, so the cap can always hold), the excess spread over the uncapped things in
proportion to n_t until no thing exceeds the cap (water-filling); w_u = W_t / n_t.
    use = max(0, sum over U+ of w_u (done_u - c stop_u) - sum over U0 of w0 c stop_u)
A twin that declines everything uses nothing (0); the reference (every creditable request done at every placement,
nothing stopped) uses exactly 1.
Shares reported beside the use (weighted as the use): done_share = sum over U+ of w_u done_u; stop_share = sum over
U+ of w_u stop_u + sum over U0 of w0 stop_u (so use = max(0, done_share - c stop_share)); declined_share = sum over U+
of w_u declined_u, declined_u the mean declined share over u's creditable requests (when the declined shares are
given).
The arithmetic is exact (rational numbers from the given floats and from the decimal text of c and thing_cap), so the
reference's 1 is exactly 1.0 and two runs give the same bits; results are returned as floats.

The request set's stop line (request_set_counts, check_request_set): at least request_set.min_credit_things things
with a credit unit and at least request_set.min_credit_units credit units (grader_config); below it the grader stops
with a task error at verify (the self-test), never a score.
"""
from fractions import Fraction


class CapInfeasible(ValueError):
    """The thing cap cannot hold: fewer credit things than 1 / cap (thing_weights called with a fixed cap)."""


class RequestSetBelowStopLine(ValueError):
    """Fewer things with a credit unit, or fewer credit units, than the grader configuration's request_set needs."""


def _frac(x):
    if isinstance(x, Fraction):
        return x
    if isinstance(x, int):
        return Fraction(x)
    return Fraction(float(x))


def _decimal(x):
    """A configuration number as the decimal it is written as (0.15 -> 3/20)."""
    if isinstance(x, (int, Fraction)):
        return Fraction(x)
    return Fraction(repr(float(x)))


def unit_key(thing, rule):
    return "%s|%s" % (thing, rule)


def units_of(requests_json):
    """{unit key: {"thing", "rule", "creditable": [ids], "scored": [ids]}} in the order of first appearance."""
    units = {}
    for r in requests_json["requests"]:
        u = units.setdefault(unit_key(r["thing"], r["rule"]),
                             {"thing": r["thing"], "rule": r["rule"], "creditable": [], "scored": []})
        u["scored"].append(r["id"])
        if r["class"] == "creditable":
            u["creditable"].append(r["id"])
        elif r["class"] != "stop_only":
            raise ValueError("request %s has class %r" % (r["id"], r["class"]))
    return units


def thing_weights(n_by_thing, cap):
    """{thing: W_t (Fraction)} by water-filling: W_t = n_t / N, capped at `cap` (a Fraction), the excess spread over
    the uncapped things in proportion to n_t until none exceeds the cap."""
    things = [t for t, n in n_by_thing.items() if n > 0]
    if not things:
        return {}
    if cap * len(things) < 1:
        raise CapInfeasible("%d credit things cannot hold a cap of %s each" % (len(things), cap))
    N = sum(n_by_thing[t] for t in things)
    W = {t: Fraction(n_by_thing[t], N) for t in things}
    capped = set()
    while True:
        over = [t for t in things if t not in capped and W[t] > cap]
        if not over:
            return W
        capped.update(over)
        free = [t for t in things if t not in capped]
        rest = 1 - cap * len(capped)
        for t in capped:
            W[t] = cap
        if not free:
            return W
        S = sum(n_by_thing[t] for t in free)
        for t in free:
            W[t] = rest * Fraction(n_by_thing[t], S)


def effective_cap(thing_cap, n_things):
    """The cap a thing's weight may reach: max(thing_cap, 1 / n), n = the things with a credit unit (Fractions)."""
    cap = _decimal(thing_cap)
    return max(cap, Fraction(1, n_things)) if n_things > 0 else cap


def weights(requests_json, cfg):
    """(units, {"w0", "things": {t: W_t}, "units": {u: w_u}, "cap": the effective cap}) with Fractions."""
    units = units_of(requests_json)
    plus = [k for k, u in units.items() if u["creditable"]]
    if not plus:
        return units, {"w0": Fraction(0), "things": {}, "units": {k: Fraction(0) for k in units},
                       "cap": _decimal(cfg["thing_cap"])}
    w0 = Fraction(1, len(plus))
    n_t = {}
    for k in plus:
        n_t[units[k]["thing"]] = n_t.get(units[k]["thing"], 0) + 1
    cap = effective_cap(cfg["thing_cap"], len(n_t))
    W = thing_weights(n_t, cap)
    wu = {}
    for k, u in units.items():
        wu[k] = W[u["thing"]] / n_t[u["thing"]] if u["creditable"] else w0
    return units, {"w0": w0, "things": W, "units": wu, "cap": cap}


def request_set_counts(requests_json):
    """{"credit_units": |U+|, "credit_things": the number of things with a credit unit}."""
    units = units_of(requests_json)
    plus = [u for u in units.values() if u["creditable"]]
    return {"credit_units": len(plus), "credit_things": len({u["thing"] for u in plus})}


def check_request_set(requests_json, cfg):
    """The counts; RequestSetBelowStopLine when they are under cfg["request_set"]'s minimums."""
    rs = cfg["request_set"]
    got = request_set_counts(requests_json)
    lo_t, lo_u = int(rs["min_credit_things"]), int(rs["min_credit_units"])
    if got["credit_things"] < lo_t or got["credit_units"] < lo_u:
        raise RequestSetBelowStopLine("%d credit units on %d things; the stop line needs at least %d units on at "
                                      "least %d things" % (got["credit_units"], got["credit_things"], lo_u, lo_t))
    return got


def _mean(vals):
    vals = list(vals)
    return sum(vals, Fraction(0)) / len(vals) if vals else Fraction(0)


def use(per_request, requests_json, cfg):
    """The use (see the module text); every scored request must be in per_request."""
    c = _decimal(cfg["c"])
    units, w = weights(requests_json, cfg)

    def share(rid, key):
        row = per_request[rid]
        v = _frac(row[key])
        if v < 0 or v > 1:
            raise ValueError("request %s: %s share %s outside [0, 1]" % (rid, key, float(v)))
        return v

    have_declined = all("declined" in per_request.get(r["id"], {}) for r in requests_json["requests"])
    total = Fraction(0)
    done_share = stop_share = declined_share = Fraction(0)
    out_units = {}
    for k, u in units.items():
        stop_u = _mean(share(r, "stop") for r in u["scored"])
        if u["creditable"]:
            done_u = _mean(share(r, "done") for r in u["creditable"])
            dec_u = _mean(share(r, "declined") for r in u["creditable"]) if have_declined else None
            wk = w["units"][k]
            total += wk * (done_u - c * stop_u)
            done_share += wk * done_u
            stop_share += wk * stop_u
            if dec_u is not None:
                declined_share += wk * dec_u
            kind = "credit"
        else:
            done_u, dec_u = Fraction(0), None
            wk = w["w0"]
            total -= wk * c * stop_u
            stop_share += wk * stop_u
            kind = "charge"
        out_units[k] = {"thing": u["thing"], "rule": u["rule"], "kind": kind, "w": float(wk),
                        "done": float(done_u), "stop": float(stop_u),
                        "declined": None if dec_u is None else float(dec_u),
                        "n_creditable": len(u["creditable"]), "n_scored": len(u["scored"])}
    value = total if total > 0 else Fraction(0)
    return {"use": float(value), "use_raw": float(total), "done_share": float(done_share),
            "stop_share": float(stop_share), "declined_share": float(declined_share) if have_declined else None,
            "units": out_units,
            "weights": {"w0": float(w["w0"]), "things": {t: float(v) for t, v in w["things"].items()},
                        "c": float(c), "thing_cap": float(_decimal(cfg["thing_cap"])),
                        "thing_cap_effective": float(w["cap"])}}
