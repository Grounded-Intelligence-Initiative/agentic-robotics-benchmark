"""messy_desk_score — the episode score, computed from the simulator's privileged state.

    S = 1/2 * O + 1/2 * R          (scaled by steps/H when the episode was cut short)

O, organization (anytime): every step, every object contributes v_o(t) - b_o, where v is +1
placed in a right container, -1 placed in a wrong container, 0 on the table / held / in flight,
and b_o is the object's baseline, the value it had when it entered the scene or when the
world last moved it. Summed over objects and steps and divided by the gain that was
available, sum_t sum_o max(0, 1 - b_o), then clipped to [0, 1]. An untouched scene scores
exactly 0. Removed objects keep their last value (container service, delivery) or -1 (lost
off the table) for the rest of the episode, also when their instance is reused.

R, request progress: points divided by twice the number of opened requests, including an ongoing one.
The first hold of the requested target earns 1 point once; a verified delivery or correct
absence declaration brings that request to 2 points. Repeated holds earn nothing more.
A wrong mark or expiry keeps any first-hold credit but adds none. No requests gives 0.

Stdlib-only; shared byte-identically by the verify stage and the practice grader.
"""

W_ORG, W_REQ = 0.5, 0.5
SUCCESS_EVENTS = ("request_delivered", "request_absent_correct")
FAILURE_EVENTS = ("request_claim_wrong", "request_absent_wrong", "request_expired")


class EpisodeScorer(object):
    def __init__(self, horizon):
        self.horizon = float(horizon)
        self.steps = 0
        self.num = 0.0            # sum of (v - b)
        self.den = 0.0            # sum of the gain available, max(0, 1 - b)
        self.ghosts = []          # (value, baseline) of removed incarnations whose instance was reused
        self.prev = {}            # id -> (present, removed, value, baseline) at the previous step
        self.request_points = {}  # request id -> 0, 1 (first hold), or 2 (success)
        self.held_requests = set()
        self.counts = {"requests_opened": 0, "requests_success": 0, "requests_failed": 0,
                       "requests_delivered": 0, "absent_correct": 0, "claims_wrong": 0, "absent_wrong": 0,
                       "requests_expired": 0, "first_holds": 0, "arrivals": 0, "containers_cleared": 0,
                       "disturbances": 0, "objects_lost": 0}

    def add_step(self, state):
        for o in state.get("objects", []):
            oid = o["id"]
            v, b = float(o.get("value", 0)), float(o.get("baseline", 0))
            was = self.prev.get(oid)
            if was is not None and was[1] and o.get("present") and not o.get("removed"):
                self.ghosts.append((was[2], was[3]))       # the removed incarnation keeps counting
            self.prev[oid] = (bool(o.get("present")), bool(o.get("removed")), v, b)
            self.num += v - b
            self.den += max(0.0, 1.0 - b)
        for v, b in self.ghosts:
            self.num += v - b
            self.den += max(0.0, 1.0 - b)
        for e in state.get("events", []):
            kind = e.get("event", "")
            if kind == "request_opened" or kind == "request_first_hold" or kind in SUCCESS_EVENTS or kind in FAILURE_EVENTS:
                rid = e["request"]
                self.request_points.setdefault(rid, 0.0)
                if kind == "request_first_hold":
                    self.request_points[rid] = max(self.request_points[rid], 1.0)
                    if rid not in self.held_requests:
                        self.held_requests.add(rid)
                        self.counts["first_holds"] += 1
                elif kind in SUCCESS_EVENTS:
                    self.request_points[rid] = 2.0
            if kind in SUCCESS_EVENTS:
                self.counts["requests_success"] += 1
            elif kind in FAILURE_EVENTS:
                self.counts["requests_failed"] += 1
            if kind == "request_delivered":
                self.counts["requests_delivered"] += 1
            elif kind == "request_absent_correct":
                self.counts["absent_correct"] += 1
            elif kind == "request_claim_wrong":
                self.counts["claims_wrong"] += 1
            elif kind == "request_absent_wrong":
                self.counts["absent_wrong"] += 1
            elif kind == "request_expired":
                self.counts["requests_expired"] += 1
            elif kind == "request_opened":
                self.counts["requests_opened"] += 1
            elif kind == "arrival":
                self.counts["arrivals"] += 1
            elif kind == "container_cleared":
                self.counts["containers_cleared"] += 1
            elif kind.startswith("disturb_"):
                self.counts["disturbances"] += 1
            elif kind == "fell_off_desk":
                self.counts["objects_lost"] += 1
        self.steps += 1

    def organization(self):
        if self.den <= 0:
            return 0.0
        return min(1.0, max(0.0, self.num / self.den))

    def request_rate(self):
        closed = self.counts["requests_success"] + self.counts["requests_failed"]
        return self.counts["requests_success"] / float(closed) if closed else 0.0

    def request_progress(self):
        opened = len(self.request_points)
        return sum(self.request_points.values()) / (2.0 * opened) if opened else 0.0

    def score(self):
        s = W_ORG * self.organization() + W_REQ * self.request_progress()
        if self.steps < self.horizon:                    # cut short: the steps never run earn nothing
            s *= self.steps / self.horizon
        return s

    def live(self):
        """Compact running view for the practice tools: `score` is what the episode would score if
        it were cut now (steps/H scaling included), `score_running` the unscaled 1/2 O + 1/2 R."""
        closed = self.counts["requests_success"] + self.counts["requests_failed"]
        return {"score": round(self.score(), 5), "score_running": round(W_ORG * self.organization() + W_REQ * self.request_progress(), 5),
                "organization": round(self.organization(), 5), "request_rate": round(self.request_rate(), 5),
                "request_progress": round(self.request_progress(), 5),
                "org_num": round(self.num, 1), "org_den": round(self.den, 1),
                "requests_success": self.counts["requests_success"], "requests_closed": closed,
                "requests_open": self.counts["requests_opened"] - closed, "steps": self.steps, "horizon": int(self.horizon)}

    def summary(self):
        d = dict(self.counts)
        d.update(score=self.score(), organization=self.organization(), request_rate=self.request_rate(),
                 request_progress=self.request_progress(),
                 org_num=self.num, org_den=self.den, steps=self.steps)
        return d
