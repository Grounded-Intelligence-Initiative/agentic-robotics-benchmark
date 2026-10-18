"""messy_desk_score — the episode score, computed from the simulator's privileged state.

Per step and object: w * (v(t) - v(0)), where v is +1 resting in the object's own bin
(the red bin for the requested object while its request is open), -1 resting in another
bin, 0 otherwise; removed objects keep their last value. The requested object weighs
W_TARGET while its request is open. One-time bonuses: the first gripper contact with a
requested object, a correct "absent" declaration; a wrong declaration costs the same.

    score = max(0, sum) / horizon

Stdlib-only; shared byte-identically by the verify stage and the practice grader.
"""

W_TARGET = 2.0
BONUS_CONTACT = 200.0
BONUS_ABSENT = 200.0
PENALTY_ABSENT_WRONG = 200.0


class EpisodeScorer(object):
    def __init__(self, horizon):
        self.horizon = float(horizon)
        self.total = 0.0
        self.steps = 0
        self.counts = {"arrivals": 0, "requests_opened": 0, "requests_delivered": 0,
                       "requests_expired": 0, "absent_correct": 0, "absent_wrong": 0,
                       "first_contacts": 0, "bins_cleared": 0, "disturbances": 0, "fell_off_desk": 0}

    def add_step(self, state):
        req = state.get("request") or {}
        target = req.get("object") if req.get("status") == "open" else None
        for o in state.get("objects", []):
            w = W_TARGET if (target is not None and o["id"] == target) else 1.0
            self.total += w * (float(o.get("value", 0)) - float(o.get("baseline", 0)))
        for e in state.get("events", []):
            kind = e.get("event", "")
            if kind == "request_first_contact":
                self.total += BONUS_CONTACT; self.counts["first_contacts"] += 1
            elif kind == "request_absent_correct":
                self.total += BONUS_ABSENT; self.counts["absent_correct"] += 1
            elif kind == "request_absent_wrong":
                self.total -= PENALTY_ABSENT_WRONG; self.counts["absent_wrong"] += 1
            elif kind == "request_delivered":
                self.counts["requests_delivered"] += 1
            elif kind == "request_expired":
                self.counts["requests_expired"] += 1
            elif kind == "request_opened":
                self.counts["requests_opened"] += 1
            elif kind == "arrival":
                self.counts["arrivals"] += 1
            elif kind == "bin_cleared":
                self.counts["bins_cleared"] += 1
            elif kind.startswith("disturb_"):
                self.counts["disturbances"] += 1
            elif kind == "fell_off_desk":
                self.counts["fell_off_desk"] += 1
        self.steps += 1

    def score(self):
        return max(0.0, self.total) / self.horizon

    def summary(self):
        d = dict(self.counts)
        d.update(score=self.score(), raw_sum=self.total, steps=self.steps)
        return d
