"""messy_desk_score — the episode score, computed from the simulator's privileged state.

Per step and object: w * (v(t) - v(0)), where v is +1 resting in the object's own bin
(the red bin for the requested object while its request is open), -1 resting in another
bin, 0 otherwise; removed objects keep their last value. The requested object weighs
W_TARGET while its request is open.

Requests are closed by the agent's own mark (action[7]: 1 = "done: delivered", 2 = "done:
no such object") and judged against the physical truth. One-time terms, in units of the
horizon H so that they stay comparable to the sorting term:

    delivered and the object really rests in the red bin    +BONUS_REQUEST * H
    "no such object" and the object really is absent        +BONUS_REQUEST * H
    either mark when the truth is the opposite              -PENALTY_WRONG * H
    deadline passes with no mark                             0
    first gripper contact with a requested object           +BONUS_CONTACT * H

    score = max(0, sum) / H

Stdlib-only; shared byte-identically by the verify stage and the practice grader.
"""

W_TARGET = 2.0
BONUS_REQUEST = 0.5      # fraction of the horizon
PENALTY_WRONG = 0.5
BONUS_CONTACT = 0.05


class EpisodeScorer(object):
    def __init__(self, horizon):
        self.horizon = float(horizon)
        self.total = 0.0
        self.steps = 0
        self.counts = {"arrivals": 0, "requests_opened": 0, "requests_delivered": 0, "claims_wrong": 0,
                       "requests_expired": 0, "absent_correct": 0, "absent_wrong": 0,
                       "first_contacts": 0, "bins_cleared": 0, "disturbances": 0, "fell_off_desk": 0}

    def add_step(self, state):
        req = state.get("request") or {}
        target = req.get("object") if req.get("status") == "open" else None
        for o in state.get("objects", []):
            w = W_TARGET if (target is not None and o["id"] == target) else 1.0
            self.total += w * (float(o.get("value", 0)) - float(o.get("baseline", 0)))
        H = self.horizon
        for e in state.get("events", []):
            kind = e.get("event", "")
            if kind == "request_first_contact":
                self.total += BONUS_CONTACT * H; self.counts["first_contacts"] += 1
            elif kind == "request_delivered":
                self.total += BONUS_REQUEST * H; self.counts["requests_delivered"] += 1
            elif kind == "request_claim_wrong":
                self.total -= PENALTY_WRONG * H; self.counts["claims_wrong"] += 1
            elif kind == "request_absent_correct":
                self.total += BONUS_REQUEST * H; self.counts["absent_correct"] += 1
            elif kind == "request_absent_wrong":
                self.total -= PENALTY_WRONG * H; self.counts["absent_wrong"] += 1
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
