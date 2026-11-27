"""Normalize completion against the task's recorded historical reference."""

from robotics_grader import ratio_rewards, read_anchor


def score_verdict(left, right, episodes, aborted, lock):
    if episodes <= 0 or not (0 <= left <= 6 * episodes and 0 <= right <= 7 * episodes):
        raise ValueError("invalid completion counts")
    metric = (left + right) / (13 * episodes)
    anchor, meta = read_anchor()
    return ratio_rewards(
        metric, anchor, metric_name="completion_fraction", full_at=meta["full_at"],
        locks_ok=bool(lock["match_lock_ok"]),
        extra={"left": left, "right": right,
               "left_total": 6 * episodes, "right_total": 7 * episodes,
               "episodes": episodes, "episodes_aborted": aborted, **lock},
    )
