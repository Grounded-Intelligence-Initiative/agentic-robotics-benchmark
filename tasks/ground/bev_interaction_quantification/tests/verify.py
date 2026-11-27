"""Hidden grader for bev_interaction_quantification.

Reads /home/user/submission/answers.csv (names and numbers only, never through a link; nothing the agent wrote is
executed) and scores it against the hidden answers (tests/ref/answers.csv) with iedd_score.py, the module the agent
has a byte-identical copy of. Metric `interaction_accuracy`: the fraction of the test events whose two answers
(interaction type, peak intensity) are both correct (higher is better). A missing or malformed file scores 0.
Separate verifier sandbox.
"""
import json
import os
import stat
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import iedd_score as S  # noqa: E402
from robotics_grader import ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
REF = os.path.join(HERE, "ref")
METRIC = "interaction_accuracy"


def read_regular(dfd, name):
    """Bytes of the regular file `name` of the open folder dfd (no link followed), else None."""
    try:
        fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=dfd)
    except OSError:
        return None
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return None
        chunks, total = [], 0
        while total <= S.MAX_BYTES:
            b = os.read(fd, 65536)
            if not b:
                break
            chunks.append(b)
            total += len(b)
        return b"".join(chunks)
    finally:
        os.close(fd)


def main():
    t0 = time.time()
    config = stage_config()
    anchor, meta = read_anchor()
    anchored = anchor is not None
    with open(os.path.join(REF, "clips.json")) as f:
        test_ids = [e["id"] for e in json.load(f)["test"]]
    answers = S.read_answers(os.path.join(REF, "answers.csv"))
    answers = {c: answers[c] for c in test_ids}
    try:
        dfd = os.open(os.path.join(HOME, "submission"), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError:
        write_verdict(zero_verdict(reason="no submission directory", anchored=anchored))
        return
    try:
        data = read_regular(dfd, "answers.csv")
    finally:
        os.close(dfd)
    if data is None:
        write_verdict(zero_verdict(reason="no answers.csv in the submission directory", anchored=anchored))
        return
    try:
        pred = S.parse_estimate(data, test_ids)
    except S.Malformed as exc:
        write_verdict(zero_verdict(reason="answers.csv is malformed: %s" % str(exc)[:160], anchored=anchored))
        return
    measured, per = S.score_all(pred, answers)
    extra = {"score_" + c: round(per[c]["score"], 4) for c in test_ids}
    for k in S.CHECKS:
        extra["accuracy_" + k] = round(sum(1 for c in test_ids if per[c][k]) / len(test_ids), 4)
    missing = sum(1 for c in test_ids if c not in pred)
    extra["events_missing"] = float(missing)
    extra["grading_seconds"] = round(time.time() - t0, 1)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=METRIC, extra=extra, full_at=meta.get("full_at", 1.0))
    text = "%s=%.4f over %d events (%d missing); %s" % (
        METRIC, measured, len(test_ids), missing,
        ", ".join("%s %.2f" % (k, extra["accuracy_" + k]) for k in S.CHECKS))
    write_verdict(verdict, diagnostic=text)
    print("[grader] %s anchor=%s reward=%.3f %.1fs" % (text, anchor, verdict["rewards"]["reward"], time.time() - t0),
          file=sys.stderr)


if __name__ == "__main__":
    main()
