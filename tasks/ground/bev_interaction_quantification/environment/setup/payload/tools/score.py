"""Practice scorer: the grader's rule on the labelled events, and a format check of your submission.

  python3 tools/score.py --check [submission/answers.csv]   check the file's format against the test events
  python3 tools/score.py <estimates.csv>                     score your estimates of the labelled events

The estimates file has the deliverable's format; for the labelled events, name them by their ids in clips.json.
"""
import json
import os
import sys

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HOME, "tools"))

import iedd_score as S  # noqa: E402


def load_clips():
    with open(os.path.join(HOME, "clips.json")) as f:
        return json.load(f)


def main(argv):
    clips = load_clips()
    if argv and argv[0] == "--check":
        path = argv[1] if len(argv) > 1 else os.path.join(HOME, "submission", "answers.csv")
        ids = [e["id"] for e in clips["test"]]
        with open(path, "rb") as f:
            pred = S.parse_estimate(f.read(), ids)
        bad = sorted(c for c, v in pred.items() if v[0] not in S.TYPES)
        missing = [c for c in ids if c not in pred]
        print("format OK: %d of %d test events present" % (len(pred), len(ids)))
        if missing:
            print("missing (they score 0): " + " ".join(missing))
        if bad:
            print("interaction_type not one of %s: %s" % (", ".join(S.TYPES), " ".join(bad)))
        return 0
    if len(argv) != 1:
        print(__doc__)
        return 2
    answers = S.read_answers(os.path.join(HOME, "labelled", "answers.csv"))
    with open(argv[0], "rb") as f:
        pred = S.parse_estimate(f.read(), list(answers))
    metric, per = S.score_all(pred, answers)
    for k in S.CHECKS:
        print("%-10s %.3f" % (k, sum(1 for r in per.values() if r[k]) / len(per)))
    print("interaction_accuracy on %d labelled events: %.4f" % (len(per), metric))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
