"""Scoring of per-event interaction estimates against the answers.

One module, used unchanged by the grader and by the practice tool (tools/score.py).

An estimate file is CSV: the header line `clip,interaction_type,peak_intensity`, then one line per event, in any order.
Two answers of each event are checked:

1. `interaction_type` equals the answer (one of TYPES, IEDD's labels);
2. `peak_intensity` is within TOL_PEAK of the answer: the largest IEDD intensity Q_i(t) of either vehicle of the event
   over the event's interval.

An event is correct when both checks pass. An event missing from the file is wrong, and a number that is not finite
fails its check. The metric `interaction_accuracy` is the fraction of correct events. (Either check alone would leave
a guess too much credit: one constant interaction type is right for a quarter of a balanced test set.)
"""
import csv
import io
import math

HEADER = ["clip", "interaction_type", "peak_intensity"]
TYPES = ("Car-follow", "Merging", "Crossing", "Head-on")
CHECKS = ("type", "peak_intensity")

# Q lies in [0, 1]. Over the 374 pair events of the IEDD repository's sample annotations the peak has a standard
# deviation of 0.144, and the median of each type is within 0.05 of 19% of the peaks. TODO(author): confirm the
# tolerance with an expert pipeline on the labelled events and state the result in the pull request.
TOL_PEAK = 0.05

MAX_BYTES = 1024 * 1024


class Malformed(Exception):
    """An estimate file that does not follow the format."""


def _rows(text):
    rows = list(csv.reader(io.StringIO(text.replace("\r\n", "\n"))))
    while rows and not any(c.strip() for c in rows[-1]):
        rows.pop()
    return rows


def parse_estimate(data, clip_ids):
    """Bytes of an estimate file -> {clip: (type, peak_intensity)}. Raises Malformed.

    Every line must name one of `clip_ids`, at most once; events left out are simply absent from the result.
    """
    if len(data) > MAX_BYTES:
        raise Malformed("larger than %d bytes" % MAX_BYTES)
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise Malformed("not ASCII text")
    rows = _rows(text)
    if not rows or [c.strip() for c in rows[0]] != HEADER:
        raise Malformed("the first line is not '%s'" % ",".join(HEADER))
    wanted = set(clip_ids)
    out = {}
    for line_no, row in enumerate(rows[1:], start=2):
        if len(row) != len(HEADER):
            raise Malformed("line %d has %d fields, not %d" % (line_no, len(row), len(HEADER)))
        clip, kind, peak = [c.strip() for c in row]
        if clip not in wanted:
            raise Malformed("line %d names an unknown event %r" % (line_no, clip[:40]))
        if clip in out:
            raise Malformed("event %s appears twice" % clip)
        try:
            out[clip] = (kind, float(peak))
        except ValueError:
            raise Malformed("line %d: peak_intensity is not a number" % line_no)
    return out


def read_answers(path):
    """An answers file (the deliverable's format) -> {clip: (type, peak_intensity)}."""
    with open(path, newline="") as f:
        rows = _rows(f.read())
    if not rows or [c.strip() for c in rows[0]] != HEADER:
        raise ValueError("%s does not start with the header '%s'" % (path, ",".join(HEADER)))
    out = {}
    for row in rows[1:]:
        clip, kind, peak = [c.strip() for c in row]
        if kind not in TYPES:
            raise ValueError("%s: event %s has an unknown interaction type %r" % (path, clip, kind))
        out[clip] = (kind, float(peak))
    return out


def score_event(pred, truth):
    """pred (or None) and truth: (type, peak) -> the two checks and the event score (0 or 1)."""
    t_kind, t_peak = truth
    res = {k: False for k in CHECKS}
    if pred is not None:
        p_kind, p_peak = pred
        res["type"] = p_kind == t_kind
        res["peak_intensity"] = math.isfinite(p_peak) and abs(p_peak - t_peak) <= TOL_PEAK
    res["score"] = 1.0 if all(res[k] for k in CHECKS) else 0.0
    return res


def score_all(pred, answers):
    """Fraction of correct events over the events of `answers`; `pred` may lack some. Returns (metric, per event)."""
    per = {clip: score_event(pred.get(clip), truth) for clip, truth in answers.items()}
    return sum(r["score"] for r in per.values()) / len(per), per
