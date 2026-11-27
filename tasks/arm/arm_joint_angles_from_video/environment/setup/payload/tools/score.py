"""Practice tool: the scoring rule on a labelled recording, and a format check of the test estimates.

  python3 tools/score.py <estimate.csv> <labelled id>   score an estimate of a labelled recording
  python3 tools/score.py --check [<folder>]             check the files T01.csv ... of a folder
                                                        (default: the submission folder)

The scoring rule is the one of jointscore.py, the module that the grader uses. The format
check reads no log and gives no score.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import jointscore as J  # noqa: E402


def load_recordings():
    with open(os.path.join(ROOT, "recordings.json")) as f:
        return json.load(f)


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read(J.MAX_BYTES + 1)


def cmd_score(est_path, rec_id):
    rec = load_recordings()
    entry = next((r for r in rec["labelled"] if r["id"] == rec_id), None)
    if entry is None:
        raise SystemExit("no labelled recording '%s' in recordings.json" % rec_id)
    try:
        pred = J.parse_estimate(read_bytes(est_path), entry["frames"])
    except J.Malformed as exc:
        raise SystemExit("malformed estimate %s: %s" % (est_path, exc))
    t, s = J.read_log(os.path.join(ROOT, entry["log"]))
    r = J.score_recording(pred, t, s)
    print("recording %s: %d frames, %d inside the log at time offset %.4f s" % (rec_id, r["frames"], r["frames_scored"], r["offset_s"]))
    print("score (fraction of frames with all four joints within %.1f deg and the aperture within %.1f mm): %.4f (%d frames)" % (
        J.TOL_DEG, J.TOL_MM, r["score"], r["frames_correct"]))
    if r["mean_abs_error_deg"] is not None:
        print("mean absolute error per joint (deg): %s; aperture (mm): %.2f" % (
            " ".join("%.2f" % v for v in r["mean_abs_error_deg"]), r["mean_abs_error_aperture_mm"]))
    print("fraction of frames within the tolerance, q1 q2 q3 q4 aperture: %s" % " ".join("%.3f" % v for v in r["fraction_within_tol"]))


def cmd_check(folder):
    rec = load_recordings()
    good = 0
    for entry in rec["test"]:
        path = os.path.join(folder, entry["id"] + ".csv")
        if not os.path.isfile(path) or os.path.islink(path):
            print("%s: no regular file %s" % (entry["id"], path))
            continue
        try:
            pred = J.parse_estimate(read_bytes(path), entry["frames"])
        except J.Malformed as exc:
            print("%s: malformed: %s" % (entry["id"], exc))
            continue
        bad_rows = int((~J.np.isfinite(pred).all(axis=1)).sum())
        if bad_rows:
            print("%s: well formed, %d rows with a non-finite number (wrong frames)" % (entry["id"], bad_rows))
        else:
            print("%s: well formed, %d rows" % (entry["id"], entry["frames"]))
        good += 1
    print("%d of %d files are well formed" % (good, len(rec["test"])))
    return 0 if good == len(rec["test"]) else 1


def main():
    args = sys.argv[1:]
    if len(args) >= 1 and args[0] == "--check" and len(args) <= 2:
        sys.exit(cmd_check(args[1] if len(args) == 2 else os.path.join(ROOT, "submission")))
    if len(args) == 2:
        cmd_score(args[0], args[1])
        return
    raise SystemExit(__doc__.strip())


if __name__ == "__main__":
    main()
