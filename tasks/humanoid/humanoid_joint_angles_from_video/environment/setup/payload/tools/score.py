"""Practice tool: the scoring rule on a labelled recording, and a format check of the test estimates.

  python3 tools/score.py <estimate.csv> <labelled id>   score an estimate of a labelled recording (L01 ... L06)
  python3 tools/score.py --check [<folder>]             check the files T01.csv ... of a folder
                                                        (default: the submission folder)

The scoring rule is the one of humanoidscore.py, the module that the grader uses. The format check reads no true
angles and gives no score.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import humanoidscore as H  # noqa: E402


def load_recordings():
    with open(os.path.join(ROOT, "recordings.json")) as f:
        return json.load(f)


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read(H.MAX_BYTES + 1)


def cmd_score(est_path, rec_id):
    rec = load_recordings()
    entry = next((r for r in rec["labelled"] if r["id"] == rec_id), None)
    if entry is None:
        raise SystemExit("no labelled recording '%s' in recordings.json" % rec_id)
    try:
        pred = H.parse_estimate(read_bytes(est_path), entry["frames"])
    except H.Malformed as exc:
        raise SystemExit("malformed estimate %s: %s" % (est_path, exc))
    truth = H.read_angles(os.path.join(ROOT, entry["joints"]), entry["frames"])
    r = H.score_recording(pred, truth, H.Kinematics())
    print("recording %s: %d frames (%d with finite angles)" % (rec_id, r["frames"], r["frames_finite"]))
    print("score (fraction of the %d body points within %.0f cm of their true place, over all frames): %.4f" % (
        len(H.KEYPOINTS), H.TOL_M * 100, r["score"]))
    if r["mean_point_error_m"] is not None:
        print("mean distance of a point from its true place: %.1f cm" % (r["mean_point_error_m"] * 100))
    print("fraction correct per point:")
    for k, v in r["fraction_correct_per_point"].items():
        print("  %-16s %.3f" % (k, v))


def cmd_check(folder):
    rec = load_recordings()
    good = 0
    for entry in rec["test"]:
        path = os.path.join(folder, entry["id"] + ".csv")
        if not os.path.isfile(path) or os.path.islink(path):
            print("%s: no regular file %s" % (entry["id"], path))
            continue
        try:
            pred = H.parse_estimate(read_bytes(path), entry["frames"])
        except H.Malformed as exc:
            print("%s: malformed: %s" % (entry["id"], exc))
            continue
        bad_rows = int((~H.np.isfinite(pred).all(axis=1)).sum())
        if bad_rows:
            print("%s: well formed, %d rows with a number that is not finite (all their points wrong)" % (entry["id"], bad_rows))
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
