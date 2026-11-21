"""Hidden grader for humanoid_joint_angles_from_video.

Reads /home/user/submission/T01.csv ... T24.csv (numbers only, never through a link; nothing the agent wrote is
executed) and scores each estimate against the true joint angles of its recording (tests/ref) with
humanoidscore.py, the module the agent has a byte-identical copy of (with g1_keypoints.xml). Metric
`keypoint_accuracy_5cm`: the mean over the 24 test recordings of the fraction of the 21 body points, over all
frames, that forward kinematics from the estimate places within 5.0 cm of where the true angles place them
(higher is better). A missing or malformed file scores 0 for its recording. Separate verifier sandbox.
"""
import json
import os
import stat
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import humanoidscore as H  # noqa: E402
from robotics_grader import ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
REF = os.path.join(HERE, "ref")
METRIC = "keypoint_accuracy_5cm"


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
        while total <= H.MAX_BYTES:
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
    with open(os.path.join(REF, "recordings.json")) as f:
        test = json.load(f)["test"]
    try:
        dfd = os.open(os.path.join(HOME, "submission"), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError:
        write_verdict(zero_verdict(reason="no submission directory", anchored=anchored))
        return
    kin = H.Kinematics()
    extra, notes = {}, []
    missing = malformed = 0
    total = 0.0
    try:
        for entry in test:
            rid = entry["id"]
            score = 0.0
            data = read_regular(dfd, rid + ".csv")
            if data is None:
                missing += 1
            else:
                try:
                    pred = H.parse_estimate(data, entry["frames"])
                except H.Malformed as exc:
                    malformed += 1
                    notes.append("%s malformed: %s" % (rid, str(exc)[:120]))
                else:
                    truth = H.read_angles(os.path.join(REF, rid + ".csv"), entry["frames"])
                    score = H.score_recording(pred, truth, kin)["score"]
            extra["score_" + rid] = round(score, 4)
            total += score
    finally:
        os.close(dfd)
    if missing == len(test):
        write_verdict(zero_verdict(reason="no estimate file in the submission directory", anchored=anchored))
        return
    measured = total / len(test)
    extra["files_missing"] = float(missing)
    extra["files_malformed"] = float(malformed)
    extra["grading_seconds"] = round(time.time() - t0, 1)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=METRIC, extra=extra, full_at=meta.get("full_at", 1.0))
    text = "%s=%.4f over %d recordings (%d missing, %d malformed)" % (METRIC, measured, len(test), missing, malformed)
    if notes:
        text += "; " + "; ".join(notes[:8])
    write_verdict(verdict, diagnostic=text)
    print("[grader] %s anchor=%s reward=%.3f %.1fs" % (text, anchor, verdict["rewards"]["reward"], time.time() - t0),
          file=sys.stderr)


if __name__ == "__main__":
    main()
