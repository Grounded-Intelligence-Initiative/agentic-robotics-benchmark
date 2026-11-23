"""Hidden grader for cloth_material_id_from_video.

Reads /home/user/submission/params.json (numbers only) and scores fabrics A, B, C on the fixed hidden tests
(tests/ref/fabrics.json): every test is planned by ctasks.plan in a world made of the submitted material and then
executed once in the world made of the fabric's true material. Metric `material_transfer_score` (mean over fabrics
of the mean family score, higher is better). Separate verifier sandbox; agent processes are killed first and
nothing else the agent left is read.
"""
import os
import stat
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cgrading as CG  # noqa: E402
from robotics_grader import ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT = os.path.basename(HOME.rstrip("/")) or "user"
REF = os.path.join(HERE, "ref", "fabrics.json")


def stop_agent(max_rounds=20):
    for _ in range(max_rounds):
        r = subprocess.run(["pgrep", "-u", AGENT], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode != 0:
            return True
        subprocess.run(["runuser", "-u", AGENT, "--", "kill", "-KILL", "-1"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        time.sleep(0.2)
    return False


def read_submission():
    """Bytes of params.json, never following a symlink; None if absent or not a regular file."""
    sub = os.path.join(HOME, "submission")
    try:
        dfd = os.open(sub, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError:
        return None, "no submission directory"
    try:
        try:
            fd = os.open("params.json", os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=dfd)
        except OSError:
            return None, "no regular params.json"
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                return None, "params.json is not a regular file"
            chunks, total = [], 0
            while total <= CG.MAX_BYTES:
                b = os.read(fd, 65536)
                if not b:
                    break
                chunks.append(b)
                total += len(b)
            return b"".join(chunks), None
        finally:
            os.close(fd)
    finally:
        os.close(dfd)


def _score_one(args):
    key, i, material, deadline = args
    import time as _t
    ref_f = CG.load_ref(REF)[key]
    if _t.time() > deadline:
        return key, i, 0.0, True
    q, _ = CG.score_test(material, ref_f, ref_f["tests"][i])
    return key, i, q, False


def main():
    t0 = time.time()
    config = stage_config()
    anchor, meta = read_anchor()
    anchored = anchor is not None
    budget = float(config.get("budget_s", 3000))
    cleanup_ok = stop_agent()   # best effort in a fresh verifier sandbox; recorded, never a reason to score 0
    data, why = read_submission()
    if data is None:
        write_verdict(zero_verdict(reason=why, anchored=anchored))
        return
    try:
        entries, invalid = CG.parse_submission_bytes(data)
    except CG.InvalidSubmission as exc:
        write_verdict(zero_verdict(reason="invalid params.json: %s" % exc, anchored=anchored))
        return
    ref = CG.load_ref(REF)
    deadline = t0 + budget
    jobs = [(k, i, entries[k], deadline) for k in CG.FABRICS if k in entries for i in range(len(ref[k]["tests"]))]
    jobs.sort(key=lambda j: {"drape": 0, "slide": 1, "hang": 2}[ref[j[0]]["tests"][j[1]]["fam"]])  # longest first
    qs = {k: [0.0] * len(ref[k]["tests"]) for k in CG.FABRICS}
    unscored = {k: 0 for k in CG.FABRICS}
    if jobs:
        with ProcessPoolExecutor(max_workers=int(config.get("procs", 4))) as ex:
            for k, i, q, late in ex.map(_score_one, jobs):
                qs[k][i] = q
                unscored[k] += int(late)
    per = {}
    for k in CG.FABRICS:
        if k in entries:
            per[k] = CG.aggregate(qs[k], ref[k]["tests"])
        else:
            per[k] = (0.0, {})
    measured = float(sum(per[k][0] for k in CG.FABRICS) / len(CG.FABRICS))
    extra = {"grading_seconds": round(time.time() - t0, 1), "agent_cleanup_incomplete": 0.0 if cleanup_ok else 1.0}
    lines = []
    for k in CG.FABRICS:
        s, fm = per[k]
        extra["score_" + k] = round(s, 4)
        for f, v in fm.items():
            extra["q_%s_%s" % (k, f)] = round(v, 4)
        if unscored[k]:
            extra["unscored_" + k] = float(unscored[k])
        if k in invalid:            # metrics are numbers only: the flag here, the reason in the diagnostic
            extra["invalid_" + k] = 1.0
            lines.append("fabric %s scores 0: %s" % (k, str(invalid[k])[:200]))
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name="material_transfer_score", extra=extra, full_at=meta.get("full_at", 1.0))
    lines.append("material_transfer_score=%.4f anchor=%s reward=%.3f %.0fs" % (
        measured, anchor, verdict["rewards"]["reward"], time.time() - t0))
    write_verdict(verdict, diagnostic="\n".join(lines))
    print("[grader] " + lines[-1], file=sys.stderr)


if __name__ == "__main__":
    main()
