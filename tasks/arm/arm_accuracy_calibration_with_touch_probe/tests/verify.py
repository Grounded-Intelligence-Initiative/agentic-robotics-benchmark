"""Hidden grader for arm_accuracy_calibration_with_touch_probe.

Reads one submission per graded unit, /home/user/submission/<unit id>.json (numbers only: the
predicted tool-tip position and the predicted positions of the camera-frame points for each of
the unit's published acceptance conditions, and three job programs, lists of joint targets;
submission_format.py), and compares it with the true unit in the grader's own copy of the cell
(calib_cell.py, cell_camera.py, arm_model.py, cell_data.py, hidden/units.json). A unit is accepted
when the 95th percentile and the largest error of both tests are inside the limits of cell_data.py
and its three job programs, each run once in the grader's simulator (calib_cell.run_job: process_c
mounted, the unit's gauge plate in its cell; the cold program on the true unit as measured from
every link at T_AMBIENT, the warm one on it from the unit's published warm start temperatures, the
serviced one cold on the unit after its service, calib_cell.serviced; each with its own repeatability
stream), have done every bore before any contact. The predictions and the cold and warm jobs are
graded on the unit as first measured, before its service.

Metric `accepted_unit_fraction` (higher is better): accepted units / units.

Nothing of the submission is executed: the job programs are data, joint targets that the simulator
moves the true unit to. Each file is opened without following a link, must be a regular file of at
most submission_format.MAX_BYTES, and must parse as the strict submission of submission_format.py
(a file without the warm or the serviced program does not); a unit whose file fails any of that, or provokes any
error, is not accepted and the others are graded as usual. Separate verifier sandbox: no agent code
has run here.
"""
import json
import os
import stat
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import arm_model as am  # noqa: E402
import calib_cell as cc  # noqa: E402
import submission_format as sf  # noqa: E402
from robotics_grader import ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
METRIC = "accepted_unit_fraction"
K_POINTS = len(cc.ACCEPT_CAM_POINTS)


def read_file(dfd, name):
    """Bytes of a regular file in the submission folder, never following a link; (None, why) otherwise."""
    try:
        fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=dfd)
    except OSError:
        return None, "no regular file"
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return None, "not a regular file"
        chunks, total = [], 0
        while total <= sf.MAX_BYTES:
            block = os.read(fd, 65536)
            if not block:
                break
            chunks.append(block)
            total += len(block)
        if total > sf.MAX_BYTES:
            return None, "larger than {} bytes".format(sf.MAX_BYTES)
        return b"".join(chunks), None
    finally:
        os.close(fd)


JOBS = (("job", False, False), ("job_warm", True, False), ("job_serviced", False, True))   # key, warm, serviced


def run_jobs(unit, programs):
    """The three programs run once (each its own repeatability stream 0): the cold and the warm one on the true
    unit (the warm one from the published warm start), the serviced one on the unit after its service. Returns
    {key: result} with every result's bores_done, jammed, jam_row, jam_object, rows_run (and poses)."""
    out = {}
    after = cc.serviced(unit)
    for (key, warm, service), rows in zip(JOBS, programs):
        try:
            out[key] = cc.run_job(after if service else unit, np.array(rows, dtype=float), stream=0, warm=warm)
        except cc.JobTooComplex:
            out[key] = dict(bores_done=0, jammed=True, jam_row=-1, jam_object="(the check's work limit)", rows_run=0)
    return out


def grade(units, folder):
    """Per unit: (accepted, [tip p95, tip max, camera p95, camera max] in mm or None, {"job": result,
    "job_warm": result, "job_serviced": result} or None, reason)."""
    out = {}
    try:
        dfd = os.open(folder, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError:
        return None
    try:
        for uid, seed in units.items():
            data, why = read_file(dfd, "{}.json".format(uid))
            if data is None:
                out[uid] = (False, None, None, why)
                continue
            try:
                tip, cam, *programs = sf.parse(data, cc.N_ACCEPT, K_POINTS, (am.Q_MIN, am.Q_MAX))
                unit = cc.draw_unit(seed)
                tip_err, cam_err = cc.acceptance_errors_from_predictions(np.array(tip), np.array(cam), unit)
                if not (np.all(np.isfinite(tip_err)) and np.all(np.isfinite(cam_err))):
                    raise ValueError("the errors are not finite")
            except ValueError as exc:
                out[uid] = (False, None, None, str(exc)[:120])
                continue
            except Exception as exc:  # noqa: BLE001 - whatever a submission provokes costs that unit only
                out[uid] = (False, None, None, "could not be graded ({})".format(type(exc).__name__))
                continue
            figures = [float(np.percentile(tip_err, 95) * 1e3), float(tip_err.max() * 1e3),
                       float(np.percentile(cam_err, 95) * 1e3), float(cam_err.max() * 1e3)]
            try:
                res = run_jobs(unit, programs)
            except Exception as exc:  # noqa: BLE001
                out[uid] = (False, figures, None, "job could not be run ({})".format(type(exc).__name__))
                continue
            ok = cc.accepted(tip_err, cam_err) and all(cc.job_passed(r) for r in res.values())
            out[uid] = (ok, figures, res, "")
    finally:
        os.close(dfd)
    return out


def job_text(name, job):
    return "{} {} of {} bores done{}".format(
        name, job["bores_done"], cc.N_BORES, ", jammed at row {} on {}".format(job["jam_row"], job["jam_object"])
        if job["jammed"] else ", {} rows run".format(job["rows_run"]))


def main():
    t0 = time.time()
    config = stage_config()
    anchor, meta = read_anchor()
    anchored = anchor is not None
    with open(os.path.join(HERE, "hidden", "units.json")) as handle:
        units = json.load(handle)["graded"]
    result = grade(units, os.path.join(HOME, "submission"))
    if result is None:
        write_verdict(zero_verdict(reason="no submission folder", anchored=anchored))
        return
    measured = sum(1 for r in result.values() if r[0]) / float(len(units))
    extra = {"units": float(len(units)), "units_with_a_valid_submission": float(sum(1 for r in result.values() if r[1] is not None)),
             "grading_seconds": round(time.time() - t0, 2)}
    lines = []
    for uid in sorted(result):
        ok, figures, jobs, why = result[uid]
        extra["accepted_" + uid] = 1.0 if ok else 0.0
        for name, value in zip(("tip_p95_mm_", "tip_max_mm_", "camera_p95_mm_", "camera_max_mm_"), figures or [-1.0] * 4):
            extra[name + uid] = round(value, 4)
        for key, _, _ in JOBS:
            job = jobs[key] if jobs else None
            extra[key + "_bores_done_" + uid] = float(job["bores_done"]) if job else -1.0
            extra[key + "_jammed_" + uid] = (1.0 if job["jammed"] else 0.0) if job else -1.0
        if figures is None:
            lines.append("{}: no valid submission ({})".format(uid, why))
            continue
        text = ("{} (tip p95 {:.3f} max {:.3f} mm, limits {:.2f} / {:.2f}; camera p95 {:.3f} max {:.3f} mm, limits "
                "{:.2f} / {:.2f}; ").format("accepted" if ok else "not accepted", figures[0], figures[1],
                                           cc.ACCEPT_P95 * 1e3, cc.ACCEPT_MAX * 1e3, figures[2], figures[3],
                                           cc.ACCEPT_CAM_P95 * 1e3, cc.ACCEPT_CAM_MAX * 1e3)
        if jobs is None:
            text += "jobs not run: {})".format(why)
        else:
            text += "{}; {}; {})".format(job_text("cold job:", jobs["job"]), job_text("warm job:", jobs["job_warm"]),
                                         job_text("serviced job:", jobs["job_serviced"]))
        lines.append("{}: {}".format(uid, text))
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=METRIC, extra=extra, full_at=meta.get("full_at", 1.0))
    write_verdict(verdict, diagnostic="accepted units {:.4f}\n{}".format(measured, "\n".join(lines)))
    print("[grader] {}={:.4f} anchor={} reward={:.3f} {:.1f}s".format(
        METRIC, measured, anchor, verdict["rewards"]["reward"], time.time() - t0), file=sys.stderr)


if __name__ == "__main__":
    main()
