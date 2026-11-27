"""job_selftest.py: the grader against hostile submissions, and its time in the worst case.

    python job_selftest.py

Maintainers' tooling (nothing runs it during grading). Writes submissions into a temporary folder and grades
them with verify.grade on the graded units of hidden/units.json, as the grader does:
  * files that must not parse: too large, deeply nested, a huge integer, NaN / Infinity, wrong shapes, a key
    twice, extra or missing keys (no job, no job_warm, no job_serviced), booleans, strings, null, a byte-order mark, bytes that
    are not UTF-8, a job (cold or warm) of 0 or 201 rows, a row of 5 or 7 numbers, a target outside the joint
    limits, a job that is not a list, a symbolic link (to a valid file), a directory, a FIFO;
  * job programs that parse and must fail without breaking the grader (in the cold job, and the same in the
    warm one): 200 zero-length rows at home, 200 rows swinging between the joint limits, 200 random targets, the
    pin driven through the plate in one long row; the cold program submitted as the warm one, and the warm one
    as the cold one (each must miss: one program does not serve both states); the same for the serviced job (its
    cases on the unit after its service), and the cold program submitted as the serviced one;
  * the sanity case: the truth's predictions with three programs planned on the truth, the cold one and the
    warm one from the unit's warm start on the unit as measured, the serviced one on the unit after its service
    (must be accepted);
  * routes that must do no bore: the reviewer's program that slides a horizontal pin in under the plate, its
    apex below bores 2 to 7 (job_under_plate_u1.json); a horizontal pin driven at bore depth from outside the
    plate's side towards the nearest bore (it must jam on the side face);
  * the pose cap: 200 long wrist swings far from everything (each row's first samples count: the check stops
    at JOB_MAX_POSES, a work-limit failure), in each job and in all three;
  * the worst case for the contact check's time: programs planned on the truth that slide the pin across the
    plate's top face 0.3 mm above it for 200 rows, as the cold, the warm (planned from the warm start) and the
    serviced job (planned on the serviced unit), graded on all six units; and the costliest pose there is (a pin
    lying parallel to the top face, every interval of it refined), times the cap of each of the three jobs; the
    peak memory.
Every hostile case must come back not accepted with a reason, never an exception.
"""
import json
import os
import resource
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(_HERE), os.path.join(_HERE, "..", "..", "solution", "calibrate")]

import numpy as np  # noqa: E402

import arm_model as am  # noqa: E402
import calib_cell as cc  # noqa: E402
import plate_job as pj  # noqa: E402
import verify  # noqa: E402

UNITS = json.load(open(os.path.join(os.path.dirname(_HERE), "hidden", "units.json")))["graded"]


def truth_kin(unit):
    tool = cc.TOOLS[cc.JOB_TOOL]

    def fn(q, s, temp):
        R, o = cc.true_frames(unit, q, s, temp, tool)
        return o + np.einsum("nab,b->na", R, pj.TIP_C), R
    return pj.Kin(fn)


def good_record(unit, job, job_warm, job_serviced):
    tip, cam = cc.truth_on_conditions(unit)
    return {"tip": tip.tolist(), "camera": cam.tolist(), "job": [list(map(float, r)) for r in job],
            "job_warm": [list(map(float, r)) for r in job_warm],
            "job_serviced": [list(map(float, r)) for r in job_serviced]}


def truth_planner(unit, warm=False, serviced=False):
    """The reference's planner on the truth: the unit's kinematics (after its service, with `serviced`), heating and
    cooling, and (warm) warm start."""
    return pj.Planner(truth_kin(cc.serviced(unit) if serviced else unit), np.array(unit["plate"]["pose"]),
                      heat=unit["cell"]["heat"], tau=unit["cell"]["tau_cool"],
                      temp0=cc.warm_start(unit["seed"]) if warm else None)


def grazing_program(unit, height=3e-4, rows=200, warm=False, serviced=False):
    """Rows that slide the pin's apex along the plate's long axis, `height` above its top face (between the
    bores' rows: y = 0.017 in the plate frame), back and forth, planned on the truth."""
    pl = unit["plate"]
    planner = truth_planner(unit, warm, serviced)
    R, O = pl["R"], pl["O"]
    a = R[:, 2]
    A = O + R @ np.array([-0.07, 0.017, 0.0]) + height * a
    Bp = O + R @ np.array([0.07, 0.017, 0.0]) + height * a
    best = planner.choose_spin()
    if best is None or not planner.transit(A + 0.02 * a, best[1]):
        return None
    planner.go_line(planner._apex(), A, 0.0, step=0.004)
    while len(planner.rows) < rows:
        ends = (Bp, A) if np.linalg.norm(planner._apex() - A) < 1e-3 else (A, Bp)
        if not planner.go_line(planner._apex(), ends[0], 0.0, step=0.02, tol=1e-5):
            break
    return np.array(planner.rows[:rows])


def side_program(unit, depth=0.010, n=12):
    """A horizontal pin at `depth` below the top face driven from outside the plate's side, along its length or
    its width, towards the nearest bore (planned on the true unit, rows by least squares); the first such
    route the arm can reach, or None."""
    from scipy.optimize import least_squares
    pl = unit["plate"]
    R, O = pl["R"], pl["O"]
    s1, t20, z3 = np.ones(6), np.full(6, 20.0), np.zeros(3)
    rng = np.random.default_rng(3)

    def solve(target, a, q0):
        def res(q):
            E, B, apex = cc.pin_axis(unit, q[None], s1, t20, z3)
            ax = (E[0] - B[0]) / np.linalg.norm(E[0] - B[0])
            return np.concatenate([(apex[0] - target) * 10, ax - a])
        r = least_squares(res, q0, bounds=(am.Q_MIN, am.Q_MAX), xtol=1e-12, ftol=1e-12, gtol=1e-12)
        return r.x, r.cost
    half = np.array(cc.PLATE_SIZE[:2]) / 2
    for axis, sign in ((0, -1), (0, 1), (1, -1), (1, 1)):
        bore = int(np.argmin(sign * cc.BORES[:, axis])) if sign < 0 else int(np.argmax(cc.BORES[:, axis]))
        b = cc.BORES[bore].copy()
        a = sign * -R[:, axis]                     # pointing into the plate
        start_l = b.copy()
        start_l[axis] = sign * (half[axis] + 0.04)
        start = O + R @ np.array([start_l[0], start_l[1], -depth])
        best = None
        for _ in range(40):
            q, c = solve(start, a, rng.uniform(am.Q_MIN * 0.95, am.Q_MAX * 0.95))
            if c < 1e-14:
                best = q
                break
        if best is None:
            continue
        rows, q = [best], best
        for k in range(1, n + 1):
            p = start_l + (b - start_l) * k / n
            q, c = solve(O + R @ np.array([p[0], p[1], -depth]), a, q)
            rows.append(q)
        return np.array(rows), bore
    return None


def job_line(key, job):
    return "{}: {} bores{}".format(key, job["bores_done"], ", jammed at row {} on {}".format(job["jam_row"], job["jam_object"])
                                   if job["jammed"] else "")


def write(folder, name, data):
    path = os.path.join(folder, name)
    if os.path.lexists(path):
        os.remove(path)
    with open(path, "wb") as handle:
        handle.write(data if isinstance(data, bytes) else data.encode())
    return path


def main():
    uid, seed = sorted(UNITS.items())[0]
    unit = cc.draw_unit(seed)
    good_job = truth_planner(unit).plan()
    good_warm = truth_planner(unit, warm=True).plan()
    good_serv = truth_planner(unit, serviced=True).plan()
    good = good_record(unit, good_job, good_warm, good_serv)
    base = json.dumps(good)
    n, k = cc.N_ACCEPT, len(cc.ACCEPT_CAM_POINTS)
    rng = np.random.default_rng(1)
    lim = lambda v: np.clip(v, am.Q_MIN, am.Q_MAX)    # noqa: E731
    swings = [list(np.r_[cc.Q_HOME[:3], am.Q_MIN[3] if i % 2 == 0 else am.Q_MAX[3], cc.Q_HOME[4],
                         am.Q_MAX[5] if i % 2 == 0 else am.Q_MIN[5]]) for i in range(200)]
    cases = {
        "sanity: truth and three programs planned on the truth": base,
        "too large": base + " " * (1 << 20),
        "nested 100000 deep": "[" * 100000 + "]" * 100000,
        "huge integer": json.dumps(dict(good, job=[[0.0] * 6])).replace('"job": [[0.0', '"job": [[' + "1" * 400, 1),
        "huge integer in job_warm": json.dumps(dict(good, job_warm=[[0.0] * 6])).replace(
            '"job_warm": [[0.0', '"job_warm": [[' + "1" * 400, 1),
        "NaN": base.replace('"job": [[', '"job": [[NaN, ', 1),
        "NaN in job_warm": base.replace('"job_warm": [[', '"job_warm": [[NaN, ', 1),
        "Infinity": base.replace('"tip": [[', '"tip": [[Infinity, ', 1),
        "tip 399 rows": json.dumps(dict(good, tip=good["tip"][:-1])),
        "key twice": base[:-1] + ', "job": [[0, 0, 0, 0, 0, 0]]}',
        "job_warm twice": base[:-1] + ', "job_warm": [[0, 0, 0, 0, 0, 0]]}',
        "extra key": base[:-1] + ', "x": 1}',
        "no job key": json.dumps({k_: v for k_, v in good.items() if k_ != "job"}),
        "no job_warm key (the cold-only format)": json.dumps({k_: v for k_, v in good.items() if k_ != "job_warm"}),
        "no job keys": json.dumps({"tip": good["tip"], "camera": good["camera"]}),
        "boolean in job": json.dumps(dict(good, job=[[True, 0, 0, 0, 0, 0]])),
        "string in job": json.dumps(dict(good, job=[["0", 0, 0, 0, 0, 0]])),
        "null in job": json.dumps(dict(good, job=[[None, 0, 0, 0, 0, 0]])),
        "null as job_warm": json.dumps(dict(good, job_warm=None)),
        "boolean in job_warm": json.dumps(dict(good, job_warm=[[0, 0, 0, 0, 0, False]])),
        "byte-order mark": "\ufeff" + base,
        "not UTF-8": b"\xff\xfe" + base.encode(),
        "job of 0 rows": json.dumps(dict(good, job=[])),
        "job_warm of 0 rows": json.dumps(dict(good, job_warm=[])),
        "job of 201 rows": json.dumps(dict(good, job=[list(cc.Q_HOME)] * 201)),
        "job_warm of 201 rows": json.dumps(dict(good, job_warm=[list(cc.Q_HOME)] * 201)),
        "job_warm of 100000 rows": json.dumps(dict(good, job_warm=[[0] * 6] * 100000)),
        "row of 5 numbers": json.dumps(dict(good, job=[[0, 0, 0, 0, 0]])),
        "row of 7 numbers": json.dumps(dict(good, job=[[0, 0, 0, 0, 0, 0, 0]])),
        "job_warm row of 7 numbers": json.dumps(dict(good, job_warm=[[0, 0, 0, 0, 0, 0, 0]])),
        "target outside the limits": json.dumps(dict(good, job=[list(cc.Q_HOME[:5]) + [3.2]])),
        "job_warm target outside the limits": json.dumps(dict(good, job_warm=[[3.0] + list(cc.Q_HOME[1:])])),
        "job not a list": json.dumps(dict(good, job={"0": [0] * 6})),
        "job_warm a number": json.dumps(dict(good, job_warm=1.0)),
        "job_warm nested one deeper": json.dumps(dict(good, job_warm=[good["job_warm"]])),
        "200 zero-length rows": json.dumps(dict(good, job=[list(cc.Q_HOME)] * 200)),
        "200 zero-length rows (warm)": json.dumps(dict(good, job_warm=[list(cc.Q_HOME)] * 200)),
        "200 rows between the limits": json.dumps(dict(good, job=[list(am.Q_MIN if i % 2 else am.Q_MAX) for i in range(200)])),
        "200 rows between the limits (warm)": json.dumps(dict(good, job_warm=[list(am.Q_MIN if i % 2 else am.Q_MAX)
                                                                             for i in range(200)])),
        "200 random targets": json.dumps(dict(good, job=lim(rng.uniform(am.Q_MIN, am.Q_MAX, (200, 6))).tolist())),
        "200 random targets (warm)": json.dumps(dict(good, job_warm=lim(rng.uniform(am.Q_MIN, am.Q_MAX, (200, 6))).tolist())),
        "through the plate in one row": json.dumps(dict(good, job=[list(good_job[2]), list(good_job[2] + np.array([0, 0.3, -0.3, 0, 0, 0]))])),
        "through the plate in one row (warm)": json.dumps(dict(good, job_warm=[list(good_warm[2]), list(
            good_warm[2] + np.array([0, 0.3, -0.3, 0, 0, 0]))])),
        "the cold program as the warm one": json.dumps(dict(good, job_warm=good["job"])),
        "the warm program as the cold one": json.dumps(dict(good, job=good["job_warm"])),
        "under the plate (review route, bores 2-7)": json.dumps(dict(good, job=json.load(open(os.path.join(_HERE, "job_under_plate_u1.json")))["rows"])),
        "200 long wrist swings (pose cap)": json.dumps(dict(good, job=swings)),
        "200 long wrist swings (pose cap, warm)": json.dumps(dict(good, job_warm=swings)),
        "200 long wrist swings (pose cap, both)": json.dumps(dict(good, job=swings, job_warm=swings)),
        "huge integer in job_serviced": json.dumps(dict(good, job_serviced=[[0.0] * 6])).replace(
            '"job_serviced": [[0.0', '"job_serviced": [[' + "1" * 400, 1),
        "NaN in job_serviced": base.replace('"job_serviced": [[', '"job_serviced": [[NaN, ', 1),
        "job_serviced twice": base[:-1] + ', "job_serviced": [[0, 0, 0, 0, 0, 0]]}',
        "no job_serviced key (the two-job format)": json.dumps({k_: v for k_, v in good.items() if k_ != "job_serviced"}),
        "null as job_serviced": json.dumps(dict(good, job_serviced=None)),
        "string in job_serviced": json.dumps(dict(good, job_serviced=[["0", 0, 0, 0, 0, 0]])),
        "job_serviced of 0 rows": json.dumps(dict(good, job_serviced=[])),
        "job_serviced of 201 rows": json.dumps(dict(good, job_serviced=[list(cc.Q_HOME)] * 201)),
        "job_serviced row of 5 numbers": json.dumps(dict(good, job_serviced=[[0, 0, 0, 0, 0]])),
        "job_serviced target outside the limits": json.dumps(dict(good, job_serviced=[list(cc.Q_HOME[:2]) + [3.0] + list(cc.Q_HOME[3:])])),
        "job_serviced an object": json.dumps(dict(good, job_serviced={"rows": good["job_serviced"]})),
        "200 zero-length rows (serviced)": json.dumps(dict(good, job_serviced=[list(cc.Q_HOME)] * 200)),
        "200 random targets (serviced)": json.dumps(dict(good, job_serviced=lim(rng.uniform(am.Q_MIN, am.Q_MAX, (200, 6))).tolist())),
        "through the plate in one row (serviced)": json.dumps(dict(good, job_serviced=[list(good_serv[2]), list(
            good_serv[2] + np.array([0, 0.3, -0.3, 0, 0, 0]))])),
        "the cold program as the serviced one": json.dumps(dict(good, job_serviced=good["job"])),
        "the serviced program as the cold one": json.dumps(dict(good, job=good["job_serviced"])),
        "200 long wrist swings (pose cap, serviced)": json.dumps(dict(good, job_serviced=swings)),
        "200 long wrist swings (pose cap, all three)": json.dumps(dict(good, job=swings, job_warm=swings, job_serviced=swings)),
    }
    side = side_program(unit)
    if side is not None:
        cases["from the side at bore depth towards bore {}".format(side[1])] = json.dumps(dict(good, job=side[0].tolist()))
        cases["from the side at bore depth towards bore {} (warm)".format(side[1])] = json.dumps(
            dict(good, job_warm=side[0].tolist()))
        cases["from the side at bore depth towards bore {} (serviced)".format(side[1])] = json.dumps(
            dict(good, job_serviced=side[0].tolist()))
    else:
        print("(no side route reachable on this unit)")
    folder = tempfile.mkdtemp()
    os.chmod(folder, 0o700)
    print("{:<52} {:>9} {:>7}  {}".format("case", "accepted", "time s", "reason / job"))
    bad = 0
    for name, data in cases.items():
        write(folder, uid + ".json", data)
        t0 = time.time()
        try:
            res = verify.grade({uid: seed}, folder)[uid]
        except Exception as exc:  # noqa: BLE001
            print("{:<52} EXCEPTION {}".format(name, exc))
            bad += 1
            continue
        dt = time.time() - t0
        ok, figures, jobs, why = res
        detail = why or " | ".join(job_line(key, jobs[key]) for key in ("job", "job_warm", "job_serviced"))
        print("{:<52} {:>9} {:>7.2f}  {}".format(name, str(ok), dt, detail[:120]))
        if name.startswith("sanity") != bool(ok):
            bad += 1
    # links, a directory, a FIFO
    target = write(folder, "valid.json", base)
    for name, make in (("symbolic link to a valid file", lambda p: os.symlink(target, p)),
                       ("a directory", lambda p: os.mkdir(p)), ("a FIFO", lambda p: os.mkfifo(p))):
        path = os.path.join(folder, uid + ".json")
        if os.path.isdir(path) and not os.path.islink(path):
            os.rmdir(path)
        elif os.path.lexists(path):
            os.remove(path)
        make(path)
        t0 = time.time()
        ok, figures, job, why = verify.grade({uid: seed}, folder)[uid]
        print("{:<52} {:>9} {:>7.2f}  {}".format(name, str(ok), time.time() - t0, why))
        bad += bool(ok)
        if os.path.isdir(path) and not os.path.islink(path):
            os.rmdir(path)
        else:
            os.remove(path)
    # the worst case for the check's time, on all six units: both jobs graze the top face
    print("worst case: the pin slid 0.3 mm above the top face, 200 rows planned on the truth, in the three jobs, all six "
          "units")
    t_all = time.time()
    for u_id, u_seed in sorted(UNITS.items()):
        un = cc.draw_unit(u_seed)
        rows = grazing_program(un)
        rows_w = grazing_program(un, warm=True)
        rows_s = grazing_program(un, serviced=True)
        if rows is None or rows_w is None or rows_s is None:
            print("  {}: no grazing program".format(u_id))
            continue
        write(folder, u_id + ".json", json.dumps(good_record(un, rows, rows_w, rows_s)))
    t0 = time.time()
    result = verify.grade(UNITS, folder)
    t_grade = time.time() - t0
    for u_id in sorted(result):
        ok, figures, jobs, why = result[u_id]
        print("  {}: accepted {}, {}".format(u_id, ok, why or " | ".join(
            job_line(key, jobs[key]) + ", {} poses examined".format(jobs[key].get("poses", "?")) for key in ("job", "job_warm", "job_serviced"))))
    print("  grading the six units took {:.1f} s (verifier limit 600 s); building the programs {:.1f} s".format(
        t_grade, t0 - t_all))
    # the bound: the grading time per pose examined (prediction grading included) times the cap of both jobs
    poses = sum(j.get("poses", 0) for r in result.values() if r[2] for j in r[2].values())
    if poses:
        per = t_grade / poses
        print("  {:.0f} us per pose examined (all of grading counted); three programs at the cap of {} poses each: {:.0f} "
              "s per unit, {:.0f} s for six".format(per * 1e6, cc.JOB_MAX_POSES, 3 * per * cc.JOB_MAX_POSES,
                                                    18 * per * cc.JOB_MAX_POSES))
    # the costliest pose: a pin lying parallel to the top face 0.3 mm above it (every interval of it refined)
    pl = unit["plate"]
    ang = np.linspace(0, 2 * np.pi, 3142)
    c = pl["O"] + (3e-4 + cc.PIN_RADIUS) * pl["R"][:, 2]
    d = np.cos(ang)[:, None] * pl["R"][:, 0] + np.sin(ang)[:, None] * pl["R"][:, 1]
    t0 = time.time()
    for a in range(0, len(ang), cc.POSE_BATCH):
        cc.pin_clearance(unit, c + 0.015 * d[a:a + cc.POSE_BATCH], c - 0.015 * d[a:a + cc.POSE_BATCH])
    per = (time.time() - t0) / len(ang)
    print("  costliest pose (a pin parallel to the top face): {:.0f} us; three programs at the cap of {} poses each "
          "{:.0f} s per unit, {:.0f} s for six (kinematics and bookkeeping not included)".format(
              per * 1e6, cc.JOB_MAX_POSES, 3 * per * cc.JOB_MAX_POSES, 18 * per * cc.JOB_MAX_POSES))
    print("  peak memory of this process: {:.0f} MB".format(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 2 ** 20
                                                         if sys.platform == "darwin" else
                                                         resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024))
    print("hostile cases misgraded: {}".format(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
