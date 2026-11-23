"""Practice check: run your calibration program on the practice rigs and print raw metrics.

    python3 ~/practice_grader/grade.py                 # run /home/user/submission/run.sh on every practice rig
    python3 ~/practice_grader/grade.py --rigs rig_b    # one rig
    python3 ~/practice_grader/grade.py --no-run --outputs /tmp/practice_out   # measure existing outputs

Runs are started the way grading starts them: `bash run.sh --input <rig> --output <out>` with the
working directory /home/user/submission, all selected rigs at once, each limited to 900 s of wall
clock and 6144 MB of address space. It then measures each rig's calibration.yaml and
fault_report.json against the practice rig's ground truth and its held-out sequence, with the same
measurement code grading uses (rigmetrics.py beside this file), and prints the raw metrics.
"""

import argparse
import os
import resource
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rigmetrics as M  # noqa: E402

PRACTICE = "/opt/rigcal/practice"
TRUTH = "/opt/rigcal/practice_truth"
SUBMISSION = "/home/user/submission"


def _limit(mb):
    def f():
        lim = mb * 1024 * 1024
        resource.setrlimit(resource.RLIMIT_AS, (lim, lim))
    return f


def run_all(rigs, out_root, timeout, mem_mb):
    procs = {}
    for r in rigs:
        od = os.path.join(out_root, r)
        os.makedirs(od, exist_ok=True)
        lf = open(os.path.join(out_root, r + ".log"), "wb")
        cmd = ["bash", "run.sh", "--input", os.path.join(PRACTICE, r), "--output", od]
        procs[r] = (subprocess.Popen(cmd, cwd=SUBMISSION, stdout=lf, stderr=subprocess.STDOUT,
                                     preexec_fn=_limit(mem_mb), start_new_session=True), lf, time.time())
    status = {}
    while len(status) < len(procs):
        for r, (p, lf, t0) in procs.items():
            if r in status:
                continue
            rc = p.poll()
            if rc is not None:
                status[r] = "exit %d after %.0f s" % (rc, time.time() - t0)
                lf.close()
            elif time.time() - t0 > timeout:
                os.killpg(p.pid, 9)
                p.wait()
                status[r] = "KILLED at the %.0f s limit" % timeout
                lf.close()
        time.sleep(0.5)
    return status


def measure(rig, out_dir):
    truth, ho = M.load_truth_files(os.path.join(PRACTICE, rig), os.path.join(TRUTH, rig))
    res = {}
    try:
        with open(os.path.join(out_dir, "calibration.yaml")) as fh:
            cal = M.parse_calibration(fh.read())
        res.update(M.calibration_metrics(truth, ho, cal))
    except (OSError, M.CalibrationError) as exc:
        res["calibration_problem"] = str(exc)
    try:
        with open(os.path.join(out_dir, "fault_report.json")) as fh:
            rep = M.parse_fault_report(fh.read())
        res["fault_f1"] = M.fault_f1(truth["faults"], rep)
    except (OSError, M.CalibrationError) as exc:
        res["fault_report_problem"] = str(exc)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rigs", default=",".join(sorted(os.listdir(PRACTICE))))
    ap.add_argument("--outputs", default="/tmp/practice_out")
    ap.add_argument("--no-run", action="store_true")
    ap.add_argument("--timeout", type=float, default=900.0)
    ap.add_argument("--memory-mb", type=int, default=6144)
    a = ap.parse_args()
    rigs = [r for r in a.rigs.split(",") if r]
    known = sorted(os.listdir(PRACTICE))
    unknown = [r for r in rigs if r not in known]
    if unknown:
        sys.exit("unknown practice rig(s) %s; the practice rigs are %s" % (", ".join(unknown), ", ".join(known)))
    if not a.no_run:
        status = run_all(rigs, a.outputs, a.timeout, a.memory_mb)
        for r in rigs:
            print("%s: %s (log: %s)" % (r, status[r], os.path.join(a.outputs, r + ".log")))
    units = {"projection_px": "px", "time_rms_s": "s", "time_rms_camera_s": "s", "time_rms_lidar_s": "s",
             "rpe_rot_deg": "deg", "rpe_trans_m": "m", "imu_rot_deg": "deg", "gyro_bias_err_rps": "rad/s",
             "accel_bias_err_mps2": "m/s^2", "fault_f1": ""}
    for r in rigs:
        res = measure(r, os.path.join(a.outputs, r))
        print("== %s" % r)
        for k, v in res.items():
            if isinstance(v, float):
                print("   %-22s %.6g %s" % (k, v, units.get(k, "")))
            else:
                print("   %-22s %s" % (k, v))


if __name__ == "__main__":
    main()
