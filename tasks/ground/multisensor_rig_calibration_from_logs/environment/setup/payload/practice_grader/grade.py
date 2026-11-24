"""Practice check: run your calibration program on the practice rigs and print raw metrics.

    python3 ~/practice_grader/grade.py                 # run /home/user/submission/run.sh on every practice rig
    python3 ~/practice_grader/grade.py --rigs rig_b    # one rig
    python3 ~/practice_grader/grade.py --no-run --outputs /tmp/practice_out   # measure existing outputs

Runs are started the way grading starts them: `bash run.sh --input <rig> --output <out>` with the
working directory /home/user/submission, all selected rigs at once, each limited to 900 s of its own
processor time (user + system of all the run's processes, read from /proc as the grader reads it), a
wall-clock stop at 3600 s and 6144 MB of address space. It then measures each rig's calibration.yaml and
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


CLK_TCK = os.sysconf("SC_CLK_TCK")


def _session_cpu(seen):
    """Processor time (s) per session id: user + system of every process, the largest value seen per process."""
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % name, "rb") as f:
                rest = f.read().rsplit(b")", 1)[1].split()
            seen[(int(name), int(rest[19]))] = (int(rest[3]), max(int(rest[11]) + int(rest[12]), seen.get((int(name), int(rest[19])), (0, 0))[1]))
        except (OSError, ValueError, IndexError):
            continue
    out = {}
    for sid, ticks in seen.values():
        out[sid] = out.get(sid, 0.0) + ticks / float(CLK_TCK)
    return out


def run_all(rigs, out_root, cpu_limit, wall_limit, mem_mb):
    procs = {}
    for r in rigs:
        od = os.path.join(out_root, r)
        os.makedirs(od, exist_ok=True)
        lf = open(os.path.join(out_root, r + ".log"), "wb")
        cmd = ["bash", "run.sh", "--input", os.path.join(PRACTICE, r), "--output", od]
        procs[r] = (subprocess.Popen(cmd, cwd=SUBMISSION, stdout=lf, stderr=subprocess.STDOUT,
                                     preexec_fn=_limit(mem_mb), start_new_session=True), lf, time.time())
    status, seen = {}, {}
    while len(status) < len(procs):
        cpu = _session_cpu(seen)
        for r, (p, lf, t0) in procs.items():
            if r in status:
                continue
            rc = p.poll()
            used, wall = cpu.get(p.pid, 0.0), time.time() - t0
            if rc is not None:
                status[r] = "exit %d after %.0f s of processor time, %.0f s on the clock" % (rc, used, wall)
                lf.close()
            elif used > cpu_limit or wall > wall_limit:
                os.killpg(p.pid, 9)
                p.wait()
                status[r] = "KILLED at the %s (%.0f s of processor time, %.0f s on the clock)" % (
                    "processor-time limit" if used > cpu_limit else "wall-clock stop", used, wall)
                lf.close()
        time.sleep(0.2)
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
    ap.add_argument("--cpu-s", type=float, default=900.0)
    ap.add_argument("--wall-s", type=float, default=3600.0)
    ap.add_argument("--memory-mb", type=int, default=6144)
    a = ap.parse_args()
    rigs = [r for r in a.rigs.split(",") if r]
    known = sorted(os.listdir(PRACTICE))
    unknown = [r for r in rigs if r not in known]
    if unknown:
        sys.exit("unknown practice rig(s) %s; the practice rigs are %s" % (", ".join(unknown), ", ".join(known)))
    if not a.no_run:
        status = run_all(rigs, a.outputs, a.cpu_s, a.wall_s, a.memory_mb)
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
