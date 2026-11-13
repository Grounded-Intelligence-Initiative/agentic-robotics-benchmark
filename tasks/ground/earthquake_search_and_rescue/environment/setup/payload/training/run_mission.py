#!/opt/venv/bin/python3
"""Training mission: runs YOUR program over the real wire in one mission on a site of the training ground (the test
lanes or a training rubble site, never a graded site), the way the grader runs it, and prints the trainers' debrief.

    /opt/venv/bin/python3 ~/training/run_mission.py --site lanes                     # ~/submission on the test lanes
    /opt/venv/bin/python3 ~/training/run_mission.py --site 3 --keep ~/runs/m3 --transcript ~/runs/m3/wire.jsonl
    /opt/venv/bin/python3 ~/training/run_mission.py --list                           # the sites and missions left

The program runs as you, from a fresh copy of its folder (regular files only), with the grader's environment (an empty
HOME and TMPDIR, PYTHONPATH = the given client library, MUJOCO_GL=osmesa, BLAS threads 2), the grader's launcher and
its CPU cap (4 x the program-time budget); its stdin / stdout carry the wire to the training ground, its stderr comes
out here. The training ground applies the graded mission's budgets itself (simulated time, sensor frames, program
time, silence) and ends the mission when one runs out. `reset` is this script's (the program's own `reset` calls
answer `forbidden`, as in the grader). Each run uses one mission of the training ground's allowance (robot_manual.md
section 9). Keep this script out of your submission folder: it starts a process, which the grader's audit refuses.
"""

import argparse
import json
import os
import resource
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GIVEN = os.path.join(os.path.dirname(HERE), "given")
LIB = os.path.join(GIVEN, "lib")
sys.path.insert(0, LIB)

from rubblecell.client import TRAINING_STATUS, CellError, connect_training, launcher_argv  # noqa: E402

PROGRAM_TIME_S = 1200.0
CPU_FACTOR = 4.0
NOT_ON_THE_WIRE = ("reset", "truth", "get_state_privileged", "reference_run")


def copy_program(src, dst):
    for dp, dns, fns in os.walk(src):
        dns[:] = [d for d in dns if d != "__pycache__" and not os.path.islink(os.path.join(dp, d))]
        rel = os.path.relpath(dp, src)
        os.makedirs(os.path.join(dst, rel), exist_ok=True)
        for fn in fns:
            p = os.path.join(dp, fn)
            if os.path.isfile(p) and not os.path.islink(p):
                shutil.copy2(p, os.path.join(dst, rel, fn))


def elide(obj):
    if isinstance(obj, dict):
        return {k: ("<%d bytes>" % len(v) if k.endswith("_b64") and isinstance(v, str) else elide(v)) for k, v in obj.items()}
    if isinstance(obj, list):
        return [elide(v) for v in obj]
    return obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", help="a training site: its name or number (--list)")
    ap.add_argument("--program", default=os.path.expanduser("~/submission"))
    ap.add_argument("--keep", default=None, help="keep the run's HOME and TMPDIR in this folder")
    ap.add_argument("--transcript", default=None, help="write every request and reply (images elided) here, JSON lines")
    ap.add_argument("--list", action="store_true", help="print the training ground's sites and allowance")
    args = ap.parse_args()
    if args.list or not args.site:
        try:
            print(json.dumps(json.load(open(TRAINING_STATUS)), indent=1))
        except (OSError, ValueError) as e:
            print("training ground status unreadable: %s" % e)
        return
    robot = connect_training()
    first = robot.reset(args.site)
    print("training mission %d on site %s; missions left after this one: %d" % (first["mission"], first["site"],
                                                                                first["missions_left"]), flush=True)
    run = tempfile.mkdtemp(prefix="training_run_")
    prog = os.path.join(run, "program")
    copy_program(os.path.abspath(args.program), prog)
    home, tmp = os.path.join(run, "home"), os.path.join(run, "tmp")
    os.makedirs(home)
    os.makedirs(tmp)
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "HOME": home, "TMPDIR": tmp, "LANG": "C.UTF-8",
           "MUJOCO_GL": "osmesa", "PYOPENGL_PLATFORM": "osmesa", "PYTHONPATH": LIB,
           "OMP_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}
    cpu_s = int(CPU_FACTOR * PROGRAM_TIME_S)

    def limits():
        resource.setrlimit(resource.RLIMIT_CPU, (cpu_s, cpu_s + 5))

    proc = subprocess.Popen(launcher_argv(sys.executable, LIB, prog), cwd=prog, env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True, bufsize=1, preexec_fn=limits, start_new_session=True)
    tr = open(args.transcript, "w") if args.transcript else None
    t0 = time.time()
    seen = {"requests": 0, "frames": 0, "sweeps": 0, "thermal": 0, "events": [], "sim_time": 0.0, "end": None}
    debrief = None
    send, recv = robot._send, robot._recv
    while True:
        line = proc.stdout.readline()
        if not line:
            seen["end"] = seen["end"] or "the program exited"
            break
        try:
            req = json.loads(line)
            method = str(req.get("method"))
        except ValueError:
            seen["end"] = "protocol: a line that is not a request"
            break
        seen["requests"] += 1
        if method in NOT_ON_THE_WIRE:
            reply_obj = {"id": req.get("id"), "error": "forbidden"}
        else:
            try:
                send(line if line.endswith("\n") else line + "\n")
            except CellError:
                pass
            raw = recv()
            if not raw:
                seen["end"] = seen["end"] or "the training ground closed the mission"
                break
            reply_obj = json.loads(raw)
            if reply_obj.get("event") == "mission_over":
                seen["end"] = "the mission is over: %s" % reply_obj.get("reason")
                debrief = reply_obj.get("debrief")
                break
        res = reply_obj.get("result")
        if isinstance(res, dict):
            if res.get("ok") and method == "get_camera":
                seen["frames"] += 1
            if res.get("ok") and method == "get_lidar":
                seen["sweeps"] += 1
            if res.get("ok") and method == "get_thermal":
                seen["thermal"] += 1
            if "sim_time" in res:
                seen["sim_time"] = float(res["sim_time"])
            for e in res.get("events", []) if method == "step" else []:
                seen["events"].append((round(seen["sim_time"], 1), e))
        if tr is not None:
            tr.write(json.dumps({"t_wall": round(time.time() - t0, 3), "request": req, "reply": elide(reply_obj)}) + "\n")
        try:
            proc.stdin.write(json.dumps(reply_obj) + "\n")
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            seen["end"] = "the program stopped reading"
            break
        if method == "finish" and "result" in reply_obj:
            seen["end"] = "finish"
            break
    try:
        proc.stdin.close()
    except OSError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    if debrief is None:
        # the mission's last line (the debrief) follows the program's finish
        try:
            raw = recv()
            obj = json.loads(raw) if raw else {}
            debrief = obj.get("debrief")
        except (ValueError, CellError, OSError):
            debrief = None
    robot.close()
    if tr is not None:
        tr.close()
    print("end: %s; program exit code %s; sim %.1f s; wall %.0f s; requests %d; camera frames %d; lidar sweeps %d; thermal %d" % (
        seen["end"], proc.returncode, seen["sim_time"], time.time() - t0, seen["requests"], seen["frames"], seen["sweeps"],
        seen["thermal"]))
    if seen["events"]:
        print("events: %s" % seen["events"][:40])
    print("debrief: %s" % json.dumps(debrief, indent=1))
    if args.keep:
        os.makedirs(args.keep, exist_ok=True)
        for name in ("home", "tmp"):
            dst = os.path.join(args.keep, name)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.move(os.path.join(run, name), dst)
        print("the run's HOME and TMPDIR: %s" % args.keep)
    shutil.rmtree(run, ignore_errors=True)


if __name__ == "__main__":
    try:
        main()
    except CellError as e:
        sys.exit("training ground: %s" % e)
