#!/opt/venv/bin/python3
"""Practice run: runs YOUR program over the real wire against the practice service (a training simulator of the skid
series) on the site (seed) of your choice, the way the grader runs it, and prints the shift's debrief.

    /opt/venv/bin/python3 ~/practice/grade.py                    # ~/submission on site 1
    /opt/venv/bin/python3 ~/practice/grade.py --seed 7 --program ~/submission --json debrief.json

The program runs as you, from a fresh copy of its folder (regular files only), with the grader's environment (an empty
HOME and TMPDIR, PYTHONPATH = the given client library, ARB_GIVEN_DIR = ~/given, MUJOCO_GL=osmesa, BLAS threads 2) and
the grader's launcher; its stdin / stdout carry the wire to the practice service, its stderr comes out here. The graded
run's caps apply: 2400 s of program time (the wall clock minus the cell's time inside `step` and `get_camera`) and
180 s of silence. The practice-only calls (`reset`, `debrief`) answer `forbidden` inside the run, as in the grader.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GIVEN = os.environ.get("ARB_GIVEN_DIR", os.path.join(os.path.dirname(HERE), "given"))
LIB = os.path.join(GIVEN, "lib")
sys.path.insert(0, LIB)

from skidcell.client import CellError, connect_practice, launcher_argv  # noqa: E402

PRACTICE_ONLY = ("reset", "debrief", "truth", "get_state_privileged", "reference_run")


def copy_program(src, dst):
    """The submission folder's regular files (no symbolic links, no __pycache__), as the grader copies it."""
    for dp, dns, fns in os.walk(src):
        dns[:] = [d for d in dns if d != "__pycache__" and not os.path.islink(os.path.join(dp, d))]
        rel = os.path.relpath(dp, src)
        os.makedirs(os.path.join(dst, rel), exist_ok=True)
        for fn in fns:
            p = os.path.join(dp, fn)
            if os.path.isfile(p) and not os.path.islink(p):
                shutil.copy2(p, os.path.join(dst, rel, fn))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1, help="the practice site (any integer)")
    ap.add_argument("--program", default=os.path.expanduser("~/submission"))
    ap.add_argument("--program-time-cap", type=float, default=2400.0)
    ap.add_argument("--silence", type=float, default=180.0)
    ap.add_argument("--json", default=None, help="write the debrief here")
    args = ap.parse_args()

    cell = connect_practice(seed=args.seed)
    run = tempfile.mkdtemp(prefix="practice_run_")
    prog = os.path.join(run, "program")
    copy_program(os.path.abspath(args.program), prog)
    home, tmp = os.path.join(run, "home"), os.path.join(run, "tmp")
    os.makedirs(home)
    os.makedirs(tmp)
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "HOME": home, "TMPDIR": tmp, "LANG": "C.UTF-8",
           "MUJOCO_GL": "osmesa", "PYOPENGL_PLATFORM": "osmesa", "PYTHONPATH": LIB, "ARB_GIVEN_DIR": GIVEN,
           "OMP_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}
    proc = subprocess.Popen(launcher_argv(sys.executable, LIB, prog), cwd=prog, env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True, bufsize=1)
    t0 = time.time()
    state = {"cell_s": 0.0, "inflight": None, "last_reply": t0, "reason": None}
    lock = threading.Lock()

    def program_time(now):
        with lock:
            busy = state["cell_s"] + ((now - state["inflight"]) if state["inflight"] is not None else 0.0)
        return (now - t0) - busy

    def watchdog():
        while proc.poll() is None:
            now = time.time()
            with lock:
                waiting = state["inflight"] is None
                silent = now - state["last_reply"]
            if waiting and silent > args.silence:
                state["reason"] = "silence (%.0f s without a request)" % silent
            elif program_time(now) > args.program_time_cap:
                state["reason"] = "program_time_cap"
            if state["reason"]:
                proc.kill()
                return
            time.sleep(0.5)

    threading.Thread(target=watchdog, daemon=True).start()
    send, recv = cell._send, cell._recv
    while True:
        line = proc.stdout.readline()
        if not line:
            break
        try:
            req = json.loads(line)
            method = str(req.get("method"))
        except ValueError:
            reply = json.dumps({"id": None, "error": "bad_request: one JSON object per line"}) + "\n"
            method = None
        else:
            if method in PRACTICE_ONLY:
                reply = json.dumps({"id": req.get("id"), "error": "forbidden"}) + "\n"
            else:
                ts = time.time()
                with lock:
                    if method in ("step", "get_camera"):
                        state["inflight"] = ts
                send(line if line.endswith("\n") else line + "\n")
                reply = recv()
                with lock:
                    if state["inflight"] is not None:
                        state["cell_s"] += time.time() - state["inflight"]
                        state["inflight"] = None
                if not reply:
                    state["reason"] = state["reason"] or "the practice service closed the session"
                    proc.kill()
                    break
        try:
            proc.stdin.write(reply)
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            break
        with lock:
            state["last_reply"] = time.time()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    wall = time.time() - t0
    prog_s = program_time(time.time())
    reason = state["reason"] or ("program exited (code %s)" % proc.returncode)
    try:
        cell.finish()
    except CellError:
        pass
    db = cell.debrief()
    cell.close()
    shutil.rmtree(run, ignore_errors=True)
    print("site %s (seed %d)   end: %s / %s   sim %.1f s   wall %.1f s (program %.1f s, cell %.1f s)   frames %s   illegal calls %d" % (
        db["site"], args.seed, reason, db["end_reason"], db["sim_time_s"], wall, prog_s, state["cell_s"],
        json.dumps(db["camera_frames"]), db["illegal_calls"]))
    print("startup  %s" % json.dumps(db["startup"]))
    print("line-up  %s" % json.dumps(db["lineup_at_first_start"]))
    w = db["window_in_band_fraction"]
    print("window   in band: level %.0f %%, chlorine %.0f %% of the window" % (100 * w["level"], 100 * w["conc"]))
    for e in db["events"]:
        print("event    %-15s at %6.1f s  credit %.2f%s  %s" % (e["type"], e["t"], e["credit"], "  (not judged)" if e["not_judged"] else "",
                                                             json.dumps(e["detail"])))
    print("shutdown %s" % json.dumps(db["shutdown"]))
    print("log      %s" % json.dumps(db["log_sheet"]))
    print("voids    %s" % [v["what"] for v in db["voids"]][:5])
    print("parts    %s" % json.dumps(db["parts"]))
    print("episode_score: %.4f" % db["episode_score"])
    if args.json:
        with open(args.json, "w") as f:
            json.dump(db, f, indent=1)


if __name__ == "__main__":
    main()
