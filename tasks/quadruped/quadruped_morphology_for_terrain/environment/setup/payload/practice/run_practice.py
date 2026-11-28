"""Walk a design over the practice section of the walkway, the way the grader walks it.

    /opt/venv/bin/python3 ~/practice/run_practice.py [--design ~/submission/robot.xml] [--no-video]
    /opt/venv/bin/python3 ~/practice/run_practice.py --status

The practice walkway checks the design first (a design that fails the build check costs no run and prints the
checker's message), then runs it once with the standard gait over the practice section of that run number and
answers how the run ended: the progress fraction along the 3 m track, the outcome (goal, fall, timeout, fault), its
reason and time, and the path of a side-view video of the run (/run/qmcell/videos/). A run takes up to about a
minute; its outcome is also saved as /run/qmcell/videos/practice_run_NN.json.

EACH RUN USES ONE OF THE SESSION'S 4 PRACTICE RUNS (/run/qmcell/practice_status.json). Practice sections are fixed
instances of the walkway's terrain family (run n always gets section n), never a graded instance.
"""

import argparse
import json
import os
import socket

SOCKET = "/run/qmcell/practice.sock"


def call(req, timeout=900.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(SOCKET)
    f = s.makefile("rwb")
    f.write((json.dumps(req) + "\n").encode())
    f.flush()
    line = f.readline()
    s.close()
    if not line:
        raise SystemExit("the practice walkway closed the connection")
    return json.loads(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--design", default=os.path.expanduser("~/submission/robot.xml"))
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--status", action="store_true")
    a = ap.parse_args()
    if a.status:
        print(json.dumps(call({"method": "status"})["result"]))
        return
    with open(a.design) as f:
        xml = f.read()
    rep = call({"method": "run", "params": {"robot_xml": xml, "video": not a.no_video}})
    if "error" in rep:
        raise SystemExit(rep["error"])
    r = rep["result"]
    print("practice run %d: progress %.3f, %s (%s) at t = %.1f s; runs left %d" % (
        r["run"], r["progress"], r["outcome"], r["reason"], r["time_s"], r["runs_left"]))
    if r.get("video"):
        print("video:", r["video"])


if __name__ == "__main__":
    main()
