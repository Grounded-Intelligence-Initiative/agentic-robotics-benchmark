"""The practice walkway: the site's test section, for a few practice runs of a design during the agent's session.

environment/Dockerfile installs this folder under /opt/qmcell for the account `qmcell` only (mode 700: the agent
account can neither read nor list it), together with the simulator (cell/qm_sim.py, a byte copy of the grader's
tests/qm_sim.py) and the catalog, design and gait modules; environment/setup/setup.sh starts it before the agent's
session, under a loop that restarts it should it ever exit.

It listens on /run/qmcell/practice.sock (mode 666). One connection carries one JSON request line and gets one JSON
reply line:

    {"method": "status"}                               -> {"result": {"runs_total", "runs_used", "runs_left"}}
    {"method": "run", "params": {"robot_xml": "<...>", "video": true}}
        -> {"result": {"run", "runs_left", "progress", "outcome", "reason", "time_s", "video"}}  or {"error": ...}

A design that fails the build check is answered with the checker's message and costs no run. A run walks the design
with the standard gait over the practice section of run number n (fixed practice instances of the walkway's terrain
family, never a graded one: graded instances come from the grader's own entropy). The reply holds only the outcome:
the progress fraction, how the run ended and when, and the path of a video of the run (side view, 640x480, made
readable for everybody); the same outcome is kept in practice_run_NN.json beside the video. At most RUNS runs in the whole session (the count survives a restart, in state/).
"""

import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import qm_design as D  # noqa: E402
import qm_sim as S  # noqa: E402

SOCKET = "/run/qmcell/practice.sock"
STATUS = "/run/qmcell/practice_status.json"
VIDEOS = "/run/qmcell/videos"
STATE = os.path.join(HERE, "state")
RUNS = 4
PRACTICE_SEEDS = [8100003 + 7919 * i for i in range(RUNS)]
MAX_LINE = 4 * D.MAX_BYTES
CATALOG = D.load_catalog(os.path.join(HERE, "catalog.json"))


def used():
    try:
        with open(os.path.join(STATE, "ledger.json")) as f:
            return int(json.load(f)["used"])
    except (OSError, ValueError, KeyError):
        return 0


def set_used(n):
    tmp = os.path.join(STATE, "ledger.json.tmp")
    with open(tmp, "w") as f:
        json.dump({"used": n}, f)
    os.replace(tmp, os.path.join(STATE, "ledger.json"))
    with open(STATUS + ".tmp", "w") as f:
        json.dump({"runs_total": RUNS, "runs_used": n, "runs_left": RUNS - n}, f)
    os.chmod(STATUS + ".tmp", 0o644)
    os.replace(STATUS + ".tmp", STATUS)


def log(msg):
    with open(os.path.join(STATE, "practiced.log"), "a") as f:
        f.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))


def run_practice(design, n, video):
    terrain = S.make_terrain(np.random.default_rng(PRACTICE_SEEDS[n]))
    ep = S.Episode(design, CATALOG, terrain)
    frames, renderer, cam = [], None, None
    if video:
        import mujoco
        ep.m.light_castshadow[:] = 0  # software rendering: no shadows keeps a run with its video near half a minute
        renderer = mujoco.Renderer(ep.m, 480, 640)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.azimuth, cam.elevation = 1.6, 75.0, -12.0

        def cb(e, step):
            if step % 4 == 0:
                cam.lookat[:] = [e.d.qpos[0], 0.0, e.d.qpos[2] - 0.08]
                renderer.update_scene(e.d, cam)
                frames.append(renderer.render().copy())
    else:
        cb = None
    r = ep.run(frame_cb=cb)
    path = ""
    if frames:
        import cv2
        os.makedirs(VIDEOS, exist_ok=True)
        path = os.path.join(VIDEOS, "practice_run_%02d.mp4" % n)
        vw = cv2.VideoWriter(path + ".tmp.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 12.5, (640, 480))
        for k, f in enumerate(frames):
            img = cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
            cv2.putText(img, "practice run %d   t = %.1f s" % (n, k * 4 * S.TIMESTEP * S.SUBSTEPS), (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            vw.write(img)
        vw.release()
        os.chmod(path + ".tmp.mp4", 0o644)
        os.replace(path + ".tmp.mp4", path)
        renderer.close()
    return r, path


def handle(req):
    method = req.get("method")
    params = req.get("params") or {}
    if method == "status":
        n = used()
        return {"result": {"runs_total": RUNS, "runs_used": n, "runs_left": RUNS - n}}
    if method != "run":
        return {"error": "unknown method"}
    xml = params.get("robot_xml")
    if not isinstance(xml, str):
        return {"error": "params.robot_xml must be the text of robot.xml"}
    try:
        design, _ = D.check(xml.encode("utf-8"), CATALOG)
    except D.DesignError as exc:
        return {"error": "design rejected (no run used): %s" % exc}
    n = used()
    if n >= RUNS:
        return {"error": "the practice runs are used up (%d)" % RUNS}
    set_used(n + 1)
    log("run %d started" % n)
    r, path = run_practice(design, n, bool(params.get("video", True)))
    log("run %d: %s %s %.3f" % (n, r["outcome"], r["reason"], r["progress"]))
    res = {"run": n, "runs_left": RUNS - n - 1, "progress": round(float(r["progress"]), 4),
           "outcome": r["outcome"], "reason": r["reason"], "time_s": r["time_s"], "video": path}
    # the outcome is also kept beside the videos, for a client that lost its connection during the run
    os.makedirs(VIDEOS, exist_ok=True)
    out = os.path.join(VIDEOS, "practice_run_%02d.json" % n)
    with open(out + ".tmp", "w") as f:
        json.dump(res, f)
    os.chmod(out + ".tmp", 0o644)
    os.replace(out + ".tmp", out)
    return {"result": res}


def serve_one(conn):
    conn.settimeout(60.0)
    rf = conn.makefile("rb")
    wf = conn.makefile("wb")
    try:
        line = rf.readline(MAX_LINE + 1)
        if len(line) > MAX_LINE:
            rep = {"error": "request too long"}
        else:
            try:
                req = json.loads(line)
                rep = handle(req) if isinstance(req, dict) else {"error": "malformed request"}
            except ValueError:
                rep = {"error": "malformed request"}
            except Exception as exc:  # noqa: BLE001
                log("internal error: %r" % exc)
                rep = {"error": "the practice walkway failed on this request"}
        wf.write((json.dumps(rep) + "\n").encode())
        wf.flush()
    except (socket.timeout, OSError):
        pass
    finally:
        for f in (rf, wf):
            try:
                f.close()
            except OSError:
                pass
        conn.close()


def main():
    os.makedirs(STATE, exist_ok=True)
    set_used(used())
    try:
        os.unlink(SOCKET)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(4)
    while True:
        conn, _ = srv.accept()
        serve_one(conn)


if __name__ == "__main__":
    main()
