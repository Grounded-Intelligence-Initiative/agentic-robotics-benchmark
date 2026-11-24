"""The commissioning service: the camera holder on development procedures, for the agent's own trials.

Installed for the account `endosim` alone (the agent's account can neither read nor list this folder) and started by
environment/setup/setup.sh. It runs the benchmark's simulator (endosim/, a byte copy of the grader's) on the
development procedures of dev_procedures.json, never on a graded one, and does two things:

1. on its first start it records the development set (endosim/recorder.py: every development procedure as video
   with labels) into /var/lib/endocam/dev_procedures, which the agent reads as ~/given/dev_procedures;
2. it serves commissioning sessions on the Unix socket /run/endocam/commission.sock: one development procedure played
   for the caller's controller with the grader's observations, latency and limits. A fixed number of sessions, one at
   a time. No ground truth, no labels, no score, no summary: a session answers observations and nothing else.

    request (one JSON object per line)                         reply
    {"cmd": "status"}                                          {"ok": true, "ready": .., "recorded": n, "sessions_left": n, "procedures": [..], ..}
    {"cmd": "start", "procedure": "dev03"}                     {"ok": true, "session": k, "steps": N, "t": 0, "obs": {...}}
    {"cmd": "step", "action": [pan, tilt, insertion, roll]}    {"ok": true, "t": t, "done": false, "obs": {...}}   (no obs once done)
    {"cmd": "probe_copy", "shift": [dx, dy], "joint_offset": [a, b, c]}   {"ok": true, "obs": {...}}   a perturbed copy of the current observation

A session ends with the procedure, when the connection closes, or after IDLE_S without a request; it counts from
`start` whatever happens next.
"""

import json
import os
import socket
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from endosim import procedure, recorder  # noqa: E402
from endosim.render_client import Renderer, RendererError  # noqa: E402
from endosim.session import Session  # noqa: E402

STATE_DIR = os.environ.get("ENDOCAM_STATE_DIR", "/var/lib/endocam")
SOCKET = os.environ.get("ENDOCAM_SOCKET", "/run/endocam/commission.sock")
PYTHON = os.environ.get("ENDOCAM_PYTHON", "/opt/venv/bin/python3")
IDLE_S = 120.0
FRAME_DELAY = 1


def log(*parts):
    with open(os.path.join(STATE_DIR, "service.log"), "a") as f:
        f.write(time.strftime("%Y-%m-%dT%H:%M:%S ") + " ".join(str(p) for p in parts) + "\n")


class Service(object):
    def __init__(self):
        with open(os.path.join(HERE, "dev_procedures.json")) as f:
            self.doc = json.load(f)
        self.items = {p["name"]: p for p in self.doc["procedures"]}
        self.steps = int(self.doc["steps"])
        self.state_path = os.path.join(STATE_DIR, "state.json")
        self.state = {"sessions_used": 0, "recorded": []}
        if os.path.isfile(self.state_path):
            with open(self.state_path) as f:
                self.state.update(json.load(f))
        self.lock = threading.Lock()            # one user of the renderer at a time: the recorder, or a session
        self.state_lock = threading.Lock()
        self.renderer = None
        self.ready = False
        self.failure = None

    def save(self):
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f)
        os.replace(tmp, self.state_path)

    def start_renderer(self):
        self.renderer = Renderer(HERE, python=PYTHON, log_path=os.path.join(STATE_DIR, "renderer.log"), start_timeout=1500.0,
                                 cwd=STATE_DIR)
        canary = self.renderer.canary()
        log("renderer up", json.dumps(self.renderer.started), "canary", json.dumps(canary)[:500])
        bad = [s for s, r in canary.get("scopes", {}).items() if r["found"] < r["markers"] or (r["max_error_px"] or 99) > 2.5]
        if bad or not canary.get("ok"):
            raise RendererError("the render canary failed: {}".format(json.dumps(canary)[:400]))

    def record_all(self):
        """The development set, once: every development procedure as video with labels."""
        out = os.path.join(STATE_DIR, "dev_procedures")
        os.makedirs(out, exist_ok=True)
        os.chmod(out, 0o755)
        for item in self.doc["procedures"]:
            if item["name"] in self.state["recorded"]:
                continue
            with self.lock:
                proc = procedure.generate(item["seed"], item.get("factors"), steps=self.steps)
                recorder.record(proc, self.renderer, out, item["name"], every=int(self.doc.get("record_every", 2)),
                                operator_seed=item["seed"], touch=bool(item.get("touch")), log=log)
            for root, dirs, files in os.walk(os.path.join(out, item["name"])):
                os.chmod(root, 0o755)
                for name in files:
                    os.chmod(os.path.join(root, name), 0o644)
            with self.state_lock:
                self.state["recorded"].append(item["name"])
                self.save()
        with open(os.path.join(out, "COMPLETE"), "w") as f:
            f.write("{} recordings\n".format(len(self.doc["procedures"])))
        os.chmod(os.path.join(out, "COMPLETE"), 0o644)
        log("development set complete")

    def status(self):
        with self.state_lock:
            return {"ok": True, "ready": self.ready, "failure": self.failure, "recorded": len(self.state["recorded"]),
                    "recordings": len(self.doc["procedures"]), "sessions_left": int(self.doc["sessions"]) - int(self.state["sessions_used"]),
                    "procedures": sorted(self.items), "steps": self.steps, "busy": self.lock.locked()}

    # -- one connection ------------------------------------------------------------------------------------------------
    def serve(self, conn):
        conn.settimeout(IDLE_S)
        rfile, session, holding = conn.makefile("r", encoding="utf-8"), None, False

        def reply(obj):
            conn.sendall((json.dumps(obj) + "\n").encode("utf-8"))

        try:
            for line in rfile:
                try:
                    msg = json.loads(line)
                    cmd = msg.get("cmd")
                except (ValueError, AttributeError):
                    reply({"ok": False, "error": "a request is one JSON object per line"})
                    continue
                if cmd == "status":
                    reply(self.status())
                elif cmd == "start":
                    if session is not None:
                        reply({"ok": False, "error": "this connection already has a session"})
                        continue
                    name = msg.get("procedure")
                    if name not in self.items:
                        reply({"ok": False, "error": "unknown procedure; one of {}".format(sorted(self.items))})
                        continue
                    if not self.ready:
                        reply({"ok": False, "error": self.failure or "the service is still starting"})
                        continue
                    if not self.lock.acquire(blocking=False):
                        reply({"ok": False, "error": "busy: another session, or the development set is still being recorded"})
                        continue
                    holding = True
                    with self.state_lock:
                        left = int(self.doc["sessions"]) - int(self.state["sessions_used"])
                        if left <= 0:
                            reply({"ok": False, "error": "no commissioning sessions left"})
                            self.lock.release()
                            holding = False
                            continue
                        self.state["sessions_used"] += 1
                        number = self.state["sessions_used"]
                        self.save()
                    item = self.items[name]
                    proc = procedure.generate(item["seed"], item.get("factors"), steps=self.steps)
                    session = Session(proc, self.renderer, frame_delay=FRAME_DELAY, privileged=False)
                    log("session", number, name, "started")
                    reply({"ok": True, "session": number, "sessions_left": left - 1, "steps": self.steps, "t": 0, "obs": session.observation()})
                elif cmd == "step":
                    if session is None or session.done:
                        reply({"ok": False, "error": "no procedure is running"})
                        continue
                    try:
                        session.step(msg.get("action"))
                    except (ValueError, TypeError) as exc:
                        reply({"ok": False, "error": str(exc)})
                        continue
                    out = {"ok": True, "t": session.t, "done": session.done}
                    if not session.done:
                        out["obs"] = session.observation()
                    reply(out)
                    if session.done:
                        log("session ended: procedure complete")
                        break
                elif cmd == "probe_copy":
                    if session is None or session.done:
                        reply({"ok": False, "error": "no procedure is running"})
                        continue
                    try:
                        shift = [float(v) for v in msg.get("shift")][:2]
                        offs = [float(v) for v in msg.get("joint_offset")][:3]
                        reply({"ok": True, "obs": session.probe_copy(session.observation(), shift, offs)})
                    except (ValueError, TypeError, RendererError) as exc:
                        reply({"ok": False, "error": str(exc)[:200]})
                else:
                    reply({"ok": False, "error": "unknown cmd"})
        except (OSError, socket.timeout, RendererError) as exc:
            log("connection ended:", type(exc).__name__, str(exc)[:200])
        finally:
            if holding:
                self.lock.release()
            try:
                conn.close()
            except OSError:
                pass


def main():
    os.umask(0o077)                 # the service's own files (state, logs) are its account's alone
    os.makedirs(STATE_DIR, exist_ok=True)
    svc = Service()
    os.makedirs(os.path.dirname(SOCKET), exist_ok=True)
    try:
        os.remove(SOCKET)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(8)

    def bring_up():
        try:
            svc.start_renderer()
            svc.ready = True
            with open(os.path.join(STATE_DIR, "ready"), "w") as f:
                f.write("ready\n")
            svc.record_all()
        except Exception:  # noqa: BLE001
            svc.failure = "the commissioning service failed: " + traceback.format_exc()[-600:]
            log(svc.failure)
            with open(os.path.join(STATE_DIR, "failed"), "w") as f:
                f.write(svc.failure)

    threading.Thread(target=bring_up, daemon=True).start()
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=svc.serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
