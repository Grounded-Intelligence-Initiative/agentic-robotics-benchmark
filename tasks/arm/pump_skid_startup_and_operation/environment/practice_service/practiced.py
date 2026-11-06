"""The practice service: the skid series' cell as a black box for the agent's development (a training simulator).

environment/Dockerfile installs this folder (with the cell library, skidcell/) under /opt/skidcell-practice for the
service account `skidcell` only (mode 700: the agent account can neither read nor list it); environment/setup/setup.sh
starts this program under that account before the agent's session, in a loop that restarts it should it ever exit. It
listens on a unix socket every account may connect to (/run/skidcell/practice.sock). A connection is one practice session in a forked child: JSON-lines
requests {"id": n, "method": ..., "params": {...}} answered with {"id": n, "result": ...} or {"id": n, "error": ...},
the graded cell's wire. A session answers the cell's public calls, `reset(seed)` (a fresh shift on the site of that
seed; first call of a session) and `debrief()` (the shift's score breakdown, once the episode is over). It answers no
ground truth: what a practising engineer learns, they learn through the cameras and the debriefs.
"""

import json
import os
import signal
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from skidcell.server import SkidCellServer  # noqa: E402

SOCKET = os.environ.get("SKIDCELL_PRACTICE_SOCKET", "/run/skidcell/practice.sock")
GIVEN = os.environ.get("SKIDCELL_PRACTICE_GIVEN", os.path.join(HERE, "given"))   # holds panda/ (the robot model)
MAX_SESSIONS = int(os.environ.get("SKIDCELL_PRACTICE_MAX", "6"))
IDLE_S = float(os.environ.get("SKIDCELL_PRACTICE_IDLE_S", "1800"))
LOG_PATH = os.environ.get("SKIDCELL_PRACTICE_LOG", os.path.join(HERE, "practiced.log"))


def log(msg):
    try:
        with open(LOG_PATH, "a") as f:
            f.write("%s [%d] %s\n" % (time.strftime("%H:%M:%S"), os.getpid(), msg))
    except OSError:
        pass


def debrief(server):
    """The shift's debrief: what an instructor's report gives the operator after the shift (no hidden state)."""
    s = server.finalize()
    st = server.layout["site"]["tags"]
    logs = [{"t": round(r["t"], 1), "tag": r["tag"], "value": r["value"], "unit": r["unit"], "in_window": r["in_window"],
             "within_tolerance": r["ok"]} for r in s.get("logs", [])]
    return {
        "site": server.layout["site"]["name"], "seed": s["seed"], "episode_score": s["episode_score"], "voided": s["voided"],
        "voids": [{"t": round(v["t"], 1), "what": v["what"]} for v in s["voids"]], "parts": s["parts"], "startup": s["startup"],
        "lineup_at_first_start": {st.get(k, k): v for k, v in (s.get("lineup_at_start") or {}).items()},
        "first_start_t": s["first_start_t"], "window_in_band_fraction": s["window_fraction"],
        "events": [{"type": e["type"], "t": round(e["t"], 1), "credit": e["credit"], "not_judged": e["excluded"], "detail": e["detail"]}
                   for e in s["events"] if e["t"] <= s["sim_time_s"]],          # only what happened before the shift ended
        "shutdown": s["shutdown"], "log_sheet": {st.get(k, k): v for k, v in s["log"].items()}, "log_entries": logs,
        "safety_ok": s["safety_ok"], "violations": [{"t": round(v["t"], 1), "what": v["what"]} for v in s["violations"]],
        "end_reason": s["done_reason"], "sim_time_s": round(s["sim_time_s"], 2), "camera_frames": s["camera_frames"],
        "illegal_calls": s["illegal_call_count"],
    }


def serve(conn):
    conn.settimeout(IDLE_S)
    rf = conn.makefile("r", encoding="utf-8", newline="\n")
    wf = conn.makefile("w", encoding="utf-8", newline="\n")
    server = None
    while True:
        try:
            line = rf.readline()
        except (OSError, socket.timeout):
            break
        if not line:
            break
        try:
            req = json.loads(line)
            rid, method, params = req.get("id"), str(req.get("method")), req.get("params") or {}
        except (ValueError, AttributeError):
            reply = {"id": None, "error": "bad_request: one JSON object per line"}
        else:
            if method == "reset":
                try:
                    seed = int(params.get("seed", 0))
                    if server is None:
                        server = SkidCellServer(GIVEN, mode="practice", privileged=False, allow_reset=True, render=True)
                    server.reset(seed)
                    res = {"ok": True, "seed": seed}
                except (TypeError, ValueError):
                    res = {"error": "bad_params: reset(seed) takes an integer seed"}
                except Exception as e:  # noqa: BLE001 - the session survives; nothing of the code is shown
                    log("reset failed: %r" % e)
                    res = {"error": "internal_error"}
            elif server is None:
                res = {"error": "no episode yet: call reset(seed) first"}
            elif method == "debrief":
                if not server.done:
                    res = {"error": "the episode is not over: call finish() first (or let it run to its end)"}
                else:
                    try:
                        res = debrief(server)
                    except Exception as e:  # noqa: BLE001
                        log("debrief failed: %r" % e)
                        res = {"error": "internal_error"}
            elif method in ("truth", "get_state_privileged", "reference_run"):
                res = {"error": "forbidden"}
            else:
                res = server.handle(method, params)
            if isinstance(res, dict) and "error" in res and len(res) == 1:
                reply = {"id": rid, "error": res["error"]}
            else:
                reply = {"id": rid, "result": res}
        try:
            wf.write(json.dumps(reply) + "\n")
            wf.flush()
        except OSError:
            break
    for f in (wf, rf):
        try:
            f.close()
        except OSError:
            pass
    conn.close()


def main():
    try:
        os.unlink(SOCKET)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(16)
    log("listening on %s (at most %d sessions)" % (SOCKET, MAX_SESSIONS))
    children = set()
    while True:
        conn, _ = srv.accept()
        for pid in list(children):
            try:
                done, _st = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                done = pid
            if done:
                children.discard(pid)
        if len(children) >= MAX_SESSIONS:
            try:
                conn.sendall((json.dumps({"id": None, "error": "busy: at most %d practice sessions at a time" % MAX_SESSIONS})
                              + "\n").encode())
            finally:
                conn.close()
            continue
        pid = os.fork()
        if pid == 0:
            srv.close()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            try:
                serve(conn)
            except Exception as e:  # noqa: BLE001
                log("session failed: %r" % e)
            finally:
                os._exit(0)
        children.add(pid)
        conn.close()


if __name__ == "__main__":
    main()
