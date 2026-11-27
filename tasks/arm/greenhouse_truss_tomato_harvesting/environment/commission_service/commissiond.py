"""The commissioning service: the cell on the practice row sections, for the automation engineer's own trials.

Runs in the agent's sandbox as the account `rowcell` (started by environment/setup/setup.sh before the agent's
session), the only account that can read this folder. It listens on a unix socket and runs one session at a time:
a session is one practice row section in a fresh cell process (`rowcell/server.py`, the cell the grader runs), driven
with the cell's public actions. It never asks the cell for its privileged view or for a yield (the cell is started
without the grader's switch, so it would refuse), and it reports nothing about a session but what the program saw.

The wire (one JSON object per line, per connection):

    {"op": "status"}                       -> {"sessions_left": n, "sessions_used": k, "rows": [1, 2, ...], "busy": bool}
    {"op": "open", "row": 1}               -> {"obs": <first observation>, "session": k, "sessions_left": n}
    {"op": "step", "action": <action>}     -> {"obs": <observation>}   or   {"error": "..."}
    {"op": "close"}                        -> {"ok": true}

A session is counted when it opens. It ends when the row section ends (the observation's `ended`), when the client
closes or disconnects, or after `idle_s` without a request. The number of sessions is fixed for the agent's whole
session and kept in this account's state folder, so a restart of the service does not reset it.
"""

import json
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rowcell import cellproc  # noqa: E402
from rowcell import farm as F  # noqa: E402

RUN_DIR = os.environ.get("ROWCELL_RUN_DIR", "/run/rowcell")
STATE_DIR = os.environ.get("ROWCELL_STATE_DIR", os.path.join(HERE, "state"))
SOCK = os.path.join(RUN_DIR, "commission.sock")
with open(os.path.join(HERE, "dev_rows.json")) as _f:
    CONF = json.load(_f)
LOCK = threading.Lock()
BUSY = threading.Lock()


def log(*a):
    print(time.strftime("[commission %H:%M:%S]"), *a, file=sys.stderr, flush=True)


def state():
    try:
        with open(os.path.join(STATE_DIR, "sessions.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"used": 0, "log": []}


def save(st):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = os.path.join(STATE_DIR, "sessions.json.tmp")
    with open(tmp, "w") as f:
        json.dump(st, f)
    os.replace(tmp, os.path.join(STATE_DIR, "sessions.json"))
    public = {"sessions": CONF["sessions"], "sessions_used": st["used"], "sessions_left": max(0, CONF["sessions"] - st["used"]),
              "rows": list(range(1, len(CONF["rows"]) + 1))}
    tmp = os.path.join(RUN_DIR, "status.json.tmp")
    with open(tmp, "w") as f:
        json.dump(public, f)
    os.chmod(tmp, 0o644)
    os.replace(tmp, os.path.join(RUN_DIR, "status.json"))
    return public


def cell_env():
    env = {k: v for k, v in os.environ.items() if k.startswith(("NVIDIA_", "ROWCELL_", "OMNI_", "OPENBLAS_", "OMP_"))}
    work = os.path.join(STATE_DIR, "cell")
    for d in ("kit", "tmp", "optix", "warp", "home"):
        os.makedirs(os.path.join(work, d), exist_ok=True)
    env.update(HOME=os.path.join(work, "home"), PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", LANG="C.UTF-8",
               OMNI_KIT_ACCEPT_EULA="YES", ACCEPT_EULA="Y", PRIVACY_CONSENT="Y", TMPDIR=os.path.join(work, "tmp"),
               OPTIX_CACHE_PATH=os.path.join(work, "optix"), WARP_CACHE_PATH=os.path.join(work, "warp"))
    env.setdefault("ROWCELL_KIT_DIR", os.path.join(work, "kit"))
    return env, work


def serve(conn):
    f = conn.makefile("rwb")
    cell, held = None, False

    def send(obj):
        f.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))
        f.flush()

    try:
        conn.settimeout(float(CONF["idle_s"]))
        while True:
            line = f.readline()
            if not line:
                break
            try:
                msg = json.loads(line.decode("utf-8"))
                op = msg.get("op")
            except (ValueError, AttributeError):
                send({"error": "one JSON object per line"})
                continue
            if op == "status":
                with LOCK:
                    st = save(state())
                send(dict(st, busy=BUSY.locked()))
            elif op == "open":
                k = msg.get("row")
                if cell is not None:
                    send({"error": "this connection already has a session"})
                elif isinstance(k, bool) or not isinstance(k, int) or not 1 <= k <= len(CONF["rows"]):
                    send({"error": "row must be an integer from 1 to %d" % len(CONF["rows"])})
                elif not BUSY.acquire(timeout=float(CONF.get("busy_wait_s", 120))):
                    # (a session that has just closed needs some seconds to stop its cell: wait for it)
                    send({"error": "busy: one session at a time"})
                else:
                    held = True
                    with LOCK:
                        st = state()
                        if st["used"] >= CONF["sessions"]:
                            send({"error": "no commissioning sessions left"})
                            BUSY.release()
                            held = False
                            continue
                        st["used"] += 1
                        st["log"].append({"row": k, "opened": time.strftime("%H:%M:%S")})
                        public = save(st)
                    entry = CONF["rows"][k - 1]
                    row = F.draw_entry(entry)
                    env, work = cell_env()
                    try:
                        with open(os.path.join(work, "cell.log"), "wb") as err:
                            cell = cellproc.CellProcess(grader=False, env=env, python="/opt/venv/bin/python3" if os.path.exists("/opt/venv/bin/python3") else sys.executable,
                                                        cwd=work, stderr=err, start_timeout=float(CONF["start_timeout_s"]))
                        reply = cell.request({"cmd": "reset", "row": row, "config": CONF.get("cell") or {}, "cams": msg.get("cams") or []},
                                             timeout=float(CONF["start_timeout_s"]))
                        send({"obs": reply["obs"], "session": public["sessions_used"], "sessions_left": public["sessions_left"]})
                        log("session", public["sessions_used"], "opened on practice row", k)
                    except cellproc.CellError as exc:
                        log("the cell failed to start:", str(exc)[:300])
                        with LOCK:                      # a session the cell could not start is not counted
                            st = state()
                            st["used"] = max(0, st["used"] - 1)
                            save(st)
                        send({"error": "the cell did not start; the session was not counted, try again"})
                        if cell is not None:
                            cell.kill()
                        cell = None
                        BUSY.release()
                        held = False
            elif op == "step":
                if cell is None:
                    send({"error": "open a session first"})
                    continue
                try:
                    reply = cell.request({"cmd": "step", "action": msg.get("action")}, timeout=600.0)
                except cellproc.CellError as exc:
                    log("the cell failed:", str(exc)[:300])
                    send({"error": "the cell failed; the session is over"})
                    break
                send({"obs": reply["obs"]} if "obs" in reply else {"error": str(reply.get("error", "refused"))})
            elif op == "close":
                send({"ok": True})
                break
            else:
                send({"error": "unknown op"})
    except (OSError, socket.timeout):
        pass
    finally:
        if cell is not None:
            cell.close()
            log("session closed")
        if held:
            BUSY.release()
        try:
            conn.close()
        except OSError:
            pass


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    with LOCK:
        save(state())
    try:
        os.unlink(SOCK)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK)
    os.chmod(SOCK, 0o666)
    srv.listen(8)
    log("listening on", SOCK, "-", CONF["sessions"], "sessions on", len(CONF["rows"]), "practice rows")
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
