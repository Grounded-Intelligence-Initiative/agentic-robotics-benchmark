"""The practice cell: the catching cell, for a few practice runs of throws during the agent's session.

environment/Dockerfile installs this folder under /opt/dexcell for the account `dexcell` only (mode 700: the agent
account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session, under a loop
that restarts it should it ever exit. cell.py here is a byte copy of the grader's cell.

It listens on /run/dexcell/practice.sock (mode 666). A connection is one practice run of THROWS throws, on one of the
cell's practice seeds (never a graded run: graded runs draw 63-bit seeds from the grader's entropy). Requests are
JSON lines {"method": ..., "params": {...}}; replies {"result": ...} or {"error": ...}:

    reset            -> {"obs": <wire observation>, "runs_left": n}      (counts one run of the budget)
    step {action}    -> {"obs": <wire observation or null>, "done": bool, "throw": <per-throw outcome or null>}
    close            -> {"throws": [<per-throw outcome>, ...]}

The cell steps once per `step` request and never waits: the practice tool (run_practice.py) applies the grader's
timing rule itself by sending the previous action again for the steps a late reply missed. A per-throw outcome holds
the tool, the requested grip zone, whether it was caught and held, and the credit and its parts; no pose or other
state of the simulator is ever answered. At most RUNS runs in the whole session (the count survives a restart, in
state/ledger.json); one at a time; a run with no request for SILENCE_S seconds is closed.
"""

import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ALLEGRO_XML", os.path.join(HERE, "wonik_allegro", "right_hand.xml"))
import cell as C  # noqa: E402
import wire as W  # noqa: E402

SOCKET = "/run/dexcell/practice.sock"
STATUS = "/run/dexcell/practice_status.json"
STATE = os.path.join(HERE, "state")
RUNS = 4
THROWS = 4
SILENCE_S = 300.0
PRACTICE_SEEDS = [8200003 + 7919 * i for i in range(RUNS)]
MAX_LINE = 16384


def ledger_path():
    return os.path.join(STATE, "ledger.json")


def used():
    try:
        with open(ledger_path()) as f:
            return int(json.load(f)["used"])
    except (OSError, ValueError, KeyError):
        return 0


def set_used(n):
    tmp = ledger_path() + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"used": n}, f)
    os.replace(tmp, ledger_path())
    with open(STATUS + ".tmp", "w") as f:
        json.dump({"runs_total": RUNS, "throws_per_run": THROWS, "runs_used": n, "runs_left": RUNS - n}, f)
    os.chmod(STATUS + ".tmp", 0o644)
    os.replace(STATUS + ".tmp", STATUS)


def log(msg):
    with open(os.path.join(STATE, "practiced.log"), "a") as f:
        f.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))


def outcome(r):
    """The per-throw outcome the practice cell reports (the cell's own judgement of the throw)."""
    out = {"tool": r.get("tool"), "zone": r.get("zone"), "caught_and_held": bool(r.get("caught")),
           "credit": round(float(r.get("score", 0.0)), 4)}
    for k in ("along", "handle", "axis", "palm", "steady", "up_deg", "across_deg", "disp", "wrist"):
        if k in r:
            out[k] = round(float(r[k]), 4)
    return out


def serve_one(conn):
    conn.settimeout(SILENCE_S)
    rf = conn.makefile("rb")
    wf = conn.makefile("wb")
    env, done, throws = None, False, []

    def reply(obj):
        wf.write((json.dumps(obj) + "\n").encode())
        wf.flush()

    try:
        while True:
            line = rf.readline(MAX_LINE + 1)
            if not line:
                break
            if len(line) > MAX_LINE:
                reply({"error": "request too long"})
                break
            try:
                req = json.loads(line)
                method = req.get("method")
                params = req.get("params") or {}
            except (ValueError, AttributeError):
                reply({"error": "malformed request"})
                continue
            if method == "reset":
                if env is not None:
                    reply({"error": "one run per connection; open a new connection"})
                    continue
                n = used()
                if n >= RUNS:
                    reply({"error": "the practice budget is used up (%d runs)" % RUNS})
                    break
                set_used(n + 1)
                env = C.Episode(PRACTICE_SEEDS[n], n_throws=THROWS, render=True)
                obs = env.reset()
                log("run %d started" % n)
                reply({"result": {"obs": W.to_wire(obs), "runs_left": RUNS - n - 1}})
            elif method == "step":
                if env is None or done:
                    reply({"error": "no run going" if env is None else "the run is over; close it"})
                    continue
                try:
                    obs, done, info = env.step(params.get("action"))
                except C.ActionError as exc:
                    reply({"error": str(exc)})
                    continue
                th = outcome(info["throw_result"]) if "throw_result" in info else None
                if th is not None:
                    throws.append(th)
                reply({"result": {"obs": W.to_wire(obs) if obs is not None else None, "done": bool(done),
                                  "throw": th}})
            elif method == "close":
                reply({"result": {"throws": throws}})
                break
            else:
                reply({"error": "unknown method"})
    except (socket.timeout, OSError, ValueError):
        pass
    finally:
        if env is not None:
            log("run closed: %s" % [t["credit"] for t in throws])
            env.close()
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
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        serve_one(conn)


if __name__ == "__main__":
    main()
