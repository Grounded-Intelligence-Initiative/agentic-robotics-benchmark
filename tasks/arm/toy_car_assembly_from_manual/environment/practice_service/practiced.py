"""The practice cell: the toy-car cell on the practice kit, for a few practice episodes during the agent's session.

environment/Dockerfile installs this folder under /opt/toycell for the account `toycell` only (mode 700: the agent
account can neither read nor list it), with toycar_env.py and toycar_manual.py (byte copies of the grader's cell).
environment/setup/setup.sh starts it before the agent's session, under a loop that restarts it should it ever exit.
Each practice episode draws its car of the family from its fixed practice seed, exactly as a graded episode does from
its hidden seed; the agent's home holds those practice cars' numbers and manuals (~/examples/), and the episode's
manual also arrives with the observation at t = 0, as in a graded episode.

It listens on /run/toycell/practice.sock (mode 666). A connection is one practice episode, on one of the cell's
fixed practice seeds (never a graded episode: graded episodes draw 64-bit seeds from the grader's entropy and build
the graded kit). Requests are JSON lines {"method": ..., "params": {...}}; replies {"result": ...} or {"error": ...}:

    reset            -> {"obs": <wire observation>, "episodes_left": n}      (spends one episode of the window)
    step {action}    -> {"obs": <wire observation>, "done": bool}
    close            -> {"steps": t, "mates_with_kit_parts": n, "rolls": bool}

Nothing else of the simulator is ever answered: no poses, no latch events. At most EPISODES episodes in the whole
session (the count survives a restart, in state/ledger.json); one at a time; an episode with no request for
SILENCE_S seconds is closed.
"""

import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import toycar_env as E  # noqa: E402

SOCKET = "/run/toycell/practice.sock"
STATUS = "/run/toycell/practice_status.json"
STATE = os.path.join(HERE, "state")
EPISODES = 3                     # difficulty knob: practice episodes per session
SILENCE_S = 300.0
# The graded episode's deadlines hold here too (difficulty knob): after each reply the next request must come within
# STEP_DEADLINE_S, and the time spent between replies and requests adds up to at most REPLY_BUDGET_S per episode.
STEP_DEADLINE_S = 30.0
REPLY_BUDGET_S = 600.0
PRACTICE_SEEDS = [9100003 + 7919 * i for i in range(EPISODES)]
MAX_LINE = 4096


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
        json.dump({"episodes_total": EPISODES, "episodes_used": n, "episodes_left": EPISODES - n}, f)
    os.chmod(STATUS + ".tmp", 0o644)
    os.replace(STATUS + ".tmp", STATUS)


def log(msg):
    with open(os.path.join(STATE, "practiced.log"), "a") as f:
        f.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))


def counts(env):
    roll = env.roll_test()
    return {"steps": int(env.t), "mates_with_kit_parts": int(env.credit()), "rolls": bool(roll["rolls"])}


def serve_one(conn):
    conn.settimeout(SILENCE_S)
    rf = conn.makefile("rb")
    wf = conn.makefile("wb")
    env, done = None, False
    clock = {"sent": None, "used": 0.0}

    def reply(obj):
        wf.write((json.dumps(obj) + "\n").encode())
        wf.flush()
        clock["sent"] = time.monotonic()

    try:
        while True:
            line = rf.readline(MAX_LINE + 1)
            if not line:
                break
            if env is not None and not done and clock["sent"] is not None:
                gap = time.monotonic() - clock["sent"]
                clock["used"] += gap
                if gap > STEP_DEADLINE_S or clock["used"] > REPLY_BUDGET_S:
                    why = ("no request within %.0f s of the last reply" % STEP_DEADLINE_S if gap > STEP_DEADLINE_S
                           else "the controller's time in this episode passed %.0f s" % REPLY_BUDGET_S)
                    log("episode ended at t=%d: %s" % (env.t, why))
                    reply({"error": "episode over: %s (the grader's deadlines)" % why})
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
                    reply({"error": "one episode per connection; open a new connection"})
                    continue
                n = used()
                if n >= EPISODES:
                    reply({"error": "the practice window is used up (%d episodes)" % EPISODES})
                    break
                set_used(n + 1)
                env = E.ToyCarEnv()
                obs = env.reset(PRACTICE_SEEDS[n])
                log("episode %d started" % n)
                reply({"result": {"obs": E.to_wire(obs), "episodes_left": EPISODES - n - 1}})
            elif method == "step":
                if env is None or done:
                    reply({"error": "no episode running" if env is None else "the episode is over; close it"})
                    continue
                try:
                    obs, done, _info = env.step(params.get("action"))
                except E.ActionError as exc:
                    reply({"error": str(exc)})
                    continue
                reply({"result": {"obs": E.to_wire(obs), "done": bool(done)}})
            elif method == "close":
                reply({"result": counts(env) if env is not None else {"steps": 0, "mates_with_kit_parts": 0,
                                                                       "rolls": False}})
                break
            else:
                reply({"error": "unknown method"})
    except (socket.timeout, OSError, ValueError):
        pass
    finally:
        if env is not None:
            log("episode closed at t=%d" % env.t)
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
