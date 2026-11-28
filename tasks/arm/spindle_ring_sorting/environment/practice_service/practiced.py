"""The practice cell: the ring cell, for a few practice episodes during the agent's session.

environment/Dockerfile installs this folder under /opt/ringcell for the account `ringcell` only (mode 700: the agent
account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session, under a loop
that restarts it should it ever exit. ring_env.py here is a byte copy of tests/ring_env.py, the grader's cell.

It listens on /run/ringcell/practice.sock (mode 666). A connection is one practice episode, on one of the cell's
practice seeds (never a graded episode: graded episodes draw 64-bit seeds from the grader's entropy). Requests are
JSON lines {"method": ..., "params": {...}}; replies {"result": ...} or {"error": ...}:

    reset            -> {"obs": <wire observation>, "episodes_left": n}      (counts one episode of the window)
    step {action}    -> {"obs": ..., "done": bool, "info": {"t", "fault"}}
    probe            -> {"obs": ...}   a copy of the last observation as the grader's repeated-t probe builds it
    close            -> {"rings_sorted": r, "spindles_complete": c, "rings_damaged": d, "t": ...}

At close the cell reports what its end-of-line inspection reports: the rings sorted, the complete spindles and the
damaged rings, the raw numbers of the task's metric. No poses or other state of the simulator are ever answered. At
most EPISODES episodes in the whole session (the count survives a restart, in state/ledger.json); one at a time; an
episode with no request for SILENCE_S seconds is closed.
"""

import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ring_env as K  # noqa: E402

SOCKET = "/run/ringcell/practice.sock"
STATUS = "/run/ringcell/practice_status.json"
STATE = os.path.join(HERE, "state")
EPISODES = 1
SILENCE_S = 300.0
PRACTICE_SEEDS = [5300017 + 7919 * i for i in range(EPISODES)]
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


def serve_one(conn):
    conn.settimeout(SILENCE_S)
    rf = conn.makefile("rb")
    wf = conn.makefile("wb")
    env, done, last = None, False, None
    prng = None

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
                    reply({"error": "one episode per connection; open a new connection"})
                    continue
                n = used()
                if n >= EPISODES:
                    reply({"error": "the practice window is used up (%d episodes)" % EPISODES})
                    break
                set_used(n + 1)
                env = K.RingEnv(render=True)
                obs = env.reset(PRACTICE_SEEDS[n])
                last = obs
                import numpy as np
                prng = np.random.default_rng(PRACTICE_SEEDS[n] + 1)
                log("episode %d started" % n)
                reply({"result": {"obs": K.to_wire(obs), "episodes_left": EPISODES - n - 1}})
            elif method == "step":
                if env is None or done:
                    reply({"error": "no episode running" if env is None else "the episode is over; close it"})
                    continue
                try:
                    obs, done, info = env.step(params.get("action"))
                except K.ActionError as exc:
                    reply({"error": str(exc)})
                    continue
                last = obs
                reply({"result": {"obs": K.to_wire(obs), "done": bool(done),
                                  "info": {"t": info["t"], "fault": info["fault"]}}})
            elif method == "probe":
                if env is None or last is None:
                    reply({"error": "no episode running"})
                    continue
                reply({"result": {"obs": K.to_wire(env.perturbed_observation(last, prng))}})
            elif method == "close":
                if env is not None:
                    r, c = env.score()
                    res = {"rings_sorted": int(r), "spindles_complete": int(c), "rings_damaged": int(sum(env.damaged)),
                           "t": env.t}
                else:
                    res = {"rings_sorted": 0, "spindles_complete": 0, "rings_damaged": 0, "t": 0}
                reply({"result": res})
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
