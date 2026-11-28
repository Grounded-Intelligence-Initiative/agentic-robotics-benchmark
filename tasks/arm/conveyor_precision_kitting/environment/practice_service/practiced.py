"""The practice cell: the conveyor kitting cell, for a few practice shifts during the agent's session.

environment/Dockerfile installs this folder under /opt/kitcell for the account `kitcell` only (mode 700: the agent
account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session, under a loop
that restarts it should it ever exit. cpk_env.py here is a byte copy of tests/cpk_env.py, the grader's cell.

It listens on /run/kitcell/practice.sock (mode 666). A connection is one practice shift, on one of the cell's
practice seeds (never a graded shift: graded shifts draw 64-bit seeds from the grader's entropy). Requests are JSON
lines {"method": ..., "params": {...}}; replies {"result": ...} or {"error": ...}:

    reset            -> {"obs": <wire observation>, "shifts_left": n}      (counts one shift of the window)
    step {action}    -> {"obs": ..., "done": bool, "info": {"t", "time", "fault", "station_busy", "hold_steps",
                                                             "ended"}}
    close            -> {"kit": [per pocket: "seated" | "unseated" | "damaged" | "empty"], "lost": n, "t": ...}

The cell does not wait for its client, exactly as in grading (tests/verify.py): the time from sending an
observation until the next `step` request arrives is the client's reply time; for every whole REPLY_BUDGET_S it took,
the cell first runs one hold step (no motion, the gripper as last commanded, no images, no turnover request) and the
client never sees those observations (`hold_steps` says how many ran). A request later than REQUEST_LIMIT_S ends
the shift, and so does SHIFT_WALL_S of wall-clock time since the reset (`ended` gives the reason); the kit keeps what
is seated at that moment. A shift driven by hand therefore watches the parts go by.

`kit` is what the downstream kit inspection reports for the shift's kit when the shift ends (or when it is closed
early); `lost` counts the parts that left the pick zone or fell off the table. No poses or other state of the
simulator are ever answered. At most SHIFTS shifts in the whole session (the count survives a restart, in
state/ledger.json); one at a time; a shift with no request for SILENCE_S seconds is closed.
"""

import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cpk_env as K  # noqa: E402

SOCKET = "/run/kitcell/practice.sock"
STATUS = "/run/kitcell/practice_status.json"
STATE = os.path.join(HERE, "state")
SHIFTS = 2                      # the practice budget (a difficulty knob)
REPLY_BUDGET_S = 0.1            # per control step, as in grading (tests/grader_config.json reply_budget_s)
REQUEST_LIMIT_S = 30.0          # a step request later than this ends the shift
SHIFT_WALL_S = 90.0          # wall-clock cap per practice shift: the reference needs about 40 s (in the image,
                                # 4 CPUs, 2026-11-27); about 2x for a busy sandbox
SILENCE_S = REQUEST_LIMIT_S + 5.0
PRACTICE_SEEDS = [7300001 + 7919 * i for i in range(SHIFTS)]
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
        json.dump({"shifts_total": SHIFTS, "shifts_used": n, "shifts_left": SHIFTS - n}, f)
    os.chmod(STATUS + ".tmp", 0o644)
    os.replace(STATUS + ".tmp", STATUS)


def log(msg):
    with open(os.path.join(STATE, "practiced.log"), "a") as f:
        f.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))


def serve_one(conn):
    conn.settimeout(SILENCE_S)
    rf = conn.makefile("rb")
    wf = conn.makefile("wb")
    env, done = None, False
    obs, grip, sent_at, started_at = None, -1.0, 0.0, 0.0

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
                    reply({"error": "one shift per connection; open a new connection"})
                    continue
                n = used()
                if n >= SHIFTS:
                    reply({"error": "the practice window is used up (%d shifts)" % SHIFTS})
                    break
                set_used(n + 1)
                env = K.CellEnv(render=True)
                obs = env.reset(PRACTICE_SEEDS[n])
                log("shift %d started" % n)
                reply({"result": {"obs": K.to_wire(obs), "shifts_left": SHIFTS - n - 1}})
                started_at = sent_at = time.monotonic()
            elif method == "step":
                if env is None or done:
                    reply({"error": "no shift running" if env is None else "the shift is over; close it"})
                    continue
                now = time.monotonic()
                spent = now - sent_at
                ended, holds = None, 0
                if spent > REQUEST_LIMIT_S:
                    ended = "no step request for %.0f s" % REQUEST_LIMIT_S
                elif now - started_at > SHIFT_WALL_S:
                    ended = "the shift's wall-clock cap of %.0f s" % SHIFT_WALL_S
                if ended is not None:
                    done = True
                    log("shift ended: " + ended)
                    reply({"result": {"obs": K.to_wire(obs), "done": True,
                                      "info": {"t": env.t, "time": obs["time"], "fault": env.fault,
                                               "station_busy": obs["station_busy"], "hold_steps": 0,
                                               "ended": ended}}})
                    continue
                # the world kept moving while the client thought: whole budgets spent run on a hold command
                for _ in range(int(spent // REPLY_BUDGET_S)):
                    obs, done, info = env.step([0.0, 0.0, 0.0, 0.0, grip, 0, 0])
                    holds += 1
                    if done:
                        break
                if not done:
                    try:
                        action = [float(a) for a in params.get("action")]
                        obs, done, info = env.step(action)
                    except (K.ActionError, TypeError, ValueError) as exc:
                        reply({"error": str(exc)})
                        sent_at = time.monotonic()
                        continue
                    grip = float(min(1.0, max(-1.0, action[4])))
                reply({"result": {"obs": K.to_wire(obs), "done": bool(done),
                                  "info": {"t": env.t, "time": obs["time"], "fault": env.fault,
                                           "station_busy": obs["station_busy"], "hold_steps": holds,
                                           "ended": "the shift is over" if done else None}}})
                sent_at = time.monotonic()
            elif method == "close":
                if env is None:
                    reply({"result": {"kit": [], "lost": 0, "t": 0}})
                else:
                    reply({"result": {"kit": env.inspect_kit(), "lost": env.status.count("lost"), "t": env.t}})
                break
            else:
                reply({"error": "unknown method"})
    except (socket.timeout, OSError, ValueError):
        pass
    finally:
        if env is not None:
            try:
                log("shift closed at t=%d kit=%s" % (env.t, env.inspect_kit()))
            except Exception:  # noqa: BLE001
                pass
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
