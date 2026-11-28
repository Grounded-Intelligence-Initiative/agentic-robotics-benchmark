"""The commissioning service of server_hot_swap_maintenance: the graded cell (srv_sim / srv_env / episode, byte copies
of the grader's) on development units, for the agent's commissioning sessions.

environment/Dockerfile installs this folder under /opt/srvcell-commission for the account `srvcell` only (mode 700:
the agent account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session
under a loop that restarts it should it exit. It listens on the unix socket /run/srvcell/commission.sock.

A connection is one session, one episode on one development unit (dev_units.json; none is a graded unit, each has its
own hidden values drawn from the same ranges). Protocol, one JSON object per line:
  client  {"type": "open", "unit": "d1"}
  service {"ok": true, "session": k, "sessions_left": n, "max_steps": N}  then  {"obs": <observation>}
  client  {"action": <action>}            (repeated)
  service {"obs": <observation>}  or, when the episode is over, {"obs": ..., "over": "<the cell's message>"}
The cell's message says why the episode ended (the controller said done, the step limit, or the incident that
stopped the cell); it never says whether the work order was done right: no credit, no ground truth. A session
counts when it is opened, whatever happens next; the ledger survives a restart of the service. The status (sessions
used and left) is in /run/srvcell/commission_status.json (world-readable).
"""

import fcntl
import hashlib
import json
import os
import select
import signal
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SOCKET = os.environ.get("SRV_COMMISSION_SOCKET", "/run/srvcell/commission.sock")
STATUS = os.environ.get("SRV_COMMISSION_STATUS", "/run/srvcell/commission_status.json")
STATE = os.environ.get("SRV_COMMISSION_STATE", os.path.join(HERE, "state"))
DEV = json.load(open(os.path.join(HERE, "dev_units.json")))
MAX_LINE = 1 << 22
SILENCE_S = 120.0
WALL_CAP_S = 3600.0


def log(msg):
    try:
        with open(os.path.join(STATE, "commissiond.log"), "a") as f:
            f.write("%s [%d] %s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S"), os.getpid(), msg))
    except OSError:
        pass


def _write_json(path, obj, mode):
    tmp = path + ".tmp%d" % os.getpid()
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
    os.chmod(tmp, mode)
    os.replace(tmp, path)


class Ledger(object):
    def __init__(self):
        os.makedirs(STATE, exist_ok=True)
        self.path = os.path.join(STATE, "ledger.json")
        self.lock = os.path.join(STATE, "ledger.lock")

    def _read(self):
        try:
            with open(self.path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {"used": 0, "sessions": []}

    def status(self, d=None):
        d = d or self._read()
        n = int(DEV["sessions"])
        return {"sessions": n, "used": int(d["used"]), "left": max(0, n - int(d["used"])),
                "units": sorted(DEV["units"])}

    def publish(self):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            _write_json(STATUS, self.status(), 0o644)

    def take(self, unit):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            if int(d["used"]) >= int(DEV["sessions"]):
                return None, self.status(d)
            d["used"] = int(d["used"]) + 1
            d["sessions"].append({"session": d["used"], "unit": unit, "started": time.time()})
            _write_json(self.path, d, 0o600)
            st = self.status(d)
            _write_json(STATUS, st, 0o644)
            return d["used"], st

    def close(self, k, reason, steps):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            for e in d["sessions"]:
                if e.get("session") == k:
                    e.update({"ended": time.time(), "end": reason, "steps": int(steps)})
            _write_json(self.path, d, 0o600)


class Conn(object):
    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def read(self, timeout):
        deadline = time.time() + timeout
        while b"\n" not in self.buf:
            if len(self.buf) > MAX_LINE:
                return None
            left = deadline - time.time()
            if left <= 0:
                return None
            r, _, _ = select.select([self.sock], [], [], left)
            if not r:
                return None
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                return None
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        try:
            obj = json.loads(line.decode("utf-8"))
        except ValueError:
            return None
        return obj if isinstance(obj, dict) else None

    def send(self, obj):
        self.sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))


def session_seed(unit, k):
    h = hashlib.blake2b(("%s|%s|%d" % (DEV["salt"], unit, k)).encode(), digest_size=8).digest()
    return int.from_bytes(h, "little")


def serve(sock, ledger):
    import episode as EP
    import srv_env as E
    c = Conn(sock)
    req = c.read(30.0)
    if not req or req.get("type") != "open":
        c.send({"error": "a session starts with {\"type\": \"open\", \"unit\": ...}"})
        return
    if req.get("status"):
        c.send(ledger.status())
        return
    unit = req.get("unit")
    if unit not in DEV["units"]:
        c.send({"error": "unknown unit; the development units are %s" % ", ".join(sorted(DEV["units"]))})
        return
    k, st = ledger.take(unit)
    if k is None:
        c.send({"error": "all %d commissioning sessions are used" % st["sessions"]})
        return
    env = E.SrvEnv(DEV["units"][unit], unit, render=True)
    ep = EP.Episode(env, session_seed(unit, k))
    reason, t0 = "service error", time.time()
    log("session %d: unit %s" % (k, unit))
    try:
        obs = ep.reset()
        c.send({"ok": True, "session": k, "sessions_left": st["left"], "max_steps": EP.EPISODE_STEPS})
        w = E.to_wire(obs)
        w["unit"] = unit
        c.send({"obs": w})
        while True:
            if time.time() - t0 > WALL_CAP_S:
                reason = "the session's wall-clock cap (%d s) ran out" % WALL_CAP_S
                c.send({"over": reason})
                break
            msg = c.read(SILENCE_S)
            if msg is None:
                reason = "the client went silent or closed"
                break
            try:
                obs, done, _ = ep.step(msg.get("action"))
            except (ValueError, TypeError, AttributeError, KeyError):
                reason = "malformed action"
                c.send({"over": reason})
                break
            w = E.to_wire(obs)
            w["unit"] = unit
            if done:
                end = ep.end
                if end == "incident":
                    reason = "cell stopped: " + str(env.w.lost)
                elif end == "unstable":
                    reason = "cell stopped: the arms' controllers faulted (an unstable motion)"
                elif end == "done":
                    reason = "the controller said done"
                else:
                    reason = "the step limit (%d) was reached" % EP.EPISODE_STEPS
                c.send({"obs": w, "over": reason})
                break
            c.send({"obs": w})
    except (OSError, BrokenPipeError):
        reason = "the client went away"
    except Exception as e:  # noqa: BLE001 - a session's failure never takes the service down
        log("session %d failed: %r" % (k, e))
        try:
            c.send({"over": "service error"})
        except OSError:
            pass
    finally:
        ledger.close(k, reason, env.t)
        log("session %d: over (%s) at step %d, wall %.0f s" % (k, reason, env.t, time.time() - t0))
        env.close()


def main():
    ledger = Ledger()
    try:
        os.unlink(SOCKET)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(8)
    ledger.publish()
    log("listening on %s: %d sessions" % (SOCKET, DEV["sessions"]))
    signal.signal(signal.SIGCHLD, signal.SIG_IGN)
    while True:
        conn, _ = srv.accept()
        pid = os.fork()
        if pid == 0:
            srv.close()
            try:
                serve(conn, ledger)
            except Exception as e:  # noqa: BLE001
                log("session failed: %r" % e)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
                os._exit(0)
        conn.close()


if __name__ == "__main__":
    main()
