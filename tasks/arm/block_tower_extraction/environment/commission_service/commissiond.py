"""The commissioning service: the tower cell on its practice towers, for the agent's commissioning runs.

It runs the graded cell (towercell/, a byte copy of the grader's tests/towercell/) on the practice towers listed in
sites.json, which are not graded towers (their seeds come from their own salt; tests/review/check_task.py checks the
two sets are disjoint), and answers exactly what the graded cell answers: the cell's public calls, plus
`reset(site)`, which starts an episode on practice tower `site`. It answers no ground truth, no score, no debrief.
environment/Dockerfile installs this folder under /opt/towercell-commission for the account `towercell` only (mode
700: the agent account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session,
under a loop that restarts it should it ever exit. It listens on the unix socket /run/towercell/commission.sock.

A connection is one commissioning episode. Its first request is reset(site); the episode then runs under the graded
episode's budgets, served by the grader's own wire loop (cellkit/wire.py, a byte copy of the grader's). When a budget
ends the episode, the service sends one line {"id": null, "event": "shift_over", "reason": ...} and closes the
connection. The commissioning window: at most `shifts` episodes in the whole session (counted from their reset; the
count survives a restart of the service, in state/ledger.json) and at most `open` at a time; the window is shown in
/run/towercell/commission_status.json (world-readable).
"""

import fcntl
import hashlib
import json
import os
import queue
import select
import signal
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from cellkit import wire  # noqa: E402  (the grader's wire loop; standard library only)

SOCKET = os.environ.get("TOWERCELL_COMMISSION_SOCKET", "/run/towercell/commission.sock")
STATUS = os.environ.get("TOWERCELL_COMMISSION_STATUS", "/run/towercell/commission_status.json")
STATE = os.environ.get("TOWERCELL_COMMISSION_STATE", os.path.join(HERE, "state"))
SITES = json.load(open(os.path.join(HERE, "sites.json")))
PUBLIC = ("describe", "get_state", "get_camera", "step", "ik", "finish")


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


def _sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def check_packages():
    """Practice towers carry their seed in sites.json itself; nothing to check beyond its shape ([] when fine)."""
    bad = []
    for s in SITES["sites"]:
        if not isinstance(s.get("seed"), int):
            bad.append(str(s.get("site")))
    return bad


class Ledger(object):
    """The commissioning window's shift count, kept under an exclusive lock (the sessions run in forked children)."""

    def __init__(self):
        os.makedirs(STATE, exist_ok=True)
        self.path = os.path.join(STATE, "ledger.json")
        self.lock = os.path.join(STATE, "ledger.lock")

    def _read(self):
        try:
            with open(self.path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {"shifts_used": 0, "shifts": []}

    def status(self, d=None):
        d = d or self._read()
        n = int(SITES["shifts"])
        return {"commissioning_shifts": n, "shifts_used": int(d["shifts_used"]), "shifts_left": max(0, n - int(d["shifts_used"])),
                "development_shifts": len(SITES["sites"]), "open_at_a_time": int(SITES["open"])}

    def publish(self):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            _write_json(STATUS, self.status(), 0o644)

    def take(self, site):
        """One shift of the window for `site` -> (shift number, status) or (None, status) when the window is used up."""
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            if int(d["shifts_used"]) >= int(SITES["shifts"]):
                return None, self.status(d)
            d["shifts_used"] = int(d["shifts_used"]) + 1
            d["shifts"].append({"shift": d["shifts_used"], "site": int(site), "started": time.time(), "pid": os.getpid()})
            _write_json(self.path, d, 0o600)
            st = self.status(d)
            _write_json(STATUS, st, 0o644)
            return d["shifts_used"], st

    def give_back(self, shift):
        """A reset refused after the window was taken (a bad inject): the shift is not used."""
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            d["shifts"] = [e for e in d["shifts"] if e.get("shift") != shift]
            d["shifts_used"] = max(0, int(d["shifts_used"]) - 1)
            _write_json(self.path, d, 0o600)
            _write_json(STATUS, self.status(d), 0o644)

    def close(self, shift, reason, sim_time):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            for e in d["shifts"]:
                if e.get("shift") == shift:
                    e.update({"ended": time.time(), "end": reason, "sim_time_s": round(float(sim_time), 2)})
            _write_json(self.path, d, 0o600)


class SocketTransport(object):
    """One client connection as the wire loop's program end: next_request(timeout) -> (kind, obj) (queue.Empty on
    silence), reply(obj, timeout) -> bool with a `stalled` flag; request lines bounded, parsed by the wire's guard."""

    def __init__(self, sock, max_line=wire.MAX_LINE):
        self.sock = sock
        self.buf = b""
        self.max_line = int(max_line)
        self.stalled = False

    def next_request(self, timeout):
        deadline = time.time() + float(timeout)
        while True:
            while b"\n" not in self.buf:
                if len(self.buf) > self.max_line:
                    return "bad", "line too long"
                left = deadline - time.time()
                if left <= 0:
                    raise queue.Empty()
                try:
                    r, _, _ = select.select([self.sock], [], [], left)
                except (OSError, ValueError):
                    return "eof", None
                if not r:
                    raise queue.Empty()
                try:
                    chunk = self.sock.recv(1 << 20)
                except OSError:
                    return "eof", None
                if not chunk:
                    return "eof", None
                self.buf += chunk
            line, self.buf = self.buf.split(b"\n", 1)
            if len(line) > self.max_line:
                return "bad", "line too long"
            kind, obj = wire.parse_request(line)
            if kind is None:
                continue
            return kind, obj

    def reply(self, obj, timeout=None):
        view = memoryview((json.dumps(obj, default=wire._jsonable) + "\n").encode("utf-8"))
        deadline = None if timeout is None else time.time() + float(timeout)
        while len(view):
            wait = None if deadline is None else deadline - time.time()
            if wait is not None and wait <= 0:
                self.stalled = True
                return False
            try:
                _, w, _ = select.select([], [self.sock], [], wait)
            except (OSError, ValueError):
                return False
            if not w:
                continue
            try:
                n = self.sock.send(view[:1 << 20])
            except BlockingIOError:
                continue
            except OSError:
                return False
            view = view[n:]
        return True


class PublicCell(object):
    """The cell as the commissioning wire serves it: the public calls only (anything else answers forbidden and is
    counted, as the graded cell counts it); every attribute the wire loop reads or sets is the cell's own."""

    def __init__(self, server):
        object.__setattr__(self, "_cell", server)

    def handle(self, method, params):
        if method == "reset":
            return {"error": "one_shift_per_connection", "detail": "this connection's shift has begun: open a new one"}
        if method not in PUBLIC:
            self._cell.illegal_calls += 1
            return {"error": "forbidden"}
        return self._cell.handle(method, params)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_cell"), name)

    def __setattr__(self, name, value):
        setattr(object.__getattribute__(self, "_cell"), name, value)


def build_cell(entry):
    from towercell.server import CellServer
    from towercell.tower import draw_tower
    return CellServer(draw_tower(int(entry["seed"]), entry.get("overrides")), duration_s=float(entry["duration_s"]),
                      privileged=False, mode="commission")


def _reply(rid, result):
    if isinstance(result, dict) and "error" in result and set(result) <= {"error", "detail"}:
        return dict({"id": rid}, **result)
    return {"id": rid, "result": result}


def serve(sock, ledger):
    """One commissioning shift on one connection."""
    b = SITES["budgets"]
    silence = float(b.get("silence_s", 20.0))
    start_s = float(b.get("start_s", wire.START_S))
    conn = SocketTransport(sock)
    server = None
    shift = None
    reason = None
    t0 = time.time()
    try:
        while server is None:                       # until reset(site) has started the shift
            try:
                kind, req = conn.next_request(start_s)
            except queue.Empty:
                reason = "stalled"
                return
            if kind != "req":
                reason = "crashed" if kind == "eof" else "protocol"
                return
            rid, method, params = req.get("id"), req.get("method"), req.get("params")
            params = params if isinstance(params, dict) else {}
            n_sites = len(SITES["sites"])
            if method != "reset":
                res = {"error": "no_shift", "detail": "a commissioning episode starts with reset(site), site 1 .. %d" % n_sites}
            else:
                site = params.get("site")
                if isinstance(site, bool) or not isinstance(site, int) or not 1 <= site <= n_sites:
                    res = {"error": "bad_params", "detail": "reset(site): site is 1 .. %d (the practice towers)" % n_sites}
                else:
                    shift, st = ledger.take(site)
                    if shift is None:
                        res = {"error": "commissioning_window_used",
                               "detail": "all %d commissioning episodes are used" % st["commissioning_shifts"]}
                    else:
                        entry = SITES["sites"][site - 1]
                        cell = build_cell(entry)
                        server = PublicCell(cell)
                        log("episode %d: practice tower %d" % (shift, site))
                        res = {"ok": True, "site": site, "shift": shift,
                               "shifts_left": st["shifts_left"], "duration_s": cell.cell.duration,
                               "program_time_s": cell.program_budget_s, "frames": cell.cell.frame_budget}
            if not conn.reply(_reply(rid, res), timeout=silence):
                reason = "stalled" if conn.stalled else "crashed"
                return
        t0 = time.time()
        b = SITES["budgets"]
        reason = wire.serve_episode(server, conn, server.program_budget_s, float(server.silence_s), log=log,
                                    start_s=float(b.get("start_s", wire.START_S)),
                                    frame_price_s=float(b.get("frame_price_s", 0.2)),
                                    call_price_s=float(b.get("call_price_s", 0.005)),
                                    wall_cap_s=float(b.get("episode_wall_cap_s", 2700.0)))
    except Exception as e:  # noqa: BLE001 - a session's failure never takes the service down
        reason = reason or "service_error"
        log("shift %s: session failed: %r" % (shift, e))
    finally:
        if server is not None and reason != "finished":
            try:                                    # the shift is over: say so before the connection closes
                conn.reply({"id": None, "event": "shift_over", "reason": reason}, timeout=5.0)
            except Exception:  # noqa: BLE001
                pass
        if server is not None:
            sim_t = float(server.cell.t)
            log("shift %d: ended (%s) at sim %.1f s, wall %.0f s, program %.0f s" % (
                shift, reason, sim_t, time.time() - t0, float(server.program_used or 0.0)))
            ledger.close(shift, reason, sim_t)
            try:
                server.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            sock.close()
        except OSError:
            pass


def main():
    bad = check_packages()
    if bad:
        log("development shift packages missing or altered: %s; the service does not start" % ", ".join(bad))
        sys.exit("development shift packages missing or altered: %s" % ", ".join(bad))
    ledger = Ledger()
    try:
        os.unlink(SOCKET)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(16)
    ledger.publish()
    log("listening on %s: %d development shifts, %d shifts, %d at a time" % (
        SOCKET, len(SITES["sites"]), SITES["shifts"], SITES["open"]))
    children = set()
    busy = set()
    while True:
        conn, _ = srv.accept()
        for group in (children, busy):
            for pid in list(group):
                try:
                    done, _st = os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                    done = pid
                if done:
                    group.discard(pid)
        if len(children) >= int(SITES["open"]):
            # as many shifts open as the service takes: the connection's first request is answered `busy` (by a child,
            # so that the service never waits on a client), and the connection is closed; no shift is used
            pid = os.fork()
            if pid == 0:
                srv.close()
                try:
                    c = SocketTransport(conn)
                    kind, req = c.next_request(10.0)
                    c.reply({"id": (req or {}).get("id") if kind == "req" else None, "error": "busy",
                             "detail": "at most %d commissioning shifts at a time" % SITES["open"]}, timeout=5.0)
                except Exception:  # noqa: BLE001
                    pass
                finally:
                    os._exit(0)
            busy.add(pid)
            conn.close()
            continue
        pid = os.fork()
        if pid == 0:
            srv.close()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            try:
                serve(conn, ledger)
            except Exception as e:  # noqa: BLE001 - a session's failure never takes the service down
                log("session failed: %r" % e)
            finally:
                os._exit(0)
        children.add(pid)
        conn.close()


if __name__ == "__main__":
    main()
