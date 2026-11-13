"""The training ground: the team's rubble training site, for the agent's development missions.

A response-robot team does not deploy an autonomy mode that never ran in rubble: it trains on the standard test lanes
and on training rubble piles (a training site's buildings are collapsed for exercises, a volunteer plays the trapped
person), and after each exercise the trainers debrief the team. This service is that training ground. It runs the
graded robot cell (rubblecell/, a byte copy of the grader's tests/rubblecell/ without the reference program) on the
sites listed in sites.json: the test lanes and training sites, none of them a graded site or a candidate of the hidden
set's selection, and answers exactly what the graded cell answers (the robot's public calls), plus `reset(site)`, which
starts a mission on a site. When a mission ends, the trainers' debrief (where the person was in the robot's map frame,
how the report, the kit and the return went, any secondary collapse or contact with the person) is sent as the last
line of the connection and saved, world-readable, in /run/rubblecell/debriefs/. Nothing else of the site (no map, no
layout) is ever sent.

environment/Dockerfile installs this folder under /opt/rubblecell-training for the account `rubblecell` only (mode 700:
the agent account can neither read nor list it); environment/setup/setup.sh starts it before the agent's session,
under a loop that restarts it should it ever exit. It listens on the unix socket /run/rubblecell/training.sock.

A connection is one mission. Its first request is reset(site) (a site name or its number, sites.json); the mission
then runs under the graded mission's budgets: the cell's own (simulated time, camera frames, lidar sweeps, thermal
frames) and the wire's (program time = the wall clock since the reset minus the cell's time inside step and the
sensors; silence between requests). When a budget ends the mission, the service sends {"event": "mission_over"} with
the debrief and closes the connection. The allowance: at most `missions` missions in the whole session (counted from
their reset; the count survives a restart of the service) and at most `open` at a time; the status is in
/run/rubblecell/training_status.json (world-readable).
"""

import fcntl
import json
import os
import signal
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SOCKET = os.environ.get("RUBBLECELL_TRAINING_SOCKET", "/run/rubblecell/training.sock")
STATUS = os.environ.get("RUBBLECELL_TRAINING_STATUS", "/run/rubblecell/training_status.json")
DEBRIEFS = os.environ.get("RUBBLECELL_TRAINING_DEBRIEFS", "/run/rubblecell/debriefs")
STATE = os.environ.get("RUBBLECELL_TRAINING_STATE", os.path.join(HERE, "state"))
SITES = json.load(open(os.path.join(HERE, "sites.json")))
PUBLIC = ("describe", "get_state", "drive", "set_flippers", "set_gauge", "set_mast", "set_head", "set_arm", "arm_to",
          "gripper", "step", "get_camera", "get_lidar", "get_thermal", "report_survivor", "finish")
AFTER_DONE_MAX_STEPS = 50
AFTER_DONE_MAX_REQUESTS = 200
AFTER_DONE_MAX_S = 60.0
MAX_LINE = 4 * 1024 * 1024


def log(msg):
    try:
        with open(os.path.join(STATE, "trainingd.log"), "a") as f:
            f.write("%s [%d] %s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S"), os.getpid(), msg))
    except OSError:
        pass


def _write_json(path, obj, mode):
    tmp = path + ".tmp%d" % os.getpid()
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
    os.chmod(tmp, mode)
    os.replace(tmp, path)


def _site_list():
    return [{"number": i + 1, "name": s["name"], "about": s.get("about", "")} for i, s in enumerate(SITES["sites"])]


class Ledger(object):
    """The training ground's mission count, kept under an exclusive lock (the missions run in forked children)."""

    def __init__(self):
        os.makedirs(STATE, exist_ok=True)
        self.path = os.path.join(STATE, "ledger.json")
        self.lock = os.path.join(STATE, "ledger.lock")

    def _read(self):
        try:
            with open(self.path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {"missions_used": 0, "missions": []}

    def status(self, d=None):
        d = d or self._read()
        n = int(SITES["missions"])
        return {"training_missions": n, "missions_used": int(d["missions_used"]),
                "missions_left": max(0, n - int(d["missions_used"])), "open_at_a_time": int(SITES["open"]),
                "sites": _site_list(), "debriefs": DEBRIEFS}

    def publish(self):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            _write_json(STATUS, self.status(), 0o644)

    def take(self, site_idx):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            if int(d["missions_used"]) >= int(SITES["missions"]):
                return None, self.status(d)
            d["missions_used"] = int(d["missions_used"]) + 1
            d["missions"].append({"mission": d["missions_used"], "site": SITES["sites"][site_idx]["name"],
                                  "started": time.time(), "pid": os.getpid()})
            _write_json(self.path, d, 0o600)
            st = self.status(d)
            _write_json(STATUS, st, 0o644)
            return d["missions_used"], st

    def close(self, mission, reason, sim_time):
        with open(self.lock, "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            d = self._read()
            for e in d["missions"]:
                if e.get("mission") == mission:
                    e.update({"ended": time.time(), "end": reason, "sim_time_s": round(float(sim_time), 2)})
            _write_json(self.path, d, 0o600)


class Conn(object):
    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def read(self, timeout):
        deadline = time.time() + timeout
        while b"\n" not in self.buf:
            left = deadline - time.time()
            if left <= 0:
                raise socket.timeout()
            self.sock.settimeout(left)
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                return None
            self.buf += chunk
            if len(self.buf) > MAX_LINE and b"\n" not in self.buf:
                raise ValueError("line too long")
        line, self.buf = self.buf.split(b"\n", 1)
        obj = json.loads(line.decode("utf-8"))
        if not isinstance(obj, dict) or "method" not in obj:
            raise ValueError("one request object per line")
        return obj

    def send(self, obj, timeout):
        self.sock.settimeout(timeout)
        self.sock.sendall((json.dumps(obj, default=_jsonable) + "\n").encode("utf-8"))


def _jsonable(obj):
    if hasattr(obj, "tolist"):
        return obj.tolist()
    return str(obj)


def _reply(rid, result):
    if isinstance(result, dict) and "error" in result and set(result) <= {"error", "detail"}:
        return dict({"id": rid}, **result)
    return {"id": rid, "result": result}


def _site_index(v):
    names = [s["name"] for s in SITES["sites"]]
    if isinstance(v, str) and v in names:
        return names.index(v)
    try:
        k = int(v)
    except (TypeError, ValueError):
        return -1
    return k - 1 if 1 <= k <= len(names) else -1


def debrief(server, mission, site_name, reason):
    """What the trainers tell the team after the exercise (the person's position in the robot's map frame)."""
    import math
    s = server.score()
    tor = server.surv_torso
    xm, ym, _ = server.odom.to_map(float(tor[0]), float(tor[1]), 0.0)
    sv = server.plan["survivor"]
    out = {
        "mission": mission, "site": site_name, "ended": reason, "sim_time_s": round(float(server.d.time), 1),
        "person_position_map_frame": {"x": round(xm, 2), "y": round(ym, 2), "z": 0.0,
                                      "note": "the middle of the person's torso, in the map frame of this mission"},
        "person_lying": {"back": "on the back", "side": "on the side", "prone": "face down"}.get(sv.get("pose"), "?"),
        "report": {"reports_made": s["reports"], "reports_received_by_the_team": s["reports_sent"],
                   "error_m": None if s["report_error_m"] is None else round(s["report_error_m"], 2),
                   "person_seen_in_a_frame_before_the_report": s["seen_before_report"]},
        "kit": {"delivered_within_reach": bool(s["parts"]["kit"] > 0),
                "distance_to_nearest_head_or_hand_m": None if s["kit_dist_m"] is None else round(s["kit_dist_m"], 2)},
        "robot": {"back_at_deployment_point": bool(s["home_dist_m"] <= server.cfg["score"]["home_radius_m"]),
                  "distance_from_deployment_point_m": round(s["home_dist_m"], 2), "lost": server.lost},
        "safety": [v["what"] for v in s["voids"]],
        "mission_score": round(float(s["score"]), 3), "score_parts": s["parts"],
    }
    if math.isnan(out["mission_score"]):
        out["mission_score"] = 0.0
    return out


def serve(sock, ledger):
    """One training mission on one connection."""
    conn = Conn(sock)
    silence = float(SITES["silence_s"])
    cap = float(SITES["program_time_cap_s"])
    server = None
    mission = None
    site_name = None
    reason = None
    t0 = None
    n_after = 0
    done_since = None
    try:
        while True:
            if server is not None:
                prog = (time.time() - t0) - float(server.work_wall)
                if prog >= cap:
                    reason = "program_time_cap"
                    break
                wait = min(silence, cap - prog)
            else:
                wait = silence
            try:
                req = conn.read(wait)
            except socket.timeout:
                reason = "silence" if server is None or (time.time() - t0) - float(server.work_wall) < cap else "program_time_cap"
                break
            except (ValueError, UnicodeDecodeError) as e:
                reason = "protocol (%s)" % str(e)[:60]
                break
            if req is None:
                reason = "eof"
                break
            rid, method, params = req.get("id"), str(req.get("method")), req.get("params")
            params = params if isinstance(params, dict) else {}
            if method == "reset":
                if server is not None:
                    res = {"error": "one_mission_per_connection", "detail": "this connection's mission has begun: open a new one"}
                else:
                    k = _site_index(params.get("site"))
                    if k < 0:
                        res = {"error": "bad_params", "detail": "reset(site): one of %s (or its number)" %
                               ", ".join(s["name"] for s in SITES["sites"])}
                    else:
                        mission, st = ledger.take(k)
                        if mission is None:
                            res = {"error": "training_allowance_used", "detail": "all %d training missions are used" % st["training_missions"]}
                        else:
                            from rubblecell.server import RubbleCellServer
                            site = SITES["sites"][k]
                            site_name = site["name"]
                            server = RubbleCellServer(config=SITES["config"], mode="hidden", privileged=False, render=True, log=log)
                            server.reset(int(site["seed"]), site["kind"])
                            t0 = time.time()
                            server.wall_start = t0
                            log("mission %d: site %s" % (mission, site_name))
                            res = {"ok": True, "site": site_name, "mission": mission, "missions_left": st["missions_left"]}
            elif server is None:
                res = {"error": "no_mission", "detail": "a training mission starts with reset(site)"}
            elif method in PUBLIC:
                res = server.handle(method, params)
            else:
                server.illegal_calls += 1
                res = {"error": "forbidden"}
            conn.send(_reply(rid, res), timeout=silence)
            if server is not None and server.done:
                if method == "finish":
                    reason = "finish"
                    break
                if done_since is None:
                    done_since = time.time()
                n_after += 1
                if method == "step":
                    server.steps_after_done = getattr(server, "steps_after_done", 0) + 1
                if (getattr(server, "steps_after_done", 0) > AFTER_DONE_MAX_STEPS or n_after > AFTER_DONE_MAX_REQUESTS
                        or time.time() - done_since > AFTER_DONE_MAX_S):
                    reason = server.done_reason or "done"
                    break
    except (OSError, socket.timeout) as e:
        reason = reason or "connection (%s)" % type(e).__name__
    finally:
        if server is not None:
            if not server.done:
                server._end(reason or "closed")
            try:
                db = debrief(server, mission, site_name, reason)
                os.makedirs(DEBRIEFS, exist_ok=True)
                _write_json(os.path.join(DEBRIEFS, "mission_%03d.json" % mission), db, 0o644)
            except Exception as e:  # noqa: BLE001
                db = {"error": "no debrief (%s)" % type(e).__name__}
            try:
                conn.send({"id": None, "event": "mission_over", "reason": reason, "debrief": db}, timeout=5.0)
            except (OSError, socket.timeout):
                pass
            log("mission %d: ended (%s) at sim %.1f s, wall %.0f s" % (mission, reason, float(server.d.time), time.time() - t0))
            ledger.close(mission, reason, float(server.d.time))
            try:
                server.sensors.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            sock.close()
        except OSError:
            pass


def main():
    ledger = Ledger()
    try:
        os.unlink(SOCKET)
    except FileNotFoundError:
        pass
    os.makedirs(DEBRIEFS, exist_ok=True)
    os.chmod(DEBRIEFS, 0o755)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET)
    os.chmod(SOCKET, 0o666)
    srv.listen(16)
    ledger.publish()
    log("listening on %s: %d sites, %d missions, %d at a time" % (SOCKET, len(SITES["sites"]), SITES["missions"], SITES["open"]))
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
        if len(children) >= int(SITES["open"]):
            pid = os.fork()
            if pid == 0:
                srv.close()
                try:
                    c = Conn(conn)
                    req = c.read(10.0)
                    c.send({"id": (req or {}).get("id"), "error": "busy",
                            "detail": "at most %d training missions at a time" % SITES["open"]}, timeout=5.0)
                except Exception:  # noqa: BLE001
                    pass
                finally:
                    os._exit(0)
            conn.close()
            try:
                os.waitpid(pid, 0)
            except ChildProcessError:
                pass
            continue
        pid = os.fork()
        if pid == 0:
            srv.close()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            try:
                serve(conn, ledger)
            except Exception as e:  # noqa: BLE001 - a mission's failure never takes the service down
                log("mission failed: %r" % e)
            finally:
                os._exit(0)
        children.add(pid)
        conn.close()


if __name__ == "__main__":
    main()
