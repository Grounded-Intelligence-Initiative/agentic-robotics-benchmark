"""The grader. Runs as root in the verify stage, in a sandbox the agent never touched.

It owns the simulator (cell_server.py under the simulator's interpreter) and drives the agent's controller over the
kit's stdio wire, one hidden shift per simulator process. The metric, part_pickup_score, comes from the simulator's own
state; nothing the controller prints is a scoring input.

Wire, per step t: obs = {"t", "proprio"[61], "odometry"[3], "cameras": {name: {rgb_png, depth_mm_u16, shape, K, pos_base,
rot_base}}}; reply = {"action": [26 numbers]}: the robot's 23, then one flag per camera for the next observation. Every frame asked for costs
a fixed 0.1 s of simulated time inside the cell.
"""
import json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from robotics_grader import (ProbeLock, Solver, SolverAborted, SolverMissing, ratio_rewards, read_anchor,  # noqa: E402
                             seed_rng, stage_config, write_verdict, zero_verdict)
from robotics_grader.stage import agent_home  # noqa: E402
from sim_client import SimClient, SimError  # noqa: E402

CAMERAS = ("head", "left_wrist", "right_wrist")
STATION = "drop"


def parse_reply(action):
    """-> (command[23], cameras) or SolverAborted. The kit hands over the reply's "action": 26 numbers, the robot's 23
    followed by one flag per camera (head, left_wrist, right_wrist; above 0.5 = send that frame with the next observation).
    A malformed reply aborts the shift; it must never crash the grader."""
    try: a = [float(v) for v in action]
    except (TypeError, ValueError, OverflowError): raise SolverAborted("malformed action")
    if len(a) != 26 or not all(math.isfinite(v) for v in a): raise SolverAborted("action must be 26 finite numbers")
    return a[:23], [c for c, f in zip(CAMERAS, a[23:]) if f > 0.5]


def oracle_token():
    """The reference leaves its token in its submission; nothing an agent can write there has the recorded hash."""
    try: return open(os.path.join(agent_home(), "submission", ".oracle_token")).read().strip()
    except OSError: return ""


def perturbed(obs, lock):
    """Probe copy: the joint readings shifted a little. A controller that ignores its observation answers both alike."""
    p = json.loads(json.dumps(obs)); vals = p["proprio"]; p["proprio"] = lock.perturb(vals); return p


class CpuMeter(object):
    """CPU seconds used so far by the controller: every process of its process group (the kit starts it in a session
    of its own). A reply's deadline is counted in the controller's CPU time, not in wall time: on a loaded grading
    machine a reply can wait for a core through no fault of the controller, and the same controller must not pass on
    one machine and fail on the other because of the other tenants."""
    TICK = float(os.sysconf("SC_CLK_TCK"))

    def __init__(self, pgid): self.pgid, self.pids, self.n = int(pgid), [], 0

    def _stat(self, pid):
        try:
            f = open("/proc/{}/stat".format(pid)).read(); x = f[f.rindex(")") + 2:].split()
            return int(x[2]), (int(x[11]) + int(x[12])) / self.TICK
        except (OSError, ValueError, IndexError): return None, 0.0

    def seconds(self):
        if self.n % 40 == 0: self.pids = [d for d in os.listdir("/proc") if d.isdigit() and self._stat(d)[0] == self.pgid]
        self.n += 1
        return sum(self._stat(p)[1] for p in self.pids)


def run_shift(sim, solver, lock, config, seed):
    """One shift. The cell does not wait for the controller: the time a reply takes (the controller's CPU time) is time in the cell. A reply that
    arrives within one control period (1/30 s) is applied on the next step; every further period it took is one more step
    the cell runs first, the robot holding its previous command, the part moving. Time is never scored: it only decides
    what state of the cell an action meets."""
    parts = int(config.get("events_per_shift", 20)); horizon = int(config.get("max_steps_per_shift", 30000))
    slack = float(config.get("program_slack_s", 60.0)); pace = float(config.get("program_time_per_sim_time", 1.0))
    token = oracle_token()
    obs = sim.reset(STATION, seed, parts, cameras=CAMERAS, token=token)      # the reference's token also gives it assisted grasping
    if token:
        try: obs["truth"] = sim.truth(token); obs["truth"]["proprio"] = list(obs["proprio"])
        except SimError: token = ""
    solver.reset(); t = 0; program = 0.0; t0 = int(obs["t"]); behind = False
    meter = CpuMeter(os.getpgid(solver.proc.pid)); cpu0 = meter.seconds(); cap = float(config.get("reply_wall_cap_s", 2.0))
    while not obs["done"] and t < horizon:
        probed = lock.due(); tic = time.monotonic()
        reply = lock.probe(solver, t, obs, perturbed(obs, lock)) if probed else solver.act(t, obs)
        wall = time.monotonic() - tic; cpu1 = meter.seconds()
        # What a reply costs is the controller's own CPU time (see CpuMeter): the same controller then meets the same cell
        # on a busy grading host and on an idle one. A reply that takes more than `cap` of wall time counts in full.
        took = (wall if wall > cap else (cpu1 - cpu0)) / (2.0 if probed else 1.0); cpu0 = cpu1          # a probed step is asked twice
        program += took; late = int(took * 30.0)
        action, cams = parse_reply(reply)
        obs = sim.step(action, cams, late=late); t += 1
        if token:                                  # the privileged reference: truth only for the holder of the token
            try: obs["truth"] = sim.truth(token); obs["truth"]["proprio"] = list(obs["proprio"])   # the true joint readings: a probe shifts only obs["proprio"]
            except SimError: token = ""            # wrong token: no truth, no error
    s = sim.summary(); s["program_time_s"] = round(program, 2); s["simulated_time_s"] = round((int(obs["t"]) - t0) / 30.0, 2); s["fell_behind"] = bool(behind)
    return s


def main():
    config = stage_config(); rng = seed_rng(); anchor, meta = read_anchor()
    shifts = json.load(open(os.path.join(HERE, "hidden", "shifts.json")))["seeds"][: int(config.get("shifts", 6))]
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1), dims=61)
    scores, aborted, details = [], 0, []
    try:
        solver = Solver(step_timeout=config.get("step_timeout_s", 30))
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None)); return
    try:
        for k, seed in enumerate(shifts):
            env = dict(os.environ, CELL_TOKEN_SHA256=open(os.path.join(HERE, "oracle_token.sha256")).read().strip())
            sim = SimClient(os.path.join(HERE, "cell_server.py"), log_path="/tmp/cell-sim-{}.log".format(k), env=env)
            try:
                s = run_shift(sim, solver, lock, config, seed); scores.append(float(s["score"]))
                details.append({k2: s[k2] for k2 in ("score", "placed", "held_only", "bystanders_moved", "score_first_third", "score_last_third", "steps", "frames_charged", "program_time_s", "simulated_time_s", "fell_behind")})
            except SolverAborted as exc:      # the controller's fault: the shift scores what was earned so far, i.e. nothing more
                aborted += 1; scores.append(0.0); details.append({"aborted": str(exc)[:160]})
            finally:
                sim.close()
            print("[grader] shift {}: {}".format(k, details[-1]), file=sys.stderr, flush=True)
    except SimError as exc:
        raise SystemExit("task error: {}".format(exc))      # nonzero exit: no score, which is not a zero
    finally:
        solver.close()
    measured = sum(scores) / len(scores) if scores else 0.0
    flat = {"shift{}_{}".format(k, key): float(d[key]) for k, d in enumerate(details) for key in ("score", "placed", "score_first_third", "score_last_third", "program_time_s", "simulated_time_s", "fell_behind") if key in d}   # metrics are numbers only
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True, metric_name=meta.get("metric", "part_pickup_score"),
                            locks_ok=lock.ok, extra=dict(lock.rewards(), shifts_aborted=aborted, **flat), full_at=meta.get("full_at", 1.0))
    verdict["diagnostic"] = json.dumps({"shifts": details})[:4000]
    write_verdict(verdict)


if __name__ == "__main__":
    main()
