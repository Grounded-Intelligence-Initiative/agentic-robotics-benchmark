"""battery_pack_disassembly_to_modules: the grader's episode loop over the closed_loop wire (used by verify.py).

One hidden pack is one episode. For each pack a fresh cell process (`packcell/server.py`, Isaac Sim) is started, the
controller is reset, and the loop asks the controller for one action per step and hands it to the cell until the
pack ends: the controller says `done`, the cell's simulated time is used up, a tool touches a live busbar before
isolation, a module falls, or the controller is given up on (a missed deadline, a closed wire, a malformed action,
its time budget used up). In every case the pack's credit is what lies where it belongs at that moment
(`rules.Rules.credit`, read from the physics engine's state); nothing is subtracted.

- The probe lock (`IdleAwareProbeLock`): a step is asked twice with the same `t`, the true and a perturbed
  observation (`wire.perturbed`: the joints shifted, the poses those of the shifted joints, built as the cell builds
  an observation and from the cell's own angles before rounding) in a hidden order; only the true reply is applied.
  Which steps, the order and the shift come from one stream per pack, a function of the pack's index and seed alone.
  `wire.residual` of both observations of every probed step is kept with the pack's record, and how often the true
  one's is the larger: a copy must not differ from a true observation in it.
- `truth` (the cell's privileged view) is added to every observation only in a token run (the caller decides).
- The budget (`controller_budget_s`, per pack) is the controller's own time, kept by a `Clock`: what the grader waits
  for each reset and each reply. The cell's simulation and rendering are not charged.
- A failure of the cell itself (`cellproc.CellError`): the controller is stopped with its whole process group (and,
  as root, every process of its account) before anything else, so that nothing of it runs while a fresh cell starts
  and builds the pack, and the pack is run once more from the start with that cell and a restarted controller (the
  restart is the grader's, not one of the controller's `max_restarts`). A second failure on the same pack is the
  controller's when it can have caused it: when it happened after the controller was started again (its GPU memory,
  its memory, its disk or its load beside the cell), when the controller could not be stopped completely, or when
  the caller's `charge_files()` finds that its files can have filled a disk the cell writes to. That pack then
  counts 0 (`ended` "cell_failed") and grading goes on with the next one. Any other second failure propagates (a
  task error: no submission can have caused it).
"""

import hashlib
import time

from packcell import cellproc, wire
from packcell import series as S
from robotics_grader import ProbeLock, SolverAborted, seed_rng

PROBE_TAG = b"battery_pack_disassembly_to_modules verify probe schedule 1"
CONFIG = {"probe_rate": 0.02, "probe_pass_fraction": 0.1, "step_timeout_s": 30, "controller_budget_s": 1200.0,
          "max_steps": 20000, "cell": {}, "cell_start_timeout_s": 900.0, "cell_reply_timeout_s": 600.0}


class Clock(object):
    """The controller's own time (s): what the grader waited for the controller."""

    def __init__(self):
        self.s = 0.0

    def timed(self, fn, *args, **kwargs):
        t0 = time.monotonic()
        try:
            return fn(*args, **kwargs)
        finally:
            self.s += time.monotonic() - t0


class IdleAwareProbeLock(ProbeLock):
    """The kit's ProbeLock on the cell's action (compared as `wire.canonical`'s 15 numbers), except that a pair in
    which both replies command no motion is not counted: a controller that holds still while the spindle turns, a
    tool is exchanged or the pack discharges shows nothing either way.

    Which controllers fail it (fewer than `pass_fraction` of the counted pairs differ): any whose motions do not
    depend on the robot's own readings (the joint angles and the tool poses that follow from them), the parts the
    perturbed copy shifts. A replayed action stream does; a controller whose every displacement is computed from
    the observed tool pose or joint angles toward a target passes."""

    def __init__(self, *args, **kwargs):
        ProbeLock.__init__(self, *args, **kwargs)
        self.idle_skipped = 0

    def record(self, action_probe, action_real):
        if wire.is_idle(action_probe) and wire.is_idle(action_real):
            self.idle_skipped += 1
            return False
        return ProbeLock.record(self, action_probe, action_real)

    def rewards(self):
        out = ProbeLock.rewards(self)
        out["probes_idle_skipped"] = float(self.idle_skipped)
        return out


def canonical(action):
    try:
        return wire.canonical(action)
    except ValueError as exc:
        raise SolverAborted("malformed action: {}".format(exc))


def probe_stream(index, seed):
    digest = hashlib.blake2b(b"%s|%d|%d" % (PROBE_TAG, int(index), int(seed)), digest_size=32).digest()
    return seed_rng(entropy=int.from_bytes(digest, "little"))


def run_pack(solver, lock, clock, index, entry, truth, config, start_cell, log):
    """One pack. Returns its record: credit, why it ended, steps, the cell's own counts."""
    pack = S.draw_pack(entry["seed"], variant=entry.get("variant"), harness=entry.get("harness"), cameras=entry.get("cameras"),
                       sealant_seed=entry.get("sealant_seed"), appearance_seed=entry.get("appearance_seed"))
    rec = {"index": index, "variant": pack["variant"], "credit": 0.0, "ended": None, "aborted": None, "steps": 0,
           "probe_residual_true": 0.0, "probe_residual_copy": 0.0, "probe_copies": 0, "probe_true_residual_larger": 0}
    lock.rng = probe_stream(index, entry["seed"])
    budget = float(config["controller_budget_s"])
    used0 = clock.s
    ran = False                          # whether the controller was (re)started beside this cell
    try:
        cell = start_cell()
    except cellproc.CellError as exc:
        exc.controller_ran = ran
        raise
    t0 = time.monotonic()
    try:
        reply = cell.request({"cmd": "reset", "pack": pack, "config": config.get("cell") or {}},
                             timeout=float(config["cell_start_timeout_s"]))
        obs = reply["obs"]
        t = 0
        try:
            ran = True
            clock.timed(solver.reset)
            while not obs["ended"]:
                if clock.s - used0 > budget:
                    raise SolverAborted("controller time budget of the pack used up")
                if t >= int(config["max_steps"]):
                    raise SolverAborted("step limit of the pack reached")
                if truth:
                    obs["truth"] = cell.request({"cmd": "truth"}, timeout=60.0)["truth"]
                if lock.due():
                    exact = cell.request({"cmd": "joints"}, timeout=60.0)["joints"]
                    other = wire.perturbed(obs, lock.rng, exact)
                    r_true, r_copy = wire.residual(obs), wire.residual(other)
                    rec["probe_residual_true"] = max(rec["probe_residual_true"], r_true)
                    rec["probe_residual_copy"] = max(rec["probe_residual_copy"], r_copy)
                    rec["probe_copies"] += 1
                    rec["probe_true_residual_larger"] += int(r_true > r_copy)
                    action = clock.timed(lock.probe, solver, t, obs, perturbed=other)
                else:
                    action = clock.timed(solver.act, t, obs)
                canonical(action)
                reply = cell.request({"cmd": "step", "action": action}, timeout=float(config["cell_reply_timeout_s"]))
                if "error" in reply:
                    raise SolverAborted(str(reply["error"])[:200])
                obs = reply["obs"]
                t += 1
        except SolverAborted as exc:
            rec["aborted"] = str(exc)[:200]
        score = cell.request({"cmd": "score", "final": True}, timeout=120.0)["score"]
        rec.update(credit=float(score["credit"]), ended=score["ended"], steps=t, sim_s=score["time"],
                   frames=score["frames"], protective_stops=score["protective_stops"], stripped=len(score["stripped"]),
                   placed=sum(1 for v in score["detail"].values() if v), items=len(score["detail"]),
                   detail=score["detail"], events=score["events"], cell_wall=score["wall"])
    except cellproc.CellError as exc:
        exc.controller_ran = ran
        raise
    finally:
        cell.close()
    rec.update(wall=round(time.monotonic() - t0, 1), controller_s=round(clock.s - used0, 1))
    if log:
        # the variant only: the log is kept with a run's records, and a seed of the fixed set must not be in it
        log("pack {} ({}): credit={:.4f} ended={} steps={} sim={:.0f}s stops={} placed={}/{} wall={:.0f}s{}".format(
            index, rec["variant"], rec["credit"], rec["ended"], rec["steps"], rec.get("sim_s", 0.0),
            rec.get("protective_stops", 0), rec.get("placed", 0), rec.get("items", 0), rec["wall"],
            " ABORTED: " + rec["aborted"] if rec["aborted"] else ""))
    return rec


def stop_for_cell(solver, stop_account=None):
    """Stop the controller with its whole process group (and, with `stop_account`, every process of its account)
    before a fresh cell starts, and have the next reset start it again without counting that restart against its
    `max_restarts`. True when nothing of it is left running."""
    if not solver._broken and not solver._dead:
        solver.max_restarts += 1          # the grader's restart, not one of the controller's
    solver._broken = True
    solver._stop(grace=0)
    return stop_account() if stop_account is not None else True


def run_packs(solver, packs, truth, config, start_cell, log=None, clock=None, on_pack=None, stop_account=None,
              charge_files=None, progress=None):
    """Every pack of the set in order over one controller. Returns (records, lock). `progress` (a dict), when given,
    holds the lock and the records of the packs graded so far while this runs (a deadline reads it)."""
    cfg = dict(CONFIG)
    cfg.update({k: v for k, v in (config or {}).items() if k in CONFIG})
    clock = Clock() if clock is None else clock
    lock = IdleAwareProbeLock(None, rate=cfg["probe_rate"], pass_fraction=cfg["probe_pass_fraction"], tolerance=1e-4,
                              canonical=canonical)
    records = []
    if progress is not None:
        progress.update(lock=lock, records=records)
    for index, entry in enumerate(packs):
        if on_pack is not None:
            on_pack(index)
        try:
            rec = run_pack(solver, lock, clock, index, entry, truth, cfg, start_cell, log)
        except cellproc.CellError as exc:
            stopped = stop_for_cell(solver, stop_account)
            if log:
                log("pack {}: the cell failed ({}); the controller is stopped{} and the pack runs once more".format(
                    index, str(exc)[:300], "" if stopped else " (not completely)"))
            try:
                rec = run_pack(solver, lock, clock, index, entry, truth, cfg, start_cell, log)
            except cellproc.CellError as again:
                cause = None
                if getattr(again, "controller_ran", True):
                    cause = "the cell failed again while the controller ran beside it"
                elif not stopped:
                    cause = "the cell failed again; the controller's processes could not all be stopped"
                elif charge_files is not None:
                    cause = charge_files()
                if cause is None:
                    raise                      # nothing of the submission ran: the grading environment's failure
                stop_for_cell(solver, stop_account)
                if log:
                    log("pack {} ({}): credit=0.0000 ended=cell_failed: {} ({})".format(
                        index, entry.get("variant"), cause, str(again)[:300]))
                rec = {"index": index, "variant": entry.get("variant"), "credit": 0.0, "ended": "cell_failed",
                       "aborted": cause[:200], "steps": 0, "cell_failed": str(again)[:300]}
            rec["cell_retried"] = True
        records.append(rec)
    return records, lock
