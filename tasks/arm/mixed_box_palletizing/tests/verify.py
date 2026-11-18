"""The grader of mixed_box_palletizing. Runs as root in the verify stage, in a fresh sandbox
that holds only the submission; the agent is gone.

It owns the simulator (pallet_cell.py, the file the agent's lab ran), the hidden
catalogue of cartons and the hidden orders (hidden/), and drives the submitted controller over
the stdio wire at the agent's UID. The metric is the fraction of the orders that ship complete:
an order counts when every one of its cartons ships (PalletCell.finish). Nothing the controller prints is a
scoring input.

Match lock: at about 1 % of the steps the same step index is asked twice, once with the real
observation and once with a copy whose joint readings (and the pad position that follows from
them) and force reading are slightly perturbed, in a hidden order; only the reply to the real
observation is applied. A pair in which both replies command no motion is not counted. If fewer
than `probe_pass_fraction` of the counted pairs differ, the run scores 0.
"""

import hashlib
import json
import math
import os
import stat
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pallet_cell as pc  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)

HERE = os.path.dirname(os.path.abspath(__file__))


def _action(reply):
    """The controller's reply as an action, or a protocol error that aborts the episode."""
    try:
        values = [float(v) for v in reply]
    except (TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(reply)[:160])
    if len(values) != 7 or not all(math.isfinite(v) for v in values):
        raise SolverAborted("an action is 7 finite numbers")
    cams = int(round(values[5]))
    if cams < 0 or cams > 7 or abs(values[5] - cams) > 1e-6:
        raise SolverAborted("cams is an integer from 0 to 7")
    return values


def _perturbed(obs, rng):
    """A copy of the observation with slightly different joint readings (and the pad pose that
    follows from them) and force reading. Everything else, the images included, is the same."""
    copy = dict(obs)
    q = [v + rng.uniform(-0.001, 0.001) for v in obs["q"]]
    xyz, yaw = pc.fk(q)
    copy["q"] = [round(v, 6) for v in q]
    copy["tcp"] = [round(float(v), 6) for v in (xyz[0], xyz[1], xyz[2], yaw)]
    wrench = list(obs["wrench"])
    for i in range(3):
        wrench[i] = round(wrench[i] + rng.uniform(-0.3, 0.3), 4)
    copy["wrench"] = wrench
    return copy


def oracle_token_ok(workdir):
    """True when the submission's .oracle_token (a regular file, not followed if a link, at most 256
    bytes) hashes to the recorded value. Only then does every observation also carry `truth`."""
    path = os.path.join(workdir, ".oracle_token")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return False
        data = os.read(fd, 257)
    finally:
        os.close(fd)
    if len(data) > 256:
        return False
    with open(os.path.join(HERE, "hidden", "oracle_token.sha256")) as handle:
        want = handle.read().strip()
    return hashlib.sha256(data.strip()).hexdigest() == want


def run(config, rng):
    with open(os.path.join(HERE, "hidden", "catalog.json")) as handle:
        catalog = json.load(handle)
    with open(os.path.join(HERE, "hidden", "episodes.json")) as handle:
        episodes = json.load(handle)
    budget = float(config.get("run_budget_s", 5400))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1))
    cell = pc.PalletCell(catalog)
    order = list(range(len(episodes)))
    for i in range(len(order) - 1, 0, -1):          # hidden order of the episodes
        j = int(rng.random() * (i + 1))
        order[i], order[j] = order[j], order[i]
    scores, volume, notes, aborted = {}, {}, [], 0
    started = time.monotonic()
    solver = Solver(step_timeout=config.get("step_timeout_s", 30))
    truth = oracle_token_ok(solver.submission)
    print("[grader] oracle token: {}".format(truth), file=sys.stderr, flush=True)

    def seen(observation):
        if truth:
            observation["truth"] = cell.truth()
        return observation
    try:
        for index in order:
            episode = episodes[index]
            name = episode["name"]
            if time.monotonic() - started > budget:
                scores[name] = 0.0
                volume[name] = 0.0
                notes.append("{}: not run (the grading run was out of time)".format(name))
                continue
            obs = seen(cell.reset(episode["order"], episode["seed"]))
            try:
                solver.reset()
                while not cell.done:
                    if lock.due():
                        other = _perturbed(obs, rng)
                        if rng.random() < 0.5:
                            probe, real = solver.act(obs["t"], other), solver.act(obs["t"], obs)
                        else:
                            real, probe = solver.act(obs["t"], obs), solver.act(obs["t"], other)
                        action, shadow = _action(real), _action(probe)
                        if any(abs(v) > 1e-9 for v in action[:4] + shadow[:4]):
                            lock.record(shadow[:4], action[:4])
                    else:
                        action = _action(solver.act(obs["t"], obs))
                    obs = seen(cell.step(action))
                    if time.monotonic() - started > budget:
                        raise SolverAborted("the grading run was out of time")
            except SolverAborted as exc:
                aborted += 1
                scores[name] = 0.0
                volume[name] = 0.0
                notes.append("{}: aborted at step {} ({})".format(name, cell.t, str(exc)[:120]))
                continue
            result = cell.finish()
            volume[name] = float(result["shipped_volume_fraction"])
            scores[name] = 1.0 if all(carton["shipped"] for carton in result["cartons"]) else 0.0
            lost = {}
            for carton in result["cartons"]:
                if not carton["shipped"]:
                    reason = carton["reason"].split(" (")[0]
                    lost[reason] = lost.get(reason, 0) + 1
            notes.append("{}: {}, volume {:.3f}, in {} steps{}".format(
                name, "complete" if scores[name] else "incomplete", volume[name], result["steps"],
                "" if not lost else "; not shipped: " + ", ".join("{} x {}".format(n, r) for r, n in sorted(lost.items()))))
            print("[grader] " + notes[-1], file=sys.stderr, flush=True)
    finally:
        solver.close()
        cell.close()
    return scores, volume, notes, aborted, lock, truth


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    try:
        scores, volume, notes, aborted, lock, truth = run(config, rng)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    measured = sum(scores.values()) / len(scores) if scores else 0.0
    verdict = ratio_rewards(
        measured,
        anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=meta.get("direction", "higher") == "higher",
        metric_name="complete_order_fraction",
        locks_ok=lock.ok,
        extra=dict(lock.rewards(), episodes_aborted=aborted, episodes=len(scores), oracle_token=1.0 if truth else 0.0,
                   shipped_volume_fraction=sum(volume.values()) / len(volume) if volume else 0.0),
        full_at=meta.get("full_at", 1.0),
    )
    names = sorted(scores)
    text = "complete orders {:.4f} of {} orders ({}); match lock {} ({} of {} counted pairs differed)\n{}".format(
        measured, len(scores), ", ".join("{} {:.0f}".format(n, scores[n]) for n in names),
        "ok" if lock.ok else "FAILED", lock.passed, lock.total, "\n".join(sorted(notes)))
    write_verdict(verdict, diagnostic=text)
    print("[grader] measured={:.4f} anchor={} reward={:.3f} lock={} aborted={}".format(
        measured, anchor, verdict["rewards"]["reward"], lock.ok, aborted), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
