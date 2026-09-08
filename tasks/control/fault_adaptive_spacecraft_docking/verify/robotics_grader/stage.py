"""Verify-stage basics: where things are, who runs them, and how a run ends.

The framework tells a stage script three things through the environment, and this module
is the only place that reads them, so a task never hardcodes a path the framework owns.
"""

import json
import os
import subprocess
import sys

__all__ = [
    "agent_user",
    "deprivileged",
    "fail",
    "require",
    "stage_dir",
    "verdict_path",
    "write_verdict",
]


def stage_dir():
    """This stage's folder inside the sandbox (root-only, under the framework's /opt/ale).

    Assets that must not exist while the agent works — the anchor, grader config, hidden
    reference data — live here, because a stage folder is copied in when its stage runs.
    """
    return os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(sys.argv[0])))


def verdict_path():
    """Where rewards are written. The agent's UID cannot pre-create or overwrite it."""
    return os.environ["ALE_VERDICT_PATH"]


def agent_home():
    """The unprivileged account's home — the workspace, per the image's declared user."""
    return os.environ.get("ALE_HOME", "/home/user")


def agent_user():
    """The account the agent ran as, derived from its home.

    The image declares it (`ale.user`) and the framework passes the home; taking the leaf
    of the home keeps the two consistent without a second source of truth.
    """
    return os.path.basename(agent_home().rstrip("/")) or "user"


def fail(reason):
    """End the run as a TASK ERROR, not as a zero.

    The distinction is load-bearing: a zero says the agent did not solve the task, and a
    task error says the task is broken. Conflating them lets a defective grader read as a
    stream of legitimately failing agents.
    """
    print("[grader] TASK ERROR: {}".format(reason), file=sys.stderr, flush=True)
    raise SystemExit(1)


def require(condition, reason):
    """`fail` unless the condition holds — for the grader's own preconditions."""
    if not condition:
        fail(reason)


def write_verdict(verdict, path=None):
    """Write the verdict envelope the framework reads back.

    `verdict` is `{"rewards": {...}, "metrics": {...}}`. The split is load-bearing under
    `ale validate`: every key in `rewards` must be exactly 0 on an untouched sandbox
    (the engine fails the run otherwise) and exactly 1 for the oracle (the engine only
    records a non-1.0 oracle as a `partial_oracle` warning; the exact all-one rule is
    ALE Robotics domain policy, enforced by the maintainers' onboarding tooling), so
    `rewards`
    carries only the gated score —
    normally the single key `reward`, in [0, 1] — and everything worth keeping but not
    gating (the raw metric, the capped ratio, lock outcomes, abort counts) goes in
    `metrics`, which the engine persists to the run record and never judges.

    When the engine's own `ale_verify` package is present (it stages it into the sandbox's
    `python3` at verify time), the same envelope is also recorded through it, because the
    engine cross-checks the reward file against the verification record and rejects a
    mismatch. Outside a sandbox — the kit's selftest, a practice run — the plain JSON
    envelope is enough.

    `ALE_VERIFICATION_PATH` set without an importable `ale_verify` is a task error (the
    image's python is too old, or the stage runs outside the engine) — unless `ALE_DRYRUN=1`,
    the local dry-run's marker, in which case a plain JSON verification record
    (`{"status": "completed", "criteria": [...], "metrics": {...}, "dryrun": true}`) is
    written there instead and the verdict envelope is written as usual.
    """
    rewards = verdict.get("rewards") or {}
    metrics = verdict.get("metrics") or {}
    require("reward" in rewards, "rewards map has no primary 'reward' key")
    require(all(0.0 <= float(v) <= 1.0 for v in rewards.values()),
            "every gated reward must lie in [0, 1]; report raw values under metrics")

    envelope = {"rewards": {k: float(v) for k, v in rewards.items()},
                "metrics": {k: float(v) for k, v in metrics.items()}}

    verification_path = os.environ.get("ALE_VERIFICATION_PATH")
    if verification_path:
        try:
            from ale_verify import CheckResult, Verification
        except ImportError:
            if os.environ.get("ALE_DRYRUN") != "1":
                fail("ALE_VERIFICATION_PATH is set but ale_verify is not importable — the "
                     "image's python3 is older than the engine's 3.12 floor, or the stage "
                     "is running outside the engine (set ALE_DRYRUN=1 for a local dry-run)")
            # The local dry-run (`_tools/dryrun.py` in the template download): the same
            # grader code runs on a contributor's machine without the engine, so there is
            # no `ale_verify` to record through. Write a plain verification record in the
            # engine's shape — one `check` criterion per reward key, the metrics as stats —
            # marked `dryrun`, and the verdict envelope beside it exactly as the engine's
            # `Verification.write()` would. A maintainer's engine run is still the gate.
            record = {
                "status": "completed",
                "criteria": [{"name": name, "score": envelope["rewards"][name], "source": "check"}
                             for name in sorted(envelope["rewards"])],
                "metrics": envelope["metrics"],
                "dryrun": True,
            }
            with open(verification_path, "w") as handle:
                json.dump(record, handle)
            with open(path or verdict_path(), "w") as handle:
                json.dump(envelope, handle)
            return
        record = Verification()
        for name, value in sorted(rewards.items()):
            record.check(name, CheckResult(float(value), "computed by the task's grader"))
        for name, value in sorted(metrics.items()):
            record.stat(name, float(value))
        record.write()   # writes ALE_VERIFICATION_PATH and ALE_VERDICT_PATH atomically
        return

    with open(path or verdict_path(), "w") as handle:
        json.dump(envelope, handle)


def deprivileged(argv, **kwargs):
    """Run `argv` as the agent account instead of as root.

    Two callers need this and both are security-critical:

    * a live solver the agent wrote (closed_loop) — at root it could read the grader's
      memory, environment and `/opt/ale`;
    * deserializing an artifact the agent produced (artifact_rollout) — `torch.load` and
      `pickle` execute arbitrary code, so doing it as root hands over the grader.

    Yama's default `ptrace_scope` blocks tracing across UIDs, so the boundary holds even
    though both processes share a sandbox.

    Outside the engine — a local dry-run on a contributor's machine — the grader is not
    root and there is no second account. The UID boundary exists in the engine's sandbox;
    a non-root process has nothing to drop, and `runuser` would only fail. So when the
    effective UID is not 0 the command runs directly as the current user. The root path
    (the engine, and the dry-run's `--docker` mode) is unchanged.
    """
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        return subprocess.Popen(list(argv), **kwargs)  # noqa: S603 — caller-fixed argv
    return subprocess.Popen(  # noqa: S603 — caller-fixed argv, no shell
        ["runuser", "-u", agent_user(), "--"] + list(argv), **kwargs
    )
