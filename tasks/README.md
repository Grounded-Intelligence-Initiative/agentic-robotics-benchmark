# The tasks: what the robotics domain adds

This folder is the task collection the upstream ALE engine loads when it runs an Agentic
Robotics Benchmark task. The engine is pinned as the submodule at `../vendor/ale`; nothing here
reimplements it.

The task shape is upstream's. What a task folder contains, what `task.yaml` may declare, what
each stage may do, and the bar a task has to clear are defined in two documents. Read them
first; this file covers only what the robotics domain adds.

- Task authoring: https://github.com/AgentsLastExam/ale/blob/main/docs/specs/task-authoring.md
- Task quality standard: https://github.com/AgentsLastExam/ale/blob/main/docs/specs/task-quality-standard.md

## Layout

```
tasks/<direction>/<slug>/         the tasks (this folder); one task = one self-contained folder
../templates/task/                the one task template (ships with a null anchor, see below)
../identity-history.json          past (spec_hash, content_digest) pairs per task (registry v2)
../docs/evidence/                 real verdicts and audits
```

A task's identity is the manifest's `name`, a collection-unique slug, and the folder is named
after it. `metadata.platform` and `metadata.direction` take the website registry's keys;
`direction` names the folder a task lands in, so a task lives at exactly
`tasks/<metadata.direction>/<name>/` (the domain lint checks this placement). Both
contribution channels put a new task there directly: a pull request opened by hand, or the
website upload whose bot opens the same pull request. There is one kind of task and no genre
field.

The template uses the required stages plus `setup/`. It does not use the upstream `assets/`
directories (per-stage, git-ignored, platform-managed data) or `tools/` (task-owned skills
and MCP servers). Both are available to a task that needs them; the authoring spec says how.

## What the robotics domain adds

### The artifact and the three verify patterns

The agent leaves any artifact in `/home/user/submission/`: a controller process behind
`run.sh`, a plan file or planner program, a URDF or parameter design, generated code, a
report. `verify/` scores it on hidden seeds after the agent is gone. `oracle/` produces the
reference artifact that scores full: a real method run in place of the agent, or a hand-made
file copied in. The verify pattern is a choice inside `verify/verify.py`, not a manifest
field. `instruction.md` still has to describe the grading protocol, because the agent cannot
build the right deliverable otherwise; the anchor is what stays hidden, not the protocol.

| Pattern | The agent produces | verify/verify.py does |
|---|---|---|
| open_loop | answers, plans or designs on disk during the agent phase | reads them and re-simulates or measures under its own dynamics; rejects an answer that does not start where it claims |
| closed_loop | a controller process behind a stdio wire | owns the simulator, steps the controller at the agent's UID by step index `t`, asks probed steps twice with the same `t` (real and perturbed, hidden order; nothing on the wire marks the probe) |
| artifact_rollout | a frozen checkpoint, a planner program, generated code | loads or runs it in a deprivileged subprocess and rolls it out on hidden seeds |

### Metric discipline

The metric is an outcome on a positive scale and hardware-invariant: success rate, reward,
SPL, ATE, collision rate, path cost. Never wall-clock time, throughput, FPS or frequency;
those measure the machine. A negative reward becomes a positive cost with
`direction: lower`. The kit refuses a non-positive anchor because it would invert the
ratio, so a better result would score lower.

### Scoring

```
ratio  = clamp(measured / anchor, 0, cap)        # inverted for lower-is-better metrics; cap = 1.5
reward = clamp(ratio / full_at, 0, 1)            # the single gated reward key
```

The anchor is the reference implementation's own measured value from a real `ale validate`
run. Never a published figure: that came from other hardware, other seeds, and usually a
kinder evaluation. `full_at` is the ratio at which the reward saturates to 1.0: the worst
ratio an honest on-reference run lands on under fresh hidden seeds. Size it from the spread
of several oracle runs; the template's numbers are in `templates/README.md`.

`rewards` carries only `reward`. The raw metric, the capped `ratio` (the leaderboard number,
where "matched the reference" and "beat it" stay distinguishable), lock outcomes and counts
ride in the verdict's `metrics`, which the engine records and never gates on.

`ale validate` runs the task twice. An untouched sandbox must produce a non-empty all-zero
reward map, and the oracle must score exactly one on every key. Either miss fails validation
(`untouched_nonzero`, `oracle_not_full`; exit 2; the stdout line says `FAIL`, the run's
`validation.json` names the code). An anchor starts null: the first run measures, reports
the value in the oracle record's `metrics`, scores 0, and fails with `oracle_not_full`.
Record the value and run again. `harness/ale_onboard.py` reads the
records and reports that state as `needs_anchor`, not as an error.

### The vendored kit

`../shared/robotics_grader/` is the shared verify-stage machinery: the solver wire and
agent-UID spawn (`Solver`, `deprivileged`), hidden seeds (`seed_rng`), the probe lock
(`ProbeLock`), ratio scoring (`ratio_rewards`), the honest zero (`zero_verdict`) and verdict
writing (`write_verdict`, which records through the engine's `ale_verify`). Upstream has no
repository kit mechanism, so every task carries a byte-identical copy at
`verify/robotics_grader/`. CI diffs each copy against the canonical one and fails the pull
request on a difference; nothing rewrites a task after it lands, so the copy a maintainer
validated is the copy that scores.

### The probe wire

`Solver` speaks one JSON object per line to the agent's `run.sh`. Every `act` request
carries the step index `t`, owned by the grader and restarting at 0 after each `reset`.
`ProbeLock` asks a probed step twice with the same `t`, once with the real observation and
once with a perturbed copy, in a hidden order, and applies only the real reply. Nothing on
the wire marks a probe. The instruction tells the agent that a step may be queried more than
once and that repeated queries do not advance the episode, so an honest solver needs no
special case and a dishonest one has nothing to key on.

### setup/payload

`setup/run.sh` stages everything under `setup/payload/` into the agent home without naming
any file. The practice grader there speaks the same wire on visible seeds and prints the raw
metric only: never a score, a ratio or the anchor, and it never imports `verify/`.

### Isolation

What keeps answers away from the agent is timing. A stage's folder reaches the sandbox when
that stage runs, so `verify/` and `oracle/` are absent while the agent works. On top of that
`/opt/ale` is `drwx------ root:root` and the solver runs at the agent's UID. Real runs:
`../docs/evidence/isolation-audit.md`.

### Images

Every task owns its image. Two idioms are load-bearing: the py3.12 `/opt/venv` (the engine
stages `ale_verify` into whatever answers `python3` and refuses anything older) and the
agent-UID import self-check at the end of the Dockerfile (a venv the agent account cannot
traverse fails only at solve time, as a misleading `ModuleNotFoundError`). A prebuilt stack
can declare `image.ref` instead of a Dockerfile.

## Building a task

```bash
# instantiate + gate in one step (the spec fills metadata; you write the physics)
uv run python -m harness.ale_onboard <spec>.yaml --json

# or gate a folder you already wrote
uv run python -m harness.ale_onboard --gate-only tasks/<direction>/<task> --json

# the domain lint (metric discipline, anchor provenance, visible surface, image hygiene, placement)
uv run python scripts/lint_domain.py .

# the kit's self-test; no docker needed
cd shared && python3 -m robotics_grader.selftest
```

Outcomes: `built` (untouched all-zero and oracle all-one) · `needs_anchor` (it ran; here is
the measured value) · `needs_input` (lint failed, no verdict, the untouched pass did not
score an honest zero, or the oracle cannot reach 1.0) · `gave_up` (this cannot be scored
fairly) · `error`.

An agent goes through `skills/onboard-task/SKILL.md`, which routes to a pattern skill and
owns the give-up rules.

## Verified state

| What | Evidence |
|---|---|
| `templates/task` on engine `90a7c1c` (2026-09-05) | as committed (null anchor): `FAIL  template_reach: untouched={'reward': 0.0}, oracle={'reward': 0.0}`, exit 2, `validation.json` failures `[oracle_not_full]`; the oracle record carries `mean_goal_distance` in `metrics`. With that value recorded: `ok    template_reach: untouched={'reward': 0.0}, oracle={'reward': 1.0}` (`../docs/evidence/template-validate-{oracle,untouched}-result.json`) |
| a constant-action solver speaking the wire perfectly | `match_lock_ok` 0.0, reward 0.0; validate fails the oracle pass |
| an untouched sandbox | an honest all-zero verdict (`zero_verdict`), not a task error |
| `tasks/control/drone_hover` on engine `a0d3534` | `ok  untouched={'reward': 0.0}, oracle={'reward': 1.0}`; the oracle trains PPO, mean_episode_reward vs anchor 472.07 (`../docs/evidence/drone_hover-validate-{oracle,untouched}-result.json`) |
| the agent cannot reach `/opt/ale`, the kit, or the verdict | `../docs/evidence/isolation-audit.md` |
