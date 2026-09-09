# Maintainer notes: ball_physics_stepper

This is a small CPU-only **simulator implementation** task, intended as an onboarding
and integration candidate. The agent implements physical transitions, not a controller.
It has not yet been shown to challenge a frontier model or approved for benchmark admission.

## Artifact and physics

`/home/user/submission/run.sh` starts a persistent JSON-lines process. A `reset` supplies
gravity, mass, radius, restitution, optional infinite horizontal plate height, and initial
position/velocity. Each `step` supplies an impulse and duration and must return position,
velocity and cumulative time. A `close` acknowledges and exits. The complete contract,
including units, contact behavior and tolerances, is in `instruction.md`.

The task excludes finite paddles, rotation, friction, drag and multiple interacting balls.
It requires no robot hardware, downloaded data, model credentials, GPU, or network.

## Verification boundary

- Setup contains a protocol skeleton and ten hand-worked public checks, not the oracle.
- The agent sees setup material and the task instruction. The engine stages the verifier
  only after agent execution and artifact collection.
- The canonical `robotics_grader.Solver` runs the submitted executable as the agent UID.
  The grader owns the hidden inputs, independent reference states, and final score.
- This is a custom **stateful reset/step** protocol. It uses the canonical `request()`
  transport, not its controller-oriented `act()` or ProbeLock: repeating a `step` advances
  state and must not be treated as a stateless controller query.
- A separate 50-digit Decimal implementation supplies expected states. It does not import
  the submitted artifact or the float oracle. The implementations use different arithmetic
  and different searches for large bounce counts, and are checked against hand examples.
- The hidden suite uses fresh seeds: eight parameter/trajectory families, eight cases per
  family, forty steps per case, plus six edge cases. All 2,650 returned states are checked.
- A trajectory passes only if every reset/step state is within the published tolerance.
  Wrong trajectories stay in the denominator. Invalid numbers, protocol failures, crashes,
  timeouts, or an invalid close yield zero. Huge errors are only capped in diagnostics.
- `trajectory_pass_rate` is normalized against the measured anchor 1.0. No wall-clock
  performance score is used. Process timeouts are safeguards, not a throughput metric.

## Validation evidence (2026-09-09 UTC)

Pinned ALE engine: `90a7c1cfa649c5dbf1078714773f99840e44cba3`.
Three final-code Docker validations: `validate-9ef472f0`, `validate-979c31f1`,
`validate-84f69359`. Each returned untouched=0, oracle=1, with 70/70 complete trajectories
and 2,650/2,650 state responses passing. Thus `full_at=1.0`; no tolerance relaxation
was selected to hide reference failures. Largest absolute state error across these runs
was 7.28e-11, well below the published per-scalar tolerance.

Real ALE negative controls (same grader and anchor, deliberately broken oracle fixtures):

| Artifact | Reward | Expected interpretation |
| --- | ---: | --- |
| Missing submission | 0 | No free baseline score |
| Constant zero states | 0 | Cannot replay a canned answer |
| NaN state | 0 | Invalid numeric output is rejected |
| Correct free flight, but ignores plate | 17/70 | Partial credit only; all contact families fail |
| Correct states, invalid close | 0 | Lifecycle contract is checked |

The normal `ale validate` command exits nonzero for those deliberately broken oracle
fixtures, as expected; their verifier episodes complete with the intended verdict.
Additional local tests cover seven physics mutants, reset isolation, subdivided time
steps, inelastic rest/liftoff, zero gravity, and very large bounce counts.

## Reproduce

From the benchmark repository root:

```bash
python3 tasks/simulation/ball_physics_stepper/verify/test_simulator.py
python3 scripts/lint_domain.py .
```

From `vendor/ale`:

```bash
VIRTUAL_ENV= uv run --frozen ale lint ../../tasks/simulation/ball_physics_stepper
VIRTUAL_ENV= uv run --frozen ale validate ../../tasks/simulation/ball_physics_stepper --runs-dir ../../outputs/ball-physics-check
```

The task carries an unchanged copy of `shared/robotics_grader/`. Its image follows the
official template and ALE base image; the runtime numerical dependency is pinned in
`image/requirements.txt`. New task-specific code is contributed under the repository's
license, with no third-party assets bundled. Build the official base image first on a
fresh host as described in the repository contribution guide.

**Registry review:** `platform=manipulation` is the nearest existing platform;
`direction=simulation` is proposed for simulator construction. Domain lint accepts the
placement, but the website taxonomy has not been confirmed. A maintainer must confirm
this key or choose an appropriate registered direction before listing the task.
