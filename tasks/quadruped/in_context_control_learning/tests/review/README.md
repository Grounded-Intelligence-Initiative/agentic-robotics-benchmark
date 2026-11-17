# v4 Review Records

The current task is the force-driven precision-docking upgrade integrated with
maintainer round2 commit `8b7195d`. This is not round2's world version 3.

## Frozen Inputs

- Round2's 24 hidden seeds, all 40 candidates and their order are unchanged.
  `check_task.py` compares their bytes with the archived maintainer files.
- Budgets: 500 presses, 120 separately requested camera observations, 660 total
  service requests and six hours. Act/advance return status only.
- Runtime, verifier and solution/private copies of all six modules must match.
- The participant service has no unlock/token/truth operation. The privileged
  oracle uses only engine-staged private metadata and ordinary counted actions.
- The current controller uses native MuJoCo force integration with lag, traction,
  slip, load-dependent mass/inertia and sequential stable pose docking. It is
  a stabilized planar Go1-shaped base, not an articulated gait simulation.

## Current Evidence

`v4/` holds the completed 24/24 current cohort and live first-seed pilot receipts.
All seeds reached 6/6 with exact replay, zero contacts and one camera observation;
167-246 presses (median 200.5), at most 252 requests, 351.2 seconds total on CPU.
The live pilot scored 1.0 in 25.9 seconds, with 214 presses and 220 requests.
Eight native rendered replay views passed exact state checks. The remote evidence directory is
`koa04:/home/wang100/projects/ale-validator-hardening-20261116/evidence/icl-worker-v4-20261117T0108`.
The official CPU image is `ale-icl-maintainer-env:20261116`,
`sha256:33860ab2d415874bfd4205d7133a070805b26c19321595f1dbd7a4591184c31e`.
The initial-image manifest covers every fixed seed without success filtering.

`check_task.py` covers dynamics, precision, hysteresis, load, camera budgeting,
no public truth, all-three-copy parity, fail-closed audit/replay, and cohort
continuation after grader `SystemExit`. Its current native CPU run passed
13 dynamics/contracts + 17 security + 3 author-runner tests.

The author feasibility cohort takes approximately six minutes on two CPU cores.
Initial-image generation takes about one minute; the live first-seed oracle
pilot is separate. These are not frontier-model results.

An independent, actual pinned-engine `ale validate` also passed on v4:
untouched reward 0, privileged oracle reward 1. All 47 active source/data files,
parsed task settings and score normalization match the validated snapshot.
`v4/ale_validation.json` records the engine commit, source hashes, full lifecycle
outcomes, verdicts and reproduction command. Native rendered/headless replay
also matched exactly. No v4 image-only feasibility or model difficulty claim is
made; a fresh Opus 5.5 run is still required before judging the difficulty target.

## Remote Commands

Only task source is staged at the worker's isolated `icl/` directory. Create a
fresh evidence directory with mode 1777 (Docker user-namespace remapping), mount
source read-only and never stage credentials. On koa04:

```sh
BASE=/home/wang100/projects/ale-validator-hardening-20261116
OUT=$BASE/evidence/icl-worker-UNIQUE
mkdir -p "$OUT"
chmod 1777 "$OUT"
docker run --rm --user 0 --cpus 2 --memory 4g --network none \
  -e MUJOCO_GL=osmesa -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e LP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  --entrypoint /opt/venv/bin/python3 \
  -v "$BASE/icl:/task:ro" -v "$OUT:/evidence" \
  ale-icl-maintainer-env:20261116 /task/tests/review/check_task.py
```

With the same Docker options replace the Python script/arguments with:

```sh
/task/tests/review/privileged_reference.py --all-hidden --output /evidence/hidden24
/task/tests/review/build_oracle_fingerprints.py --output /evidence/fingerprints
/task/tests/review/render_ledger.py --ledger /evidence/hidden24/1699855537204788234.ledger.json --output /evidence/views
```

For a live service pilot use `--entrypoint /bin/bash` and
`/task/tests/review/run_episode.sh 1699855537204788234 /evidence oracle`.
This exercises setup, unprivileged oracle socket calls, root ledger and replay,
but uses a synthetic oracle trajectory; it is not a replacement for ALE validate.

## Legacy Evidence

`legacy/round2-8b7195d/` preserves the maintainer's reference, source, selection,
baseline/development/margin records byte-for-byte. Its image-only reference relies
on removed compass/coordinates; its unprotected token endpoint is NOT active.
`hidden_set/selection.jsonl` and the historical portion of `hidden_set/RULE.md`
describe seed provenance only, not current performance.

`v3/` records the pre-integration candidate on the previous hidden set.
`v3/exitfix/` is its full 24-seed rerun after fixing replay SystemExit handling.
Neither is a v4 success or difficulty measurement. The earlier ceiling policy is
in git history. No obsolete reference is invoked by the current `solve.sh`.
