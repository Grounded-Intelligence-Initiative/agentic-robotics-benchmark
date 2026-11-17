# PR37 V3 Worker Handoff

Implementation frozen 2026-11-16. No commit or push. Scope is this task only.
Parent/reviewer owns tests/grader.py, tests/review/check_security.py, official
ALE validation, final anchor, frontier measurement and global report.

## Bounded P2 Logging Follow-up

The only production follow-up after the freeze is service.py line 94 in both
environment/setup/runtime and tests/runtime: action diagnostic slices now retain
two values instead of one. Dispatch, dynamics, scoring and budgets are unchanged.
World, mechanics and oracle remain byte-identical to the frozen hashes below.

New tests/review/test_service_logging.py runs the real Runtime request/ledger
code with a stub world. Both tests pass locally and in the remote task image:
scout and both carrier channels survive the on-disk ledger, logged actions are
copied, and malformed actions still have bounded two-value diagnostics/errors.

A NEW eight-site native run completed 8/8 with the same action counts below.
Receipt: tests/review/v3_hidden_native_service_logging.json. Remote raw traces:
koa04:/home/wang100/projects/ale-validator-hardening-20261116/evidence/cross-worker-v3-20261116/hidden-service-logging-20261116/.
The new receipt records service.py SHA256
6c457a159fae235afe353c750cff0a907d27a94af3e61b7fafe900b90ca24bcc.
Previous receipts were not edited or relabeled. Native direct-World replay does
not exercise the service ledger; that behavior is covered by the new tests.
The parent's already-running official ALE validation uses the prior service.

Exact focused command from the repository root:
`python3 -B tasks/quadruped/cross_embodiment_adaptation/tests/review/test_service_logging.py`

Rerun command inside the remote image with /task:ro and /evidence mounted:
`/opt/venv/bin/python3 -B /task/tests/review/v3_rollout.py --seeds hidden --out /evidence/hidden-service-logging-20261116`

Changed in this bounded follow-up only: the two service.py mirrors, the new
test_service_logging.py, the new v3_hidden_native_service_logging.json, and this
handoff addendum. No other feature or production changes.

## Results

- Native final-source privileged reference: 8/8 frozen hidden seeds.
- Carrier actions in seed order: 136, 144, 133, 144, 148, 147, 135, 124.
- Contiguous development seeds 1000-1019: full 20/20; blind, scout_route,
  xy and circumscribed each 0/20. These are limited mechanistic baselines,
  not a frontier difficulty measurement.
- 10 focused tests pass in the remote task image; locally 7 pass and 3 native
  checks skip because MuJoCo is absent. All 39 Python sources parse.
- Runtime and public payload mirrors match. Domain lint and diff whitespace
  checks pass. Parsed task.toml settings and hidden/sites.json match HEAD.
- Mission run.sh, solve.sh, nonet.py, service.py and scoring/identity settings
  are unchanged by this worker.
- Current low-mast front/rear frames are in solution/v3-views; old chase images
  are not current. Carrier front image SHA256:
  adf03f66e5237e1007077367cb4ebd5cc86c833b68ea639b2d4a87ddcbc18d64.

## Evidence

Host: koa04, checkout /home/wang100/projects/ale-validator-hardening-20261116/cross.
Image: ale-cross-maintainer-env:20261116 (parent-provisioned CPU image).
All worker containers used 2 CPUs, 4 GB, no network, no GPU; all have exited.

Remote output root:
 /home/wang100/projects/ale-validator-hardening-20261116/evidence/cross-worker-v3-20261116

Final evidence subdirectories: hidden-final, development-final, render-mast.
The first two contain raw per-mission state/action traces and summary.json.
Copies with source hashes are tracked at tests/review/v3_hidden_native.json and
tests/review/v3_development_native.json. No-render native diagnostics are NOT
official ALE separate-verifier evidence. Anchor provenance explicitly says so.

Frozen world SHA256:
 61339998d5657ed2fbd8b830288f21395f8ec15aec5f131194b3a9ef2561d087
Frozen mechanics SHA256:
 10069ba6a3715a8383b62e5a755c46b33304ee025bf18dc0ba567e2176177188
Frozen scene_assets SHA256:
 e6b4e839f3e9ec0c1bccd6bc44e2d1f0e042f80ad3d1ed272dd16557d9b66676
Frozen oracle SHA256:
 28c25d4a83b3ec05feca3944838b4ff588edb0edf6ed5ed6c6ec42ec70a46699

## Exact Verification Commands

From the local repository:
```bash
python3 -B tasks/quadruped/cross_embodiment_adaptation/tests/review/test_v3_mechanics.py
python3 scripts/lint_domain.py tasks/quadruped/cross_embodiment_adaptation
git diff --check
```

Remote focused test (no output bind needed):
```bash
ssh -o BatchMode=yes -o IdentityAgent=none -o ForwardAgent=no -o StrictHostKeyChecking=yes koa04 \
  docker run --rm --cpus 2 --memory 4g --network none \
  -e MUJOCO_GL=osmesa -e LIBGL_ALWAYS_SOFTWARE=1 -e LP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  --entrypoint /opt/venv/bin/python3 \
  -v /home/wang100/projects/ale-validator-hardening-20261116/cross:/task:ro \
  ale-cross-maintainer-env:20261116 -B /task/tests/review/test_v3_mechanics.py
```

For native rollouts, add a writable fresh evidence mount at /evidence to that
docker command and replace its final script/arguments with:
```bash
/task/tests/review/v3_rollout.py --seeds hidden --out /evidence/hidden
/task/tests/review/v3_rollout.py --seeds 1000-1019 --modes full,blind,scout_route,xy,circumscribed --out /evidence/development
/task/tests/review/v3_render.py --seed 1001 --out /evidence/render
```

The parent's pinned ALE validation must still exercise official oracle and nop.
Author calls to grader.run_mission explicitly use oracle_authorized=True.
The current reference is PRIVILEGED; no image-only success or v3 Opus score is claimed.

## Worker Changed Paths

Paths below are repository-relative; v2 deletions were moved intact into the
explicitly historical legacy_v2 directory. Grader/security files are excluded
because those belong to the concurrent reviewer.

- tasks/quadruped/cross_embodiment_adaptation/environment/setup/payload/README.md
- tasks/quadruped/cross_embodiment_adaptation/environment/setup/payload/client.py
- tasks/quadruped/cross_embodiment_adaptation/environment/setup/payload/mcp_stdio.py
- tasks/quadruped/cross_embodiment_adaptation/environment/setup/runtime/mechanics.py
- tasks/quadruped/cross_embodiment_adaptation/environment/setup/runtime/scene_assets.py
- tasks/quadruped/cross_embodiment_adaptation/environment/setup/runtime/world.py
- tasks/quadruped/cross_embodiment_adaptation/instruction.md
- tasks/quadruped/cross_embodiment_adaptation/solution/README.md
- tasks/quadruped/cross_embodiment_adaptation/solution/agent-view.png
- tasks/quadruped/cross_embodiment_adaptation/solution/oracle/client.py
- tasks/quadruped/cross_embodiment_adaptation/solution/oracle/oracle.py
- tasks/quadruped/cross_embodiment_adaptation/solution/preview.jpg
- tasks/quadruped/cross_embodiment_adaptation/solution/reference/README.md
- tasks/quadruped/cross_embodiment_adaptation/solution/reference/client.py
- tasks/quadruped/cross_embodiment_adaptation/solution/reference/reference.py
- tasks/quadruped/cross_embodiment_adaptation/solution/reference/run.sh
- tasks/quadruped/cross_embodiment_adaptation/solution/v3-views/1001-0-ego_front.png
- tasks/quadruped/cross_embodiment_adaptation/solution/v3-views/1001-0-ego_rear.png
- tasks/quadruped/cross_embodiment_adaptation/solution/v3-views/1001-1-ego_front.png
- tasks/quadruped/cross_embodiment_adaptation/solution/v3-views/1001-1-ego_rear.png
- tasks/quadruped/cross_embodiment_adaptation/task.toml
- tasks/quadruped/cross_embodiment_adaptation/tests/anchor.json
- tasks/quadruped/cross_embodiment_adaptation/tests/hidden/selection.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/README.md
- tasks/quadruped/cross_embodiment_adaptation/tests/review/hollowing.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/README.md
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/agent-view.png
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/map_ablation.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/preview.jpg
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/reference/client.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/reference/reference.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/reference/run.sh
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/reference_dev.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/selection.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/legacy_v2/selection.md
- tasks/quadruped/cross_embodiment_adaptation/tests/review/map_ablation.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/map_ablation.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/reference_dev.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/select_sites.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/selection.md
- tasks/quadruped/cross_embodiment_adaptation/tests/review/test_v3_mechanics.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/v3_development_native.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/v3_hidden_native.json
- tasks/quadruped/cross_embodiment_adaptation/tests/review/v3_render.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/v3_rollout.py
- tasks/quadruped/cross_embodiment_adaptation/tests/runtime/mechanics.py
- tasks/quadruped/cross_embodiment_adaptation/tests/runtime/scene_assets.py
- tasks/quadruped/cross_embodiment_adaptation/tests/runtime/world.py
- tasks/quadruped/cross_embodiment_adaptation/tests/setup_payload/README.md
- tasks/quadruped/cross_embodiment_adaptation/tests/setup_payload/client.py
- tasks/quadruped/cross_embodiment_adaptation/tests/setup_payload/mcp_stdio.py
- tasks/quadruped/cross_embodiment_adaptation/tests/review/worker_handoff.md
