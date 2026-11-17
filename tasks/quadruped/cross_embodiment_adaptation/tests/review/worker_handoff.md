# PR37 V4 Integration Handoff

## Frozen Source

Local checkpoint: `07f3a3a`. Integrated maintainer round-2 head: `9d8cc14`.
The containing merge commit has these two parents. No push was performed.
All changes are inside `tasks/quadruped/cross_embodiment_adaptation/`.

Production source was frozen on 2026-11-16 (Toronto) for the parent's new ALE
snapshot. Native payload contacts, force-driven nonholonomic dynamics, oriented
docking, low onboard views and the privileged reference are unchanged after that
freeze. This is candidate v4, distinct from maintainer round 2's v3.

Exact maintainer budgets: 500 scout actions, 240 carrier actions, 1000 requests,
600 s program time, 1800 s mission wall time, 16200 s separate verifier, 7200 s
agent session. Both runtime copies agree byte-for-byte. Parsed task settings,
hidden sites, complete candidate list and grader config match `9d8cc14` exactly.
Mission-program run.sh, solve.sh and nonet.py remain unchanged.

Frozen SHA256 values (paths relative to the task):

| Path | SHA256 |
| --- | --- |
| tests/runtime/world.py | 71935f6c54af97273108ca7c140a3d23a66d42c0a2354c19cff577bca3e93a19 |
| tests/runtime/mechanics.py | cfb9f2fc34947453f139f160bb16cafd1fc1baf604d34c45762554552fcae04b |
| tests/runtime/service.py | af327a5a11f1af7dc61ecc021d1ce66aa282ff551aa95d5f586fd814733b0702 |
| solution/oracle/oracle.py | d735beb9138d80b59e1d179d1653f33267781e20ab243c89b8ee3f83a24cca60 |
| tests/grader.py | d301521c40c4e9c107cede660b6840de949a9121bb562d7f08c98a72195276fb |
| tests/hidden/sites.json | 2f4d92b32d94dccb5fd2873cc3010fd442c186af2bb4b5ca5a14232b8f580f85 |

## Fresh Results

The exact 240-action frozen source was rerun on koa04 using MuJoCo 3.16,
2 CPUs, 4 GB RAM, CPU rendering configuration and no network:

- Latest hidden suite: 8/8 successes, zero contacts; carrier counts in site order
  127, 130, 148, 146, 129, 138, 141, 136. Receipt: v4_hidden_native.json.
- Development seeds 1000-1019: full reference 20/20, zero contacts, 128-150
  carrier actions. Blind, scout_route, xy and circumscribed each 0/20.
  Receipt: v4_development_native.json. All 100 outcomes are retained.
- Focused remote tests: 13/13 (11 mechanics/budget/native tests plus two service
  ledger regressions). Local critic authorization unit tests: 7/7.
- Domain lint and staged whitespace checks pass. Every receipt source hash was
  checked against the current local file; seeds were checked in exact order.
- Both carrier channels survive in-memory and persisted service diagnostics;
  invalid action logging remains bounded. No scoring change from this P2 fix.

These are privileged direct-World diagnostics, not official ALE validation or
frontier-model difficulty measurements. The baseline failures have limited scope:
XY/scout replay can fail in the initial heading maneuver, and circumscribed
inflation rejects the route before driving. They do not prove every XY planner
fails or establish image-only feasibility. Parent owns the new independent ALE
oracle/nop run, final anchor, security report and frontier evaluation.

The completed critic-owned check_security_integration.py is included unchanged.
The parent reported its 8/8 stolen-token denials on checkpoint 07f3a3a; this is
historical security evidence, not a fresh v4 fixture run. Oracle authorization
remains strict and engine-owned, with ordinary run_mission calls unauthorized
by default. Author review calls explicitly pass oracle_authorized=True.

## Remote Receipt And Commands

Host source: `koa04:/home/wang100/projects/ale-validator-hardening-20261116/cross-v4`.
Image: `ale-cross-maintainer-env:20261116`.
Image ID: `sha256:419e6251d06b502c97a80a3c02d78dfbc9fc0a2c0399d88454e845eee0ca07c0`.
Output root: `koa04:/home/wang100/projects/ale-validator-hardening-20261116/evidence/cross-worker-v4-20261116`.
Current per-mission traces and summaries: `hidden-240/` and `development-240/`.
Earlier `hidden/` and `development/` runs retain the pre-freeze 260-action source
hashes; they were not relabeled and are not the active receipts.

Run on koa04, using a fresh output subdirectory on repeat:

```bash
ROOT=/home/wang100/projects/ale-validator-hardening-20261116
SOURCE=$ROOT/cross-v4
OUT=$ROOT/evidence/cross-worker-v4-20261116
IMAGE=ale-cross-maintainer-env:20261116
run_cross() {
  docker run --rm --cpus 2 --memory 4g --network none \
    -e MUJOCO_GL=osmesa -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e LP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
    --entrypoint /opt/venv/bin/python3 \
    -v "$SOURCE:/task:ro" -v "$OUT:/evidence" "$IMAGE" -B "$@"
}
run_cross -m unittest discover -s /task/tests/review -p 'test_*.py' -v
run_cross /task/tests/review/check_security.py
run_cross /task/tests/review/v4_rollout.py --seeds hidden --out /evidence/hidden-240-repeat
run_cross /task/tests/review/v4_rollout.py --seeds 1000-1019 \
  --modes full,blind,scout_route,xy,circumscribed --out /evidence/development-240-repeat
```

Local lightweight checks, from the repository root:

```bash
python3 -B tasks/quadruped/cross_embodiment_adaptation/tests/review/check_security.py
python3 scripts/lint_domain.py tasks/quadruped/cross_embodiment_adaptation
git diff --check
```

## Changed Paths And History

- `environment/setup/runtime/` and `tests/runtime/`: mirrored native world,
  predictive mechanics, visual assets and two-channel ledger fix.
- `environment/setup/payload/`, `tests/setup_payload/`, `solution/oracle/`:
  current interface docs/decoders, v4 protocol and privileged reference.
- `tests/grader.py`: critic truth gate preserved; latest 600/1800 defaults merged.
  `tests/grader_config.json`, `tests/hidden/sites.json`, `candidates.json` and
  `task.toml`: latest suite and settings preserved.
- `instruction.md`, `solution/README.md`, `solution/reference/README.md`,
  `tests/anchor.json`, hidden selection and reviewer docs: current mechanics,
  limits, preliminary native provenance and explicit historical separation.
- `solution/v4-views/`: actual development-seed low onboard frames, not the old
  high chase-camera images. Rendering geometry did not change during integration.
- `tests/review/test_service_logging.py`, `test_v4_mechanics.py`, `v4_rollout.py`,
  `v4_render.py`, authorization tests and fresh JSON receipts: focused verification.
- `tests/review/legacy_round2/`: 53 upstream files checked byte-for-byte against
  9d8cc14, including full reference, ablations, measurements, source and selection.
  Original task.toml is named maintainer-task.toml to avoid nested task discovery.
- `tests/review/legacy_v2/`: unchanged from checkpoint. `legacy_candidate_v3/`:
  unedited pre-integration native receipts and old handoff, clearly historical.

No new features or further world tuning are pending in this worker's scope.
