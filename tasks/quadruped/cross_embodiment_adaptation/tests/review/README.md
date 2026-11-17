# V4 Reviewer Material

## Evidence and Limits

Current native remote diagnostics (MuJoCo 3.16, 2 CPUs, 4 GB, CPU only):
full privileged reference 8/8 latest 9d8cc14 hidden seeds and 20/20 development
seeds 1000-1019, freshly rerun with the exact 500/240/1000 maintainer budgets
after integration (not relabeled old receipts).
Blind, scout-route replay, XY-width planning and conservative circumscribed
inflation each completed 0/20 development missions. The first three contact
obstacles; the circumscribed approximation rejects the service opening.
These deliberately limited baselines do not establish frontier-model difficulty
or prove that every possible XY planner fails. In particular XY/scout replay may
fail during the initial heading maneuver, not exclusively inside the dogleg.

- v4_hidden_native.json and v4_development_native.json: per-mission results.
- test_v4_mechanics.py: orientation-dependent fit, endpoint-versus-sweep collision,
  dogleg width versus turn orientation, native child-payload contact, inertia,
  traction, heading/rest docking, public observation schema and runtime parity.
- v4_rollout.py: trusted direct-World reference and matched controller baselines.
- v4_render.py: actual front/rear onboard frames, both phases.
- select_sites.py: author-only fixed-suite verifier runner. Never reselects seeds.
- selection.md: unchanged seeds and outcome-independent v4 geometry policy.
- legacy_v2/: quarantined old measurements, selection features and XY solver.
- legacy_round2/: 53 byte-preserved files from 9d8cc14, including its full
  reference, ablation, measurements, private source and original selection.
- legacy_candidate_v3/: pre-integration receipts for the superseded seed suite.
- test_service_logging.py: both carrier channels in the persisted service ledger.
- check_security.py / check_security_integration.py: critic-owned authorization
  checks and a disposable real-grader fixture; not frontier-model evaluations.

The author reference is explicitly PRIVILEGED. The public v4 observation pipeline
has not yet been solved by an image-only reference, and no Opus 5.5 v4 score is
claimed. Parent owns the independent ALE oracle/nop validation, security checks,
final anchor and frontier evaluation.

## Reproduce Remotely

Mount only this task at /task:ro and a fresh writable output directory at /evidence.
The following require the task image and its assets; run substantial rollouts on
the provisioned remote CPU, not ubt2025:

```bash
/opt/venv/bin/python3 -B /task/tests/review/test_v4_mechanics.py
/opt/venv/bin/python3 -B /task/tests/review/v4_rollout.py --seeds hidden --out /evidence/hidden
/opt/venv/bin/python3 -B /task/tests/review/v4_rollout.py --seeds 1000-1019 --modes full,blind,scout_route,xy,circumscribed --out /evidence/development
/opt/venv/bin/python3 -B /task/tests/review/v4_render.py --seed 1001 --out /evidence/render
```

Provenance: koa04:/home/wang100/projects/ale-validator-hardening-20261116/cross-v4,
image ale-cross-maintainer-env:20261116, outputs under
evidence/cross-worker-v4-20261116/hidden-240/ and development-240/.
See worker_handoff.md for frozen hashes and exact Docker commands.
These no-render diagnostics use the real native
physics but bypass the service/security/runner. They are not official ALE results.
Author run_mission calls must explicitly pass oracle_authorized=True; ordinary
submissions may not gain truth just by copying the token.
