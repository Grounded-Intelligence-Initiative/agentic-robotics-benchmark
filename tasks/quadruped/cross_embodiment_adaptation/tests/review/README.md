# V3 Reviewer Material

## Evidence and Limits

Current native remote diagnostics (MuJoCo 3.16, 2 CPUs, 4 GB, CPU only):
full privileged reference 8/8 fixed hidden and 20/20 development seeds 1000-1019.
Blind, scout-route replay, XY-width planning and conservative circumscribed
inflation each completed 0/20 development missions. The first three contact
obstacles; the circumscribed approximation rejects the service opening.
These deliberately limited baselines do not establish frontier-model difficulty
or prove that every possible XY planner fails. In particular XY/scout replay may
fail during the initial heading maneuver, not exclusively inside the dogleg.

- v3_hidden_native.json and v3_development_native.json: per-mission results.
- test_v3_mechanics.py: orientation-dependent fit, endpoint-versus-sweep collision,
  dogleg width versus turn orientation, native child-payload contact, inertia,
  traction, heading/rest docking, public observation schema and runtime parity.
- v3_rollout.py: trusted direct-World reference and matched controller baselines.
- v3_render.py: actual front/rear onboard frames, both phases.
- select_sites.py: author-only fixed-suite verifier runner. Never reselects seeds.
- selection.md: unchanged seeds and outcome-independent v3 geometry policy.
- legacy_v2/: quarantined old measurements, selection features and XY solver.

The author reference is explicitly PRIVILEGED. The public v3 observation pipeline
has not yet been solved by an image-only reference, and no Opus 5.5 v3 score is
claimed. Parent owns the independent ALE oracle/nop validation, security checks,
final anchor and frontier evaluation.

## Reproduce Remotely

Mount only this task at /task:ro and a fresh writable output directory at /evidence.
The following require the task image and its assets; run substantial rollouts on
the provisioned remote CPU, not ubt2025:

```bash
/opt/venv/bin/python3 -B /task/tests/review/test_v3_mechanics.py
/opt/venv/bin/python3 -B /task/tests/review/v3_rollout.py --seeds hidden --out /evidence/hidden
/opt/venv/bin/python3 -B /task/tests/review/v3_rollout.py --seeds 1000-1019 --modes full,blind,scout_route,xy,circumscribed --out /evidence/development
/opt/venv/bin/python3 -B /task/tests/review/v3_render.py --seed 1001 --out /evidence/render
```

Provenance: koa04:/home/wang100/projects/ale-validator-hardening-20261116/cross,
image ale-cross-maintainer-env:20261116, outputs under
evidence/cross-worker-v3-20261116/. These no-render diagnostics use the real native
physics but bypass the service/security/runner. They are not official ALE results.
Author run_mission calls must explicitly pass oracle_authorized=True; ordinary
submissions may not gain truth just by copying the token.
