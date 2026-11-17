# Cross-Embodiment Adaptation v4

## Current Reference

`oracle/` is the **PRIVILEGED author reference**, not an image-only solution.
`solve.sh` installs its mission program; the existing oracle/run.sh entrypoint
is unchanged. The separate verifier must authorize the official oracle episode
as well as validating its secret token. Ordinary submissions must not gain
truth by copying the public author code/token.

The reference receives exact geometry, pose, dynamics and channel assignment.
It searches rounded service-aisle routes in position/heading, checking the
swept elongated footprint at 2.5 cm spacing, then executes low-speed feedback
through the same force-driven MuJoCo interface and brakes into the oriented dock.
It does not use site seeds, stored trajectories, or modify the world. It exploits
the published constructive aisle topology, so it is a feasibility reference,
not a general-purpose motion planner or proof of frontier difficulty.

Fresh native, no-render diagnostics on koa04 completed all eight latest hidden
seeds retained from 9d8cc14, with 127-148 carrier actions. This is not an official separate-verifier
anchor. The parent owns that run and final scoring/security evidence.
See tests/review/v4_hidden_native.json and tests/review/README.md for provenance.
The complete round-2 floor-localization source/reference/evidence is preserved
byte-for-byte under tests/review/legacy_round2. The older v2 material remains
under legacy_v2, and pre-integration candidate receipts under legacy_candidate_v3.
None of these historical measurements is presented as v4 evidence.

## Mechanics and Observability

MuJoCo integrates x/y slide and yaw hinge dynamics with native contacts,
including the welded child payload. Traction-bounded forces, lateral stabilization,
steering lag and yaw inertia are explicit abstractions. Real Go1 meshes remain
visible inside an open rack whose outline matches a conservative filled collision
rectangle. There is no foot locomotion, payload articulation or hardware validation.

Public observations are two low, body-mounted perspective cameras, no absolute
pose or visibility disk. The cyan/white rack ends indicate body orientation and
extent. The 2.24 x 1.10 m blue docking bay has a yellow front band. The larger
painted bay keeps the alignment cue observable around the load. All these are
physical scene appearances, not overlays or annotations.

V4 deliberately replaces round 2's floor-only carrier localization and odometer
with native long-payload dynamics, swept-volume turns and oriented docking.
It preserves the later maintainer's larger budgets, fixed suite and isolation.
PR37's earlier reviewer completed v2 and the later floor-localization candidate.
Those results do not establish v4 difficulty. A new frontier evaluation and an
image-only feasibility demonstration are still needed. The camera change and
new action schema alone must not be counted as genuine difficulty evidence.

## Reproduction

Inside the task image on remote CPU:

```bash
/opt/venv/bin/python3 -B /task/tests/review/test_v4_mechanics.py
/opt/venv/bin/python3 -B /task/tests/review/v4_rollout.py --seeds hidden --out /evidence/hidden
/opt/venv/bin/python3 -B /task/tests/review/v4_rollout.py --seeds 1000-1019 --modes full,blind,scout_route,xy,circumscribed --out /evidence/ablations
/opt/venv/bin/python3 -B /task/tests/review/v4_render.py --seed 1001 --out /evidence/render
```

The direct-World harness is a development test, not a substitute for ALE validate.
From the pinned engine, run the unchanged lint/validate mission-program workflow.

## Assets

Poly Haven CC0 workcell assets and textures; licensed Menagerie Go1 meshes;
MuJoCo's Apache-2.0 lighting probe. Pinned URLs, hashes and notices remain in
environment/visual_assets.json and environment/LICENSE.*.
