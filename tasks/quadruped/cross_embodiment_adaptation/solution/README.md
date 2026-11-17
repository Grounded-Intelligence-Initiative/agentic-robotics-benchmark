# Cross-Embodiment Adaptation v3

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

Direct native, no-render diagnostics on koa04 completed all eight frozen seeds
with 124-148 carrier actions. This is not yet an official separate-verifier
anchor. The parent owns that run and final scoring/security evidence.
See tests/review/v3_hidden_native.json and tests/review/README.md for provenance.
Old v2 screenshots, image-only solver and measurements are explicitly archived
under tests/review/legacy_v2; none is current evidence.

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

PR37's earlier reviewer completed v2 and a later localization-only prototype.
Those results do not establish v3 difficulty. A new frontier evaluation and an
image-only feasibility demonstration are still needed. The camera change and
new action schema alone must not be counted as genuine difficulty evidence.

## Reproduction

Inside the task image on remote CPU:

```bash
/opt/venv/bin/python3 -B /task/tests/review/test_v3_mechanics.py
/opt/venv/bin/python3 -B /task/tests/review/v3_rollout.py --seeds hidden --out /evidence/hidden
/opt/venv/bin/python3 -B /task/tests/review/v3_rollout.py --seeds 1000-1019 --modes full,blind,scout_route,xy,circumscribed --out /evidence/ablations
/opt/venv/bin/python3 -B /task/tests/review/v3_render.py --seed 1001 --out /evidence/render
```

The direct-World harness is a development test, not a substitute for ALE validate.
From the pinned engine, run the unchanged lint/validate mission-program workflow.

## Assets

Poly Haven CC0 workcell assets and textures; licensed Menagerie Go1 meshes;
MuJoCo's Apache-2.0 lighting probe. Pinned URLs, hashes and notices remain in
environment/visual_assets.json and environment/LICENSE.*.
