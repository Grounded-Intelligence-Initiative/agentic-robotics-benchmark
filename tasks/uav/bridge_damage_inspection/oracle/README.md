# UAV Bridge Inspection

**Draft task candidate.** Use a UAV's local RGB view to find a missing bridge-deck section and submit its position with an authentic observed frame ID.

![Actual simulator capture, with reviewer and agent views](preview.jpg)

The large image is a reviewer view. The inset and [agent-view.png](agent-view.png)
show the actual restricted observation; the agent never receives the reviewer camera.
Robot meshes and textured scenery are present in the live simulator, not a marketing mockup.

## Scene randomization

Damage position, gap width, bridge lateral offset and deck width; lighting and surrounding details. The defect remains physically visible and reachable within the inspection footprint.

The trusted service draws a private scene seed per episode. A repeated seed reproduces
the scene. Reconnecting does not resample it or refund budgets. The environment varies
between episodes, not arbitrarily between steps of the same learning problem.

## Local validation

- Pinned ALE engine: `90a7c1cfa649c5dbf1078714773f99840e44cba3`; CPU only, no model API calls.
- Image-based scripted reference: **8/8** randomized validation scenes. Image-only scripted policy under the task action budget, not a frontier model.
- Successful development example: seed 101, 14 actions, 0 collisions.
- Six-seed determinism, variation and applicable phase-persistence checks passed.
- Fourteen shared service permission/forgery checks passed through the bridge-task ALE sandbox;
  forged success scored zero. This does not constitute a complete security audit.
- Anchor `success_rate` measured from 2 real ALE oracle runs;
  final gate repeated separately after calibration.

```text
ok    bridge_damage_inspection: untouched={'reward': 0.0}, oracle={'reward': 1.0}
```

Raw per-case outcomes, real gate episode IDs and metrics are in
[validation-evidence.json](validation-evidence.json). The successful run is in
[success-example.json](success-example.json), with its [interaction log](success-interaction.json).
These are validation scenes, not an untouched benchmark test set or model results.

## Reproduce

From the repository's pinned `vendor/ale` directory:

```bash
uv run ale lint ../../tasks/uav/bridge_damage_inspection
uv run ale validate ../../tasks/uav/bridge_damage_inspection --runs-dir /tmp/bridge_damage_inspection-validation
```

For reviewer-only scene checks, run `tools/check_randomization.py` and
`tools/review_task.py --out /tmp/bridge_damage_inspection-review --seeds 101 102 103`
in the task image, with `VISUAL_ROBOTICS_ASSETS=/opt/visual_robotics/assets`.
The latter saves real scene images, interactions and per-seed outcomes.

## Limits and merge blockers

Altitude-stabilized planar UAV motion and a missing deck segment, not subtle crack recognition or full flight control. The local view is calibrated orthographic imaging, not a perspective onboard camera. This first variant may be too easy for a frontier model.



Frontier/Astra benchmarking has **not** been run on this revision. Passing the oracle
gate proves implementation feasibility, not the benchmark's desired difficulty.
Keep this PR in Draft until model evaluation, independent review and coordination-sheet
registration are confirmed. The task must be strengthened if frontier scores are too high.

## Assets

Licensed Menagerie Crazyflie/Go1 meshes, CC0 Poly Haven textures and authored scene geometry.
Sources, pinned hashes and full notices are under `image/visual_assets.json` and
`image/LICENSE.*`. No proprietary warehouse or lab scene has been copied.
