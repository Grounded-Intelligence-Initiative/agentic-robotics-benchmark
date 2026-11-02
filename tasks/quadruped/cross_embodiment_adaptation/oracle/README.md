# Cross-Embodiment Adaptation

**Draft task candidate.** Explore a partially observed map with a small robot, switch to a larger robot with different button responses, then reach the navigation goal using the persistent map.

![Actual simulator capture, with reviewer and agent views](preview.jpg)

The large image is a reviewer view. The inset and [agent-view.png](agent-view.png)
show the actual restricted observation; the agent never receives the reviewer camera.
Robot meshes and textured scenery are present in the live simulator, not a marketing mockup.

## Scene randomization

Wall openings, starting poses, stage goals, target-body footprint, button mapping and response gain; lighting and background details. The map persists across the body switch; the target body's mapping and gain change only once.

The trusted service draws a private scene seed per episode. A repeated seed reproduces
the scene. Reconnecting does not resample it or refund budgets. The environment varies
between episodes, not arbitrarily between steps of the same learning problem.

## Local validation

- Pinned ALE engine: `90a7c1cfa649c5dbf1078714773f99840e44cba3`; CPU only, no model API calls.
- Image-based scripted reference: **7/8** randomized validation scenes. Image-only scripted policy under the task action budget, not a frontier model.
- Successful development example: seed 101, 147 actions, 0 collisions.
- Six-seed determinism, variation and applicable phase-persistence checks passed.
- Fourteen shared service permission/forgery checks passed through the bridge-task ALE sandbox;
  forged success scored zero. This does not constitute a complete security audit.
- Anchor `success_rate` measured from 2 real ALE oracle runs;
  final gate repeated separately after calibration.

```text
ok    cross_embodiment_adaptation: untouched={'reward': 0.0}, oracle={'reward': 1.0}
```

Raw per-case outcomes, real gate episode IDs and metrics are in
[validation-evidence.json](validation-evidence.json). The successful run is in
[success-example.json](success-example.json), with its [interaction log](success-interaction.json).
These are validation scenes, not an untouched benchmark test set or model results.

## Reproduce

From the repository's pinned `vendor/ale` directory:

```bash
uv run ale lint ../../tasks/quadruped/cross_embodiment_adaptation
uv run ale validate ../../tasks/quadruped/cross_embodiment_adaptation --runs-dir /tmp/cross_embodiment_adaptation-validation
```

For reviewer-only scene checks, run `tools/check_randomization.py` and
`tools/review_task.py --out /tmp/cross_embodiment_adaptation-review --seeds 101 102 103`
in the task image, with `VISUAL_ROBOTICS_ASSETS=/opt/visual_robotics/assets`.
The latter saves real scene images, interactions and per-seed outcomes.

## Limits and merge blockers

Both bodies use scaled Go1 meshes and stabilized planar motion. Post-switch calibration consumes the same action budget: this is budgeted adaptation, not strict zero-shot. Whether prior exploration improves an actual model over no exploration is not yet measured.

One visual-reference case exhausted its 320-action budget. A privileged-state feasibility controller completed the same scene in 72 actions (one contact). This establishes solvability, not equal-information performance. Reference robustness remains a merge blocker; the failed case is retained in the evidence.

Frontier/Astra benchmarking has **not** been run on this revision. Passing the oracle
gate proves implementation feasibility, not the benchmark's desired difficulty.
Keep this PR in Draft until model evaluation, independent review and coordination-sheet
registration are confirmed. The task must be strengthened if frontier scores are too high.

## Assets

Licensed Menagerie Crazyflie/Go1 meshes, CC0 Poly Haven textures and authored scene geometry.
Sources, pinned hashes and full notices are under `image/visual_assets.json` and
`image/LICENSE.*`. No proprietary warehouse or lab scene has been copied.
