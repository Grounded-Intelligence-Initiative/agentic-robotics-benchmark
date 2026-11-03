# Cross-Embodiment Adaptation

**Draft task candidate.** Explore a partially observed map with a small robot, switch to a larger robot with different button responses, then reach the navigation goal using the persistent map.

![Actual CPU simulator capture](preview.jpg)

The large image is a reviewer camera. The inset and [agent-view.png](agent-view.png)
show the actual restricted observation. Both use the same PBR scene, not a marketing mockup.

## Visual refresh

MuJoCo 3.16 Filament runs on CPU Vulkan (lavapipe), with no GPU fallback.
The scenes include CC0 industrial scans, correctly scaled surface UVs, normal/roughness
maps, fixed-exposure lighting, and spatial supersampling. Temporal post-processing is
disabled so an unchanged scene produces identical camera pixels. Tracking stickers
remain visible; no semantic overlays or privileged state are added to observations.

Original action semantics, calibrated cameras, collision proxies, scoring and budgets
are retained. Detailed scenery has zero mass and contact response. This is a visual
upgrade of a lightweight simulation, not a claim of hardware-accurate dynamics.

## Randomization

Wall openings, starting poses, stage goals, target-body footprint, button mapping and response gain; lighting and background details. The map persists across the body switch; the target body's mapping and gain change only once.

The private seed persists throughout an episode. Reconnecting does not resample the
scene or restore budgets. Six-seed determinism/variation/phase checks passed.

## Validation

- Image-only scripted reference: **3/3** private randomized validation scenes.
- Development demonstration: seed 101, 147 actions, 0 contacts.
- Real ALE gate: **untouched 0.0 / oracle 1.0**, after measuring the new anchor.
- Anchor `success_rate`: 1.000000000, from 1 actual oracle runs.
- CPU only; no Astra/frontier-model calls. Pinned engine `90a7c1cfa649c5dbf1078714773f99840e44cba3`.
- Shared-service security probe: 14 permission/forgery checks, forged success scored zero.

The numbers above are feasibility checks, not model difficulty measurements
or a sealed benchmark test set.

[Raw evidence](validation-evidence.json) · [Success example](success-example.json) ·
[Interaction log](success-interaction.json)

## Reproduce

From `vendor/ale`:

```bash
uv run ale lint ../../tasks/quadruped/cross_embodiment_adaptation
uv run ale validate ../../tasks/quadruped/cross_embodiment_adaptation
```

Inside the task image, `tests/review/check_randomization.py` checks six scenes and
`tests/review/review_task.py --out /tmp/review --seeds 101 102 103` captures reviewer evidence.

## Limits

Both bodies use scaled Go1 meshes and stabilized planar motion. Post-switch calibration consumes the same action budget: this is budgeted adaptation, not strict zero-shot. Whether prior exploration improves an actual model over no exploration is not yet measured.

Keep this PR in **Draft**. Independent review, task registration and frontier-model
difficulty evaluation remain pending. Scanned props do not make a task difficult by
themselves. Earlier failures and pilot images are retained in the author's local logs.

## Assets

Powered by Poly Haven: CC0 scanned utility cabinet, generator, concrete barrier and
surface textures. Menagerie Crazyflie/Go1 visual meshes retain their original licenses.
The lighting probe ships with the Apache-2.0 MuJoCo package. Pinned URLs, SHA256 hashes
and notices are in `environment/visual_assets.json` and `environment/LICENSE.*`. No commercial
warehouse pack, private laboratory scene or generated marketing image is included.
