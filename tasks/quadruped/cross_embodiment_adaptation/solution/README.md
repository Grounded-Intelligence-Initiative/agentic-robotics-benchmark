# Cross-Embodiment Adaptation: solution

![A development site (not a hidden one): the hall as reviewers see it, the privileged oracle's carrier route in orange,
the scout's 1.4 m and the carrier's 0.5 m view circles; right: the scout's and the carrier's actual observations and
the delivered carrier](preview.jpg)

The large image is a reviewer view of development seed 1001 (no agent ever gets it). The agent's own observation is
the masked top view on the right, as in [agent-view.png](agent-view.png): the scout sees 1.4 m around itself, the
carrier 0.5 m, and the delivery pad's beacon is always visible.

## What is here

| path | what it is |
|---|---|
| `solve.sh` | the oracle episode: copies `oracle/` into `/home/user/submission/` |
| `oracle/` | the privileged oracle mission program. It holds a token whose sha256 only `tests/hidden/oracle_token.sha256` knows, so the graded robot service answers its `truth` call (the site's obstacles and the robot's exact state and dynamics). It presses 12 scout buttons in place, switches, and drives the carrier along a route found by A* over whole button pulses in position-velocity space, each pulse predicted exactly and checked for clearance at its 10 ms substeps. It shows that every hidden mission can be completed; it is not an agent's solution. |
| `reference/` | the image-only reference mission program (no token, no truth): it uses only what the instruction gives an agent. The scout identifies the buttons with four presses and explores until the carrier's planned route is seen end to end (plan the route with unseen cells assumed free, visit the first unseen stretch of its corridor, repeat); walls, crates and the pad are segmented by colour. After the switch, eight presses fit the carrier's lagged response (one pole and a steady speed), then A* over pulses on the scouted map, inflated by the worst-case 0.86 m footprint plus a margin. It is the evidence that the observations suffice (`tests/review/reference_dev.json`). |

## Validation

- The privileged oracle on the 8 hidden sites: see `tests/anchor.json` (the anchor) and `tests/review/selection.md`
  (the pre-registered selection and its results).
- The image-only reference on development seeds: `tests/review/reference_dev.json`.
- How much of the map the carrier needs: `tests/review/map_ablation.json` (a perfectly controlled carrier with the
  full map, with the partition but no crates, and with no map).

## Reproduce

From the benchmark's pinned engine (`vendor/ale`):

```bash
uv run ale lint ../../tasks/quadruped/cross_embodiment_adaptation
uv run ale validate ../../tasks/quadruped/cross_embodiment_adaptation
```

## Limits

Both bodies use scaled Go1 meshes on simple collision boxes with supplied leg stabilization and planar base motion;
the carrier's lag is a heavier body under the same velocity drive. This is a simulated task, not a hardware-validated
digital twin.

## Assets

Powered by Poly Haven: CC0 scanned utility cabinet, generator, concrete barrier and surface textures. Menagerie
Crazyflie/Go1 visual meshes retain their original licenses. The lighting probe ships with the Apache-2.0 MuJoCo
package. Pinned URLs, SHA256 hashes and notices are in `environment/visual_assets.json` and `environment/LICENSE.*`.
