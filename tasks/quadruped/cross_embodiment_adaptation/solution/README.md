# Cross-Embodiment Adaptation: solution

![A development site (not a hidden one). Left: the hall as reviewers see it, with the scout's track in white, the
carrier's in orange, the dock and the pad. Middle: the map the reference's scout made in its own frame (floor mosaic;
green: floor seen in the trusted centre of a frame; red: feet of walls and footprints of crates) with the carrier's
route. Right: what the two robots see: a frame of the scout's mast camera and one of the carrier's floor
camera](preview.jpg)

The pictures are of development seed 1001 (no agent ever gets the left one). The agent's own observations are on the
right, as in [agent-view.png](agent-view.png): the scout's mast camera (a pinhole 2.0 m above the floor: walls and
crates lean away from the centre and hide the floor behind them) and the carrier's floor camera (about 0.6 m of floor
under its base, nothing else).

## What is here

| path | what it is |
|---|---|
| `solve.sh` | the oracle episode: copies `oracle/` into `/home/user/submission/` |
| `oracle/` | the privileged oracle mission program. It holds a token whose sha256 only `tests/hidden/oracle_token.sha256` knows, so the graded robot service answers its `truth` call (the site's obstacles and the robot's exact pose, velocity and dynamics). It presses 12 scout buttons in place, switches, and drives the carrier by model-predictive control on the true map: before every press it reads the true state and rolls every sequence of the next five presses through the exact model; the first press must stay clear even if it slips (a slip cannot be known in advance, not even by the oracle). It shows that every hidden mission can be completed; it is not an agent's solution. |
| `reference/` | the reference mission program from the robots' own observations (no token, no truth): the evidence that they suffice (`tests/review/reference_dev.json`). `vision.py`: image geometry and registration. `perception.py`: what a frame of the mast camera says about the floor plane (floor seen, a wall's foot, a crate's footprint from its lid). `mapping.py`: the floor mosaic and the vote maps in the scout's frame; every frame is registered on the mosaic, so slips and turns are measured. `explore.py`: find the pad, then see every cell of a route the carrier can take. `floorloc.py`: the carrier finds its pose in the scout's map from its floor camera (every heading, five spans), then tracks it by visual odometry corrected on the mosaic. `driving.py`: fits the carrier's lagged response as it drives and steers by model-predictive control. `planning.py`: routes on the map. |

## Validation

- The privileged oracle on the 8 hidden sites: see `tests/anchor.json` (the anchor) and `tests/review/selection.md`
  (the pre-registered selection and its results).
- The reference on development seeds, run by the task's grader: `tests/review/reference_dev.json`.
- What the carrier needs to know: `tests/review/ablation.json` (a perfectly controlled carrier with the truth; without
  the scout's map; with the map but without finding itself in it, in position, in heading, or while it drives).

## Reproduce

From the benchmark's pinned engine (`vendor/ale`):

```bash
uv run ale lint ../../tasks/quadruped/cross_embodiment_adaptation
uv run ale validate ../../tasks/quadruped/cross_embodiment_adaptation
```

## Limits

Both bodies use scaled Go1 meshes on simple collision boxes with supplied leg stabilization and planar base motion;
the carrier's lag is a heavier body under the same velocity drive; a slip is a press that delivers part of the
commanded velocity and turns the body. The scout's camera is an ideal pinhole and the carrier's an ideal orthographic
view with a simple sensor model (exposure, colour balance, vignetting, noise). This is a simulated task, not a
hardware-validated digital twin.

## Assets

Powered by Poly Haven: CC0 scanned utility cabinet, generator, concrete barrier and surface textures. Menagerie
Crazyflie/Go1 visual meshes retain their original licenses. The lighting probe ships with the Apache-2.0 MuJoCo
package. Pinned URLs, SHA256 hashes and notices are in `environment/visual_assets.json` and `environment/LICENSE.*`.
