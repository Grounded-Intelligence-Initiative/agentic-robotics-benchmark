# In-Context Control Learning: the oracle

A frozen model teleoperates a quadruped delivery robot through six deliveries in a dark bay, one button press per
tool call. It has to learn from the camera images what the four unlabeled buttons do (body-frame forward, back, step
left and step right, at an unknown heading, with unequal gait speeds, all changed by a loading stop after the third
delivery), where the partitions and their openings are, and keep both in mind across the episode. No program may
drive the robot or process the images; the grader checks that from the session.

## The oracle

`solve.sh` runs `oracle_run.py`, the image-only scripted reference in `policy.py`. It gets what the agent gets (the
image and the JSON status of each reply, the camera calibration from the instruction) and nothing else:

- it presses each button once to measure its displacement, and again after the loading stop;
- it adds every lit wall pixel to a map, which persists across deliveries;
- it plans with A* over its own measured button displacements, unknown floor counting as free, and re-plans after
  every press; presses that hit nothing refine the displacement estimates;
- it advances as soon as a delivery point is reached.

It is a feasibility proof, not an LLM result. The grader recognises it by the engine's trajectory (agent name
`oracle`) and skips the direct-control audit for it, since a script necessarily talks to the robot socket.

## Measurements

| what | result |
|---|---|
| hidden set (24 scenes, `tests/review/hidden_set/`) | 24 of 24 at six of six delivery points, 194 to 406 presses (median 300) of 500, at most 412 of 560 requests |
| 40 development seeds (3001-3040) | 40 of 40 at six of six |
| random buttons (100 development seeds) | 0.012 of the points on average |
| privileged greedy, no map memory (100 development seeds) | 0.148 |

`tests/review/check_task.py` runs the task's self-checks inside the image; `tests/review/run_episode.sh` runs one
episode the way the engine does; `tests/review/baselines.py` the two baselines.

## What the simulation is

A planar base under a velocity controller with a scaled Unitree Go1 body (MuJoCo Menagerie meshes) and stabilized
gaits, not legged locomotion; walls and the fence are boxes. The images are rendered on the CPU with MuJoCo 3.16's
Filament renderer (lavapipe), with Poly Haven CC0 textures and props; the render assets come from the benchmark's
Hugging Face dataset, pinned in `environment/assets.sha256`.
