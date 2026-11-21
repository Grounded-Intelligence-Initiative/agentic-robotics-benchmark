# In-Context Control Learning: the oracle and the reference

A frozen model teleoperates a quadruped delivery robot through six deliveries in a dark bay, one button press per
tool call. It is told where the robot starts and where each delivery point is; everything else it has to work out in
context: which of the four unlabeled buttons is which gait (walk forward, walk back, turn left, turn right), how far a
press moves the robot (again after the loading stop, which slows every gait), where the robot is (it gets a compass,
no position), and where the two partitions and their openings are, from at most 120 frames of the robot's own forward
camera under a floodlight that reaches 1.0 m. No program may drive the robot or process the frames; the grader checks
that from the session.

Two programs are kept here. Neither is an LLM result.

## The oracle: `solve.sh` → `oracle_run.py`, `oracle_policy.py`

Privileged. It sends the oracle token (`oracle_token`, a file that no agent sandbox ever holds) to the robot service's
`unlock` request and receives the scene's ground truth (walls, button assignment, gait speeds, load factor, the six
points) and, from then on, the robot's true pose in every reply. It plans on the true map (`nav.py`: a travel-distance
field over a 5 cm grid, then a short search over press sequences, a new plan after every press) and never looks
through the camera.

The service keeps no secret: it answers any `unlock`, and records the sha256 of the token it was sent. Only `tests/`
holds the expected hash (`tests/oracle_token.sha256`); the grader scores a ledger with an `unlock` 0 unless the hash is
that one. The MCP bridge does not expose `unlock`, so for an agent the request would also be a request that did not
come from its `robot_*` tools, which the instruction says scores 0.

The oracle fixes what a full score is. On the hidden set it reaches six of six on every scene (the anchor,
`tests/review/hidden_set/`).

## The image-only reference: `reference_run.py`, `reference_policy.py`

Not privileged: no token, no ground truth. It uses what an agent gets (the status of every reply, the start pose and
the camera frames, the instruction's camera model and bay description) and is the evidence that those observations
suffice:

- the remote: the two turns from the compass; then it faces the nearest fence side, walks until a wall shows in the
  middle of the frame, and takes the walking button that brings it nearer as forward; the step is the change of that
  range over a run of forward presses; after the loading stop five presses of each turn give the slow-down factor;
- perception: in every image column, the row where the lit floor ends (at a wall's foot or at the edge of the light)
  and the row of the wall's top edge, turned into a range with the camera model: a 90 degree fan of ranges per frame;
  the pad from its green foot print;
- position: dead reckoning with the compass and the learned step, corrected by the fence sides and the partition
  faces in view (their x is estimated together with the robot's), by the pad, and by how far the wall feet moved
  between two frames (which also shows a press that a wall stopped);
- map and driving: an occupancy grid; the same planner as the oracle on it, with unseen parts of the partition
  bands passable at a price; a frame only where a wall could be ahead that no frame has covered, near walls, before
  an opening, and every sixth step otherwise.

`tests/review/run_episode.sh <seed> <out> reference` runs it through the socket and grades its ledger like an agent's.

## Measurements

See `tests/review/README.md` for the numbers (oracle on the hidden set and on development seeds, the reference on 100
development seeds, the baselines, and the design measurements that led to the stated light radius and budgets).

## What the simulation is

A planar base (x, y, yaw) under a velocity controller, with a scaled Unitree Go1 body (MuJoCo Menagerie meshes) and
gait speeds, not legged locomotion; walls and the fence are boxes. The frames are rendered on the CPU with MuJoCo
3.16's Filament renderer (lavapipe) from a camera on the robot, with Poly Haven CC0 textures and props; what the
floodlight does not reach is blacked out from a ray cast of the same view. The render assets come from the
benchmark's Hugging Face dataset, pinned in `environment/assets.sha256`.
