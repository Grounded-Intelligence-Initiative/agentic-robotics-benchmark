# block_tower_extraction: maintainers' notes

**Status: work in progress (minimum viable package; anchor from the oracle's measured run; difficulty not yet measured
with frontier models).** Development notes live outside the task folder with the contributor.

## What is graded
`play.py` (solve(cell)) runs once per hidden tower (tests/hidden/: four fixed packages written by
`review/make_sets.py`, rule in `hidden/selection_rule.txt`, salt and sha256 in `hidden/selection.json`), the four towers
in parallel, each under its own runner account (btrun1..4). The cell's referee (`towercell/cell.py`) records removals
(a tower block moved > 40 mm from its reference, turntable frame; legal when its level is below the second-highest
complete level), placements (a removed block at rest, not touching the robot, in a free slot of the top level or a new
level within tolerance) and collapse (any tower block > 5 mm below its reference or tilted > 5 deg, permanent). Tower
score = 0 if collapsed, else legally relocated blocks still in place after a 3 s final settle / 30. block_score = mean
over the towers x the contract gate; reward = ratio against the anchor (the oracle's value).

## Layout of tests/
* `towercell/`: config (public numbers and ranges), tower (hidden per-tower draw, MuJoCo bodies), cell (world,
  controller, referee, score), cameras, server (the calls), client (the program's library; byte-copied to the payload).
* `cellkit/`: the program's isolation and wire (see NOTICE.md); `pauser.py` stops the program while the cell works.
* `verify.py`, `test.sh`, `grader_config.json`, `anchor.json`, `numerics_check.py` + `numerics.json`,
  `given_hashes.json`, `setup_payload/` (byte copy of environment/setup/payload/), `hidden/`.
* `review/`: maintainers' tools (nothing runs them at verify): `make_given.py`, `make_sets.py`, `phase_a/` (the physics
  prototype, its oracle and the sanity baselines).

## The oracle
`solution/solve.sh` copies `solution/reference/play.py` and the 256-bit token. With the token the cell answers
`get_state_privileged` (true block poses, load from above, gap to the level above, the referee's levels and slots) and
`contacts_above(i)`; the reference uses them to choose loose blocks, to aim its motions and to keep a pulled block clear
of the level above. It stops when no loose, legal block whose removal leaves at least two blocks in its level is left.
