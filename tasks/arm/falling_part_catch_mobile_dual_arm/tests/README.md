# Grader notes (for reviewers)

- `verify.py` owns the simulator (`cell_server.py`, run by `/opt/sim/bin/python`) and drives the submission's `run.sh`
  over the kit's stdio wire (`robotics_grader`), one simulator process per hidden shift (`hidden/shifts.json`).
- Metric: `part_pickup_score`, computed by `cell_env.py` from the simulator's own state (part in the tote 1.0, held for
  1 s 0.5, anything else 0, -0.5 per bystander moved more than 5 cm), mean over the shift's 20 parts and over the
  shifts. Nothing the controller prints is a scoring input. The reward is the ratio to `anchor.json`.
- Time: the cell does not wait for the controller. A reply within one 30 Hz period is applied on the next step; each
  further period it took is one more step the cell runs first with the previous command held (`late` in
  `cell_env.step`). Each camera frame asked for advances the cell three more steps (a fixed price). Time is never a
  scoring input. The author's decision (2026-11-25): the world must not pause while the controller thinks, as on a
  real line; a reply's time is the controller's CPU time (`CpuMeter`, the
  chute task's), so that other tenants of the grading host do not change what a controller meets; a controller close to
  33 ms of CPU per reply still scores differently on faster and slower processors.
- The parts are rods in sponge sleeves (`props.write_rod`: contact friction 1.6, combine mode max), hanging upright,
  20 different ones per shift (radius 12.5 to 30 mm, length 0.28 to 0.42 m, mass 0.05 to 0.40 kg).
- The feeder holds every rod for its own time, drawn uniformly between 1 and 4 s (`shift.py`): there is no rhythm to
  learn, the release has to be seen (the author's decision, 2026-11-25; a frontier-model controller had learnt the
  earlier per-shift rhythm and scheduled its grasp from it).
- A rod touched by a finger while the feeder holds it, or before it has fallen 0.25 m (`MIN_FALL` in `cell_env.py`),
  is lost (`early_touch`), so a controller cannot take a rod out of the feeder at rest: it has to be caught in free
  fall, at 2.2 m/s or more. A rod that reaches the floor is lost. How narrow this is: the reference, which reads the
  rod's true state, catches 20 of 20 when it commands the grasp on the computed step, loses 11 of 20 to `early_touch`
  when it commands one 30 Hz step earlier and 20 of 20 when two steps earlier (same shift, 2026-11-25).
- Action: 26 numbers (23 robot command values, 3 camera flags). The match lock perturbs the 61 proprioceptive values.
- The reference (`solution/`) is privileged: `verify.py` attaches ground truth to the observation only when the
  submission holds `.oracle_token`, whose SHA-256 is `oracle_token.sha256`. The token is not in the agent's image.
- `review/test_plumbing.py`: CPU tests of the wire, the packing of frames, the truth gate and the reply parser.
- Physics is PhysX on the CPU (`gm.USE_GPU_DYNAMICS = False`); the GPU renders the cameras. `anchor.json` records the
  measured spread of the reference.

## The cell's state machine (`cell_env.py`, `_tick`)

One shift is one continuous run of the cell; each of its 20 parts goes through the same states, and the next part is
not staged until the previous one has ended.

| state | entered when | leaves when |
|---|---|---|
| `gap` | a part's event ended (or the shift starts) | the feed's timetable says so (`staged_at`, fixed before the shift: releases 7 to 11 s apart, nothing the robot does moves it); after the 20th part: `done` |
| `staged` | the part is placed at the feed point and held still | its release step (drawn from the shift's hidden rhythm): `live`; a finger on the part: `lost/early_touch` |
| `live` | the part is released | one of the endings below |

Endings of `live`, checked every 30 Hz step on the simulator's own state: `placed/in_tote` (rests in the tote, no
finger on it; credit 1.0), `lost/hit_floor`, `lost/early_touch` (first finger contact before the part has fallen 0.25 m), `lost/time_limit`, `lost/next_part` (the next part appeared: this one's time is up wherever it is). A lost part that was held for 30 consecutive steps (both fingers of one gripper on it, clear of
the feed point and the tote, at rest relative to the hand) keeps credit 0.5. Lost parts are cleared away, placed ones
stay in the tote, so the tote fills over the shift. The shift ends early only if the controller falls behind real time.

- Grasping: physical for a contestant (finger friction alone), assisted for the reference only. The oracle token, sent with `reset`, turns on the simulator's assisted grasp (`cell_env.Cell(assisted=True)`: a part touched by both fingers of a hand for 0.3 s is attached to it); without the token the robot is built with `grasping_mode="physical"`. The author's decision (2026-11-26), as on the chute task.
- Finger pads: friction 2.0 on the finger links (`cell_env.FINGER_FRICTION`), so that a firmly squeezed part is held without the simulator's attachment. Either hand or both may be used. The author's decisions (2026-11-26).
- The rack (2026-11-26, the author's decision after a frontier model scored 0.79 raw against the reference's 0.90 with one
  rod hanging: the hand waited under it and the catch was timing only). Ten rods hang at once (`cell_env.RACK`, two rows
  of five, 0.10 m pitch); each event lets go the rod at one place, drawn with the shift's hidden weights (Dirichlet 1.5,
  rng [51, seed]); the place is refilled when the next event starts, with rods that are still hanging when the shift
  ends (nine more objects). A finger on any hanging rod during an event ends it (`early_touch`). The reference is told
  which rod goes (truth carries the current event's rod), which is its privilege; it waits 0.45 m below the rod's
  middle and moves by one bounded Cartesian servo step per reply (no global inverse-kinematics search), with a height
  guard so that a carried rod clears the hanging ones. Its wire (`solution/privileged/reference_main.py`) advances the
  controller once per step on the true joint readings (`truth["proprio"]`) and echoes the difference between the
  readings it was sent and the true ones on the idle left arm, so the match lock's probe changes its reply without
  changing what it does. Statements above about "every rod hangs at a different place"
  and a feeder height of 1.40 to 1.55 m describe the earlier version: the plate is now at 1.55 to 1.70 m.
