# Grader notes (for reviewers)

- `verify.py` owns the simulator (`cell_server.py`, run by `/opt/sim/bin/python`) and drives the submission's `run.sh`
  over the kit's stdio wire (`robotics_grader`), one simulator process per hidden shift (`hidden/shifts.json`).
- Metric: `part_pickup_score`, computed by `cell_env.py` from the simulator's own state (part in the tote 1.0, held for
  1 s 0.5, anything else 0, -0.5 per bystander moved more than 5 cm), mean over the shift's 20 parts and over the
  shifts. Nothing the controller prints is a scoring input. The reward is the ratio to `anchor.json`.
- Time: each camera frame asked for advances the cell three more 30 Hz steps (a fixed price). The controller's own
  time must stay within the simulated time elapsed plus 60 s, or the shift ends with what it had earned: a pass/fail
  real-time threshold in the manner of `dual_arm_parcel_sorting_on_conveyor`, never a scored time.
- A part that stands still on the chute for 1 s unheld is lost (jam), so a controller cannot stop a part and pick it
  up at rest: it has to be taken while it moves.
- Action: 26 numbers (23 robot command values, 3 camera flags). The match lock perturbs the 61 proprioceptive values.
- The reference (`solution/`) is privileged: `verify.py` attaches ground truth to the observation only when the
  submission holds `.oracle_token`, whose SHA-256 is `oracle_token.sha256`. The token is not in the agent's image.
- `review/test_plumbing.py`: CPU tests of the wire, the packing of frames, the truth gate and the reply parser.
- Physics is PhysX on the CPU (`gm.USE_GPU_DYNAMICS = False`); the GPU renders the cameras. `anchor.json` records the
  measured spread of the reference.
- Camera frames: only the steps that lead to a requested frame are rendered (the action's own step and the three steps
  each frame costs); physics is stepped without rendering otherwise. Depth frames match those of OmniGibson's
  `Environment.step()` path (which renders every step); colour frames are slightly darker, by 3 to 6 of 255 on average.
- Hidden set: `hidden/shift_selection.json` (rule, the 12 candidates' scores, the 6 kept, the second run).
  `review/select_shifts.py` states the rule and reproduces `hidden/shifts.json` and `anchor.json` from two grading runs.
- Each reply has a deadline of 0.1 s (`reply_deadline_s`); three late replies are forgiven in a shift, the fourth ends it
  with what it had earned (the first reply has a 5 s grace). Together with the total budget this is a pass/fail real-time
  threshold in the manner of the race-car tasks' per-tick budget; the verdict reports `late_replies` and `slowest_reply_s`.
- Camera latency: a frame is delivered 3 to 6 steps after it was taken (a hidden constant of the shift), stamped with
  `t_capture`, as in `fault_adaptive_spacecraft_docking` and `dual_arm_parcel_sorting_on_conveyor`.
- Graded shifts are drawn with a secret salt (`hidden/shifts.json`), so a seed does not reproduce one in the lab.
- Isolation at grading: the simulator's temporary files, log and application data live in a 0700 folder of the grader's
  account (`verify.py`); the controller runs deprivileged through the kit. Tripwire: a shift with parts taken and no
  frame requested (and no reference token) is void (`blind`, score 0). The instruction lists what a controller may use
  and declares the rest disqualifying. Not checked on the author's cluster, which has no account separation.
- The line feeds on a schedule of its own (`cell_env.py`: `release_at`, `staged_at`, a hidden random stream): a part every
  9 to 13 s, whatever the controller does. An event is over at the latest when the next part goes on the chute.
- Grasping is physical for a controller (finger friction only). The reference's token gives it, besides ground truth and
  what is coming, OmniGibson's assisted grasping (the simulator attaches a part held by both fingers for 0.3 s). The
  anchor is therefore the score of a controller with these three privileges; say so when comparing.
