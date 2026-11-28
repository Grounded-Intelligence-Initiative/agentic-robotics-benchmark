# server_hot_swap_maintenance: maintainers' notes (verify stage only; never shown to the agent)

## 1. What the task is

Two Panda arms swap a top-loading storage server's failed drive caddy and failed fan module. The agent writes a
closed_loop controller (`run.sh`); `verify.py` drives it over the kit's stdio wire (READY / reset / act / close, probe
pairs through `srv_grade.IdleAwareProbeLock`) on the fixed hidden set `hidden/episodes.json`: 10 episodes, one on each
hidden unit of `units.json`. The metric `mean_episode_credit` is the mean of the per-episode credit (`episode.py`,
all or nothing); the reward is the kit's `ratio_rewards` against `anchor.json`.

The four levers the task is built on:
1. **closed loop with probe pairs**: low-level arm commands only (Cartesian steps of an operational-space controller,
   posture, joint steps, gripper); no cell API, no IK service, no "goal reached" signal;
2. **camera-only state with real sensors**: nothing gives the server's pose (+-15 mm, +-3 deg) or any camera's pose
   (overhead mount off by up to 15 mm / 1.5 deg, wrist mounts by up to 4 mm / 1 deg); calibration plates and the arms'
   kinematics are there to identify them; active-stereo depth (noise ~z^2, 1 mm steps, holes); the failed parts show
   only by 2.4 mm LEDs under visors, seen from in front at low elevation (not from the overhead camera), flickering,
   amber steady or blinking, at a hidden brightness under a hidden hall light;
3. **bimanual, long, irreversible**: the lever may not carry its caddy off the rails (the other arm must take the
   body first); the fan bay may not stay empty past a hidden 20-40 s window (the spare must be at hand); a healthy
   drive unseated, a dropped part or a hard push on the server ends the episode at 0;
4. **contact physics**: caddies and fans are free bodies on contact rails (hidden friction); buttons, levers, latches
   and connectors are joints, equality constraints and force laws applied every 2 ms sub-step (`srv_sim.World`).

## 2. Files

- `srv_sim.py` (the cell), `srv_env.py` (observation, action, probe copy, truth), `episode.py` (end, credit):
  byte-identical copies in `environment/commission_service/`.
- `srv_grade.py` (episode loop, probe pairs, budget), `verify.py` (privilege drop, seccomp launcher, numerics and
  render self-tests, verdict), `nonet_exec.py`, `numerics_check.py` + `numerics.json` (written in the task image),
  `robotics_grader/` (the kit, byte-identical to `shared/robotics_grader/`).
- `review/make_sets.py` (development units, hidden candidates, salts), `review/run_local.py` (the grader's loop
  over the real wire without root), `review/select_set.py` (the hidden set), `review/naive_baseline/` (the reference
  planning at the server's nominal pose).

## 3. The reference and the anchor

`solution/oracle/oracle.py` reads the token truth and plans every move from it (its own tool pose included, so the
plan does not depend on which copy of a probe pair comes first); each reply's Cartesian step is measured from the
observed tool pose (so probe pairs differ). The hidden set holds 10 of the first 14 candidates (in index order, the
ones the reference completed over the real wire; `hidden/episode_selection.json` records all 24 candidate runs:
16 completed). `anchor.json` records the reference's measured value on the set.

## 4. Measurements

See `anchor.json` and the pull request; dev notes outside the repository.
