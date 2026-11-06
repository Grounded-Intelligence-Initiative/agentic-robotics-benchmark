# fault_adaptive_spacecraft_docking: grading notes

Notes for maintainers and reviewers. The agent-facing rules are in `instruction.md`; the simulator is `env.py` in
this folder, byte-identical to the agent's development copy (`environment/setup/payload/spacecraft_docking_env.py`)
and to the reference's copy (`solution/privileged/spacecraft_docking_env.py`). This folder is the verify stage: it is
absent while the agent works and root-only at verify time.

## The owner's direction (2026-11-05)

The anchor is the measured score of a reference that plans from ground truth the agent never gets, on a fixed set of
hidden episodes it completes, so that the anchor sits near full and a controller that identifies the plant from its
observation has real headroom. The task author's identifying controller stays as the non-privileged reference. What
the agent sees and must do is unchanged, apart from the instruction's account of the fixed set, the network and the
time budget.

## The fixed episode set and how it was selected

`hidden/episodes.json` holds the hidden episodes: 48 seeds of 64 bits (each at least 2^32, so none is a practice
seed: those are below 2^13), each with the sha256 of the world `env.reset(seed)` builds, and the hidden key of the
match-lock streams. The same 48 seeds run in the same order for every submission. `verify.load_episodes` checks the
format, the key, the seeds and every world digest before anything else runs; any defect is a task error whose message
names no value of the set. The digest formats every float with 13 significant digits (`format(v, ".12e")`), so a
last-bit libm difference on another host cannot fail it, while any change to env.py's draws does.

The selection was pre-registered (`review/select_set.py`, the rule in its docstring):

1. **Freeze.** The reference (`solution/privileged/oracle.py` and its simulator copy, `solution/solve.sh`) and the
   grader (`verify.py`, `env.py`) were committed first (08ccda6); their sha256 are in the set's `source`. Since then
   only `verify.py` changed, after the set was made: its network guard fails closed (The network guard, below). The
   episode loop the selection ran is the same in effect, and the reference's verify reproduced the anchor to the last
   bit afterwards (The anchor). `select_set.py run` and `make` compare against the hashes recorded at the draw, so they
   refuse to extend this selection: a new selection starts with a new draw.
2. **Draw.** 64 candidates from `os.urandom` (8 bytes each) and the probe key from `os.urandom(32)`, written to
   `selection.json` before any candidate ran (2026-11-05T16:31:23Z).
3. **Run.** Every candidate through `verify.run_episode`: a token run of the `solve.sh` submission, a fresh program per
   candidate, its own lock stream, the full 400 steps, in a CPU container of the task image.
4. **Keep.** A candidate is kept when it docked without an abort; the first 48 kept, in drawn order, are the set.

62 of 64 docked. Kept: draws 0-28 and 30-48. Dropped: draw 29 (a collision at step 395). Draws 49-63 ran after the
cutoff and are not used (draw 58 among them timed out). `hidden/selection.json` records every candidate: its outcome,
the policy the reference chose, how many of the reference's 266 policies dock from its planning state, and its world
(the hidden draws below).

**Hollowing check.** The kept 48 span every hidden range of `instruction.md`: the first event at steps 59-103
(range 58-103), the re-key's earliest step 108-128 (108-128) and deadline 148-165 (148-165), its proximity 1.181-1.299 m
(1.18-1.30), all four degraded scales across their ranges, the boom's amplitude 0.100-0.157 m (0.10-0.16) and
frequency 0.182-0.278 rad/s (0.18-0.28), the start radius 2.870-3.648 m (2.85-3.65) and bearing from the port
-1.444..1.414 rad (+-1.45), both slew rates to within 0.002 rad/s of their limits. The outward authority the re-keyed
plant has at the docking pose in the initial swirl pattern (the largest outward acceleration with no tangential and no
yaw acceleration, the bias included) runs from -0.045 to 0.228 m/s^2 in the kept set, and 38 to 64 of the 64 swirl
patterns have outward authority. The one drop lies inside all of these ranges (`review/select_set.py report`).

## One lock stream per episode; determinism

Each episode's match lock (`ProbeLock`: a probed step with probability 0.06, three draws for the copy and one for the
order) draws from `seed_rng(entropy=(key << 64) | seed)`: the same probes for the same submission every time, whatever
ran before. Nothing in a verify draws from OS entropy or the clock (only the per-request deadlines are wall clock).

The privileged reference is deterministic on the set: the selection runs, three fresh-container verifies
(`review/regrade.sh`; the third on 2026-11-06, with the network guard failing closed) and the in-process replays
(`review/oracle_batch.py`) gave the same 48 episode qualities to the last bit, mean 0.9835537203921798. It is pure
Python (no numpy, so no BLAS kernels and no tie-breaking in `argpartition`); on 20 dev seeds its replies under host
CPython 3.12.11 and the image's 3.12.14 are identical, and a program serving 48 episodes in a row answers exactly as a
fresh one per episode. Before 3.12 the builtin `sum` is not compensated, so fuel's last bit differs; the grader
always runs 3.12 or newer.

## What the set fixes, and what it does not

A seed fixes every hidden parameter of its episode: env.py draws them all in `reset`, and `step` never draws. After
the reset, randomness is only the sensor noise and the dropouts in `_observe`. Before the re-key the blackout
schedule is fixed by the seed, so every controller meets the same noise; the re-key fires when the controller comes
close enough, and its own blackout shifts the noise stream, so after the re-key two controllers see different noise on
the same seed. The selection therefore vouches for the reference's own trajectory: another controller meets the same
world, not the same episode. The reference reads no observation, so its success does not depend on the noise.

## The privileged channel

`solution/solve.sh` installs the privileged reference and the reference token (`.oracle_token`; this folder keeps
only its sha256, `hidden/oracle_token.sha256`). `verify.token_state` reads the token before any submission code runs: a
regular file, opened without following a link and without blocking, at most 4 KB, compared by `hmac.compare_digest`.
A token that is there but wrong (a wrong value, a link, a directory, a FIFO) is a mismatch and is otherwise ignored.
The metrics record `privileged_mode` and `oracle_token_mismatch`; nothing agent-visible names the channel (a repository
test checks the instruction and every file under `environment/`).

With the token, every observation is `{"vector": <the 16 numbers>, "privileged": {"format": ..., "state": {...}}}`.
The block (`verify.privileged_block`) is the simulator's current truth, named as env.py names it: every quantity
`env.step` reads that is in effect now (`PRIV_STATE`, `PRIV_ROUTING` once the first event has fired, `PRIV_CONSTANTS`).
It never carries the pre-drawn future (the event times, the re-key's plant, its slew, the encoder polarities) nor
anything of the observation model; each future value reaches the block when it takes effect, so a perfect identifier
would know it one step late, as the reference does. Building the block reads attributes only: it draws nothing and
changes nothing. A probed step's copy shifts the block's true x, y and yaw by the same draws as the camera pose (the
rule of hazard_bay_build), so a program that reads the block answers the copy on its own. Without the token the
requests are byte for byte what they were before the channel existed (the repository test replays the old loop).

## The network guard

The verifier sandbox is provisioned with open egress, and once the set is public a controller could fetch it and
identify its episode from the first observation. So the controller runs behind `nonet_exec.py` (byte-identical to the
mechanism box's): no_new_privs, a seccomp filter under which only AF_UNIX sockets open, the agent account, and a fixed
environment (PATH, HOME, LANG). A self-test before episode 0 checks that the filter holds. Where it cannot be
installed, the verify fails closed: a task error ("the grading environment cannot block the controller's network")
before any controller starts, so nothing is graded with the network open. A verdict therefore always records
`egress_guard_seccomp` 1. Measured with a spy submission in a fresh container: AF_INET and AF_INET6 sockets (even to
127.0.0.1) fail with EPERM, AF_UNIX sockets and socketpair work, `/opt/ale` is unreadable, and the environment holds
only PATH, HOME and LANG (plus what bash adds).

Until 2026-11-06 a detect fallback (inherited from hazard_bay_build and the mechanism box) let a controller run without
the filter while the grader looked at the account's internet sockets every 50 steps. It never looked before READY, at
a reset or between episodes, and it skipped closed and unconnected sockets: in a review spy, a controller that fetched
bytes before READY and closed the socket went unseen. The fallback is gone; a repository test checks that a verify
without the filter is a task error and that no controller starts.

## The two references

**The privileged reference** (`solution/privileged/oracle.py`) plans from the block alone. It rebuilds the plant from
each block (`Replica`: env.py's own class with no observation and no event to come), tells the real request from a
probe copy by exact prediction (only a verified request commits anything, so the copy gets the same law on its shifted
pose and never touches the plan), stages at r = 1.60 m on the port's radial line before the re-key, pins the relay
trigger once per event, and allocates the wrench exactly on the next step's true plant. Two steps after the re-key it
starts simulating, from the state eight steps ahead, a fixed family of 266 policies (descent delay, hold gap, speed, a
load penalty on one nozzle, a relative-speed cap, a yaw offset) to the episode's end, and commits the first that docks. Planned and executed outcomes are the same
computation. On 2000 dev seeds (`review/oracle_batch.py`, random.Random(20261105)) it docks 1943 (97.15%), the probes
pass at 0.9998, planned equals executed in all 2000, and its slowest request (a plan) took 5.1 s on a loaded host and
2.9 s in a 2-CPU container. The prototypes' 80 policies alone dock 95.45% of those seeds; a 0.6 rad waypoint in the
far field (the prototypes') let 96 of 2000 re-keys fire late.

**The non-privileged reference** (`solution/reference/solver.py`, the task author's controller, bytes unchanged)
works from the observation alone. `solution/reference/install.sh [dir]` installs it. On the fixed set in a fresh
container it scored 0.53997 (26 of 48 docked, 4 collisions; the lock 794 of 1064 probes), ratio 0.549, reward 0.56.
Its trajectory depends on the probe schedule (it keeps whichever of a repeated step's answers fits its estimate best),
so its score repeats only with the pinned lock streams and numpy 2.5.4 (`environment/requirements.txt`); across numpy
builds it flipped 7 of 480 outcomes through tie-breaking in `argpartition`.

## The anchor

`anchor.json`: 0.9835537203921798, the lowest of the two fresh-container verifies (identical), `full_at` 0.98. Reward
by docks on the set, with the anchor's per-episode qualities and the lost episodes the best ones, scored 0: 48 docked
1.00; 47 0.999 (rounds to 1.00); 46 0.978; 44 0.935; 40 0.850; 36 0.765; 24 0.509; 12 0.254; 0 0. `full_at` 0.98 is the
margin for one episode flipped by a last-bit libm difference on another host, and no more. Before: 0.4580 with
`full_at` 0.75 (16 docks of 48 earned full reward). The verify of 2026-11-06, with the guard failing closed, measured
the anchor again to the last bit (reward 1.0, ratio 1.0, 48 of 48 docked, the lock 777 of 778).

## What remains undocked even with the truth

Draw 29 of the selection, and about 3% of random draws (57 of 2000 dev seeds): none of the 266 policies docks from the
planning state, although the re-key fires on time. These are near-field failures: inside r = 1.47 m the plume
attenuates the nozzles that push outward and turns most of their thrust into a swirl side force, the hold's own loads
charge the swirl cells and can flip one mid-hold, and in some swirl patterns the re-keyed plant cannot produce outward
acceleration at the docking pose at all (draw 29's initial pattern: -0.006 m/s^2). Whether some controller could dock
them (a planner that programs the cells' flips, say) is not proven.

## Review tools (`review/`, never run by the grader)

- `oracle_batch.py`: the dev batch in process (dock rate, probes, planned vs executed, verification failures, timing).
- `run_episode.py`: grader-exact episodes of a submission folder (its run.sh started as the current user), with an
  in-process comparison and a one-process mode (reset hygiene).
- `select_set.py`: the pre-registered selection (`draw`, `run`, `make`, `report`).
- `regrade.sh`: one verify in a fresh CPU container of the task image, the separate verifier's layout, no network.
