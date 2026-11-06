# fault_adaptive_spacecraft_docking: grading notes

Notes for maintainers and reviewers. The agent-facing rules are in `instruction.md` and in the plant model it points
to (`environment/setup/payload/given/model.md`, staged at `/home/user/given/model.md`); the simulator is `env.py` in
this folder, byte-identical to the reference's copy (`solution/privileged/spacecraft_docking_env.py`). The agent gets
no copy of it. This folder is the verify stage: it is absent while the agent works and root-only at verify time.

## The owner's direction (2026-11-05)

The anchor is the measured score of a reference that plans from ground truth the agent never gets, on a fixed set of
hidden episodes it completes, so that the anchor sits near full and a controller that identifies the plant from its
observation has real headroom. The task author's identifying controller stays as the non-privileged reference. What
the agent must do is unchanged, apart from the instruction's account of the fixed set, the network and the time budget.

The same day the owner removed the practice tools, by the rule applied to PR #31's round 3: a spacecraft with unknown
faults is docked zero-shot, so the agent gets neither the grader's simulator nor a practice grader on visible seeds of
the graded distribution. Until then the payload held a byte-identical copy of `env.py` and `practice_grader/grade.py`
(five visible seeds), and the instruction called the simulator's source the complete statement of every range. Now
the payload holds one document, `given/model.md` (The agent's documents, below).

## The fixed episode set and how it was selected

`hidden/episodes.json` holds the hidden episodes: 48 seeds of 64 bits (each at least 2^32; the former practice seeds
were below 2^13), each with the sha256 of the world `env.reset(seed)` builds, and the hidden key of the
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

62 of 64 docked. Kept: draws 0-28 and 30-48. Dropped: draw 29, undocked by all 266 policies from its planning state,
re-key on time; cause not identified (its episode ended in a collision at step 395: The two references). Draws 49-63
ran after the cutoff and are not used (draw 58 among them timed out). `hidden/selection.json` records every candidate:
its outcome, the policy the reference chose, how many of the reference's 266 policies dock from its planning state,
and its world (the hidden draws below).

**Hollowing check.** The kept 48 span every hidden range of `instruction.md`: the first event at steps 59-103
(range 58-103), the re-key's earliest step 108-128 (108-128) and deadline 148-165 (148-165), its proximity 1.181-1.299 m
(1.18-1.30), the four degraded scales 0.352-0.497 (0.35-0.50), 0.561-0.697 (0.55-0.70), 0.457-0.596 (0.45-0.60) and
0.651-0.796 (0.65-0.80), the boom's amplitude 0.100-0.157 m (0.10-0.16) and frequency 0.182-0.278 rad/s (0.18-0.28),
the start radius 2.870-3.648 m (2.85-3.65) and bearing from the port -1.444..1.414 rad (+-1.45), the target's slew rate
before the re-key -0.1229..0.1228 rad/s (+-0.125: 0.0021 and 0.0022 rad/s short of the two limits) and after it
-0.0437..0.0446 rad/s (+-0.045: 0.0013 and 0.0004 rad/s short), with the smallest speeds 0.0612 and 0.0203 rad/s
(0.060 and 0.020). The outward authority the re-keyed plant has at the docking pose in the initial swirl pattern (the
largest outward acceleration with no tangential and no yaw acceleration, the bias included) runs from -0.045 to
0.228 m/s^2 in the kept set (negative in 9 kept draws and undefined in one, all of which dock), and 38 to 64 of the 64
swirl patterns have outward authority. The one drop lies inside all of these ranges but one: its moderate scale of
the first event (`secondary_failure_scale`, 0.551) is below the kept 0.561-0.697, though inside its range 0.55-0.70;
that scale acts from the first event (step 89 in draw 29) until the re-key replaces it (step 115), before the
reference's planning state. None of the other quantities tells the drop apart (`review/select_set.py report`).

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
load penalty on one nozzle, a relative-speed cap, a yaw offset) to the episode's end, and commits the first that docks.
Its invariant is that a planned dock is an executed dock: the planned rollout and the episode are the same
computation, so a committed policy that docks in its rollout docks in the episode at the same step with the same
quality. When no policy docks, the reference keeps the one with the best quality among rollouts that stop as soon as
a dock can no longer be completed in time (about step 371): that fallback's last steps are never simulated, and the
episode can still end in a collision or an escape. On 2000 dev seeds (`review/oracle_batch.py`,
random.Random(20261105)) it docks 1943 (97.15%): every planned dock was executed at the planned step with the planned
quality, none of the 57 episodes without a planned dock docked, and 15 of those 57 ended in a collision or an escape
(draw 29 of the selection ended so too: fallback policy 112, a collision at step 395). The probes pass at 0.9998, and
its slowest request (a plan) took 5.1 s on a loaded host and 2.9 s in a 2-CPU container. The prototypes' 80 policies
alone dock 95.45% of those seeds; a 0.6 rad waypoint in the far field (the prototypes') let 96 of 2000 re-keys fire
late.

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

## What the reference does not dock

Draw 29 of the selection: undocked by all 266 policies from its planning state, re-key on time; cause not identified.
About 3% of random draws are undocked the same way: on 57 of the 2000 dev seeds none of the policies docks from the
planning state, although the re-key fires at its arming step in all 57; they have not been examined one by one.
Whether another controller could dock them is not known.

## The agent's documents

The agent gets `instruction.md` and `given/model.md`, nothing else: no simulator, no grader of its own, no visible
episode. `given/model.md` states the model `env.py` simulates, for a capable agent to write the controller and to
rebuild a faithful simulator for rehearsal from the text alone: what an episode draws (every distribution, as env.py
draws it), the thrusters (the nominal geometries and the installation errors), the bus (routing, both events, the
relay, the valves, the stride), the target, the boom and the port, the plume (the field, the swirl cells and their
thresholds), one step in env.py's order of operations, the observation (all 16 fields, the delay, the noise, the
dropouts and the blackouts, the IMU's mounting and remount), docking, the end and the quality, and an episode on the
wire (the probe copy, one verbatim `act` line from a dev seed). It names no seed of the set and nothing of the channel
(the repository test checks every agent-visible file). Its formulas are written in the order env.py evaluates them,
and it names the two builtins whose rounding matters (`hypot` for a length, the builtin `sum` of the six duties), so
a simulator that follows the page literally computes env.py's numbers.

The check is in `review/`. `docplant.py` is a plant written from the page alone (it imports nothing of the task, and
`PAGE_SHA256` pins the page it was written from). `doc_vs_env.py` steps it beside `env.py` with the same commands,
given each episode's draws named as the page names them (read off the numbers env.py's `reset` drew, through the
page's ranges, so a range that differs shows) and each observation's raw noise numbers, and compares every attribute
of env.py's simulator, every observation field and every step's quality, end and errors with `==` after every step.
Run on 2026-11-06 (CPython 3.12.11): over 400 open-loop dev episodes (`open`: five kinds of scripted commands, half
moved inside the plume's field; 72,696 steps with 1136 swirl flips, 1194 and 1308 reversals of the two rotors, 5995
two-slot strides, 29 proximity and 180 deadline re-keys, 2143 dropouts, 46 timeouts, 129 collisions and 225 escapes)
and the reference's own episodes on 100 dev seeds (`reference`: random.Random(20261106), the dev lock key; 27,853
steps, 94 docked and 6 timed out), every field agreed to the last bit after every step, so every end came at the same
step with the same quality. The page's table of draws has the distributions of env.py's worlds (`draws`): on two sets
of 50,000 dev worlds against as many episodes drawn from the table with `random.Random`, all 92 tests (a two-sample
Kolmogorov-Smirnov test for each of the 63 continuous draws, a chi-square homogeneity test for each of the 29 discrete
ones that take more than one value) pass at a family-wise 1% (the smallest p-values 0.0019 and 0.017, the Bonferroni
line 0.00011), and every discrete draw takes the same values on both sides. The page's `act` line is env.py's
(`act`; its dev seed is not a candidate of the selection). The repository test (`PagePlantTests`) runs ten of the
open-loop episodes, two of the reference's and the `act` line on every pull request, and fails as soon as the page or
`env.py` changes, until the check has been run again and the pins (`docplant.PAGE_SHA256`, `doc_vs_env.ENV_SHA256`)
moved.

`env.py`'s own text still describes the agent's copy (the module docstring's "An identical development copy is staged
for the agent" and "this source can be shared with the agent", `_RNG`'s "The agent holds this code"): the file is
frozen (its sha256 pins the set and the reference's copy), so those lines stay, and this README is the current
account.

## Review tools (`review/`, never run by the grader)

- `oracle_batch.py`: the dev batch in process (dock rate, probes, planned vs executed, verification failures, timing).
- `run_episode.py`: grader-exact episodes of a submission folder (its run.sh started as the current user), with an
  in-process comparison and a one-process mode (reset hygiene).
- `select_set.py`: the pre-registered selection (`draw`, `run`, `make`, `report`).
- `regrade.sh`: one verify in a fresh CPU container of the task image, the separate verifier's layout, no network.
- `docplant.py`: a plant written from `given/model.md` alone: the page's table of draws, the plant, the observation.
- `doc_vs_env.py`: that plant beside `env.py`, compared with `==` after every step (`open`, `reference`, `act`), and
  the page's table of draws against env.py's worlds (`draws`) (The agent's documents).
