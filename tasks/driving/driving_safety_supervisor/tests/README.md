# driving_safety_supervisor: grading notes for maintainers

The grader (`verify.py`) runs as root in a separate verifier sandbox. The agent never sees this folder. This note records how the hidden set, the reference expert and the anchor came about, and how to redo each step.

## Owner direction (2026-11-06)

- The token-privileged reference expert must complete every graded episode: no collision, no hazard hit, no red light, no abort, the route completed. The graded episodes are therefore a fixed hidden set, selected once by a pre-registered rule, and the anchor is measured on it.
- The staging fixes S1 to S4 (below) are in. They make every graded episode completable and remove no difficulty.
- The excused-wait claim for walkers is deleted. It would only have removed a pace penalty that the expert pays too, so it could not raise the difficulty. The only excused wait is time stopped at a red or yellow light, up to 90 s an episode.
- The pace formula stays as it is.
- `full_at` follows the design's rule: 0.98 if the verify repeats the selection to 4 decimals, F1 otherwise.
- Agent-visible docstrings do not name the oracle token. The practice `"privileged"` feature stays.
- A test runs once: one selection run per candidate, and one anchor verify.

## The fixed hidden set

`hidden/episodes.json` has the format string `driving_safety_supervisor fixed set 1` and holds 24 episodes. Each episode has an id, a town, a variant, a kind, a seed, a distractor count, a lock key and a layout digest. The episodes are in grading order, which is `drive_variants.graded_cells` over `grader_config.towns`: Town04 (7 episodes), Town05 (10, three of them shoulder_pass), then Town10HD (7).

What `verify.load_set` checks:
- the format string;
- one episode per cell, in grading order;
- each kind matches the one its variant stages;
- seeds are distinct integers (not bools) in [2^32, 2^62);
- distractor counts are 0 to 2;
- lock keys and layout digests are 64 lowercase hex digits;
- lock keys and ids are distinct.

Any defect is a task error, and its message names no value of the set.

At every reset, `LayoutChecked` compares the bridge's `truth["layout"]` with the recorded digest. The digest covers the town, the route's lane keys, the spawned blueprints in spawn order, and the generator's state after every draw of the reset. A mismatch is a task error, never a silently different episode.

Each episode draws its probes from its own stream (`seed_rng(entropy=lock_key)`). Specs travel only over root's pipe to the bridge, and the grader's log prints no seed.

### Selection (`review/select_set.py`; the rule is its module docstring)

1. **Freeze.** The following files were committed first: the bridge (both copies), `drive_protocol`, `drive_variants`, `episode_score`, `verify`, `sim_client`, the expert, `footprints.json` and `solve.sh`. They are at `edeb0fb` (the freeze commit is `c74c883`). `draw` refuses to start on a tree where any of them differs from HEAD; `run` and `make` refuse once any of them differs from the draw's record. The repository test also fails when any of them changes, because the set would then have to be selected again.
2. **Draw** (2026-11-06T20:55:23Z). 96 candidates, four rounds of the 24 cells, all from `os.urandom`:
   - the seed is 8 bytes, little-endian, mod 2^62, drawn again below 2^32 or on a repeat;
   - the distractor count is one byte mod 3;
   - the lock key is 32 bytes.

   `hidden/selection.json` was committed (`64ba8cd`) before any candidate ran.
3. **Run.** Each candidate ran once, exactly as graded. `review/run_episode.py` calls `verify.grade_episode` in a container of the task image. The run is a token run of the `solve.sh` submission, through netguard, with the bridge running as `carlasim` from `/run` and the candidate's own lock key. There is one simulator per container, and the world is reloaded before every episode. Round r runs only the cells that do not have a kept candidate yet.
4. **Keep.** A candidate is kept if all of these hold:
   - termination is `completed`;
   - no abort;
   - walker, vehicle and static collisions are 0, and hazard collisions are 0;
   - red lights are 0;
   - the box clearance to every actor is at least 0.15 m;
   - the timeout clock (time minus excused time) is at most 0.85 of the time limit.

   Pace, harsh brakes and the score are not part of the rule.
5. **Result.** 25 candidates ran: 24 kept, 1 dropped, 71 not run.
   - c003 (Town04 overtaking_oncoming) was dropped on the clearance clause. The oncoming overtaker came 1.09 m into the lane, the top of the drawn range of 0.8 to 1.1. The nudge left 0.118 m of box clearance. There was no contact, and the episode completed at 0.9722.
   - Its cell's next candidate, c027, was kept.
   - The kept scores range from 0.7358 to 0.9800, with a mean of 0.9147.
6. **Hollowing check** (`select_set.py report`): no flag. The thresholds are:
   - no variant drops more than 1/3 of the candidates it ran (overtaking_oncoming dropped 1 of 4);
   - no cause is shared by two drops;
   - no value of a categorical staging parameter holds two drops.

   The kept staging parameters cover the run candidates' ranges in every variant. The one exception is the dropped intrusion: the kept values span 0.91 to 1.06, the run values 0.91 to 1.09.
7. **Replacement rule.** A kept episode that fails the keep rule in an anchor verify is replaced by its cell's next kept candidate. A second such failure is an expert defect: fix it, freeze again and draw a new selection. In the one anchor verify, every episode met the recorded clauses. The grader does not measure clearance, so that clause could not be checked there.

`hidden/selection.json` holds every candidate: its draw, its run (truth, staging record, score, layout, probes, clearance, speed limits, slowest reply, wall time), the clauses it failed, the cause found, and its outcome (kept, dropped or not run).

## Determinism

- **Bridge.** Synchronous settings are applied before every load. The world is reloaded for every episode. Physics substeps are explicit: 0.01 s, at most 10 per step. Collisions and camera frames are taken tick by tick on the main thread, with exactly one camera frame per tick. The layout digest is computed at every reset. The old code carried state between episodes: run first and third in one process, the same episode differed from tick 0. This one does not: the two runs differ no earlier than two fresh processes do (`review/determinism.py`).
- **Residual.** CARLA's vehicle physics splits by one float ulp about 10 ticks into an episode, even in a fresh process, and diverges from there. Verifies are therefore not bit-identical.
  - The selection runs and the anchor verify agree on 11 of 24 episode scores to 4 decimals. The rest differ by at most 0.0252 (mean 0.0072).
  - In development, a repeat of 30 episodes moved scores by a mean of 0.011 and at most 0.080.
  - Layouts are identical across processes and GPUs.
- **Pinning.** Neither the CARLA tarball nor the requirements are pinned. A rebuilt image that lays an episode out differently becomes a task error through the digest, but drift in physics or the planner would be silent.

## Staging changes (`drive_variants.py`, grader only)

- **S1 dropped_cargo.** The truck stays on its handbrake until it has stood still for 1 s and the car is within 30 m. Every piece is drawn at staging. Pieces fall on a scripted, physics-off path and rest inside the car's lane, so the next lane stays open.
- **S2 crowd_against_light.** A turner who has not reached the pavement within its out-distance divided by its speed, plus 2 s, is put back on its spot. Crossers are marked done once across.
- **S3 reversing_car.** `left_free` walks the left lane from 60 m behind to 30 m ahead, across road ids. For an oncoming lane it also refuses while a car there would arrive within 8 s.
- **S4 shoulder_pass.** The warning sign stands 0.7 to 0.9 m left of the lane centre, as the wrecks do. A centred trailer, whose measured collision half-width is 1.436 m, left too little room on the narrowest shoulder.
- **S5.** Every staging records what it drew in `truth["staging"]`, which never reaches the wire.
- **S6.** Kerb props and pavement people stand beyond the road's real edge, never on a shoulder or parking lane.

## Grader fixes made just before the freeze (both bridge copies)

- **Red lights count once per crossing.** The Tracker used to count a red light on every tick the car moved through the junction while still in the light's trigger volume: 12 for one run. A light now counts once for the crossing. The score's ×0.7 per red light (up to 3) uses this count.
- **`lights` in the privileged block.** The block lists the traffic lights whose stop line lies on the route block's stretch, each with its stop point and state. The planner sees a light only a few metres before its line, too late at the expert's pace: in d7-crowd_-0 (a dev seed) the next junction's light, inside the route's tail, was run. The block is computed only for a privileged spec. Nothing a graded supervisor sees changes, and the layout digest does not involve it.

## The reference expert (`solution/solver.py`)

The expert submits the oracle token (`solution/reference/oracle_token`; its sha256 is in `oracle_token.sha256`), so every observation carries the bridge's ground-truth block. The block contains:
- vehicles within 250 m, with their reversing lights;
- walkers and props within 80 m;
- the route's lane geometry;
- the lights on the route;
- `red_light` and `speed_limit`.

The expert never looks at the camera. It reacts to the probe's pasted road user, which the probe copy of the block lists as id −1, so the probe lock holds.

The expert is pure Python with no randomness and no clock. Its module docstring and constants describe how it drives:
- probe bookkeeping that does not depend on the order of a probe pair;
- a swept-path rollout of the car's lagged bicycle model against measured footprints (`solution/footprints.json`);
- feasible ramps;
- re-checking and turning back;
- handling of backing and stopped cars, and of walkers;
- gearbox-aware pace at the posted limit.

Just before the freeze it gained one more rule: it stops at the line of a red light on the route ahead, and of a yellow one when 5 m/s² is enough at the moment the light turns yellow.

## Anchor

`anchor.json`: 0.9143304645520832 with `full_at` 0.955. It comes from one verify, the separate verifier's way (`review/regrade.sh`, run `pr24t2-g4-spy`, 377 s): 24 of 24 completed with no infraction, and the lock held (27 of 62).
- **Why `full_at` is 0.955.** The run is not deterministic (see above), so `full_at` is F1 = ⌊200 · (1 − max_k s_k / Σ s_k) / 0.995⌋ / 200. Losing any one episode still rounds to reward 1.00; losing two gives about 0.95.
- **Why the reference stays below 1.** Completion ends at 97 %, pace counts from a standstill, and every full stop costs pace and a harsh brake. The ratio absorbs this, because every submission meets the same episodes.

The same verify ran the leak audit's spy beside the expert, as the agent's account under netguard:
- **Refused:**
  - every AF_INET and AF_INET6 socket, to the internet, to CARLA's 127.0.0.1:2000, to the Traffic Manager on 2050, to the other listening ports and to ::1;
  - raw ICMP, packet and vsock sockets;
  - name lookup;
  - every read of `/opt/ale/tests`, `/run/drive-bridge-*` (the stagings) and `/tmp/drive-simlog-*`;
  - the environ, maps, mem and fd of the root and `carlasim` processes.
- **Readable:**
  - an AF_NETLINK route socket opened; netguard denies the IP families, and nothing listens on netlink;
  - the guard's own folder;
  - process command lines, none of which holds a seed, lock key, digest or the token's digest.

`ptrace_scope` was 1.

## What stays open

- **CARLA server crashes.** The server can die (signal 11) in a reset late in a long server life; it was seen at the 31st reset. `sim_client` then starts a new bridge, which builds the same episode again. A server that dies during an episode is still a task error. A verify is 24 resets.
- **Staging physics.** In a development run, a crowd walker came at the stopped car at 28.7 m/s, a physics launch. No candidate of the set showed this.
- **Overtaking near the top of the range.** The expert's nudge leaves little room for an oncoming overtaker near the top of the intrusion range. The keep rule dropped the one such candidate.
- **Speeding headroom.** The score has no speeding penalty, so an agent that drives above the posted limit can buy back some pace.
- **Kit docstrings.** The kit's docstrings still speak of fresh seeds (`robotics_grader/seeds.py`). The kit stays byte-identical to the shared one.

## How to redo it

All commands run from this task folder. On apollo, CPU work goes under `taskset -c 16-31`. CARLA runs only in `pr24t2-*` containers of the task image with `--network none` and one GPU that has at least 8 GB free.

```
python3 -B tests/review/dev_batch.py make --out SPECS.json --per-variant 8 --rng N        # dev seeds < 2**32
python3 -B tests/review/dev_batch.py run --specs SPECS.json --out DIR --gpus 0,1 --cpus 16-23,24-31
python3 -B tests/review/dev_batch.py report DIR
python3 -B tests/review/select_set.py draw --selection tests/hidden/selection.json        # commit it, then:
python3 -B tests/review/select_set.py run --selection tests/hidden/selection.json --round R --out DIR
python3 -B tests/review/select_set.py make --selection tests/hidden/selection.json --out tests/hidden/episodes.json --causes CAUSES.json
python3 -B tests/review/select_set.py report --selection tests/hidden/selection.json
ALE_HOME=/tmp/h bash solution/solve.sh   # then cp tests/review/spy/{spy.py,run.sh} /tmp/h/submission/ for the audit
CPUSET=16-23 bash tests/review/regrade.sh 0 pr24t2-verify /tmp/h/submission OUT
```

Any change to a frozen file invalidates the set: freeze again, draw a new selection, and measure the anchor again.
