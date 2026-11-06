# shoe_lacing_and_boxing: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the cell is described in `environment/setup/payload/given/cell/cell_manual.md` and
its wire in `cell_api.md`. This page records the design decisions, what the grader checks and why, and what is
known to be rough in version 0.2 (2026-10-30, Yuanfang Peng; reference lacing and the lace physics reworked 2026-11-05).

## 1. The job and why it is worth measuring

Lacing is the last manual operation of a sneaker line (Adidas's Speedfactory left it to people; Pou Chen and Solomon
hold patents on camera-guided lacing cells). The cell here is the end of a children's-sneaker line: pair after pair,
two Panda arms seat each lasted-open shoe in a cradle, lace it criss-cross with a lacing hook (from below the eyelet
flaps) and a gripper (from above), erect the flat box blank, pack the laced pair heel-to-toe, put the lid on and push
the box to the outfeed. It is a Physical Execution task: a work order of one pair per episode (900 s simulated, 6 hidden episodes), a long
chain of dependent stages, explicit exceptions (a shoe on its side, an off-centre lace), and nothing reported about
the objects: the program works from calibrated RGB-D cameras only. The closest prior art is UniLace (Unity, Obi rope,
bimanual lacing simulator), RLBench's `put_shoes_in_box`, pi0 / RL-100 box folding and EmbodiedSWE-Bench's packing
tasks; none chains lacing with box erecting and packing, runs in CPU MuJoCo, or scores a coding agent that has to build
its own perception.

## 2. The cell (`environment/setup/payload/given/lib/shoecell/`, byte-identical to `tests/shoecell/`)

* `physics.py`: the scene composed with MjSpec from the blocks `cell.py` (bench, pedestals, Menagerie Pandas, hook
  tool, welds, lights), `cameras.py`, `shoe.py` (a rigid hollow shell with real eyelet holes: the flaps are box
  segments between 8-capsule rings), `lace.py` (a 63-link chain of 10 mm segments on two-hinge joints, bending only,
  with two aglets; see the constants at the top of `physics.py` for why 10 mm hinge pairs), `carton.py` (hinged
  panels with joint-equality snap-locks) and a rigid lid tray. MuJoCo 3.14, timestep 2 ms, implicitfast, Newton,
  elliptic cones, dense Jacobian, 20 iterations; a 10 ms control tick = 5 steps. The laces have `condim 1`
  (frictionless against everything but the eyelet rings, which have priority 2, condim 3 and friction 0.3): this is
  what makes the full scene run at 0.6-1.6x real time instead of 0.2x; the rings' friction is what holds a lace in a
  ring. A per-body linear drag on the lace (`LACE_DRAG`, applied in `Scene.step_physics`) brings loose cord to rest
  (MuJoCo's fluid viscosity and a condim-3 lace were both measured at 2-3x the step cost). The batch has two
  eyelet rows (row 1 delivered threaded and tacked; section 5). Caught aglets glide into their seat on the gripper or
  the hook over 0.3 s (a snap to the canonical pose drove the lace through the ring wire), and a held aglet pulled
  harder than 6 N for 30 ms slips out (a welded pull on a pinned lace otherwise diverged).
* Contact bits (`Scene._index`, documented in the manual): ordinary geoms (1, 31), the gripper hand + fingers
  (1, 11), the hook tool (8, 0), lace segments (2, 5); a body held by the gripper becomes (4, 0) (shoe, panel, lid) or
  (16, 0) (an aglet) while held. Hence a lace never touches itself, another lace or the hook; a held shoe keeps
  colliding with its lace (it used to fall out of row 1 on every carry); a held aglet touches neither its lace nor
  the hand. Closing the gripper on an aglet that sits on the hook takes it off the hook (the handover).
* Delivery (`Scene._thread_lace`): the lace path is computed in the world frame from the shoe's actual pose (upright
  or on its side), the tails are laid on the tray plate as a spiral with 90-degree corners that stays 3 cm inside the
  rim and clear of the lid spot and the other shoe. Row 1 survived the settle on all 28 seeds tried.
* Blow-up guard: `Scene.step_physics` flags `physics_fault` when MuJoCo's BADQACC auto-reset fires (time goes
  backwards or the warning counter is set); the server ends the episode (`done_reason physics_fault`) instead of
  crediting a reset world. The safety watch also counts arm-arm contacts above the force limit.
* Grasping is a documented attach rule (a weld equality retargeted at run time), not friction: UniLace made the same
  choice because "friction alone cannot hold an aglet". The gripper attaches any documented handle inside its pinch
  box at the current pose (an aglet at a canonical pose along the approach axis); the hook's clip catches an aglet
  within 12 mm of its tip. The cradle clamp and the carton locks are welds / joint equalities too.
* One batch per episode (shoe model and box size from the seed), two shoe bodies, two laces, one blank and one lid
  re-used from pair to pair (colours change through `geom_rgba`; a shipped pair is parked under the bench and the
  next pair is set down at once). Geometry cannot change at run time, so the batch is what varies between episodes.
* `episode.py`: the work order (seeded), the faults, the lacing topology check and the milestones.
* `server.py`: the API, the attach / clamp / lock logic in `physics_tick`, the camera budget and noise, the safety
  watch, the summary. `mode="practice"` additionally answers `reset` and `truth`; `privileged=True` (the oracle token)
  answers `get_state_privileged` and `reference_run`.

## 3. What the grader checks (`episode.py`)

Per pair, judged on the physical state when the box ships (frozen) or at the end of the episode:

| milestone | credit | check |
|---|---|---|
| lacing of each shoe | 0.30 + 0.30 | the lace polyline is intersected with every eyelet ring's disc; walking each tail from row 1 outward, the passes must alternate sides and every ring must be entered from the outside (row 2 IN on the other side, the lace back out through the throat slot, row 3 IN on its own side, ...: the "throat-return" criss-cross); rows are credited consecutively until the first miss; snug = lace between consecutive rows <= 1.5 x the straight distance (+ 3 cm for the dip under the flap and the throat return; a pulled lace springs back 0.5-2.5 cm through the ring when let go, a loose one lies at 2.5-5 x); 0 if an aglet was left inside the cavity: an end that passed in and was never seen back out (the work order latches, every 0.5 s, each end it sees clearly out of the shoe after its last pass; a finished end that later slides or falls back in, e.g. while the shoe is packed, does not count) |
| erected | 0.12 | the four wall locks engaged |
| packed | 0.13 | both shoe centres inside the box interior, sole down within 15 deg, headings within 30 deg of opposite, sole bottom on the floor |
| closed | 0.10 | the lid tray centred within 12 mm, aligned within 6 deg (mod 180), at wall-top height, resting, with the pair inside |
| shipped | 0.05 | closed, pair inside, floor centre across the outfeed line |

The work order is a laced pair in a shipped box: packed, closed and shipped are credited times the fraction of the
pair's shoes laced through every row (0, 1/2 or 1; snugness aside), so a pair boxed unlaced earns the erected credit
(0.12) only. (Before 2026-11-05 the box credits did not depend on the lacing: an Opus 5.5 run erected, packed, closed
and shipped in three of four episodes without lacing a shoe and read 0.33 = 0.64 of full credit.)

`work_score` = mean over the hidden episodes of (sum of credits) / pairs. An untouched cell scores exactly 0 (the
delivered lace passes only row 1; nothing is erected). A safety violation (a Panda link other than the pads / hook
pressing more than 30 N on the bench, the cradle or an object, or any arm-arm contact above 30 N, for three control
ticks in a row (a one-step spike of MuJoCo's soft contact is not a press), or an object faster than 3 m/s) voids the credit of the pair at work when it happens (`safety_void` in the pair record; the whole episode
was zeroed in an earlier draft, which made one 30 N touch of cardboard worth the whole window); a physics
fault ends the episode with the shipped pairs' credit only. The contract gate (`verify.py`) is the
one of `robotic_arm_full_assembly_so101_panda`: a regular `packer.py` with `solve`, the route gate, the static and
runtime network audit, the deprivileged run as `nobody` from a read-only snapshot, the stage lock, the late-write
gate; the run directories carry opaque tags (`ep01`), never the seed. Verification is separate (`task.toml`): the
grader runs in a fresh verifier sandbox from the task image with only the submission restored, so nothing else the
agent left reaches it; `environment/setup/setup.sh` does not run there, and `tests/test.sh` builds the grader's
root-owned copy of the givens at `/opt/arb` from `tests/setup_payload/` (a byte copy of
`environment/setup/payload/`: change both together, then `tests/given_hashes.json`) and the image's Panda.

## 4. The reference (oracle)

`solution/solve.sh` installs `packer.py` (one privileged call, `reference_run`) and the token. The controller
(`tests/reference/`) runs inside the grader with the exact state: `primitives.py` (the Rig: Cartesian / joint moves
through the cell's own controllers, seeded IK for wrist-flip poses, gripper / hook / clamp calls), `lace_skills.py`
(seat; per tail: the gripper picks the aglet from the bench and drops it IN through the row-2 ring on the other side,
the hook enters the cavity through the throat, catches the aglet under the flap, rises out through the throat slot and
draws the lace until the row 1 -> row 2 segment is snug, then lays the aglet on the bench; unseat back to the shoe's
own delivery spot), `box_skills.py` (erect, pack, lid, ship). It obeys the same rate limits, attach rules and physics
as a submitted program; its measured `work_score` is the anchor. The hidden seeds are drawn from OS entropy per verify
run (`cellkit/seeds.py`), so `full_at` must cover the oracle's seed-to-seed spread.

**State on 2026-11-05.** `grader_config.json` runs the full reference (`reference_lacing: true`). Measured on the host
with the grader's configuration on seeds 1-64 (one episode each, the call verify makes): mean work_score 0.695; both
shoes laced in 30 episodes, one in 26 (its box credits count half), none in 4 (the erected credit only), 4 voided by a
sustained contact (a pitched hand's forearm on the lid while folding the wall next to it, a finger on the shoe while
re-picking an aglet left on the flap, the hand on the shoe while picking an aglet beside it), 0 physics faults. Per
shoe the lacing succeeds about 80 % of the time when the shoe arrives upright and about half the time when it arrives
on its side (section 5). The verify statistic is the mean over 6 episodes. The four validate runs voided 5 of 24
reference episodes against 4 of 64 here (all by the contacts of section 5; the reference reads no wall clock, so this is
seed-to-seed spread or a host/engine difference not found), so full_at is sized for a 12-21 % void rate:
`tests/anchor.json` has value 0.695 and full_at 0.50, below which the 6-episode mean falls in 1.8-6.1 % of verify
runs (bootstrap of these episodes; the last validate read ratio 0.94). Six
episodes rather than four because with four the same risk needs a much lower full_at. A run that erects every box but
laces nothing reads 0.12 per episode, i.e. reward 0.35; lacing about one shoe per pair and boxing it reads full.

**State on 2026-11-06 (the reference reworked by the maintainers after the review of PR #48).** The seat carries the
shoe high to above the cradle, lets the tails hang straight and lowers it straight down, looking under the sole before
it lets go (the earlier sideways approach left a tail under the sole after every upright seat); a slip is noticed at
once and each tail gets up to four tries, each from where the tail lies; a bench aglet is grasped high in the pinch box
while the hand rises as the cell seats it; every move of either arm stops and backs off at 8 N on a link (12 N on a
finger pad), checked every tick; the shift of a righted shoe on the infeed tray, the shoe grasps and the wall folds are
planned clear of the tray, the neighbour shoe and the lid (a guard cannot stop a link that meets something at speed); a
lid tray lying in the way of a wall fold is lifted back onto the infeed tray; the box is pushed only with its lid
closed. Measured as above (the reference of `2e9191d`; one queue of `2eed6f9` and seed 1004, the one episode whose path
`2e9191d` changes, again), seeds 1-64: mean work_score 0.993, 63 of 64 episodes at 1.0, both shoes laced in 63, one in
1, none in 0, 0 voided, 0 physics faults; seeds 1001-1064: mean 0.968, 60 of 64 at 1.0, 0 voided, 0 physics faults. Per
shoe the lacing succeeds in 184 of 187 upright and 67 of 69 side-lying arrivals; an episode takes 423-608 s simulated
and 678-1568 s of wall clock on one core of the host (mean 942 s). The anchor (`tests/anchor.json`, above) is still the
2026-11-05 measurement: with the reworked reference its value and full_at are to be measured again (an owner decision;
not done here).

## 5. Known limitations of version 0.2

* The shoe is rigid; "snug" is a lace-slack criterion, not throat closure.
* The lace has no self-collision and no lace-lace collision (speed); a knot is not modelled and not scored.
* The hook can only reach eyelets within ~11 mm of the throat edge (its foot is 14 mm): the batch's eyelet inset is
  9 mm. Its J shape means it can only carry an aglet upward through a ring; the gripper does the downward passes.
* Two eyelet rows per shoe in this version (one throat-return pass per tail): each pass consumes about 13 cm of
  cord twice over (down into the cavity and back out through the slot), so two passes per tail exhaust the 0.64 m
  lace and draw the other tail back through row 1. A three-row batch needs a ~0.9 m lace (60 chain bodies, ~30 %
  slower physics) and a re-tuned reference: the intended next step.
* Why the throat-return pattern instead of the textbook criss-cross (the lace coming OUT through the next ring from
  under the flap): the hook can lift an aglet through a ring by only ~6 mm (its foot stays under the flap), so every
  such pass would end with the gripper taking the aglet off the hook tip right at the ring, where the hook arm's wrist
  stands; two Pandas cannot share that space (two rounds of oracle work ended in arm-arm contacts or an infeasible
  column). With every ring entered from above by the gripper and the hook only fetching the aglet out through the
  slot, the two arms work in turn at a safe distance and the hand-over happens in free space or via the bench.
* The cradle holds the shoe with its throat slot along y (toe toward the hook arm): the hook arm works over the toe,
  the gripper over the heel.
* The lace is tacked at row 1 (a weld between the shoe and the segment crossing the cavity, activated at delivery):
  without it the first lift of a tail drew the other tail back through row 1 on most seeds (the reference lost the
  delivered row on 4 of 5 seeds), and the "uneven tails" fault that relied on sliding through row 1 is retired.
* The lid is a separate rigid tray (a one-piece hinged-lid blank was tried: its flat footprint, 0.94 x 0.43 m, is
  out of a bench-mounted Panda's reach).
* The oracle's sequence is scripted and conservative; its timing sets the pairs-per-window budget. Hidden episodes are
  expensive (11-26 min of wall clock each for the reference, 16 on average), so `grader_config.json` keeps the episode
  count at six, the least that keeps `full_at` well above the boxing-only level (section 4).
* The reference's weak spot (2026-11-06) is a tail that comes to rest where neither arm can pick it up. With the toe
  toward the hook arm, the tail that leaves a seated shoe on its R side can end within 0.22 m of the hook arm's base
  (the hook arm cannot reach the bench that close to itself) and more than 0.68 m from the gripper's base (beyond its
  top-down reach), and a hook-only aglet re-laid toward the gripper can end inside the gripper's 0.26 m. The seat lowers
  the shoe again up to three times while an aglet lies out of reach, a tail gets four tries and an aglet only the hook
  reaches is re-laid in the gripper's zone, but an aglet in that dead zone stays there: 4 of the 128 episodes of
  2026-11-06 lost a shoe (one lost both) this way. Until 2026-11-05 the weak spot was the side-lying shoe (its tails
  were dragged under the sole by the sideways seat); the reworked seat laces side-lying arrivals as often as upright
  ones (67 of 69 and 184 of 187).
* The reference's voids: none in the 128 episodes of 2026-11-06. Every move stops and backs off at 8 N on a link,
  checked every tick, but a link that meets something at speed builds its press within one 10 ms tick, so the contacts
  found (the forearm on the lid at a pitched wall fold, a finger on the shoe while re-picking an aglet, the hand on the
  shoe beside an aglet, the forearm on the infeed tray and the hand on the neighbour after a righted shoe's shift) are
  avoided by planning, and a box is pushed only with its lid closed (pushed from where a failed packing had left it,
  near the gripper's base, the hand met the bench at 295 N). The closest call: a finger on the lid tray at 32 N for 2
  ticks (the rule voids at 3) in the joint-space line home from above the infeed tray after a righted shoe was set down
  beside the lid (seed 48, `box_skills._go_home`; that line is checked kinematically, but it runs even when the check
  finds it blocked at both lift heights).
* The lid tray's delivery spot (tray centre y - 0.14 +- 0.012) puts its near wall over the infeed tray's rim at y =
  0.02: of the 128 seeds above, 45 lids rest tilted 3-4 degrees on the rim and 4 are pushed off the tray at reset, 12 cm
  toward the blank, in the way of the wall folds (the reference lifts such a lid back onto the tray). Drawing the lid at
  y - 0.12 would keep every lid flat on the plate (a delivery change, not made).
* Three-row batches need a longer lace (section 2).
