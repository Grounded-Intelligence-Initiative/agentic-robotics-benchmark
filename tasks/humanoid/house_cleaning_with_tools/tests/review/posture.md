# The reference's arm postures (2026-11-09)

The reference cleaner (`house_cell/reference.py`) is the oracle's route and the evidence that the job can be done with
this robot. Its demo videos showed the arms in postures no person would take: a plate carried with the arm swung back
and up beside the head, the sponge held under the faucet with the elbow over the shoulder, the hand wrung against a
wrist limit while wiping, a free arm that already hung at home lifted up and back before every walk. Measured over
whole episodes (below), one arm or the other was in such a posture for 48-62 % of the time. Commit `1170d84` changes
how the reference plans its arms, nothing else: the cell's controller and its IK, which a program commands, are
unchanged.

## What changed

* **NaturalIK** (the planner's IK): the cell's damped least squares on the grasp site, plus a posture term descended
  in the arm's redundancy (the elbow's swivel about the shoulder-wrist line; with a position-only target the wrist as
  well), as a second task in the target's null space: the elbow at least 8 cm below the shoulder and not behind or far
  out beside the body, every joint at least 0.35 rad off its limits, the twisting joints (shoulder yaw, wrist roll)
  near their middle. The term acts only once the target is nearly reached, so whatever the cell's IK reaches this
  reaches as well (and the cell's IK answers where it does not). A path's waypoints change the posture by at most
  0.1 rad each, so the joint-space moves between them stay on the Cartesian line. Every stance check, path, carry and
  the constrained motions (a lid, the faucet lever, the sponge on the table) use it.
* **Carrying** (`canonical_carry`): a held item goes to the most natural of a few carry points and turns: in front of
  the chest at about elbow height (was shoulder height), a little nearer the middle or farther out, an item symmetric
  about its vertical axis turned about it; litter for a bin also beside the hip with the arm hanging (a can held from
  above dangles from the hand; carried at the chest it puts the elbow up), napkins and the apple core at any angle; the
  item kept clear of the robot's own body. Where a carried item would sweep down into the counter as the robot leans
  in, it is lifted first (`go_put`), as a person lifts a plate over the counter.
* **Grasps chosen for what follows**: an object's grasp frames are ordered by the carry each leaves (`carry_order`);
  the sponge's grasp and stance at its tray by the take's posture plus the wiping and rinsing postures its grip leaves
  (`sponge_work_costs`).
* **A free arm at home stays there**: `retreat` no longer lifts an arm that hangs at home (it swung it up and back
  before every walk).

## How it is measured (`tools/posture_run.py`)

One privileged episode per scene, run as the oracle's runs (`run_ref.py`), both arms sampled every 0.2 s of simulated
time. An arm is in an unnatural posture when its elbow is less than 5 cm below its shoulder (at or above it), a joint is
within 0.10 rad of its limit, the upper arm is swung out to the side more than 75 deg, or swung back more than 40 deg
(torso frame, from hanging down). The table gives the time either arm is so, and per criterion (seconds: elbow high /
joint limit / out / back), on development scenes 11-15, before (`6bc2ded`) and after (`1170d84`):

| scene | before: sim time | unnatural | per criterion | stage score | after: sim time | unnatural | per criterion | stage score |
|---|---|---|---|---|---|---|---|---|
| 11 | 3064 s | 1483 s (48.4 %) | 1052 / 725 / 282 / 627 | 0.9167 | 2518 s | 187 s (7.4 %) | 102 / 101 / 24 / 41 | 0.9028 |
| 12 | 4042 s | 2521 s (62.4 %) | 2169 / 748 / 1598 / 866 | 0.9074 | 3236 s | 249 s (7.7 %) | 147 / 134 / 36 / 34 | 0.9352 |
| 13 | 3659 s | 2113 s (57.8 %) | 1698 / 750 / 1305 / 797 | 0.7000 | 2829 s | 205 s (7.2 %) | 106 / 127 / 44 / 41 | 0.9491 |
| 14 | 3498 s | 1732 s (49.5 %) | 1364 / 803 / 436 / 702 | 0.9537 (safety violation) | 2405 s | 172 s (7.1 %) | 108 / 116 / 16 / 22 | 0.7100 |
| 15 | 3364 s | 1951 s (58.0 %) | 1625 / 777 / 1143 / 700 | 0.8677 | 3001 s | 215 s (7.2 %) | 125 / 106 / 39 / 37 | 0.9213 |

Over the five scenes: 55.6 % before, 7.3 % after. The episodes are also shorter (no swinging of the free arm, fewer re-plans).

What remains is short and mostly forced by the hand's orientation, which the object fixes: a grasp or a put with a
wrist near its limit for a few seconds (at the low coffee table, the dish tray, a bin), the left arm reaching across the
counter to the faucet's lever with the elbow level with the shoulder.

## Round 4 (2026-11-09): the arm's swivel resolved along every executed path

The round-3 video still showed the arms contorted much of the time. Contact sheets of whole development episodes, every
5 s from two views (in front of the robot to its right and from its left side), showed what the round-3 measure missed:
the elbow winged out or turned in about the shoulder-wrist line while the hand held an item level (a plate or the
sponge carried across the chest, the sponge at the pump and under the faucet, wiping), and the wrist bent hard. The
round-3 posture term acted only past thresholds (the elbow 8 cm below the shoulder, out more than 12 cm), so inside
them the elbow's place on its circle was left to wherever the solver drifted.

**What changed** (`reference.py`):

* `NaturalIK.terms`, the redundancy resolution: a swivel term that is always active pulls the elbow towards its
  natural place on its circle about the shoulder-wrist line. The natural place is down and a little out and back (for a
  hand hanging below the shoulder, back), and the pull fades out as the arm straightens. Beside it: the elbow not above
  the shoulder, not behind the body, not far out beside it; the wrist near its neutral where the hand's pose leaves it
  free (its sideways bend within 0.3 rad, its flexion within 0.55 rad, its roll within 1 rad); the shoulder's twist
  within 1 rad; the elbow neither straightened past straight nor folded tight. The joint limits (0.35 rad margin)
  outweigh all of these, so the swivel never wrings a wrist against its stop.
* `Reference.refine_path`: every joint path the reference executes keeps its planned hand path, but each configuration
  is solved again from the previous refined one with that resolution, drifting by at most 0.15 rad a waypoint (0.45 at
  the last). The hand, and what it holds, passes the same poses. A refined configuration that would touch the room
  where the planned one did not, or a move between two that leaves the hand's path by more than the planned one did
  (+ 4 mm), keeps the planned path.
* The planner's own choices (stances, grasps, carries) still rank by the round-3 measure (`terms_rank`); with them
  ranked by the new one, the plates' carries changed and the dish tray and the faucet became unreachable in every
  development scene.
* A held item that touches what the robot stepped up to (the sponge at its tray) is lifted 4.5 cm once before the
  put's last re-plan; before, the put was given up.

**How it is measured**: `tools/posture_run.py` now also reports a finer measure (its docstring): the elbow more than
55 deg from its natural direction about the shoulder-wrist line (the arm bent more than 25 deg), the elbow hyperextended,
the wrist bent (pitch or yaw beyond 0.95 rad, roll beyond 1.55 rad), the shoulder twisted beyond 1.45 rad, a wrist
flip. Both rounds were measured with this tool, on development scenes 11-15, one privileged episode each, both arms
every 0.2 s (time with either arm flagged; seconds summed over both arms per flag):

| scene | round 3: stage score | round-3 measure | finer measure | swivel off | wrist bent | at a limit | round 4: stage score | round-3 measure | finer measure | swivel off | wrist bent | at a limit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 11 | 0.9028 | 7.4 % | 67.7 % | 845 s | 1305 s | 101 s | 0.9120 | 8.1 % | 66.2 % | 470 s | 1512 s | 65 s |
| 12 | 0.9352 | 7.7 % | 67.7 % | 991 s | 1618 s | 134 s | 0.9306 | 7.6 % | 63.2 % | 490 s | 1588 s | 92 s |
| 13 | 0.9491 | 7.2 % | 65.6 % | 866 s | 1326 s | 127 s | 0.9028 | 7.3 % | 64.5 % | 511 s | 1589 s | 79 s |
| 14 | 0.7100 | 7.1 % | 72.9 % | 1046 s | 1317 s | 116 s | 0.6900 | 7.9 % | 68.3 % | 477 s | 1497 s | 69 s |
| 15 | 0.9213 | 7.2 % | 69.5 % | 977 s | 1529 s | 106 s | 0.9213 | 7.2 % | 64.6 % | 582 s | 1631 s | 82 s |

The elbow's swivel is off its natural direction for about half the time it was (4725 s to 2530 s over the five
scenes), and the joints are at a limit about a third less often. No wrist flips, no hyperextension, in either round.
The wrist is bent somewhat more often (7095 s to 7817 s): with the hand's pose fixed by the plan, the one free degree
of the arm trades a lower elbow for a more bent wrist, and the term is weighted for the elbow. The remaining wrist bend
comes from the hand's orientation, which the object and the plan fix: an item held level by the grip its take left (the
sponge for 700 s of an episode while wiping, rinsing and soaping, a plate by its rim), the lever of the faucet reached
across the sink. Changing those orientations means changing the plan's grasp and carry choices, which made the
reference fail in every development scene (above).
