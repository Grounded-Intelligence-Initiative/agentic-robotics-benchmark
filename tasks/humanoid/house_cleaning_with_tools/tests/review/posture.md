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
