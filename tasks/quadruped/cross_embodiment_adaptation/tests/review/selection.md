# The graded set

Eight fixed sites, `tests/hidden/sites.json`: eight of the 24 candidates of `tests/hidden/candidates.json`
(`os.urandom(8)` values in drawn order; candidates 0-4, 10, 11 and 16), chosen in the second maintainer round
(`9d8cc14`) by that round's pre-registered rule and unchanged since. No seed was replaced when the task's world
or its properties changed.

What a seed decides:

- bits 14 and 15: on which side the wide passage of the first and of the last rack lies;
- bit 33: the launch's handedness; bit 39: which fitting comes first (duct or deck);
- bit 30: which opening of the middle rack the pallet closes at survey time; bit 27: the pallet was moved
  to the other opening before the deployment; bit 43: the dock was moved; bit 37: a bollard was taken away or
  added (version 6; chosen by the seeds' bits alone, before any physics, so that each is set on three or four of
  the eight sites);
- the remaining bits: the continuous layout (rack positions and depths, opening widths, the dock, the bollards)
  and, by independent streams, the launch pose (stream 3702), the per-site physical properties (stream 3703), the
sensors' biases and noise (stream 3705) and the amounts of the changes (stream 3706).

| Site | Wide passage sides, west to east | Launch bank | First fitting |
|---|---|---|---|
| 0 | South, North | North | Duct |
| 1 | South, South | South | Duct |
| 2 | North, South | North | Deck |
| 3 | North, North | South | Deck |
| 4 | North, South | South | Duct |
| 5 | South, South | North | Deck |
| 6 | North, North | North | Duct |
| 7 | South, North | South | Deck |

## Per-site physical properties (version 5, 2026-11-20)

The rule, written before any site was generated with it: every site draws each property below uniformly and
independently from its range, from stream 3703 of the site's seed; the commissioning pad draws from the same
ranges with a random seed of its own per episode (stream 3704). The ranges are in the instruction and the
datasheet; the drawn values are never sent to a program.

| property | range |
|---|---|
| carried mass (saddle and instrument pack) | 2.50 - 4.50 kg |
| pack position ahead of the trunk origin | 0.05 - 0.28 m |
| servo stiffness factor, per joint | 0.85 - 1.15 |
| torque-limit factor, per site | 0.85 - 1.00 |
| friction of every walked surface | 0.45 - 0.90 (0.40 - 0.90 until version 8) |
| duct underside above the floor | 0.410 - 0.445 m |
| guide crest above the deck | 0.070 - 0.112 m (to 0.125 m until version 8) |

How the ranges were set (development sites only, before the graded set was run with the final ranges): wide
enough that a controller with nominal postures and no correction from its own readings fails most sites, and
within what the oracle's gait completes with margin. The carried mass was first tried up to 5.5 kg; above about
4.5 kg the trot stalled on the deck's apron on low-friction sites, so the range ends at 4.5 kg. The guides' rise
was first 0.070 - 0.092 m; a single tall posture then cleared every site, so the range was extended to 0.125 m,
where the posture has to follow the rise the survey measures. The tick budget
(48,000) is 1.5 times the oracle's slowest site on 48 development sites.

Each observable on site: the load and the pack's place from the joint angles and the gravity reading under the
robot's own weight; servo stiffness and strength from the same; friction from odometry against the commanded
gait; the duct's clearance and the guides' rise from the parallax of the teal covers in the survey.

## What changes after the survey, and the sensing (version 6, 2026-11-21)

The rule, written before any site was generated with it:

| | rule |
|---|---|
| middle rack | two openings (centres 0.66-0.80 m either side of the hall's axis, half-widths 0.42-0.46 m); a 1.2 x 0.8 x 0.14 m pallet on its short edge closes one (bit 30) |
| pallet moved | bit 27: it stands in the other opening when the Go1 arrives |
| dock moved | bit 43: 0.10-0.25 m west and 0.10-0.25 m north or south (stream 3706); the beacon moves with it |
| bollards | nine in the survey (five round the staging bay, four along the rails; six along the rails until version 9); 24 mast exposures a deployment (40 until version 8); bit 37: one of them is gone, or one is added on a rail (stream 3706) |
| odometry | heading error rate within +/-0.80 degrees per second (0.40 until version 8), distance factor 0.94-1.06 (stream 3703) |
| encoders, IMU | noise 0.002 rad / 0.01 rad/s / 0.01 and constant biases within 0.008 rad / 0.015 rad/s (stream 3705) |
| UAV tracking | displacement factor 0.97-1.03 (stream 3703) |

| Site | pallet moved | dock moved | bollard changed |
|---|---|---|---|
| 0 | | | |
| 1 | yes | | |
| 2 | yes | yes | |
| 3 | | yes | yes |
| 4 | yes | | |
| 5 | yes | yes | yes |
| 6 | | | |
| 7 | | yes | yes |

Two of the eight sites are unchanged, so a program that always detours or always distrusts its survey gains
nothing there, and one that always trusts it loses six.

Observable with the stated sensors (`observability.py`, the oracle's path on the graded sites, frames rendered
outside the exposure budget): at least three bollard caps are in view of the mast camera's eight detents at
every 0.4 m of the route (median 12 or more), the pallet is in view before the middle rack on every site, and the
dock beacon before the dock on every site.

Not included, and why: a raised strip or debris field that needs foot placement. The only free floor is in the
bays, where the Go1 changes aisle sideways; crossing a raised strip sideways or diagonally is not something the
oracle's gait does robustly, a strip that is in the survey is one more fitting to plan for in advance, and one
that is not in the survey cannot be seen by a camera that looks up.

## The equipment and the barrier as real objects (version 7, 2026-11-21)

No rule and no range changed. The two boxes the Go1 carried are the envelopes of real devices now: a 3D lidar head
(Ouster OS1 class, a cylinder 87 mm in diameter and 74.5 mm tall) on a mast with a compute unit, and a floor
profiler (SICK TiM561 class, 60 x 60 x 86 mm) on a bracket under the nose. The lidar's top is where the guard's top
was (0.199 m above the trunk origin) and the profiler's bottom where the scanner's was (0.163 m below it), so the
duct's and the guides' ranges stand and both fittings ask for what they asked. The fixed equipment weighs 1.10 kg
(0.70 kg before); the carried total stays 2.5-4.5 kg.

The barrier is a 1.2 x 0.8 x 0.14 m pallet on its short edge (0.80 m wide, 1.20 m tall), inside the opening and in
line with the rack. A first draft stood it on its long edge in openings widened to 1.22-1.30 m; replayed on that
draft, the program a frontier model had written for version 6 (1 of 8 graded, 4 of 32 development sites there)
docked 4 of 8 and 10 of 24, so the openings went back to version 6's width class before any new episode was run.

## The equipment's reach restored (version 8, 2026-11-21)

Version 7 scored 5 of 8 with a frontier model, and the program it wrote docked 13 of 32 development sites; the
same program on version 6's sites, where the Go1 carried the two boxes, docked 6 of 32. The slim devices passed
where the boxes had caught a rack end or the pallet. So the equipment gets its reach back with real parts, fixed
before any run of version 8:

- a tubular guard frame round the lidar (12 mm tube, 0.204 x 0.170 m, top at 0.199 m above the trunk origin: the
  old guard's footprint, centred on the lidar), and the lidar's head 12 mm under it;
- the profiler as a line profiler mounted crosswise (LMI Gocator 2340 class, 0.197 m wide where the scanner box
  was 0.176 m; 0.74 kg where the TiM-class unit was 0.25 kg).

The measure, also fixed before any run: the version-7 program replayed on 32 development sites, target 8 of 32
or fewer; if above, in this order and stopping at the first that reaches it while the oracle keeps 8 of 8 graded
sites and 45 of 48 per development batch and the audit passes: odometry heading error rate to 0.8 degrees per
second, mast exposures 40 to 24, rail bollards six to four.

For the oracle (no agent-side reason): with 0.49 kg more on the nose its gait stalled on the apron of the tallest
guides at the lowest frictions (graded 6 of 8; development 45 and 44 of 48). The guides' rise ends at 0.112 m
instead of 0.125 m and friction starts at 0.45 instead of 0.40; after that 8 of 8 and 46 and 47 of 48.

What the measure gave (the version-7 program on 32 development sites; 13 on version 7):

| step | as written | with its own exposure cap set to the new budget |
|---|---|---|
| the reach restored | 13 | |
| + heading error rate to 0.8 degrees per second | 11 of 31 | |
| + 24 mast exposures | 0 | 7 |

The reach alone changed nothing for this program: its contacts are its rear legs, not its equipment. The heading
drift changed little: it re-localises often enough. With 24 exposures the program as written spends them before
the middle rack and then stands still, which says nothing about the task; with its one constant set to 24 it docks
7, so fewer fixes do cost it. The chain stopped there; the rail bollards stay six. The oracle and the audit are
unchanged by the last two steps (the oracle does not use odometry or exposures).

## The graded set grows from eight to sixteen (version 9, 2026-11-21)

A measurement fix, written before any of the new sites was generated or run. With eight sites one site is an
eighth of the score, and in three versions a fresh program's graded score sat above its level on 32 development
sites (0.89 against 0.75, 0.64 against 0.41, 0.51 against 0.22). Nothing the agent sees changes except the stated
number of graded sites.

The rule is the one the set was made by: the candidates of `tests/hidden/candidates.json` in drawn order, each run
once by the privileged oracle as the grader runs a mission, and a candidate the oracle does not complete is dropped
and recorded. The second round's two cells described a world that no longer exists and are not applied. The eight
sites kept so far (orders 0-4, 10, 11, 16) stay, in their places; the next eight are the first eight of the
remaining orders (5, 6, 7, 8, 9, 12, 13, 14, then 15, 17, ...) that the oracle completes. Anchor and `full_at` are
unchanged; the verifier's time limit scales with the number of missions.

Result: the oracle completed orders 5, 6, 7, 8, 9, 12, 13 and 14 at the first try (25,850 to 31,040 ticks); none
was dropped, none beyond 14 was run. They are sites 8 to 15.

| Site | order | pallet moved | dock moved | bollard changed | first fitting |
|---|---|---|---|---|---|
| 8 | 5 | yes | yes | yes | deck |
| 9 | 6 | | | | deck |
| 10 | 7 | | yes | | duct |
| 11 | 8 | | | yes | deck |
| 12 | 9 | yes | yes | | deck |
| 13 | 12 | yes | | | duct |
| 14 | 13 | yes | yes | | duct |
| 15 | 14 | yes | | | duct |

Of the sixteen, three are unchanged since the survey (sites 0, 6, 9).

Re-graded on the sixteen sites without a model call, the version-8 program (4 of 8 on the first eight) docks 7 of
16. The line set beforehand for taking the re-graded value as the result was 0.40; 7 of 16 is above it, so the
third step of version 8's chain follows, written here before it is built or run: the rail bollards go from six to
four (two along each long wall, near x = 2.85 m and x = 4.95 m, each within 0.30 m of that), nine bollards in the
survey. The audit's line, also set here: at least two caps in view of the mast camera at every 0.4 m of the route
(two caps with their ranges fix a position and a heading), the pallet in view before the middle rack and the
beacon before the dock, on every graded site. The oracle has to dock all sixteen.
