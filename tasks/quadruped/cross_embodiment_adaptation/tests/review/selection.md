# The graded set

Eight fixed sites, `tests/hidden/sites.json`: eight of the 24 candidates of `tests/hidden/candidates.json`
(`os.urandom(8)` values in drawn order; candidates 0-4, 10, 11 and 16), chosen in the second maintainer round
(`9d8cc14`) by that round's pre-registered rule and unchanged since. No seed was replaced when the task's world
or its properties changed.

What a seed decides:

- bits 14 and 15: on which side the wide passage of the first and of the last rack lies;
- bit 33: the launch's handedness; bit 39: which fitting comes first (duct or deck);
- bit 30: which opening of the middle rack the pallet stack closes at survey time; bit 27: the stack was moved
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
| friction of every walked surface | 0.40 - 0.90 |
| duct underside above the floor | 0.410 - 0.445 m |
| guide crest above the deck | 0.070 - 0.125 m |

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
| middle rack | two openings (centres 0.66-0.80 m either side of the hall's axis, half-widths 0.42-0.47 m); a pallet stack closes one (bit 30) |
| stack moved | bit 27: it stands in the other opening when the Go1 arrives |
| dock moved | bit 43: 0.10-0.25 m west and 0.10-0.25 m north or south (stream 3706); the beacon moves with it |
| bollards | eleven in the survey (five round the staging bay, six along the rails); bit 37: one of them is gone, or one is added on a rail (stream 3706) |
| odometry | heading error rate within +/-0.40 degrees per second, distance factor 0.94-1.06 (stream 3703) |
| encoders, IMU | noise 0.002 rad / 0.01 rad/s / 0.01 and constant biases within 0.008 rad / 0.015 rad/s (stream 3705) |
| UAV tracking | displacement factor 0.97-1.03 (stream 3703) |

| Site | stack moved | dock moved | bollard changed |
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
every 0.4 m of the route (median 12), the pallet stack is in view before the middle rack on every site, and the
dock beacon before the dock on every site.

Not included, and why: a raised strip or debris field that needs foot placement. The only free floor is in the
bays, where the Go1 changes aisle sideways; crossing a raised strip sideways or diagonally is not something the
oracle's gait does robustly, a strip that is in the survey is one more fitting to plan for in advance, and one
that is not in the survey cannot be seen by a camera that looks up.
