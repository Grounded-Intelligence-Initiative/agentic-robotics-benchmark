# The graded set

Eight fixed sites, `tests/hidden/sites.json`: eight of the 24 candidates of `tests/hidden/candidates.json`
(`os.urandom(8)` values in drawn order; candidates 0-4, 10, 11 and 16), chosen in the second maintainer round
(`9d8cc14`) by that round's pre-registered rule and unchanged since. No seed was replaced when the task's world
or its properties changed.

What a seed decides:

- bits 14 and 15: on which side the wide passage of the first and of the last rack lies;
- bit 33: the launch's handedness; bit 39: which fitting comes first (duct or deck);
- the remaining bits: the continuous layout (rack positions and depths, opening widths, the dock, the bollards)
  and, by independent streams, the launch pose (stream 3702) and the per-site physical properties (stream 3703).

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
