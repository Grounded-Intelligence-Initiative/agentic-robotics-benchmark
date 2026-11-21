# What partial and degraded flights score

The privileged pilot (`solution/oracle/pilot.py`) and its mutants (`PILOT_MUTANT=<name>`), each flown once on the eight
graded sites with the grader's own simulator. A mutant degrades one part of the plan; the "stop" mutants are the
partial programs. The table gives the mean over the eight sites; per-site values are not published.

| Flight | Mean progress | How the flights ended |
|---|---|---|
| reference | 1.00 | handoff on all eight |
| `stop_after_entry`: in through the inspection slot, then stop | 0.05 | stopped |
| `stop_upstairs`: up a hoist well, then stop | 0.09 | stopped |
| `stop_over_litter`: fly over the litter, then stop | 0.17 | stopped |
| `stop_after_grasp`: latch the litter, then stop | 0.32 | stopped |
| `stop_after_lift`: lift it to cruise height, then stop | 0.42 | stopped |
| `stop_after_egress`: carry it out under the cargo shutter, then stop | 0.67 | stopped |
| `wrong_tool`: mount the other collet | 0.17 | the collet does not latch |
| `wrong_station`: fly to the closed station | 0.67 | contact with the lowered shutter |
| `no_board_alignment`: never turn the board into the direction of travel | 0.42 | contact with a wall at the first doorway after the lift |
| `release_high`: release from cruise height | 0.67 | unsafe release |
| `pose_noise_10cm`, `pose_noise_25cm`: the pilot's pose feedback with 0.10 m / 0.25 m noise | 0.00 | the pilot never settles on a tool stand and stops before entering |

The means above 0.05 include the rooms-searched share (up to 0.03), which depends on the route. The weights are the
ones in force since they moved from reaching the litter to the work on it (entry 0.05, upper storey 0.04, over the litter
0.06, latch 0.15, lift 0.10, carry-out 0.25).

The reference needs 443 to 541 of the 4,400 actions per site and under 2 s of its own time. The anchor is the
reference's mean progress, 1.0; `full_at` is 0.98.
