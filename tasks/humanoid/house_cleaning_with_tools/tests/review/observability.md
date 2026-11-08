# What the cameras show: the crumbs and the grease are observable

The agent's program sees the mess only through the four RGB-D cameras (cell_manual.md section 6); nothing about the
objects, the grease or the crumbs is on the wire. This note shows that the two smallest targets, the crumbs (7-14 mm)
and the grease film, are visible in frames the program can take, from viewpoints the robot can reach.

## Method (`tools/crumb_views.py`, `tools/survey_photos.py`)

* The cell is built on a scene's seed exactly as the grader builds it (`CellServer`, not privileged). The robot is
  placed (not walked) at stances on a ring round the dining set: south and north of the table 0.42 m and 0.95 m from
  its long edges (x every 0.25 m from 2.10 to 3.60), west and east beyond the chairs, facing the table; a stance where
  the robot's body would touch the furniture is skipped (all 38 stances were free in every scene). Two postures, inside the
  robot's limits: squatting (stand height 0.55 m, waist pitch 0.55 rad: the head camera about 0.95 m over the floor,
  looking down at it) and standing (0.79 m, waist pitch 0.30 rad: the table's top).
* At each stance the head camera's colour is rendered twice with the cell's own renderer (Filament on lavapipe, the
  frames' clean colour before the sensor model): with the crumbs, and with every crumb moved out of the room. A crumb
  counts as seen from a stance when at least 3 pixels in its projected footprint change by more than 12 grey levels,
  6 times the cameras' pixel noise (sigma 0.008, 2 levels). The grease is tested the same way (the film's cells set to
  clean).

## Results

| scene | crumbs on the floor seen | crumbs on the table seen | pixels changed per crumb (min / median, best stance) |
|---|---|---|---|
| development scene a (seed 11, survey `scene_a`) | 46 of 46 | 14 of 14 | 6 / 18 |
| development scene b (seed 12, survey `scene_b`) | 46 of 46 | 14 of 14 | 4 / 17 |
| hidden scene 1 | 46 of 46 | 14 of 14 | 6 / 17 |
| hidden scene 2 | 46 of 46 | 14 of 14 | 6 / 13 |
| hidden scene 3 | 45 of 46 | 14 of 14 | 2 / 15 |

The one crumb not seen (hidden scene 3) is the smallest of its 60, a 7 mm stray on the oak floor 0.53 m south of the
table: it changes at most 2 pixels from the ring's stances and 1 pixel from three standing stances placed for it
(`evidence/crumb_views_extra_hidden3.json`). The floor's credit is full at 90 % of the crumbs binned, so full credit
does not need it, and a broom stroke over the area takes it along.

The grease film changes 5,000-10,600 pixels by more than 30 grey levels in a head frame from the table's side
(`head_dining_south`), and 580-900 in the dining ceiling camera's frame. Per-crumb records: `evidence/` (`*_dev11`, `*_dev12`: the survey scenes; `*_hidden1-3`: the hidden set in its order).

## Amendment 1 (2026-11-08): the repeated selection's hidden set

The hidden set is now candidates 1, 2 and 3 of `candidates.json` (`hidden_set.md`, amendment 1). Candidate 2 was
hidden scene 1 above; the other two are new, measured the same way (the cell at `792cbb9`; the crumbs and the
cameras are unchanged by it):

| scene | crumbs on the floor seen | crumbs on the table seen | pixels changed per crumb (min / median, best stance) |
|---|---|---|---|
| hidden scene 1 (candidate 1) | 46 of 46 | 14 of 14 | 3 / 14.5 |
| hidden scene 2 (candidate 2, hidden scene 1 above) | 46 of 46 | 14 of 14 | 6 / 17 |
| hidden scene 3 (candidate 3) | 46 of 46 | 14 of 14 (one from a closer stance) | 1 / 14 on the ring; 7 for that crumb |

The crumb of hidden scene 3 that the ring sees by only 1 pixel is a 7 mm crumb on the table's top at (3.24, 2.62):
standing north of the table at (3.24, 3.10), waist pitch 0.50, the head camera sees it change 7 pixels (6.6 px
across, 0.47 m away; `evidence/crumb_views_extra_a1_hidden3.json`). Per-crumb records: `evidence/*_a1_hidden1.json`
and `evidence/*_a1_hidden3.json` (squatting and standing, as above).
