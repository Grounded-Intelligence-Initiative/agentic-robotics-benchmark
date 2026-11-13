# Observability and acceptance evidence (grader-side; not shipped to the agent)

Drive B (cell belt-station/0.3.0): crank C, alternator ALT, the smooth backside idler I on the top run, water pump WP,
the automatic tensioner T on the left (opening clockwise), crank turning clockwise in the run-in. This file collects
the measurements behind the cell's acceptance and the sensor audit. The cell's belt chain, contact model, grip rule,
controller, cameras and sensor model are the ones of the accessory-drive belt station this task family shares; what is
new here (the drive, the idler, the peg position, the window) was measured on this cell.

## Acceptance thresholds (grader-only constants in `tests/beltcell/drive.py`, `ACCEPT`)

The agent's documents say only that every rib section must sit in its groove zone on the groove floor, not on a
shoulder or edge, and that the belt's back must run on the faces of I and T between their flanges; they give the
geometry (22 mm groove zone, 2.5 mm shoulders, 45 deg front chamfer; smooth faces d = 15-40 mm between 4 mm flanges).
The numbers below stay in `tests/`.

Definitions (after the run-in, per pulley, over the belt links on the pulley inside its drawn wrap sector):

- **winding**: |winding number| of the belt's joint polygon about the pulley's axis: 1 for a pulley inside the loop
  (ribs on C, ALT, WP), 0 for one outside it (the back on I and T).
- **lift** (gross) = link-centre radius - (pulley radius + half the belt's thickness). A seated link reads about
  +0.5 mm (the sphere colliders rest at their 0.5 mm contact margin).
- **axial** = |belt-centre depth - zone centre| (grooves: 27 mm; smooth faces: 27.5 mm).
- **cover** = links found in the trimmed sector / the sector's length in links.

| pulleys | sector trim | cover | lift | axial |
|---|---|---|---|---|
| C, ALT, WP (grooved) | 12 deg each end | >= 0.6 | <= 1.5 mm | <= 3.2 mm |
| T (smooth) | 12 deg each end | >= 0.6 | <= 1.5 mm | <= 3.0 mm |
| I (smooth, wraps ~34 deg) | 4 deg each end | >= 0.5 | <= 1.5 mm | <= 3.0 mm |

The groove thresholds (1.5 / 3.2 mm) are the midpoints of the gaps between seated and shoulder-riding belts on the
shared belt and groove model (564 samples: seated lift <= 1.06 mm, axial <= 0.86 mm; riding lift >= 1.90 mm, axial
>= 5.62 mm). Checked again on drive B (a correct fit placed on the drive kinematically -- the placement of `misroute_probe.py` -- a 2 s
settle with the arms at home, then the grader's own finalize): **50/50 correct fits** (seeds 1-50) pass every pulley, with I's lift
<= 1.00 mm, axial <= 0.52 mm and cover >= 0.60 (1-2 links in I's trimmed 25.6 deg sector), every grooved pulley seated.

I is a hard gate (in g, not q) because both wrong I routes can leave the pointer in the window:

| placement (12 engines each, kinematic, then the grader's settle and run-in) | tensioner after the run-in | I | score |
|---|---|---|---|
| correct (seeds 1-50) | -42.0 .. -32.4 deg | passes 50/50 | 1.0 on 50/50 |
| the belt over I (its ribs on I's top) | -42.5 .. -34.8 deg: **inside the window** | winding 1: fails 12/12 | 0 on 12/12 (WP and ALT also fail) |
| the belt in front of I's front flange (I not wrapped) | placed -25.9 .. -34.6 deg; in the run-in the span jumps | fails 12/12 | 0 on 12/12 (tensioner overspeed 95-245 deg/s) |
| T on the belt's rib side | -23.9 deg (on its closed stop) | - | 0 on 12/12 |

Separation of the three I routes at I's centre line (from the drawing; the placements above confirm them): the belt's
mid-line under I runs at z 247.7 mm (I's face 35 mm + half the belt), over I at z 322.3 mm (74.6 mm apart); a run in
front of I (clear of its front flange, belt centre at depth >= 53 mm against 27.5 mm on I's face) is at least 25.5 mm
in front of a correct run and, laid straight from ALT's top to WP's top, about 34 mm higher. All are well above the
front camera's depth noise and the honest eyes' centreline error (p95 about 5 mm).

The idler's judged arc and the element pitch: I's drawn back wrap is 33.6 deg; trimmed by 4 deg at each end it is
25.6 deg, 16.6 mm of belt at the belt's middle line, 1.66 element lengths of the longest belt (10.05 mm). Any arc of
at least one element length holds at least one element centre, so a correct wrap reads 1 or 2 elements (cover 0.6 or
1.2 against the 0.5 needed) by phase alone and never 0. `drive.py` asserts this coupling when the cell is imported
(`idler_sector_links(CELL_VARIANT) >= 1.5`), so a later change of the trim or of the pitch cannot silently make correct
fits fail.

When the window is read: the drive is inspected the moment the run-in motor stops (no further settle;
`episode.run_in`), the documents say so (cell_manual.md, Acceptance). Correct fits after the run-in sit 3.5 deg or more
inside the closed-side mark, and the run-in moves a correct fit's arm by 0.2-1.6 deg (placed vs. after), so reading
at the end of the run-in rather than after a further settle does not decide any correct fit.

Window: (-47, -28) deg, set from these data (correct fits at rest -42.8 .. -31.5 deg over the 74 placements, corners
included; the closed-side mark 4 deg from the closed stop -24 deg, 3.5 deg beyond the nearest correct fit).

## Run-in slip of correct fits over the hidden ranges

Hidden ranges: belt-pulley friction mu 0.6-0.9; flatwise stiffness EI 0.6-1.2 x 0.012 N m^2; spring preload 1.84 N m and
rate 0.83 N m/rad, each +-15 %; belt length +-0.5 % of 1.21 m; drag 0.2-0.4 N m per accessory; mass 0.22-0.26 kg/m; I's
bearing friction 0.005-0.02 N m. Slip = max over WP and ALT of |1 - accessory angle / (crank angle x ratio)|.

- Correct fits, seeds 1-50: slip 0.1-1.01 % (p95 0.77 %).
- Corners (placed correct fits, both drags 0.4 N m, I bearing 0.02 N m): EI {0.6, 1.2, 1.5} x length {0.995, 1.005} x
  spring {0.85, 1.15} x mu {0.6, 0.9}, 24 corners: all pass, slip <= 1.96 % (the worst at EI 1.5 x, beyond the hidden
  range), tensioner -42.8 .. -31.5 deg.

Contact-model robustness (softer contact, solref x2) and the flat-belt friction note are properties of the shared
belt and groove model and were not re-measured on drive B: the run-in verdicts are stated for the nominal contact.
Physical note: the belt is modelled flat (no V-rib wedge action); a poly-V belt's effective friction is about
mu / sin 20 deg, so the simulated slip over-states a real belt's; the 3 % run-in spec is kept at its real-world value.

## The honest program (the oracle)

`solution/honest/` (byte-identical to `tests/review/honest/`): public API, the given documents, the given Panda model,
the cameras and the force/torque sensors only. Its record by version is in `oracle_record.md`. Development seeds 1-12
reached 12/12 in the seventh correction round; the same version scored 12/12 on seeds 13-24 (never used in
development) and, in the pre-registered evaluation (`full_at_rule.md`: 16 sealed fresh engines + the 4 hidden
engines), 19/20 on the development host and 18/20 in the task image (full_at 0.83 from the lower). The failures: a
protective stop on fresh engine f03 (the right hand's joint transit to a seating push at C swept low and struck the
pallet, 467 N; the program's own kinematic model holds the engine's face but not the pallet or the base plate), and in
the image fresh engine f01, where the C step's carry check read 0.82 N against its 1.00 N threshold and the program
gave the step up.

What the honest program needed on drive B, for the record: laying the loop's left side over WP from the upper left at
z 0.325 m puts the top run under I by itself (44 of 44 episodes of the final version needed no tuck); entering the
plane higher (0.365 m, as on the earlier drive) left the run over I on several of the development engines, where the pointer
still read inside the window -- the trap the I gate is for.

## Sensor audit and exposure (`tests/review/observe.py`, the 4 hidden engines, cell 0.3.0)

Generated by `observe.py` (in-process transport to the cell with its sensor model; the honest program's own eyes from
`tests/review/honest/`). Values: range over the 4 hidden engines.

| what | spec (cell_manual.md) | measured | method |
|---|---|---|---|
| engine pose (honest eyes, front frame) | +-4 mm / +-1 deg yaw / +-0.5 deg roll of nominal | error at the pulley centres 0.10-1.63 mm (uncalibrated estimator 1.8-3.1 mm; the bias is measured on the given nominal-pose survey frame and removed); face normal 0.001-0.002 deg | face-plane fit + the grooved pulleys' front faces, rigid in-plane fit (a face disk partly hidden in the frame is left out) |
| tensioner pointer (front frame, at the closed stop) | window marks [-47, -28] deg | read -25.41 .. -23.93 deg (true -23.93) | the white blade's pixels inside the marks' inner radius (< 39.5 mm from the pivot) |
| loop on its peg (right wrist frame, 0.32-0.46 m) | belt black (15, 15, 15) | centreline p50 2.9-3.4 mm, p95 4.8-5.3 mm; depth p50 1.2-1.3 mm, p95 1.5-1.6 mm | dark pixels, polar binning, front edge less half the width |
| F/T at rest (Cartesian and joint mode) | 0.3 N / 0.02 N m white noise per axis | 0.27-0.35 N, 0.018-0.023 N m rms | 200 ticks still |
| F/T bias | within +-1 N / +-0.05 N m, removed by tare | mean of reading - (true wrench + drawn bias) within 0.04 N | as above |
| hand weight in the reading | about 7.5 N | 7.44-7.49 N | rest reading less the bias |
| depth noise, front | 1.0 mm + 2.5 mm/m^2 z^2 | per pixel, measured noise over the spec at that pixel's depth: 1.01 | two frames of a still scene |
| depth noise, wrist | 0.2 mm + 1.5 mm/m^2 z^2 | 0.506 mm at 0.456 m (spec 0.512 mm); per-pixel ratio 1.005 | same |
| invalid depth | 2 % random + depth edges | front 3.2 %; wrist ~39 % at the home pose (beyond its 0.5 m range) | same |
| colour noise | about 2 grey levels | 2.01-2.02 | same |

**Exposure** (the clean front render's 5th / 95th-percentile channel per feature, under every combination of the
documented extremes, gain 0.94 / 1.06 and brightness 0.92 / 1.08):

| feature | darkest channel (p5, worst) | brightest channel (p95, worst) | saturated pixels (worst) |
|---|---|---|---|
| belt | 8.6 | 22.9 | 0 |
| plate | 73.5 | 207.2 | 0 |
| engine face / block | 98.6 | 169.4 | 0 |
| pallet | 57.1 | 254.1 | 3 % |
| tensioner arm | 57.9 | 113.3 | 0 |
| peg / post | 21.6 | 52.7 | 0 |
| pulley steel (C, ALT, I, WP, T) | 158.3 | 290.8 (clips at 255) | 13 % |
| pointer / window marks | 192.9 / 165.2 | 291.9 (clips) | 96-100 % |
| belt print | 111.8 | 291.9 (clips) | 76 % |

The belt is never brighter than 23 while the surfaces next to it are never darker than 57 (the honest program's belt
threshold is 40); the pointer and the window marks clip to white but are separated from the steel pulleys by shape and
position (a thin blade 30-46 mm from the pivot; marks 40-52 mm). The peg's post overlaps the belt's dark range only in
the darkest corner; the honest program masks it by its drawn position.
