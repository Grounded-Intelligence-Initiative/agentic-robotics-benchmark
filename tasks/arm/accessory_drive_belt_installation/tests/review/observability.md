# Observability and acceptance evidence (grader-side; not shipped to the agent)

This file collects the measurements behind the cell's acceptance and the sensor audit. Sections marked *pending* are
filled by `tests/review/observe.py` once the sensor audit is run on the hidden engines.

## Groove acceptance thresholds (grader-only constants in `tests/beltcell/drive.py`, `ACCEPT`)

The agent's documents say only that every rib section must sit in its groove zone on the groove floor, not on a
shoulder or edge, and give the groove geometry (22 mm zone, 2.5 mm shoulders, 45 deg front chamfer). The numbers below
stay in `tests/`.

Definitions (after the run-in, per grooved pulley, over the belt links on the pulley inside its drawn wrap sector
trimmed by 12 deg at each end):

- **lift** (gross) = link-centre radius - (groove-floor radius + half the belt's thickness). A seated link reads about
  +0.5 mm, because the belt's sphere colliders rest at their 0.5 mm contact margin; the value is not corrected for it.
- **axial** = |belt-centre depth - groove-zone centre depth| (zone centre 27 mm in front of the face).

Data (2026-11-09, 564 pulley samples): 50 correct fits placed on the drive (the nominal correct-fit batch), 38 robot-free
privileged installs (seeds 13-50, mocap hands under the cell's force limits), 50 + 50 belts started riding the front
shoulder of WP or C (shifted 11 mm and lifted onto the shoulder top).

| cluster | n | lift (mm) | axial (mm) |
|---|---|---|---|
| seated (placed, installed, and riding starts that self-seated) | 395 | max 1.06 (placed max 0.86, installed max 0.92; median 0.50) | max 0.86 |
| riding a shoulder | 169 | 1.90 - 2.87 | 5.62 - 13.52 |

Thresholds at the midpoints of the gaps: **lift 1.5 mm** (gap 1.06 .. 1.90), **axial 3.2 mm** (gap 0.86 .. 5.62). No
sample falls on different sides of the two criteria (lift and axial agree on every sample).

Riding starts: WP stays riding (52 of 150 WP-start pulley samples high), C walks the whole belt onto the front shoulders
during the run-in (117 of 150 high); ALT starts self-seat in the run-in (flat-belt tracking: every link entering ALT
follows the in-plane span). Counts on the final physics (seeds 1-50): a single-pulley
shoulder start fails 33/50 at nominal contacts and 37/50 at solref x2 (the ALT starts are the ones that pass).

Soundness checks on the final physics (mu 0.6-0.9, EI 0.6-1.2 x, preload 1.84 N m, the 105-link belt; seeds 1-50): correct fits
pass 50/50 at nominal contacts (slip 0.10-1.06 %, rest -146.2 .. -138.2 deg) and 50/50 at solref x2 (slip 0.70-2.74 %);
slack belts (the tensioner left at its install stop) fail 50/50 at both (slip 99.6-99.95 %); the free release of the
lever with the tensioner off the belt reaches 118-266 deg/s (damage), the guided return at most 6.3 deg/s; a welded
link pulled to slip lets go in 100/100 pulls without a physics fault.

## Contact-parameter robustness

Verdicts were checked only towards softer contact: at solref x2 (8 ms) correct fits pass 50/50 (slip <= 2.74 %),
slack belts fail 50/50, shoulder starts fail 37/50 (final physics). Stiffer contact cannot be run at the 2 ms step: MuJoCo's refsafe
clamps the time constant to 2 x timestep (solref x0.5 is bit-identical to x1), and with refsafe disabled every run
diverges (6/6 BADQACC); a stiffer impedance (solimp 0.99/0.999/0.25 mm) also faults or slips. The run-in verdicts are
therefore stated for the nominal and softer contact models only.

## Run-in slip of correct fits over the hidden ranges

Final ranges (2026-11-09): belt-pulley friction mu 0.6-0.9; flatwise stiffness EI 0.6-1.2 x 0.012 N m^2 (droop class
32-71 mm: a 200 mm free end held level, measured on the cell's own chain for EI 0.6-1.2 x and mass 0.9-1.1 x); spring
preload 1.84 N m and rate 0.83 N m/rad, each +-15 %; belt length +-0.5 %; drag 0.2-0.4 N m per accessory; mass
0.22-0.26 kg/m. Slip = max over WP and ALT of |1 - accessory angle / (crank angle x ratio)| over the 4 s run-in.

Corner sweep: a correct fit placed on the drive (the nominal correct-fit placement) at every corner of mu {0.6, 0.9} x drag (both
accessories) {0.2, 0.4} x spring (rate and preload together) {0.85, 1.15} x length {0.995, 1.005} x EI {0.6, 1.2} x
mass {0.9, 1.1} (64 corners), run in at the nominal contact (x1) and at contact solref x2 (x2):

| mu | EI | x1 max (median) | x2 max (median) | x2 corners above 3 % |
|---|---|---|---|---|
| 0.6 | 0.6 x | 1.05 % (0.61 %) | 2.85 % (1.97 %) | 0 / 16 |
| 0.6 | 1.2 x | 2.12 % (0.61 %) | 3.51 % (1.47 %) | 2 / 16 |
| 0.9 | 0.6 x | 1.19 % (1.02 %) | 3.19 % (2.63 %) | 4 / 16 |
| 0.9 | 1.2 x | 1.07 % (0.79 %) | 2.82 % (2.03 %) | 0 / 16 |

All 128 placements pass the groove and face inspection; the tensioner rests at -147.2 .. -137.1 deg (window -149 ..
-133 deg). The x1 slip-worst corner is mu 0.6, drag 0.4, spring x0.85, length 1.005, EI 1.2 x, mass 1.1 x: 2.12 %
placed; robot-free mocap installs at that corner (mass 0.9 and 1.1 x) end at WP / ALT 1.93 / 2.49 % and 1.95 / 2.48 %.
The corner sweep fixes the draws it does not vary (pivot friction, damper, engine pose) by its seed; at the slip-worst
corner those draws were varied too (pivot friction {0.04, 0.12} N m x damper {0.7, 1.3} N m s/rad x three engine poses):
x1 1.66-2.37 %, x2 3.24-4.03 %. Every x1 corner therefore meets the 3 % spec with at least 0.5 % margin (placed max
2.37 %, installed max 2.49 %).

x2 (the softer contact, a robustness check; not tuned): 6 of the 64 corners exceed 3 %, by at most 0.51 %: two at
mu 0.6 + drag 0.4 + spring x0.85 + length 1.005 + EI 1.2 x (3.33 %, 3.51 %), four at mu 0.9 + drag 0.2 + spring x1.15 +
EI 0.6 x (3.02-3.19 %); with the slip-worst corner's other draws varied, up to 4.03 % (by 1.03 %). Under the softer
contact a correct fit of an engine near those corners could fail the slip item.

Below the range (spot check, mu 0.55 at EI 1.2 x, spring x0.85, length 1.005, drag {0.2, 0.4} x mass {0.9, 1.1}):
x1 0.63-2.48 %, x2 1.71-4.90 %.

How the ranges were set: with mu 0.5-0.9 and EI up to 1.7 x three x1 corners exceeded 3 % (max 8.8 %); mu 0.6-0.9 left
two (max 4.65 %, EI 1.7 x); EI <= 1.4 x left one (3.38 %); EI <= 1.2 x leaves none above 2.12 %. The slip-worst corners
are always the low-tension, high-load ones: soft spring with the longest belt (the tensioner rests nearest its closed
stop, at the lowest spring torque), drag 0.4, the lowest mu and the stiffest belt. The residual noted at the preload
decision (those four parameters at their extremes together, joint probability of order 1e-4) is removed for x1 by the
narrower mu and EI ranges; for x2 it remains at the corners listed above. The hidden set is not filtered on slip.

Physical note: the belt is modelled flat (no V-rib wedge action). A poly-V belt's ribs wedge in the grooves, which
raises its effective friction to about mu / sin 20 deg = 2.9 mu, so the flat-belt mu simulated here under-states the
real grip and the simulated slip is a conservative over-estimate; the 3 % run-in spec is kept at its real-world value.

## The development reference (privileged eyes, public decisions)

The development reference (the contributor's development code, not shipped) runs both arms at
stiffness 1.0 through the cell's own controller and rules. Its eyes are privileged (belt link poses, the tensioner's
angle, the engine's pose); every decision that an outcome hangs on is taken on public signals: grasp success (the hand
reached the link without its in-move tripping the force limit, the closed width, then the flange load of the stage's
first carrying move), L's release in the return (its own flange F/T), the post-return check (the front camera's
pointer reading, the wrist cameras' depth over each wrap), the lever's load. The planner keeps max(8 mm, 2 x lag) from
the other arm (26 mm) and 30 mm from the tensioner's arm and lever on joint transits.

Reference check on the final physics (2026-11-09, consecutive development seeds 1-20): 20/20 accepted;
every run released L on its own F/T (lateral push at -94.4 .. -100.7 deg), margin_ok, no protective stop, no damage,
no physics fault, no grip slip (the lever hold peaked at 25.6 N against the 30 N rule; the pin-force guard of 26 N
tripped once, on seed 9's opening, which then went on at half speed); the tensioner rested at -145.0 .. -138.4 deg,
slip <= 0.75 %; 10 check-and-correct rounds over 9 seeds; the public grasp verdicts matched the grip rule's state on
every grab. The public post-return check agreed with the cell's inspection on 117 of 120 pulley readings; the three
misses were borderline front-shoulder rides (lift 2.7-2.9 mm, axial 3.2-4.4 mm) that the run-in seated.

Stress result, grasp points one link off (every chosen belt link shifted by +-1 link, 10 mm along the belt, 40 runs on seeds 1-20): 23/40 accepted. The failures: mid_pre's grab of a link one along could
not reach it (6; the fingers met T's flange or the neighbouring links), the bottom at C placed about 4 mm further out
than the base runs and not seated by the pushes, or swinging 10 cm in front of C after the release (7), ALT left on
its front shoulder going into the return and lost there (2), the planning margin cut by 1.4 mm on the drape or the
tensioner's opening (2). The C step and the drape release need the grasp point within about 5 mm: that precision is
part of what the task tests. It is within reach of the documented perception: the honest program's front-camera belt
centreline is off by 1.8 mm (median) and 4.7 mm (95th percentile) when the loop is in view, the engine's depth by
less than 0.05 mm (development measurements). The robustness criterion of the reference check is therefore set at that scale (grasp points
+-5 mm along the belt plus a perception error drawn from eyes.py's measured statistics, 19/20 under each of two
draws); the one-link result stays here as a stress figure.

Grip depth: the grip rule takes a link whose centre lies 94.4-111.4 mm along the hand's approach axis plus 3 mm of
documented slack (91.4-114.4 mm). The Franka pads (collision boxes) span 94.4-111.4 mm and the fingertip mesh ends at
112.3 mm (measured on the Menagerie model the cell builds). The validated reference's drape grips its link at 114.4 mm,
2 mm past the physical fingertip, on the slack's edge: with that grip the released loop comes off the fingers; gripped
4 mm deeper (110.4 mm) it hooks on them (base seeds 11, 12, 14, 17, 19, 20). Perception-scale grasp perturbation
(the reference check's robustness test: +-5 mm along the belt, 2.5 mm across and in depth): every perturbed engine missed the drape
grip on the 114.4 mm edge; the tip-7 mm variant with an unhook move passed 6/8 but never freed the
loop cleanly. The reference check therefore passes on the base 20/20 only; the honest program must drape with its grip
inside the physical pads (<= 112 mm), and is the robustness proof.

## The belt against the finger pads (cell 0.2.0)

The Menagerie fingertip pad is a flat block (8.5 mm half-wide, 94.4-111.4 mm along the approach axis) with four 1.5 mm
ridges in front of it (6 mm x 6-7 mm, 4 mm apart along the approach axis). The belt's contact model is a row of four
spheres across its width per 10 mm link (radius 2.2 mm, 5.2 mm apart). In cell 0.1.0 a belt left resting on an open
pad (the drape's release over WP: with any grip inside the pads the lower finger must stay 18 mm above WP's rim, so the
loop's top lies on that finger when the fingers open) could have one of its spheres nest in the 4 mm gap between two
ridges, with contact normals of about +-0.75 along the approach axis: the hand then carried the loop out of the drive
as it withdrew (honest-program development, cheat-eyes drape test: 1-4 of 4 seeds hooked depending on the release
move; contacts logged by a development script). A continuous 20 mm belt cannot do this; it is an artefact of
the belt's discretisation against the ridge boxes. Cell 0.2.0 therefore lets the belt (and a held part) touch the pads'
flat faces only: the ridges keep their contacts with everything else (rules.BITS ridge_L / ridge_R), the grip rule is
unchanged, and the manual says so ("The belt meets a finger pad on its flat face"). On 0.2.0 the validated reference (grip at 114.4 mm)
first gave 17/20 on the reference check's base seeds: on seeds 12, 15 and 19 the loop's bottom came to rest about
1 cm in front of C after the C step (depth 0.055 m against 0.044-0.046 m elsewhere) and the belt was lost in the
return. One correction (two rounds): the reference now takes the bottom once more when its depth at C after the step
exceeds 0.050 m (C_RETRY 1). Reference check, base seeds, on 0.2.0: **20/20** accepted, the retake used on
3 seeds, margin_ok on all (closest approach 36.2 mm, close-quarters lag <= 11.8 mm), lever load <= 26.2 N, the
tensioner at rest -145.0 .. -138.4 deg, slip <= 0.88 %, public check agreeing with the cell's inspection on 111 of
112 readings; soundness spot checks on 0.2.0: the welded pull-out (seeds 1-8) slips every time without a fault, the
mocap correct fit (x1, seeds 1-8) passes every time. Honest drape release (grip at 109.7 mm, inside the pads, the
belt resting on the open lower finger, cheat-eyes drape test, seeds 1-12): 0.1.0 hooked 1-3 of 4 seeds depending on
the release move (best workaround 11/12); 0.2.0 releases 12/12 with a plain open-and-withdraw (flange load after
withdrawing <= 0.86 N).

**Cell 0.2.1 (joint-mode damping capped, see the sensor audit below).** The reference changed in one place: when its
pre-C span set (mid_pre, an aid for long or flexible belts) cannot take the span, it opens, backs out and goes on
without it (seed 5: the span at z 0.16 lay against ALT's front flange, all three grabs stopped on it at 11-16 N).
Reference check, base seeds, on 0.2.1: **20/20** accepted, margin_ok on all. The public check agreed with the
cell's own inspection on 106 of 108 readings. The two disagreements are both at check0 (right after the return, before
the run-in), both in the same direction (the public check passes a pulley the cell's inspection fails), and neither
cost an episode:
- seed 7, ALT: one short section rode the front chamfer (lift 2.36 mm, axial 5.6 mm against 1.5 / documented
  "in the groove zone on the groove floor"). The reference therefore made no correction; the run-in seated it and the
  episode was accepted.
- seed 8, C: lift 2.79 mm, axial 4.8 mm. The same check flagged ALT; the ALT correction and the return_fix also
  reseated C, and check1 agreed on all four pulleys.
This is the public check's resolution, not a documentation gap. It judges each wrap from a wrist depth frame in 12
angle bins (the 90th-percentile outer radius and the median front-edge depth per bin), so a riding section one or two
10 mm links long is averaged away. The cell's inspection takes the maximum over the links. The manual states the
criterion and that the verdict comes after the run-in ("A belt on a shoulder may ride up further or walk off during the
run-in", catalog.md). Both are true here: the short riding sections reseated. No doc change.
The full_at evaluation on 0.2.1 (tests/review/full_at_rule.md, one single-episode verify per engine in the task image) gave **20/20**: the oracle accepted all 16 sealed fresh v2 engines and all 4 hidden engines, so
full_at = 0.93 and value 1.0 (tests/anchor.json).

## Sensor audit and exposure (`tests/review/observe.py`, the 4 hidden v2 engines, cell 0.2.1)

Generated by `observe.py` (in-process transport to the cell with its sensor model; the honest program's own eyes from
`tests/review/honest/`; on the contributor's workstation; the cell 0.2.0 run gave the same values except the row marked).
Values: range over the 4 engines.

| what | spec (cell_manual.md) | measured | method |
|---|---|---|---|
| engine pose (honest eyes, front frame) | +-4 mm / +-1 deg yaw / +-0.5 deg roll of nominal | error at the pulley centres 0.01-0.14 mm (uncalibrated estimator 1.9-2.0 mm; the bias is measured on the given nominal-pose survey frame and removed); face normal 0.002-0.003 deg | face-plane fit + the grooved pulleys' faces, rigid in-plane fit |
| tensioner pointer (front frame) | window marks [-149, -133] deg | read -152.19 .. -151.93 deg at the closed stop (true -152.08) | the white blade's pixels 29-40 mm from the pivot |
| loop on its peg (left wrist frame, 0.38 m) | belt black (15, 15, 15) | centreline p50 2.2-3.7 mm, p95 4.9-5.1 mm; depth p50 1.2 mm, p95 1.4-1.5 mm | dark pixels, polar binning, front edge less half the width |
| F/T at rest, Cartesian mode | 0.3 N / 0.02 N m white noise per axis | 0.27-0.34 N, 0.018-0.022 N m rms | 200 ticks still |
| F/T at rest, joint mode | (same) | 0.27-0.33 N, 0.018-0.022 N m rms | 200 ticks still at home. Cell 0.2.0 read 0.79-0.88 / 1.28-1.35 / 0.58-0.65 N rms here (the joint PD's explicit damping made the light distal joints chatter, ~60 Hz, tcp 0.15-0.34 mm); fixed in 0.2.1 by capping the damping per joint (cell_manual.md), now at rest without motion |
| F/T bias | within +-1 N / +-0.05 N m, removed by tare | mean of reading - (true wrench + drawn bias) within 0.045 N | as above |
| hand weight in the reading | about 7.5 N | 7.44-7.50 N | rest reading less the bias |
| depth noise, front | 1.0 mm + 2.5 mm/m^2 z^2 | per pixel, the measured noise over the spec at that pixel's depth: 1.00-1.01 (the median-pixel comparison, 2.8 mm vs 2.0 mm at 0.63 m, was dominated by the far pixels) | two frames of a still scene |
| depth noise, wrist | 0.2 mm + 1.5 mm/m^2 z^2 | 0.496 mm at 0.455 m (spec 0.51 mm); per-pixel ratio 1.00 | same |
| invalid depth | 2 % random + depth edges | front 3.1-3.2 %; wrist ~40 % at the home pose (beyond its 0.5 m range) | same |
| colour noise | about 2 grey levels | 2.01-2.02 | same |

**Exposure.** For every feature, the clean front render's 5th / 95th-percentile channel values
multiplied by every combination of the documented extremes (channel gain 0.94 / 1.06, brightness 0.92 / 1.08):

| feature | darkest channel (p5, worst) | brightest channel (p95, worst) | saturated pixels (worst) |
|---|---|---|---|
| belt | 8.6-9.5 | 22.9 | 0 |
| plate | 71.8-73.5 | 209.5-214.1 | 0 |
| engine face / block | 98.6 | 169.4 | 0 |
| pallet | 57.1 | 254.1 | 3 % |
| tensioner arm | 57.9 | 112.2 | 0 |
| peg / post | 21.6 | 52.7 | 0 |
| pulley steel | 158.3 | 291.9 (clips at 255) | 15 % |
| pointer / window marks | 165.2 | 291.9 (clips) | 84-88 % |
| belt print | 119-131 | 291.9 (clips) | 68-71 % |

The contrasts a program relies on survive the whole documented range: the belt is never brighter than 23 while the
next-darkest surfaces near it (tensioner arm, pallet) are never darker than 57 (the honest program's belt threshold:
40); the pointer and the window marks clip to white at the bright end but stay separable from the steel pulleys by shape
(a thin radial blade 29-46 mm from the pivot). The peg (blue, max channel <= 53) overlaps the belt's dark range only in
the darkest corner; the honest program masks the peg's post by its drawn position.
