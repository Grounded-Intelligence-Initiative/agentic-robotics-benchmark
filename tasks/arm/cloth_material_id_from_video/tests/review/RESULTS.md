# Checks on the hidden set (verify / maintainers only; never shown to the agent)

Everything here was registered in `tests/PREREG.md` before the draw and is reported as it came out.

## The draw (`tests/ref/selection.json`)

Eleven fabric candidates examined in order, eleven accepted (A-J and P), none rejected. The reference scored 1.0 on
94.3 % of the slide and 86.4 % of the overhang test candidates; the first four such candidates per skill are each
fabric's tests. The calibration moved the thickness by +1 % to +17 % and the friction by at most 4 %.

## Fixed submissions, by exact execution (`mutants.py`, `mutants.json`; 2760 runs)

| submission | metric | slide mean | overhang mean |
|---|---|---|---|
| reference | 1.000 | 1.00 | 1.00 |
| real materials as they are | 0.868 | 0.98 | 0.88 |
| reference, friction x 0.95 | 0.810 | 0.84 | 0.96 |
| reference, friction x 0.9 | 0.281 | 0.30 | 0.94 |
| reference, friction x 1.1 | 0.546 | 0.57 | 0.96 |
| reference, thickness x 0.9 | 0.735 | 0.98 | 0.75 |
| reference, thickness x 1.1 | 0.588 | 1.00 | 0.59 |
| reference, thickness x 1.2 | 0.237 | 0.98 | 0.24 |
| reference, Young's modulus x 2 | 0.132 | 0.93 | 0.15 |
| reference, Young's modulus x 0.5 | 0.216 | 0.82 | 0.25 |
| reference, damping x 3 | 0.906 | 0.95 | 0.95 |
| reference, Poisson's ratio x 0.7 | 0.924 | 1.00 | 0.92 |
| friction from the reference, stiffness blind | 0.086 | 0.92 | 0.09 |
| stiffness from the reference, friction blind | 0.142 | 0.14 | 0.95 |
| the middle of every range | 0.000 | 0.26 | 0.05 |
| the first version's example material | 0.000 | 0.03 | 0.29 |
| the first version's practice material | 0.000 | 0.00 | 0.05 |
| fabric A's reference for all ten | 0.175 | 0.31 | 0.28 |
| fabric B's reference for all ten | 0.118 | 0.35 | 0.27 |
| fabric C's reference for all ten | 0.100 | 0.17 | 0.29 |
| fabric D's reference for all ten | 0.100 | 0.10 | 0.13 |
| fabric E's reference for all ten | 0.139 | 0.33 | 0.27 |
| fabric F's reference for all ten | 0.131 | 0.16 | 0.25 |
| fabric G's reference for all ten | 0.126 | 0.24 | 0.29 |
| fabric H's reference for all ten | 0.113 | 0.24 | 0.27 |
| fabric I's reference for all ten | 0.100 | 0.10 | 0.25 |
| fabric J's reference for all ten | 0.135 | 0.19 | 0.26 |
| best blind constant of the grid, executed in full (friction 0.7, thickness 0.0020) | 0.097 | 0.23 | 0.23 |

Grid of blind constants (8 friction x 7 thickness values, Young's modulus 1e5 Pa), product of the skill means per
fabric: the five best are 0.098 (friction 0.7, thickness 0.0020), 0.086 (friction 0.9, thickness 0.0045), 0.084 (friction 0.6, thickness 0.0065), 0.076 (friction 0.7, thickness 0.0090), 0.067 (friction 0.3, thickness 0.0090); the best one executed in full as one
submission is the last row of the table.

Against what was registered:
- the reference 1.0, no file 0.0 (`ale validate`), the real materials below the reference and above 0.5: as expected;
- friction: x 0.95 gives 0.81 and x 0.9 gives 0.28 as expected; **x 1.1 gives 0.55, where "under 0.5" was registered**;
- thickness and Young's modulus: as expected; damping and Poisson's ratio above 0.9: barely scored, as registered;
- friction from the reference with blind stiffness 0.09; stiffness from the reference with blind friction 0.14 (a
  friction of 0.55 is within 5 % of two of the ten fabrics): the product pays neither;
- the range middle, the first version's example and practice materials: 0.000;
- the best blind constant of the grid, executed in full: 0.097;
- **one fabric's reference written for all ten: 0.100 to 0.175, where "about 0.1, not more" was registered.** Fabric
  A's reference scores 0.175 because fabrics A and E came out close (friction 0.846 and 0.843, bending length 6.4
  and 6.9 cm); five more rows are between 0.12 and 0.14. These are not blind constants (each is one fabric's
  answer), and nothing was changed after seeing them.

## What the recordings carry (`observability.py`, `observability.json`; privileged, not a solution)

Each fabric's own drag and pull clips replayed on the real model with the quantity changed by the sensitivity step.

| clips | step | shift of the measurable (mm): min / median / max | in pixels of the best camera: min / median / max | drawn surface minus model (mm): mean, range |
|---|---|---|---|---|
| drag_release (30) | friction x 1.05 | 0.28 / 1.86 / 11.77 | 0.15 / 0.93 / 8.39 | -0.001, -0.005 to 0.003 |
| pull_over_edge (20) | thickness x 1.08 | 0.27 / 2.17 / 9.59 | 0.22 / 1.83 / 9.29 | -0.29, -0.45 to 0.04 |

Per fabric, its best clip: friction x 1.05 moves the stop position by 2.5 to 11.8 mm (1.3 to 8.4 px); thickness x 1.08
moves the pulled edge's height by 3.4 to 9.6 mm (3.0 to 9.3 px) for nine fabrics and by 1.1 mm (0.9 px) for G, the
limpest (bending length 3.2 cm), whose tests are as insensitive to thickness. An edge on a settled frame can be
placed to a fraction of a pixel, so the recordings support friction to about 5 % and bending length to about 8 %;
the slowest drag of a fabric carries the least (0.15 px), the fastest the most.

The recordings draw a shell laid on the model's vertices (`environment/gen/csheet.py`): what a camera reads from it
differs from the scored quantity by 0.001 mm (stop position) and by 0.3 mm on average, 0.45 mm at most (edge height),
against tolerances of 3 and 6 mm.

## The arm in the recordings (`clip_checks.json`, written by the renderer)

77 clips: the arm follows the clamp to 0.97 mm and 0.015 degrees at worst, stays 0.13 rad or more from
every joint limit, 12 mm or more from the table and 3.3 mm or more from the fabric (the clamp's own
jaws left out: they close on the fabric where it lies). Over the extremes of both scored skills the same arm
reaches every pose with under 1 mm of error and 0.12 rad to a limit.

## Numerics

`tests/numerics.json` was computed in the task image's Python under the image's pin on an AVX2 host (Zen 3) and on an
AVX-512 host (Zen 5): the same fingerprint. Without the pin the AVX-512 host gives another one (the real model's and
NumPy's parts differ, the planning simulator's does not).
