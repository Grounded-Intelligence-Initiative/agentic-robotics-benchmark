# Perception baseline: the job from what a tested agent has

A submission program for this task (`run.sh` and what it needs, the contract of `instruction.md`) that takes every
pack apart from the frames, the cell's published numbers (the cameras' intrinsics, nominal mounting and data sheets,
the calibration plates), the documents and the nutrunner's readings. It carries no token, gets no `truth`, and reads
nothing of `tests/` but its own folder at run time. It is maintainers' evidence that the task can be solved without
the privileged view (`tests/README.md`, sections 4 to 6); nothing runs it in grading.

There are two versions. The first (frozen at `82e5f2f`) was written for version 0.1.0 of the cell, whose depth was
exact and whose cameras stood exactly at their published poses; it scored 1.0 on the hidden set, which showed that
the task was too easy (`tests/README.md`, section 6). The cell then got an active-stereo sensor's depth, cameras off
their nominal mounting by a per-cell offset, and sealant over some heads (section 2 of the same file). The first
version scores 0 on that cell (section "Results" below). This folder holds the second version, revised for it.

## What it does

`controller.py` is a generator-based program in the reference's structure (one action per step; a repeated step is
answered from the observation it carries, the program advancing once per `t`). Its motion skills and the order of the
job are adapted from `solution/reference/controller.py` (as its docstring says); every value the reference takes
from `truth` comes here from perception, from the manual or from this program's own bookkeeping instead:

| the reference reads from `truth` | the baseline uses |
|---|---|
| (nothing: the reference needs no camera) | **this cell's cameras**, identified at the start of every pack (`calib.py`): the overhead camera by one PnP on the inner corners of both calibration plates in one overhead frame (each plate found in a crop around its nominal projection, trying the colour channels, zooms and a contrast boost, because the renderer's denoiser can wash a colour over part of a plate; up to six frames until both plates are found with under 0.6 px of error); the wrist camera's place on the flange from views of the near plate (the bit's tip 0.10 m above the plate's centre, two yaws, square and tilted 8 degrees): each whole-board view gives the camera in the base frame by PnP, the frame's nominal `pose` gives the flange's pose (it is the flange's pose times the nominal mounting), and the views that agree within 0.8 mm with the most central one are averaged |
| the variant | the lid's ribs in an overhead depth frame (each variant's rib lines of the manual, section 2, sampled for a 2 to 6 mm rise), confirmed after the lid is off by the module handles standing where the manual puts them |
| the pack's placement | the silhouette of the pack's top (lid, or the tray's walls) in the overhead depth, its four sides fitted as lines in the base frame |
| each fastener's place, presence and head | from the overhead frames, averaged over six (the depth's noise is 2.5 mm there and independent from frame to frame): the median height of the place's core above the median of a ring around it says head, hole (or no reading on a dark spot) or something lying over it, and the bump gives a first position; the head's type is left to a wrist frame, which then looks at each fastener with the head on its axis: the head's flat top is the dominant height plateau above the seat, its median height gives the type (M6: S10 6 mm, T30 4.5 mm; M8: S13 8 mm, T40 6 mm), and the head's outline is fitted to the plateau's rim where it drops to the seat (a circle of the pan head's radius, a hexagon of the head's width across flats with its turn free): under sealant or behind a neighbour there is no such drop, so the fit uses what is left of the rim. Without enough rim (less than 45 % of the circumference) the region standing above the seat is taken, but only when its area is a bare head's; otherwise the camera looks from the other side, then from a third |
| corroded or not | not needed: a fastener that stalls at the first limit (1.3 times the assembly torque) is pressed again with the limit raised to twice the assembly torque, within the thread's "never exceed" value (manual, section 5) |
| `captured` (the fastener is out) | the nutrunner's trace: the breakaway, then the manual's turns to free, then a free-running spindle; `eject: ok` from the cell confirms it |
| the harness' path | the blue cable in the overhead frame (on the lid; inside, after the lid), a grasp across a straight, level stretch of it, at the height of its centre from the averaged depth |
| the parts' grip points | the manual's grip points and closing directions (section 4) placed with the pack's placement |
| a part still held | the program's own record of which fasteners came out; a lift that ends in a protective stop lets go |
| isolation | the time the service disconnect came out (gripper holding after the pull) plus the manual's 30 s |
| the exact commanded pose | the program's own record of the pose it last commanded completely (a repeated step's copy carries shifted readings); a wrist frame's pose is used only when it agrees with that record |

`vision.py` holds the perception (NumPy and OpenCV only), `calib.py` the calibration, `manual.json` the manual's
tables (transcribed by `dev/parse_manual.py` from `given/service_manual.md`), `cell.json` a copy of
`given/cell/cell.json`. Numbers measured on development frames: the ratio of a bare head's hull to its nominal area
(`HULL_OVER_AREA`), the rim's inset (`RIM_INSET`, 0.15 mm), the blue of the cable.

## How it was developed

Only on the four development packs of `environment/commission_service/dev_packs.json`, with tools of its own in
`dev/` (maintainers' tools; they start a cell with the grader's switch, which the program never sees):

- `dev/survey.py`: the privileged reference does the job on a development pack while the tool takes overhead frames
  at the stages (eight at the lid), twenty wrist views of the near calibration plate and wrist frames over every
  fastener, with the truth beside them (the cell's true camera mounting included). The perception was written and
  measured offline on those frames.
- `dev/devrun.py`: the program's `run.sh` behind the wire on a development pack, with the grader's deadlines and
  repeated steps (shifted joints, `packcell_client.perturbed`), then the credit, and the program's own estimates
  (written only when `PB_DEBUG_DIR` is set) against the truth: `dev/compare.py`, `dev/stage_table.py`. With
  `DEVRUN_TRACE=1` it also records the gripper against the parts' grip points.

Development, 2026-11-25, 14:10 to 19:05 (the cell changed under it twice: sealant at 15:38, the look at 17:27).
Cell sessions on development packs: 29 in all (a tested agent gets ten): six surveys (two on each version of the
cell), one probe of the plates' use before the offsets were drawn (development pack 3 with offsets picked by hand),
four closed-loop rounds of four packs, one more run stopped after its first pack (nb3, the nb2 program on an
intermediate tree), and six sessions to find the one failure below (two traced runs, two runs of a copy that forced
the failing order, one isolated reproduction, one traced run that did not fail). Credit per development pack:

| round | cell | what the program had | 1: P4, harness on busbars | 2: P5, harness on lid | 3: P6 | 4: P6, harness on lid |
|---|---|---|---|---|---|---|
| nb1 | cameras and sensor (`21e2d44`) | calibration, averaged overhead frames, a head as the hull of what stands above its seat | 0.06 | 1.0 | 1.0 | 1.0 |
| nb2 | + sealant (`6b259e7`) | + the rim fit, three sides | 1.0 | 1.0 | 1.0 | 1.0 |
| nb4 | + the look (`9f20b46`) | + white balance, hull silhouette | 0.06 | 0.06 | 1.0 | 1.0 |
| nb5 | the same | + exchanges cross the cell at -0.10 m | 1.0 | 1.0 | 1.0 | 0.995 |

The 0.06 of nb1 and nb4 was the program's own fault, found with a trace and a copy of the program that forced the
order: when the last lid bolt out was one re-typed at the wrist (and so left to the end), it was ejected into the
left fasteners bin, and the swing toward the tool stand at the dock's approach height (the tool's centre 0.18 m above
the table) caught the freed lid's handle (0.206 m) and dragged the lid to the other side. Tool and bit exchanges now
cross the cell with the tool's centre at -0.10 m (0.30 m above the table); with that order forced, the pack scored
1.0. The 0.995 of nb5 is one busbar bolt that bounced off the fasteners bin's wall onto the table.

Perception in round nb5 against the development truth (`dev/compare.py`): the pack's placement within 0.92 mm and
0.02 degrees; the variant right on all four; the overhead camera identified so that a point on the lid is at most
0.48 to 1.05 mm off; the wrist camera's mounting so that a point 0.35 m ahead is 0.18 to 0.19 mm off; fasteners from
the averaged overhead frames median 0.52 to 1.18 mm, at most 2.78 mm; from the wrist frame median 0.14 to 0.24 mm, at
most 2.20 mm (that one a socket head, inside its 2 mm with the bit's float); every seated head typed right and taken
out, every missing one seen as a hole. White balance from the plates: the plate's white read 154 to 185 levels per
channel across the four packs' lights.

## How to run it

As a submission: copy `run.sh`, `main.py`, `controller.py`, `vision.py`, `calib.py`, `manual.json` and `cell.json`
into `/home/user/submission/`. The measurement tools need Isaac Sim and the cell (`TASK` pointing at the task folder,
`PACKCELL_*` as for `review/check_observability.py`):

    python3 dev/devrun.py <dev pack index 0..3> <program folder> <out folder>
    python3 dev/compare.py <out folder>; python3 dev/stage_table.py <out folder> ...

## Results

RESULTS_HIDDEN

## What it does not do

- It has no code for a stripped head (the extractor bit): nothing it did stripped one, and no field pack arrives
  with one.
- It reads rust from nothing (the stall says it), and logs to stderr only.
- It does not tell sealant from a head by colour: it relies on the depth's geometry (a flat plateau with a rim) and
  on looking from another side.
