# Perception baseline: the job from what a tested agent has

A submission program for this task (`run.sh` and what it needs, the contract of `instruction.md`) that takes every
pack apart from the frames, the published calibration, the documents and the nutrunner's readings. It carries no
token, gets no `truth`, and reads nothing of `tests/` but its own folder at run time. It is maintainers' evidence
that the task can be solved without the privileged view (`tests/README.md`, sections 4 to 6); nothing runs it in
grading.

## What it does

`controller.py` is a generator-based program in the reference's structure (one action per step; a repeated step is
answered from the observation it carries, the program advancing once per `t`). Its motion skills and the order of the
job are adapted from `solution/reference/controller.py` (as its docstring says); every value the reference takes
from `truth` comes from perception, from the manual or from the program's own bookkeeping instead:

| the reference reads from `truth` | the baseline uses |
|---|---|
| the variant | the lid's ribs in an overhead depth frame (each variant's rib lines of the manual, section 2, sampled for a 2 to 6 mm rise), confirmed after the lid is off by the module handles standing where the manual puts them |
| the pack's placement | the silhouette of the pack's top (lid, or the tray's walls) in the overhead depth frame, its four sides fitted as lines in the base frame |
| each fastener's place, presence and head | from the overhead depth frame: a bump at the manual's place, its height above the seat gives the head (M6: S10 6 mm, T30 4.5 mm; M8: S13 8 mm, T40 6 mm; manual, section 5), no bump means missing or hidden; then one wrist frame per fastener, taken with the head on the camera's axis: the depth points of the head's top face give its centre and height again, or an empty hole |
| corroded or not | not needed: a fastener that stalls at the first limit (1.3 times the assembly torque) is pressed again with the limit raised to twice the assembly torque, within the thread's "never exceed" value (manual, section 5) |
| `captured` (the fastener is out) | the nutrunner's trace: the breakaway, then the manual's turns to free, then a free-running spindle; `eject: ok` from the cell confirms it |
| the harness' path | the blue cable in the overhead frame (on the lid; inside, after the lid), a grasp across a straight, level stretch of it, at the height of its centre from the depth |
| the parts' grip points | the manual's grip points and closing directions (section 4) placed with the pack's placement |
| a part still held | the program's own record of which fasteners came out; a lift that ends in a protective stop lets go |
| isolation | the time the service disconnect came out (gripper holding after the pull) plus the manual's 30 s |
| the exact commanded pose | the program's own record of the pose it last commanded completely (a repeated step's copy carries shifted readings); a wrist frame's pose is used only when it agrees with that record |

Frames per pack: about 4 overhead and one wrist frame per fastener (32 to 44 of the 400 allowed).

`vision.py` holds the perception (NumPy and OpenCV only), `manual.json` the manual's tables (transcribed by
`dev/parse_manual.py` from `given/service_manual.md`), `cell.json` a copy of `given/cell/cell.json`. Two numbers were
measured on development frames: the size in pixels of a whole head's top face in a wrist frame taken 0.105 m above it
(`TOP_FACE_PX`, used to notice a head partly hidden and look again from the other side) and the blue of the cable.

## How it was developed

Only on the four development packs of `environment/commission_service/dev_packs.json`, with tools of its own in
`dev/` (maintainers' tools; they start a cell with the grader's switch, which the program never sees):

- `dev/survey.py`: the privileged reference does the job on a development pack while the tool takes overhead frames at
  the stages and wrist frames over every fastener, with the truth beside them. Run on development packs 3, 1 and 2.
  The perception was written and measured offline on those frames.
- `dev/devrun.py`: the program's `run.sh` behind the wire on a development pack, with the grader's deadlines and
  repeated steps (shifted joints, `packcell_client.perturbed`), then the credit, and the program's own estimates
  (written only when `PB_DEBUG_DIR` is set) against the truth: `dev/compare.py`, `dev/stage_table.py`.

Cell sessions on development packs, 2026-11-25 (a tested agent gets ten, with no truth): eleven, all on the four
development packs (numbered 1 to 4 as the commissioning service numbers them; the tools take the index 0 to 3).
Three survey sessions (development packs 3, 1, 2; a fourth on pack 4 was stopped as it started); four closed-loop
runs of the first complete program (round r1); four of the frozen program (round r2, after two retries were added
that the development packs never needed: another place along the harness when a grasp on it fails, and a second try
of every pick). The program was frozen after round r2 and then run once on the hidden set.

## How to run it

As a submission: copy `run.sh`, `main.py`, `controller.py`, `vision.py`, `manual.json` and `cell.json` into
`/home/user/submission/`. The measurement tools need Isaac Sim and the cell (`TASK` pointing at the task folder,
`PACKCELL_*` as for `review/check_observability.py`):

    python3 dev/devrun.py <dev pack index 0..3> <program folder> <out folder>
    python3 dev/compare.py <out folder>; python3 dev/stage_table.py <out folder> ...

## Results

**Development packs** (native, the author's server; `dev/devrun.py`; rounds r1 and r2 gave the same credit on every
pack, the table is r2, the frozen program). Value recovered per stage of the job over the value in the pack
(fasteners 0.5 each, with their group; missing fasteners are not in the pack):

| pack | lid (+ bolts) | MSD, plugs, harness | busbars (+ bolts) | brackets (+ bolts) | modules | credit | simulated s | steps | frames | program s |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: P4, harness on the busbars | 9.5/9.5 | 6/6 | 15/15 | 12/12 | 32/32 | 1.0 | 711 | 1084 | 32 | 7.0 |
| 2: P5, harness on the lid | 9.5/9.5 | 6/6 | 18/18 | 14.5/14.5 | 40/40 | 1.0 | 859 | 1289 | 38 | 8.8 |
| 3: P6, no harness | 12/12 | 5/5 | 21/21 | 17.5/17.5 | 48/48 | 1.0 | 947 | 1479 | 43 | 8.4 |
| 4: P6, harness on the lid | 11.5/11.5 | 6/6 | 21/21 | 17.5/17.5 | 48/48 | 1.0 | 991 | 1521 | 44 | 8.9 |

No protective stop, nothing stripped, every component where it belongs. Perception against the development truth
(`dev/compare.py`, round r2; the same in r1):

| pack | variant | placement error | heads typed right | missing seen as holes | overhead position error, median / max | wrist position error, median / max |
|---|---|---|---|---|---|---|
| 1 | right | 0.43 / 0.43 mm, 0.000 deg | 27/27 | 1/1 | 0.64 / 0.98 mm | 0.27 / 0.33 mm |
| 2 | right | 0.44 / 0.44 mm, 0.000 deg | 30/30 | 4/4 | 0.70 / 1.30 mm | 0.28 / 0.44 mm |
| 3 | right | 0.43 / 0.43 mm, 0.000 deg | 39/39 | 1/1 | 0.69 / 1.06 mm | 0.27 / 0.33 mm |
| 4 | right | 0.44 / 0.43 mm, 0.001 deg | 38/38 | 2/2 | 0.72 / 1.22 mm | 0.27 / 0.31 mm |

The bits' tolerances are 2.0 mm (socket) and 1.2 mm (Torx); every one of the 134 seated fasteners was driven and
taken out at the first press. The 23 corroded fasteners stalled at the first limit and broke away at the raised one
(M6 at 16.0 to 18.8 N m, M8 at 38.0 to 40.7 N m). The harness was taken at the first grasp on all three packs that
have one.

**Hidden set** (the frozen program at `82e5f2f`, run once on 2026-11-25 through the native verify stage, staged as
the reference is: `tests/` copied fresh, the six program files in `home/submission`, no token):

| pack | variant, harness | credit | steps | simulated s | frames | program s |
|---|---|---|---|---|---|---|
| 0 | P4, lid | 1.0 | 1139 | 768 | 32 | 5.8 |
| 1 | P4, busbars | 1.0 | 1079 | 712 | 32 | 5.4 |
| 2 | P5, none | 1.0 | 1301 | 847 | 37 | 6.2 |
| 3 | P5, lid | 1.0 | 1326 | 882 | 38 | 6.4 |
| 4 | P6, busbars | 1.0 | 1471 | 961 | 44 | 7.3 |
| 5 | P6, none | 1.0 | 1493 | 954 | 43 | 7.3 |

`mean_pack_credit` 1.00000, reward 1.0 (the reference's value; it took 852 to 1185 steps on these packs, the
baseline 24 to 28 % more); the probe lock 96 of 96 counted pairs; no protective stop, nothing stripped; 38.6 s of
the program's own time for the six packs.

## What it does not do

- It has no code for a stripped head (the extractor bit): nothing it did stripped one, and no field pack arrives
  with one.
- Its retries (another grasp along the harness, a second pick, a second look at the lid's bolts when the lid stays,
  a wrist frame taken again) were never needed on the development packs or the hidden set, so they are untested.
- It leans on exact depth: the head's type by its height and its centre from the depth points of its top face. With
  noisy depth frames those would need another method; none was tried.
- It reads rust from nothing (the stall says it), and logs to stderr only.
