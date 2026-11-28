# Write the MuJoCo description and the calibration of the hand in the photographs

You are the engineer of a lab that built one four-digit robot hand from its designers' public files: three fingers and
a thumb on a palm, 16 hinges, 13 of them driven by a servo actuator each (the last hinge of each finger has no servo and
follows the hinge before it through a crossed four-bar), a tactile module at each fingertip. The lab has shipped this unit to a
partner, who will take it out of its box, switch it on, mount it on an arm and pinch small parts between the thumb and
the index finger, commanding it only through the hand's host software with the calibration file the lab delivers. Before the
hand left, a colleague photographed it: five postures, two views each, with one bus read-out of the 13 servos taken
with each photograph, and one last photograph of the hand switched off and packed in its box. The output horns were
fitted without the assembly guide's 180-degree step, and the bench PC that held the hand's calibration was reinstalled
since: there is no calibration file. The partner does not home the hand and does not accept homing as a calibration.
Nobody can touch the hand again.

Write the MuJoCo model of THIS unit and its calibration file in the host software's own format, so that the partner's
commands put the real hand's pads where the partner means them to be. Your files are graded against the unit itself,
which only the grader has.

Everything you need is in your home directory, `/home/user/`. There is no network and no GPU. The sandbox has
8 CPUs and 16 GB of memory. Use the interpreter `/opt/venv/bin/python3` (call it by its full
path): MuJoCo 3.14 (offscreen rendering through OSMesa, `MUJOCO_GL=osmesa`), numpy, scipy, trimesh,
manifold3d, CoACD, Pinocchio, OpenCV (`cv2`, headless), scikit-image, Pillow, matplotlib, CadQuery, jsonschema and
pyyaml are installed there. How you get the numbers out of the photographs and documents is up to you; the grader
scores the files you leave behind.

## Given files (in `/home/user/`)

| file | what it is |
|---|---|
| `README.md` | what is where |
| `photos/photo_01.jpg` ... `photo_11.jpg`, `photos/photos.json` | the photographs (one phone and its lens, not calibrated; the focal length is not the same in every photograph; camera poses not recorded) and per photograph its posture, its view and its read-out |
| `bus/readouts.csv`, `bus/readouts_header.txt` | the bus read-outs of the photo session, one per posed photograph: per servo id its Present Position(132) count, temperature and input voltage |
| `bus/control_tables/servo_00.txt` ... `servo_12.txt` | the control-table export of each servo, read off the bus |
| `parts/meshes/*.stl`, `parts/assembly.json`, `parts/bom.csv`, `parts/print_settings.csv` | the part files: every made part (tags P01 ... P52), every given part type and every fastener type as a mesh in its own frame (metres); every instance of the hand with its placement in the palm frame at q = 0; the bill of materials; the print settings of the made parts |
| `docs/servo_VKR-4LBA/manual.md`, `control_table.md`, `performance_curve.png` | the servo actuator's manual pages |
| `docs/design_table.md`, `docs/assembly_guide.md`, `docs/host_software.md` | excerpts of the designers' documents: servo ids, axes and travel; the assembly steps; the host software's formula and calibration file |
| `docs/bench_record.md`, `docs/handover_note.md`, `docs/weigh_slip.md` | the lab's own records: how the hand was built and photographed, what the partner does with it, what was weighed |
| `contract/contract.md`, `contract/zero_pose.png`, `contract/pad_definition.png`, `contract/collision_pairs.json` | the contract of the files you deliver and of what the grader does with them |
| `check_submission.py`, `checker/` | the grader's own gate (it says whether a folder is a valid description, not how accurate it is) |

## Deliverable: `/home/user/submission/`

```
/home/user/submission/
├── robot.xml     the MJCF of the hand (contract: names, frames, coupling, pads, actuators, inertials, MJCF rules)
├── assets/       every mesh robot.xml references
└── config.yaml   the host software's calibration file of this hand: motor_ids, home_offsets (rad), joint_signs
```

```bash
/opt/venv/bin/python3 ~/check_submission.py ~/submission     # the contract gate (no knowledge of the unit)
```

## How the submission is graded

The grader runs after you finish and executes none of your code; it never steps your model. The gate `model_loads`
(every rule of the contract's "Gate", 1 to 11) multiplies everything:

score = model_loads x (0.55 pinch + 0.15 single + 0.10 pose + 0.10 travrej + 0.10 force_load)

Band: L(e; full, zero) = 1 for e <= full, 0 for e >= zero, and ln(zero / e) / ln(zero / full) between (every halving
of an error earns about the same increment).

How the hand is used in grading (full text: `contract/contract.md`): the partner takes the hand out of its box and
switches it on; from then on it is commanded only through the host software with your `config.yaml`. For a request
the grader solves the targets on YOUR model with its own IK, converts the solution to goal counts through YOUR
`config.yaml` and sends them to the servos of the digits of the request only; every other servo stays where it was at
switch-on. The real hand moves (within its own ends of travel) and the grader measures its pads. For a reading it takes
the 13 counts of the real hand at a pose, turns them into angles of YOUR model through YOUR `config.yaml` and compares
your model's pads with the real hand's. The requests and readings are fixed (the same at every grading run) and
hidden; they lie inside the real hand's travel, where no two parts of different digits (or of a digit and the palm)
touch.

| item | weight | what is measured |
|---|---|---|
| `pinch` | 0.55 | 36 pinch requests between the thumb pad and the index pad (each: a midpoint m, a unit axis u from the index pad to the thumb pad and a gap g of a few millimetres between pads that face each other): the thumb and the index finger are commanded as above; z x L(e_close; 0.75 mm, 3 mm) x L(e_mid; 0.75 mm, 5 mm) with e_close = the length of (p_thumb - p_index) - g u and e_mid = the distance of (p_thumb + p_index) / 2 from m, the real pads at the reached pose; mean over the requests |
| `single` | 0.15 | 32 single-pad requests (8 per digit: a target point for one digit's pad): z x L(e_pad; 0.75 mm, 5 mm), e_pad = the real pad's distance from the target; mean |
| `pose` | 0.10 | 64 readings x 4 digits: o x (0.7 L(e_pos; 0.75 mm, 5 mm) + 0.3 L(e_nrm; 0.5 deg, 10 deg)), e_pos and e_nrm = the errors of your pad point and pad normal against the real hand's at that reading; mean |
| `travrej` | 0.10 | 0.6 x travel + 0.4 x rejection. Travel: the 32 ends of travel (26 of the servo-driven hinges, converted to the real hand's angles through your calibration, and 6 of the fingers' last hinges), each L(e_end; 2 deg, 15 deg); mean. Rejection: 16 single-pad targets that the real hand cannot reach must be rejected, i.e. the grader's IK on your model must fail on them; rejection = (rejected / 16) x (accepted feasible single-pad requests / 32), so that a model which rejects everything earns nothing here |
| `force_load` | 0.10 | 0.5 x pad force + 0.5 x load. Pad force: at 48 readings, per digit, the largest force the pad holds along its normal, f = min over the digit's servo-driven hinges i with abs(g_i) > 1e-9 m of F_i / abs(g_i), g_i = (dp / dq_i) . n (the pad point's derivative with respect to hinge i with the coupling included, in m / rad; n the pad normal; F_i the force range of `act_i`; no gravity, no friction), against the real hand's: o x L(abs(f_sub - f_real) / f_real; 3 %, 30 %), mean over the readings, times the digit's factor C; mean over the 4 digits. Load: at 6 postures, for each digit (the subtree of `thumb_roll` or `f_abad` with its loop bodies) and for the whole hand: o x sqrt(L(relative mass error; 3 %, 30 %) x L(centre-of-mass error in the palm frame; 1 mm, 20 mm)), mean over the postures, times the group's factor C; mean over the 5 groups |

- z = 0 (the request scores 0) when the grader's IK on your model does not reach every target pad within
  1 mm, when your chain does not close at a configuration the IK needs, when a commanded servo would be
  driven more than 2 deg beyond the real hand's end of travel, or when two parts of the real hand are
  pressed more than 1 mm into each other at the reached pose (pressing deeper would exceed the partner's
  force cap); else z = 1. A servo commanded at most 2 deg beyond an end stops at the end. z = 0 also
  for every request of a digit one of whose servo-driven hinges is locked out by the partner's range check: before the
  hand is used, the partner's software drives each servo-driven hinge, alone, from the zero posture (every other
  servo-driven hinge at zero), to both ends of the range YOUR model states for it (through YOUR `config.yaml`); a
  hinge that stops more than 2 deg short of a stated end is locked out, together with its digit,
  until the description is corrected. A stated range that ends inside the real hand's travel locks nothing out; your
  model then cannot reach the requests beyond it.
- o = 0 (the reading scores 0 for a digit) when your calibration maps a servo of that digit more than
  2 deg outside your model's range of its hinge, or when that digit's chain does not close at the
  reading (for the whole-hand load group: any servo, any chain); else o = 1.
- The factor C of a digit is its share of the requests it took part in: C_thumb = the mean of the 36 pinch
  scores and the 8 thumb single-pad scores; C_index = the mean of the same
  36 pinch scores and its 8 single-pad scores; C of each other finger = the mean of its
  8 single-pad scores; C of the whole hand = the mean of the four.
- The grader's IK: damped least squares on the driven hinges of the request's digits, clamped to your ranges at every
  iteration, 24 starts (the middle of the ranges, then fixed Halton points over the ranges shrunk by
  2 deg), at most 150 iterations each, a start done at 0.1 mm; a pinch also weighs the
  pads' facing with 0.002. Loops are closed by Gauss-Newton from q = 0 in steps of at most
  2 deg to 1e-9 m.
- The 1 mm rule is measured with the convex hull of each part listed in `contract/collision_pairs.json` (each made
  part, each tactile module, and each servo as its solid is given (the case with its output horn and rear bearing: one
  part file)); pairs of different digits and of a digit and the palm count, except
  parts on parent and child bodies and the listed exclusions.
- Gate caps (contract, "Gate"): 200 MB and 20 000 files and folders in the submission, 5 000 000
  mesh triangles, `config.yaml` 64 KB, compile 120 s, YAML nesting 4 and indent
  16 spaces, home offsets within +-1000 rad, the grading after the gate 300 s of CPU time.
- The description of the unit itself scores 1.0.

## Practical notes

- The bus read-outs are the servos' own encoders (exact counts); the cameras are not known. The photographs are meant
  for programmatic processing; viewing all of them costs a large share of your context. `photos/photos.json` says which
  photograph shows what.
- MuJoCo renders offscreen through OSMesa here (on the CPU).
- The grader reads its own copies of the given files; your home's copies are yours to read.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session with whatever is in `/home/user/submission/` at that moment. Write
  large files in several parts (one part or one function group per write) and keep every single response well below
  that limit.
- Session time: this session is stopped after 6 hours of wall-clock time from its start. A session
  stopped by that limit is not graded at all, so a late finish scores nothing. Note the start time with `date` when you
  begin, keep a complete submission in place from early on, and end the session yourself (stop working and say that you
  are done) at least 15 minutes before the limit.
