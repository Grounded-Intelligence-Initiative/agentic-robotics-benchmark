# Observability: what a controller can see (maintainer evidence, 2026-11-12)

The task's observations carry no object state. This folder shows, on development kits only, that the camera images are
enough to find everything the job needs, with code that uses only what a controller receives (the observation's RGB
images, `T_world_wrist` and the published camera matrices of the data sheet); the simulator's state is read only to
score the estimates. These kits (`sample_kit` seeds 1000-1029) are maintainer kits: neither graded kits, nor selection
candidates, nor the commissioning service's development kits.

- `perceive.py`: the perception (nut colours with a kit-aware hue segmentation that copes with the lights washing the
  colours out; peg shaft tops; peg bands; the ring centre by a circle fitted to its outer edge in a close-up).
- `check_perception.py`: per kit, the first observation's free images, then one wrist close-up over each nut's rough
  position (plain actions, as a controller would move), every estimate against the simulator.
- `perception_dev_kits.json`: 30 development kits (`sample_kit` seeds 1000-1029, layout seeds +4000), 90 pegs, 90 nuts.

| what | from | result (30 kits) |
|---|---|---|
| peg positions | the wrist camera's view from the start pose | 90 of 90 found; error 0.5 mm median, 1.3 mm max |
| which peg is which (1, 2, 3 bands) | the fixed left and right cameras | 90 of 90 right |
| rough nut positions | the fixed cameras | 90 of 90 found; 20 mm median (the blob's centre lies towards the handle), 24 mm max |
| ring centre and handle direction | one wrist close-up over the rough position | centre 0.15 mm median, 0.3 mm p90, 88 of 90 within 0.6 mm (one at 1.2 mm; one off by 15 mm with its handle misread, which a second look would catch); handle 0.1 degree median |

The hole fits the shaft with 1.7 mm of radial clearance, so these estimates are enough to grip a nut across its ring
and put it on its peg. The groove itself is found by feel or by looking into the hole (instruction.md says both are
possible): the nut's drops into a riser (15, 15 and 9.5 mm) show in the gripper's height, and a blocked turn in the
gripper's yaw. A by-feel controller built on this perception during the review (the ring gripped across its centre, the
groove felt through the gripper's height and yaw) put 21 of the 22 nuts it picked up on 12 development kits onto their
pins and into a riser, told false entrances by their short travel, and assembled one or two single nuts per run of 12
kits, but completed no kit within the review's effort (its turns stall in the narrow mouth of the next riser); it is not
shipped.

## A controller on the observations alone, within the takt (2026-11-16)

`observation_only_controller_dev_kits.json`: the submission of the first frontier measurement (Claude Opus 5.5,
2026-11-12) acts on the observations alone: it reads the layout from the first images and finds each groove by touch,
with no access to any state when it runs. It was made for the earlier version, whose episodes lasted 2400 steps, and
completed 9 of that version's 10 graded kits in 1047 to 1834 steps. Run by the grader on the commissioning service's
16 development kits under today's 1200-step takt (a one-episode plan per kit through `tests/review/regrade.sh`), it
completes 3 of 16 kits (1090, 1152 and 1175 steps) and assembles 30 of 48 nuts; on the other 13 kits the takt runs out
with one or two nuts assembled. The reference, which is given the grooves, needs 598 to 838 steps on the same kits.
So the observations are enough to complete kits, also within the takt for a controller that was not written for it;
what the takt asks for is a groove search about as direct as the reference's path.

The perception check above was repeated on the amended image the same day: every kit's numbers are identical.

Run (inside the task image; `tests/` mounted read-only):

    docker run --rm --network none -v <task>/tests:/stage:ro --entrypoint /opt/venv/bin/python3 <task image> \
        -I -B /stage/review/observability/check_perception.py --first 1000 --n 15
