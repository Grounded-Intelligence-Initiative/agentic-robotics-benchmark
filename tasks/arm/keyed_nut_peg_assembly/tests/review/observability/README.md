# Observability: what a controller can see (maintainer evidence, 2026-11-12; round 3 on 2026-11-17)

The task's observations carry no object state. This folder shows, on development kits only, that the camera images are
enough to find everything the job needs, with code that uses only what a controller receives (the observation's RGB
images, `T_world_wrist` and the published camera calibration of the data sheet); the simulator's state is read only to
score the estimates. These kits (`sample_kit` seeds 1000-1029, layout seeds + 4000) are maintainer kits: neither graded
kits, nor selection candidates, nor the commissioning service's development kits.

- `perceive.py`: the perception (nut colours with a kit-aware hue segmentation that copes with the lights washing the
  colours out; peg shaft tops; peg bands; the ring centre by a circle fitted to its outer edge in a close-up; the
  shaft top in a close-up, by its shape).
- `check_perception.py`: per kit, the first observation's free images, then wrist close-ups over each peg and each
  nut's rough position (plain actions, as a controller would move), every estimate against the simulator.
- `perception_dev_kits.json`: 30 kits, 90 pegs, 90 nuts, on the cell with tolerances (amendment 2).

Since round 3 every episode is a real kit on the cell of the day: the real cameras sit within their tolerances of
the published calibration (data sheet, section 11), so an image position is off by millimetres. What the published
calibration alone gives, and what looking twice gives:

| what | from | result (30 kits, with tolerances) |
|---|---|---|
| peg positions | the wrist camera's view from the start pose | 90 of 90 found; error 3.6 mm median, 5.8 mm p90, 14 mm max |
| which peg is which (1, 2, 3 bands) | the fixed left and right cameras | 90 of 90 right |
| a peg's shaft top | one wrist close-up from above | 1.6 mm median, 3.2 mm p90, 5.0 mm max |
| the same | two close-ups, the hand turned half a turn between them (about 60 steps) | 0.8 mm median, 1.5 mm p90, 2.2 mm max: the wrist camera's error is fixed in the hand's frame, so it cancels |
| the same | one close-up, corrected by the offset measured on the kit's first peg | 1.4 mm median, 2.4 mm p90, 3.1 mm max |
| rough nut positions | the fixed cameras | 90 of 90 found; 18 mm median (the blob's centre lies towards the handle), 28 mm max |
| ring centre and handle direction | one wrist close-up over the rough position | centre 1.8 mm median, 3.4 mm p90 (17 of 90 over 3 mm, three of them with the handle read the wrong way round, which a second look would catch); handle 0.3 degrees median |

Before the tolerances (the drawing's cell, 2026-11-12 and 2026-11-16) the same code gave: peg positions 0.5 mm median
(1.3 mm max), ring centres 0.15 mm median. The cameras' calibration error is what the difference is.

The groove itself is found by feel or by looking into the hole (instruction.md says both are possible): the nut's
drops into a riser (15, 15 and 9.5 mm) show in the gripper's height, and a blocked turn in the gripper's yaw and in
the wrist's torque.

## A controller on the observations alone (2026-11-17, amendment 2)

`observation_only_controller_dev_kits.json`: the submission of the second frontier measurement (Claude Opus 5.5,
2026-11-16) acts on the observations alone: it reads the layout from the first images and wrist close-ups and finds
each groove by touch, with no access to any state when it runs. It was made for the drawing's cell (no tolerances)
and a 1200-step takt, where it completed 7 of 10 graded kits. Run by the grader on the commissioning service's 16
development kits under amendment 2's rules (a one-episode plan per kit through `tests/review/regrade.sh`), it
completes **9 of 16 kits** (664 to 992 steps) and assembles 32 of 48 nuts. The reference, which is given the real
grooves, needs 636 to 804 steps on the same kits. Its failures are insertions that miss the shaft (the calibration
error is larger than its search); with the cameras' error switched off it completes 12 of 16.

So the observations are enough to complete kits within the takt, also for a controller that was not written for these
rules, and tolerances with a 100 s takt are not a hard enough version (`../commissioning.md`).

Earlier (2026-11-16, round 2's rules: no tolerances, 1200 steps): the first measurement's submission, made for 2400
steps, completed 3 of round 2's 16 development kits and assembled 30 of 48 nuts.

Run (inside the task image; `tests/` mounted read-only):

    docker run --rm --network none -v <task>/tests:/stage:ro --entrypoint /opt/venv/bin/python3 <task image> \
        -I -B /stage/review/observability/check_perception.py --first 1000 --n 10
