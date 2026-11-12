# Observability: what a controller can see (maintainer evidence, 2026-11-12)

The task's observations carry no object state. This folder shows, on development kits only, that the camera images are
enough to find everything the job needs, with code that uses only what a controller receives (the observation's RGB
images, `T_world_wrist` and the public camera matrices); the dev sim's state is read only to score the estimates.

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
gripper's yaw. A by-feel controller built on this perception during the review reached the pin with every nut it
carried, told false entrances by their short travel and assembled single nuts on development kits, but did not complete
a kit within the review's effort (the second turn's drop jams); it is not shipped.

Run (inside the task image; `tests/` mounted read-only):

    docker run --rm --network none -v <task>/tests:/stage:ro --entrypoint /opt/venv/bin/python3 <task image> \
        -I -B /stage/review/observability/check_perception.py --first 1000 --n 15
