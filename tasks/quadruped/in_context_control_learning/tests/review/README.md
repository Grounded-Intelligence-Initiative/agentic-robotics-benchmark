# Review records of in_context_control_learning (round 2: onboard camera, turning gait)

Everything here was measured on development seeds (3001-3100) or on the pre-registered candidates; no frame of a
hidden scene was rendered for a person, and no model was run for these numbers.

| file | what |
|---|---|
| `hidden_set/RULE.md`, `candidates.json`, `selection.jsonl` | the hidden set's pre-registered rule, its 40 candidates, the run and the result |
| `select_hidden_set.py`, `run_episode.sh` | the selection driver and the one-episode harness it uses (setup, a policy through the socket as the agent account, the grader on the ledger) |
| `check_task.py` | self-checks, run in the task image (copies, determinism, stated ranges, camera model, budgets, oracle channel, audit) |
| `reference_dev.jsonl` | the image-only reference on 100 development seeds, through the socket and the grader |
| `baselines.py`, `baselines.jsonl` | random buttons and a camera-less dead-reckoner on 100 development seeds |
| `margins.py`, `margins.jsonl` | the reference under other light radii, budgets and a coarser camera |

## The image-only reference (`solution/reference_policy.py`)

No token, no ground truth: the replies and the frames an agent gets, and the instruction. On development seeds
3001-3100, run with `select_hidden_set.py --reference` (one container per seed, the reference as the agent account
through the robot socket, the grader replaying its ledger):

- six of six delivery points on 99 of 100 seeds, delivered fraction 0.998;
- 244 to 486 presses of the 500 (median 333, 90th percentile 413);
- 62 to 118 frames of the 120 (median 91, 90th percentile 109);
- the one seed short of six (3064) spent 34 frames on finding out which button walks forward and ran out of frames
  on the last leg.

So the observations suffice, and the budgets leave the reference little to spare: it uses two thirds of the presses
and three quarters of the frames on a median seed, and nearly all of both on the worst ones.

## Baselines (`baselines.py`, no rendering)

| policy | delivered fraction | seeds with no point |
|---|---|---|
| random buttons | 0.003 | 98 of 100 |
| camera-less dead reckoning with a perfect calibration (told the buttons and the exact steps; compass, start pose, coordinates; never sees a wall) | 0.035 | 83 of 100 |

There is no luck floor, and the coordinates and the compass alone do not get the robot through the partitions.

## How the stated settings were chosen

The raise replaced the ceiling camera by the robot's own forward camera and the sideways steps by turns in place, and
gave the delivery points as coordinates. Measured with the reference at each step, on development seeds:

1. First version of the raise, as built during development (it is not in this commit): a lamp on the active pad
   that shows at any distance, a floodlight of 1.4 m, a frame with every reply, 600 presses. The reference, looking
   after every step and not using the lamp, reached six of six on 48 of 48 seeds with 232 to 437 presses (median
   303): more than a quarter of the presses unused on the worst seed, and no limit on looking. That was easy in the
   reference's margins, so the fallbacks the raise allowed for were taken, one at a time.
2. No lamp. The reference does not need it (same 48 seeds, six of six on all, 233 to 432 presses), and with it a
   driver that has lost its position can still walk towards the light. The pad now shows only under the floodlight,
   so reaching it takes dead reckoning good to about a metre.
3. Frames on request and a frame budget. `robot_act` returns the status only; `robot_observe` returns the frame,
   120 of them per episode. Between frames the robot has to be driven on the compass and the learned step. (The
   harness we measure models with keeps at most 100 images in a request, so many more frames would not be more
   memory for the model.)
4. A floodlight of 1.0 m and 500 presses. Rows of `margins.jsonl`, 100 seeds each, budgets as stated unless said:

| setting | six of six | delivered fraction | presses on full runs (median, max) | frames on full runs (median, max) |
|---|---|---|---|---|
| as stated: light 1.0 m, 500 presses, 120 frames | 99 | 0.998 | 333, 486 | 91, 118 |
| light 1.4 m | 100 | 1.000 | 330, 465 | 87, 113 |
| light 0.8 m | 95 | 0.987 | 340, 494 | 99, 120 |
| 90 frames | 94 | 0.987 | 329, 461 | 90, 90 |
| 60 frames | 56 | 0.873 | 319, 456 | 60, 60 |
| 400 presses | 87 | 0.977 | 325, 396 | 89, 113 |
| the reference sees each frame in 2 × 2 pixel blocks (128 × 128) | 99 | 0.998 | 339, 455 | 91, 120 |
| in 4 × 4 pixel blocks (64 × 64) | 51 | 0.832 | 351, 495 | 116, 120 |

With the stated settings the reference is close to both budgets on its worst seeds and loses deliveries as soon as
either is cut or its view is four times coarser; a harder layout (more walls, dead ends), the remaining fallback,
was therefore not added. The light radius matters little to the reference between 0.8 and 1.4 m, because it places
itself by the compass and its step and looks only where a wall can be; 1.0 m is kept as the smaller of the two
radii at which it still reaches six of six on (nearly) every seed.

## The oracle (`solution/oracle_policy.py`)

Privileged (ground truth through the oracle token); it takes no frame. On development seeds 3001-3100 it reaches
six of six on all 100, with 173 to 337 presses (median 241) and no wall contact. On the hidden set: see
`hidden_set/RULE.md`.

## Reproducing

In the task image, from the task folder (`/task`, read-only is enough):

```
/opt/venv/bin/python3 tests/review/check_task.py
/opt/venv/bin/python3 tests/review/baselines.py blind /tmp/b.jsonl 3001 3002
/opt/venv/bin/python3 tests/review/margins.py /tmp/m.jsonl --seeds 3001-3005 --light 1.4
```

On the host, with Docker: `python3 tests/review/select_hidden_set.py --image <task image> --reference --seeds
3001-3100 --out <dir> --rows <rows.jsonl>` for the reference, and without `--reference` for the selection itself.
