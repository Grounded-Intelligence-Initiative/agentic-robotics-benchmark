# Interaction type and peak intensity of driving interactions from bird's-eye-view video

A fleet-safety team triages vehicle interactions for the scenario-based testing of an automated-driving stack. Each
interaction event is a bird's-eye-view (BEV) video rendered from the tracked trajectories of the vehicles in the scene,
the rendering of the Interactive Enhanced Driving Dataset (IEDD). For 24 test events the trajectories and most of the
annotations were lost: only the videos, the ids of each event's two vehicles and its time interval remain. For every
test event, recover the type of the interaction and its peak intensity.

You have 4 hours, 4 CPU cores, 16 GB of memory (processes that exceed it are killed), no GPU and no network.
Python 3.12 is `python3` (NumPy, SciPy, OpenCV (headless, with FFmpeg), Pillow, scikit-image, scikit-learn,
Matplotlib). There is no display.

## What you get (`/home/user`)

```
test/T01.mp4 ... test/T24.mp4    the 24 test events
labelled/<id>/video.mp4          events that still have their answers, for calibration
labelled/answers.csv             the answers of the labelled events, in the deliverable's format
clips.json                       every event: its number of frames, the start and end of its interval (s) and the ids
                                 of its two vehicles
iedd_reference/                  IEDD's own code (Apache-2.0) that computed every answer from trajectories
tools/iedd_score.py              the scoring module (the grader uses the same module)
tools/score.py                   the scoring rule on the labelled events, and a format check of your file
submission/                      write answers.csv here
```

- **Videos.** H.264, 10 frames per second, one frame per trajectory sample (0.1 s) of the event's interval. `clips.json`
  gives the interval's ends rounded to 0.1 s, so a clip can hold one frame fewer than they suggest; the text box of
  every frame gives its time. Each frame is a 2000 x 2000 pixel Matplotlib figure with equal-aspect axes labelled `X (m)` and
  `Y (m)` and tick labels in metres (world coordinates). The axes span 160 m by 160 m centred on one vehicle of the
  event, and only vehicles within 80 m of it are drawn; in a frame where that vehicle has no valid state, the axes
  show the whole scene instead. Every vehicle is an outlined rectangle around its centre and heading, drawn at 1.8
  times its length and width, with its id in bold just above it; the vehicle the axes follow is drawn in red
  (outline and id), every other vehicle in black. A text box gives the time.
- **Vehicles.** `clips.json` gives the two vehicle ids of each event as they appear in the video (an id can be
  `ego`). One of them is the red vehicle; the other is drawn in black like the rest of the traffic. Either can
  interact with other vehicles over the same time.
- **Interaction types.** IEDD's four pairwise labels, `Car-follow`, `Merging`, `Crossing` and `Head-on`, as
  `detect_and_classify_interactions` in `iedd_reference/utils/IEDD_compute.py` assigns them to the event's two
  vehicles.
- **Peak intensity.** IEDD computes an intensity Q_i(t) in [0, 1] for each vehicle every 0.1 s
  (`calculate_all_metrics`: pose adjustment, risk variation from time to collision and post-encroachment time, and an
  interactive potential field, combined by `calculate_q_i` with weights that depend on the interaction type). A
  vehicle's Q_i(t) is the largest over all of its interactions at time t, not only the one with the other vehicle of
  the event. The answer is the largest Q_i(t) of either vehicle of the event over the event's interval.
- **Labelled events.** They come from the same IEDD source datasets as the test events, but from other scenes, and
  cover the four types.

## Deliverable

One file, `/home/user/submission/answers.csv`: the header line `clip,interaction_type,peak_intensity`, then one line
per test event (`T01` ... `T24`, in any order). `interaction_type` is one of the four labels above and
`peak_intensity` is a number. `python3 tools/score.py --check` checks the file's format.

## Scoring

The grader holds the lost annotations. For each test event it checks two of your answers:

1. the interaction type is the same;
2. the peak intensity is within 0.05 of the true value.

An event is correct when both checks pass; an event missing from your file, or a number that is not finite, is
wrong. The metric, `interaction_accuracy`, is the fraction of the 24 test events that are correct (higher is better;
1.0 means every answer of every event is correct). A missing or malformed file scores 0.
`python3 tools/score.py <your csv>` computes the same score for estimates of the labelled events. The grader reads only
`answers.csv`; nothing you write is executed.
