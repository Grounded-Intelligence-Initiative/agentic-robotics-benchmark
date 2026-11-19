# V7 author controls: two camera frames, one deployment

V6 gave the target its UAV-frame position and compass heading. V7 gives it
only launch-frame planar odometry, gravity, gyro, joint/foot sensing, and
on-demand RGB from a trunk-mounted inspection camera. There is no launch
mark. Five identical safety caps are visible above and from the mast; their
metric arrangement varies independently of launch and route. A frozen
program must connect the target to the surveyed route. [Suite design](selection.md)
describes the eight balanced cases and two additional counterfactuals.

## Measurement

The reference projects the green spherical silhouettes from actual nadir
images using the calibrated cap height. In eight body-relative ground
views, silhouette tangent angles and the known radius give approximate
range and bearing; gravity levels the camera rays. The estimator compares
unlabeled point pairs, assigns compatible caps, and refines the planar
rigid transform with greater weight on bearing than range. It receives
neither a seed nor a catalog of possible poses. A single estimated transform
maps subsequent odometry to the original route; no second localization,
target practice, or gait change. Only after the policy returns does the
author harness compare its estimate to private settled-pose truth.

| Cases / arm | First wide choice | First fully crossed | Middle crossed | Last fully crossed | Safe dock |
|---|---:|---:|---:|---:|---:|
| 8 main, registered | 8 | 8 | 8 | 8 | 8 |
| 8 main, identity transform | 4 | 1 | 0 | 0 | 0 |
| 2 mirrored, registered | 2 | 2 | 2 | 2 | 2 |
| 2 mirrored, identity transform | 2 | 0 | 0 | 0 | 0 |

Both arms saw identical RGB maps and exposures and used identical route
plans, odometry, joint controller, action limits and scoring. The identity
arm treats local coordinates as chart coordinates. Every failure was actual
MuJoCo contact, not a localization threshold, fall or timeout.

A stronger no-registration control reuses the *measured* transform from a
successful original deployment on sites 0 and 3. With this frozen calibration
it docked on both original starts. On their otherwise identical reflected
starts the same calibrations were wrong by 1.38 m / 119.43 degrees and
1.13 m / 91.82 degrees respectively; both runs hit the first rack. This
author-only test favors the ablation by giving it a valid calibration for
the exact scene, but no observation-dependent update on the current launch.

| Site | Registered translation error (mm) | Yaw error (deg) | Registered outcome | Identity contact |
|---|---:|---:|---|---|
| 0 | 3.46 | 0.043 | Dock | West wall |
| 1 | 3.54 | 0.023 | Dock | First rack |
| 2 | 5.48 | 0.083 | Dock | First rack |
| 3 | 4.69 | 0.063 | Dock | West wall |
| 4 | 7.20 | 0.127 | Dock | West wall |
| 5 | 6.22 | 0.016 | Dock | West wall |
| 6 | 3.22 | 0.039 | Dock | First rack (after initial crossing) |
| 7 | 10.44 | 0.069 | Dock | First rack |
| 0 mirrored | 4.40 | 0.046 | Dock | First rack |
| 3 mirrored | 6.49 | 0.115 | Dock | First rack |

All ten registered deployments matched five caps before the first joint
command. Cross-ray RMS was 0.25-2.92 mm. Each took 49 scout moves and 8
of 16 mast exposures. Docking used 15,702-19,381 of 36,000 ticks and
1,351-1,658 of 4,000 requests; final dock errors were 0.079-0.094 m
(limit 0.17 m). No tuning or retries of a failed registered site.

## Identifiability

The mast is aimed at the elevated fixtures, not at the floor. With a level
trunk its lowest center-column ray is one degree above horizontal. All
azimuths can be inspected without disturbing the launch pose. A rendered
counterfactual test changes only the last route bit: the aerial images
change, but all eight parked mast PNGs and the initial proprioceptive
observation are identical. Reflecting only the launch reverses which image
stream changes. All eight main sites also produced identical initial public
non-image observations on their original and reflected starts.

This is an idealized indoor registration task. Caps share known size and
height; cameras and drift-free local odometry are calibrated; no RGB noise
or lighting distribution shift is claimed. The ground views are available
during motion too. The controls test the shared-frame shortcut, not an
impossibility theorem for every alternative visual navigation strategy. No
pose estimate or threshold is submitted to the verifier for credit.

## Reproduction

Use the task's pinned packages and assets with OSMesa software rendering.
From the task root, with a fresh writable output directory for each command:

```bash
python -B tests/review/test_v7_contract.py
python -B tests/review/v7_controls.py pilot --pilot
python -B tests/review/v7_controls.py final
python -B tests/review/v7_controls.py replay --replay-from final/results.jsonl --sites 0 3 --mirrors 0 3
```

Author work: `koa04:/home/wang100/projects/cross-registration-author-20261119`.
`final/results.jsonl` holds 20 per-site/arm rows (SHA-256
`ec65d8a40274d43dc93d8632e77081ea4b54be3950d5c2f2d22a39b9588565ff`).
The same directory contains each service ledger and representative native
UAV and mast PNGs. `pilot-final/` is the stationary development check on
the final optics. `replay/results.jsonl` holds the four frozen-calibration
controls (SHA-256 `6443a69f2844b80b43a9003df4b9d472d6a8be274b5a14a4ff6dad1e1401f59a`).
Earlier horizontal-camera data are not candidate results.
These are CPU-only author socket controls on MuJoCo 3.16.0, not formal ALE
validation or blinded model evidence.

## Verifier delta

The service changes the target observation frame and meters 16 optional
ground exposures. Existing passage, docking, budgets, time limits, contact
termination and scoring rules remain. The verifier adds per-site/mean
`ground_looks` and a separate `contact_bollard` cause; it does not publish
the launch transform. Grading is still safe docks divided by eight,
anchored at 1.0. Commissioning cannot deploy or use the ground camera.
