# Reviewer Guide

This is a local feasibility prototype submitted for review, **not yet qualified
for frontier-model difficulty**. It asks an agent to implement an image controller,
not to operate the drone solely through an in-context conversation.

## Input, Output and Success

The agent sees two calibrated RGB cameras and remaining budget, explores using
collective thrust and attitude commands, and submits an executable `run.sh`.
The verifier runs it on four fresh private scenes. Success requires ordered
yellow/green/magenta gate crossings, no collisions and two seconds of stable
hover above the goal. The score is normalized completion rate; speed is simulated
time, never host execution speed.

`oracle/run.sh` constructs an actual successful submission and records its live
interaction. `oracle/policy.py` is an image-only scripted solution: color tracking,
pixel calibration and feedback control. It never imports the private world or
reads scene parameters. The known-camera/color setting is deliberate for a first
reviewable task. It may be too easy for frontier agents; measure that before merge.

## Reproduce

Use the ALE revision pinned by the repository's `vendor/ale` submodule. From the
repository root, with that engine installed:

```bash
TASK=tasks/uav/visual_low_level_flight_control
ale lint "$TASK"
python scripts/lint_domain.py "$TASK"
ale validate "$TASK" --runs-dir runs/crazyflie-validate
```

The image build fetches checksum-locked robot meshes. Execution is offline and
uses two CPUs, 4 GiB memory and no GPU. No model API is called by `ale validate`.
The successful submitted files, camera captures and interaction JSON are in the
oracle episode's collected artifacts. The trusted evaluation episode outcomes
and post-run replay seeds are in `trace.execution.jsonl` and `verification.json`.

For a host-side visual demo, install `image/requirements.txt`, `libosmesa6`, and
optionally `imageio==2.37.0 imageio-ffmpeg==0.6.0` for video:

```bash
python "$TASK/image/fetch_visual_assets.py" "$TASK/image/visual_assets.json" runs/crazyflie-assets
export MUJOCO_GL=osmesa LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export VISUAL_ROBOTICS_ASSETS="$PWD/runs/crazyflie-assets"
python -m unittest discover -s "$TASK/tools/tests" -v
python "$TASK/tools/review.py" --out runs/crazyflie-demo --seeds 101 102 103 --video
python "$TASK/tools/review.py" --out runs/crazyflie-random --seeds 101 102 103 --mode random
python "$TASK/tools/check_security.py" --ale "$(command -v ale)" --out runs/crazyflie-security
```

The preview/poster is a real simulator render. Its third-person reviewer camera
is not an additional agent observation. The video runs at five frames per second,
one frame per 0.2-second action, except a terminal action may finish early.
Development seeds above are not sealed benchmark test seeds. Keep run products
outside the task; do not commit videos, caches, fetched meshes or experiment logs.

## Model and Licence

The unscaled Crazyflie 2 model comes from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/f3b6013629c7b94f8ee93fd977b161fa8350f9bc/bitcraze_crazyflie_2),
under its MIT licence. The pinned download includes that licence, attribution,
source MJCF and visual/collision meshes. SHA-256 checks are mandatory.
The model is derived upstream from `whoenig/crazyflie_ros`.

We retain the nominal 0.027 kg mass, inertias and collision meshes. We add an
original three-gate course, cameras, a small tracking sticker and a bounded
attitude servo. Thrust/torque limits are benchmark abstractions, not measured
hardware limits. There is no firmware, rotor-speed, battery or real-world
validation claim. No geometry, textures or trajectories are copied from a
third-party drone-racing or rescue demonstration.

## Integrity and Remaining Admission Checks

The root-owned service owns simulation state and budgets; clients only receive
images. Reconnection and unsupported reset/report commands cannot restore budget
or claim success. The verifier stops the exploration service, freezes all agent
files read-only, then runs the submission as the unprivileged account. The
canonical `robotics_grader` and its observation-dependence check are unmodified.

Before admission: register/confirm the task in the coordination sheet, agree on
the exact frontier model and harness, measure its scores with adequate reasoning
budget, and tune genuine task challenges if it proves too easy. Do not interpret
environment/protocol failures as model inability.
