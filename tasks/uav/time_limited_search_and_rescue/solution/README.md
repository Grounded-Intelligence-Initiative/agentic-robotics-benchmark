# Visual Odometry Diagnostic Author Notes

This diagnostic branches from `d2ef66f`, the final candidate of the
prior local-discovery pilot. It does not replace that pilot or its
reported Sol successes. The physical mezzanine, local doorway sensing,
pickup, shutter change and loaded cargo egress are retained. What
changes is the reliability of integrating commanded horizontal motion.

## Perturbation and recovery

Neighboring station centers are spaced 1.58–1.90 m apart. Door frames,
walls and the mezzanine floor use those continuous positions and the
launch hover starts up to 0.13 m off center on each horizontal axis.
The pad position varies independently and must be located from RGB for
the reference set-down. Ordinary clear openings
are 1.18 m wide; the low inspection slot retains its smaller opening.
The XY servo target has
fixed per-axis gains of 0.96–1.045 and cross-axis movement of 0.032 m
per metre of forward command empty or 0.125 m per metre loaded.
Matched seeds `2k` and `2k+1` share all site geometry, launch positions,
gains and object placement; only the cross-axis direction changes.

This is a small, deterministic tracking-error model, not wind, random
noise or an aerodynamics claim. It uses stabilized position and yaw
control (and small ordinary servo lag); no horizontal command means
no change to the target during a hold. Height and yaw are not given
new biases. Four same-looking maintenance dots form a 0.58 m square
at each station. They reveal local scale and centering, not place IDs,
the next doorway, or the sled. The controller can hold, turn to obtain
an unobstructed image, and issue body-relative corrections.

## Reference and controls

The reference imports no simulator or seed. It discovers the wing
from local RGB as before. It uses nominal distances only to get near
the next station. A fitted square at the calibrated hover height gives
an offset in camera/body metres; the pilot takes a short correction,
settles and observes again. Re-observing a station also places the
existing local aperture detector in its range of validity. Sampled
simulator positions are used only by the author harness to check
localization residuals, never by the flying policy.

The author ablations run the same navigation and rescue policy
without position fixes: once for the entire sortie and once only
after attempted pickup (identical camera-feedback ingress). They still
read RGB for door states, the sled and delivery pad. The pickup-matched
ablation tests the loaded phase in isolation. The
motion tape controls replay an actual successful positive's actions
on the opposite tracking direction with and without 0.025 standard
deviation normalized XY command jitter. A same-direction jitter replay
is a sensitivity check. The jitter applies to the
controller input in that author control only, not to participant flight.
A matched retrace attempts the original inspection slot after a real
pickup and lift.

The protected score remains physical delivery with partial rescue
progress; a fix count, map, localization residual, or visit is not
required. A failed baseline can lose visual registration and stop
rather than hit a wall. The comparison does not establish that every
open-loop program fails, separate the effects of spacing and trim
exactly, or measure flight under unmodeled yaw, height or wind errors.
Learning a motion model or using other visible features remains valid.

In the seed 0 author example, the loaded correction at frames 422–424
reduces the camera-estimated offset from 25.4 cm to 0.8 cm. All 42
completed fixes in that episode are within 2.7 cm of the station in
the simulator. Four matched RGB runs (seeds 0, 1, 38 and 39) delivered
in 354–500 actions, leaving at least 500 actions of margin in the sampled
cases under the 1,000-action allowance. The wall-clock cap is
unchanged at 30 minutes. `loaded_trim_m` sums absolute applied
cross-axis contributions; `command_tracking_offset_m` is the
magnitude of the sum of gain and coupling increments, not a
measurement of route or camera-position error.

On the two matched sites, integrated-pose control delivered 0/4,
and the pickup-matched ablation delivered 0/4 despite all four
achieving lift with identical command prefixes. Those loaded controls
stopped on lost visual registration rather than colliding. The
opposite-direction tapes (exact and jittered) hit walls in 4/4
runs. Same-direction jitter delivered in 1/2; the other hit a wall
with the pod while loaded. This is sample evidence about this
controller and tape transfer, not a claim that feedback is necessary
for all controllers.

## Reproduce

With the pinned ALE engine and `TASK` set to this task directory:

```bash
ale lint "$TASK"
ale run "$TASK" --agent oracle --runs-dir /tmp/ale-rescue-odometry-oracle
ale validate "$TASK" --runs-dir /tmp/ale-rescue-odometry-validate
```

Author checks use the built environment image and an optional writable
`/frames` mount for authentic captures and navigation records:

```bash
docker run --rm --network none --cpus 2 --memory 3g \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD:/task:ro" \
  --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /task/tests/review/check_rescue.py
```

Staged smoke uses the agent UID, real socket and protected verifier; it
is distinct from official ALE validation. On koa04 it scored 0.00
untouched and 1.00 for a fresh RGB oracle, which used 590 actions
against the same 1,000-action allowance. No author test calls a model.
