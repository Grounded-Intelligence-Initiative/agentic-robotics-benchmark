# Successful Reference Submission

![Actual Crazyflie simulation](preview.png)

The v7 oracle supplies `run.sh` and `policy.py`, completes the exploration scene,
and leaves its initial/final camera PNGs plus `example_interaction.json` in the
collected submission. The verifier then runs that controller on four new scenes.
The recorded demonstration itself is not a grading input.

It reconstructs metric gate poses from the single onboard camera's corner
markings, maintains a motion observer, and uses feedback for approach, crossing,
search and hover. No simulator imports or privileged runtime-state reads occur.
It has **author-supplied design priors**, including nominal switchback geometry,
safe transit waypoints and dynamics. The nominal route is registered to visual
landmarks; actual gate poses and the goal are measured from images. This is a
solvability reference, not a fair uninformed model baseline.

The image above is an actual simulator capture from development seed 101. The
large view and close-up are reviewer cameras; the inset is the real agent camera.
It is not a photorealistic rendering, an LLM run or a hardware demonstration.

![Four-gate route and measured flight](route.png)

Four gates require yellow > green > magenta > cyan crossings. White floor arrows
specify direction; colored entry faces have white corners and gray backs. The
course reverses entry directions, alternates low/high gates, routes around an
opaque cabinet and returns toward the starting side.
The route image overlays reviewer labels and the measured trajectory on an
overhead capture unavailable to the controller. Only the onboard inset is public.

The frozen reference completed 26 development/regression/holdout scenes with
zero collisions (41.83-46.42 simulated seconds); 20 probed scenes passed 323/432
probes. ALE calibration `validate-2f68303c` then completed four fresh private
scenes, zero collisions/aborts, 70/92 aggregate probes, reward 1.0. One scene's
11/23 probe sensitivity was below half; the task aggregates probes across scenes.
Provenance is in `../verify/anchor.json`. No earlier-version results are reused.
These checks demonstrate solvability, not frontier-model difficulty or hardware
robustness. The separate `tools/privileged_reference.py` uses true state and must
never be confused with this submitted image-only reference.

See [the reviewer guide](../tools/REVIEW.md) to reproduce the tests, generate a
video and check that forged success and reset attempts receive no credit.
