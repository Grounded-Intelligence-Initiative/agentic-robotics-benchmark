# Successful Reference Submission

![Actual Crazyflie simulation](preview.png)

The v6 oracle supplies `run.sh` and `policy.py`, completes the exploration scene,
and leaves its initial/final camera PNGs plus `example_interaction.json` in the
collected submission. The verifier then runs that controller on four new scenes.
The recorded demonstration itself is not a grading input.

It reconstructs metric gate poses from the single onboard camera's corner
markings, maintains a motion observer, and uses feedback for approach, crossing,
search and hover. No simulator imports or privileged runtime-state reads occur.
It has **author-supplied design priors**, including left-turn topology and nominal
dynamics. It is a solvability reference, not a fair uninformed model baseline.

The image above is an actual simulator capture from development seed 101. The
large view and close-up are reviewer cameras; the inset is the real agent camera.
It is not a photorealistic rendering, an LLM run or a hardware demonstration.

![Four-gate route and measured flight](route.png)

Four gates require yellow > green > magenta > cyan crossings. White floor arrows
specify direction; colored entry faces have white corners and gray backs. The
course turns around an opaque cabinet and returns toward the starting side.
The route image overlays reviewer labels and the measured trajectory on an
overhead capture unavailable to the controller. Only the onboard inset is public.

Six development seeds (101-106) and sixteen additional perturbed scenes (201-208,
301-308) completed without collisions. The latter passed 148/215 observation
probes; repeated-time calls are branched and duplicate calls are idempotent.
Final-policy ALE calibration completed 4/4 fresh scenes with zero collisions and
30/39 probes, in 25.816-26.960 simulated seconds. Provenance is in
`../verify/anchor.json`; no old external-camera calibration is reused.
These checks demonstrate solvability, not frontier-model difficulty or hardware
robustness. The separate `tools/privileged_reference.py` uses true state and must
never be confused with this submitted image-only reference.

See [the reviewer guide](../tools/REVIEW.md) to reproduce the tests, generate a
video and check that forged success and reset attempts receive no credit.
