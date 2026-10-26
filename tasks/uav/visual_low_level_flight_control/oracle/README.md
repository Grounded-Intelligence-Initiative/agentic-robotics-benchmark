# Successful Reference Submission

![Actual Crazyflie simulation](preview.png)

The oracle supplies `run.sh` and `policy.py`, completes the exploration scene,
and leaves its initial/final camera PNGs plus `example_interaction.json` in the
collected submission. The verifier then runs that controller on four new scenes.
The recorded demonstration itself is not a grading input.

The image above is an actual simulator capture from development seed 101. The
large view and close-up are reviewer cameras; the inset is the real agent camera.
It is not a photorealistic rendering, an LLM run or a hardware demonstration.

Six development seeds (101-106) completed all three gates and the required hover
without a collision. Two independent ALE calibration runs also completed all
four private evaluation scenes each. Exact calibration provenance is in
`../verify/anchor.json`. These checks demonstrate solvability and a working
submission lifecycle, not frontier-model difficulty or broad control robustness.

See [the reviewer guide](../tools/REVIEW.md) to reproduce the tests, generate a
video and check that forged success and reset attempts receive no credit.
