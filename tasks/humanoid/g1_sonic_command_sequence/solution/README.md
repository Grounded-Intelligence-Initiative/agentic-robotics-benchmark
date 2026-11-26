# Reference controller

The controller receives the current measured pelvis-relative distance, heading error, base velocity and yaw rate. It synthesizes ten future poses from a 0.9 s procedural stepping cycle, encodes those references with the released SONIC V1 encoder, and returns one native 64-coordinate FSQ row. The verifier executes that row through the frozen decoder and live MuJoCo contacts. Each action depends on the latest packet, so repeated queries at a single tick do not advance the policy.

The two forward commands use remaining distance and forward velocity to shorten the stride near the target and yaw feedback to reduce cross-track motion. The pivot uses signed yaw error to turn either way, measured yaw rate to predict braking, and the measured offset from the pivot center to limit drift. Phase and warmup start at each command reveal. The long approach, pivot and final departure share a continuous physical world and decoder history. The reference uses the original benchmark author's analytical gait and the public frozen codec. It reads no motion clips, saved code tapes or verifier seeds.

Run `bash solution/solve.sh` to install the entry point in `$ALE_HOME/submission` (default `/home/user/submission`). The shipped practice runner accepts `--turn left` and `--turn right` for a five-metre approach, a 90° pivot, and a 1.5 m departure.
