# Reviewing mobile SONIC control

The task is one continuous free-base MuJoCo episode. The robot starts behind the table. Its pelvis and both feet must advance, it must settle inside the dock, then it must lift the object with a supported opposed grasp and release it fully on the pad. Decoder memory, contact and velocity are preserved throughout the episode.

The scorer evaluates docking over 250 consecutive 2 ms intervals. Each interval uses starting pose and contact geometry together with planar pelvis speed after integration. Lift and release use the aligned pre-integration object pose, velocity and contact forces.

| What to inspect | Source |
| --- | --- |
| Solver contract, reference encoder, practice commands | [Participant instructions](../../instruction.md) |
| Public sensors, action validation and native code expansion | [Mobile wire](../../environment/mobile_io.py) |
| Walk and dock geometry, contact loads, timing | [Mobile simulation](../../environment/sonic_sim/mobile.py) |
| Independent physical stage scoring | [Native scorer](../mobile_score.py) |
| Raw denominator, alternate observations and scoring | [Verifier](../verify.py), [configuration](../grader_config.json), [reference anchor](../anchor.json) |

## Reproduce the public physical metrics

From the public benchmark checkout root on Linux x86-64, build the native CPU image. Its build fetches and verifies the pinned robot geometry and model weights. Each command below runs the corresponding controller in a fresh network-isolated container as the participant user, with read-only mounts for the reference and practice code. [Public asset checksums](../../environment/public_assets.sha256)

```bash
task=tasks/humanoid/g1_sonic_mobile_pick_place
docker build -t g1-sonic-mobile-review "$task/environment"

docker run --rm --network none --user user --cpus=4 --memory=8g \
  --mount "type=bind,src=$PWD/$task/solution,dst=/review/solution,readonly" \
  --mount "type=bind,src=$PWD/$task/environment/setup/payload,dst=/review/practice,readonly" \
  g1-sonic-mobile-review bash -euc '
    bash /review/solution/solve.sh
    /opt/venv/bin/python3 /review/practice/practice.py run \
      --course one-metre --submission /home/user/submission --task cylinder_to_pad --seed 19'

docker run --rm --network none --user user --cpus=4 --memory=8g \
  --mount "type=bind,src=$PWD/$task/solution,dst=/review/solution,readonly" \
  --mount "type=bind,src=$PWD/$task/environment/setup/payload,dst=/review/practice,readonly" \
  g1-sonic-mobile-review bash -euc '
    bash /review/solution/five_metre/install.sh
    /opt/venv/bin/python3 /review/practice/practice.py run \
      --course five-metre --submission /home/user/submission --task cylinder_to_pad --seed 19'
```

The one-metre course supplies the graded task. The five-metre course is optional practice. Its public CPU seed-19 cylinder run reached the dock after 5.013 m of pelvis travel in world +x and completed all stages in 1,286 control rows (25.72 s). Each practice summary includes native physical stages, repeated-observation controls and a `motion` object with actual control costs.

The one-metre CPU Mesa practice invocation reached a stable release at tick 757 (15.14 s) for the public cylinder at seed 19. The dock qualified at tick 176, hand contact first occurred at 481, and the supported lift qualified at 520. A separately recorded canonical film of the same start also docked at 176 and placed at 757. That recording first contacted at 484 and qualified its lift at 518. The public box at the same seed walked, docked and lifted, then reached the horizon without a strict placement. Treat these as two visible software examples. The verifier counts strict placements across its own 16 fresh starts and separately measures action sensitivity on matched, repeated-time observations.

The high-level submission sees calibrated images and robot proprioception. World object and pad poses, support forces and stage flags are available on the host for scoring. The practice summary exposes final metrics after the physical run. Use `practice.py inspect --out DIR --task cylinder_to_pad --seed 19` to save the three initial camera PNGs and their calibration. The run command’s short `--steps` mode is for debugging and reports `step_limit` when that cutoff ends a nonterminal run.


## Physical smoothness

At reset and after each applied 20 ms control row, the verifier samples the 29 body-joint velocities from native proprioception. For each available pair five ticks apart, it divides the velocity change by 0.10 s. The panel metric `joint_accel_rms_rad_s2` is the square root of the pooled sum of squared accelerations divided by `joint_acceleration_samples`. This denominator counts scalar joint measurements across all applied rows and episodes, including completed prefixes of aborted episodes. Each episode begins a fresh window. A four-tick prefix contributes zero measurements, a five-tick prefix contributes 29, and an empty pool reports zero with denominator zero. The separate panel diagnostic `mean_target_step_rms_rad` measures changes in decoder joint targets. Physical success and the closed-loop observation checks determine reward.


## Run the official ALE physical checks

From the benchmark checkout root, initialize its pinned ALE engine and provision the runner following the contribution guide. Store evaluation runs in an external writable directory with sufficient free space. [Contribution guide](../../../../../CONTRIBUTING.md)

```bash
git submodule update --init vendor/ale
task=tasks/humanoid/g1_sonic_mobile_pick_place
runs_dir="$PWD/../g1-mobile-validation-runs"
(
  cd vendor/ale
  uv sync --frozen --all-groups
  images/build.sh ale-qemu-runner
  uv run ale lint "../../$task"
  uv run ale validate "../../$task" --runs-dir "$runs_dir"
)
```

The validation runs untouched and oracle submissions in separate sandboxes. Inspect the raw `strict_mobile_placement_fraction`, physical stage counts, observation controls, native motion diagnostics and normalized `reward` in the generated per-arm `verification.json` files. The full-credit threshold uses the frozen four-panel procedure. [Calibration protocol](anchor_protocol.json)

The null-anchor bootstrap and spread panels produced 10 / 10 / 13 / 13 strict placements out of 16 in order. The nominal reference fraction is 0.625 and full credit requires 9 placements with a passing observation check. [Measured calibration receipts](measured_calibration.json)
