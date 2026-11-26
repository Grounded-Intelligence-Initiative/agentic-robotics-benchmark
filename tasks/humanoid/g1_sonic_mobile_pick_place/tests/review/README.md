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

Install the task by the repository's normal procedure. Copy the reference submission into the disposable task environment with its [installer](../../solution/solve.sh), then run this command as the participant. The practice driver starts the real solver on the same JSONL interface, renders the current views, and applies only the body codes and finger commands on the selected branch.

```sh
cd /home/user
/opt/venv/bin/python3 practice.py run --submission submission --task cylinder_to_pad --seed 19
```

The CPU Mesa practice invocation above reached a stable release at tick 757 (15.14 s) for the public cylinder at seed 19. The dock qualified at tick 176, hand contact first occurred at 481, and the supported lift qualified at 520. A separately recorded canonical film of the same start also docked at 176 and placed at 757. That recording first contacted at 484 and qualified its lift at 518. The public box at the same seed walked, docked and lifted, then reached the horizon without a strict placement. Treat these as two visible software examples. The verifier counts strict placements across its own 16 fresh starts and separately measures action sensitivity on matched, repeated-time observations.

The high-level submission sees calibrated images and robot proprioception. World object and pad poses, support forces and stage flags are available on the host for scoring. The practice summary exposes final metrics after the physical run. Use `practice.py inspect --out DIR --task cylinder_to_pad --seed 19` to save the three initial camera PNGs and their calibration. The run command’s short `--steps` mode is for debugging and reports `step_limit` when that cutoff ends a nonterminal run.


## Physical smoothness

At reset and after each applied 20 ms control row, the verifier samples the 29 body-joint velocities from native proprioception. For each available pair five ticks apart, it divides the velocity change by 0.10 s. The panel metric `joint_accel_rms_rad_s2` is the square root of the pooled sum of squared accelerations divided by `joint_acceleration_samples`. This denominator counts scalar joint measurements across all applied rows and episodes, including completed prefixes of aborted episodes. Each episode begins a fresh window. A four-tick prefix contributes zero measurements, a five-tick prefix contributes 29, and an empty pool reports zero with denominator zero. The separate panel diagnostic `mean_target_step_rms_rad` measures changes in decoder joint targets. Physical success and the closed-loop observation checks determine reward.
