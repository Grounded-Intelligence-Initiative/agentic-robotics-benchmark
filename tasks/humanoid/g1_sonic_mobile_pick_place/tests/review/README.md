# Reviewing mobile SONIC control

The task is one continuous free-base MuJoCo episode. The robot starts behind the table. Its pelvis and both feet must advance, it must settle inside the dock, then it must lift the object with a supported opposed grasp and release it fully on the pad. Decoder memory, contact and velocity are preserved throughout the episode.

| What to inspect | Source |
| --- | --- |
| Solver contract, reference encoder, practice commands | [Participant instructions](../../instruction.md) |
| Public sensors, action validation and native code expansion | [Mobile wire](../../environment/mobile_io.py) |
| Walk and dock geometry, contact loads, timing | [Mobile simulation](../../environment/sonic_sim/mobile.py) |
| Independent physical stage scoring | [Native scorer](../mobile_score.py) |
| Raw denominator, alternate observations and scoring | [Verifier](../verify.py), [configuration](../grader_config.json), [reference anchor](../anchor.json) |

## Reproduce a visible instance

Install the task by the repository's normal procedure. Copy the reference submission into the disposable task environment with its [installer](../../solution/solve.sh), then run this command as the participant. The practice driver starts the real solver on the same JSONL interface, renders the current views, and applies only the body codes and finger commands on the selected branch.

```sh
cd /home/user
/opt/venv/bin/python3 practice.py run --submission submission --task cylinder_to_pad --seed 19
```

The canonical public cylinder at seed 19 reached a stable release at tick 757 (15.14 s). Its dock qualified at tick 176, first hand contact occurred at 484, and the supported lift qualified at 518. The public box at the same seed walked, docked and lifted, then reached the horizon without a strict placement. Treat these as two visible software examples. The verifier counts strict placements across its own 16 fresh starts and separately measures action sensitivity on matched, repeated-time observations.

The high-level submission sees calibrated images and robot proprioception. World object and pad poses, support forces and stage flags are available on the host for scoring. The practice summary exposes final metrics after the physical run. Its short `--steps` mode is for debugging and reports `step_limit` when that cutoff ends a nonterminal run.
