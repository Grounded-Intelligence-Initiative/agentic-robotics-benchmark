# Reviewing the G1 command sequence

A submitted process sends one to five rows of 64 original SONIC integer codes. Its only command is the currently revealed stage. One MuJoCo simulation and decoder history run continuously from the first forward walk through the pivot and final departure. The full route is held by the host.

| Review item | Source |
| --- | --- |
| Participant interface, units, score and starter | [Instructions](../../instruction.md) |
| Stage boundaries, public packet, action validation and physical stop checks | [Sequence interface](../../environment/sequence_io.py), [physical scorer](../../environment/sonic_loco/score.py) |
| Official panel and separate feedback checks | [Verifier](../verify.py), [configuration](../grader_config.json), [anchor](../anchor.json) |
| Working reference installation | [Reference installer](../../solution/solve.sh), [reference description](../../solution/README.md) |

## Run the reference in a disposable task environment

First provision the task's public environment and complete its participant setup. From a checkout or evaluation fixture where `TASK_DIR` points to this task's directory and `/home/user` is the participant home, run the installer as the participant. It replaces the practice submission in that home.

```sh
ALE_HOME=/home/user bash "$TASK_DIR/solution/solve.sh"
cd /home/user
/opt/venv/bin/python3 practice.py --turn left  --seed 7 --submission submission --trace /tmp/left.jsonl.gz
/opt/venv/bin/python3 practice.py --turn right --seed 7 --submission submission --trace /tmp/right.jsonl.gz
```

Each command prints one JSON summary of the reached stages, stop times, physical errors, applied code count and motion diagnostics. Each trace contains the chosen public observation, proposed body rows and number applied. A transition can consume part of a proposed chunk. The following packet carries the newly revealed command.

## Public seed 7

The offline CPU reference reproduced the 5 m approach, 90 degree pivot and 1.5 m departure in both directions. These are public practice examples. Each official score uses its own 16 trials and measured response checks.

| Turn | First stop qualified | Turn revealed | Departure revealed | Final stop | Final position error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left | 782 | 857 | 997 | 1,222 (24.44 s) | 0.039 m |
| Right | 782 | 857 | 1,007 | 1,235 (24.70 s) | 0.036 m |

Table entries before the elapsed time are 50 Hz control ticks. Both courses share every actual input and code row through tick 856. The first command and row difference occurs when the turn is revealed at tick 857. The responses changed for both opposite-turn probes and stayed identical on all six duplicate queries across the two practice runs. The host withheld each departure distance until that run's own pivot settled. A stopped course that never earns its pivot receives no turn or departure command.

## Official reference calibration

The four prescheduled null-anchor reference panels completed 15, 16, 15 and 13 courses out of 16. All four passed both feedback checks. The first panel fixed the reference at 15/16. The registered minimum full-credit count of 12 and the one-course allowance give `max(12, min(15,16,15,13) - 1) = 12`, so `full_at = 12/15 = 0.8`. Once calibrated, either failed feedback check gives zero. With both passed, the reward is `min(1, completed_courses / 12)` while the raw fraction stays `completed_courses / 16`.
