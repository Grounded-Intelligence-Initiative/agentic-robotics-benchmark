# Reviewing the G1 command sequence

A submitted process sends one to five rows of 64 original SONIC integer codes. Its only command is the currently revealed stage. One MuJoCo simulation and decoder history run continuously from the first forward walk through the pivot and final departure. The full route is held by the host.

| Review item | Source |
| --- | --- |
| Participant interface, units, score and starter | [Instructions](../../instruction.md) |
| Stage boundaries, public packet, action validation and physical stop checks | [Sequence interface](../../environment/sequence_io.py), [physical scorer](../../environment/sonic_loco/score.py) |
| Official panel and separate feedback checks | [Verifier](../verify.py), [configuration](../grader_config.json), [anchor](../anchor.json) |
| Working reference installation | [Reference installer](../../solution/solve.sh), [reference description](../../solution/README.md) |

## Run the reference in a disposable task environment

From the benchmark checkout on a Linux x86-64 Docker host with AVX2 and FMA, build the public task image. The build downloads and verifies the released SONIC encoder, decoder and G1 assets. The two practice episodes run offline on CPU. The installer writes to the disposable container's participant home.

```sh
task=tasks/humanoid/g1_sonic_command_sequence
docker build -t g1-sonic-sequence-review "$task/environment"
docker run --rm --network none --user user --cpus=4 --memory=8g \
  --mount "type=bind,src=$PWD/$task/solution,dst=/review/solution,readonly" \
  --mount "type=bind,src=$PWD/$task/environment/setup/payload,dst=/review/practice,readonly" \
  g1-sonic-sequence-review bash -euc '
    bash /review/solution/solve.sh
    for turn in left right; do
      /opt/venv/bin/python3 /review/practice/practice.py --turn "$turn" --seed 7 --submission /home/user/submission
    done'
```

Each command prints one JSON summary of the reached stages, stop times, physical errors, applied code count and motion diagnostics. In an existing ALE task environment, add `--trace /tmp/left.jsonl.gz` or `--trace /tmp/right.jsonl.gz` to retain the observations, proposed rows and actual applied counts. A transition can consume part of a proposed chunk. The following packet carries the newly revealed command.

## Run the official verification

From a full benchmark checkout, initialize the pinned engine and build its network helper once on the Docker host. The verifier measures 16 fresh courses in its own sandbox and writes the raw completion and feedback counts to the oracle's `verification.json`.

```sh
task=tasks/humanoid/g1_sonic_command_sequence
git submodule update --init vendor/ale
(cd vendor/ale && VIRTUAL_ENV= uv sync --frozen --all-groups && images/build.sh ale-qemu-runner)
(cd vendor/ale && VIRTUAL_ENV= uv run ale lint "../../$task" && \
  VIRTUAL_ENV= uv run ale validate "../../$task" --runs-dir /tmp/ale-g1-sonic-sequence)
```

## Public seed 7

The offline CPU reference reproduced the 5 m approach, 90 degree pivot and 1.5 m departure in both directions. These are public practice examples. Each official score uses its own 16 trials and measured response checks.

| Turn | First stop qualified | Turn revealed | Departure revealed | Final stop | Final position error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left | 782 | 857 | 997 | 1,222 (24.44 s) | 0.039 m |
| Right | 782 | 857 | 1,007 | 1,235 (24.70 s) | 0.036 m |

Table entries before the elapsed time are 50 Hz control ticks. Both courses share every actual input and code row through tick 856. The first command and row difference occurs when the turn is revealed at tick 857. The responses changed for both opposite-turn probes and stayed identical on all six duplicate queries across the two practice runs. The host withheld each departure distance until that run's own pivot settled. A stopped course that never earns its pivot receives no turn or departure command.

## Official reference calibration

The four prescheduled null-anchor reference panels completed 15, 16, 15 and 13 courses out of 16. All four passed both feedback checks. The first panel fixed the reference at 15/16. The registered minimum full-credit count of 12 and the one-course allowance give `max(12, min(15,16,15,13) - 1) = 12`, so `full_at = 12/15 = 0.8`. Once calibrated, either failed feedback check gives zero. With both passed, the reward is `min(1, completed_courses / 12)` while the raw fraction stays `completed_courses / 16`.
