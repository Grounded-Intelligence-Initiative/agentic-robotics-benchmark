# Review the SONIC walk, turn and stop task

A submission receives a goal and the measured state of a free-standing Unitree G1. Its only action is a row of 64 native SONIC codes for the next 20 ms. It must translate to a waypoint or turn clockwise about its starting position, then hold still. The [participant instructions](../../instruction.md) describe the JSONL interface, the released codec and how to install the neutral starter.

## Run the shipped controller

From the benchmark repository root on Linux x86-64, build the CPU image once. The build fetches the original model weights, robot description and meshes and verifies the [pinned checksums](../../environment/public_assets.sha256). Both practice runs below use the same image and run without network access or a GPU.

```bash
task=tasks/humanoid/g1_sonic_locomotion
docker build -t g1-sonic-locomotion-review "$task/environment"
docker run --rm --network none --user user --cpus=4 --memory=8g \
  --mount "type=bind,src=$PWD/$task/solution,dst=/review/solution,readonly" \
  --mount "type=bind,src=$PWD/$task/environment/setup/payload,dst=/review/practice,readonly" \
  g1-sonic-locomotion-review bash -euc '
    bash /review/solution/solve.sh
    /opt/venv/bin/python3 /review/practice/practice.py --task straight_stop --distance-m 5 --seed 7
    /opt/venv/bin/python3 /review/practice/practice.py --task right_turn_stop --turn-deg -90 --seed 7'
```

Each line of output is a JSON result for a fresh start. On these two practice routes, the reference stopped 4.893 m forward and turned 84.65° clockwise, respectively. Both held the specified endpoint for 0.75 s. The [reference implementation](../../solution/README.md) generates its own joint-space gait, encodes it into integer SONIC coordinates, and adjusts its next action using measured progress and velocity. The native simulator applies the released decoder on every row.

## Reproduce the official score

```bash
task=tasks/humanoid/g1_sonic_locomotion
git submodule update --init vendor/ale
(cd vendor/ale && uv sync --frozen --all-groups && images/build.sh ale-qemu-runner)
(cd vendor/ale && uv run ale lint "../../$task" && \
  uv run ale validate "../../$task" --runs-dir /tmp/ale-g1-locomotion)
uv run python scripts/lint_domain.py "$task"
python3 scripts/task_assets.py check
(cd shared && python3 -B -m robotics_grader.selftest)
uv run pytest -q
```

A validation runs the untouched baseline and the reference solver. Open the oracle's `verification.json` in the run directory to inspect the raw results and the normalized ALE reward. As a separate experiment, `ale run` with a coding-agent harness evaluates whether that agent can build its own controller from the participant prompt. The [contribution guide](../../../../../CONTRIBUTING.md) covers model runs and budgets.

| Measure | Interpretation |
| --- | --- |
| Successful stops | The denominator includes all 16 trials, eight straight and eight right turns. Distance, signed angle, drift, speed, uprightness and continuous dwell determine physical success. |
| Feedback | Hidden queries change the measured state with the goal held fixed, and change the requested magnitude with the robot held fixed. Identical repeats measure action noise. A blind replay can complete a practiced route and still score zero. |
| Simulated time | The trustworthy clock is native simulation time. The stop or failure can occur partway through a 20 ms row. Inspect successful completion times alongside failures and timeouts. |
| Motion-token use | Each applied row costs 64 FSQ scalars. Adjacent-row Hamming fraction measures how often the coordinates change. Discarded probe replies consume no motion tokens. |
| Smoothness and gait | RMS angular acceleration uses 29 measured joint velocities and a 0.10 s difference interval. Foot-load transitions and straight or angular path efficiency add context to that number. |

The normalizer and eligibility checks are specified in the [calibration plan](anchor_protocol.json) and [scoring guide](scoring_protocol.md). Time, motion-token use and acceleration are reported separately from the primary success score. ALE records a coding agent's model-token consumption in its own `trace.transport.jsonl` outside the simulator.
