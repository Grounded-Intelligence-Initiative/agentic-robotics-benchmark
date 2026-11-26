# G1 SONIC token control

This humanoid task evaluates whether a submitted program can control a free-standing G1 through the native token space of the released GEAR-SONIC V1 decoder. It receives synchronized RGB and robot proprioception, then sends integer body codes and two hand commands. A strict placement requires load-bearing grasp, lift, release and stable support on the pad.

**For the participant**, [instruction.md](../../instruction.md) is the complete task prompt, including a runnable starter, the action and observation formats, camera calibration and a local decoder example. This README is for contributors and reviewers.

The interactive companion explains the scoring rule, contrasts feedback control with a successful open-loop replay, and shows four synchronized native rollouts. Use the language switch in its upper-right corner to read it in English or Chinese. [Interactive report](https://zi-ang-cao.github.io/agentic-robotics-benchmark/g1-sonic-token-control/)

## Reproduce a practice rollout

Run these commands from the benchmark repository root on a Linux x86-64 host with Docker and an AVX2/FMA-capable CPU. Building fetches the pinned public model, robot assets and Python wheels. The resulting image runs the task offline using Mesa software rendering.

```bash
task=tasks/humanoid/g1_sonic_token_control
docker build -t g1-sonic-review "$task/environment"
docker run --rm --network none --user user --cpus=4 --memory=8g \
  --mount "type=bind,src=$PWD/$task/solution,dst=/review/solution,readonly" \
  --mount "type=bind,src=$PWD/$task/environment/setup/payload,dst=/review/practice,readonly" \
  -e SONIC_REFERENCE_POLICY=v4 g1-sonic-review \
  bash -euc 'bash /review/solution/solve.sh
    /opt/venv/bin/python3 /review/practice/practice.py run \
      --submission /home/user/submission --task "$1" --seed "$2"' review box_to_pad 7
```

Change `v4` to `p1` for the visual planner, or `box_to_pad` to `cylinder_to_pad` for the other shape. Each invocation starts a fresh container. The mounts make the references available only for this reviewer exercise. The official image contains the public simulator, wire library, original decoder and robot assets. ALE setup places the practice driver in the participant's home. ALE stages the reference and grading code in their designated phases.

| Reference | Input and output at a decision |
| --- | --- |
| **V4 — direct token prediction** | A recurrent visual policy takes three native 128×96 images and robot state, and predicts five consecutive body-token and hand rows. |
| **P1 — visual planning** | A calibrated three-view planner estimates the object and pad from 480×360 images. It evaluates candidate codes against the public decoder history and sends one body-token and hand row at a time. |

Both references are ordinary JSONL submissions. Their implementation, model provenance and installation instructions are in [solution/README.md](../../solution/README.md).

## Run the official benchmark gates

```bash
task=tasks/humanoid/g1_sonic_token_control
git submodule update --init vendor/ale
(cd vendor/ale && uv sync --frozen --all-groups && images/build.sh ale-qemu-runner)
(cd vendor/ale && uv run ale lint "../../$task" && \
  uv run ale validate "../../$task" --runs-dir /tmp/ale-g1-sonic-validation)
uv run python scripts/lint_domain.py "$task"
python3 scripts/task_assets.py check
(cd shared && python3 -B -m robotics_grader.selftest)
uv run pytest -q
```

A successful validation produces an untouched reward of 0.0 and an oracle reward of 1.0. The verifier uses a fresh sandbox, restoring only `/home/user/submission`. Read the oracle episode's `verification.json` for the raw `strict_placement_fraction`, the normalized reward and feedback diagnostics. The benchmark model-difficulty run is a separate `ale run` with an authenticated harness. The repository's [contribution guide](../../../../../CONTRIBUTING.md) describes its configuration and the evidence requested in a pull request.

## Assets and reproducibility

The runtime uses the unmodified NVIDIA body decoder and the Unitree G1/Dex3 description from MuJoCo Menagerie. The Docker build checks every model and mesh against the committed hashes and includes the original licenses. The simulator samples object and pad layouts from named, keyed draws over a 64-bit episode seed. Pick a practice seed to reproduce a layout in this task version. Input frames are rendered at each advertised resolution. The scorer tests forces, footprint and motion at 2 ms intervals; control rows last 20 ms. [Asset notice](../../environment/NOTICE) · [Checksums](../../environment/public_assets.sha256)
