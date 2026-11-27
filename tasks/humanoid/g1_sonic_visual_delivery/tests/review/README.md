# Review visual delivery

A single free-root G1 starts near or far from a tabletop object, establishes a quiet dock, and delivers the object to the pad that lights up. Both signal directions are physically instantiated on each layout. Object, robot, contact model, receiver geometry and camera rig match across the two members. Before docking both pads are gray. At docking the selected existing geom changes material colour. The controller receives native RGB-D, measured joints and body-frame IMU readings. The protected grader separately integrates physical support, grasp and release from ten native contact samples per body action.

## Run from a public checkout

Run these commands from the repository root with Docker available. The build fetches checksum-pinned public SONIC and G1 assets. The first command saves three RGB PNGs, three 16-bit axial depth PNGs and camera intrinsics to a directory owned by the invoking user.

```bash
task_dir=$PWD/tasks/humanoid/g1_sonic_visual_delivery
docker build -t g1-sonic-visual-delivery:review "$task_dir/environment"
mkdir -p "$PWD/visual-delivery-views"
docker run --rm --network none --cpus 4 --memory 8g \
  --user "$(id -u):$(id -g)" --env HOME=/tmp \
  -v "$task_dir/environment/setup/payload:/practice:ro" \
  -v "$PWD/visual-delivery-views:/output" \
  g1-sonic-visual-delivery:review /opt/venv/bin/python3 /practice/practice.py \
  inspect --seed 19 --start far --out /output
```

Run both signal directions for a public layout. The commands install the reference within a disposable container, then start each practice case as the image's unprivileged participant. Each host JSON file contains physical stages and terminal reason, action counts, motion cost, and repeated-query measurements. Set `--start far` to exercise the walking requirement. On the pinned software renderer the example's left trial completed in 523 rows (10.46 s). The right trial lifted the box and reached the 1,100-row time limit without a qualifying release.

```bash
task_dir=$PWD/tasks/humanoid/g1_sonic_visual_delivery
mkdir -p "$PWD/visual-delivery-results"
for side in left right; do
  docker run --rm --network none --cpus 4 --memory 8g \
    -v "$task_dir/environment/setup/payload:/practice:ro" \
    -v "$task_dir/solution:/solution:ro" \
    g1-sonic-visual-delivery:review /bin/bash -ec '
      bash /solution/solve.sh
      chown -R user:user /home/user/submission
      exec runuser -u user -- /opt/venv/bin/python3 /practice/practice.py run \
        --seed 406101 --start near --shape box --target "$1" --submission /home/user/submission
    ' _ "$side" > "$PWD/visual-delivery-results/near-box-$side.json"
done
```

Provision the pinned engine and runner image using the repository's [contributor setup](../../../../../CONTRIBUTING.md). Run this block from the checkout root.

```bash
task_dir=$PWD/tasks/humanoid/g1_sonic_visual_delivery
(
  cd vendor/ale
  .venv/bin/ale lint "$task_dir"
  .venv/bin/ale validate "$task_dir"
)
```

## Scored layouts and generalization

Each verdict independently selects one of two reference-qualified layouts for each near/far × box/cylinder combination. The continuous generator remains available in public practice. The reference was frozen before the qualification seeds were opened. Both signal directions were run in their registered order and the first two complete layouts in each stratum were retained. Across all attempted layouts, 18 of 20 directional runs completed. The qualification record includes every attempted seed and failure, the nonce and recipe, the exact source and image. [Qualification record](layout_qualification.json)

Measurements on newly drawn, unqualified layouts describe generalization. They do not enter the finite challenge score. The initial official random-domain bootstrap completed 12 of 12 physical deliveries on four layouts, and the null anchor returned zero normalized reward. Trials sharing the same layout and signal had identical physical and motion measurements across all four repeated signals.

## Measurement

The graded unit is one physical layout with three independently initialized controllers. The grader samples one of the six nonconstant three-signal sequences uniformly, then interleaves cases across layouts. Both destinations occur. Each receives its own fresh process, private copy of the submitted files, writable temporary area and randomized probe schedule. The layout earns one when all three episodes reach a correct, loaded, contained, released placement. Opposite lights cannot change the native dynamics before the controller takes a different action. The first fully qualified placement on the wrong pad terminates the case.

A far start must satisfy forward pelvis and ankle travel, foot rise, alternating real support and continuous docking before hand contact. A near start already lies within reach and must satisfy the docking dwell. Torso and object progress come from simulated contact loads, geom extents, velocities and native positions. The independent scorer uses the resolved pose/contact solve in each 2 ms interval, object velocity from immediately before integration, and pelvis planar velocity at the end of that interval. Its criterion must agree with native scene success for a scored delivery.

Each successfully applied 0.02 s row contributes 64 FSQ scalars. Coordinate churn counts successive rows within each episode across 64 slots. Motion smoothness pools squared differences of the 29 measured joint velocities, spaced by five rows (0.10 s) and divided by 0.10 s. Each episode contributes 29 × max(0, applied rows − 4) scalar acceleration samples. A zero denominator yields zero. Unapplied commands, repeats and episode boundaries contribute no transitions or samples. In the ALE verification record, `trial_01_…` and subsequent numbered metrics identify each layout, start, shape, signal, physical stages, actual action count, simulated seconds and smoothness denominators. Event times appear when the stage was reached. The aggregate counts include individual deliveries for each start, shape and destination.

Repeated queries are scheduled from time alone, with random order. The two unlit pads produce a neutral comparison before docking. Once the light is visible, the alternative uses physically rendered RGB with the selection exchanged and identical captured depths. Neither response differences nor rehearsal tick timing contributes reward. A fixed trajectory that reaches a single pad may earn an individual delivery. Its layout score is zero because at least one episode requires the other destination. A current-signal-blind controller with perfect physical delivery can guess all three at most one time in six, even if it remembers earlier cues. An all-correct sequence follows just one path through its past observations. Across four independent layouts the corresponding bound for full credit is 1/1,296. Timing, randomness and read-only metadata can remain shared on ordinary Linux systems; the scoring bound permits knowledge of all previous signals. On the public near-box development case 406101, a measured 507-row tape yielded one correct left release and one right wrong-pad termination at the same native tick. A pixel-responsive edit on the penultimate row selected first-coordinate bins 0 and 7 for the two lights and gave the same physical outcomes. Both controls scored zero on the two opposite directions, guaranteeing zero for any three-member evaluation containing these same two physical outcomes. The executable replay is `tests/review/replay_controls.py` and accepts an independently collected tape with aligned `codes` and `hands` arrays.

The official Linux verifier requires Landlock ABI 3 or newer for per-episode writable storage and seccomp for network and kernel IPC restrictions. The published AMD64 image checks its Haswell-compatible numerical runtime before starting the grader. The CPU environment pins MuJoCo and the native SONIC V1 decoder and encoder. The reference controller uses public images and robot-local sensors through the same JSONL interface as participants. A successful diagnostic or a published development seed is specific to its physical conditions.
