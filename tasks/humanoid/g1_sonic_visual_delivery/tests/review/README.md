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

Run both signal directions for a public layout. The commands install the reference within a disposable container, then start each practice case as the image's unprivileged participant. Each host JSON file contains physical stages and terminal reason, action counts, motion cost, and repeated-query measurements. Set `--start far` to exercise the walking requirement.

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

With the checkout's ALE environment provisioned, run the engine from the checkout root.

```bash
vendor/ale/.venv/bin/ale lint tasks/humanoid/g1_sonic_visual_delivery
vendor/ale/.venv/bin/ale validate tasks/humanoid/g1_sonic_visual_delivery
```

## Measurement

The graded unit is one physical layout with two separately initialized controllers and opposite signals. Each receives its own fresh process, writable temporary area and randomized probe schedule. The layout earns one when both controllers reach a correct, loaded, contained, released placement. Opposite lights cannot change the native dynamics before the controller takes a different action. The first fully qualified placement on the wrong pad terminates the case.

A far start must satisfy forward pelvis and ankle travel, foot rise, alternating real support and continuous docking before hand contact. A near start already lies within reach and must satisfy the docking dwell. Torso and object progress come from simulated contact loads, geom extents, velocities and native positions. The independent scorer uses the resolved pose/contact solve in each 2 ms interval, object velocity from immediately before integration, and pelvis planar velocity at the end of that interval. Its criterion must agree with native scene success for a scored delivery.

Each successfully applied 0.02 s row contributes 64 FSQ scalars. Coordinate churn counts successive rows within each episode across 64 slots. Motion smoothness pools squared differences of the 29 measured joint velocities, spaced by five rows (0.10 s) and divided by 0.10 s. Each episode contributes 29 × max(0, applied rows − 4) scalar acceleration samples. A zero denominator yields zero. Unapplied commands, repeats and episode boundaries contribute no transitions or samples. The verifier writes per-episode physical details and aggregate counts, including individual deliveries for each start, shape and destination.

Repeated queries are scheduled from time alone, with random order. The two unlit pads produce a neutral comparison before docking. Once the light is visible, the alternative uses physically rendered RGB with the selection exchanged and identical captured depths. Neither response differences nor rehearsal tick timing contributes reward. A fixed trajectory that reaches a single pad may earn an individual delivery. Its paired score is zero because that destination is incorrect in the other member. A controller can still guess both directions by chance. On the public near-box development case 406101, a measured 507-row tape yielded one correct left release and one right wrong-pad termination at the same native tick. A pixel-responsive edit on the penultimate row selected first-coordinate bins 0 and 7 for the two lights and gave the same physical outcomes. Both controllers scored zero on the pair. The executable replay is `tests/review/replay_controls.py` and accepts an independently collected tape with aligned `codes` and `hands` arrays.

The CPU environment pins MuJoCo, the native SONIC V1 decoder/encoder and the numerical runtime. The reference controller uses public images and robot-local sensors through the same JSONL interface as participants. A successful diagnostic or a published development seed is specific to its physical conditions.
