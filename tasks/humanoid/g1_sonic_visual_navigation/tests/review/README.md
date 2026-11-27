# Reproducing timed cabinet navigation

A free-root G1 must find a traversable opening between a stationary island and two moving shutters. At least one shutter can retract fully. A restricted shutter still moves visibly. Many starts also have a second, offset cabinet after the timed doorway. The robot continues through the blue stop, signed pivot and final walking stop in one native MuJoCo trajectory. The G1 receives an egocentric RGB-D camera and robot-local sensors and sends one or more genuine 64-code SONIC V1 body rows per decision.

The verifier reconstructs the exposed opening from the **measured native slide position** at the forward moving-plane crossing. It checks the reported plane position, nearest-edge clearance, crossing time, chosen side, and the later fixed-frame passage through that same opening. A contact by any robot collision geom with a moving or static obstacle invalidates the course, even after the pelvis clears the moving plane. No hidden side label, deadline within the shutter cycle or response to a visual probe grants score. [Full sampling and measurement protocol](scoring_protocol.md)

## Run the public course

Run from the checkout root with a working Docker daemon. These commands build the checksum-pinned task environment and install the public reference as its ordinary unprivileged participant. The two physical rollouts use the same robot and layout, with their two shutter phases exchanged. The file for each rollout contains its actual gate crossings, measured slide position, stop and contact events, time and body code cost.

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
docker build -t g1-sonic-timed-navigation:review "$task_dir/environment"
mkdir -p "$PWD/navigation-review"
for trial in 1 2; do
  if [ "$trial" -eq 1 ]; then right=6.3; left=11.8; else right=11.8; left=6.3; fi
  docker run --rm --network none --cpus 4 --memory 8g \
    -v "$task_dir:/task:ro" g1-sonic-timed-navigation:review bash -ec '
      cd /task/environment/setup && bash setup.sh
      cp -R /task/solution /tmp/navigation-solution && chmod -R a+rX /tmp/navigation-solution
      runuser -u user -- bash /tmp/navigation-solution/solve.sh
      cd /home/user
      runuser -u user -- /opt/venv/bin/python practice.py --seed 7 --layout-seed 7 --service both \
        --tail right --right-phase-s "$1" --left-phase-s "$2" --turn left --submission submission
    ' _ "$right" "$left" > "$PWD/navigation-review/phase-$trial.json"
done
```

Run the public sensor and scorer checks against the same built image.

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
docker run --rm --network none --cpus 4 --memory 8g -v "$task_dir/tests:/grade:ro" \
  g1-sonic-timed-navigation:review /opt/venv/bin/python /grade/review/check_camera.py
docker run --rm --network none --cpus 4 --memory 8g -v "$task_dir/tests:/grade:ro" \
  g1-sonic-timed-navigation:review /opt/venv/bin/python /grade/review/check_obstacle_score.py
```

The camera test puts the same robot and furniture in corresponding scenes. It checks that physically different shutter phases are visible in RGB and depth, and that an unrevealed future turn leaves the current sensor packet and motion unchanged. The scorer test checks 64 independent public sixteen-course roster draws and rejects stale or fabricated shutter bounds, negative clearance, reversed or missing passages, robot contacts and failed stops.

## Official verification

Provision the recursive checkout and pinned runner using the contributor setup. [Contributor setup](../../../../../CONTRIBUTING.md)

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
(
  cd vendor/ale
  .venv/bin/ale lint "$task_dir"
  .venv/bin/ale validate "$task_dir"
)
```

The two protected reference measurements and the separate held native cohort use different reset draws. The official record reports the raw completed-course fraction, numerical reward, per-family outcomes, actual passages, contacts, native seconds, applied FSQ coordinates, token churn and pooled joint acceleration. Each recorded shutter crossing includes its measured width and both edges. A sampled pelvis clearance is a geometric point measurement. Whole-body collision monitoring covers every 2 ms native step.
