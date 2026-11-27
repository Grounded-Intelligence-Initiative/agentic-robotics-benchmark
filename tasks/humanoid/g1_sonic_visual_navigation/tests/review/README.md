# Reproducing warehouse navigation

A free-root Unitree G1 passes a moving shutter, stops at a blue station, turns toward the revealed outgoing aisle, passes underneath a warehouse shelf and stands at a green inspection station. The two outgoing racks physically exist before the turn cue. The agent receives genuine RGB-D and robot-local sensors and supplies 64 native SONIC V1 body codes at each 50 Hz row. The cameras, body meshes, contacts, doors and racks use the same MuJoCo trajectory throughout.

The public sampler supports rack height and opening-offset families. A low shelf is 1.205–1.225 m above the floor, compared with 1.36–1.42 m for a high shelf. Both offset openings are 0.96–1.06 m wide. The protected grader checks measured gate and rack passage, head-mesh clearance below the shelf, actual whole-body exit, zero obstacle contact at each 2 ms sample and physical quiet stops. At the green station the actual head-mesh apex must stay at least 1.25 m high throughout the final 0.75 s dwell. The participant can choose any body posture that fits these physical requirements. [Physical scoring, sampling, budget and calibration protocol](scoring_protocol.md)

## Public simulation

Run from the checkout root. Docker builds the environment and verifies public assets. The two example runs retain the same initial robot, shutters, geometry seed and final station distance. One selects the low rack offset to the left of the right-going aisle. The other selects the high rack offset to the right of the left-going aisle. The host JSON records physical crossing positions and times, stop summaries and applied code counts.

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
docker build -t g1-sonic-warehouse-navigation:review "$task_dir/environment"
mkdir -p "$PWD/warehouse-review"
for turn in right left; do
  docker run --rm --network none --cpus 4 --memory 8g \
    -v "$task_dir:/task:ro" g1-sonic-warehouse-navigation:review bash -ec '
      cd /task/environment/setup && bash setup.sh
      cp -R /task/solution /tmp/warehouse-solution && chmod -R a+rX /tmp/warehouse-solution
      runuser -u user -- bash /tmp/warehouse-solution/solve.sh
      cd /home/user
      runuser -u user -- /opt/venv/bin/python practice.py --seed 7 --layout-seed 7 --service both \
        --tail right --right-phase-s 6.3 --left-phase-s 11.8 --rack-seed 7 \
        --right-rack low-left --left-rack high-right --turn "$1" --last-m 4.0 --submission submission
    ' _ "$turn" > "$PWD/warehouse-review/$turn.json"
done
```

Run the optics, physical scoring and interrupted-panel accounting fixtures against the same built image.

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
for check in check_camera check_obstacle_score check_panel_budget; do
  docker run --rm --network none --cpus 4 --memory 8g -v "$task_dir/tests:/grade:ro" \
    g1-sonic-warehouse-navigation:review /opt/venv/bin/python "/grade/review/$check.py"
done
```

The camera check exchanges the unrevealed future turn while fixing both racks, both station distances and the first stage. It compares the physical geometry, current packet and native robot motion at reset and over ten additional samples. It also exchanges the shutter phases and verifies a visible RGB and depth difference at reset. The extended dynamics of independently moving shutters can introduce double-precision robot-sensor roundoff on the order of $2 \times 10^{-13}$ before contact. The public sensor protocol contains no explicit shutter phase, outgoing branch selection or private rack geometry fields.

The physical scorer fixture includes mirrored positive routes, nonfinite geometry, roof bypass, incomplete trailing-body exit, mistimed passages and unsuccessful stops. The budget fixture intentionally interrupts a worker after three successful and two failed courses: the actual watchdog and verdict writer preserve the raw score of 3/16. At the start of each protected course the verifier restores the authored submission at its ordinary writable path and provides a new temporary directory.

## Official verification

Provision the recursive checkout and pinned engine using the repository contributor instructions. [Contributor setup](../../../../../CONTRIBUTING.md)

```sh
task_dir="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
(
  cd vendor/ale
  .venv/bin/ale lint "$task_dir"
  .venv/bin/ale validate "$task_dir"
)
```

Each protected draw assigns sixteen courses, twelve with low shelves and four with high shelves. Review the raw low and high counts, contact samples, entry/exit and whole-head-under-roof events, upright inspection dwell and applied native motion alongside normalized reward. A full native trajectory has at most 4,350 applied rows (87 seconds). The verifier allows 60 wall seconds for `READY`, 15 wall seconds per reset/action exchange, and 3,540 seconds for the entire protected panel inside the engine’s 3,600-second limit.
