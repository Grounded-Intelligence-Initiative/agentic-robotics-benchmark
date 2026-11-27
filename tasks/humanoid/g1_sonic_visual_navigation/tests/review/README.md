# Reproducing visual obstacle navigation

The free-root G1 walks through collidable gates, stops and holds, turns in place toward a newly lit heading, walks the final leg and stops. The native body-mounted camera renders the real MuJoCo scene. The distance cues and turn lamps have no contact geometry. Gate barriers participate in ordinary native contacts with all robot geoms throughout the rollout. Scoring uses the ordered first passage of the pelvis through every opening together with contact-bearing samples from the full robot and all three measured stops.

## Run the public course

From a full benchmark checkout, build the task image and run a new physical practice case. The public layout seed controls the gate layout. The robot and layout seeds are separate. Change either to examine additional courses.

```sh
task=tasks/humanoid/g1_sonic_visual_navigation
docker build -t g1-visual-navigation "$task/environment"
docker run --rm --network none --cpus 4 --memory 8g -v "$PWD/$task:/task:ro" \
  g1-visual-navigation bash -ec '
    cd /task/environment/setup && bash setup.sh
    cp -R /task/solution /tmp/navigation-solution && chmod -R a+rX /tmp/navigation-solution
    runuser -u user -- bash /tmp/navigation-solution/solve.sh
    cd /home/user
    runuser -u user -- /opt/venv/bin/python practice.py --turn left --seed 7 --layout-seed 19 --submission submission
  '
docker run --rm --network none --cpus 4 --memory 8g -v "$PWD/$task/tests:/grade:ro" \
  g1-visual-navigation /opt/venv/bin/python /grade/review/check_obstacle_score.py
```

The second command exercises the actual hidden-course generation algorithm on public test streams. It checks the strict scoring rule against fabricated contact, skipped, wrong-side and out-of-order crossings, unfinished stops, aborts and changed body-code coordinates.

The preceding public controller was also run through the restricted JSONL verifier on robot and obstacle seed 7, with a 6 m approach, 90° left pivot and 1.5 m final leg. It walked into the first cabinet with the right index-finger link at 4.750 s, on control row 238. Six 2 ms samples had a robot–cabinet contact. It ended with no qualified passages or stops and a raw course score of zero.

## Official verification

Provision the recursive checkout and pinned runner according to the contributor guide. [Contributor setup](../../../../../CONTRIBUTING.md)

```sh
task="$PWD/tasks/humanoid/g1_sonic_visual_navigation"
(
  cd vendor/ale
  .venv/bin/ale lint "$task"
  .venv/bin/ale validate "$task"
)
```

Each graded panel contains sixteen fresh physical courses. Every run reports actual native rows, FSQ costs, within-episode coordinate changes, joint acceleration, elapsed simulation, individual passage times, contact counts and stage outcomes. Physical successes remain visible at their raw denominator even while the anchor is unset. [Sampling and calibration protocol](scoring_protocol.md)
