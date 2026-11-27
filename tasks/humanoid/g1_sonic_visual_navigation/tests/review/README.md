# Reproducing visual navigation

The participant and verification environments use the original G1/Dex3 meshes and SONIC V1 encoder/decoder from the SHA-256 manifest. Both lamps and camera depth are rendered in the live MuJoCo scene. The lamps have no contact geometry. The optical camera is rigid on the pelvis. Mesh bounds entirely outside its frustum are culled for CPU rendering. Full robot geometry remains available to the external camera and the unmodified physical contact engine.

## Public cold run

From the benchmark repository root:

```sh
task=tasks/humanoid/g1_sonic_visual_navigation
docker build -t g1-visual-navigation "$task/environment"
docker run --rm --network none -v "$PWD/$task:/task:ro" g1-visual-navigation bash -ec '
  cd /task/environment/setup && bash setup.sh
  cp -R /task/solution /tmp/navigation-solution && chmod -R a+rX /tmp/navigation-solution
  runuser -u user -- bash /tmp/navigation-solution/solve.sh
  cd /home/user
  runuser -u user -- /opt/venv/bin/python practice.py --turn left --seed 7 --submission submission
  runuser -u user -- /opt/venv/bin/python practice.py --turn right --seed 7 --submission submission
'
docker run --rm --network none -v "$PWD/$task/tests/review:/review:ro" \
  g1-visual-navigation /opt/venv/bin/python /review/check_camera.py
```

The regression test compares two independently integrated robots at seed 7. Their future turns and final waypoints differ. Every camera/proprioception packet and every applied state in the shared prefix must agree, including after rendering extra optical alternatives in only one world. A displaced waypoint affects RGB and depth. A relit waypoint retains the original measured depth.

## Official verification

```sh
git submodule update --init vendor/ale
cd vendor/ale
VIRTUAL_ENV= uv run ale lint ../../tasks/humanoid/g1_sonic_visual_navigation
VIRTUAL_ENV= uv run ale validate ../../tasks/humanoid/g1_sonic_visual_navigation --runs-dir /tmp/ale-g1-visual-navigation
```

Read `scoring_protocol.md` for the hidden course sampling, raw physical score, visual response checks and predeclared calibration rule. The public hand-picked left and right practice examples share one initial physical state. The official reference and untouched controller run through the guarded subprocess boundary and fresh verifier entropy. The 16-course raw fraction includes every assigned course.

## Measurements

The first valid official calibration completed 16/16 physical courses, eight in each direction. All 16 signed-turn probes changed at least two coordinates. Seventeen of 20 waypoint probes did so, with zero changes under relighting or exact repetition. The 17,078 applied rows represent 341.56 simulated seconds and 1,092,992 FSQ coordinates. The registered rule sets full credit at 15/16 courses with both visual response checks satisfied. The null-anchor run itself returns reward zero by design. See [the measurement and raw receipts](anchor_measurement.json).

A separately frozen [native 16-start review](independent_native.json) completed 15/16 physical courses. One start lost the additional approach hold. Its 15 signed-turn and 22 waypoint checks all responded, with zero relighting or repeat changes. It used a separate JSONL solver and fresh roster. This secondary review did not select the numeric anchor.
