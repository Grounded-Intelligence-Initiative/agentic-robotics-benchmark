# Site brief: walkway W-3, pump hall basement

The client's inspection robot has to cross walkway W-3 of the pump hall basement on its own: 3.0 m from the
start mark to the goal line, in at most 60 s, carrying the client's inspection pod (0.9 kg, a 0.14 x 0.10 x 0.06 m
box bolted on top of the body module, centred). Every section of the walkway is built the same way, but no two
sections are the same: the robot will be judged on sections nobody has measured.

## The walkway

Seen from the start, along the walking direction, every section has: flat floor to the foot of a first flight of
steps up; a landing; a ramp down; a short stretch of floor; a second, shorter flight of steps up; floor to the goal
and beyond. Loose flat blocks of rubble (4 to 12 cm on a side, 1 to 3.5 cm high) lie on the landing and on
the floor after the last steps. Pipes (6 cm diameter) cross the walkway overhead in some sections. The walkway is
1.0 m wide between two walls; the robot starts on the centre line, facing the goal.

## The survey

The survey team measured four sections (distances from the start mark along the centre line; a step's rise is its
height above the ground before it; a pipe's clearance is the height of its underside above the highest ground
within 25 cm of it):

| section | steps up: position (m) and rise (cm) | ramp down | overhead pipes: position (m) and clearance (cm) |
|---|---|---|---|
| S1 | 0.55: 9.7, 0.82: 7.3, 1.16: 8.2; then 2.28: 7.1, 2.64: 4.0 | at 1.77 m, 13.5 deg, 0.33 m long | 1.33: 41 |
| S2 | 0.60: 7.4, 0.86: 10.1; then 2.12: 4.7 | at 1.55 m, 12.8 deg, 0.33 m long | 1.09: 38, 2.50: 34 |
| S3 | 0.51: 6.9, 0.82: 6.3, 1.09: 9.5; then 2.33: 5.8, 2.69: 11.8 | at 1.81 m, 9.9 deg, 0.36 m long | 1.11: 39, 1.46: 32 |
| S4 | 0.52: 11.8, 0.86: 6.0, 1.20: 6.6; then 2.43: 8.7, 2.76: 11.2 | at 1.90 m, 11.1 deg, 0.35 m long | 1.48: 35, 1.58: 33 |

The team's note: "The unmeasured sections look like these four. We did not see a rise or a clearance outside what
we measured here by more than a centimetre or two, but we only measured four."

## What counts

- **Progress**: how far the centre of the body module gets along the walkway, as a fraction of the 3.0 m
  (1.0 at the goal line), within 60 s.
- **A fall ends the run** and keeps the best progress made at least 0.5 s before the fall. A fall is: the body
  module or the pod touching anything (ground, step, wall, pipe); any leg part (actuator or link) touching a pipe;
  the body tilting more than 50 degrees; the underside of the body module coming within 2.5 cm of the ground below
  it (measured straight down from its centre and its four bottom corners).
- **A fault scores the run 0**: the simulation diverging, the body moving faster than 4 m/s, the body more than a
  leg reach plus 25 cm above the ground, or the contact solver overflowing.
- A design that is not buildable from the catalog, or breaks a limit, is not built: it scores 0 everywhere.

## How the robot is simulated (the grader's and the practice walkway's model)

- MuJoCo 3.16, time step 2 ms, implicit-fast integrator; control at 50 Hz. The gait's joint targets computed from
  the sensors of one control period are applied in the next period (one period of latency).
- Every part is the catalog's: the body module a box, actuators cylinders (abduction and hip actuators lie between
  their mount and their flange; a knee actuator is centred on its axis), link segments capsules, feet spheres, each
  with the catalog mass spread uniformly over its shape; the pod is a 0.9 kg box on top of the body. No other
  parameter of the robot exists, and none can be set from robot.xml.
- Contacts: only the **feet** touch the ground (friction: the foot's, from the catalog). Leg links and actuators pass
  through the ground and steps but **do hit pipes** (a fall). The body module and the pod touch everything (a fall).
- Actuators: each joint has a servo `tau = kp (q_target - q) + kd (qd_target - qd)` (kp, kd from the catalog; the
  target is interpolated over the control period), clipped to the actuator's torque, which falls linearly to zero
  at its rated speed in the direction of motion (torque-speed line); joint ranges from the catalog; each actuator
  adds its rotor inertia (armature).
- Sensors: joint angles (noise 2 mrad), IMU gravity direction and angular rate (small noise), the terrain scanner
  (the ground height wherever the gait asks; it does not see pipes).

## Your tools

- `~/qm24/catalog.json`: the parts, their prices, masses and ratings, and the client's limits.
- `~/qm24/qm_design.py`: the robot.xml dialect (read its docstring) and the build check; `~/qm24/check_design.py`
  runs it on a file and prints the design's figures and the gait sizes it gets.
- `~/qm24/qm_gait.py`: the standard gait, exactly as the robot runs it.
- `~/qm24/example_robot.xml`: a buildable (not a good) design, to show the dialect.
- `~/practice/run_practice.py`: one run of a design over a practice section of the walkway, with a video; 4 runs
  for the whole session.
