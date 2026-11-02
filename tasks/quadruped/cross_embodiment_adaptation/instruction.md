# Cross-Embodiment Adaptation

## Goal
First explore the environment with a small stabilized quadruped-shaped robot.
Then switch to a larger robot with different button responses and complete
navigation to the blue goal using the environment knowledge you retained.
Unknown wall geometry and narrow passages make a route usable by one body
potentially unusable by the other.

`robot_act` uses the same four-button encoding as described here: one-element
action list containing exactly 0, 0.3333333333333333, 0.6666666666666666, or 1.
Infer the permutation and response magnitudes from images. You see only a local
top-view neighborhood plus the goal beacon; the side view is intentionally blank.

After at least 12 exploratory actions call `robot_advance` once to switch bodies.
The shared allowance is 320 actions, including scouting, all new-body probes, and
final execution. Repeated switching/resetting is not permitted. Direct operation,
programmatic policies, and mixed approaches are allowed with identical observations
and budgets. No controller artifact is required: the protected interaction is scored.

## Scope and outcome
Success requires reaching the goal AFTER the switch. Reported metrics include
scout and post-switch action counts and contacts. This evaluates transfer of
environment knowledge, control response adaptation and body-clearance reasoning.
Leg stabilization and planar base motion are supplied; static leg geometry is
not a learned locomotion policy. Post-switch adaptation is budgeted, so this is
NOT strict zero-shot cross-embodiment control.
Each episode samples new wall positions and two differently sized openings. The map
is unchanged at the switch; the large robot cannot fit through the narrow opening.

## Interface and permissions

The protected service supplies images; no state vector, world source, scene seed,
map, or target answer is exposed. Use the `robot_observe`, `robot_act`,
`robot_advance`, and `robot_report` tools as applicable. Observations contain
base64 PNG `images`, a `frame_id`, `phase`, `done`, `completed_stages`,
`steps_remaining`, and `damage_remaining`. The MCP tools also display the images.
The red tracking marker is attached to your robot. Ground navigation goals and the arm tray are blue.
The live environment contains licensed nominal robot meshes and a detailed workcell,
not just reviewer-only decoration. Flight uses scaled Crazyflie geometry with the
stated benchmark dynamics; ground robots use scaled Go1 meshes with stabilized planar
control, not a learned walking policy. The arm is a three-axis industrial Cartesian
suction gantry, not a serial Panda manipulator. Visual meshes use simplified collision
proxies. These are simulated tasks, not hardware-validated digital twins.

Each valid action advances MuJoCo by 0.2 simulated seconds. This is not a
wall-clock speed contest. `observe` does not advance physics. At most 2048 service
requests are accepted. Reset is not available. Reconnecting, restarting your
client, or changing a local file does not restore action or damage budgets.
Only `advance` performs the specified one-way phase transitions. Public clients
are available at `/home/user/robot/client.py`; the MCP endpoint itself is immutable.
The environment is a root-owned process with a private authoritative ledger.

`images[0]` is the top view and `images[1]` is the side view. The camera output
is a calibrated 256-by-256 orthographic RGB view. Pixel center
`(u,v)` maps to top-view coordinates `x=(u+0.5)/32-4`,
`y=4-(v+0.5)/32` in metres. Side-view vertical coordinate is
`z=6-(v+0.5)/32`. Cameras are part of the instrument specification, not privileged
measurements. Colors are visual appearances, not object-state fields.
There is no internet dependency and no GPU requirement.
For tasks permitting code, run Python with `/opt/venv/bin/python3`, including in
`run.sh`. The harness may remove `/opt/venv/bin` from `PATH`; bare `python3` may
not have the required packages. In-Context Control Learning still forbids code execution.

Do not attempt to read protected files, modify the service, forge a ledger, or
recover a hidden seed. Agent-written reports of success have no grading authority.
Inspection/search may submit evidence from the final permitted action frame.
This does not advance time or reopen the exhausted movement budget.

## Scene variation
Wall openings, starting poses, stage goals, target-body footprint, button mapping and response gain; lighting and background details. The map persists across the body switch; the target body's mapping and gain change only once.
