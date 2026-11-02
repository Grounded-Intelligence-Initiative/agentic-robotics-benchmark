# UAV Bridge Inspection

## Goal
Inspect a bridge from a UAV and locate its missing or damaged deck segment. Fly to
acquire a close visual observation and submit its position with image evidence.
The first version uses one missing deck patch, not semantic recognition of subtle
material fatigue. Most of the bridge is hidden outside your local camera view.

The UAV is altitude-stabilized. `robot_act` accepts `{"action":[vx,vy]}` in [-1,1],
mapping to planar velocity commands up to 1.6 m/s. There are 160 actions (32
simulated seconds). Only the local top camera is informative.

## Report and evaluation
Call `robot_report` with `{"frame_id":123,"point":[x,y]}`. The frame must have
actually been obtained during this episode and show the damage from nearby.
Do not submit an invented screenshot or a list of guesses. At most three reports
are permitted. Localization must be within 0.3 m of the true damaged region.
The server verifies the frame and position and records false reports and actions.
The image evidence is preserved by the trusted service.
Direct operation, a search program, or a mixed approach are equally allowed.
There is no required controller architecture or separate artifact submission.

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
Damage position, gap width, bridge lateral offset and deck width; lighting and surrounding details. The defect remains physically visible and reachable within the inspection footprint.
