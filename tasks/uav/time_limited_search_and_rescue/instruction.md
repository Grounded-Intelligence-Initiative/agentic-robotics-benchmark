# UAV Search and Rescue

## Goal
Find and localize the orange human-shaped target among structures in a partially
observed arena within a simulated-time allowance. Search rather than receiving
the target's state. This first version covers detection and localization only,
not physical rescue, pickup or transport.

The UAV is altitude-stabilized. `robot_act` accepts `{"action":[vx,vy]}` in [-1,1],
mapping to planar velocity commands up to 1.6 m/s. There are 160 actions (32
simulated seconds). Buildings and a limited local camera footprint obstruct the
view. The side camera is blank; there is no global-map shortcut.

## Report and evaluation
Call `robot_report` with `{"frame_id":123,"point":[x,y]}`. Your cited frame must
really show the target and the location must be within 0.3 m. Up to three reports
are accepted. Success must occur before the action/simulated-time budget expires.
False reports, budget use and authoritative image evidence are recorded.
Direct control, a search program or mixed operation is allowed under the same
observation and interaction budget. Do not leave a controller for offline replay:
the actual protected interaction is the task outcome.

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
Target position and heading, equipment positions and dimensions; lighting and surrounding details. Rejection sampling keeps the target clear of opaque equipment roofs.
