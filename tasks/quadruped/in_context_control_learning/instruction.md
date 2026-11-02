# In-Context Control Learning

## Goal and rules
You are a frozen model learning the effects of four unknown buttons by looking
at the camera and acting within ONE conversation. Complete four navigation stages.
Retain your observations and discoveries in context. The button mapping is fixed
throughout the episode, but goals change after each successful stage.

Use ONLY direct `robot_observe`, `robot_act`, and `robot_advance` tool calls.
Do not write or run a controller, call a terminal, delegate to another agent, or
train/change model parameters. No program submission is requested or evaluated.
The verifier matches the engine's trusted tool-call trajectory against the
protected environment ledger; self-reported compliance does not count. A run
without an auditable trajectory is not a valid In-Context result.

`robot_act` accepts `{"action":[button]}` where button is exactly 0, 1/3, 2/3, or 1
(use 0.3333333333333333 and 0.6666666666666666). Discover their effects from images.
The quadruped-shaped body has stabilization and planar motion provided. You are
learning button semantics and navigation, not learning to stand or walk from scratch.
Only a local neighborhood and the goal beacon are visible. Walls and the narrow
and wide openings are randomized between episodes, but persist between stages.
Context can therefore retain both button semantics and a map of previously seen areas.

## Progress and evaluation
After the blue goal is reached, call `robot_advance` for the next stage. This
does not reset the shared 500-action (100 simulated second) budget. The environment
returns to a new starting pose; previously learned button semantics still apply.
The outcome is completion of all four stages. Stage completion, contacts and
action counts are also recorded. A scripted oracle validates physics/interface
feasibility ONLY; it is explicitly marked and is not evidence of LLM learning.

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
Button mapping, response gain, wall openings, starting poses and stage goals; lighting and background details. The mapping and map persist across all four stages.
