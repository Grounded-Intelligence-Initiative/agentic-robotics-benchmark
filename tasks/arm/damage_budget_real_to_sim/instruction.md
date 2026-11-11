# Damage-Budget Real-to-Sim-to-Real

## Goal
Explore a tabletop scene cautiously using a Cartesian suction arm. Infer the scene
from images, reconstruct a simulator, and develop a controller that lifts the green
fragile object around the gray obstacle into the blue tray and releases it.
This first version is hidden simulation -> reconstructed simulation -> hidden
simulation, NOT physical-world deployment. The arm has stabilized Cartesian
position actuators, not a learned six-axis manipulator. Their three response gains
are unknown, independently sampled in [35,230] N/m, and fixed within the scene.
The initial image does not reveal those gains. Small free-space motions can identify
the response before touching the fragile object. Estimate an executable model from
your image sequences; the exact scene gains are never exposed as metadata.

`robot_act` accepts `{"action":[dx,dy,dz,grip]}` in [-1,1]. Translation components
increment the position command by up to 0.14 m per action. `grip>0.5` attempts
suction attachment nearby; `grip<-0.5` releases. Your exploration budget is 180
actions. Fragile-object impacts above 50 N break the object and cost 20 units;
the cumulative allowance is 40 units across exploration and evaluation. A broken
object cannot complete a transfer. Do not drop it or push it into an obstacle.

## Deliverables
Leave these files in `/home/user/submission/`:
1. `run.sh`: controller using the READY/reset/act/close JSON-lines protocol.
2. `scene.xml`: a runnable MuJoCo reconstruction of the observed scene.
3. `predict.py`: read `{"actions":[[dx,dy,dz,grip],...]}` on stdin and print
   `{"positions":[[arm_x,arm_y,arm_z,object_x,object_y,object_z],...]}`.
   Predict the state AFTER each action, starting from the original scene.
   Load your reconstructed scene, not an environment service. No images or states
   are provided with these held-out action sequences.

The reconstruction must contain bodies named `agent` (the moving arm tip) and
`object` (the fragile object), at least three geoms and at least six generalized
position coordinates. The arm's x-, y- and z-axis slide joints occupy the first
three position/velocity coordinates, use the world axes and have three position
actuators in that order. The first equality is a suction weld from `agent` to
`object`. The object starts at the pose encoded in your XML.
`/home/user/robot/model_rollout.py` is the exact public action driver used to test
your XML. Inspect it for initialization, command integration and suction semantics;
it contains no hidden scene parameters. Test it from your submission directory with
`/opt/venv/bin/python3 /home/user/robot/model_rollout.py < actions.json`.

The controller must print `READY`, then read one JSON request per line:
`{"type":"reset"}`, `{"type":"act","t":0,"obs":{...}}`, or `{"type":"close"}`.
Reply respectively with `{"ok":true}`, `{"action":[dx,dy,dz,grip]}`, and
`{"ok":true}`. During verification, use the supplied `obs` images; the exploration
service is no longer available. Write diagnostics only to stderr.
Print `READY` within 40 seconds and reply within 10 seconds per request. A failed
episode receives zero and remains in the average; crashed processes may restart
up to three times. Reconstruction predictions have a 20-second limit per sequence.
The grader freezes your submission before running it. All required files must be
inside `submission/`, which becomes the read-only working directory. Put temporary
files in `/tmp`. Symlinks, special files, non-agent-owned files, more than 4096
entries, nesting beyond 16 levels, or more than 128 MiB are not accepted. Resolve
`scene.xml` and your other assets relative to the working directory, not by an
absolute `/home/user/submission` path.
All XML includes and asset paths must be relative paths within the submission;
absolute paths and `..` path segments are rejected.

## Evaluation
Three withheld excitation sequences test reconstruction, including pickup and
transport; returning a constant object pose is not sufficient. Four rollouts
evaluate the controller: a return to the explored scene and three unseen scene
initializations, all using image observations. Raw reconstruction
RMSE, transfer success and cumulative damage are recorded separately. The verifier
also runs your `scene.xml` with its immutable copy of the public action driver.
An unrelated XML cannot be rescued by a predictor using some other model.
The primary metric is success_rate * exp(-max(prediction_RMSE, scene_rollout_RMSE)/0.15) *
max(0,1-cumulative_damage/40). Reconstruction is checked as an executable artifact;
all submitted XML and code run at the agent UID, never as root.

## Interface and permissions

The protected service supplies images; no state vector, world source, scene seed,
map, or target answer is exposed. Use the `robot_observe`, `robot_act`,
`robot_advance`, and `robot_report` tools as applicable. Observations contain
base64 PNG `images`, a `frame_id`, `phase`, `done`, `completed_stages`,
`steps_remaining`, and `damage_remaining`. The MCP tools also display the images.
The red tracking marker is attached to your robot. Ground navigation goals and the arm tray are blue.
The live environment uses CPU Filament PBR rendering, CC0 industrial scans, normal
and roughness maps, and nominal robot meshes in a detailed workcell,
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
Object, tray and obstacle positions; three independent hidden actuator gains; lighting and workcell details. Gains remain fixed throughout exploration and reconstruction of that scene.

## Repeated observations
The verifier may ask the SAME step index `t` more than once with different images,
in a hidden order. Only the action for the actual observation is executed. Do not
advance internal time or blindly integrate a motion command on every request.
Maintain state keyed by `t`, or handle candidate histories consistently. A shared
observation-dependence check requires different actions for at least half of the
sampled image perturbations; saturation on individual states is allowed. This is
a protocol integrity check, not evidence of frontier difficulty.
