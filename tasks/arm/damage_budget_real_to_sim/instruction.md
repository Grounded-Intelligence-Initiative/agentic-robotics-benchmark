# Damage-Budget Real-to-Sim-to-Real

## Goal
Commission a replacement planar drive and identify a long green cartridge using
only the workcell cameras. Reconstruct the explored drive, cartridge and arm in
MuJoCo. Deliver a controller that
extracts the cartridge from its shipping sleeve, lifts it clear of the
obstacle, carries it into the slotted receiving fixture,
and releases it upright on both blue pads of a balanced receiving cradle,
clear of the captive packing stop.
The stop is a second moving body, not a button. The fixture has physical lower
supports and overhead clearance rails; placing the cartridge near the fixture
or hovering over it is not a transfer. This is hidden simulation -> reconstructed
simulation -> hidden simulation, not a physical-world deployment.

## Equipment manual
The replacement cartridges have the same external shape and registration marks.
Inside, a compact ballast block can be on either side of the shell center; its
location is fixed for an episode. Independently of which side holds the ballast,
the cartridge can be light (0.80-0.94 kg, thin cap) or heavy
(1.43-1.55 kg, reinforced cap). Both cap variants look the same. The horizontal
center-of-mass offset is along the cartridge's long axis, with absolute magnitude
0.315-0.410 m. The nominal central shell is 0.26 kg; the remaining mass is an
8 cm cube. Neither the mass nor the offset is telemetered. The cartridge starts
aligned with the world X axis. Its collision box is 1.38 x 0.34 x 0.16 m
(length x width x height); the identical green marks are 0.57 m either side
of center. A pink 1.00 m painted line on the side face joins their axis. A
visible end and the line can be used to measure pitch if the other end is
behind a fixture. The markings are the same on all cartridges.

At the source, two ochre overhead cheeks cover the ends of the cartridge.
The turquoise back rail can be on either Y side; the ochre cheeks extend
from it toward the open end. The cartridge initially lies centered in X,
0.255 m from the rail center toward that opening.
The cheeks' underside is at Z = 0.205 m. Their central slot is 0.350-0.396 m
wide, slightly offset in X (at most 0.031 m). They project 0.22-0.34 m
forward of the initial shell center. Sight windows between the rear uprights
and overhead fingers expose the shell's end marks to the fixed side camera.
The shell rests on a 0.006 m-high low-friction sheet (sliding coefficient
0.14) under an L-shaped guide. The draw direction is away from the turquoise
back rail. A dark blue cantilever retains the trailing end at a 0.195 m
underside height. The long side wall opens into a transverse passage; the
violet front fence stops a straight draw. A second violet face closes the
far end of the pocket on either the left or the right. These are contact
surfaces, not indicator lights.

The bend and lateral displacement of the pocket vary independently (1.05-1.35 m
along Y and 0.34-0.54 m along X). A centered shell has 0.09 m working clearance
from each inner violet face; the fences are 0.06 m thick. The slot, pocket and
stops are visible. Grips may be changed. Nothing unlocks in software.

The X/Y slides have two pretensioned timing belts and a replaceable enclosed
two-motor cassette. Belt A spans the sum of the slide displacements, X+Y;
belt B spans the difference, X-Y. The cassette fits opposite ends of the belt
loops. This reverses both windings together (installation sign s = +1 or -1).
Each output pulley independently has 16 or 24 teeth; the motor encoder is
scaled for a nominal 20-tooth pulley. Write rA and rB for the actual tooth
counts divided by 20. Motor position coordinates, in nominal belt metres, are
`A=s*(X+Y)/rA` and `B=s*(X-Y)/rB`. Belt tension acts back through this same
transmission. There is no slip or backlash. The cover conceals which parts
are installed; there is no readback. The upper red band is rigid on the
carriage, centered 0.052 m above the slide origin; its height is 0.032 m.
The cup below it pivots. The area around the home pose at its initial height is clear for
short, vented planar jogs. The drive stays fixed within a session; subsequent
sessions may have different installations and pulleys.

The cup rides on a passive Y-axis hinge with 13 N m/rad spring stiffness,
0.72 N m s/rad damping, inertia augmentation 0.003 kg m^2, and +/-24 degree
mechanical travel. Its radius is 0.08 m and its half-height is 0.03 m.
The translating carriage mass is 0.47 kg, the cup mass 0.04 kg. The carriage has
gravity compensation (5.0031 N upward) and 12 N s/m slide damping per axis.
The two diagonal modes (unit directions along X+Y and X-Y) have independent
stiffnesses K of 55-175 N/m. With the belt and gear convention above each
motor's MJCF position gain is K*r^2/2. The direct Z gain is 60-88 N/m.

When level, the 1.58 m-long blue deck is 0.40 m high. The two pads
are then 0.43 m high, with centers 0.46 m to either side along X.
The clearance rails start at 0.714 m and have a 0.22 m gap between them for the
tool stem. The obstacle is 0.28 x 0.44 m with its top at 0.57 m. Scene positions
vary; the cameras show their placement.

A transverse packing bolt starts across the receiving pocket, between the
supports. Its magenta lug is on either the +Y or -Y side, and its X station
varies along the fixture. The 0.44 kg bolt slides along Y through the visible
bearing; it has 0.73 m of outward travel, modest sliding resistance, and no
return spring. The inboard tip starts 0.30 m across the fixture centerline.
The lug center starts 0.82 m outboard, at Z = 0.701 m. Its working face is
0.21 m wide and 0.17 m tall. The vented cup transmits normal contact force.
There is no required order, stroke length, marked parking coordinate, or
additional cost for touching the hardware. The released cartridge must be stable on both
pads without resting against the bolt, and the bolt must not be moving.

The blue deck and both pads ride on a single spring-return weighbeam.
The Y-axis pivot at deck center is 0.33 m above the bench. Its torsional
spring is 58-68 N m/rad with 8 N m s/rad damping and mechanical stops at
+/-14 degrees. An enclosed tare produces an unknown signed mass moment
of 1.21-1.37 kg m, independent of the cartridge. The two yellow front
flags are 1.46 m apart on the same beam: their side-view slope measures
its actual tilt. An orange index on the sliding counterweight is 0.258 m
in front (-Y) of its fork. The fork is 1.87 m in front of the deck center.
The 3.25-3.55 kg rider moves +/-0.71 m along the beam. Its passive friction
shoe holds about 9 N. There is no motor or locking action.

The rider's two fork jaws transmit contact force. The cartridge adds its
own gravity moment when the pads bear its weight. Finish with the shell
released, both pads carrying load, the tool not propping the cradle, and
the beam and rider settled. The pad centers must differ by no more than
0.065 m in height for one second; no particular rider setting is required.

## Control and damage
`robot_act` takes `{"action":[dA,dB,dz,vacuum]}` in [-1,1]. The live robot,
controller verifier, and public model rollout snap a finite numeric component
to the endpoint only if its overshoot is at most `1e-12`. Larger overshoots,
booleans, nonfinite values, and malformed actions are rejected. A unit increment
is 0.14 nominal belt metres on A or B, or 0.14 world metres on Z. The
corresponding world Cartesian target is clipped to
[-2.6, 2.6] m in X/Y and [0.17, 1.50] m in Z; the next increment starts
from that clipped target. Each action advances physics by
0.2 simulated seconds. Positive vacuum requests a new pressure each action;
zero or negative vents. The cup seals only below 0.19 m/s relative speed, with
the surface normals within about 5 degrees, and its center over the flat face.
In cartridge coordinates, the allowed cup-center region is |X| <= 0.59 m,
|Y| <= 0.087 m, and Z = 0.109-0.150 m
above the shell center. The seal is modeled as a compliant weld at the contact
patch, not a detailed simulation of a flexible rubber cup.

Full vacuum provides 20 N of normal capacity. The seal opens when
`hypot(tension, shear / 0.85)` exceeds the requested capacity; an open seal alone
does not incur damage. The thin cap fractures above 13 N of requested capacity
while sealed; the reinforced cap is rated to 20 N. Even the lightest heavy
cartridge weighs more than 13 N, so there is no common setting for unsupported
transfer. Either shell fractures if a contact load exceeds 65 N. The first contact above 29 N leaves a
permanent scuff costing 5 damage units but does not prevent completing a transfer.
A fracture costs another 20 units and ends the episode. Simply moving or
experimenting safely does not consume damage. There are 1200 actions per episode,
including source extraction and adjustment of the receiver;
the 40-unit damage budget is shared across exploration and evaluation.

The cartridge parameters are not revealed after exploration. Use the supplied
equipment ratings and camera observations to choose any safe identification
procedure; the evaluation does not prescribe a set of probing actions.

## Deliverables
In `/home/user/submission/` leave:
1. `run.sh`: your image-based controller using the JSON-lines protocol below.
2. `scene.xml`: a runnable MuJoCo reconstruction of the **explored** cartridge,
   arm and cleared receiving fixture in the diagnostic setup described below.
3. `predict.py`: read `{"actions":[[dA,dB,dz,vacuum],...]}` on stdin and output
   `{"positions":[[arm_x,arm_y,arm_z,object_x,object_y,object_z,object_pitch],...]}`.
   Predict pose **after** each action from the initial scene. Coordinates are
   in metres and pitch is in radians, positive when the object's +X end slopes
   down. No images accompany the sequences.

In MJCF, name the moving carriage `agent`, the hinged child `tool`, and the
cartridge `object` with collision geom `fragile`. The first three joints are
the world X/Y/Z slides, then the Y-axis tool hinge, then the free object.
The first two position actuators drive fixed tendons on the X and Y joints:
coefficients (1,1) for A and (1,-1) for B, with actuator gears s/rA and s/rB.
The third is a unit-gear Z joint position actuator. The public driver reads
the actual MJCF transmission, including gear. The first equality is an initially inactive weld
from `tool` to `object`. Use a 0.01 s timestep. Supply positive scalar custom
numerics `cup_capacity_n`, `cap_proof_n`, `impact_limit_n`, `scuff_limit_n`.
The driver initializes the carriage to (-1.9,-1.7,1); encode the cartridge's
initial pose and inertial properties in MJCF.
`/home/user/robot/model_rollout.py` and `arm_dynamics.py` contain the exact
public action driver and seal interface, with no hidden scene values.
Run the driver from your submission directory with
`/opt/venv/bin/python3 /home/user/robot/model_rollout.py < actions.json`.

Print `READY` within 40 seconds. Read one JSON request per line and answer within
10 seconds: `{"type":"reset"}` -> `{"ok":true}`;
`{"type":"act","t":0,"obs":{...}}` -> `{"action":[dA,dB,dz,vacuum]}`;
`{"type":"close"}` -> `{"ok":true}`. Diagnostics go to stderr. The live service
is unavailable during verification. A step index `t` may arrive twice with
different images; only the action for the real image is applied. Do not advance
your history twice. A shared integrity check expects distinct actions on at
least half the sampled image perturbations. Aborted transfers count as zero.

All submitted files and assets must be inside the frozen, read-only submission
directory; code runs at the agent UID. Use relative XML asset paths without
absolute paths or `..`. Symlinks, special files, non-agent-owned entries,
more than 4096 entries, depth >16, or total size over 128 MiB are not accepted.
Put temporary files in `/tmp`. Use `/opt/venv/bin/python3` in `run.sh`.

## Evaluation
Four fresh, withheld action sequences on the explored scene test off-center
loaded dynamics, pitch, support contact, placement, and a near-balance lift
at a common low vacuum setting. For these cartridge diagnostics only, the
packing stop starts retracted and the detachable source sleeve and slip sheet
are removed and the nominal receiver is bolted level, without the movable
weighbeam or rider. The cartridge is at its **original reset X/Y**, resting on the
bare bench at center Z = 0.080 m, not at the later handoff position. Omit
the shipping hardware from this MJCF. Live exploration and all transfers
start packaged, with the receiving stop engaged.
The eight hidden transfers include all eight drive cassettes (both windings
and both pulley sizes per channel), independently paired with the eight
draw-direction, lateral-pocket and tare-side combinations and the eight
ballast, cartridge and bolt families. Each reconstruction sequence has a
20-second prediction limit. The verifier runs both your predictor and your MJCF
with the immutable action driver. Eight new independent sessions test your image
controller: one of each ballast-side, cartridge-class and stop-side combination,
in an undisclosed order. None reuses
the exploration scene. Physical transfer success and cumulative damage are
recorded separately. The raw metric is
`success_rate * exp(-max(prediction_RMSE, scene_rollout_RMSE)/0.15) * max(0,1-cumulative_damage/40)`.
RMSE weights pitch by the shell half-length (0.69 m/rad) to put the angle error
on an end-height length scale. You have 30 minutes for the agent phase.

## Cameras and tools
Use `robot_observe` and `robot_act`. `/home/user/robot/client.py` supplies a Python
client. No reset is available and reconnecting restores neither budget.
Observing doesn't advance physics. The service caps requests at 2048; the
protected ledger is authoritative. Do not access protected files, recover seeds,
modify the service, or forge evidence.

Observations contain two base64 PNG `images`, `frame_id`, `phase`, `done`,
`completed_stages`, `steps_remaining`, and `damage_remaining`, with no state
vector. Cameras are calibrated 256-by-256 RGB orthographic views with seeded
read noise of at most two levels per channel. In the overhead view, pixel center
(u,v) maps to `x=(u+0.5)/32-4`, `y=4-(v+0.5)/32` metres. The side view uses the
same X calibration and `z=6-(v+0.5)/32`. Red marks the tool, green the cartridge
ends, and blue the receiving pads. These are physical surfaces. CPU Filament
renders the existing industrial scans and gantry; no network or GPU is required.
