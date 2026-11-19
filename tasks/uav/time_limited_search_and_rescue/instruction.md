# UAV Search and Rescue: Medical Litter Transfer

Recover the **RESCUE MANNEQUIN** on its full-length medical litter from
an indoor training wing and set it down on the blue center of the
receiving mat beyond the loading pocket. This is a simulated
civilian exercise. The doors, turns, and rescue bay vary. There is no
floor plan: explore with the UAV's two local RGB cameras and keep track
of the passages and landmarks you actually see.

- Fly by the architecture. Stone grain and grout seams are fixed to the
  floor, and timber skirting follows real walls to the jambs. Keep some
  floor in successive down views to measure motion. On a return trip,
  compare what you see with an earlier view of the same doorway or floor
  patch. There are no position markers or room IDs. Takeoff can be
  slightly off-center. Use the actual receiving mat for final alignment.
- Find the **INSPECTION** slot off the staging strip. Align on the
  violet floor apron, settle at about **0.71 m** altitude, and fly
  through empty. Once clear, climb before leaving that pocket.
- Reserve time to search the mezzanine. Look into unfamiliar openings
  before committing. White aprons are ordinary doorways; amber aprons
  mark a change of floor with a raised sill. The lower floor is the
  launch datum; the mezzanine floor is **0.60 m higher**. A useful cruise
  altitude is **1.30 m above the local floor**. For an amber connection,
  climb to **1.90 m above launch** before entering, hold until fully
  clear on the other side, then adjust to the local floor. The walls
  are higher than the flight envelope.
- Search the marked **RESCUE BAY** rooms for the orange pickup panel on
  the green litter board. Its lifting
  fitting is a cyan circular flange under a dark cover. The cover looks
  the same on both fitting types; the green centering button on top is
  common to both. Offset to the side with clear space, face
  the sled and lower the UAV to see under the cover with the forward
  camera. From roughly half a metre to the side at a 0.43 m hover, the
  narrow flange is about 8 cm across and the wide one about 19 cm.
  Remember the profile and the route back to the sled.
- Return empty through **INSPECTION** to the **TOOL DEPOT** at the
  right-hand end of staging (as seen from launch facing the wing).
  The rack holds two cyan collet heads. Choose by geometry: the small
  head has a smaller opening and fits the narrow flange; the large
  head fits the wide flange. Rack positions can vary. Center directly
  over the chosen head, settle at **0.43 m above staging** and close
  to mount it. The head lifts with the aircraft and its stand is left
  empty. One head can be carried at a time; to swap, return it to its
  own stand at the same hover height and release before taking the other.
- Fly back through **INSPECTION** with the head and find the sled again.
  Use the down camera to center over the fitting, settle at **0.43 m
  above the local floor**, then close and lift. The jaws close from
  the sides, under the cover. A mismatched head will not latch: the
  wide flange cannot enter the small seat, and the narrow flange slips
  past the large head's retaining lip. Check that the sled lifted before
  committing to egress. A miss leaves the sled on the floor.
- Lifting the load opens the blue **CARGO** shutter and closes an
  interior safety shutter. A closed safety shutter has a red apron
  and a visible red panel. Recheck doors on the return and route around
  the obstruction. The cargo run connects to the search rooms at the
  far end and leads back down to staging. The litter cannot fit back
  through the inspection slot.

## Loaded Transport

The rigid board is **1.80 m long and 0.44 m wide**. The violet end grips
are 1.62 m apart. Center the down camera over the board to see both.
The clamp holds the relative pose at the instant of pickup: the long
axis need not point along the UAV nose. Yaw turns the entire load.
You can translate sideways or backwards without changing that yaw.
Carrying the board across the UAV rather than under its camera boom
gives a clearer down view. The board can obscure the floor; changing
the viewing angle exposes different patches and wall junctions.

Across staging from the cargo exit, **LITTER TRANSFER** leads to a finite
turning pocket. Beyond it, a perpendicular throat leads to **RECEIVING**.
The two openings are offset; inspect the jambs to see which way and how far.
Each clear gap is 1.10 m wide and 0.56 m deep. Lime stripes follow the inner
jamb faces and extend 1.24 m onto the approach. Use their midpoint for the
load centerline. Turn and translate where both ends have room; clearing
the aircraft does not mean the trailing end is through. The pocket
is 4.20 m across by 4.60 m deep.
There is no prescribed heading or checkpoint maneuver. The walls, jambs
and litter occupy the volumes you see. Bring the fitting over the blue
center of the 2.50 by 1.48 m green receiving mat, settle at about 0.43 m,
open the gripper and let the load come to rest with the entire board over the mat.

## Controls

`robot_act({"action":[right, forward, up, yaw_left, grip]})` takes five
numbers in `[-1,1]`. The nominal increments of the stabilized setpoint are up to
**0.32 m** on each horizontal axis, **0.19 m** vertically, and **30°**
in yaw per action. Positive yaw turns left; a zero holds that axis.
Horizontal increments are relative to the **heading setpoint at the
start of the action**. Horizontal travel can differ by a few percent
from the nominal distance, and forward motion can couple sideways by
about 0.13 m per metre when loaded (less when empty). Do not assume
command totals are pose. Zero horizontal commands settle to a
stationary hover: stop, correct using a fresh image, and look again.
The UAV starts at 1.30 m with its heading facing the wing.
Altitude setpoints are limited to 0.39–2.09 m above launch.

`grip=+1` attempts a close, `-1` releases, and `0` leaves it alone.
It applies before movement. Steady the UAV before mounting, latching
or releasing. Hover directly over the rack head to mount it (the rack
guide has about 8 cm of horizontal capture range). At the sled, the
self-centering jaws accept about 13 cm of horizontal misalignment.
Releasing the sled leaves the head attached to the UAV.
High or fast release, or striking a wall, sill, or shutter with the
airframe **or load**, ends the sortie. Each action advances 0.2 simulated
seconds; the allowance is **2,400 actions** (480 simulated seconds).
There is no reset. Observations do not advance time, but the live service
accepts at most 5,000 calls in total, including actions and observations.

## Cameras

`robot_observe()` and each action return two 256 × 256 RGB images:
`images[0]` looks straight down; `images[1]` looks forward and slightly
down by about 7°. Both rotate with the UAV. In the down view, right and up are body
right and forward. The down camera is 0.46 m forward and 0.145 m below
the UAV center, with a 94° perspective field of view. On a horizontal
surface `d` metres below that camera, a metre projects to
`(128 / tan(47°)) / d` pixels. The point under the gripper is centered
horizontally and 0.46 m down-image of the camera's ground center.
Neighboring wing rooms are 2.60–2.80 m apart; the spacing is not uniform.
The partitions are about 15 cm thick, with skirting less than 1 cm proud of the wall.
Doorway wear plates extend from the openings onto the approach. Distance
to a wall can be measured where its skirting meets the floor. At a constant
height and heading, floor features move opposite the camera's translation;
do not mistake the attached load or a raised sill for the floor. Use the
forward view to inspect height, clearance and obstructions.

Only the images, `frame_id`, `done`, and `steps_remaining` are exposed.
There is no pose, depth, object list, map, or hidden identifier. Use the
tools or the public client at `/home/user/robot/client.py` with
`/opt/venv/bin/python3`. Scoring uses the protected live interaction.
Full credit is an intact, settled delivery after a physical lift; no
particular room, route, or map format is required.
