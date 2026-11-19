# UAV Search and Rescue: Interchangeable Rescue Heads

Recover an orange **RESCUE MANNEQUIN** and sled from an indoor training
wing and deliver it to the blue staging pad. This is a simulated
civilian exercise. The doors, turns, and rescue bay vary. There is no
floor plan: explore with the UAV's two local RGB cameras and keep track
of the passages and landmarks you actually see.

- Recenter visually at the four pink maintenance dots before crossing
  a doorway. The dots form the same **0.58 m square** at each station.
  Its center is a safe observation pose, not a waypoint you must visit.
  The dots have no room IDs. The takeoff hover can start slightly
  off-center, and the actual blue delivery pad is offset from the
  staging square. Use the pad itself for final alignment.
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
- Search the marked **RESCUE BAY** rooms for the orange sled. Its lifting
  fitting is a cyan circular flange under a dark cover. The cover looks
  the same on both sled types. Offset to the side with clear space, face
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
  far end and leads back down to staging. The sled cannot fit back
  through the inspection slot. Descend over the pad, settle at about
  0.43 m, open the gripper, and let the sled come to rest.

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
seconds; the allowance is **1,000 actions** (200 simulated seconds).
There is no reset. Observations do not advance time, but the live service
accepts at most 2,048 calls in total, including actions and observations.

## Cameras

`robot_observe()` and each action return two 256 × 256 RGB images:
`images[0]` looks straight down; `images[1]` looks forward and slightly
down by about 7°. Both rotate with the UAV. In the down view, right and up are body
right and forward. The down camera is 0.46 m forward and 0.145 m below
the UAV center, with a 94° perspective field of view. On a horizontal
surface `d` metres below that camera, a metre projects to
`(128 / tan(47°)) / d` pixels. The point under the gripper is centered
horizontally and 0.46 m down-image of the camera's ground center.
Neighboring stations are 1.58–1.90 m apart; the spacing is not uniform.
Apron paint is about 0.63 m from its station center. If the sled masks
a dot, turn while hovering or use another visible side of the square.
Use the forward view to inspect height, clearance and obstructions.

Only the images, `frame_id`, `done`, and `steps_remaining` are exposed.
There is no pose, depth, object list, map, or hidden identifier. Use the
tools or the public client at `/home/user/robot/client.py` with
`/opt/venv/bin/python3`. Scoring uses the protected live interaction.
Full credit is an intact, settled delivery after a physical lift; no
particular room, route, or map format is required.
