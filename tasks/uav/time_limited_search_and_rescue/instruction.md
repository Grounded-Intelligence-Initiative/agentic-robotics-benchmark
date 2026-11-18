# UAV Search and Rescue

Evacuate the orange **RESCUE MANNEQUIN** and its sled from Sector B to the
blue evacuation pad in Sector A. This is a simulated civilian training
exercise. The drone starts hovering over the pad in A, facing B.

1. Survey **both** floor forecasts in A. The black-framed, four-tile board
   is straight ahead of the pad. The blue-framed, two-tile board is a little
   farther ahead and to the right. On each board, count the dark dots on
   the **teal-green tile**. The first board selects a horizontal lane; the
   second independently selects the altitude passage.
2. Fly through the first crossing below the overhead beam. The lane with
   the matching number of **white dots** on its floor plate is the only safe
   lane. Beyond it is a staging strip and then a full-width bulkhead. The
   second board's **one dot** means the lower-right opening; **two dots**
   means the upper-left opening. Matching floor plates on both sides of
   the bulkhead line up with the centers of the openings. Change altitude
   before entering the bulkhead and hold it until clear of the far plate.
   Neither crossing indicates its current safe choice locally. Wrong
   openings, baffles, the bulkhead, and cabinets end the sortie on contact.
3. Search Sector B, beyond the bulkhead. The target is an orange rectangular
   sled with a yellow dummy and a white grapple bar. Hover over the bar,
   descend, settle, then close the gripper. Raise the sled clear of the
   ground and take it back through **both** crossings in reverse, using
   the same safe passages. Keep the load high enough to clear the cabinets;
   lower it for the lower opening only after reaching that approach plate.
   At the blue pad, descend and settle again,
   open the gripper just above the pad, and let the sled come to rest.
   Releasing from height or while moving fast is unsafe.

## Flight and gripper

- `robot_observe()` returns the current two RGB images without advancing time.
- `robot_act({"action":[dx,dy,dz,grip]})` takes four numbers in `[-1,1]`.
  The first three increment the stabilized **position setpoint** by
  `[0.32*dx, 0.32*dy, 0.19*dz]` metres. Real motion follows with a short
  lag. Positive X is right, positive Y is forward, positive Z is up; the
  heading is fixed. Zero on an axis holds that setpoint. Allow a couple of
  hold actions for settling before grasp or release.
- Gripper: `+1` attempts to close on the bar, `-1` opens/releases,
  `0` leaves it alone. The command applies **before** the action's motion.
  Pickup needs alignment within about 13 cm and a slow hover; a UAV center
  altitude near 0.43 m works for both pickup and set-down. The drone starts
  at 1.12 m. Around 1.15 m works for yard transit and the first crossing,
  where the center must stay below 1.43 m. At the bulkhead, use the center
  of the opening and hold 0.75 m for the lower passage (safe center band
  0.57 to 0.92 m), or 2.20 m for the upper passage (1.98 to 2.42 m).
  These bands already allow for the carried sled.
- Each action is 0.2 simulated seconds; there are 184 actions (36.8 seconds).
  There is no reset, phase-advance command, or separate coordinates report.

## Cameras and reference points

Both cameras move with the UAV and return 256 × 256 RGB. `images[0]` is
straight down, with right = +X, up = +Y, and a fixed 3.5 m square footprint.
The point under the gripper is approximately pixel **(128, 142)**; align the
grapple bar and the pad center there. `images[1]` faces forward and tilts
toward the ground. No world map, position vector, target identifier, heat
state, or grip state is returned. The other fields are `frame_id`, `done`,
and `steps_remaining`.

The first board is about two metres ahead of the pad. The two barriers are
separated by roughly three metres; leave room to settle at the chosen
altitude before crossing. Use the downward image to center over the dotted
lead-in plates and the forward image to check clearance. The setpoint is
bounded to X = ±3.39 m, Y = -0.45 to +13.35 m relative to the pad, and
Z = 0.39 to 2.55 m above ground.

Use the tools directly or the Python client at `/home/user/robot/client.py`
with `/opt/venv/bin/python3`. The protected live interaction is what counts.
Partial progress is credited for surveying each board, crossing each
barrier outbound, getting a view of the target, lifting, and returning
through each barrier with the load. Full credit requires a stationary
delivery on the pad.
