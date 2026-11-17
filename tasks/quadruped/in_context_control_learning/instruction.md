# In-Context Control Learning: precision delivery

Teleoperate a Go1-shaped delivery robot through six sequential docking stops.
Learn its four-button remote and its motion from onboard images, then drive it
yourself, one button press per tool call. This is a stabilized planar base with
realistic Go1 visual meshes, NOT simulated articulated walking gait.

## Observation and Bay

Each reply includes one 384 x 384 RGB perspective image from a camera rigidly
mounted on the robot, 0.88 m above the floor and 0.12 m behind its centre. It looks
forward and 35 degrees down with a 100-degree vertical field of view. The view
turns and translates with the body. There is no overhead view, global pixel-to-map
conversion, map, compass, odometry, position, velocity, or numeric goal bearing.
Observe does not advance time, stop motion, or accumulate docking time.

The illuminated bay is a fenced 7.1 m square. Two parallel, 0.24 m thick partition
walls split it into three rooms. Each has a narrow opening 0.62-0.72 m wide and a
wide opening 1.0-1.25 m wide. Their locations vary by episode and remain fixed.
The robot's collision footprint is a disc of radius 0.16 m. The blue-grey walls
are solid. Contacts cause sliding or stopping, but do not damage the robot or
directly reduce the score. The camera and beacon can see over the low partitions;
seeing a destination does not imply a clear route to it.

The active dock has a blue 0.5 m square floor pad, a white direction arrow and
centre cross, and a small blue beacon 1.1 m above it. These are physical objects,
subject to perspective, field of view and occlusion. No marker is drawn through
an occluder or at a screen position computed from hidden coordinates.

## The Four Buttons

`robot_act` takes `{"action": [b], "frames": n}`. The four exact values of b are
0, 0.3333333333333333, 0.6666666666666666 and 1. Their assignment is unknown, but
each always has one of these meanings for the entire episode, including loading:

- Forward: apply forward drive, with a zero yaw-rate target.
- Turn left: apply counterclockwise steering, with a zero forward-drive target.
- Turn right: apply clockwise steering, with a zero forward-drive target.
- Brake: target zero forward and angular motion. Braking is not instantaneous.

There is no reverse or sideways command. Turning can rotate a stopped robot in
place; when moving it produces a curved, slipping path until forward motion
decays. Opposite steering first counters existing yaw momentum. Forward and brake
also take time to remove existing yaw. The base has physical mass and inertia,
bounded force and torque, and separate drive/steering response filters. A press
does not assign a displacement or instantly set a velocity. Equal presses from
different histories need not produce equal motion.

`frames` is an integer from 1 to 120, default 30. Each frame advances 0.01 s, so
presses last 0.01-1.2 s. Short presses permit fine corrections. Simulation pauses
between calls: thinking or observing does not coast or brake the robot. Adjacent
identical commands have the same physics as their combined duration, but every
call counts as a press.

## Terrain and Loading

The ordinary grey floor is dry. The broad turquoise band with dark stripes is
slippery: drive force, lateral grip and steering response are weaker, and actuator
lag is longer. The ochre striped band is rough: forward speed is lower, lateral
grip is stronger and steering response differs. Both bands are visible physical
floor surfaces spanning the bay, with fixed locations. Their effects depend on
the base centre, with a 0.04 m transition hysteresis on either side of the painted
boundary. Crossing back over the edge may briefly retain the preceding response.

Unloaded dry-floor forward speed is approximately 0.96-1.20 m/s; left/right turn
targets are independently 1.05-1.35 rad/s. Dry actuator time constants are
0.18-0.30 s for drive and 0.16-0.26 s for steering. Slippery terrain uses 90% of
the dry speed target, 43% drive traction, 60% yaw response and 1.45 times the
actuator lag; rough terrain uses 62%, 85%, 78% and 1.15 times respectively. Lateral
damping is 38 on dry, 7 on slippery and 48 on rough floor (N per m/s). Actual
transients also depend on inertia and force limits: drive force is capped at
22 N times terrain traction, and yaw torque at 1.8 Nm times terrain yaw response.

Advancing after the third delivery loads the robot. Its initial mass of 8 kg
and all planar inertias increase by the reciprocal of a fixed factor between
0.60 and 0.75. Forward speed targets decrease by that factor and turn-rate
targets by its square root. Loading does not teleport or rotate the robot, clear
its filters, change the assignment, reset velocity, or restore any budget.
`loaded` becomes true. These dynamics remain for deliveries four through six.

## Precision Docking

Only the active pad can score. Enter within 0.16 m of its centre and within
12 degrees of its arrow heading to arm docking. Once armed, a small hysteresis
margin applies: remain within 0.19 m and 16 degrees. Leaving either margin
disarms docking and resets the timer. While armed, translation speed must remain
at or below 0.045 m/s and angular speed at or below 0.08 rad/s for 0.50 continuous
simulated seconds. Exceeding either speed resets the timer. Conditions are checked
every physics frame, not just at the end of a press. A fly-through, wrong heading,
or brief stop is not a delivery. Brake presses advance time; observations do not.

The reply then reports `goal_reached: true`. Credit is permanent; the same pad
cannot score again. Call `robot_advance` to activate the next pad. Consecutive pads
are in different rooms and some legs cross both partitions. The sixth completed
dock ends the episode. Earlier successful docks retain their partial credit.

## Tools and Limits

- `robot_observe`: image and status without advancing physics.
- `robot_act`: one button held for the specified frames, then image and status.
- `robot_advance`: activate the next dock, only after reaching the current one.

Public status contains `frame_id`, `delivery`, `deliveries`, `delivered`,
`goal_reached`, `loaded`, `steps_remaining`, `requests_remaining` and `done`.
There is no reset. Limits remain 500 button presses, 560 total service requests
(including invalid requests), and 3 hours of wall-clock time. Longer presses do
not cost extra presses, but give less frequent feedback.

## Direct-Control Rules and Scoring

Drive using your own direct `robot_observe`, `robot_act` and `robot_advance` calls.
Do perception and planning in your reasoning from the images in this conversation.
Do not run commands or code, use scripts or other socket clients, write or edit
files, or start other agents. To-do lists and read-only file tools are allowed;
the private service, scene metadata and records are not participant-readable.

The grader audits the session against the private service ledger. A forbidden
tool or a service request not accounted for by your own direct robot tool calls
scores zero. Harness-rejected calls that never reached the service do not require
a ledger event. Otherwise your score is the fraction of the six completed docks.
Presses, requests and contacts carry no additional penalty.
