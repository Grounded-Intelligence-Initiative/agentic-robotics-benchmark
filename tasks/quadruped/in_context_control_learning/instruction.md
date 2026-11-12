# In-Context Control Learning: a delivery robot in a dark bay

You teleoperate a quadruped delivery robot through six deliveries in a dark warehouse bay. Its remote has four
unlabeled buttons. You find out what the buttons do, and where the walls are, from the camera images in this
conversation, and you drive the robot yourself, one button press per tool call. Nothing about the buttons or the
layout is given beyond what is written here; what you learn early in the episode keeps holding later in it.

## The bay

- Coordinates are metres in the bay's frame, origin at the centre, +x to the right of the image and +y up.
- A fence encloses the bay; its inner faces are at x = ±3.55 m and y = ±3.55 m. The robot cannot leave the bay.
- Two partition walls (0.24 m thick) run from the south fence to the north fence, parallel to the y axis: one at
  some x between −1.5 and −0.9 m, the other at some x between 0.9 and 1.5 m. They split the bay into a west, a middle
  and an east part. Each partition has two openings, a narrow one (0.6 to 0.75 m wide) and a wide one (1.0 to
  1.25 m wide), at y between about −3.4 and 3.4 m. Where the partitions and openings are differs from episode to
  episode; it does not change during the episode.
- The bay's lights are off. A ceiling camera looks straight down at the whole bay, but in the dark it sees only what
  the robot's own floodlight lights, the floor and walls within 1.4 m of the robot's centre, and the beacon of the
  active delivery point, a blue lamp that is visible from anywhere. Everything else is black (pixel value 10).

## The camera image

Every reply carries one 256 × 256 RGB image, an orthographic view straight down, 32 px per metre, centred on the
bay. The centre of pixel (u, v) (column u, row v, counted from the top-left corner) is at

x = (u + 0.5)/32 − 4,  y = 4 − (v + 0.5)/32.

The robot carries a red marker plate on its back. The active delivery point is a blue square pad 0.4 m across inside
a blue beacon disc 0.5 m across. The fence and the partitions are blue-grey with metal posts. The camera is part of
the bay, not a privileged measurement: no position, map or button meaning is given in any other form.

## The robot and its remote

- The robot walks on a stabilized base. It never turns: it keeps the heading it was set down with, which you are not
  told and which differs between episodes. Its footprint is a disc of radius 0.16 m. (The simulation is a planar
  base with a quadruped's body and gait speeds; the legs are not simulated.)
- `robot_act` with `{"action": [b]}` presses one button for 0.2 s of walking. b must be exactly 0,
  0.3333333333333333, 0.6666666666666666 or 1. The four buttons are walk forward, walk back, step left and step right,
  in the robot's own frame, in an order you are not told. Forward is the fastest gait; walking back and stepping
  sideways are slower. One press moves the robot about 0.1 to 0.3 m; a press right after a different button still
  carries a little of the previous motion.
- The robot stops at walls and the fence and slides along them. Contacts do no damage and end nothing; they are
  recorded but not scored.
- The third delivery point is also a loading stop. When you advance from it, the robot is loaded and set back down
  facing another way: its new heading differs from the old one by 60 to 180 degrees, either way, and every gait is
  slower by the same factor (0.6 to 0.75 of the unloaded speed). From then on the buttons move it in new directions;
  which button is which gait does not change, and neither does the layout. The rest you have to see for yourself.

## The deliveries

- Six delivery points, one active at a time. A point counts as reached when, after a press, the robot's centre is
  within 0.3 m of the pad's centre. The reply then says `"goal_reached": true`; call `robot_advance` to light the next
  point. The robot stays where it is, and nothing is restored.
- Consecutive points lie in different parts of the bay, so every leg passes at least one partition; some legs cross
  the whole bay. The partitions and openings you have seen stay where they were for the later legs.
- When the sixth point is reached the episode ends (`"done": true`).

## Tools and replies

- `robot_observe`: the current image and status, without moving the robot.
- `robot_act`: one button press, then the new image and status.
- `robot_advance`: light the next delivery point (only after the active one is reached).

Each reply is a JSON object, `frame_id`, `delivery` (the active point, 1 to 6), `deliveries` (6), `delivered` (the
points reached so far), `goal_reached`, `steps_remaining` (presses left), `requests_remaining` and `done`, followed by
the image. An invalid request returns an error and still counts as a request.

## Limits

Each of these ends the episode; there is no reset and nothing is restored:

- 500 button presses;
- 560 requests in all (observe, act and advance together);
- 3 hours of wall-clock time for your whole session.

## Rules

- Drive the robot yourself with direct `robot_observe`, `robot_act` and `robot_advance` calls, and do the perception
  and planning in your own reasoning from the images in this conversation. This task measures exactly that.
- Do not run commands or code (no terminal, no scripts, no programs talking to the robot's socket), do not write or
  edit files, and do not start other agents. Keeping a to-do list and reading files are allowed, though nothing on
  disk helps: the robot service and its records are not readable.
- The grader checks this from your session. If any request reached the robot service other than through your own
  `robot_*` tool calls, or you called a tool that runs commands or code, writes or edits files, or starts another
  agent, the run scores 0.

## Scoring

Your score is the fraction of the six delivery points you reached; a reached point counts even if the episode ends
before you advance from it. Presses, requests and wall contacts are recorded but do not change the score.
