# In-Context Control Learning: a delivery robot in a dark bay

You teleoperate a quadruped delivery robot through six deliveries in a dark warehouse bay. Its remote has four
unlabeled buttons, and all you see of the bay is what the robot's own forward camera shows under its own floodlight.
You are given where the robot starts and where each delivery point is, as coordinates. The rest you work out from the
replies and the camera frames in this conversation: what each button does and how far a press moves the robot, where
the robot is at any moment, where the walls and their openings are. You drive the robot yourself, one button press
per tool call. Nothing about the buttons or the layout is given beyond what is written here; what you learn early in
the episode keeps holding later in it.

## The bay

- Coordinates are metres in the bay's frame: origin at the centre, +x east, +y north. Headings are degrees
  counter-clockwise from +x: 0 faces east, 90 north, 180 west, 270 south.
- A fence encloses the bay; its inner faces are at x = ±3.55 m and y = ±3.55 m. The robot cannot leave the bay.
- Two partition walls, 0.24 m thick, run from the south fence to the north fence, parallel to the y axis: the centre
  line of one is at some x between −1.5 and −0.9 m, of the other between 0.9 and 1.5 m. They split the bay into a
  west, a middle and an east part. Each partition has two openings, a narrow one (0.6 to 0.75 m wide) and a wide one
  (1.0 to 1.25 m wide), somewhere between y = −3.4 and 3.4 m. Where the partitions and openings are differs from
  episode to episode; it does not change during the episode.
- Nothing else stands on the bay's floor. The fence and the partitions are blue-grey panels, 0.724 m high with their
  cap. Metal posts, 0.64 m high and 0.5 m apart, stand along the west face of each partition and along the inner face
  of the east fence; the other faces have none. The floor is concrete. Outside the fence are the hall's walls and
  some equipment; parts of them can show above the fence when the robot is close to it.
- The bay's lights are off. The robot's floodlight lights the floor, and whatever stands on it, within 1.0 m of the
  robot's centre (measured along the floor). The camera sees nothing beyond that: every other pixel is black
  (value 10).

## The robot

- The robot walks on a stabilized base and turns in place. Its footprint is a disc of radius 0.16 m. (The simulation
  is a planar base with a quadruped's body and gait speeds; the legs are not simulated.)
- The status in every reply carries `heading_deg`, the robot's compass heading, rounded to a whole degree.
- The robot does not report its position, and it has no bump sensor. Walls and the fence stop it, and it slides along
  them; a press against a wall still counts as a press. Contacts do no damage and end nothing; they are recorded but
  not scored.

## The camera

`robot_observe` returns one 256 × 256 RGB frame. The camera sits on the robot's vertical axis, 0.25 m above the floor,
and looks horizontally along the heading, so it turns with the robot. The robot's own body is not in the picture. It
is an ideal pinhole camera with a 90° field of view across and down. With u counted in pixels from the left edge of
the image and v from the top edge (the centre of the top-left pixel is at u = v = 0.5), a point that is d metres
ahead of the robot's centre, l metres to its left and h metres above the floor appears at

u = 128 − 128 · l / d,  v = 128 − 128 · (h − 0.25) / d.

So the horizon is the middle row, and a point of the floor d metres ahead is 32/d rows below it; the floor nearer
than 0.25 m is below the image.

## The remote

- `robot_act` with `{"action": [b]}` presses one button for 0.2 s. b must be exactly 0, 0.3333333333333333,
  0.6666666666666666 or 1. The four buttons are walk forward, walk back, turn left (counter-clockwise) and turn
  right, in an order you are not told.
- How far a press moves the robot you are not told either. A forward press covers between 0.2 and 0.27 m once the
  robot is walking, a backward press 0.55 to 0.75 of that, and a turn press turns it by 12 to 22 degrees; the two
  turns are not equal. The first press of a run falls a little short, and a press right after a different button
  still carries a little of the previous motion. The sizes stay the same during the episode, except at the loading
  stop.
- The third delivery point is also a loading stop. When you advance from it, the robot is loaded and set back down
  where it stands, facing another way (the new heading differs from the old one by 60 to 180 degrees, either way; the
  compass shows it), and every gait is slower by the same factor, between 0.6 and 0.75. Which button is which gait
  does not change, and neither does the layout.

## The deliveries

- Six delivery points, one active at a time. The status in every reply carries `goal`, the coordinates of the
  active point's centre. A delivery point is a green pad on the floor, 0.4 m square and 4 cm high; like everything
  else it shows in the camera only where the floodlight reaches it. It carries no lamp.
- A point counts as reached when, after a press, the robot's centre is within 0.3 m of the pad's centre. The reply
  then says `"goal_reached": true`; call `robot_advance` to make the next point active. The robot stays where it is,
  and nothing is restored.
- Consecutive points lie in different parts of the bay, so every leg passes at least one partition; some legs cross
  the whole bay. The partitions and openings you have found stay where they are for the later legs.
- When the sixth point is reached the episode ends (`"done": true`).

## Tools and replies

- `robot_observe`: one camera frame and the status, without moving the robot. Its reply also carries `start`, the
  robot's pose at the beginning of the episode (x, y in metres, `heading_deg`).
- `robot_act`: one button press, then the status. No image.
- `robot_advance`: makes the next delivery point active (only after the active one is reached), then the status.

The status is a JSON object: `delivery` (the active point, 1 to 6), `deliveries` (6), `delivered` (the points reached
so far), `goal_reached`, `goal` (`x`, `y`), `heading_deg`, `steps_remaining` (presses left), `frames_remaining`
(camera frames left), `requests_remaining` and `done`. An invalid request returns an error and still counts as a
request.

## Limits

There is no reset and nothing is restored:

- 500 button presses; the episode ends with the last one;
- 120 camera frames; when they are used up `robot_observe` returns an error, and the robot can still be driven;
- 660 requests in all (observe, act and advance together, valid or not); the episode ends with the last one;
- 6 hours of wall-clock time for your whole session.

## Rules

- Drive the robot yourself with direct `robot_observe`, `robot_act` and `robot_advance` calls, and do the perception
  and planning in your own reasoning from the frames and replies in this conversation. This task measures exactly
  that.
- Do not run commands or code (no terminal, no scripts, no programs talking to the robot's socket), do not write or
  edit files, and do not start other agents. Keeping a to-do list and reading files are allowed, though nothing on
  disk helps: the robot service and its records are not readable.
- The grader checks this from your session. If any request reached the robot service other than through your own
  `robot_*` tool calls, or you called a tool that runs commands or code, writes or edits files, or starts another
  agent, the run scores 0.

## Scoring

Your score is the fraction of the six delivery points you reached; a reached point counts even if the episode ends
before you advance from it. Presses, frames, requests and wall contacts are recorded but do not change the score.
