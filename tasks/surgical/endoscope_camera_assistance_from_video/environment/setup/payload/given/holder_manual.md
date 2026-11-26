# Camera holder CH-4: integration manual

This is what the device's engineer has: the holder's kinematics and servo, the two laparoscopes it carries, the
operating-room set-up it is used in, the rules by which a camera view is judged and when the surgeon takes the camera
back, the labelled development set and the commissioning service. `holder_model.py` (next to this file) is the reference implementation of sections 2, 3, 4 and
6: every number below is a constant in it, and the benchmark's own simulator computes with the same file.

## 1. The device

CH-4 holds a 10 mm laparoscope in the camera trocar and moves it with four motorised joints about the trocar's remote
centre of motion (RCM): pan, tilt, insertion and roll. It replaces the assistant who holds the camera: while the
surgeon works with two instruments, the holder has to keep them in a view the surgeon can work in. It sees what the
surgeon sees, the endoscope's video, and it knows its own joints. Nothing else: no instrument tracking, no phase
signal from the procedure, no word from the surgeon.

## 2. Frames, joints, kinematics

The patient frame W has its origin at the RCM, x to the surgeon's right, y away from the surgeon (towards the
patient's head), z up (out of the abdomen). Metres and radians throughout.

Joints `q = [pan, tilt, insertion, roll]`:

```
R_WS = Ry(-pan) . Rx(tilt)          the scope's axis is a = R_WS . (0, 0, -1)
p    = insertion * a                the lens: `insertion` metres past the RCM along the scope
R_WC = R_WS . Rz(roll) . Rx(-d) . Rz(h)
                                    the camera's axes in W; d = the scope's direction of view (0 or 30 degrees),
                                    h = the camera head's angle on the scope's eyepiece (section 4)
```

* At pan = tilt = 0 the scope points straight down (-z). Positive pan swings it towards +x, positive tilt towards +y.
* Roll turns the scope and its camera head together about the scope's axis. Positive roll turns the camera
  counter-clockwise as seen from behind it (the picture turns clockwise). The camera head's own angle on the
  eyepiece, h, turns the picture about the view direction; it is set by hand (section 4) and no joint moves it.
* The camera frame C is the graphics one: x right in the image, y up in the image, the camera looks along -z. A world
  point P has camera coordinates `pc = R_WC^T (P - p)`, depth `-pc.z` along the view direction, and falls on the pixel
  `u = cx + fx * pc.x / depth`, `v = cy - fy * pc.y / depth` (u to the right, v down, origin at the top-left corner).
* A 30 degree scope looks 30 degrees below its own axis, towards -y of the rolled scope. Rolling it swings the view
  round the scope's axis; inserting it moves the picture as well as the working distance.

| joint | lower | upper | rate limit | acceleration limit |
|---|---|---|---|---|
| pan | -1.05 rad | 1.05 rad | 0.50 rad/s | 2.5 rad/s^2 |
| tilt | -0.35 rad | 1.25 rad | 0.50 rad/s | 2.5 rad/s^2 |
| insertion | 0.010 m | 0.200 m | 0.050 m/s | 0.30 m/s^2 |
| roll | -2.80 rad | 2.80 rad | 0.80 rad/s | 4.0 rad/s^2 |

Every procedure starts at the home pose `[0, 0.55, 0.020, 0]`: the scope withdrawn into its cannula, looking forward
and down into the cavity.

## 3. The servo

The holder takes rate commands, `[pan_rate, tilt_rate, insertion_rate, roll_rate]` in rad/s, rad/s, m/s, rad/s, one
per control step of 0.1 s. A command is clipped to the rate limits and held for the step. Inside the step the servo
runs five ticks of 0.02 s; in each, every joint's rate moves towards the command with a first-order lag of 0.12 s,
by at most the acceleration limit, and the joint integrates that rate. A joint that reaches a stop stays there with
zero rate. `holder_model.Holder.tick` is this servo.

The joint readings you receive are the joints at the moment the observation is sent, rounded to 1e-5.

## 4. The scopes and the picture

| scope | direction of view | fx = fy | cx | cy |
|---|---|---|---|---|
| "0" | 0 degrees | 480 px | 320 | 240 |
| "30" | 30 degrees | 456 px | 320 | 240 |

The picture is 640 x 480, rectified (no lens distortion), delivered as a JPEG. The endoscope's round field shows as a
disk of radius 310 px about the image centre; outside it the picture is black. The camera head has automatic
exposure, and the light comes from the scope's own tip, so what is near is bright and what is far is dim. The
shutter is open for half a control step: when the holder moves the picture fast, the picture is blurred along its
motion.

Which scope is fitted is decided by the surgical team for each procedure and is not reported to the holder: the two
are told apart by how the picture moves when the holder does.

**The camera head.** The camera head is clamped onto the scope's eyepiece by hand when the scope is set up for a
procedure, at whatever angle the scrub nurse leaves it: its angle h on the eyepiece is anywhere within +-40 degrees of
the eyepiece's reference mark (`holder_model.HEAD_ANGLE_MAX_DEG`; spread evenly over that range in the installations we
know), and not reported to the holder. It turns the picture about the view direction (`R_WC` above,
`holder_model.camera_pose(q, scope, head)`), so it decides which way the picture moves when a joint does, and where the
horizon is: the roll that levels the picture is `holder_model.level_roll(pan, tilt, scope, head)`. It is identified,
like the scope, from how the picture moves when the holder does.

**The camera head is knocked during the procedure.** The camera cable is pulled, the head is bumped: once or twice in
a procedure, equally often, the head turns on the eyepiece to another angle and stays there. The first time comes at a
moment anywhere between 8 and 40 s into the procedure, the second (in half of the procedures) anywhere from 15 s after
the first up to 76 s. Each time the head turns within 0.3 s (three control steps) to an angle anywhere within the same
+-40 degrees, at least 15 degrees from where it was. Nothing reports it: the picture turns about the view direction
while the holder stands still, and from then on the joints move the picture another way and the level roll is another
one.

**The video chain.** The latency from the lens to the holder depends on the operating room's video equipment (camera
control unit, routing, recording): 1, 2 or 3 control steps (0.1 to 0.3 s; `holder_model.LATENCY_STEPS`), each about
equally common, the same for the whole procedure, and not reported. The frame in an observation shows the scene and
the holder as they were that many control steps earlier (early in a procedure, a frame from before the start shows
the start). The joint readings in the same observation are current. The latency shows in when the picture starts to
move after a commanded motion.

## 5. The set-up in the operating room

* The camera trocar is the RCM. The abdominal wall arches over a cavity about 38 cm wide (x) and 40 cm long (y); the
  tissue below is about 170 mm under the RCM in the middle and rises towards the walls, with organs standing up to
  about 45 mm proud of it. No map of it is given.
* **The tissue moves, and not as one piece.** It rises and falls with the patient's breathing, and the breaths are
  irregular: each takes 2.9 to 5.3 s and lifts the tissue by 2 to 10 mm, the patient's own rhythm and depth varying by
  up to 15 and 25 % from breath to breath. The lift is larger towards the patient's head: 0.45 of a breath's depth at
  y = -0.05 m and below, rising evenly to 1.4 of it at y = 0.25 m and beyond. As it rises the tissue also moves towards
  the patient's feet (-y), by 0.3 to 0.7 of the lift, the same share everywhere in a procedure. Where the surgeon holds
  the tissue with an instrument, it follows the instrument: the left hand holds it while it dissects with the right
  and when it stretches the tissue aside, the right hand while it applies a clip; the tissue round the grip is dragged
  with the instrument's tip, fully at the grip and less with distance (a gaussian of 10 to 18 mm), and springs back when
  the instrument lets go. Instruments that rest on the tissue move with it. A map of the tissue built from earlier
  frames is out of date wherever the tissue has moved since.
* The surgeon works on the floor of the cavity in front of the camera trocar, roughly x within +-0.10 m and y from
  0.03 to 0.17 m, and the place of work travels through that area during a procedure.
* The surgeon's two instruments come through trocars to the left and to the right of the camera trocar, nominally at
  (x, y) = (-0.075, -0.045) and (0.075, -0.045) m on the wall (z about -0.03 m); each is placed within 12 mm of its
  nominal place. The left hand's instrument is the "left" instrument, the right hand's the "right" one.
* The assistant's trocar, when there is an assistant's instrument, is on one flank, nominally at (+-0.125, 0.070) m,
  on either side.
* Every instrument has a straight shaft that pivots in its trocar, 5 mm or 10 mm in diameter. A 10 mm instrument is
  built like a 5 mm one at twice the size: shaft, wrist, jaws and marker bands. About one instrument in three is a
  10 mm one, of any kind and any finish, and nothing reports which: an instrument's apparent size alone does not tell
  how far away it is. An instrument brought in by an exchange may be of the other size. Its tip is the far end of
  the instrument on the shaft's axis (the end of the closed jaws, of a hook, of a suction tube). The instruments work
  at the tissue, and often in it: at a place of work an instrument's tip may be worked 2 to 6 mm into the tissue (the
  right hand's at about one place in three, the left hand's at about one in five), hidden from the camera but for its
  shaft, where it enters the tissue; it still counts where it is.
* An energy device (the hook, the scissors) makes smoke at its tip while it cuts: a plume that rises from the tip and
  spreads into a haze over the picture. While the smoke is thick the plume's core, just above the tip, hides that tip
  and anything else close to it.
* Things lie in the field: gauze swabs (white, or soaked with blood) and metal clips, on the tissue near the work.
* The assistant's instrument holds beside the work, 2 to 4 cm to one side of it, and reaches into the middle of the
  work every eight seconds or so on average.

## 6. When a view is usable

A control step's view is usable when all of the following hold. They are judged on the true positions of the
instruments and of the holder, never on the picture.

1. **Safe zone.** Every counting tip lies in front of the lens and inside the central safe zone of the image:
   `|u - cx| <= 0.30 * 640 = 192 px` and `|v - cy| <= 0.30 * 480 = 144 px`.
2. **Zoom band.** Every counting tip is at a working distance at which a 5 mm shaft shows between 22 and 44 px at
   the tip: `fx * 0.005 m / depth` within 22 to 44 px, with `depth` the tip's distance along the view direction (with
   fx = 480 that is a depth between 109 and 55 mm). The rule is a working distance and is the same for a 10 mm
   instrument, whose shaft then shows twice as wide (44 to 88 px).
3. **Horizon.** The display's up reference is `r = (0, cos 45, sin 45)` in W: half-way between "towards the
   patient's head" and "towards the ceiling". The horizon error is the signed angle between the image's up direction
   and r's projection into the image, `e = atan2(r . x_C, r . y_C)` with x_C and y_C the first two columns of R_WC
   (the camera head's angle included); it must stay within +-10 degrees. `holder_model.level_roll(pan, tilt, scope,
   head)` is the roll at which e = 0; with the camera head at 0 it does not depend on the scope.
4. **Steadiness during fine work.** The surgeon is doing fine work at step t when, over the last 1.0 s (steps t-10
   to t), every counting tip has stayed within 9 mm of its own mean position over that time, and no instrument has
   started or stopped counting in it. During fine work the camera must not move the picture of the working point
   faster than 32 px/s: the working point is the mean of the counting tips at step t, and the speed is the distance
   between its pixel under the camera pose of step t-1 and under the pose of step t, divided by 0.1 s. Outside fine
   work the camera may move as fast as it likes.
5. **A clean lens.** The lens touches tissue when it comes within 4 mm of the tissue's surface (the moving surface,
   section 5). The scope then stops where it is, and the lens is smeared: the picture is blurred and stained, and no
   view is usable until the lens has been cleaned. It is cleaned by holding the scope withdrawn in its cannula,
   insertion <= 0.025 m, for 10.0 s without interruption (the scrub nurse wipes it); then the scope can go back to
   work. Nothing reports a smeared lens but the picture. A touch can also end the procedure for the holder (below).

**When the surgeon takes the camera back.** Some mistakes are not forgiven. Then the surgeon takes the camera back
from the holder for the rest of the procedure: from that step on no view of the procedure is usable, and the holder
is not asked again until the next procedure begins. It happens when

* **the lens is driven into the tissue**: it comes within 4 mm of the surface while closing on it faster than
  15 mm/s (`holder_model.LENS_INJURY_SPEED`; a lens held still that the breathing tissue rises onto closes far slower
  and only smears);
* **the lens touches the tissue a second time** in one procedure (`LENS_CONTACTS_TOLERATED`);
* **a critical action is not seen**: fewer than a quarter of the control steps of a critical action have a usable
  view (`CRITICAL_MIN_USABLE`). The critical actions are the moments the surgeon must see: a clip being applied (from
  the moment the clip applier, pushed into the tissue, closes its jaws until it opens them again, 1.2 to 2 s), a
  needle bite (the first half of the needle driver's loop through the tissue, 0.8 to 1.2 s), and an energy activation
  (its first second: when smoke starts to rise from the hook's or the scissors' tip). Each is announced by what the
  instruments do just before (the clip applier advancing on the tissue, the needle driver settling for its bite, the
  hook or scissors at the tissue during dissection). Critical actions in the first 10 s of a procedure, in the first
  3 s after the work has moved to a new place, and those that overlap the 4 s from the moment the camera head is
  knocked (section 4) are not held against the holder.

The score is still the fraction of usable steps: a takeover subtracts nothing, it only leaves the rest of that
procedure without a usable view.

**Which instruments count.** The surgeon's two instruments count; the assistant's never does. An exchange of
instruments takes one of the surgeon's instruments out through its trocar for a while and brings it, or another
instrument in its place, back. That instrument stops counting the moment it starts to withdraw and counts again when
its tip is back within 10 mm of the place where it will work. While it is away, only the other one counts.

The score of a procedure is the fraction of its control steps with a usable view. There is nothing else in it.

## 7. What differs from one procedure to the next

Everything a camera assistant meets from one operation to the next: what kind of procedure it is and so how the
instruments move (dissection with short strokes, suturing with bites and pulls, exploration with long moves from
place to place, clipping and cutting); which instruments are used and what they look like (graspers, scissors, hooks,
needle drivers, clip appliers, suction tubes; black, grey, steel, white or coloured shafts, with or without marker
bands); the tissue (its colours, fat, vessels, how wet it shines); smoke from energy devices, sometimes dense;
highlights; the scope (0 or 30 degrees) and its light; the camera head's angle and the video chain's latency
(section 4); which instruments are 10 mm ones (section 5); whether instruments are exchanged, and when; whether the
assistant has a third instrument in the field, where it comes from, and what it does (it holds near the work, it
stays behind when the work moves on, it reaches into the middle of the field now and then). The graded procedures are
not among the development procedures, and they differ from them in all of this.

**The development set shows only part of this range.** Its recordings use three shaft finishes (black, graphite,
steel) of the eleven the graded procedures bring, two tissue colourings of six, cleaner instruments (in a graded
procedure an instrument's working end may be covered with blood nearly to the colour of the tissue), and less lying in
the field. The commissioning procedures (section 9) bring the graded range. What all of them share is the instruments'
geometry, where they come from, and how they are used.

## 8. The development set (`~/given/dev_procedures/`)

Ten recorded development procedures, `dev01` to `dev10`, each a folder:

* `frames/<step>.jpg`: the endoscope's frame of every second control step (the frame as captured at that step: a
  recording has no latency);
* `labels.jsonl`: the annotation team's labels, one JSON object for every fourth control step (every second recorded
  frame), describing the state the frame shows;
* `meta.json`: the scope and its intrinsics, the step length, the kind of procedure, and what was measured in the lab
  for that recording: `camera_head_deg` (the camera head's angle h at the start), `camera_head_changes` (the steps at
  which the head was knocked and the angle it was turned to, each turn taking three steps from that step) and
  `video_latency_steps` (the latency of the video chain it was recorded through; the frames on file are as captured,
  without it).

A label is what annotators marked by hand on the frame: `step`, `frame`; `joints` (the holder's joints at that step,
from its log); `scope`; `camera_head_deg` (the lab's value for that step); `lens_smeared`; and `instruments`, one entry
per instrument with `role` (`left`, `right`, `assistant`), `present` (inside the cavity), `active` (whether it
counts), `shaft_mm` (its catalogue size, 5 or 10, from the instrument list), `visible` (whether its tip can be seen in
the frame) and `tip_px` ([u, v] where the annotator marked the tip, only when it is visible, else null). A tip is
visible when it is inside the image disk, in front of the lens, out of the tissue with no tissue between it and the
lens, and not inside a thick smoke plume; a marked tip is off by about 1.5 px in each coordinate (at most 4 px). The
labels hold no depth and no judgement of the view: that is for your own controller to work out.

The camera in these recordings was held by an ordinary assistant, not by a good controller: it reacts late, lets the
work drift before it corrects, and in some recordings touches tissue with the lens and has to clean it (nothing took
the camera from it: the recordings show the picture as it was). The recordings are material for building and checking
perception; they are not examples of how to hold the camera.

The set is written while your session starts: the folder fills up over the first minutes, and a file `COMPLETE`
appears in it when all ten recordings are there.

## 9. The commissioning service

A service in your sandbox runs the holder for your own controller on ten commissioning procedures of its own, `c01`
to `c10`: a bench rehearsal in the conditions of the operating rooms the holder is sold to. They are not the recorded
development procedures: they are drawn like the graded ones (the same kinds of procedure as the development set, in
the same order; the graded procedures' range of instruments, tissue and clutter; each its own scope, camera head,
knocks of the head, latency and instrument sizes), and nothing about them is on file. A session plays a procedure from
its first step, sends the observations the grader will send (same picture, same joint readings), and applies your
rate commands with the same servo and the same lens. It gives **no ground truth, no labels and no judgement of the
view**, and it never plays a graded procedure. When the surgeon takes the camera back (section 6) the session ends
there, without saying why.

* `~/given/endocam_client.py`: the client (`Commissioning`, `decode_image`).
* `~/commission/run_session.py <procedure>`: runs a controller folder (`run.sh`) through one session exactly as the
  grader will: the handshake, the deadlines, the repeated requests. `--save DIR` keeps frames and a log of joints and
  actions.
* 12 sessions in all, one at a time, each at most one procedure (800 steps, 80 s of procedure time). A session is
  counted when it starts. Use `--status` (or `Commissioning().status()`) to see what is left. Sessions are refused
  while the development set is still being written.

## 10. Working in the sandbox

Python 3.12 at `/opt/venv/bin/python3` with numpy, OpenCV (`cv2`), Pillow, PyTorch and torchvision (CUDA; the
sandbox has one GPU, which the commissioning service's renderer also uses). There is no network. Your controller runs
in a fresh copy of this environment that holds nothing of yours but `/home/user/submission`: no `~/given`, no
commissioning service. Put `holder_model.py` and everything else your controller needs inside the submission folder.
