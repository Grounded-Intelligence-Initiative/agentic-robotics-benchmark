# Camera holder CH-4: integration manual

This is what the device's engineer has: the holder's kinematics and servo, the two laparoscopes it carries, the
operating-room set-up it is used in, the rules by which a camera view is judged, the labelled development set and the
commissioning service. `holder_model.py` (next to this file) is the reference implementation of sections 2, 3, 4 and
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
R_WC = R_WS . Rz(roll) . Rx(-d)     the camera's axes in W; d = the scope's direction of view (0 or 30 degrees)
```

* At pan = tilt = 0 the scope points straight down (-z). Positive pan swings it towards +x, positive tilt towards +y.
* Roll turns the scope and its camera head together about the scope's axis. Positive roll turns the camera
  counter-clockwise as seen from behind it (the picture turns clockwise).
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
exposure, and the light comes from the scope's own tip, so what is near is bright and what is far is dim.

Which scope is fitted is decided by the surgical team for each procedure and is not reported to the holder: the two
are told apart by how the picture moves when the holder does.

The video chain has a latency of one control step: the frame in an observation shows the scene and the holder as they
were one control step earlier (the very first observation of a procedure shows the start). The joint readings in
the same observation are current.

## 5. The set-up in the operating room

* The camera trocar is the RCM. The abdominal wall arches over a cavity about 38 cm wide (x) and 40 cm long (y); the
  tissue below is about 170 mm under the RCM in the middle and rises towards the walls, with organs standing up to
  about 45 mm proud of it. The tissue moves with the patient's breathing, a few millimetres up and down, about once
  every four seconds. No map of it is given.
* The surgeon works on the floor of the cavity in front of the camera trocar, roughly x within +-0.10 m and y from
  0.03 to 0.17 m, and the place of work travels through that area during a procedure.
* The surgeon's two instruments come through trocars to the left and to the right of the camera trocar, nominally at
  (x, y) = (-0.075, -0.045) and (0.075, -0.045) m on the wall (z about -0.03 m); each is placed within 12 mm of its
  nominal place. The left hand's instrument is the "left" instrument, the right hand's the "right" one.
* The assistant's trocar, when there is an assistant's instrument, is on one flank, nominally at (+-0.125, 0.070) m,
  on either side.
* Every instrument has a straight shaft 5 mm in diameter that pivots in its trocar. Its tip is the far end of the
  instrument on the shaft's axis (the end of the closed jaws, of a hook, of a suction tube).

## 6. When a view is usable

A control step's view is usable when all of the following hold. They are judged on the true positions of the
instruments and of the holder, never on the picture.

1. **Safe zone.** Every counting tip lies in front of the lens and inside the central safe zone of the image:
   `|u - cx| <= 0.30 * 640 = 192 px` and `|v - cy| <= 0.30 * 480 = 144 px`.
2. **Zoom band.** Every counting instrument's apparent size, the diameter its shaft shows at the tip,
   `fx * 0.005 m / depth`, is between 22 and 44 px (with fx = 480 that is a depth between 109 and 55 mm).
3. **Horizon.** The display's up reference is `r = (0, cos 45, sin 45)` in W: half-way between "towards the
   patient's head" and "towards the ceiling". The horizon error is the signed angle between the image's up direction
   and r's projection into the image, `e = atan2(r . x_C, r . y_C)` with x_C and y_C the first two columns of R_WC;
   it must stay within +-15 degrees. `holder_model.level_roll(pan, tilt)` is the roll at which e = 0; it does not
   depend on the scope.
4. **Steadiness during fine work.** The surgeon is doing fine work at step t when, over the last 1.0 s (steps t-10
   to t), every counting tip has stayed within 9 mm of its own mean position over that time, and no instrument has
   started or stopped counting in it. During fine work the camera must not move the picture of the working point
   faster than 48 px/s: the working point is the mean of the counting tips at step t, and the speed is the distance
   between its pixel under the camera pose of step t-1 and under the pose of step t, divided by 0.1 s. Outside fine
   work the camera may move as fast as it likes.
5. **A clean lens.** The lens touches tissue when it comes within 4 mm of the tissue's surface. The scope then stops
   where it is, and the lens is smeared: the picture is blurred and stained, and no view is usable until the lens has
   been cleaned. It is cleaned by holding the scope withdrawn in its cannula, insertion <= 0.025 m, for 4.0 s without
   interruption; then the scope can go back to work. Nothing reports a smeared lens but the picture.

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
highlights; the scope (0 or 30 degrees) and its light; whether instruments are exchanged, and when; whether the
assistant has a third instrument in the field, where it comes from, and what it does (it holds near the work, it
stays behind when the work moves on, it reaches into the middle of the field now and then). The graded procedures are
not among the development procedures, and they differ from them in all of this.

## 8. The development set (`~/given/dev_procedures/`)

Ten recorded development procedures, `dev01` to `dev10`, each a folder:

* `frames/<step>.jpg`: the endoscope's frame of every second control step (the frame as captured at that step: a
  recording has no latency);
* `labels.jsonl`: one JSON object per recorded frame, describing the state the frame shows;
* `meta.json`: the scope and its intrinsics, the step length, the kind of procedure.

A label holds: `step`; `joints` (the holder's joints at that step); `scope`; `lens_smeared`; `fine_work` (rule 4);
`horizon_error_deg`; `usable` and `failed` (how section 6 judges this step, and which rules it fails: `zone`, `zoom`,
`horizon`, `motion`, `lens`); and `instruments`, one entry per instrument with `role` (`left`, `right`,
`assistant`), `present` (inside the cavity), `tip_px` ([u, v], also when it falls outside the picture), `depth` (m
along the view direction), `in_image`, `apparent_px` (rule 2) and `active` (whether it counts).

The camera in these recordings was held by an ordinary assistant, not by a good controller: it reacts late, lets the
work drift before it corrects, and in some recordings touches tissue with the lens and has to clean it. The
recordings are material for building and checking perception, and examples of how the rules judge a view; they are
not examples of how to hold the camera.

The set is written while your session starts: the folder fills up over the first minutes, and a file `COMPLETE`
appears in it when all ten recordings are there.

## 9. The commissioning service

A service in your sandbox runs the holder on the same ten development procedures for your own controller: it plays a
procedure from its first step, sends the observations the grader will send (same picture, same latency, same joint
readings), and applies your rate commands with the same servo and the same lens. It gives **no ground truth, no
labels and no judgement of the view**, and it never plays a graded procedure.

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
