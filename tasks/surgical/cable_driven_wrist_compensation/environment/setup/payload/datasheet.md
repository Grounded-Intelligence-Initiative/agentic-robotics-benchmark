# 8 mm articulated needle driver, model ND-8W: integration datasheet (excerpt)

Drive unit: 3-axis instrument drive, firmware 4.2. The instruments are reusable and are swapped
between procedures: every ND-8W on the shelf is a little different (see the tolerances below).
The ND-8W's wrist and jaws are those of the da Vinci Research Kit's Large Needle Driver
(instrument 400006); its CAD model is in `instrument_model/` (below).

## Kinematics

The wrist has three joints driven by three motors of the drive unit through cable pairs:

| joint | symbol | axis | range |
|---|---|---|---|
| wrist pitch | `theta_p` | instrument x axis | +-90 deg |
| jaw 1 | `theta_1` | yaw axis (pitched y axis) | +-120 deg |
| jaw 2 | `theta_2` | yaw axis (pitched y axis) | +-120 deg |

Commanding software usually speaks **yaw** and **grip** instead of the two jaw angles:
`yaw = (theta_1 + theta_2) / 2`, `grip = theta_1 - theta_2` (the opening between the jaws).

Instrument frame (millimetres): origin on the pitch axis, x along the pitch axis, z along the
shaft towards the tip, y completing a right-handed frame. Rotating the pitch joint by
`theta_p` rotates the clevis about x. The yaw axis lies on the clevis, `L1` from the pitch
axis along the clevis's z, parallel to the clevis's y. A jaw at angle `theta_j` points along
`(sin theta_j, 0, cos theta_j)` in the clevis frame and its tip lies `L2` from the yaw axis:

```
tip_j = Rx(theta_p) * ( [0, 0, L1] + L2 * [sin theta_j, 0, cos theta_j] )
Rx(a) = [[1, 0, 0], [0, cos a, -sin a], [0, sin a, cos a]]
```

`L1 = 9.1 mm`, `L2 = 9.65 mm` (the jaw's length from the yaw axis to its tip; the jaws close
on the plane x = 0 of the clevis frame, where their tips meet).

## Drive and cables

Motor angles are in radians at the motor shaft. The cable ratios (joint radians per motor
radian) and the routing of the jaw cables over the pitch pulleys give, nominally:

```
theta_p = 1.0186 * m_p
theta_1 = 1.2177 * m_1 - 0.8154 * theta_p
theta_2 = 1.2177 * m_2 - 0.8154 * theta_p
```

These are design values: pulley diameters vary by up to +-6 % between instruments, the
routing coupling by up to +-15 %, and the motor zero (the instrument's engagement on the
drive) is not referenced: expect an offset of up to +-0.3 rad per motor after every mount.
The stock firmware uses exactly the nominal relations above, with no offsets.

Each motor runs a position servo (closed-loop bandwidth roughly 8 Hz, varying by up to about
30 % between axes and units), rate-limited to 30 rad/s. The drive takes a
new command every 5 ms (200 Hz) and holds it until the next. Commands beyond +-8 rad are
clipped by the drive.

**Settling.** Right after an instrument is mounted the drive dithers its wrist at the first
command of the calibration recording, so the recording starts from a settled state. The drive
does not dither again: after the recording it holds the recording's last command, and every
later run starts in whatever state the run before left the cables in, the drive stepping to
the new run's first command.

## The calibration recording

Right after an instrument is mounted, the robot makes a calibration recording: for two minutes
an operator moves the wrist about through the stock firmware while the stereo endoscope films
it. The recording starts settled. The jaws are rigid and never pass through each other: the drive
seats a freshly mounted instrument so that the stock firmware's calibration motion never closes
the jaws onto each other (if it would, the drive re-engages the instrument, which draws new
motor zero offsets). Throughout a calibration recording the jaws therefore stay at least 2
degrees apart, jaw 1 at the larger angle. Its folder:

`motors.csv` (200 Hz, drive clock, starting at 0):
`t_s, cmd_pitch, cmd_jaw1, cmd_jaw2, enc_pitch, enc_jaw1, enc_jaw2`. A row's command is the one
sent at `t_s` and held for 5 ms; its encoder values are the motor angles read at `t_s`.

`endoscope/`: the robot's stereo endoscope filming the wrist, about 15 frames per second, on
the endoscope's own clock, which is not synchronised with the drive's and carries an unknown
offset of under 1.5 s, different in every recording. `frames.csv`: `frame, camera_t_s`; `left/<frame>.jpg` and
`right/<frame>.jpg`: the two 320 x 240 RGB images of that frame (the exposure is short enough to
freeze the motion). A few frames are lost in transfer, sometimes a second or two in a row, and smoke from the
energy devices now and then hazes the view for a few seconds. The assistant's instruments are usually in the
field too, held still for the recording; one may cross the view and hide part of the wrist.

`endoscope_pose.json`: the left camera's pose in the instrument frame as the robot's kinematics
reports it. `R_left_camera`'s columns are the camera's x (image right), y (image down) and z
(viewing direction) axes; `p_left_camera` is its optical centre (mm). The endoscope stays put
during a recording and sits somewhere else in every one; the reported pose is good to about a
millimetre and a degree.

## The endoscope

Two rectified pinhole cameras without distortion: pixel (u, v) (column, row, from 0 at the
top-left pixel's centre) of either image looks along (u - 159.5, v - 119.5, 210.0) in that
camera's frame. Both cameras have the left camera's orientation; the right camera's optical
centre sits 4.4 mm along the left camera's x axis. The light comes from the endoscope tip.

## The instrument as the endoscope sees it

A black composite shaft, 8.7 mm across, ending at the pitch axis; a steel clevis with its
pulleys and cables; two steel jaws with serrated grips, against pink tissue. `instrument_model/`
holds the instrument's surface as triangle meshes (Wavefront OBJ, millimetres; from the dVRK's
model, github.com/jhu-dvrk/dvrk_model, MIT licence, simplified), one file per rigid part, each
in the frame its part moves with:

| file | part | frame |
|---|---|---|
| `shaft.obj` | the shaft's distal end | instrument frame (fixed) |
| `clevis.obj` | the clevis, its pulleys and the yaw pin | clevis frame: the instrument frame rotated by `Rx(theta_p)` |
| `jaw1.obj`, `jaw2.obj` | jaw 1, jaw 2 | jaw frame: the clevis frame moved to the yaw axis `(0, 0, L1)` and rotated about its y axis by `theta_j` |

A point `p` of `jaw1.obj` is at `Rx(theta_p) * ([0, 0, L1] + Ry(theta_1) * p)` in the instrument
frame, with `Ry(a) = [[cos a, 0, sin a], [0, 1, 0], [-sin a, 0, cos a]]`; at `theta_j = 0` a jaw
points along +z, and its tip `(0, 0, L2)` (the middle of the jaws' closing line) is the tip above.
Jaw 1's grip faces -x and jaw 2's +x, so the jaws open as `theta_1 - theta_2` grows. The two jaws
look alike: jaw 1 is the one at the larger angle (the grip is positive whenever the jaws are apart).
