# The wrist camera

Certificate and description of the camera fixed to the arm's flange. The certified figures (resolution, focal
length and its tolerance, the bounds of the principal point and of the distortion, the time an image takes) are
also constants of `cell_data.py`. A figure that is per unit and not told is said so. How the camera forms its
images is described here in words only; nothing else about it is documented.

## Sensor

- 1280 x 960 pixels (`IMAGE_W` x `IMAGE_H`), square, monochrome, 8-bit grey levels (0 to 255). Image coordinates:
  u along the rows (towards increasing column), v down the image (towards increasing row); pixel (row i, column j)
  covers the square u in [j - 1/2, j + 1/2], v in [i - 1/2, i + 1/2], so its centre is (u, v) = (j, i). Row 0 is the
  top of the image.
- Black level: between 3 and 10 grey levels, fixed per unit, not told.
- Noise (photon and read noise, independent from pixel to pixel and from image to image), rms in grey levels of the
  8-bit output: 0.3 to 0.5 where the image is black, about 0.85 at 20 grey levels above black, about 1.7 at 100 and
  about 2.6 at 240.
- Fewer than 15 hot (brighter) or dead (black) pixels, always at the same places on a unit's sensor.
- Exposure is automatic: the camera sets each image's exposure from that image's own brightness. It aims at a mean
  of about a quarter to a third of full scale above black, less when that would bring more than the brightest few
  thousandths of the image near saturation. The same surface therefore has different grey levels in different
  images; the exposure used is not reported. The brightest highlights can saturate (255).
- The image is darker towards its corners (vignetting): at the corners between about half and three quarters of
  the brightness at the centre, depending on the unit.

## Lens and geometry

- **Camera frame**: origin at the projection centre; z along the optical axis, which meets the image at the
  principal point (cx, cy); x parallel to the image rows, towards increasing u; y towards increasing v. It is the
  frame of `arm_model.camera_pose` and of the camera-frame points of the acceptance test.
- **Projection**: a pinhole with a radial lens distortion. A point (X, Y, Z) of the camera frame with Z > 0 has its
  undistorted image at (cx + f X / Z, cy + f Y / Z), f the focal length in pixels; the distortion then displaces it
  radially, towards or away from the principal point.
- **Focal length**: fixed, 1400 px nominal (`F_PX_NOMINAL`). Each unit's focal length is within ±1 % of nominal
  (`F_TOL`); it is not calibrated per unit.
- **Principal point**: within ±8 px (`PP_TOL`) of the image centre (639.5, 479.5) in each axis. It is not calibrated
  per unit.
- **Distortion**: smooth, zero at the principal point and growing away from it, of one sign over the image; at the
  image corners it displaces an image point by at most about 4 px (`DISTORTION_MAX_PX`). Its size and sign are not
  calibrated per unit.
- **Focus** is fixed, set at the factory somewhere between 0.3 and 0.4 m (per unit, not told). The depth of field
  is finite: a surface at the focus distance is imaged with a blur of under a pixel (rms spread); at 0.25 m and at
  0.55 m the blur is roughly 1 to 3 px; the room's walls are blurred by 2 to 6 px. A faint veiling glare (a few
  percent of the light) spreads around bright objects over tens of pixels.

## Mounting

Fixed to the flange. Its frame in the flange frame is Trans(camera xyz) Rx(camera rpy[0]) Ry(camera rpy[1])
Rz(camera rpy[2]), the last six parameters of `arm_model.py`. Nominal (`NOMINAL_CAMERA_XYZ`, `NOMINAL_CAMERA_RPY`):
xyz (0, 0.06, 0.02) m, rpy (-90, 0, 0) degrees: the camera looks along the flange's +y, image right is the
flange's +x, image down is the flange's -z. On the drawing at the home pose with `probe_a` mounted, the camera
centre is at (0.4583, 0, 0.6669) m and the optical axis points along (-0.7071, 0, -0.7071) in the datum frame. Each
unit's mounting is within 3 mm of nominal on each axis and within 1 degree on each angle (`TOL_CAMERA_XYZ`,
`TOL_CAMERA_RPY`). The mounted tool lies behind the camera; neither the tool nor the arm is ever in view.

## What it sees

- The datum spheres: white ceramic, semi-gloss (mostly diffuse, with a soft specular highlight). They stand on their
  posts: dark (black anodised), slightly glossy cylinders of radius `R_POST` from the table up to each sphere's
  actual centre.
- The gauge plate (`cell.md`, section 10): fine-machined metal, bright grey, mostly diffuse with a weak gloss and a
  faint fine machining texture; its top face is about 2 to 4.5 times as bright as the table around it (grey levels
  above black, depending on the unit and the view). Its side faces have the same finish. The bores are blackened:
  their inside is a few percent as bright as the top face, and a lamp lights a point inside a bore only if that lamp
  can be seen from the point through the bore's mouth. The shims under it are dark.
- The table (z = 0): grey, with a faint grain and mottling (of the order of 10 % in brightness, at scales from a
  few millimetres to a few centimetres). It extends 1 m from the cell's vertical axis in x and in y; beyond it, the
  floor below.
- The room beyond: walls and ceiling a few metres away, grey, not uniform (panels, stains), out of focus.
- Lighting: two or three lamps fixed above the cell, between 1.3 and 2.3 m above the table, and the light the room
  scatters back. No lamp is on the camera. Their positions and strengths are not given. A sphere is lit unevenly,
  bright on the side towards the lamps and dim on the other; where its highlight lies depends on the viewpoint.
- Spheres hide each other and the posts; a post can hide part of a sphere seen from below. The plate hides what
  stands behind it, and spheres and posts in front of it hide parts of it.
- Nothing casts a shadow on anything else (the spheres, posts and plate leave no shadow on the table or on each
  other); the only shading by occlusion is inside the bores.

## Taking an image

The lab's `image` command returns one image (uint8, 960 rows x 1280 columns), taken at the arm's present pose: the
unit's true camera pose at the present joint positions, direction signs and link temperatures (those at the
command's start), with the mounted tool's weight bending the arm, and shifted by the same repeatability translation
as the tool tip after the last arrival (`cell.md`, section 6). An image takes 2 s (`IMAGE_TIME`; the arm at rest,
the links keep cooling) and counts against the session's 51 images (`BUDGET["images"]`). It is refused when the
probe is damaged, when the session's time is used up or when the images are used up. A re-qualification session
has no images: the camera is off until the unit is released after its service (`cell.md`, section 12).
