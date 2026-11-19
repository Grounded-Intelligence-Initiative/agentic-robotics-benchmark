# Cross-Embodiment Adaptation: register an aerial survey to a first Go1 deployment

Write an unattended program that surveys an unknown floor plan from a UAV, then
walks a Unitree Go1 to the blue dock on its first deployment. The program must
transfer route information from its own scouting history. There is no target
map, dock sensor, walking primitive, balance controller, or prelearned gait.

## Mission

The flat warehouse workcell is about 8 m by 4 m. The target starts somewhere
in the western staging bay. A blue dock with a yellow east edge is near the east
end. Three transverse rack lines separate four staging bays. The first and
last rack lines each have a near-center shortcut and a side passage. Both
are real openings, but a clear centerline does not mean the Go1's trunk,
hips and stepping feet will fit. Determine which openings are usable by
measuring the scene at each rack; the side passage can be north or south
independently at the two racks. The middle rack forces a return toward the
center. Leave room to align before entering a passage and to clear the rack
before changing direction. Passage positions, widths and dock center vary.
The first frame does not show the openings. The orange caps mark actual
collision footprints; they do not encode passability.

Only the UAV uses the survey frame: origin at the UAV's first camera center
projected onto the floor, +X east and +Y north. The Go1 is absent during
the survey. It deploys with independently chosen launch translation and yaw;
there is no common tracker, compass, launch mark, or target pose on the map.
To transfer a route, use the same fixed objects as seen from above and from
the Go1. Around the staging bay are steel bollards with identical green
spherical safety caps, radius 0.105 m, centers 0.72 m above the floor.
Their arrangement varies; individual caps carry no identifiers.

## Phase 0: UAV

- `observe()` returns one 256 x 256 nadir RGB PNG in `images` and
  `tracking_xy_m: [x,y]`. It does not step time. The pinhole camera has a 78-degree
  vertical field of view and its center is 1.65 m above the ground. Image
  right is +X; image up is +Y. Barrier tops are 0.32 m above the floor.
  There is no full-site image, semantic mask, or depth output.
- `act([dx,dy])` moves the idealized survey UAV at fixed altitude. The
  displacement norm cannot exceed 0.60 m; the allowed tracking rectangle is
  `[-0.45,6.80] x [-1.55,1.55]`. Movement clips at its edge. The action
  returns a new camera observation. There are 80 moves, counted by
  `scout_actions_remaining`.
- In grading, call `advance()` once after at least four scout actions. It
  permanently retires the UAV, instantiates the Go1 and returns its first
  observation. It cannot be reversed.

## Phase 1: Go1

Send `act({"target_rad": [12 angles], "ticks": n})`, where `n` is an
integer from 1 to 40, not exceeding `ticks_remaining`. The native MuJoCo
position servos hold those exact angle targets for `n` 2 ms physics steps
(or until the mission ends). Out-of-range or nonfinite commands are rejected.
Order is FR, FL, RR, RL, each in hip (abduction about +X), thigh (about +Y),
calf (about +Y) order. In the trunk frame +X is forward, +Y left and +Z up.

| Joint | Target range (radians) |
|---|---|
| Hip | [-0.863, 0.863] |
| Thigh | [-0.686, 4.501] |
| Calf | [-2.818, -0.888] |

The hip pivots sit at x = +/-0.1881 m (front/rear) and y = +/-0.04675 m
(left/right). The thigh axis is a further 0.08 m outward, so opposite
thigh axes are about 0.254 m apart even at zero hip abduction. Thigh and shank
are each 0.213 m along local -Z at zero angle. Nominal home targets are
`[0, 0.9, -1.8]` per leg. The Go1 starts at its settled home stance, and
servo stiffness varies by up to 10%. The trunk has six free degrees of
freedom. There are no hidden force, yaw, touchdown, or foot-placement
commands. The exact simulator model and runtime are service-only.

Target `observe()`, `advance()` and target `act()` supply `odom_xy_m` and
`odom_yaw_rad`: ideal planar displacement (mm rounding) and CCW yaw
change (radians, wrapped to [-pi,pi]) relative to the **settled launch
trunk frame**, not to the survey. Both start at zero. The odometry axes are
fixed for the mission: initial forward is +x, initial left is +y. The other
readings are `gravity_body_unit`, `gyro_body_rad_s`, `joint_rad`,
`joint_rad_s`, `foot_contact` in leg order and `ticks_remaining`. There
is no range, absolute heading, absolute position, or depth output.

During a live deployment, `look(detent)` returns one actual local RGB PNG in
`images`, plus `requests_remaining`, with **no physics step**. It does not
turn the Go1. An ideal pan mount shares one optical center 0.20 m above the
trunk origin along trunk +Z. Integer detents 0..7 pan counterclockwise by
`detent * 45` degrees from trunk +X toward trunk +Y. At each detent the
camera points **36 degrees up** from the trunk's horizontal plane toward
the elevated fixtures; it tilts with the trunk. Image right is clockwise
from the pan direction, image up is toward higher elevation. The square
256 x 256 undistorted pinhole has a 70-degree vertical field of view, principal
point (127.5,127.5) in zero-based pixel-center coordinates, and square pixels.
You can scan all azimuths at rest before issuing any joint command. Each
site allows 16 exposures at any time while the mission is live. Regular
target observations are image-free. Normal observations in both phases
include `phase` (0/1), `done`, and `requests_remaining`. Budgets remain
36,000 physics ticks and 4,000 requests total per site.

Dock centered within 0.17 m and headed within 16 degrees of survey +X. Remain upright
(trunk above 0.16 m and body-up dot world-up greater than 0.80), below
0.09 m/s linear and 0.35 rad/s angular speed, for 0.45 s continuously
in simulation. Any barrier or bollard contact, persistent fall (0.20 s failing the
upright test), or budget exhaustion ends the attempt. Scoring is safe docks
out of eight sites. Passage choices, crossings, dock approach and stop
causes are diagnostics, not score.

## Submission and zero-shot rules

- Leave `/home/user/submission/run.sh` and its dependencies. ALE freezes that
  submission before the first target deployment. A fresh, separate verifier
  launches it once per hidden site from a read-only copy with a fresh HOME
  and TMPDIR. No reset, restart, persistent state across sites, network,
  pretrained walking policy, target practice (including offline target
  rollouts), or weight updates. You may write an analytical control program
  and use ordinary feedback within the one mission. Do not access protected
  files, guess site seeds, or hardcode site maps.
- Development is **scout-only**. `new_site()` supplies another random
  UAV site; target deployment is disabled. The MCP tools and
  `/home/user/robot/client.py` use the same socket as the submission.
  During grading use the Unix socket in `$ROBOT_SOCKET` (default
  `/run/visual_robotics/world.sock`) through the client at
  `/opt/visual_robotics/client/client.py`. Its `camera_image(obs)` decodes
  either camera's RGB. Use `/opt/venv/bin/python3`.
- The development session is two hours. The verifier has 2 CPUs, 4 GB,
  no GPU, a 600 s program budget (excludes service time), a 1,800 s
  mission wall limit, and a 10 s grace period after termination.
  Only regular submission files transfer (max 2,000 files and 64 MB,
  no symlinks or `__pycache__`). Missing `run.sh` scores zero.
