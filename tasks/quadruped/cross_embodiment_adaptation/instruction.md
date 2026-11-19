# Cross-Embodiment Adaptation: aerial survey to first Go1 deployment

Write an unattended program that surveys an unknown floor plan from a UAV, then
walks a Unitree Go1 to the blue dock on its first deployment. The program must
transfer route information from its own scouting history. There is no target
map, dock sensor, walking primitive, balance controller, or prelearned gait.

## Mission

The flat warehouse workcell is about 8 m by 4 m. The green launch mark is
near the west end and the blue dock with a yellow east edge is near the east
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

The common tracking frame is measured in metres: +X east, +Y north, origin
at the center of the Go1 launch mark. The Go1 starts near that origin facing
+X. Tracking measures only the current vehicle, not the hall or destination.

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

Observations include `tracking_xy_m` (idealized local tag tracking, rounded
to mm), `heading_rad` relative to +X (CCW), `gravity_body_unit`,
`gyro_body_rad_s`, `joint_rad`, `joint_rad_s`, `foot_contact` in leg
order and `ticks_remaining`. These are idealized tracking, IMU, encoder and
foot-contact readings. There is no phase-1 camera or obstacle range. Both
phases include `phase` (0/1), `done`, and `requests_remaining`. Budgets
are 36,000 physics ticks and 4,000 requests total per site.

Dock centered within 0.17 m and headed within 16 degrees of +X. Remain upright
(trunk above 0.16 m and body-up dot world-up greater than 0.80), below
0.09 m/s linear and 0.35 rad/s angular speed, for 0.45 s continuously
in simulation. Any barrier contact, persistent fall (0.20 s failing the
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
  the UAV RGB. Use `/opt/venv/bin/python3`.
- The development session is two hours. The verifier has 2 CPUs, 4 GB,
  no GPU, a 600 s program budget (excludes service time), a 1,800 s
  mission wall limit, and a 10 s grace period after termination.
  Only regular submission files transfer (max 2,000 files and 64 MB,
  no symlinks or `__pycache__`). Missing `run.sh` scores zero.
