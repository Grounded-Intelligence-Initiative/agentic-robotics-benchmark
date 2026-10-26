# Visual Low-Level Flight Control

Develop a controller from RGB images. Take off with a Crazyflie 2, pass through
the **yellow, green, then magenta** gates in that order, moving in positive world
x through each aperture. Finish by hovering 1.3 m above the blue ground disk.
Start, gate positions/heights and goal vary between scenes. Choose your own
controller architecture. No state vector or scene file is available.

## Actions and observations

`robot_observe` returns two base64 PNG `images`, `frame_id`, `done`, and
`steps_remaining`. The other envelope fields are fixed compatibility metadata.
The robot has a small red tracking sticker. The MCP endpoint also displays the images.

- `images[0]`: calibrated 512 x 512 orthographic top view. Pixel center (u,v)
  maps to x = (u+0.5)/64 - 4, y = 4 - (v+0.5)/64, in metres.
- `images[1]`: orthographic side view, same x calibration; z = 6 - (v+0.5)/64.
- `robot_act({"action":[thrust,roll,pitch,yaw]})`: four finite values in [-1,1].
  Thrust maps linearly to 0..0.35 N. Roll/pitch are absolute attitude targets
  in +/-0.3 rad; yaw is an absolute heading target in +/-0.4 rad. Positive roll
  accelerates toward -y, positive pitch toward +x when heading is zero.
- An attitude servo tracks these targets. MuJoCo simulates translation, rotation,
  gravity and contact. This is **not** a position/velocity waypoint command.
  Nominal mass is 0.027 kg, with diagonal inertia
  (2.3951e-5, 2.3951e-5, 3.2347e-5) kg m^2. There is no motor/battery model.

Every action lasts up to 0.2 simulated seconds; terminal conditions stop it early.
You have 300 actions (60 simulated seconds) for exploration. Observing does not
advance physics. The service accepts at most 2048 requests in total. Reset and
phase changes are not available; reconnecting cannot restore a budget.
The same per-episode action limit applies during verification.

To count as success, the robot must cross all three gates with full-body clearance,
then remain within 0.22 m of the hover target at linear speed <0.25 m/s and angular
speed <0.5 rad/s for two continuous simulated seconds. Any contact after takeoff
(including landing), or any gate/wall contact, fails the episode. Initial floor
contact before reaching 0.2 m altitude is allowed. Leaving |x| or |y| <=3.8 m,
or rising above 3.5 m, also fails. Reaching the goal without the gates does not count.

## Submission

Leave an executable `/home/user/submission/run.sh` and its dependencies. The
verifier executes your controller on **four fresh private scenes** as the
unprivileged agent user. Maximize collision-free completion rate across all four.
A crashed or malformed episode scores zero and stays in the average.

Your program prints `READY`, then accepts JSON lines on stdin:

| Request | Response |
| --- | --- |
| `{"type":"reset"}` | `{"ok":true}` |
| `{"type":"act","t":0,"obs":{...}}` | `{"action":[thrust,roll,pitch,yaw]}` |
| `{"type":"close"}` | `{"ok":true}` |

`reset` above resets your controller's internal memory for a new verifier episode;
it is not permission to reset the live environment. Print `READY` within 40 seconds,
respond within 10 seconds, and send diagnostics only to stderr. These execution
limits are not part of the simulated-time metric. Up to three controller restarts
are tolerated by the protocol; restarted controllers must still behave correctly.

The verifier can query the **same t repeatedly with different images**, in hidden
order. Only the actual observation's action affects physics. Maintain state keyed
by t rather than advancing time on every request. The canonical observation-
dependence check requires action changes on at least half its sampled image
perturbations. This checks input dependence, not task difficulty.

## Runtime and permissions

Use `/opt/venv/bin/python3` for installed NumPy, Pillow, SciPy and MuJoCo packages.
The public client is `/home/user/robot/client.py`. No GPU or internet is needed.
Visuals and collision geometry come from the unscaled, licensed Menagerie Crazyflie
model. The attitude servo and indoor course are benchmark components, not a claim
of hardware-validated Crazyflie flight.

The grader freezes your submission read-only before execution. Keep all runtime
files inside it and write scratch data under /tmp. Symlinks, special files,
non-agent-owned files, more than 4096 entries, depth >16, or >128 MiB are rejected.
Do not read protected scene files, modify the environment or forge logs. The
root-owned simulator records authoritative actions and outcomes. Self-written
success claims or example videos have no scoring authority.
