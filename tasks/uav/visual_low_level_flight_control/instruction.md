# Visual Low-Level Flight Control

Develop a controller from a single onboard RGB camera. Take off with a Crazyflie 2, pass through
the **yellow, green, magenta, then cyan** gates in that order. Each gate has a
**colored entry face with four white corner squares**; the reverse face is gray.
A white floor arrow also indicates its required crossing direction. Cross from
the colored side through the aperture, not around the frame. This is a switchback
course, not a circular lap: entry directions reverse, and gate heights alternate
between higher and lower passages. You may need to go around a gate to reach its
entry side. Flying monotonically or always turning the same way cannot complete it. Finish
by hovering 1.3 m above the blue ground disk, 0.65 m in front of the blue square
panel and level with its center. A tall central equipment cabinet obstructs both
vision and motion. Later gates may be out of view or hidden behind it; use flight
history and changes in viewpoint to find a safe route.
Start, gate positions/heights/orientations and goal vary between scenes. Choose
your own controller architecture. No state vector or scene file is available.

## Actions and observations

`robot_observe` returns one base64 PNG in `images`, `frame_id`, `done`, and
`steps_remaining`. The other envelope fields are fixed compatibility metadata.
The MCP endpoint also displays the image. There are no external tracking cameras,
depth maps, segmentation masks, positions, velocities or orientation readouts.

- `images[0]`: 512 x 512 perspective RGB, vertical field of view 100 degrees.
  The camera is rigidly attached at body position (0.06, 0, 0.015) m, looking
  along body +x tilted down 8 degrees. Image right is body -y. Its view rotates
  with the drone; there is no pixel-to-world coordinate conversion.
- Each gate has a 0.9 x 0.9 m aperture and 0.06 m wide bars. Entry corner squares
  have centers 0.96 m apart horizontally and vertically. The final blue panel is
  0.6 x 0.6 m, with white corner centers 0.54 m apart. These are visual dimensions,
  not a supplied map. The panel is a solid obstacle, not a fifth gate.
- `robot_act({"action":[thrust,roll,pitch,yaw]})`: four finite values in [-1,1].
  Thrust maps linearly to 0..0.35 N. Roll/pitch are absolute attitude targets
  in +/-0.3 rad; yaw is an absolute heading target in +/-pi rad, with shortest-angle
  wrapping. The full yaw range allows looking around. Positive roll
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

To count as success, the robot must cross all four gates with full-body clearance,
in the specified order and direction. Reverse or out-of-order crossings earn no
credit; the next required gate remains unchanged. After the final gate, remain
within 0.22 m of the hover target at linear speed <0.25 m/s and angular
speed <0.5 rad/s for two continuous simulated seconds. Any contact after takeoff
(including landing), or any gate/wall contact, fails the episode. Initial floor
contact before reaching 0.2 m altitude is allowed. Leaving |x| or |y| <=3.8 m,
or rising above 3.5 m, also fails. Reaching the goal without the gates does not count.

## Submission

The authoring phase allows up to 60 minutes of wall-clock time to explore and
develop your submission. This is separate from the simulated-time flight budget.

Leave an executable `/home/user/submission/run.sh` and its dependencies. The
verifier executes your controller on **four fresh private scenes** as the
unprivileged agent user. Maximize collision-free completion rate across all four.
A crashed or malformed episode scores zero and stays in the average.
The report also records ordered gates passed and their fraction of the four-gate
course, so partial progress remains visible even when full completion is zero.

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
