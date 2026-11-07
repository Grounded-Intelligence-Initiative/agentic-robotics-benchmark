# pump_skid_startup_and_operation: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the skid is described in `environment/setup/payload/given/sop.md`,
`equipment_manual.md` and `pid.png`, the cell in `given/cell/cell_manual.md` and its wire in `cell_api.md`. This page
records the design decisions, what the grader checks and why, and what is known to be rough in version 0.3
(2026-11-06, Yuanfang Peng; earlier versions are described in section 6).

**Design rule (the owner's, 2026-11-06): the agent gets what an operator or automation engineer at a real site would
have, and nothing an operator would not.** It gets the site's documents (series P&ID, SOP, equipment manual, the
site's equipment list), the robot's model, the cell's API and calibration, an instrument catalog (datasheet drawings
of the dial faces and scales). Like a field operator taking over a shift, it does not rehearse on the site: there is no
simulator of the skid in its sandbox (version 0.4: the practice service of v0.3 is removed, section 6). It does not get
the simulator's code (the process model, the site generator with its fault schedule, the scoring code), any ground
truth (device poses, valve positions, process values), or how many faults a shift will bring.

## 1. The job and why it is worth measuring

A field operator's shift on a process skid: line up, prime and start a centrifugal pump, bring a product tank and a
chlorine residual into their bands, hold them while a consumer draws, handle the abnormal situations by their
procedures (a clogging strainer, a changed demand, an air-locked metering pump), keep a log sheet, and shut down in
order. Plants that run unmanned shifts (remote water and wastewater sites, offshore platforms) send rail-mounted or
legged robots to read gauges and check valve positions (the ARGOS challenge; rail inspection robots in utility
tunnels and substations); operating the valves by procedure is the next step and what this task asks. It is a
Physical Execution task: the horizon comes from the process (a 1800 s shift, a state machine of start-up, operation,
events and shutdown, every step conditioned on the plant's state), not from repeating a short task. Nothing of the
plant is on the wire: every episode is a different site of the skid series, numbered and built its own way, so the
agent's program finds the devices by their tag plates (legible only to the wrist camera) and the site's equipment list,
reads dial gauges, a rotameter, level gauges, sight glasses, drums, lamps and a 7-segment display from RGB-D frames,
diagnoses faults that share their first symptoms, and turns handwheels, levers and knobs through a grip rule.

## 2. The cell (`tests/skidcell/`)

Where it runs: only in the verifier (the grader imports `tests/skidcell/`); nothing of it is in the image or the
agent's sandbox. The agent's `~/given/lib/skidcell/` holds only `client.py` (the proxy, `decode_frame`, the launcher)
and its own `__init__.py`; its program meets the cell for the first time in the graded episodes. The grader's
`tests/skidcell/client.py` keeps the v0.3 client's text (with the practice connection, which the graded cell answers
`forbidden`); the wire, the bootstrap and the launcher are the same as the given client's.

* `layout.py` draws the episode's skid from its seed: the site (an area number and a random site tag for every
  device; `site["equipment"]` is the equipment list on the wire), the arrangement (left-to-right or mirrored, the flow
  meter before or after V-105, the control station's pump groups either way round, a few cm of jitter, the suction /
  discharge / make-up line heights), how each valve is built (V-101 / V-103 gate valve with a handwheel or ball valve
  with a lever, V-104 lever or small handwheel, the vents lever or needle valve; handwheel diameter, spokes, turns,
  frictions; lever lengths), the fitted gauge scales (`textures.GAUGE_KINDS`), the previous shift's leftovers (1-2
  valves out of their line-up position; the agent's documents say only that some may be), the process parameters and the window's events (always a demand step plus 2
  or 3 of the five faults filter_slug, supply_loss, dosing_airlock, drum_empty, pump_trip; >= 150 s apart).
* `devices.py` / `textures.py`: MjSpec builders; textures drawn with Pillow's built-in font into in-memory MuJoCo
  textures (dial faces, tag plates, scales, labels). Every operable hinge turns about -y (counter-clockwise as the
  operator sees it opens); the instruments' needles, the rotameter float, the rising stems are kinematic bodies set from
  the process before a frame is rendered (`Scene.update_visuals`); the level gauges' flap columns are resized geoms; the
  lamps, sight glasses and 7-segment segments switch colour.
* `process.py`: a lumped model updated every 10 ms tick: the pump curve against the suction losses (strainer, V-101)
  and two discharge branches (the product line with V-105 and its bypass V-107 in parallel; the minimum-flow line with
  its orifice), solved for the total flow by bisection; air in the casing (vent to prime), cavitation (NPSHa < NPSHr:
  head loss, needle flutter, a cumulative timer), deadheading, dry running and motor overload trips, the duplex
  strainer (fouling, changeover, the standby chamber's air), the tanks (a float-valve supply to T-101 that can stick
  shut, the make-up bypass V-106, T-101's overflow; the consumer's draw from T-201 from 600 s), the dosing drums (A
  duty, B standby, the selector V-303; an empty drum air-locks the pump, venting needs liquid at the suction), the
  chlorine residual in T-201 (well mixed) and the analyser's 20 s lag; the control station's latches, permissive, the
  common alarm, the fault lamp and the horn.
* `physics.py`: the scene (skid, pipes, tanks, the local control station, the Panda on a rail carriage with a lift),
  the grip rule (a weld retargeted at run time, as in the shoe cell; slip at 60 N / 6 N m for 30 ms), the button and
  selector readings, the contact classes of the safety watch. MuJoCo 3.14, timestep 2 ms, implicitfast, Newton,
  elliptic cones; a 10 ms tick = 5 steps; about 30x real time without contacts on one core.
* Two decisions came out of failures of the reference program and apply to every program: (1) the Panda's base rides
  a lift (0.05-0.60 m), because with the tool pointing horizontally into the skid the Panda reaches only 0.5-1.1 m
  above its base, and all operable handles sit in a 0.7-1.25 m operator band; (2) the arm controller's damped least
  squares uses lambda 0.01 (the shoe cell's 0.05 moved the near-singular wrist direction at a quarter of the commanded
  speed, so a held lever's weld load grew until it slipped). The operable parts are gravity-compensated: MuJoCo's
  friction loss is a soft constraint, and a lever's own weight crept a closed vent valve open over minutes.
* `episode.py`: the procedure's checks and the score (below); `server.py`: the API, the grip / panel logic in
  `physics_tick`, the camera budget and noise, the safety watch, the summary. `mode="practice"` additionally answers
  `reset` and `truth` (a development mode for maintainers only: since v0.4 nothing in the image runs it);
  `privileged=True` (the oracle token) answers `get_state_privileged` and `reference_run`.

## 3. What the grader checks (`episode.py`)

The episode score is the sum of five parts (`sop.md` section 6 states it for the agent): start-up 0.20 (line-up of
eleven valves at the first start, primed casing, a clean start, V-103 open in time, dosing at 600 s), operation 0.25
(the window's fraction with LG-201 in 55-75 % and with AI-201 in 1.8-2.6 mg/L), events 0.25 (split between the
episode's events; a slug that never raises the strainer alarm, a supply loss that never reaches T-101's low alarm and
a trip of a pump that was not running are not judged), shutdown 0.15 (the order of section 4), the log sheet 0.15
(each of six instruments' first in-window entry, by its site tag, within 3 % of the instrument's span of the true
value, any of the accepted units). Voids (score 0): deadheading 30 s, 300 s of cavitation, T-201 or T-101 at 95 %, the
emergency stop, a safety violation (robot vs skid > 40 N, vs a handle or button > 150 N, for 3 ticks). Every check
reads the cell's own state and the process's true values; nothing the program sends is scored except its log entries.

Diagnosis is built into the faults' consequences, not into penalties: the strainer slug and the stuck float valve both
raise the common alarm and pull the suction pressure down, and opening the make-up valve V-106 while the float valve
works overfills T-101 (a void); the dosing air lock and the empty duty drum both stop the dosing with P-301 running,
and the SOP's AOP-3 counts only when dosing resumes from the drum that was lined up, AOP-4 only for a changeover made
once drum A is below 5 % (a blanket "change drums and vent" answers one of them, not both).

## 4. The reference program and the anchor

`tests/reference/controller.py` + `skills.py`. The skills do the robot's work with privileged eyes (the devices' true
poses and joint angles) but drive it only through the cell's public calls, under the same grip rule, rate limits and
safety watch: grips planned with collision-checked IK (a stroke of up to 110 degrees per grip, re-grips in place),
valves turned compliantly (stiffness 0.4, 35 degrees/s in 3-degree increments, waiting for the arm to catch up: the
Menagerie servos lag a moving target by about 0.1 s).

The controller decides only on what an operator at the site can see (`Eyes`: the instruments' displays read
perfectly, the lamps and the horn, the sight glasses, the drums' visible levels, the valves' positions) and on series
facts that are documented or measured on the shift itself (the tanks' volume per percent of their gauges, which the
fill before 600 s shows against FI-105 with no draw yet, the analyser's 20 s lag, the alarm set points; the consumer's
usual draw, a typical dosing gain and valve gains as first guesses only). It never reads the process's hidden state (the consumer's draw, the pump's curve, the hypochlorite's
strength, the metering pump's capacity, the supply pressure, the fault flags, the event schedule). So it estimates the
draw from T-201's balance over the last 40 s (FI-105 in, LG-201's change), sets V-105 by trying against FI-105 (a
proportional first step, then a secant), learns the dosing gain from AI-201's response, primes and vents until the
sight glasses show liquid, and diagnoses by the SOP's table: P-101's RUN lamp out in the window -> AOP-6 (the make-up
throttled first if T-101 would otherwise fill past 85 % during the restart); PDI-111 above 0.45 bar (a little before
the alarm) -> AOP-1; LG-101 10 % below where the float valve held it and still falling, or its low alarm -> AOP-5
(V-106 set from T-101's balance, T-101 held at 60 %); drum A's visible level below 4 % -> AOP-4; AI-201 falling
faster than 60 % of what the transfer alone washes out while P-301 runs -> AOP-4 if the lined-up drum is empty, else
AOP-3. With one arm for everything it is sometimes late: a fault that strikes during another procedure (a trip while
it vents the dosing pump, a demand step while it changes the strainer) costs it credit on some seeds, as it would cost
any operator. Its eyes are perfect and instant and it never searches for a device, which a camera program pays for in
time and frames; the anchor is what a perfect-sighted operator achieves under the task's information rule. One episode
takes about 1.5-2.5 min of wall clock. The anchor's numbers and their source are in `anchor.json`; until the v0.3
revision the reference read the process's hidden state (section 6).

## 5. Known limitations (version 0.3)

* The process is a lumped, quasi-steady model; the instruments show its true values (with flutter under cavitation),
  not sensor dynamics beyond the analyser's lag.
* Valves are turned by the grip rule (a weld), not by friction between fingers and rim; the gripper's force does not
  matter once the rule engages.
* The P-301 / P-101 interlock, the trips and the alarms are the minimal set the procedure needs.
* The fixed overview camera cannot resolve tag plates, dial scales or the analyser's digits (checked on renders: the
  digits are about 4 px tall there); programs read through the wrist camera, which costs time and frames and competes
  with the valve work for the one arm.

## 6. History

* v0.1 (2026-11-05): one fixed arrangement up to mirroring, series tags on the plates, three fixed cameras, two fault
  types (strainer slug, dosing air lock) with their own alarm lamps. Opus 5.5 (Claude Code, effort high) scored reward
  1.0 (operation_score 0.972 over 4 hidden episodes) in 2 h 42 min: its program hard-coded the skid from the public
  layout code (only the mirror flag was perceived), read the level, the flow and the chlorine from the side cameras and
  had the device's tag in every `grasped:<tag>` event.
* v0.2 (2026-11-06): the hardening against those shortcuts (sites with their own tags and builds, the equipment list,
  one overview camera, small plates and digits, tagless grip events, five faults with a common alarm and shared
  symptoms). The oracle's numbers are in `anchor.json`.
* v0.3 (2026-11-06): the owner's realism rule (top of this page): the simulator's code and `truth()` left the agent's
  sandbox; development goes through the practice service (a black box under its own account) and its debriefs; the
  instrument catalog replaces the drawing code; the instruction no longer says how many faults a shift brings. The
  graded cell, the scoring and the reference are those of v0.2.
* v0.3 revision (2026-11-06, after a pull-request readiness review): the reference decides on what an operator can see
  only (section 4; its anchor re-measured, `anchor.json`); the image installs the practice service and setup restarts
  it should it ever exit; the equipment manual's section 6, the SOP and the instruction no longer read like the
  generator's parameters (no count of the valves left out of place, the process varies as on any plant); the SOP's
  AI-201 fall rate when the dosing stops corrected (0.2-0.5 mg/L a minute, not 0.1). The graded cell and the scoring
  are unchanged (pump-skid-cell/0.3.0). The grader now waits, before the first episode, until the frozen desktop is
  quiet (`verify._settle_desktop`): the image's supervisord respawns the agent account's desktop after the verify-start
  kill, and a respawned process opened one loopback TCP connection about a second later, which two validate runs
  counted in their first episode as the program's network activity (contract_ok 0 for a correct oracle).
* v0.4 (2026-11-07, review): the practice service is removed (environment/practice_service/, the image's `skidcell`
  account, setup's service loop, `~/practice/grade.py`, the client's `connect_practice` / `reset` / `debrief`, the
  instruction's Practice section and the manuals' mentions). A field operator of the series is expected to run a shift
  on a site from the SOP, the P&ID, the equipment manual and the site's equipment list without rehearsing it on a site
  simulator, and the measured round with the service showed it gave the agent a large advantage. The graded cell, the
  scoring, the reference and the anchor are unchanged; the givens' hashes follow the edited documents and client.
