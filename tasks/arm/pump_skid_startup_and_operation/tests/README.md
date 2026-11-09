# pump_skid_startup_and_operation: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the skid is described in `environment/setup/payload/given/sop.md`,
`equipment_manual.md` and `pid.png`, the cell in `given/cell/cell_manual.md` and its wire in `cell_api.md`. This page
records the design decisions, what the grader checks and why, and what is known to be rough in version 0.5
(2026-11-07; version 0.3 by Yuanfang Peng, 2026-11-06; the versions are described in section 6).

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
  frictions; lever lengths), the fitted gauge scales (`textures.GAUGE_KINDS`), the previous shift's leftovers (2-3
  valves out of their line-up position; the agent's documents say only that some may be), the process parameters
  (including the strainer chambers' fouling at the start and the supply tank's level) and the window's events (v0.5:
  always a demand step plus 3 of the five faults filter_slug, supply_loss, dosing_airlock, drum_empty, pump_trip). The
  schedule is drawn as a whole by rejection: the events at least 120 s apart, each early enough that its judgement falls
  inside the window (the demand step by 1140 s, the 300 s procedures by 1200 s, the slug by 1050 s), no pump trip inside
  the 360 s that judge the demand step, and with both dosing faults the air lock after drum A ran dry (an air lock in
  drum A's last few percent would look like AOP-4 on every instrument).
* `devices.py` / `textures.py`: MjSpec builders; textures drawn with Pillow's built-in font into in-memory MuJoCo
  textures (dial faces, tag plates, scales, labels). Every operable hinge turns about -y (counter-clockwise as the
  operator sees it opens); the instruments' needles, the rotameter float, the rising stems are kinematic bodies set from
  the process before a frame is rendered (`Scene.update_visuals`); the level gauges' flap columns are resized geoms; the
  lamps, sight glasses and 7-segment segments switch colour.
* `process.py`: a lumped model updated every 10 ms tick: the pump curve against the suction losses (strainer, V-101)
  and two discharge branches (the product line with V-105 and its bypass V-107 in parallel; the minimum-flow line with
  its orifice), solved for the total flow by bisection; air in the casing (vent to prime), cavitation (NPSHa < NPSHr:
  head loss, needle flutter, a cumulative timer), deadheading, dry running and motor overload trips, the duplex
  strainer (fouling, changeover, the standby chamber's air; a debris slug lands in the chamber in service when it
  comes, and each change-over is recorded with the air the incoming chamber held), the tanks (a float-valve supply to T-101 that can stick
  shut, the make-up bypass V-106, T-101's overflow; the consumer's draw from T-201 from 600 s), the dosing drums (A
  duty, B standby, the selector V-303; an empty drum air-locks the pump, venting needs liquid at the suction), the
  chlorine residual in T-201 (well mixed) and the analyser's 20 s lag; the control station's latches, permissive, the
  common alarm, the fault lamp and the horn.
* `physics.py`: the scene (skid, pipes, tanks, the local control station, the Panda on a rail carriage with a lift),
  the grip rule (a weld retargeted at run time, as in the shoe cell; slip at 60 N / 6 N m for 30 ms), the button and
  selector readings, the contact classes of the safety watch. MuJoCo 3.14, timestep 2 ms, implicitfast, Newton,
  elliptic cones; a 10 ms tick = 5 steps; about 30x real time without contacts on one core.
* `cameras.py` / `look.py` (v0.5.3): the agent cameras. The colour comes from MuJoCo's Filament renderer (physically
  based, Vulkan on Mesa's lavapipe, CPU) on a render twin of the cell compiled from a copy of the cell's MjSpec: the
  skid with physically based materials and the cell's own colours and plate textures, the skid's back wall carried on
  as the room's lining wall, the water-treatment room and its props of the approved OVRTX demo scene (static decor
  meshes, nothing within the skid or the rail), the two ceiling panels as shadow-casting spot lights and an image light
  distilled from an OVRTX probe of the room; the light strengths and per-class colour gains were fitted in linear light
  to OVRTX stills of both cameras on 14 states of a development shift (within 1 % on the fitted states for most
  classes, within 3-10 % on 6 held-out states). Each frame is drawn at twice the camera's size and averaged down in
  linear light (the analyser's 1.1 mm strokes stay legible in the overview, as with the classic renderer's
  multisampling). The depth comes from the classic renderer on a depth twin: the cell's own geometry plus the room's
  meshes within 8 m. Both twins copy the physics qpos and the instruments' changing parts (lamps, segments, sight
  glasses, flap columns, drum liquids) at every frame; the physics model and every random generator are untouched.
  The site's light factor scales every light as before. Frame time on 8 CPUs: about 1.0 s for the overview (colour and
  depth), 0.4 s for the wrist, 3 s for an episode's first frame; eight episodes in one process peak at 2.8 GB.
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
(v0.5: two rounds, 600-1050 s and 1050-1500 s; in each, each of six instruments' first entry made while P-101 runs,
by its site tag, within 1 % of the instrument's span of the true value (AI-201 0.05 mg/L), any of the accepted units;
`log_reading`'s reply follows from the clock alone). Voids (score 0): deadheading 30 s, 300 s of cavitation, T-201 or T-101 at 95 %, the
emergency stop, a safety violation (robot vs skid > 40 N, vs a handle or button > 150 N, for 3 ticks). Every check
reads the cell's own state and the process's true values; nothing the program sends is scored except its log entries.

Diagnosis is built into the faults' consequences: the strainer slug and the stuck float valve both raise the common
alarm and pull the suction pressure down; the dosing air lock and the empty duty drum both stop the dosing with P-301
running, and the SOP's AOP-3 counts only when dosing resumes from the drum that was lined up, AOP-4 only for a
changeover made once drum A is below 5 % (a blanket "change drums and vent" answers one of them, not both). Pre-emptive
moves earn nothing (v0.5, review findings 53-scoring-2 / -4): the slug clogs whichever chamber is in service, so a
change-over before it is undone by the slug and AOP-1 counts only a change-over away from the clogged chamber after the
slug came; AOP-5 counts only a V-106 opened after the float valve stuck (the float valve throttles, so a make-up below
the transfer does not overfill T-101 and opening it beforehand would otherwise be free). AOP-1's "vented" credit reads
the air the incoming chamber held when it came into service, before it went into the pump (53-scoring-3).

Network (v0.5): every process of the program, the import check included, runs under the seccomp filter of
`cellkit/nonet.py` (socket() for anything but AF_UNIX fails with EPERM, io_uring and foreign syscall ABIs are refused),
proven by a self-test before the first episode (a failure is a task error). The gate counts only the runner account's
remote inet sockets; the namespace-wide counters of `netaudit.NetCounters` (which also count loopback and every other
account) are kept as statistics. An episode in which the cell failed (an internal error, a physics fault, a frame it
could not render after one in-call retry; such a frame is not charged to the budget) is run once more from scratch and
the second run counts; a render failure that repeats is a task error.

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

v0.5 changes to the reference: a grip that slips before the device moved is retried on another handle (a 3-spoke
handwheel at its closed stop slipped on the same spoke 30 times and never opened, which voided some sites by
deadheading); the line-up is checked and corrected before the start; both log rounds are logged; AOP-1 changes over
from whichever chamber is in service.

**The fixed hidden set (v0.5).** `hidden/seeds.json` pins the eight graded sites; `hidden/selection.json` holds the
rule, written down with the 24 candidate seeds (drawn from `os.urandom`) and the sha256 of every file of the frozen
reference and cell before any candidate ran, and the selection run: the reference ran each candidate once, as the
grader drives the oracle; a candidate counts as completed when there is no void, start-up, shutdown and log sheet are
full and all four events are judged and credited in full (the operation part, the in-band fractions, is what the anchor
measures). 20 of 24 completed; the hidden set is the first eight (candidates 1-8; candidate 0 was dropped, the demand
step not matched in time). The eight cover all five faults (air lock 5, slug 5, pump trip 5, drum 5, supply loss 4);
the hollowing check (selection.json) finds no feature the drops removed. One full verify of the oracle on the set
(`anchor.json`) reproduced every episode's selection score to four decimals: anchor 0.9779, `full_at` 0.98. A camera
program's reading of the instruments: `review/observability.md`. The frozen files changed after the selection only as
recorded in `selection.json`: the log tolerance (`post_selection_changes`) and the v0.5.3 amendment (`amendments`: the
agent cameras' renderer, with v0.5.2's visual-only carriage geometry), whose check, the reference's full verify of the set
on the amended image, repeated every selection run (score and parts of each episode to four decimals; two runs, bit for
bit the same).

## 5. Known limitations (version 0.5)

* The process is a lumped, quasi-steady model; the instruments show its true values (with flutter under cavitation),
  not sensor dynamics beyond the analyser's lag.
* Valves are turned by the grip rule (a weld), not by friction between fingers and rim; the gripper's force does not
  matter once the rule engages.
* The P-301 / P-101 interlock, the trips and the alarms are the minimal set the procedure needs.
* The fixed overview camera cannot resolve tag plates (about 6 px tall there) or a dial's pointer to 1 % of span (a
  dial is 22-25 px across); programs read those through the wrist camera, which costs time and frames and competes
  with the valve work for the one arm. The overview does show more than v0.5's manual said (measured on 20
  development sites, v0.5.2): the analyser's digits are about 4 px tall and legible in an enlarged crop, FI-105's float
  moves along a scale about 67 px long and LG-201's flap column along one about 245 px long, so a program can follow
  the chlorine and the product level from the overview alone; the cell manual now says so.
* The float valve throttles: a make-up opened while it works overfills T-101 only when it passes more than the
  transfer, so AOP-5's rule (an opening before the fault earns nothing) is a procedure rule, not a physical one.
* A program could still toggle the strainer back and forth blindly; each change-over back into a clogged chamber starves
  the pump, so this only adds cavitation time (a void at 300 s) and earns nothing a reading of PDI-111 does not.
* The hidden set is eight sites the reference completes; its in-band fractions (0.65-1.0 of the window) are what the
  anchor measures.
* Log entries made at a round's first moment on a calm process are still partly predictable: over the 24 candidates,
  the best single constant per instrument (fitted on the true values, which no document gives and no practice cell
  shows) would pass 38-96 % of the first round's entries made at 600 s and 17-38 % of the second round's made at
  1050 s (about 0.07 of the 0.15 log part in expectation, for a program that knew them; 0.09 at 2 % of span).

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
* v0.5 (2026-11-07, maintainers' review fixes and a harder hidden set; pump-skid-cell/0.5.0). Scoring: the debris slug
  lands in the chamber in service, AOP-1 counts a change-over away from the clogged chamber and reads "vented" before
  the air goes into the pump; AOP-5 counts only a make-up opened after the float valve stuck; AOP-2 is always judged
  360 s after the change and every event's judgement falls inside the window; the log sheet has two rounds at 1 % of
  span (a camera program reads the dials to a median 0.2-0.6 % of span, `review/observability.md`), and the start-up fouling and supply level vary per site so that constant entries no longer pass;
  `log_reading`'s reply follows from the clock alone. Isolation: the program (import check and episodes) runs under
  the no-network seccomp filter, the gate counts only the runner account's sockets, the import check's 120 s / 120
  CPU-s and the CPU cap are documented; a cell failure reruns the episode once; a frame the cell cannot render is not
  charged. Harder shifts: three faults on every shift, two or three valves left out of place, wider demand steps (0.6-1.0
  m3/h) and draws, events at least 120 s apart; 8 hidden episodes (1200 s program time and 600 frames each; verify 6 h)
  on a fixed set selected as above; anchor 0.9779 / `full_at` 0.98 (was 0.965 / 0.92 on fresh draws).
* v0.5.1 (2026-11-08, grader only): a process of the agent account that outlives the verify-start kill is recorded and
  frozen, no longer a contract failure. In the separate verifier nothing of the agent's runs, and the image's
  supervisord respawns the account's desktop as fast as it is killed: in the first Opus episode of v0.5 one respawned
  desktop process zeroed the submission before any episode ran (that submission was graded again with this grader).
* v0.5.2 (2026-11-08, visuals and documents; cell API pump-skid-cell/0.5.0 unchanged): the carriage's lift mast passed
  through the lift platform, the Panda's link0 and the folded elbow, and a low platform sank into the sled and the floor
  track. The track, the sled and the mast now stand 0.32-0.46 m behind the base and the platform hangs off a guide
  plate on the mast (all visual geoms, as before: the carriage collides with nothing); the carriage's and the lift's
  inertials are written out with v0.5's compiled values, so the compiled dynamics are bit-identical (two development
  shifts of the reference: the same state hash every 100 ticks and the same score as v0.5; the arm keeps at least
  66 mm from the mast). The cell manual's camera table now gives what the overview frame resolves (section 5). The
  hidden set, the anchor and the scoring are those of v0.5.
* v0.5.3 (2026-11-09, the agent cameras; cell API pump-skid-cell/0.5.0 unchanged): the owner approved the OVRTX demo
  of the shift and asked for its light in the agent's images. The overview and wrist cameras draw their colour with
  MuJoCo's Filament renderer in the demo's water-treatment room, lit by the two ceiling panels and an image light
  distilled from an OVRTX probe of the room, with per-class gains fitted to OVRTX stills of both cameras; the depth
  stays the classic renderer's on the cell's own geometry plus the room (`look.py`, section 2). Intrinsics, extrinsics,
  noise and encoding are unchanged; on every pixel where the old depth saw the skid, the robot or the wall the new depth
  is equal (to 1e-4 m), and only room geometry in front of the old floor or background differs (about 11 % of the
  overview's pixels, 0-8 % of a wrist frame's). What the overview resolves is unchanged (same geometry and camera: the
  analyser's digits about 4 px tall and legible in an enlarged crop, FI-105's scale about 67 px, LG-201's about
  245 px); the see-through parts blend with their own alpha, so the rotameter's float, the drums' hypochlorite and the
  sight glasses' liquid show as before; lit lamps and segments glow. The cell manual's camera section says what the
  images show. The image adds Mesa's lavapipe and the render bundle (`environment/assets.sha256`). The hidden set's
  amendment (`hidden/selection.json`) and its check: the reference's full verify on the amended image gave the
  selection's eight scores (0.9779). The anchor and the scoring are those of v0.5.
