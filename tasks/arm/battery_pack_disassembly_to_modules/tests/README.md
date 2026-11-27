# battery_pack_disassembly_to_modules: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the pack series is described in
`environment/setup/payload/given/service_manual.md`, the tools in `tool_catalog.md`, the cell and its wire in
`cell/cell_interface.md`. This page records the design decisions, what the grader checks and why, what is known to be
rough, what was and was not verified, and a dated status log (version 0.1.0, 2026-11-24; built in one unattended
session on a server without Docker, so the engine's own gates are still owed: sections 6 and 7).

**Design rule: the agent gets what the automation engineer of a real disassembly cell would have, and nothing
else.** It gets the pack series' workshop manual (drawings, nominal positions, the torque table, the isolation
procedure), the tool catalog with the tools' tolerances, the cell's interface document (the cameras' exact
intrinsics, their nominal mounting and how far a cell's own may be off it, the depth sensors' data sheets, two
surveyed calibration plates), the robot's model, and a commissioning service that runs the cell on development
packs. It does not get the cell's code,
any ground truth (a part's pose, a fastener's head, state or wear), a credit or a debrief of any session, or a
rehearsal on a graded pack.

## 1. The job and why it is worth measuring

Taking traction battery packs apart to module level is the step that recycling and second-life lines still do
largely by hand: packs arrive in many variants, dirty, corroded, with fasteners replaced or missing, and under
voltage. A robot cell for it needs what manipulation benchmarks usually leave out: a long procedure with ordering
constraints that come from the product (nothing live may be touched before isolation; a part comes off only when
everything that holds it is out), tool and bit exchanges, fasteners that punish a wrong bit or a sloppy alignment,
and perception that has to deliver millimetres (a Torx bit engages within 1.2 mm; holes are within 2 mm of nominal,
and the pack lies within 12 mm and 1.5 degrees on its fixture). Nothing of the pack is on the wire: the program
identifies the variant, locates every fastener, tells a hexagon from a Torx head, notices rust, missing fasteners
and a harness remnant from RGB-D frames, and reads the nutrunner's torque and angle to know whether a fastener
turns, stalls or slips. It is a Physical Execution task for the `arm` platform; nothing among the arm tasks covers
disassembly.

## 2. The cell (`tests/packcell/`)

Where it runs: in the verifier (the grader starts `packcell/server.py`, one process per pack, as the image's account
`packcell`) and in the agent's sandbox as the commissioning service (below), from a byte copy that only that account
can read. One process is one pack: Isaac Sim 6.2 headless, PhysX on the CPU (`enableGPUDynamics` false, TGS solver,
120 Hz, six physics steps per 0.05 s control tick), the RTX renderer on the GPU for the two cameras only.

- **`series.py`**: the cell's geometry, the tools, the fastener catalog, the three variants and `draw_pack(seed)`:
  a pack's hidden differences from a 64-bit seed through a BLAKE2b counter stream. Per pack: the variant; the
  placement on the fixture (uniform within 12 mm and 1.5 degrees); for every fastener an offset within 2 mm, a head
  (the group's head, replaced by the other head of the same thread with probability 0.12), corroded (lid 0.15,
  busbar 0.08, bracket 0.12; breakaway 1.6 to 1.9 times the assembly torque) or missing (lid 0.06, bracket 0.05);
  a harness remnant (none, on the lid over one or two lid bolts, or along the busbars over two or three bolts).
  Two more draws come with a set's entry, each from a key of its own (`review/select_set.py`): the camera offsets of
  the cell a pack is taken apart in (rule 2) and a seed for the dabs of sealant over some heads (rule 3, `SEALANT`).
- **`scene.py`**: the USD stage (pxr only): the UR20 of NVIDIA's asset library, referenced unmodified, with two
  links added to its articulation: `ee_link` (the tool carrier, fixed to the flange) and `spindle_link` (the
  nutrunner's spindle on a prismatic joint with a spring drive: the 25 mm float of a real nutrunner). Both tools'
  bodies are children of `ee_link`; the cell shows and collides the mounted one and sets the link's mass. The pack's
  parts and fasteners are rigid bodies, kinematic until the rules free them.
- **`rules.py`**: what is not left to PhysX (next list). NumPy only; no simulator needed to read or test it.
- **`cell.py`**: the runtime: Cartesian and joint moves through damped-least-squares inverse kinematics on the
  commanded joint state (`kin.py`), smooth start and stop, speed limits; the external wrench at the flange from the
  articulation's incoming joint force on the tool carrier, the tool's own weight taken out; protective stops; the
  cameras (Replicator render products with pinned RTX settings; depth is `distance_to_image_plane`); the
  observation; `truth()` and `score()` for the grader only.
- **`sensors.py`**: what the cameras' depth channel makes of the renderer's exact depth (below). NumPy and OpenCV
  only; `review/test_sensors.py` holds it to its data sheet without a simulator.
- **`wire.py`**, **`server.py`**, **`cellproc.py`**: the action's schema, the cell as a process, and the client the
  grader and the service start it with.

**The cameras: a sensor's depth and a cell's own mounting (since 2026-11-25).** Until then the depth frames were the
renderer's exact distances and the cameras stood exactly at their published poses. A perception baseline written in
about an hour typed every head by its height above its seat in one overhead frame and located everything through
the published calibration, and scored 1.0 on the hidden set (section 6, the entry "2026-11-25, 10:00 to 11:35"). A
real cell offers neither shortcut. Two changes, both perception and engineering, neither a penalty:

- *The depth is an active-stereo sensor's* (`packcell/sensors.py`; data sheet `series.SENSORS`, stated in the
  interface document's section 5 the way a data sheet states it; also in `cell.json`). The renderer's exact depth
  goes through a lateral smoothing of 1 pixel; temporal noise of sigma_k z^2 (overhead 0.0009 per metre: 2.5 mm at the
  fixture, 1.65 m away; wrist 0.0025: 0.3 mm at 0.35 m), correlated over about 2 pixels, independent from frame to
  frame, up to twice as large on dim surfaces; quantization to the disparity grid (1/16 pixel at a 0.10 m and a
  0.018 m baseline: steps of 1.0 mm at the fixture, 0.3 mm at 0.35 m); and no reading outside the working range, at
  depth jumps of more than 2 disparity pixels, in the stereo pair's occlusion band beside tall edges, on dark
  surfaces (luma 40 down to 12) and on specular highlights (a colour channel 215 up to 245), lost in blobs. The sizes
  are those of industrial active-stereo cameras at these distances, a long-range model overhead and a short-range
  one on the wrist. The dark and bright dropouts are keyed to the rendered colour image (a sensor's own infrared
  image is not rendered); the stated thresholds let a program predict where it will have no reading. The colour
  image is unchanged. What this does to the job (`review/test_sensors.py`; section 6): one overhead frame types
  about a third of the heads wrong even when the head's centre is known, 32 averaged overhead frames or one close
  wrist frame type them right; the plated heads lose part of their top face's readings in a wrist frame (their
  highlights), so a centre from depth alone is up to about 1.6 mm off and the colour image's outline is what brings
  it within the Torx bit's 1.2 mm.
- *The cameras' poses on file are nominal* (as upstream #177 did for `lab_sample_handling`). A cell's overhead
  camera sits off its nominal pose by up to 15 mm and 1.5 degrees per component, its wrist camera off its nominal
  place on the flange by up to 4 mm and 1 degree (`series.MOUNT_TOL`, an offset in the camera's own frame: true =
  nominal times offset). Located through the nominal poses, a point on the fixture is typically 4 cm off (up to
  8 cm), a point 0.35 m in front of the wrist camera 6 mm (up to 13 mm). Every pack is taken apart in a cell of its
  own, graded and development packs alike, so a program identifies its cameras in every pack. What it identifies
  them with is on file and exact: two checkerboard plates on the fixture's strips that no pack covers
  (`series.CAL_PLATES`, 5 x 20 squares of 10 mm, surveyed; drawn only, nothing collides with them), the intrinsics,
  the robot's kinematic model and its joint readings. A wrist frame's `pose` on the wire is the nominal pose (the
  flange's pose times the nominal mounting), which is how `wire.perturbed` and the agent's library build a probe's
  copy too. The offsets were drawn under `review/select_set.py`'s rule 2 with a key of their own, after the rule was
  committed. Measured with standard OpenCV on a probe cell with hand-picked offsets (section 6): the two plates in
  one overhead frame give the overhead camera to 0.3 to 0.6 mm on the lid, and five whole-plate wrist views give the
  wrist camera's mounting to 0.2 mm.

**Sealant partly over heads (since 2026-11-25, the third lever).** With the two changes above, a revised
perception baseline still took three of four development packs apart completely after about an hour and a half of
work (section 6): a close wrist frame's depth gives a head's outline, its centre and its type. A resealed field pack
breaks that: about a third of the lid's bolts and a quarter of the others carry a dab of grey sealant
(`series.SEALANT`; service manual, section 7), a dome standing on the seat with its centre 0.3 to 0.9 head radii off
the axis, 0.6 to 1.0 head radii wide and 1 to 2.5 mm above the head's top. It covers a median 22 % of a head's top
(at most about half) and runs over the edge onto the seat, so the region that stands above the seat is the head and
part of the dab: its centre is a millimetre off in the median and up to 3 mm, more than a Torx bit's 1.2 mm, and part
of a Torx recess may be hidden. A program has to tell the sealant from the head (its matte grey against plated zinc
or rust, its round dome against the head's flat top and straight or circular edge) and fit the head's known outline
to what remains. It is drawn only (section 2's list); the bit's rules are unchanged. Drawn per pack under rule 3 of
`review/select_set.py` with a key of its own, after the rule was committed; `review/test_sensors.py` checks the
draw against the manual's statement.

**A pack's look (since 2026-11-25, the fourth lever).** With the sealant too, the revised baseline took all four
development packs apart completely after about three hours of work in all (section 6): a program that has found its
way through calibration, noise and sealant is then left with fixed colours (a dark hole is dark, a plated head
bright, the harness's blue a known blue). A recycling line does not offer that either. Per pack (`series.APPEARANCE`,
rule 4; interface document section 5, service manual section 7): the hall's light has a brightness 0.6 to 1.3 times
the nominal and a colour temperature of 3500 to 6500 K, and the four lamps differ by up to a quarter; 4 to 9 patches
of dark grime or light dust (30 to 140 mm across) lie anywhere on the lid, over bolts too; 1 to 4 white service
labels with a printed bar code are stuck to the lid's flat areas, clear of bolts, handle and ribs; about 30 % of the
clean zinc heads carry a dull film of dust. All of it is drawn only, on the lid or its bolts. The depth does not follow
the hall's light: `sensors.measure` takes the colour image back to the nominal light before it decides about dark and
bright surfaces (an active sensor lights the scene itself), and the interface document says so. A program has to
read colours relative to their surroundings, under a light it does not know.

**The second round (since 2026-11-26): seized fasteners, swollen modules, foam pads.** One Claude Opus 5.5 episode
on the four-lever tree scored reward 0.84 (`mean_pack_credit` 0.795) in 70 minutes of its six hours (section 6, the
entry "2026-11-26, from 07:52"): every lever of the first round was one identification step with a documented
recipe, and it found and ran each in minutes (plates, ribs, a close wrist frame per head, the torque trace). It ran
every fastener at the "highest safe torque", lifted every module, and never ran its blind path for a fastener it
could not see. The second round asks for what that episode did not have to master: closed-loop work where the plain
strategy is irreversible, a two-sided judgement on a sub-millimetre measurement, and work on fasteners no camera
sees; all three are stated in the documents as a service manual states them, each drawn under a rule of its own
committed before its draw (`review/select_set.py`, rules 5 to 7):

- *Seized fasteners and the shank's strength* (`series.SEIZED`, `series.WORK`; rules only). A thread has a shear
  torque (M6 20, M8 46 N m): a fully engaged bit that holds it either way twists the shank off; the head rides out on
  the bit and the stub holds its part for good (`rules._shear`). Clean and corroded fasteners break away below it.
  The head's limit now holds when loosening too, and a Torx bit cams out at a stall when it is not pressed hard
  enough for the torque it holds. A development pack has one seized fastener, a hidden pack two to four (any group,
  a bracket's twice as likely, clean-looking or rusty as drawn): its breakaway is 1.15 to 1.6 times the shear torque,
  and it comes loose only by working (a loosening stall at 85 % of the shear torque or more for 0.5 s, then a
  tightening stall at the assembly torque or more for 0.25 s; each cycle multiplies the breakaway's excess over a
  clean one's by the fastener's factor, 0.55 to 0.80; one to five cycles). The episode's program runs M6 at 20.5 N m:
  it now shears every seized M6 fastener (a seized lid bolt keeps the lid, and the pack is lost).
- *Swollen modules* (`series.SWOLLEN`; a drawn-only bump, rules). Every module's top plate carries a smooth bump
  h (1 - rho^2)^2: a healthy one's crown of 0 to 0.25 mm, a swollen one's bulge of 0.6 to 3.0 mm; the criterion is
  0.4 mm above the plate's rim. Two development packs and four hidden packs carry one. A swollen module stays in its
  seat (freed of its brackets and busbars it counts 8); moved 5 mm it vents and the pack ends (`rules.vent_check`,
  checked every tick); a healthy module left in the tray counts nothing. The overhead camera's noise (2.5 mm a
  pixel) and the handle strap over the bump's middle make it a measurement for the wrist camera close by and a
  plane fitted to the plate's rim. The episode's program lifts every module.
- *Foam pads* (`series.FOAM`; drawn only). Some lid and busbar fasteners (development packs 1, 0, 1, 0; hidden
  packs 2, 1, 1, 2, 0, 1) are under a pad of soft foam, 40 to 60 mm across, 10 to 14 mm high, glued to the part the
  fastener holds: the head, or the empty hole of a missing fastener, is invisible to both cameras, and the bit goes
  through the pad. Where the fastener is (its nominal place, within 2 mm), whether it is there and which head it has
  are found by touch (the spindle's stroke where the bit rests: a head's height above the seat) and by trial (a bit
  that does not fit turns freely). Four of the six hidden packs have a pad over a lid bolt.
- *Composition.* The development packs show each condition, not the combinations and numbers of the hidden packs
  (rules 5 to 7 fix how often each appears in each set; the documents state ranges that cover both).
- *Not used, with reasons:* a wider fixture tolerance (a lid-outline fit meets any placement: one more identification
  step), adhesive on the lid or modules (a force ramp is a recipe, and the cell's protective stops already teach it),
  pre-damaged heads (little at stake: the extractor frees the part), bit wear (a counter is a recipe).

**The third round (since 2026-11-26, evening): what stays hard when the procedure is known.** A fresh Claude Opus 5.5
episode on the second round's tree scored reward 0.97 (`mean_pack_credit` 0.926) in 75 minutes (section 6, the entry
"2026-11-26, from 19:00"). It took the manual's working cycle as a fixed recipe, the bit order as a free trial, the
nominal places as a map that a 2 mm socket tolerance made exact enough, a test turn on a clean trace as a free judge
of engagement, and it skipped whatever would not come out at no other cost. Every lever until then was a rule with a
recipe. The third round asks one question of each lever: when an expert knows exactly what to do, is it still hard to
do well? It combines four of them, each a physical or facility consequence with a line in the documents and a
precursor that can be observed, with conditions of the job taken from the merged arm tasks (rules 8 to 12 of
`review/select_set.py`, committed before their draws):

- *Rule 8, precision engagement, damage at once.* The 10 mm socket drives within 0.8 mm (was 2.0), the 13 mm socket
  within 1.2 mm (amended after the fresh-agent proxies' hidden runs: section 6), a Torx bit within 0.5 mm (was 1.2),
  all within 2 degrees (was 3). A bit that is on a head without driving it (outside those, one size off,
  a Torx bit pressed too lightly), run at a torque limit of 1 N m or more, strips the head at once; below 1 N m the
  spindle only stalls, the same reading as on a head it drives, so no reading tells a partial engagement before it
  is too late. A reamed Torx recess takes no extractor (its part stays held); a rounded hexagon still does. With holes
  within 2 mm of nominal, every head's centre has to be measured to sub-millimetre (a close wrist frame, the outline
  fitted where sealant does not hide it, the camera's own mounting known to 0.2 mm), or found by touch under a pad.
- *Rule 9, every shank its own, read through a real transducer.* A corroded or seized fastener's shank has lost
  section: its shear torque is 70 to 100 % of the thread's, drawn per fastener, invisible from outside; it yields at
  82 to 88 % of its own shear torque (86 to 92 % until the relaxation of 2026-11-27, section 6). The angle reading
  winds up with the torque held (1.2 to 1.8 degrees per N m on M6, 0.6 to 0.9 on M8, per fastener), and past the yield
  point six times as fast, a set that stays. The torque reading carries 0.08 N m + 0.5 % of noise per sample, the angle
  0.1 degrees (twice that until the relaxation). Working a thread loose needs a loosening stall at 75 % or more of its
  own shear torque (80 % until then): between that and its yield point lies a window of 7 to 13 % of its shear torque,
  and its shear torque lies 14 to 22 % above the yield point; only a measured yield point finds them. A fixed limit is safe for no fastener of unknown strength (the episode's 19.5 N m shears
  every weakened M6 shank whose breakaway lies above its strength, and many corroded ones now do).
- *Rule 12, a cell's own nutrunner.* Each cell's torque transducer reads 0.88 to 1.12 times the torque plus -0.3 to
  0.3 N m, and the spindle stalls where that reading meets the commanded limit: the manual's torques are torques on
  the bit, and a limit commanded from the table holds a different torque in every cell. A development cell shows what
  the cells are like, not what a graded cell's values are (the sentence `arm/lab_sample_handling` uses). What
  calibrates a cell on site: the free-running drag (0.15 N m) and a fastener running out (12 % of its assembly
  torque).
- *Rule 10, consequences, not deductions.* Four safety events condemn the pack, the facility's rule for a possible
  thermal event (electrolyte and hydrogen fluoride from a damaged cell may have reached what was sorted from it): a
  tool on a live part before isolation, a loose fastener at a busbar still in its seat (a short, before isolation or
  after it: a module keeps its own voltage), a module that falls, a swollen module moved off its seat; a condemned
  pack counts 0. A freed fastener rides out on the magnetic bit only when drawn straight out of its hole first (15 mm
  for M6, 20 mm for M8, before 1.5 mm sideways) and carried gently (the rigid tool centre point under 4 m/s^2, the
  tool axis within 25 degrees); otherwise it falls where it is. The third protective stop in a pack latches the cell
  and ends the pack (an unattended cell needs a person to reset it), keeping what was sorted. The reference makes none
  of these events (section 4).
- *Rule 11, the swollen decision at the criterion.* Bulges 0.45 to 0.80 mm, healthy crowns 0.10 to 0.33 mm, the
  criterion 0.4 mm unchanged (they were 0.6 to 3 mm and up to 0.25 mm). Telling them apart needs the documented shape
  fitted over the rim plane from close wrist frames; a wrong lift vents (condemned), a wrong leave fails the gate.
- *The completion gate* (with the study of the merged arm tasks, 2026-11-26, 19:45; all seven studied tasks have one:
  "A run that does not count scores 0, whatever else was done"). A pack counts only when it has been taken apart to
  module level: every healthy module standing in the rack and a swollen one freed in its seat. A pack with a module
  still held, or anywhere else, goes to the shredder whole and counts 0. Inside a pack that counts, the credit is the
  share it was. The round-2 agent's habit of skipping what would not come out (0.86 to 0.97 a pack) now costs the pack.
- *Conditions of the job* (the user's answer of 2026-11-26, 19:27, to the starting session's question; precedent
  `arm/kitchen_appliance_tending`, 2 h and one trial never refunded, and `arm/lab_sample_handling`, 2 h and a
  commissioning budget on units that differ from the scored ones): three commissioning sessions in all, never
  refunded, fewer than the four development packs; the agent's session 2 h (was 6 h); the development packs and cells
  differ from the graded ones in stated ways (combinations and counts, every cell's own camera mounting and
  transducer). The simulated-time allowance per pack is set at about 1.3 times the reference's slowest hidden pack
  (section 6), as `arm/keyed_nut_peg_assembly` set its takt against its reference's pace.
- *The documents keep facts and lose procedures* (`arm/keyed_nut_peg_assembly` took the method out of its data
  sheet): the working cycle is stated as how a thread's bond responds, not as numbered steps; "look at every head",
  "raise the limit", "tell the variant from", the touch method under pads, the trial order of bits and the hint to
  calibrate the cameras from the plates are gone; every number an expert needs stays.
- *Not used, with reasons:* the pack shifting in its fixture or a knocked camera (moving every kinematic body of a
  pack mid-run is a risk this deadline does not allow; a careful program re-measures each head before it engages
  anyway), revision layouts not on file (they would make a padded fastener unknowable), adhesive peel and lift-tilt
  jamming (a force ramp and a vertical lift are recipes an expert runs without difficulty), fewer development packs
  (the conditions above already make practice scarce).

**What is rule-based (the stated simplifications).**

- *Fasteners are rule-based constraints.* A fastener's body sits on its seat (its head is a real collider the bit
  presses on, through the spindle's spring); it turns only by the rule: the mounted bit is the head's own, the bit's
  axis is within 2.0 mm (socket) or 1.2 mm (Torx) and 3 degrees of the fastener's, the axial force (spindle stroke
  times spring rate) is at least 8 N (Torx: 10 N + 1.5 N per N m), and the torque limit reaches the fastener's
  breakaway torque; then loosening turns accumulate and after 7 (M6) or 9 (M8) turns the fastener is out and rides
  on the bit until `eject` turns it into a falling rigid body (whose body is pushed out of an overlap at 0.3 m/s
  at most, so one let go into a pile settles on it instead of being thrown off: section 5). A loose bit (a 13 mm socket on a 10 mm hexagon, a
  T30 in a T40 recess), an engagement outside the tolerances (up to 3.5 / 2.5 mm, 8 degrees), a cammed-out Torx bit
  and a tightening run above the head's limit wear the head; at wear 1 it is stripped and only the extractor bit
  removes it. Since 2026-11-26 a fully engaged bit that holds the thread's shear torque (either way, the extractor
  too) twists the shank off (the head is captured, the stub keeps the part held, neither counts), a stall above the
  head's limit wears the head when loosening as well, a Torx bit cams out at a stall when the force is short of the
  torque it holds, and a seized fastener's breakaway falls only by working cycles (`rules._work`). Thread
  engagement, the fastener rising as it turns and the socket sliding over the head are not
  simulated: the bit's collider is a capsule whose round end stands on the head's flat top (a flat-ended
  cylinder on a head of nearly its own radius was thrown off sideways now and then: section 6), and that contact
  has a high friction (3.0, combine mode max), which stands in for the socket closing over the head. A part whose
  last fastener has just come out is freed only when the bit has lifted off it.
- *A part is held until what holds it is out.* The lid by all its bolts, a busbar by two, a bracket by one, a module
  by its two brackets and its two busbars (they have to leave their seats), the service disconnect and the plugs by
  a latch that gives at a pull of 60 N / 30 N along their axis. A held part is a kinematic body: pulling on it
  overloads the robot (a protective stop), it does not tear loose.
- *The grip is a rule.* Closing the fingers on a part's grip feature within the tool catalog's tolerances attaches
  the part to the tool carrier with a fixed joint (created at that moment, with the pose the part has: no snapping);
  opening removes the joint. The fingers are drawn but have no colliders; the gripper's body and the tool carrier do.
  There is no slip: a held part stays held until the gripper opens.
- *Exchanges are rules.* A tool is released or locked with the flange in the dock within 6 mm and 3 degrees at
  rest; a bit is exchanged with its tip in the bit station likewise. The parked tools and bits are drawn only.
- *Live parts.* Before isolation (the service disconnect out of its socket for 30 s) a tool point (the bit's tip;
  the gripper's centre point and fingertips) inside a busbar's box (3 mm margin) ends the pack. Checked once per
  control tick.
- *A dropped module* is a free, unheld module moving down faster than 0.7 m/s.
- *The harness remnant* is one rigid body (capsules along a polyline), kinematic where it lies until it is first
  gripped; a turning bit pressed on it marks it damaged.
- *A swollen module* (since 2026-11-26) differs from a healthy one only in its drawn bump and in two rules: its
  place is its seat (within 5 mm, level within 5 degrees, freed), and its body more than 5 mm from the seat ends the
  pack (`module_vented`). The bump has no collider; the handle and the grip are as on any module.
- *Foam pads* (since 2026-11-26) are drawn only (a prism on the lid or the busbar, a child of that part's body): no
  collider, no mass, the bit passes through; they leave with their part.
- *Sealant over a head* (since 2026-11-25) is drawn only: a dome mesh under the fastener's body, no collider and no
  mass, so a bit seats on the head as it would without it, and the dab leaves with its fastener. It is soft sealant
  in the manual's words; what it does is hide part of the head from the cameras (below).

Everything else is rigid-body physics: the arm with its position drives and torque limits, contacts of the tool
with the pack, the bins and the tables (which is what the wrench shows and what triggers protective stops), parts
carried on the grasp joint with their weight, parts and fasteners falling into bins and lying there, a module
standing in its bay. The robot's self-collision is off (the articulation's own links do not collide with each
other).

**The commissioning service (`environment/commission_service/`).** `commissiond.py` runs as `packcell`, started by
`environment/setup/setup.sh`, on `/run/packcell/commission.sock`: one session at a time, a session is one of four
development packs in a fresh cell process started without the grader's switch (the process then refuses `truth`
and `score`), ten sessions in all, counted in the account's own state folder. The development packs are drawn by
the same rule as the hidden set under another tag (`review/select_set.py`); none is a graded pack. The agent gets
`given/lib/packcell_client.py` (the connection) and `~/commission/run_session.py` (runs `run.sh` in a session with
the grader's deadlines and repeated steps, and prints what the program saw: the end, steps, frames, stops, the
cell's messages).

## 3. What the grader checks (`verify.py`, `pack_grade.py`)

- No `/home/user/submission/run.sh`: an honest zero, no simulator started.
- A canary cell on a pack outside the set: the simulator starts, both cameras render (shape, not flat, sane
  brightness, depth present), the robot follows a small move. A failure is a task error, never a zero.
- The fixed hidden set `hidden/packs.json`: six packs (each variant twice, each harness case twice), the same for
  every submission, drawn by `review/select_set.py` under a rule fixed before any pack was run; no pack was chosen
  or dropped for how the reference does on it. Each pack carries the camera offsets of the cell it is taken apart in
  (rule 2 of the same script, drawn with a key of its own); `verify.py` refuses a set without them.
- Per pack a fresh cell and the kit's `Solver` (the controller at the agent's UID through `nonet_exec.py`: no
  internet socket), one action per step until the pack ends. The probe lock (`IdleAwareProbeLock`, rate 0.02, pass
  fraction 0.1): a probed step is asked twice with the same `t`, once with the robot's readings shifted
  (`wire.perturbed`), in a hidden order; pairs in which neither reply commands a motion are not counted. The
  schedule is a function of the pack's index and seed, so the grader adds no randomness. The copy is the
  observation the cell would give with its joints shifted (up to 0.01 rad a joint, a whole number of the readings'
  last digit): the grader asks the cell for its angles before rounding (`joints`, a grader-only request) and writes
  the angles, both tool poses and a wrist frame's pose from them as `cell.observe` does; the wrench is left as it
  is. So a copy's poses agree with its joints as a true observation's do, and the robot's model, which the agent
  holds, does not tell them apart in a single observation (upstream #173; the measurement is in section 6); across
  steps it can (section 5). `probe_residual_true`,
  `probe_residual_copy` and `probe_true_residual_larger` of `probe_copies` in the verdict's metrics show it for
  every run.
- The credit (`rules.Rules.credit`): after the end and 1.5 s of settling, the value of the components that lie
  undamaged where they belong over the value of everything in the pack, from the bodies' poses in PhysX. A module:
  centre over a bay, bottom on the bay's floor (-5 to +15 mm), upright within 8 degrees. The lid: centre within
  0.10 m of the stand's centre, flat within 5 degrees, at the stand's height. A bin: centre over the inner footprint
  and at most 0.03 m above the walls. Not held by the gripper, not damaged (a stripped fastener, a chafed harness,
  a dropped module).
- `mean_pack_credit` is the mean over the six packs. Since 2026-11-26 (the third round) a pack counts only when it is
  taken apart to module level (every healthy module in a bay, a swollen one freed in its seat: `detail["incomplete"]`
  otherwise), and a safety event (`live_contact`, `fastener_short`, `module_dropped`, `module_vented`) condemns it
  (`detail["condemned"]`): either way its credit is 0. A pack that ends early otherwise (the simulated time, the third
  protective stop, the controller given up on, its time budget) keeps what it had if it passes the gate; nothing is
  subtracted anywhere. A controller that does nothing earns 0 (nothing lies where it belongs when a pack arrives).
- The verdict's metrics count each ending (`packs_condemned`, `packs_incomplete`, `packs_counted`,
  `packs_fastener_short`, `packs_stops_latched`, ...), the fasteners stripped, sheared, dropped from the bit and
  loaded past their yield point, and the run's largest distance to each cliff of rules 8 to 10
  (`margin_engaged_lat_mm_hex` / `_torx`, `margin_engaged_tilt_deg`, `margin_load_over_yield`, `margin_carry_accel`,
  `margin_carry_tilt_deg`, `margin_carry_side_mm_before_out`): how close the reference comes to each threshold.
- Pixels are never scored. Measured on the author's server: two renders of one scene on the same GPU differ in
  about half the pixels (mean absolute difference 0.36 of 255).
- A failure of the cell itself (`cellproc.CellError`): the controller is stopped with its process group (as root,
  every process of its account) before a fresh cell starts, and the pack runs once more with a restarted
  controller; that restart is the grader's, not one of the controller's three (`grader_restarts` in the metrics;
  `controller_restarts` counts the controller's own). Since 2026-11-26 (the re-review's N2), as root, the account is
  also emptied before the retry (`verify.scrub_account`): every file of it outside the submission folder (a walk of
  the whole sandbox without a bound, links not followed, `/proc` and `/sys` left out), every SysV IPC object of it
  (`/proc/sysvipc`, IPC_RMID), and the submission folder restored from a root-only copy taken when it was staged. So
  nothing of the submission is left when the fresh cell starts: neither its files on a disk nor shared memory held
  against the sandbox's memory, nor notes about the pack for a second attempt (a clean-up that itself fails, or a
  submission that could not be copied at staging, is logged; the first makes the retry count as the controller's).
  A second failure on that pack is
  scored when it came after the controller was started again (its GPU memory, memory, disk or load beside the
  cell) or when the controller could not be stopped completely: the pack counts 0 (`ended` `cell_failed`) and
  grading goes on. A second failure with nothing of the submission left is a task error (the disk-charge rule of
  2026-11-26 03:50 is gone with the reason it existed). This follows `longtail_manipulation_recovery` (a kitchen
  that cannot be built because the controller's files filled the disk is a score, any other simulator failure a task
  error) and `kitchen_appliance_tending` (an unstable simulation ends the episode as a score; a renderer failure,
  which its paused controller cannot cause, is a task error). `review/test_grader.py` checks it against a stub cell
  (the account's files and its SysV memory removed, the submission restored, links not followed).
- More than 64 processes of the agent's account (the kit Solver's `max_processes`; since 2026-11-26 counted before
  every request by `GuardedSolver.request`, as upstream `cable_routing_through_clips` does) end the pack as a score
  and restart the controller. Threads are not counted (section 5).
- The kit's `VerifierDeadline`, armed at the start with `verifier_deadline_s` (20,400 s, 1,200 s inside
  `[verifier] timeout_sec`): if grading is still running then, the verdict is written from the packs graded so far,
  the others count 0, and `verifier_deadline_expired` is 1. `review/test_grader.py` runs `verify.main` with the
  deadline cut to 7 s.

## 4. The reference program and the anchor

`solution/reference/controller.py` is a privileged reference in the sense of the pump skid and pantry tasks: the
submission carries a 256-bit token, the grader then adds `truth` (every part's pose and grip features, every
fastener's head and seat, the harness' path, the exact commanded targets) to its observations. With that it skips
perception and state estimation: it works through the public actions, pays for every tool and bit exchange, presses
and turns every fastener under the same rules, and decides from the nutrunner's torque and angle that a fastener is
stalled (and raises the torque limit for corroded fasteners; since 2026-11-26 to the highest that harms nothing:
since the third round 97 % of the shank's own yield point, from `truth`, commanded through the cell's transducer
gain and offset, also from `truth`; it works a seized fastener loose with cycles at that limit and at 1.1 times the
assembly torque); that a fastener
is out (`captured`), how long to work a seized one (its breakaway), which module is swollen (it stays in its seat),
which parts are locked or have left their seat, whether the pack is isolated and its waypoints' exact targets it
takes from `truth` (its displacements are taken from the observation it answers, so the lock is honest).
Since the third round it carries a freed fastener slowly (a straight lift at 0.12 m/s, transfers at 0.15 m/s and
0.35 rad/s), under the magnet's limit. Its order: a harness on the lid, the lid's bolts by head type, the lid to its stand (a half turn of the first
joint), the service disconnect and the plugs, the wait for isolation, a harness on the busbars, the busbar and
bracket bolts, the busbars, brackets and modules. Transfers go round the base by a turn of the first joint (a straight line near the base's axis runs the arm
through its shoulder singularity). It answers a repeated step from the observation it comes with: its plan yields a
function of an observation, and every displacement is a planned waypoint minus that observation's commanded pose.

**A perception-based baseline** (`review/perception_baseline/`, maintainers' evidence; nothing runs it in grading)
does the same job from what a submission gets: no token, no `truth`, only the frames, the cell's published numbers
(the cameras' intrinsics, nominal mounting and data sheets, the calibration plates), the manual's tables, the cell's
messages and the nutrunner's readings. It keeps the reference's motion skills and order and replaces every value
the reference reads from `truth`: this cell's cameras from the calibration plates (PnP for the overhead camera;
whole-board wrist views and the robot's kinematics for the wrist camera's mounting; the plates' white for the hall's
light), the variant from the lid's ribs, the pack's placement from the hull of its top's silhouette, each fastener
from six averaged overhead frames and one close wrist frame (the head's flat top gives its type; its outline,
fitted to its rim, its centre where sealant or a neighbour hides part of it), the harness from its blue cable, "out"
from the nutrunner's trace. Its first version, written for exact depth and exact calibration, scored 1.00000 on the
hidden set (section 6, the entry "2026-11-25, 10:00 to 11:35"); it scores 0.00000 on the cell as it is now. The
revised version, for the cell as it is now, scored `mean_pack_credit` 1.00000 on the hidden set (reward 1.0; one run of a program frozen on the development packs before it) (section 6, the entry "2026-11-25, 13:50 to 21:30"). Its third version, adapted to the second round's seized fasteners, foam pads and
swollen modules (section 6, the entry "2026-11-26, from 07:52"), scored `mean_pack_credit` 1.00000 (reward 1.000; one
run, frozen before it).

The anchor and its spread: see the status log (section 6) for the measured runs. `tests/anchor.json` holds the
value measured natively on the third round's version (section 6, the entry "2026-11-26, from 19:00": run ref3 on
`383db135`); `ale validate` of that version is owed. The second round's value through `ale validate` on an RTX 3090
host was the same, in the same steps as the native runs (the entry "2026-11-26, from 16:31").

## 5. Known limitations and open decisions (version 0.1.0)

- **Validated in the engine on one host.** `ale validate` passed on 2026-11-26 on the author's validate host (one
  RTX 3090 on driver 535, rootless Docker without a CPU quota; section 6, the entries "2026-11-26, from 03:54" and,
  for the second round's version, "2026-11-26, from 16:31"): the image builds, setup starts the commissioning service, the verify stage's root-only paths ran (the cell as the
  account `packcell` from a private copy, the seccomp launcher, the kit's account clearing), the untouched sandbox
  scored 0.0 and the oracle 1.0. Not exercised there: an agent phase (the commissioning sessions as the agent
  reaches them), any cloud GPU model, a host with a CPU quota.
- **Difficulty of the third round is not measured by a frontier model.** Two Claude Opus 5.5 episodes ran earlier
  versions: reward 0.84 before the second round and 0.97 on the second round's tree (section 6). The third round's
  numbers are native proxies (section 6): the second episode's frozen program and a fresh-agent proxy that I adapted
  from it under the agent's conditions. Neither is a frontier episode; only a new `ale run` on the pushed tip says
  where a fresh agent lands, and whether the task is now too hard.
- **The third round's cliffs are rules, stated as such.** Damage at once under load (rule 8), the magnet's limits and
  the short check (rule 10) and the gate are rules of the cell, not physics: a socket does not slide over a hexagon
  in PhysX, the magnet's hold is a threshold on the rigid tool centre point's acceleration, a short is a loose
  fastener's body within 3 mm of a seated busbar's box. The transducer of rule 12 is linear with one gain and one
  offset per cell; the shank's yield is a two-slope torque-angle law with a set above the knee. Each is in the
  documents with its numbers, and the reference's distance from each is in the verdict's `margin_*` metrics.
- **The completion gate makes a pack's credit nearly all or nothing**, so a submission's score moves in steps of a
  sixth with each pack that is or is not taken apart, and two runs of a program that is close to an edge can differ
  by that much. The reference is far from every edge (section 6).
- **The arm's own tracking sets the floor of rule 8.** A bit's lateral offset is measured on the physics' tip, and
  the arm's position drives follow their targets to a fraction of a millimetre at rest; the reference engages from
  exact targets, so its offsets measure that floor (`margin_engaged_lat_mm_*`), which is the margin a program has
  before its own perception error is counted.
- **The job can be done without the privileged view, and it takes engineering** (section 6, the entries "2026-11-25,
  10:00 to 11:35" and "2026-11-25, 13:50 to 21:30"). The first perception baseline scored 1.00000 on the hidden set in
  about an hour of work, through two shortcuts: exact depth (a head typed by its height in one overhead frame) and
  cameras exactly at their published poses. Both are closed (a sensor's depth, each cell's own camera mounting), and
  two more levers were added because a revised baseline still found the job easy (sealant over heads, a pack's
  look). The first version, unchanged, scores 0.00000. The revised version scored `mean_pack_credit` 1.00000 on the hidden set (reward 1.0; one run of a program frozen on the development packs before it). After the second round
  (seized fasteners, foam pads, swollen modules) its third version scored 1.00000 (reward 1.000), developed in six
  sessions on the development packs. It needed: calibration of
  both cameras from the plates in every pack (PnP; a hand-eye from whole-board views and the robot's kinematics; a
  robust mean; detector variants against the renderer's colour wash), six averaged overhead frames, a fit of each
  head's outline to its rim at the wrist, white balance from the plates, a safe transfer height, and 29 development
  sessions over about five hours of an author who designed the levers and had the development truth and the
  reference's motion code to start from. A tested agent has none of that, six hours and ten sessions. How hard that
  is for a frontier model is for `ale run`.
- **What a validator would check, checked natively** (2026-11-25, no Docker; as upstream #165's review did):
  1. *Numbers.* `review/check_numbers.py` holds every number of the hand-written instruction (tolerances, the
     cameras' mounting ranges and noise, deadlines, budgets, the set, the sessions, the values) to the code, and
     the generated documents (`service_manual.md`, `tool_catalog.md`, `cell_interface.md`, `cell.json`) to what
     `make_documents.py` writes from the code now: 19 statements (since 2026-11-26 also the submission's limits, the
     process cap and the grader's deadline), 4 documents, no difference.
  2. *Commissioning against grading.* The blind replay of the reference's recorded actions on hidden pack 2 ran once
     through the grader (the small checks) and once through a commissioning daemon whose one development pack was set
     to that pack (a copy of the service's folder, natively), with `run_session.py` and its own repeated steps: 1040
     steps, 730 s of simulated time and no protective stop both ways (the service shows no credit by design).
     Since 2026-11-26 `run_session.py` writes a request as the kit does (without blocking, against the request's one
     30 s deadline), sums and prints the program's own time against the 1200 s budget, stops at the step limit, and
     counts the repeated pairs that differ by the grader's rule (`packcell_client.replies_differ`, the client's copy
     of `wire.canonical` and `is_idle`); `review/test_practice.py` holds the copy to the grader's on 4000 actions and
     runs the launcher against a stub session.
  3. *Nothing hidden within reach.* The hidden set and every key (`key_hex`, `cells_key_hex`, `sealant_key_hex`,
     `appearance_key_hex`) are only in `tests/hidden/`, which never enters the image; the development packs' values
     are in the commissioning service's folder, which the image installs for the account `packcell` alone and
     checks at build time that the agent account can neither list nor read; no offset value and no seed appears in
     a visible file (`review/test_sensors.py`); without the token an observation carries no `truth` and the
     reference ends at once with 0 (the small checks); the service's wire refuses `truth` and `score`. What rests on
     file modes and the verify stage's root-only paths is unexercised until the image is built (below).
  4. *The credit's edge cases.* `review/test_rules.py` (check 12): nothing in place 0, everything 1, a stripped
     fastener loses its 0.5, a module held or beside its bay its 8, a missing fastener is not in the denominator.
  5. *The probe's copy under the sensor model.* A copy carries the true observation's pictures, so its depth has
     the same noise and dropouts by construction, and no rule on the pictures alone tells the two apart; on the
     poses, the true observation had the larger residual in 63 of 136 copies (the same on every tree; chance is
     68). See the next bullet for what remains.
  6. *Every hidden quantity can be recovered from what a program sees.* The revised baseline's estimates against
     the development truth (its README; section 6): the pack's placement within 0.92 mm, the variant 4 of 4, the
     overhead camera so that a lid point is within 1.05 mm, the wrist camera's mounting within 0.19 mm at 0.35 m,
     every seated head typed right and found within 0.14 to 0.24 mm (median) at the wrist, every missing fastener
     seen as a hole, the harness found where it lies, under every light the development packs have.
- **A probe's copy keeps the true observation's pictures** (unchanged by this revision; the brief kept the probe
  lock as it is). A program that locates its wrist camera in a frame (on a calibration plate, say) could notice
  that a copy's reported pose, from joints shifted by up to 0.01 rad, does not match the picture, on the steps
  whose observation carries a wrist frame (at most 400 of a pack's frames). Rendering a copy's wrist frame from the
  shifted pose would close it; it was not done.
- **The new looks are drawn only.** Sealant, grime, dust and labels have no colliders and no mass (a bit engages
  under sealant as without it, the manual says so); the hall's light changes only the colour image. The depth's
  dropouts on dark and bright surfaces are decided on the colour image taken back to the nominal light through an
  approximation of the renderer's display curve (good to about two levels of 255), so a surface keeps its readings
  under any light to that accuracy.
- **Rendering is not repeatable, so neither is a perception program's order of work.** A head read one way in one
  run and another way in the next changes which bolt comes out last; one such order exposed a fault of the revised
  baseline's own (section 6). A program that is right only for one order can score differently from run to run.
- **The head type can also be found by trial.** A bit that does not fit a head turns freely and harms nothing
  (only the two loose combinations and a bad alignment wear a head), so a program can run one bit over a whole group
  and come back with the other for the heads that did not turn. That costs a second pass, not credit. Since
  2026-11-26 the instruction and `task.toml` say so (the review's S9); making a wrong family harmful would not be
  true to the tools. Left as it is for the owner to decide.
- **Levers if frontier models turn out to do too well.** Seven are used. Since 2026-11-25 (section 2, "The cameras",
  "Sealant" and "A pack's look"): the cameras' poses on file are nominal and each cell's own mounting offset has to
  be identified; the depth is an active-stereo sensor's; fastener heads are partly hidden by sealant; the hall's
  light, grime, dust and labels vary from pack to pack. Since 2026-11-26 (section 2, "The second round"), after a
  frontier episode scored 0.84: seized fasteners with a shear torque below the plain strategy's limit (worked loose
  under torque control), a swollen module that must stay in its seat (a sub-millimetre measurement), foam pads that
  hide fasteners from both cameras (found by touch), with the hidden packs combining them as the development packs
  do not. Not used: a wider placement tolerance of the fixture (a program that finds the pack from its silhouette
  meets it anyway), adhesive, pre-damaged heads, bit wear (section 2 gives the reasons). All are perception or
  engineering, none a penalty.
- **GPU models.** The task has run on two models: natively, on the author's RTX 6000 Ada with driver 570 (DLSS
  falls back to an older denoiser preset there), and in `ale validate` on an RTX 3090 with driver 535, below
  Omniverse RTX's R550 floor, with the renderer's driver check off (`--/rtx/verifyDriverVersion/enabled=false` in
  `packcell/server.py`; the canary judges the frames). Since 2026-11-26 `gpu_types` lists `RTX-4090`, `RTX-5090` and
  `RTX-PRO-6000`, the cloud models on which the benchmark validated the same Isaac Sim 6.2 RTX stack
  (`hazard_bay_build`), then `RTX-6000-Ada` and `RTX-3090`; the three cloud models follow that validation of the
  stack, not a run of this task. H100 and H200 stay out (RTX rendering of this stack is unproven there). A model that does not render
  fails the canary as a task error, never as a 0; a validate per model confirms or corrects the list.
- **Reproducibility.** PhysX on the CPU repeated the reference's packs on the author's server (section 6), and the
  validate host (another CPU, an Intel i9-12900K) ran every pack in the same number of steps; across CPUs in general
  it is not expected to be bit-identical, and the renderer is not repeatable even on one GPU. The credit tolerates
  that (it reads where bodies lie), a program's behaviour may not: an agent whose perception is marginal can score
  differently on different hosts.
- **A fastener let go into a pile appears inside it, and is pushed out slowly** (section 6, the entries "19:10 to
  20:30" and "2026-11-25, 01:20 to 02:40"). `eject` places the fastener's body at the bit and makes it dynamic at
  once, and the copy that rides on the bit collides with nothing, so a bit lowered into a pile lets the body
  appear inside the fasteners lying there. With PhysX's default separation speed (3 m/s) that threw five fasteners
  of two packs out of their bins on `15d8694` (the reference releases 70 mm above the table on nine spots per bin),
  and the reference's value fell to 0.9944. Since `8439170` every fastener's body carries a maximum depenetration
  velocity of 0.3 m/s: the overlap is undone without throwing anything, and the reference scores 1.0 again. What
  remains is the simplification itself: a real fastener on a bit pushed into a pile would meet the pile, here it
  appears in it and is pushed up and aside. A reviewer may prefer the cell to release the body only above what lies
  there; that changes what every program meets and needs the full series again.
- **The baseline shows that the information suffices, not that the job fits the stated conditions** (the
  independent review of `846a4f49`, S10). It was developed with the development packs' truth (the survey and trace
  tools compare estimates with it), the reference's motion code, 29 development sessions and the knowledge of the
  levers' designer; a tested agent has ten sessions, no truth and six hours. The reviewer re-ran it on `846a4f49`:
  1.00000 on all six packs, with at most 77 frames of 400, 1649 steps of 20000, 1099 s of simulated time of 2700 and
  68 s of program time of 1200 a pack. A development round without truth within ten sessions was not done: the
  frontier runs answer the question.
- **The probe's copy can be told from the true observation by continuity** (review S11). One observation does not
  tell them apart (the pose-against-joints residual is at chance: 48.8 % of 6000 synthetic pairs, 63 of 136 of the
  grader's), but a program that remembers its previous observation and the action it sent can predict the
  commanded tool pose and pick the candidate whose `tcp_target` is nearer: the reviewer's rule labelled 136 of the
  grader's 136 pairs in the reference run and 5980 of 6000 synthetic pairs built with `wire.perturbed`. It earns
  nothing by itself (a replayed stream still cannot follow the hidden packs); it means that the lock rejects only
  programs that do not try to pass it. Not changed.
- **Busy threads.** The process cap (64) counts processes, not threads, and the controller is neither paused nor
  in the idle class between requests, only at nice 19. The reviewer measured a nice-0 loop's share of one CPU at
  3.65, 2.33, 1.08 and 0.34 s in 4 s against 0, 15, 60 and 250 busy nice-19 loops, so a controller with hundreds of
  busy threads can slow the cell's CPU physics; since 2026-11-26 the grader's deadline turns that into a score (the
  packs graded by then) instead of a run without one. Pausing the account between requests, as
  `kitchen_appliance_tending` and `longtail_manipulation_recovery` do, was not done.
- **GPU memory and memory are shared, not partitioned.** The instruction asks a program to stay below 12 GB of GPU
  memory and 16 GB of memory (the cell took about 3.5 GB and 6 GB in the reviewer's run); nothing enforces it, and a
  program that takes more risks the cell-failure rule (section 3: the pack counts 0).
- **Numerics.** Since 2026-11-26 the graded cell runs with the image's `OPENBLAS_CORETYPE=Haswell`,
  `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1`, as the commissioning cell does (review S8: 3000 `kin.ik_step`
  calls gave the same bits unset and with Haswell on the reviewer's Zen 3 CPU, other bits with Sandybridge kernels).
  The full pin of `docs/contributing-a-task.md` section 5 (`NPY_DISABLE_CPU_FEATURES`, `OPENCV_IPP`, a fingerprint
  check) is not used: PhysX and the renderer differ across hosts anyway. The controller gets `PATH`, `HOME` and
  `LANG` only, as the interface document says.
- **CUDA's abstract socket** (review, not checked). The cell process listens on CUDA's abstract unix socket
  `@cuda-uvmfd-<ns>-<pid>`; the seccomp filter allows AF_UNIX, so the controller can connect to it in the verifier.
  Whether CUDA authenticates the peer was not examined; it is engine-wide rather than this task's, a question for
  the maintainers.
- **Smaller points of the review, left as they are:** the 1 MiB cap on a reply line (`verify.py`) is not stated (an
  action is a few hundred bytes); the oracle token is a static file in a public repository (the accepted pattern,
  as in `pump_skid_startup_and_operation` and `pantry_jar_refilling`); `full_at` 0.95 is not sized from a measured
  spread (the reference repeats step for step here; `ale validate` runs on the evaluation hosts must size it); this
  README is long, and section 7's report keeps statements that later entries supersede (its "Since this report"
  says which).
- **Simplifications that a reviewer may want changed**: no finger colliders; no slip; the socket does not slide
  over the head; a harness is rigid; the robot does not collide with itself; the live-parts check is geometric and
  once per tick.
- **The task name is provisional** (not yet registered in the coordination sheet); it appears only in the folder
  name and in `task.toml`.
- **The values** (module 8, lid 5, service disconnect 3, busbar 2, bracket 1, plug 1, harness 1, fastener 0.5)
  are the author's. In an HVB-P5 the lid with its bolts is about 12 % of the pack's value, the service disconnect
  and the plugs 6 %, the busbars with their bolts 20 %, the brackets with theirs 17 %, the modules 45 %: the
  modules are what the line is there for, and every earlier stage still shows in the credit. A reviewer may prefer
  another weighting; it lives in `packcell/series.py` (`VALUES`) and the documents follow it
  (`review/make_documents.py`).

## 6. Status log

(Newest last. Times are the author's server's, EST.)

**2026-11-24, 02:50 to 03:40. The stack and the cell's skeleton.** Read the reference tasks (`hazard_bay_build`
for Isaac Sim, `pump_skid_startup_and_operation` and `pantry_jar_refilling` for arm cells, documents and the
commissioning service). Probed Isaac Sim 6.2 natively (the author's Python 3.12 venv): the UR20 asset
loads and articulates on CPU PhysX with two links added to its articulation; 240 physics steps in 0.2 s; the
incoming joint force of the added tool carrier is the flange wrench in the carrier's frame (checked with the tool
down and tilted 90 degrees: 32.4 N for 3.3 kg); a kinematic body can be made dynamic at run time; a fixed joint
between a link and a body can be created and removed at run time; two cameras render in 0.05 s a frame set once
`/exts/omni.replicator.core/maxAssetLoadingTime` is set to 1 before Replicator loads (the 100 s stall did not
occur). Wrote `kin.py`, `series.py`, `scene.py`, `rules.py`, `cell.py`, `wire.py`, `server.py`.

**03:40 to 04:20. The reference, and what it found in the cell.** A sleeping articulation reports stale joint
speeds and forces (the dock's "at rest" check failed for it): the arm's sleep threshold is 0 and joint speeds are
differences over the tick. A fastener's body moved by the tensor API came back to its seat when it was made
dynamic: bodies the rules move are moved by authoring their pose on the stage. A fastener leaving its seat in the
tick in which its part was freed flung the lid away (a kinematic body sweeping through a dynamic one): a freed
fastener's colliders are off while it waits, and a part is freed one tick after its last fastener is out. After a
protective stop the arm kept pressing with its target at the stopped pose: it now backs off to where it was 0.25 s
earlier. Straight Cartesian transfers past the base ran the arm through its shoulder singularity: the reference
turns the first joint instead. A gripper taking a bracket by its short side hit the tray's wall (the body is long
along the closing axis): the reference takes brackets by the long side. Parts dropped on one spot of a bin piled up
under the gripper: the reference spreads them. A one-tick remainder of a long move jerked a carried module into a
protective stop: the reference splits moves evenly and never sends a short action for a real distance.
First complete pack natively: credit 1.0 (a P4, 28 fasteners, 21 parts), 611 s of simulated time, 827 steps, 150 s
of wall clock.

**04:20 to 04:40. Grader, service, documents.** `verify.py` and `pack_grade.py` ran natively through the kit's
`Solver` and probe lock on hidden pack 0 (credit 1.0, 978 steps, lock 11 of 11 counted pairs, 8 idle pairs not
counted). That run showed 44 protective stops when the harness on the lid was gripped: the harness' ramp over a
bolt cut through the head's edge, and the harness is pushed out when it becomes dynamic. The harness now bridges a
head level (wider than any head) and keeps clear of every fastener it does not cover (`review/test_rules.py`
checks 360 draws); the same pack then ran with no stop. `ale lint` on the folder: ok. `scripts/lint_domain.py`: only
the two messages of a null anchor. The kit copy is byte-identical. Wrote the Dockerfile (after
`hazard_bay_build`'s), `setup.sh`, the commissioning service, the client library, `run_session.py`, the example
program, the documents' generator, the drawings and photographs, `select_set.py` (the hidden and development sets
drawn under its rule), `test_rules.py` (36 checks, no simulator).

**04:40 to 05:50. The whole hidden set, and what it found.** The first native verify of all six packs (the tree of
04:34, old values) gave 1.0, 0.866, 0.997, 1.0, 0.347, 1.0: the reference itself was not robust. Causes, each
found with an in-process trace, and the fixes:

- The reference turned the gripper to each part's direction by the shortest rotation from wherever the last joint
  stood; over a P6's many transfers the joint ran into its 360 degree limit, moves were clipped, parts were not
  reached, and a module was lowered across a rack divider (1600 N, a protective stop). It now picks, among the
  turns a task allows (exact, half-turn symmetric for the gripper, free for the nutrunner and for a part over a
  bin), the one that leaves the last joint nearest its middle, and makes it together with the first joint's turn.
- A plug that hung still on the gripper for a while stayed in the air when the gripper opened: the body had gone to
  sleep and a sleeping body does not notice that its joint is gone. The cell now wakes a part when it is released.
- A harness on the busbars could reach under a module's handle bar (it then pressed down on the tool when gripped).
  Its ends now stay short of the handles (`review/test_rules.py` checks it on 360 draws).
- A bit pressed on a bracket bolt was now and then kicked 16 mm off the head (a lateral 230 N, the wrist's drive
  saturated). The bit-on-head contact now has the high friction described in section 2.
- The pull on a plug raised the force from under 60 N to over 220 N within one tick (release and overload in the
  same tick): the reference pulls the first 4 mm at 4 mm/s.

After these, packs 0, 1, 2 and 4 each ran to credit 1.0 with no protective stop (direct runs of the reference).
The values were rebalanced (section 5) and the documents regenerated.

**What the frames carry** (`review/check_observability.py`, two development packs, public means only: one overhead
frame, the published calibration, one wrist frame per candidate; compared with the truth afterwards):

| pack | lid fasteners and holes found | overhead position error (median / max) | wrist position error (median / max) | head kind right | rust right |
|---|---|---|---|---|---|
| development pack 3 (P6, no harness) | 14 of 14 | 0.60 / 1.46 mm | 0.23 / 1.22 mm | 14 of 14 | 14 of 14 |
| development pack 2 (P5, harness on the lid over L03, three holes) | 11 of 12 (the one under the harness is the one not found) | 0.75 / 2.39 mm | 0.97 / 1.11 mm | 10 of 10 | 7 of 7 |

On the second pack one corroded hexagon was found from above (2.4 mm off) but not again by the script's wrist
step; hexagon heads are located about 1 mm off by the wrist step (the script takes the centre of the visible blob,
and the camera looks at the head 11 degrees off its axis), Torx heads within 0.5 mm. The bits' tolerances are
2.0 mm (socket) and 1.2 mm (Torx). The script is a measurement, not a solution: it does nothing about the variant,
the inside of the pack or the harness.

**The manual's photographs** were taken with the cell's cameras on an example pack (`review/make_photos.py`); round
primitives are drawn with a finer tessellation since (a fastener's head read as a ten-sided polygon before), and
the lights were turned down (the tray and the plated heads were blown out).

**05:50 to 06:20. The verify stage's small checks (native, null anchor, the tree of `6a08a7e`).**

| submission | set | result |
|---|---|---|
| none (an empty `/home/user/submission`) | six packs | "no usable submission": the honest zero, no simulator started |
| the example program (takes one frame, says `done`) | six packs | credit 0 on every pack (0 of 49, 48, 59, 58, 67, 69 items), `mean_pack_credit` 0; the whole stage 186 s |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0 |
| the reference, its applied actions recorded | hidden pack 2 | credit 0.9944 (58 of 59), 1007 steps, the lock 11 of 11 counted pairs (9 idle pairs not counted) |
| a blind replay of those recorded actions | hidden pack 2 | the same credit to the last digit, 0.9944, in the same 1007 steps (PhysX repeated the pack exactly on this machine), and **the lock fails: 0 of 11 pairs**, so the reward is 0 |

The one item the recorded reference lost was a busbar bolt that bounced off the pile in the fasteners bin and out
over the wall (ejected 5 cm above the walls, always at the same corner). The reference now lowers the bit between
the walls and spreads the fasteners; the pack then ran to 1.0.

The commissioning service's first native run showed that a session opened right after another had closed was
refused as busy (the service was still stopping the first cell): `open` now waits up to 120 s for the cell to stop.

**06:15. The commissioning service, natively** (`commissiond.py` as the author's account with its run and state
folders redirected; the agent's own `packcell_client.py` and `run_session.py` against it): the status file and
request; `run_session.py` with the example program on development pack 2 (session 1: READY, reset, two steps, one
overhead frame, `done`; 28 s of wall clock); then an interactive session on pack 1 opened right after (it waited
for the first cell to stop): the first observation carries no `truth`, the overhead frame decodes to 1440 x 1080
with depth up to 2.05 m, a move and a wrist frame work and the frame count goes down, a malformed action is
refused without effect, and `truth` and `score` are unknown requests on this wire. Two sessions counted, eight
left, the count kept in the state file. Not run: the service under its own account behind the image's permissions
(`setup.sh` as root), which needs the image.

**06:20 to 07:15. A first full run on the frozen tree, and the bit's collider.** One native verify of all six
packs on `7d2ef4e`: `mean_pack_credit` 0.99920 (1.0, 1.0, 1.0, 1.0, 1.0, 0.9952; one busbar bolt of pack 5 not in
a bin), lock 85 of 85, 1332 s for the stage; but 4 and 3 protective stops on packs 1 and 3, all with the bit
pressed on a fastener: the bit was thrown 15 mm sideways with 250 to 2500 N in a tick in which nothing was
commanded. 160 scripted presses with centred, converged approaches never showed it, at any friction, so it is not
friction (the note on friction in the list above stays true but was not the cause). The cause was not isolated;
two suspects were removed together: the bit's collider is a capsule (a round end on a flat head: one contact point
with the head's normal; the suspicion is that a flat-ended cylinder on a head of nearly its own radius is a
degenerate pair for the contact generator), and a part whose last fastener has just come out is not freed while the
bit still presses on it (the suspicion: a 0.12 kg busbar squeezed between the bit and its post is shot out). After
both, packs 1 and 3 ran with no stop, credit 1.0 each. Which of the two did it, or whether a third cause is only
hidden, is not known: the evidence is that all six packs have run without a stop since (the entries below).

(The session was interrupted here from 07:15 to 14:50; nothing was running in between.)

**14:50 to 15:40. Resumed; a cell that did not start; the run the anchor is taken from.** The changes above were
committed (`82837d8`). Hidden pack 5, the one that had lost a busbar bolt in the run on `7d2ef4e`, ran directly
with the new tip: credit 1.0, no stop. The first attempt at three full runs then failed at its first cell ("the
cell did not answer within 900 s"): other users' jobs had the server at a load average of 50 on its 32 cores, and a
bare start of Isaac Sim took 348 s at that moment (17 s when the machine is idle). The failure showed a defect:
`CellProcess` left a simulator that had not come up running, so the next run started a second one beside it. A cell
that does not come up is now stopped (`230187b`). `grader_config.json` keeps its 900 s (a cell that does not start
is a task error or a pack that is run once more, never a zero); the native runs since were made on a stage copy
with 3000 s, the only difference from the committed `tests/`.

One native verify of the whole hidden set on `230187b`: `mean_pack_credit` 1.00000, every pack 1.0 (49, 48, 59,
58, 67 and 69 of their items placed), no protective stop, no stripped fastener, the lock 77 of 77 counted pairs
(50 idle pairs not counted), 852 to 1185 steps and 601 to 813 s of simulated time a pack, 6 s of the reference's
own time, 1166 s for the stage. `tests/anchor.json` records that value (`21ac344`), `full_at` 0.95.

**15:40 to 16:30. Repeats, and every check again with the anchor recorded** (the tree of `21ac344`; cell, grader
and reference are those of `230187b`).

| submission | set | result |
|---|---|---|
| the reference, second full run | six packs | `mean_pack_credit` 1.00000, reward 1.0, lock 77 of 77, stage 1077 s |
| the reference, third full run | six packs | `mean_pack_credit` 1.00000, reward 1.0, lock 77 of 77, stage 1076 s |
| none (an empty `/home/user/submission`) | six packs | "no usable submission": reward 0.0, no simulator started |
| the example program (one frame, then `done`) | six packs | credit 0 on every pack, reward 0.0, stage 140 s |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0, reward 0.0 |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 11 of 11 (9 idle), reward 1.0 |
| a blind replay of those recorded actions | hidden pack 2 | the same credit, 1.0, in the same 1040 steps, and **the lock fails, 0 of 11**: reward 0.0 |

The three full runs are the same run three times: every pack took the same number of steps and the same simulated
time in each (PhysX on the CPU repeats on this machine), so they show no spread, and they say nothing about another
CPU. What a change of the tree does is visible one entry up: the run on `7d2ef4e` had lost one bolt (0.99920).
`full_at` 0.95 is therefore a judgment, not a measured spread (the anchor's `note`). The reference asks for no
camera frame (it reads `truth`), so in its runs the renderer works only in the canary; frames inside a pack were
exercised by the example program, the commissioning sessions and `review/check_observability.py`.

The commissioning service ran natively again on this tree, with the same result as at 06:15 (the example program
through `run_session.py` on development pack 2 in 22 s; an interactive session on pack 1 right after: no `truth`
in an observation, both cameras decode, a malformed action refused, `truth` and `score` unknown requests; two
sessions counted, eight left).

The image's asset step ran natively: `environment/fetch_isaac_assets.py` with `environment/isaac_assets.sha256`
downloaded the UR20's seven files from NVIDIA's public bucket, every digest matched, and the files are the ones
the native runs used.

Static gates on this tree: `ale lint` on `tasks/` ok; `scripts/lint_domain.py` ok; `tests/robotics_grader/`
byte-identical to `shared/robotics_grader/`; `scripts/task_assets.py check` 0 problems; `scripts/gpu_types.py
check` 0 errors; `review/check_copies.py` ok; `review/test_rules.py` 36 checks ok. The repository's `pytest`: all
pass but two. `tests/test_task_cards.py` fails because this task has no card in `tasks/cards.toml` (a maintainer's
step before the merge; the author was told not to edit that file). `tests/test_longtail_pause_and_close.py` fails
on this server only: its temporary folder is on NFS and cannot be removed while a file in it is open; with
`TMPDIR` on a local disk it passes. It does not touch this task.

**18:20 to 19:00. The kit of upstream #173, and the probe's copy rebuilt (tree `7e59d13`; the native verify ran
on `152fdf7`, the same task files).** `upstream/main` moved to `1c13a47` after this branch was cut: #173 changed
`shared/robotics_grader/` (the solver and everything it starts run at nice 19; `replies_differ` is the lock's one
rule; `probe.stale_copy`; `deadline.VerifierDeadline`) and copied it into every task. The branch was merged with
`upstream/main` and the kit copied again (byte-identical to `shared/`). The task against #173's three findings:

1. *The solver's priority.* The kit's own: `GuardedSolver` only replaces `_spawn`, and the kit starts whatever
   `_spawn` a task has from its low-priority thread. Nothing to change.
2. *One rule for probe pairs, in practice too.* `~/commission/run_session.py` repeats steps but counts no pairs,
   so there is no second rule. Nothing to change.
3. *A probe copy must be built the way the real observation is.* This task's copy was not. `wire.perturbed`
   shifted the joint readings (up to 0.02 rad) and, independently, the tool poses (up to 12 mm and 0.03 rad) and
   the wrench (up to 3 N). The cell reports `tcp` as the kinematics of its own joint reading times the tool, so a
   true observation's pose and joints agree to the rounding (about 1e-6) and the copy's disagreed by millimetres:
   one line with the robot's model, which is among the givens, named the copy every time. A program could have
   answered the copy differently on purpose and replayed everything else.

The copy is now the observation the cell would give with its joints shifted (section 3): `q` and `q_target` by
one shift, `tcp`, `tcp_target` and a wrist frame's `pose` from the shifted angles as `cell.observe` builds them,
the wrench, the joint speeds and the pictures unchanged. Two finer points were measured and closed on the way.
Building the copy from the rounded readings made the hidden digits of `q` and `q_target` the same number (true
observations' are independent), and a copy of a commanded angle with round digits had hidden digits where the true
one has none (a threshold on that pair's residual named the copy in 86 % of 4000 observations). So the grader
takes the cell's angles before rounding (`cell.joints`, served only to the grader, like `truth`) and the shift is
a whole number of the readings' last digit: what a reading hides is then the same in both. Measured without the
simulator, on 4000 observations built as the cell builds them, with the grader's builder:

| commanded angles | the true observation has the larger residual | best single threshold on the residual names the copy |
|---|---|---|
| any digits (the usual case: a target from inverse kinematics) | 1984 of 4000 (49.6 %) | 51.0 % (50 % is chance) |
| round digits (a joint move by a round amount) | 1979 of 4000 (49.5 %) | 50.9 % |

The tool centre point of a copy lies 9 mm from the true one in the median (3 mm at the 5th percentile, 31 mm at
most), so a program that works from the observed poses answers differently. The agent's library
(`packcell_client.perturbed`, used by `run_session.py`) builds the same copy from `ur20.json` and `cell.json`; it
has only the rounded readings, so its copies are right in the usual case and not in the round-digits one, which
does not matter for a rehearsal. The interface document and the instruction say what a repeated request carries.
`review/test_rules.py` (37 checks) holds the copy to these properties and the library to the grader's builder.

What ran on `7e59d13`: `ale lint` ok; `scripts/lint_domain.py` ok; the kit comparison; `scripts/gpu_types.py
check`; `scripts/task_assets.py check`; the kit's selftest; `review/check_copies.py`; `review/test_rules.py`; and
the grader's loop (`pack_grade.run_packs`) against a stand-in cell without a simulator: a controller that works
from the observed angles, 81 of 81 probed pairs differing; one that does not, 0 of 81 and the lock fails; the
cell asked for its angles once per probe; the residuals of both observations about 1.5e-6.

**The native verify on this tree** (`152fdf7`: the task files of `7e59d13` and this entry; 18:42 to 19:19, one
Isaac process at a time, confined to GPU 3 of the author's server, which other users' jobs shared). The first
attempt, at 18:34, was stopped within a minute because the GPU it was confined to had just been filled by another
job. What the change touches is which steps are probed and what the copied observation holds; the cell's physics
and the actions executed on true observations are the same code, and the runs show it:

| submission | set | result on `152fdf7` | on `21ac344` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` 1.00000, reward 1.0, the lock 83 of 83 (53 idle pairs not counted), stage 1394 s | 1.00000, lock 77 of 77 (50 idle), stage 1076 s |
| none (an empty `/home/user/submission`) | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 10 of 10 (6 idle), reward 1.0 | lock 11 of 11 (9 idle) |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0 in 1040 steps, **the lock fails, 0 of 10**: reward 0.0 | 0 of 11 |
| the example program (one frame, then `done`) | six packs | credit 0 on every pack, reward 0.0, stage 188 s | the same, 140 s |

Every pack of the reference's run took the same number of steps as on `21ac344` (921, 852, 1040, 1066, 1151 and
1185) and the same simulated time (601 to 813 s), with every item placed, no protective stop and no stripped
fastener; the reference's own time was 6 s. Which steps are probed is drawn afresh, so the lock's counts differ
from the earlier tree's; the longer wall times are the shared GPU's. The commissioning service, started from this
worktree, behaved as before: the example program through `run_session.py` on development pack 2 (2 steps, 1
frame, 0 repeated, 27 s); an interactive session on pack 1 right after: no `truth` in an observation, both
cameras decode (overhead 1440 x 1080, wrist 960 x 720), a malformed action refused ("move.type must be one of
hold, tcp, joint"), `truth` and `score` unknown requests; two sessions counted, eight left.

What the grader recorded about the copies (`probe_*` in the verdict's metrics) did not match the measurement
without the simulator above: of 136 copies (one for every probed step, idle pairs included), the true observation
had the larger residual in 81 (59.6 %; for a fair coin 68 ± 5.8, so 2.2 standard deviations above half), and the
largest true residual was 4.6e-4 to 8.0e-4 (an entry of a rotation matrix) on five of the six packs (one copy, on
pack 4, reached 6.9e-4), where the rounding leaves about 1e-6. The cause is the conversion of a pose near a half
turn (the next entry).

**19:10 to 20:30. A pose's rotation vector near a half turn (tree `15d8694`), and fasteners thrown out of the
bins.** The cause of the residuals above is `kin.R_rotvec`, the conversion of a rotation matrix to a rotation vector
that `wire.pose6` uses for every pose of an observation (and `kin.pose_error` for the cell's inverse kinematics). It
took the angle from `acos` of the trace and the axis from a division by `sin`; within about 1e-6 to 1e-5 rad of a
half turn both lose precision, and a pose written on the wire and read back was off by up to 2e-3 in a matrix
entry. The tool pointing straight down is a rotation of nearly exactly a half turn, and with the drives' small
following error the measured tool pose lands in that band, while a probe's copy, its joints shifted by up to 0.01
rad, is out of it: the true observation carried the large residual, the copy did not. The wrist camera, tilted 11
degrees from the tool's axis, is far from a half turn in those states.

The angle now comes from `atan2` of the antisymmetric part's norm and the trace, and within 1e-3 rad of a half
turn the axis from the symmetric part, signed by the antisymmetric one: in `tests/packcell/kin.py` (copied to the
commissioning service by `review/check_copies.py --sync`), in the agent's library (`packcell_client.R_rotvec`) and
in the reference. Nothing else in the folder converts a matrix to a rotation vector (`rules.angle_deg` is the angle
between two vectors, for the tilt rules). Measured without the simulator (`review/test_rules.py`, now 41 checks,
which fail with the old conversion; the round trip is the largest entry difference over 200 random axes):

| | the old conversion (`acos`) | the new one |
|---|---|---|
| a pose written and read back, angle pi - 1e-2 to pi - 1e-4 | 6e-7 | 6e-7 |
| at pi - 1e-5 | 2.1e-5 | 6.7e-7 |
| at pi - 2e-6 | 5.6e-4 | 6.4e-7 |
| at pi - 1e-6 | 1.9e-3 | 6.5e-7 |
| at pi - 1e-7, at pi | 7.6e-7, 6.4e-7 | 7.0e-7, 6.4e-7 |
| 400 near-downward states (`downward_states`): true residuals above 1e-5, copies above 1e-5, the true one the larger | 82, 0, 267 (67 %) | 0, 0, 184 (46 %) |

Away from a half turn (angles 1e-6 to pi - 1e-3) the two agree to 7e-10. Below about 1.5e-8 rad the old one
returned exactly zero (`acos` of a trace that rounds to 1) where the new one returns the rotation. The library's
copy of a wrist pose is now the grader's exactly; it was up to 7e-6 off at random joint angles, which a check had
put down to `cell.json`'s nine decimals, and was the conversion.

The native series on `15d8694` (19:30 to 20:04, GPU 3 only, as above):

| submission | set | result on `15d8694` | on `152fdf7` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` **0.99440** (packs 0 and 1: 0.97315 and 0.99324, packs 2 to 5: 1.0), reward 1.0 (against the anchor of then, 1.0), the lock 83 of 83 (53 idle), no protective stop, no stripped fastener, stage 1330 s | 1.00000, 83 of 83 |
| none | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 10 of 10 (6 idle), reward 1.0 | the same |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0 in 1040 steps, **the lock fails, 0 of 10**: reward 0.0 | the same |
| the example program | six packs | credit 0 on every pack, reward 0.0, stage 168 s | the same |

The commissioning service behaved as before (the example program in 26 s; the interactive session as above; two
sessions counted, eight left). The probe's copies now balance: the true observation had the larger residual in 66
of 136 (48.5 %; 0.3 standard deviations below half, against 2.2 above on `152fdf7`), and the largest residual was
1.6e-6 for both (8.0e-4 and 6.9e-4 before).

*The five fasteners.* Every pack took the same number of steps as on `152fdf7`, and every fastener came out (the
same `fastener_out` events, within 0.05 s): the plan ran as before, and what changed is where released fasteners
came to rest. A second run of packs 0 and 1, from a stage copy whose `pack_grade.py` also keeps the cell's `where`,
lost the same five: three on the floor 1 to 2.5 m away (L08, B03a and R3af of pack 0), two on the table 10 and 20
cm beside a bin (B00a of pack 0, R4af of pack 1). A third run of pack 0, with the cell's `_eject` instrumented in
a stage copy, showed how. The reference releases every fastener with the bit's tip 70 mm above the table (between
the walls: the fix for the bounce of the 05:50 entry), on nine spots per bin that it uses in turn, so from the
tenth fastener of a bin on, a released one appears exactly over earlier ones (0.0 to 0.1 mm apart horizontally),
and once two lie there its shank is inside them. The cell places the ejected body at the bit and makes it dynamic
at once, and the physics engine pushes overlapping bodies apart fast: a normal release has fallen at 0.49 m/s
after its first tick; the release of R3af over L08 and B03a threw L08 at 5.6 m/s, and it ended on the floor 2.3 m
away with the other two; two more releases into a pile reached 1.9 and 1.6 m/s. The earlier trees used the same
spots, and their piles lay differently by chance: the trajectories differ in detail now (the poses near a half
turn are read exactly, and the inverse kinematics sees rotation errors below 1.5e-8 rad that were zero before),
and nothing had left a bin.

*What was decided* (nothing about the reference or the cell was changed; both would need several full runs, and
the cell's physics is the owner's to decide). The conversion stays fixed: it is right, and the probe's balance
shows it. The lost fasteners are a weak spot that the earlier runs hid, of the reference's drop rule (a fixed
height, reused spots) and of the cell's `eject`, which makes a body dynamic wherever it is, also inside another one;
a program of the agent's that releases into its own pile meets the same (section 5). `tests/anchor.json` now holds
the reference's value on this tree, 0.9944 (1.0 before, measured on the earlier trees). `full_at` stays 0.95: the
only spread measured so far is this one, a small perturbation of the reference's trajectory that cost 0.56 % of
the mean and 2.7 % of one pack, and 0.95 leaves about nine times that. With that value the reference's ratio on
this tree is 0.99440 / 0.9944 = 1.000, reward 1.0 (computed from the run above, not run again). Static gates on
the committed tree: `ale lint` ok, `scripts/lint_domain.py` ok, the kit comparison, `review/check_copies.py`,
`review/test_rules.py` 41 checks.

**2026-11-25, 01:20 to 02:40. The fasteners' depenetration cap (tree `8439170`; GPU 3 of the server only, as
above).** The owner chose to change the cell rather than keep 0.9944. `packcell/scene.py` now gives every
fastener's body PhysX's `maxDepenetrationVelocity` of 0.3 m/s (`FASTENER_MAX_DEPENETRATION`; the schema's default
is 3 m/s), and nothing else changed: the parts, the robot, the rules, the grader and the reference are those of
`15d8694`. The cap bounds only the speed the solver adds to undo an overlap (a fastener on its seat is kinematic
and does not meet it at all).

First a run of packs 0 and 1 from a stage copy with `_eject` instrumented as in the third run above (the tree of
`9fa0782` with the change not yet committed): both packs 1.0 (49 of 49, 48 of 48) in the same 921 and 852 steps.
Of the 53 ejects, the fastest fastener body within ten ticks of one moved at 0.94 m/s (a body that falls freely
moves at 0.49 m/s after those ten ticks), against 5.6, 1.9 and 1.6 m/s before; the release of R3af exactly over
L08 and B03a, which had thrown L08 to the floor, now moved B03a at 0.75 m/s, and nothing left a bin.

The native series on `8439170`:

| submission | set | result on `8439170` | on `15d8694` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` **1.00000** (every pack 1.0, in the steps of every tree since `21ac344`: 921, 852, 1040, 1066, 1151 and 1185), the lock 83 of 83 (53 idle), no protective stop, stage 1674 s | 0.99440 |
| none | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), lock 10 of 10 (6 idle), reward 1.0 | the same |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0, **the lock fails, 0 of 10**: reward 0.0 | the same |
| the example program | six packs | credit 0 on every pack, reward 0.0, stage 196 s | the same |

The commissioning service behaved as before (the example program in 28 s; the interactive session as above; two
sessions counted, eight left). The probe's copies balance as on `15d8694`: the true observation had the larger
residual in 63 of 136 (46.3 %, 0.9 standard deviations below half), the largest residual 1.6e-6 for both.

*What was decided.* `tests/anchor.json` holds 1.0 again, the reference's value on this tree. `full_at` stays 0.95:
the only spread measured is still the perturbation of `15d8694`, and its 0.56 % came from the defect fixed here;
0.95 leaves room for another CPU's contact simulation. Section 5 keeps what remains of the simplification (the
released body appears where the bit is, also inside a pile, and is pushed out slowly).

**2026-11-25, 10:00 to 11:35. A perception-based baseline (`review/perception_baseline/`, frozen at `82e5f2f`; GPU
3 of the server only, as above).** The question a reviewer asks of a privileged reference: can the job be done from
what a tested agent has? The answer is a program, measured on the hidden set like the reference.

*The program.* A submission folder (`run.sh`, `main.py`, `controller.py`, `vision.py`, `manual.json`, `cell.json`).
It reads only its own folder, the observations and replies on the wire; it carries no token and runs without one.
`cell.json` is the agent's copy of the cell's configuration, `manual.json` the service manual's tables
(transcribed by `dev/parse_manual.py`). NumPy and OpenCV only. The motion skills and the order of the job are the
reference's (copied, with a line saying so); every value the reference takes from `truth` comes from perception, the
manual or the program's own bookkeeping instead (the folder's README has the table):

- the pack's placement: the silhouette of the pack's top in an overhead depth frame (the lid at 164 mm, the tray's
  walls at 160 mm once it is off), its four sides fitted as lines in the base frame;
- the variant: each variant's rib lines (manual, section 2) sampled for a 2 to 6 mm rise on the lid; after the lid,
  the module handles that stand where the manual puts them;
- each fastener: a bump at the manual's place in the overhead depth frame, its height above the seat giving the head
  (M6: S10 6 mm, T30 4.5 mm; M8: S13 8 mm, T40 6 mm), none for a missing or hidden one; then one wrist frame with the
  head on the camera's axis (the bit's tip 0.105 m above it, the camera on the side where nothing stands in the way:
  along the bracket for the brackets' bolts), and the depth points of the head's top face give its centre and height
  again, or an empty hole; a head that shows less than 93 % of its top face, or lies 1.4 mm from the overhead
  estimate, is looked at again from the other side;
- corroded or not: not needed; a fastener that stalls at 1.3 times the assembly torque is pressed again with the
  limit at twice the assembly torque, within the "never exceed" value;
- "out": the breakaway, the manual's turns to free and a free-running spindle in the nutrunner's trace (the first
  quarter turn slowly, so that a bit that slips or turns free is stopped before it wears the head), confirmed by
  `eject: ok`;
- the harness: the blue cable in the overhead frame, a grasp across a straight, level stretch at its centre's height
  (other places along it if a grasp fails);
- grip points and isolation: the manual's places in the pack's frame; the service disconnect's pull plus 30 s;
- the commanded pose: the program's own record after a completed move; a wrist frame's pose is used only when it
  agrees with that record within 1.5 mm, so a repeated step's copy, whose readings are shifted, is never taken for
  the camera's pose.

*How it was developed* (development packs only; the cell with the grader's switch is used by the tools in `dev/`,
never by the program). Three survey sessions (`dev/survey.py` on development packs 3, 1 and 2, in the service's numbering 1 to 4; a
fourth on pack 4 was stopped as it started): the reference does the job while the tool takes overhead frames at the stages and wrist
frames over every fastener, with the truth beside them. The perception was written and measured offline on those
frames (from above: the variant right on three of three, every seated head typed right, lid heads within 1.02 mm and
inner ones within 1.30 mm; from the wrist: 0.27 mm median, with outliers up to 1.75 mm only where a busbar or a
module hid part of a bracket's head from one of the survey's two fixed camera sides, a side the program does not use,
and on the one bolt the survey looked at with the harness still on it). Then closed-loop runs
(`dev/devrun.py`: `run.sh` behind the wire with the grader's deadlines and repeated steps, the truth only afterwards,
`dev/compare.py`, `dev/stage_table.py`): round r1 of the first complete program and round r2 of the frozen one (two
retries added that the development packs never needed). Eleven cell sessions in all; a tested agent gets ten.

| development pack | lid (+ bolts) | MSD, plugs, harness | busbars (+ bolts) | brackets (+ bolts) | modules | credit r1 / r2 | simulated s, steps, frames (r2) |
|---|---|---|---|---|---|---|---|
| 1: P4, harness on the busbars | 9.5/9.5 | 6/6 | 15/15 | 12/12 | 32/32 | 1.0 / 1.0 | 711, 1084, 32 |
| 2: P5, harness on the lid | 9.5/9.5 | 6/6 | 18/18 | 14.5/14.5 | 40/40 | 1.0 / 1.0 | 859, 1289, 38 |
| 3: P6, no harness | 12/12 | 5/5 | 21/21 | 17.5/17.5 | 48/48 | 1.0 / 1.0 | 947, 1479, 43 |
| 4: P6, harness on the lid | 11.5/11.5 | 6/6 | 21/21 | 17.5/17.5 | 48/48 | 1.0 / 1.0 | 991, 1521, 44 |

(Value recovered per stage over the value in the pack; fasteners 0.5 each with their group.) Perception against the
development truth, the same in both rounds: the variant 4 of 4; the placement within 0.44 mm and 0.001 degrees;
134 of 134 seated heads typed right; 8 of 8 missing fasteners seen as holes; overhead position error median 0.64 to
0.72 mm, at most 1.30 mm; wrist position error median 0.27 to 0.28 mm, at most 0.44 mm (the bits need 2.0 and
1.2 mm). Every fastener was driven at the first press; the 23 corroded ones stalled at the first limit and broke away
at the raised one (M6 16.0 to 18.8 N m, M8 38.0 to 40.7 N m). No protective stop, nothing stripped, no wrist frame
taken a second time.

*The hidden set*, run once (`hidden1`: staged as `full_runs.sh` stages the reference, `tests/` copied fresh from
`82e5f2f` with nothing uncommitted, the six program files in `home/submission`, no token, the same start timeout;
then the native verify stage):

| pack | variant, harness | credit | steps (reference) | simulated s (reference) | frames | program s | placed |
|---|---|---|---|---|---|---|---|
| 0 | P4, lid | 1.0000 | 1139 (921) | 768 (666) | 32 | 5.8 | 49/49 |
| 1 | P4, busbars | 1.0000 | 1079 (852) | 712 (601) | 32 | 5.4 | 48/48 |
| 2 | P5, none | 1.0000 | 1301 (1040) | 847 (730) | 37 | 6.2 | 59/59 |
| 3 | P5, lid | 1.0000 | 1326 (1066) | 882 (757) | 38 | 6.4 | 58/58 |
| 4 | P6, busbars | 1.0000 | 1471 (1151) | 961 (804) | 44 | 7.3 | 67/67 |
| 5 | P6, none | 1.0000 | 1493 (1185) | 954 (813) | 43 | 7.3 | 69/69 |

`mean_pack_credit` **1.00000**, ratio 1.000, **reward 1.0**; the probe lock 96 of 96 counted pairs (74 idle pairs
not counted); no protective stop, no stripped fastener, no live contact, no dropped module; the program's own time
38.6 s for the six packs (the budget is 1200 s a pack); stage 1476 s. Of the 170 probe copies the true observation
had the larger residual in 92 (1.1 standard deviations above half), the largest residual 1.6e-6 for both, as for the
reference. There was no second run: nothing failed. The program's stderr is not kept by the verify stage, so the
hidden packs have no perception record; what the baseline gets wrong there is nothing that cost credit.

*What it does not do, and what it says.* It never needed and so never exercised: the extractor bit (it has no code
for a stripped head), its retries (another grasp along the harness, a second pick, the lid's second look), a frame
taken again because of a repeated step. It does not tell rust from the picture (the stall does). It leans on exact
depth: with noisy depth the head-by-height rule and the top-face centre would need another method, untried. The
task is therefore solvable from what a tested agent is given, by a program of about 950 lines of controller and 300
of perception. How hard that is for an agent without the development truth and without the reference's code to
start from is not measured here (section 5).

**2026-11-25, 13:50 to 21:30. The baseline's shortcuts closed: a sensor's depth, each cell's own camera mounting,
sealant over heads, a pack's look (trees `7c6441a` to `9f20b46`; GPU 3 of the author's server only, one Isaac process
at a time).** The perception baseline of the entry above took the job apart through two shortcuts no real cell
offers: exact depth (a head typed by its height in one overhead frame) and cameras exactly at their published poses.
Both were closed, and two more levers of section 5 were added after a revised baseline showed that the job stayed
easy for an expert each time (section 2 describes each change and why it is there; this entry records the order,
the draws and the measurements).

*The order of the work, each rule committed before its values were drawn* (keys of their own in
`hidden/selection.json`, `review/select_set.py`'s docstring):

| step | commit | what |
|---|---|---|
| rule 2 and the sensor | `7c6441a` | `packcell/sensors.py` and its data sheet; cameras' poses on file nominal, `MOUNT_TOL`; the two calibration plates; `review/test_sensors.py` |
| draw 2 | `0ae4b4a` | each hidden and development cell's camera offsets; through the nominal poses a point on the fixture is 23 to 60 mm off (overhead), a point 0.35 m in front of the wrist camera 4.5 to 8.6 mm |
| rule 3 | `1352c44` | sealant over heads (`SEALANT`) |
| draw 3 | `6b259e7` | per-pack sealant seeds; sealed fasteners per hidden pack 9, 10, 8, 11, 12, 7 |
| photographs | `578582a` | the manual's photographs again (plates in view, three sealed heads) |
| rule 4 | `38b809d`, `2f5ce83`, `ed97bd4` | a pack's look (`APPEARANCE`), its ranges in `cell.json` |
| draw 4 | `601152a` | per-pack appearance seeds |
| drawing fix | `b833cfd` | a grime patch near the lid's edge overhung the lid in the drawing (seen in a development survey): cut at the edge; the drawn values are unchanged |
| sensor fix | `9f20b46` | the dark and bright dropouts were decided on the colour divided by the light's gain, which over-brightens a tone-mapped render: in a dim hall the plated heads lost 28 % of their readings against 11 % under the nominal light. The colour is now taken back to the nominal light through the display curve (2.2 gamma and the ACES approximation; it predicts a dimmed render within about two levels) |
| validator tooling | `9b8152b` | `review/check_numbers.py`, a missing fastener's credit, the image's import check of the sensor model |
| the revised baseline | `9cc3b1c` | frozen after its fifth development round |

*That the calibration can be identified* was measured before rule 2 was committed, on a probe cell with offsets
picked by hand (not drawn): both plates in one overhead frame give the overhead camera by PnP to 0.3 to 0.6 mm on the
lid; whole-board wrist views give the wrist camera's mounting to 0.2 mm. On the development cells (surveys with the
truth beside the frames): the lid within 0.3 to 0.74 mm from every overhead frame in which both plates were found;
the wrist mounting from 15 to 16 views, robustly averaged, to 0.14 to 0.34 mm (0.16 to 0.18 mm at a point 0.35 m
ahead). One renderer effect matters for it: the denoiser can wash a nearby colour (the blue harness) over part of a
plate, which the red channel or the darkest channel sees through.

*The reference* (native verify stage, the six hidden packs, a clean export of the commit):

| tree | `mean_pack_credit` | steps per pack | lock | stage |
|---|---|---|---|---|
| `21e2d44` (sensor, mounting) | 1.00000 | 921, 852, 1040, 1066, 1151, 1185 | 83 of 83 (53 idle) | 1599 s |
| `6b259e7` (+ sealant) | 1.00000 | the same | 83 of 83 (53 idle) | 1993 s |
| `9cc3b1c` (+ the look; the final tree) | 1.00000 | the same | 83 of 83 (53 idle) | 2031 s |

The reference asks for no frame and nothing it touches collides with the new drawings, so its packs run step for step
as on every tree since `21ac344`. `tests/anchor.json` is unchanged (1.0, `full_at` 0.95). The probe's copies balance
as before (63 of 136 with the larger residual on the true observation).

*The small checks on the final tree* (`9cc3b1c`): none (an empty submission folder): "no usable submission", reward
0.0; the reference without its token (hidden pack 2): no `truth`, it ends at once, credit 0; the reference with its
applied actions recorded (hidden pack 2): credit 1.0 (59 of 59) in 1040 steps, the lock 10 of 10 (6 idle); a blind
replay of those actions: the same credit in the same 1040 steps, and the lock fails, 0 of 10: reward 0.0; the example
program: credit 0 on every pack, reward 0.0. The commissioning service (natively, as the author's account): the
example program through `run_session.py` (2 steps, 1 frame); an interactive session on development pack 1 right
after: no `truth` in an observation, both cameras decode (overhead 1440 x 1080, wrist 960 x 720), a malformed action
refused, `truth` and `score` unknown requests; two sessions counted, eight left.

*The first perception baseline, frozen at `82e5f2f`, unchanged* (its six program files): on `21e2d44` (sensor and
mounting only) it scores **0.0 on each of the four development packs** and **`mean_pack_credit` 0.00000 on the
hidden set** (reward 0.0, lock 30 of 30). On seven of the ten packs it finds no pack in its first overhead frame (the
noisy depth seen through the nominal pose) and ends after 4 steps; on the other three it works for 600 to 1400 steps
and places nothing (on the development pack among them it had the pack 2 cm and 1.9 degrees off, and every bit
missed its head).

*The revised baseline* (`review/perception_baseline/`, its README says what changed): developed on the development
packs only, under the same information rules as the first (only what a tested agent has; the development truth only
in the separate measurement tools of `dev/`; no `tests/`, no token).

| round | tree | program | dev 1 (P4, harness on busbars) | dev 2 (P5, lid harness) | dev 3 (P6) | dev 4 (P6, lid harness) |
|---|---|---|---|---|---|---|
| nb1 | `21e2d44` | calibration from the plates, six overhead frames averaged, a head from one wrist frame (hull of what stands above the seat) | 0.06 (below) | 1.0 | 1.0 | 1.0 |
| nb2 | `6b259e7` | + the head's outline fitted to its rim (a circle of the pan head's radius, a hexagon of its width), the hull only when its area is a bare head's, up to three sides | 1.0 | 1.0 | 1.0 | 1.0 |
| nb4 | `9f20b46` | + white balance from the plates' white, the lid's silhouette from its convex hull | 0.06 | 0.06 | 1.0 | 1.0 |
| nb5 | `9f20b46` | + tool and bit exchanges cross the cell with the tool's centre at -0.10 m | 1.0 | 1.0 | 1.0 | 0.995 |

The 0.06 of nb1 and nb4 (all lid bolts out, then "the gripper did not close on lid") was the program's own fault. A
trace of the cell's truth showed the freed lid travelling with the nutrunner to the other side of the cell; a copy of
the program that forced the order of the failing runs reproduced it: when the last lid bolt out was one re-typed at
the wrist (and so left to the end), it was ejected into the left fasteners bin, and the swing toward the tool stand
at the dock's approach height (the tool's centre 0.18 m above the table) caught the freed lid's handle (0.206 m)
and dragged the lid off its tray. Rendering is not repeatable, so which bolt came out last changed from run to run.
With the higher transfer and that order forced, the pack scored 1.0. The 0.995 of nb5 is one busbar bolt that
bounced off a fasteners bin's wall onto the table. One more round (nb3, the nb2 program on an intermediate tree of
the look) was stopped after its first pack when a sensor-model artefact was found and fixed (`9f20b46`, above).

Perception in round nb5 against the development truth (`dev/compare.py`): placement within 0.92 mm and 0.02
degrees; variant 4 of 4; the overhead camera so that a point on the lid is 0.48 to 1.05 mm off at most; the wrist
camera's mounting so that a point 0.35 m ahead is 0.18 to 0.19 mm off; fasteners from the averaged overhead frames
median 0.52 to 1.18 mm (at most 2.78 mm), from the wrist frame median 0.14 to 0.24 mm (at most 2.20 mm, a socket
head); every seated head typed right and taken out, every missing one seen as a hole.

*The hidden set, run once* (frozen at `9cc3b1c`; staged as the reference is, a clean export, no token): 

| pack | variant, harness, sealed heads, hall light | credit | steps (reference) | simulated s | frames | program s | placed |
|---|---|---|---|---|---|---|---|
| 0 | P4, lid, 9, 0.80 x nominal, 5108 K | 1.0000 | 1309 (921) | 893 | 64 | 15.9 | 49/49 |
| 1 | P4, busbars, 10, 1.29 x, 6377 K | 1.0000 | 1224 (852) | 832 | 60 | 13.6 | 48/48 |
| 2 | P5, none, 8, 1.14 x, 5115 K | 1.0000 | 1432 (1040) | 937 | 61 | 14.8 | 59/59 |
| 3 | P5, lid, 11, 1.26 x, 5372 K | 1.0000 | 1459 (1066) | 979 | 69 | 16.8 | 58/58 |
| 4 | P6, busbars, 12, 0.98 x, 5207 K | 1.0000 | 1628 (1151) | 1075 | 75 | 17.8 | 67/67 |
| 5 | P6, none, 7, 1.07 x, 6221 K | 1.0000 | 1605 (1185) | 1011 | 68 | 15.3 | 69/69 |

`mean_pack_credit` **1.00000**, ratio 1.000, **reward 1.0**; the probe lock 100 of 100 counted pairs (90 idle pairs
not counted); no protective stop, no stripped fastener, no live contact, no dropped module; the program's own time
94 s for the six packs; stage 2211 s. Of the 190 probe copies the true observation had the larger residual in 101
(0.9 standard deviations above half). There was no second run: nothing failed.

*The effort.* About five hours of development (14:10 to 19:05, interleaved with the work on the cell), of which about
three went into the program: the calibration from the plates (about 40 minutes, with the detector variants the
renderer's colour wash called for), perception for noisy depth (30), the rim fit for sealed heads (30), white balance
and the hull silhouette (20), and the diagnosis and fix of the dragged lid (40). 29 development cell sessions against
the agent's ten. The first baseline took about an hour. Both were written by the task's author, who designed every
lever, had the first version's 950-line controller and the reference's motion code to start from, and the development
truth in the survey and trace tools; a tested agent has none of that.

*What it says about difficulty.* The shortcuts are closed: the first baseline, unchanged, scores 0.00000, and the job
now needs what a real cell needs (calibrating both cameras in every cell, averaging noisy depth, telling a head from
sealant by its geometry, reading colours under an unknown light) on top of the long manipulation program it always
needed. An expert program still takes every pack apart, so the task stays solvable from what is given. Whether a
frontier model lands near the target of 0.3 is not something this server can measure: an agent has to write the whole
program (about 1,700 lines here: 1,070 of control, 450 of perception, 200 of calibration) in six hours with ten
sessions and no truth, and the expert's effort, five hours with every advantage, is the only proxy available here.
The frontier-model runs (`ale run`) decide it.

**2026-11-26, 02:50 to 04:10. The independent review of `846a4f49`, and what changed** (the user chose the scope:
the documents, the GPU list and two grader gaps; the grader on `f2703af6`, the rest on `bfc4b3c0`; GPU 3 of the server
only, every measurement from a clean `git archive` of the committed tree). The review's findings, each with what
was done:

- *B1* (no `ale validate`, no frontier run): not here (no Docker on this server); the validate host does both.
- *B2* (`gpu_types` named only `RTX-6000-Ada`, which no cloud host offers): both tables list `RTX-4090`, `RTX-5090`,
  `RTX-PRO-6000` (the models on which the benchmark validated this Isaac Sim 6.2 stack, `hazard_bay_build`) and
  `RTX-6000-Ada`, the one model this task has run on; the notes in `task.toml` and section 5 say so.
  `scripts/gpu_types.py check`: 0 errors.
- *S1* (three aborts said to restart `run.sh` do not): the documents now say what the code does. A missed deadline,
  a closed output, a line that is not a JSON object, an answer without `action` and (new) too many processes
  restart it; an invalid or rejected action, the budget and the step limit end the pack and the same program gets
  the next one. No change of behaviour.
- *S2* (the budget is checked before a request): stated in the instruction and section 6 of the interface document.
- *S3* (the practice launcher): `run_session.py` writes a request without blocking against its one 30 s deadline,
  sums and prints the program's own time against the budget, stops at the step limit and counts the repeated pairs
  that differ by the grader's rule, from a copy of `wire.canonical` and `is_idle` in `packcell_client`
  (`replies_differ`); the interface document states the rule (1e-4 on the action's 15 numbers, a tenth of the
  compared pairs). `review/test_practice.py` (8 checks, no simulator): the copy gives the grader's values on 4000
  random actions and 2000 pairs; a program that stops reading is given up on at the deadline (4.3 s with a 3 s
  deadline and a 4 MB request; a blocking write hung for good in the review's test), its own time is printed, a
  program that follows its observation has 30 of 31 pairs differ and a replayed stream 0 of 31.
- *S4* (the submission's limits): "at most 5000 files and 200 MB in all; a larger folder is not run and scores 0"
  in the instruction; the zero verdict's reason names both.
- *S5* (no last-resort deadline; busy processes): `verify.py` arms the kit's `VerifierDeadline` at
  `verifier_deadline_s` = 20,400 s (1,200 s inside `[verifier] timeout_sec`); its `on_expire` writes the verdict
  from the packs graded so far, the rest at 0, `verifier_deadline_expired` 1. The kit Solver's `max_processes` is
  64 (the reference and the baseline run one process each behind `exec`). Both are in the instruction.
  `review/test_grader.py` (15 checks, a stub cell, the real kit Solver): `verify.main` with the deadline cut to 7 s
  ends with status 0 at 7.3 s and a verdict of 2 packs graded, `mean_pack_credit` 2/6; with no deadline reached it
  writes all six itself; a controller that starts 40 children has the next pack ended at its reset as a score and
  is started again. Threads are not counted and the controller is not paused: section 5.
- *S6* (a cell failure caused by the controller charged to the environment): on a `CellError` the controller is
  stopped with its process group (as root every process of its account) before a fresh cell starts, and that
  restart is not one of its three; a second failure on the pack counts the pack 0 and grading goes on when the
  controller was started again before it (its GPU memory, memory, disk or load), could not be stopped, or (as root)
  holds 64 MiB or more of files while a disk the cell writes to is nearly full; only a second failure with nothing
  of the submission running is `fail()`. As `longtail_manipulation_recovery` (a disk filled by the controller is a
  score) and `kitchen_appliance_tending` (what the controller can cause is a score). The instruction gives the GPU
  and memory share (below 12 GB of GPU memory and 16 GB of memory). `review/test_grader.py`: no process of the
  controller (nor its helper) is alive when the fresh cell starts; the retried pack is graded in full; restarts 1
  of 4 after one failure, 2 of 5 after two; a second failure while the controller ran is `cell_failed` with credit
  0 and the next pack is graded; a second failure at the cell's start with nothing running raises; with the files
  rule it is scored.
- *S7* (`anchor.json`'s `source` named `8439170`): it names `9cc3b1c` (the measurement), `846a4f49` (the review's
  repeat) and `f2703af6` (this one).
- *S8* (the OpenBLAS pins did not reach the graded cell): the grader's cell gets `OPENBLAS_CORETYPE=Haswell`,
  `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1` as root, as the commissioning cell does; the interface document no
  longer says the controller gets them (it gets `PATH`, `HOME` and `LANG`; the reference and the baseline set them
  in `run.sh`). Only the root path changes, which no native run exercises.
- *S9* (the bit "has to be chosen from the picture"): the instruction and `task.toml` say the head type can also be
  found by trying bits in an order that harms no head, at the cost of exchanges, and which bits wear a head.
- *S10*, *S11*, the reviewer's open question on CUDA's abstract socket and the nits: known limitations in section 5,
  with the review's numbers. Fixed among the nits: the reference's description (it skips perception and state
  estimation: "out", the locks and the isolation come from `truth`; section 4, `solve.sh`), "more torque than the
  default allows" (there is no default: "than a clean one's breakaway"), the stage times in `task.toml` (18 to 34
  minutes), the PR draft's facts.

*The reference on `f2703af6`* (`harder/final10`): `mean_pack_credit` 1.00000, reward 1.0; every pack 1.0 in 921, 852, 1040, 1066, 1151 and 1185 steps (the steps of every tree since `21ac344`), 49, 48, 59, 58, 67 and 69 items placed; no protective stop, no cell retry, no restart; the probe lock 83 of 83 (53 idle pairs not counted); probe copies 136, the true observation's residual the larger in 63; the controller's own time 7.1 s; stage 1429 s. The anchor (1.0, `full_at` 0.95) is unchanged.

*The small checks on `bfc4b3c0`* (`harder/small_checks2.log`): as on every earlier tree: no submission 0 (the zero verdict now names the 5000-file and 200 MB limits); the reference without its token 0 (it ends at once); the reference recorded on hidden pack 2: credit 1.0, lock 10 of 10 (6 idle); its blind replay: the same credit 1.0 in the same 1040 steps and the lock 0 of 10, so reward 0; the example program 0 on all six packs; the commissioning service: `run_session.py` with the example on development pack 2 (it now prints the program's own time, 0.0 s, and the pair count), an interactive session (a move, frames, a refused action), `truth` and `score` unknown on its wire, two sessions counted.

*Gates on the tree of this entry:* `ale lint` on the folder ok; `scripts/lint_domain.py` ok; the kit identical to
`shared/robotics_grader/`; `scripts/gpu_types.py check` 0 errors; `scripts/task_assets.py check` 0 problems;
`review/test_rules.py` 41, `review/test_sensors.py` 11, `review/check_numbers.py` (19 statements, 4 documents),
`review/check_copies.py`, `review/test_grader.py` 15, `review/test_practice.py` 8, all ok; the scan for private
names finds nothing.

**2026-11-26, from 03:54. The first `ale validate`, on an RTX 3090 host (trees `95b843e`, `cf31bdc` and this
entry's).** Where: the author's second server, used as a validate host (Ubuntu 22.04, Intel i9-12900K, one RTX 3090
24 GB, NVIDIA driver 535.183.01), the pinned engine `bfb7505`, Docker 29.10.1 in rootless mode (the account has no
root). Two things of that host differ from an evaluation host and belong to this record:

* The driver is below Omniverse RTX's floor (R550), and the renderer refuses to start on it. `95b843e` turns the
  check off in both copies of `packcell/server.py` (`--/rtx/verifyDriverVersion/enabled=false`, joined to the Kit
  folder arguments), as the endoscope task did for the same host; the canary still judges the frames. `RTX-3090`
  joined both `gpu_types` lists for this run and stays, since validate passed.
* Rootless Docker there cannot set a CPU quota (the cgroup cpu controller is not delegated to user services), and
  the engine passes `--cpus 8` to every sandbox. A wrapper in front of the docker CLI dropped `--cpus`, and a wrapper
  around runc pinned every process started through `docker exec` (the stage commands among them) to 8 CPUs through
  the OCI field `execCPUAffinity`. The lock records `cpus = 8` as requested; the limit was an affinity, not a quota.
  Memory (32 GB) and the GPU were the engine's own.

*First run* (03:58, tree `95b843e`; it ended at the image build, so it has no run id): the Dockerfile's self-check
imported `packcell.scene` as the account `packcell`, and scene's `from pxr import ... PhysxSchema ...` failed
("cannot import name 'PhysxSchema' from 'pxr'"). Isaac Sim's pip install loads those pxr modules through its
extensions, inside a running `SimulationApp` (the cell imports scene after it). The task's fault, never met before
because the image had never been built. `cf31bdc` imports the modules that need no simulator (series, rules, wire,
kin, sensors, cellproc) and compiles scene, cell and server, still as `packcell`; the cell is unchanged.

*Second run* (`validate-8d636430`, tree `cf31bdc`, 04:20 to 04:38):

    ok    grounded-intelligence-initiative/battery_pack_disassembly_to_modules: untouched={'reward': 0.0}, oracle={'reward': 1.0}

The untouched verdict: "no usable submission (no /home/user/submission/run.sh, or more than 5000 files or
200 MB)", verify 8 s, no simulator started. The oracle: setup 0.7 s (the givens staged, the commissioning service
listening on its socket); in the separate verifier sandbox the grader secured the root path, cleared the agent
account (49 processes killed, the desktop's supervisord programs disabled, 4.1 s), ran the controller behind the
seccomp guard, and the canary passed (the cell up in 15.2 s, a pack built and rendered in 125.4 s on a cold cache).
`mean_pack_credit` 1.00000, ratio 1.000, reward 1.0: every pack 1.0 (49, 48, 59, 58, 67 and 69 items placed) in 921,
852, 1040, 1066, 1151 and 1185 steps, the native runs' steps exactly, 91 to 123 s of wall clock a pack; no
protective stop, no cell retry, no restart; the probe lock 83 of 83 (53 idle pairs not counted); probe copies 136,
the true observation's residual the larger in 63; the controller's own time 3.7 s; verify 866 s (natively 1429 to
2031 s, on a server shared with other jobs).

*In the image* (the same build, no GPU, no network): `review/check_copies.py`, `review/test_rules.py` (41),
`review/test_sensors.py` (11) and `review/test_practice.py`, and, as the image's account `user` on a writable copy,
`review/check_numbers.py` (19 statements, 4 documents current) and `review/test_grader.py`: all ok.

*What was decided.* `tests/anchor.json` keeps 1.0, its source now this run; `full_at` stays 0.95 (the validate host
repeated every pack's steps, so there is still no spread to size it from). `RTX-3090` stays in `gpu_types`. A
confirmation `ale validate` on the commit of this entry follows; its `ok` line goes into the pull request's
description. Not exercised: an agent phase in the engine (the commissioning sessions through the agent's client;
a frontier episode on this host follows), any cloud GPU model, a host with a CPU quota.

**2026-11-26, from 07:52. The second hardening round, after a frontier episode scored 0.84, and the re-review's
findings** (the user chose another round and the fixes in one pass; session `battery-harder2`, GPU 3 of the server
only, after the endoscope session's runs).

*The episode* (`4c00590b`, Claude Opus 5.5, claude-code 2.1.309, effort high, no web tools, on `33ce4599`; its
trajectory read in full). Reward 0.84 (`mean_pack_credit` 0.795; packs 0.785, 0.892, 0.888, 0.438, 0.884, 0.885; 100
protective stops, one stripped head, lock 114 of 114) in 70 of its 360 minutes. One interactive session on a
development pack gave it, within 22 minutes, both cameras' calibration from the plates (0.1 to 0.2 px), the lid's
outline in depth and the variant from the ribs, each head found and typed in one close wrist frame, and the torque
trace's engagement test; nine runs of `run_session.py` went to bugs. It ran every fastener at the "highest safe
torque" (M6 20.5, M8 44.4 N m), lifted every module, and its own list of what it never ran on the cell holds the
extractor and the blind try at a fastener it cannot see. Every lever of the first round was one identification step
with a documented recipe; the program barely managed parts that stayed held (a missed lid bolt lost a development
pack; 100 stops in grading came from lifts against held parts).

*The levers* (section 2, "The second round"; rules 5 to 7 of `review/select_set.py`, committed in `24ab841c` before
their keys were drawn in `9215ee1a`; nothing redrawn): seized fasteners with the shank's shear torque and working
cycles; swollen modules that stay in their seats; foam pads over fasteners; the hidden packs combining them as the
development packs do not (hidden: seized 2 to 4 a pack, 15 in all, 5 of them lid bolts; swollen modules in packs
0, 1, 3, 4; pads over 7 fasteners in 5 packs, 6 of them lid bolts. Development: one seized fastener each, swollen
modules in packs 1 and 2, pads over L03 in packs 0 and 2). Why each, and what was not used and why: section 2.
Simulator-free checks: `review/test_rules.py` checks 16 to 23 (the shear torque either way, working, the Torx bit at
a stall, the head's limit both ways, the swollen module's vent and credit, the draws' statistics, a leak test: no
key or seed of the new rules in a file the agent can read).

*N1 to N3 and the nits of the re-review of `33ce4599`.* N1: `wire._num` and `packcell_client._num` treat a JSON integer
too large for a float as a malformed number (both copies; `test_rules.py` check 21 on `check_action`, `canonical`,
`is_idle` and the client; `test_grader.py`: a 400-digit integer applied and in a probe pair ends the pack as
malformed, never an `OverflowError`); no other numeric parsing on the wire had the hole (`ticks` is an integer
compared exactly). N2: option (a) of the review: before a cell retry the grader empties the account (every file of
it outside the submission, without a bound; its SysV IPC objects) and restores the submission from a root-only copy
taken at staging; the disk-charge branch is gone; a clean-up that fails counts the retry as the controller's.
N3: `GuardedSolver.request` counts the account's processes before every request (as `cable_routing_through_clips`).
Nits: the deadline is disarmed after the last verdict; `controller_restarts` leaves out the grader's own
(`grader_restarts`); the instruction states that a controller that cannot be stopped scores the pack 0 and, at the
end, the run 0; the PR draft's frontier row, folder size and `full_at` claim corrected.

*A lever corrected after a development run.* The baseline's calibration failed on development pack 2: a pad over L03
overhung the lid's long edge by up to 28 mm, and seen from the overhead camera its top covered inner rows of the near
calibration plate, where the interface document says the camera sees both plates whenever the arm is out of the way.
Since `e4927a6e` a pad on the lid is drawn cut off at the lid's outline (`scene.clip_rect`, as the grime is); no
drawn value changed, every head and hole under a pad stays hidden, and the manual says so. In the hidden set no lid
pad had overhung toward a plate's rows (from the drawn outlines).

*Simulator-free checks of the new code* (outside the repository): the stage built with plain `pxr` for all ten packs
(pads under their parts, every module's bump at its drawn height); a kinematic stub of the nutrunner with the real
`rules.Rules`: the reference's and the baseline's `unscrew` took every fastener of the hidden set out unharmed (167
clean, 14 corroded, 15 seized), and the baseline's touch routine found or ruled out and typed all 142 lid and busbar
fasteners, centre within 0.30 mm.

*The reference* (GPU 3, shared with another job of the account from 12:55). `final1` on `87ad1f2d`:
`mean_pack_credit` 1.00000, every pack 1.0 (steps 919, 843, 1067, 1062, 1149, 1208; 584 to 823 s simulated), lock 85
of 85 (52 idle), no stop, nothing stripped or sheared, no vent, 34 working cycles in all, the swollen modules counted
in their seats. `final2` on `dca001d1` (the cell with the pad cut, `e4927a6e`): 1.00000 again, every pack in the
same steps as `final1`, lock 85 of 85; both stages about 1600 s with GPU 3 shared. `tests/anchor.json` is unchanged
(1.0, its source the `ale validate` run of `33ce4599`'s tree; the validate host records the anchor from its oracle
again). The small checks on `3be3069b` (stages under the session's own folder): no submission 0, no simulator
started; the reference without its token 0; the recorded reference on hidden pack 2 1.0 with the lock 9 of 9; its
blind replay credit 1.0 but the lock 0 of 9, reward 0; the example 0 on all six packs; the commissioning service
(a `run_session.py` run, an interactive session, a refused action, `truth` unknown on its wire): all as before.

*The frozen Opus program* (`4c00590b`'s submission, rebuilt from its trajectory: every write into `~/submission`
replayed in order; all nine files have the sizes of the episode's last listing and `mission.py` matches the three
reads the episode made of it, line for line). Development packs, cell of `83b7da4e`: pack 0 credit 0.054 (it runs
M6 at 20.5 N m: the seized lid bolt L06 sheared, the program logged it "freed", the lid stayed), pack 2 0.802 (the
seized bracket bolt R3bf stripped by its extractor; bracket R3b and the swollen M3 stayed held, so M3 was never
lifted). Hidden set, cell of `83b7da4e`, that is before the lid-pad cut of `e4927a6e` (not run again on the final cell, to
push sooner): **`mean_pack_credit` 0.16489, reward 0.174** (0.795 and 0.84 on `33ce4599`): packs 0.067, 0.419, 0.062, 0.326, 0.053, 0.062; in four packs
a seized lid bolt sheared and the lid stayed; in two the lid came off, a seized busbar and a bracket bolt sheared,
and lifting the swollen M1 vented it; 9 sheared, 3 stripped, 718 protective stops, lock 54 of 54. The cut only
stops a lid pad's drawing at the lid's edge: on the hidden packs it removes the overhang of the lid pads beyond the
short sides (packs 0, 1, 3) and the long sides away from the plates' rows (packs 3, 5); no pad value changed, every
head stays hidden, and both cameras saw both plates before it. Every credit this program lost on the hidden set
came from a sheared seized fastener or a vented swollen module, which the cut does not touch, so its result is not
expected to move. This number
shows only that the first round's program does not handle rules 5 to 7; it does not predict a fresh agent's score.
Every failure here is one a fresh agent avoids by reading sections 5 and 7 of the manual, and the evidence says
fresh agents do: in round 1 the old baseline fell to 0 and a fresh model reached 0.84, and on the endoscope task a
frozen program fell from 0.88 to 0.114 while a fresh Opus 5.5 episode on the same version then scored 0.92. The
difficulty number is the next fresh episode's. Which lever a careful fresh agent is most likely still to lose credit
on, from what the documents give it: the pads (L7). Under the hidden lid pads sit four T30 heads 0.44 to 1.96 mm from
their nominal places and two S10 heads 1.84 and 1.87 mm off, against a Torx bit's 1.2 mm and a socket's 2 mm of full
engagement; nothing but touch locates them, a guess outside the tolerance wears the head, and a lid bolt left in loses
the pack. The development packs show one pad each in two packs (S10 1.67 mm and T30 1.46 mm off), so a blind try at the
nominal place half works there. Next the seized fasteners (L5: a per-fastener state machine near the shear torque,
a Torx bit kept pressed for the torque it holds, one instance per development pack). Least the swollen module (L6:
the hidden bulges are 1.07 to 2.29 mm, far above the 0.4 mm criterion, and the manual says what to do).

*The revised baseline* (`review/perception_baseline/`, its README's "The third version": seized fasteners worked from
the torque trace, pad-covered fasteners by touch, module bumps measured at the wrist; developed on the development
packs only, six development sessions and about an hour and a half of work, the levers' designer's, with the
development truth only in `dev/devrun.py`'s comparison afterwards). `mean_pack_credit` 1.00000, reward 1.000 on the hidden set (one run of the program frozen at `3be3069b`, on the final cell): packs 1, 1, 1, 1, 1, 1; 1379 to 1842 steps, 974 to 1234 s simulated; lock 125 of 125; 0 protective stops, 0 stripped, 0 sheared, 0 vented; the program's own time 142 s for the six packs (stage 2781 s, GPU 3 shared). The task can be
done from what an agent is given, after the second round too.

**2026-11-26, from 16:31. The second round's version through `ale validate`, on the RTX 3090 host (tree
`ce208f10`).** The host of the entry "2026-11-26, from 03:54" above, with the same two deviations from an evaluation
host (the cell's driver check off, still in both copies of `packcell/server.py`; the CPUs limited by an affinity of 8
instead of a quota) and the same engine `bfb7505`. The image was rebuilt (`environment/` changed); nothing of the task
had to change for this host.

*Run* `validate-f1ab66c2` (16:32 to 17:06, the image build included), first try:

    ok    grounded-intelligence-initiative/battery_pack_disassembly_to_modules: untouched={'reward': 0.0}, oracle={'reward': 1.0}

The untouched verdict: "no usable submission", verify 10 s. The oracle: setup 0.7 s; in the separate verifier sandbox
the root path secured, the agent account cleared (49 processes), the controller behind the seccomp guard, the canary
passed (the cell up in 14.9 s, a pack built and rendered in 123.8 s on a cold cache). `mean_pack_credit` 1.00000,
ratio 1.000, reward 1.0: every pack 1.0 (49, 48, 59, 58, 67 and 69 items placed) in 919, 843, 1067, 1062, 1149 and
1208 steps, the steps of the second round's native runs `final1` and `final2` exactly, 89 to 126 s of wall clock a
pack; no protective stop, no cell retry, nothing stripped or sheared, no module vented; the probe lock 85 of 85 (52
idle pairs not counted); probe copies 137, the true observation's residual the larger in 77; the controller's own
time 4.2 s; verify 864 s.

*What was decided.* `tests/anchor.json` keeps 1.0, its source now this run; `full_at` stays 0.95. No second validate:
the commit of this entry changes only `tests/anchor.json` and this file, so the `ok` line above stands for it (the
pull request's description says so). A frontier episode on the pushed commit follows; its numbers go into the pull
request's description. Not exercised in the engine: the second round's levers in an agent's hands (the episode
follows), any cloud GPU model, a host with a CPU quota.

**2026-11-26, from 19:00. The third hardening round, after a fresh frontier episode scored 0.97** (background
session `battery-harder3`, started by the 3090 host's waiter; GPU 3 of the server only, shared by turns with the
endoscope's session). *The episode* (Claude Opus 5.5, run `ec7bd269` on `c8b2edc5`, reward 0.97, credit 0.926, 75 of
360 minutes, all ten sessions): one interactive session to try everything once, the controller written in five calls,
a mock cell of its own, nine runs on the development packs. It turned the manual's working cycle into fixed torques
(M6 19.5 / 18.6 / 12, M8 44.5 / 42 / 30 N m), used the nominal places as a map that a 2 mm socket tolerance made exact,
read engagement from a 3-tick test turn on a clean trace, measured each module's bump with a fixed shape against
bulges far above the criterion, and skipped what would not come out, at no other cost (0 stops, strips, shears or
vents; packs 0.973, 0.973, 0.860, 0.949, 0.860, 0.942). The program was rebuilt exactly from the trajectory (every
file the size of the agent's own last listing; its `sed -n` prints equal) and frozen outside the repository.

*Levers* (rules 8 to 12, committed `2bff9457` and `14648cf0` before their draws `b270feae` and `3c89b0c8`; section 2):
precision engagement with damage at once, every shank its own with its yield point in a noisy torque-angle trace, a
cell's own transducer, safety events that condemn a pack and a latch at the third stop, swollen modules at the
criterion; with the study of the merged arm tasks (the user's request, 19:22, relayed by the starting session) a
completion gate per pack, an allowance of 1500 s simulated a pack, the documents' procedures removed; and the
conditions the user approved (19:27): three commissioning sessions never refunded, a session of 2 h. Rule 8 was
amended before any run (damage "within 10 degrees" would have let a slow test turn read a partial engagement cheaply:
now at once under a 1 N m limit). A defect found by the first reference run and fixed (`e664be14`): drawn again under
rule 11, the module that rule 6 had chosen kept its seat as its destination.

*The reference* (`ref2` on `e664be14`, then `ref3` on `383db135`): 1.00000 both times, every pack 1.0, lock 95/95
and 101/101, no stop, strip, shear, drop or condemned pack. `ref2` showed the one thin margin: pressed on, the arm's
drives leave the tip up to 0.395 mm off its command, against the Torx bit's 0.5 mm; the reference now lifts and
presses again with the command shifted by its tip's offset (privileged), and `ref3` engages within 0.023 mm. Its other
distances: tilt 0.081 degrees (limit 2), 0.97 of a yield point (shear lies 8.7 % or more above it), a freed fastener
at 1.45 m/s^2 (4), 0.105 degrees (25), 0.42 mm sideways before it was out (1.5). Slowest pack 1148 s: the allowance
is 1.31 times it.

*The frozen Opus program* (unchanged; a lower bound on a fresh agent, never an estimate of one): development pack 2
(P6) credit 0, ended by the latch at 415 s simulated. Its 3-tick test turn read the free spin of a socket on a Torx
head as 0.39 to 0.56 N m (that cell's transducer offset and the noise; its threshold for "turns freely" is 0.4), so it
judged the head worn, ran the extractor on the sound head (stripped at once), and the reamed recess took no
extractor; the lid stayed bolted and three lifts made three protective stops. On the hidden set
(`opus_hidden`, tree `e430a7ca`): `mean_pack_credit` 0.00000, lock 63/63; every pack latched while lifting a lid
still bolted, after its fixed limits (19.5 N m to loosen, 18.6 N m to work, commanded as readings) sheared weakened
lid bolts (3, 3, 3, 1, 1, 1: 12 shanks) and its extractor fallback stripped sound heads (0, 2, 0, 1, 0, 0); 2 shanks
loaded past their yield point, nothing dropped from the bit, no pack condemned; 5 to 13 items placed before the
latch. It was 0.926 on the second round's cell. This says that the episode's program cannot do the third round's
job; it says nothing about how far a fresh agent gets.

*A fresh-agent proxy.* Written down before freezing what a careful agent would build from the new documents in two
hours with three sessions (the notes of the session that made it): from the episode's own program, a head centre
measured from two averaged close wrist frames with sealant rejected; a low-limit contact check that harms no head; the
cell's transducer calibrated on site from the free-running drag and fasteners running out; a stall ramp that reads
the wind-up per step from the angle trace and stops at the yield point, then works the thread just below it; freed
fasteners carried under the magnet's limit; the documented bump shape fitted over the rim plane, swollen at 0.39 mm.
I adapted the frozen program that way myself from 20:00 to 22:41, with development material only: the program's own
logs of three sessions (dev packs 0, 2 and 2 again; the truth and the credit sealed until it was frozen), the
documents, and a Monte Carlo of the ramp against the documented model. Session 1 (P4): every fastener out, four yield
points found and worked; the pack failed on a module wrongly kept as swollen (its fit had dropped the rim). Session 2
(P6): noise taken for yield points left two lid bolts in; latched. Session 3 (P6): every fastener out by the
program's reading, six yield points worked; then a bracket would not lift (its bolt, a clean S13, had been stripped:
the socket 0.80 mm off), three stops, latched. *v1, frozen at 22:41 and run once on the hidden set*: `mean_pack_credit`
0.00000 (lock 46/46): in every pack one seized or corroded lid bolt stayed in (three because my last change before
freezing capped the ramp at 17.8 N m, under the highest documented yield point of 18.4; one sheared on a second ramp
above a supposed knee; two did not come loose in the cycles it gave them), so the lid stayed and the pack did not
count; no latch (one stop a pack), nothing condemned. These are the proxy's own torque logic, not missing
information. *v2* (disclosed: informed by v1's hidden records, so a bound for an expert with more time, not an
estimate of a fresh agent): v1 with the caps at 19.5 / 44 N m (above every documented yield point), no second ramp
above a supposed knee, 45 working cycles and the tightening stall at 1.12 times the assembly torque. On `e430a7ca`:
0.00000 again (lock 135/135), but the lid came off in five of six packs; what kept every pack from counting were nine
stripped bracket bolts, nearly all the S13 hexagons on the service-bay side, each leaving a module held (pack 0 also
latched; pack 5 ran 1475 of its 1500 s). The camera check had shown why: the locator's fit sits 0.6 to 0.8 mm off on
those deep S13 heads.

*The amendment* (`0f32dbb8`). With the fresh-agent proxy at 0 and the improved one completing no pack, the brief's
rule applies (a lever that went too far is corrected): the 13 mm socket now drives within 1.2 mm (its larger lead-in
chamfer), the 10 mm socket still within 0.8 mm and a Torx bit within 0.5 mm. Rule 8 draws no values; nothing was
drawn again. The small checks ran on `e430a7ca` (none of them comes near the S13 edge; the reference engages within
0.023 mm). FINAL_RERUNS

*The camera-only check* (fairness: is the information there?): the proxy's wrist locator against the sealed truth of
its three sessions: Torx heads 50 of 51 within 0.5 mm (T30 median 0.07 mm), S10 heads 8 of 8 within 0.8 mm (median
0.28), S13 heads 16 of 18 within 0.8 mm (median 0.63, max 1.04, a common offset of 0.6 to 0.8 mm along x on the deep
bracket heads: a systematic bias of its circle fit that an expert can measure by touch and correct). The documented
yield model, simulated (4,000 draws from the documented ranges each, outside the repository): the proxy's final ramp
sheared no M6 or M8 shank; of the M6 fasteners that stalled it found a usable yield point on 1,473 of 1,480 (7 worked
below 80 % of their shear torque), of the M8 ones on 1,391 of 1,403 (12 reached its cap without a knee).

## 7. Final report (2026-11-24, 16:30 EST)

**Since this report:** the kit of upstream #173 and a rebuilt probe copy (section 6, the entry "18:20 to 19:00"),
then an exact conversion of a pose near a half turn (the entry "19:10 to 20:30"), then a cap on how fast the
physics pushes a fastener out of an overlap (the entry "2026-11-25, 01:20 to 02:40"). The reference scored
1.00000 on `152fdf7`, 0.99440 on `15d8694` (five fasteners of two packs thrown out of the bins when the reference
released into its own pile, section 5), and 1.00000 on `8439170`, which has the cap. The anchor is 1.0 again,
measured on `8439170`; the small checks and the probe lock are as before on every tree. The numbers below are the
earlier trees'.
Then a perception-based baseline (`review/perception_baseline/`, the entry "2026-11-25, 10:00 to 11:35") scored
`mean_pack_credit` 1.00000 on the hidden set without the token, reward 1.0, lock 96 of 96: the task can be solved
from what a tested agent is given (this supersedes "A human-expert, perception-based solution. None exists" below).
The task's files, the reference and the anchor are unchanged.
Then (the entry "2026-11-25, 13:50 to 21:30") the first baseline's shortcuts were closed and two more levers added:
a sensor's depth, each cell's own camera mounting identified from calibration plates, sealant over heads, a pack's
look. The reference scored 1.00000 on every new tree in the same steps (the anchor is unchanged); the first
baseline, unchanged, scores 0.00000; the revised baseline scored `mean_pack_credit` 1.00000 on the hidden set (reward 1.0; one run of a program frozen on the development packs before it). The validator's checks that need no Docker
are in section 5.
Then (the entry "2026-11-26, 02:50 to 04:10") an independent review of `846a4f49` and its fixes: the documents say
which aborts restart `run.sh` and which do not, that the budget is checked before a request, the submission's limits
and the bit by trial; the grader arms the kit's deadline, caps the account's processes at 64, charges a cell
failure to the submission only when it can have caused it, and gives the graded cell the image's OpenBLAS pins; the
practice launcher runs a pack as grading does (deadline, own time, the pair count); `gpu_types` lists `RTX-4090`,
`RTX-5090`, `RTX-PRO-6000` and `RTX-6000-Ada` (the task has run on the last only). The reference scored 1.00000 on
`f2703af6` in the same steps; the anchor is unchanged; the small checks are as before on `bfc4b3c0`. The `gpu_types`
paragraph below is superseded by section 5.
Then (the entry "2026-11-26, from 03:54") the first `ale validate`, on an RTX 3090 host: the image's self-check
corrected (it imported a module that needs the running simulator), the renderer's driver check off for that host's
driver; untouched 0.0, oracle 1.0, the reference's packs in the native steps. The anchor's source is that run, and
everything below that says the engine has not run is superseded by it.
Then (the entry "2026-11-26, from 07:52") a second hardening round after a frontier episode scored 0.84 on
`33ce4599`: seized fasteners with the shank's shear torque, swollen modules that stay in their seats, foam pads
over fasteners, and the re-review's N1 to N3; the reference 1.00000 again, the frozen frontier program 0.165. The
validate and frontier numbers above and below are `33ce4599`'s: a new `ale validate` and a new episode follow.
Then (the entry "2026-11-26, from 16:31") the second round's version through `ale validate` on the RTX 3090 host:
untouched 0.0, oracle 1.0, the reference's packs in the second round's native steps; the anchor's source is that
run.

**State.** The folder is complete and committed on the branch `claude/battery-pack-disassembly-8e1039` (not
pushed, no pull request, nothing outside the folder edited, the coordination sheet untouched). The work stops
here because everything left needs Docker and the engine, which the author's server does not have.

**Verified on the author's server (native, outside the engine; RTX 6000 Ada, driver 570.195.03, Isaac Sim
6.2.0.0, PhysX on the CPU, every simulator process confined to GPU 0):**

- The static gates listed in the last status entry.
- The grader's episode loop end to end (`tests/verify.py` as an ordinary account: the kit's `Solver`, the probe
  lock, one fresh cell process per pack, the kit's scoring and verdict): no submission scores 0.0; the example
  program scores 0.0; the reference scores `mean_pack_credit` 1.00000 = reward 1.0, three times; the reference
  without its token scores 0; a blind replay of the reference's own actions earns the same credit and is refused by
  the probe lock.
- The commissioning service with the agent's client library and `run_session.py`, as the author's account.
- The image's download of the robot model.
- What a non-privileged program can read from the frames, for the lid's fasteners only (section 6, the table).

**Not verified, because it cannot be here:**

- **The Docker image.** `environment/Dockerfile` was written after `hazard_bay_build`'s and never built. Unknown
  until it is: that the build succeeds, the agent-UID import check, the pinned settings' effect in the container.
- **Everything the verify stage does only as root**: the cell under its own account `packcell` from a private
  copy, the seccomp launcher `nonet_exec.py` around the submission (the native runs print `egress guard: none`),
  the kit's clearing of the agent account and securing of the path, the records under `/opt/arb-verify`, the
  verdict through `ale_verify` into `/logs/verifier`.
- **`environment/setup/setup.sh`** and the commissioning service under its own account behind the image's
  permissions (that the agent cannot read the service's folder or the development packs' draws rests on file modes
  that were never exercised).
- **`ale validate`**: the untouched sandbox at 0.0 and the oracle at 1.0 inside the engine. The anchor has to be
  measured again there; if the engine's hosts give the reference less than 1.0 on some pack, the anchor and
  `full_at` follow that measurement, not this one.
- **`ale run` with frontier models.** The difficulty is unmeasured. The target (0.3 or below) is a design intent.
- **A human-expert, perception-based solution.** None exists; the reference is privileged (section 5).
- **Any GPU model but the RTX 6000 Ada**, and any CPU but this server's.

**`gpu_types`.** Both tables name `["RTX-6000-Ada"]`, the one model the task has run on. The engine matches a name
by its words in order, so this accepts exactly that model. The models the benchmark's cloud hosts offer (RTX 4090,
RTX 5090, RTX PRO 6000, H100, H200) are not named because they were not tried: the task needs an RTX-capable GPU
for the cameras, the renderer is not repeatable even on one GPU, and a perception program's behaviour can depend on
the denoiser the driver picks. Before a model is added: the canary's frames on it and one reference pack.
`hazard_bay_build`, on the same Isaac Sim stack, lists the first three (`RTX-4090`, `RTX-5090`, `RTX-PRO-6000`),
which is the reason to expect them to work and to try them first; it does not list H100 or H200, and whether Isaac
Sim's renderer runs on those was not looked into here: they should be named only after a run. As long as the list
holds only the RTX 6000 Ada, a host without that model cannot run the task, so the first step of validation is to
try the image on a cloud model and extend the list.

**Decisions left to the owner** (the conservative choice was taken in each; section 5 has the reasons): the task's
name; the value weights; that the head type can be found by trial at the cost of time; `full_at`; which GPU models
to admit; whether to make the task harder or easier once models have run it.

**Next commands** (on a host with Docker, from `vendor/ale`):

```bash
VIRTUAL_ENV= uv run ale lint ../../tasks
VIRTUAL_ENV= DOCKER_HOST=unix:///var/run/docker.sock uv run ale validate \
    ../../tasks/arm/battery_pack_disassembly_to_modules --runs-dir /tmp/ale-runs
```
