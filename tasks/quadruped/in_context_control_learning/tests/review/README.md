# V9: split-deck inspection

Scope: one terrain and support change at the original first camera stop.
The first approach is ordinary proximity. The first dock adds a physical
bearing condition to the same quiet dwell: at least 8 N from soles on each of
the two runner tops. Side, underside, web and sump contacts do not count.
Neither a declared wiring map, foot identity, contact order, cumulative
contact count nor sensor-reading flag can satisfy this predicate. The
simulator does not change the terrain on an advance. All original points,
six route centres, headings, native time limits and participant tools remain.

See [case registration](support-registration.md), [paired trials](support_pair.py)
and [geometry check](check_support.py). The optional `--mcp` bench operates the
released Python stdio transport over a real Unix socket; the script is an
author, never an eligible participant. Its inputs are the published dimensions,
own action history, current onboard PNG and public milestone bits. The harness
knows the seeds for pairing and logs instrumented foot data separately; it
does not send that data to the operator. The literal wire requests and PNG
hashes, private journals and replay results are separate evidence, not model
trajectories. Historical V8 numbers below are not V9 measurements.

In the pinned CPU task image, use a private writable evidence mount and one
worker. For the MCP bench provide an empty writable `/run/visual_robotics`:

```sh
python3 -B /task/tests/review/check_task.py
python3 -B /task/tests/review/support_pair.py --mcp --end 6 --poses 4800 --output /evidence/paired
python3 -B /task/tests/review/support_pair.py --mcp --matched /evidence/paired --output /evidence/ordinary
python3 -B /task/tests/review/support_pair.py --mcp --matched /evidence/paired --control fixed-left --output /evidence/fixed-left
```

The 4,800-pose endpoint exists only in that diagnostic invocation. Native
validation uses the unmodified ALE CLI and the task's original timeouts.
Do not infer participant difficulty, success rate, or a conversational
learning effect from this author bench.

## Historical V8: service aisle commissioning

The six V7 centres, required orientations, launch pose, motor gains, balanced
24-way connector lot and original scene IDs are retained. The new plant is
fixed in the physical world on both sides of the mirrored route. No path was
lengthened and there is no new participant tool or quota. The licensed
Menagerie Go1 XML, native actuators, meshes, free base and explicit trunk
inertial remain intact. Two visible fixed collision geoms represent the
existing onboard camera's protective hood and stanchion. Both the solver and
the separate verifier integrate the same MuJoCo geometry.

## Physical work and partial credit

The launch rack still indicates the local presence of real soles only through
the labelled board in the onboard RGB view. Commissioning can earn one point
for low clearance and one for banking left. Each of six docks pays one point
for upright approach and one for quiet, aligned parking: reward = points/14.
There is no reward for naming a gait or identifying a hidden symbol.

Dock 1 is inside a guarded motor bay. Its open doorway to dock 2 is under a
rigid, striped duct. Back/side panels, a sight-through steel guardrail and
jambs close the rest of that bay. The original stance's camera hood cannot
fit through the nominal 0.38-0.39 m opening; passing requires an actually low
envelope, whether achieved by commanded posture, tilt or legal contact.
Dock 3 is physically on top of a 4.6-5.6 cm cable inspection cover with a
1.8 cm leading lip and beveled apron. The normal quiet dwell also needs two
loaded soles on the flat deck surface. Proximity, edge brushing or hovering
do not qualify as top support. Foot order is unrestricted. Roll, pitch, fall
and completion all use physics; no impact counter reduces the score. Review
metrics retain first duct crossing, last-foot clearance, contact ticks, deck
support, centre height at parking and all original milestone pose/tick data.
Numeric physics telemetry is verifier-only.

Native participant limits are unchanged: each 12-joint pose holds for 1-500
ticks of 2 ms; six hours of agent wall time and seven for separate verification.
No task-wide action, observation, sim-time, recovery or retry cap or fee.
Recoveries restore the physical checkpoint, never consumed resources or earned
milestones. Report external author diagnostic truncation separately from a
physical fall or successful completion; a fixed-policy prefix is not a bound
on a participant who observes and adapts.

## Reference and independent controls

On the first registered scene the ordinary adaptive controller parked at all
six docks with 2/2 calibration, 6/6 approaches and 6/6 supported/aligned stops.
It used 1,335 poses / 21,453 ticks = 42.906 simulated seconds: 5 calibration
poses / 2.230 s and 1,330 route poses / 40.676 s. These are measured costs, not
limits. The posture switch operates in the author only; the route controller
and simulator never read a historical participant journal.

The flat-only control retains RGB feedback, physical port probes, parking,
steering and fall handling but uses ordinary flat-floor joint motion throughout.
The stay-low control is identical through dock 2 and continues low instead of
adapting to the raised cover. Both receive the adaptive partner's observed pose
opportunity. Hashes of actual journal prefixes check comparability before the
intervention. Every observed prefix, fall, truncation and genuine success counts.
These tests measure physical feasibility and sensitivity; they are not blind
model evaluations or evidence of improved conversational memory.

`open_loop.py` is an optional **offline** historical transfer. It reads the
owner-provided, originally audited native event log, passes all recorded
operations and arguments in the original order, and discards replies. Rejected
commands after divergence are recorded, not changed or omitted. Prove exact
reproduction against the original immutable V7 world first. V8 gets a fresh
journal and exact headless replay. No model is queried; no recorded actions
or private scene material are shipped in participant instructions or payload.

## No-paid reproduction

Use the pinned ALE engine's `ale lint "$TASK"` and
`ale validate "$TASK" --runs-dir "$FRESH_RUNS"`. For author-only checks in the
engine-built CPU task image, mount sealed source read-only at `/task` and
a unique private evidence directory at `/evidence`:

```sh
python3 -B /task/tests/review/check_task.py
python3 -B /task/tests/review/reference_check.py --all-hidden --workers 2 --author-poses 3200 --output /evidence/adaptive24
python3 -B /task/tests/review/reference_check.py --first 6 --mode flat --matched-adaptive /evidence/adaptive24 --output /evidence/flat6
python3 -B /task/tests/review/reference_check.py --first 6 --mode stay-low --matched-adaptive /evidence/adaptive24 --output /evidence/low6
python3 -B /task/tests/review/open_loop.py --history /evidence/history --output /evidence/transfer
```

The 3,200-pose author diagnostic horizon exists only in that standalone review
command, not in native validation or the task. Earlier V7 commissioning results,
including the original paid participant, belong to `b3bdf659c50986d7a95c09884231a7452a77bbb8`.
The old flat-lane straight-line benchmark in `turn_controls.py` used V7's open
floor: run it at that parent, not against the V8 bay boundary.
