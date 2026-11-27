# Warehouse navigation scoring and calibration

## Assigned courses

A protected draw uses sixteen fresh courses. Eight pivot left and eight right. The approach distance is stratified over 5.50–7.25 m, pivot magnitude over 80–100° and departure station distance over 3.80–4.30 m. The stages share the same free-root G1 trajectory. The verifier uses separate random 64-bit identities for the robot, approach geometry, shutter phase and outgoing racks. The course distribution and thresholds are public. Individual draw identities and live geometry are private.

Each pivot group contains four dual-service shutter forks, two left-service and two right-service forks. Four approach courses in that group have a second, offset cabinet. Two of those cabinets open to the robot’s left and two to its right. At the fork, independently phased shutters sweep across actual apertures. A restricted leaf can move visibly but exposes at most 0.12 m. Passage through any physically open side qualifies.

Both outgoing aisles exist before the pivot cue. Independently shuffle each arm’s profile list within each group of eight. Each arm receives three low racks offset left, three low offset right, one high offset left and one high offset right. Thus twelve selected racks are low and four are high. The active lateral offset is left on eight trials and right on eight. The other aisle has the same marginal distribution. The protected grader assigns fork, roof, width, rack length and station distance before the episode. The solver sees the egocentric camera and robot-local signals.

## Physical success

Only a strict full-course completion earns raw credit. The official score equals strict completions divided by all sixteen assigned courses.

- On the approach, the pelvis passes the actual moving-leaf plane and then the fixed fork frame through the same opening. The verifier reconstructs the opening using the measured native slide at the crossing. Any downstream approach cabinet must also be traversed through its physical opening, in order.
- The approach, signed pivot and departure each require the native 0.75 s quiet stop. The approach requires another 1.50 s of continuous standing after its first accepted stop. The approach and pivot reveal the next cue only on a five-row boundary. The departure ends on final stop acceptance.
- In the outgoing aisle, the pelvis crosses the rack’s front and back inner spans. At a native sample the entire collision head mesh must lie horizontally under the physical deck with its apex at or below the actual underside. The measured last point of all active robot collision shapes must subsequently clear the rack’s back plane. The protected verifier checks all three event positions, signed clearances, clock intervals and the corresponding physical branch.
- Every native 2 ms interval is checked for actual robot contact with any obstacle, including shutters, posts, shelf decks, fixed cabinets and side guards. Contact on entry or after pelvis exit still disqualifies the course. At the green station the physical head-mesh apex must stay at least 1.25 m above the floor throughout the accepted final dwell. Any physically valid posture can pass the rack and meet the station condition.

The action stream is 50 Hz, with 64 SONIC V1 FSQ scalar codes per applied body row and ten 2 ms physics samples per row. The time cap for the initial approach, pivot and departure is 2,600, 450 and 1,300 applied rows respectively. The full 4,350-row horizon is 87 native seconds. Falls, missed passages, collisions, lost holds, boundary errors, solver failures and time limits remain in the assigned denominator.

## Transport, timing and evidence

The solver has 60 wall seconds from launch for `READY` and 15 wall seconds per reset/action exchange. The whole sixteen-course protected verifier has a 3,540 s wall budget within the 3,600 s engine limit. If this watchdog fires, finalized successes retain the original sixteen-course denominator. Unfinished episodes count separately. The partial ledger identifies the completed episode count. Motion and physical diagnostics on an interrupted panel cover only finalized trials.

A trial’s stage `*_scorer_terminal_tick` records the first control row on which the physical scorer reached a terminal decision. An accepted dwell additionally emits `*_settled_tick`. The `*_exit_tick` records stage exit; `*_stop_tick` is the historical alias for a successful stage’s exit. Rows for an accepted approach stop can precede actual stage exit by 75–79 ticks.

Reports include low/high rack completion, offset, front/back crossings, measured head-under-shelf clearance, whole-body trail exit, direct roof-to-robot separation when a pair comes within 0.30 m, actual final-dwell head height and three-stage outcomes. A blank near-roof distance means no pair was recorded below that distance at the monitored samples. The maximum head height recorded while the pelvis lies between the rack planes is a diagnostic. It does not prescribe an entry or exit posture. Motion reports include applied rows, scalar codes, within-course changed coordinates, eligible comparisons, actual native time and pooled RMS acceleration from the 29 measured joint velocities spaced 0.10 s apart.

## Prospective numerical rule

Freeze the environment, submitted participant, grader and sampling code before drawing the two official null-anchor reference measurements. Both the bootstrap and independent spread must complete at least 15/16 full courses with zero solver aborts. Successful trials must include both pivot directions, every door service family, both rack heights and both rack offsets. Retain any deficient attempt. A new calibration attempt requires a substantive revision, prior registration and independently initialized courses.

The bootstrap raw fraction sets the anchor. The smaller qualifying physical success count sets full-credit saturation (15 or 16). The grader’s saturation ratio equals that count divided by bootstrap successes. After freezing the numeric source, two further ordinary ALE validations must each return zero for the untouched workspace and one for the submitted reference. The final scientific report includes raw counts for every draw. A separately authored blind model measurement uses its own records and has no role in calculating the numeric anchor.
