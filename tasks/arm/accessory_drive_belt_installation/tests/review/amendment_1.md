# Amendment 1 (grader side only; registered before any formal-run result is read)

Applies to the package frozen as `task_frozen_c892e1b77ad5` (manifest sha256 c892e1b7...4019). Nothing the agent
sees changes: no file under environment/, no instruction.md, no given document, no reply of the cell's API.

## M1: the damage rules also hold while the robots are switched off

The documented rules (cell_manual.md "Safety and damage", the describe() `damage` text): the tensioner's damper is
rated for 60 deg/s of arm speed, faster damages it; a belt that touches the base plate is scrap; the engine with
damage is rejected. The frozen grader read `damage` before the acceptance's arms-off stepping, and the per-tick damage
check ran only inside the program's `step` calls, so a lever let go by the switch-off (finish, crash or timeout while
a hand holds it) swung back unjudged.

Change: the same per-tick damage check (`Rules._damage_tick`: tensioner arm speed > 60 deg/s, a belt point below
4 mm) runs after every physics tick of the switch-off, the 1 s settle and the run-in, and the engine's damage is read
after the run-in. Nothing else in the acceptance changes (no new threshold, no change to the inspection, the slip
rule, the window or the score formula).

Check before it is adopted: the oracle's 20 engines of the full_at evaluation (fresh v2 16 + hidden 4) re-run with the
amended grader must keep belt_acceptance 1.0 each. If the oracle's own run-in trips a rule, the amendment is reworked
and re-registered before any formal result is read.

New probe: a program that hangs the belt, takes the lever to its install stop with a hand and calls finish while
holding it must score 0 (damage tensioner_overspeed) on every engine.

## M2: the verifier's time limit from the documented bounds

The verifier runs 4 engines serially plus at most one re-run. The frozen `[verifier] timeout_sec = 7200` was sized
from the oracle's measured walls, below what a program inside its documented budgets may take (2400 s of program
time alone per episode). Change: `timeout_sec` = (4 + 1) x the per-episode wall bound + the import check and the
image start, with 25-40 % margin, where the per-episode wall bound = 2400 s program time + the cell's wall for 1500 s
of simulated time + 600 frames x the measured render wall per frame + the build, the settle and the run-in, all
measured in the task image on the contributor's 4-CPU machine. verify.py passes the same per-episode wall bound (plus
margin) to `serve_episode` as `wall_cap_s`: an episode inside every documented budget never reaches it (a safety net
only; it ends an episode with the reason `wall_cap` and scores the state as it is, as every other cap does).

## Re-grading

Each formal run's submission (artifacts/0000/submission) is re-graded with the amended tests/ (the task's verify), and
the frozen grade and the amended grade are reported side by side.

Registered: see the sha256 and UTC time recorded with this file in the README status log and sent to the
coordinator as witness.
