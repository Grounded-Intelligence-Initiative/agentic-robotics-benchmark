# The oracle's record (grader-side; maintainers' evidence for full_at)

The oracle is `solution/honest/` (byte-identical to `tests/review/honest/`): a program that uses only the public API,
the given documents and the given Panda model. Every number below is a single attempt per engine; no engine was
re-run to improve a count. "Version" names the oracle by the hidden-set registration that recorded it
(`tests/hidden/selection.json`).

## Out-of-sample record by version

| oracle version | engines | accepted | where |
|---|---|---|---|
| 2026-11-10 09:18Z (first honest program) | fresh set v1 (12 engines drawn before its evaluation) | 10 / 12 | task image, single-episode verifies |
| 2026-11-10 09:18Z | the 4 hidden engines | 4 / 4 | same |
| **2026-11-10 16:14Z (the shipped oracle)** | development seeds 149-180 (never used in its development; in-process cell, same physics and sensor model) | **29 / 32 = 0.906** | contributor's workstation, one CPU per engine |
| **16:14Z (shipped)** | **fresh set v2: 16 engines (`fresh_v2_seeds.json`, sha256 7df50e2d...b883, drawn 2026-11-10 09:32:51Z, before the rule)** | **16 / 16** | task image, single-episode verifies (the full_at evaluation) |
| **16:14Z (shipped)** | **the 4 hidden engines** | **4 / 4** | same evaluation |
| 16:14Z (shipped) | the 4 hidden engines, `ale validate` x 2 (validate runs 3c76f381, 4e8b21c7) | reward 1.0, 1.0 | the engine's own validate |
| 16:14Z (shipped) | the same 20 engines (fresh v2 + hidden) re-graded on the final package (cell 0.2.2, amendments 1-2b) | 20 / 20, every engine's simulated time identical to the full_at evaluation | task image, single-episode verifies |
| 16:14Z (shipped) | the 4 hidden engines, `ale validate` x 2 on the final package (validate runs 23705e85, c04ee71a) | reward 1.0, 1.0 | the engine's own validate |

Pooled over the shipped version's unseen engines: 49 / 52 = 0.94. Earlier development versions on further unseen
development seeds (each batch new, each version different): 28/32, 27/32, 28/32 (0.86-0.88).

full_at follows the rule registered before the evaluation (`full_at_rule.md`): p = 20/20 on fresh v2 + hidden gives
full_at = floor_to_0.01(1 - 1/16) = 0.93. At the shipped version's measured unseen rate of about 0.91-0.94, a 20/20 run
had a probability of roughly 0.14-0.29: the rule's 1/16 allowance is what keeps full_at below 1.0. On 4 engines a
program at the oracle's level scores 1.0 with 4/4, and 3/4 gives a reward of 0.75 / 0.93 = 0.81.

## The shipped oracle's failures on unseen engines (development seeds 149-180)

| seed | what happened |
|---|---|
| 151 | the C step left the loop's bottom deep after its retakes; the last span then lay off the drive's right side and the span set before the return found no belt at z 0.16 (program error, safe end; score 0) |
| 158 | the span set before the return pulled the held span to 24.5 N (taut); in the guided return the belt came off C, ALT and WP and fell (damage) |
| 179 | the third C take dropped the loop's bottom below C (lowest point z -0.034 m); the program stopped (safe end; score 0) |

Failure classes seen in earlier versions: the C retake leaving the bottom in front of C; the left hand letting go in
the return under rising load (belt off); arm-to-arm clearance on a joint transit; a belt grip judged missed at a
2.38 mm closing width.

## Per-engine results of the full_at evaluation (cell 0.2.1, task image, one CPU per engine)

| engine seed | set | belt_acceptance | simulated s | episode wall s | frames |
|---|---|---|---|---|---|
| 7001416511751741712 | fresh v2 | 1.0 | 868 | 809 | 44 |
| 5619918603710544512 | fresh v2 | 1.0 | 691 | 673 | 44 |
| 6365081185696590101 | fresh v2 | 1.0 | 440 | 369 | 26 |
| 7207704597274426006 | fresh v2 | 1.0 | 677 | 650 | 44 |
| 1373904542696983033 | fresh v2 | 1.0 | 451 | 442 | 32 |
| 1169210542203514312 | fresh v2 | 1.0 | 839 | 758 | 52 |
| 1694044155420608538 | fresh v2 | 1.0 | 440 | 381 | 26 |
| 1429760602571962615 | fresh v2 | 1.0 | 818 | 731 | 44 |
| 618960965062833123 | fresh v2 | 1.0 | 472 | 377 | 30 |
| 8823538061013385858 | fresh v2 | 1.0 | 871 | 812 | 44 |
| 2644449267144930108 | fresh v2 | 1.0 | 593 | 517 | 36 |
| 4222753323446286472 | fresh v2 | 1.0 | 924 | 731 | 47 |
| 1441751303446464541 | fresh v2 | 1.0 | 792 | 716 | 44 |
| 7410585534573944865 | fresh v2 | 1.0 | 490 | 414 | 26 |
| 3253352991761677769 | fresh v2 | 1.0 | 460 | 389 | 30 |
| 1952952379411512801 | fresh v2 | 1.0 | 593 | 508 | 36 |
| 8326556230279516505 | hidden | 1.0 | 486 | 407 | 26 |
| 5445062517772755066 | hidden | 1.0 | 855 | 654 | 44 |
| 1548452981879765867 | hidden | 1.0 | 439 | 365 | 26 |
| 4451312637613931692 | hidden | 1.0 | 866 | 631 | 44 |
