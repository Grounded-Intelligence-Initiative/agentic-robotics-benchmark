# The fixed hidden scene set (tests/hidden/seeds.json)

**Amendment 1 (2026-11-08):** the cell and the reference changed, the selection was repeated under the same rule,
and the hidden set became candidates 1, 2 and 3 (anchor 0.9352). **Amendment 2 (2026-11-09):** the reference's arm
postures and the robot state changed; the selection was repeated under the same rule (candidates 2, 3 and 4, anchor
0.9552). **Amendment 3 (2026-11-09, at the end):** the reference's arm postures (round 4) and the agent cameras' look
changed; the selection is repeated with one more rule: the kept scenes may not all share a layout fact a program could
assume. The sections before each amendment record the earlier selections.

The grader runs the three scenes of `tests/hidden/seeds.json`, for agents and the oracle alike (verify.py
`episode_seeds`; the oracle's privileged cells get the same layouts, not a nominal one). A scene is the cell's draw
from its seed (house_cell/layout.py: the objects, their facings, the grease stain, the crumbs, which bin stands where,
the robot's start pose; the sensor noise), by the rules cell_manual.md section 2 states.

## Selection rule (written before any candidate ran)

1. The oracle is frozen first: the commit and the sha256 of every file of `tests/house_cell/` and `solution/` are
   recorded below.
2. 16 candidate seeds are drawn from `os.urandom` (63 bits each, `tools/draw_candidates.py`), in one call; they are
   listed below in drawn order, before any of them runs.
3. Each candidate runs once: the privileged reference exactly as the oracle's episode does it (`tools/run_ref.py`: a
   fresh privileged `CellServer` on the seed, the oracle's four camera frames, `reference_clean`, `finalize`, scored by
   `house_cell/scoring.py` with the damage charges), in the task image, one process per candidate, with tests/test.sh's
   numeric environment (`tools/launch_ref.sh`).
4. A candidate passes when its episode has no safety violation and no damage (no broken dish, nothing of the
   customer's left on the floor), every part of the checklist but the floor has its full credit (trash 2, dishes 1,
   books 1, table 2.5, finishing 1), and at least half of the crumbs are in the general-waste bin. (The reference
   sweeps the crumb groups its broom and dustpan can reach in the open; the rest of the floor's credit is the margin
   above the anchor.)
5. The hidden set is the first three passing candidates in drawn order. If fewer than three of the 16 pass, 16 more
   are drawn the same way and the rule goes on in their order.
6. Every candidate's result is recorded below, kept or not, with a hollowing check: the kept scenes' layout features
   (`tools/features.py`) against all candidates'.

The anchor (`tests/anchor.json`) is then one full verify of the oracle submission on the kept set (`ale validate`'s
oracle pass), and `full_at` is sized from the difference between that verify and the selection runs.

## The frozen oracle

Commit `9213936` (house_cleaning_with_tools: maintainer fixes and a harder, zero-shot version). sha256 of the files the
oracle's episodes run (the cell, the reference cleaner, the oracle submission, the grader's configuration):

```
2f93e5e0db358d838599fadc7631b3cc3df63da4c65ba3f96bdd4961a780215e  tests/house_cell/cell.py
d88e94a34316fa9d9590eedf4e40b7f35ed0b893a0672c08e890f7854f1f762c  tests/house_cell/cleaning.py
2c44d49074a8b66ed548ba38dcb884478d9234187a27bdf943aebcac00637561  tests/house_cell/control.py
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  tests/house_cell/__init__.py
93c439787b73108e64125326aa47a21fbc0cd3009e59a1b9a88a1f56e536f8d1  tests/house_cell/layout.py
2814c5419023f70edc9e9507029be8583da7475138b9cc5da0fbd8af648ac272  tests/house_cell/objects.py
108048c18b6c0ae81da230a6b5e8dd21e4222391a24ce5351bd726ba4f9f6e7b  tests/house_cell/physics.py
03880c6204b6bdd7e3412e93a7b9d85ab501dfe84e6c2b9efce2f3fb44ce0c49  tests/house_cell/props.py
c8518d22141b7f1a7c38ae3fa1dbc44ffb370e2f5a970c8f89322846aa1537c6  tests/house_cell/reference.py
fa6232f3431683ebb618a68bd202aec66fcc766d66e61007fad8992e89b65581  tests/house_cell/render.py
223b284fd0ce69c6ed3f6b0d7f2348d67e0bcf150acdd70e567c8e6e0e7198e8  tests/house_cell/robot.py
59015301eb370fdd8e965d65a26c4ae01af0b7b32b5b9821094399a43a1ff1ed  tests/house_cell/room.py
3afb3e781ab4649602be28b0a3985f18009b450f11b1a9ac0ce7740e81195477  tests/house_cell/scene.py
151591966409b31bbd2038c4f59d0648fbbe8db1d21d6b6fc40e92a7aff14288  tests/house_cell/scoring.py
3a74be8b8c5a00b00367bb9d71f21e0c5eef0350c6a68e49cffce7f94ced1f36  tests/house_cell/server.py
dbac8d1a656f5d93bc0354d8748091aab64b7167ed4d85d08b6f4968d5c1f729  tests/house_cell/walk_core.py
6b660c0ab3e277835736f1371ca27e54c7dcfae061d49974806514a72fac3012  tests/house_cell/look.json
4f76a26e1fee4147327880b7a41d7e0e16855f7be57a8b8310a1353d80ebddc6  solution/solve.sh
7e224c2848476398821fce93c03244c0b6e45274dc60a5e32742c574df789e9b  solution/reference/clean.py
9a4c6c83b2d73f3b2fe5fffb1656ec250edbd11d3aefe5f25084699729208939  solution/reference/oracle_token
e88f7bc29cc3bdf8cffd1ed17e854295e0b73f25164982b0a11682c0719d1038  tests/grader_config.json
```

## The candidates (drawn 2026-11-07T21:24:24Z, `os.urandom(8) & (2^63 - 1)`, in drawn order)

| # | seed |
|---|---|
| 1 | 6790790475345571576 |
| 2 | 3125492756787262100 |
| 3 | 7022738416721159354 |
| 4 | 4089632381925228296 |
| 5 | 6569624632418969883 |
| 6 | 4920960843796068851 |
| 7 | 7640796830471677594 |
| 8 | 7981839151774950588 |
| 9 | 4745791118675327858 |
| 10 | 376632851020783183 |
| 11 | 3945038779305284334 |
| 12 | 4387728151438419368 |
| 13 | 6432759289746745107 |
| 14 | 18184323288814303 |
| 15 | 6438526276579322594 |
| 16 | 2636808825281067254 |

## Results (each candidate run once, 2026-11-07 21:26-21:55Z, apollo, image built from this commit's environment/)

| # | seed | stage score | crumbs binned | sim s | wall s | result | why |
|---|---|---|---|---|---|---|---|
| 1 | 6790790475345571576 | 0.9537 | 0.733 | 3462 | 1531 | fail | safety: robot_contact:a/right_wrist_pitch_link/kitchen |
| 2 | 3125492756787262100 | 0.9120 | 0.583 | 3307 | 1542 | pass (kept) |  |
| 3 | 7022738416721159354 | 0.8759 | 0.583 | 2754 | 1310 | fail | safety: robot_contact:a/right_wrist_pitch_link/kitchen; short: grease 2.44/2.5, sponge 0.00/0.3 |
| 4 | 4089632381925228296 | 0.8357 | 0.417 | 3824 | 1554 | fail | short: tools 0.00/0.3; crumbs binned 0.42 |
| 5 | 6569624632418969883 | 0.6250 | 0.000 | 3379 | 1325 | fail | short: books 0.50/1, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |
| 6 | 4920960843796068851 | 0.7100 | 0.000 | 2861 | 1117 | fail | safety: robot_contact:a/right_wrist_pitch_link/dining, robot_contact:a/right_wrist_pitch_link/dining; short: tools 0.00/0.3, lids 0.10/0.2; crumbs binned 0.00 |
| 7 | 7640796830471677594 | 0.9444 | 0.700 | 3277 | 1608 | fail | safety: robot_contact:a/right_wrist_pitch_link/kitchen |
| 8 | 7981839151774950588 | 0.7200 | 0.000 | 3863 | 1586 | fail | short: tools 0.00/0.3; crumbs binned 0.00 |
| 9 | 4745791118675327858 | 0.9019 | 0.817 | 3170 | 1531 | fail | short: books 0.50/1 |
| 10 | 376632851020783183 | 0.9213 | 0.617 | 3504 | 1385 | fail | safety: robot_contact:a/right_wrist_pitch_link/kitchen |
| 11 | 3945038779305284334 | 0.8816 | 0.667 | 2762 | 1083 | fail | short: trash 1.71/2 |
| 12 | 4387728151438419368 | 0.9352 | 0.667 | 3319 | 1387 | pass (kept) |  |
| 13 | 6432759289746745107 | 0.9398 | 0.683 | 3730 | 1643 | pass (kept) |  |
| 14 | 18184323288814303 | 0.6250 | 0.000 | 3252 | 1760 | fail | short: books 0.50/1, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |
| 15 | 6438526276579322594 | 0.9398 | 0.683 | 3461 | 1335 | pass |  |
| 16 | 2636808825281067254 | 0.6750 | 0.000 | 3426 | 1763 | fail | short: dishes 0.75/1, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |

4 of the 16 pass. The failures are the reference's own (a candidate can fail for several reasons): a safety
violation in 5 (a wrist pressed on the counter at the soap pump in 1, 3, 7 and 10, on the dining table in 6); the floor
less than half swept in 6 (4, 5, 6, 8, 14, 16: the dustpan or the broom stuck; the lids also left open in 5, 14 and
16); an item left in 5 (a book in 5, 9 and 14, the water bottle in 11, a glass in 16). Per-candidate
records: `selection/seed_<seed>.json`.

**The hidden set** (`tests/hidden/seeds.json`): candidates 2, 12 and 13, stage scores 0.9120, 0.9352 and 0.9398 (mean
0.9290). Candidate 15 also passed; it is not used (the rule keeps the first three).

## Hollowing check (`selection/features.json`)

| feature | kept 2 / 12 / 13 | all 16 candidates |
|---|---|---|
| recycling bin in the north slot | yes / no / yes | 7 of 16 |
| the glasses' distance from the nearer long edge of the table (m), nearer / farther glass | 0.287, 0.306 / 0.105, 0.196 / 0.165, 0.187 | 0.084-0.287 / 0.116-0.315 |
| the can's distance from the nearer long edge (m) | 0.096 / 0.257 / 0.200 | 0.079-0.266 |
| grease at the start (units) | 43.8 / 36.9 / 41.3 | 17.7-62.8 (mean 41.2) |
| floor crumbs south / north of the table | 22, 24 / 23, 23 / 24, 22 | 2-44 / 2-44 |

The kept scenes cover the candidates' range in the bins' order, the glasses' and the can's reach and the amount of
grease. One feature is narrowed: all three have their floor crumbs split about evenly between the two sides of the
table (two patches to sweep), where the candidates range from nearly all on one side to an even split; the
reference's broom fails more often when one large group lies along one side. For an agent this is not an easier
floor: it has two patches to find and sweep instead of one.

## The anchor's second measurement

`ale validate` from a clean checkout of `ac9820e` (validate-502efd9f, 2026-11-07, apollo, engine 1cdbc02, a separate
verifier sandbox of 8 CPUs): `ok`, untouched reward 0.0, oracle reward 1.0. The oracle's three episodes scored 0.912,
0.935 and 0.940 with the same simulated times as their selection runs (3306.7, 3318.9 and 3729.9 s): cleaning_score
0.9290, ratio 1.00001 to the anchor; verify 4882 s. The scenes replay exactly, so `full_at` 0.98 is only a margin for
a last-digit difference on another machine.

## Amendment 1 (2026-11-08): the cell and the reference changed; the selection is repeated

The owner's real-world check of the task (2026-11-08) changed the cell and the reference cleaner at commit `792cbb9`:
the robot state reports joint torque estimates (`tau_est`), the dishes break like ordinary porcelain and glass (an
impact above a 30 cm fall, a load above 600 N / 400 N), the fingers stop where they meet what the hand holds, and the
reference takes the sponge from its tray from the stance of its most natural arm posture. The frozen oracle of the
selection above is therefore gone, and the selection is repeated under the same rule, written here before any run:

1. The oracle is frozen at `792cbb9` (sha256 below).
2. The candidates are the 16 drawn on 2026-11-07 (`candidates.json`), in the same order; no new draw unless step 5
   of the rule needs one.
3. Each candidate runs once, exactly as before (`tools/launch_ref.sh`, `tools/run_ref.py`), in the task image built
   from `environment/` at `792cbb9`.
4. The pass criterion is unchanged (no safety violation, no damage, everything but the floor at full credit, at least
   half of the crumbs binned).
5. The hidden set is the first three passing candidates in drawn order; fewer than three: 16 more drawn by
   `tools/draw_candidates.py` and the rule goes on in their order.
6. Every candidate's result is recorded below (`selection_a1/seed_<seed>.json`), with the hollowing check if the kept
   set changes. The anchor is the mean of the kept selection runs; `ale validate`'s oracle pass on the final head is
   its full-verify measurement.

The frozen oracle (`792cbb9`):

```
4723e7f2818e4949600d4211b971c22128eb517ed8073e9add75e75d455135a9  tests/house_cell/cell.py
d88e94a34316fa9d9590eedf4e40b7f35ed0b893a0672c08e890f7854f1f762c  tests/house_cell/cleaning.py
5c9046ada48dc935538340de1eaceab078ae941de25a1bf6fd2b12aa3ffd0572  tests/house_cell/control.py
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  tests/house_cell/__init__.py
93c439787b73108e64125326aa47a21fbc0cd3009e59a1b9a88a1f56e536f8d1  tests/house_cell/layout.py
2814c5419023f70edc9e9507029be8583da7475138b9cc5da0fbd8af648ac272  tests/house_cell/objects.py
5b9a26199f1e692bd6e0d4beb34699ac8fcf110920069dc5736fb01e33f94198  tests/house_cell/physics.py
03880c6204b6bdd7e3412e93a7b9d85ab501dfe84e6c2b9efce2f3fb44ce0c49  tests/house_cell/props.py
cd8dec14d0e20b44ef6f1bf1be311622ad2fb2a96af2c8d905e6b97ad49ff90e  tests/house_cell/reference.py
fa6232f3431683ebb618a68bd202aec66fcc766d66e61007fad8992e89b65581  tests/house_cell/render.py
223b284fd0ce69c6ed3f6b0d7f2348d67e0bcf150acdd70e567c8e6e0e7198e8  tests/house_cell/robot.py
59015301eb370fdd8e965d65a26c4ae01af0b7b32b5b9821094399a43a1ff1ed  tests/house_cell/room.py
3afb3e781ab4649602be28b0a3985f18009b450f11b1a9ac0ce7740e81195477  tests/house_cell/scene.py
fb9c83456a9275d0555da22f959c35ec44aa5c4909c72fc10e7de9cda5024cc4  tests/house_cell/scoring.py
3a74be8b8c5a00b00367bb9d71f21e0c5eef0350c6a68e49cffce7f94ced1f36  tests/house_cell/server.py
dbac8d1a656f5d93bc0354d8748091aab64b7167ed4d85d08b6f4968d5c1f729  tests/house_cell/walk_core.py
6b660c0ab3e277835736f1371ca27e54c7dcfae061d49974806514a72fac3012  tests/house_cell/look.json
4f76a26e1fee4147327880b7a41d7e0e16855f7be57a8b8310a1353d80ebddc6  solution/solve.sh
7e224c2848476398821fce93c03244c0b6e45274dc60a5e32742c574df789e9b  solution/reference/clean.py
9a4c6c83b2d73f3b2fe5fffb1656ec250edbd11d3aefe5f25084699729208939  solution/reference/oracle_token
e88f7bc29cc3bdf8cffd1ed17e854295e0b73f25164982b0a11682c0719d1038  tests/grader_config.json
```

### Results (amendment 1; each candidate run once, 2026-11-08 13:02-13:40Z, apollo, the image built from environment/ at `792cbb9`)

| # | seed | stage score | crumbs binned | sim s | wall s | result | why |
|---|---|---|---|---|---|---|---|
| 1 | 6790790475345571576 | 0.9398 | 0.683 | 3384 | 1395 | pass (kept) |  |
| 2 | 3125492756787262100 | 0.9259 | 0.633 | 3356 | 1524 | pass (kept) |  |
| 3 | 7022738416721159354 | 0.9398 | 0.683 | 3667 | 1458 | pass (kept) |  |
| 4 | 4089632381925228296 | 0.9213 | 0.617 | 3519 | 1421 | pass |  |
| 5 | 6569624632418969883 | 0.8694 | 0.700 | 3461 | 1240 | fail | damage: book_red on the floor; short: books 0.50/1 |
| 6 | 4920960843796068851 | 0.8981 | 0.533 | 3197 | 1251 | pass |  |
| 7 | 7640796830471677594 | 0.9398 | 0.683 | 3288 | 1204 | pass |  |
| 8 | 7981839151774950588 | 0.9259 | 0.633 | 3459 | 1358 | pass |  |
| 9 | 4745791118675327858 | 0.8741 | 0.717 | 3343 | 1516 | fail | damage: book_red on the floor; short: books 0.50/1 |
| 10 | 376632851020783183 | 0.9213 | 0.617 | 3342 | 2133 | pass |  |
| 11 | 3945038779305284334 | 0.8955 | 0.717 | 3249 | 1197 | fail | damage: bottle on the floor; short: trash 1.71/2 |
| 12 | 4387728151438419368 | 0.9120 | 0.583 | 3276 | 1321 | pass |  |
| 13 | 6432759289746745107 | 0.9537 | 0.733 | 3716 | 1478 | pass |  |
| 14 | 18184323288814303 | 0.6250 | 0.000 | 3278 | 1601 | fail | damage: book_red on the floor; short: books 0.50/1, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |
| 15 | 6438526276579322594 | 0.9259 | 0.633 | 3314 | 1262 | pass |  |
| 16 | 2636808825281067254 | 0.8787 | 0.733 | 4092 | 1408 | fail | damage: glass_2 broken and on the floor; short: dishes 0.75/1 |

11 of the 16 pass (4 did under the first frozen oracle). The reference's safety violations at the soap pump and the
dining table are gone (the sponge is now taken from a stance beside the tray, which also changes the arm's path to the
pump), and its broom and dustpan now reach the crumb groups in 15 of 16 scenes. The failures are the reference's own:
a book left on the floor (5, 9, 14), the water bottle on the floor (11), a glass dropped and broken on the way to the
tray (16); in 14 the floor was also not swept. Per-candidate records: `selection_a1/seed_<seed>.json`.

**The hidden set** (`tests/hidden/seeds.json`): candidates 1, 2 and 3, stage scores 0.9398, 0.9259 and 0.9398 (mean
0.9352 = the anchor). Candidate 2 was kept before too; 12 and 13, kept before, pass again but come later in the order.

**Hollowing check** (`selection/features.json`; the layouts are drawn as before):

| feature | kept 1 / 2 / 3 | all 16 candidates |
|---|---|---|
| recycling bin in the north slot | no / yes / yes | 7 of 16 |
| the glasses' distance from the nearer long edge of the table (m), nearer / farther glass | 0.139, 0.280 / 0.287, 0.306 / 0.152, 0.310 | 0.084-0.287 / 0.116-0.315 |
| the can's distance from the nearer long edge (m) | 0.079 / 0.096 / 0.183 | 0.079-0.266 |
| grease at the start (units) | 42.6 / 43.8 / 62.8 | 17.7-62.8 (mean 41.2) |
| floor crumbs south / north of the table | 44, 2 / 22, 24 / 42, 4 | 2-44 / 2-44 |

The kept scenes cover the candidates' range in the bins' order, the glasses' reach, the grease (the largest stain of
all 16 is kept) and the floor (two scenes with nearly all crumbs on one side, one even split: the narrowing of the
first set is gone). The can lies near the edge in two of them.

### The anchor's full-verify measurement (amendment 1)

`ale validate` from a clean checkout of `1d8d2ce` (validate-1f2b6818, 2026-11-08, apollo, engine 1cdbc02, a separate
verifier sandbox of 8 CPUs): `ok`, untouched reward 0.0, oracle reward 1.0. The oracle's three episodes repeat their
selection runs to the last digit: cleaning_score 0.93519 (ratio 0.99998 to the anchor), mean simulated time 3468.86 s
(the selection runs': 3384.19, 3355.63 and 3666.75 s), no safety violation, no broken dish; verify 3354 s.

## Amendment 2 (2026-11-09): natural arm postures; the selection is repeated

The reference's arm postures were reviewed against how a person moves (the arms were often raised over the head, swung
back or wrung at their joint limits, in 48-62 % of an episode's time on development scenes 11-15) and changed at
`1170d84`: every planned arm configuration is solved with a posture term in the arm's redundancy, held items are
carried in natural postures, grasps are chosen for the carry and the work they leave (`posture.md`). The robot state
also reports every readable motor's `q`, `dq` and `tau_est` (the hands' motors and the velocities are new; nothing the
reference reads). The frozen oracle of amendment 1 is therefore gone, and the selection is repeated under the same
rule, written here before any run:

1. The oracle is frozen at `1170d84` (sha256 below).
2. The candidates are the 16 drawn on 2026-11-07 (`candidates.json`), in the same order; no new draw unless step 5
   needs one.
3. Each candidate runs once, exactly as before (`tools/launch_ref.sh`, `tools/run_ref.py`), in the task image built
   from `environment/` at `1170d84`.
4. The pass criterion is unchanged (no safety violation, no damage, everything but the floor at full credit, at least
   half of the crumbs binned).
5. The hidden set is the first three passing candidates in drawn order; fewer than three: 16 more drawn by
   `tools/draw_candidates.py` and the rule goes on in their order.
6. Every candidate's result is recorded below (`selection_a2/seed_<seed>.json`), with the hollowing check if the kept
   set changes (and the observability evidence for a newly kept scene). The anchor is the mean of the kept selection
   runs; `ale validate`'s oracle pass on the final head is its full-verify measurement.

The frozen oracle (`1170d84`):

```
f3a03a97db9fbe7697bf711c8c602492fe89d5142b0d1e57be6b9b2bda5752fd  tests/house_cell/cell.py
d88e94a34316fa9d9590eedf4e40b7f35ed0b893a0672c08e890f7854f1f762c  tests/house_cell/cleaning.py
5c9046ada48dc935538340de1eaceab078ae941de25a1bf6fd2b12aa3ffd0572  tests/house_cell/control.py
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  tests/house_cell/__init__.py
93c439787b73108e64125326aa47a21fbc0cd3009e59a1b9a88a1f56e536f8d1  tests/house_cell/layout.py
2814c5419023f70edc9e9507029be8583da7475138b9cc5da0fbd8af648ac272  tests/house_cell/objects.py
5b9a26199f1e692bd6e0d4beb34699ac8fcf110920069dc5736fb01e33f94198  tests/house_cell/physics.py
03880c6204b6bdd7e3412e93a7b9d85ab501dfe84e6c2b9efce2f3fb44ce0c49  tests/house_cell/props.py
895164520c7ad09cc668fea07591ddbc9e16f64de09a71932fb8ab55662796db  tests/house_cell/reference.py
fa6232f3431683ebb618a68bd202aec66fcc766d66e61007fad8992e89b65581  tests/house_cell/render.py
223b284fd0ce69c6ed3f6b0d7f2348d67e0bcf150acdd70e567c8e6e0e7198e8  tests/house_cell/robot.py
59015301eb370fdd8e965d65a26c4ae01af0b7b32b5b9821094399a43a1ff1ed  tests/house_cell/room.py
3afb3e781ab4649602be28b0a3985f18009b450f11b1a9ac0ce7740e81195477  tests/house_cell/scene.py
fb9c83456a9275d0555da22f959c35ec44aa5c4909c72fc10e7de9cda5024cc4  tests/house_cell/scoring.py
3a74be8b8c5a00b00367bb9d71f21e0c5eef0350c6a68e49cffce7f94ced1f36  tests/house_cell/server.py
dbac8d1a656f5d93bc0354d8748091aab64b7167ed4d85d08b6f4968d5c1f729  tests/house_cell/walk_core.py
6b660c0ab3e277835736f1371ca27e54c7dcfae061d49974806514a72fac3012  tests/house_cell/look.json
4f76a26e1fee4147327880b7a41d7e0e16855f7be57a8b8310a1353d80ebddc6  solution/solve.sh
7e224c2848476398821fce93c03244c0b6e45274dc60a5e32742c574df789e9b  solution/reference/clean.py
9a4c6c83b2d73f3b2fe5fffb1656ec250edbd11d3aefe5f25084699729208939  solution/reference/oracle_token
e88f7bc29cc3bdf8cffd1ed17e854295e0b73f25164982b0a11682c0719d1038  tests/grader_config.json
```

### Results (amendment 2; each candidate run once, 2026-11-09 05:33-06:20Z, apollo, the image built from environment/ at `1170d84`)

| # | seed | stage score | crumbs binned | sim s | wall s | result | why |
|---|---|---|---|---|---|---|---|
| 1 | 6790790475345571576 | 0.8637 | 0.633 | 1971 | 1262 | fail | short: grease 2.18/2.5, sponge 0.00/0.3 |
| 2 | 3125492756787262100 | 0.9398 | 0.683 | 2645 | 1519 | pass (kept) |  |
| 3 | 7022738416721159354 | 0.9583 | 0.750 | 3037 | 1506 | pass (kept) |  |
| 4 | 4089632381925228296 | 0.9676 | 0.783 | 3048 | 1805 | pass (kept) |  |
| 5 | 6569624632418969883 | 0.9583 | 0.750 | 2703 | 1262 | pass |  |
| 6 | 4920960843796068851 | 0.9398 | 0.683 | 3106 | 1469 | pass |  |
| 7 | 7640796830471677594 | 0.8955 | 0.717 | 3000 | 1934 | fail | damage: can_red on the floor; short: trash 1.71/2 |
| 8 | 7981839151774950588 | 0.9352 | 0.667 | 2898 | 1323 | pass |  |
| 9 | 4745791118675327858 | 0.9537 | 0.733 | 2508 | 1538 | pass |  |
| 10 | 376632851020783183 | 0.9398 | 0.683 | 2975 | 1340 | pass |  |
| 11 | 3945038779305284334 | 0.6700 | 0.000 | 1987 | 946 | fail | short: sponge 0.00/0.3, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |
| 12 | 4387728151438419368 | 0.9259 | 0.633 | 2815 | 1552 | pass |  |
| 13 | 6432759289746745107 | 0.9583 | 0.750 | 2988 | 1557 | pass |  |
| 14 | 18184323288814303 | 0.7000 | 0.000 | 2561 | 2114 | fail | short: tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |
| 15 | 6438526276579322594 | 0.7530 | 0.317 | 2582 | 1529 | fail | damage: the sponge on the floor; short: sponge 0.00/0.3, tools 0.00/0.3; crumbs binned 0.32 |
| 16 | 2636808825281067254 | 0.6450 | 0.000 | 2466 | 1143 | fail | damage: the sponge on the floor; short: sponge 0.00/0.3, tools 0.00/0.3, lids 0.00/0.2; crumbs binned 0.00 |

10 of the 16 pass (11 under amendment 1). The failures are the reference's own: the sponge slipped from the hand while
wiping (1); a can was dropped on the way to its bin (7); the sponge was not set back into its tray (11, 15, 16: leaning
in, the sponge carried lower than before would touch the counter, so its put was given up; in 15 and 16 the sponge
ended on the floor), after which the robot, by the dining table, pressed on it on its way to the broom and gave up the floor (11,
16); in 14 the walks to hang the broom back, to take the dustpan again and to the lids failed (the pan not emptied,
the lids left open). Per-candidate records: `selection_a2/seed_<seed>.json`.

**The hidden set** (`tests/hidden/seeds.json`): candidates 2, 3 and 4, stage scores 0.9398, 0.9583 and 0.9676 (mean
0.9552 = the anchor). Candidates 2 and 3 were kept under amendment 1 too; candidate 1 fails now (the sponge slipped).

**Hollowing check** (`selection/features.json`; the layouts are drawn as before):

| feature | kept 2 / 3 / 4 | all 16 candidates |
|---|---|---|
| recycling bin in the north slot | yes / yes / yes | 7 of 16 |
| the glasses' distance from the nearer long edge of the table (m), nearer / farther glass | 0.287, 0.306 / 0.152, 0.310 / 0.130, 0.254 | 0.084-0.287 / 0.116-0.315 |
| the can's distance from the nearer long edge (m) | 0.096 / 0.183 / 0.222 | 0.079-0.266 |
| grease at the start (units) | 43.8 / 62.8 / 56.5 | 17.7-62.8 (mean 41.2) |
| floor crumbs south / north of the table | 22, 24 / 42, 4 / 29, 17 | 2-44 / 2-44 |

The kept scenes cover the candidates' range in the glasses' and the can's reach and the floor (an even split, nearly
all on one side, in between); their stains are larger than the candidates' mean (the largest of all 16 is kept). One
feature is narrowed: all three have the recycling bin in the north slot (7 of 16 candidates do). The bins differ in
colour and the instruction says their order varies, so a program has to tell them apart in its images; one that
assumed the north bin were always the recycling one would sort every hidden scene right by luck.

### The anchor (amendment 2)

0.9552, the mean of the kept selection runs, with `full_at` 0.98. `ale validate` from a clean checkout of `94ef0d6`
(validate-9944b842, 2026-11-09, apollo, engine 1cdbc02, a separate verifier sandbox of 8 CPUs): `ok`, untouched reward
0.0, oracle reward 1.0. The oracle's three episodes repeat their selection runs to the last digit: cleaning_score
0.95525 (ratio 1.00005 to the anchor), mean simulated time 2909.71 s (the selection runs': 2644.84, 3036.52 and
3047.76 s), no safety violation (peak robot contact 121 N), no broken dish; verify 4579 s.

## Amendment 3 (2026-11-09): natural arm postures (round 4), the agent cameras' look; a fairer selection rule

Two changes since amendment 2, both before any run of this amendment:

* **The reference's arm postures** (`reference.py`, `posture.md` round 4): every joint path the reference executes keeps
  the planned hand path but resolves the arm's redundancy as a person's (the swivel that puts the elbow down and a
  little out, the wrist near neutral where the hand's pose allows, the shoulder untwisted), configuration by
  configuration from the previous one (`Reference.refine_path`); the planner's own choices are unchanged.
* **The agent cameras' look** (`render.py`, `scene.py`, `look.json`, the room decor and the survey photos on the asset
  dataset at `3062dead`): the approved path-traced (OVRTX) look of the apartment baked into Filament. It changes the
  cameras' colour, the depth only where the new decor (pictures, curtains, intercom, sockets) or the pendant's shade
  (it was inside out) stands; the physics and the reference's privileged run are unaffected by it.

The frozen oracle of amendment 2 is therefore gone, and the selection is repeated, written here before any run:

1. The oracle is frozen at `a7880f3` (sha256 below; `render.py`, `scene.py` and `look.json` are among the frozen
   files, so the renderer's change is part of this freeze).
2. The candidates are the 16 drawn on 2026-11-07 (`candidates.json`), in the same order; no new draw unless step 5
   needs one.
3. Each candidate runs once, exactly as before (`tools/launch_ref.sh`, `tools/run_ref.py`), in the task image built
   from `environment/` at `a7880f3`.
4. The pass criterion is unchanged (no safety violation, no damage, everything but the floor at full credit, at least
   half of the crumbs binned).
5. **New: no layout shortcut shared by the whole set.** A program could get a layout fact right by assuming it, without
   looking, if every hidden scene had the same value. The facts that decide what the job does and that the layout
   draws from two values are: which bin slot (south or north) holds the recycling bin, and on which side of the dining
   table (south or north) most of the floor's crumbs lie. (The other discrete draws decide nothing a program could
   assume: which plate lies on which side, which way round a book lies (shelved either way round), the stain's and
   the floor groups' counts.) The hidden set is the first three passing candidates, in drawn order, whose layouts do
   not all share either fact: of all sets of three passing candidates that satisfy this, the one whose drawn
   positions, sorted, come first lexicographically. If no such set exists among the 16, 16 more are drawn by
   `tools/draw_candidates.py` and the rule goes on in their order.
6. Every candidate's result is recorded below (`selection_a3/seed_<seed>.json`), with the hollowing check. No camera
   frame of a hidden scene is rendered for the record (the maintainers render development scenes only); the look's
   effect on what the cameras show is measured on the survey's development scenes (`observability.md`,
   amendment 3). The anchor is the mean of the kept selection runs; `ale validate`'s oracle pass on the final head is
   its full-verify measurement.

The frozen oracle (`a7880f3`):

```
f3a03a97db9fbe7697bf711c8c602492fe89d5142b0d1e57be6b9b2bda5752fd  tests/house_cell/cell.py
d88e94a34316fa9d9590eedf4e40b7f35ed0b893a0672c08e890f7854f1f762c  tests/house_cell/cleaning.py
5c9046ada48dc935538340de1eaceab078ae941de25a1bf6fd2b12aa3ffd0572  tests/house_cell/control.py
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  tests/house_cell/__init__.py
93c439787b73108e64125326aa47a21fbc0cd3009e59a1b9a88a1f56e536f8d1  tests/house_cell/layout.py
2814c5419023f70edc9e9507029be8583da7475138b9cc5da0fbd8af648ac272  tests/house_cell/objects.py
5b9a26199f1e692bd6e0d4beb34699ac8fcf110920069dc5736fb01e33f94198  tests/house_cell/physics.py
03880c6204b6bdd7e3412e93a7b9d85ab501dfe84e6c2b9efce2f3fb44ce0c49  tests/house_cell/props.py
754c6f1323219a7638caa6532e1f0a52a1147f446b2a90759636e5b232ac2734  tests/house_cell/reference.py
5b231503b38139893a887c541c0bd8fa0cf1f0ee8272f6f6c9e7d80ecdd1cf45  tests/house_cell/render.py
223b284fd0ce69c6ed3f6b0d7f2348d67e0bcf150acdd70e567c8e6e0e7198e8  tests/house_cell/robot.py
59015301eb370fdd8e965d65a26c4ae01af0b7b32b5b9821094399a43a1ff1ed  tests/house_cell/room.py
8f3413807a8547ec05e56d65de6287457195cdf522b8b2c1c59e1538a3f46be8  tests/house_cell/scene.py
fb9c83456a9275d0555da22f959c35ec44aa5c4909c72fc10e7de9cda5024cc4  tests/house_cell/scoring.py
3a74be8b8c5a00b00367bb9d71f21e0c5eef0350c6a68e49cffce7f94ced1f36  tests/house_cell/server.py
dbac8d1a656f5d93bc0354d8748091aab64b7167ed4d85d08b6f4968d5c1f729  tests/house_cell/walk_core.py
4dcb59ed702796d95e442aafdaffdad191036f3008bc54e1cb783892be9b4fa7  tests/house_cell/look.json
4f76a26e1fee4147327880b7a41d7e0e16855f7be57a8b8310a1353d80ebddc6  solution/solve.sh
7e224c2848476398821fce93c03244c0b6e45274dc60a5e32742c574df789e9b  solution/reference/clean.py
9a4c6c83b2d73f3b2fe5fffb1656ec250edbd11d3aefe5f25084699729208939  solution/reference/oracle_token
e88f7bc29cc3bdf8cffd1ed17e854295e0b73f25164982b0a11682c0719d1038  tests/grader_config.json
```

The rule's facts per candidate (from the layout draw alone, `tools/features.py`, before any run):

| # | seed | recycling bin's slot | floor crumbs south / north of the table (most on) |
|---|---|---|---|
| 1 | 6790790475345571576 | south | 44 / 2 (south) |
| 2 | 3125492756787262100 | north | 22 / 24 (north) |
| 3 | 7022738416721159354 | north | 42 / 4 (south) |
| 4 | 4089632381925228296 | north | 29 / 17 (south) |
| 5 | 6569624632418969883 | south | 42 / 4 (south) |
| 6 | 4920960843796068851 | north | 24 / 22 (south) |
| 7 | 7640796830471677594 | south | 2 / 44 (north) |
| 8 | 7981839151774950588 | south | 42 / 4 (south) |
| 9 | 4745791118675327858 | south | 16 / 30 (north) |
| 10 | 376632851020783183 | north | 42 / 4 (south) |
| 11 | 3945038779305284334 | south | 3 / 43 (north) |
| 12 | 4387728151438419368 | south | 23 / 23 (even) |
| 13 | 6432759289746745107 | north | 24 / 22 (south) |
| 14 | 18184323288814303 | north | 21 / 25 (north) |
| 15 | 6438526276579322594 | south | 23 / 23 (even) |
| 16 | 2636808825281067254 | south | 5 / 41 (north) |
