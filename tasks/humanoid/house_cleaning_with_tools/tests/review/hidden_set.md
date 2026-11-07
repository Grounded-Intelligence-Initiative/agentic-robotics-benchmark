# The fixed hidden scene set (tests/hidden/seeds.json)

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
