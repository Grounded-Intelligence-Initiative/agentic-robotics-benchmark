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
