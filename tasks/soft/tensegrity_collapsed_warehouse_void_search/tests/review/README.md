# tests/review/: maintainer tools (never run by verify.py)

Hidden side only, like the rest of tests/. Nothing in this folder is imported or executed by `tests/test.sh` or
`tests/verify.py`; nothing here reaches the agent, the payload or the practice ground. Written for the grader's own QC
on 2026-11-17 (task version 1.0); the QC scripts, the generators, the calibration site's generator and the degenerate
submissions are version 1.1's (2026-11-19).

| Folder | What | Source of the bytes |
|---|---|---|
| `variants/` | the maintainer variants of the privileged reference (`qc_reference.py`) and their plans with expected values written before the runs (`plans.json`): QC Q2 / Q3 through the real verify.py. The plans are version 1.0's (credit 0.33 / 0.66 m, weights 0.15 / 0.25 / 0.60); version 1.1's are in `qc/variants/` (byte copies of the development folder's `tools/qc/variants/`) | written here (originals) |
| `qc/` | the QC scripts: `qc_verify.py` (runs the real verify.py on the frozen hidden set: oracle, oracle without / with a wrong token, a program folder, or a variant through the stage's `reference_dir`), `check_runs.py` (recomputes every episode's score from the cell's record by its own reading of the scoring rules and judges the runs), `q1_frames.py` (a fixed 20-frame sequence for the two-process frame check), `q4_interface.py` (bad arguments, reply whitelist, mission without layout, refusals, privileged calls, the pose switch, no debrief), `q6_recompute.py` (every G predicate recomputed from the stored site packages), `variant_inproc.py` (a development check of one variant in one cell), `q5_v11_extra.py` (version 1.1's hidden quantities against the agent-visible files and the image's file list), `d9_practice_debrief.py` (D9 on the practice path with the real debrief) | byte copies of the development folder's `tools/qc/` |
| `sitegen/w2gen/` | the W2 site generator (the writer side: `family.py`, `mjcf.py`, `predicates.py`, `grid.py`, `envelope.py`, `views.py`, `cli.py`) | byte copy of the development folder's `tools/sitegen/w2gen/` |
| `sitegen/w3gen/` | the W3 site generator of version 1.1 (the rubble under the slab included; it imports w2gen) | byte copy of the development folder's `tools/sitegen/w3gen/` |
| `sitegen_cal/calgen.py` | the calibration site's generator (calgen 0.2.0; it imports w2gen and w3gen) | byte copy of `tools/sitegen_cal/calgen.py` |
| `degenerate/` | the degenerate submissions D0-D4 (programs) and D8, D9 (plans of the maintainer variant: they need the true pose), `run_verify.py` (a development hidden set) and `practice_sync.py` (refreshes the practice ground's copy of the cell) | byte copy of `tools/degenerate/` |

`SHA256SUMS` lists every file of this folder with its sha256 (the table above names the source of each folder). The
copies refer to the development folder's layout (`DEV = <this file>/../..`): run them from the development folder, not
from here. The selection, packaging and oracle-verify tools are NOT here either: the admission rule and the selection
tools stay in the development folder (`tests/README.md` section 8 describes how the set was selected and packed), and
rebuilding a package from its seed runs there too (the generators' copies here read the development folder's
capability tables and art tools). The leak scan and its word list are NOT here (the word list names the prototype; it
stays in the development folder's `tools/scan/`).

How the variants run (the maintainers' switch): `qc_verify.py --submission oracle --variant NAME` copies `tests/` to a
scratch stage, replaces the stage's `reference/reference.py` by `qc/variants/qc_reference.py` and writes the plan as
`reference/qc_variant.json`; the submission is the oracle's own (`solution/solve.sh`, the real token), so the cell
runs the variant as its privileged `reference_run`. Everything else (verify.py, the cell, the scoring, the frozen
hidden set) is the task's bytes. A variant moves a search point or the pad only in the cell's in-memory copy of the
site and only after the robot has stopped: a search point and the pad are not physical objects (site.xml has no geom
for them), so the world and the trajectory stay the same and only the distance the scoring measures changes.
