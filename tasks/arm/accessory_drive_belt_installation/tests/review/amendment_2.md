# Amendment 2 (the final package, cell 0.2.2; registered before any run on it)

Base: amendment 1 (`amendment_1.md`) on the package frozen as `task_frozen_c892e1b7...`. Amendment 2 adds the
agent-visible corrections the independent review asked for, so that the formal runs are made on the exact package that
is submitted. The hidden set is unchanged (the same 4 engines; no re-draw). No physics, no rule, no threshold, no
budget the program sees and no grader decision changes; the cell's describe() text and data sheet change as listed.

Content (the draft patch `patch_B_docs_draft.diff`, sha256 cb8e310d54fed941af5625c251026f1498658ec05730b4e688c61d66eb7f146f,
plus the version bump and the regenerated example):

1. S1 (cell_manual.md grip rule, describe()["grip_rule"]): a hold slips when the moment about the grip on a held belt
   exceeds 1.0 N m about any axis (bending it flatwise or edgewise, or twisting it) for 50 ms; was "twists a held belt".
   The rule's code is unchanged (it always used the moment about any axis). catalog.md's construction row and
   describe()["belt"] give the edgewise joints' stiffness as 8 x the flatwise (`edgewise_to_flatwise_stiffness` 8.0).
2. S2 (instruction.md): at most one episode per grading is re-run; a further cell failure scores 0 for that engine
   (the grader's existing behaviour).
3. S4 (cell_manual.md, cartesian mode): the IK posture (`cmd_q`) is kept inside joint_range; a joint at its limit can
   leave the tcp short of its reference without an error; how to see it and free it.
4. N1/N7 (cell_manual.md, catalog.md): the joint-mode lag per joint after the damping cap (was "roughly 0.1 s"); the
   F/T sign sentence completed (it ended in "..." and cited worked numbers that do not exist); "a chain of rigid
   10-long elements (about 105)".
5. S8 (task.toml comment only): the declared simplifications add the belt's mass (about twice a real 6PK's) and the
   frictionless contact of everything but the pulleys.
6. S5 (packaging): the survey photos ship once (environment/setup/payload/given/survey); environment/Dockerfile copies
   them into the image (/opt/task-assets/survey) and tests/test.sh stages the verifier's copy from there;
   tests/setup_payload no longer duplicates them; tests/given_hashes.json is unchanged in content (every given file,
   the survey's included, is still checked).
7. CELL_VERSION belt-station/0.2.2; describe_example.json regenerated from the cell (make_describe_example.py) and
   tests/setup_payload re-synced (sync_payload.py).
8. task.toml [verifier] timeout_sec 25200 and grader_config episode_wall_cap_s 4800 from amendment 1 are part of the
   final package (amendment 1 could not apply them to runs on the frozen package).

Checks before freezing: the oracle on the 20 engines of the full_at evaluation (fresh v2 16 + hidden 4) in the new
image, 20/20 required, trajectories compared with the full_at evaluation's (simulated time per engine); the anti-hack
suite (16 probes) on the final package; ale lint, the domain lint, tomllib, ale validate x 2; the agent-visible files
diffed against c892e1b7 must show exactly items 1-7. full_at and the anchor are kept (no change to what the oracle is
judged on); if the oracle is not 20/20, the package is not frozen and the failure is reported.

Registered: sha256 and UTC time recorded in the README status log and sent to the coordinator as witness.
