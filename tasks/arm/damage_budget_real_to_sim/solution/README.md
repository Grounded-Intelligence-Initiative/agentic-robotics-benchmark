# V9 reference: identifying a crossed-belt workcell

The camera cannot distinguish the belt winding or pulley ratios at reset.
The reference moves the vented tool without touching the cartridge, observes
the displacement, and estimates the drive and vertical response. It uses
these estimates to reconstruct an executable MuJoCo model. In each transfer
it identifies the drive afresh, extracts the cartridge around the source
fixture and settles it on the receiving cradle without damage.

`solve.sh` is the ALE oracle entry. It produces the same three deliverables
required of a participant: `scene.xml`, `predict.py` and a controller entry
point (`run.sh`). The verifier tests the reconstruction on unseen actions and
runs the frozen controller in eight scenes; a successful exploration alone
does not establish a successful submission.

**V9 development results:** the native oracle completed 8/8 transfers with
zero damage and 1.89 mm reconstruction RMSE; the untouched submission scored
zero. The unmodified program that previously scored 1.00 on V8 completed 0/8
on those same V9 scenes. That is a transfer test, not a fresh Astra run.

For the benchmark gate use the pinned engine's `ale lint` and `ale validate`
on this task. Reviewer-side [belt controls](../tests/review/check_benchcal.py)
check that reset images are identical, safe probing identifies the drive, and
a matched action sequence succeeds where the crossed sequence does not.
