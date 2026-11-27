"""Hidden-shift selection for chute_realtime_bimanual_tracking, and the anchor that goes with it.

Rule (fixed before the screening ran, 2026-11-25): the candidates are 12 seeds drawn once with
numpy.random.default_rng(20261125).choice(arange(1000, 10000), 12, replace=False). Every candidate is graded once with
the reference over the real grading path (solution/solve.sh + tests/verify.py, 20 parts). The hidden set is the 6
candidates with the highest part_pickup_score (ties: the smaller seed). The anchor value is the mean of the reference's
score over the hidden set in a SECOND, independent grading run of those 6 shifts; full_at is the lower of the two
runs' means divided by the higher, less 0.05, capped at 0.98.

    python select_shifts.py screen <dir with seed*/verdict.json>            -> prints the ranking and the 6 kept
    python select_shifts.py anchor <screen dir> <second-run dir> <task dir> -> writes hidden/shifts.json,
                                                                               hidden/shift_selection.json, anchor.json
"""
import glob, hashlib, json, os, sys


def scores(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, "seed*", "verdict.json"))):
        v = json.load(open(f)); m = v["metrics"]; seed = int(os.path.basename(os.path.dirname(f))[4:])
        out[seed] = {"part_pickup_score": m["part_pickup_score"], "placed": m.get("shift0_placed"), "fell_behind": m.get("shift0_fell_behind"),
                     "program_time_s": m.get("shift0_program_time_s"), "simulated_time_s": m.get("shift0_simulated_time_s"), "match_lock_ok": m.get("match_lock_ok")}
    return out


def keep(sc, n=6):
    return sorted(sorted(sc, key=lambda s: (-sc[s]["part_pickup_score"], s))[:n])


if __name__ == "__main__":
    mode = sys.argv[1]; sc = scores(sys.argv[2]); kept = keep(sc)
    for s in sorted(sc, key=lambda s: (-sc[s]["part_pickup_score"], s)): print(s, sc[s], "KEPT" if s in kept else "")
    print("kept", kept, "mean of kept in screening", round(sum(sc[s]["part_pickup_score"] for s in kept) / len(kept), 4))
    if mode == "anchor":
        second = scores(sys.argv[3]); task = sys.argv[4]; assert all(s in second for s in kept), "second run incomplete"
        a = sum(sc[s]["part_pickup_score"] for s in kept) / len(kept); b = sum(second[s]["part_pickup_score"] for s in kept) / len(kept)
        full_at = round(min(0.98, min(a, b) / max(a, b) - 0.05), 2)
        frozen = {f: hashlib.sha256(open(os.path.join(task, f), "rb").read()).hexdigest() for f in
                  ("tests/verify.py", "tests/cell_env.py", "tests/shift.py", "tests/props.py", "tests/grader_config.json", "solution/privileged/oracle.py", "solution/privileged/reference_params.json")}
        os.makedirs(os.path.join(task, "tests", "hidden"), exist_ok=True)
        hp = os.path.join(task, "tests", "hidden", "shifts.json"); hid = json.load(open(hp)) if os.path.exists(hp) else {}
        hid["seeds"] = kept; json.dump(hid, open(hp, "w"))      # the salt already in the file is kept
        json.dump({"format": "chute_realtime_bimanual_tracking shift selection 1", "rule": __doc__.split("Rule")[1].split("python select")[0].strip(),
                   "candidates": {str(s): sc[s] for s in sorted(sc)}, "kept": kept, "second_run": {str(s): second[s] for s in kept}, "frozen": frozen},
                  open(os.path.join(task, "tests", "hidden", "shift_selection.json"), "w"), indent=1)
        anc = json.load(open(os.path.join(task, "tests", "anchor.json")))
        anc.update(value=round(b, 4), full_at=full_at,
                   source="Own measurement, 2026-11-25: the privileged reference (solution/solve.sh, token-gated ground truth) on the 6 hidden shifts of tests/hidden/shifts.json over the real grading path (tests/verify.py, 20 parts a shift, PhysX on the CPU, Isaac Sim 6.1 on an NVIDIA H200). value is the mean of the second of two independent runs ({:.4f}); the first, which selected the shifts, gave {:.4f} (tests/hidden/shift_selection.json).".format(b, a),
                   note="part_pickup_score = mean per-part credit (1.0 in the tote, 0.5 held for 1 s, 0 lost, -0.5 per disturbed bystander), mean over the hidden shifts. The reference is a ground-truth controller, not a ceiling: it leaves alone the parts wider than the gripper opens and still loses parts in motion. full_at is sized from the two runs' means (the lower over the higher, less 0.05).")
        json.dump(anc, open(os.path.join(task, "tests", "anchor.json"), "w"), indent=2)
        print("anchor", anc["value"], "full_at", full_at, "runs", round(a, 4), round(b, 4))
