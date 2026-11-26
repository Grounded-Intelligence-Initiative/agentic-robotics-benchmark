"""Isolated closed-loop verifier. The submission controls actions, never the world or the score."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cameras import Cameras
from world import Home
from robotics_grader import (
    Solver,
    SolverAborted,
    SolverMissing,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)


def main():
    cfg = stage_config()
    anchor, meta = read_anchor()
    rng = seed_rng()
    try:
        solver = Solver(step_timeout=cfg["step_timeout_s"])
    except (SolverAborted, SolverMissing) as err:
        write_verdict(zero_verdict(reason=str(err), anchored=anchor is not None))
        return
    rows = []
    aborted = 0
    try:
        for _ in range(cfg["episodes"]):
            w = Home(rng.seed(bits=64), cfg["ticks"], cfg["flight_ticks"])
            cam = Cameras(w)
            try:
                solver.reset()
                while w.t < w.budget:
                    action = solver.act(w.t, cam.observation())
                    try:
                        w.advance(action)
                    except ValueError as exc:
                        raise SolverAborted(str(exc)) from exc
                    if w.result()["success"]:
                        break
                rows.append(w.result())
            except SolverAborted:
                rows.append(
                    {
                        "success": False,
                        "score": 0,
                        "parked": False,
                        "clean": {},
                        "ticks": 0,
                    }
                )
                aborted += 1
            finally:
                cam.close()
    finally:
        solver.close()
    n = len(rows)
    extra = {
        "full_success_rate": sum(r["success"] for r in rows) / n,
        "carton_parked_rate": sum(r["parked"] for r in rows) / n,
        "carton_staged_rate": sum(r.get("staged", False) for r in rows) / n,
        "mean_treatment_preserved": sum(r.get("treatment_preserved", 0) for r in rows)
        / n,
        "mean_clean_fraction": sum(sum(r["clean"].values()) / 3 for r in rows) / n,
        "episodes_aborted": aborted,
        "episodes": n,
        "unscuffed_rate": sum(r.get("scuffed_runners") == 0 for r in rows) / n,
    }
    wins = [r["ticks"] for r in rows if r["success"]]
    if wins:
        extra["mean_ticks_on_success"] = sum(wins) / len(wins)
    write_verdict(
        ratio_rewards(
            sum(r["score"] for r in rows) / n,
            anchor,
            metric_name="mean_cleaning_score",
            higher_is_better=True,
            full_at=meta["full_at"],
            extra=extra,
        )
    )


if __name__ == "__main__":
    main()
