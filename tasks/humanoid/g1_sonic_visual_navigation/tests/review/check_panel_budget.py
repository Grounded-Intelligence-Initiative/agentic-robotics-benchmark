"""Exercise the production deadline, accounting and verdict writer in disposable child processes."""

import argparse
from contextlib import nullcontext
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

TESTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TESTS))
import verify


def injected_child(mode):
    cfg = dict(episodes=16, horizon=5000, scoring_cap=1.5, verifier_deadline_s=0.20, step_timeout_s=15)
    spec = {"full_at": 1.0}

    def simulated_courses(factory, config, rng, assets=None, on_complete=None):
        rows = []
        n = {"none": 0, "partial": 5, "partial-scaled": 5, "complete": 16}[mode]
        for i in range(n):
            rows.append({"strict_success": i < 3, "simulated_s": 2.0})
            on_complete(tuple(rows))
        if mode != "complete":
            time.sleep(30)
            raise AssertionError("canonical verifier timer failed to terminate its child")
        return aggregate(rows, cfg), rows

    def aggregate(rows, cfg):
        return {
            "episodes": len(rows),
            "successes": sum(r["strict_success"] for r in rows),
            "actual_elapsed_sim_s": sum(r["simulated_s"] for r in rows),
        }

    class Ready:
        returncode = 0

        def communicate(self, timeout):
            return "offline", ""

    with patch.multiple(
        verify,
        stage_config=lambda: cfg,
        read_anchor=lambda: (15 / 16 if mode == "partial-scaled" else 1.0, spec),
        check_numerics=lambda: None,
        clear_agent_account=lambda force: {"cleared": True},
        deprivileged=lambda *a, **k: Ready(),
        solver_environment=lambda: {},
        offline_command=lambda *args: [],
        seed_rng=lambda: object(),
        summarize=aggregate,
        run_panel=simulated_courses,
        participant_episodes=lambda _: nullcontext(None),
    ):
        verify.verify()


def check():
    with tempfile.TemporaryDirectory() as tmp:
        for mode, finished, wins in (
            ("none", 0, 0),
            ("partial", 5, 3),
            ("partial-scaled", 5, 3),
            ("complete", 16, 3),
        ):
            output = Path(tmp) / (mode + ".json")
            env = dict(os.environ, ROBOTICS_GRADER_VERDICT_PATH=str(output), PYTHONDONTWRITEBYTECODE="1")
            env.pop("ALE_VERIFY_PYTHON", None)
            subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--child", mode],
                env=env,
                check=True,
                timeout=10,
            )
            verdict = json.loads(output.read_text())
            metrics = verdict["metrics"]
            assert verdict["rewards"]["reward"] == (wins / 15 if mode == "partial-scaled" else wins / 16)
            assert metrics["strict_course_fraction"] == wins / 16
            assert metrics["episodes"] == 16 and metrics["successes"] == wins
            assert (
                metrics["episodes_completed"] == finished and metrics["episodes_unfinished"] == 16 - finished
            )
            assert metrics["actual_elapsed_sim_s"] == 2 * finished
            assert bool(metrics["panel_deadline_exhausted"]) == (mode != "complete")
            ledger = json.loads(output.with_name(mode + "-courses.json").read_text())
            assert len(ledger) == finished
            if mode != "complete":
                assert (
                    metrics["mean_completion_sim_s_failure_at_horizon"] == (2 * wins + (16 - wins) * 100) / 16
                )
    print("0/16 on an empty panel, 3/16 after intentional interruption and 3/16 after ordinary completion.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--child", choices=("none", "partial", "partial-scaled", "complete"))
    args = parser.parse_args()
    if args.child:
        injected_child(args.child)
    else:
        check()
