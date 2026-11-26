"""Run public practice at seed 7 and compare its per-row motion report to the formal diagnostic."""
import argparse
from contextlib import redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK / "environment/setup/payload"), str(TASK / "tests"), "/opt/sonic"]
import practice
import verify
from sonic_io import LiveEpisode
from sonic_sim import BenchSession

# State/target digests from the original published V4 RGB practice recordings.
PRACTICE = {
    "box": (554, "00eda0d606f8ee12e4e63e886a2be1da1c4ce04babb27d3c568df5d428bfb5ee",
            "840d42f2e3afd08f7dd820aba4bcaf8d0b9d4b556581dab9d2e8c991c9db8afc"),
    "cylinder": (551, "d6f5bf2e28c3b5c9a2c3fcf805596ccca3694c4a5e5e037f3bbf0bb848fa11f1",
                 "234e9df2860c605a8d61e2581228e4a8cc3474af308ef1c56f5bf392b2cc76d4"),
}


def state_bytes(state):
    return json.dumps(state, sort_keys=True, separators=(",", ":"), allow_nan=False,
                      default=lambda a: a.tolist()).encode() + b"\n"


def check(home, mode, steps, shape="box"):
    assets = Path("/opt/sonic/assets")
    submission = home / "submission"
    submission.mkdir(parents=True)
    if mode == "v4":
        subprocess.run(["bash", str(TASK / "solution/solve.sh")], check=True,
                       env={**os.environ, "ALE_HOME": str(home), "SONIC_REFERENCE_POLICY": "v4"})
    else:
        shutil.copyfile(TASK / "environment/setup/payload/starter.py", submission / "controller.py")
        if mode == "single":
            (submission / "single.py").write_text(
                "import controller\n"
                "controller.act=lambda obs: {'rows':[{'codes':[obs['tick'] % 32]+[16]*63,'hand':[0,0]}]}\n"
                "controller.main()\n")
        program = "single.py" if mode == "single" else "controller.py"
        (submission / "run.sh").write_text(f"exec /opt/venv/bin/python3 -u {program}\n")

    with BenchSession(shape + "_to_pad_challenge", 7, assets) as native:
        first = native.state
        official = verify.MeasuredSession(native, first["observation"]["dq"])
        state_trace = hashlib.sha256(state_bytes(first))
        target_trace = hashlib.sha256()
        step = official.step

        def record(token, left, right):
            state = step(token, left, right)
            state_trace.update(state_bytes(state))
            target_trace.update(np.ascontiguousarray(native.last_target, dtype="<f8").tobytes())
            return state

        official.step = record
        episode = LiveEpisode(official)
        args = SimpleNamespace(seed=7, task=shape+"_to_pad", steps=steps, assets_root=assets, submission=submission)
        output = io.StringIO()
        with redirect_stdout(output):
            practice.run(episode, args)
        result = json.loads(output.getvalue())
        assert episode.session is official
        ticks = native.tick
        expected = verify.motion_metrics([dict(ticks=ticks, body_code_changes=official.body_code_changes,
                                              acceleration_squared_sum=official.acceleration_squared_sum)])
        assert result["motion"] == expected and result["steps"] == ticks
        assert result["metrics"] == native.state["metrics"]
        assert result["repeats"]["identical_queries"] == 1
        if mode == "v4":
            expected_ticks, states, targets = PRACTICE[shape]
            assert (ticks, state_trace.hexdigest(), target_trace.hexdigest()) == (expected_ticks, states, targets)
            assert result["reason"] == "success" and result["metrics"]["placed"]
            assert ticks % 5 != 0
        else:
            assert ticks == steps and result["reason"] == "step_limit"
            assert expected["body_code_changes"] == (ticks - 1 if mode == "single" else 0)
            assert ("joint_acceleration_rms_rad_s2" in expected) == (steps >= 5)
            assert ("body_code_churn_fraction" in expected) == (steps >= 2)
        return dict(mode=mode, shape=shape, steps=ticks, reason=result["reason"], motion=expected,
                    repeats=result["repeats"], states_sha256=state_trace.hexdigest(),
                    targets_sha256=target_trace.hexdigest())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("short", "box", "cylinder", "all"), default="all")
    args = parser.parse_args()
    rows = []
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        if args.case in ("short", "all"):
            for mode, steps in (("neutral", 1), ("neutral", 4), ("neutral", 7), ("single", 7)):
                rows.append(check(root / f"{mode}-{steps}", mode, steps))
        for shape in PRACTICE:
            if args.case in (shape, "all"):
                rows.append(check(root / shape, "v4", 750, shape))
    print(json.dumps(rows, indent=2, allow_nan=False))
