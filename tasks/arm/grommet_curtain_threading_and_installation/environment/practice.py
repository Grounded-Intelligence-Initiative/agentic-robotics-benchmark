"""Run a submission on one public layout through the same controller interface."""

from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys

from isaaclab.app import AppLauncher


def run(submission, steps, log_dir):
    from curtain_env import CurtainEnv

    env = CurtainEnv(log_dir, seed=17)
    with subprocess.Popen(["bash", "run.sh"], cwd=submission, stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, text=True) as child:
        def request(message):
            print(json.dumps(message), file=child.stdin, flush=True)
            return json.loads(child.stdout.readline())

        if child.stdout.readline().strip() != "READY":
            raise ValueError("run.sh must print READY on startup")
        try:
            request({"type": "reset"})
            for t in range(steps):
                reply = request({"type": "act", "t": t, "obs": env.observation()})
                env.step(reply["action"])
            request({"type": "close"})
            result = env.measure()
            print(json.dumps({key: result[key] for key in (
                "correct_eyelets", "eyelets_between_hooks", "rod_on_both_brackets", "installed",
            )}), flush=True)
        finally:
            if child.poll() is None:
                child.terminate()


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit("Usage: python3 /opt/curtain/practice.py SUBMISSION [STEPS]")
    submission = Path(sys.argv[1]).resolve()
    steps = int(sys.argv[2]) if len(sys.argv) == 3 else 120
    if steps < 1:
        raise ValueError("STEPS must be positive")
    out = Path.home() / "output" / datetime.now().strftime("%y%m%d-%H%M%S-curtain-practice")
    out.mkdir(parents=True)
    app = AppLauncher(headless=True, enable_cameras=True,
                      kit_args=f"--/log/file={out}/kit.log --/plugins/carb.tasking.plugin/threadCount=8").app
    try:
        run(submission, steps, out)
    except BaseException:
        import traceback
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    else:
        app.close()


if __name__ == "__main__":
    main()
