"""Maintainers' tool: the grader's own episode loop outside the engine, on this machine's GPU.

    <isaac python> tests/review/run_native.py --submission reference|idle|none|<folder> --out DIR [--only 0,1,2]

It calls tests/verify.py's `grade` and `score` (the same hidden set, sessions, probes, budgets, renderer and canary)
with two launchers of its own: the controller is started as the calling account instead of through `runuser`, and
the renderer as the calling account from this folder. Nothing else differs from the verify stage. The verdict (the
kit's plain envelope) goes to DIR/verdict_<name>.json, the log to stderr.

`reference` builds the reference submission as solution/solve.sh does; `idle` is the agent's example controller,
which never moves the holder; `none` is an absent submission; `bad:<mode>` is tests/review/bad_controller.py.

A native run is not `ale validate`: the image, the accounts, the engine's stages and the cloud GPU models are not
exercised (tests/README.md, status log).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))
os.environ.setdefault("ALE_STAGE_DIR", os.path.join(TASK, "tests"))

import verify  # noqa: E402
from endosim.render_client import Renderer, RendererError  # noqa: E402
from robotics_grader import SolverAborted, SolverMissing, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402


class LocalSolver(verify.TimedSolver):
    """The controller as the calling account (no `runuser`, no account clearing): everything else is the kit's."""

    def _spawn(self):
        return subprocess.Popen(["bash", self.entry], cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=open(os.path.join(self.submission, "..", "controller_stderr.log"), "ab"),
                                universal_newlines=True, bufsize=1, start_new_session=True)


def build(name, work):
    given = os.path.join(TASK, "environment", "setup", "payload", "given")
    sub = os.path.join(work, "submission")
    if name == "none":
        return sub
    if name == "reference":
        os.makedirs(sub)
        ref = os.path.join(TASK, "solution", "reference")
        for f in ("controller.py", "run.sh"):
            shutil.copy(os.path.join(ref, f), sub)
        shutil.copy(os.path.join(given, "holder_model.py"), sub)
        shutil.copy(os.path.join(ref, "oracle_token"), os.path.join(sub, ".oracle_token"))
    elif name.startswith("bad:"):
        os.makedirs(sub)
        shutil.copy(os.path.join(HERE, "bad_controller.py"), sub)
        with open(os.path.join(sub, "run.sh"), "w") as f:
            f.write("BAD_MODE={} exec /opt/venv/bin/python3 -B -s bad_controller.py\n".format(name.split(":", 1)[1]))
    elif name == "idle":
        shutil.copytree(os.path.join(TASK, "environment", "setup", "payload", "example"), sub)
        shutil.copy(os.path.join(given, "holder_model.py"), sub)
    else:
        shutil.copytree(name, sub)
    run = os.path.join(sub, "run.sh")       # outside the task image there is no /opt/venv: this interpreter instead
    text = open(run).read().replace("exec /opt/venv/bin/python3", 'exec "${ARB_PYTHON:-/opt/venv/bin/python3}"')
    open(run, "w").write(text)
    return sub


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default="reference")
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", help="indices of the hidden procedures to run (default: all)")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.environ["ARB_PYTHON"] = sys.executable
    config = stage_config()
    anchor, meta = read_anchor()
    name = os.path.basename(a.submission.rstrip("/")).replace(":", "_") + a.tag
    out = os.path.join(a.out, "verdict_{}.json".format(name))
    work = tempfile.mkdtemp(prefix="endo_native_", dir=a.out)
    sub = build(a.submission, work)
    only = [int(x) for x in a.only.split(",")] if a.only else None

    def make_renderer(cfg):
        path = os.path.join(a.out, "renderer_{}.log".format(name))
        return Renderer(os.path.join(TASK, "tests"), log_path=path, start_timeout=float(cfg.get("renderer_start_timeout_s", 2400)),
                        render_timeout=float(cfg.get("render_timeout_s", 120)), max_restarts=int(cfg.get("renderer_restarts", 3))), path

    try:
        records, lock, extra, diagnostic = verify.grade(
            sub, config, lambda timeout: LocalSolver(step_timeout=timeout, submission=sub, clear_account=False), make_renderer, only=only)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None), path=out)
        print(json.dumps(json.load(open(out))))
        return
    except RendererError as exc:
        raise SystemExit("the grading environment could not render: {}".format(exc))
    verdict = verify.score(records, lock, extra, diagnostic, config, anchor, meta, path=out)
    with open(os.path.join(a.out, "records_{}.json".format(name)), "w") as f:
        json.dump({"records": records, "diagnostic": diagnostic}, f, indent=1)
    print(diagnostic)
    print(json.dumps({"reward": verdict["rewards"]["reward"], "metrics": {k: v for k, v in verdict["metrics"].items() if not k.startswith("procedure_")}}))


if __name__ == "__main__":
    main()
