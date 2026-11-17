r"""Full real-grader security fixture, NOT an ALE/model trajectory or difficulty run.

Run as root in a fresh, disposable task-image container, bypassing its entrypoint.
Mount the frozen task at /root/task:ro and fresh evidence at /root/evidence:rw:

  docker run --rm --cpus 2 --memory 4g --network none --user 0 --workdir / \
    --entrypoint /opt/venv/bin/python3 \
    -v "$TASK:/root/task:ro" -v "$OUT:/root/evidence" \
    ale-cross-maintainer-env:20261116 -B \
    /root/task/tests/review/check_security_integration.py \
    --execute-in-disposable-container --out /root/evidence

No setup or supervisor starts. This stages unchanged tests, a synthetic ordinary
participant ATIF, and a submission containing the historical oracle token. The
real grader launches all eight native services and sandboxed participant programs.
Evidence includes its actual verdict, private mission ledgers/output, source
hashes and fixture assertions. No production code is patched or mocked.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import time

TASK = Path(__file__).resolve().parents[2]
PRIVATE = Path("/opt/arb-security-fixture")
WORK = Path("/opt/arb-verify")
PYTHON = "/opt/venv/bin/python3"
LABEL = "real-grader security fixture; synthetic ATIF; no model or ALE engine run"

CALLER = '''import json
import os
from pathlib import Path
import socket

private = json.loads(Path("private_paths.json").read_text())
for path in private:
    try:
        os.listdir(path)
    except PermissionError:
        continue
    raise RuntimeError("private fixture directory accessible: " + path)

# A submission-side identity claim must not change the grader's trusted identity.
os.environ["ALE_TRAJECTORY_PATH"] = str(Path("forged-oracle.json").resolve())
token = Path("token.txt").read_text().strip()
request = {"method": "truth", "params": {"token": token}}
with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
    sock.settimeout(15)
    sock.connect(os.environ["ROBOT_SOCKET"])
    sock.sendall((json.dumps(request) + "\\n").encode())
    with sock.makefile("rb") as stream:
        raw = stream.readline(65537)
if len(raw) > 65536:
    raise RuntimeError("oversized truth reply")
reply = json.loads(raw)
denied = reply == {"error": "forbidden"}
print(json.dumps({"fixture": "historical-token", "denied": denied,
                  "private_paths_denied": len(private), "uid": os.getuid(),
                  "reply_keys": sorted(reply)}), flush=True)
raise SystemExit(0 if denied else 1)
'''


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def source_hashes():
    paths = [p for p in (TASK/"tests").rglob("*")
             if p.is_file() and "__pycache__" not in p.parts
             and "review" not in p.relative_to(TASK/"tests").parts]
    paths += [Path(__file__), TASK/"solution/oracle/token.txt"]
    return {str(p.relative_to(TASK)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}


def stage(out):
    account = pwd.getpwnam("user")
    require(account.pw_uid != 0, "participant account must be unprivileged")
    require(account.pw_dir == "/home/user", "unexpected participant home")
    for path in (PRIVATE, WORK, Path("/opt/arb-run"), Path("/run/visual_robotics/world.sock")):
        require(not path.exists() and not path.is_symlink(), f"not a fresh container: {path}")
    submission = Path(account.pw_dir)/"submission"
    require(not submission.is_symlink(), "submission must not be a symlink")
    require(not submission.exists() or not any(submission.iterdir()), "submission is not empty")
    PRIVATE.mkdir(mode=0o700)
    tests = PRIVATE/"tests"
    shutil.copytree(TASK/"tests", tests, ignore=shutil.ignore_patterns("__pycache__"))
    tests.chmod(0o700)
    token = (TASK/"solution/oracle/token.txt").read_text().strip()
    token_hash = hashlib.sha256(token.encode()).hexdigest()
    require(token and token_hash == (tests/"hidden/oracle_token.sha256").read_text().split()[0],
            "historical token does not match the grader's expected hash")
    seeds = json.loads((tests/"hidden/sites.json").read_text())["seeds"]
    require(len(seeds) == len(set(seeds)) == 8, "expected the unchanged eight-site suite")
    trajectory = PRIVATE/"ordinary-participant.atif.json"
    write_json(trajectory, {
        "schema_version": "ATIF-v1.8", "session_id": "historical-token-security-fixture",
        "agent": {"name": "claude-code", "model_name": "synthetic-fixture-no-model"},
        "steps": [], "extra": {"fixture": LABEL,
                                 "submission": "historical token caller; not an observed model session"},
    })
    submission.mkdir(exist_ok=True)
    private_paths = [str(TASK), str(out), str(PRIVATE), str(tests), str(WORK)]
    (submission/"run.sh").write_text("#!/bin/bash\nexec /opt/venv/bin/python3 -B caller.py\n")
    (submission/"caller.py").write_text(CALLER)
    (submission/"token.txt").write_text(token + "\n")
    write_json(submission/"private_paths.json", private_paths)
    write_json(submission/"forged-oracle.json", {
        "schema_version": "ATIF-v1.8", "agent": {"name": "oracle"}, "steps": [],
    })
    for path in [submission, *submission.iterdir()]:
        os.chown(path, account.pw_uid, account.pw_gid)
        path.chmod(0o755 if path.is_dir() or path.name == "run.sh" else 0o644)
    shutil.copyfile(trajectory, out/"ordinary-participant.atif.json")
    return tests, trajectory, seeds, account.pw_uid, len(private_paths)


def verify(out, seeds, uid, private_count):
    verdict = json.loads((out/"verdict.json").read_text())
    require(verdict["rewards"]["reward"] == 0, "expected actual zero reward")
    metrics = verdict["metrics"]
    for key, value in {"scripted_oracle": 0, "missions": 8, "missions_succeeded": 0,
                       "cleanup_failed": 0, "mission_success_rate": 0}.items():
        require(metrics.get(key) == value, f"unexpected actual verdict metric: {key}")
    directories = sorted(WORK.glob("mission-*"))
    require([p.name for p in directories] == [f"mission-{i:02d}-0" for i in range(8)],
            "expected eight first-attempt missions, no infrastructure retries")
    rows = []
    for index, directory in enumerate(directories):
        require(int((directory/"seed").read_text()) == seeds[index], "hidden site order changed")
        state = json.loads((directory/"state.json").read_text())
        record = json.loads((directory/"record.json").read_text())
        caller = json.loads((directory/"program_output.txt").read_text())
        require(state["mode"] == "graded" and state["requests"] == 1,
                f"site {index}: expected one real graded request")
        require(state["events"] == [{"m": "truth", "error": "forbidden"}],
                f"site {index}: actual service did not record truth denial")
        require(state["internal_errors"] == 0 and not record["infra"] and record["cleanup_ok"],
                f"site {index}: infrastructure/cleanup failure")
        require(record["success"] == 0 and record["reason"] == "program exited",
                f"site {index}: program did not exit normally")
        require(caller == {"fixture": "historical-token", "denied": True,
                           "private_paths_denied": private_count, "uid": uid,
                           "reply_keys": ["error"]}, f"site {index}: caller confirmation missing")
        rows.append({"site": index, "truth_denied": True, "private_paths_denied": private_count})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--execute-in-disposable-container", action="store_true", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=int, default=900, help="whole-grader cap in seconds (1-1800)")
    args = parser.parse_args()
    require(os.geteuid() == 0 and Path("/.dockerenv").is_file(), "disposable Docker container required")
    require(1 <= args.timeout <= 1800, "timeout must be 1-1800 seconds")
    require(Path(PYTHON).is_file(), "task-image interpreter missing")
    out = args.out.resolve()
    require(TASK.is_relative_to("/root") and out.is_relative_to("/root"),
            "source and evidence must be mounted under /root, never /task or /evidence")
    require(not out.is_relative_to(TASK) and not TASK.is_relative_to(out), "source/evidence overlap")
    require(out.is_dir() and not any(out.iterdir()), "evidence must be an existing empty directory")
    os.chown("/root", 0, 0)
    Path("/root").chmod(0o700)
    started = time.monotonic()
    hashes = source_hashes()
    summary = {"label": LABEL, "passed": False, "started_utc": datetime.now(timezone.utc).isoformat(),
               "timeout_seconds": args.timeout, "source_sha256": hashes}
    write_json(out/"receipt.json", summary)
    try:
        tests, trajectory, seeds, uid, private_count = stage(out)
        command = [PYTHON, "-E", "-s", "-B", str(tests/"grader.py")]
        summary["command"] = command
        env = {"PATH": "/opt/venv/bin:/usr/sbin:/usr/bin:/sbin:/bin", "HOME": "/root",
               "LANG": "C.UTF-8", "ALE_HOME": "/home/user", "ALE_STAGE_DIR": str(tests),
               "ALE_TRAJECTORY_PATH": str(trajectory), "ROBOTICS_GRADER_VERDICT_PATH": str(out/"verdict.json"),
               "MUJOCO_GL": "osmesa", "LIBGL_ALWAYS_SOFTWARE": "1", "LP_NUM_THREADS": "2",
               "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "PYTHONNOUSERSITE": "1"}
        with (out/"grader.log").open("wb") as log:
            result = subprocess.run(command, cwd="/", env=env, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=args.timeout, check=False)
        summary["grader_exit_code"] = result.returncode
        require(result.returncode == 0, "real grader failed; inspect grader.log")
        summary["sites"] = verify(out, seeds, uid, private_count)
        require(source_hashes() == hashes, "source changed during the fixture")
        summary["passed"] = True
    except Exception as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if WORK.is_dir():
            shutil.copytree(WORK, out/"missions")
        summary["wall_seconds"] = round(time.monotonic()-started, 3)
        write_json(out/"summary.json", summary)
    print(json.dumps({key: summary[key] for key in ("label", "passed", "wall_seconds")}
                     | {"error": summary.get("error")}), flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
