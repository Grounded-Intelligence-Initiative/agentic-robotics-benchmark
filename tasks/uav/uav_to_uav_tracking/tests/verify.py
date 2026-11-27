"""Private labels remain in the parent; submitted Python/weights run deprivileged."""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from robotics_grader import (clear_agent_account, deprivileged, require, stage_config, stage_dir,
                             read_anchor, ratio_rewards, write_verdict, zero_verdict)
from nonet import nonet_argv
from uav_eval import infer, metrics, SubmissionError

def spawn(argv, **kwargs):
    # This seccomp launcher forbids Internet sockets and io_uring; inherited by children.
    env={"PATH":"/opt/venv/bin:/usr/bin:/bin",
         "HOME":os.environ.get("ALE_HOME","/home/user"),
         "OMP_NUM_THREADS":"4","PYTHONDONTWRITEBYTECODE":"1"}
    return deprivileged(nonet_argv(argv,sys.executable),env=env,**kwargs)

def main():
    config=stage_config()
    anchor,meta=read_anchor()
    home=Path(os.environ.get("ALE_HOME","/home/user"))
    submission=home/"submission"
    required=("model.py","model.pt","train.sh","config.json","training_log.jsonl")
    if any(not (submission/name).is_file() for name in required):
        write_verdict(zero_verdict(reason="missing submission artifacts",anchored=anchor is not None))
        return
    data=Path(config["data_root"])
    require((data/"annotations.json").is_file(),"held-out asset package has not been staged")
    report = clear_agent_account()
    require(report.get("cleared", False), "agent account could not be cleared before isolation")
    # Fail as a TASK error if the kernel cannot enforce the required network boundary.
    probe=spawn([sys.executable,"-I","-B","-c",
        "import socket\ntry: socket.socket(socket.AF_INET,socket.SOCK_STREAM)\nexcept PermissionError: pass\nelse: raise SystemExit(3)"])
    try:
        require(probe.wait(timeout=30)==0,"no-network isolation self-test failed")
    except subprocess.TimeoutExpired:
        probe.kill(); probe.wait()
        require(False,"no-network isolation self-test timed out")
    # Only this trusted, label-free adapter is made readable by the inference account.
    with tempfile.TemporaryDirectory(prefix="uav-worker-") as temp:
        os.chmod(temp,0o755)
        worker=Path(temp)/"worker.py"
        shutil.copyfile(Path(stage_dir())/"worker.py",worker)
        worker.chmod(0o644)
        try:
            dataset,predictions=infer(submission,data,worker,spawn=spawn,
                startup_timeout=config["startup_timeout_s"],frame_timeout=config["frame_timeout_s"],
                total_timeout=config["total_timeout_s"])
        except SubmissionError as exc:
            write_verdict(zero_verdict(reason=str(exc),anchored=anchor is not None))
            return
    result=metrics(dataset,predictions)
    verdict=ratio_rewards(result["uav_ap50"],anchor,metric_name="uav_ap50",
        higher_is_better=True,cap=config["scoring_cap"],full_at=meta.get("full_at",1.0),
        extra={k:v for k,v in result.items() if k!="uav_ap50"})
    write_verdict(verdict)

if __name__=="__main__":
    main()
