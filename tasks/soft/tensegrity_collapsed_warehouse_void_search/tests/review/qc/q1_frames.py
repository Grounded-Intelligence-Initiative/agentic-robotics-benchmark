"""QC Q1 (frames): a fixed 20-frame sequence on a frozen hidden site, rendered by the task's cell.

    q1_frames.py OUT.json [--site h03]

One in-process cell (tests/voidcell/server.py, mode hidden, the task's grader_config.json render settings and the
hidden time limit) on a frozen hidden package with its pinned seed. The sequence: 20 x (one camera frame, then a
fixed command: a primitive with the true bottom triple, run until idle, or a plain step), so the frames show the robot
at 20 different states and every exposure is a different index. Writes every frame's index, time and sha256 (the
cell's own Frame.sha256 of the rgb and depth arrays), the robot's state hash after the sequence and the process id.
Run it twice in two processes and compare (the QC compares the two files).
"""
import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEV = os.path.normpath(os.path.join(HERE, "..", ".."))
TASK = DEV[:-len("-dev")]
TESTS = os.path.join(TASK, "tests")
sys.path.insert(0, TESTS)

from voidcell.server import VoidCellServer  # noqa: E402
from voidcell import client as vclient  # noqa: E402

SEQ = ("roll_120_120", None, "ccw", "roll_100_140", None, "cw", "roll_140_100", "ccw", None, "roll_120_140",
       "roll_100_120", None, "cw", "roll_140_140", "ccw", None, "roll_100_100", "cw", None, "roll_120_100")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--site", default="h03")
    a = ap.parse_args()
    cfg = json.load(open(os.path.join(TESTS, "grader_config.json")))
    lim = json.load(open(os.path.join(TESTS, "hidden", "limits.json")))
    seeds = json.load(open(os.path.join(TESTS, "hidden", "seeds.json")))
    row = [r for r in seeds["sites"] if r["tag"] == a.site][0]
    c = {k: cfg[k] for k in ("limits", "pose_source", "render_backend", "msaa_rgb") if k in cfg}
    c["limits"] = dict(c.get("limits") or {}, time_limit_s=float(lim["time_limit_s"]))
    srv = VoidCellServer(config=c, mode="hidden", render=True)
    srv.reset(os.path.join(TESTS, "hidden", row["archive"]), seed=int(row["seed"]), kind="hidden")
    st = vclient.connect_handler(srv.handle)
    frames = []
    for cmd in SEQ:
        f = st.get_frame()
        h = hashlib.sha256(f["rgb"].tobytes() + f["depth"].tobytes()).hexdigest()
        frames.append({"index": int(f["index"]), "t": float(f["t"]), "sha256_rgb_depth": h,
                       "cell_sha256": srv.frame_hashes[-1][1]})
        if cmd is None:
            st.step(1.0)
            continue
        b = srv.robot_truth.bottom()
        if not b["rollable"]:
            st.step(1.0)
            continue
        st.run_primitive(cmd, list(b["nodes"]))
        while st.status()["busy"]:
            st.step(0.5)
        st.step(0.3)
    out = {"site": a.site, "seed": int(row["seed"]), "pid": os.getpid(), "n_frames": len(frames), "frames": frames,
           "state_hash": srv.S.state_hash(), "t_end": float(srv.X.t()),
           "render_backend": srv.cam.backend.version() if srv.cam is not None else None,
           "env": {k: os.environ.get(k) for k in ("LP_NUM_THREADS", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                                                  "MUJOCO_GL", "PYOPENGL_PLATFORM", "OPENBLAS_CORETYPE")}}
    srv.close()
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps({"n": len(frames), "state": out["state_hash"][:16], "t_end": out["t_end"],
                      "distinct": len({x["sha256_rgb_depth"] for x in frames})}))


if __name__ == "__main__":
    main()
