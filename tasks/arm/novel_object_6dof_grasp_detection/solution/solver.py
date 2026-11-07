"""Privileged reference detector. It shows that the task can be done when the ground truth of the frame is known.

Candidates come from grasplib and are ranked by a gradient-boosted scorer (the non-privileged reference). When the
grader hands over the frame's ground truth (object poses and meshes, only for a submission that carries the oracle
token, see tests/), the best candidates are labelled with graspnetAPI's own collision and force-closure test and
re-ranked by the true label. Without ground truth it returns the scorer's ranking."""
import os, sys, time, pickle
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grasplib  # noqa: E402

_MODEL = pickle.load(open(os.path.join(HERE, "scorer.pkl"), "rb"))
MAX_OUT = 2000
GEN_BUDGET = 20.0
N_LABEL = 500
LABEL_DEADLINE = 36.0        # seconds into the call by which labelling must be done
SEC_PER_LABEL = 0.03         # measured: about 0.028 s per candidate on one core
FRIC = np.array([0.2, 0.4, 0.6, 0.8, 1.0, 1.2])
_SCENE = {}


def _fallback(depth, intrinsics, depth_scale):
    pts = grasplib.backproject(depth, intrinsics, depth_scale)
    c = pts.mean(0) if len(pts) else np.array([0, 0, 0.5])
    row = np.zeros((1, 17)); row[0, 1] = 0.08; row[0, 2] = 0.02; row[0, 3] = 0.02
    row[0, 4:13] = np.eye(3)[:, [2, 0, 1]].reshape(-1); row[0, 13:16] = c; row[0, 16] = -1
    return row


def _scene(root, sid, view):
    """Meshes and grasp-quality models of one scene, kept for the next frame of the same scene only."""
    if _SCENE.get("key") == (root, sid):
        return _SCENE["val"]
    from graspnetAPI import GraspNetEval
    from graspnetAPI.utils.eval_utils import create_table_points, voxel_sample_points
    from graspnetAPI.utils.config import get_config
    from graspnetAPI.utils.dexnet.grasping.grasp_quality_config import GraspQualityConfigFactory
    ge = GraspNetEval.__new__(GraspNetEval); ge.root, ge.camera = root, "realsense"
    config = get_config()
    fc_list = np.array([1.2, 1.0, 0.8, 0.6, 0.4, 0.2])
    fcq = {}
    for v in fc_list:
        v = round(float(v), 2)
        config["metrics"]["force_closure"]["friction_coef"] = v
        fcq[v] = GraspQualityConfigFactory.create_config(config["metrics"]["force_closure"])
    table = create_table_points(1.0, 1.0, 0.05, dx=-0.5, dy=-0.5, dz=-0.05, grid_size=0.008)
    models, dexmodels, _ = ge.get_scene_models(sid, ann_id=view)
    sampled = [voxel_sample_points(m, 0.008) for m in models]
    _SCENE.clear()
    _SCENE.update(key=(root, sid), val=(ge, fc_list, fcq, table, dexmodels, sampled))
    return _SCENE["val"]


def _true_labels(G, root, sid, view):
    from graspnetAPI.utils.eval_utils import (transform_points, compute_closest_points, collision_detection,
                                              get_grasp_score)
    ge, fc_list, fcq, table, dexmodels, sampled = _scene(root, sid, view)
    _, pose_list, camera_pose, align_mat = ge.get_model_poses(sid, view)
    table_trans = transform_points(table, np.linalg.inv(np.matmul(align_mat, camera_pose)))
    mtl = [transform_points(m, pose_list[i]) for i, m in enumerate(sampled)]
    seg = np.concatenate([np.full(len(m), i) for i, m in enumerate(mtl)])
    owner = seg[compute_closest_points(G[:, 13:16], np.concatenate(mtl))]
    order = [np.nonzero(owner == i)[0] for i in range(len(mtl))]
    coll, _, dexg = collision_detection([G[o] for o in order], mtl, dexmodels, pose_list,
                                        np.concatenate(mtl + [table_trans]), outlier=0.05, return_dexgrasps=True)
    y = np.zeros(len(G))
    for i, o in enumerate(order):
        for j, gi in enumerate(o):
            if coll[i][j] or dexg[i][j] is None:
                continue
            s = get_grasp_score(dexg[i][j], dexmodels[i], fc_list, fcq)
            if s > 0:
                y[gi] = float(np.mean(s <= FRIC))
    return y


def predict(rgb, depth, intrinsics, depth_scale):
    t0 = time.time()
    try:
        root, frame = os.environ.get("ORACLE_GT_ROOT"), os.environ.get("ORACLE_FRAME")
        sid = view = None
        if root and frame:
            sid, view = (int(v) for v in frame.split("_"))
            _scene(root, sid, view)                       # the slow part; cached for the next frame of the scene
        loaded = time.time() - t0
        n_seeds = 400 if loaded < 15 else 150             # a frame that had to load its scene gets a lighter pass
        gen_budget = max(5.0, min(GEN_BUDGET, 34.0 - loaded))
        cand, _ = grasplib.generate(depth, intrinsics, depth_scale, n_seeds=n_seeds, deadline=t0 + gen_budget)
        if cand is None:
            return _fallback(depth, intrinsics, depth_scale)
        X = cand["feat"]
        s = np.nan_to_num(_MODEL.predict(X), nan=-1.0)
        n_label = int(min(N_LABEL, (LABEL_DEADLINE - (time.time() - t0)) / SEC_PER_LABEL)) if root and frame else 0
        if n_label >= 100:
            top = np.argsort(-s)[:n_label]
            G = grasplib.to_rows(cand, s, top)
            y = _true_labels(G, root, sid, view)
            G[:, 0] = y + 1e-3 * (s[top] - s[top].min()) / (np.ptp(s[top]) + 1e-9)
            rows = G[np.argsort(-G[:, 0])]
        else:
            idx = np.argsort(-s)[:MAX_OUT]
            rows = grasplib.to_rows(cand, s, idx)
        R = rows[:, 4:13].reshape(-1, 3, 3)
        ok = np.isfinite(rows).all(1) & (np.abs(np.linalg.det(R) - 1) < 1e-5)
        ok &= np.abs(np.einsum("nij,nik->njk", R, R) - np.eye(3)).max((1, 2)) < 1e-5
        ok &= (rows[:, 1] >= 0) & (rows[:, 3] > 0) & (rows[:, 3] <= 0.1)
        rows = rows[ok]
        return rows if len(rows) else _fallback(depth, intrinsics, depth_scale)
    except Exception:
        return _fallback(depth, intrinsics, depth_scale)
