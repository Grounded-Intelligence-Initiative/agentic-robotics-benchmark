"""Check the installed public native runtime: python -m sonic_loco.check /opt/sonic/assets."""
import argparse
from copy import copy
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from . import LocomotionSession


def _vectors(state):
    return np.concatenate([np.ravel(state["observation"][k])
                           for k in ("q", "dq", "base_pos", "base_quat", "base_velocity_world",
                                     "base_gyro", "left_foot_pos", "right_foot_pos")])


def _run(session, seed, render=False):
    initial = session.reset(seed)
    actual = session.data
    model = session.model
    shadow = copy(actual)
    assert (model.nq, model.nv, model.nu, model.neq) == (50, 49, 43, 0)
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert sum(model.geom_bodyid == 0) == 1 and model.geom("floor").id >= 0
    assert 0 not in model.actuator_trnid[:, 0]
    origin = np.asarray(initial["metrics"]["base_xy"])
    heading = initial["metrics"]["heading_rad"]
    direction = np.array([np.cos(heading), np.sin(heading)])
    goal = session.relative_goal(origin + direction, heading - np.pi / 2)
    assert np.allclose([goal["forward_m"],goal["left_m"],goal["heading_error_rad"]],
                       [1.,0.,-np.pi/2], rtol=0., atol=1e-14)
    digest = hashlib.sha256()
    digest.update(_vectors(initial).tobytes())
    if render:
        image = session.camera("overview",128,96)
        assert image.shape == (96,128,3) and image.dtype == np.uint8
        assert session.tick == 0
    rows = 5
    for i in range(rows):
        row = np.roll(np.linspace(-1., .9375, 64, dtype=np.float32), i)
        history = session.decoder_history()
        assert history.shape == (930,) and np.array_equal(history,session.decoder_history())
        decoder = session._decoder
        predicted = decoder.joint_targets(decoder.decode_observation(np.concatenate([row,history])))
        out = session.step(row, .25, .75)
        assert np.array_equal(session.last_target,predicted)
        assert out["tick"] == i+1 and len(session.last_substeps)==10
        assert out["reason"] == ("time_limit" if i+1==rows else None)
        shadow.ctrl[:] = actual.ctrl
        for j in range(10):
            velocity = shadow.qvel[:2].copy()
            time = shadow.time
            mujoco.mj_step(model,shadow)
            if j:
                sample = session.last_substeps[j-1]
                assert abs(sample["time_s"] - time) < 1e-14
                assert np.array_equal(sample["velocity_xy"],velocity)
                assert np.array_equal(sample["base_xy"],shadow.body("pelvis").xpos[:2])
                assert np.array_equal(sample["left_foot_pos"],shadow.site("left_foot").xpos)
                assert np.array_equal(sample["right_foot_pos"],shadow.site("right_foot").xpos)
        mujoco.mj_forward(model,shadow)
        assert np.array_equal(session.last_substeps[-1]["velocity_xy"],shadow.qvel[:2])
        assert not np.any(actual.xfrc_applied) and not np.any(actual.qfrc_applied)
        for name in ("qpos","qvel","ctrl","qacc","actuator_force"):
            values = getattr(actual,name)
            assert np.array_equal(values,getattr(shadow,name))
            digest.update(values.tobytes())
        digest.update(_vectors(out).tobytes())
    return digest.hexdigest(), origin, heading


def check(assets, render=False):
    tested = (0, 7, 2**32-1, 2**64-1)
    with LocomotionSession(assets, 0, horizon=5) as session:
        fingerprints = {}
        starts = []
        for seed in tested:
            fingerprint, position, heading = _run(session,seed,render)
            second, _, _ = _run(session,seed)
            assert fingerprint == second
            fingerprints[str(seed)] = fingerprint
            starts.append((*position,heading))
        assert len(set(starts)) == len(tested)
        for invalid in (-1,2**64,True):
            try:
                session.reset(invalid)
            except ValueError:
                continue
            raise AssertionError("invalid seed accepted")
    return dict(repeated_resets=len(tested),native_control_rows=2*len(tested)*5,
                native_substeps=2*len(tested)*50,exact_native_fingerprints=fingerprints,
                renderer_tested=render)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets_root",type=Path)
    parser.add_argument("--render",action="store_true")
    args = parser.parse_args()
    print(json.dumps(check(args.assets_root,args.render),indent=2))
