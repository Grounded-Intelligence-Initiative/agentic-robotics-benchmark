"""Stationary receiver geometry and exported scene reproducibility."""
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

ENV = Path(__file__).resolve().parents[1] / "environment"
sys.path.insert(0, str(ENV))
from arena import build_arena, export_mjcf, preview_data
from workcell import ArenaRound, check_physics


def test_receivers_remain_fixed_while_robot_and_physics_advance():
    world = ArenaRound(17)
    try:
        targets = {item.target for item in world.items.values()}
        assert len(world.items) == 13
        assert len(targets) == 11
        before = {name: (world.data.geom(name).xpos.copy(),
                         world.data.geom(name).xmat.copy()) for name in targets}
        for name in targets:
            body = world.model.geom(name).bodyid[0]
            assert world.model.body_weldid[body] == 0, name
        initial_qpos = world.data.qpos.copy()
        for _ in range(160):
            for side, arm in world.arms.items():
                target = np.array([.075 if side == "left" else -.075, 1.21, 1.24, .04, 0., 0.])
                world.drive(arm, target, .021)
            previous_time = float(world.data.time)
            mj.mj_step(world.model, world.data)
            check_physics(world.data, previous_time)
        assert world.data.time > 0
        assert np.max(np.abs(world.data.qpos - initial_qpos)) > .001
        for name, (position, orientation) in before.items():
            np.testing.assert_array_equal(world.data.geom(name).xpos, position)
            np.testing.assert_array_equal(world.data.geom(name).xmat, orientation)
    finally:
        world.close()


def test_exported_mjcf_preserves_stationary_scene_and_physics(tmp_path):
    model, root = build_arena(37)
    data = preview_data(model)
    export_mjcf(root, tmp_path)
    exported = mj.MjModel.from_xml_path(str(tmp_path / "arena.xml"))
    replay = preview_data(exported)
    np.testing.assert_array_equal(data.qpos, replay.qpos)
    np.testing.assert_array_equal(data.ctrl, replay.ctrl)
    np.testing.assert_array_equal(model.body_weldid, exported.body_weldid)
    np.testing.assert_array_equal(model.geom_bodyid, exported.geom_bodyid)
    for _ in range(160):
        for current_model, current_data in ((model, data), (exported, replay)):
            previous_time = float(current_data.time)
            mj.mj_step(current_model, current_data)
            check_physics(current_data, previous_time)
    np.testing.assert_array_equal(data.qpos, replay.qpos)
    np.testing.assert_array_equal(data.qvel, replay.qvel)
    np.testing.assert_array_equal(data.qacc_warmstart, replay.qacc_warmstart)
    np.testing.assert_array_equal(data.geom_xpos, replay.geom_xpos)
