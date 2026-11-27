"""Sensor-only observation contract and physical image checks."""
import base64
import io
from pathlib import Path
import sys
import zlib

import mujoco as mj
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'environment'))
from workcell import ArenaRound


def test_observation_contains_only_rgbd_and_robot_proprioception():
    world = ArenaRound(17)
    try:
        observation = world.observation()
        assert set(observation) == {'time', 'arms', 'cameras'}
        assert set(observation['arms']) == {'left', 'right'}
        for arm in observation['arms'].values():
            assert set(arm) == {'joints', 'joint_velocity', 'finger_positions', 'finger_velocity',
                                'reference', 'jaw_reference'}
        assert set(observation['cameras']) == {'left_overhead', 'left_oblique', 'right_overhead', 'right_oblique'}
        for camera in observation['cameras'].values():
            assert set(camera) == {'width', 'height', 'K', 'T_world_camera', 'rgb_png_base64',
                                   'depth_f32_zlib_base64', 'depth_units', 'depth_convention', 'depth_dtype'}
            rgb, depth = decode(camera)
            assert rgb.shape == (480, 640, 3)
            assert depth.shape == (480, 640)
            assert np.isfinite(depth).all() and np.min(depth) > 0
    finally:
        if hasattr(world, 'close'):
            world.close()


def decode(camera):
    rgb = np.array(Image.open(io.BytesIO(base64.b64decode(camera['rgb_png_base64']))))
    depth = np.frombuffer(zlib.decompress(base64.b64decode(camera['depth_f32_zlib_base64'])), dtype='<f4')
    return rgb, depth.reshape(camera['height'], camera['width'])


def test_depth_unprojects_to_metric_world_plane():
    from rgbd_sensor import RGBDSensor
    model = mj.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" pos="0 0 .72" size="10 10 .1"/></worldbody></mujoco>')
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    sensor = RGBDSensor(model)
    try:
        for camera in sensor.observe(data).values():
            _, depth = decode(camera)
            rows, columns = np.meshgrid(np.arange(80, 401, 80), np.arange(80, 561, 80), indexing='ij')
            pixels = np.stack((columns.ravel(), rows.ravel(), np.ones(rows.size)))
            intrinsic = np.array(camera['K']).reshape(3, 3)
            transform = np.array(camera['T_world_camera']).reshape(4, 4)
            points = np.linalg.solve(intrinsic, pixels) * depth[rows.ravel(), columns.ravel()]
            world_points = transform[:3, :3] @ points + transform[:3, 3, None]
            np.testing.assert_allclose(world_points[2], .72, atol=2e-5)
    finally:
        sensor.close()


def test_opaque_surface_hides_rear_object_in_rgb_and_depth():
    from rgbd_sensor import RGBDSensor
    model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
        <geom type="plane" size="10 10 .1"/>
        <geom name="rear" type="sphere" pos=".6 1.35 1" size=".08" rgba="1 0 0 1"/>
        <geom name="front" type="box" pos=".6 1.35 1.4" size=".14 .14 .05" rgba=".2 .8 .2 1"/>
        </worldbody></mujoco>''')
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    sensor = RGBDSensor(model)
    try:
        first_rgb, first_depth = decode(sensor.observe(data)['left_overhead'])
        model.geom('rear').rgba[:] = [0, 0, 1, 1]
        hidden_rgb, hidden_depth = decode(sensor.observe(data)['left_overhead'])
        np.testing.assert_array_equal(first_rgb, hidden_rgb)
        np.testing.assert_array_equal(first_depth, hidden_depth)
        model.geom('front').rgba[3] = 0
        visible_rgb, visible_depth = decode(sensor.observe(data)['left_overhead'])
        assert np.max(np.abs(visible_rgb[235:245, 315:325].astype(float) - hidden_rgb[235:245, 315:325])) > 30
        assert visible_depth[240, 320] - hidden_depth[240, 320] > .3
    finally:
        sensor.close()


def test_observation_does_not_advance_or_change_integrated_physics():
    world = ArenaRound(17)
    specification = mj.mjtState.mjSTATE_INTEGRATION
    state = np.zeros(mj.mj_stateSize(world.model, specification))
    after = np.zeros_like(state)
    mj.mj_getState(world.model, world.data, state, specification)
    parameters = [world.model.body_mass.copy(), world.model.body_inertia.copy(), world.model.body_pos.copy(),
                  world.model.geom_pos.copy(), world.model.geom_size.copy(), world.model.actuator_gainprm.copy()]
    try:
        first = world.observation()
        second = world.observation()
        assert first == second
        mj.mj_getState(world.model, world.data, after, specification)
        np.testing.assert_array_equal(state, after)
        for before, current in zip(parameters, (world.model.body_mass, world.model.body_inertia, world.model.body_pos,
                                                world.model.geom_pos, world.model.geom_size, world.model.actuator_gainprm)):
            np.testing.assert_array_equal(before, current)
    finally:
        world.close()
    assert world._sensor is None


def test_real_motion_changes_rgb_and_depth():
    world = ArenaRound(17)
    try:
        first = world.observation()
        arm = world.arms['left']
        row = [0., *arm['ref'], arm['jaw']]
        world.step({'arm': 'left', 'item': 'left_item0', 'waypoints': [row, [6., *row[1:]]]})
        second = world.observation()
        rgb_changes = depth_changes = 0
        for name in first['cameras']:
            before_rgb, before_depth = decode(first['cameras'][name])
            after_rgb, after_depth = decode(second['cameras'][name])
            rgb_changes += np.count_nonzero(before_rgb != after_rgb)
            depth_changes += np.count_nonzero(np.abs(before_depth - after_depth) > .001)
        assert rgb_changes > 100
        assert depth_changes > 100
        assert second['time'] > 5.99
    finally:
        world.close()


def test_recording_is_private_and_contains_actual_world_frames(tmp_path):
    from arena_episode import run_episode

    class Controller:
        def act(self, turn, observation):
            assert set(observation) == {'time', 'arms', 'cameras'}
            if turn:
                return {'finish': True}
            arm = observation['arms']['left']
            row = [0., *arm['reference'], arm['jaw_reference']]
            return {'arm': 'left', 'item': 'left_item0', 'waypoints': [row, [6., *row[1:]]]}

    class NoProbe:
        def due(self):
            return False

    result = run_episode(17, 37, Controller(), NoProbe(), record_path=tmp_path)
    assert result['actions'] == 1
    frames = np.load(tmp_path / 'frames.npz')
    assert set(frames.files) == {'qpos', 'time', 'action'}
    assert frames['qpos'].dtype == np.float32
    assert frames['time'][0] == 0 and frames['time'][-1] > 5.99
    assert len(frames['qpos']) == len(frames['time']) > 20
    assert (tmp_path / 'result.json').is_file()


def test_pixel_projection_preserves_world_axes_and_rgb_depth_registration():
    from rgbd_sensor import RGBDSensor
    import xml.etree.ElementTree as ET

    root = ET.Element('mujoco')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', type='plane', size='10 10 .1')
    markers = {}
    for side, y in (('left', 1.35), ('right', -1.24)):
        markers[side] = [(np.array([.3, y - .3, 1.0]), 0),
                         (np.array([.9, y - .3, 1.0]), 1),
                         (np.array([.6, y + .3, 1.2]), 2)]
        for point, channel in markers[side]:
            rgba = [0., 0., 0., 1.]
            rgba[channel] = 1.
            ET.SubElement(world, 'geom', type='sphere', pos=' '.join(map(str, point)),
                          size='.04', rgba=' '.join(map(str, rgba)))
    model = mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    sensor = RGBDSensor(model)
    try:
        for name, camera in sensor.observe(data).items():
            intrinsic = np.array(camera['K']).reshape(3, 3)
            transform = np.array(camera['T_world_camera']).reshape(4, 4)
            rgb, depth = decode(camera)
            for point, channel in markers[name.split('_')[0]]:
                camera_point = transform[:3, :3].T @ (point - transform[:3, 3])
                projected = intrinsic @ camera_point
                u, v = np.rint(projected[:2] / projected[2]).astype(int)
                assert 0 <= u < 640 and 0 <= v < 480
                pixel = rgb[v, u].astype(float)
                assert pixel[channel] > 2 * np.max(np.delete(pixel, channel)), (name, point, pixel)
                measured = np.linalg.solve(intrinsic, [u, v, 1.]) * depth[v, u]
                world_point = transform[:3, :3] @ measured + transform[:3, 3]
                assert abs(np.linalg.norm(world_point - point) - .04) < .002
    finally:
        sensor.close()
