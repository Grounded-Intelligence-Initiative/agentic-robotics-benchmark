"""Decode calibrated RGB-D observations and compute the robot's own kinematics."""
import argparse
import base64
import io
import json
from pathlib import Path
import zlib

import numpy as np
from PIL import Image


def decode_camera(camera):
    """Return uint8 RGB [H,W,3] and float32 axial depth [H,W] in metres."""
    if (camera['depth_units'], camera['depth_convention'], camera['depth_dtype']) != ('m', 'camera_z', '<f4'):
        raise ValueError('Expected metre-valued camera-z depth with little-endian float32 encoding')
    width, height = int(camera['width']), int(camera['height'])
    if width <= 0 or height <= 0:
        raise ValueError('Image dimensions must be positive')
    with Image.open(io.BytesIO(base64.b64decode(camera['rgb_png_base64'], validate=True))) as image:
        if image.format != 'PNG' or image.size != (width, height):
            raise ValueError('RGB PNG dimensions disagree with camera metadata')
        rgb = np.asarray(image.convert('RGB')).copy()
    raw = zlib.decompress(base64.b64decode(camera['depth_f32_zlib_base64'], validate=True))
    if len(raw) != width*height*4:
        raise ValueError('Depth byte count disagrees with camera dimensions')
    depth = np.frombuffer(raw, dtype='<f4').reshape(height, width).copy()
    return rgb, depth


def unproject(camera, pixels_uv, depth_m):
    """Map pixel centres and axial depths into the calibrated world frame."""
    pixels = np.asarray(pixels_uv, dtype=float)
    depth = np.asarray(depth_m, dtype=float)
    if pixels.shape[-1:] != (2,) or not np.isfinite(pixels).all():
        raise ValueError('Pixels must have finite [...,2] coordinates')
    if not np.isfinite(depth).all() or np.any(depth <= 0):
        raise ValueError('Projection needs finite positive depth')
    K = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    T = np.asarray(camera['T_world_camera'], dtype=float).reshape(4, 4)
    homogeneous = np.concatenate((pixels, np.ones(pixels.shape[:-1]+(1,))), axis=-1)
    rays = homogeneous@np.linalg.inv(K).T
    points = rays*(depth/rays[..., 2])[..., None]
    return points@T[:3, :3].T+T[:3, 3]


def project(camera, world_points):
    """Return pixel-centre coordinates and camera-z depths for world points."""
    points = np.asarray(world_points, dtype=float)
    if points.shape[-1:] != (3,) or not np.isfinite(points).all():
        raise ValueError('Points must have finite [...,3] coordinates')
    K = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    T = np.asarray(camera['T_world_camera'], dtype=float).reshape(4, 4)
    local = (points-T[:3, 3])@T[:3, :3]
    if np.any(local[..., 2] <= 0):
        raise ValueError('Points must be in front of the camera')
    pixels = local@K.T
    return pixels[..., :2]/pixels[..., 2, None], local[..., 2]


def pose_matrix(pose):
    w, x, y, z = np.asarray(pose['quaternion_wxyz'], dtype=float)/np.linalg.norm(pose['quaternion_wxyz'])
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = pose['position']
    return result


def forward_kinematics(joints, arm, robot=None):
    """Return the grasp TCP's world transform from six measured arm angles."""
    if robot is None:
        robot = json.loads(Path(__file__).with_name('robot_kinematics.json').read_text())
    q = np.asarray(joints, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('Expected six finite arm joint angles')
    if arm not in robot['bases']:
        raise ValueError('Unknown arm')
    transform = pose_matrix(robot['bases'][arm])
    for angle, link in zip(q, robot['links']):
        transform = transform@pose_matrix(link['fixed_pose'])
        axis = np.asarray(link['axis'], dtype=float)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        rotation = np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross)
        pivot = np.asarray(link['pivot'], dtype=float)
        joint = np.eye(4)
        joint[:3, :3] = rotation
        joint[:3, 3] = pivot-rotation@pivot
        transform = transform@joint
    return transform@pose_matrix(robot['tcp'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sample', type=Path, help='Recorded observation JSON, e.g. ~/sensor_sample.json')
    parser.add_argument('--output', type=Path, default=Path('sensor_views'))
    args = parser.parse_args()
    sample = json.loads(args.sample.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = {}
    for name, camera in sample['cameras'].items():
        if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
            raise ValueError('Camera name is not a safe filename')
        rgb, depth = decode_camera(camera)
        Image.fromarray(rgb).save(args.output/(name+'_rgb.png'))
        np.save(args.output/(name+'_depth.npy'), depth)
        valid = np.isfinite(depth)&(depth > 0)
        preview = np.zeros(depth.shape, dtype=np.uint8)
        near, far = (float(x) for x in np.percentile(depth[valid], [1, 99])) if valid.any() else (0., 1.)
        preview[valid] = np.round(255*(1-np.clip((depth[valid]-near)/max(far-near, 1e-6), 0, 1))).astype(np.uint8)
        Image.fromarray(preview).save(args.output/(name+'_depth_preview.png'))
        metadata[name] = dict(K=camera['K'], T_world_camera=camera['T_world_camera'], depth_preview_range_m=[near, far])
    (args.output/'camera_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps({'output': str(args.output.resolve()), 'cameras': list(metadata)}))


if __name__ == '__main__':
    main()
