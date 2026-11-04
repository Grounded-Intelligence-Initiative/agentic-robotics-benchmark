"""Rasterize MuJoCo finger-pad contact forces in the wrist camera image frame.

This is a camera-aligned display of physical contact points and normal forces,
not an optical-flow or learned pseudo-tactile signal.
"""
import numpy as np
import mujoco
from PIL import Image


def _project_world_point(sim, world_point, height, width):
    """Use the same pinhole convention as robosuite's camera_utils."""
    cam_id = sim.model.camera_name2id("robot0_eye_in_hand")
    origin = np.asarray(sim.data.cam_xpos[cam_id], dtype=float)
    rotation = np.asarray(sim.data.cam_xmat[cam_id], dtype=float).reshape(3, 3)
    camera_rotation = rotation @ np.diag([1.0, -1.0, -1.0])
    local = camera_rotation.T @ (np.asarray(world_point, dtype=float) - origin)
    if local[2] <= 1e-6:
        return None
    fovy = float(sim.model.cam_fovy[cam_id])
    focal = 0.5 * height / np.tan(fovy * np.pi / 360)
    return focal * local[0] / local[2] + width / 2, focal * local[1] / local[2] + height / 2


def _pad_contact_names(env, contact):
    model = env.sim.model
    a = model.geom_id2name(int(contact.geom1)) or ""
    b = model.geom_id2name(int(contact.geom2)) or ""
    return a, b


def contact_heatmap(env, height=128, width=128):
    """Return uint8 force heatmap and count of finger-pad contacts."""
    sim = env.sim
    heat = np.zeros((height, width), dtype=np.float32)
    yy, xx = np.mgrid[:height, :width]
    count = 0
    for idx in range(sim.data.ncon):
        contact = sim.data.contact[idx]
        a, b = _pad_contact_names(env, contact)
        if not ("pad_collision" in a or "pad_collision" in b):
            continue
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(sim.model._model, sim.data._data, idx, force)
        normal_newtons = max(0.0, float(force[0]))
        if normal_newtons <= 0:
            continue
        projected = _project_world_point(sim, contact.pos, height, width)
        if projected is None:
            continue
        x, y = projected
        if not (-6 <= x < width + 6 and -6 <= y < height + 6):
            continue
        amplitude = min(255.0, 75.0 * np.log1p(normal_newtons))
        heat += amplitude * np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * 3.0 ** 2))
        count += 1
    return np.clip(heat, 0, 255).astype(np.uint8), count


def render_contact_overlay(wrist_rgb, heatmap):
    """Return RGB image with cyan contact highlights aligned to wrist RGB."""
    frame = np.asarray(wrist_rgb, dtype=np.uint8).copy()
    if frame.shape[:2] != heatmap.shape:
        raise ValueError("wrist frame and tactile heatmap dimensions differ")
    alpha = (heatmap.astype(np.float32) / 255.0 * .8)[..., None]
    highlight = np.zeros_like(frame)
    highlight[..., 1] = 250
    highlight[..., 2] = 255
    frame = np.clip(frame * (1 - alpha) + highlight * alpha, 0, 255).astype(np.uint8)
    return Image.fromarray(frame)

