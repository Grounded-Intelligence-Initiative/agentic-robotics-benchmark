"""Motion and perception primitives of the reference controller.

A behaviour is a Python generator: it yields an action (10 numbers) and is sent the next
observation. `Robot` keeps the latest observation and the targets it last commanded.
"""

import math
import os
import sys

import mujoco
import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree

import remote_env as E
from kin import Kin, rot_from_axes

DOWN_Y = rot_from_axes((0, 0, -1), (0, 1, 0))      # hand pointing down, fingers closing along y
DOWN_X = rot_from_axes((0, 0, -1), (1, 0, 0))
FWD_V = rot_from_axes((1, 0, 0), (0, 0, 1))        # hand pointing at the sofa, fingers closing vertically
FWD_H = rot_from_axes((1, 0, 0), (0, 1, 0))


def down(yaw):
    """Hand pointing down with the closing direction at `yaw` in the floor plane."""
    return rot_from_axes((0, 0, -1), (math.cos(yaw), math.sin(yaw), 0.0))


class Locator(object):
    """Finds where the settee and the chest stand from one overview depth image.

    Analysis by synthesis on a private copy of the simulator: the furniture of the copy is put
    at the present estimate, the overview camera's depth image of the copy is rendered, and the
    points it shows of a piece are matched to the nearest measured points; the planar rigid
    motion that brings the matched pairs together corrects the estimate."""

    PIECES = {"sofa": ("sofa", "pad_left", "pad_right"),
              "cabinet": ("cabinet", "drawer_upper", "drawer_mid", "drawer_lower")}

    def __init__(self):
        self.env = E.Env()
        env = self.env
        env.spec = E.make_scene(0, remote=E.OUT_OF_ROOM[0])
        env._apply_look(env.spec)
        self.renderer = env._renderer("overview")
        self.fx = E.camera_intrinsics("overview")
        self.cam = overview_pose()
        m = env.model
        self.bodies = {piece: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in names]
                       for piece, names in self.PIECES.items()}

    def _points(self, depth_m, mask):
        fx, fy, cx, cy = self.fx
        v, u = np.nonzero(mask)
        z = depth_m[v, u]
        cam = np.stack([(u + 0.5 - cx) / fx * z, (v + 0.5 - cy) / fy * z, z, np.ones_like(z)], axis=1)
        return (cam @ self.cam.T)[:, :3]

    def _render(self, kin, q):
        """Depth (m) and the body of every pixel, with the copy's furniture where `kin` has it."""
        env = self.env
        m, d = env.model, env.data
        mujoco.mj_resetData(m, d)
        for name in ("sofa", "cabinet"):
            b = env.body[name]
            src = mujoco.mj_name2id(kin.model, mujoco.mjtObj.mjOBJ_BODY, name)
            m.body_pos[b] = kin.model.body_pos[src]
            m.body_quat[b] = kin.model.body_quat[src]
        for name in ("pad_left", "pad_right", "hat"):
            d.qpos[env.free_qadr[name]:env.free_qadr[name] + 7] = kin.model.qpos0[kin.free_adr[name]:kin.free_adr[name] + 7]
        for name in ("remote", "hanger"):
            d.qpos[env.free_qadr[name]:env.free_qadr[name] + 3] = (5.0, 5.0, 5.0)
        for adr, value in zip(env.arm_qadr, q):
            d.qpos[adr] = value
        mujoco.mj_forward(m, d)
        r = self.renderer
        r.enable_depth_rendering()
        r.update_scene(d, camera=env.cam_id["overview"], scene_option=env.scene_option)
        depth = r.render().copy()
        r.disable_depth_rendering()
        r.enable_segmentation_rendering()
        r.update_scene(d, camera=env.cam_id["overview"], scene_option=env.scene_option)
        seg = r.render().copy()
        r.disable_segmentation_rendering()
        geom = np.where(seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM), seg[:, :, 0], -1)
        body = np.where(geom >= 0, m.geom_bodyid[np.maximum(geom, 0)], -1)
        return depth, body

    def locate(self, kin, depth_mm, q, rounds=12):
        """(sofa pose, cabinet pose), each (dx, dy, turn) from the nominal layout. `kin` is left
        with the result as its layout."""
        measured = depth_mm.astype(float) / 1000.0
        scene = self._points(measured, (depth_mm > 0) & (measured < 4.0))
        scene = scene[(scene[:, 2] > 0.03) & (scene[:, 0] > 0.20)]
        tree = cKDTree(scene)
        poses = {"sofa": np.zeros(3), "cabinet": np.zeros(3)}
        pivots = {"sofa": np.array(E.SOFA_PIVOT), "cabinet": np.array(E.CAB_PIVOT)}
        for i in range(rounds):
            kin.set_layout(poses["sofa"], poses["cabinet"])
            depth, body = self._render(kin, q)
            trim = max(0.015, 0.10 * 0.6 ** i)
            for piece, ids in self.bodies.items():
                model = self._points(depth, np.isin(body, ids))
                model = model[model[:, 2] > 0.03][::3]
                dist, idx = tree.query(model, distance_upper_bound=trim)
                ok = np.isfinite(dist)
                if ok.sum() < 200:
                    continue
                a, b = model[ok][:, :2], scene[idx[ok]][:, :2]
                ca, cb = a.mean(axis=0), b.mean(axis=0)
                h = (a - ca).T @ (b - cb)
                turn = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
                c, s = math.cos(turn), math.sin(turn)
                rot = np.array([[c, -s], [s, c]])
                # The corrected placement of the pivot, and the corrected turn.
                pose = poses[piece]
                at = pivots[piece] + pose[:2]
                at = rot @ (at - ca) + cb
                poses[piece] = np.array([at[0] - pivots[piece][0], at[1] - pivots[piece][1], pose[2] + turn])
        # Matching nearest points settles the distance and the turn of a front face well, but
        # it lets a piece slide along that face. So finish in the image: step each number of
        # each pose to where the rendered depth image differs least from the measured one.
        valid = depth_mm > 0

        def cost():
            kin.set_layout(poses["sofa"], poses["cabinet"])
            depth, _ = self._render(kin, q)
            return float(np.minimum(np.abs(depth - measured), 0.05)[valid].mean())

        for scale in (1.0, 0.4, 0.15):
            for piece in ("sofa", "cabinet"):
                for k, step in ((1, 0.010), (0, 0.004), (2, 0.008), (1, 0.010)):
                    step *= scale
                    centre = poses[piece][k]
                    costs = []
                    for j in (-2, -1, 0, 1, 2):
                        poses[piece][k] = centre + j * step
                        costs.append(cost())
                    j = int(np.argmin(costs))
                    best = float(j - 2)
                    if 0 < j < 4:
                        curve = costs[j - 1] - 2.0 * costs[j] + costs[j + 1]
                        if curve > 1e-12:
                            best += 0.5 * (costs[j - 1] - costs[j + 1]) / curve
                    poses[piece][k] = centre + best * step
        kin.set_layout(poses["sofa"], poses["cabinet"])
        return poses["sofa"], poses["cabinet"]


class Robot(object):
    def __init__(self):
        self.kin = Kin()
        self.kin.set_layout((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.locator = Locator()
        self.obs = None
        self.q_cmd = np.array((E.RAIL_HOME,) + E.ARM_HOME)
        self.grip_cmd = E.GRIP_RANGE[1]
        self.fx = {n: E.camera_intrinsics(n) for n in E.CAMERAS}
        self.via = None

    # -- the wire ---------------------------------------------------------------------------
    def action(self, cams=0):
        return [float(v) for v in self.q_cmd] + [float(self.grip_cmd), int(cams)]

    def tick(self, cams=0):
        self.obs = yield self.action(cams)

    def q(self):
        return np.array(self.obs["q"])

    # -- motion -----------------------------------------------------------------------------
    def wait(self, steps):
        for _ in range(steps):
            yield from self.tick()

    def settle(self, tol=0.01, max_steps=80):
        for _ in range(max_steps):
            yield from self.tick()
            if np.abs(self.q() - self.q_cmd).max() < tol and np.abs(self.obs["dq"]).max() < 0.05:
                return

    def move_q(self, goal, speed=0.6, settle=True):
        """Joint-space move: all axes arrive together; `speed` scales the axis speed limits."""
        goal = np.asarray(goal, float)
        start = self.q_cmd.copy()
        limits = np.array(E.MAX_SPEED) * speed * E.CONTROL_DT
        n = max(1, int(math.ceil(np.max(np.abs(goal - start) / limits))))
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)
            self.q_cmd = start + s * (goal - start)
            yield from self.tick()
        if settle:
            yield from self.settle()

    def solve(self, pos, rot, rail=None, seed=None, posture=None):
        seed = self.q_cmd if seed is None else seed
        q, ep, er = self.kin.ik(pos, rot, seed, rail=rail, posture=posture)
        return q, (ep < 2e-3 and er < 2e-2)

    def path_free(self, q0, q1, samples=24, clearance=0.02):
        """No collision along the joint-space segment; all but its last fifth keeps `clearance`."""
        for i in range(1, samples + 1):
            s = i / samples
            if self.kin.collides(q0 + (q1 - q0) * s, self.grip_cmd, clearance if s < 0.8 else 0.0):
                return False
        return True

    def rail(self, y):
        """The rail position that is `y` in the frame in use (rail positions are the world's)."""
        w = self.kin.to_world((0.0, y, 0.0))[1]
        return float(min(max(w, E.RAIL_RANGE[0] + 0.01), E.RAIL_RANGE[1] - 0.01))

    def home_at(self, rail):
        return np.concatenate([[rail], self.kin.home[1:]])

    def best_q(self, pos, rot, rails, sym=True, direct=False, then=None):
        """A collision-free IK solution of the hand pose, with a collision-free joint-space
        path to it (from the home posture at the same rail position, or with `direct` from the
        present posture), and the largest margin to the joint limits; over several seeds, both
        wrist turns (when the grasp is symmetric) and the rail positions in `rails`.
        Returns (q, found)."""
        rots = [np.asarray(rot)]
        if sym:
            rots.append(np.asarray(rot) @ np.diag([-1.0, -1.0, 1.0]))
        home = self.kin.home
        seeds = [self.q_cmd, home]
        for j1 in (-1.2, -0.6, 0.6, 1.2):
            for j7 in (0.785, -0.785):
                seed = home.copy()
                seed[1] = j1
                seed[7] = j7
                seeds.append(seed)
        fallback = None
        for rl in rails:
            if not direct and not self.path_free(self.home_at(self.q_cmd[0]), self.home_at(rl),
                                                 samples=2 + int(abs(rl - self.q_cmd[0]) / 0.04), clearance=0.0):
                continue
            found = []
            for r in rots:
                for seed in seeds:
                    q, ep, er = self.kin.ik(pos, r, seed, rail=rl)
                    if ep > 2e-3 or er > 2e-2:
                        continue
                    if any(np.abs(q - f[1]).max() < 0.05 for f in found):
                        continue
                    margin = float(np.min(np.minimum(q[1:] - self.kin.lo[1:], self.kin.hi[1:] - q[1:])))
                    found.append((margin, q))
            found.sort(key=lambda f: -f[0])
            if found and fallback is None:
                fallback = found[0][1]
            for margin, q in found:
                if margin < 0.05 or self.kin.collides(q, self.grip_cmd):
                    continue
                if then is not None:
                    # The pose is a pre-pose: the hand goes on by `then` in a straight line.
                    q2, ep, er = self.kin.ik(np.asarray(pos) + then, None, q, rail=q[0], posture=q)
                    if ep > 2e-3 or er > 2e-2 or self.kin.collides(q2, self.grip_cmd):
                        continue
                start = self.q_cmd if direct else self.home_at(q[0])
                if self.path_free(start, q):
                    self.via = None
                    return q, True
                # By way of a pose drawn back from the goal.
                for offset in ((0.0, 0.0, 0.25), (-0.10, 0.0, 0.25), (-0.15, 0.0, 0.10), (-0.10, 0.0, 0.40)):
                    qv, ep, er = self.kin.ik(np.asarray(pos) + offset, None, q, rail=q[0], posture=q)
                    if ep > 2e-3 or er > 2e-2 or self.kin.collides(qv, self.grip_cmd):
                        continue
                    if self.path_free(start, qv) and self.path_free(qv, q):
                        self.via = qv
                        return q, True
        if os.environ.get("REF_DEBUG"):
            print("   best_q: no free solution for", np.round(pos, 3), "rails", np.round(rails, 2), file=sys.stderr)
            for rl in rails:
                for r in rots:
                    q, ep, er = self.kin.ik(pos, r, home, rail=rl)
                    print("     rail %.2f ik %.4f %.3f goal-collides %s home-slide-free %s" % (
                        rl, ep, er, self.kin.why(q, self.grip_cmd),
                        self.path_free(self.home_at(self.q_cmd[0]), self.home_at(rl), clearance=0.0)), file=sys.stderr)
        found = [(0.0, fallback)] if fallback is not None else []
        if found:
            return found[0][1], False
        return self.q_cmd.copy(), False

    def go_home(self, rail=None, speed=1.0):
        """Tuck the arm into the home posture, then slide to `rail`."""
        here = self.home_at(self.q_cmd[0])
        if np.abs(self.q_cmd - here).max() > 1e-3:
            # Draw the hand back and up in straight lines until the way to the tuck is clear.
            for offset in ((-0.12, 0.0, 0.0), (0.0, 0.0, 0.25), (-0.08, 0.0, 0.20), (0.0, 0.0, 0.20)):
                if self.path_free(self.q_cmd, here, clearance=0.01):
                    break
                p, _ = self.kin.fk(self.q_cmd)
                yield from self.move_line(p + np.array(offset), step=0.01, settle=False)
            yield from self.move_q(here, speed=speed, settle=False)
        if rail is not None and abs(rail - self.q_cmd[0]) > 1e-3:
            yield from self.move_q(self.home_at(rail), speed=speed, settle=False)

    def move_pose(self, pos, rot, rails=None, speed=0.7, sym=True, direct=False, then=None):
        """Move to a hand pose through joint space: directly, or by way of the home posture.
        `then`: a displacement the hand will make next in a straight line; the posture is
        chosen so that the end of that move is free of collisions too."""
        rails = [self.q_cmd[0]] if rails is None else [self.rail(r) for r in rails]
        q, ok = self.best_q(pos, rot, rails, sym=sym, direct=direct, then=None if then is None else np.asarray(then, float))
        via = self.via if ok else None
        if not direct:
            yield from self.go_home(q[0])
        if via is not None:
            yield from self.move_q(via, speed=speed, settle=False)
        yield from self.move_q(q, speed=speed)
        return ok

    def move_line(self, pos, rot=None, step=0.008, settle=True, rail_to=None):
        """Straight-line move of the grip site from its commanded pose to `pos` (and `rot`)."""
        p0, r0 = self.kin.fk(self.q_cmd)
        rot = r0 if rot is None else np.asarray(rot)
        pos = np.asarray(pos, float)
        rail0 = self.q_cmd[0]
        n = max(1, int(math.ceil(np.linalg.norm(pos - p0) / step)))
        if rot is not r0:
            import mujoco
            q0 = np.empty(4); q1 = np.empty(4)
            mujoco.mju_mat2Quat(q0, r0.reshape(-1)); mujoco.mju_mat2Quat(q1, rot.reshape(-1))
            if q0 @ q1 < 0:
                q1 = -q1
            ang = 2 * math.acos(min(1.0, abs(float(q0 @ q1))))
            n = max(n, int(math.ceil(ang / 0.04)))
        for i in range(1, n + 1):
            s = i / n
            p = p0 + s * (pos - p0)
            if rot is r0:
                r = r0
            else:
                qq = (1 - s) * q0 + s * q1
                qq /= np.linalg.norm(qq)
                r = np.empty(9)
                mujoco.mju_quat2Mat(r, qq)
                r = r.reshape(3, 3)
            rail = rail0 if rail_to is None else rail0 + s * (rail_to - rail0)
            q, _, _ = self.kin.ik(p, r, self.q_cmd, rail=rail, iters=30, posture=self.q_cmd)
            self.q_cmd = q
            yield from self.tick()
        if settle:
            yield from self.settle()

    def grip(self, width, steps=14):
        self.grip_cmd = width
        yield from self.wait(steps)

    # -- perception -------------------------------------------------------------------------
    def look(self, cams):
        """Ask for the cameras in the mask `cams`; returns {name: (rgb, depth_mm, T_world_cam)}."""
        yield from self.tick(cams)
        out = {}
        for name, item in self.obs["images"].items():
            pose = np.array(self.obs["T_world_wrist"]) if name == "wrist" else overview_pose()
            out[name] = (E.decode_image(item["rgb"]), E.decode_image(item["depth"]), pose)
        return out

    def points(self, name, depth, pose, mask):
        """Points of the masked pixels of a depth image, in the frame in use."""
        fx, fy, cx, cy = self.fx[name]
        v, u = np.nonzero(mask & (depth > 0))
        z = depth[v, u].astype(float) / 1000.0
        cam = np.stack([(u + 0.5 - cx) / fx * z, (v + 0.5 - cy) / fy * z, z, np.ones_like(z)], axis=1)
        return self.kin.to_frame((cam @ pose.T)[:, :3])


_OVERVIEW = None


def overview_pose():
    global _OVERVIEW
    if _OVERVIEW is None:
        pos = np.array(E.OVERVIEW_POS)
        f = np.array(E.OVERVIEW_LOOKAT) - pos
        f /= np.linalg.norm(f)
        r = np.cross(f, (0, 0, 1.0))
        r /= np.linalg.norm(r)
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack([r, np.cross(f, r), f])
        pose[:3, 3] = pos
        _OVERVIEW = pose
    return _OVERVIEW


def find_remote(points_rgb, z_range, xy_box=None, min_points=40, dark=False, sized=True):
    """The remote among points of the frame in use: a slab of points in a height band above its
    surface (with `dark`, only its dark ones, for a far view whose depth is too coarse to tell
    the slab). The points are split into separate things, and a thing counts as the remote only
    if it has the remote's size: the phone is wider, the glasses case wider and taller.
    Returns (centre xy, yaw of its long axis, number of points) or None."""
    pts, rgb = points_rgb
    top = z_range[1] if (dark or not sized) else z_range[0] + 0.06
    keep = (pts[:, 2] > z_range[0]) & (pts[:, 2] < top)
    if xy_box is not None:
        (x0, x1), (y0, y1) = xy_box
        keep &= (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    if dark:
        keep &= rgb.max(axis=1) < 60
    sel = pts[keep]
    if len(sel) < min_points:
        return None
    # Things: connected groups of occupied cells (6 mm; 12 mm for a far view).
    cell = 0.012 if dark else 0.006
    ij = np.floor((sel[:, :2] - sel[:, :2].min(axis=0)) / cell).astype(int) + 1
    grid = np.zeros(ij.max(axis=0) + 2, dtype=bool)
    grid[ij[:, 0], ij[:, 1]] = True
    labels, count = label(grid, structure=np.ones((3, 3)))
    of = labels[ij[:, 0], ij[:, 1]]
    best = None
    for k in range(1, count + 1):
        thing = sel[of == k]
        if len(thing) < min_points:
            continue
        xy = thing[:, :2]
        centre = xy.mean(axis=0)
        w, v = np.linalg.eigh(np.cov((xy - centre).T))
        axis, side = v[:, 1], v[:, 0]
        along, across = (xy - centre) @ axis, (xy - centre) @ side
        a0, a1 = np.percentile(along, 2), np.percentile(along, 98)
        c0, c1 = np.percentile(across, 2), np.percentile(across, 98)
        length, width = a1 - a0, c1 - c0
        if dark:
            # A far view under a lifted pad: part of the remote may be hidden.
            fits = 0.05 < length < 0.21 and width < 0.058
        else:
            fits = 0.12 < length < 0.21 and 0.02 < width < 0.058 and np.percentile(thing[:, 2], 90) < z_range[1]
        if os.environ.get('REF_DEBUG'):
            print('   thing', len(thing), 'length %.3f width %.3f' % (length, width), 'fits', fits, file=sys.stderr)
        if sized and not fits:
            continue
        if best is None or len(thing) > best[2]:
            best = (centre + axis * 0.5 * (a0 + a1) + side * 0.5 * (c0 + c1), math.atan2(axis[1], axis[0]), len(thing))
    return best
