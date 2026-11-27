"""Estimate the G1 pelvis and wrist poses from metric depth and measured joints.

Each camera measures its pose using the visible tabletop plane and edges. The
only orientation prior selects a side of the known rectangle. A kinematic copy of
the public robot supplies surface points. Registration uses each live depth
image and never opens a scene, queries the running simulator, or integrates it.
"""
from pathlib import Path

import mujoco
import numpy as np
from scipy.ndimage import binary_erosion
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


# Coarse viewing directions disambiguate the two long edges of a rectangle.
# The metric camera rotation and translation are estimated from the imaged table.
CAMERA_HEADING = {"overview": 120., "hand": 105., "side": 180.}
TABLE_BOUNDS = np.array([[.202,.712],[-.530,.190]])


def _locate_edges(axis, border, size, margin):
    low, high = np.quantile(axis, [.001,.999])
    candidates = [low-margin, high-size+margin, (low+high-size)/2]
    def loss(left):
        distance = np.minimum(abs(border-left), abs(border-left-size))
        edge_gain = np.maximum(0.,1.-distance/.010).sum()
        spill = np.maximum(np.maximum(left-axis,axis-left-size)-.006,0.)
        return edge_gain - 60.*spill.sum()
    scores = [loss(c) for c in candidates]
    i = int(np.argmax(scores))
    return scores[i], candidates[i]


def _table_pose(name, view, pinhole):
    rgb = view["rgb"].astype(float)
    r, g, b = rgb.transpose(2,0,1)
    wood = (r>1.18*g)&(r<1.65*g)&(b>.46*g)&(b<.83*g)&(g>60)
    depth = np.asarray(view["depth_mm"],dtype=float)*.001
    xyz = pinhole * depth[...,None]
    dense = xyz[wood & (depth>.08) & (depth<5.)]
    if len(dense)<100:
        raise ValueError("the visible brown station surface is too small to calibrate "+name)
    p=dense[np.linspace(0,len(dense)-1,min(2500,len(dense)),dtype=int)]
    # Majority-plane fit separates the horizontal top from vertical brown side faces.
    rng=np.random.default_rng(270)
    three=p[rng.integers(len(p),size=(42,3))]
    ns=np.cross(three[:,1]-three[:,0],three[:,2]-three[:,0])
    ns/=np.maximum(np.linalg.norm(ns,axis=1,keepdims=True),1e-12)
    scores=np.abs(p@ns.T-np.sum(three[:,0]*ns,axis=1))
    j=np.argmax(np.sum(scores<.0025,axis=0))
    plane=p[scores[:,j]<.0035]
    center=plane.mean(axis=0)
    q=plane-center
    normal=np.linalg.eigh(q.T@q)[1][:,0]
    if normal@center>0:normal=-normal
    gap=xyz@normal-center@normal
    mask=wood & (abs(gap)<.004)
    edge=mask & ~binary_erosion(mask)
    edge[:2]=edge[-2:]=False
    edge[:,:2]=edge[:,-2:]=False
    x0=np.array([1.,0.,0.])-normal[0]*normal
    x0/=np.linalg.norm(x0)
    y0=np.cross(normal,x0)
    plane2=np.array([x0,y0]).T
    pts=xyz[mask]@plane2
    # Ignore isolated anti-aliased pixels outside the board's main component.
    mid=np.median(pts,axis=0)
    pts=pts[np.linalg.norm(pts-mid,axis=1)<.85]
    border=xyz[edge]@plane2
    border=border[np.linalg.norm(border-mid,axis=1)<.85]
    pts=pts[np.linspace(0,len(pts)-1,min(3200,len(pts)),dtype=int)]
    scale=min(.004,np.median(depth[mask])/view['intrinsics']['focal_px']*.36)
    guess=90.-CAMERA_HEADING[name]
    def orient(angle):
        theta=np.deg2rad(angle)
        ex=np.array([np.cos(theta),np.sin(theta)])
        ey=np.array([-ex[1],ex[0]])
        sx,lx=_locate_edges(pts@ex,border@ex,.510,scale)
        sy,ly=_locate_edges(pts@ey,border@ey,.720,scale)
        return sx+sy, np.array([ex,ey]), np.array([lx,ly])
    choices=np.arange(guess-24.,guess+24.01,1.)
    best=choices[np.argmax([orient(deg)[0] for deg in choices])]
    score,axes,limits=max((orient(deg) for deg in np.arange(best-1.,best+1.01,.1)),key=lambda c:c[0])
    global_axes=np.vstack([axes@plane2.T,normal])
    origin=np.r_[TABLE_BOUNDS[:,0]-limits,.730-center@normal]
    return origin,global_axes.T
LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
JOINTS = ([f"{s}_{j}_joint" for s in ("left", "right") for j in LEG]
          + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
          + [f"{s}_{j}_joint" for s in ("left", "right") for j in ARM])


def _sample_surfaces(model):
    """Uniform-by-area samples and surface normals from visible public meshes."""
    gen = np.random.default_rng(7124)
    points, normals, geom_ids, head = [], [], [], []
    for g in np.flatnonzero(model.geom_group == 2):
        mesh = model.geom_dataid[g]
        if mesh < 0 or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        name = model.mesh(mesh).name
        if "hand_" in name or "logo" in name:
            continue
        verts = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
        faces = model.mesh_face[model.mesh_faceadr[mesh]:model.mesh_faceadr[mesh]+model.mesh_facenum[mesh]]
        triangles = verts[faces].astype(np.float64)
        area_normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        weight = np.linalg.norm(area_normal, axis=1)
        total = weight.sum()
        count = max(45, min(2200, int(total * 6500)))
        selected = gen.choice(len(weight), count, p=weight/total)
        u, v = gen.random((2, count, 1))
        u = np.sqrt(u)
        tr = triangles[selected]
        points.append((1-u)*tr[:, 0]+u*(1-v)*tr[:, 1]+u*v*tr[:, 2])
        normals.append(area_normal[selected]/weight[selected, None])
        geom_ids.extend([g]*count)
        head.extend([name == "head_link"]*count)
    return np.concatenate(points), np.concatenate(normals), np.array(geom_ids), np.array(head)


class VisualBody:
    """Kinematic surface registration with a resettable optical camera calibration."""

    def __init__(self, assets_root):
        source = Path(assets_root) / "menagerie_g1" / "g1_with_hands.xml"
        self.model = mujoco.MjModel.from_xml_path(str(source))
        self.local = mujoco.MjData(self.model)
        self.qadr = [self.model.joint(n).qposadr[0] for n in JOINTS]
        self.vertices, self.normals, self.geoms, self.head = _sample_surfaces(self.model)
        self.rays = {}
        self.calibrations = {}

    def reset(self):
        self.calibrations.clear()

    def snapshot(self):
        return dict(self.calibrations)

    def restore(self, state):
        self.calibrations = {} if state is None else dict(state)

    def _observed_surface(self, name, view):
        depth = np.asarray(view["depth_mm"])
        h, w = depth.shape
        intr = view["intrinsics"]
        f = float(intr["focal_px"])
        cx, cy = map(float, intr["center_px"])
        key = name, h, w, f, cx, cy
        if key not in self.rays:
            y, x = np.mgrid[:h, :w]
            self.rays[key] = np.stack(((x-cx)/f,(y-cy)/f,np.ones_like(x)),axis=-1)
        if key not in self.calibrations:
            self.calibrations[key] = _table_pose(name,view,self.rays[key])
        origin,rot = self.calibrations[key]
        measured = depth[::2, ::2].astype(np.float64) * .001
        xyz = origin + (self.rays[key][::2, ::2] @ rot) * measured[...,None]
        x, y, z = xyz.transpose(2, 0, 1)
        # The floor, table and workpiece do not extend into this standing-body region.
        region = ((measured > .08) & (measured < 5.) & (z > .11) & (z < 1.8)
                  & (x > -1.55) & ((x < .185) | (z > 1.04)) & (x < .52) & (abs(y) < .55))
        return xyz[region]

    def estimate(self, proprio, views, tick):
        del tick
        down = np.array(proprio["gravity_body"], dtype=float, copy=True)
        down /= max(np.linalg.norm(down), 1e-9)
        body_up = -down
        cross = np.cross(body_up, [0.,0.,1.])
        cosine = np.clip(body_up[2], -1., 1.)
        tilt = Rotation.from_rotvec(cross * (np.arccos(cosine)/max(np.linalg.norm(cross),1e-9)))
        self.local.qpos[:3] = 0.
        self.local.qpos[3:7] = tilt.as_quat()[[3,0,1,2]]
        self.local.qpos[self.qadr] = proprio["q"]
        mujoco.mj_kinematics(self.model, self.local)
        rot = self.local.geom_xmat.reshape(-1,3,3)[self.geoms]
        predicted = np.einsum("nij,nj->ni",rot,self.vertices) + self.local.geom_xpos[self.geoms]
        normals = np.einsum("nij,nj->ni",rot,self.normals)
        tree = cKDTree(predicted)
        clouds = [self._observed_surface(name, views[name]) for name in CAMERA_HEADING]
        overview = clouds[0]
        if len(overview) < 10:
            raise ValueError("overview depth does not contain enough G1 surface samples")
        top = np.quantile(overview[:,2], .995)
        cap = overview[overview[:,2] > top-.065]
        template = predicted[self.head]
        highest = np.quantile(template[:,2],.98)
        template_cap = template[template[:,2] > highest-.065]
        pos = np.median(cap, axis=0)-np.median(template_cap, axis=0)
        measured = []
        for cloud in clouds:
            near = cloud[(abs(cloud[:,0]-pos[0])<.48) & (abs(cloud[:,1]-pos[1])<.50)]
            if len(near):
                measured.append(near[np.linspace(0,len(near)-1,min(700,len(near)),dtype=int)])
        observed = np.concatenate(measured)
        yaw = 0.
        for _ in range(22):
            spin = Rotation.from_rotvec([0.,0.,yaw])
            received = spin.inv().apply(observed-pos)
            distance, nearest = tree.query(received, distance_upper_bound=.085)
            keep = np.isfinite(distance)
            if keep.sum() < 20:
                break
            closest = spin.apply(predicted[nearest[keep]])
            n = spin.apply(normals[nearest[keep]])
            residual = np.sum(n*(observed[keep]-pos-closest),axis=1)
            # Point-to-plane registration ignores off-model points and discounts outliers.
            w = np.sqrt(np.minimum(1., .012/np.maximum(abs(residual),1e-8)))
            yaw_jac = n[:,1]*closest[:,0] - n[:,0]*closest[:,1]
            a = np.column_stack([n, yaw_jac])*w[:,None]
            b = residual*w
            delta = np.linalg.solve(a.T@a+np.diag([.01,.01,.01,.05]),a.T@b)
            pos += np.clip(delta[:3],-.045,.045)
            yaw += float(np.clip(delta[3],-.10,.10))
            if max(abs(delta)) < .0003:
                break
        spin = Rotation.from_rotvec([0.,0.,yaw])
        torso = self.local.body("torso_link")
        def quat(rotation):
            return rotation.as_quat()[[3,0,1,2]].tolist()
        result = {"base_pos":pos.tolist(), "base_quat":quat(spin*tilt),
                  "torso_pos":(pos+spin.apply(torso.xpos)).tolist(),
                  "torso_quat":quat(spin*Rotation.from_quat(torso.xquat[[1,2,3,0]]))}
        for side,y in (("left",-.048),("right",.048)):
            wrist = self.local.body(side+"_wrist_yaw_link")
            pocket = wrist.xpos + wrist.xmat.reshape(3,3) @ np.array([.115,y,0.])
            result[side+"_hand_pos"]=(pos+spin.apply(pocket)).tolist()
        return result
