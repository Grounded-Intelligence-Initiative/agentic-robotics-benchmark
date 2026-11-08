"""Privileged, physical xArm reference: observations may be privileged, motion is not.

Every move uses the public cell tick, bounded actuators and real pad contacts.
This module never writes live part poses, equality activation or coupling state.
"""
import math
import sys
import time
from dataclasses import dataclass

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

from motion import Motion, PlanningError, DOWN, body_subtree, axis_aligned_rotation


@dataclass
class Grasp:
    p: np.ndarray
    R: np.ndarray
    width: float
    label: str
    target_yaw: float = 0.
    squeeze: float = .006
    settle_s: float = .5


def _log(*args):
    print("[reference]", *args, file=sys.stderr, flush=True)


def _tip(width):
    a = Motion.angle_for_width(width)
    return .059098 + math.sin(a)*.035465 + math.cos(a)*.042039 + .0595


def surface_grasps(cell, name):
    """Antipodal candidates on the imported surface in the settled pick frame.

    A protruding cable or lens must not move a grasp to the bounding-box centre.
    Endpoint normals and finite jaw width are measured from actual mesh rays;
    the planner subsequently checks the complete stock gripper and held part.
    """
    import trimesh
    kind = cell.graph["instances"][name]["kind"]
    part = cell.kit["parts"][kind]
    mesh = trimesh.load_mesh(cell.assets / "given/kit/meshes" / part["mesh"])
    top = cell.pose(name)[:3, :3].T @ DOWN
    candidates = []
    # Collision decomposition can slightly enlarge a fin or connector. Measure
    # its actual envelope too, so a command based only on one source ray does
    # not drive the pad several millimetres into a neighbouring feature.
    proxy_pieces = [
        trimesh.load_mesh(cell.assets / "given/kit/collision" / fn)
        for fn in part["collision"]]
    # The motor's grasp surface is its passive bell child. Its aggregate
    # source mesh already includes the child at the nominal hinge offset;
    # the collision footprint must use that same physical assembly.
    if part.get("rotor"):
        rotor=part["rotor"]
        for fn in rotor["collision"]:
            piece=trimesh.load_mesh(cell.assets / "given/kit/collision" / fn)
            piece.apply_translation(rotor["hinge_pos_m"])
            proxy_pieces.append(piece)
    proxy = trimesh.util.concatenate(proxy_pieces)

    def footprint(contact, R):
        # Stock pad_2 is 30 x 19 mm. The candidate contact denotes its distal
        # tip, so its material extends back towards the wrist along -tool Z.
        # Keep the complete pad envelope, with finer central rays for a
        # small curved lens or bell whose tangent patch is only a few mm wide.
        offsets=np.array([[x,0.,z] for z in (0.,-.002,-.004,-.006,-.008,-.010,-.012,-.014,-.016,-.018)
                          for x in (-.014,-.010,-.005,-.004,-.002,0.,.002,.004,.005,.010,.014)])
        centers=contact+offsets@R.T
        origins=centers-R[:,1]*.15
        directions=np.tile(R[:,1],(len(origins),1))
        source_hits,source_rays,source_faces=mesh.ray.intersects_location(origins,directions,multiple_hits=True)
        proxy_hits,proxy_rays,_=proxy.ray.intersects_location(origins,directions,multiple_hits=True)
        if not len(proxy_hits):return None
        sides=[];normals=[]
        for k in range(len(origins)):
            use=np.flatnonzero(source_rays==k)
            if len(use)<2:continue
            use=use[np.argsort(source_hits[use]@R[:,1])]
            a,b=use[0],use[-1]
            sides.append([source_hits[a]@R[:,1],source_hits[b]@R[:,1]])
            normals.append([-mesh.face_normals[source_faces[a]]@R[:,1],
                            mesh.face_normals[source_faces[b]]@R[:,1]])
        if len(sides)<8:return None
        sides=np.asarray(sides);normals=np.asarray(normals)
        envelope=np.array([min(proxy_hits@R[:,1]),max(proxy_hits@R[:,1])])
        width=float(envelope[1]-envelope[0])
        if not .006<=width<=.078:return None
        # Require a finite patch near each supporting outside plane, rather
        # than one opposing normal at a small connector or corner.
        near=(np.abs(sides-envelope)<.0012)&(normals>math.cos(math.radians(30)))
        support=near.sum(0)
        # A cylindrical motor bell meets a flat pad along a narrow axial
        # strip, so four supported rays can describe a valid finite strip.
        if min(support)<(4 if part.get("rotor") else 6):return None
        contact=contact+R[:,1]*(envelope.mean()-contact@R[:,1])
        return contact,width,int(min(support))
    for yaw in (0., math.pi/2, math.pi, -math.pi/2):
        R = top @ Rotation.from_euler("z", yaw).as_matrix()
        points_in_hand = mesh.vertices @ R
        low, high = points_in_hand.min(0), points_in_hand.max(0)
        # Sample the upper half of the physical part, with several depths for
        # stepped housings. Source geometry determines available material.
        zs = low[2] + np.array([.2, .35, .5, .65, .70, .73, .8]) * (high[2]-low[2])
        xs = np.linspace(low[0]+.002, high[0]-.002, 15)
        grid = np.array([[x, low[1]-.01, z] for z in zs for x in xs])
        origins = grid @ R.T
        hits, rays, faces = mesh.ray.intersects_location(
            origins, np.tile(R[:, 1], (len(origins), 1)), multiple_hits=True)
        for i in range(len(grid)):
            use = np.flatnonzero(rays == i)
            if len(use) < 2:
                continue
            ordered = use[np.argsort(hits[use] @ R[:, 1])]
            # Outermost hits span the whole part on this ray: inner recesses
            # are not falsely treated as the two outside gripping surfaces.
            a, b = ordered[0], ordered[-1]
            width = float((hits[b]-hits[a]) @ R[:, 1])
            if not .006 <= width <= .075:
                continue
            normal_a, normal_b = mesh.face_normals[faces[a]], mesh.face_normals[faces[b]]
            limit = math.cos(math.radians(25))
            if normal_a @ R[:, 1] > -limit or normal_b @ R[:, 1] < limit:
                continue
            measured=footprint((hits[a]+hits[b])*.5,R)
            if measured is None:continue
            contact,width,support=measured
            p = contact - R[:, 2]*(_tip(width)-.158)
            # Prefer broad surfaces near the source's main solid instead of
            # thin cable segments. This is a candidate rank, not a pose write.
            pad_center=contact-R[:,2]*.0095
            delta=(pad_center-np.asarray(part.get("inertial",{}).get("center_m",mesh.center_mass)))@R
            # Keep the COM near the pad's long-axis centre. A wide patch at
            # the far end of a board can still roll out under its own weight.
            priority = 2.*abs(delta[0]) + .05*abs(delta[1]) + .5*abs(delta[2]) - .00003*support
            label = f"surface-{yaw:.2f}-{i}"
            candidates.append((priority, Grasp(p, R, width, label, squeeze=.004)))
    seen = set()
    for _, candidate in sorted(candidates, key=lambda x: x[0]):
        key = tuple(np.round(candidate.p/.003).astype(int)) + tuple(np.round(candidate.R[:, 1], 2))
        if key not in seen:
            seen.add(key)
            yield candidate


def _lidar_housing_width(proxy, R, center):
    """Measured jaw span across the LiDAR's upper housing, or None.

    Rays along the jaw axis cover pad_2 and pad_1 behind it. Every hit must lie
    on the round housing (<= 26.2 mm from the axis): the drafted base shoulder,
    its fillet and the connector/cable must not reach a pad.
    """
    tip=center+R[:,2]*.0095
    offs=np.array([[x,0.,z] for z in np.linspace(0.,-.037,15) for x in np.linspace(-.0145,.0145,11)])
    points=tip+offs@R.T
    hits,rays,_=proxy.ray.intersects_location(points-R[:,1]*.15,np.tile(R[:,1],(len(points),1)),multiple_hits=True)
    if not len(rays):
        return None
    span=(np.asarray(hits)-points[rays])@R[:,1]
    if np.max(np.abs(span))>.0262:
        return None
    pad2=offs[rays][:,2]>=-.019
    if not pad2.any() or min(span[pad2].max(),-span[pad2].min())<.0252:
        return None
    return float(span[pad2].max()-span[pad2].min())


def lidar_housing_grasps(cell, name):
    """LiDAR: jaws across the rotationally symmetric upper housing only.

    Pad contacts are single points per convex pair, with no torsional friction,
    so a held part turns about its jaw line until the COM hangs below it. On the
    base body the 2.5 deg side draft puts those contacts on the bottom band and
    the connector keeps them >= 7 mm off the COM plane: that grasp hangs tilted
    over the cap (measured 27 deg after lift-off). Across the round housing the
    pads touch its y=0 tangent lines above the COM, so the hanging equilibrium is
    the upright mounting attitude. Bench to cap is a 90 deg turn about this same
    jaw axis. Top-down approaches come first: with the pad's long axis along the
    housing generator the sensor creeps least in the hand (~13 deg after
    lift-off, versus ~60 deg for a 30 deg lean towards the dome, the fallback).
    """
    import trimesh
    part=cell.kit["parts"]["lidar"]
    proxy=trimesh.util.concatenate([trimesh.load_mesh(cell.assets/"given/kit/collision"/fn)
                                    for fn in part["collision"]])
    down=cell.pose(name)[:3,:3].T@np.array([0.,0.,-1.])
    for theta in LIDAR_APPROACH_TILTS_DEG:
        for flip in (1.,-1.):
            jaw=np.array([flip,0.,0.])
            d=down-(down@jaw)*jaw;d/=np.linalg.norm(d)
            e=np.array([0.,0.,-1.]);e-=(e@jaw)*jaw+(e@d)*d;e/=np.linalg.norm(e)
            th=math.radians(theta)
            approach=math.cos(th)*d+math.sin(th)*e
            R=np.column_stack([np.cross(jaw,approach),jaw,approach])
            for yc in (0.,.002,-.002,.004):
                for zc in (.046,.044,.048):
                    center=np.array([0.,yc,zc])
                    width=_lidar_housing_width(proxy,R,center)
                    if width is None:
                        continue
                    tool=center+approach*.0095-approach*(_tip(width)-.158)
                    yield Grasp(tool,R,width,f"housing-th{theta:.0f}-y{yc*1000:.0f}-z{zc*1000:.0f}-{'+' if flip>0 else '-'}x",
                                squeeze=.004,settle_s=.5)


LIDAR_APPROACH_TILTS_DEG=(0.,15.,30.)


def _lidar_align(motion, pos, R, body, *, tolerance, max_passes):
    """Measured-pose alignment of the held LiDAR; its jaw-line tilt is gravity's.

    The two housing contacts leave the sensor free to turn about the jaw line,
    where it hangs ~0.5-1.7 deg from upright. Turning it with the hand only
    winds the hand around the hanging sensor, which creeps back within a second
    (measured: +0.5 deg in-hand per pass, never reaching the generic 0.57 deg;
    a corrected 0.18 deg crept back to 0.9 deg and shifted the origin 0.5 mm).
    Translate it to the target with that hanging tilt instead, and turn it
    back only beyond 2 deg. The mate accepts 3 deg.
    """
    motion.hold_body(body)
    b=motion.held[0]
    limit=math.radians(2.)
    for _ in range(max_passes):
        p_now,r_now=motion.body_pose(b)
        ep=float(np.linalg.norm(np.asarray(pos)-p_now))
        er=float(Rotation.from_matrix(np.asarray(R)@r_now.T).magnitude())
        if er>limit:
            desired=np.asarray(R)
        elif ep<=tolerance:
            return {"position_error_m":ep,"angle_error_rad":er}
        else:
            desired=r_now
        p,r=motion.body_goal(pos,desired,b)
        motion.line(p,r,speed=.01,clearance=False,allow_pad_body=b)
    p_now,r_now=motion.body_pose(b)
    ep=float(np.linalg.norm(np.asarray(pos)-p_now))
    er=float(Rotation.from_matrix(np.asarray(R)@r_now.T).magnitude())
    if ep>2*tolerance or er>math.radians(2.5):
        raise PlanningError(f"lidar alignment did not converge: {ep*1000:.3f} mm, {math.degrees(er):.3f} deg")
    _log("lidar alignment residual accepted",round(ep*1000,3),"mm",round(math.degrees(er),3),"deg")
    return {"position_error_m":ep,"angle_error_rad":er}
# Pad centres along the cheek's upper arm (part x), nearest the spine first
# that still leaves the inner finger plate (x +-16 mm) clear of it.
CHEEK_RIM_CENTERS_M = (-.012, -.0125, -.0115, -.013, -.011)


def cheek_rim_grasps(cell, name):
    """Sensor cheek (flat 4 mm C plate on bench pins): jaws across its upper arm.

    The jaws close across the upper arm (part y) from above, with the inner
    finger in the C opening, which keeps the wrist above the installed guard.
    The arm's straight rim (x -18..+5.6 mm) is shorter than a stock pad (30 mm)
    and its finger plate (32 mm): at the free end a hook prong stands 1.9 mm
    proud of the inner edge, at the other end the spine closes the opening.
    The inner finger plate must clear the spine, so the pad spans the prong;
    the jaw gap is the collision proxy's span over the pad footprint (outer rim
    to the prong tip), i.e. the gap the closing pads actually stop at. (The
    earlier fixed 7.75 mm straight-rim gap at x -10 put the inner pad 1.9 mm
    inside the prong and the finger plate's side 0.2 mm inside the spine.)
    place()'s closed-jaw finger check (_grip_clear) takes the first clear one.
    Known limit (physics, not planning): the prong's collision piece face is
    tilted 4.6 deg, so the squeeze pushes the cheek ~1 N along the pads and it
    creeps until the pad end meets the spine corner, where the finger plate,
    1 mm wider than the pad, sits 1 mm inside the spine during the carry.
    """
    import trimesh
    part = cell.kit["parts"][cell.graph["instances"][name]["kind"]]
    proxy = trimesh.util.concatenate([trimesh.load_mesh(cell.assets/"given/kit/collision"/fn)
                                      for fn in part["collision"]])
    for cx in CHEEK_RIM_CENTERS_M:
        rays = np.array([[cx+x, 0., z] for x in np.linspace(-.015, .015, 31) for z in (.0005, .002, .0035)])
        inner, _, _ = proxy.ray.intersects_location(rays+[0., .008, 0.], np.tile([0., 1., 0.], (len(rays), 1)))
        outer, _, _ = proxy.ray.intersects_location(rays+[0., .080, 0.], np.tile([0., -1., 0.], (len(rays), 1)))
        if not len(inner) or not len(outer):
            continue
        lo, hi = float(inner[:, 1].min()), float(outer[:, 1].max())
        if not .006 <= hi-lo <= .015:
            continue
        for yaw in (0., math.pi):
            R = Rotation.from_euler("z", yaw).as_matrix() @ DOWN
            yield Grasp(np.array([cx, (lo+hi)/2, .012359397152226951]), R, hi-lo, f"rim-x{1000*cx:.1f}-{yaw:.2f}")


def camera_grasps(cell, name):
    """Axial pinch for the board camera (camera only): board front <-> lens front.

    The jaws close along the lens axis (part x) on the 38 mm board's front
    face and on the lens front, with the hand running down the board plane
    (approach = part -z), so the hand is exactly DOWN at the mount and the
    camera's COM lies in the jaw axis' vertical plane: gravity has no moment
    about the jaw axis, and the 24 mm span resists the other two axes.

    Pinches across the lens axis are not used. With the stock pads (condim 3,
    no torsional friction) MuJoCo's soft friction lets a held part creep about
    the jaw axis when gravity has a moment about it: the 17.6 mm lens ring
    (one collision piece per pad, 3.4 mm contact) turned freely (88 deg in
    transport) and the board's 2 mm side edges crept 1.2-1.5 deg/s at 18-76 N
    at the mount orientation. The axial pinch needs the board
    front and the lens free: it is only yielded when the bench pose leaves
    both above the table (the camera leaning on its board edge and lens rim),
    not for the kit's board-down rest.
    """
    import trimesh
    part = cell.kit["parts"][cell.graph["instances"][name]["kind"]]
    proxy = trimesh.util.concatenate([
        trimesh.load_mesh(cell.assets/"given/kit/collision"/fn) for fn in part["collision"]])
    T = cell.pose(name)
    approach = np.array([0., 0., -1.])
    if (T[:3, :3] @ approach)[2] > -math.cos(math.radians(45)):
        _log("camera: no bench pose access for the axial pinch (board front/lens on the table); no grasp")
        return
    s_grid = np.linspace(-.0165, .0165, 12)
    t_grid = np.linspace(-.0229, .0171, 15)
    pad = np.array([[s_, 0., t_] for t_ in t_grid for s_ in s_grid])
    candidates = []
    for sign in (1., -1.):
        jaw = np.array([sign, 0., 0.])
        R = np.column_stack([np.cross(jaw, approach), jaw, approach])
        for z_tip in (.016, .018, .020, .022):
            width0 = .0244
            p = np.array([0., 0., z_tip])-approach*(_tip(width0)-.158)
            # Both pads' lowest corners stay 2 mm above the table plane (z = 0).
            corners = np.array([p+R @ np.array([s, y, t]) for s in (-.015, .015)
                                for y in (-.017, .017) for t in (-.0214, .0156)])
            lowest = float(((corners @ T[:3, :3].T)[:, 2]).min()+T[2, 3])
            if lowest < .002:
                continue
            origins = p+pad @ R.T-jaw*.05
            hits, _, _ = proxy.ray.intersects_location(
                origins, np.tile(jaw, (len(origins), 1)), multiple_hits=True)
            if not len(hits):
                continue
            along = (hits-p) @ jaw
            low, high = float(along.min()), float(along.max())
            width = high-low
            if not .022 <= width <= .027:
                continue
            support = min(int(np.sum(along < low+.0012)), int(np.sum(along > high-.0012)))
            if support < 4:
                continue
            # z18 first: the 2 mm lower z16 pinch passed the bench checks on one
            # host but lost the camera in transport; z18
            # held in every full run.
            candidates.append((100*abs(z_tip-.018)-.01*support,
                               Grasp(p+jaw*(low+high)/2, R, width,
                                     f"axial-z{1000*z_tip:.0f}-{'+' if sign > 0 else '-'}",
                                     # 12-14 N holds still at the mount
                                     # orientation; at ~35 N the curved lens
                                     # front squeezed out of the pads.
                                     squeeze=.004, settle_s=1.)))
    for _, g in sorted(candidates, key=lambda c: c[0]):
        yield g


def grasps(cell, name):
    """Antipodal box/cylinder grasps, followed by collision-mesh ray candidates."""
    kind = cell.graph["instances"][name]["kind"]
    part = cell.kit["parts"][kind]
    if kind == "lidar":
        yield from lidar_housing_grasps(cell, name)
        return
    if kind == "camera":
        yield from camera_grasps(cell, name)
        return
    if part.get("grasp_mode") == "surface":
        yield from surface_grasps(cell, name)
        return
    bb = np.asarray(part["bounds_m"])
    center = (bb[0]+bb[1])/2
    if kind in ("prop_cw", "prop_ccw"):
        # Pinch the pitched blade across its thickness, away from the three
        # roots and hub. CCW is the source mesh's reflected CW geometry.
        mirror = 1. if kind == "prop_cw" else -1.
        R = Rotation.from_euler("z",240 if mirror > 0 else 120,degrees=True).as_matrix()
        R = R @ Rotation.from_euler("x",45*mirror,degrees=True).as_matrix() @ DOWN
        p = np.array([-.023529567724055858,-.026829716583874622*mirror,.013179790526569504])
        for target_yaw in (0.,math.pi/2,-math.pi/2,math.pi):
            for yaw in (0.,120.,240.):
                symmetry=Rotation.from_euler("z",yaw,degrees=True).as_matrix()
                yield Grasp(symmetry @ p,symmetry @ R,.00586788925370038,
                            f"pitched-blade-normal-{yaw:.0f}",target_yaw=target_yaw,settle_s=1.)
    if kind == "landing_strut":
        import trimesh
        shaft_mesh=trimesh.load_mesh(cell.assets/"given/kit/meshes"/part["mesh"])
        def shaft_grasp(R,height,label,target_yaw=0.):
            # Measure the actual shaft over the pad's longitudinal footprint;
            # the native aluminium shaft is narrower than the old 8 mm proxy.
            centerline=np.array([center[0],center[1],height])
            centers=[centerline+offset*R[:,0] for offset in (-.012,0.,.012)]
            origins=np.asarray(centers)-R[:,1]*.05
            hits,rays,faces=shaft_mesh.ray.intersects_location(origins,np.tile(R[:,1],(3,1)),multiple_hits=True)
            mids=[];spans=[]
            for k in range(3):
                use=np.flatnonzero(rays==k)
                if len(use)<2:return None
                use=use[np.argsort(hits[use]@R[:,1])];a,b=use[0],use[-1]
                if shaft_mesh.face_normals[faces[a]]@R[:,1]>-math.cos(math.radians(25)) or shaft_mesh.face_normals[faces[b]]@R[:,1]<math.cos(math.radians(25)):
                    return None
                spans.append(float((hits[b]-hits[a])@R[:,1]));mids.append((hits[a]+hits[b])/2)
            width=max(spans)
            if not .004<=width<=.020:return None
            # Centre the real shaft in pad_2's 19 mm height. The old fixed
            # 8 mm tool offset put this narrower shaft near the distal tip;
            # its initial 6 mm slip then rolled it out during reorientation.
            pad_center_offset=_tip(width)-.158-.0095
            return Grasp(np.mean(mids,axis=0)-R[:,2]*pad_center_offset,R,width,label,
                         target_yaw,squeeze=.004,settle_s=1.)
        # Align the long pad dimension with the settled shaft's horizontal
        # projection; a valid resting pose need not point along world X.
        body_R=cell.pose(name)[:3,:3]
        shaft_horizontal=body_R[:,2].copy();shaft_horizontal[2]=0.
        if np.linalg.norm(shaft_horizontal)<1e-6:
            shaft_horizontal=np.array([1.,0.,0.])
        else:
            shaft_horizontal/=np.linalg.norm(shaft_horizontal)
        downward=np.array([0.,0.,-1.])
        world_hand=np.column_stack([shaft_horizontal,np.cross(downward,shaft_horizontal),downward])
        Rtop = body_R.T @ world_hand
        for target_yaw in (0.,math.pi/2,-math.pi/2,math.pi):
            for height in (.040,.035,.045):
                candidate=shaft_grasp(Rtop,height,f"stable-top-{height:.3f}-yaw{target_yaw:.2f}",target_yaw)
                if candidate is not None:yield _bench_tip_lift(cell,name,candidate)
        # Grasp the measured shaft beneath the upper flange. A horizontal hand can
        # reach under the fixed base and insert the upright leg from below.
        for yaw in (-math.pi/2, 0, math.pi/2, math.pi):
            R = Rotation.from_euler("z", yaw).as_matrix() @ np.array([[0,0,1],[0,-1,0],[1,0,0.]])
            for height in (.040, .045, .035, .030):
                candidate=shaft_grasp(R,height,f"shaft-side-{yaw:.2f}-{height:.3f}")
                if candidate is not None:yield candidate
        return
    if kind in ("esc", "flight_controller", "compute", "battery", "lidar", "camera", "motor"):
        for yaw in (0, math.pi/2, math.pi, -math.pi/2):
            Rz = Rotation.from_euler("z", yaw).as_matrix()
            width = (bb[1]-bb[0])[1 if abs(math.sin(yaw)) < .5 else 0]
            if width > .078:
                continue
            engagement = min(.014, (bb[1,2]-bb[0,2])*.5)
            z = bb[1,2]-engagement+_tip(width)-.158
            yield Grasp(np.array([center[0],center[1],z]), Rz @ DOWN, width, f"top-{yaw:.2f}")
        return
    if kind == "sensor_cap":
        # The full 30 mm pad touches the outer arc at its center and the
        # inner arc near both ends. Its effective span is about 10 mm;
        # a single center ray incorrectly predicts a 4.8 mm strip at x=-20 mm.
        for yaw in (math.pi/2,-math.pi/2):
            R=Rotation.from_euler("z",yaw).as_matrix() @ DOWN
            yield Grasp(np.array([-.0175,0.,_tip(.010)-.158-.003]),R,.010,
                        f"pad-footprint-arc-{yaw:.2f}",squeeze=.003,settle_s=2.)
    if kind == "sensor_cheek":
        yield from cheek_rim_grasps(cell, name)
    # Thin manufactured plates and propellers: cast on the actual union of the
    # documented convex pieces and require opposing contact normals. The tool
    # descends only after checking the complete hand at its preshaped width.
    import trimesh
    meshes = [trimesh.load_mesh(cell.assets/"given/kit/collision"/fn) for fn in part["collision"]]
    mesh = trimesh.util.concatenate(meshes)
    z = .001 if kind not in ("prop_cw", "prop_ccw") else .004
    candidates = []
    xs = np.linspace(bb[0,0]+.004,bb[1,0]-.004,13)
    ys = np.linspace(bb[0,1]+.004,bb[1,1]-.004,13)
    centers = [np.array([0.,0.,z])] + [np.array([x,y,z]) for x in xs for y in ys]
    for yaw in (0, math.pi/2, math.pi, -math.pi/2, math.pi/4, -math.pi/4):
        Rz = Rotation.from_euler("z", yaw).as_matrix()
        axis = Rz[:,1]
        origins = np.array([p-axis*.5 for p in centers])
        points, rays, faces = mesh.ray.intersects_location(origins,np.tile(axis,(len(origins),1)),multiple_hits=True)
        for i,p in enumerate(centers):
            use = np.flatnonzero(rays == i)
            if len(use) < 2:
                continue
            order = use[np.argsort(points[use] @ axis)]
            # Adjacent solid spans allow edge/rim grasps rather than requiring
            # the gripper to span a plate's full bounding box.
            for u,v in list(zip(order[:-1], order[1:])) + ([(order[0],order[-1])] if len(order)>2 else []):
                a,b = points[u],points[v]
                width = float((b-a) @ axis)
                if not (.0035 if kind == "sensor_cap" else .006) <= width <= .075:
                    continue
                n1,n2 = mesh.face_normals[faces[u]],mesh.face_normals[faces[v]]
                if n1 @ axis > -math.cos(math.radians(15)) or n2 @ axis < math.cos(math.radians(15)):
                    continue
                pp = (a+b)*.5
                pp[2] = .017 if kind in ("prop_cw","prop_ccw") else _tip(width)-.158-.003
                if kind == "sensor_cap" and width>.020:
                    # The cap's end rims sit directly above the cheek edges.
                    # Engage its upper millimetre so the metal backers clear
                    # both installed supports during precision placement.
                    pp[2]=_tip(width)-.158+.0005
                priority = float(np.linalg.norm(pp[:2]))
                candidates.append((priority, Grasp(pp,Rz @ DOWN,width,f"ray-{yaw:.2f}-{pp[0]:.3f}-{pp[1]:.3f}")))
    seen = set()
    for _,g in sorted(candidates,key=lambda x:x[0]):
        key=tuple(np.round(g.p/.005).astype(int))+tuple(np.round(g.R[:,1],2))
        if key not in seen:
            seen.add(key); yield g


# Stock finger link vs its pads: the finger plate ends 1.5 mm beyond the pad
# tips, and the parallel linkage reaches furthest from the hand (pad tips at
# .1736 m) near an 18 mm aperture, i.e. while the open jaws approach a thin shaft.
FINGER_TIP_BEYOND_PAD_M = .0015
LINKAGE_REACH_MAX_M = .059098+math.hypot(.035465, .042039)+.0595
BENCH_TIP_CLEARANCE_M = .0005


def _bench_tip_lift(cell, name, grasp):
    """Raise a straight top-down bench grasp until the finger tips clear the table.

    Landing struts only. The stable-top grasp centres the pads on the lying
    shaft's axis, 10.4 mm above the bench: the pad tips end 0.9 mm above it and
    the finger plates 0.6-0.9 mm inside it (open approach and grip; the finger
    links are visual-only in the cell, so only the planning model's finger
    colliders see this). Moving the tool point back along the approach keeps
    the shaft's full diameter between the pads (the contact lines stay at its
    equator) and the finger tips BENCH_TIP_CLEARANCE_M above the table plane
    (z = 0) at any aperture.
    """
    p, R = _world(cell.pose(name), grasp)
    if R[2, 2] > -.999:
        return grasp
    low = p[2]+R[2, 2]*(LINKAGE_REACH_MAX_M-.158+FINGER_TIP_BEYOND_PAD_M)
    lift = BENCH_TIP_CLEARANCE_M-low
    if lift > 0:
        grasp.p = grasp.p-grasp.R[:, 2]*lift
        grasp.label += f"-lift{1000*lift:.1f}"
    return grasp


def _world(T, grasp):
    return T[:3,3]+T[:3,:3] @ grasp.p, T[:3,:3] @ grasp.R


def _target(cell,mate,grasp):
    T=cell.coupling.target(mate).copy()
    if grasp.target_yaw:
        T[:3,:3] = T[:3,:3] @ Rotation.from_euler("z",grasp.target_yaw).as_matrix()
    return T


def _least_travel_first(cell, motion, mate, candidates, pre_offset, count=4):
    """Camera / flight controller only: order the leading grasps by joint travel.

    Their best candidates come in mirrored pairs (the same contact patch with
    the jaws swapped) whose priorities tie to ~1e-9, so bench micro-drift used
    to decide the arm branch. With joint 7 wound near its limit, the wrong
    twin spent ~30 s unwinding the wrist (FC 75.6 s instead of ~44 s). Rank
    the first `count` IK-feasible candidates by the joint travel to the pick
    and from there to the mount hover; the remaining candidates follow in
    their original order. Planning only: nothing moves here.
    """
    T = cell.pose(mate["part"])
    q_now = motion.q
    head = []
    for g in candidates:
        p, R = _world(T, g)
        gp, gR = _world(_target(cell, mate, g), g)
        pick = motion.ik(p, R, width=min(.085, g.width+.012), attempts=10, max_solutions=1, clearance=False)
        mount = motion.ik(gp+pre_offset, gR, width=g.width, attempts=10, max_solutions=1,
                          seed=pick[0]) if pick else []
        if mount:
            travel = float(np.max(np.abs(pick[0]-q_now))+np.max(np.abs(mount[0]-pick[0])))
            head.append((travel, len(head), g))
            _log("travel", mate["part"], g.label, round(travel, 2))
        if len(head) >= count:
            break
    # Near-ties (within TRAVEL_TIE_RAD) keep the original grasp priority: two FC
    # patches 2 mm apart (surface-1.57-96 / -97) tie to <0.02 rad, and the
    # arm's history then decided between the validated 38.5 mm Cube pinch and a
    # 40.5 mm pinch that slipped out in transport.
    # Flight controller only; the camera keeps the plain (travel, priority) order.
    tie = TRAVEL_TIE_RAD if cell.graph["instances"][mate["part"]]["kind"] == "flight_controller" else 0.
    best = min((h[0] for h in head), default=0.)
    near = sorted((h for h in head if h[0] <= best+tie), key=lambda h: h[1])
    far = sorted((h for h in head if h[0] > best+tie), key=lambda h: h[:2])
    for _, _, g in near+far:
        yield g
    yield from candidates


TRAVEL_TIE_RAD = .1


# Per-pad squeeze assumed by the closed-jaw grasp check: the live pads press
# 0.05-0.6 mm into a gripped part (soft contacts under the closing force).
GRIP_CHECK_SQUEEZE_M = .0005


def _grip_clear(motion, q, grasp, body, name):
    """Closed-jaw check of a grasp at its pick, with the finger link colliders.

    The jaws close on the part at the measured contact width, squeeze each pad
    a few tenths of a millimetre into it, and closing lowers the fingertips
    (parallel linkage). At the contact width and at the squeezed width only
    pad-face contact with the part is accepted (Motion.collision): a finger
    side, tip or back inside the part (a cheek rim, a leg flange, the LiDAR
    housing edge) or any contact with another body rejects the candidate
    before the arm moves. Planning only.
    """
    for w in (grasp.width, max(0., grasp.width-2*GRIP_CHECK_SQUEEZE_M)):
        if motion.collision(q, w, allow_pad_body=body, clearance=False):
            _log("reject grasp", name, grasp.label, "closed width", round(1000*w, 2), "mm:", motion.last_contacts[:3])
            return False
    return True


def _open_clear(motion, body, width, seconds=.6):
    """Open the jaws on a released part only as far as the swept finger links
    stay clear of everything but pad-face contact with that part.

    The opening moves each finger's back and sides ~6 mm outwards. A contact
    that already exists at the closed jaw may remain while it does not get
    deeper (as Motion.line's escape rule); a new or deepening one ends the
    sweep. If even the first millimetre is blocked, open that millimetre
    (logged): the grip must be released before the retreat. With a clear
    sweep this is set_width().
    """
    motion.sync()
    current=motion.width()
    target=current
    motion.collision(motion.q,width=current,allow_pad_body=body,clearance=False)
    previous={}
    for a,b,d in motion.last_contacts:
        key=tuple(sorted((a,b)));previous[key]=min(d,previous.get(key,math.inf))
    for w in np.linspace(current,width,10)[1:]:
        motion.collision(motion.q,width=w,allow_pad_body=body,clearance=False)
        now={}
        for a,b,d in motion.last_contacts:
            key=tuple(sorted((a,b)));now[key]=min(d,now.get(key,math.inf))
        if any(key not in previous or d<previous[key]-5e-5 for key,d in now.items()):
            _log("release aperture limited at",round(1000*w,2),"mm by",motion.last_contacts[:2])
            break
        previous=now
        target=w
    if target<current+.001 and width>current+.001:
        _log("release aperture blocked; minimal opening",round(1000*(current+.001),2),"mm")
        target=current+.001
    motion.set_width(target, seconds=seconds)
    return target


def _grip_forces(cell, name):
    members = body_subtree(cell.m,cell.ids[name])
    sides={"left":0.,"right":0.}
    for i,c in enumerate(cell.d.contact):
        for g,h in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
            gn = cell.m.geom(int(g)).name or ""
            if cell.m.geom_bodyid[h] in members and "finger_pad" in gn:
                force=np.zeros(6)
                mujoco.mj_contactForce(cell.m,cell.d,i,force)
                sides[gn.split("_")[0]]+=max(0.,float(force[0]))
    return sides


def _gripped(cell, name):
    return min(_grip_forces(cell,name).values())>.05


def _prop_squeeze(cell, motion, name, *, min_force=1.5, max_s=4.0):
    """Wait until the stock jaws have actually pinched a thin propeller blade.

    Near full closure the gripper servo's residual force is below the joint
    friction of the stock linkage, so the jaws creep shut over seconds. The
    source blade is ~1 mm thick and twisted: the pads first touch it lightly
    at two points, and the squeeze needs 1-3 s to build. Lifting at first
    touch drags the blade out of the pads. Only waits; never commands more.
    """
    history=[]
    t_end=cell.d.time+max_s
    while cell.d.time<t_end:
        motion.wait(.1)
        forces=_grip_forces(cell,name)
        history.append((round(float(cell.d.time),2),round(1000*motion.width(),2),
                        round(forces["left"],2),round(forces["right"],2)))
        if (len(history)>=3 and min(forces.values())>=min_force
                and abs(history[-3][1]-history[-1][1])<.05):
            break
    _log("prop squeeze",name,history[::2]+history[-1:])
    return history[-1]


def _preinsert(motion, pre_p, goal_p, R, body, *, retreat=None, open_width=None, attempts=64, clearance=True, transport_clearance=None, speed=.25):
    """Choose an IK branch for a complete straight approach and optional exit."""
    if transport_clearance is None:
        transport_clearance=clearance
    solutions=motion.ik(pre_p,R,width=motion.width(),attempts=attempts,max_solutions=16,allow_pad_body=body,clearance=clearance)
    errors=[]
    for candidate in solutions:
        prev=candidate
        ok=True
        for u in np.linspace(0,1,34)[1:]:
            q=motion._ik_one((1-u)*np.asarray(pre_p)+u*np.asarray(goal_p),R,prev,width=motion.width())
            if q is None or not motion.segment_free(prev,q,width=motion.width(),allow_pad_body=body,clearance=clearance):
                ok=False;break
            prev=q
        if not ok:
            continue
        if retreat is not None:
            # The detached component stays at its final pose in the planning
            # copy while the open gripper retreats. Verify this exit on the
            # same IK branch before moving the real arm into a dead end.
            motion._fk(prev,motion.width())
            saved_held=motion.held
            motion.held=None
            try:
                for u in np.linspace(0,1,32)[1:]:
                    q=motion._ik_one(np.asarray(goal_p)+u*np.asarray(retreat),R,prev,width=open_width)
                    if q is None or not motion.segment_free(prev,q,width=open_width,allow_pad_body=body,clearance=clearance):
                        ok=False;break
                    prev=q
            finally:
                motion.held=saved_held
            if not ok:
                continue
        try:
            motion.move_joints(candidate,width=motion.width(),allow_pad_body=body,speed=speed,clearance=transport_clearance)
            return
        except PlanningError as e:
            errors.append(str(e))
    raise PlanningError(f"no collision-free Cartesian approach branch for {body or 'empty gripper'}: {errors[:2]} last {motion.last_contacts[:3]}")


def _prop_axis_error(cell, name, target):
    """Lateral bore-to-shaft-axis error, height above seat and tilt of a held prop."""
    T=cell.pose(name)
    axis=target[:3,2]
    delta=T[:3,3]-target[:3,3]
    height=float(delta@axis)
    lateral=float(np.linalg.norm(delta-height*axis))
    tilt=math.degrees(math.acos(float(np.clip(T[:3,2]@axis,-1,1))))
    return lateral,height,tilt


# Prop seat geometry (fixture frame): hub bottom 0.12 mm above the prop origin,
# rotor shaft end 12 mm above the seat. Above PROP_GATE_M the bore is not yet on
# the shaft, so the servo only descends while the hub is centred on the axis.
PROP_ENTRY_M=.014
PROP_GATE_M=.011
# The bell's convex top meets the 13-sided bore edge ~0.17 mm below nominal;
# end the descent with the hub resting on it, not held above it.
PROP_REST_M=-.00025
# Free-space joint speed for the empty-hand move to a propeller pick.
PROP_FREE_SPEED=.6


# Arm self-clearance kept along a propeller insertion (the planning copy's
# link-link shell is 2 x 4 mm). The in-grasp pivot keeps turning the hand
# during the servo descent, so a branch must also stay contact-free for a few
# degrees of pivot either way; the servo swivels the elbow below a 3 mm gap.
PROP_SELF_SHELL=.008
PROP_SELF_ACCEPT=.003
PROP_SERVO_SELF_MIN=.003
# Servo outer integral (1/s) and its clamp.
PROP_SERVO_KI=1.5
PROP_SERVO_BIAS_MAX=.003
# The prop's final yaw is free: turning it about its own axis at the hover
# gives the hand (45 deg pitched-blade pinch) a different orientation. Pivot
# variants: ~12 deg build up during transport, a few more during the descent.
PROP_YAW_TURNS_DEG=(0.,30.,-30.,60.,-60.,90.,-90.,120.,-120.,150.,-150.,180.)
PROP_TRANSPORT_PIVOT_DEG=(12.,)
PROP_HOVER_PIVOT_DEG=(6.,)


def _prop_robot_names(motion):
    """(robot body names, gripper body names): the arm's self-distance ignores
    pairs inside the gripper (its two fingers sit ~2 mm apart on a blade)."""
    names=getattr(motion,"_prop_robot_names",None)
    if names is None:
        robot=frozenset(motion.m.body(int(b)).name for b in motion.robot_bodies)
        hand=frozenset(motion.m.body(int(b)).name for b in body_subtree(motion.m,motion.hand))
        names=motion._prop_robot_names=(robot,hand)
    return names


def _prop_contacts(motion, q, width, body):
    """One planning query: (contacts that penetrate, arm self-distance).

    Same pair rules as Motion.collision; with clearance on, pairs inside their
    planning shells are listed with their signed distances, so penetration and
    the arm's link self-distance (link-link and link-gripper pairs, capped at
    the 8 mm shell) come from one query."""
    motion.collision(q,width,body,True)
    robot,hand=_prop_robot_names(motion)
    pen=[c for c in motion.last_contacts if c[2]<-2e-5]
    own=[c[2] for c in motion.last_contacts
         if c[0] in robot and c[1] in robot and not (c[0] in hand and c[1] in hand)]
    return pen,(min(own) if own else PROP_SELF_SHELL)


def _prop_variant(held, delta, width):
    """The held transform `held` turned by delta about the jaw axis through
    the pad contact (the direction the pinched blade pivots in)."""
    b,adr,p,rel=held
    if not delta:
        return held
    c=np.array([0.,0.,_tip(width)-.005])
    Ry=Rotation.from_euler("y",delta).as_matrix()
    return (b,adr,c+Ry@(np.asarray(p)-c),Ry@np.asarray(rel))


def _prop_tool(motion, held, pos, R):
    """Tool pose that puts the held part (transform `held`) at pos/R."""
    _b,_a,p,rel=held
    hand_R=np.asarray(R)@np.asarray(rel).T
    return np.asarray(pos)+hand_R@(motion.offset-np.asarray(p)),hand_R


def _prop_branch_check(motion, body, q0, target, desR, width, open_width, *, h_top, h_end, pivots_deg=(6.,)):
    """Plan-only check of one arm branch for the whole propeller insertion.

    From configuration q0 (at or near the hover) the straight descent of the
    held prop along its shaft axis, in 3 mm steps, must stay contact-free with
    the arm's link self-distance >= PROP_SELF_ACCEPT; with the measured grasp
    turned by +-each of pivots_deg about the jaw axis the same descent must stay
    contact-free; and from the seat one of the exits of _prop_exit must be
    clear with the jaws open and the prop left on its seat. Never moves the
    robot. Returns (ok, min self-distance of the measured-grasp descent capped
    at the 8 mm shell, reason)."""
    axis=target[:3,2]
    saved=motion.held
    worst=math.inf
    heights=np.linspace(h_top,h_end,max(2,int(math.ceil((h_top-h_end)/.003))+1))
    deltas=(0.,)+tuple(math.radians(sign*p) for p in pivots_deg for sign in (1.,-1.))
    bits=getattr(motion,"_finger_bits",None)
    try:
        for delta in deltas:
            held=_prop_variant(saved,delta,width)
            motion.held=held
            margin=PROP_SELF_ACCEPT if not delta else 0.
            prev=np.asarray(q0,float)
            for j,hk in enumerate(heights):
                tp,tR=_prop_tool(motion,held,target[:3,3]+axis*hk,desR)
                q=motion._ik_one(tp,tR,prev,width=width)
                if q is None:
                    return False,worst,f"no IK at {1000*hk:.0f} mm (pivot {math.degrees(delta):+.0f} deg)"
                if j and np.max(np.abs(q-prev))>.3:
                    return False,worst,f"branch jump at {1000*hk:.0f} mm (pivot {math.degrees(delta):+.0f} deg)"
                pen,own=_prop_contacts(motion,q,width,body)
                if not delta:
                    worst=min(worst,own)
                if pen:
                    return False,worst,f"contact {pen[0][:2]} at {1000*hk:.0f} mm (pivot {math.degrees(delta):+.0f} deg)"
                if own<margin-1e-6:
                    return False,worst,f"self-distance {1000*own:.1f} mm at {1000*hk:.0f} mm (pivot {math.degrees(delta):+.0f} deg)"
                prev=q
            if not delta:
                # Exit from the seat: the prop stays where the last held FK put
                # it; as after release it is an ordinary obstacle again.
                motion._fk(prev,width)
                motion.held=None
                if bits is not None:
                    ct,ca=motion.pm.geom_contype.copy(),motion.pm.geom_conaffinity.copy()
                    motion.pm.geom_contype[:],motion.pm.geom_conaffinity[:]=bits
                try:
                    pp,rr=motion._fk(prev,open_width)
                    rr=rr.copy()
                    exits=[-rr[:,2]*d for d in (.08,.06,.04,.03)]+[rr[:,0]*d for d in (.05,.045)]
                    exits.append((rr[:,0]-rr[:,2])*(.06/math.sqrt(2)))
                    clear=any(_prop_line_free(motion,prev,v,open_width) for v in exits)
                finally:
                    if bits is not None:
                        motion.pm.geom_contype[:],motion.pm.geom_conaffinity[:]=ct,ca
                if not clear:
                    return False,worst,"no clear exit from the seat"
        return True,worst,""
    finally:
        motion.held=saved
        motion.sync()


def _prop_branch_search(motion, body, target, base_R, h_top, width, open_width, *, pivots_deg, q_ref=None,
                        yaw_turns=PROP_YAW_TURNS_DEG, ik_per_yaw=6, keep=3):
    """Hover branches for the held prop that pass _prop_branch_check.

    Candidates are the prop's free-yaw turns about its own axis x the
    contact-free IK branches of the hover pose, tried nearest to q_ref in joint
    space first (the shortest arm move). Returns ([(q, yaw_deg, desR,
    min self-distance, joint distance)], rejection reasons, candidate count)."""
    axis=target[:3,2]
    q_ref=motion.q if q_ref is None else np.asarray(q_ref,float)
    candidates=[]
    for yaw in yaw_turns:
        R=Rotation.from_rotvec(axis*math.radians(yaw)).as_matrix()@np.asarray(base_R)
        tp,tR=_prop_tool(motion,motion.held,target[:3,3]+axis*h_top,R)
        # Contact-free IK only: the pinched blade sits inside the hand's
        # planning shell; _prop_branch_check applies the arm's own margin.
        for qc in motion.ik(tp,tR,width=width,attempts=32,max_solutions=ik_per_yaw,allow_pad_body=body,clearance=False):
            candidates.append((float(np.linalg.norm(qc-q_ref)),yaw,R,qc))
    candidates.sort(key=lambda c:c[0])
    passed,reasons=[],[]
    for dist,yaw,R,qc in candidates:
        ok,worst,why=_prop_branch_check(motion,body,qc,target,R,width,open_width,h_top=h_top,
                                        h_end=PROP_REST_M,pivots_deg=pivots_deg)
        if not ok:
            reasons.append(f"yaw {yaw:+.0f}: {why}")
            continue
        passed.append((qc,yaw,R,worst,dist))
        if len(passed)>=keep:
            break
    # Prefer the full 8 mm clearance among the nearest passing branches.
    passed.sort(key=lambda c:(c[3]<PROP_SELF_SHELL-1e-6,c[4]))
    return passed,reasons,len(candidates)


def _prop_preinsert(motion, body, target, pre_distance, open_width, *, speed=.65):
    """Transport the held prop to a hover branch verified for the whole
    insertion even after the in-grasp pivot expected during transport
    (PROP_TRANSPORT_PIVOT_DEG, then PROP_HOVER_PIVOT_DEG if none passes).
    Returns (yaw turn deg, min self-distance). Raises PlanningError."""
    width=motion.width()
    errors=[]
    for pivots in (PROP_TRANSPORT_PIVOT_DEG,PROP_HOVER_PIVOT_DEG):
        passed,reasons,n=_prop_branch_search(motion,body,target,target[:3,:3],pre_distance,width,open_width,pivots_deg=pivots)
        for qc,yaw,R,worst,dist in passed:
            try:
                motion.move_joints(qc,width=width,allow_pad_body=body,speed=speed,clearance=True)
                return yaw,worst,pivots
            except PlanningError as error:
                errors.append(str(error)[:100])
        errors.extend(reasons[:2])
    raise PlanningError(f"no verified insertion branch at the hover: {errors[:4]}")


def _prop_select_branch(cell, motion, mate, name, body, target, open_width):
    """Re-check the arm branch at the hover with the grasp as it is NOW.

    The pinched blade pivots in the jaws during transport (11-13 deg seen), so
    the hover alignment turns the hand well away from the planned branch; with
    the 45 deg pitched-blade hand that can fold the elbow (joint4 ~0.08) so far
    that link5 meets link2 on the way down, for every IK branch of that hand
    pose. If the present branch fails _prop_branch_check, move at the hover
    (planned joint move, prop held) to the nearest branch that passes, turning
    the prop about its own (free) axis if needed. Returns True if it moved."""
    motion.hold_body("part_"+name)
    width=motion.width()
    P=cell.pose(name)
    desR,_=axis_aligned_rotation(P[:3,:3],target[:3,:3])
    axis=target[:3,2]
    h_top=float((P[:3,3]-target[:3,3])@axis)
    ok,worst,why=_prop_branch_check(motion,body,motion.q,target,desR,width,open_width,h_top=h_top,
                                    h_end=PROP_REST_M,pivots_deg=PROP_HOVER_PIVOT_DEG)
    if ok:
        _log("prop branch",name,"hover branch ok; min self-distance mm",round(1000*worst,2))
        return False
    _log("prop branch",name,"hover branch rejected:",why)
    q_before=motion.q
    passed,reasons,n=_prop_branch_search(motion,body,target,desR,h_top,width,open_width,pivots_deg=PROP_HOVER_PIVOT_DEG)
    for qc,yaw,R,worst,dist in passed:
        try:
            motion.move_joints(qc,width=width,allow_pad_body=body,speed=.4,clearance=True)
        except PlanningError as error:
            reasons.append(str(error)[:100])
            continue
        _log("prop branch",name,"re-configured at the hover: yaw turn deg",yaw,"joint move",round(dist,2),
             "min self-distance mm",round(1000*worst,2),"dq",np.round(qc-q_before,2))
        return True
    # Nothing better reachable: keep the present branch; the servo's elbow
    # swivel (_prop_swivel) remains the last guard.
    _log("prop branch",name,"no better hover branch",n,"candidates:",reasons[:3])
    return False


def _prop_swivel(motion, q_goal, tool_p, hand_R, width, body, q_cmd, max_step=.05):
    """Elbow self-motion for one servo step: same tool pose, joint-3 offset
    seeds; the solution with the largest arm self-distance that penetrates
    nothing and stays within max_step of the last command, or None."""
    best=None
    for k in range(1,7):
        for sign in (1.,-1.):
            seed=np.asarray(q_goal,float).copy();seed[2]+=sign*.015*k
            q=motion._ik_one(tool_p,hand_R,seed,width=width)
            if q is None or np.max(np.abs(q-q_cmd))>max_step:
                continue
            pen,own=_prop_contacts(motion,q,width,body)
            if pen:
                continue
            if best is None or own>best[1]:
                best=(q,own)
        if best is not None and best[1]>=PROP_SELF_SHELL:
            break
    return best


def _prop_report(cell, mate, name, target, stage):
    lateral,height,tilt=_prop_axis_error(cell,name,target)
    _log("prop insert",name,stage,"lateral_mm",round(1000*lateral,3),"height_mm",round(1000*height,3),
         "tilt_deg",round(tilt,3),"state",cell.coupling.state[mate["id"]],"t",round(float(cell.d.time),2))
    return lateral,height,tilt


def _prop_servo(cell, motion, mate, name, body, target, *, end_h, fast=.025, slow=.006,
                lateral_gate=.00015, max_s=12., wall_deadline=math.inf):
    """Closed-loop descent of a held propeller hub onto its rotor shaft.

    The 1.6 mm blade is pinched at a few pad points and slowly pivots about the
    jaw axis under its own weight (soft contact friction), ~1 mm/s at the hub.
    An open-loop line therefore drifts off a shaft that the bore clears by only
    0.43 mm. Every 50 ms this re-measures the live part, refreshes the held
    transform and commands a small, collision-checked IK step that puts the hub
    back on the shaft axis while descending. Above the shaft end the descent
    pauses until the hub is centred. Only arm actuator targets are written.
    """
    axis=target[:3,2]
    P=cell.pose(name)
    h_ref=float((P[:3,3]-target[:3,3])@axis)
    q_cmd=motion.q
    width=motion.width()
    # Seed every servo IK step from the straight-descent branch planned now
    # with the measured grasp (the branch transport planning verified for the
    # exit). Seeding only from the last command lets the 7-DOF null space
    # drift over a hundred steps, e.g. into a link2/link5 self-contact.
    def make_branch(q_from,h_from):
        R0,_=axis_aligned_rotation(cell.pose(name)[:3,:3],target[:3,:3])
        hs=np.linspace(h_from,end_h,max(2,int(math.ceil((h_from-end_h)/.002))+1))
        qs=[np.asarray(q_from,float).copy()]
        for hk in hs[1:]:
            bp,bR=motion.body_goal(target[:3,3]+axis*hk,R0,body)
            qk=motion._ik_one(bp,bR,qs[-1],width=width)
            if qk is None:
                raise PlanningError(f"{name} no straight descent branch at height {1000*hk:.1f} mm")
            qs.append(qk)
        return hs,np.asarray(qs)
    branch_h,branch_q=make_branch(q_cmd,h_ref)
    def branch_seed(hh):
        return np.array([np.interp(hh,branch_h[::-1],branch_q[::-1,j]) for j in range(7)])
    swivels=0
    t_end=float(cell.d.time)+max_s
    paused=0
    history=[]
    # Outer integral on the measured hub's lateral error. While the servo
    # follows the in-grasp pivot the arm reference keeps moving, which freezes
    # Motion._drive's own integral, and the joints' friction dead band then
    # leaves the hand ~0.8 mm short of every command. Frozen once the bore is
    # on the shaft (the shaft then holds the hub; never push it sideways).
    bias=np.zeros(3)
    while True:
        if float(cell.d.time)>t_end:
            _prop_report(cell,mate,name,target,"servo-timeout")
            raise PlanningError(f"{name} servo descent timed out")
        if time.monotonic()>wall_deadline:
            raise TimeoutError("reference wall cap during prop servo")
        if cell.coupling.state[mate["id"]] in ("LOCKED","LOCKED_OFF"):
            break  # fastened at rest on the bell: stop commanding the descent
        P=cell.pose(name)
        delta=P[:3,3]-target[:3,3]
        h=float(delta@axis)
        lateral=float(np.linalg.norm(delta-h*axis))
        history.append(h)
        # A hub that stops descending while commanded down is obstructed
        # (e.g. resting on the shaft end): stop instead of pushing.
        if len(history)>20 and history[-21]-h<.0001 and h_ref<h-.0003 and h>end_h+.0005:
            _prop_report(cell,mate,name,target,"blocked")
            raise PlanningError(f"{name} hub blocked {1000*(h-end_h):.1f} mm above its seat")
        if h_ref<=PROP_GATE_M or lateral<=lateral_gate:
            step=(fast if h_ref>PROP_ENTRY_M else slow)*.05
            # Never lead the measured hub by more than 1.5 mm.
            h_ref=max(end_h,h_ref-step,h-.0015)
        else:
            paused+=1
        if h>PROP_GATE_M:
            bias+=PROP_SERVO_KI*.05*(delta-h*axis)
            norm=float(np.linalg.norm(bias))
            if norm>PROP_SERVO_BIAS_MAX:
                bias*=PROP_SERVO_BIAS_MAX/norm
        desired_R,_=axis_aligned_rotation(P[:3,:3],target[:3,:3])
        tool_p,hand_R=motion.body_goal(target[:3,3]+axis*h_ref-bias,desired_R,body)
        # Bound each 50 ms correction (1.5 mm, 0.8 deg) for a smooth command.
        cur_p,cur_R=motion._fk(q_cmd,width)
        dp=tool_p-cur_p
        if np.linalg.norm(dp)>.0015:
            tool_p=cur_p+dp*(.0015/np.linalg.norm(dp))
        rv=Rotation.from_matrix(hand_R@cur_R.T).as_rotvec()
        if np.linalg.norm(rv)>math.radians(.8):
            hand_R=Rotation.from_rotvec(rv*(math.radians(.8)/np.linalg.norm(rv))).as_matrix()@cur_R
        seed=q_cmd+.3*(branch_seed(h_ref)-q_cmd)
        q_goal=motion._ik_one(tool_p,hand_R,seed,width=width)
        if q_goal is None:
            raise PlanningError(f"{name} servo IK failed at height {1000*h:.2f} mm")
        pen,own=_prop_contacts(motion,q_goal,width,body)
        if pen or own<PROP_SERVO_SELF_MIN:
            # The arm's own links (e.g. link5 towards link2) are closing in:
            # swivel the elbow for this step with the tool pose unchanged.
            alt=_prop_swivel(motion,q_goal,tool_p,hand_R,width,body,q_cmd)
            if alt is not None and (pen or alt[1]>own):
                q_goal,own=alt;pen=[]
                swivels+=1
                # Re-seed the remaining descent from the swivelled posture.
                branch_h,branch_q=make_branch(q_goal,max(end_h,h_ref))
        if pen:
            _prop_report(cell,mate,name,target,"servo-collision")
            raise PlanningError(f"{name} servo step in collision: {pen[:3]}")
        for k in range(1,6):
            motion._drive(q_cmd+(q_goal-q_cmd)*k/5);cell.tick()
        q_cmd=q_goal
        if h_ref<=end_h and h-end_h<.0003 and lateral<.0004:
            break
    _log("prop servo",name,"paused_cycles",paused,"elbow_swivels",swivels,"lateral_bias_mm",np.round(1000*bias,3))
    return q_cmd


def _prop_insert(cell, motion, mate, name, body, target, ins, pre_distance, wall_deadline=math.inf, open_width=.018):
    """Lower a held propeller's 13-sided bore over its passive rotor shaft.

    The bore (inradius 2.43 mm) clears the 2 mm shaft by only ~0.43 mm and
    neither has a chamfer, so a hub that is not centred on the axis when it
    reaches the shaft end lands on top of it (see _prop_servo).
    """
    _prop_report(cell,mate,name,target,"arrived")
    # Coarse correction of the transport pivot; _prop_servo centres the hub.
    try:
        motion.align_body(target[:3,3]-ins*pre_distance,target[:3,:3],body=body,
                          tolerance=.0015,angle=.03,max_passes=3,axis_only=True)
    except PlanningError as error:
        if "did not converge" not in str(error):
            raise
        _log("prop insert",name,"coarse hover alignment incomplete:",str(error))
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} during hover alignment")
    if _prop_select_branch(cell,motion,mate,name,body,target,open_width):
        try:
            motion.align_body(target[:3,3]-ins*pre_distance,target[:3,:3],body=body,
                              tolerance=.0015,angle=.03,max_passes=3,axis_only=True)
        except PlanningError as error:
            if "did not converge" not in str(error):
                raise
            _log("prop insert",name,"coarse hover alignment incomplete:",str(error))
        if not _gripped(cell,name):
            raise PlanningError(f"lost grip on {name} during hover re-configuration")
    _prop_report(cell,mate,name,target,"hover")
    q=_prop_servo(cell,motion,mate,name,body,target,end_h=PROP_REST_M,wall_deadline=wall_deadline)
    _prop_report(cell,mate,name,target,"seated")
    return q


def _prop_seat(cell, motion, mate, name, body, q, released_width, wall_deadline=math.inf,
               hold_s=.8, settle_s=3.):
    """Wait for the at-rest fastening of a hub resting on its bell.

    If the pinch keeps the prop from being still against the passive rotor,
    open the jaws (aperture sweep checked) and let the prop rest on the shaft
    and bell under its own weight; the fastening then locks the resting pose.
    """
    key=mate["id"]
    def locked():
        return cell.coupling.state[key] in ("LOCKED","LOCKED_OFF")
    for _ in range(int(round(hold_s/.01))):
        if time.monotonic()>wall_deadline:
            raise TimeoutError("reference wall cap during seat hold")
        motion._drive(q);cell.tick()
        if locked():
            _log("prop seat",name,"locked while held")
            return
    motion.sync()
    current=motion.width()
    target_width=current
    for w in np.linspace(current,released_width,10)[1:]:
        if motion.collision(q,width=w,allow_pad_body=body,clearance=False):
            break
        target_width=w
    _log("prop seat",name,"release to",round(1000*target_width,2),"mm; state",cell.coupling.state[key])
    command=float(np.clip(100*Motion.angle_for_width(target_width)/.333,0,255))
    start=float(cell.d.ctrl[motion.ga])
    ramp=60
    for k in range(int(round(settle_s/.01))):
        if time.monotonic()>wall_deadline:
            raise TimeoutError("reference wall cap during seat release")
        if k<ramp:
            cell.d.ctrl[motion.ga]=start+(command-start)*(k+1)/ramp
        motion._drive(q);cell.tick()
        if locked() and k>=ramp:
            _log("prop seat",name,"locked after release at",round(float(cell.d.time),2))
            return


def _prop_open(motion, body, width):
    """Open the jaws beside an installed propeller only as far as the swept
    finger linkage stays clear of everything but the blade it just released."""
    motion.sync()
    current=motion.width()
    target=current
    for w in np.linspace(current,width,10)[1:]:
        if motion.collision(motion.q,width=w,allow_pad_body=body,clearance=False):
            break
        target=w
    motion.set_width(target)
    return target


def _prop_line_free(motion, q0, vector, width):
    """Dry-run of Motion.line's precheck from configuration q0 (no pad contact allowed)."""
    p0,r0=motion._fk(q0,width)
    p0,r0=p0.copy(),r0.copy()
    motion.collision(q0,width,None,False)
    initial={}
    for a,b,d in motion.last_contacts:
        key=tuple(sorted((a,b)));initial[key]=min(d,initial.get(key,math.inf))
    n=max(1,int(math.ceil(np.linalg.norm(vector)/.003)))
    prev=q0
    for u in np.linspace(0,1,n+1)[1:]:
        q=motion._ik_one(p0+u*np.asarray(vector),r0,prev,width=width)
        if q is None:
            return False
        steps=max(1,int(math.ceil(np.max(np.abs(q-prev))/.007)))
        for v in np.linspace(0,1,steps+1)[1:]:
            if motion.collision((1-v)*prev+v*q,width,None,False):
                for a,b,d in motion.last_contacts:
                    key=tuple(sorted((a,b)))
                    if key not in initial or d<initial[key]-5e-5:
                        return False
        prev=q
    return True


def _prop_exit(cell, motion, name, released_width):
    """Withdraw the open jaws from an installed propeller without touching it.

    Exits stay in the pad plane: back along the approach (tool -z) or off the
    blade along its span (tool +x, outward). The in-grasp pivot changes the
    hand orientation after transport planning verified its exit, and the seat
    posture can then leave link5 near link2. If every straight exit is blocked,
    swivel the elbow in small IK steps with the tool pose fixed (a self-motion
    of the 7-DOF arm) until one is clear. No pad contact with the prop allowed.
    """
    motion.sync()
    width=max(released_width,motion.width())
    pp,rr=motion.tool_pose()
    exits=[-rr[:,2]*d for d in (.08,.06,.04,.03)]+[rr[:,0]*d for d in (.05,.045)]
    exits.append((rr[:,0]-rr[:,2])*(.06/math.sqrt(2)))
    q0=motion.q
    postures=[[q0]]
    for sign in (-1.,1.):
        path=[q0];q=q0.copy()
        for _ in range(12):
            seed=q.copy();seed[2]+=sign*.05
            qk=motion._ik_one(pp,rr,seed,width=width)
            if qk is None or abs(qk[2]-q[2])<.002:
                break
            if not motion.segment_free(q,qk,width=width,allow_pad_body=None,clearance=False):
                break
            path.append(qk);q=qk
            postures.append(list(path))
    for path in postures:
        for vector in exits:
            if not _prop_line_free(motion,path[-1],vector,width):
                continue
            if len(path)>1:
                _log("retreat",name,"elbow self-motion dq3",round(float(path[-1][2]-q0[2]),3))
                motion.execute(path,speed=.1)
            p_now,_=motion.tool_pose()
            motion.line(p_now+vector,rr,width=width,allow_pad_body=None,clearance=False,speed=.03)
            _log("retreat",name,"exit_mm",np.round(1000*(rr.T@vector),1),"t",round(float(cell.d.time),2))
            return
    raise PlanningError(f"cannot retract from {name}: no clear exit after elbow self-motion")


def _prepare_width(motion, width):
    """Open only where the complete linkage's swept aperture stays clear."""
    current=motion.width()
    def clear():
        motion.sync()
        return all(not motion.collision(motion.q,width=w,clearance=True)
                   for w in np.linspace(current,width,12))
    if clear():
        return
    p,R=motion.tool_pose()
    errors=[]
    for height in (.08,.14):
        try:
            motion.move_pose(p+np.array([0.,0.,height]),R,width=current,speed=.25)
            if clear():
                return
        except PlanningError as error:
            errors.append(str(error))
    motion.move_pose(np.array([-.10,0.,.50]),DOWN,width=current,attempts=48,speed=.25)
    if not clear():
        raise PlanningError(f"no clear aperture sweep for {width}: {errors[:2]}")


# ----------------------------------------------------------------------------
# Landing struts (kind landing_strut). Part-specific; nothing here runs for
# other kinds.
#
# * Creep: MuJoCo's soft friction lets a pinched part slip at about
#   (1-d)/d*g/b ~ 1 mm/s along any pad-tangent direction that carries gravity,
#   whatever the grip force. A strut hanging from a downward-pointing hand walks
#   out of the fingertips, so it is lifted and turned upright right after the
#   pick (slow, smooth Cartesian moves: fast turns make the arm sag into the
#   staged guard). Upright, the shaft slides a little until the flange fillet
#   rests on the top edges of both pads, a support that does not creep.
# * Rattle: the shaft collider is a 9-sided prism, so one pad always meets a
#   facet and the other a vertex edge. MuJoCo reports that edge as a single
#   contact jumping between the pad ends every step: a hanging strut rocks at
#   ~50 Hz (60-120 deg/s) and the at-rest coupling never sees it still. At the
#   seat the hand therefore drives 1 mm past flush: the pad tops push the fillet
#   up and clamp the flange face flat against the plate, which stops the rocking.
# * Twist: the stable-top pick fixes the strut's twist in the hand, so with the
#   4-fold mount only four seat approach directions exist. Where none of them is
#   reachable (leg_4's corner) the strut is stood up on the bench and grasped
#   again from a direction that maps onto a reachable approach.
STRUT_PICK_STATION = .045       # shaft station (local z) of the pad centre at the pick
STRUT_PAD_HALF = .015           # half length of a stock pad along tool x
STRUT_LIFT = .075               # lift before turning upright: foot swing and hand over the staged guard
STRUT_HOVER = .020              # hover below the seat before the vertical insertion
STRUT_PRESS = .0010             # drive the seat this far past flush: the pad tops press the
                                # flange fillet up, clamping the flange flat against the plate
STRUT_HOLD_SQUEEZE = .0007      # lighter jaw command once the strut is upright
STRUT_OUTSIDE = .08             # horizontal stand-off of the hover outside the plate footprint


STRUT_FILLET = .0609            # local z where the flange fillet (cone) starts above the shaft
STRUT_SEATED_STATION = STRUT_FILLET-STRUT_PAD_HALF   # pad centre once the fillet rests on the pad tops


def _strut_seated(g):
    """The same physical grasp once the upright shaft has slid down onto the pad tops."""
    return Grasp(g.p+np.array([0., 0., STRUT_SEATED_STATION-STRUT_PICK_STATION]), g.R, g.width,
                 g.label+"-seated", g.target_yaw, g.squeeze, g.settle_s)


def _strut_in_hand(motion, body):
    """Measured strut origin (m) and shaft axis in the hand frame."""
    b = motion.m.body(body).id
    Rh = motion.d.xmat[motion.hand].reshape(3, 3)
    p = Rh.T @ (motion.d.xpos[b]-motion.d.xpos[motion.hand])
    axis = Rh.T @ motion.d.xmat[b].reshape(3, 3)[:, 2]
    return p, axis


def _strut_turn_pose(part_R, p, R, g):
    """Hand pose that turns the lifted strut upright about its grasped shaft section."""
    tcp = p+np.array([0., 0., STRUT_LIFT])
    upright, angle = axis_aligned_rotation(part_R, np.eye(3))
    turn = upright @ part_R.T
    centre = tcp+R[:, 2]*(_tip(g.width)-.158-.0095)
    return centre+turn @ (tcp-centre), turn @ R, angle


STRUT_RRT_TIMEOUT = 30.         # RRT-Connect nominal budget for strut moves (x PLAN_EXTENDS_PER_S extends; base default 12)


def _strut_goto(motion, targets, *, width, allow_pad_body=None, what="strut move"):
    """Plan (RRT-Connect, longer timeout) and execute to the first reachable target.

    The base planner's 12 s wall-clock budget is short on a loaded host; strut
    moves try every candidate branch with a longer budget before giving up.
    """
    errors = []
    for q in targets:
        try:
            path = motion.plan(q, width=width, allow_pad_body=allow_pad_body, timeout=STRUT_RRT_TIMEOUT)
        except PlanningError as error:
            errors.append(str(error)[:120])
            continue
        motion.execute(path, speed=.25)
        return
    raise PlanningError(f"{what}: no path to any of {len(targets)} branches: {errors[:3]}")


def _strut_approach_branches(motion, pre_p, goal_p, R, body=None, *, clearance=False, attempts=64):
    """IK branches at pre_p whose straight approach to goal_p is clear (as in _preinsert)."""
    out = []
    for candidate in motion.ik(pre_p, R, width=motion.width(), attempts=attempts, max_solutions=16,
                               allow_pad_body=body, clearance=clearance):
        prev, ok = candidate, True
        for u in np.linspace(0, 1, 34)[1:]:
            q = motion._ik_one((1-u)*np.asarray(pre_p)+u*np.asarray(goal_p), R, prev, width=motion.width())
            if q is None or not motion.segment_free(prev, q, width=motion.width(), allow_pad_body=body,
                                                    clearance=clearance):
                ok = False
                break
            prev = q
        if ok:
            out.append(candidate)
    return out


def _strut_pick_branches(cell, motion, name, g, p, R, width, *, prefer_margin=False):
    """IK branches above the pick whose descent, lift and upright turn are all clear.

    The branch the arm arrives on decides where its elbow ends up after the
    turn (a fresh run comes from the ESC and met the staged guard with link4).
    The empty-hand descent, then lift and turn at the closed jaw, are dry-run
    from each candidate branch before the arm moves. With ``prefer_margin`` the
    branches are ordered by the joint-limit margin of the arm after the turn
    (a strut that is stood up and regrasped next must be carried with a wrist
    that is not wound up near its limit: final05's leg_4 arrived with joint 2
    at 1.82 and joint 7 at -5.18 rad and no upright stand carry existed).
    """
    turn_p, turn_R, _ = _strut_turn_pose(cell.pose(name)[:3, :3], p, R, g)
    branches, margins, errors = [], [], []
    for q in motion.ik(p-R[:, 2]*.10, R, width=width, attempts=48, max_solutions=12):
        try:
            qd = motion.line(p, R, width=width, clearance=False, start_q=q, execute=False)[-1]
            ql = motion.line(p+np.array([0., 0., STRUT_LIFT]), R, width=g.width, clearance=False,
                             start_q=qd, execute=False)[-1]
            qt = motion.line(turn_p, turn_R, width=g.width, clearance=True, start_q=ql, execute=False)[-1]
        except PlanningError as error:
            errors.append(str(error)[:120])
            continue
        branches.append(q)
        margins.append(_joint_margin(motion, qt))
    if not branches:
        raise PlanningError(f"{name}: no pick branch with a clear lift and upright turn: {errors[:2]}")
    if prefer_margin:
        order = sorted(range(len(branches)), key=lambda i: -margins[i])
        _log("pick branches", name, "post-turn joint margins", [round(margins[i], 3) for i in order])
        branches = [branches[i] for i in order]
    return branches


def _joint_margin(motion, q):
    """Smallest normalised distance of q to its joint limits (0 at a limit, .5 centred)."""
    lo, hi = motion.limits[:, 0], motion.limits[:, 1]
    return float(np.min(np.minimum(q-lo, hi-q)/(hi-lo)))


def _strut_pick_upright(cell, motion, name, body, g, p, R, width, *, prefer_margin=False):
    """Pick the resting strut from above, lift it and turn it upright, flange up."""
    _prepare_width(motion, width)
    motion.set_width(width)
    motion.wait(1.5)
    width = motion.width()
    _log("preshape", name, "measured_gap", round(width, 4))
    _strut_goto(motion, _strut_pick_branches(cell, motion, name, g, p, R, width, prefer_margin=prefer_margin),
                width=width, what=f"{name} pick")
    motion.line(p, R, width=width, clearance=False, speed=.025)
    motion.close(g.width, squeeze=g.squeeze)
    motion.wait(max(g.settle_s, .5))
    _log("clamp", name, "gap", round(motion.width(), 4), "forces", _grip_forces(cell, name))
    if not _gripped(cell, name):
        raise PlanningError(f"two-sided pad contact missing for {name}")
    motion.hold_body(body)
    t_pick = float(cell.d.time)
    # Lift quickly and turn without an intermediate hold: while the hand points
    # down, gravity drags the pinched shaft towards the fingertips.
    motion.line(p+np.array([0., 0., STRUT_LIFT]), R, allow_pad_body=body, clearance=False, speed=.05,
                hold=10, smooth=True)
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} after lift")
    # Turn the strut upright about its grasped shaft section, flange up, using
    # the measured part axis and the commanded lift pose.
    turn_p, turn_R, angle = _strut_turn_pose(cell.pose(name)[:3, :3], p, R, g)
    motion.line(turn_p, turn_R, allow_pad_body=body, clearance=True, speed=.05, angular_speed=.5, smooth=True)
    # A full 4 mm squeeze makes the faceted shaft rattle and walk towards the
    # fingertips; a lighter command keeps several N per pad.
    motion.set_width(g.width, seconds=.3, squeeze=STRUT_HOLD_SQUEEZE)
    pin, axis = _strut_in_hand(motion, body)
    _log("upright", name, "turn_deg", round(math.degrees(angle), 1), "horizontal_s",
         round(float(cell.d.time)-t_pick, 2), "in_hand_mm", np.round(pin*1000, 2),
         "axis_hand", np.round(axis, 3))
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} while turning upright")


STRUT_REGRASP_STATION = .0445   # pad centre station of a side regrasp (pad tops 1.4 mm under the fillet)
STRUT_REGRASP_DEPTH = 0.        # shaft axis at the TCP: 15 mm of pad remain towards the tip
STRUT_REGRASP_STANDOFF = .06    # straight horizontal approach of the side regrasp
STRUT_STAND_EXIT = (.06, .04, .025)   # open-hand exits after standing a strut (fingertips clear at 20 mm)


def _strut_side_hand(point, theta, station, depth=STRUT_REGRASP_DEPTH):
    """Horizontal hand (tool x up) holding an upright shaft from world direction theta."""
    z = np.array([math.cos(theta), math.sin(theta), 0.])
    x = np.array([0., 0., 1.])
    R = np.column_stack([x, np.cross(z, x), z])
    return np.array([point[0], point[1], point[2]+station])-z*depth, R


def _strut_yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


class _Detached:
    """Plan hypothetical grasps with no held body attached in the planning copy."""
    def __init__(self, motion):
        self.motion = motion

    def __enter__(self):
        self.held, self.motion.held = self.motion.held, None

    def __exit__(self, *exc):
        self.motion.held = self.held


def _strut_mount_angles(cell, motion, mate, body, width):
    """World approach angles with a collision-free upright seat and hover branch."""
    target = cell.coupling.target(mate)
    ins = np.asarray(mate["insertion_axis"], float)
    angles = []
    with _Detached(motion):
        for k in range(24):
            theta = 2*math.pi*k/24
            p, R = _strut_side_hand(target[:3, 3], theta, STRUT_SEATED_STATION)
            if (motion.ik(p, R, width=width, attempts=12, max_solutions=1, allow_pad_body=body) and
                    motion.ik(p-ins*STRUT_HOVER, R, width=width, attempts=12, max_solutions=1,
                              allow_pad_body=body)):
                angles.append(theta)
    return angles


def _strut_stand_spots(cell, name, clearance=.05):
    """Free bench spots near the fixture for standing a strut upright.

    Candidates lie on a 40 mm grid in front of the fixture; a spot is free when
    its 25 mm foot circle keeps `clearance` from every other part's resting
    bounding box and from the fixture pedestal.
    """
    others = []
    for n, inst in cell.graph["instances"].items():
        if n == name or n in cell.coupling.assigned.values():
            continue
        part = cell.kit["parts"][inst["kind"]]
        T = cell.pose(n)
        bb = np.asarray(part["bounds_m"], float)
        corners = np.array([[x, y, z] for x in bb[:, 0] for y in bb[:, 1] for z in bb[:, 2]]) @ T[:3, :3].T+T[:3, 3]
        if corners[:, 2].min() > .12:
            continue   # parts already on the raised assembly
        others.append((corners[:, :2].min(0), corners[:, :2].max(0)))
    fixture = np.asarray(cell.graph["fixture_pos_m"][:2], float)
    others.append((fixture-.025, fixture+.025))
    # Keep the clamped chassis' footprint free too: the strut must leave the
    # stand spot straight upwards for its upright carry route.
    plate = np.asarray(cell.kit["parts"]["base"]["bounds_m"], float)[:, :2]+cell.pose("base")[:2, 3]
    others.append((plate[0]+.02, plate[1]-.02))
    here = cell.pose(name)[:2, 3]
    spots = []
    for x in np.arange(-.26, .07, .04):
        for y in np.arange(-.18, .181, .04):
            xy = np.array([x, y])
            if all(np.any(xy+.0125+clearance < lo) or np.any(xy-.0125-clearance > hi) for lo, hi in others):
                spots.append(np.array([x, y, 0.]))
    spots.sort(key=lambda s: float(np.linalg.norm(s[:2]-here)))
    return spots


def _lowest_point(cell, body):
    """Lowest world z of a body's collision meshes (live pose)."""
    m, d = cell.m, cell.d
    b = m.body(body).id
    low = math.inf
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        k = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k]+m.mesh_vertnum[k]]
        z = v @ d.geom_xmat[g].reshape(3, 3)[2]+d.geom_xpos[g][2]
        low = min(low, float(z.min()))
    return low


def _strut_regrasp(cell, motion, mate, name, body, g):
    """Stand the upright strut on its foot and take a side grasp at a chosen twist.

    The stable-top pick fixes the shaft's twist in the hand, and the 4-fold
    mount then allows only four approach directions at the seat. When none is
    reachable, the strut is set down upright on the bench and grasped again from
    a direction that maps onto a reachable seat approach.
    """
    open_width = min(.085, g.width+.012)
    mounts = _strut_mount_angles(cell, motion, mate, body, g.width)
    _log("regrasp mount angles", name, [round(math.degrees(a)) for a in mounts])
    if not mounts:
        raise PlanningError(f"{name}: no upright seat approach at any twist")
    # Prefer the middle of the widest reachable window (15 deg scan steps).
    step = 2*math.pi/24
    index = {int(round(a/step)) % 24 for a in mounts}
    def depth(theta):
        i, d = int(round(theta/step)) % 24, 0
        while d < 12 and (i+d+1) % 24 in index and (i-d-1) % 24 in index:
            d += 1
        return d
    mounts.sort(key=lambda a: -depth(a))
    leg = cell.pose(name)
    psi0 = _strut_yaw(leg[:3, :3])
    plan = None
    for spot in _strut_stand_spots(cell, name)[:10]:
        # Side-regrasp directions that reach the standing strut at this spot.
        with _Detached(motion):
            free = set()
            for d in range(0, 360, 15):
                p2, R2 = _strut_side_hand(spot, math.radians(d), STRUT_REGRASP_STATION)
                if (motion.ik(p2, R2, width=open_width, attempts=8, max_solutions=1) and
                        motion.ik(p2-R2[:, 2]*STRUT_REGRASP_STANDOFF, R2, width=open_width,
                                  attempts=8, max_solutions=1)):
                    free.add(d)
        _log("regrasp spot", name, np.round(spot[:2], 3), "side directions", sorted(free))
        if not free:
            continue
        for turn_deg in (0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90):
            psi = psi0+math.radians(turn_deg)
            options = []
            for theta3 in mounts:
                for k in range(4):
                    theta2 = math.degrees(theta3+psi-k*math.pi/2) % 360
                    if int(round(theta2/15)) % 24*15 in free:
                        options.append((theta3, k, theta2))
            if not options:
                continue
            # Stand it exactly upright (the hand absorbs any in-hand pivot).
            stand_R = Rotation.from_euler("z", psi).as_matrix()
            place_p, place_R = motion.body_goal(spot+np.array([0., 0., .003]), stand_R, body)
            q_place = motion.ik(place_p, place_R, width=motion.width(), attempts=12, max_solutions=1,
                                allow_pad_body=body)
            if not q_place:
                continue
            try:
                # Final 3 mm descent with the strut, then the open hand's exit.
                q_low = motion.line(place_p-np.array([0., 0., .0028]), place_R, allow_pad_body=body,
                                    clearance=False, start_q=q_place[0], execute=False)[-1]
                with _Detached(motion):
                    motion.line(place_p-np.array([0., 0., .0028])-place_R[:, 2]*STRUT_STAND_EXIT[-1],
                                place_R, width=open_width, start_q=q_low, execute=False)
            except PlanningError:
                continue
            with _Detached(motion):
                for theta3, k, theta2 in options:
                    p2, R2 = _strut_side_hand(spot, math.radians(theta2), STRUT_REGRASP_STATION)
                    q_pre = motion.ik(p2-R2[:, 2]*STRUT_REGRASP_STANDOFF, R2, width=open_width,
                                      attempts=12, max_solutions=1)
                    if not q_pre:
                        continue
                    try:
                        motion.line(p2, R2, width=open_width, clearance=False, start_q=q_pre[0], execute=False)
                    except PlanningError:
                        continue
                    plan = (spot, turn_deg, stand_R, theta3, k)
                    break
            if plan:
                break
        if plan:
            break
    if plan is None:
        raise PlanningError(f"{name}: no stand spot with a side regrasp onto a reachable seat")
    spot, turn_deg, stand_R, theta3, k = plan
    _log("regrasp plan", name, "spot", np.round(spot, 3), "turn_deg", turn_deg, "seat_deg",
         round(math.degrees(theta3)), "yaw_k", k)
    # Set the strut down upright, foot first, with the planned twist.
    touch_p, touch_R = motion.body_goal(spot+np.array([0., 0., .003]), stand_R, body)
    _strut_stand_carry(motion, name, body, spot, stand_R, touch_p, touch_R)
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} during the stand carry")
    motion.line(touch_p, touch_R, allow_pad_body=body, clearance=False, speed=.02)
    # Lower the last few mm until the lowest collider point is 0.1 mm above the
    # bench; letting go then leaves the strut standing on its foot. Re-measure
    # the grasp first so the planning copy sees the strut where it really is.
    motion.hold_body(body)
    tcp, Rh = motion.tool_pose()
    motion.line(tcp-np.array([0., 0., _lowest_point(cell, body)-.0001]), Rh, allow_pad_body=body,
                clearance=False, speed=.004)
    motion.hold_body(None)
    _open_clear(motion, body, open_width, seconds=1.)
    motion.wait(.5)
    tcp, Rh = motion.tool_pose()
    errors = []
    for distance in STRUT_STAND_EXIT:
        try:
            motion.line(tcp-Rh[:, 2]*distance, Rh, allow_pad_body=body, clearance=True, speed=.03)
            break
        except PlanningError as error:
            errors.append(str(error)[:120])
    else:
        raise PlanningError(f"cannot leave the standing {name}: {errors}")
    leg = cell.pose(name)
    tilt = math.degrees(math.acos(float(np.clip(leg[2, 2], -1, 1))))
    _log("stood", name, "pos", np.round(leg[:3, 3], 4), "tilt_deg", round(tilt, 2),
         "twist_deg", round(math.degrees(_strut_yaw(leg[:3, :3])), 2))
    if tilt > 3.:
        raise PlanningError(f"{name} did not stay upright on the bench ({tilt:.1f} deg)")
    # Side regrasp, corrected for the measured standing pose.
    psi = _strut_yaw(leg[:3, :3])
    theta2 = theta3+psi-k*math.pi/2
    p2, R2 = _strut_side_hand(leg[:3, 3], theta2, STRUT_REGRASP_STATION)
    _strut_goto(motion, _strut_approach_branches(motion, p2-R2[:, 2]*STRUT_REGRASP_STANDOFF, p2, R2),
                width=motion.width(), what=f"{name} regrasp approach")
    motion.line(p2, R2, width=motion.width(), clearance=False, speed=.025)
    motion.close(g.width, squeeze=g.squeeze)
    motion.wait(.5)
    _log("regrasp clamp", name, "deg", round(math.degrees(theta2), 1), "gap", round(motion.width(), 4),
         "forces", _grip_forces(cell, name))
    if not _gripped(cell, name):
        raise PlanningError(f"two-sided pad contact missing for {name} after the regrasp")
    motion.hold_body(body)
    # Lift it clear of the bench before the carry is planned: the low regrasp
    # pose among the staged parts leaves the planner little room (final04).
    for rise in (.10, .05, .025):
        try:
            motion.line(p2+np.array([0., 0., rise]), R2, allow_pad_body=body, clearance=rise > .03,
                        speed=.03, smooth=rise > .03)
            break
        except PlanningError as error:
            _log("regrasp lift shortened", name, rise, str(error)[:80])
    motion.set_width(g.width, seconds=.3, squeeze=STRUT_HOLD_SQUEEZE)
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} after the regrasp lift")
    leg = cell.pose(name)
    tcp, Rt = motion.tool_pose()
    G = np.linalg.inv(leg) @ np.r_[np.c_[Rt, tcp], [[0, 0, 0, 1]]]
    return Grasp(G[:3, 3], G[:3, :3], g.width, f"side-regrasp-{math.degrees(theta2):.0f}",
                 target_yaw=k*math.pi/2, squeeze=g.squeeze, settle_s=.5)


STRUT_TILT_OK = 25.             # accept a carry path whose strut stays this close to upright (deg)


def _strut_path_tilt(motion, path, width):
    """Largest strut tilt from vertical along a joint path (planning copy)."""
    b = motion.held[0]
    worst = 0.
    for a, c in zip(path[:-1], path[1:]):
        n = max(1, int(math.ceil(np.max(np.abs(np.asarray(c)-a))/.03)))
        for t in np.linspace(0, 1, n+1):
            motion._fk((1-t)*np.asarray(a)+t*np.asarray(c), width)
            z = motion.pd.xmat[b].reshape(3, 3)[2, 2]
            worst = max(worst, math.degrees(math.acos(float(np.clip(z, -1, 1)))))
    return worst


STRUT_RRT_ITERATIONS = 150      # deterministic search budget of the upright carry RRT
STRUT_RRT_WALL_CAP = 900.       # wall-clock hang guard only (the iteration budget ends the search)


def _strut_upright_rrt(motion, body, width, goal_p, goal_yaw, *, iterations=STRUT_RRT_ITERATIONS):
    """Task-space RRT over the held strut's position and yaw, strut kept upright.

    Nodes are IK solutions (measured grasp) for an upright strut; each edge is a
    short joint segment that is collision-checked and whose strut tilt stays
    within STRUT_TILT_OK. Deterministic: samples come from motion.rng.
    """
    b, _adr, p_rel, rel = motion.held
    rng = motion.rng
    def solve(leg_p, yaw, seed):
        hand_R = Rotation.from_euler("z", yaw).as_matrix() @ rel.T
        q = motion._ik_one(leg_p+hand_R @ (motion.offset-p_rel), hand_R, seed, width=width)
        if q is None or np.max(np.abs(q-seed)) > .5 or motion.collision(q, width, body, True):
            return None
        return q
    def edge(qa, qb, resolution=.015):
        return (motion.segment_free(qa, qb, width=width, allow_pad_body=body, clearance=True,
                                    resolution=resolution) and
                _strut_path_tilt(motion, [qa, qb], width) <= STRUT_TILT_OK)
    q0 = motion.q
    motion._fk(q0, width)
    P = [motion.pd.xpos[b].copy()]
    Y = [_strut_yaw(motion.pd.xmat[b].reshape(3, 3))]
    Q, parent = [q0], [-1]
    lo, hi = np.array([-.30, -.45, .05]), np.array([.32, .45, .40])
    started = time.monotonic()
    for _iteration in range(iterations):
        if time.monotonic()-started > STRUT_RRT_WALL_CAP:
            break
        if rng.random() < .3:
            sp, sy = np.asarray(goal_p, float), goal_yaw
        else:
            sp, sy = rng.uniform(lo, hi), rng.uniform(-math.pi, math.pi)
        dp = sp-np.asarray(P)
        dy = np.array([math.remainder(sy-y, 2*math.pi) for y in Y])
        i = int(np.argmin(np.linalg.norm(dp, axis=1)+.05*np.abs(dy)))
        # RRT-Connect style: keep stepping towards the sample until blocked.
        reached = False
        for _ in range(40):
            dpi = sp-P[i]
            dyi = math.remainder(sy-Y[i], 2*math.pi)
            length = float(np.linalg.norm(dpi))
            if length < 1e-4 and abs(dyi) < 1e-3:
                break
            f = min(1., .03/(length+1e-9), .25/(abs(dyi)+1e-9))
            cp, cy = P[i]+f*dpi, Y[i]+f*dyi
            q = solve(cp, cy, Q[i])
            if q is None or not edge(Q[i], q):
                break
            P.append(cp); Y.append(cy); Q.append(q); parent.append(i)
            i = len(Q)-1
            if np.linalg.norm(cp-goal_p)+.05*abs(math.remainder(cy-goal_yaw, 2*math.pi)) < .03:
                reached = True
                break
        if reached:
            cp, cy, q = P[-1], Y[-1], Q[-1]
            qg = solve(np.asarray(goal_p, float), goal_yaw, q)
            if qg is None or not edge(q, qg):
                continue
            path, k = [qg], len(Q)-1
            while k >= 0:
                path.append(Q[k]); k = parent[k]
            path.reverse()
            out, k = [path[0]], 0
            while k < len(path)-1:
                j = len(path)-1
                while j > k+1 and not edge(path[k], path[j], .007):
                    j -= 1
                if j == k+1 and not edge(path[k], path[j], .007):
                    raise PlanningError("upright carry path failed its fine check")
                out.append(path[j]); k = j
            return out
    raise PlanningError(f"upright carry RRT found no path ({len(Q)} nodes, {time.monotonic()-started:.0f} s)")


def _strut_stand_carry(motion, name, body, spot, stand_R, touch_p, touch_R):
    """Carry the upright strut to 25 mm above its stand spot without tipping it.

    Upright task-space RRT first, once more after an 80 mm lift; the joint-space
    RRT fallback only takes a path whose strut tilt stays within STRUT_TILT_OK
    (final05: an unchecked fallback path tipped the strut to ~80 deg and it slid
    out of the pads).
    """
    goal = spot+np.array([0., 0., .025])
    width = motion.width()
    errors = []
    for attempt in range(2):
        if attempt:
            tcp, R = motion.tool_pose()
            try:
                motion.line(tcp+np.array([0., 0., .08]), R, allow_pad_body=body, clearance=True,
                            speed=.03, smooth=True)
            except PlanningError as error:
                errors.append(str(error)[:120])
                break
        try:
            path = _strut_upright_rrt(motion, body, width, goal, _strut_yaw(stand_R))
        except PlanningError as error:
            errors.append(str(error)[:120])
            _log("stand carry rejected", name, "attempt", attempt, errors[-1])
            continue
        motion.execute(path, speed=.25)
        return
    above_p, _ = motion.body_goal(goal, stand_R, body)
    for q in _strut_approach_branches(motion, above_p, touch_p, touch_R, body, attempts=48):
        try:
            path = motion.plan(q, width=width, allow_pad_body=body, timeout=STRUT_RRT_TIMEOUT)
        except PlanningError as error:
            errors.append(str(error)[:120])
            continue
        tilt = _strut_path_tilt(motion, path, width)
        if tilt > STRUT_TILT_OK:
            errors.append(f"RRT path tilts the strut {tilt:.0f} deg")
            _log("stand carry fallback rejected", name, errors[-1])
            continue
        _log("stand carry fallback rrt", name, "waypoints", len(path), "max_tilt_deg", round(tilt, 1))
        motion.execute(path, speed=.25)
        return
    raise PlanningError(f"{name}: no stand carry keeps the strut upright: {errors[-3:]}")


def _strut_carry_once(cell, motion, mate, body, g, ins, yaws):
    """Carry the upright strut to the hover beneath its seat.

    The strut goes to a point outside the chassis footprint at hover height
    (STRUT_OUTSIDE back along the approach) and then straight in under its
    corner. A plain joint-space RRT path can tip it far over; while the hand
    points down gravity walks the pinched shaft towards the fingertips (a05: 62
    deg tilt, 5 mm walk, 14 deg pivot), so the carry is planned by a task-space
    RRT that keeps the strut upright. Yaws needing the least hand turn go first.
    """
    width = motion.width()
    leg_now = cell.pose(mate["part"])
    yaw_now = _strut_yaw(leg_now[:3, :3])
    def turn(yaw):
        g.target_yaw = yaw
        return abs(math.remainder(_strut_yaw(_target(cell, mate, g)[:3, :3])-yaw_now, 2*math.pi))
    errors = []
    for yaw in sorted(yaws, key=turn):
        g.target_yaw = yaw
        target = _target(cell, mate, g)
        pre = target[:3, 3]-ins*STRUT_HOVER
        hover_p, hover_R = motion.body_goal(pre, target[:3, :3], body)
        out = hover_R[:, 2].copy()
        out[2] = 0.
        out /= np.linalg.norm(out)
        motion.set_mate_contact(["part_"+cell.coupling.carrier(mate)], target[:3, 3], target[:3, :3])
        try:
            path = _strut_upright_rrt(motion, body, width, pre-out*STRUT_OUTSIDE, _strut_yaw(target[:3, :3]))
            motion.line(hover_p, hover_R, allow_pad_body=body, start_q=path[-1], execute=False)
        except PlanningError as error:
            errors.append(str(error)[:160])
            _log("carry yaw rejected", mate["part"], round(yaw, 3), errors[-1])
            continue
        _log("carry", mate["part"], "yaw", round(yaw, 3), "waypoints", len(path),
             "max_tilt_deg", round(_strut_path_tilt(motion, path, width), 1))
        motion.execute(path, speed=.25)
        motion.line(hover_p, hover_R, allow_pad_body=body, clearance=True, speed=.03, smooth=True)
        return
    # Last resort: the plain joint-space RRT with the least strut tilt, and only
    # within STRUT_TILT_OK (a tipped strut walks out of the pads: final05 lost one
    # on an unchecked path).
    best = None
    for yaw in sorted(yaws, key=turn):
        g.target_yaw = yaw
        target = _target(cell, mate, g)
        pre = target[:3, 3]-ins*STRUT_HOVER
        hover_p, hover_R = motion.body_goal(pre, target[:3, :3], body)
        out = hover_R[:, 2].copy()
        out[2] = 0.
        out /= np.linalg.norm(out)
        motion.set_mate_contact(["part_"+cell.coupling.carrier(mate)], target[:3, 3], target[:3, :3])
        for q_out in motion.ik(hover_p-out*STRUT_OUTSIDE, hover_R, width=width, attempts=48, max_solutions=4,
                               allow_pad_body=body):
            try:
                motion.line(hover_p, hover_R, allow_pad_body=body, start_q=q_out, execute=False)
                path = motion.plan(q_out, width=width, allow_pad_body=body, clearance=True,
                                   timeout=STRUT_RRT_TIMEOUT)
            except PlanningError as error:
                errors.append(str(error)[:160])
                continue
            tilt = _strut_path_tilt(motion, path, width)
            if tilt > STRUT_TILT_OK:
                errors.append(f"RRT path tilts the strut {tilt:.0f} deg")
                continue
            if best is None or tilt < best[0]:
                best = (tilt, yaw, path, hover_p, hover_R)
    if best is None:
        raise PlanningError(f"no carry path for {mate['part']}: {errors[:3]}")
    tilt, yaw, path, hover_p, hover_R = best
    g.target_yaw = yaw
    target = _target(cell, mate, g)
    motion.set_mate_contact(["part_"+cell.coupling.carrier(mate)], target[:3, 3], target[:3, :3])
    _log("carry fallback rrt", mate["part"], "yaw", round(yaw, 3), "waypoints", len(path), "max_tilt_deg", round(tilt, 1))
    motion.execute(path, speed=.25)
    motion.line(hover_p, hover_R, allow_pad_body=body, clearance=True, speed=.03, smooth=True)


def _strut_carry(cell, motion, mate, body, g, ins, yaws):
    """Carry the upright strut to its hover; if no path exists, lift it 80 mm and retry once."""
    try:
        return _strut_carry_once(cell, motion, mate, body, g, ins, yaws)
    except PlanningError as error:
        _log("carry retry after lift", mate["part"], str(error)[:160])
    tcp, R = motion.tool_pose()
    motion.line(tcp+np.array([0., 0., .08]), R, allow_pad_body=body, clearance=True, speed=.03, smooth=True)
    return _strut_carry_once(cell, motion, mate, body, g, ins, yaws)


def _place_strut(cell, motion, mate, *, choice=0, wall_deadline=math.inf):
    name, body = mate["part"], "part_"+mate["part"]
    T = cell.pose(name)
    carrier = cell.coupling.carrier(mate)
    contact_carriers = ["part_"+carrier]
    ins = cell.pose(carrier)[:3, :3] @ np.asarray(mate["insertion_axis"], float)
    label = f"stable-top-{STRUT_PICK_STATION:.3f}"
    candidates = [g for g in grasps(cell, name) if g.label.startswith(label)]
    if not candidates:
        raise PlanningError(f"no stable-top shaft grasp for {name}")
    g0 = candidates[0]
    p, R = _world(T, g0)
    width = min(.085, g0.width+.012)
    if not motion.ik(p, R, width=width, attempts=10, max_solutions=1, clearance=False):
        raise PlanningError(f"{name}: pick pose unreachable")
    if not motion.ik(p-R[:, 2]*.10, R, width=width, attempts=12, max_solutions=1):
        raise PlanningError(f"{name}: pick approach unreachable")
    # Upright insertion branches for the seated grasp, checked before touching the strut.
    yaws = []
    for g in candidates:
        seated = _strut_seated(g)
        target = _target(cell, mate, seated)
        gp, gR = _world(target, seated)
        hover = motion.ik(gp-ins*STRUT_HOVER, gR, width=g.width, attempts=16, max_solutions=1, allow_pad_body=body)
        seat = hover and motion.ik(gp, gR, width=g.width, attempts=16, max_solutions=1, allow_pad_body=body)
        _log("strut branch", name, g.label, "hover", bool(hover), "seat", bool(seat))
        if hover and seat:
            yaws.append(g.target_yaw)
    if choice:
        yaws = yaws[choice % len(yaws):]+yaws[:choice % len(yaws)]
    g = g0
    _log("pick", name, g.label, "width", round(g.width, 4), "pose", np.round(p, 4), "yaws", np.round(yaws, 3))
    _strut_pick_upright(cell, motion, name, body, g, p, R, width, prefer_margin=not yaws)
    if not yaws:
        # No seat approach exists for the twist the bench pose gives the
        # stable-top grasp: stand the strut up and take it again from the side.
        g = _strut_regrasp(cell, motion, mate, name, body, g)
        yaws = [g.target_yaw]
    _strut_carry(cell, motion, mate, body, g, ins, yaws)
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} during transport")
    target = _target(cell, mate, g)
    motion.set_mate_contact(contact_carriers, target[:3, 3], target[:3, :3])
    pin, axis = _strut_in_hand(motion, body)
    _log("hover", name, "in_hand_mm", np.round(pin*1000, 2), "axis_hand", np.round(axis, 3))
    motion.align_body(target[:3, 3]-ins*STRUT_HOVER, target[:3, :3], body=body,
                      tolerance=.0005, angle=.008, max_passes=10)
    if not _gripped(cell, name):
        raise PlanningError(f"lost grip on {name} during hover alignment")
    flush = target[:3, 3]
    pp, rr = motion.body_goal(flush, target[:3, :3], body)
    motion.line(pp, rr, allow_pad_body=body, clearance=False, speed=.015)
    pin, axis = _strut_in_hand(motion, body)
    _log("flush", name, "error_mm", np.round(1000*(cell.pose(name)[:3, 3]-target[:3, 3]), 3),
         "in_hand_mm", np.round(pin*1000, 2), "state", cell.coupling.state[mate["id"]])
    # Press: same measured grasp, PRESS beyond flush along the insertion axis.
    motion.line(pp+ins*STRUT_PRESS, rr, allow_pad_body=body, clearance=False, speed=.005)
    q = motion.q
    for step in range(150):
        if time.monotonic() > wall_deadline:
            raise TimeoutError("reference wall cap during seat hold")
        motion._drive(q); cell.tick()
        if step % 25 == 0:
            _log("press", name, step, cell.coupling.state[mate["id"]], "error_mm",
                 np.round(1000*(cell.pose(name)[:3, 3]-target[:3, 3]), 3), "forces", _grip_forces(cell, name))
        if cell.coupling.state[mate["id"]] in ("LOCKED", "LOCKED_OFF") and step >= 10:
            break
    state = cell.coupling.state[mate["id"]]
    event = next((e for e in cell.events[::-1] if e.get("mate") == mate["id"]), {})
    _log("seat", name, state, "error_mm", round(1000*float(np.linalg.norm(cell.pose(name)[:3, 3]-target[:3, 3])), 3),
         "lock_error_mm", round(1000*event.get("error_m", float("nan")), 3), "lock_angle_deg",
         round(event.get("angle_deg", float("nan")), 3))
    if state != "LOCKED":
        raise PlanningError(f"{name} did not fasten correctly ({state})")
    motion.hold_body(None)
    released_width = _open_clear(motion, body, min(.085, g.width+.012))
    pp, rr = motion.tool_pose()
    errors = []
    for distance in (.08, .06, .04):
        try:
            motion.line(pp-rr[:, 2]*distance, rr, allow_pad_body=body, clearance=True, speed=.03)
            break
        except PlanningError as e:
            errors.append(str(e))
    else:
        raise PlanningError(f"cannot retract from {name}: {errors}")
    if cell.violations:
        raise PlanningError(f"safety failure after {name}: {cell.violations[:2]}")
    return dict(part=name, grasp=g.label, candidate=0, sim_time_s=float(cell.d.time), state=state,
                lock_error_mm=1000*event.get("error_m", float("nan")), lock_angle_deg=event.get("angle_deg", float("nan")))


def place(cell, motion, mate, *, choice=0, wall_deadline=math.inf):
    name, body = mate["part"], "part_"+mate["part"]
    kind=cell.graph["instances"][name]["kind"]
    if kind == "landing_strut":
        return _place_strut(cell, motion, mate, choice=choice, wall_deadline=wall_deadline)
    is_prop = kind in ("prop_cw","prop_ccw")
    pre_distance=.040 if kind == "lidar" else .065
    retreat_distance=.040 if kind == "lidar" else .080
    T = cell.pose(name)
    carrier=cell.coupling.carrier(mate)
    contact_carriers=["part_"+carrier]
    if name == "sensor_cap":
        # The roof seats on both installed cheek rims; only its near-seat
        # contact envelope is exempted, exactly as for the named carrier.
        contact_carriers.append("part_"+cell.coupling.assigned.get("mount_cheek_right","cheek_right"))
    if kind == "lidar":
        # Seated on the cap plate, the LiDAR base is only the cap's 1.5 mm
        # above both cheek rims (inside the 2 mm held-part planning margin).
        # Exempt the cheeks only in the same near-seat envelope as the carrier.
        for key,default in (("mount_cheek_left","cheek_left"),("mount_cheek_right","cheek_right")):
            contact_carriers.append("part_"+cell.coupling.assigned.get(key,default))
    nominal_carrier=cell.graph["instances"].get(mate["carrier"].split("/")[0],{})
    nominal_R=Rotation.from_euler("xyz",nominal_carrier.get("target_euler_xyz_deg",[0,0,0]),degrees=True).as_matrix()
    # Graph arrows are in the nominal fixture frame. Follow carrier drift,
    # without applying its nominal CAD rotation a second time.
    ins = cell.pose(carrier)[:3,:3] @ nominal_R.T @ np.asarray(mate["insertion_axis"],float)
    selected = None
    count = 0
    pick_prepared = False
    pickup_errors = []
    candidates = grasps(cell,name)
    if kind in ("camera", "flight_controller"):
        candidates = _least_travel_first(cell,motion,mate,candidates,-ins*pre_distance)
    for i,g in enumerate(candidates):
        if time.monotonic() > wall_deadline:
            raise TimeoutError("reference wall cap during grasp search")
        p,R = _world(T,g)
        target = _target(cell,mate,g)
        gp,gR = _world(target,g)
        width = .085 if kind == "sensor_cap" and g.width>.020 else min(.085,g.width+.012)
        # Test both ends; the actual transport is planned again with its measured
        # held part after pickup. Final insertion is checked outside-in.
        pick=motion.ik(p,R,width=width,attempts=10,max_solutions=1,clearance=False)
        if not pick:
            continue
        if not _grip_clear(motion,pick[0],g,body,name):
            continue
        if not motion.ik(gp-ins*pre_distance,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body):
            continue
        if not motion.ik(gp,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body,clearance=False):
            continue
        if not motion.ik(p-R[:,2]*.10,R,width=width,attempts=12,max_solutions=1):
            continue
        if count < choice:
            count += 1; continue
        if is_prop or kind == "camera":
            # A blade's endpoints can be clear while its oblique pickup path
            # clips a neighbouring staged component. Try the equivalent blade
            # before touching the prop, with the actual settled jaw aperture.
            # The camera's oblique approach down its board plane passes the
            # installed guard ring; take a candidate whose whole approach is clear.
            try:
                _prepare_width(motion,width)
                motion.set_width(width)
                motion.wait(1.5)
                width=motion.width()
                _preinsert(motion,p-R[:,2]*.10,p,R,None,clearance=False,
                           transport_clearance=True,attempts=128,
                           speed=PROP_FREE_SPEED if is_prop else .25)
            except PlanningError as error:
                if cell.violations:
                    raise
                pickup_errors.append(str(error))
                _log("reject pickup path",name,g.label,str(error))
                continue
            pick_prepared=True
        selected=(i,g,p,R,gp,gR,width);break
    if selected is None:
        raise PlanningError(f"no pick+preinsert grasp for {name}; pickup failures {pickup_errors[:3]}; last contacts {motion.last_contacts[:3]}")
    i,g,p,R,gp,gR,width=selected
    _log("pick",name,"candidate",i,g.label,"width",round(g.width,4),"pose",np.round(p,4))
    if not pick_prepared:
        _prepare_width(motion,width)
        motion.set_width(width)
        # Large opening changes continue moving after the command ramp.
        motion.wait(1.5)
        width=motion.width()
    _log("preshape",name,"measured_gap",round(width,4))
    approach = R[:,2]
    if not pick_prepared:
        _preinsert(motion,p-approach*.10,p,R,None,clearance=False,transport_clearance=True)
    if is_prop:
        # Light part on pins: close most of the checked approach briskly and
        # only the last 20 mm slowly.
        motion.line(p-approach*.02,R,width=width,clearance=False,speed=.06)
    motion.line(p,R,width=width,clearance=False,speed=.025 if not is_prop else .02)
    motion.close(g.width,squeeze=g.squeeze)
    if is_prop:
        _prop_squeeze(cell,motion,name,max_s=3.0)
    else:
        motion.wait(max(g.settle_s,.5))
    _log("clamp",name,"gap",round(motion.width(),4),"forces",_grip_forces(cell,name))
    if not _gripped(cell,name):
        raise PlanningError(f"two-sided pad contact missing for {name}")
    motion.hold_body(body)
    if is_prop:
        # On its pins the blade yields by turning while the jaws creep shut,
        # so the pad force at lift-off varies (0.1-1.2 N). Lift 3 mm clear of
        # the pins and let the blade settle into the pads before the full lift.
        motion.line(p-approach*.003,R,allow_pad_body=body,clearance=False,speed=.01)
        _prop_squeeze(cell,motion,name,min_force=1.0,max_s=2.0)
        if not _gripped(cell,name):
            raise PlanningError(f"lost grip on {name} at lift-off")
    # The pinched blade pivots slowly in the jaws (see _prop_servo); do not
    # linger in the lift.
    motion.line(p-approach*.10,R,allow_pad_body=body,clearance=False,speed=.05 if is_prop else .025)
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} after lift")
    base_yaw=g.target_yaw
    symmetry=int(mate.get("axial_symmetry",1))
    free_yaw=symmetry==0
    if free_yaw:
        target_yaws=(base_yaw,base_yaw+math.pi/2,base_yaw-math.pi/2,base_yaw+math.pi)
    elif symmetry>1:
        target_yaws=tuple(base_yaw+2*math.pi*k/symmetry for k in range(symmetry))
    else:
        target_yaws=(base_yaw,)
    transport_errors=[]
    # Propellers first try only hover branches verified for the whole
    # insertion (_prop_preinsert); the plain transport is the fallback.
    for verified in ((True,False) if is_prop else (False,)):
        # The verified search covers every free-yaw turn itself: run it once.
        for yaw in (target_yaws[:1] if verified else target_yaws):
            g.target_yaw=yaw
            target = _target(cell,mate,g)
            pre = target[:3,3]-ins*pre_distance
            pp,rr=motion.body_goal(pre,target[:3,:3],body)
            goal_p,goal_R=motion.body_goal(target[:3,3],target[:3,:3],body)
            motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3],axis_only=free_yaw)
            _log("transport",name,"pre",np.round(pre,4),"yaw",round(yaw,3))
            retreat=-goal_R[:,2]*(.06 if kind == "landing_strut" else retreat_distance)
            try:
                if verified:
                    turn,worst,pivots=_prop_preinsert(motion,body,target,pre_distance,min(.085,g.width+.012),speed=.65)
                    # The prop's final yaw is free; record the chosen turn.
                    g.target_yaw=yaw+math.radians(turn)
                    _log("prop branch",name,"verified transport branch: yaw turn deg",turn,"pivot check deg",pivots,
                         "min self-distance mm",round(1000*worst,2))
                else:
                    _preinsert(motion,pp,goal_p,goal_R,body,retreat=retreat,open_width=min(.085,g.width+.012),
                               clearance=not is_prop,transport_clearance=True,
                               attempts=96 if is_prop else 64,speed=.65 if is_prop else .25)
                break
            except PlanningError as error:
                transport_errors.append(str(error))
        else:
            continue
        break
    else:
        raise PlanningError(f"all axial orientations failed for {name}: {transport_errors}")
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} during transport")
    target=_target(cell,mate,g)
    motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3],axis_only=free_yaw)
    if is_prop:
        q=_prop_insert(cell,motion,mate,name,body,target,ins,pre_distance,wall_deadline=wall_deadline,
                       open_width=min(.085,g.width+.012))
        _prop_seat(cell,motion,mate,name,body,q,min(.085,g.width+.012),wall_deadline)
    else:
        if kind == "landing_strut" or cell.kit["parts"][kind].get("grasp_mode") == "surface":
            # Real contact grasps can creep during transport. Correct the full
            # measured pose while still clear of the receiving surface, rather
            # than trying to rotate a misaligned flange after it touches the base.
            if kind == "lidar":
                _lidar_align(motion,target[:3,3]-ins*pre_distance,target[:3,:3],body,tolerance=.0007,max_passes=10)
            else:
                motion.align_body(target[:3,3]-ins*pre_distance,target[:3,:3],body=body,
                                  tolerance=.0007,angle=.01,max_passes=10,axis_only=free_yaw)
            if not _gripped(cell,name):
                raise PlanningError(f"lost grip on {name} during hover alignment")
        pp,rr=motion.body_goal(target[:3,3],target[:3,:3],body)
        try:
            motion.line(pp,rr,allow_pad_body=body,clearance=False,execute=False)
        except PlanningError as error:
            # The part moved in the hand during the carry (re-measured by
            # body_goal): from the transport plan's IK branch the straight
            # descent to the re-measured seat goal is blocked. Choose again,
            # as for the transport, an IK branch at the hover whose straight
            # descent is clear, move there, then descend. Planning-only check
            # first: a clear descent is executed exactly as before.
            _log("descent blocked from the transport branch:",name,str(error)[:160])
            hp,hr=motion.body_goal(target[:3,3]-ins*pre_distance,target[:3,:3],body)
            _preinsert(motion,hp,pp,rr,body,clearance=False,transport_clearance=True,attempts=96,speed=.25)
            pp,rr=motion.body_goal(target[:3,3],target[:3,:3],body)
        motion.line(pp,rr,allow_pad_body=body,clearance=False,speed=.015)
        if cell.coupling.state[mate["id"]] not in ("LOCKED","LOCKED_OFF"):
            if kind == "lidar":
                _lidar_align(motion,target[:3,3],target[:3,:3],body,tolerance=.0005,max_passes=8)
            else:
                motion.align_body(target[:3,3],target[:3,:3],body=body,
                                  tolerance=.0005,angle=.01,max_passes=8,axis_only=free_yaw)
        q=motion.q
        for step in range(80):
            if time.monotonic()>wall_deadline:
                raise TimeoutError("reference wall cap during seat hold")
            motion._drive(q);cell.tick()
    state=cell.coupling.state[mate["id"]]
    _log("seat",name,state,"error_mm",1000*np.linalg.norm(cell.pose(name)[:3,3]-target[:3,3]))
    if state != "LOCKED":
        raise PlanningError(f"{name} did not fasten correctly ({state})")
    motion.hold_body(None)
    released_width=min(.085,g.width+.012)
    if is_prop:
        released_width=_prop_open(motion,body,released_width)
    else:
        released_width=_open_clear(motion,body,released_width)
    if is_prop:
        # Linkage motion continues after the opening ramp; check the retreat
        # at its final aperture so a widening knuckle cannot graze a neighbour.
        motion.wait(1.0)
    pp,rr=motion.tool_pose()
    if is_prop:
        _prop_exit(cell,motion,name,released_width)
    elif kind == "landing_strut":
        # A long back-out can fold into the arm or approach another propeller.
        # Precheck shorter escapes, then let RRT plan the next free-space move.
        errors=[]
        for distance in (.08,.06,.04):
            try:
                motion.line(pp-rr[:,2]*distance,rr,width=None,
                            allow_pad_body=body,clearance=True,speed=.03)
                break
            except PlanningError as e:
                errors.append(str(e))
        else:
            raise PlanningError(f"cannot retract from {name}: {errors}")
    else:
        motion.line(pp-rr[:,2]*retreat_distance,rr,allow_pad_body=body,clearance=False,speed=.03)
    if cell.violations:
        raise PlanningError(f"safety failure after {name}: {cell.violations[:2]}")
    return dict(part=name,grasp=g.label,candidate=i,sim_time_s=float(cell.d.time),state=state)


def assemble(cell, *, only=None, choices=None, wall_cap_s=7200):
    deadline=time.monotonic()+wall_cap_s
    motion=Motion(cell)
    choices=choices or {}
    mates={m["part"]:m for m in cell.graph["mates"]}
    order = ["esc"] + [n for i in range(1,5) for n in (f"leg_{i}",f"motor_{i}")] + [
        f"prop_{i}" for i in range(1,5)] + [
        "guard","camera","flight_controller","compute","compute_cover","battery",
        "cheek_left","cheek_right","sensor_cap","lidar"]
    if only is not None:
        order = list(only)
    results=[]
    for name in order:
        if time.monotonic()>deadline:
            raise TimeoutError("reference assembly wall cap")
        results.append(place(cell,motion,mates[name],choice=choices.get(name,0),wall_deadline=deadline))
    result=cell.result();result["placements"]=results
    return result
