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

from motion import Motion, PlanningError, DOWN, body_subtree


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


def grasps(cell, name):
    """Antipodal box/cylinder grasps, followed by collision-mesh ray candidates."""
    kind = cell.graph["instances"][name]["kind"]
    part = cell.kit["parts"][kind]
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
                if candidate is not None:yield candidate
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
        # A measured straight section of the actual upper rim fits the stock
        # 30 mm pads and keeps the wrist above the installed guard.
        for yaw in (0.,math.pi):
            R=Rotation.from_euler("z",yaw).as_matrix() @ DOWN
            yield Grasp(np.array([-.010,.03815193014894713,.012359397152226951]),R,
                        .007750342853049097,f"upper-straight-rim-{yaw:.2f}")
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


def _world(T, grasp):
    return T[:3,3]+T[:3,:3] @ grasp.p, T[:3,:3] @ grasp.R


def _target(cell,mate,grasp):
    T=cell.coupling.target(mate).copy()
    if grasp.target_yaw:
        T[:3,:3] = T[:3,:3] @ Rotation.from_euler("z",grasp.target_yaw).as_matrix()
    return T


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


def place(cell, motion, mate, *, choice=0, wall_deadline=math.inf):
    name, body = mate["part"], "part_"+mate["part"]
    kind=cell.graph["instances"][name]["kind"]
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
    nominal_carrier=cell.graph["instances"].get(mate["carrier"].split("/")[0],{})
    nominal_R=Rotation.from_euler("xyz",nominal_carrier.get("target_euler_xyz_deg",[0,0,0]),degrees=True).as_matrix()
    # Graph arrows are in the nominal fixture frame. Follow carrier drift,
    # without applying its nominal CAD rotation a second time.
    ins = cell.pose(carrier)[:3,:3] @ nominal_R.T @ np.asarray(mate["insertion_axis"],float)
    selected = None
    count = 0
    pick_prepared = False
    pickup_errors = []
    for i,g in enumerate(grasps(cell,name)):
        if time.monotonic() > wall_deadline:
            raise TimeoutError("reference wall cap during grasp search")
        p,R = _world(T,g)
        target = _target(cell,mate,g)
        gp,gR = _world(target,g)
        width = .085 if kind == "sensor_cap" and g.width>.020 else min(.085,g.width+.012)
        # Test both ends; the actual transport is planned again with its measured
        # held part after pickup. Final insertion is checked outside-in.
        if not motion.ik(p,R,width=width,attempts=10,max_solutions=1,clearance=False):
            continue
        if not motion.ik(gp-ins*pre_distance,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body):
            continue
        if not motion.ik(gp,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body,clearance=False):
            continue
        if not motion.ik(p-R[:,2]*.10,R,width=width,attempts=12,max_solutions=1):
            continue
        if count < choice:
            count += 1; continue
        if is_prop:
            # A blade's endpoints can be clear while its oblique pickup path
            # clips a neighbouring staged component. Try the equivalent blade
            # before touching the prop, with the actual settled jaw aperture.
            try:
                _prepare_width(motion,width)
                motion.set_width(width)
                motion.wait(1.5)
                width=motion.width()
                _preinsert(motion,p-R[:,2]*.10,p,R,None,clearance=False,
                           transport_clearance=True,attempts=128)
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
    motion.line(p,R,width=width,clearance=False,speed=.025)
    motion.close(g.width,squeeze=g.squeeze)
    motion.wait(max(g.settle_s,1.0 if kind in ("prop_cw","prop_ccw") else .5))
    _log("clamp",name,"gap",round(motion.width(),4),"forces",_grip_forces(cell,name))
    if not _gripped(cell,name):
        raise PlanningError(f"two-sided pad contact missing for {name}")
    motion.hold_body(body)
    motion.line(p-approach*.10,R,allow_pad_body=body,clearance=False,speed=.025)
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
    for yaw in target_yaws:
        g.target_yaw=yaw
        target = _target(cell,mate,g)
        pre = target[:3,3]-ins*pre_distance
        pp,rr=motion.body_goal(pre,target[:3,:3],body)
        goal_p,goal_R=motion.body_goal(target[:3,3],target[:3,:3],body)
        motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3],axis_only=free_yaw)
        _log("transport",name,"pre",np.round(pre,4),"yaw",round(yaw,3))
        retreat=-goal_R[:,2]*(.06 if kind == "landing_strut" else retreat_distance)
        try:
            _preinsert(motion,pp,goal_p,goal_R,body,retreat=retreat,open_width=min(.085,g.width+.012),
                       clearance=not is_prop,transport_clearance=True,
                       attempts=96 if is_prop else 64,speed=.65 if is_prop else .25)
            break
        except PlanningError as error:
            transport_errors.append(str(error))
    else:
        raise PlanningError(f"all axial orientations failed for {name}: {transport_errors}")
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} during transport")
    target=_target(cell,mate,g)
    motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3],axis_only=free_yaw)
    if is_prop or kind == "landing_strut" or cell.kit["parts"][kind].get("grasp_mode") == "surface":
        # Real contact grasps can creep during transport. Correct the full
        # measured pose while still clear of the receiving surface, rather
        # than trying to rotate a misaligned flange after it touches the base.
        motion.align_body(target[:3,3]-ins*pre_distance,target[:3,:3],body=body,
                          tolerance=.0015 if is_prop else .0007,angle=.01,max_passes=10,axis_only=free_yaw)
        if not _gripped(cell,name):
            raise PlanningError(f"lost grip on {name} during hover alignment")
    pp,rr=motion.body_goal(target[:3,3],target[:3,:3],body)
    motion.line(pp,rr,allow_pad_body=body,clearance=False,speed=.015)
    if cell.coupling.state[mate["id"]] not in ("LOCKED","LOCKED_OFF"):
        motion.align_body(target[:3,3],target[:3,:3],body=body,
                          tolerance=.0007 if is_prop else .0005,angle=.01,
                          max_passes=15 if is_prop else 8,axis_only=free_yaw)
    q=motion.q
    for step in range(600 if is_prop else 80):
        if time.monotonic()>wall_deadline:
            raise TimeoutError("reference wall cap during seat hold")
        motion._drive(q);cell.tick()
        if is_prop:
            if cell.coupling.state[mate["id"]] in ("LOCKED","LOCKED_OFF"):
                break
            if step == 149:
                # Hold the measured thin-section aperture instead of leaving
                # the jaws commanded fully shut while the fastening settles.
                motion.set_width(max(0.,motion.width()-.001),seconds=.3)
    state=cell.coupling.state[mate["id"]]
    _log("seat",name,state,"error_mm",1000*np.linalg.norm(cell.pose(name)[:3,3]-target[:3,3]))
    if state != "LOCKED":
        raise PlanningError(f"{name} did not fasten correctly ({state})")
    motion.hold_body(None)
    released_width=min(.085,g.width+.012)
    motion.set_width(released_width)
    if is_prop:
        # Linkage motion continues after the opening ramp; check the retreat
        # at its final aperture so a widening knuckle cannot graze a neighbour.
        motion.wait(1.5)
    pp,rr=motion.tool_pose()
    if kind == "landing_strut" or is_prop:
        # A long back-out can fold into the arm or approach another propeller.
        # Precheck shorter escapes, then let RRT plan the next free-space move.
        errors=[]
        for distance in ((.08,.06,.04,.03,.02) if is_prop else (.08,.06,.04)):
            try:
                motion.line(pp-rr[:,2]*distance,rr,width=max(released_width,motion.width()) if is_prop else None,
                            allow_pad_body=body,clearance=not is_prop,speed=.03)
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
