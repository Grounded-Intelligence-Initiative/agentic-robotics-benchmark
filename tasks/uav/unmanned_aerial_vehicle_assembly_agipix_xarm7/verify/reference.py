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

from motion import Motion, PlanningError, DOWN


@dataclass
class Grasp:
    p: np.ndarray
    R: np.ndarray
    width: float
    label: str
    target_yaw: float = 0.


def _log(*args):
    print("[reference]", *args, file=sys.stderr, flush=True)


def _tip(width):
    a = Motion.angle_for_width(width)
    return .059098 + math.sin(a)*.035465 + math.cos(a)*.042039 + .0595


def grasps(cell, name):
    """Antipodal box/cylinder grasps, followed by collision-mesh ray candidates."""
    kind = cell.graph["instances"][name]["kind"]
    part = cell.kit["parts"][kind]
    bb = np.asarray(part["bounds_m"])
    center = (bb[0]+bb[1])/2
    if kind == "landing_strut":
        # The hull's stable rest includes a harmless axial twist. Derive the
        # radial grasp from that measured rest instead of assuming a box face.
        Rtop = cell.pose(name)[:3,:3].T @ DOWN
        for target_yaw in (0.,math.pi/2,-math.pi/2,math.pi):
            for height in (.040,.035,.045):
                yield Grasp(np.array([0.,0.,height])-Rtop[:,2]*.008,Rtop,.008,
                            f"stable-top-{height:.3f}-yaw{target_yaw:.2f}",target_yaw)
        # Grasp the 8 mm shaft beneath the upper flange. A horizontal hand can
        # reach under the fixed base and insert the upright leg from below.
        for yaw in (-math.pi/2, 0, math.pi/2, math.pi):
            R = Rotation.from_euler("z", yaw).as_matrix() @ np.array([[0,0,1],[0,-1,0],[1,0,0.]])
            for height in (.040, .045, .035, .030):
                yield Grasp(np.array([0.,0.,height])-R[:,2]*.008, R, .008, f"shaft-side-{yaw:.2f}-{height:.3f}")
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
    bid = cell.ids[name]
    sides={"left":0.,"right":0.}
    for i,c in enumerate(cell.d.contact):
        for g,h in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
            gn = cell.m.geom(int(g)).name or ""
            if cell.m.geom_bodyid[h] == bid and "finger_pad" in gn:
                force=np.zeros(6)
                mujoco.mj_contactForce(cell.m,cell.d,i,force)
                sides[gn.split("_")[0]]+=max(0.,float(force[0]))
    return sides


def _gripped(cell, name):
    return min(_grip_forces(cell,name).values())>.05


def _preinsert(motion, pre_p, goal_p, R, body, *, retreat=None, open_width=None, attempts=64, clearance=True, transport_clearance=None):
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
                    if q is None or not motion.segment_free(prev,q,width=open_width,allow_pad_body=body,clearance=True):
                        ok=False;break
                    prev=q
            finally:
                motion.held=saved_held
            if not ok:
                continue
        try:
            motion.move_joints(candidate,width=motion.width(),allow_pad_body=body,speed=.25,clearance=transport_clearance)
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
        if not motion.ik(gp-ins*.06,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body):
            continue
        if not motion.ik(gp,gR,width=g.width,attempts=10,max_solutions=1,allow_pad_body=body,clearance=False):
            continue
        if not motion.ik(p-R[:,2]*.10,R,width=width,attempts=12,max_solutions=1):
            continue
        if count < choice:
            count += 1; continue
        selected=(i,g,p,R,gp,gR,width);break
    if selected is None:
        raise PlanningError(f"no pick+preinsert grasp for {name}; last contacts {motion.last_contacts[:3]}")
    i,g,p,R,gp,gR,width=selected
    _log("pick",name,"candidate",i,g.label,"width",round(g.width,4),"pose",np.round(p,4))
    _prepare_width(motion,width)
    motion.set_width(width)
    if kind == "sensor_cap" and g.width>.020:
        motion.wait(1.5)
    width=motion.width()
    approach = R[:,2]
    _preinsert(motion,p-approach*.10,p,R,None,clearance=False,transport_clearance=True)
    motion.line(p,R,width=width,clearance=False,speed=.025)
    motion.close(g.width,squeeze=.006)
    motion.wait(1.0 if kind in ("prop_cw","prop_ccw") else .5)
    _log("clamp",name,"gap",round(motion.width(),4),"forces",_grip_forces(cell,name))
    if not _gripped(cell,name):
        raise PlanningError(f"two-sided pad contact missing for {name}")
    motion.hold_body(body)
    motion.line(p-approach*.10,R,allow_pad_body=body,clearance=False,speed=.025)
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} after lift")
    base_yaw=g.target_yaw
    free_yaw=cell.graph["instances"][name]["kind"] in ("landing_strut","motor","prop_cw","prop_ccw")
    transport_errors=[]
    for yaw in ((base_yaw,base_yaw+math.pi/2,base_yaw-math.pi/2,base_yaw+math.pi) if free_yaw else (base_yaw,)):
        g.target_yaw=yaw
        target = _target(cell,mate,g)
        pre = target[:3,3]-ins*.065
        pp,rr=motion.body_goal(pre,target[:3,:3],body)
        goal_p,goal_R=motion.body_goal(target[:3,3],target[:3,:3],body)
        motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3])
        _log("transport",name,"pre",np.round(pre,4),"yaw",round(yaw,3))
        retreat=-goal_R[:,2]*(.06 if cell.graph["instances"][name]["kind"] == "landing_strut" else .08)
        try:
            _preinsert(motion,pp,goal_p,goal_R,body,retreat=retreat,open_width=min(.085,g.width+.012))
            break
        except PlanningError as error:
            transport_errors.append(str(error))
    else:
        raise PlanningError(f"all axial orientations failed for {name}: {transport_errors}")
    if not _gripped(cell,name):
        raise PlanningError(f"lost grip on {name} during transport")
    target=_target(cell,mate,g)
    motion.set_mate_contact(contact_carriers,target[:3,3],target[:3,:3])
    pp,rr=motion.body_goal(target[:3,3],target[:3,:3],body)
    motion.line(pp,rr,allow_pad_body=body,clearance=False,speed=.015)
    if cell.coupling.state[mate["id"]] not in ("LOCKED","LOCKED_OFF"):
        motion.align_body(target[:3,3],target[:3,:3],body=body,tolerance=.0005,angle=.01)
    q=motion.q
    for _ in range(80):
        if time.monotonic()>wall_deadline:
            raise TimeoutError("reference wall cap during seat hold")
        motion._drive(q);cell.tick()
    state=cell.coupling.state[mate["id"]]
    _log("seat",name,state,"error_mm",1000*np.linalg.norm(cell.pose(name)[:3,3]-target[:3,3]))
    if state != "LOCKED":
        raise PlanningError(f"{name} did not fasten correctly ({state})")
    motion.hold_body(None)
    motion.set_width(min(.085,g.width+.012))
    pp,rr=motion.tool_pose()
    if cell.graph["instances"][name]["kind"] == "landing_strut":
        # A long straight back-out can fold the wrist into the upper arm.
        # Precheck successively shorter escapes, then let RRT plan the next move.
        errors=[]
        for distance in (.08,.06,.04):
            try:
                motion.line(pp-rr[:,2]*distance,rr,allow_pad_body=body,clearance=True,speed=.03)
                break
            except PlanningError as e:
                errors.append(str(e))
        else:
            raise PlanningError(f"cannot retract from {name}: {errors}")
    else:
        motion.line(pp-rr[:,2]*.08,rr,allow_pad_body=body,clearance=False,speed=.03)
    if cell.violations:
        raise PlanningError(f"safety failure after {name}: {cell.violations[:2]}")
    return dict(part=name,grasp=g.label,candidate=i,sim_time_s=float(cell.d.time),state=state)


def assemble(cell, *, only=None, choices=None, wall_cap_s=7200):
    deadline=time.monotonic()+wall_cap_s
    motion=Motion(cell)
    choices=choices or {}
    mates={m["part"]:m for m in cell.graph["mates"]}
    order = ["esc"] + [n for i in range(1,5) for n in (f"leg_{i}",f"motor_{i}")] + [
        "guard","flight_controller","compute","compute_cover","battery",
        "cheek_left","cheek_right","sensor_cap","lidar","camera"] + [f"prop_{i}" for i in range(1,5)]
    if only is not None:
        order = list(only)
    results=[]
    for name in order:
        if time.monotonic()>deadline:
            raise TimeoutError("reference assembly wall cap")
        results.append(place(cell,motion,mates[name],choice=choices.get(name,0),wall_deadline=deadline))
    result=cell.result();result["placements"]=results
    return result
