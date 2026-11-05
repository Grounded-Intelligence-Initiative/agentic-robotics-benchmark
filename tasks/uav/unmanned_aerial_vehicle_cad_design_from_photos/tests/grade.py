"""Deterministic geometry grading of the extracted rigid UAV assembly, in millimetres."""
import itertools
import json
import math
import re
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform


def band(error, full, zero):
    return float(np.clip((zero - error) / (zero - full), 0, 1))


def rising(value, zero, full):
    return float(np.clip((value - zero) / (full - zero), 0, 1))


def transform(s, matrix):
    tr = gp_Trsf()
    tr.SetValues(*np.asarray(matrix)[:3].ravel().tolist())
    return cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped, tr, True).Shape())


def mesh(s):
    v, f = s.tessellate(.15, .2)
    return trimesh.Trimesh([p.toTuple() for p in v], f, process=False)


def fit_points(a, b):
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a-ca).T @ (b-cb))
    r = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    t = np.eye(4); t[:3, :3] = r; t[:3, 3] = cb-r@ca
    return t


def rigid_alignment(a, b):
    """Base-only registration. Never permits reflection or a scale change."""
    av, bv = np.asarray(mesh(a).vertices), np.asarray(mesh(b).vertices)
    if len(av) == len(bv):
        t = fit_points(av, bv)
        if np.max(np.linalg.norm(av @ t[:3, :3].T+t[:3, 3]-bv, axis=1)) < 1e-5:
            return t, 0.
    apts = trimesh.sample.sample_surface(mesh(a), 2200, seed=741)[0]
    bpts = trimesh.sample.sample_surface(mesh(b), 2200, seed=741)[0]
    ca, cb = apts.mean(0), bpts.mean(0)
    _, ea = np.linalg.eigh(np.cov(apts.T)); _, eb = np.linalg.eigh(np.cov(bpts.T))
    tree = cKDTree(bpts)
    best = (float("inf"), None)
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([-1, 1], repeat=3):
            r = eb[:, perm] @ np.diag(signs) @ ea.T
            if np.linalg.det(r) < 0:
                continue
            t = np.eye(4); t[:3, :3] = r; t[:3, 3] = cb-r@ca
            for _ in range(18):
                moved = apts@t[:3, :3].T+t[:3, 3]
                distance, ix = tree.query(moved)
                keep = distance <= np.quantile(distance, .92)
                delta = fit_points(moved[keep], bpts[ix[keep]])
                t = delta @ t
            err = float(np.mean(tree.query(apts@t[:3, :3].T+t[:3, 3])[0]))
            if err < best[0]:
                best = err, t
    return best[1], best[0]


def intersection(a, b):
    if a.wrapped.IsNull() or b.wrapped.IsNull() or a.Volume() < 1e-9 or b.Volume() < 1e-9:
        return 0.
    ab,bb=a.BoundingBox(),b.BoundingBox()
    if any(min(getattr(ab,k+'max'),getattr(bb,k+'max')) <= max(getattr(ab,k+'min'),getattr(bb,k+'min')) for k in 'xyz'):
        return 0.
    return max(0., min(a.Volume(), b.Volume(), a.intersect(b).Volume()))


def iou(a, b):
    v = intersection(a, b)
    return v / max(a.Volume()+b.Volume()-v, 1e-12)


def surface_error(a, b):
    ma, mb = mesh(a), mesh(b)
    pa = trimesh.sample.sample_surface(ma, 700, seed=821)[0]
    pb = trimesh.sample.sample_surface(mb, 700, seed=821)[0]
    da = trimesh.proximity.closest_point(mb, pa)[1]
    db = trimesh.proximity.closest_point(ma, pb)[1]
    return float(np.quantile(np.r_[da, db], .9))


def fixed_connected(model):
    graph = {c["name"]: set() for c in model.get("components", [])}
    for joint in model.get("joints", []):
        if joint.get("suppressed") or joint.get("error") or joint.get("type", "").lower() != "fixed":
            continue
        a, b = joint.get("part1"), joint.get("part2")
        if a in graph and b in graph:
            graph[a].add(b); graph[b].add(a)
    reached = set(model.get("grounded", [])) & graph.keys()
    todo = list(reached)
    while todo:
        for nxt in graph[todo.pop()] - reached:
            reached.add(nxt); todo.append(nxt)
    return reached


def mass_properties(items):
    total = sum(m for s, m in items)
    if not total:
        return 0., np.zeros(3), np.zeros(3)
    center = sum(np.array(s.Center().toTuple())*m for s,m in items)/total
    inertia = np.zeros((3,3))
    for s,m in items:
        d = np.array(s.Center().toTuple())-center
        inertia += np.array(cq.Shape.matrixOfInertia(s))*m/s.Volume() + m*(d@d*np.eye(3)-np.outer(d,d))
    return total, center, np.linalg.eigvalsh(inertia)


def grade(extraction, reference, config):
    extraction, reference = Path(extraction), Path(reference)
    result = {"design_score": 0., "criteria": {k: 0. for k in config["weights"]}, "checks": {}, "parts": {}, "statistics": {}, "diagnostics": []}
    if not (extraction / "model.json").is_file():
        result["diagnostics"].append("No readable FreeCAD assembly.")
        return result
    model = json.loads((extraction / "model.json").read_text())
    if model.get("errors") or len(model.get("components", [])) > 150:
        result["diagnostics"].extend(model.get("errors", ["Too many assembly components."]))
        return result
    spec = json.loads((reference / "reference.json").read_text())
    ref = {k: cq.Shape.importBrep(str(reference/v["file"])) for k,v in spec["parts"].items()}
    connected = fixed_connected(model)
    parts, others, component_ids = {}, [], {}
    for comp in model.get("components", []):
        for leaf in comp.get("leaves", []):
            if not leaf.get("valid") or leaf.get("n_solids") != 1 or not leaf.get("brep"):
                result["diagnostics"].append("Every assembly shape must be one valid closed solid.")
                return result
            path = (extraction/leaf["brep"]).resolve()
            if not path.is_relative_to(extraction.resolve()):
                raise ValueError("Extractor returned an escaping shape path")
            s = cq.Shape.importBrep(str(path))
            label = leaf.get("label", "").strip()
            if label in spec["parts"]:
                if leaf.get("type") != "PartDesign::Body":
                    result["diagnostics"].append("Manufactured part must be a PartDesign Body: "+label)
                    return result
                if label in parts:
                    result["diagnostics"].append("Duplicate manufactured-part label: "+label)
                    return result
                parts[label] = s; component_ids[label] = comp["name"]
            else:
                others.append({"shape": s, "label": label, "component": comp["name"]})
    if "P01" not in parts:
        result["diagnostics"].append("The tagged base P01 is required for assembly alignment.")
        return result
    result["checks"]["cad_valid"] = 1.
    alignment, residual = rigid_alignment(parts["P01"], ref["P01"])
    result["statistics"]["base_alignment_residual_mm"] = residual
    parts = {k: transform(s, alignment) for k,s in parts.items()}
    for entry in others:
        entry["shape"] = transform(entry["shape"], alignment)
    floors = {}
    for pid, target in ref.items():
        if pid not in parts:
            result["parts"][pid] = {"iou": 0., "shape": 0., "present": False}; floors[pid] = 0.
            continue
        similarity = iou(parts[pid], target)
        error = surface_error(parts[pid], target)
        score = .6*band(error, *config["surface_mm"])+.4*rising(similarity, .25, .95)
        result["parts"][pid] = {"iou": similarity, "surface_p90_mm": error, "shape": score, "present": True}
        floors[pid] = rising(similarity, *config["shape_floor_iou"])
    quality = float(np.mean(list(floors.values())))
    result["criteria"]["shape"] = float(np.mean([p["shape"] for p in result["parts"].values()]))

    feature_scores = []
    for feature in spec["features"]:
        pid = feature["part"]
        if pid not in parts:
            feature_scores.append(0.); continue
        c, size = np.array(feature["center_mm"]), np.array(feature["size_mm"])
        roi = cq.Solid.makeBox(*size, cq.Vector(*(c-size/2)))
        if feature["kind"] == "mount_hole":
            witness = cq.Solid.makeCylinder(1.35, size[2], cq.Vector(c[0],c[1],c[2]-size[2]/2))
            empty = 1-intersection(parts[pid], witness)/witness.Volume()
            feature_scores.append(floors[pid]*rising(empty,.5,.99))
        else:
            a, b = parts[pid].intersect(roi), ref[pid].intersect(roi)
            feature_scores.append(floors[pid]*rising(iou(a,b), *config["feature_iou"]))
    result["criteria"]["features"] = float(np.mean(feature_scores))

    # Full-shape recognition, followed by one-to-one assignment. Labels are only search hints.
    vendor = {k:cq.Shape.importBrep(str(reference/v["file"])) for k,v in spec["purchased"].items()}
    recognised = {k:[] for k in vendor}
    for entry in others:
        s = entry["shape"]
        for kind, target in vendor.items():
            if abs(s.Volume()/target.Volume()-1) > .012:
                continue
            local_to_world, err = rigid_alignment(target, s)
            similarity = iou(transform(target, local_to_world), s)
            if similarity >= .985:
                recognised[kind].append(dict(entry, transform=local_to_world)); break
    matched, position_scores, fit_scores = {}, [], []
    connected_count = sum(c in connected for c in component_ids.values())
    for kind in vendor:
        wanted = [i for i in spec["placements"] if i["model"] == kind]
        found = recognised[kind]
        if found:
            cost = np.array([[np.linalg.norm(np.array(w["xyz_mm"])-f["transform"][:3,3]) for f in found] for w in wanted])
            wi, fi = linear_sum_assignment(cost)
            pair = dict(zip(wi,fi))
        else:
            pair = {}
        for i, w in enumerate(wanted):
            if i not in pair:
                position_scores.append(0.); fit_scores.append(0.); continue
            f = found[pair[i]]; matched[w["id"]] = f
            r,t = f["transform"][:3,:3], f["transform"][:3,3]
            pos = band(float(np.linalg.norm(t-w["xyz_mm"])),*config["position_mm"])
            axis = r[:,2] if kind in ["MTR-29","POST-D"] else r[:,0]
            target_axis = np.array([0,0,1] if kind in ["MTR-29","POST-D"] else [1,0,0])
            angle = math.degrees(math.acos(float(np.clip(axis@target_axis,-1,1))))
            # Rectangular compute and battery are physically symmetric under 180-degree rotations.
            if kind in ["CPU-70","BAT-130","POST-D"]:
                angle = min(angle,180-angle)
            position_scores.append(pos*(.7+.3*band(angle,*config["axis_deg"])))
            expected = vendor[kind].translate(tuple(w["xyz_mm"]))
            support = [pid for pid,s in ref.items() if s.distance(expected)<.04]
            fit = np.mean([band(parts[pid].distance(f["shape"]),.15,3)*floors[pid] if pid in parts else 0. for pid in support]) if support else 0.
            penetration = sum(intersection(f["shape"],s) for s in parts.values())
            fit_scores.append(float(fit)*band(penetration/max(f["shape"].Volume(),1),.0001,.05))
            connected_count += int(f["component"] in connected)
    graph_fraction = connected_count/(len(ref)+len(spec["placements"]))
    result["statistics"].update(purchased_matched=len(matched), rigid_connected_fraction=graph_fraction, shape_floor=quality)
    result["criteria"]["installation"] = quality*(.8*float(np.mean(position_scores))+.2*graph_fraction)
    result["criteria"]["fit"] = quality*float(np.mean(fit_scores))

    expected_mass = [(s, s.Volume()*spec["parts"][pid]["density_g_mm3"]) for pid,s in ref.items()]
    actual_mass = [(s, s.Volume()*spec["parts"][pid]["density_g_mm3"]) for pid,s in parts.items()]
    for w in spec["placements"]:
        expected_mass.append((vendor[w["model"]].translate(tuple(w["xyz_mm"])), spec["purchased"][w["model"]]["mass_g"]))
        if w["id"] in matched:
            actual_mass.append((matched[w["id"]]["shape"],spec["purchased"][w["model"]]["mass_g"]))
    assigned = {id(entry["shape"]) for entry in matched.values()}
    for entry in others:
        if id(entry["shape"]) not in assigned:
            actual_mass.append((entry["shape"],entry["shape"].Volume()*.00124))
    am,ac,ai = mass_properties(actual_mass); bm,bc,bi = mass_properties(expected_mass)
    result["statistics"].update(mass_g=am, com_error_mm=float(np.linalg.norm(ac-bc)))
    result["criteria"]["mass"] = quality*(.3*band(abs(am/bm-1),.02,.25)+.35*band(float(np.linalg.norm(ac-bc)),1.,15.)+.35*band(float(np.max(abs(ai/bi-1))),.04,.5))
    sweeps, clear = [], []
    for w in [i for i in spec["placements"] if i["model"]=="MTR-29"]:
        if w["id"] not in matched:
            clear.append(0.); continue
        f = matched[w["id"]]
        disc = transform(cq.Solid.makeCylinder(65.,4.,cq.Vector(0,0,24.)),f["transform"])
        collisions = sum(intersection(disc,s) for s in parts.values())
        collisions += sum(intersection(disc,entry["shape"]) for entry in others if id(entry["shape"])!=id(f["shape"]))
        collisions += sum(intersection(disc,other) for other in sweeps)
        sweeps.append(disc)
        clear.append(band(collisions,.05,200.))
    result["criteria"]["clearance"] = quality*float(np.mean(clear))
    result["design_score"] = float(sum(config["weights"][k]*v for k,v in result["criteria"].items()))
    return result


if __name__ == "__main__":
    import argparse
    p=argparse.ArgumentParser();p.add_argument("extraction");p.add_argument("reference");p.add_argument("output");p.add_argument("--config",default=str(Path(__file__).with_name("grader_config.json")))
    a=p.parse_args();result=grade(a.extraction,a.reference,json.loads(Path(a.config).read_text()))
    Path(a.output).write_text(json.dumps(result,indent=2)+"\n");print(json.dumps({k:v for k,v in result.items() if k!="parts"},indent=2))
