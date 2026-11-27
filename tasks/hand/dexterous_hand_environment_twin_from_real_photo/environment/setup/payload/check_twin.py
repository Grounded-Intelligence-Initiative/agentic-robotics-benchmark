#!/usr/bin/env python3
"""check_twin: check your twin the way the grader's gate sees it, settle it, and rehearse the pick sheet on it. It
scores nothing and reads nothing about the true scene.

    python3 check_twin.py [SUBMISSION]                     # default /home/user/submission: the gate
    python3 check_twin.py --settle                         # + the standard settle (a blow-up fails the gate)
    python3 check_twin.py --rehearse                       # + every request of the pick sheet planned and executed on
                                                           #   your own twin (implies --settle)
    python3 check_twin.py --rehearse --ids R0001 R0042     # only these requests
    python3 check_twin.py --rehearse --lift                # + the lift trial of every planned body pinch on your twin
    python3 check_twin.py --overlay overlay.png            # your twin's collision outlines through photo_cam on the
                                                           #   photo (MuJoCo's CPU renderer)
    python3 check_twin.py --json                           # everything as one JSON document on stdout

What it does, in the grader's order (one pipeline):
  1. twin_gate.gate(): PASS / FAIL with every reason, each thing with its status (ok, or excluded with the reasons),
     the support's top face, and what is deleted before any physics;
  2. --settle: twin_sim.settle() on the gate's physics model, the held things of the pick sheet kept still: per thing
     the largest displacement and rotation, and the blow-up check (a blow-up fails the gate);
  3. --rehearse: from the settled state, the pieces of every thing (twin_sim.pieces_from_model) in the rehearsal frame
     (twin_sim.rehearsal_frame: the support's top, x along photo_cam's line of sight; without photo_cam the top's own
     x, with a warning), then hand_instrument.rehearse() on that scene: for every request whether the instrument
     plans it (or why it declines it) and the outcome at each of the placements when executed on your own twin;
     --lift adds the lift trial of each planned body pinch in your twin (twin_sim.planning_xml), at the centre
     placement, with the pads' friction at instrument_config's lift.rehearse_friction and the pick sheet's held
     things welded (the lift trial is for your own rehearsal: the assessment runs no lift trial and judges every
     pick geometrically, twin_contract.md section 8). A request on a thing your twin does not contain is listed as
     absent (declined_by "absent", never passed to the instrument);
  4. --overlay: a segmentation of the twin's collision geoms through photo_cam, outlines drawn on photo.png.

Everything printed comes from your submission and the public rules. Exit code 0 when the gate passes (and, with
--settle, the settle does not blow up), 1 otherwise, 2 for a usage error.
"""
import argparse
import copy
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
DEFAULT_SUB = "/home/user/submission"


def _gl_setup():
    """MuJoCo renders on the CPU through OSMesa here (no GPU): set the backend before mujoco is imported."""
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    if os.environ.get("MUJOCO_GL") == "osmesa":
        os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")


def _r(v, n=4):
    if v is None:
        return None
    v = float(v)
    return round(v, n) if math.isfinite(v) else v


def _err(exc):
    return "%s: %s" % (type(exc).__name__, str(exc)[:400])


def _jsonable(x):
    import numpy as np
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return [_jsonable(v) for v in x.tolist()]
    if isinstance(x, (np.floating, float)):
        v = float(x)
        return v if math.isfinite(v) else str(v)
    if isinstance(x, np.integer):
        return int(x)
    return x


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def camera_summary(model, name="photo_cam"):
    """photo_cam as OpenCV intrinsics, its pose, and the world's up direction in camera coordinates."""
    import mujoco
    import numpy as np
    import twin_camera
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if cid < 0:
        return None
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    out = {"name": name}
    try:
        K, W, H = twin_camera.decode(model, cid)
        out.update({"fx": _r(K[0, 0], 3), "fy": _r(K[1, 1], 3), "cx": _r(K[0, 2], 3), "cy": _r(K[1, 2], 3),
                    "W": int(W), "H": int(H)})
    except Exception as exc:  # noqa: BLE001 - twin_camera.CameraError
        out["decode_error"] = str(exc)
    R_cv, p = twin_camera.camera_frame(model, d, cid)
    up = R_cv.T @ np.array([0.0, 0.0, 1.0])
    fwd = R_cv[:, 2]
    out.update({"position_world_m": [_r(v, 6) for v in p], "world_up_in_camera": [_r(v, 6) for v in up],
                "optical_axis_below_horizontal_deg": _r(math.degrees(math.asin(max(-1.0, min(1.0, -fwd[2])))), 3)})
    return out


# ---------------------------------------------------------------------------------------------------------------
# the pipeline after the gate

def run_settle(res, held, out):
    import twin_sim
    t0 = time.process_time()
    try:
        st = twin_sim.settle(res.physics_model(), res.roles, held=held)
    except Exception as exc:  # noqa: BLE001 - twin_sim.TwinSimError
        out["settle"] = {"ran": False, "why": _err(exc)}
        return None
    out["settle"] = {"ran": True, "seconds_simulated": st["seconds"], "steps": st["steps"],
                     "cpu_s": round(time.process_time() - t0, 2), "blowup": st["blowup"], "finite": st["finite"],
                     "exploded": st["exploded"], "mujoco_warnings": st["warnings"], "held": st["held"],
                     "things": {k: {"disp_mm": st["disp_mm"][k], "rot_deg": st["rot_deg"][k],
                                    "max_disp_mm": st["max_disp_mm"][k], "max_rot_deg": st["max_rot_deg"][k]}
                                for k in sorted(st["disp_mm"])}}
    return st


def rehearsal_scene(res, st):
    """(the instrument's scene, the frame (world_from_frame), its warning, the model, the settled data): the settled
    twin's pieces in the rehearsal frame, mm."""
    import hand_instrument
    import twin_sim
    m = res.physics_model()
    d = twin_sim.data_at(m, st["qpos_end"])
    pieces = twin_sim.pieces_from_model(m, d, res.roles)
    top = twin_sim.support_frame(m, d, res.roles)
    T, warn = twin_sim.rehearsal_frame(m, d, top)
    scene = hand_instrument.Scene.from_pieces(twin_sim.to_frame_mm(pieces, T))
    return scene, T, warn, m, d


def lift_variant(cfg):
    """[mass scale, pad friction] of the rehearsal's lift trials: the pads take instrument_config's
    lift.rehearse_friction; the instrument's own default (1.0, 0.6) when a config does not give it. The lift trial
    is for your own rehearsal: the assessment runs none (twin_contract.md section 8)."""
    fr = (cfg.get("lift") or {}).get("rehearse_friction")
    return [1.0, 0.6] if fr is None else [1.0, float(fr)]


def run_rehearse(res, st, sheet, cfg, envelope, ids, lifts, out):
    import hand_instrument
    import twin_sim
    t0 = time.process_time()
    reqs = sheet["requests"]
    if ids:
        known = {r["id"] for r in reqs}
        unknown = [i for i in ids if i not in known]
        if unknown:
            out["rehearse"] = {"ran": False, "why": "not in the pick sheet: %s" % ", ".join(unknown)}
            return
        want = set(ids)
        reqs = [r for r in reqs if r["id"] in want]
    try:
        scene, T, warn, m, d = rehearsal_scene(res, st)
        present = set(res.roles.keys)
        run = [r for r in reqs if r["thing"] in present]
        absent = [r["id"] for r in reqs if r["thing"] not in present]
        xml = bodies = None
        held = [k for k in sheet.get("held", []) if k in present]
        variant = lift_variant(cfg)
        if lifts:
            xml, bodies = twin_sim.planning_xml(m, d, res.roles, T, held=held)
        rows = hand_instrument.rehearse(scene, run, cfg, envelope, lifts=bool(lifts), model_xml=xml, bodies=bodies,
                                        variant=variant, held=held)
    except Exception as exc:  # noqa: BLE001 - a broken twin or request must not hide the gate result
        out["rehearse"] = {"ran": False, "why": _err(exc)}
        return
    n_pl = len(cfg["placements_mm"])
    summary = {"requests": len(reqs), "absent": len(absent), "planned": sum(1 for r in rows if r.get("planned")),
               "declined_by": {}, "done_at_every_placement": 0, "stop_somewhere": 0}
    for r in rows:
        if not r.get("planned"):
            k = str(r.get("declined_by"))
            summary["declined_by"][k] = summary["declined_by"].get(k, 0) + 1
            continue
        oc = r.get("outcomes", [])
        if len(oc) == n_pl and all(o == "done" for o in oc):
            summary["done_at_every_placement"] += 1
        if any(o == "stop" for o in oc):
            summary["stop_somewhere"] += 1
    out["rehearse"] = {"ran": True, "frame_world_from_rehearsal": [[_r(v, 9) for v in row] for row in T.tolist()],
                       "frame_warning": warn, "cpu_s": round(time.process_time() - t0, 2), "summary": summary,
                       "absent": absent, "rows": rows, "lifts": bool(lifts),
                       "lift_variant": variant if lifts else None, "lift_held": held if lifts else None}


# ---------------------------------------------------------------------------------------------------------------
# --overlay

def overlay(res, photo_path, out_path, cam="photo_cam"):
    """Segmentation of the twin's collision geoms through photo_cam, outlines drawn on the photo."""
    import mujoco
    import numpy as np
    m = copy.deepcopy(res.physics_model())
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    if cid < 0:
        return {"error": "your twin has no camera %s" % cam}
    w, h = (int(v) for v in m.cam_resolution[cid])
    m.vis.global_.offwidth = max(int(m.vis.global_.offwidth), w)
    m.vis.global_.offheight = max(int(m.vis.global_.offheight), h)
    for g in range(m.ngeom):
        visual = m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
        m.geom_group[g] = 5 if visual else 0
        if not visual:
            m.geom_rgba[g, 3] = 1.0
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = 1
    opt.sitegroup[:] = 0
    t0 = time.process_time()
    r = mujoco.Renderer(m, height=h, width=w)
    try:
        r.enable_segmentation_rendering()
        r.update_scene(d, camera=cam, scene_option=opt)
        seg = r.render().copy()
    finally:
        r.close()
    t_render = time.process_time() - t0
    ids = np.where(seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM), seg[:, :, 0], -1).astype(int)
    ids[(ids < 0) | (ids >= m.ngeom)] = -1
    roles = res.roles
    keys = list(roles.keys)
    root = np.asarray(m.body_rootid, int)
    thing_of = {int(roles.item_ids[k]): i + 1 for i, k in enumerate(keys)}
    lut = np.zeros(max(1, m.ngeom), int)
    for g in range(m.ngeom):
        rb = int(root[int(m.geom_bodyid[g])])
        lut[g] = thing_of.get(rb, 200000 + g)               # each support box keeps its own outline
    lab = np.zeros(ids.shape, int)
    ok = ids >= 0
    lab[ok] = lut[ids[ok]]
    note = None
    try:
        import cv2
    except ImportError:  # pragma: no cover - OpenCV is in the image
        cv2 = None
    img = None
    if photo_path and os.path.isfile(photo_path):
        if cv2 is not None:
            ph = cv2.imread(photo_path, cv2.IMREAD_COLOR)
            if ph is not None:
                ph = cv2.cvtColor(ph, cv2.COLOR_BGR2RGB)
                if ph.shape[:2] != (h, w):
                    ph = cv2.resize(ph, (w, h), interpolation=cv2.INTER_AREA)
                img = ph.astype(np.float32)
        else:
            from PIL import Image
            img = np.asarray(Image.open(photo_path).convert("RGB").resize((w, h))).astype(np.float32)
    if img is None:
        img = np.full((h, w, 3), 40.0, np.float32)
        note = "the photo %s was not found: outlines drawn on grey" % photo_path
    edge = np.zeros(lab.shape, bool)
    edge[:, 1:] |= lab[:, 1:] != lab[:, :-1]
    edge[1:, :] |= lab[1:, :] != lab[:-1, :]
    edge[:, :-1] |= edge[:, 1:].copy()
    edge[:-1, :] |= edge[1:, :].copy()
    rng = np.random.default_rng(7)
    pal = rng.integers(60, 256, size=(len(keys) + 2, 3)).astype(np.float32)
    colour = np.zeros(lab.shape + (3,), np.float32)
    for k in range(1, len(keys) + 1):
        colour[lab == k] = pal[k]
    colour[lab >= 200000] = (255.0, 255.0, 255.0)
    tint = (lab > 0) & (lab <= len(keys))
    img[tint] = 0.75 * img[tint] + 0.25 * colour[tint]
    img[edge] = np.where(lab[edge][:, None] > 0, colour[edge], 255.0)
    img = np.clip(img, 0, 255).astype(np.uint8)
    visible = {}
    for k, key in enumerate(keys):
        mask = lab == k + 1
        visible[key] = int(mask.sum())
        if visible[key] and cv2 is not None:
            ys, xs = np.nonzero(mask)
            cv2.putText(img, key, (int(xs.mean()) - 4 * len(key), int(ys.mean())), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
    if cv2 is not None:
        if not cv2.imwrite(out_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR)):
            raise OSError("cannot write %s" % out_path)
    else:
        from PIL import Image
        Image.fromarray(img).save(out_path)
    res_ = {"out": os.path.abspath(out_path), "size": [w, h], "render_cpu_s": round(t_render, 2),
            "visible_px": visible, "support_px": int((lab >= 200000).sum()), "geoms": "collision only"}
    if note:
        res_["note"] = note
    return res_


# ---------------------------------------------------------------------------------------------------------------
# main

def run(a):
    import twin_gate
    out = {"format": "hand-check-twin-report/1", "submission": os.path.abspath(a.submission)}
    t0 = time.process_time()
    res = twin_gate.gate(a.submission)
    rep = res.report
    out["gate"] = {"pass": bool(res.ok), "version": rep.get("gate_version"), "reasons": rep.get("reasons", []),
                   "notes": rep.get("warnings", []), "stats": rep.get("stats", {}),
                   "cpu_s": round(time.process_time() - t0, 2)}
    out["things"] = rep.get("items", [])
    out["support"] = rep.get("support")
    out["deleted"] = rep.get("deleted", [])
    status = 0 if res.ok else 1
    if res.model is not None:
        out["camera"] = camera_summary(res.model)
    if not res.ok:
        if a.settle or a.rehearse or a.overlay:
            out["not_run"] = "the gate failed: nothing else runs"
        return out, status
    if a.overlay:
        try:
            out["overlay"] = overlay(res, a.photo or os.path.join(HERE, "photo.png"), a.overlay)
        except Exception as exc:  # noqa: BLE001 - a rendering problem must not hide the gate result
            out["overlay"] = {"error": _err(exc), "hint": "MuJoCo renders on the CPU with MUJOCO_GL=osmesa"}
    if not (a.settle or a.rehearse):
        return out, status
    sheet_path = a.pick_sheet or os.path.join(HERE, "pick_sheet.json")
    sheet = load_json(sheet_path) if os.path.isfile(sheet_path) else None
    held = list((sheet or {}).get("held", []))           # the things the contract holds still in the settle
    st = run_settle(res, held, out)
    if st is None:
        return out, 1
    if st["blowup"]:
        out["gate"]["pass"] = False
        out["gate"]["reasons"].append("the standard settle blows up")
        return out, 1
    if a.rehearse and sheet is None:
        out["rehearse"] = {"ran": False, "why": "no pick sheet at %s" % sheet_path}
    elif a.rehearse:
        cfg = load_json(a.instrument_config or os.path.join(HERE, "instrument_config.json"))
        env_path = a.envelope or os.path.join(HERE, "hand_envelope.json")
        env = load_json(env_path) if (a.envelope or os.path.isfile(env_path)) else None
        run_rehearse(res, st, sheet, cfg, env, a.ids, a.lift, out)
    return out, status


def print_report(out):
    g = out["gate"]
    print("twin_gate %s: %s  (%s)" % ("PASS" if g["pass"] else "FAIL", out["submission"], g.get("version")))
    for r in g["reasons"]:
        print("  FAIL  " + r)
    for w in g["notes"]:
        print("  note  " + w)
    sup = out.get("support")
    if sup and sup.get("top_geom"):
        print("support: top face of box %s, %.4f m^2" % (sup["top_geom"], sup["top_area_m2"]))
    c = out.get("camera")
    if c:
        if "fx" in c:
            print("photo_cam (OpenCV): fx %.3f  fy %.3f  cx %.3f  cy %.3f  (%d x %d)" % (
                c["fx"], c["fy"], c["cx"], c["cy"], c["W"], c["H"]))
        elif "decode_error" in c:
            print("photo_cam: cannot decode: " + c["decode_error"])
        print("  position (world, m): %s; optical axis %.2f deg below the horizontal" % (
            c["position_world_m"], c["optical_axis_below_horizontal_deg"]))
    things = out.get("things", [])
    if things:
        print("things (%d mapped):" % len(things))
        print("  %-14s %-24s %-9s %9s %9s %7s" % ("key", "body", "status", "mass g", "kg/m^3", "pieces"))
        for it in things:
            print("  %-14s %-24s %-9s %9s %9s %7s" % (
                it.get("key"), it.get("body"), it.get("status"),
                "%.1f" % (1000 * it["mass_kg"]) if "mass_kg" in it else "-",
                "%.0f" % it["density_kg_m3"] if "density_kg_m3" in it else "-", it.get("n_collision_geoms", "-")))
            for r in it.get("reasons", []):
                print("      - " + r)
    if out.get("not_run"):
        print(out["not_run"])
    ov = out.get("overlay")
    if ov:
        if "error" in ov:
            print("overlay: FAILED %s%s" % (ov["error"], (" (%s)" % ov["hint"]) if "hint" in ov else ""))
        else:
            hidden = [k for k, v in ov["visible_px"].items() if v == 0]
            print("overlay: %s (%d x %d)%s%s" % (ov["out"], ov["size"][0], ov["size"][1],
                                                 "; not visible through photo_cam: " + ", ".join(hidden) if hidden else "",
                                                 ("; " + ov["note"]) if ov.get("note") else ""))
    st = out.get("settle")
    if st:
        if not st.get("ran"):
            print("settle: not run (%s)" % st.get("why"))
        else:
            print("settle (standard simulation, %.1f s simulated): %s" % (
                st["seconds_simulated"], "BLOW-UP (the gate fails)" if st["blowup"] else "no blow-up"))
            if st["mujoco_warnings"]:
                print("  MuJoCo warnings: %s" % st["mujoco_warnings"])
            print("  %-14s %12s %12s %s" % ("thing", "max disp mm", "max rot deg", ""))
            for k, e in st["things"].items():
                print("  %-14s %12.3f %12.3f %s" % (k, e["max_disp_mm"], e["max_rot_deg"],
                                                     "(held)" if k in st["held"] else ""))
    rh = out.get("rehearse")
    if rh:
        if not rh.get("ran"):
            print("rehearse: not run (%s)" % rh.get("why"))
        else:
            s = rh["summary"]
            print("rehearse on your own twin: %d requests, %d planned, %d done at every placement, %d with a stop, "
                  "%d on absent things" % (s["requests"], s["planned"], s["done_at_every_placement"],
                                           s["stop_somewhere"], s["absent"]))
            if rh.get("frame_warning"):
                print("  note  " + rh["frame_warning"])
            if rh.get("lifts"):
                print("  lift trials: centre placement, pads' friction %s, held (welded): %s" % (
                    rh["lift_variant"][1], ", ".join(rh["lift_held"]) or "none"))
            if rh.get("absent"):
                print("  absent things' requests (declined, not planned): " + " ".join(rh["absent"]))
            if s["declined_by"]:
                print("  declined: " + ", ".join("%s %d" % kv for kv in sorted(s["declined_by"].items())))
            for r in rh["rows"]:
                oc = r.get("outcomes") or []
                line = "  %s %s" % (r["id"], "planned" if r.get("planned") else "declined (%s)" % r.get("declined_by"))
                if oc and r.get("planned"):
                    line += ": " + ", ".join("%s %d" % (o, oc.count(o)) for o in ("done", "miss", "stop") if o in oc)
                if "lift" in r and r["lift"] is not None:
                    line += "; lift %s" % ("yes" if r["lift"].get("lifted") else "no (%s)" % r["lift"].get("reason"))
                print(line)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check a twin against the public gate, settle it and rehearse the pick "
                                             "sheet on it. Scores nothing and reads nothing about the true scene.")
    ap.add_argument("submission", nargs="?", default=DEFAULT_SUB)
    ap.add_argument("--settle", action="store_true", help="the standard settle (a blow-up fails the gate)")
    ap.add_argument("--rehearse", action="store_true", help="plan and execute the pick sheet on your twin")
    ap.add_argument("--lift", action="store_true", help="with --rehearse: also the lift trials on your twin")
    ap.add_argument("--ids", nargs="+", default=None, metavar="RNNNN", help="with --rehearse: only these requests")
    ap.add_argument("--overlay", metavar="OUT.png", help="photo_cam outlines of your twin drawn on the photo")
    ap.add_argument("--photo", default=None, help="the photo (default: photo.png next to this script)")
    ap.add_argument("--pick-sheet", default=None, help="default: pick_sheet.json next to this script")
    ap.add_argument("--instrument-config", default=None, help="default: instrument_config.json next to this script")
    ap.add_argument("--envelope", default=None,
                    help="hand_instrument's envelope argument: a hand_envelope.json file (default: the one next to "
                         "this script if there is one, else none)")
    ap.add_argument("--json", action="store_true", help="print one JSON document instead of the text report")
    a = ap.parse_args(argv)
    if (a.lift or a.ids) and not a.rehearse:
        ap.error("--lift and --ids need --rehearse")
    if a.overlay:
        _gl_setup()
    out, status = run(a)
    if a.json:
        print(json.dumps(_jsonable(out), indent=1, sort_keys=True))
    else:
        print_report(out)
        if out["gate"]["pass"] and "settle" not in out:
            print("The gate also requires the standard settle not to blow up: run with --settle.")
    return status


if __name__ == "__main__":
    sys.exit(main())
