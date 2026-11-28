"""dev measurement tool: the program's own estimates (estimates.json of a devrun) against the development pack's
truth at its arrival (truth0.json), and the credit by stage (result.json).

    python3 compare.py <devrun out dir> [...]
"""
import json, math, os, sys
import numpy as np

STAGES = {"lid": ["lid"], "isolation": ["msd", "J1", "J2", "harness"], "busbars": ["busbar"], "brackets": ["bracket"], "modules": ["module"]}

for d in sys.argv[1:]:
    tr = json.load(open(d + "/truth0.json"))
    try:
        est = json.load(open(d + "/estimates.json"))
    except FileNotFoundError:
        est = None
    res = json.load(open(d + "/result.json"))
    r = res["result"]
    print("=" * 100)
    print(d, r["variant"], "harness", r["harness"], "| ended", r["ended"], "credit %.4f" % r["credit"], "steps", r["steps"], "t_sim", r["t_sim"],
          "frames", r["frames_used"], "program %.0f s" % r["program_s"], "stops", r["stops"], "stripped", r["stripped"])
    PT = np.array(tr["pack_T"])
    if est and est.get("pack"):
        pk = est["pack"]
        pose = pk["pose"]
        print("variant", pk["variant"], "(truth %s)" % tr["variant"], "pose err %.2f %.2f mm yaw %.3f deg" % (
            1000 * (pose[0] - PT[0, 3]), 1000 * (pose[1] - PT[1, 3]), math.degrees(pose[2] - math.atan2(PT[1, 0], PT[0, 0]))))
        h = tr["parts"].get("harness")
        for key in ("harness_lid", "harness_in"):
            g = pk.get(key)
            if g:
                print(key, "grasp (pack frame) %.3f %.3f z %.3f" % (g["x"], g["y"], g["z"]), "truth path" if h else "NO HARNESS IN TRUTH",
                      np.round(np.array(h["path"])[:, :3] - PT[:3, 3], 3).tolist() if h else "")
        if h and not pk.get("harness_lid") and not pk.get("harness_in"):
            print("harness NOT SEEN; truth covers", h["covers"])
        # the cell's own cameras as the program identified them, against the development cell's true mounting
        if est["dbg"].get("cal_overhead") is not None and os.environ.get("TASK"):
            sys.path.insert(0, os.environ["TASK"] + "/tests")
            from packcell import series as S
            entry = json.load(open(os.environ["TASK"] + "/environment/commission_service/dev_packs.json"))["packs"][r["pack"]]
            To, Tc = S.camera_mount("overhead", entry["cameras"]["overhead"]), np.array(est["dbg"]["cal_overhead"])
            Wt, Wc = S.camera_mount("wrist", entry["cameras"]["wrist"]), np.array(est["dbg"]["cal_mount"])

            def hit(Tcam, Tref, p):
                pc = Tref[:3, :3].T @ (p - Tref[:3, 3])
                ray = Tcam[:3, :3] @ (pc / pc[2])
                return Tcam[:3, 3] + (p[2] - Tcam[2, 3]) / ray[2] * ray
            pts = [np.array([x, y, -0.236]) for x in (0.56, 0.86, 1.16) for y in (-0.45, 0.0, 0.45)]
            eo = max(1000 * np.linalg.norm(hit(Tc, To, q)[:2] - q[:2]) for q in pts)
            q = np.array([0.0, 0.0, 0.35, 1.0])
            ew = 1000 * np.linalg.norm((Wc @ q)[:3] - (Wt @ q)[:3])
            print("calibration: overhead, a lid point off by at most %.2f mm; wrist mounting, a point 0.35 m ahead off by %.2f mm" % (eo, ew))
        dbg = est["dbg"]["fasteners"]
        rows = {"overhead": [], "wrist": []}
        wrong = []
        for fid, f in sorted(tr["fasteners"].items()):
            rec = pk["fast"].get(fid)
            if rec is None:
                wrong.append("%s not in the program's list" % fid)
                continue
            want = "missing" if f["status"] == "missing" else "seated"
            line = "%-5s %-4s %-8s%s -> %-8s %-4s" % (fid, f["head"], f["status"], "/c" if f["corroded"] else "  ", rec["state"], rec["kind"])
            ov = dbg.get(fid, {}).get("overhead", {})
            if ov.get("est"):
                e = 1000 * math.hypot(ov["est"][0] - f["top"][0], ov["est"][1] - f["top"][1])
                rows["overhead"].append(e)
                line += " ov %.2f" % e
            for w in dbg.get(fid, {}).get("wrist", []) or []:
                if w and not w.get("hole"):
                    e = 1000 * math.hypot(w["x"] - f["top"][0], w["y"] - f["top"][1])
                    line += " wr %.2f(n%d)" % (e, w["n"])
            ws = [w for w in dbg.get(fid, {}).get("wrist", []) or [] if w and not w.get("hole")]
            if ws:
                rows["wrist"].append(1000 * math.hypot(ws[-1]["x"] - f["top"][0], ws[-1]["y"] - f["top"][1]))
            for u in dbg.get(fid, {}).get("unscrew", []):
                line += " [%s %s]" % (u["result"], u["kind"])
            line += " out" if rec["out"] else ""
            if (rec["state"] == "missing") != (want == "missing") or (want == "seated" and rec["kind"] != f["head"]) or (want == "seated" and not rec["out"]):
                line += "   <--"
            print(line)
        for k, v in rows.items():
            if v:
                print("%s position error: n %d median %.2f max %.2f mm" % (k, len(v), float(np.median(v)), max(v)))
    det = res["score"]["detail"]
    kinds = {}
    for pid, p in tr["parts"].items():
        kinds[pid] = p["kind"]
    print("not credited:", r["not_credited"])
    if est:
        for n in est["dbg"]["notes"][-12:]:
            print("   note:", n[:200])
