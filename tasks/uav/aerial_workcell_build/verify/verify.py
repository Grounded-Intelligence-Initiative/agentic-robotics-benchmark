"""The grader: measure the agent's scene and compare it with the reference build.

1. Lockdown: every process of the agent's account is killed; the submission is copied to a root-owned snapshot.
2. scan.py (static: files, stage, required prims, OmniGraph rules). Errors: the scene scores 0.
3. probes.py, one fresh Isaac Sim process per group, as an unprivileged account created here (the snapshot holds no
   code; the scan made sure of it): layout, settle, the bin's lift / open / button / drop probes.
4. Score each check in [0, 1] (linear between a full-credit and a zero-credit error), weight, sum:
   acceptance_score in [0, 1]. Layout and identities are compared with truth.json (the same probes run on the
   reference build); the bin's behaviour with the brief's figures (grader_config.json).
"""

import json
import os
import pwd
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotics_grader import agent_user, ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE_USER = "arbprobe"
KIT_SHARED = "/opt/venv/lib/python3.12/site-packages/isaacsim/kit"  # cache / logs / data: shared with the agent's runs
GROUPS = ("layout", "settle", "bin_lift", "bin_open", "bin_button", "bin_drop")


def log(*a):
    print("[grader]", *a, file=sys.stderr, flush=True)


def lin(err, full, zero):
    """1 up to `full`, 0 from `zero` on, linear between (None -> 0)."""
    if err is None:
        return 0.0
    if err <= full:
        return 1.0
    if err >= zero:
        return 0.0
    return (zero - err) / (zero - full)


def dist(a, b):
    if a is None or b is None:
        return None
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def lockdown(sub):
    user = agent_user()
    subprocess.run(["/usr/bin/pkill", "-KILL", "-u", user], check=False)
    time.sleep(1.0)
    for d in ("cache", "logs", "data"):  # nothing the agent's runs left there reaches the grader's Isaac Sim
        path = os.path.join(KIT_SHARED, d)
        shutil.rmtree(path, ignore_errors=True)
        os.makedirs(path, mode=0o1777, exist_ok=True)
        os.chmod(path, 0o1777)
    snap = "/tmp/arb_verify/submission"
    shutil.rmtree("/tmp/arb_verify", ignore_errors=True)
    os.makedirs("/tmp/arb_verify", mode=0o755)
    shutil.copytree(sub, snap, symlinks=False, ignore_dangling_symlinks=True)
    for d, dirs, files in os.walk(snap):
        os.chown(d, 0, 0)
        os.chmod(d, 0o755)
        for f in files:
            p = os.path.join(d, f)
            if os.path.islink(p):
                os.unlink(p)
                continue
            os.chown(p, 0, 0)
            os.chmod(p, 0o644)
    return snap


def probe_account():
    home = "/tmp/arb_verify/probe_home"  # (also its passwd home: some Kit extensions look it up there, not in $HOME)
    os.makedirs(home, exist_ok=True)
    try:
        pwd.getpwnam(PROBE_USER)
    except KeyError:
        subprocess.run(["/usr/sbin/useradd", "--system", "--no-create-home", "--home-dir", home, "--shell",
                        "/usr/sbin/nologin", PROBE_USER], check=True)
    pw = pwd.getpwnam(PROBE_USER)
    os.chown(home, pw.pw_uid, pw.pw_gid)
    os.chmod(home, 0o700)
    return home


def as_probe(args, home, timeout):
    env = {"HOME": home, "PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "LANG": "C.UTF-8",
           "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
           "NVIDIA_DRIVER_CAPABILITIES": os.environ.get("NVIDIA_DRIVER_CAPABILITIES", "")}
    cmd = ["/usr/sbin/runuser", "-u", PROBE_USER, "--", "/opt/venv/bin/python3", "-I", "-s"] + args
    return subprocess.run(cmd, env=env, cwd=home, capture_output=True, text=True, timeout=timeout)


def run_probes(scene, home, timeout):
    out_dir = "/tmp/arb_verify/out"
    os.makedirs(out_dir, exist_ok=True)
    os.chmod(out_dir, 0o777)
    shutil.copy(os.path.join(HERE, "probes.py"), "/tmp/arb_verify/probes.py")
    os.chmod("/tmp/arb_verify/probes.py", 0o644)
    groups = {}
    for g in GROUPS:
        path = os.path.join(out_dir, f"{g}.json")
        t0 = time.time()
        tail = ""
        try:
            r = as_probe(["/tmp/arb_verify/probes.py", scene, path, g], home, timeout)
            rc, tail = r.returncode, (r.stderr or "")[-1500:]
        except subprocess.TimeoutExpired:
            rc = "timeout"
        data = {}
        if os.path.isfile(path):
            try:
                data = json.load(open(path))["groups"].get(g, {})
            except Exception:  # noqa: BLE001
                data = {}
        if not data:
            data = {"error": f"no result (exit {rc})"}
        if "error" in data:
            log(f"probe {g} failed:", data["error"][-600:], "| stderr:", tail[-900:])
        groups[g] = data
        log(f"probe {g}: {round(time.time() - t0)} s", "error" if "error" in data else "ok")
    return groups


# --------------------------------------------------------------------------------------------- scoring
def score(scan, got, truth, spec):
    checks = []

    def add(name, weight, value, detail=None):
        checks.append(dict(name=name, weight=weight, score=round(max(0.0, min(1.0, value)), 4), detail=detail))

    lay, tlay = got.get("layout", {}), truth["groups"]["layout"]
    for t in ("Table_A", "Table_B"):
        a, b = lay.get(t), tlay[t]
        if not a:
            add(f"layout.{t}", 0.04, 0.0)
            continue
        ext = max(abs(x - y) for x, y in zip(a["extent"], b["extent"]))
        add(f"layout.{t}", 0.04, (lin(dist(a["center"], b["center"]), 0.05, 0.25) + lin(abs(a["top"] - b["top"]), 0.02, 0.10)
                                  + lin(ext, 0.05, 0.25)) / 3, dict(center=a["center"], top=a["top"], extent=a["extent"]))
    a, b = lay.get("bin_lid"), tlay["bin_lid"]
    if a:
        ext = max(abs(x - y) for x, y in zip(a["extent"], b["extent"]))
        add("layout.bin", 0.05, (lin(dist(a["center"], b["center"]), 0.05, 0.25) + lin(abs(a["top"] - b["top"]), 0.02, 0.10)
                                 + lin(ext, 0.04, 0.15)) / 3, a)
    else:
        add("layout.bin", 0.05, 0.0)
    a = (lay.get("button") or {}).get("center")
    add("layout.button", 0.03, lin(dist(a, tlay["button"]["center"]), 0.04, 0.20), a)
    # items: matched by the library asset they reference, then by position
    set_items = got.get("settle", {}).get("items", {})
    t_items = truth["groups"]["settle"]["items"]
    t_assets = truth["scan"]["facts"]["items"]
    g_assets = scan["facts"].get("items", {})
    used = set()
    for tname, tv in t_items.items():
        key = os.path.basename(t_assets[tname][0])
        cands = [n for n, refs in g_assets.items() if n not in used and n in set_items
                 and any(os.path.basename(r) == key for r in refs)]
        best = min(cands, key=lambda n: dist(set_items[n]["center"], tv["center"]), default=None)
        if best is None:
            add(f"layout.item.{tname}", 0.02, 0.0, "not found")
            add(f"physics.mass.{tname}", 0.01, 0.0, "not found")
            continue
        used.add(best)
        gv = set_items[best]
        add(f"layout.item.{tname}", 0.02, lin(dist(gv["center"], tv["center"]), 0.04, 0.20), dict(as_=best, center=gv["center"]))
        add(f"physics.mass.{tname}", 0.01, lin(abs(gv["mass"] - tv["mass"]) / tv["mass"], 0.05, 0.30), gv["mass"])
    robots, t_robots = got.get("settle", {}).get("robots", {}), truth["groups"]["settle"]["robots"]
    for d in ("Drone_L", "Drone_S"):
        g, t = robots.get(d), t_robots[d]
        gb, tb = (lay.get("robots") or {}).get(d), tlay["robots"][d]  # where the whole robot is (its bounding box)
        add(f"layout.{d}", 0.01, lin(dist(gb["center"][:2], tb["center"][:2]), 0.08, 0.40) if gb else 0.0, gb)
        add(f"physics.{d}.mass", 0.02, lin(abs(g["mass"] - t["mass"]) / t["mass"], 0.05, 0.30) if g else 0.0,
            g and g["mass"])
        add(f"physics.{d}.joints", 0.01, 1.0 if g and g["dofs"] == t["dofs"] and sorted(g["dof_types"]) == sorted(t["dof_types"])
            else 0.0, g and g["dofs"])
        # which library robots it is built from (any file of the model's folder counts: the model or one of its layers)
        refs = scan["facts"].get("robots", {}).get(d, {}).get("assets", [])
        add(f"identity.{d}.arm", 0.02, 1.0 if any("so101_new_calib" in r for r in refs) else 0.0)
        if d == "Drone_S":
            add("identity.Drone_S.airframe", 0.03, 1.0 if any("ARL-Robot-1" in r or "arl_robot_1" in r for r in refs) else 0.0)
        cams = (lay.get("cameras") or {}).get(d, {}).get("cameras", [])
        tc = tlay["cameras"][d]["cameras"][0]
        if cams:
            c = min(cams, key=lambda c: dist(c["pos_in_base"], tc["pos_in_base"]))
            add(f"sensors.{d}.camera", 0.03, (lin(dist(c["pos_in_base"], tc["pos_in_base"]), 0.03, 0.15)
                                              + lin(abs(c["pitch_down_deg"] - tc["pitch_down_deg"]), 5, 20)
                                              + lin(abs((c["hfov_deg"] or 0) - tc["hfov_deg"]), 5, 20)) / 3, c)
        else:
            add(f"sensors.{d}.camera", 0.03, 0.0)
    moved = got.get("settle", {}).get("moved", {})
    items_moved = [v for k, v in moved.items() if k.startswith("/World/Items/")]
    rest_moved = [v for k, v in moved.items() if not k.startswith("/World/Items/")]
    add("physics.settle.items", 0.025, lin(max(items_moved), 0.01, 0.10) if items_moved else 0.0, max(items_moved, default=None))
    add("physics.settle.bin_robots", 0.025, lin(max(rest_moved), 0.01, 0.10) if rest_moved else 0.0, max(rest_moved, default=None))
    # the bin's behaviour, against the brief
    lift, opn, btn, drop = (got.get(k, {}) for k in ("bin_lift", "bin_open", "bin_button", "bin_drop"))
    add("bin.rest_closed", 0.02, lin(lift.get("rest_deg"), 1.0, 5.0), lift.get("rest_deg"))
    add("bin.hold_small_pull", 0.03, lin(lift.get("hold_1N_max_deg"), 2.0, 10.0), lift.get("hold_1N_max_deg"))
    f = lift.get("lift_threshold_N")
    add("bin.lift_force", 0.05, lin(abs(f - spec["lift_force_N"]) if f is not None else None, 0.8, 2.5), f)
    fb = lift.get("fall_back_deg") if lift.get("lifted_to_20") else None
    add("bin.falls_back", 0.04, lin(fb, 2.0, 10.0), fb)
    o8 = opn.get("open_after_8s_deg") if opn.get("lifted_to_35") else None
    add("bin.strut_carries_open", 0.06, lin(abs(o8 - spec["open_deg"]) if o8 is not None else None, 3.0, 15.0), o8)
    drift = opn.get("stay_drift_deg") if o8 is not None and o8 > 80 else None
    add("bin.stays_open", 0.03, lin(drift, 1.0, 5.0), drift)
    sc_ok = opn.get("pulled_back_to") is not None and o8 is not None and o8 > 80
    sc_t, sc_r = opn.get("soft_close_s"), opn.get("soft_close_max_rate_dps")
    soft = 0.0
    if sc_ok:
        soft = (lin(opn.get("soft_close_end_deg"), 3.0, 20.0)
                + (1.0 if sc_t is not None and spec["soft_close_s"][0] <= sc_t <= spec["soft_close_s"][1] else 0.0)
                + lin(sc_r, spec["soft_close_max_dps"], 3 * spec["soft_close_max_dps"])) / 3
    add("bin.soft_close", 0.05, soft, dict(t=sc_t, max_rate=sc_r, end=opn.get("soft_close_end_deg")))
    add("bin.button_travel", 0.02, lin(-(btn.get("button_travel_mm") or 0.0), -4.0, -2.0), btn.get("button_travel_mm"))
    add("bin.button_opens", 0.02, lin(-(btn.get("open_6s") or 0.0), -90.0, -60.0), btn.get("open_6s"))
    dur = btn.get("open_duration_s")
    add("bin.button_duration", 0.02, lin(abs(dur - spec["button_open_s"]) if dur is not None else None, 2.0, 6.0), dur)
    add("bin.button_closes", 0.02, lin(btn.get("open_30s"), 3.0, 20.0) if btn.get("open_max", 0) > 60 else 0.0, btn.get("open_30s"))
    add("bin.button_again", 0.02, lin(-(btn.get("second_press_open_6s") or 0.0), -90.0, -60.0), btn.get("second_press_open_6s"))
    add("bin.catches_drop", 0.02, 1.0 if drop.get("cube_inside") else 0.0, drop.get("cube_end"))
    total_w = sum(c["weight"] for c in checks)
    return sum(c["weight"] * c["score"] for c in checks) / total_w, checks


def main():
    config, (anchor, meta) = stage_config(), read_anchor()
    sub = os.path.join(os.environ.get("ALE_HOME", "/home/user"), "submission")
    scene = next((os.path.join(sub, n) for n in ("scene.usda", "scene.usd", "scene.usdc") if os.path.isfile(os.path.join(sub, n))), None)
    if scene is None:
        write_verdict(zero_verdict(reason="no scene in /home/user/submission", anchored=anchor is not None))
        return
    snap = lockdown(sub)
    scene = os.path.join(snap, os.path.basename(scene))
    home = probe_account()
    shutil.copy(os.path.join(HERE, "scan.py"), "/tmp/arb_verify/scan.py")
    os.chmod("/tmp/arb_verify/scan.py", 0o644)
    r = as_probe(["/tmp/arb_verify/scan.py", scene, "--json"], home, 600)
    try:
        scan = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:  # noqa: BLE001
        scan = dict(ok=False, errors=[f"scan crashed: {r.stderr[-500:]}"], facts={})
    log("scan:", "ok" if scan["ok"] else scan["errors"][:5])
    extra = {"scan_ok": float(scan["ok"])}
    if not scan["ok"]:
        write_verdict(ratio_rewards(0.0, anchor, cap=config.get("scoring_cap", 1.5), metric_name=meta["metric"],
                                    higher_is_better=True, full_at=meta.get("full_at", 1.0), extra=extra))
        return
    got = run_probes(scene, home, int(config.get("probe_timeout_s", 900)))
    truth = json.load(open(os.path.join(HERE, "truth.json")))
    measured, checks = score(scan, got, truth, config["spec"])
    for c in checks:
        log(f"{c['name']:32s} w={c['weight']:.3f} s={c['score']:.3f}  {json.dumps(c['detail'])[:120] if c['detail'] is not None else ''}")
    extra.update({f"check.{c['name']}": c["score"] for c in checks})
    extra["probe_errors"] = float(sum(1 for g in got.values() if "error" in g))
    log("acceptance_score", round(measured, 4))
    write_verdict(ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), metric_name=meta["metric"],
                                higher_is_better=True, full_at=meta.get("full_at", 1.0), extra=extra))


if __name__ == "__main__":
    main()
