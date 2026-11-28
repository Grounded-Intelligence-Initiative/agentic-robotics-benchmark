"""dev measurement tool (not part of the program): run the baseline's run.sh on a development pack the way it is
graded (wire, deadlines, repeated steps with shifted joints), with a cell started with the grader's switch only to
read the credit and the truth afterwards, and compare the program's own estimates (PB_DEBUG_DIR) with the truth.

    $PY devrun.py <dev pack 0..3> <program dir> <out dir> [max_steps]
"""
import json, os, queue, random, subprocess, sys, threading, time
import numpy as np

TASK = os.environ["TASK"]
sys.path.insert(0, TASK + "/tests")
sys.path.insert(0, TASK + "/environment/setup/payload/given/lib")
from packcell import series as S, cellproc  # noqa: E402
import packcell_client as pc  # noqa: E402

idx, prog, OUT = int(sys.argv[1]), sys.argv[2], sys.argv[3]
max_steps = int(sys.argv[4]) if len(sys.argv) > 4 else 20000
os.makedirs(OUT, exist_ok=True)
e = json.load(open(TASK + "/environment/commission_service/dev_packs.json"))["packs"][idx]
pack = S.draw_entry(e)
cell = cellproc.CellProcess(grader=True, stderr=open(OUT + "/cell.err", "w"))
obs = cell.request({"cmd": "reset", "pack": pack})["obs"]
truth0 = cell.request({"cmd": "truth"})["truth"]
json.dump(truth0, open(OUT + "/truth0.json", "w"))
P = lambda *a: print("##", *a, flush=True)

env = {"PATH": os.path.dirname(sys.executable) + ":/usr/bin:/bin", "HOME": os.environ.get("HOME", "/tmp"), "LANG": "C.UTF-8",
       "PB_DEBUG_DIR": OUT}
proc = subprocess.Popen(["bash", "run.sh"], cwd=prog, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=open(OUT + "/program.err", "w"),
                        universal_newlines=True, bufsize=1, env=env, start_new_session=True)
lines = queue.Queue()
threading.Thread(target=lambda: [lines.put(l) for l in proc.stdout] + [lines.put(None)], daemon=True).start()
waited = 0.0


def get(deadline_s):
    global waited
    t0 = time.time()
    while True:
        line = lines.get(timeout=deadline_s)
        if line is None:
            raise RuntimeError("program closed its output")
        if line.strip():
            waited += time.time() - t0
            return line.strip()


def ask(msg):
    proc.stdin.write(json.dumps(msg) + "\n")
    proc.stdin.flush()
    return json.loads(get(30.0))


assert get(120.0) == "READY"
ask({"type": "reset"})
rng = random.Random(idx * 7919 + 13 + int(os.environ.get("DEVRUN_SEED", "0")))
t, why, repeats, t0 = 0, None, 0, time.time()
traces = []
last_log = 0.0
while t < max_steps and not obs["ended"]:
    try:
        if rng.random() < 0.02:
            other = pc.perturbed(obs, rng)
            pair = [("true", obs), ("other", other)]
            rng.shuffle(pair)
            replies = {n: ask({"type": "act", "t": t, "obs": o}) for n, o in pair}
            reply = replies["true"]
            repeats += 1
        else:
            reply = ask({"type": "act", "t": t, "obs": obs})
    except Exception as exc:  # noqa: BLE001
        why = "program: %r" % exc
        break
    a = reply.get("action")
    rr = cell.request({"cmd": "step", "action": a})
    if "error" in rr:
        why = "bad action %s: %s" % (rr["error"], json.dumps(a)[:300])
        break
    prev_stops = obs["status"]["protective_stops"]
    obs = rr["obs"]
    if obs["status"]["protective_stops"] > prev_stops:
        P("STOP at t=%d time %.2f action %s tcp_t %s wrench %s tool %s" % (t, obs["time"], json.dumps(a)[:300], [round(v, 3) for v in obs["tcp_target"]],
          [round(v, 1) for v in obs["wrench"]], obs["tool"]))
    if obs["events"]:
        P("events", obs["time"], obs["events"])
    w0, w1 = float(os.environ.get("DEVRUN_TRACE_FROM", "1e9")), float(os.environ.get("DEVRUN_TRACE_TO", "-1"))
    if os.environ.get("DEVRUN_TRACE") and (obs["tool"] == "gripper" or w0 <= obs["time"] <= w1):
        # a maintainer's trace (truth): where the gripper is against the parts' grip points
        tr = cell.request({"cmd": "truth"})["truth"]
        with open(OUT + "/trace.jsonl", "a") as fh:
            fh.write(json.dumps({"t": t, "time": obs["time"], "action": a, "tcp": obs["tcp"], "tcp_target": obs["tcp_target"], "q": obs["q"],
                                 "gripper": obs["gripper"], "status": obs["status"], "lid": tr["parts"].get("lid"), "tool": obs["tool"],
                                 "spindle": obs["spindle"], "tip": tr.get("tip"), "captured": tr.get("captured"),
                                 "fast_lid": {k: v["status"] for k, v in tr["fasteners"].items() if v["group"] == "lid"}}) + "\n")
    if obs["time"] - last_log >= 60:
        last_log = obs["time"]
        sc = cell.request({"cmd": "score"})["score"]
        P("t_sim %.0f steps %d wall %.0f credit %.3f stops %d frames_left %d prog %.1fs" % (obs["time"], t, time.time() - t0, sc["credit"], sc["protective_stops"], obs["frames_left"], waited))
    t += 1
try:
    proc.stdin.write(json.dumps({"type": "close"}) + "\n"); proc.stdin.flush(); proc.wait(timeout=10)
except Exception:  # noqa: BLE001
    proc.kill()
sc = cell.request({"cmd": "score", "final": True})["score"]
cell.close()
res = {"pack": idx, "variant": e["variant"], "harness": e["harness"], "ended": obs["ended"] or why, "t_sim": obs["time"], "steps": t,
       "repeats": repeats, "frames_used": 400 - obs["frames_left"], "program_s": round(waited, 1), "wall_s": round(time.time() - t0),
       "credit": sc["credit"], "stops": sc["protective_stops"], "stripped": sc["stripped"],
       "not_credited": sorted(k for k, v in sc["detail"].items() if not v)}
json.dump({"result": res, "score": sc}, open(OUT + "/result.json", "w"), indent=1, default=str)
P("RESULT", json.dumps(res))
