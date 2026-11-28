"""dev measurement tool: credit per stage of the job (value-weighted, from the grader's per-component detail) and the
perception errors, for devrun or verify outputs.   python3 stage_table.py <run dir with result.json/truth0.json> ..."""
import json, sys
VAL = {"module": 8.0, "lid": 5.0, "msd": 3.0, "busbar": 2.0, "bracket": 1.0, "connector": 1.0, "harness": 1.0}
STAGE = {"lid": "lid", "msd": "isolation", "connector": "isolation", "harness": "isolation", "busbar": "busbars", "bracket": "brackets", "module": "modules"}
GROUP = {"lid": "lid", "busbar": "busbars", "bracket": "brackets"}
ORDER = ["lid", "isolation", "busbars", "brackets", "modules"]
print("| run | pack | " + " | ".join(ORDER) + " | credit |")
for d in sys.argv[1:]:
    r = json.load(open(d + "/result.json")); tr = json.load(open(d + "/truth0.json"))
    det = r["score"]["detail"]
    got, tot = {s: 0.0 for s in ORDER}, {s: 0.0 for s in ORDER}
    for pid, p in tr["parts"].items():
        s = STAGE[p["kind"]]; v = VAL[p["kind"]]
        tot[s] += v; got[s] += v * bool(det.get(pid))
    for fid, f in tr["fasteners"].items():
        if f["status"] == "missing":
            continue
        s = GROUP[f["group"]]
        tot[s] += 0.5; got[s] += 0.5 * bool(det.get(fid))
    cells = ["%.1f/%.1f" % (got[s], tot[s]) for s in ORDER]
    print("| %s | %s %s | %s | %.4f |" % (d.split("/")[-2], r["result"]["variant"], r["result"]["harness"], " | ".join(cells), r["result"]["credit"]))
