"""dev: transcribe the service manual's tables (section 2, 4, 5) into manual.json for the program."""
import json, re, sys
src, out = sys.argv[1], sys.argv[2]
text = open(src).read()
def tables(block):
    rows, cur = [], []
    for line in block.splitlines():
        if line.startswith("|"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if set("".join(cells)) <= set("-: "):
                continue
            cur.append(cells)
        elif cur:
            rows.append(cur); cur = []
    if cur:
        rows.append(cur)
    return rows
m = {"variants": {}}
for v in ("P4", "P5", "P6"):
    block = text.split("### HVB-" + v)[1].split("###")[0].split("## 5.")[0]
    ft, ct = tables(block)
    m["variants"][v] = {
        "fasteners": [{"id": r[0], "thread": r[1], "x": float(r[2]) / 1000, "y": float(r[3]) / 1000, "seat_z": float(r[4]) / 1000, "holds": r[5]} for r in ft[1:]],
        "parts": [{"id": r[0], "kind": r[1], "x": float(r[2]) / 1000, "y": float(r[3]) / 1000, "z": float(r[4]) / 1000} for r in ct[1:]]}
# section 2: the series table
sec2 = tables(text.split("## 2.")[1].split("## 3.")[0])[0]
for r in sec2[1:]:
    v = r[0].replace("HVB-", "")
    ribs = []
    for rib in r[3].split(";"):
        mm = re.search(r"along (x|y) at (x|y) = (-?\d+) mm", rib)
        ribs.append({"along": mm.group(1), "at": mm.group(2), "value": float(mm.group(3)) / 1000})
    m["variants"][v].update(modules=int(r[1]), lid_bolts=int(r[2]), ribs=ribs, msd_y=float(r[4].split()[0]) / 1000, bms_y=float(r[5].split()[0]) / 1000)
sec5 = tables(text.split("## 5.")[1].split("## 6.")[0])
m["heads"] = {r[0]: {"thread": r[1], "height": float(r[4]) / 1000} for r in sec5[0][1:]}
m["torque"] = {}
for r in sec5[1][1:]:
    lo, hi = [float(x) for x in r[3].split(" to ")]
    m["torque"][r[0]] = {"assembly": float(r[1]), "breakaway": float(r[2]), "corroded": [lo, hi], "never_exceed": float(r[4]), "turns": float(r[5])}
sec4 = tables(text.split("## 4.")[1].split("### HVB-P4")[0])[0]
m["grips"] = {r[0]: {"feature": r[1], "closing": r[2], "width": r[3], "mass": r[4]} for r in sec4[1:]}
json.dump(m, open(out, "w"), indent=1)
print({v: (len(d["fasteners"]), len(d["parts"]), d["modules"], d["lid_bolts"]) for v, d in m["variants"].items()})
print(m["heads"], m["torque"])
