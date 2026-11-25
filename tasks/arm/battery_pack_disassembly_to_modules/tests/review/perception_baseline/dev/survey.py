"""dev measurement tool (not part of the program): capture frames of a development pack at the stages of the job,
with the cell's truth beside them, to develop the perception offline.

    $PY survey.py <dev pack index 0..3> <out dir>

A cell with the grader's switch (truth only for this tool); the privileged reference does the job; at stage points
the tool takes overhead frames (arm out of view) and wrist frames over every fastener (the bit's tip 0.105 m above
the head's top, tool axis down; two yaws), saving the raw PNGs and a JSON with the frame poses and the truth."""
import base64, json, math, os, sys, time
import numpy as np

TASK = os.environ["TASK"]
sys.path.insert(0, TASK + "/tests")
sys.path.insert(0, TASK + "/solution/reference")
from packcell import series as S, cellproc  # noqa: E402
import controller as C  # noqa: E402

idx, OUT = int(sys.argv[1]), sys.argv[2]
os.makedirs(OUT, exist_ok=True)
e = json.load(open(TASK + "/environment/commission_service/dev_packs.json"))["packs"][idx]
pack = S.draw_pack(e["seed"], variant=e["variant"], harness=e["harness"], cameras=e.get("cameras"))
cell = cellproc.CellProcess(grader=True, stderr=open(OUT + "/cell.err", "w"))
obs = cell.request({"cmd": "reset", "pack": pack, "config": {"frame_budget": 3000}})["obs"]
meta = {"pack": idx, "variant": e["variant"], "harness": e["harness"], "frames": [], "first_obs": {k: v for k, v in obs.items() if k != "images"}}
P = lambda *a: print("##", *a, flush=True)
P("first obs tool", obs["tool"], "bit", obs["bit"], "q", obs["q"])


def save(o, tag, tr):
    for cam, im in (o.get("images") or {}).items():
        base = "%s/%s_%s" % (OUT, tag, cam)
        open(base + "_rgb.png", "wb").write(base64.b64decode(im["rgb"]))
        open(base + "_depth.png", "wb").write(base64.b64decode(im["depth"]))
        meta["frames"].append({"tag": tag, "cam": cam, "pose": im.get("pose"), "t": im.get("t"), "tcp_target": o["tcp_target"], "tool": o["tool"], "bit": o["bit"]})
    json.dump(tr, open("%s/%s_truth.json" % (OUT, tag), "w"))


class Survey(C.Reference):
    def __init__(self):
        super().__init__(log=lambda *a: P("[ref]", *a))
        self.done_stages = set()

    def do(self, maker):
        o = yield maker
        self.o, self.tr = o, o["truth"]
        if not getattr(self, "capturing", False):
            o_new = yield from self.hooks(o)
            if o_new is not o:
                o = dict(o_new, events=o["events"] + o_new["events"])
                self.o, self.tr = o, o["truth"]
        return o

    def overhead(self, tag):
        cur = C.T_of(self.tr["tcp_target"])
        swung = cur[0, 3] > 0.15
        q0 = np.array(self.tr["q_target"])
        if swung:
            yield from self.lift(-0.05)
            q1 = np.array(self.tr["q_target"])
            yield from self.move_joints(q1 + [2.3, 0, 0, 0, 0, 0], speed=1.0)
        o = yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 4}, "cams": ["overhead"]})
        save(o, tag, o["truth"])
        if swung:
            yield from self.move_joints(q1, speed=1.0)
            yield from self.move_joints(q0, speed=0.5)
        P("overhead", tag, "t_sim", o["time"])

    def wrist_over(self, fids, tag):
        """wrist frames over the given fasteners; the arm returns to where it was (joints) afterwards"""
        q0 = np.array(self.tr["q_target"])
        yield from self.lift(-0.05)
        for k, fid in enumerate(fids):
            f = self.tr["fasteners"][fid]
            top = np.array(f["top"])
            for yi, yaw in enumerate((0.0, math.pi / 2)):
                R = C.rot_z(yaw) @ C.DOCK_R
                goal = C.mk(R, top + [0, 0, 0.105])
                yield from self.go(goal, -0.05, yaw="goal")
                yield from self.wait(0.3)
                o = yield from self.do(lambda ob: {"move": {"type": "hold", "ticks": 2}, "cams": ["wrist"]})
                save(o, "%s_%s_y%d" % (tag, fid, yi), {"fastener": f, "fid": fid})
        yield from self.lift(-0.05)
        yield from self.move_joints(q0, speed=0.8)
        P("wrist", tag, len(fids), "fasteners")

    def hooks(self, o):
        tr = self.tr
        st = self.done_stages
        self.capturing = True
        try:
            if "P1" not in st:
                st.add("P1")
                yield from self.overhead("P1_lid")
            if "W1" not in st and o["tool"] == "nutrunner" and not tr["isolated"]:
                st.add("W1")
                yield from self.wrist_over(sorted(f for f, v in tr["fasteners"].items() if v["group"] == "lid"), "W1")
            lid = tr["parts"].get("lid")
            if "P2" not in st and lid and lid["left_seat"] and not lid["grasped"]:
                st.add("P2")
                yield from self.overhead("P2_open")
            if "P3" not in st and tr["isolated"]:
                st.add("P3")
                yield from self.overhead("P3_isolated")
            if "W2" not in st and tr["isolated"] and o["tool"] == "nutrunner":
                st.add("W2")
                yield from self.wrist_over(sorted(f for f, v in tr["fasteners"].items() if v["group"] != "lid"), "W2")
            if "P4" not in st and "W2" in st and o["tool"] == "gripper":
                st.add("P4")
                yield from self.overhead("P4_unbolted")
            parts = tr["parts"]
            if "P5" not in st and "P4" in st and all(v["left_seat"] and not v["grasped"] for v in parts.values() if v["kind"] in ("busbar", "bracket")):
                st.add("P5")
                yield from self.overhead("P5_modules")
        finally:
            self.capturing = False
        return self.o


ref = Survey()
t, t0 = 0, time.time()
while t < 30000:
    obs["truth"] = cell.request({"cmd": "truth"})["truth"]
    a = ref.act(t, obs)
    rr = cell.request({"cmd": "step", "action": a})
    if "error" in rr:
        P("ACTION ERROR", rr, a)
        break
    obs = rr["obs"]
    if obs["events"]:
        P("events", obs["time"], obs["events"])
    t += 1
    if obs["ended"]:
        break
sc = cell.request({"cmd": "score"})["score"]
P("ENDED", obs["ended"], "t_sim", obs["time"], "steps", t, "wall", round(time.time() - t0), "credit", round(sc["credit"], 4), "stops", sc["protective_stops"])
meta["score"] = {k: sc[k] for k in ("credit", "detail", "stripped", "protective_stops")}
json.dump(meta, open(OUT + "/meta.json", "w"), indent=1)
cell.close()
