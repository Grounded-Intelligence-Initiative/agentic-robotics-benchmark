"""Sequential pre-registered RGB trials; private physics is logged but never returned to the policy."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
from grader import replay
from reference import Controller, PartialView, fixture_view
from service import Runtime
from support_transport import Bridge


class OrdinaryTrack(Controller):
    def foot_track(self, side):
        return (0.,0.), 0.


class FixedLeft(Controller):
    def foot_track(self, side):
        return super().foot_track(1)


class EndTrial(Exception):
    pass


def trial(out, seed, index, mode, end, horizon, baseline, mcp):
    out.mkdir()
    sim = Runtime(out/"ledger", seed, index)
    prefix = hashlib.sha256()
    w = sim.world
    policy = None
    error = None
    reason = "complete"
    trace = (out/"trace.jsonl").open("w", buffering=1)
    bridge = Bridge(sim, out, TASK) if mcp else None

    def request(method, **params):
        if method == "act" and w.steps >= horizon:
            raise EndTrial("matched pose horizon" if baseline else "diagnostic pose horizon")
        if method == "advance" and w.delivered >= end:
            raise EndTrial("target dock completed")
        r = bridge.call(method, params) if bridge else sim.call({"method": method, "params": params})
        if not w.current or (w.current == 1 and w.steps == w.cal_steps):
            prefix.update(json.dumps([method, params, r], sort_keys=True).encode())
        if method == "observe":
            xyz = w.data.xpos[w.trunk].tolist()
            rot = w.data.xmat[w.trunk].reshape(3,3)
            try:
                visual = list(fixture_view(r["images"][0])) if w.current in (1,2) else None
            except PartialView:
                visual = None
            trace.write(json.dumps({"dock":w.current,"poses":w.steps,"xyz":xyz,
                "yaw":math.atan2(rot[1,0],rot[0,0]), "rgb_estimate":visual,
                "feet_xyz":w.data.geom_xpos[w.feet].tolist(), "dwell":w.dwell, **w.passage})+"\n")
            label = "launch" if w.current == 1 and w.steps == w.cal_steps else None
            if w.current == 1 and w.approaches:
                label = "approach"
            if w.current == 1 and not label and w.passage["both_runner_ticks"]:
                label = "both"
            if label and not (out/(label+".png")).exists():
                (out/(label+".png")).write_bytes(base64.b64decode(r["images"][0]))
        return r

    try:
        policy = {"adaptive":Controller,"ordinary":OrdinaryTrack,"fixed-left":FixedLeft}[mode](request)
        policy.solve()
    except EndTrial as exc:
        reason = str(exc)
    except Exception as exc:
        error = repr(exc)
        reason = "physical fall" if str(exc).startswith("reference fell:") else "controller error"
    finally:
        if bridge:
            bridge.close()
        trace.close()
        w.close()
    state = json.loads((out/"ledger/state.json").read_text())
    state["events"] = [json.loads(line) for line in (out/"ledger/events.jsonl").read_text().splitlines()]
    ok = replay(state) == state["metrics"] and len(state["events"]) == state["requests"]
    row = {"index":index,"seed":seed,"mode":mode,"target":end,"horizon":horizon,"reason":reason,
           "error":error,"replay_ok":ok,"mcp_stdio":mcp,"seen_side":getattr(policy,"runner_side",None),
           "used_loom":list(policy.legs_by_port),"prefix":prefix.hexdigest(), **state["metrics"]}
    row["matched_prefix"] = row["prefix"] == baseline["prefix"] if baseline else None
    (out/"result.json").write_text(json.dumps(row,indent=2)+"\n")
    print(json.dumps(row), flush=True)
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--end",type=int,choices=(1,2,6),default=1)
    p.add_argument("--poses",type=int,default=640)
    p.add_argument("--mcp",action="store_true",help="use actual stdio MCP + Unix socket; private bench only")
    p.add_argument("--matched",type=Path,help="prior adaptive directory; use its actual dock-1 pose count")
    p.add_argument("--control",choices=("ordinary","fixed-left"),default="ordinary")
    a = p.parse_args()
    if a.poses < 1 or (a.matched and a.end != 1):
        p.error("invalid diagnostic horizon or matched endpoint")
    a.output.mkdir()
    names = ["tests/"+s+".py" for s in ("world","lane","service","grader")]
    names += ["solution/reference.py","tests/review/support_pair.py","tests/review/support-registration.md",
              "tests/review/support_transport.py","environment/setup/payload/mcp_stdio.py"]
    (a.output/"receipt.json").write_text(json.dumps({"argv":sys.argv,"sources":{
        n:hashlib.sha256((TASK/n).read_bytes()).hexdigest() for n in names}},indent=2)+"\n")
    seeds = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"][:2]
    rows = []
    for i, seed in enumerate(seeds):
        baseline = json.loads((a.matched/str(i)/"result.json").read_text()) if a.matched else None
        if baseline and not (baseline["replay_ok"] and baseline["dock_1_pose"] and baseline["mode"]=="adaptive"):
            raise ValueError("need an exact same-scene adaptive dock-1 trace")
        rows.append(trial(a.output/str(i),seed,i,a.control if baseline else "adaptive",a.end,
                          baseline["dock_1_pose"] if baseline else a.poses,baseline,a.mcp))
    (a.output/"summary.json").write_text(json.dumps(rows,indent=2)+"\n")
    return int(not all(r["replay_ok"] and (r["matched_prefix"] and r["reason"] != "controller error"
                       if a.matched else r["delivered"] >= a.end) for r in rows))


if __name__ == "__main__":
    sys.exit(main())
