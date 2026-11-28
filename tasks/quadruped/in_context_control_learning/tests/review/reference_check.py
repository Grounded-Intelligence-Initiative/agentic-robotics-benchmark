"""Run the ordinary RGB/servo author reference on unfiltered seeds. Does not invoke a model."""
import argparse
import base64
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import time

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
from grader import replay
from reference import Controller
from service import Runtime


class FixedWiring(Controller):
    """Run the identical observation/probe sequence; only the retained map changes."""
    def identify(self):
        self.measured = super().identify()
        return (0,1,2,3)


class MatchedHorizon(Exception):
    pass


class Flat(Controller):
    def stance(self, dock):
        return "flat"


class StayLow(Controller):
    def stance(self, dock):
        return "duct" if dock in (2,3,4) else "flat"


def check(seed, index, out, capture, matched_dir, mode, diagnostic_poses):
    out.mkdir()
    start = time.monotonic()
    runtime = Runtime(out/"ledger", seed, index)
    error = None
    captured, poses, probe_frames = set(), 0, 0
    prefix = hashlib.sha256()
    first_route_rgb = None
    baseline = json.loads((matched_dir/str(seed)/"result.json").read_text()) if matched_dir else None
    if baseline and not (baseline["replay_ok"] and not baseline["error"] and baseline["mode"]=="adaptive"):
        raise ValueError("need an exact adaptive trace on the same seed")
    horizon = baseline["steps"] if baseline else diagnostic_poses
    stop = "complete"

    def request(method, **params):
        nonlocal poses, probe_frames, first_route_rgb
        if method == "act" and horizon and poses >= horizon:
            raise MatchedHorizon
        result = runtime.call({"method": method, "params": params})
        poses += method == "act"
        if runtime.count <= 10:  # neutral RGB, then three identical (single port, RGB, rack) probes
            prefix.update(json.dumps([method,params,result],sort_keys=True).encode())
        if method == "observe":
            if not result["dock"]:
                label = f"commission-{probe_frames}"
                probe_frames += 1
            else:
                label = f"dock-{result['dock']}-near-{int(result['approach_reached'])}"
                if first_route_rgb is None:
                    first_route_rgb = hashlib.sha256(base64.b64decode(result["images"][0])).hexdigest()
            if capture and label not in captured:
                (out/(label+".png")).write_bytes(base64.b64decode(result["images"][0]))
                captured.add(label)
        return result

    try:
        controller = {"adaptive":Controller,"flat":Flat,"stay-low":StayLow,"fixed-wiring":FixedWiring}[mode](request)
        controller.solve()
    except MatchedHorizon:
        stop = "matched pose horizon" if baseline else "author diagnostic pose horizon"
    except RuntimeError as exc:
        if str(exc).startswith("reference fell:"):
            stop = "physical fall"
        else:
            error, stop = repr(exc), "error"
    except Exception as exc:
        error, stop = repr(exc), "error"
    finally:
        runtime.world.close()
    state = json.loads((out/"ledger/state.json").read_text())
    state["events"] = [json.loads(s) for s in (out/"ledger/events.jsonl").read_text().splitlines()]
    try:
        ok = replay(state) == state["metrics"] and len(state["events"]) == state["requests"]
    except (Exception, SystemExit) as exc:
        ok, error = False, f"{error or ''} replay: {exc!r}"
    chain, docks = hashlib.sha256(), {}
    for event in state["events"]:
        chain.update(json.dumps(event,sort_keys=True).encode())
        if event.get("delivered"):
            docks.setdefault(str(event["delivered"]),chain.hexdigest())
    row = {"seed": seed, "index": index, "mode": mode, "journal_prefix_at_dock":docks,
           "probe_readings": [list(v) for v in controller.probe_readings],
           "inferred": list(getattr(controller,"measured",controller.legs_by_port)),
           "used": list(controller.legs_by_port), "probe_prefix_sha256": prefix.hexdigest(),
           "route_start_rgb_sha256": first_route_rgb, "stop_reason": stop,
           "matched_poses": baseline["steps"] if baseline else None, "diagnostic_poses":horizon,
           **state["metrics"], "requests": state["requests"],
           "replay_ok": ok, "error": error, "wall_seconds": round(time.monotonic()-start, 3)}
    if baseline:
        row["matched_observations"] = all(row[k] == baseline[k] for k in (
            "probe_prefix_sha256","probe_readings","inferred","route_start_rgb_sha256"))
        checkpoint = "2" if mode=="stay-low" else "1"
        if mode in ("flat","stay-low"):
            row["matched_initial_route"] = (checkpoint in docks and
                docks[checkpoint]==baseline["journal_prefix_at_dock"].get(checkpoint))
    (out/"result.json").write_text(json.dumps(row, indent=2)+"\n")
    print(json.dumps(row), flush=True)
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--seed", type=int)
    group.add_argument("--all-hidden", action="store_true")
    group.add_argument("--first",type=int,choices=range(1,25),help="prefix of the registered order")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--workers", choices=(1, 2), type=int, default=1)
    p.add_argument("--matched-adaptive",type=Path,help="paired adaptive results; match each actual pose opportunity")
    p.add_argument("--mode",choices=("adaptive","flat","stay-low","fixed-wiring"),default="adaptive")
    p.add_argument("--author-poses",type=int,help="external finite diagnostic only; not used by native validate")
    args = p.parse_args()
    if (args.mode!="adaptive") != bool(args.matched_adaptive) or (args.author_poses is not None and args.author_poses<=0):
        p.error("controls require paired adaptive results; diagnostic horizon must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    names = ["tests/"+n+".py" for n in ("world", "lane", "service", "audit", "grader")]
    names += ["tests/hidden_scenes.json", "solution/reference.py", "tests/review/reference_check.py"]
    receipt = {"scope": "scripted author reference, not a model or an official validate run",
               "argv": sys.argv, "sha256": {n: hashlib.sha256((TASK/n).read_bytes()).hexdigest() for n in names}}
    (args.output/"receipt.json").write_text(json.dumps(receipt, indent=2)+"\n")
    seeds = hidden[:args.first] if args.all_hidden or args.first else [args.seed]
    cases = [(s, hidden.index(s) if s in hidden else -1, args.output/str(s), i == 0, args.matched_adaptive,
              args.mode,args.author_poses)
             for i, s in enumerate(seeds)]
    if args.workers == 1:
        rows = [check(*case) for case in cases]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            jobs = [pool.submit(check, *case) for case in cases]
            rows = [job.result() for job in jobs]  # report the original registered order
    # A well-formed fixed control can honestly succeed. No threshold on how badly it must score.
    passed = all(r["replay_ok"] and not r["error"] and
                 ((r["matched_observations"] and r.get("matched_initial_route",True))
                  if args.matched_adaptive else r["milestone_fraction"] == 1)
                 for r in rows)
    (args.output/"summary.json").write_text(json.dumps({"passed": passed, "runs": rows}, indent=2)+"\n")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
