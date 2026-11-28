"""Offline replay of an existing direct-action journal. Never connects to a participant or model."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

task_tests = Path("/task/tests") if Path("/task/tests/world.py").is_file() else Path(__file__).resolve().parents[1]
sys.path.insert(0,str(task_tests))
from grader import replay
from service import Runtime


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--history",type=Path,required=True,help="private state.json, events.jsonl and original verification.json")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--original",action="store_true",help="prove exact original-world reproduction rather than transfer")
    args=p.parse_args()
    args.output.mkdir(exist_ok=False)
    state=json.loads((args.history/"state.json").read_text())
    old=json.loads((args.history/"verification.json").read_text())
    raw=(args.history/"events.jsonl").read_bytes()
    events=[json.loads(l) for l in raw.splitlines()]
    assert len(events)==state["requests"]==old["metrics"]["ledger_events_matched"]
    assert old["metrics"]["direct_control_audit_passed"]==1 and not old["metrics"]["scripted_oracle"]
    record={"kind":"historical open-loop control, NOT a fresh model run","source_sha256":hashlib.sha256(raw).hexdigest(),
            "scene_index":state["scene_index"],"seed":state["seed"],"original_reward":old["reward"],
            "events_presented":len(events),"source_poses":sum(row["method"]=="act" for row in events)}
    if args.original:
        assert replay(dict(state,events=events))==state["metrics"]
        record.update(exact_original_physics_and_fingerprints=True,metrics=state["metrics"])
    else:
        runtime=Runtime(args.output/"ledger",state["seed"],state["scene_index"],render=False)
        errors=[]
        try:
            for i,row in enumerate(events):
                try:
                    # Keep EVERY command, including advances/recoveries after transfer diverges. Discard all replies.
                    runtime.call({key:row[key] for key in ("method","params")})
                except (ValueError,TypeError,KeyError,OverflowError) as exc:
                    errors.append({"event":i+1,"method":row["method"],"error":str(exc)})
        finally:
            runtime.world.close()
        transfer=json.loads((args.output/"ledger/state.json").read_text())
        journal=[json.loads(l) for l in (args.output/"ledger/events.jsonl").read_text().splitlines()]
        assert [(e["method"],e["params"]) for e in events]==[(e["method"],e["params"]) for e in journal]
        assert replay(dict(transfer,events=journal))==transfer["metrics"]
        record.update(rejected=errors,exact_transfer_replay=True,metrics=transfer["metrics"])
    (args.output/"result.json").write_text(json.dumps(record,indent=2)+"\n")
    print(json.dumps(record),flush=True)


if __name__=="__main__":
    main()
