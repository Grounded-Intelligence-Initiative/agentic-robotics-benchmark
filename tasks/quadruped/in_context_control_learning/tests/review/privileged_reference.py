"""AUTHOR-ONLY v3 feasibility runs, on the remote CPU pool, never a participant tool.

--all-hidden uses every frozen seed in its registered order, without selection.
Every run writes its ledger, metrics, replay result and source digest, including
failures. --seed is for explicitly labelled development/debug runs only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
from grader import replay
from privileged import Controller, Session
from world import KIND, VERSION, World


def run(seed, index, output, render=False):
    start = time.monotonic()
    world = World(seed, render=render)
    session = Session(world)
    error = None
    try:
        session.call("observe", {})
        Controller(world, session.call).solve()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {"kind": KIND, "version": VERSION, "seed": seed, "scene_index": index,
              "requests": len(session.events), "events": session.events, "metrics": world.metrics(),
              "state_fingerprint": world.state_fingerprint()}
    (output/f"{seed}.ledger.json").write_text(json.dumps(ledger, indent=2)+"\n")
    if render:
        from PIL import Image
        Image.fromarray(world.image()).save(output/f"{seed}.final.png")
    world.close()
    try:
        replayed = replay(ledger)
        replay_ok = replayed == ledger["metrics"]
    except (Exception, SystemExit) as exc:
        replay_ok = False
        # Grader require() raises SystemExit for replay divergence. Record that
        # failed seed and continue the fixed cohort; do not swallow interrupts.
        error = (error or "")+f" replay: {type(exc).__name__}: {exc}"
    row = {"label": "PRIVILEGED v3 feasibility, not image-only or frontier evidence", "seed": seed,
           "scene_index": index, **ledger["metrics"], "requests": len(session.events),
           "replay_ok": replay_ok, "error": error, "wall_seconds": time.monotonic()-start}
    (output/f"{seed}.result.json").write_text(json.dumps(row, indent=2)+"\n")
    print(json.dumps(row), flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--all-hidden", action="store_true")
    choice.add_argument("--seed", type=int)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    sources = [TASK/"tests"/name for name in ("world.py", "service.py", "grader.py", "hidden_scenes.json")]
    sources += [TASK/"solution/privileged.py", Path(__file__)]
    (args.output/"receipt.json").write_text(json.dumps({
        "version": VERSION, "argv": sys.argv, "all_hidden": args.all_hidden,
        "source_sha256": {str(p.relative_to(TASK)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    }, indent=2)+"\n")
    seeds = hidden if args.all_hidden else [args.seed]
    rows = [run(s, hidden.index(s) if s in hidden else -1, args.output, args.render) for s in seeds]
    passed = all(r["delivered"] == 6 and r["replay_ok"] and r["error"] is None for r in rows)
    (args.output/"summary.json").write_text(json.dumps({"passed": passed, "runs": rows}, indent=2)+"\n")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
