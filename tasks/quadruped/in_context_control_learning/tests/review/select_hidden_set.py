"""The hidden set's pre-registered selection (RULE.md in hidden_set/), run on the host with Docker.

    python3 tests/review/select_hidden_set.py --image <task image> --jobs 20 [--out <scratch dir>]

Runs every candidate of hidden_set/candidates.json once, in parallel containers (2 CPUs and 4 GB each, no network),
with run_episode.sh; appends one row per candidate to hidden_set/selection.jsonl; applies the rule (the first 24
candidates in drawn order on which the oracle reaches all six points and the grader's replay agrees with the service);
writes tests/hidden_scenes.json and its copy environment/setup/scenes.json; prints the hollowing check.
"""
import argparse
import concurrent.futures as cf
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

TASK = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent/"hidden_set"
KEEP = 24


def features(image, seeds):
    """Scene.features() of every seed, computed in the task image (the host needs no MuJoCo)."""
    code = ("import json, sys; sys.path.insert(0, '/task/tests'); from world import Scene; "
            "print(json.dumps({s: Scene(int(s)).features() for s in sys.argv[1:]}))")
    out = subprocess.run(["docker", "run", "--rm", "--network", "none", "-v", f"{TASK}:/task:ro", "--entrypoint",
                          "/opt/venv/bin/python3", image, "-c", code, *map(str, seeds)],
                         capture_output=True, text=True, check=True).stdout
    return {int(k): v for k, v in json.loads(out.strip().splitlines()[-1]).items()}


def run(image, seed, out):
    name = f"icl-select-{seed}"
    subprocess.run(["docker", "run", "--rm", "--name", name, "--network", "none", "--cpus", "2", "--memory", "4g",
                    "-e", "LP_NUM_THREADS=2", "-v", f"{TASK}:/task:ro", "-v", f"{out}:/out",
                    "--entrypoint", "bash", image, "/task/tests/review/run_episode.sh", str(seed), "/out"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    row = {"seed": seed}
    try:
        verdict = json.loads((out/f"{seed}.verdict.json").read_text())
        row.update(reward=verdict["rewards"]["reward"], **{k: verdict["metrics"].get(k) for k in (
            "delivered", "steps", "requests", "collisions")})
    except (OSError, ValueError, KeyError) as exc:
        row.update(reward=None, error=f"no verdict: {exc}; {(out/f'{seed}.grader.log').read_text()[-300:] if (out/f'{seed}.grader.log').exists() else ''}")
    row["oracle_seconds"] = float((out/f"{seed}.time").read_text()) if (out/f"{seed}.time").exists() else None
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--image", required=True)
    p.add_argument("--jobs", type=int, default=20)
    p.add_argument("--out", type=Path, default=Path("/tmp/icl-selection"))
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    os.chmod(args.out, 0o777)
    candidates = json.loads((HERE/"candidates.json").read_text())["seeds"]
    with cf.ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(lambda s: run(args.image, s, args.out), candidates))
    scene = features(args.image, candidates)
    with open(HERE/"selection.jsonl", "w") as handle:
        for order, row in enumerate(rows):
            row = {"order": order, **row, **scene[row["seed"]]}
            row["kept"] = False
            rows[order] = row
        kept = 0
        for row in rows:
            if kept < KEEP and row.get("reward") == 1.0 and row.get("delivered") == 6:
                row["kept"], kept = True, kept+1
            handle.write(json.dumps(row)+"\n")
    seeds = [r["seed"] for r in rows if r["kept"]]
    if len(seeds) < KEEP:
        sys.exit(f"only {len(seeds)} candidates passed; the rule needs {KEEP}")
    text = json.dumps({"rule": "tests/review/hidden_set/RULE.md", "seeds": seeds}, indent=1)+"\n"
    (TASK/"tests/hidden_scenes.json").write_text(text)
    (TASK/"environment/setup/scenes.json").write_text(text)
    keys = ["heading_deg", "loaded_turn_deg", "forward_speed", "backward_ratio", "left_speed", "right_speed", "load",
            "wall_x0", "wall_x1", "narrow0", "wide0", "narrow1", "wide1", "long_legs", "straight_path_m", "steps"]
    print(f"{'feature':16s} {'kept mean':>10s} {'all mean':>10s} {'kept min':>9s} {'kept max':>9s}")
    for k in keys:
        a = [r[k] for r in rows if r.get(k) is not None]
        b = [r[k] for r in rows if r["kept"] and r.get(k) is not None]
        print(f"{k:16s} {statistics.mean(b):10.3f} {statistics.mean(a):10.3f} {min(b):9.3f} {max(b):9.3f}")
    print(f"kept {len(seeds)} of {len(rows)}; dropped before the 24th kept: "
          f"{[r['order'] for r in rows if not r['kept'] and r['order'] <= max(x['order'] for x in rows if x['kept'])]}")


if __name__ == "__main__":
    main()
