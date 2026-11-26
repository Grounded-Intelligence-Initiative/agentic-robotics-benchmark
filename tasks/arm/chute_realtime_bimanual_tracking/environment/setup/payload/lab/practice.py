"""Run a controller on a practice shift, the way the grader runs it on a hidden one.

    python3 practice.py [--submission ~/submission] [--seed 1000] [--parts 20] [--out run_dir]

Same cell, robot, items, prices and wire as the graded shifts. Practice seeds are yours to choose; the graded shifts use
seeds that are not in this image, and every seed has its own constants. The summary printed at the end (per-part
outcome and credit) exists only here: a graded controller gets no such feedback while it runs.
"""
import argparse, json, os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from sim_client import SimClient  # noqa: E402
from cpu_meter import CpuMeter  # noqa: E402

CAMERAS = ("head", "left_wrist", "right_wrist")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--seed", type=int, default=1000); ap.add_argument("--parts", type=int, default=20)
    ap.add_argument("--out", default="practice_run"); ap.add_argument("--max-steps", type=int, default=30000)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    sol = subprocess.Popen(["bash", "run.sh"], cwd=a.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
    assert sol.stdout.readline().strip() == "READY", "the controller must print READY first"

    def ask(msg):
        sol.stdin.write(json.dumps(msg) + "\n"); sol.stdin.flush(); return json.loads(sol.stdout.readline())

    sim = SimClient(os.path.join(HERE, "cell_server.py"), log_path=os.path.join(a.out, "simulator.log"))
    t0 = time.monotonic()
    try:
        obs = sim.reset("track", a.seed, a.parts, cameras=CAMERAS); ask({"type": "reset"}); t = 0; program = 0.0; start = int(obs["t"]); behind = False; late = 0; slowest = 0.0; meter = CpuMeter(sol.pid); c0 = meter.seconds()
        while not obs["done"] and t < a.max_steps:
            tic = time.monotonic(); reply = ask({"type": "act", "t": t, "obs": obs}); wall = time.monotonic() - tic
            c1 = meter.seconds(); took = max(0.0, c1 - c0); c0 = c1; took = max(took, wall) if wall > 10.0 else took; program += took
            if took > (0.1 if t > 0 else 5.0): late += 1                                   # the grader's rules: a late reply ...
            slowest = max(slowest, took if t > 0 else 0.0)
            if late > 3 or program > (int(obs["t"]) - start) / 30.0 + 10.0: behind = True; break      # ... the fourth ends the shift
            a26 = [float(v) for v in reply["action"]]; assert len(a26) == 26, "action: 23 robot numbers + 3 camera flags"
            obs = sim.step(a26[:23], [c for c, f in zip(CAMERAS, a26[23:]) if f > 0.5]); t += 1
        s = sim.summary()
    finally:
        sim.close()
        try: ask({"type": "close"})
        except Exception: pass
        os.killpg(sol.pid, 9)
    json.dump(s, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print("part_pickup_score {:.3f}  placed {}  held only {}  bystanders moved {}  ({} actions, {:.0f} s)".format(
        s["score"], s["placed"], s["held_only"], s["bystanders_moved"], t, time.monotonic() - t0))
    print("program time {:.1f} s for {:.1f} s of simulated time{}".format(program, (int(obs["t"]) - start) / 30.0, "  FELL BEHIND REAL TIME: shift ended" if behind else ""))
    print("late replies (over 0.1 s of CPU time): {} of 3 allowed; slowest reply {:.3f} s".format(late, slowest))
    print("camera latency of this shift: {} steps".format(s.get("camera_latency_steps")))
    for r in s.get("records", []): print("  part {:2d} {:24s} {:7s} {:10s} credit {}".format(r["index"], r["item"], r["outcome"], r["reason"], r["credit"]))


if __name__ == "__main__":
    main()
