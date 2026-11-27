"""Run a controller on a practice shift, the way the grader runs it on a hidden one.

    python3 practice.py [--submission ~/submission] [--seed 1000] [--parts 20] [--out run_dir]

Same cell, robot, items, prices and wire as the graded shifts. Practice seeds are yours to choose; the graded shifts use
seeds that are not in this image, and every seed has its own constants. The summary printed at the end (per-part
outcome and credit) exists only here: a graded controller gets no such feedback while it runs.
"""
import argparse, json, os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from sim_client import SimClient  # noqa: E402

CAMERAS = ("head", "left_wrist", "right_wrist")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--seed", type=int, default=1000); ap.add_argument("--parts", type=int, default=20)
    ap.add_argument("--out", default="practice_run"); ap.add_argument("--max-steps", type=int, default=30000)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    sol = subprocess.Popen(["bash", "run.sh"], cwd=a.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
    assert sol.stdout.readline().strip() == "READY", "the controller must print READY first"

    def cpu():
        """CPU seconds used by the controller's processes (its process group), as the grader measures them."""
        tick = float(os.sysconf("SC_CLK_TCK")); total = 0.0; pg = os.getpgid(sol.pid)
        for d in os.listdir("/proc"):
            if not d.isdigit(): continue
            try:
                f = open("/proc/{}/stat".format(d)).read(); x = f[f.rindex(")") + 2:].split()
                if int(x[2]) == pg: total += (int(x[11]) + int(x[12])) / tick
            except (OSError, ValueError, IndexError): pass
        return total

    def ask(msg):
        sol.stdin.write(json.dumps(msg) + "\n"); sol.stdin.flush(); return json.loads(sol.stdout.readline())

    sim = SimClient(os.path.join(HERE, "cell_server.py"), log_path=os.path.join(a.out, "simulator.log"))
    t0 = time.time()
    try:
        obs = sim.reset("drop", a.seed, a.parts, cameras=CAMERAS); ask({"type": "reset"}); t = 0; program = 0.0; start = int(obs["t"]); behind = False
        while not obs["done"] and t < a.max_steps:
            tic = time.time(); c0 = cpu(); reply = ask({"type": "act", "t": t, "obs": obs}); wall = time.time() - tic
            took = wall if wall > 2.0 else cpu() - c0; program += took                 # as the grader counts it: the controller's CPU time
            late = int(took * 30.0)                                                        # the grader's rule: the cell does not wait
            a26 = [float(v) for v in reply["action"]]; assert len(a26) == 26, "action: 23 robot numbers + 3 camera flags"
            obs = sim.step(a26[:23], [c for c, f in zip(CAMERAS, a26[23:]) if f > 0.5], late=late); t += 1
        s = sim.summary()
    finally:
        sim.close()
        try: ask({"type": "close"})
        except Exception: pass
        sol.kill()
    json.dump(s, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print("part_pickup_score {:.3f}  placed {}  held only {}  bystanders moved {}  ({} actions, {:.0f} s)".format(
        s["score"], s["placed"], s["held_only"], s["bystanders_moved"], t, time.time() - t0))
    print("program time {:.1f} s for {:.1f} s of simulated time{}".format(program, (int(obs["t"]) - start) / 30.0, "; steps the cell ran while the controller was still computing: {}".format(s.get("late_steps", 0))))
    for r in s.get("records", []): print("  part {:2d} {:24s} {:7s} {:10s} credit {}".format(r["index"], r["item"], r["outcome"], r["reason"], r["credit"]))


if __name__ == "__main__":
    main()
