"""The cell as a process: `python3 -I -s server.py [--grader]`, one pack per process.

It speaks one JSON object per line: requests on stdin, replies on the descriptor that was stdout when it started
(everything Isaac Sim prints goes to stderr instead).

    {"cmd": "reset", "pack": <pack>, "config": {...}, "render": true}  -> {"obs": <observation>, "info": {...}}
    {"cmd": "step", "action": <action>}                                -> {"obs": <observation>}
    {"cmd": "truth"}        (--grader only)                            -> {"truth": {...}}
    {"cmd": "score"}        (--grader only)                            -> {"score": {...}}   ("final": true ends the pack first)
    {"cmd": "close"}                                                   -> {"ok": true}, then exit

A bad action is answered with {"error": "bad action: ..."} and changes nothing. An internal failure is answered
with {"fatal": "..."}: the cell is then gone.
"""

import json
import os
import sys
import time
import traceback


def main():
    grader = "--grader" in sys.argv[1:]
    out = os.fdopen(os.dup(1), "w", buffering=1)
    os.dup2(2, 1)                                   # Kit prints on stdout: keep the wire clean
    sys.stdout = sys.stderr
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(here))

    def reply(obj):
        out.write(json.dumps(obj, separators=(",", ":")) + "\n")
        out.flush()

    def log(*a):
        print("[packcell %7.1f]" % (time.time() - t0), *a, file=sys.stderr, flush=True)

    t0 = time.time()
    try:
        from isaacsim import SimulationApp
        cfg = {"headless": True, "multi_gpu": False}
        gpu = os.environ.get("PACKCELL_GPU")
        if gpu not in (None, ""):
            cfg.update(active_gpu=int(gpu), physics_gpu=int(gpu))
        kit = os.environ.get("PACKCELL_KIT_DIR")
        if kit:
            for d in ("cache", "data", "logs"):
                os.makedirs(os.path.join(kit, d), exist_ok=True)
            cfg["extra_args"] = ["--/app/folder/cache=%s/cache" % kit, "--/app/folder/data=%s/data" % kit,
                                 "--/app/folder/logs=%s/logs" % kit]
        app = SimulationApp(cfg)
        from packcell import series as S
        from packcell import wire
        from packcell.cell import Cell
        log("app up")
    except Exception:  # noqa: BLE001
        reply({"fatal": "the simulator did not start: " + traceback.format_exc()[-1500:]})
        return 1
    robot_usd = os.environ.get("PACKCELL_ROBOT_USD", "/opt/assets/isaac/Isaac/Robots/UniversalRobots/ur20/ur20.usd")
    cell = None
    reply({"ready": True, "version": S.VERSION, "start_s": round(time.time() - t0, 1)})
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
            cmd = msg.get("cmd")
            if cmd == "reset":
                if cell is not None:
                    reply({"error": "one pack per process"})
                    continue
                cell = Cell(app, msg["pack"], msg.get("config"), robot_usd=robot_usd, render=bool(msg.get("render", True)), log=log)
                cams = [c for c in (msg.get("cams") or []) if c in wire.CAMS]
                reply({"obs": cell.observe(cams=cams), "info": {"render_pinned": getattr(cell, "render_pinned", {})}})
            elif cmd == "step":
                if cell is None:
                    reply({"error": "reset first"})
                    continue
                try:
                    action = wire.check_action(msg.get("action"), cell.cfg["max_ticks"])
                except ValueError as exc:
                    reply({"error": "bad action: %s" % exc})
                    continue
                reply({"obs": cell.step(action)})
            elif cmd == "truth" and grader and cell is not None:
                reply({"truth": cell.truth()})
            elif cmd == "score" and grader and cell is not None:
                if msg.get("final") and not cell.ended:
                    cell.ended = "given_up"            # the grader gave the controller up: nothing moves any more
                reply({"score": cell.score()})
            elif cmd == "close":
                reply({"ok": True})
                break
            else:
                reply({"error": "unknown request"})
        except Exception:  # noqa: BLE001
            log("FATAL", traceback.format_exc())
            reply({"fatal": traceback.format_exc()[-1500:]})
            break
    try:
        app.close()
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    os._exit(main())
