"""The cell as a process: `python3 -I -s server.py [--grader]`, one row section per process.

It speaks one JSON object per line: requests on stdin, replies on the descriptor that was stdout when it started
(everything Isaac Sim prints goes to stderr instead).

    {"cmd": "reset", "row": <row>, "config": {...}, "render": true}    -> {"obs": <observation>, "info": {...}}
    {"cmd": "step", "action": <action>}                                -> {"obs": <observation>}
    {"cmd": "truth"}        (--grader only)                            -> {"truth": {...}}
    {"cmd": "joints"}       (--grader only)                            -> {"joints": {"q": [...], "q_target": [...]}}
    {"cmd": "score"}        (--grader only)                            -> {"score": {...}}   ("final": true ends the row first)
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
        # Straight to the process's own stderr (the cell's log file), never through sys.stderr: Kit puts an interceptor
        # of its own there, and a line through it can raise (below). A log line must not be what ends a cell.
        try:
            os.write(2, ("[rowcell %7.1f] %s\n" % (time.time() - t0, " ".join(str(x) for x in a))).encode("utf-8", "replace"))
        except OSError:
            pass

    t0 = time.time()
    try:
        from isaacsim import SimulationApp
        # Omniverse RTX refuses to render on an NVIDIA driver older than R550 ("rtx driver verification failed"). The
        # check is off: whether the frames are usable is the canary's to decide (tests/verify.py), not the driver's
        # version. On a supported driver it would pass anyway.
        cfg = {"headless": True, "multi_gpu": False, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
        gpu = os.environ.get("ROWCELL_GPU")
        if gpu not in (None, ""):
            cfg.update(active_gpu=int(gpu), physics_gpu=int(gpu))
        kit = os.environ.get("ROWCELL_KIT_DIR")
        if kit:
            for d in ("cache", "data", "logs"):
                os.makedirs(os.path.join(kit, d), exist_ok=True)
            cfg["extra_args"] += ["--/app/folder/cache=%s/cache" % kit, "--/app/folder/data=%s/data" % kit,
                                  "--/app/folder/logs=%s/logs" % kit]
        app = SimulationApp(cfg)
        from rowcell import farm as F
        from rowcell import wire
        from rowcell.cell import Cell
        # Kit routes sys.stdout and sys.stderr through interceptors that push every line onto a message queue of its
        # scripting layer. Once, in a grading sandbox, Kit came up without that queue (and took 125 s to start instead
        # of 13 s): every print then raised. A simulator in that state is not used: this line goes through the
        # interceptor on purpose, and when it raises the process says that it did not start and ends, for whoever
        # started it to start another (the grader does, once; the commissioning service does not count the session).
        print("[rowcell] the simulator's scripting layer answers", file=sys.stderr, flush=True)
        log("app up")
    except Exception:  # noqa: BLE001
        reply({"fatal": "the simulator did not start: " + traceback.format_exc()[-1500:]})
        return 1
    robot_usd = os.environ.get("ROWCELL_ROBOT_USD", "/opt/assets/isaac/Isaac/Robots/UniversalRobots/ur20/ur20.usd")
    cell = None
    reply({"ready": True, "version": F.VERSION, "start_s": round(time.time() - t0, 1)})
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
            cmd = msg.get("cmd")
            if cmd == "reset":
                if cell is not None:
                    reply({"error": "one row section per process"})
                    continue
                cell = Cell(app, msg["row"], msg.get("config"), robot_usd=robot_usd, render=bool(msg.get("render", True)), log=log)
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
            elif cmd == "joints" and grader and cell is not None:
                reply({"joints": cell.joints()})
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
