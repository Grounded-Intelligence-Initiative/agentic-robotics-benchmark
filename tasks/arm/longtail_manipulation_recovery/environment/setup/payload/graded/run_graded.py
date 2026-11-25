"""Run a controller (a folder with a run.sh) on the GRADED episode, over the grading wire.

    /opt/venv/bin/python3 /home/user/graded/run_graded.py [--submission /home/user/submission]
        [--resume] [--max-steps N] [--finish] [--images DIR]

It starts `bash run.sh` in the folder as the practice grader does (working directory the folder, only PATH, HOME and
LANG set), reads READY, sends `reset`, then starts the graded episode (once per session: the start is charged at once
and never refunded, and it freezes your submission folder) or, with --resume, continues the one already started from
its last observation (the controller then gets a `reset` and its first `act` carries that observation's `t`). It sends
the episode's observations to the controller one `act` at a time and steps the episode with each reply, until the
episode ends or --max-steps steps were sent. Every step is asked once, and there is no controller-time cap and no
pause here: a reply has 300 s, READY 120 s. If the run stops before the episode's end (--max-steps, a controller fault,
a malformed action, which the service refuses), the episode stays where it is, to be continued by hand or by another
run; with --finish it is ended then. `--images DIR` saves the RGB images of the episode's last observation that
carried any (PNG). It prints how the run went and the episode's status, never whether the job was done.
"""

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HOME)
import wire_rules as W  # noqa: E402  (the grader's action check, in your home)
from graded.client import GradedEpisode, GradedError, decode_image  # noqa: E402

READY_TIMEOUT, REPLY_TIMEOUT = 120.0, 300.0
CONTROLLER_ENV = {"PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": "/home/user", "LANG": "C.UTF-8"}


class WireError(Exception):
    pass


class Controller(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1, start_new_session=True,
                                     env=dict(CONTROLLER_ENV))
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        t0 = time.monotonic()
        try:
            for _ in range(200):
                if self._read(READY_TIMEOUT - (time.monotonic() - t0), "READY").strip() == "READY":
                    return
            raise WireError("the controller never printed READY")
        except WireError:
            self.stop(polite=False)
            raise

    def _pump(self):
        while True:
            try:
                line = self.proc.stdout.readline(W.MAX_LINE + 2)
            except (ValueError, OSError, UnicodeDecodeError):
                break
            if not line or W.line_too_long(line):
                break
            self.lines.put(line)
        self.lines.put(None)

    def _read(self, timeout, what):
        try:
            line = self.lines.get(timeout=max(0.0, timeout))
        except queue.Empty:
            raise WireError("the controller exceeded its {} deadline".format(what))
        if line is None:
            raise WireError("the controller closed its stdout (or wrote a line over the length cap)")
        return line

    def request(self, msg, timeout=REPLY_TIMEOUT):
        t0 = time.monotonic()
        try:
            self.proc.stdin.write(json.dumps(msg) + "\n")
            self.proc.stdin.flush()
            for _ in range(1000):
                line = self._read(timeout - (time.monotonic() - t0), "%d s reply" % timeout).strip()
                if line:
                    break
            else:
                raise WireError("too many blank lines")
            reply = json.loads(line)
            if not isinstance(reply, dict):
                raise WireError("the reply is not a JSON object")
            return reply
        except (OSError, ValueError) as exc:
            raise WireError(str(exc)[:200])

    def stop(self, polite=True):
        if polite:
            try:
                self.request({"type": "close"}, timeout=10.0)
            except WireError:
                pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except OSError:
            pass


def has_images(obs):
    return any(isinstance(e, dict) and "rgb" in e for e in (obs.get("images") or {}).values())


def save_images(obs, folder):
    os.makedirs(folder, exist_ok=True)
    saved = []
    for cam, entry in sorted((obs.get("images") or {}).items()):
        raw, dtype, shape = decode_image(entry["rgb"])
        try:
            from PIL import Image
            path = os.path.join(folder, "t{:04d}_{}.png".format(int(obs["t"]), cam))
            Image.frombytes("RGB", (shape[1], shape[0]), raw).save(path)
        except ImportError:
            path = os.path.join(folder, "t{:04d}_{}_{}x{}.rgb".format(int(obs["t"]), cam, shape[1], shape[0]))
            with open(path, "wb") as f:
                f.write(raw)
        saved.append(path)
    return saved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default="/home/user/submission")
    ap.add_argument("--resume", action="store_true", help="continue the graded episode already started")
    ap.add_argument("--max-steps", type=int, default=0, help="stop after this many steps (0: run to the episode's end)")
    ap.add_argument("--finish", action="store_true", help="end the episode if this run stops before its end")
    ap.add_argument("--images", default=None)
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.submission, "run.sh")):
        print(json.dumps({"error": "no run.sh in {}".format(a.submission)}))
        return 2
    out = {"submission": a.submission, "resume": a.resume}
    try:
        g = GradedEpisode()
    except GradedError as exc:
        print(json.dumps({"error": str(exc)}))
        return 1
    ctl, sent, done, ended, last_images, stopped = None, 0, False, None, None, None
    try:
        ctl = Controller(a.submission)
        ctl.request({"type": "reset"})
        r = g.observe() if a.resume else g.start()
        obs, done = r["obs"], bool(r.get("done"))
        while not done and (not a.max_steps or sent < a.max_steps):
            if has_images(obs):
                last_images = obs
            reply = ctl.request({"type": "act", "t": int(obs["t"]), "obs": obs})
            if "action" not in reply:
                raise WireError("the reply has no action")
            try:
                W.check_action(reply["action"])
            except W.Malformed as exc:
                raise WireError("malformed action: {}".format(exc))
            r = g.step(reply["action"])
            sent += 1
            obs, done, ended = r["obs"], bool(r["done"]), r.get("ended")
        if not done:
            stopped = "stopped by --max-steps"
        if has_images(obs):
            last_images = obs
    except WireError as exc:
        stopped = "controller fault: {}".format(exc)
    except GradedError as exc:
        stopped = "graded episode: {}".format(exc)
    finally:
        if ctl is not None:
            ctl.stop()
        try:
            st = g.status()
            if a.finish and st["started"] and not st["ended"]:
                g.finish()
                st = g.status()
            out["status"] = st
        except (GradedError, OSError, ValueError) as exc:
            out["status_error"] = str(exc)
        g.close()
    if a.images and last_images is not None:
        out["images"] = save_images(last_images, a.images)
    out.update({"steps_sent": sent, "done": done, "ended": ended, "stopped": stopped,
                "note": "the graded episode; nothing here says whether the job was done"})
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
