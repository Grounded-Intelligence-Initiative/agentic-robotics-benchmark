"""Drive the graded episode with the task's privileged reference controller (solution/privileged/oracle.py).

The controller is unchanged: a program that speaks the offline grader's stdin/stdout wire (READY, reset,
act, close). This driver plays the grader's part of that wire and the agent's part of the graded episode:
it starts the episode through the agent's own client (graded_episode.py in the agent home) with the
reference token, so the service adds the simulator's ground truth to every observation as the offline
grader does for a submission that carries the token; then, step by step, it hands the controller the
observation and steps the episode with the controller's action, until the episode reaches its horizon.
A dropped connection is reopened and the episode continued from the observation it stands at.
Python 3.12 standard library.
"""

import argparse
import json
import os
import subprocess
import sys
import time


def log(msg):
    print("[drive_oracle] " + msg, file=sys.stderr, flush=True)


class Controller(object):
    """The controller process and its wire (one JSON object per line)."""

    def __init__(self, path):
        self.proc = subprocess.Popen([sys.executable, "-B", path], cwd=os.path.dirname(path), stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, universal_newlines=True, bufsize=1)
        line = self._line()
        if line.strip() != "READY":
            raise RuntimeError("the controller did not announce READY: %r" % line[:200])
        self.ask({"type": "reset"})

    def _line(self):
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("the controller closed its stdout (exit code %s)" % self.proc.poll())
        return line

    def ask(self, msg):
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        while True:
            line = self._line().strip()
            if line.startswith("{"):
                return json.loads(line)

    def act(self, t, obs):
        reply = self.ask({"type": "act", "t": int(t), "obs": obs})
        return reply["action"]

    def close(self):
        try:
            self.ask({"type": "close"})
        except Exception:  # noqa: BLE001
            pass
        try:
            self.proc.wait(timeout=30)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True)
    ap.add_argument("--token-file", required=True)
    ap.add_argument("--client", required=True, help="the agent's graded_episode.py")
    args = ap.parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(args.client)))
    from graded_episode import GradedEpisode, GradedEpisodeError

    with open(args.token_file) as fh:
        token = fh.read().strip()
    ctl = Controller(os.path.abspath(args.controller))
    t0 = time.monotonic()
    ep = None
    for attempt in range(1, 6):
        try:
            ep = GradedEpisode()
            log("status before the start: %s" % json.dumps(ep.status()))
            obs = ep.start(token=token)
            break
        except GradedEpisodeError as exc:
            if exc.code != "simulator_unavailable" or attempt == 5:
                raise
            log("start attempt %d: %s; retrying in 60 s" % (attempt, exc))
            ep.close()
            time.sleep(60.0)
    log("episode started after %.0f s" % (time.monotonic() - t0))
    steps, last_report = 0, time.monotonic()
    while not ep.done:
        action = ctl.act(obs["t"], obs)
        try:
            obs = ep.step(action)
        except GradedEpisodeError as exc:
            log("the service ended the episode at t=%s: %s" % (obs.get("t"), exc))
            break
        except (OSError, ValueError) as exc:
            log("connection lost at t=%s (%s); reconnecting" % (obs.get("t"), exc))
            try:
                ep.close()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(1.0)
            ep = GradedEpisode()
            st = ep.status()
            if st["ended"]:
                break
            if st["steps"] == obs["t"]:
                continue        # the step never reached the service: ask the controller again
            obs = ep.observe()
            continue
        steps += 1
        if time.monotonic() - last_report >= 600:
            last_report = time.monotonic()
            log("t=%d after %.0f s" % (obs["t"], last_report - t0))
    try:
        ep.finish()
    except GradedEpisodeError as exc:
        log("finish: %s" % exc)
    log("episode over at t=%s after %.0f s; %s" % (obs.get("t"), time.monotonic() - t0, json.dumps(ep.status())))
    ep.close()
    ctl.close()


if __name__ == "__main__":
    main()
