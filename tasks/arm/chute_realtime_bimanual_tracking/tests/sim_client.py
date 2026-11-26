"""Talk to cell_server.py from the task's own interpreter (stdlib only)."""
import json, os, subprocess, threading, queue

SIM_PYTHON = os.environ.get("CELL_SIM_PYTHON", "/opt/sim/bin/python")


class SimError(Exception):
    """The simulator failed: a task error, never the solver's fault."""


class SimClient(object):
    def __init__(self, server, python=None, ready_timeout=1800.0, step_timeout=300.0, env=None, log_path=None):
        self.step_timeout, self.ready_timeout = float(step_timeout), float(ready_timeout)
        self._log = open(log_path, "a") if log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen([python or SIM_PYTHON, server], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log, env=env, text=True, bufsize=1)
        self._q = queue.Queue(); threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout: self._q.put(line)
        self._q.put(None)

    def call(self, timeout=None, **req):
        try:
            self.proc.stdin.write(json.dumps(req) + "\n"); self.proc.stdin.flush()
            line = self._q.get(timeout=timeout or self.step_timeout)
        except (BrokenPipeError, queue.Empty) as exc:
            raise SimError("simulator did not answer {}: {}".format(req.get("op"), type(exc).__name__))
        if line is None: raise SimError("simulator exited during {}".format(req.get("op")))
        rep = json.loads(line)
        if not rep.get("ok"): raise SimError(rep.get("error", "request failed"))
        return rep

    def reset(self, station, seed, parts=20, cameras=(), salt=0, token=""):
        return self.call(timeout=self.ready_timeout, op="reset", station=station, seed=seed, parts=parts, cameras=list(cameras), salt=int(salt), token=str(token))["obs"]

    def step(self, action, cameras=()): return self.call(op="step", action=[float(v) for v in action], cameras=list(cameras))["obs"]
    def truth(self, token): return self.call(op="truth", token=token)["truth"]
    def summary(self): return self.call(op="summary")["summary"]

    def close(self):
        try: self.call(timeout=60, op="close")
        except Exception: pass
        try: self.proc.wait(timeout=60)
        except Exception: self.proc.kill()
