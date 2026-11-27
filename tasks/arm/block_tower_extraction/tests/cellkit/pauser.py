"""Stop the program between requests (the rule of longtail_manipulation_recovery and kitchen_appliance_tending).

While the cell works on a request (simulation, rendering), every process of the program is stopped (SIGSTOP), so the
program never shares the CPUs with the cell and cannot compute ahead of a request; it is continued (SIGCONT) just before
its reply is written. `freeze` stops every process of the set with a running thread and confirms it thread by thread:
a scan that finds one running signals it again and scans anew (a fork chain is caught process by process); the pause
holds when two consecutive scans find the same processes, none running. Not reached within FREEZE_TIMEOUT_S, or more
than MAX_PROCESSES processes at a pause: the shift ends (`stalled`). After the cell's turn `breached` checks that the
pause held throughout (no process running, no CPU time used, no new process: a process can be continued by a timer
of its own, e.g. timer_create with SIGEV_SIGNAL / SIGCONT); `cpu_seconds` is what the wire bills as CPU time. The process set is the runner account's processes
(as root) or the program's session (an authoring run as one's own account). stdlib only.
"""

import os
import signal
import time

FREEZE_TIMEOUT_S = 5.0
MAX_PROCESSES = 512
STOPPED_STATES = ("T", "t")
GONE_STATES = ("Z", "X")


def _thread_states(pid):
    states = []
    try:
        tids = os.listdir("/proc/{}/task".format(pid))
    except OSError:
        return states
    for tid in tids:
        try:
            with open("/proc/{}/task/{}/stat".format(pid, tid)) as f:
                states.append(f.read().rsplit(")", 1)[1].split()[0])
        except (OSError, IndexError):
            continue
    return states


def _runs(pid):
    return any(st not in STOPPED_STATES and st not in GONE_STATES for st in _thread_states(pid))


def _alive(pid):
    return any(st not in GONE_STATES for st in _thread_states(pid))


def uid_pids(uid):
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as fh:
                for line in fh:
                    if line.startswith("Uid:"):
                        if int(line.split()[1]) == int(uid):
                            out.append(int(name))
                        break
        except (OSError, ValueError):
            continue
    return out


def session_pids(sid):
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/stat".format(name)) as fh:
                fields = fh.read().rsplit(")", 1)[1].split()
            if int(fields[3]) == int(sid):         # field 6 of /proc/pid/stat: the session id
                out.append(int(name))
        except (OSError, ValueError, IndexError):
            continue
    return out


class Pauser(object):
    def __init__(self, pids_fn):
        self.pids_fn = pids_fn
        self._stopped = set()
        self.unstoppable = []
        self.count = 0
        self.peak = 0
        self.pauses = 0
        self._cpu = {}
        self.cpu_cached = 0.0
        self._frozen_set = set()
        self.tck = float(os.sysconf("SC_CLK_TCK"))

    def freeze(self):
        previous = None
        deadline = time.monotonic() + FREEZE_TIMEOUT_S
        self.unstoppable = []
        pids = []
        while True:
            pids = self.pids_fn()
            running = [p for p in pids if _runs(p)]
            for pid in running:
                try:
                    os.kill(pid, signal.SIGSTOP)
                    self._stopped.add(pid)
                except OSError:
                    pass
            current = frozenset(pids)
            if not running and current == previous:
                break
            previous = None if running else current
            if time.monotonic() > deadline:
                self.unstoppable = [str(p) for p in running[:5]]
                break
            if running:
                time.sleep(0.0005)
        self.count = sum(1 for p in pids if _alive(p))
        self.peak = max(self.peak, self.count)
        self.pauses += 1
        self._frozen_set = set(pids)
        self.cpu_seconds(pids)
        return not self.unstoppable and self.count <= MAX_PROCESSES

    def _cpu_of(self, pid):
        try:
            with open("/proc/{}/stat".format(pid)) as f:
                fields = f.read().rsplit(")", 1)[1].split()
            return sum(int(fields[k]) for k in (11, 12, 13, 14)) / self.tck   # utime stime cutime cstime
        except (OSError, ValueError, IndexError):
            return None

    def cpu_seconds(self, pids=None):
        """CPU seconds the program's processes have used (user + system, waited-for children included); a process
        that has gone keeps what it was last seen to use, so the sum never decreases. `pids`: the processes to read
        (default: a fresh scan). The result is kept in `cpu_cached` (what the wire bills between scans)."""
        for pid in (self.pids_fn() if pids is None else pids):
            c = self._cpu_of(pid)
            if c is not None:
                self._cpu[pid] = max(self._cpu.get(pid, 0.0), c)
        self.cpu_cached = sum(self._cpu.values())
        return self.cpu_cached

    def breached(self, cpu_at, tolerance_s=0.03):
        """Why the pause did not hold through the cell's turn ("" when it held): a process running now, CPU time used
        since `cpu_at` (beyond scheduler tick rounding), or a process that was not there at the freeze."""
        pids = self.pids_fn()
        running = [p for p in pids if _runs(p)]
        if running:
            return "%d process(es) running" % len(running)
        new = [p for p in pids if p not in self._frozen_set and _alive(p)]
        if new:
            return "%d new process(es)" % len(new)
        used = self.cpu_seconds(pids) - cpu_at
        if used > tolerance_s:
            return "%.2f CPU s used" % used
        return ""

    def thaw(self):
        for pid in sorted(self._stopped):
            try:
                os.kill(pid, signal.SIGCONT)
            except OSError:
                pass
        self._stopped = set()

    def still_frozen(self):
        """True when no process of the set runs (a pause that holds); checked before the cell works."""
        return not any(_runs(p) for p in self.pids_fn())
