"""Linux process-group CPU accounting for the controller and its reaped workers.

The kernel retains a waited child's CPU time in its parent's cutime/cstime. Count
those fields as well as live processes, so short-lived workers do not disappear
between reply samples. Read ancestors before descendants to avoid counting a
child both while alive and after its time has transferred to a waiting parent.
This is reply-budget accounting, not a substitute for process isolation.
"""

import os
from pathlib import Path


class CpuMeter:
    def __init__(self, pgid, proc_root="/proc"):
        self.pgid = int(pgid)
        self.root = Path(proc_root)
        self.tick = float(os.sysconf("SC_CLK_TCK"))
        self.previous = 0.0

    def _stat(self, pid):
        try:
            text = (self.root / str(pid) / "stat").read_text()
            fields = text[text.rindex(")") + 2:].split()
            # fields[0] is field 3 (state); field 22 identifies PID reuse.
            return (int(fields[1]), int(fields[2]), int(fields[19]),
                    sum(int(fields[i]) for i in (11, 12, 13, 14)) / self.tick)
        except (OSError, ValueError, IndexError):
            return None

    def seconds(self):
        # Discover every sample. A 40-call PID cache misses newly spawned workers.
        members = {}
        for path in self.root.iterdir():
            if path.name.isdigit():
                row = self._stat(path.name)
                if row is not None and row[1] == self.pgid:
                    members[int(path.name)] = row

        def depth(pid):
            seen = set()
            while pid in members and pid not in seen:
                seen.add(pid)
                pid = members[pid][0]
            return len(seen)

        order = sorted(members, key=lambda pid: (depth(pid), pid))
        total = 0.0
        for attempt in range(2):
            total = 0.0
            disappeared = False
            for pid in order:
                row = self._stat(pid)
                if row is None or row[1:3] != members[pid][1:3]:
                    disappeared = True
                    continue
                total += row[3]
            if not disappeared:
                break
            # A child may have been reaped after its parent's sample. Resample
            # the parent first; its waited-child counter now retains that work.
        self.previous = max(self.previous, total)
        return self.previous
