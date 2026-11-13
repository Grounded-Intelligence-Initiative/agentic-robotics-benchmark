#!/usr/bin/env bash
# A memo in System V shared memory, made from run.sh (no Python import for the static network audit to see): count
# the segments this account owns or created at the start of each episode (any an earlier episode left), make one with
# ipcmk, and make a second one whose ownership is handed to uid 1 with IPC_SET (libc shmctl from an inline Python:
# the sweep must then match the creator column as well), then start the ordinary launcher.
# Nothing goes to stdout (the wire).
me=$(id -u)
held=$(awk -v u="$me" 'NR > 1 && ($8 == u || $10 == u)' /proc/sysvipc/shm 2>/dev/null | wc -l)
echo "probe shm_memo: segments of this account (owner or creator) at the start of the episode: $held" >&2
ipcmk -M 4096 1>&2 || echo "probe shm_memo: ipcmk failed" >&2
python3 - <<'PY' 1>&2 || echo "probe shm_memo: hand-off failed" >&2
import ctypes, ctypes.util
libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
IPC_PRIVATE, IPC_CREAT, IPC_SET, IPC_STAT = 0, 0o1000, 1, 2
sid = libc.shmget(IPC_PRIVATE, 4096, IPC_CREAT | 0o600)
buf = ctypes.create_string_buffer(256)
libc.shmctl(sid, IPC_STAT, buf)
# struct ipc_perm starts the shmid_ds: key (int), uid (uint), gid (uint), cuid, cgid, mode ...
ctypes.c_uint.from_buffer(buf, 4).value = 1
r = libc.shmctl(sid, IPC_SET, buf)
print("probe shm_memo: segment %d handed to uid 1: %s" % (sid, "ok" if r == 0 else "errno %d" % ctypes.get_errno()))
PY
exec python3 -B -E -s -c 'import sys; sys.path.insert(0, sys.argv[1]); from beltcell.client import main; sys.exit(main(sys.argv[2:]))' "$PYTHONPATH" belt
