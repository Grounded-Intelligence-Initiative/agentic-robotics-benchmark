"""How the agent's program is executed during verify: the run regime (account, per-run directory, minimal environment,
limits, the launcher) and the clean-up after each run.

The engine uploads the verify stage folder world-readable (files 0644, directories 0755, FRAMEWORK identity) and
this task executes agent code DURING verify, so the grader itself must

  * chmod 0700 the stage root and every hidden file for the duration of the runs (`lock_stage`, restored in
    `finally`; stat `stage_locked`),
  * run the program as a dedicated account (`nobody` via `runuser`, fallback the agent account;
    `runner_account`) from a fresh per-run directory that holds a root-owned READ-ONLY copy of the submission
    (regular files only, symbolic links and __pycache__ dropped), a root-owned copy of the cell's client code
    (`lib/`) and an empty writable HOME / TMPDIR (`prepare_run_dir`),
  * give it a minimal environment (`child_env`: PATH, HOME, TMPDIR, LANG, MUJOCO_GL, PYTHONPATH=lib,
    ARB_GIVEN_DIR = the root-owned copy of the givens, BLAS threads pinned; nothing of the framework's
    ALE_* variables or secrets),
  * start the interpreter with `-B -E -s` (`launcher_argv`: no bytecode, PYTHON* variables ignored, no user
    site) through a bootstrap that puts `lib/` on sys.path, optionally under `prlimit --cpu`,
  * kill every process of the runner account after each run (`kill_uid_processes`; stats
    `runner_processes_killed`, `stray_processes`) and sweep what it wrote under the watched roots
    (`snapshot_tree`, `tree_changes`, `sweep_new_entries`).

No RLIMIT_NPROC (per uid across every container sharing the uid; kills BLAS thread starts, measured
2026-09-26 by the requirements lane). stdlib only.
"""

import os
import pwd
import shutil
import subprocess
import signal
import stat
import time

__all__ = ["PYTHON_FLAGS", "BOOTSTRAP", "launcher_argv", "wrap_limits", "child_env", "runner_account",
           "prepare_run_dir", "copy_regular_files", "chmod_tree", "lock_stage", "unlock_stage",
           "uid_processes", "kill_uid_processes", "snapshot_tree", "tree_changes", "sweep_new_entries"]

PYTHON_FLAGS = ["-B", "-E", "-s"]
# the default launcher: put the (root-owned) client library first on sys.path, then run the client's main,
# which imports `<module>` from the current directory (the read-only copy of the submission).
BOOTSTRAP = ("import sys; sys.path.insert(0, sys.argv[1]); from beltcell.client import main; "
             "sys.exit(main(sys.argv[2:]))")


def launcher_argv(python, lib_dir, program_dir, module="belt", allow_run_sh=True):
    """argv of the program: `bash run.sh` when the submission ships one (regular file), else the client launcher
    `python -B -E -s -c <bootstrap> <lib_dir> <module>`. `allow_run_sh=False` always uses the launcher."""
    run_sh = os.path.join(program_dir, "run.sh")
    if allow_run_sh and os.path.isfile(run_sh) and not os.path.islink(run_sh):
        return ["bash", "run.sh"]
    return [python] + PYTHON_FLAGS + ["-c", BOOTSTRAP, lib_dir, module]


FSIZE = 256 * 1024 * 1024          # bytes: the largest file the program may write (its stderr included)


def wrap_limits(argv, cpu_s=None, user=None, nonet=False):
    """`prlimit --cpu=<s> --core=0` (when available and cpu_s is given) and `runuser -p -u <user> --` (when a user
    is given and the caller is root); nonet=True runs argv under the no-network seccomp filter (cellkit.nonet), inside
    the account switch and the limits."""
    wrapped = list(argv)
    if nonet:
        from .nonet import nonet_argv
        wrapped = nonet_argv(wrapped)
    if cpu_s is not None and shutil.which("prlimit"):
        # CPU seconds, no core files, and every file the program writes (its stderr included) at most FSIZE bytes
        wrapped = ["prlimit", "--cpu={}".format(int(cpu_s)), "--core=0", "--fsize={}".format(FSIZE), "--"] + wrapped
    if user and os.getuid() == 0 and shutil.which("runuser"):
        # -p keeps the (minimal) environment the caller built: without it runuser resets HOME to the account's
        # home (/nonexistent for nobody) and the promised writable per-run HOME is lost
        wrapped = ["runuser", "-p", "-u", user, "--"] + wrapped
    return wrapped


def _ipc_ids(kind, uid):
    """The ids of the System V IPC objects of one kind ("shm", "sem", "msg") owned by `uid` (from /proc/sysvipc)."""
    try:
        lines = open("/proc/sysvipc/" + kind).read().splitlines()
    except OSError:
        return []
    if not lines:
        return []
    head = lines[0].split()
    try:
        i_id, i_uid = head.index({"shm": "shmid", "sem": "semid", "msg": "msqid"}[kind]), head.index("uid")
    except ValueError:
        return []
    i_cuid = head.index("cuid") if "cuid" in head else i_uid
    out = []
    for ln in lines[1:]:
        f = ln.split()
        # owner OR creator: a run can hand a segment's ownership to another uid (IPC_SET); its creator can still
        # remove it, and a uid-only match would miss it
        if len(f) > max(i_id, i_uid, i_cuid) and (int(f[i_uid]) == int(uid) or int(f[i_cuid]) == int(uid)):
            out.append(f[i_id])
    return out


def sweep_ipc(uid):
    """Remove every SysV shared-memory segment, semaphore set and message queue owned by `uid`, and its POSIX message
    queues, so nothing a run leaves in kernel IPC reaches the next run. Returns the number removed (only what is
    actually gone); raises when any is left. ipcrm runs as the owning account: removing another account's IPC object
    needs CAP_SYS_ADMIN, which a container's root does not hold (a root-run ipcrm failed silently and a segment made by
    run.sh outlived its episode: humanoid_aisle_order_picking's amendment 9, the same kit)."""
    try:
        name = pwd.getpwuid(int(uid)).pw_name
    except (KeyError, ValueError):
        name = None
    n = 0
    for kind, flag in (("shm", "-m"), ("sem", "-s"), ("msg", "-q")):
        ids = _ipc_ids(kind, uid)
        if not ids:
            continue
        if not shutil.which("ipcrm"):
            raise RuntimeError("ipcrm is missing: %d %s object(s) of uid %d cannot be removed" % (len(ids), kind, uid))
        for i in ids:
            cmd = ["ipcrm", flag, i]
            if name and os.getuid() == 0 and os.getuid() != int(uid) and shutil.which("runuser"):
                cmd = ["runuser", "-u", name, "--"] + cmd
            subprocess.run(cmd, capture_output=True, timeout=30)
        left = _ipc_ids(kind, uid)
        n += len(ids) - len(left)
        if left:
            raise RuntimeError("%d %s object(s) of uid %d could not be removed" % (len(left), kind, uid))
    mq = "/dev/mqueue"
    if os.path.isdir(mq):
        left = 0
        for e in os.listdir(mq):
            p = os.path.join(mq, e)
            try:
                if os.lstat(p).st_uid != int(uid):
                    continue
            except OSError:
                continue
            try:
                os.unlink(p)
            except OSError:
                if name and os.getuid() == 0 and shutil.which("runuser"):
                    subprocess.run(["runuser", "-u", name, "--", "rm", "-f", p], capture_output=True, timeout=30)
            if os.path.lexists(p):
                left += 1
            else:
                n += 1
        if left:
            raise RuntimeError("%d POSIX message queue(s) of uid %d could not be removed" % (left, uid))
    return n


def child_env(home, tmpdir, lib_dir, given_dir, threads=2, mujoco_gl=None, extra=None):
    """A minimal environment: fresh HOME / TMPDIR, PYTHONPATH = the client library, ARB_GIVEN_DIR = the read-only
    copy of the givens, BLAS threads pinned; nothing of the caller's environment except PATH."""
    env = {"PATH": os.environ.get("PATH", "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin"),
           "HOME": home, "TMPDIR": tmpdir, "TMP": tmpdir, "TEMP": tmpdir, "LANG": "C.UTF-8", "MPLBACKEND": "Agg",
           "PYTHONPATH": lib_dir, "ARB_GIVEN_DIR": given_dir,
           "MUJOCO_GL": mujoco_gl or os.environ.get("MUJOCO_GL", "osmesa"), "PYOPENGL_PLATFORM": "osmesa",
           "OMP_NUM_THREADS": str(int(threads)), "OPENBLAS_NUM_THREADS": str(int(threads)), "MKL_NUM_THREADS": str(int(threads))}
    if extra:
        env.update(extra)
    return env


def runner_account(preferred="nobody", fallback=None):
    """(name, uid, gid) of the account the program runs as: `preferred` when it exists and the caller is root,
    else `fallback` (the agent account); None when not root (authoring runs: run as self)."""
    if os.getuid() != 0:
        return None
    for name in [n for n in (preferred, fallback) if n]:
        try:
            pw = pwd.getpwnam(name)
            return name, pw.pw_uid, pw.pw_gid
        except KeyError:
            continue
    return None


def copy_regular_files(src, dst):
    """Copy the tree `src` to `dst` keeping regular files and directories only (symbolic links, special files
    and __pycache__ skipped). Returns the number of skipped entries."""
    skipped = 0
    os.makedirs(dst, exist_ok=True)
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = dst if rel == "." else os.path.join(dst, rel)
        keep = []
        for d in dirnames:
            p = os.path.join(dirpath, d)
            if d == "__pycache__" or os.path.islink(p):
                skipped += 1 if os.path.islink(p) else 0
                continue
            os.makedirs(os.path.join(target, d), exist_ok=True)
            keep.append(d)
        dirnames[:] = keep
        for f in filenames:
            p = os.path.join(dirpath, f)
            if os.path.islink(p) or not stat.S_ISREG(os.lstat(p).st_mode):
                skipped += 1
                continue
            shutil.copyfile(p, os.path.join(target, f))
    return skipped


def chmod_tree(root, dir_mode, file_mode):
    os.chmod(root, dir_mode)
    for dirpath, dirnames, filenames in os.walk(root):
        for d in dirnames:
            os.chmod(os.path.join(dirpath, d), dir_mode)
        for f in filenames:
            os.chmod(os.path.join(dirpath, f), file_mode)


def prepare_run_dir(root, program_dir, lib_dirs, owner=None, private=False):
    """Lay out one run under `root` (created 0755): root/program = read-only copy of `program_dir` (world-readable,
    or owner-only when `private`: the grader's own anchor), root/lib/<name> = read-only copies of the client
    library packages (`lib_dirs`: {name: path}), root/home and root/tmp empty and owned by `owner` = (uid, gid).
    Returns {root, program, lib, home, tmp, skipped}."""
    os.makedirs(root, exist_ok=True)
    os.chmod(root, 0o755)
    program = os.path.join(root, "program")
    skipped = copy_regular_files(program_dir, program)
    chmod_tree(program, 0o700 if private else 0o755, 0o600 if private else 0o644)
    lib = os.path.join(root, "lib")
    os.makedirs(lib, exist_ok=True)
    for name, path in lib_dirs.items():
        copy_regular_files(path, os.path.join(lib, name))
    chmod_tree(lib, 0o755, 0o644)
    out = {"root": root, "program": program, "lib": lib, "skipped": skipped}
    for name in ("home", "tmp"):
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        os.chmod(d, 0o700)
        if owner is not None:
            os.chown(d, owner[0], owner[1])
        out[name] = d
    return out


def lock_stage(root):
    """chmod 0700 every directory and 0600 every file under `root` (root only). Returns the modes to restore,
    or None when not root."""
    if os.getuid() != 0:
        return None
    saved = []
    for dirpath, dirnames, filenames in os.walk(root):
        for name in [""] + filenames:
            p = os.path.join(dirpath, name) if name else dirpath
            try:
                mode = stat.S_IMODE(os.lstat(p).st_mode)
            except OSError:
                continue
            saved.append((p, mode))
            try:
                os.chmod(p, 0o700 if not name else 0o600)
            except OSError:
                pass
    return saved


def unlock_stage(saved):
    for p, mode in saved or []:
        try:
            os.chmod(p, mode)
        except OSError:
            pass


def uid_processes(uid):
    """PIDs of every live process owned by `uid` (Linux /proc)."""
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as fh:
                for line in fh:
                    if line.startswith("Uid:"):
                        if int(line.split()[1]) == int(uid):
                            pids.append(int(name))
                        break
        except (OSError, ValueError):
            continue
    return pids


def kill_uid_processes(uid, grace_s=5.0):
    """SIGKILL every process of `uid` until none is left (or grace_s elapses). Returns (killed, remaining)."""
    killed = 0
    seen = set()
    deadline = time.time() + grace_s
    while True:
        pids = uid_processes(uid)
        for pid in pids:
            if pid not in seen:
                seen.add(pid)
                killed += 1
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        if not pids or time.time() > deadline:
            return killed, len(pids)
        time.sleep(0.1)


def snapshot_tree(root, exclude=()):
    """{relpath: (size, mtime_ns, uid)} of every entry under `root` (paths under `exclude` skipped)."""
    snap = {}
    if not os.path.isdir(root):
        return snap
    ex = tuple(os.path.abspath(e) for e in exclude)
    for dirpath, dirnames, filenames in os.walk(root):
        if any(os.path.abspath(dirpath).startswith(e) for e in ex):
            dirnames[:] = []
            continue
        for name in filenames + dirnames:
            p = os.path.join(dirpath, name)
            try:
                st = os.lstat(p)
            except OSError:
                continue
            snap[os.path.relpath(p, root)] = (st.st_size, st.st_mtime_ns, st.st_uid)
    return snap


def tree_changes(before, after, uid=None):
    """(new, modified) relpath lists; with `uid` only entries owned by that uid count."""
    new = sorted(k for k in after if k not in before and (uid is None or after[k][2] == uid))
    modified = sorted(k for k in after if k in before and before[k][:2] != after[k][:2] and (uid is None or after[k][2] == uid))
    return new, modified


def sweep_new_entries(root, before, after, uid, exclude=()):
    """Remove every entry under `root` that is new since `before` and owned by `uid` (any owner when None)."""
    removed = 0
    ex = tuple(os.path.abspath(e) for e in exclude)
    for rel in sorted((k for k in after if k not in before), key=len, reverse=True):
        p = os.path.join(root, rel)
        if any(os.path.abspath(p).startswith(e) for e in ex):
            continue
        try:
            st = os.lstat(p)
        except OSError:
            continue
        if uid is not None and st.st_uid != uid:
            continue
        try:
            if stat.S_ISDIR(st.st_mode) and not os.path.islink(p):
                shutil.rmtree(p, ignore_errors=True)
            else:
                os.remove(p)
            removed += 1
        except OSError:
            pass
    return removed
