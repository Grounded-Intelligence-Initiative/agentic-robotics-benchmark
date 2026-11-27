"""The two file helpers of the verify-start lockdown's snapshot (lockdown.snapshot_dir), stdlib only: the benchmark's
robotic-arm graders keep them in their assembly cell's runner module; this grader has no cell, so they live here,
copied function for function."""

import os
import shutil
import stat

__all__ = ["copy_regular_files", "chmod_tree"]


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
