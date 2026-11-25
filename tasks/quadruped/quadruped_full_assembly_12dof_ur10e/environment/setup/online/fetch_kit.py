#!/usr/bin/env python3
"""The cell's hidden kit for the graded episode's service (run as root by environment/setup/setup.sh, before the agent
phase, while the engine's setup still has network).

The kit (gt/: kit.json with the bodies, slots and carve keys, meshes/ with the units' visual meshes and collision
pieces) is not in the repository and not in the image: the offline grader downloads it at verify start
(tests/verify.py fetch_hidden_gt). The graded episode's cell needs the same kit in the agent's sandbox, so this script
does the same here with the same pin (grader_config.json "hidden_gt", a byte copy of the grader's: the URL at a
dataset commit, the tarball's sha256 and the kit's file count): download (ARB_HIDDEN_GT_TARBALL, a maintainer's local
copy of the same tarball, replaces it), check the sha256, refuse any member that is not a file or a folder under gt/,
unpack, check the file count and kit.json. The kit lands at --dest, owned by root with group --group (the lab account's)
and no access for anyone else: the service's worker reads it as the lab account, the agent cannot. --receipt records
the tarball's sha256 and the file count (root-only; the service's record carries them). Any failure exits non-zero,
and setup
fails with it: nothing is served without the exact kit.
"""

import argparse
import grp
import hashlib
import json
import os
import shutil
import sys
import tarfile
import tempfile
import time
import urllib.request


def log(msg):
    print("[fetch_kit] " + msg, file=sys.stderr, flush=True)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for b in iter(lambda: fh.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def download(url, path, tries, backoff):
    for k in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=120) as r, open(path, "wb") as fh:
                for block in iter(lambda: r.read(1 << 20), b""):
                    fh.write(block)
            return
        except Exception as e:  # noqa: BLE001 - retried, then a failure of setup
            log("download %d/%d failed: %r" % (k + 1, tries, e))
            if os.path.exists(path):
                os.remove(path)
            if k == tries - 1:
                raise SystemExit("could not download the cell's kit from %s: %r" % (url, e))
            time.sleep(backoff * (k + 1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--grader-config", required=True, help="grader_config.json (its hidden_gt block)")
    ap.add_argument("--dest", required=True, help="where the kit goes (a new folder; its parent must be root-owned)")
    ap.add_argument("--receipt", required=True, help="the receipt (JSON) to write")
    ap.add_argument("--group", required=True, help="the group that may read the kit (the lab account's)")
    args = ap.parse_args(argv)
    if os.geteuid() != 0:
        raise SystemExit("fetch_kit: must run as root")
    with open(args.grader_config) as fh:
        spec = json.load(fh)["hidden_gt"]
    url, want, nfiles = spec["url"], spec["sha256"], int(spec["files"])
    tries, backoff = max(1, int(spec.get("tries", 4))), float(spec.get("backoff_s", 5.0))
    gid = grp.getgrnam(args.group).gr_gid
    dest = os.path.abspath(args.dest)
    parent = os.path.dirname(dest)
    st = os.lstat(parent)
    if st.st_uid != 0 or (st.st_mode & 0o022):
        raise SystemExit("fetch_kit: %s must be root-owned and not group/world-writable" % parent)
    t0 = time.time()
    work = tempfile.mkdtemp(prefix=".kit-", dir=parent)       # 0700, root's
    try:
        tarball = os.path.join(work, "gt.tar.xz")
        local = os.environ.get("ARB_HIDDEN_GT_TARBALL")
        if local:
            shutil.copyfile(local, tarball)
            source = "maintainer copy"
        else:
            download(url, tarball, tries, backoff)
            source = "download"
        got = sha256_file(tarball)
        if got != want:
            raise SystemExit("fetch_kit: the kit (%s) has sha256 %s; grader_config.json pins %s" % (source, got, want))
        part = os.path.join(work, "part")
        os.mkdir(part, 0o700)
        with tarfile.open(tarball, "r:xz") as tf:
            bad = [m.name for m in tf.getmembers()
                   if not (m.isfile() or m.isdir()) or not (m.name == "gt" or m.name.startswith("gt/"))]
            if bad:
                raise SystemExit("fetch_kit: the kit's tarball holds unexpected members: %s" % bad[:5])
            tf.extractall(part, filter="data")
        gt = os.path.join(part, "gt")
        n = sum(len(fns) for _dp, _dn, fns in os.walk(gt))
        if n != nfiles or not os.path.isfile(os.path.join(gt, "kit.json")):
            raise SystemExit("fetch_kit: the kit unpacked to %d files (kit.json %s); grader_config.json expects %d"
                             % (n, os.path.isfile(os.path.join(gt, "kit.json")), nfiles))
        for dirpath, dirnames, filenames in os.walk(gt):
            os.chown(dirpath, 0, gid)
            os.chmod(dirpath, 0o750)
            for name in filenames:
                p = os.path.join(dirpath, name)
                os.chown(p, 0, gid, follow_symlinks=False)
                os.chmod(p, 0o640)
        if os.path.lexists(dest):
            shutil.rmtree(dest)
        os.rename(gt, dest)
        receipt = {"sha256": got, "files": n, "source": source, "url": url, "seconds": round(time.time() - t0, 1)}
        tmp = args.receipt + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(receipt, fh, sort_keys=True)
        os.chmod(tmp, 0o600)
        os.replace(tmp, args.receipt)
        log("the cell's kit (%s): %d files, sha256 %s..., in %.1f s" % (source, n, got[:12], receipt["seconds"]))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
