"""Fetch the cell's hidden kit for the graded episode's service (dual mode, experimental): run by
environment/setup/setup.sh as root before the agent phase (the engine opens the sandbox's network for setup and seals
it before the agent starts).

    fetch_kit.py <the cell configuration (the byte copy of tests/grader_config.json)> <destination: the kit folder>
                 <the lab account>

The kit (gt/: kit.json + meshes/, the units' visual meshes and collision pieces) is the one the offline verifier
fetches at verify start (tests/verify.py fetch_hidden_gt, the same logic): one tarball named in the configuration's
"hidden_gt" block (its URL at an asset-dataset commit, its sha256, its file count), downloaded with its retries; its
sha256 must be the pinned one, every member must be under gt/ and be a file or a folder, and the unpacked folder must
hold the pinned number of files and kit.json, else setup fails (nothing runs without the exact kit). It is unpacked
into <destination> owned by root, group <the lab account>, folders 0750 and files 0640: the service's worker reads it
as that account, the agent's account (not in the group) cannot. ARB_HIDDEN_GT_TARBALL (a maintainer's local copy of
the same tarball) replaces the download. Standard library only.
"""

import grp
import hashlib
import json
import os
import shutil
import sys
import tarfile
import time
import urllib.request


def log(msg):
    print("[fetch_kit] " + msg, file=sys.stderr, flush=True)


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main(argv):
    cfg_path, dest, group = argv[1], argv[2], argv[3]
    with open(cfg_path) as fh:
        spec = json.load(fh)["hidden_gt"]
    url, want = spec["url"], spec["sha256"]
    tries, backoff = max(1, int(spec.get("tries", 4))), float(spec.get("backoff_s", 5.0))
    gid = grp.getgrnam(group).gr_gid
    parent = os.path.dirname(os.path.abspath(dest))
    work = os.path.join(parent, ".kit-fetch")
    shutil.rmtree(work, ignore_errors=True)
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(work, mode=0o700)
    t0 = time.time()
    src = os.environ.get("ARB_HIDDEN_GT_TARBALL")
    source = "maintainer copy"
    if not src:
        src, source = os.path.join(work, "gt.tar.xz"), "download"
        for k in range(tries):
            try:
                with urllib.request.urlopen(url, timeout=120) as r, open(src, "wb") as fh:
                    for block in iter(lambda: r.read(1 << 20), b""):
                        fh.write(block)
                break
            except Exception as e:  # noqa: BLE001 - retried, then a setup error
                log("download %d/%d failed: %r" % (k + 1, tries, e))
                if os.path.exists(src):
                    os.remove(src)
                if k == tries - 1:
                    log("could not download the cell's kit")
                    return 1
                time.sleep(backoff * (k + 1))
    got = sha256_file(src)
    if got != want:
        log("the cell's kit (%s) has sha256 %s; the configuration pins %s" % (source, got, want))
        return 1
    with tarfile.open(src, "r:xz") as tf:
        bad = [m.name for m in tf.getmembers()
               if not (m.isfile() or m.isdir()) or not (m.name == "gt" or m.name.startswith("gt/"))]
        if bad:
            log("the cell's kit tarball holds unexpected members: %s" % bad[:5])
            return 1
        tf.extractall(work, filter="data")
    os.rename(os.path.join(work, "gt"), dest)
    shutil.rmtree(work, ignore_errors=True)
    n = 0
    for dp, dns, fns in os.walk(dest):
        os.chown(dp, 0, gid)
        os.chmod(dp, 0o750)
        for fn in fns:
            p = os.path.join(dp, fn)
            os.chown(p, 0, gid)
            os.chmod(p, 0o640)
            n += 1
    if n != int(spec["files"]) or not os.path.isfile(os.path.join(dest, "kit.json")):
        log("the cell's kit unpacked to %d files (kit.json %s); the configuration expects %d"
            % (n, os.path.isfile(os.path.join(dest, "kit.json")), int(spec["files"])))
        return 1
    log("the cell's kit (%s): %d files, sha256 %s..., in %.1f s" % (source, n, got[:12], time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
