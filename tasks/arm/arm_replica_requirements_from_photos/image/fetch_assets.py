"""Fetch this task's large agent-visible files from the benchmark's Hugging Face asset repository at image build time.

    python3 fetch_assets.py <assets.sha256> <dest_dir>

`assets.sha256` (in image/, so in the build context) holds a header with the repository and the pinned commit, then
one `<sha256>  <relative path>` line per file, as `sha256sum` writes them:

    # repo: grounded-intelligence-initiative/agentic-robotics-last-exam
    # revision: <40-hex commit of the dataset repository>
    # prefix: tasks/arm/arm_replica_requirements_from_photos/payload
    <sha256>  photos/PH-01.jpg

Every file is downloaded from `<prefix>/<relative path>` at that commit and written to `<dest_dir>/<relative path>`
only if its sha256 matches; any mismatch or failed download fails the build. Stdlib only (runs before any venv).
"""
import hashlib
import os
import sys
import time
import urllib.request

BASE = "https://huggingface.co/datasets/{repo}/resolve/{rev}/{path}"


def read_list(path):
    head, files = {}, []
    with open(path) as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line.strip():
                continue
            if line.startswith("#"):
                k, _, v = line[1:].partition(":")
                head[k.strip()] = v.strip()
                continue
            digest, rel = line.split(None, 1)
            files.append((digest.lower(), rel.strip().lstrip("*")))
    for k in ("repo", "revision", "prefix"):
        if not head.get(k):
            sys.exit("assets list %s has no '# %s:' header" % (path, k))
    return head, files


def fetch(url, tries=4):
    for k in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=120) as r:
                return r.read()
        except Exception as exc:  # noqa: BLE001  (retried, then fatal)
            if k == tries - 1:
                sys.exit("download failed: %s (%s)" % (url, exc))
            time.sleep(5 * (k + 1))


def main():
    lst, dest = sys.argv[1], sys.argv[2]
    head, files = read_list(lst)
    for digest, rel in files:
        if os.path.isabs(rel) or ".." in rel.split("/"):
            sys.exit("unsafe path in assets list: %s" % rel)
        data = fetch(BASE.format(repo=head["repo"], rev=head["revision"], path=head["prefix"] + "/" + rel))
        got = hashlib.sha256(data).hexdigest()
        if got != digest:
            sys.exit("sha256 mismatch for %s: %s != %s" % (rel, got, digest))
        out = os.path.join(dest, rel)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "wb") as fh:
            fh.write(data)
    print("[assets] %d file(s) from %s@%s verified into %s" % (len(files), head["repo"], head["revision"][:12], dest))


if __name__ == "__main__":
    main()
