"""tests/setup_payload = a byte copy of environment/setup/payload, and tests/given_hashes.json = the sha256 of every
file in it (maintainers' tool; run after any change to the payload)."""
import hashlib
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
PKG = os.path.dirname(TESTS)
SRC = os.path.join(PKG, "environment", "setup", "payload")
DST = os.path.join(TESTS, "setup_payload")
shutil.rmtree(DST, ignore_errors=True)
shutil.copytree(SRC, DST, ignore=shutil.ignore_patterns("__pycache__", ".DS_Store"))
out = {}
for dp, dns, fns in os.walk(DST):
    dns[:] = sorted(dns)
    for fn in sorted(fns):
        p = os.path.join(dp, fn)
        out[os.path.relpath(p, DST)] = hashlib.sha256(open(p, "rb").read()).hexdigest()
json.dump(out, open(os.path.join(TESTS, "given_hashes.json"), "w"), indent=1, sort_keys=True)
print(len(out), "files")
