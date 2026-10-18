#!/usr/bin/env python3
"""PLACEHOLDER reference controller: speaks the wire and stands still (scores 0).

The real oracle (depth-based tabletop segmentation, reference-feature classification,
top-down pinch grasps, unpack/repack search, absence by exhaustion) replaces this file
before the anchor is recorded. Keeping a wire-correct stand-in lets `ale validate` exercise
the whole pipeline meanwhile.
"""
import json
import sys


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "reset":
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            print(json.dumps({"action": [0.0] * 8}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        else:
            print(json.dumps({"error": "unknown request"}), flush=True)


if __name__ == "__main__":
    main()
