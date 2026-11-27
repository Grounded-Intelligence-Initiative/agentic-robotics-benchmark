"""Check that a response file has the right shape for its request. It knows nothing about
how good the commands are.

  python3 ~/tools/check_response.py <request.json> <response.json>
"""

import json
import math
import sys


def main():
    req = json.load(open(sys.argv[1]))
    resp = json.load(open(sys.argv[2]))
    answers = {e.get("id"): e.get("motor_cmd") for e in resp.get("episodes", [])
               if isinstance(e, dict)}
    problems = 0
    for ep in req["episodes"]:
        n = len(ep["desired"])
        cmd = answers.get(ep["id"])
        if cmd is None:
            print("{}: missing".format(ep["id"]))
            problems += 1
            continue
        if len(cmd) != n or any(len(row) != 3 for row in cmd):
            print("{}: expected {} rows of 3 numbers".format(ep["id"], n))
            problems += 1
            continue
        flat = [v for row in cmd for v in row]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in flat):
            print("{}: non-finite or non-numeric values".format(ep["id"]))
            problems += 1
            continue
        over = sum(1 for v in flat if abs(v) > 8.0)
        if over:
            print("{}: {} values beyond +-8 rad (the drive clips them)".format(ep["id"], over))
    print("ok" if not problems else "{} episode(s) unusable".format(problems))


if __name__ == "__main__":
    main()
