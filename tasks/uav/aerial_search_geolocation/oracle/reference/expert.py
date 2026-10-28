"""Privileged reference expert for aerial_search_geolocation.

Graded with the oracle token (see verify/verify.py), every observation carries
`privileged`: the objects placed in the scene, by description, in the mission start frame.
Each reply reports every asked target from it: found at its true position, or declared
absent when no placed object has that description (a decoy). The drone turns in place for
`TURNS` steps, which costs no path length and keeps the episode long enough for the probe
lock to sample it, then stops.

Without the privileged block (no or a wrong token) it reports nothing and stops. stdlib only.
"""

import json
import sys

STOP, ROTR = 0, 7
FOUND, ABSENT = 1, 2
TURNS = 12


def reply(t, obs):
    truth = obs.get("privileged")
    asked = [target["description"] for target in obs["targets"]]
    if truth is None:
        return [STOP]
    where = {item["description"]: item["position"] for item in truth["objects"]}
    reports = []
    for description in asked:
        if description in where:
            reports.extend([FOUND] + list(where[description]))
        else:
            reports.extend([ABSENT, 0.0, 0.0, 0.0])
    return [ROTR if obs["steps_used"] < TURNS else STOP] + reports


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        message = json.loads(line)
        kind = message.get("type")
        if kind == "act":
            out = {"action": reply(message["t"], message["obs"])}
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return 0
        else:
            out = {"ok": True}
        print(json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
