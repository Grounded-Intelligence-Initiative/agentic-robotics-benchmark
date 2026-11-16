"""Oracle: answers each chunk with the reply the grader supplies under the token (`obs["_gt"]`): an alarm on the
chunk that contains the failure's onset, nothing otherwise. Shows only that the wire and the grader work."""
import json
import sys


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "reset":
            print(json.dumps({"ok": True}), flush=True)
        elif msg["type"] == "act":
            print(json.dumps({"action": msg["obs"].get("_gt", {"alarm": False})}), flush=True)
        elif msg["type"] == "close":
            print(json.dumps({"ok": True}), flush=True)
            return


if __name__ == "__main__":
    main()
