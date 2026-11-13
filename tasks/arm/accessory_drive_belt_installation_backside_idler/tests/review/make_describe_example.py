"""describe_example.json for the payload (maintainers' tool; never run by the grader): one episode's full describe()
answer through the cell's own request handler (the JSON the wire carries).

    python3 make_describe_example.py <panda dir> <out json> [seed]
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from beltcell.server import BeltCellServer  # noqa: E402


def main():
    panda, out = sys.argv[1], sys.argv[2]
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 20261109
    cfg = json.load(open(os.path.join(os.path.dirname(HERE), "grader_config.json")))
    srv = BeltCellServer(os.path.abspath(panda), config=cfg, render=True, log=lambda s: None)
    srv.reset(seed)
    reply = srv.handle("describe", {})
    if isinstance(reply, dict) and set(reply) == {"error"}:
        sys.exit("describe failed: %s" % reply["error"])
    reply = json.loads(json.dumps(reply))          # exactly what the wire carries
    json.dump(reply, open(out, "w"), indent=1, sort_keys=False)
    print("wrote", out, len(json.dumps(reply)), "bytes")


if __name__ == "__main__":
    main()
