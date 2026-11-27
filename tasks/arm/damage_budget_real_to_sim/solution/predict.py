"""Predict with the submitted MJCF and a copy of the published action driver."""
import json
import sys

from model_rollout import predict


if __name__ == "__main__":
    print(json.dumps({"positions": predict("scene.xml", json.load(sys.stdin)["actions"])}))
