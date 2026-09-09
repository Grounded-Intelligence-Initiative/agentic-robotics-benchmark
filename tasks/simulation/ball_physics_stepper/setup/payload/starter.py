"""Optional protocol skeleton. Implement the physics and copy into submission/."""
import json
import sys


class Simulator:
    def reset(self, config, state):
        raise NotImplementedError("Store configuration, position, velocity and zero time")

    def step(self, dt, impulse):
        raise NotImplementedError("Apply impulse; advance with gravity and plate contacts")


def main():
    sim = Simulator()
    print("READY", flush=True)
    for line in sys.stdin:
        req = json.loads(line)
        if req["type"] == "close":
            print('{"ok":true}', flush=True)
            return
        state = (sim.reset(req["config"], req["state"]) if req["type"] == "reset"
                 else sim.step(req["dt"], req["impulse"]))
        print(json.dumps({"state": state}, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
