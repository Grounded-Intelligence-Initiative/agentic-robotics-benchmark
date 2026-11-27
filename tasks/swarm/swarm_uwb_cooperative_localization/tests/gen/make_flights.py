"""Regenerate the agent-visible flights from the hidden world.

Run from the task folder:  python3 tests/gen/make_flights.py
Writes environment/setup/payload/flights/flight_01, flight_02, flight_03: ordinary flights,
without truth (nothing in this warehouse measures where the drones are). Deterministic seeds.
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)

import flightio  # noqa: E402
import world as W  # noqa: E402

OUT = os.path.join(TASK, "environment", "setup", "payload", "flights")
DURATION = 150.0


def main():
    for name, seed, with_truth in (("flight_01", 11, False), ("flight_02", 22, False),
                                   ("flight_03", 33, False)):
        rng = np.random.default_rng(seed)
        truth = W.generate_flight(rng, DURATION)
        sensors = W.simulate_sensors(rng, truth)
        flightio.write_flight(os.path.join(OUT, name), sensors, DURATION,
                              truth=truth if with_truth else None)
        print(name, len(sensors["uwb"]), "ranges")


if __name__ == "__main__":
    main()
