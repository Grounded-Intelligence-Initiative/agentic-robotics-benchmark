"""Outcome-independent geometry summary; never filters/replaces a seed."""
import json
from pathlib import Path
import statistics
import sys

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE/'runtime'))
from mechanics import Site


def summary(seeds):
    sites = [Site(s) for s in seeds]
    fields = dict(service_y=[s.service_y for s in sites],
                  length=[2*s.carrier_half[0] for s in sites],
                  width=[2*s.carrier_half[1] for s in sites],
                  mass=[s.carrier_mass for s in sites],
                  traction=[s.dynamics['traction'] for s in sites])
    return {k: dict(mean=statistics.mean(v), min=min(v), max=max(v)) for k, v in fields.items()}


if __name__ == '__main__':
    hidden = json.loads((STAGE/'hidden/sites.json').read_text())['seeds']
    candidates = json.loads((STAGE/'hidden/candidates.json').read_text())['seeds']
    print(json.dumps(dict(hidden=summary(hidden), original_candidates=summary(candidates),
                          development=summary(range(1000, 1020))), indent=2))
