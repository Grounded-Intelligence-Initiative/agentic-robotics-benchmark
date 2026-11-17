"""V3 entrypoint; see v3_rollout.py --modes full,blind,scout_route,xy,circumscribed.

The old v2 map_ablation.json is historical, not a v3 result.
"""
from v3_rollout import main

if __name__ == '__main__':
    raise SystemExit(main())
