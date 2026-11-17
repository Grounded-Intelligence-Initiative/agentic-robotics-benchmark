"""V4 entrypoint; see v4_rollout.py --modes full,blind,scout_route,xy,circumscribed.

The old v2 map_ablation.json is historical, not a v4 result.
"""
from v4_rollout import main

if __name__ == '__main__':
    raise SystemExit(main())
