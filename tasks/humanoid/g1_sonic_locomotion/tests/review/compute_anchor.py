"""Recompute the registered locomotion anchor from four independent official oracle receipts."""
import argparse
import hashlib
import json
from pathlib import Path


def compute(paths):
    if len(paths)!=4 or len(set(p.resolve() for p in paths))!=4:
        raise ValueError("provide four distinct verification.json files in N0, N1, N2, N3 order")
    panels=[]
    for i,path in enumerate(paths):
        raw=path.read_bytes()
        v=json.loads(raw)
        m=v["metrics"]
        total=m["successes"]
        counts=[m[k+"_successes"] for k in ("straight_stop","right_turn_stop")]
        if (v["reward"]!=0 or m["anchor_recorded"]!=0 or m["episodes"]!=16
                or m["straight_stop_episodes"]!=8 or m["right_turn_stop_episodes"]!=8
                or any(type(n) not in (float,int) or int(n)!=n or n<0 or n>8 for n in counts)
                or total!=sum(counts) or m["strict_stop_fraction"]!=total/16):
            raise ValueError(f"N{i} must be a valid balanced null-anchor measurement")
        panels.append(dict(id=f"N{i}",episode_id=path.parent.name,verification_sha256=hashlib.sha256(raw).hexdigest(),
                           strict_stops=int(total),straight=int(counts[0]),right=int(counts[1]),metrics=m))
    n0=panels[0]["strict_stops"]
    if not panels[0]["straight"] or not panels[0]["right"]:
        raise ValueError("the registered N0 reference must demonstrate both course families")
    totals=[p["strict_stops"] for p in panels]
    needed=max(12,min(totals)-1)
    if max(totals)<needed or needed/n0>1.5:
        raise ValueError("reference did not demonstrate the registered full-credit floor or a feasible capped ratio")
    return dict(value=n0/16,full_at=needed/n0,strict_stops_for_full_credit=needed,episodes_per_panel=16,panels=panels)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipts",nargs=4,type=Path,metavar="VERIFICATION")
    print(json.dumps(compute(parser.parse_args().receipts),indent=2,allow_nan=False))
