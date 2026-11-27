"""Generate a reproducible episode with one observed work state per site."""

import argparse
import hashlib
import os
import random
from pathlib import Path

from generate import case_truth, hard_survey, site_map, write_case, write_json
from island import island_site
from detour import detour_site


def build_island_episode(episode_id):
    """Build private geometry before optionally rendering its public observations."""
    digest = hashlib.sha256(f"workzone_lane_map_update/island-v2/{episode_id}".encode()).digest()
    rng = random.Random(int.from_bytes(digest[:16], "big"))
    geometry_rng = random.Random(int.from_bytes(digest[16:], "big"))
    variants = ["lane_shift", "changed_connection", "shoulder_work"] * 5
    rng.shuffle(variants)
    open_arms = ["left","right"]*5
    rng.shuffle(open_arms)
    episode = {}
    for variant in variants:
        prior, layout, _, _, _, side = site_map(rng, expert=True)
        arm = ("left" if side == 1 else "right") if variant == "shoulder_work" else open_arms.pop()
        paths, geometry, painted, cones = island_site(prior, layout, variant, side, arm, geometry_rng)
        survey = hard_survey(prior, layout, side, geometry_rng)
        truth, patch, _ = case_truth(prior, layout, paths, variant)
        truth["island"] = geometry
        signals = [(prior["nodes"][name][0], .95 * layout["length"], name == geometry["exit"]) for name in paths]
        case_id = f"case_{rng.getrandbits(64):016x}"
        episode[case_id] = {
            "prior": prior,
            "truth": truth,
            "reference": patch,
            "render": {
                "layout": layout, "paths": (), "cones": cones,
                "side": side, "camera_jitter": rng.uniform(-.15, .15), "signals": signals,
                "survey": survey,
                "painted": painted, "fixtures": geometry["fixtures"],
            },
        }
    return episode


def build_episode(episode_id):
    digest = hashlib.sha256(f"workzone_lane_map_update/channelized-diversion-v1/{episode_id}".encode()).digest()
    rng = random.Random(int.from_bytes(digest, "big"))
    # Each episode includes routes on both sides of the island and work on either bank.
    # The island belongs to the pre-construction road. Only its traffic control changes.
    choices=[True,False]*3
    rng.shuffle(choices)
    variants=[("shoulder_work",None,None)]*5
    for variant in ("bridge_diversion","partial_access"):
        variants.extend((variant,False,choices.pop()) for _ in range(3))
        variants.extend((variant,True,None) for _ in range(2))
    rng.shuffle(variants)
    episode = {}
    for variant,on_closed_road,outer in variants:
        case_id = f"case_{rng.getrandbits(64):016x}"
        episode[case_id] = detour_site(rng,variant,island_on_closed_road=on_closed_road,required_outer=outer)
    return episode


def _write_site(item):
    directory,case = item
    write_case(directory,case["prior"],**case["render"])


def write_packet(output, episode):
    output.mkdir(parents=True, exist_ok=True)
    sites = [(output/case_id,case) for case_id,case in episode.items()]
    if os.name == "posix" and any(c["render"]["layout"].get("roadview") for _,c in sites):
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(3,len(sites))) as pool:
            list(pool.map(_write_site,sites))
    else:
        for site in sites:
            _write_site(site)
    write_json(output / "manifest.json", {"cases": list(episode)})


def write_reference(output, episode):
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json(output, {"patches": {case_id: case["reference"] for case_id, case in episode.items()}})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--packet", type=Path)
    mode.add_argument("--oracle", type=Path)
    args = parser.parse_args()
    episode = build_episode(os.environ["ALE_EPISODE_ID"])
    if args.packet:
        write_packet(args.packet, episode)
    else:
        write_reference(args.oracle, episode)
