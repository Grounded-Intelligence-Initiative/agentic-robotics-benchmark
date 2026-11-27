"""Writes the hidden set (tests/hidden/: seeds.json, towers/*.json, selection.json, selection_rule.txt) and the
practice towers of the commissioning service (environment/commission_service/sites.json).

Rule: each set has its own 64-bit OS salt (drawn once, recorded); seed i of a set = splitmix64(salt + i). The hidden set
is four towers, one per variant (standard; mid_tight: levels 5-12 all-bearing with +35 % probability; low_friction:
block friction 0.30-0.36; tight_grippy: all-bearing levels with probability 0.25 and friction 0.44-0.50), all inside
the public ranges. The practice set is six standard towers. No practice seed equals a hidden seed.
Usage: python3 tests/review/make_sets.py [--hidden-salt HEX] [--practice-salt HEX]
"""
import argparse
import hashlib
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
MASK = (1 << 64) - 1
VARIANTS = [("standard", {}), ("mid_tight", {"mid_tight": 0.35}), ("low_friction", {"mu": [0.30, 0.36]}),
            ("tight_grippy", {"p_all": 0.25, "mu": [0.44, 0.50]})]


def splitmix64(x):
    x = (x + 0x9E3779B97F4A7C15) & MASK
    z = x
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
    return (z ^ (z >> 31)) & MASK


def dump(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=1)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hidden-salt", default=None)
    ap.add_argument("--practice-salt", default=None)
    a = ap.parse_args()
    hs = int(a.hidden_salt, 16) if a.hidden_salt else int.from_bytes(os.urandom(8), "big")
    ps = int(a.practice_salt, 16) if a.practice_salt else int.from_bytes(os.urandom(8), "big")
    hidden = [splitmix64(hs + i) for i in range(len(VARIANTS))]
    practice = [splitmix64(ps + i) for i in range(6)]
    assert not set(hidden) & set(practice)
    hd = os.path.join(TESTS, "hidden")
    os.makedirs(os.path.join(hd, "towers"), exist_ok=True)
    pins, chosen = [], {}
    for (name, ov), seed in zip(VARIANTS, hidden):
        rel = "towers/%s.json" % name
        dump(os.path.join(hd, rel), {"seed": seed, "overrides": ov, "duration_s": 2400.0})
        h = hashlib.sha256(open(os.path.join(hd, rel), "rb").read()).hexdigest()
        pins.append({"file": rel, "sha256": h})
        chosen[name] = {"seed": seed, "overrides": ov, "sha256": h}
    dump(os.path.join(hd, "seeds.json"), {"note": "the fixed hidden set (tests/review/make_sets.py)", "towers": pins})
    dump(os.path.join(hd, "selection.json"), {"salt_hex": "%016x" % hs, "rule": "selection_rule.txt", "chosen": chosen})
    with open(os.path.join(hd, "selection_rule.txt"), "w") as f:
        f.write(__doc__.split("Usage:")[0].strip() + "\n")
    sites = {"rule": "practice towers: tests/review/make_sets.py", "salt_hex": "%016x" % ps, "shifts": 10, "open": 2,
             "budgets": {"silence_s": 20.0, "start_s": 120.0, "frame_price_s": 0.2, "call_price_s": 0.005,
                         "episode_wall_cap_s": 2700.0},
             "sites": [{"site": i + 1, "seed": s, "duration_s": 2400.0} for i, s in enumerate(practice)]}
    dump(os.path.join(TASK, "environment", "commission_service", "sites.json"), sites)
    print("hidden salt %016x practice salt %016x" % (hs, ps))


if __name__ == "__main__":
    main()
