#!/usr/bin/env python3
"""Draw the fixed hidden set and the development packs (maintainers' tool; nothing runs it at grading).

    python3 select_set.py --write          # draws a new key and writes both sets (only when the sets are to change)
    python3 select_set.py --refresh        # the same key: rewrites the files and the record's summaries (after a change
                                           # of packcell/series.py that leaves the seeds as they are)
    python3 select_set.py --draw-cells     # rule 2, once: draws the cells' key and writes every pack's camera offsets
    python3 select_set.py --draw-sealant   # rule 3, once: draws the sealant key and writes every pack's sealant seed
    python3 select_set.py --draw-appearance   # rule 4, once: the appearance key and every pack's appearance seed
    python3 select_set.py --draw-round2    # rules 5 to 7, once: their three keys and every pack's seeds and counts
    python3 select_set.py --draw-round3    # rules 9 and 11, once: their two keys and every pack's seeds
    python3 select_set.py --draw-round3b   # rule 12, once: its key and every cell's transducer
    python3 select_set.py                  # re-derives both sets from the recorded keys and checks the files

Rule 1, fixed before any pack was drawn or run:

- A key of 32 random bytes (os.urandom) is drawn once and recorded in tests/hidden/selection.json.
- The hidden set is six packs. Their variants and harness cases are fixed by this table, so that every variant
  appears twice and every harness case twice: (P4, lid), (P4, busbars), (P5, none), (P5, lid), (P6, busbars),
  (P6, none). Pack k's seed is the first 8 bytes of BLAKE2b(key, "hidden|k"). Everything else about a pack (its
  placement, head types, replaced, corroded and missing fasteners, the harness' path) follows from its seed
  (packcell/series.py `draw_pack`).
- The development packs of the commissioning service are four packs drawn the same way under "dev|k": (P4, busbars),
  (P5, lid), (P6, none), (P6, lid).
- No pack is drawn again, dropped or reordered for any property it turns out to have, and none for how the
  reference or any other program does on it.

Rule 2, the cells' camera offsets (2026-11-25; written and committed before any offset was drawn). Every pack is
taken apart in a cell of its own, and a cell's two cameras sit off their nominal mounting (packcell/series.py,
`CAMERAS`, `MOUNT_TOL`, `camera_mount`):

- A second key of 32 random bytes (os.urandom) is drawn once (`--draw-cells`) and recorded in
  tests/hidden/selection.json as `cells_key_hex`. It is independent of the first key and of every seed.
- Pack k of the hidden set is taken apart in the cell "hidden|k", development pack k in the cell "dev|k". That
  cell's camera c (overhead, wrist) has the offset component j (0 to 5: dx, dy, dz in mm, then rx, ry, rz in
  degrees, a rotation vector; in the camera's own frame) (2 u - 1) b, rounded to 3 decimals, where u is the first
  8 bytes of BLAKE2b("cell|<set>|<k>|<c>|<j>", key = the second key) read little-endian over 2^64, and b is
  MOUNT_TOL[c]["mm"] for j < 3 and MOUNT_TOL[c]["deg"] otherwise (overhead 15 mm and 1.5 degrees, wrist 4 mm and
  1.0 degree).
- The offsets go into each pack's entry (`cameras`) of tests/hidden/packs.json and
  environment/commission_service/dev_packs.json. None is drawn again or changed for any value it turns out to have
  or for how any program does with it.

Rule 3, sealant partly over heads (2026-11-25; written and committed before any value was drawn, after rule 2's draw
and the measurements on its tree). Some fasteners carry a dab of sealant (packcell/series.py, `SEALANT`):

- A third key of 32 random bytes (os.urandom) is drawn once (`--draw-sealant`) and recorded in
  tests/hidden/selection.json as `sealant_key_hex`. It is independent of the other keys and of every seed.
- Pack k of the hidden set gets the sealant seed = the first 8 bytes of BLAKE2b("sealant|hidden|k", key = the third
  key) read little-endian, development pack k the same with "dev". Which fasteners carry a dab and its place, size
  and height follow from that seed (`draw_pack`'s `sealant_seed`: per fastener, in the pack's order, a chance of
  SEALANT["p"] for its group and four uniform draws within SEALANT's ranges; a missing fastener carries none).
- The seeds go into each pack's entry (`sealant_seed`) of tests/hidden/packs.json and
  environment/commission_service/dev_packs.json. None is drawn again or changed for any value it turns out to have
  or for how any program does with it.

Rule 4, a pack's appearance (2026-11-25; written and committed before any value was drawn, after rule 3's draw and
the measurements on its tree): the hall's light, grime, dust and labels (packcell/series.py, `APPEARANCE`).

- A fourth key of 32 random bytes (os.urandom) is drawn once (`--draw-appearance`) and recorded in
  tests/hidden/selection.json as `appearance_key_hex`, independent of the other keys and of every seed.
- Pack k of the hidden set gets the appearance seed = the first 8 bytes of BLAKE2b("appearance|hidden|k", key = the
  fourth key) read little-endian, development pack k the same with "dev". The light, the patches, the labels and the
  dusty heads follow from that seed (`draw_pack`'s `appearance_seed`, `series._appearance`).
- The seeds go into each pack's entry (`appearance_seed`) of the two files above. None is drawn again or changed for
  any value it turns out to have or for how any program does with it.

Rules 5 to 7, the second hardening round (2026-11-26; written and committed before any of their values was drawn,
after a frontier model scored 0.84 on the tree of rule 4). Each has a key of its own: 32 random bytes (os.urandom),
drawn once by `--draw-round2` and recorded in tests/hidden/selection.json as `seized_key_hex`, `swollen_key_hex` and
`foam_key_hex`, independent of each other, of the earlier keys and of every seed. For pack k of the set <set>
("hidden" or "dev") a rule's seed is the first 8 bytes of BLAKE2b("<name>|<set>|k", key = the rule's key) read
little-endian. Everything else follows from packcell/series.py (`draw_pack`, `SEIZED`, `WORK`, `SWOLLEN`, `FOAM`).
How often a condition appears in a set is part of the rule: the development packs show every condition, but not in
the combinations and numbers the hidden packs carry (the documents state the ranges that cover both).

- Rule 5, seized fasteners (name "seized"). A development pack has 1 seized fastener; hidden pack k has 2 + (the
  first 8 bytes of BLAKE2b("seized-n|hidden|k", key = the fifth key) read little-endian, modulo 3), i.e. 2 to 4.
  They are chosen one after the other from the seed (`series._seize`) among the pack's present fasteners, with
  weights lid 1, busbar 1, bracket 2; a chosen fastener's breakaway is 1.15 to 1.6 times its thread's shear torque
  (uniform); then every fastener of the pack gets its working factor, uniform in 0.55 to 0.80.
- Rule 6, swollen modules (name "swollen"). Development packs 1 and 2 and hidden packs 0, 1, 3 and 4 carry one
  swollen module (`SWOLLEN["dev"]`, `SWOLLEN["hidden"]`), chosen uniformly among the pack's modules from the seed;
  its top bulges 0.6 to 3.0 mm, every other module's top has a crown of 0 to 0.25 mm (uniform), each bump with its
  semi-axes and its centre's offset from the seed (`series._swell`).
- Rule 7, foam pads (name "foam"). The development packs have (1, 0, 1, 0) fasteners under pads, the hidden packs
  (2, 1, 1, 2, 0, 1) (`FOAM`). They are chosen one after the other from the seed (`series._pads`) among the lid's and
  the busbars' fasteners (present or missing) that a harness remnant does not cover, with weights lid 2, busbar 1;
  each pad's size, thickness, angle and centre (within 8 mm of the fastener's nominal place) follow from the seed.
- The seeds and counts go into each pack's entry (`seized_seed`, `seized_n`, `swollen_seed`, `swollen`, `foam_seed`,
  `foam_n`) of the two files above. None is drawn again or changed for any value it turns out to have or for how
  any program does with it.

Rules 8 to 11, the third hardening round (2026-11-26, evening; written and committed before any of their values was
drawn, after a frontier model scored 0.97 on the tree of rules 5 to 7). Rules 8 and 10 are rules of the cell with no
values to draw; rules 9 and 11 each have a key of their own: 32 random bytes (os.urandom), drawn once by
`--draw-round3` and recorded in tests/hidden/selection.json as `shank_key_hex` and `swollen3_key_hex`, independent of
each other, of the earlier keys and of every seed. For pack k of the set <set> a rule's seed is the first 8 bytes of
BLAKE2b("<name>|<set>|k", key = the rule's key) read little-endian. The rest follows from packcell/series.py
(`DAMAGE`, `ENGAGE`, `SHANK`, `TRANSDUCER`, `WORK`, `CONSEQUENCE`, `SWOLLEN3`) and packcell/rules.py.

- Rule 8, precision engagement (no draw). Full engagement within 0.8 mm (socket), 0.5 mm (Torx) and 2 degrees; a bit
  on a head that it does not drive, run with a torque limit of 1 N m or more, strips the head at once (amended before
  any run: the first text said "within 10 degrees of turning", which a slow test turn could still read cheaply); a
  reamed Torx recess takes no extractor.
- Rule 9, the shank's strength (name "shank"). Per fastener, in the pack's order, three uniform draws from the seed
  (`series._shank`): a corroded or seized fastener's shear torque is 0.70 to 1.00 times its thread's (a clean one's is
  its thread's); its yield point 0.86 to 0.92 times its own shear torque; the angle reading's wind-up 1.2 to 1.8 (M6)
  or 0.6 to 0.9 (M8) degrees per N m, six times that past yield. Working loose needs a loosening stall at 0.80 of the
  fastener's own shear torque or more. The torque and angle readings carry the transducer noise of `TRANSDUCER`.
- Rule 10, consequences (no draw). A safety event (live contact, a fastener short, a vented module, a dropped module)
  condemns the pack: its credit is 0. A freed fastener falls off the bit when it is moved sideways before it is drawn
  out of its hole, when the tip accelerates more than 4 m/s^2 or when the tool axis tilts more than 25 degrees; a loose
  fastener at a seated busbar shorts it. The third protective stop in a pack ends the pack.
- Rule 11, the swollen decision at the criterion (name "swollen3"). The packs that carry a swollen module are rule 6's;
  every module's bump is drawn again from the seed (`series._swell` with `SWOLLEN3`): a swollen module's bulge 0.45 to
  0.80 mm, every other module's crown 0.10 to 0.33 mm (uniform), semi-axes and centre as before. The criterion stays
  0.4 mm.
- The seeds go into each pack's entry (`shank_seed`, `swollen3_seed`) of the two files above. None is drawn again or
  changed for any value it turns out to have or for how any program does with it.

Rule 12, a cell's own nutrunner (2026-11-26, evening; written and committed before any value was drawn, after the
draw of rules 9 and 11, when a study of the merged arm tasks showed that what made them hard was a scored unit whose
values differ from the practice units' and must be identified on site). Every cell has its own torque transducer:
it reads gain x torque + offset while the spindle runs, and the spindle stalls where the reading meets the commanded
limit (packcell/series.py `TRANSDUCER_CELL`).

- A key of 32 random bytes (os.urandom) is drawn once (`--draw-round3b`) and recorded in tests/hidden/selection.json
  as `transducer_key_hex`, independent of the other keys and of every seed.
- The cell of pack k of the set <set> gets the gain 0.88 + 0.24 u and the offset -0.30 + 0.60 v (N m), rounded to 4
  decimals, where u and v are the first 8 bytes of BLAKE2b("transducer|<set>|<k>|gain" and "...|offset", key = that
  key) read little-endian over 2^64.
- The values go into each pack's entry (`transducer`) of the two files above. None is drawn again or changed for any
  value it turns out to have or for how any program does with it.
"""

import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from packcell import series as S  # noqa: E402

HIDDEN = [("P4", "lid"), ("P4", "busbars"), ("P5", "none"), ("P5", "lid"), ("P6", "busbars"), ("P6", "none")]
DEV = [("P4", "busbars"), ("P5", "lid"), ("P6", "none"), ("P6", "lid")]
SEL = os.path.join(TASK, "tests", "hidden", "selection.json")
PACKS = os.path.join(TASK, "tests", "hidden", "packs.json")
DEVFILE = os.path.join(TASK, "environment", "commission_service", "dev_packs.json")
SERVICE = {"sessions": 3, "idle_s": 900, "start_timeout_s": 900, "cell": {"sim_cap_s": 2700.0, "frame_budget": 400}}


def draw(key, tag, table):
    out = []
    for k, (variant, harness) in enumerate(table):
        seed = int.from_bytes(hashlib.blake2b(b"%s|%d" % (tag, k), key=key, digest_size=8).digest(), "little")
        out.append({"seed": seed, "variant": variant, "harness": harness})
    return out


def cells(key2, tag, n):
    """Rule 2: the camera offsets of the cells "<tag>|0" .. "<tag>|n-1"."""
    out = []
    for k in range(n):
        cams = {}
        for c in sorted(S.CAMERAS):
            bounds = [S.MOUNT_TOL[c]["mm"]] * 3 + [S.MOUNT_TOL[c]["deg"]] * 3
            vals = []
            for j, b in enumerate(bounds):
                h = hashlib.blake2b(b"cell|%s|%d|%s|%d" % (tag, k, c.encode("ascii"), j), key=key2, digest_size=8).digest()
                u = int.from_bytes(h, "little") / float(2 ** 64)
                vals.append(round((2.0 * u - 1.0) * b, 3))
            cams[c] = vals
        out.append(cams)
    return out


def with_cells(sel, tag, packs):
    if "cells_key_hex" in sel:
        offs = cells(bytes.fromhex(sel["cells_key_hex"]), tag, len(packs))
        packs = [dict(p, cameras=o) for p, o in zip(packs, offs)]
    if "appearance_key_hex" in sel:
        key4 = bytes.fromhex(sel["appearance_key_hex"])
        packs = [dict(p, appearance_seed=int.from_bytes(hashlib.blake2b(b"appearance|%s|%d" % (tag, k), key=key4, digest_size=8).digest(), "little"))
                 for k, p in enumerate(packs)]
    if "appearance_key_hex" in sel:
        key4 = bytes.fromhex(sel["appearance_key_hex"])
        packs = [dict(p, appearance_seed=int.from_bytes(hashlib.blake2b(b"appearance|%s|%d" % (tag, k), key=key4, digest_size=8).digest(), "little"))
                 for k, p in enumerate(packs)]
    if "sealant_key_hex" in sel:
        key3 = bytes.fromhex(sel["sealant_key_hex"])
        packs = [dict(p, sealant_seed=int.from_bytes(hashlib.blake2b(b"sealant|%s|%d" % (tag, k), key=key3, digest_size=8).digest(), "little"))
                 for k, p in enumerate(packs)]
    if "seized_key_hex" in sel:
        packs = round2(sel, tag, packs)
    if "shank_key_hex" in sel:
        k9, k11 = bytes.fromhex(sel["shank_key_hex"]), bytes.fromhex(sel["swollen3_key_hex"])
        packs = [dict(p, shank_seed=_seed(b"shank", tag, k, k9), swollen3_seed=_seed(b"swollen3", tag, k, k11))
                 for k, p in enumerate(packs)]
    if "transducer_key_hex" in sel:
        k12 = bytes.fromhex(sel["transducer_key_hex"])
        u = lambda k, what: int.from_bytes(hashlib.blake2b(b"transducer|%s|%d|%s" % (tag, k, what), key=k12, digest_size=8).digest(),  # noqa: E731
                                           "little") / float(2 ** 64)
        lo_g, hi_g = S.TRANSDUCER_CELL["gain"]
        lo_o, hi_o = S.TRANSDUCER_CELL["offset"]
        packs = [dict(p, transducer=[round(lo_g + (hi_g - lo_g) * u(k, b"gain"), 4), round(lo_o + (hi_o - lo_o) * u(k, b"offset"), 4)])
                 for k, p in enumerate(packs)]
    return packs


def _seed(name, tag, k, key):
    return int.from_bytes(hashlib.blake2b(b"%s|%s|%d" % (name, tag, k), key=key, digest_size=8).digest(), "little")


def round2(sel, tag, packs):
    """Rules 5 to 7: each pack's seeds and counts of the seized fasteners, the swollen module and the foam pads."""
    k5, k6, k7 = (bytes.fromhex(sel[n]) for n in ("seized_key_hex", "swollen_key_hex", "foam_key_hex"))
    hidden = tag == b"hidden"
    out = []
    for k, p in enumerate(packs):
        n5 = 2 + _seed(b"seized-n", tag, k, k5) % 3 if hidden else S.SEIZED["n_dev"]
        out.append(dict(p, seized_seed=_seed(b"seized", tag, k, k5), seized_n=n5,
                        swollen_seed=_seed(b"swollen", tag, k, k6),
                        swollen=bool((S.SWOLLEN["hidden"] if hidden else S.SWOLLEN["dev"])[k]),
                        foam_seed=_seed(b"foam", tag, k, k7),
                        foam_n=(S.FOAM["hidden"] if hidden else S.FOAM["dev"])[k]))
    return out


def main():
    if "--draw-round3b" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "transducer_key_hex" in sel:
            raise SystemExit("the key of rule 12 is drawn already (once)")
        sel["transducer_key_hex"] = os.urandom(32).hex()
        sel["transducer_rule"] = "tests/review/select_set.py (its docstring), rule 12"
        sel["transducer_drawn_utc"] = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--draw-round3" in sys.argv[1:] and "--draw-round3b" not in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "shank_key_hex" in sel:
            raise SystemExit("the keys of rules 9 and 11 are drawn already (once)")
        when = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        for name in ("shank", "swollen3"):
            sel[name + "_key_hex"] = os.urandom(32).hex()
        sel["round3_rule"] = "tests/review/select_set.py (its docstring), rules 8 to 11"
        sel["round3_drawn_utc"] = when
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--draw-round2" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "seized_key_hex" in sel:
            raise SystemExit("the keys of rules 5 to 7 are drawn already (once)")
        when = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        for name in ("seized", "swollen", "foam"):
            sel[name + "_key_hex"] = os.urandom(32).hex()
        sel["round2_rule"] = "tests/review/select_set.py (its docstring), rules 5 to 7"
        sel["round2_drawn_utc"] = when
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--draw-appearance" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "appearance_key_hex" in sel:
            raise SystemExit("the appearance key is drawn already (rule 4: once)")
        sel["appearance_key_hex"] = os.urandom(32).hex()
        sel["appearance_rule"] = "tests/review/select_set.py (its docstring), rule 4"
        sel["appearance_drawn_utc"] = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--draw-sealant" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "sealant_key_hex" in sel:
            raise SystemExit("the sealant key is drawn already (rule 3: once)")
        sel["sealant_key_hex"] = os.urandom(32).hex()
        sel["sealant_rule"] = "tests/review/select_set.py (its docstring), rule 3"
        sel["sealant_drawn_utc"] = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--draw-cells" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "cells_key_hex" in sel:
            raise SystemExit("the cells' key is drawn already (rule 2: once)")
        sel["cells_key_hex"] = os.urandom(32).hex()
        sel["cells_rule"] = "tests/review/select_set.py (its docstring), rule 2"
        sel["cells_drawn_utc"] = __import__("datetime").datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--write" in sys.argv[1:]:
        key = os.urandom(32)
        with open(SEL, "w") as f:
            json.dump({"key_hex": key.hex(), "rule": "tests/review/select_set.py (its docstring)", "series_version": S.VERSION,
                       "summaries": []}, f, indent=1)
    with open(SEL) as f:
        sel = json.load(f)
    key = bytes.fromhex(sel["key_hex"])
    hidden, dev = with_cells(sel, b"hidden", draw(key, b"hidden", HIDDEN)), with_cells(sel, b"dev", draw(key, b"dev", DEV))
    if "--write" in sys.argv[1:] or "--refresh" in sys.argv[1:]:
        sel["series_version"] = S.VERSION
        with open(PACKS, "w") as f:
            json.dump({"packs": hidden, "note": "the fixed hidden set: tests/review/select_set.py"}, f, indent=1)
            f.write("\n")
        with open(DEVFILE, "w") as f:
            json.dump(dict(SERVICE, packs=dev), f, indent=1)
            f.write("\n")
        sel["summaries"] = [S.summary(S.draw_pack(**p)) for p in hidden]
        sel["dev_summaries"] = [S.summary(S.draw_pack(**p)) for p in dev]
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
    with open(PACKS) as f:
        ok = json.load(f)["packs"] == hidden
    with open(DEVFILE) as f:
        ok = ok and json.load(f)["packs"] == dev
    for name, packs in (("hidden", hidden), ("dev", dev)):
        for k, p in enumerate(packs):
            s = S.summary(S.draw_pack(**p))
            print(name, k, s["variant"], "harness", s["harness"], s["harness_covers"], "missing", s["missing"], "corroded", s["corroded"],
                  s["heads"], "value", s["value"], "cameras", "drawn" if "cameras" in p else "NOT DRAWN", "sealed",
                  s["sealed"] if "sealant_seed" in p else "NOT DRAWN", "seized", s["seized"], "swollen", s["swollen"],
                  "pads", s["pads"], "weak", s["weak"], "bumps_mm", s["bumps_mm"])
    print("files match the key:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
