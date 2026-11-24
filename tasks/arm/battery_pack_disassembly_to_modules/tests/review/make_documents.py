#!/usr/bin/env python3
"""Write the agent's documents from the cell's own numbers (maintainers' tool; nothing runs it at grading).

    python3 make_documents.py [--photos DIR]

It writes, under environment/setup/payload/given/: `service_manual.md` with its drawings (`manual/*.png`),
`tool_catalog.md`, `cell/cell_interface.md`, `cell/cell.json` and `robot/ur20.json`, and the reference's copy of
`cell.json` (solution/reference/). Every number comes from `tests/packcell/series.py`, `wire.py` and `kin.py`, so
the documents cannot drift from the cell: after a change there, run this again and commit the result.
`--photos DIR` copies the photographs made by `make_photos.py` (which needs Isaac Sim) into `manual/`; without it the
photographs already in place are kept.
"""

import argparse
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from packcell import kin, wire  # noqa: E402
from packcell import series as S  # noqa: E402

GIVEN = os.path.join(TASK, "environment", "setup", "payload", "given")
MANUAL = os.path.join(GIVEN, "manual")
E = S.ENGAGE
D = wire.DEFAULTS
NR, GR = S.TOOLS["nutrunner"], S.TOOLS["gripper"]


def mm(v):
    return "%g" % round(v * 1000.0, 1)


def table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


# ------------------------------------------------------------------------------------------------- drawings
def draw_variant(variant, lid, path):
    from PIL import Image, ImageDraw, ImageFont
    nom = S.nominal(variant)
    sc, m = 1100.0, 90                   # pixels per metre, margin
    (sx, sy) = S.TRAY["size"]
    W, H = int(sy * sc) + 2 * m, int(sx * sc) + 2 * m
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
        small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
    except OSError:
        font = small = ImageFont.load_default()

    def px(x, y):                        # as the overhead camera sees it: +x up, +y to the left
        return (m + (sy / 2 - y) * sc, m + (sx / 2 - x) * sc)

    def rect(x0, x1, y0, y1, **kw):
        a, b = px(x1, y1), px(x0, y0)
        d.rectangle([a[0], a[1], b[0], b[1]], **kw)

    rect(-sx / 2, sx / 2, -sy / 2, sy / 2, outline="black", width=3, fill=(235, 235, 235) if lid else (250, 250, 250))
    if lid:
        for axis, c in nom["ribs"]:
            if axis == "x":
                rect(-0.25, 0.25, c - 0.015, c + 0.015, outline=(90, 90, 90), fill=(200, 200, 200))
            else:
                rect(c - 0.015, c + 0.015, -0.43, 0.43, outline=(90, 90, 90), fill=(200, 200, 200))
        rect(-0.06, 0.06, -0.015, 0.015, outline="black", fill=(240, 220, 60))
        d.text(px(0.0, -0.03), "handle", fill="black", font=small)
        for f in nom["fasteners"]:
            if f["group"] != "lid":
                continue
            u, v = px(*f["nominal"])
            d.ellipse([u - 7, v - 7, u + 7, v + 7], outline="black", width=2, fill="white")
            inward = (0.0 - f["nominal"][0], 0.0 - f["nominal"][1])
            n = math.hypot(*inward)
            tu, tv = px(f["nominal"][0] + 0.035 * inward[0] / n, f["nominal"][1] + 0.035 * inward[1] / n)
            d.text((tu - 12, tv - 8), f["id"], fill="black", font=font)
    else:
        wall = S.TRAY["wall"]
        rect(-sx / 2 + wall, sx / 2 - wall, -sy / 2 + wall, sy / 2 - wall, outline=(120, 120, 120), width=1)
        ml, mw, _ = S.MODULE["size"]
        for p in nom["parts"]:
            x, y = p["pos"][0], p["pos"][1]
            if p["kind"] == "module":
                rect(x - ml / 2, x + ml / 2, y - mw / 2, y + mw / 2, outline="black", width=2, fill=(215, 220, 228))
                rect(x - 0.05, x + 0.05, y - 0.015, y + 0.015, outline="black", fill=(240, 220, 60))
                d.text(px(x + 0.11, y + 0.02), p["id"], fill="black", font=font)
            elif p["kind"] == "busbar":
                hx, hy = (p["length"] / 2, 0.011) if p["along"] == "x" else (0.011, p["length"] / 2)
                rect(x - hx, x + hx, y - hy, y + hy, outline=(120, 60, 0), width=2, fill=(240, 150, 60))
                tu, tv = px(x - 0.03, y + 0.012)
                d.text((tu, tv), p["id"], fill=(120, 60, 0), font=small)
            elif p["kind"] == "bracket":
                rect(x - 0.015, x + 0.015, y - 0.03, y + 0.03, outline="black", fill=(210, 195, 120))
                tu, tv = px(x + (0.012 if x > 0 else -0.02), y + 0.055)
                d.text((tu, tv), p["id"], fill="black", font=small)
            elif p["kind"] == "msd":
                rect(x - 0.03, x + 0.03, y - 0.02, y + 0.02, outline="black", width=2, fill=(255, 120, 30))
                d.text(px(x - 0.04, y + 0.02), "MSD", fill="black", font=font)
            elif p["kind"] == "connector":
                rect(x - 0.011, x + 0.011, y - 0.017, y + 0.017, outline="black", fill=(170, 170, 170))
                d.text(px(x - 0.02, y + 0.012), p["id"], fill="black", font=small)
        rect(S.SERVICE_X - 0.06, S.SERVICE_X + 0.06, nom["bms_y"] - 0.09, nom["bms_y"] + 0.09, outline="black", width=1)
        d.text(px(S.SERVICE_X + 0.055, nom["bms_y"] + 0.085), "BMS", fill="black", font=small)
        for f in nom["fasteners"]:
            if f["group"] == "lid":
                continue
            u, v = px(*f["nominal"])
            r = 4 if f["group"] == "busbar" else 5
            d.ellipse([u - r, v - r, u + r, v + r], outline="black", width=1, fill="white")
    # axes
    o = px(0.0, 0.0)
    d.line([o, px(0.08, 0.0)], fill=(200, 0, 0), width=3)
    d.line([o, px(0.0, 0.08)], fill=(0, 140, 0), width=3)
    d.text(px(0.10, 0.012), "x", fill=(200, 0, 0), font=font)
    d.text(px(0.012, 0.10), "y", fill=(0, 140, 0), font=font)
    d.text((m, 20), "HVB-%s  %s  (top view: the pack frame's +x up, +y to the left; the robot stands below the picture)"
           % (variant, "lid on" if lid else "lid off"), fill="black", font=font)
    d.text((m, H - 40), "tray %s x %s mm; drawing to scale, positions in the manual's tables" % (mm(sx), mm(sy)), fill="black", font=small)
    img.save(path, optimize=True)


# ------------------------------------------------------------------------------------------------- service manual
def service_manual():
    z0 = S.Z
    t = []
    t.append("""# HVB-P traction battery: workshop manual, removal of the modules

Issue 3. Applies to the plug-in hybrid packs HVB-P4, HVB-P5 and HVB-P6 (4, 5 and 6 modules in one common housing).
This manual describes the pack as built, what a pack returned from the field may look like, and the order in which it
is taken apart to module level. Dimensions are nominal unless a tolerance is given.

## 1. Safety: high voltage

The busbars, the modules' terminal posts and the junction studs are live as long as the service disconnect (MSD) is
in its socket: the modules are then connected in series (up to 320 V). **Nothing conductive may touch a busbar, a
busbar bolt, a terminal post or a junction stud before the pack is isolated.** In the disassembly cell an insulation
monitor trips when a tool touches (comes within %s mm of) one of them before isolation: the cell stops for good and
the pack is taken away as it is.

Isolation procedure:

1. Remove the lid (section 6, steps 1 and 2).
2. Pull the service disconnect straight up out of its socket and take it away.
3. Wait %g s: the pack's link capacitor discharges through its bleed resistor.

After that the string is open and no part carries more than one module's voltage (below 60 V): busbars and their
bolts may be touched and removed. The lid's bolts, the lid, the service disconnect, the low-voltage plugs, the
brackets and their bolts and the modules' housings are never live.
""" % (mm(S.HV_MARGIN), S.ISOLATION_WAIT_S))
    t.append("""## 2. The series

All three variants use the same tray and the same lid outline; they differ in the number of modules, in where the
service disconnect and the battery management unit (BMS) sit, and in the lid:

""" + table(["variant", "modules", "lid bolts", "lid ribs (raised strips on the lid)", "MSD at y =", "BMS at y ="],
            [["HVB-" + v, V["modules"], len(V["lid_bolts"]),
              "; ".join("along %s at %s = %s mm" % ("y" if a == "y" else "x", "x" if a == "y" else "y", mm(c)) for a, c in V["ribs"]),
              mm(V["msd_y"]) + " mm", mm(V["bms_y"]) + " mm"] for v, V in sorted(S.VARIANTS.items())]) + """

The variant is not marked on the pack. Tell it from the lid (rib pattern, number of lid bolts) and, once the lid is
off, from the number of modules.

Drawings: `manual/P4_lid.png`, `manual/P4_inside.png` (and P5, P6). Photographs of an example pack and of the
fastener heads: `manual/photo_*.jpg` (section 8).
""")
    t.append("""## 3. The pack frame and the heights

The pack frame P has its origin on the fixture's surface under the centre of the tray; x and y run along the tray's
short and long sides, z is up. On the cell's fixture a pack is placed with its frame at the fixture's nominal
position, x and y parallel to the cell's x and y (`cell/cell_interface.md`), within the placement tolerance of
section 7. Every position in this manual is in the pack frame, in millimetres.

The tray is %s x %s mm outside and %s mm high; its walls are %s mm thick, its floor %s mm.

""" % (mm(S.TRAY["size"][0]), mm(S.TRAY["size"][1]), mm(S.TRAY["height"]), mm(S.TRAY["wall"]), mm(S.TRAY["bottom"]))
             + table(["height z (mm)", "what"], [
                 [mm(S.TRAY["height"]), "top of the tray's walls; underside of the lid"],
                 [mm(z0["lid_seat"]), "top of the lid; seat of the lid bolts' heads"],
                 [mm(z0["lid_center"] + S.LID["handle_z"]), "centre of the lid's handle bar (where the gripper takes it)"],
                 [mm(S.TRAY["bottom"] + S.MODULE["size"][2]), "top of a module's housing"],
                 [mm(z0["module_center"] + S.MODULE["handle_z"]), "centre of a module's handle bar (%s mm above it: the bar's top)" % mm(0.006)],
                 [mm(z0["post_top"]), "top of the terminal posts and junction studs; underside of the busbars"],
                 [mm(z0["busbar_center"]), "mid-height of a busbar (where the gripper takes it, on its orange sleeve)"],
                 [mm(z0["busbar_seat"]), "top of a busbar; seat of the busbar bolts' heads"],
                 [mm(z0["bracket_seat"]), "top of a bracket; seat of the bracket bolt's head"],
                 [mm(z0["bracket_center"]), "mid-height of a bracket (where the gripper takes it)"],
                 [mm(0.135), "centre of the service disconnect's handle bar"],
                 [mm(0.077), "mid-height of the low-voltage plugs J1 and J2 (where the gripper takes them)"],
             ]) + """

A bolt's head stands above its seat by the head height of section 5.
""")
    # per-variant tables
    t.append("## 4. Components and their nominal places\n")
    t.append("""Grip features (the gripper closes on them from above; the closing direction is the direction the fingers
move in, in the pack frame):

""" + table(["component", "grip feature", "closing direction", "width (mm)", "mass (kg)"], [
        ["lid", "handle bar at the lid's centre", "y", "30", "%g" % S.LID["mass"]],
        ["module M1..", "handle bar on top, at the module's centre", "y", "30", "%g" % S.MODULE["mass"]],
        ["service disconnect (MSD)", "handle bar on top", "y", "20", "0.35"],
        ["plugs J1, J2", "the plug's body", "x", "22", "0.05"],
        ["busbar between two modules (runs along y)", "the orange sleeve at its middle", "x", "26", "0.12"],
        ["first and last busbar (run along x)", "the orange sleeve at its middle", "y", "26", "0.12"],
        ["bracket", "the block itself", "x (30 mm) or y (60 mm)", "30 / 60", "0.15"],
        ["harness remnant (section 7)", "the cable, anywhere along it", "across the cable", "12", "0.2"],
    ]) + """

What holds what:

- the lid: all of its bolts;
- a busbar: its two bolts (`a` and `b`);
- a bracket: its one bolt;
- a module: its two brackets (`a` at the far end, `b` at the service-bay end) and the two busbars on its terminal
  posts; it comes free when all four have been taken off;
- the service disconnect: its latch; it comes out when pulled straight up with more than 60 N;
- a plug J1, J2: its latch; it comes out when pulled straight up with more than 30 N.

A component that is still held does not come away; pulling harder only overloads the robot (the cell stops it).

Appearance: the lid is dark grey with a yellow handle bar; the tray is bare aluminium; a module is grey with a
lighter top plate, a yellow handle bar and two copper terminal posts at its service-bay end; busbars are copper
with an orange sleeve; brackets are yellow-passivated steel blocks; the service disconnect is orange with a black
handle on an orange base; the BMS is a black box, J1 a grey and J2 a white plug with a black latch; the junction
blocks are red; fasteners are zinc-plated (bright) unless corroded.
""")
    for v in sorted(S.VARIANTS):
        nom = S.nominal(v)
        t.append("### HVB-%s\n" % v)
        rows = [[f["id"], f["thread"], mm(f["nominal"][0]), mm(f["nominal"][1]), mm(f["seat_z"]), f["holds"]] for f in nom["fasteners"]]
        t.append(table(["fastener", "thread", "x", "y", "seat z", "holds"], rows) + "\n")
        rows = []
        for p in nom["parts"]:
            g = p["grasps"][0]
            rows.append([p["id"], p["kind"], mm(p["pos"][0] + g["pos"][0]), mm(p["pos"][1] + g["pos"][1]), mm(p["pos"][2] + g["pos"][2])])
        t.append(table(["component", "kind", "grip point x", "y", "z"], rows) + "\n")
    t.append("""## 5. Fasteners

Two thread sizes, each with two head types as built. A group (lid, busbars, brackets) is built with one head type, but
which one differs from pack to pack, and single fasteners have been replaced in service by the other head of the same
thread: **look at every head before choosing the bit.**

""" + table(["head", "thread", "shape", "size", "head height (mm)"], [
        [h, H["thread"], "hexagon on a round flange" if H["kind"] == "hex" else "round pan head with a six-lobe (Torx) recess",
         ("%s mm across flats, flange dia. %s mm" % (mm(H["af"]), mm(2 * H["flange_r"]))) if H["kind"] == "hex"
         else ("head dia. %s mm, recess %s mm across" % (mm(2 * H["head_r"]), mm(2 * H["recess_r"]))), mm(H["height"])]
        for h, H in sorted(S.HEADS.items())]) + """

Lid and busbar fasteners are M6, bracket fasteners M8.

""" + table(["thread", "assembly torque (N m)", "breakaway, clean (N m)", "breakaway, corroded (N m)", "never exceed (N m)", "turns to free"], [
        [k, "%g" % T["tighten"], "%g" % (1.15 * T["tighten"]), "%g to %g" % (1.6 * T["tighten"], 1.9 * T["tighten"]),
         "%g" % T["head_limit"], "%g" % T["turns"]] for k, T in sorted(S.THREADS.items())]) + """

- **Loosening.** A fastener breaks away at its breakaway torque over the first %g degrees and then runs out at about
  %d %% of the assembly torque. After the number of turns in the table it is free; it stays on the bit (the bits are
  magnetic) until it is ejected.
- **Corroded fasteners** (rust-brown head, a rust stain around it) need the higher breakaway torque. A nutrunner
  whose torque limit is below a fastener's breakaway torque stalls at the limit without turning it: raise the limit,
  never above the "never exceed" value of the thread.
- **Tightening by mistake.** A seated fastener does not turn further: the spindle stalls at its limit. With the limit
  above the "never exceed" value the head gives way within half a second.
- **The wrong bit, or a bit that is not square on the head, wears the head** (section 3 of `tool_catalog.md` has the
  tolerances). A worn-out (stripped) head no longer takes its bit; only the extractor bit EX removes such a fastener.
- **Missing fasteners.** Field packs come back with fasteners missing (an empty, dark hole). There is nothing to
  remove there; what the fastener held may already be loose.
""" % (E["breakaway_deg"], round(100 * E["running_fraction"])))
    t.append("""## 6. Order of work

1. Remove every bolt of the lid.
2. Lift the lid off by its handle and put it on the lid stand.
3. Pull the service disconnect; wait %g s (section 1).
4. Pull the low-voltage plugs J1 and J2 from the BMS.
5. Remove the busbar bolts, then the busbars.
6. Remove the bracket bolts, then the brackets.
7. Lift each module out by its handle and stand it in a bay of the module rack.

Steps 4 to 6 may be interleaved as long as nothing live is touched before step 3 is complete and every component is
free before it is lifted. Sort everything that comes off (section 9).
""" % S.ISOLATION_WAIT_S)
    t.append("""## 7. What a field pack may look like

- **Placement on the fixture.** The fixture's stops locate the tray within +/- 12 mm in x and y and +/- 1.5 degrees
  about z of the nominal position.
- **Fastener positions.** Holes are drilled within 2 mm of their nominal positions.
- **Head types** vary as described in section 5; some fasteners are corroded, some are missing.
- **A harness remnant.** Some packs carry a piece of blue low-voltage harness (cable 12 mm thick, 0.2 to 0.4 m long,
  a grey plug shell at each end) that was cut and left behind: lying on the lid over one or two lid bolts, or inside
  the pack along the busbars over two or three busbar bolts. It is loose. A bolt under it cannot be reached; take
  the harness away first (grip the cable anywhere, across its direction). Never run the nutrunner on it: a turning
  bit pressed on the cable chafes it through. A harness lying on the busbars lies on live parts until the pack is
  isolated.
""")
    t.append("""## 8. Photographs

`manual/photo_example_lid.jpg` and `manual/photo_example_inside.jpg`: an example pack (an HVB-P5) under the cell's
overhead camera, lid on and lid off. `manual/photo_heads.jpg`: the four head types, clean and corroded, and an empty
hole, as the wrist camera sees them from about 0.35 m (cropped and enlarged). For these photographs the example
pack's lid was fitted with M8 heads as well, which a lid never carries. The example pack is not one of the packs you
will meet.
""")
    t.append("""## 9. Sorting and recovered value

""" + table(["component", "goes to", "value"], [
        ["module", "a bay of the module rack, standing on the bay's floor (one module per bay)", "%g" % S.VALUES["module"]],
        ["lid", "the lid stand, lying flat", "%g" % S.VALUES["lid"]],
        ["service disconnect", "electronics bin", "%g" % S.VALUES["msd"]],
        ["busbar", "copper bin", "%g" % S.VALUES["busbar"]],
        ["bracket", "steel bin", "%g" % S.VALUES["bracket"]],
        ["plug J1, J2", "electronics bin", "%g" % S.VALUES["connector"]],
        ["harness remnant", "electronics bin", "%g" % S.VALUES["harness"]],
        ["fastener", "either fasteners bin", "%g" % S.VALUES["fastener"]],
    ]) + """

A component counts when it lies undamaged where it belongs. Damaged means: a fastener with a stripped head, a harness
chafed by the nutrunner, a module that was dropped.
""")
    return "\n".join(t)


# ------------------------------------------------------------------------------------------------- tool catalog
def tool_catalog():
    fits = []
    for bit in S.BITS:
        row = [bit]
        for h in sorted(S.HEADS):
            how = "wears it" if bit == "EX" else {"fit": "drives it", "loose": "wears it"}.get(S.BIT_ON_HEAD.get((bit, h)), "turns freely")
            row.append(how)
        fits.append(row)
    return """# Disassembly cell: tool catalog

Two tools hang in the tool stand; the robot carries one at a time on its tool changer. The wrist camera sits on the
changer's robot side, so it is there with either tool (`cell/cell_interface.md`, section 5).

## 1. Tool changer

Each tool has a dock on the tool stand. A tool is set down or picked up with the flange in the dock position, the
tool's axis straight down and the flange's x axis along the cell's x axis, within %s mm and %g degrees, at rest.
`tool_release` sets the mounted tool down in its own dock; `tool_lock` picks up the tool of the dock the bare flange
is in. Either takes %g s, during which the robot does not move. A tool that holds something (a fastener on the bit,
a part in the gripper) cannot be set down.

## 2. Nutrunner NR-60

A straight electric nutrunner with a sprung, floating spindle and exchangeable magnetic bits.

""" % (mm(S.CELL["dock_tol"]["pos"]), S.CELL["dock_tol"]["angle_deg"], D["tool_change_s"]) + table(["", ""], [
        ["mass", "%g kg" % (NR["mass"] + NR["spindle_mass"])],
        ["tool centre point", "the bit's tip, %s mm from the flange along the tool axis with the spindle unloaded" % mm(NR["tcp_z"])],
        ["spindle float", "%s mm; spring %g N per mm of compression (`stroke_mm` in the observation)" % (mm(NR["stroke"]), NR["spring_N_per_m"] / 1000.0)],
        ["speed", "up to %g rpm either way; a negative speed loosens" % NR["speed_max_rpm"]],
        ["torque limit", "settable from 0 to %g N m; the spindle stalls at the limit" % NR["torque_max"]],
        ["readings", "torque (N m), speed (rpm), angle (degrees, cumulative, signed), stroke (mm)"],
        ["bit and spindle diameter", "below %s mm over the last %s mm" % (mm(2 * 0.011), mm(NR["spindle_len"] + NR["bit_len"]))],
        ["body", "diameter %s mm, from %s to %s mm from the flange" % (mm(2 * NR["body"]["r"]), mm(NR["body"]["z0"]), mm(NR["body"]["z1"]))],
    ]) + """

**Bits.** S10 and S13: sockets for 10 and 13 mm hexagons. T30 and T40: Torx bits. EX: an extractor for stripped
heads. The bit is exchanged in the bit station of the tool stand: the bit's tip in the station position, tool axis
straight down, within %s mm and %g degrees, at rest; `bit:<name>` then takes %g s. The nutrunner starts every pack
with S10.

What a turning bit does on a head:

""" % (mm(S.CELL["dock_tol"]["pos"]), S.CELL["dock_tol"]["angle_deg"], D["bit_change_s"]) + table(["bit"] + ["on " + h for h in sorted(S.HEADS)], fits) + """

## 3. Engaging a fastener

The bit drives a fastener when all of this holds:

- it is the head's own bit (table above);
- the bit's axis is within %s mm of the fastener's axis for a socket, %s mm for a Torx bit, and the two axes are
  within %g degrees of each other;
- the bit is pressed on the head with at least %g N (the spindle's spring: stroke times the spring rate); a Torx bit
  in addition needs %g N plus %g N for every N m it transmits, or it cams out of the recess;
- the torque limit is at least what the fastener needs (the service manual's torque table).

A bit that is on the head but outside these tolerances (up to %s mm off for a socket, %s mm for a Torx bit, up to
%g degrees), a bit one size too large or too small in the "wears it" cells, or a Torx bit that cams out, turns on
the head without driving it: the reading shows a low, uneven torque while the angle runs on, and the head wears. A
head is stripped after about one and a half turns of that; a Torx recess that is cammed out, after about one turn.
Stop as soon as the readings show it. Further off than that, or with a bit that does not fit at all, the bit turns
freely (torque about %g N m) and nothing happens to the head.

A spindle that is stalled at its limit on a fastener it should drive is doing no harm, as long as the limit is not
above the thread's "never exceed" value.

**The extractor EX** bites into a stripped head: within %s mm and %g degrees, pressed on with at least %g N, with
the torque limit at %g times the fastener's breakaway torque; it turns the fastener out at half the spindle's speed.
On a sound head it only does damage.

**A freed fastener** stays on the bit. `eject` lets it fall; do that over a fasteners bin. With a fastener on the
bit the nutrunner engages nothing else and the bit cannot be exchanged.

## 4. Gripper GP-85

A parallel gripper with two long fingers.

""" % (mm(E["hex"]["full"]), mm(E["torx"]["full"]), E["tilt_full_deg"], E["seat_force"], E["torx_force_base"], E["torx_force_per_Nm"],
       mm(E["hex"]["partial"]), mm(E["torx"]["partial"]), E["tilt_partial_deg"], E["free_torque"],
       mm(E["extractor"]["lateral"]), E["tilt_full_deg"], E["extractor"]["force"], E["extractor"]["torque_factor"]) + table(["", ""], [
        ["mass", "%g kg" % GR["mass"]],
        ["opening", "0 to %s mm, %s mm/s; the fingers move along the flange's y axis" % (mm(GR["open_max"]), mm(GR["speed"]))],
        ["tool centre point", "between the fingertips, %s mm from the flange along the tool axis, %s mm above the fingers' ends" % (mm(GR["tcp_z"]), mm(0.015))],
        ["fingers", "%s mm long, %s mm wide (along the flange's x axis)" % (mm(GR["finger_len"]), mm(GR["finger_w"]))],
        ["body", "%s x %s mm (flange x by y), from %s to %s mm from the flange" % (mm(GR["body"]["size"][0]), mm(GR["body"]["size"][1]), mm(GR["body"]["z0"]), mm(GR["body"]["z0"] + GR["body"]["size"][2]))],
        ["readings", "opening (m), holding (true while the fingers hold a part)"],
    ]) + """

**Gripping.** The gripper takes a part by one of its grip features (the service manual, section 4). Closing the
fingers on a feature holds the part when

- the tool axis points down onto the part within 20 degrees of the part's own vertical,
- the fingers' closing direction is within 20 degrees of the feature's (25 degrees across a harness cable),
- the feature lies between the fingers: the tool centre point within 12 mm of the grip point along the tool axis and
  along the flange's x axis, and the feature inside the opening,
- the opening before closing is at least the feature's width.

The fingers stop at the feature's width and `holding` becomes true. The part stays as it lay relative to the
fingers; it is not centred. Opening the gripper (commanding an opening more than 1 mm wider) lets go.

The gripper holds any of the pack's components against its own weight in any motion the robot's limits allow. It
does not tear a held component loose: a part that is still fastened stays where it is and the robot is overloaded
(the cell stops it). The plugs and the service disconnect are the exception: they are meant to be pulled out
(service manual, section 4).
"""


# ------------------------------------------------------------------------------------------------- the cell's interface
def cell_interface():
    c = S.public_cell()
    bins = table(["bin", "centre x, y (m)", "inner size x by y (m)", "walls (m)"],
                 [[k, "%g, %g" % tuple(b["center"]), "%g x %g" % tuple(b["size"]), "%g" % b["wall_h"]] for k, b in c["bins"].items()])
    cam = []
    for name, k in c["cameras"].items():
        cam.append("- `%s`: %d x %d pixels, fx = fy = %.2f, cx = %g, cy = %g, no distortion; mounted on the %s at %s (m), camera axes (x right, "
                   "y down, z forward) as the columns of R = %s. Depth from %g to %g m." % (
                       name, k["width"], k["height"], k["K"][0][0], k["K"][0][2], k["K"][1][2],
                       "cell (base frame)" if k["parent"] == "world" else "flange (in the flange frame)", k["pos"],
                       [[round(v, 4) for v in row] for row in k["R"]], k["clip"][0], min(k["clip"][1], 6.5)))
    return """# Disassembly cell: interface document

The cell: a Universal Robots UR20 on a pedestal, a fixture that holds one battery pack in front of it, a tool stand
with a nutrunner and a gripper, a module rack, a lid stand, five bins, an RGB-D camera over the fixture and one on the
robot's wrist. Your program drives the cell through the wire of section 2. All numbers of this document are also in
`cell.json` beside it.

## 1. Frames and places

The base frame: origin on the robot's mounting face, z up, x toward the fixture. All positions are in metres in
this frame unless a document says otherwise. The floor is at z = %g, the surface of the fixture and of every table
at z = %g.

- **Fixture.** A pack's frame (service manual, section 3) sits nominally at %s, axes parallel to the base frame's.
- **Tables** (x from, x to, y from, y to; all at table height): %s. The robot's pedestal is a column of 0.16 m
  radius under the base.
- **Bins** (open boxes standing on the tables, floor at table height):

%s

- **Module rack.** Six bays in a row, each %g m wide (x) and %g m long (y), centres at y = %g and x = %s; dividers
  %g m high. A module stands in a bay with its long side along y.
- **Lid stand.** A table with its centre at %s, %g x %g m.
- **Tool stand.** Flange position in the dock: nutrunner %s, gripper %s. Bit station (position of the bit's tip):
  %s. Parked tools reach up to about z = 0.0.
- **Robot.** Joint limits +/- 360 degrees (elbow +/- 180). Home joint angles (rad): %s. Kinematic model:
  `../robot/ur20.json`; the robot's USD description: `../robot/ur20_usd/`. The flange frame is the robot's
  `wrist_3_link` frame: origin on the flange face, z out of the flange.

## 2. The wire

The grader starts your program with `bash run.sh` in the submission folder and talks to it over its standard input
and output, one JSON object per line. Your program first prints `READY` (within 120 s), then answers every request
(each within 30 s):

```
{"type": "reset"}                        -> {"ok": true}            a new pack is on the fixture
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t of this pack
{"type": "close"}                        -> {"ok": true}, then exit
```

Standard output is the wire: write logs to standard error. `t` is the step index of the pack, owned by the grader,
starting at 0 after each `reset`.

**A step may be asked more than once with the same `t`.** A repeated request carries an observation whose robot
readings (`q`, `q_target`, `tcp`, `tcp_target`, `wrench`) are shifted by a small hidden amount (joints up to 0.02
rad, positions up to 12 mm, rotations up to 0.03 rad, forces up to 3 N), the same shift for the measured and the
commanded values of one observation; the other reading is the true one, and nothing on the wire says which is which
or which comes first. Only the answer to the true observation is executed. Answer every request from the
observation it carries, and advance your own state once per `t`, not once per request. A program whose motions do
not depend on the robot's readings it is sent (a replayed list of actions, absolute joint targets worked out in
advance) is recognised by this and scores nothing: compute each displacement from the `tcp_target` or `q_target`
of the observation you answer. Requests that command no motion are not compared.

After a missed deadline, a closed output, an answer that is not a valid action or an exhausted budget, the pack
ends where it is (what has been sorted so far still counts), your program is stopped with everything it started,
and `run.sh` is started again for the next pack (at most three such restarts; after that the remaining packs are
not worked on at all).

## 3. Actions

An action is a JSON object; every key is optional:

```
{"move": {"type": "tcp" | "joint" | "hold", "delta": [6 numbers], "ticks": 1..%d,
          "guard_force": N, "guard_stroke_mm": mm},
 "spindle": {"speed_rpm": signed, "torque_limit": N m},
 "gripper": opening in metres,
 "cmd": "eject" | "tool_release" | "tool_lock" | "bit:S10" | "bit:S13" | "bit:T30" | "bit:T40" | "bit:EX" | "done",
 "cams": ["overhead", "wrist"]}
```

- **move.** One action lasts `ticks` control ticks of %g s. `"tcp"` displaces the commanded pose of the mounted
  tool's centre point (the flange's, with no tool) by `delta` = [dx, dy, dz, rx, ry, rz]: a translation (m) and a
  rotation vector (rad), both in the base frame, the rotation applied on the left of the current orientation, on a
  straight line with a smooth start and stop. `"joint"` displaces the six commanded joint angles (rad). `"hold"`
  keeps the commanded pose. The displacement always starts from the commanded target (`tcp_target`, `q_target`),
  not from the measured pose. Speed limits: %g m/s and %g rad/s at the tool centre point, joints %s rad/s; a move
  that would exceed them, or that the arm cannot follow (near a singularity, at a joint limit, out of reach), is
  shortened and the observation says `clipped`.
- **guards.** `guard_force` ends the move when the wrench's force has changed by more than that many newtons since
  the move began; `guard_stroke_mm` ends it when the nutrunner's spindle is compressed further than that. The
  observation then says `guard`.
- **spindle.** Runs the nutrunner's spindle for the duration of this action (and only this action).
- **gripper.** The opening the gripper moves to; it keeps moving in later actions until it gets there or holds a
  part.
- **cmd.** Executed after the move: `eject` drops the fastener on the bit; `tool_release`, `tool_lock` and
  `bit:<name>` are the exchanges of the tool catalog; `done` ends the pack. The observation's `events` say whether
  a command was carried out (`"tool_lock: ok, gripper"`) or why not (`"...: refused, ..."`).
- **cams.** The cameras to take a frame from at the end of this action (section 5).

An action with an unknown key, a wrong type or a number out of range is not a valid action (section 2).

## 4. Observations

```
{"time": s,                       simulated time since the pack arrived
 "q": [6], "qd": [6],             measured joint angles (rad) and speeds (rad/s)
 "q_target": [6],                 commanded joint angles
 "tcp": [x, y, z, rx, ry, rz],    measured pose of the tool centre point: position (m), rotation vector (rad)
 "tcp_target": [...],             commanded pose of the tool centre point
 "wrench": [fx, fy, fz, tx, ty, tz],   what the world exerts on the tool: force (N) and moment about the flange's
                                  origin (N m), base frame; the mounted tool's own weight is compensated, a held
                                  part's is not; noise about 0.4 N and 0.02 N m
 "tool": "nutrunner" | "gripper" | "none", "bit": "S10" | ... | null,
 "spindle": {"torque": N m, "speed_rpm": .., "angle_deg": .., "stroke_mm": ..},
 "gripper": {"width": m | null, "holding": bool},
 "status": {"clipped": bool, "guard": bool, "stopped": bool, "busy": bool,
            "locked_s": s, "busy_s": s, "protective_stops": n},
 "trace": {"force_axial": [...], "torque": [...], "angle_deg": [...], "stroke_mm": [...]},
                                  one sample per tick of the action just executed: the wrench's force along the
                                  tool axis (N), and the spindle's torque, angle and stroke
 "events": [strings], "frames_left": n, "time_left": s,
 "ended": null | "done" | "sim_cap" | "live_contact" | "module_dropped",
 "images": {"overhead": {"rgb": .., "depth": .., "depth_unit_m": 0.0001, "t": s},
            "wrist": {..., "pose": [x, y, z, rx, ry, rz]}}}
```

The nutrunner's tool centre point is the bit's tip with the spindle unloaded: with the spindle compressed the tip is
`stroke_mm` closer to the flange. Nothing on the wire says where a part or a fastener is, or what state it is in.

**Protective stops.** When the force on the tool exceeds %g N, the moment %g N m, or a joint lags its command by
more than %g rad (a collision, a part that is still fastened, a command the arm cannot follow), the robot backs off
a little and stands still for %g s (`stopped`, then `locked_s` counting down); moves commanded meanwhile are ignored.
`busy` and `busy_s` say the same for a tool or bit exchange. Protective stops are counted but cost nothing else.

**What ends a pack**: `done`; the simulated time is used up (`sim_cap`, %g s per pack); a tool touches a live part
before isolation (`live_contact`, service manual section 1); a module falls (`module_dropped`: a free module that is
not held and drops faster than 0.7 m/s, i.e. from more than about 25 mm). After the end nothing moves any more.

## 5. Cameras

Frames are rendered only when an action asks for them, at most %d per pack in all (each camera of each request
counts one). `rgb` is a PNG (base64), `depth` a 16-bit PNG (base64) of the distance along the optical axis in units
of 0.1 mm, 0 where there is no reading. `packcell_client.decode_rgb` and `decode_depth` read them.

%s

A pixel (u, v) with depth d is the camera-frame point ((u - cx) d / fx, (v - cy) d / fy, d). The wrist frame's
`pose` is the wrist camera's pose in the base frame when the frame was taken (from the measured joint angles). The
calibration is exact. The renderer is a path tracer with a denoiser: two frames of the same scene differ slightly
in their pixel values.

## 6. Budgets

Per pack: %g s of simulated time (`time_left`), %d frames (`frames_left`), %d steps, and %g s of your program's own
time: the time the grader waits for your answers, summed (the cell's simulation and rendering are not counted).
Every single answer within 30 s.

## 7. How a pack is judged

When a pack has ended and everything has come to rest, each component counts if it lies undamaged where it belongs
(service manual, section 9) and the gripper does not hold it:

- in a bin: its centre over the bin's inner footprint and not higher than 0.03 m above the top of the walls;
- the lid: its centre within 0.10 m of the stand's centre in x and in y, lying on the stand, flat within 5 degrees;
- a module: its centre over a bay, standing on the bay's floor, upright within 8 degrees;
- a fastener: in either fasteners bin.

The pack's credit is the value of what counts divided by the value of everything in the pack (a missing fastener is
not in the pack). Nothing is subtracted for anything.

## 8. Commissioning

The commissioning service in your sandbox runs this cell on %d development packs of the series: packs with their own
placements, head types, corroded and missing fasteners and harness remnants, none of them a pack you are graded on.
You get what a commissioning engineer has at the cell: the same actions and observations as above, the camera
frames, the robot's and the tools' readings. You get no ground truth, no credit and no debrief.

- `packcell_client.Session(pack)` (in `~/given/lib`) opens a session on development pack 1 to %d and steps it
  interactively; `~/commission/run_session.py --pack N` runs your `run.sh` in a session the way it is run when it is
  graded, repeated steps included.
- **%d sessions in all**, one at a time; a session is counted when it opens and ends when the pack ends, when you
  close it or after %d s without a request. `/run/packcell/status.json` shows how many are left.
- A session starts Isaac Sim on the sandbox's GPU: opening one takes one to two minutes, the very first one several
  minutes more (the renderer compiles its shaders once).

Isaac Sim itself is installed in the image (`/opt/venv`), with the robot's model; you may build simulations of your
own with it. The cell's own code is not readable from your account.

## 9. Numerics

Every process of the image, the grader's and yours, runs with `OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_NUM_THREADS=1`
and `OMP_NUM_THREADS=1` unless it sets them otherwise; the grader starts `run.sh` with `PATH`, `HOME` and `LANG`
only, so set them in `run.sh` if your arithmetic depends on them. The physics engine and the renderer are not
bit-reproducible across machines: build a program that closes its loops on what it measures.
""" % (S.FLOOR_Z, S.TABLE_Z, c["pack_nominal"], "; ".join("%s %s" % (k, list(v)) for k, v in c["tables"].items()), bins, c["rack"]["bay_size"][0], c["rack"]["bay_size"][1], c["rack"]["bay_centers"][0][1],
       ", ".join("%g" % round(b[0], 3) for b in c["rack"]["bay_centers"]), c["rack"]["divider_h"], c["lid_stand"]["center"],
       c["lid_stand"]["size"][0], c["lid_stand"]["size"][1], c["docks"]["nutrunner"], c["docks"]["gripper"], c["bit_station"], c["home_q"],
       D["max_ticks"], S.TICK, D["tcp_speed_max"], D["tcp_rot_speed_max"], D["joint_speed_max"],
       D["overload_force"], D["overload_torque"], D["overload_joint_error"], D["stop_lock_s"], D["sim_cap_s"], D["frame_budget"],
       "\n".join(cam), D["sim_cap_s"], D["frame_budget"], GRADER["max_steps"], GRADER["controller_budget_s"],
       len(DEV["packs"]), len(DEV["packs"]), DEV["sessions"], DEV["idle_s"])


def robot_json():
    return {"name": "Universal Robots UR20", "joint_names": list(kin.JOINT_NAMES),
            "note": "every joint turns about the z axis of its own frame; `joints` gives each joint frame in its parent link "
                    "(translation in m, quaternion w x y z), as in the USD description; the flange is wrist_3_link",
            "joints": [{"name": n, "translation": list(p), "quaternion_wxyz": list(q)} for n, (p, q) in zip(kin.JOINT_NAMES, kin.JOINTS)],
            "limits_rad": kin.Q_LIMITS.tolist(), "home_q": S.CELL["home_q"],
            "tool_centre_point_z": {"nutrunner": NR["tcp_z"], "gripper": GR["tcp_z"], "none": 0.0}}


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text if text.endswith("\n") else text + "\n")
    print("wrote", os.path.relpath(path, TASK))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--photos", default=None)
    args = ap.parse_args()
    with open(os.path.join(TASK, "tests", "grader_config.json")) as f:
        GRADER = json.load(f)
    with open(os.path.join(TASK, "environment", "commission_service", "dev_packs.json")) as f:
        DEV = json.load(f)
    os.makedirs(MANUAL, exist_ok=True)
    for v in sorted(S.VARIANTS):
        draw_variant(v, True, os.path.join(MANUAL, "%s_lid.png" % v))
        draw_variant(v, False, os.path.join(MANUAL, "%s_inside.png" % v))
    write(os.path.join(GIVEN, "service_manual.md"), service_manual())
    write(os.path.join(GIVEN, "tool_catalog.md"), tool_catalog())
    write(os.path.join(GIVEN, "cell", "cell_interface.md"), cell_interface())
    cell = json.dumps(S.public_cell(), indent=1)
    write(os.path.join(GIVEN, "cell", "cell.json"), cell)
    write(os.path.join(TASK, "solution", "reference", "cell.json"), cell)
    write(os.path.join(GIVEN, "robot", "ur20.json"), json.dumps(robot_json(), indent=1))
    if args.photos:
        for name in sorted(os.listdir(args.photos)):
            if name.startswith("photo_") and name.endswith(".jpg"):
                shutil.copy(os.path.join(args.photos, name), os.path.join(MANUAL, name))
                print("copied", name)
