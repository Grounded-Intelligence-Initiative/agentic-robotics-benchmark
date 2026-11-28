"""QM-24 robot descriptions: read, check and summarise a robot.xml built from the QM-24 catalog (pure Python + numpy).

The same file is the grader's build check and the design checker in your home (`check_design.py` calls `check`).
It never runs a simulator.

The robot.xml dialect (MJCF, a strict subset; nothing else is accepted):

    <mujoco model="any_name">
      <custom>
        <numeric name="stance_height" data="0.20"/>   hip height above the ground the gait walks at (m)
        <numeric name="knee_front" data="-1"/>        -1: front knees point backward, +1: forward
        <numeric name="knee_rear" data="-1"/>         the same for the rear legs
      </custom>
      <worldbody>
        <body name="torso" childclass="BODY_M">                         the body module (no pos)
          <body name="FL_abd" childclass="ACT_S" pos="0.16 0.08 0">     optional abduction actuator at the mount
            <joint name="FL_abd" axis="1 0 0"/>
            <body name="FL_hip" childclass="ACT_M" pos="0 0.034 0">     hip pitch actuator: at the mount, or at
              <joint name="FL_hip" axis="0 1 0"/>                        the abduction flange (0, side*len, 0)
              <body name="FL_thigh1" childclass="LINK_120" pos="0 0.040 0">   at the hip flange (0, side*len, 0)
                <body name="FL_knee" childclass="ACT_M" pos="0 0 -0.12">       at the end of the last thigh segment
                  <joint name="FL_knee" axis="0 1 0"/>
                  <body name="FL_shank1" childclass="LINK_120" pos="0 0 0">    at the knee axis
                    <body name="FL_foot" childclass="FOOT_R" pos="0 0 -0.12"/> at the end of the last shank segment
                  </body>
                </body>
              </body>
            </body>
          </body>
          ... FR, RL, RR the same (R legs: side = -1, mount y negative; rear legs: mount x negative)
        </body>
      </worldbody>
    </mujoco>

A thigh or shank is one or two link segments in series (`FL_thigh1`, `FL_thigh2` with pos "0 0 -L1", likewise
`FL_shank1`, `FL_shank2`). Positions must match the catalog geometry to 1e-4 m. Every leg is mounted at the body
module's mount point of its corner (x = +mount[0] front, -mount[0] rear; y = +mount[1] left, -mount[1] right, z = 0).
The file is plain UTF-8 (no byte-order mark), at most 64 KB, without DTD, entities or includes. Allowed
attributes: mujoco/model, numeric/name,data, body/name,childclass,pos, joint/name,axis. Masses, inertias,
sizes, friction, gains and every physics parameter come from the catalog and the simulator, never from the file.
"""

import json
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}
FRONT = {"FL": 1.0, "FR": 1.0, "RL": -1.0, "RR": -1.0}
MAX_BYTES = 65536
TOL = 1e-4
MIN_CLEARANCE = 0.06  # body module underside above the ground at the declared stance (m)
STANCE_RANGE = (0.50, 0.90)  # stance_height as a fraction of the shortest leg reach (thigh + shank)
ALLOWED = {"mujoco": {"model"}, "custom": set(), "numeric": {"name", "data"}, "worldbody": set(),
           "body": {"name", "childclass", "pos"}, "joint": {"name", "axis"}}
PARAMS = ("stance_height", "knee_front", "knee_rear")


class DesignError(Exception):
    pass


def load_catalog(path=None):
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "catalog.json")
    with open(path) as f:
        return json.load(f)


def _vec(text, n=3):
    try:
        vals = [float(v) for v in text.split()]
    except (AttributeError, ValueError):
        raise DesignError("not a vector of numbers: %r" % (text,))
    if len(vals) != n or not all(math.isfinite(v) for v in vals):
        raise DesignError("expected %d finite numbers: %r" % (n, text))
    return np.array(vals)


def _parse_xml(data):
    if isinstance(data, str):
        data = data.encode("utf-8")
    if len(data) > MAX_BYTES:
        raise DesignError("robot.xml is larger than %d bytes" % MAX_BYTES)
    if data[:3] == b"\xef\xbb\xbf" or data[:2] in (b"\xff\xfe", b"\xfe\xff") or b"\x00" in data:
        raise DesignError("robot.xml must be plain UTF-8 (no byte-order mark, no NUL bytes)")
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        raise DesignError("robot.xml is not valid UTF-8")
    decl = re.match(r"\s*<\?xml[^>]*?encoding\s*=\s*[\"']([^\"']*)[\"']", text)
    if decl and decl.group(1).lower().replace("_", "-") not in ("utf-8", "utf8"):
        raise DesignError("robot.xml must declare (or default to) UTF-8, not %r" % decl.group(1))
    low = data.lower()
    for bad in (b"<!doctype", b"<!entity", b"<?xml-stylesheet", b"xinclude", b"<include"):
        if bad in low:
            raise DesignError("robot.xml may not contain %r" % bad.decode())
    try:
        root = ET.fromstring(data)
    except Exception as exc:  # noqa: BLE001 (any parser failure is an unbuildable design, never a crash)
        raise DesignError("robot.xml is not well-formed XML: %s" % exc)
    for el in root.iter():
        if not isinstance(el.tag, str) or el.tag not in ALLOWED:
            raise DesignError("element <%s> is not part of the dialect" % (el.tag,))
        extra = set(el.attrib) - ALLOWED[el.tag]
        if extra:
            raise DesignError("<%s> may not carry attribute(s) %s" % (el.tag, sorted(extra)))
        if (el.text or "").strip() or (el.tail or "").strip():
            raise DesignError("text content is not allowed (in or after <%s>)" % el.tag)
    if root.tag != "mujoco":
        raise DesignError("the root element must be <mujoco>")
    return root


def _children(el, tag=None):
    return [c for c in el if tag is None or c.tag == tag]


def parse(data, catalog):
    """robot.xml bytes -> design dict, or DesignError naming the first problem."""
    root = _parse_xml(data)
    tags = [c.tag for c in root]
    if sorted(tags) != ["custom", "worldbody"]:
        raise DesignError("<mujoco> must hold exactly one <custom> and one <worldbody>, found %s" % tags)
    params = {}
    for num in _children(root.find("custom")):
        if num.tag != "numeric":
            raise DesignError("<custom> may hold only <numeric>")
        name = num.get("name")
        if name not in PARAMS or name in params:
            raise DesignError("unknown or repeated numeric %r (allowed: %s)" % (name, ", ".join(PARAMS)))
        params[name] = float(_vec(num.get("data"), 1)[0])
    missing = [p for p in PARAMS if p not in params]
    if missing:
        raise DesignError("missing numeric(s): %s" % ", ".join(missing))
    for k in ("knee_front", "knee_rear"):
        if params[k] not in (-1.0, 1.0):
            raise DesignError("%s must be -1 or 1" % k)
    wb = root.find("worldbody")
    tops = _children(wb)
    if len(tops) != 1 or tops[0].tag != "body" or tops[0].get("name") != "torso":
        raise DesignError("<worldbody> must hold exactly one <body name=\"torso\">")
    torso = tops[0]
    bid = torso.get("childclass")
    if bid not in catalog["bodies"]:
        raise DesignError("torso childclass %r is not a body module of the catalog" % (bid,))
    if torso.get("pos") is not None:
        raise DesignError("the torso carries no pos (the simulator places it)")
    if _children(torso, "joint"):
        raise DesignError("the torso has no joint (the simulator gives it its free joint)")
    body = catalog["bodies"][bid]
    legs = {}
    for el in _children(torso):
        if el.tag != "body":
            raise DesignError("the torso may hold only the four legs' first bodies")
        name = el.get("name") or ""
        leg = name[:2]
        if leg not in LEGS or name not in (leg + "_abd", leg + "_hip") or leg in legs:
            raise DesignError("unexpected or repeated body %r under the torso" % name)
        mount = np.array([FRONT[leg] * body["mount"][0], SIDE[leg] * body["mount"][1], 0.0])
        legs[leg] = _parse_leg(el, leg, mount, catalog)
    if sorted(legs) != sorted(LEGS):
        raise DesignError("the robot needs exactly the four legs %s, found %s" % (list(LEGS), sorted(legs)))
    return {"body": bid, "legs": legs, "params": params}


def _check_pos(el, expect, what):
    got = _vec(el.get("pos") or "", 3)
    if np.max(np.abs(got - expect)) > TOL:
        raise DesignError("%s %r: pos %s, the catalog geometry puts it at %s" % (
            what, el.get("name"), " ".join("%g" % v for v in got), " ".join("%g" % v for v in expect)))


def _single_body_child(el, what):
    kids = _children(el, "body")
    others = [c for c in el if c.tag not in ("body", "joint")]
    if len(kids) != 1 or others:
        raise DesignError("%r must hold exactly one child body (%s)" % (el.get("name"), what))
    return kids[0]


def _joint(el, leg, role, axis):
    js = _children(el, "joint")
    if len(js) != 1 or js[0].get("name") != "%s_%s" % (leg, role):
        raise DesignError("body %r needs exactly one joint named %s_%s" % (el.get("name"), leg, role))
    if np.max(np.abs(_vec(js[0].get("axis") or "", 3) - np.array(axis))) > TOL:
        raise DesignError("joint %s_%s must have axis %s" % (leg, role, " ".join("%g" % a for a in axis)))


def _act(el, catalog):
    aid = el.get("childclass")
    if aid not in catalog["actuators"]:
        raise DesignError("body %r: %r is not an actuator of the catalog" % (el.get("name"), aid))
    return aid


def _segments(el, leg, part, catalog):
    """A chain of 1-2 link segments starting at `el`; returns (ids, last element, total length)."""
    ids, cur, total = [], el, 0.0
    for k in (1, 2):
        if cur.tag != "body" or cur.get("name") != "%s_%s%d" % (leg, part, k):
            if k == 1:
                raise DesignError("expected body %s_%s1, found %r" % (leg, part, cur.get("name")))
            break
        lid = cur.get("childclass")
        if lid not in catalog["links"]:
            raise DesignError("body %r: %r is not a link segment of the catalog" % (cur.get("name"), lid))
        if _children(cur, "joint"):
            raise DesignError("link segment %r has no joint" % cur.get("name"))
        if k == 2:
            _check_pos(cur, np.array([0.0, 0.0, -catalog["links"][ids[-1]]["length"]]), "link segment")
        ids.append(lid)
        total += catalog["links"][lid]["length"]
        nxt = _single_body_child(cur, "the next segment, the knee or the foot")
        last, cur = cur, nxt
    return ids, last, cur, total


def _parse_leg(el, leg, mount, catalog):
    side = SIDE[leg]
    out = {"mount": mount.tolist(), "side": side}
    cur = el
    if cur.get("name") == leg + "_abd":
        out["abd"] = _act(cur, catalog)
        _check_pos(cur, mount, "abduction actuator")
        _joint(cur, leg, "abd", (1, 0, 0))
        hip = _single_body_child(cur, "the hip actuator")
        if hip.get("name") != leg + "_hip":
            raise DesignError("the abduction actuator of %s must hold %s_hip" % (leg, leg))
        _check_pos(hip, np.array([0.0, side * catalog["actuators"][out["abd"]]["length"], 0.0]), "hip actuator")
        cur = hip
    else:
        out["abd"] = None
        _check_pos(cur, mount, "hip actuator")
    out["hip"] = _act(cur, catalog)
    _joint(cur, leg, "hip", (0, 1, 0))
    th = _single_body_child(cur, "the first thigh segment")
    _check_pos(th, np.array([0.0, side * catalog["actuators"][out["hip"]]["length"], 0.0]), "thigh segment")
    out["thigh"], last, knee, l1 = _segments(th, leg, "thigh", catalog)
    if knee.get("name") != leg + "_knee":
        raise DesignError("the thigh of %s must end in body %s_knee" % (leg, leg))
    _check_pos(knee, np.array([0.0, 0.0, -catalog["links"][out["thigh"][-1]]["length"]]), "knee actuator")
    out["knee"] = _act(knee, catalog)
    _joint(knee, leg, "knee", (0, 1, 0))
    sh = _single_body_child(knee, "the first shank segment")
    _check_pos(sh, np.zeros(3), "shank segment")
    out["shank"], last, foot, l2 = _segments(sh, leg, "shank", catalog)
    if foot.get("name") != leg + "_foot":
        raise DesignError("the shank of %s must end in body %s_foot" % (leg, leg))
    fid = foot.get("childclass")
    if fid not in catalog["feet"]:
        raise DesignError("body %r: %r is not a foot of the catalog" % (foot.get("name"), fid))
    if len(foot):
        raise DesignError("the foot %r is a leaf (no children)" % foot.get("name"))
    _check_pos(foot, np.array([0.0, 0.0, -catalog["links"][out["shank"][-1]]["length"]]), "foot")
    out["foot"] = fid
    out["L1"], out["L2"] = l1, l2
    return out


def leg_geometry(design, catalog):
    """Per leg: hip-pitch axis point in the body frame at zero abduction, lateral offset w (from the mount),
    segment lengths, foot radius."""
    acts = catalog["actuators"]
    geo = {}
    for leg, L in design["legs"].items():
        wa = acts[L["abd"]]["length"] if L["abd"] else 0.0
        wh = acts[L["hip"]]["length"]
        geo[leg] = {"mount": np.array(L["mount"]), "side": L["side"], "w": L["side"] * (wa + wh), "has_abd": L["abd"] is not None,
                    "L1": L["L1"], "L2": L["L2"], "r": catalog["feet"][L["foot"]]["radius"]}
    return geo


def parts_list(design):
    parts = [("bodies", design["body"])]
    for L in design["legs"].values():
        if L["abd"]:
            parts.append(("actuators", L["abd"]))
        parts += [("actuators", L["hip"]), ("actuators", L["knee"]), ("feet", L["foot"])]
        parts += [("links", i) for i in L["thigh"] + L["shank"]]
    return parts


def summary(design, catalog):
    parts = parts_list(design)
    price = sum(catalog[g][i]["price"] for g, i in parts)
    mass = sum(catalog[g][i]["mass"] for g, i in parts) + catalog["limits"]["payload_kg"]
    geo = leg_geometry(design, catalog)
    body = catalog["bodies"][design["body"]]
    acts = catalog["actuators"]
    xs, ys = [body["size"][0] / 2], [body["size"][1] / 2]
    for leg, L in design["legs"].items():
        g = geo[leg]
        rmax = max([acts[a]["radius"] for a in (L["abd"], L["hip"], L["knee"]) if a] + [g["r"]])
        xs.append(abs(g["mount"][0]) + rmax)
        ys.append(abs(g["mount"][1] + g["w"]) + rmax)
    reach = min(g["L1"] + g["L2"] for g in geo.values())
    h = design["params"]["stance_height"]
    rfoot = min(g["r"] for g in geo.values())
    return {"price_usd": round(price, 2), "mass_kg": round(mass, 4), "length_m": round(2 * max(xs), 4),
            "width_m": round(2 * max(ys), 4), "reach_m": round(reach, 4), "stance_height_m": h,
            "clearance_m": round(h + rfoot - body["size"][2] / 2, 4),
            "joints": joint_names(design)}


def joint_names(design):
    names = []
    for leg in LEGS:
        if design["legs"][leg]["abd"]:
            names.append(leg + "_abd")
        names += [leg + "_hip", leg + "_knee"]
    return names


def check(data, catalog):
    """(design, summary) for a buildable robot.xml within the limits, else DesignError naming the first problem."""
    design = parse(data, catalog)
    s = summary(design, catalog)
    lim = catalog["limits"]
    if s["price_usd"] > lim["budget_usd"] + 1e-9:
        raise DesignError("price %.2f USD exceeds the budget of %.2f USD" % (s["price_usd"], lim["budget_usd"]))
    if s["mass_kg"] > lim["mass_kg"] + 1e-9:
        raise DesignError("mass %.3f kg (payload included) exceeds the limit of %.3f kg" % (s["mass_kg"], lim["mass_kg"]))
    if s["length_m"] > lim["length_m"] + 1e-9 or s["width_m"] > lim["width_m"] + 1e-9:
        raise DesignError("footprint %.3f x %.3f m exceeds the envelope %.3f x %.3f m" % (
            s["length_m"], s["width_m"], lim["length_m"], lim["width_m"]))
    h, reach = design["params"]["stance_height"], s["reach_m"]
    if not (STANCE_RANGE[0] * reach - 1e-9 <= h <= STANCE_RANGE[1] * reach + 1e-9):
        raise DesignError("stance_height %.4f m outside [%.2f, %.2f] x the shortest leg reach %.4f m" % (
            h, STANCE_RANGE[0], STANCE_RANGE[1], reach))
    if s["clearance_m"] < MIN_CLEARANCE - 1e-9:
        raise DesignError("body clearance %.4f m at the declared stance is below %.3f m" % (s["clearance_m"], MIN_CLEARANCE))
    return design, s
