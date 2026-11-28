"""The serviceable equipment of the data-hall cell (cell internals): the 8-GPU nodes (chassis, the GPU tray on its
rails with its release levers and cover, the ports of every field-replaceable part) and the leaf switches.

A removable part that is in place is drawn on a static "seat" body at its port (parts.add_look); a seat's body moves
out of sight when its part leaves (mech.py hands the part over to a free body). Every port is recorded in
meta["ports"]: its kind, the body it belongs to, its frame in that body, its seat body, what it depends on (an MPO
port needs its transceiver, a plug port its power supply) and the cable it carries. Fixtures (levers, cover)
are bodies with a hinge; their grasp features go in meta["features"].
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import hall as H
from . import meshes as M
from . import parts as P
from . import textures as T
from .scene import LED_RGB, quat_axis, quat_z  # noqa: F401

PI = math.pi
SLOTS = [["SXM5", "SXM7", "SXM8", "SXM6"], ["SXM1", "SXM3", "SXM4", "SXM2"]]   # rows (front, back) x columns, left to right
TRAY_HT = H.GPU_TRAY_H - 0.004
RAIL_H = 0.12                     # the sled's side rails stand just above the heat-sinks (the chassis is the enclosure)
LEVER = {"h": 0.20, "w": 0.012, "t": 0.014, "z0": 0.030, "x": 0.2085, "open_rad": 0.60, "max_rad": 0.90}
COVER_STAY = 1.92                  # the hinged front cover rests 20 degrees past vertical (it cannot fall shut)
COVER_MAX = 1.96
COVER_L = 0.485                    # the hinged cover's length (over the modules); a fixed rear cover is over the connectors


def _rear_x(i, n, width):
    return width / 2 - (i + 0.5) * width / n


def psu_xz(i):
    """Supply bay i's (0-based) face centre: x in the rack frame and z above the floor. Two rows of three in the
    middle of the chassis rear: PSU1-3 on top and PSU4-6 below, each row from the left as seen from the rear (the
    rack's +x)."""
    col, row = i % 3, i // 3
    zb = H.NODE_Z0 + 0.003 + H.PSU_H / 2
    return 0.0745 * (1 - col), zb + (H.PSU_H + 0.003) * (1 - row)


def _port(meta, pid, kind, parent, pos, quat, seat, **kw):
    rec = {"kind": kind, "parent": parent, "pos": [float(v) for v in pos], "quat": [float(v) for v in quat],
           "seat": seat}
    rec.update(kw)
    meta["ports"][pid] = rec
    return rec


def _seat(B, parent_body, name, pos, quat):
    return parent_body.add_body(name=name, pos=[float(v) for v in pos], quat=[float(v) for v in quat])


def _to_local(pts, pos, quat):
    """Points given in the parent frame -> a seat's frame (pos, quat = the seat's pose in the parent)."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, float))
    R = R.reshape(3, 3)
    return [tuple(R.T @ (np.asarray(p, float) - np.asarray(pos, float))) for p in pts]


def _fixture(meta, name, ftype, features, **kw):
    meta["features"][name] = features
    meta["graspable"].append(name)
    meta["fixtures"][name] = dict(type=ftype, **kw)


# ------------------------------------------------------------------------------------------------ the GPU node
def add_node(B, rack, row, idx, meta, *, tray_out=0.0, cover_open=0.0, levers_open=0.0, states=None):
    s, bank = B.s, B.bank
    states = states or {}
    host = "gpn-%s%02d" % (row.lower(), idx)
    rackname = H.rack_name(row, idx)
    grey = bank.textured("powder_grey")
    alu = bank.textured("brushed_alu")
    dark = bank.plain((0.012, 0.012, 0.014), 0.5)
    z0, z1, y0, y1 = H.NODE_Z0, H.NODE_Z1, H.NODE_Y0, H.NODE_Y1
    gz0 = H.GPU_TRAY_Z0
    node = {"host": host, "rack": rack.name, "ports": []}
    # ---- chassis (static, part of the rack): CPU tray, the GPU bay, ears, handles, the front's drives and buttons
    if "node_chassis" not in B.meshes:
        parts = [M.transform(M.box((H.NODE_W / 2, H.NODE_D / 2, H.CPU_TRAY_H / 2)), t=(0, y0 + H.NODE_D / 2, z0 + H.CPU_TRAY_H / 2))]
        for sx in (-1, 1):
            parts.append(M.transform(M.box((0.002, H.NODE_D / 2, H.GPU_TRAY_H / 2)), t=(sx * (H.NODE_W / 2 - 0.002), y0 + H.NODE_D / 2, gz0 + H.GPU_TRAY_H / 2)))
        parts.append(M.transform(M.box((H.NODE_W / 2, H.NODE_D / 2, 0.0015)), t=(0, y0 + H.NODE_D / 2, z1 - 0.0015)))
        parts.append(M.transform(M.box((H.NODE_W / 2, 0.003, H.GPU_TRAY_H / 2)), t=(0, y1 - 0.003, gz0 + H.GPU_TRAY_H / 2)))
        B.mesh("node_chassis", M.merge(*parts))
        ears = [M.transform(M.box((0.0107, 0.0015, H.NODE_H / 2)), t=(sx * 0.2307, y0 - 0.0015, z0 + H.NODE_H / 2)) for sx in (-1, 1)]
        B.mesh("node_ears", M.merge(*ears))
        hd = [M.transform(M.bar_handle(0.11, 0.028, 0.0045), R=np.diag([-1.0, -1.0, 1.0]), t=(sx * 0.2307, y0 - 0.003, z0 + H.NODE_H * 0.55))
              for sx in (-1, 1)]
        B.mesh("node_handles", M.merge(*hd))
        B.mesh("cpu_front", M.panel(H.NODE_W, H.CPU_TRAY_H - 0.001, 0.003, "-y"))
        B.mesh("drive", M.panel(0.0765, 0.0385, 0.010, "-y"))
        B.mesh("rear_io", M.panel(0.430, H.U - 0.003, 0.006, "+y"))
        B.mesh("exhaust", M.quad(0.430, H.GPU_TRAY_H - H.U - 0.006, "+y", 0.0, 0.430, 0.0, H.GPU_TRAY_H - H.U - 0.006))
        B.mesh("label_host", M.panel(0.070, 0.016, 0.0006, "-y"))
        B.mesh("info_tag", M.panel(0.052, 0.018, 0.0008, "-y"))
        B.mesh("button", M.transform(M.lathe([(0.0, 0.0), (0.0065, 0.0), (0.0065, 0.004), (0.0, 0.0045)], 24, cap_top=True),
                                     R=np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)))
        B.mesh("cage_ring", M.box((0.0118, 0.0015, 0.0072)))
    B.vis(rack, "node_chassis", None, grey)
    B.vis(rack, "node_ears", None, grey)
    B.vis(rack, "node_handles", None, alu)
    B.vis(rack, "cpu_front", None, bank.plain((0.045, 0.047, 0.05), 0.5), pos=(0.0, y0 - 0.0015, z0 + H.CPU_TRAY_H / 2))
    rows, cols = H.DRIVE_BAYS
    for r_ in range(rows):
        for c in range(cols):
            bay = r_ * cols + c
            x = -0.205 + 0.0395 + c * 0.080
            z = z0 + 0.0045 + 0.0195 + (rows - 1 - r_) * 0.0405
            B.vis(rack, "drive", None, bank.image("drive_%d" % bay, T.drive_front(bay=bay), 0.5), pos=(x, y0 - 0.008, z))
            B.led_static(rack, (x + 0.030, y0 - 0.0150, z + 0.012), (0.0016, 0.0012, 0.0016), "green")
    cx, zc = 0.163, z0 + H.CPU_TRAY_H * 0.62
    for k, led in enumerate(("pwr", "id")):
        B.vis(rack, "button", None, bank.plain((0.05, 0.05, 0.055), 0.4), pos=(cx + 0.030 * k, y0 - 0.003, zc))
        B.led(rack, "%s_%s" % (host, led), (cx + 0.030 * k, y0 - 0.0035, zc), (0.0072, 0.0004),
              states.get("power" if led == "pwr" else "id", "green" if led == "pwr" else "off"),
              quat=quat_axis((1, 0, 0), PI / 2), shape="cyl")
    B.led(rack, host + "_fault", (cx + 0.052, y0 - 0.002, zc), (0.0025, 0.0006, 0.0025), states.get("fault", "off"))
    sn = "KX8-24-%04d" % (100 + 7 * idx + (0 if row == "A" else 50))
    lab = T.label([host, "KX-8H  SN " + sn], 560, 128)
    B.vis(rack, "label_host", None, bank.image("host_" + host, lab, 0.6), pos=(cx + 0.006, y0 - 0.0016, z0 + 0.016))
    tag = T.label(["ASSET %s-%02d" % (row, idx)], 416, 144, barcode="DC1-%s%02d-%04d" % (row, idx, 4400 + idx))
    # ---- no bezel: the GPU tray's fan wall and its blue release levers are open to the aisle (the status light bar runs
    # across the chassis above the fan wall; the asset tag stands on the left ear)
    B.led(rack, host + "_bar", (0.0, y0 - 0.0035, gz0 + H.GPU_TRAY_H - 0.006), (0.090, 0.0006, 0.0012), states.get("bar", "cyan"))
    if "info_tag_v" not in B.meshes:
        B.mesh("info_tag_v", M.panel(0.018, 0.052, 0.0008, "-y"))
    B.vis(rack, "info_tag_v", None, bank.image("tag_" + host, np.ascontiguousarray(np.rot90(tag)), 0.6),
          pos=(-0.2262, y0 - 0.0040, gz0 + 0.17))
    # ---- the rear: I/O strip, exhaust, the power supplies and their cords, the OSFP ports with their fibres
    B.vis(rack, "rear_io", None, bank.image("rear_io", T.rear_io(1032, 104), 0.5), pos=(0.0, y1 + 0.003, z1 - H.U / 2))
    B.vis(rack, "exhaust", None, bank.textured("honeycomb"), pos=(0.0, y1 + 0.0008, gz0 + (H.GPU_TRAY_H - H.U) / 2 - 0.001))
    cordmat = bank.plain((0.02, 0.02, 0.022), 0.6)
    aqua = bank.plain((0.10, 0.62, 0.70), 0.45)
    for i in range(H.PSUS):
        xp, zp = psu_xz(i)
        feed = "A" if i % 2 == 0 else "B"
        pid = "%s/psu%d" % (host, i + 1)
        ppos, pq = (xp, y1 + 0.003, zp), (1.0, 0.0, 0.0, 0.0)
        seat = _seat(B, rack, "seat_" + pid.replace("/", "_"), ppos, pq)
        P.add_look(B, seat, "psu", idx=i + 1, feed=feed)
        B.led(seat, "%s_psu%d" % (host, i + 1), (0.030, 0.0035, 0.014), (0.0016, 0.0008, 0.0016), states.get("psu%d" % (i + 1), "green"))
        _port(meta, pid, "psu", rack.name, ppos, pq, seat.name, host=host, slot=i + 1, feed=feed)
        # the plug in the PSU's inlet and its cord to the feed's PDU (A on the rack's +x side, B on its -x side)
        lpid = "%s/plug%d" % (host, i + 1)
        lpos = (xp - 0.010, y1 + 0.006, zp)
        lseat = _seat(B, rack, "seat_" + lpid.replace("/", "_"), lpos, pq)
        sx = 1.0 if feed == "A" else -1.0
        outlet = (sx * 0.250, 1.195, 0.62 + 0.05 * i)
        pts = [(lpos[0], lpos[1] + 0.046, zp), (lpos[0], lpos[1] + 0.09, zp - 0.02), (lpos[0] * 0.5 + sx * 0.12, 1.14, zp + 0.04 + 0.03 * i),
               (sx * 0.235, 1.16, 0.55 + 0.05 * i), outlet]
        P.add_look(B, lseat, "plug", extra=[(M.cable(_to_local(pts, lpos, pq), 0.0042, samples=5, segments=8), "black")])
        _port(meta, lpid, "plug", rack.name, lpos, pq, lseat.name, host=host, slot=i + 1, feed=feed, needs="%s/psu%d" % (host, i + 1),
              anchor=[float(v) for v in outlet], cable_len=float(sum(np.linalg.norm(np.subtract(pts[k + 1], pts[k])) for k in range(len(pts) - 1))))
    zio = z1 - H.U / 2 - 0.001
    for p in range(H.OSFP_PORTS):
        u = 0.04 + p * 0.088 + 0.088 * 0.43
        xp = 0.215 - 0.430 * u
        pid = "%s/osfp%d" % (host, p + 1)
        opos, oq = (xp, y1, zio - 0.002), (1.0, 0.0, 0.0, 0.0)
        oseat = _seat(B, rack, "seat_" + pid.replace("/", "_"), opos, oq)
        P.add_look(B, oseat, "osfp_n")
        B.vis(rack, "cage_ring", None, bank.textured("brushed_alu", tint=(0.6, 0.6, 0.62)), pos=(xp, y1 + 0.0035, zio - 0.002))
        B.led(rack, "%s_link%d" % (host, p + 1), (xp - 0.006, y1 + 0.0065, zio + 0.012), (0.0012, 0.0006, 0.0010),
              states.get("link%d" % (p + 1), "green"))
        _port(meta, pid, "osfp_n", rack.name, opos, oq, oseat.name, host=host, slot=p + 1)
        mpid = "%s/mpo%d" % (host, p + 1)
        rec = P.OSFP_RECEPTACLE["osfp_n"]
        mpos = (opos[0] + rec[0], opos[1] + rec[1], opos[2] + rec[2])
        mseat = _seat(B, rack, "seat_" + mpid.replace("/", "_"), mpos, oq)
        pts = [(mpos[0], mpos[1] + 0.050, mpos[2]), (mpos[0], mpos[1] + 0.085, mpos[2] + 0.02), (mpos[0] * 0.6 - 0.09, y1 + 0.11, mpos[2] + 0.08),
               (-0.232, 1.125, mpos[2] + 0.30)]
        top = [pts[-1], (-0.232 + 0.004 * p, 1.12, H.RACK_H - 0.05), (-0.232 + 0.004 * p, 1.10, H.RACK_H + 0.18)]
        P.add_look(B, mseat, "mpo", extra=[(M.cable(_to_local(pts, mpos, oq), 0.0026, samples=6, segments=6), "aqua")])
        B.vis(rack, "fibtop_" + mpid.replace("/", "_"), M.cable(top, 0.0026, samples=4, segments=6), aqua)
        _port(meta, mpid, "mpo", rack.name, mpos, oq, mseat.name, host=host, slot=p + 1, needs=pid,
              anchor=[float(v) for v in pts[-1]],
              cable_len=float(sum(np.linalg.norm(np.subtract(pts[k + 1], pts[k])) for k in range(len(pts) - 1))))
    # ---- the GPU tray on its rails
    tray = rack.add_body(name="tray_" + host, pos=[0.0, y0, gz0])
    B.merge_bodies[tray.name] = tray
    tray.add_joint(name="tray_" + host, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, -1, 0], range=[0.0, H.TRAY_SERVICE_TRAVEL],
                   limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=40.0, frictionloss=12.0, armature=0.5)
    meta["init_q"]["tray_" + host] = tray_out
    _gpu_tray(B, tray, host, meta, states, levers_open)
    B.col(tray, (0.213, 0.37, 0.004), (0.0, 0.43, 0.002), kind={"contype": 1, "conaffinity": 2 | 4, "group": 3, "condim": 3},
          mass=22.0)                                   # the tray's bed: parts rest on it, the robot collides with it
    for sx in (-1, 1):
        B.col(tray, (0.002, 0.37, RAIL_H / 2), (sx * 0.2135, 0.43, RAIL_H / 2), kind="equip", mass=2.0)
    B.col(tray, (0.213, 0.030, TRAY_HT / 2), (0.0, 0.030, TRAY_HT / 2), kind="equip", mass=2.0)
    node["tray"] = tray.name
    meta["nodes"][host] = node
    meta["trays"][host] = {"joint": "tray_" + host, "body": tray.name, "levers": ["lever_%s_%s" % (host, sd) for sd in ("l", "r")],
                           "cover": "cover_" + host, "rack": rackname}
    return node


def _gpu_tray(B, tray, host, meta, states, levers_open):
    bank = B.bank
    Ht = TRAY_HT
    grey = bank.textured("powder_grey")
    if "tray_pan" not in B.meshes:
        B.mesh("tray_pan", M.merge(M.transform(M.box((0.215, 0.37, 0.0015)), t=(0, 0.43, 0.0015)),
                                   *[M.transform(M.box((0.0015, 0.37, RAIL_H / 2)), t=(sx * 0.2135, 0.43, RAIL_H / 2)) for sx in (-1, 1)],
                                   *[M.transform(M.box((0.004, 0.37, 0.003)), t=(sx * 0.2115, 0.43, RAIL_H)) for sx in (-1, 1)],
                                   M.transform(M.box((0.215, 0.004, Ht / 2)), t=(0, 0.796, Ht / 2))))
        B.mesh("fan_cage", M.transform(M.box((0.2135, 0.003, Ht / 2)), t=(0.0, 0.062, Ht / 2)))
        B.mesh("lever", M.transform(M.extrude([(-LEVER["w"] / 2, 0.0), (LEVER["w"] / 2, 0.0), (LEVER["w"] / 2, LEVER["h"]),
                                               (-LEVER["w"] / 2, LEVER["h"] + 0.008)], LEVER["t"], axis="y"), t=(0, -LEVER["t"] / 2, 0)))
        B.mesh("baseboard", M.panel(0.42, 0.70, 0.0024, "+z"))
        B.mesh("nvs_fins", M.fin_stack(0.080, 0.070, 0.050, 22, 0.0007, base=0.004))
        B.mesh("silk", M.panel(0.034, 0.011, 0.0004, "+z"))
        B.mesh("cover", M.panel(0.428, COVER_L, 0.0016, "+z"))
        B.mesh("cover_rear", M.panel(0.428, 0.735 - COVER_L - 0.004, 0.0016, "+z"))
        B.mesh("tray_label", M.panel(0.27, 0.18, 0.0004, "+z"))
        B.mesh("conn", M.box((0.040, 0.014, 0.018)))
        B.mesh("guide_pin", M.lathe([(0.0, 0.0), (0.0022, 0.0), (0.0022, 0.012), (0.0012, 0.015), (0.0, 0.015)], 10, cap_top=True))
    B.vis(tray, "tray_pan", None, grey)
    B.vis(tray, "fan_cage", None, bank.plain((0.03, 0.03, 0.033), 0.5))
    # fan modules: ports on the fan wall (pulled towards the aisle: their frame turned 180 degrees about z)
    for k in range(H.FAN_MODULES):
        x = -0.2135 + 0.0425 + k * 0.0855
        pid = "%s/fan%d" % (host, k + 1)
        fpos, fq = (x, 0.0, Ht / 2), tuple(quat_z(PI))
        seat = _seat(B, tray, "seat_" + pid.replace("/", "_"), fpos, fq)
        P.add_look(B, seat, "fan", idx=k + 1)
        B.led(seat, "%s_fan%d" % (host, k + 1), (-0.030, 0.0012, Ht / 2 - 0.012), (0.0016, 0.0006, 0.0016),
              states.get("fan%d" % (k + 1), "green"))
        _port(meta, pid, "fan", tray.name, fpos, fq, seat.name, host=host, slot=k + 1)
    # release levers: hinged at their bottoms, the tops swing towards the aisle; both open unlatch the tray
    blue = bank.plain((0.06, 0.28, 0.72), 0.45)
    for sd, sx in (("l", -1), ("r", 1)):
        lv = tray.add_body(name="lever_%s_%s" % (host, sd), pos=[sx * LEVER["x"], -0.004, LEVER["z0"]])
        lv.add_joint(name="lever_%s_%s" % (host, sd), type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0],
                     range=[0.0, LEVER["max_rad"]], limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.2, frictionloss=0.05,
                     armature=0.001)
        B.vis(lv, "lever", None, blue)
        B.col(lv, (LEVER["w"] / 2, LEVER["t"] / 2, LEVER["h"] / 2), (0.0, -LEVER["t"] / 2, LEVER["h"] / 2), kind="equip", mass=0.12)
        _fixture(meta, lv.name, "lever", [("cyl", (0.0, -0.010, LEVER["h"] * 0.55), (0.0, -0.010, LEVER["h"] + 0.004), 0.006, 0.012)],
                 host=host, joint=lv.name, open_rad=LEVER["open_rad"])
        meta["init_q"][lv.name] = levers_open
    B.vis(tray, "baseboard", None, bank.image("pcb", T.pcb(), 0.45), pos=(0.0, 0.43, 0.0042))
    alu = bank.textured("brushed_alu")
    gm = H.GPU_MODULE
    for r_, yc in enumerate((0.165, 0.320)):
        for c in range(4):
            xc = -0.159 + c * 0.106
            slot = SLOTS[r_][c]
            pid = "%s/%s" % (host, slot.lower())
            gpos, gq = (xc, yc, 0.0054), (1.0, 0.0, 0.0, 0.0)
            seat = _seat(B, tray, "seat_" + pid.replace("/", "_"), gpos, gq)
            P.add_look(B, seat, "gpu")
            for sy in (-1, 1):                               # the socket's guide pins (stay with the tray)
                B.vis(tray, "guide_pin", None, bank.textured("brushed_alu"), pos=(xc + sy * 0.020, yc + sy * 0.053, 0.0054))
            _port(meta, pid, "gpu", tray.name, gpos, gq, seat.name, host=host, slot=slot)
    if "silk_all" not in B.meshes:                       # the eight slot names silkscreened beside the sockets
        quads = []
        for r_, yc in enumerate((0.165, 0.320)):
            for c in range(4):
                k = r_ * 4 + c
                u0, v0 = (k % 4) / 4.0, 1.0 - (k // 4 + 1) / 2.0
                quads.append(M.transform(M.quad(0.034, 0.011, "+z", u0, u0 + 0.25, v0, v0 + 0.5),
                                         t=(-0.159 + c * 0.106, yc - gm["d"] / 2 - 0.009, 0.0049)))
        B.mesh("silk_all", M.merge(*quads))
    B.vis(tray, "silk_all", None, bank.image("silk_all", T.slot_silk(SLOTS), 0.5))
    for c in range(4):
        B.vis(tray, "nvs_fins", None, alu, pos=(-0.159 + c * 0.106, 0.47, 0.0054))
    for c in range(6):
        B.vis(tray, "conn", None, bank.plain((0.02, 0.02, 0.022), 0.5), pos=(-0.18 + c * 0.072, 0.74, 0.0144))
    # the hinged front cover (over the modules and the NVSwitches): along the tray's -x top edge, lifted from the
    # tray's front by its pull at the front-left corner to a stay 20 degrees past vertical; a fixed rear cover beyond
    hinge = tray.add_body(name="cover_" + host, pos=[-0.214, 0.065, Ht])
    hinge.add_joint(name="cover_" + host, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, -1, 0], range=[0.0, COVER_MAX],
                    limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.5, frictionloss=0.03, armature=0.005)
    grey = bank.textured("powder_grey", tint=(1.15, 1.16, 1.2))
    B.vis(hinge, "cover", None, grey, pos=(0.214, COVER_L / 2, 0.0008))
    B.vis(hinge, "tray_label", None, bank.image("tray_label", T.tray_service_label(), 0.5), pos=(0.214, 0.30, 0.0019))
    rear_l = 0.735 - COVER_L - 0.004                    # the fixed rear cover (screwed down; no service part under it)
    B.vis(tray, "cover_rear", None, grey, pos=(0.0, 0.065 + COVER_L + 0.004 + rear_l / 2, Ht + 0.0008))
    B.col(tray, (0.214, rear_l / 2, 0.002), (0.0, 0.065 + COVER_L + 0.004 + rear_l / 2, Ht + 0.001), kind="equip", mass=0.6)
    if "cover_pull" not in B.meshes:   # the pull at the cover's front edge, near the hinge (lifted from the tray's front)
        B.mesh("cover_pull", M.merge(M.tube([(0.12, 0.02, 0.014), (0.12, 0.14, 0.014)], 0.0055, 12),
                                     M.transform(M.box((0.004, 0.004, 0.006)), t=(0.12, 0.02, 0.008)),
                                     M.transform(M.box((0.004, 0.004, 0.006)), t=(0.12, 0.14, 0.008))))
    B.vis(hinge, "cover_pull", None, bank.plain((0.06, 0.28, 0.72), 0.45))
    B.col(hinge, (0.214, COVER_L / 2, 0.002), (0.214, COVER_L / 2, 0.0010), kind="equip", mass=0.8)
    _fixture(meta, hinge.name, "cover", [("cyl", (0.12, 0.03, 0.014), (0.12, 0.13, 0.014), 0.0055, 0.012)],
             host=host, joint=hinge.name, stay_rad=COVER_STAY)


# ------------------------------------------------------------------------------------------------ leaf switches
def add_leaves(B, rack, row, meta, states=None):
    bank = B.bank
    states = states or {}
    grey = bank.textured("powder_grey")
    if "leaf_body" not in B.meshes:
        B.mesh("leaf_body", M.box((0.220, H.LEAF_DEPTH / 2, H.U / 2 - 0.0008)))
        B.mesh("leaf_ports", M.panel(0.440, H.U - 0.002, 0.004, "+y"))
        B.mesh("leaf_front", M.quad(0.440, H.U - 0.004, "-y", 0.0, 0.44, 0.0, H.U - 0.004))
    aqua = bank.plain((0.10, 0.62, 0.70), 0.45)
    for k, u in enumerate(H.LEAF_U):
        name = H.leaf_name(row, k)
        zc = H.u_z(u) + H.U / 2
        B.vis(rack, "leaf_body", None, grey, pos=(0.0, H.REAR_RAIL_Y - H.LEAF_DEPTH / 2, zc))
        B.vis(rack, "leaf_ports", None, bank.image("ports_" + name, T.switch_ports(1408, 142, name=name), 0.5), pos=(0.0, H.REAR_RAIL_Y + 0.004, zc))
        B.vis(rack, "leaf_front", None, bank.textured("honeycomb"), pos=(0.0, H.REAR_RAIL_Y - H.LEAF_DEPTH - 0.0005, zc))
        B.col(rack, (0.220, H.LEAF_DEPTH / 2, H.U / 2 - 0.001), (0.0, H.REAR_RAIL_Y - H.LEAF_DEPTH / 2, zc), kind="equip")
        meta["leaves"][name] = {"rack": rack.name, "rail": H.rail_of_leaf(row, k), "u": u}
        for port in list(range(1, 9)) + list(range(17, 25)):
            col, rr = (port - 1) // 2, (port - 1) % 2
            uplink = port >= 17                          # to the spines: drawn, never serviced (no port, no seat)
            u_tex = 0.10 + (col + 0.44) * (0.86 / 16)
            xp = 0.220 - 0.440 * u_tex
            zp = zc + (H.U / 2 - 0.002) - (0.08 + 0.46 * rr + 0.19) * (H.U - 0.002)
            pid = "%s/osfp%d" % (name, port)
            opos, oq = (xp, H.REAR_RAIL_Y + 0.004, zp), (1.0, 0.0, 0.0, 0.0)
            if uplink:
                rec = P.OSFP_RECEPTACLE["osfp_s"]
                mpos = (opos[0] + rec[0], opos[1] + rec[1], opos[2] + rec[2])
                side = 1.0 if xp > 0 else -1.0
                pts = [(mpos[0], mpos[1] + 0.050, mpos[2]), (mpos[0], mpos[1] + 0.07, mpos[2] + 0.01),
                       (side * 0.20, 1.14, mpos[2] + 0.06 + 0.004 * col), (side * (0.215 + 0.0015 * col), 1.13, zc + 0.40),
                       (side * (0.215 + 0.0015 * col), 1.11, H.RACK_H + 0.18)]
                B.vis(rack, P.ensure_look(B, "osfp_s"), None, B.bank.atlas("osfp_s", *P._atlas_images(None)), pos=opos)
                B.vis(rack, P.ensure_look(B, "mpo"), None, B.bank.atlas("mpo", *P._atlas_images(None)), pos=mpos)
                B.vis(rack, "upl_%s_%d" % (name, port), M.cable(pts, 0.0024, samples=5, segments=6), aqua)
                B.led(rack, "%s_p%d" % (name, port), (xp + 0.008, H.REAR_RAIL_Y + 0.0065, zp + (0.006 if rr == 0 else -0.006)),
                      (0.0010, 0.0006, 0.0008), states.get("%s_p%d" % (name, port), "green"))
                continue
            oseat = _seat(B, rack, "seat_" + pid.replace("/", "_").replace("-", "_"), opos, oq)
            P.add_look(B, oseat, "osfp_s")
            B.led(rack, "%s_p%d" % (name, port), (xp + 0.008, H.REAR_RAIL_Y + 0.0065, zp + (0.006 if rr == 0 else -0.006)),
                  (0.0010, 0.0006, 0.0008), states.get("%s_p%d" % (name, port), "green"))
            _port(meta, pid, "osfp_s", rack.name, opos, oq, oseat.name, leaf=name, slot=port)
            mpid = "%s/mpo%d" % (name, port)
            rec = P.OSFP_RECEPTACLE["osfp_s"]
            mpos = (opos[0] + rec[0], opos[1] + rec[1], opos[2] + rec[2])
            mseat = _seat(B, rack, "seat_" + mpid.replace("/", "_").replace("-", "_"), mpos, oq)
            side = 1.0 if xp > 0 else -1.0
            pts = [(mpos[0], mpos[1] + 0.050, mpos[2]), (mpos[0], mpos[1] + 0.07, mpos[2] + 0.01), (side * 0.20, 1.14, mpos[2] + 0.06 + 0.004 * col)]
            top = [pts[-1], (side * (0.215 + 0.0015 * col), 1.13, zc + 0.40), (side * (0.215 + 0.0015 * col), 1.11, H.RACK_H + 0.18)]
            P.add_look(B, mseat, "mpo", extra=[(M.cable(_to_local(pts, mpos, oq), 0.0024, samples=5, segments=6), "aqua")])
            B.vis(rack, "lfibtop_" + mpid.replace("/", "_").replace("-", "_"), M.cable(top, 0.0024, samples=4, segments=6), aqua)
            _port(meta, mpid, "mpo", rack.name, mpos, oq, mseat.name, leaf=name, slot=port, needs=pid,
                  anchor=[float(v) for v in pts[-1]],
                  cable_len=float(sum(np.linalg.norm(np.subtract(pts[i + 1], pts[i])) for i in range(len(pts) - 1))))
