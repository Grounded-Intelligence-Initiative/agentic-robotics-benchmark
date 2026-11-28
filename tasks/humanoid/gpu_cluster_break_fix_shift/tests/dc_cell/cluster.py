"""The data hall's cluster, its site services and the shift's scenario (cell internals: the grader's side).

The MuJoCo cell (cell.py, mech.py) simulates the robot and the parts; this module is everything behind the parts: the
eight 8-GPU nodes (power supplies and their 4+2 redundancy, power states, the GPU tray's interlocks, GPU modules and
their sockets, fans and the GPU temperature), the rail-optimised optical fabric (transceivers, MPO-12 fibres with
Type-B polarity, the end faces' contamination and the link margin, symbol errors, flaps), the training job and its
scheduler, and the site services a technician uses (NOC tickets, DCIM, BMC, switched PDUs, the fabric manager, the
scheduler, DCGM diagnostics). It draws one shift's faults from the seed and reports at the end what the grader needs.
Pure Python + numpy + Pillow; deterministic from the seed (no wall clock anywhere).

The cell drives it::

    cl = Cluster(seed, ports=meta["ports"])
    cl.on_event(ev, t)                  every physical event, in order
    cl.tick(t, phys)                    every 1.0 s of sim time (phys: PhysicalState)
    cl.api(method, params, t)           the site services; cl.api_cost_s(method) = the sim seconds a call takes
    cl.leds(t)                          every LED's state now
    cl.scope_image(face, lateral_m, angle_rad, t)   the fibre scope's picture (its record: cl.last_capture)
    cl.on_clean(face, t)                one cleaner click
    cl.final_report(t_end, parts_in_tote=[...])     the grader's record; cl.truth() the hidden scenario

Model choices where the specification leaves room (numbers' sources: research_brief.md):

* Thermal: T_target = 35 + 50 * load / airflow**1.5 (tau 40 s). The training job runs the GPUs at load 0.9 (DCGM r2
  0.8, r3 1.0, idle-on 0.1). Above 87 C the GPUs throttle: load is capped to hold 87 C but cannot go below 0.6 (the
  clock floor); released below 85 C; 95 C shuts the node down. So one failed fan module (airflow 0.8) throttles a job
  node to ~83 % pace, an empty fan slot (airflow 0.6) overheats it to shutdown in ~40 s, the degraded-rotor
  distractor (airflow 0.95) stays below the throttle point.
* Optics: margin = Rx - (-6.0 dBm). Symbol errors per receiver lane: 0 for margin >= 1 dB, else
  2 * (10**(1 - m) - 1) per minute (m clipped at 0): ~1.6/min at 0.75 dB, ~4/min at 0.5 dB, 18/min at 0 dB.
  -2 <= margin < 0: the link flaps (down 2 s every 16-24 s); below -2 dB it is down. raw BER (informational) =
  10**(-12 - 2.2 * margin) clipped to [1e-15, 1e-3]; effective BER = symbol errors / bits carried (400 Gb/s).
  A ticketed link starts with its worst lane at 0.25-0.75 dB (symbol errors, still up, node still in the job);
  the aged laser's lane therefore emits ~-3.5 dBm (not -4.5 dBm, which with the 2.0 dB budget would flap and crash
  the job at t = 0).
* GPU seating: after all four screws were run out, the first tighten-to-clutch of each screw (the snug pass) must
  start with a diagonal pair (1-4 or 2-3, the vendor's cross pattern 1-4-2-3) or the module stays seated_loose.
* The rack's out-of-band management switch is fed from outlet A6 only: switching it off cuts the BMC's network.
"""

from __future__ import annotations

import datetime as _dt
import math
import zlib
from dataclasses import dataclass, field

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from . import hall as H

# ================================================================================================ the site
HOSTS = tuple("gpn-%s%02d" % (r, i) for r in ("a", "b") for i in (2, 3, 4, 5))
NODE_ORDER = {h: n + 1 for n, h in enumerate(HOSTS)}          # a02=1 .. b05=8: the leaf port of each node
LEAVES = tuple(H.leaf_name(row, k) for row in H.ROWS for k in range(4))
LEAF_RAIL = {H.leaf_name(row, k): H.rail_of_leaf(row, k) for row in H.ROWS for k in range(4)}
RAIL_LEAF = {r: name for name, r in LEAF_RAIL.items()}
LEAF_UNIT = {H.leaf_name(row, k): H.LEAF_U[k] for row in H.ROWS for k in range(4)}
GPU_SLOT = ("sxm5", "sxm7", "sxm8", "sxm6", "sxm1", "sxm3", "sxm4", "sxm2")   # the vendor's map: GPU index -> slot
SLOT_GPU = {s: i for i, s in enumerate(GPU_SLOT)}
PCI_BUS = ("18", "2a", "3a", "5d", "9a", "ab", "ba", "db")
SLOTS = tuple("sxm%d" % i for i in range(1, 9))
SHIFT_START = _dt.datetime(2026, 11, 27, 1, 30, 0)          # sim t = 0 (UTC)
JOB_ID, JOB_NAME = 48120, "kestrel-pretrain"
GPU_RACKS = tuple(h[4].upper() + h[5:7] for h in HOSTS)
NET_RACKS = ("A01", "B01")

MODELS = {"psu": "KX-PS3300", "fan": "KX-FAN80D", "osfp_n": "OSFP-400G-SR4", "osfp_s": "OSFP-400G-SR4-F",
          "gpu": "SXM-H8-80G", "mpo": "MPO12-APC-OM4", "plug": "C20-C19-16A"}
_DIG = "0123456789"
_ALNUM = "ABCDEFGHJKLMNPRSTUVWXYZ0123456789"
SERIALS = {"psu": ("KX8-PSU-", 6, _DIG), "fan": ("FAN-", 6, _DIG), "gpu": ("SXM-", 8, _DIG),
           "osfp_n": ("OSFP-", 8, _ALNUM), "osfp_s": ("OSFP-", 8, _ALNUM), "mpo": ("CBL-", 5, _DIG),
           "plug": ("CORD-", 5, _DIG)}
DEFAULT_SPARES = ("spare:px_gpu_0", "spare:px_osfp_n_0", "spare:px_osfp_n_1", "spare:px_osfp_s_0", "spare:px_osfp_s_1",
                  "spare:px_psu_0", "spare:px_fan_0", "spare:px_fan_1")                           # scene.SPARES: the parts on the cart

ROOT_CAUSE_CODES = ("gpu_transient", "gpu_unseated", "gpu_module_fault", "gpu_baseboard", "link_dirty_connector",
                    "link_dirty_receptacle", "link_damaged_fiber", "link_transceiver", "psu_failed",
                    "psu_cord_unseated", "fan_failed")
CAUSE_CODE = {"transient": "gpu_transient", "unseated": "gpu_unseated", "module_dead": "gpu_module_fault",
              "hbm_bad": "gpu_module_fault", "baseboard": "gpu_baseboard",
              "dirty_node_connector": "link_dirty_connector", "dirty_leaf_connector": "link_dirty_connector",
              "dirty_node_receptacle": "link_dirty_receptacle", "dirty_leaf_receptacle": "link_dirty_receptacle",
              "scratched_cable_end": "link_damaged_fiber", "aged_tx_node": "link_transceiver",
              "aged_tx_leaf": "link_transceiver", "psu_failed": "psu_failed", "fan_failed": "fan_failed"}
ESCALATE_TO = {"baseboard": "vendor", "scratched_cable_end": "cabling"}
SOP = {"gpu": "SOP-GPU-01", "link": "SOP-NET-01", "psu": "SOP-PWR-01", "fan": "SOP-THM-01"}

GPU_CAUSES = (("transient", 0.25), ("unseated", 0.30), ("module_dead", 0.20), ("hbm_bad", 0.15), ("baseboard", 0.10))
LINK_CAUSES = (("dirty_node_connector", 0.15), ("dirty_leaf_connector", 0.15), ("dirty_node_receptacle", 0.10),
               ("dirty_leaf_receptacle", 0.10), ("scratched_cable_end", 0.15), ("aged_tx_node", 0.175),
               ("aged_tx_leaf", 0.175))
HOTSWAP_CAUSES = (("psu_failed", 0.5), ("fan_failed", 0.5))
MID_KINDS = (("fan_failed", 0.5), ("link", 0.5))

# ---- power, boot, job
ON_STATES = ("booting", "on", "shutting_down")
BOOT_S, SHUTDOWN_S, CYCLE_OFF_S, COLD_S = 300.0, 90.0, 10.0, 30.0
NODE_W = {"standby": 50.0, "idle": 1000.0, "full": 10200.0}
CHECKPOINT_S, RESTART_S = 600.0, 180.0
JOB_LOAD, IDLE_LOAD = 0.9, 0.1
DCGM_S = {1: 3.0, 2: 600.0, 3: 1800.0}
DCGM_LOAD = {1: 0.1, 2: 0.8, 3: 1.0}
LINK_ERR_SLOWDOWN = 0.9
# ---- thermal
T_BASE, T_RISE, T_TAU = 35.0, 50.0, 40.0
T_THROTTLE, T_RELEASE, T_SHUTDOWN = 87.0, 85.0, 95.0
THROTTLE_FLOOR, BRAKE_FACTOR = 0.6, 0.6
FAN_RPM, HOTSWAP_LIMIT_S = 12000.0, 30.0
GPU_T_OFFSET = {"sxm1": 0.0, "sxm2": 0.6, "sxm3": 1.1, "sxm4": 0.3, "sxm5": 4.2, "sxm6": 3.4, "sxm7": 4.8, "sxm8": 3.9}
# ---- optics
LINK_LOSS_DB, RX_SENS_DBM, NO_LIGHT_DBM = 2.0, -6.0, -40.0
BIAS_MA, BIAS_AGED_MA, BIAS_WARN_MA = 7.0, 11.5, 10.0
FLAP_DOWN_S, FLAP_GAP = 2.0, (16.0, 24.0)
LINE_BPS = 4e11
PARTICLE_CORE_DB, FILM_DB, SCRATCH_CORE_DB = (0.6, 1.5), (0.4, 1.2), (2.0, 5.0)
FILM_FULL_DB = 1.2                                            # a film's loss at strength 1.0
CLEAN_P, TRANSFER_P, CLEANER_LITTER_P = 0.8, 0.35, 0.03
CORE_R_UM, CLAD_R_UM, PITCH_UM = 25.0, 62.5, 250.0
USED_FIBRES = (1, 2, 3, 4, 9, 10, 11, 12)
# ---- the scope picture
SCOPE_W, SCOPE_H = 640, 480
SCOPE_UM_PX = 5.0                                             # 3.2 mm across the 640 px width
SCOPE_MARGIN = 344                                            # canvas margin for the off-centre shift (px)
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
# ---- services
API_WRITE = {"noc.update", "noc.escalate", "noc.create", "bmc.power", "bmc.identify", "pdu.set", "fabric.clear",
             "fabric.locate", "sched.drain", "sched.resume"}
API_START = {"dcgm.run", "fabric.soak"}


# ================================================================================================ helpers
def _stable_seed(*keys):
    """A 64-bit seed from any keys, stable across processes (Python's hash() is salted per process)."""
    s = "|".join(str(k) for k in keys).encode()
    return (zlib.crc32(s) << 32) ^ zlib.adler32(s)


def _label(t):
    """The shift's wall-clock label of sim time t (the shift starts at 01:30Z)."""
    return (SHIFT_START + _dt.timedelta(seconds=float(t))).strftime("%Y-%m-%dT%H:%M:%SZ")


def _r(x, n=2):
    return None if x is None else round(float(x), n)


def host_rack(host):
    return host[4].upper() + host[5:7]


def leaf_rack(leaf):
    return leaf[5].upper() + leaf[6:8]


def host_location(host):
    r = host_rack(host)
    return "row %s, rack %s, U%d-U%d" % (r[0], r, H.NODE_U[0], H.NODE_U[1])


def leaf_location(leaf):
    r = leaf_rack(leaf)
    return "row %s, rack %s, U%d" % (r[0], r, LEAF_UNIT[leaf])


def pci_addr(i):
    return "0000:%s:00.0" % PCI_BUS[i]


def link_ports(host, k):
    """The four ports of link (host, k): node OSFP, node MPO, leaf MPO, leaf OSFP (rail k, leaf port = node order)."""
    leaf, p = RAIL_LEAF[k], NODE_ORDER[host]
    return "%s/osfp%d" % (host, k + 1), "%s/mpo%d" % (host, k + 1), "%s/mpo%d" % (leaf, p), "%s/osfp%d" % (leaf, p)


def port_link(port):
    """The planned link (host, k) a node or leaf OSFP / MPO port belongs to, else None."""
    dev, _, name = port.partition("/")
    for pre in ("osfp", "mpo"):
        if name.startswith(pre) and name[len(pre):].isdigit():
            n = int(name[len(pre):])
            if dev in NODE_ORDER and 1 <= n <= 8:
                return dev, n - 1
            if dev in LEAF_RAIL and 1 <= n <= 8:
                return HOSTS[n - 1], LEAF_RAIL[dev]
    return None


def err_rate(m):
    """Post-FEC symbol errors per second on one receiver lane with optical margin m (dB): the documented model."""
    if m >= 1.0:
        return 0.0
    return 2.0 * (10.0 ** (1.0 - max(0.0, m)) - 1.0) / 60.0


def raw_ber(m):
    return float(min(1e-3, max(1e-15, 10.0 ** (-12.0 - 2.2 * m))))


def _rx_lane_of(end, pos):
    """Which receiver carries fibre position `pos` of a face at `end` ("node" | "leaf"): (receiving end, lane).
    The SR4 MDI: positions 1-4 carry this end's Tx lanes 0-3, positions 12..9 its Rx lanes 0-3. A Type-B trunk takes
    position p to 13 - p at the far end, so Tx lane n arrives at the far end's Rx lane n."""
    if pos <= 4:
        return ("leaf" if end == "node" else "node"), pos - 1
    return end, 12 - pos


def _font(size):
    try:
        return ImageFont.truetype(FONT_DIR + "/DejaVuSans.ttf", int(size))
    except OSError:
        return ImageFont.load_default()


class BadParams(Exception):
    pass


# ================================================================================================ the cell's picture
@dataclass
class PhysicalState:
    """What the cell tells the cluster every tick (dicts keyed by host or port id)."""
    tray_pos: dict = field(default_factory=dict)          # host -> m out of the rack (0 = home)
    tray_latched: dict = field(default_factory=dict)      # host -> bool
    cover_open: dict = field(default_factory=dict)        # host -> rad
    bezel_open: dict = field(default_factory=dict)        # host -> rad
    occupied: dict = field(default_factory=dict)          # port -> bool
    instance: dict = field(default_factory=dict)          # port -> part id or None
    screws_fastened: dict = field(default_factory=dict)   # GPU slot port -> bool
    screws_loose: dict = field(default_factory=dict)      # GPU slot port -> bool
    gpu_damaged: dict = field(default_factory=dict)       # GPU slot port -> bool
    port_broken: dict = field(default_factory=dict)       # port -> bool
    robot_xy: tuple | None = None
    doors: dict = field(default_factory=dict)             # optional: door name -> opening (rad or m)


def default_ports():
    """The port table as equipment.py records it (ids, kinds, hosts / leaves, slots, feeds), without the geometry."""
    ports = {}
    for h in HOSTS:
        for k in range(1, 6):
            ports["%s/fan%d" % (h, k)] = {"kind": "fan", "host": h, "slot": k}
        for k in range(1, 7):
            feed = "A" if k % 2 else "B"
            ports["%s/psu%d" % (h, k)] = {"kind": "psu", "host": h, "slot": k, "feed": feed}
            ports["%s/plug%d" % (h, k)] = {"kind": "plug", "host": h, "slot": k, "feed": feed,
                                           "needs": "%s/psu%d" % (h, k)}
        for k in range(1, 9):
            ports["%s/osfp%d" % (h, k)] = {"kind": "osfp_n", "host": h, "slot": k}
            ports["%s/mpo%d" % (h, k)] = {"kind": "mpo", "host": h, "slot": k, "needs": "%s/osfp%d" % (h, k)}
        for s in SLOTS:
            ports["%s/%s" % (h, s)] = {"kind": "gpu", "host": h, "slot": s.upper()}
    for leaf in LEAVES:
        for p in range(1, 9):
            ports["%s/osfp%d" % (leaf, p)] = {"kind": "osfp_s", "leaf": leaf, "slot": p}
            ports["%s/mpo%d" % (leaf, p)] = {"kind": "mpo", "leaf": leaf, "slot": p, "needs": "%s/osfp%d" % (leaf, p)}
    return ports


# ================================================================================================ parts and faces
class Part:
    """A field-replaceable part (or one end of a fibre cable): identity, model, serial, health."""

    def __init__(self, pid, kind, serial, spare=False):
        self.id, self.kind, self.serial, self.spare = pid, kind, serial, spare
        self.model = MODELS[kind]
        self.health = "ok"            # psu ok|failed; fan ok|failed|degraded; gpu ok|dead|hbm_bad|connector_damaged
        self.prior_health = None      # a GPU's health before a hot tray pull damaged it
        self.wedged = False           # gpu: fell off the bus; clears once the node's AC is removed >= 30 s
        self.rotors = [1.0, 1.0]      # fan: each rotor's speed as a fraction of nominal (0 = stopped)
        self.tx = [0.0] * 4           # transceiver: per-lane Tx power (dBm)
        self.bias = [BIAS_MA] * 4     # transceiver: per-lane laser bias (mA)
        self.temp = 45.0              # transceiver case temperature (C)
        self.home = None              # the port it sat in at the shift's start
        self.last_port = None         # the last port it sat in
        self.touched = False          # took part in a physical event
        self.cable = None             # mpo end: its cable's id

    def faulty(self):
        """An intrinsic hardware fault (dirt on a face is not one; a scratched connector face is: see Face)."""
        if self.kind == "gpu":
            return self.health != "ok"
        if self.kind in ("psu", "fan"):
            return self.health != "ok"
        if self.kind in ("osfp_n", "osfp_s"):
            return any(b > BIAS_WARN_MA for b in self.bias)
        return False


class Face:
    """One optical end face: an MPO connector's ("mpo:<instance>") or a transceiver receptacle's ("rx:<instance>").
    Twelve fibre positions in a row (pitch 250 um, cladding 125 um, core 50 um); defects in micrometres relative to
    their fibre's core centre: particle (removable), film (each click halves it), scratch and pit (permanent)."""

    def __init__(self, fid, kind, instance):
        self.id, self.kind, self.instance = fid, kind, instance
        self.defects = []
        self.damaged = False          # the receptacle is broken (no light passes)

    def loss(self, pos):
        s = 0.0
        for d in self.defects:
            if d["fibre"] == pos:
                s += FILM_FULL_DB * d["strength"] if d["type"] == "film" else d["loss"]
        return s

    def clean(self, rng):
        """One click of the one-click cleaner (all twelve fibres at once)."""
        keep = []
        for d in self.defects:
            if d["type"] == "particle":
                if rng.random() < CLEAN_P:
                    continue
            elif d["type"] == "film":
                d["strength"] *= 0.5
                if d["strength"] < 0.1:
                    continue
            keep.append(d)
        self.defects = keep
        if rng.random() < CLEANER_LITTER_P:
            self.defects.append(_particle(rng, int(rng.integers(1, 13)), "cladding", d=float(rng.uniform(2.0, 6.0)),
                                          origin="cleaner"))

    def has_core_scratch(self):
        return any(d["type"] == "scratch" and d["zone"] == "core" for d in self.defects)

    def summary(self):
        out = {"particles_core": 0, "particles_cladding": 0, "films": 0, "scratches_core": 0, "scratches_cladding": 0,
               "pits": 0}
        for d in self.defects:
            if d["type"] == "particle":
                out["particles_" + d["zone"]] += 1
            elif d["type"] == "film":
                out["films"] += 1
            elif d["type"] == "scratch":
                out["scratches_" + d["zone"]] += 1
            else:
                out["pits"] += 1
        out["core_loss_db"] = {p: _r(self.loss(p), 3) for p in range(1, 13) if self.loss(p) > 0.0}
        out["damaged"] = self.damaged
        return out


def _particle(rng, fibre, zone, d=None, loss=None, origin="background"):
    if zone == "core":
        if loss is None:
            loss = float(rng.uniform(*PARTICLE_CORE_DB))
        if d is None:
            d = 4.0 + 11.0 * (loss - PARTICLE_CORE_DB[0]) / (PARTICLE_CORE_DB[1] - PARTICLE_CORE_DB[0])
        r = max(0.0, CORE_R_UM - d / 2 - 0.5) * math.sqrt(float(rng.random()))
    else:
        if d is None:
            d = float(rng.uniform(2.0, 12.0))
        r = float(rng.uniform(CORE_R_UM + d / 2 + 1.0, CLAD_R_UM - d / 2 - 1.0))
        if loss is None:
            loss = 0.2 * min(1.0, max(0.0, (d - 2.0) / 13.0))
    a = float(rng.uniform(0.0, 2 * math.pi))
    return {"type": "particle", "fibre": int(fibre), "x": r * math.cos(a), "y": r * math.sin(a), "d": float(d),
            "loss": float(loss), "zone": zone, "origin": origin}


def _film(rng, fibre, strength, origin="scenario"):
    lobes = [(float(rng.uniform(-9, 9)), float(rng.uniform(-9, 9)), float(rng.uniform(7, 14)),
              float(rng.uniform(0.5, 1.0))) for _ in range(4)]
    return {"type": "film", "fibre": int(fibre), "x": float(rng.uniform(-6, 6)), "y": float(rng.uniform(-6, 6)),
            "strength": float(strength), "lobes": lobes, "zone": "core", "origin": origin}


def _scratch(rng, fibre, loss, core=True, origin="scenario"):
    off = float(rng.uniform(-14, 14)) if core else float(rng.choice([-1.0, 1.0]) * rng.uniform(32, 54))
    return {"type": "scratch", "fibre": int(fibre), "angle": float(rng.uniform(0, math.pi)), "offset": off,
            "width": 3.0, "loss": float(loss), "zone": "core" if core else "cladding", "origin": origin}


def _pit(rng, fibre):
    d = float(rng.uniform(2.0, 5.0))
    r = float(rng.uniform(CORE_R_UM + d / 2 + 1.0, CLAD_R_UM - d / 2 - 1.0))
    a = float(rng.uniform(0.0, 2 * math.pi))
    return {"type": "pit", "fibre": int(fibre), "x": r * math.cos(a), "y": r * math.sin(a), "d": d,
            "loss": float(rng.uniform(0.0, 0.05)), "zone": "cladding", "origin": "background"}


def _split_loss(rng, E, kinds):
    """Losses (dB) for defects of `kinds` ("particle" / "film") summing to E, each within its range; None if E is
    out of reach."""
    lo = np.array([PARTICLE_CORE_DB[0] if k == "particle" else FILM_DB[0] for k in kinds])
    hi = np.array([PARTICLE_CORE_DB[1] if k == "particle" else FILM_DB[1] for k in kinds])
    if not lo.sum() - 1e-9 <= E <= hi.sum() + 1e-9:
        return None
    for _ in range(300):
        x = rng.uniform(lo, hi)
        x = x * (E / x.sum())
        if np.all(x >= lo - 1e-9) and np.all(x <= hi + 1e-9):
            return [float(v) for v in x]
    f = (E - lo.sum()) / max(1e-9, hi.sum() - lo.sum())
    return [float(v) for v in lo + f * (hi - lo)]


class Cable:
    """One MPO-12 trunk: node end <-> leaf end (Type-B: fibre f at one end is fibre 13-f at the other)."""

    def __init__(self, cid, serial, host, k, node_end, leaf_end, length_m):
        self.id, self.serial, self.host, self.k = cid, serial, host, k
        self.node_end, self.leaf_end = node_end, leaf_end
        self.leaf, self.leaf_port = RAIL_LEAF[k], NODE_ORDER[host]
        self.length_m = length_m
        self.label = "%s-%02d:%d/%s-%02d:%d" % (host_rack(host), H.NODE_U[0], k + 1, leaf_rack(self.leaf),
                                                LEAF_UNIT[self.leaf], self.leaf_port)

    def other(self, end):
        return self.leaf_end if end == self.node_end else self.node_end


# ================================================================================================ nodes, links, job
class Node:
    """One 8-GPU node's live state (the parts themselves live in the cluster's part table)."""

    def __init__(self, host):
        self.host = host
        self.rack = host_rack(host)
        self.power = "on"             # off (no AC) | standby | booting | on | shutting_down
        self.timer = None             # (time, action): "on" boot done | "standby" shutdown done | "boot" start boot
        self.after_shutdown = None    # "boot" while a power cycle's graceful off runs
        self.brake = False
        self.temp = 40.0              # the hottest GPU (C)
        self.throttled = False
        self.speed = 1.0              # its pace in the job (throttle, power brake, link errors)
        self.identify = "off"
        self.sel = []
        self.ac_off_since = None
        self.post_error = False
        self.down_cause = None
        self.n_good = 6
        self.ac_any = True
        self.psu_logged = {}
        self.fan_logged = {}
        self.tray_out = False
        self.empty_since = {}
        self.overtime_logged = set()
        self.dcgm = None
        self.inlet = 23.5
        self.boots = 0
        self.link_err_recent = False
        self.load_eff = 0.0           # the GPUs' load after the power brake and thermal capping
        self.red_logged = None


class LinkState:
    """The connection that starts at node port (host, k): its band, peer and flap schedule."""

    def __init__(self, host, k):
        self.host, self.k = host, k
        self.band = "up"              # up | flapping | down
        self.peer = None              # (leaf, port) it is cabled to now, if complete
        self.was_up = True
        self.last_peer = (RAIL_LEAF[k], NODE_ORDER[host])
        self.next_flap = None
        self.down_until = -1.0
        self.n_flap = 0


class Counters:
    """One fabric port's counters (visible ones clear; tot_* never do)."""

    def __init__(self):
        self.symbol_errors = 0
        self.link_down_count = 0
        self.up_s = 0.0
        self.tot_errors = 0
        self.tot_downs = 0
        self.last_error_t = -1e9

    def clear(self):
        self.symbol_errors, self.link_down_count, self.up_s = 0, 0, 0.0


class Ticket:
    def __init__(self, tid, kind, host, opened, visible_at, priority, summary, description, fields, truth):
        self.id, self.kind, self.host = tid, kind, host
        self.opened, self.visible_at, self.priority = opened, visible_at, priority
        self.summary, self.description, self.fields, self.truth = summary, description, fields, truth
        self.status = "new"
        self.updates = []
        self.escalations = []

    def public(self, full=False):
        out = {"id": self.id, "status": self.status, "priority": self.priority, "opened": _label(self.opened),
               "host": self.host, "summary": self.summary}
        if full:
            out.update({"description": self.description, "fields": dict(self.fields),
                        "updates": [dict(u) for u in self.updates], "escalations": [dict(e) for e in self.escalations]})
        return out


class Scheduler:
    def __init__(self):
        self.state = {h: "alloc" for h in HOSTS}
        self.reason = {h: "" for h in HOSTS}
        self.since = {h: 0.0 for h in HOSTS}
        self.history = {h: [(0.0, "alloc", "")] for h in HOSTS}

    def set(self, h, state, t, reason=""):
        if self.state[h] != state or self.reason[h] != reason:
            if self.state[h] != state:
                self.since[h] = t
            self.state[h], self.reason[h] = state, reason
            self.history[h].append((round(t, 2), state, reason))


# ================================================================================================ the cluster
class Cluster:
    """The cluster, site services and scenario of one shift (see the module docstring)."""

    def __init__(self, seed, ports=None, config=None):
        self.seed = int(seed)
        self.config = dict(config or {})
        self.ports = self._check_ports(default_ports() if ports is None else ports)
        kids = np.random.SeedSequence([self.seed & 0xFFFFFFFF, (self.seed >> 32) & 0xFFFFFFFF, 0xDC]).spawn(10)
        (self.rng_serial, self.rng_optics, self.rng_faces, self.rng_scn, self.rng_tix, self.rng_err, self.rng_clean,
         self.rng_mate, self.rng_hist, self.rng_spare) = [np.random.default_rng(k) for k in kids]
        self.t = 0.0                  # the latest time processed
        self.t_tick = 0.0             # the latest time the continuous processes were integrated to
        self.version = 0              # bumped on every physical change (invalidates the optics cache)
        self._opt, self._opt_version = {}, -1
        self.occ = {p: None for p in self.ports}
        self.latched = {p: False for p in self.ports}
        self.broken = {p: False for p in self.ports}
        self.parts, self.faces, self.cables, self.cable_of = {}, {}, {}, {}
        self.port_hist = {p: [] for p in self.ports}
        self.plug_bad = {p: False for p in self.ports if self.ports[p]["kind"] == "plug"}
        self.seat_q = {p: "good" for p in self.ports if self.ports[p]["kind"] == "gpu"}
        self.socket_bad = {p: False for p in self.seat_q}
        self.slot_fault = {p: False for p in self.seat_q}
        self.screw_out = {p: set() for p in self.seat_q}
        self.snug = {p: [] for p in self.seat_q}
        self.armed = {p: False for p in self.seat_q}
        self.tray_latched = {h: True for h in HOSTS}
        self.nodes = {h: Node(h) for h in HOSTS}
        self.links = {(h, k): LinkState(h, k) for h in HOSTS for k in range(8)}
        self.ctr = {}
        for h in HOSTS:
            for k in range(8):
                self.ctr[(h, k + 1)] = Counters()
        for leaf in LEAVES:
            for p in range(1, 9):
                self.ctr[(leaf, p)] = Counters()
        self._leaf_peer = {}
        self.locate = {}
        self.pdu = {r: {f + str(n): "on" for f in "AB" for n in range(1, 7)} for r in GPU_RACKS + NET_RACKS}
        self.sched = Scheduler()
        self.job = {"id": JOB_ID, "name": JOB_NAME, "state": "running", "restart_at": None, "last_cp": 0.0,
                    "running_since": -36000.0, "since_cp_work": 0.0, "saved_work": 0.0, "work_total": 0.0,
                    "crashes": 0, "lost_gpu_s": 0.0}
        self.next_cp = CHECKPOINT_S
        self.pending = []             # [(time, kind, data)]: mid-shift fault onsets and ticket openings
        self.incidents, self.interventions, self.cleanings, self.captures, self.resumes = [], [], [], [], []
        self.dcgm_runs, self.dcgm_active, self.soaks, self.soak_active = {}, [], {}, []
        self.tickets = {}
        self.api_log = []
        self.unknown = []
        self.parts_in_tote = []
        self.phys = None
        self.last_capture = None
        self._last_click = None
        self._two_trays = False
        self._t_id = 0
        self._build_parts()
        self._build_optics()
        self._build_faces()
        self.scn = self._draw_scenario()
        self._apply_scenario()
        self._init_history()
        self._init_thermal()
        self._evaluate(0.0)
        self._make_tickets()
        for nd in self.nodes.values():               # statuses as the BMC already knows them (history written)
            nd.psu_logged = {s: self._psu_status(nd.host, s) for s in range(1, 7)}
            nd.fan_logged = {m: self._fan_status(nd.host, m) for m in range(1, 6)}
            nd.red_logged = self._redundancy(nd.host)[0]

    # ============================================================================================ construction
    def _check_ports(self, ports):
        exp = default_ports()
        missing = [p for p in exp if p not in ports]
        wrong = [p for p in exp if p in ports and ports[p].get("kind") != exp[p]["kind"]]
        if missing or wrong:
            raise ValueError("port table mismatch: missing %s, wrong kind %s" % (missing[:6], wrong[:6]))
        return exp

    def _serial(self, kind, used):
        pre, n, alpha = SERIALS[kind]
        while True:
            s = pre + "".join(alpha[int(i)] for i in self.rng_serial.integers(0, len(alpha), n))
            if s not in used:
                used.add(s)
                return s

    def _build_parts(self):
        used = set()
        for port in sorted(self.ports):
            kind = self.ports[port]["kind"]
            if kind == "mpo":
                continue
            part = Part(port, kind, self._serial(kind, used))
            self._place_home(part, port)
        for h in HOSTS:                              # the fibre cables: one serial for both ends
            for k in range(8):
                _, nm, lm, _ = link_ports(h, k)
                cid = "cable:%s/%d" % (h, k)
                serial = self._serial("mpo", used)
                rack_gap = abs(int(h[5:7]) - 1) * H.RACK_W + (2.4 if RAIL_LEAF[k][5] != h[4] else 0.0)
                length = next(L for L in (5, 7, 10, 15) if L >= 4.5 + rack_gap)
                cab = Cable(cid, serial, h, k, nm, lm, length)
                self.cables[cid] = cab
                for end in (nm, lm):
                    part = Part(end, "mpo", serial)
                    part.cable = cid
                    self.cable_of[end] = cid
                    self._place_home(part, end)
        spares = self.config.get("spares", DEFAULT_SPARES)
        for sid in spares:
            self._register_spare(sid, used)
        self._used_serials = used

    def _place_home(self, part, port):
        part.home = part.last_port = port
        self.parts[part.id] = part
        self.occ[port] = part.id
        self.latched[port] = True
        self.port_hist[port].append((0.0, part.id))

    @staticmethod
    def _spare_kind(sid):
        core = sid.split(":", 1)[-1]
        if core.startswith("px_"):
            core = core[3:]
        kind = core.rsplit("_", 1)[0]
        return kind if kind in MODELS else None

    def _register_spare(self, sid, used=None, kind=None):
        kind = kind or self._spare_kind(sid)
        if kind is None or sid in self.parts:
            return self.parts.get(sid)
        part = Part(sid, kind, self._serial(kind, used if used is not None else self._used_serials), spare=True)
        if kind in ("osfp_n", "osfp_s"):
            part.tx = [float(v) for v in self.rng_spare.uniform(-0.3, 0.3, 4)]
            part.bias = [float(v) for v in self.rng_spare.uniform(BIAS_MA - 0.3, BIAS_MA + 0.3, 4)]
            part.temp = float(self.rng_spare.uniform(50, 55) if kind == "osfp_n" else self.rng_spare.uniform(41, 47))
            self.faces["rx:" + sid] = Face("rx:" + sid, "rx", sid)        # new: clean under its dust cap
        self.parts[sid] = part
        return part

    def _build_optics(self):
        r = self.rng_optics
        for pid in sorted(self.parts):
            part = self.parts[pid]
            if part.kind in ("osfp_n", "osfp_s") and not part.spare:
                part.tx = [float(v) for v in r.uniform(-0.3, 0.3, 4)]
                part.bias = [float(v) for v in r.uniform(BIAS_MA - 0.3, BIAS_MA + 0.3, 4)]
                part.temp = float(r.uniform(49, 56) if part.kind == "osfp_n" else r.uniform(40, 47))

    def _build_faces(self):
        r = self.rng_faces
        for pid in sorted(self.parts):
            part = self.parts[pid]
            if part.spare:
                continue
            if part.kind == "mpo":
                fid = "mpo:" + pid
            elif part.kind in ("osfp_n", "osfp_s"):
                fid = "rx:" + pid
            else:
                continue
            face = Face(fid, fid[:fid.index(":")], pid)
            for _ in range(int(r.integers(0, 3))):
                face.defects.append(_particle(r, int(r.integers(1, 13)), "cladding"))
            if r.random() < 0.15:
                face.defects.append(_pit(r, int(r.integers(1, 13))))
            self.faces[fid] = face

    # ============================================================================================ the scenario
    def _draw_scenario(self):
        """One shift's faults, from the seed (each block draws a fixed number of values; config["scenario"]
        overrides any field: {"gpu": {...}, "link": {...}, "hotswap": {...}, "mid": {...}, "distractors": {...}})."""
        r = self.rng_scn
        ov = self.config.get("scenario", {})

        def pick(table, u):
            acc = 0.0
            tot = sum(p for _, p in table)
            for name, p in table:
                acc += p / tot
                if u < acc:
                    return name
            return table[-1][0]

        u = r.random(8)
        gpu = {"host": HOSTS[int(u[0] * 8)], "cause": pick(GPU_CAUSES, u[1]), "gpu": int(u[2] * 8),
               "other_gpu_rank": int(u[3] * 7), "opened": -float(600 + u[4] * 2400), "seed": int(r.integers(2 ** 62))}
        gpu.update(ov.get("gpu", {}))
        gpu["slot"] = GPU_SLOT[gpu["gpu"]]
        others = [i for i in range(8) if i != gpu["gpu"]]
        gpu["other_gpu"] = int(gpu.get("other_gpu", others[gpu["other_gpu_rank"] % 7]))
        u = r.random(8)
        rest = [h for h in HOSTS if h != gpu["host"]]
        link = {"host": rest[int(u[0] * 7)], "k": int(u[1] * 8), "cause": pick(LINK_CAUSES, u[2]),
                "target_margin": float(0.25 + 0.5 * u[3]), "opened": -float(300 + u[4] * 1500),
                "seed": int(r.integers(2 ** 62))}
        link.update(ov.get("link", {}))
        u = r.random(8)
        rest = [h for h in HOSTS if h not in (gpu["host"], link["host"])]
        hs = {"host": rest[int(u[0] * 6)], "cause": pick(HOTSWAP_CAUSES, u[1]), "psu": 1 + int(u[2] * 6),
              "loose": bool(u[3] < 0.4), "loose_rank": int(u[4] * 5), "fan": 1 + int(u[5] * 5),
              "opened": -float(60 + u[6] * 1100)}
        hs.update(ov.get("hotswap", {}))
        if "loose_psu" not in hs:
            cands = [s for s in range(1, 7) if s != hs["psu"]]
            hs["loose_psu"] = cands[hs["loose_rank"] % 5]
        if hs["cause"] != "psu_failed":
            hs["loose"] = False
        u = r.random(10)
        rest = [h for h in HOSTS if h not in (gpu["host"], link["host"], hs["host"])]
        mid = {"kind": pick(MID_KINDS, u[0]), "t": float(1500 + 1500 * u[1]), "host": rest[int(u[2] * 5)],
               "fan": 1 + int(u[3] * 5), "k": int(u[4] * 8),
               "cause": "aged_tx_node" if u[5] < 0.5 else "aged_tx_leaf", "lane": int(u[6] * 4),
               "target_margin": float(0.25 + 0.5 * u[7])}
        mid.update(ov.get("mid", {}))
        mid["onset"] = mid["t"] - (120.0 if mid["kind"] == "fan_failed" else 90.0)
        u = r.random(8)
        ticketed = {gpu["host"], link["host"], hs["host"], mid["host"]}
        neigh = sorted({"gpn-%s%02d" % (h[4], int(h[5:7]) + d) for h in ticketed for d in (-1, 1)
                        if 2 <= int(h[5:7]) + d <= 5} - ticketed)
        free = sorted(set(HOSTS) - ticketed)
        dis = {"identify": (neigh[int(u[1] * len(neigh))] if neigh and u[0] < 0.5 else None),
               "fan_degraded": ({"host": free[int(u[3] * len(free))], "fan": 1 + int(u[4] * 5), "rotor": int(u[5] * 2)}
                                if free and u[2] < 0.5 else None)}
        dis.update(ov.get("distractors", {}))
        return {"gpu": gpu, "link": link, "hotswap": hs, "mid": mid, "distractors": dis}

    def _apply_scenario(self):
        sc = self.scn
        g = sc["gpu"]
        port = "%s/%s" % (g["host"], g["slot"])
        mod = self.parts[self.occ[port]]
        if g["cause"] == "transient":
            mod.wedged = True
        elif g["cause"] == "unseated":
            self.seat_q[port] = "loose"
        elif g["cause"] == "module_dead":
            mod.health = "dead"
        elif g["cause"] == "hbm_bad":
            mod.health = "hbm_bad"
        elif g["cause"] == "baseboard":
            self.slot_fault[port] = True
        reason = ("NHC: GPU %s row remapping failure" if g["cause"] == "hbm_bad" else "NHC: GPU %s missing") % \
            pci_addr(g["gpu"])
        self.sched.set(g["host"], "down", g["opened"], reason)
        self.sched.history[g["host"]] = [(round(g["opened"], 2), "down", reason)]
        self.sched.since[g["host"]] = g["opened"]
        self._place_link_fault(sc["link"])
        hs = sc["hotswap"]
        if hs["cause"] == "psu_failed":
            self.parts[self.occ["%s/psu%d" % (hs["host"], hs["psu"])]].health = "failed"
            if hs["loose"]:
                self.plug_bad["%s/plug%d" % (hs["host"], hs["loose_psu"])] = True
        else:
            fan = self.parts[self.occ["%s/fan%d" % (hs["host"], hs["fan"])]]
            fan.health, fan.rotors = "failed", [0.0, 0.0]
        mid = sc["mid"]
        self.pending.append((mid["onset"], "mid_fault", None))
        self.pending.append((mid["t"], "mid_ticket", None))
        dis = sc["distractors"]
        if dis.get("identify"):
            self.nodes[dis["identify"]].identify = "on"
        if dis.get("fan_degraded"):
            fd = dis["fan_degraded"]
            fan = self.parts[self.occ["%s/fan%d" % (fd["host"], fd["fan"])]]
            fan.health = "degraded"
            fan.rotors[fd["rotor"]] = 0.5
        rng = np.random.default_rng(self.seed ^ 0x5EED)
        for nd in self.nodes.values():
            nd.inlet = float(rng.uniform(22.4, 24.6))
        self.version += 1

    def _link_faces(self, h, k):
        """The four faces of link (h, k) as the parts sit now: {role: Face or None}."""
        o, nm, lm, lo = link_ports(h, k)
        get = lambda pre, port: self.faces.get(pre + self.occ[port]) if self.occ[port] else None
        return {"node_receptacle": get("rx:", o), "node_connector": get("mpo:", nm),
                "leaf_connector": get("mpo:", lm), "leaf_receptacle": get("rx:", lo)}

    def _place_link_fault(self, spec, sub=None):
        """Put a link root cause on link (host, k) so that its worst lane ends at the target margin."""
        h, k, cause, m_t = spec["host"], spec["k"], spec["cause"], spec["target_margin"]
        rng = sub if sub is not None else np.random.default_rng(spec.get("seed", 1))
        o, nm, lm, lo = link_ports(h, k)
        opt = self._optics_now(h, k)
        rx = {"node": opt["node_rx"], "leaf": opt["leaf_rx"]}
        faces = self._link_faces(h, k)
        spec["applied"] = {}
        if cause.startswith("aged_tx"):
            end = "node" if cause == "aged_tx_node" else "leaf"
            part = self.parts[self.occ[o if end == "node" else lo]]
            lane = int(spec.get("lane", rng.integers(0, 4)))
            far = "leaf" if end == "node" else "node"
            m0 = rx[far][lane] - RX_SENS_DBM                 # Type-B: Tx lane n arrives at the far Rx lane n
            part.tx[lane] -= (m0 - m_t)
            part.bias[lane] = BIAS_AGED_MA + float(rng.uniform(-0.2, 0.2))
            spec["applied"] = {"part": part.id, "lane": lane, "tx_dbm": _r(part.tx[lane], 3)}
        else:
            role = {"dirty_node_connector": "node_connector", "dirty_leaf_connector": "leaf_connector",
                    "dirty_node_receptacle": "node_receptacle", "dirty_leaf_receptacle": "leaf_receptacle"}.get(cause)
            if role is None:                                      # scratched_cable_end
                role = "node_connector" if rng.random() < 0.5 else "leaf_connector"
            face = faces[role]
            end = role.split("_")[0]
            p = int(spec.get("fibre", USED_FIBRES[int(rng.integers(0, 8))]))
            rend, lane = _rx_lane_of(end, p)
            E = rx[rend][lane] - RX_SENS_DBM - m_t
            added = []
            if cause == "scratched_cable_end":
                E = min(max(E, SCRATCH_CORE_DB[0]), SCRATCH_CORE_DB[1])
                added.append(_scratch(rng, p, E))
            else:
                combos = [["particle"] * 3, ["particle", "particle", "film"], ["particle"] * 2,
                          ["particle", "film"], ["particle"] * 3 + ["film"]]
                feas = [c for c in combos if _split_loss(rng, E, c) is not None]
                combo = feas[int(rng.integers(0, len(feas)))] if feas else ["particle"] * 3
                losses = _split_loss(rng, E, combo) or [E / len(combo)] * len(combo)
                for kind, L in zip(combo, losses):
                    if kind == "particle":
                        added.append(_particle(rng, p, "core", loss=L, origin="scenario"))
                    else:
                        added.append(_film(rng, p, L / FILM_FULL_DB))
                if rng.random() < 0.5:                            # a lighter speck on a second used fibre
                    p2 = [q for q in USED_FIBRES if q != p][int(rng.integers(0, 7))]
                    rend2, lane2 = _rx_lane_of(end, p2)
                    L2 = float(rng.uniform(0.6, 0.9))
                    if rx[rend2][lane2] - RX_SENS_DBM - L2 >= 1.5:
                        added.append(_particle(rng, p2, "core", loss=L2, origin="scenario"))
            face.defects.extend(added)
            spec["applied"] = {"face": face.id, "role": role, "fibre": p, "rx_end": rend, "rx_lane": lane,
                               "extra_db": _r(sum(d["loss"] if d["type"] != "film" else FILM_FULL_DB * d["strength"]
                                                  for d in added), 3)}
        self.version += 1
        spec["applied"]["worst_margin_db"] = _r(self._worst_margin(h, k), 3)

    # ============================================================================================ history, tickets
    def _sel_raw(self, nd, t, sensor, event, severity="info"):
        nd.sel.append({"id": len(nd.sel) + 1, "time": _r(t), "timestamp": _label(t), "sensor": sensor,
                       "event": event, "severity": severity})

    def _sel(self, nd, t, sensor, event, severity="info"):
        if nd.power != "off":                                    # the BMC lives on standby power
            self._sel_raw(nd, t, sensor, event, severity)

    def _init_history(self):
        r = self.rng_hist
        sc = self.scn
        for h in HOSTS:
            nd = self.nodes[h]
            tb = -float(r.uniform(4, 30)) * 86400
            self._sel_raw(nd, tb, "System ACPI Power State", "S0/G0: working (host power on)")
            self._sel_raw(nd, tb + 296, "System Event", "OS boot completed")
        g = sc["gpu"]
        nd = self.nodes[g["host"]]
        if g["cause"] == "baseboard":
            j = g["other_gpu"]
            t_old = -float(r.uniform(2.5, 3.5)) * 86400
            g["history_t"] = t_old
            self._sel_raw(nd, t_old, "GPU%d (%s)" % (j, pci_addr(j)), "Xid 79: GPU has fallen off the bus", "critical")
            self._sel_raw(nd, t_old + 1800, "System ACPI Power State", "Host power cycle (BMC)")
            self._sel_raw(nd, t_old + 2100, "System Event", "OS boot completed")
        t_x = g["opened"] - float(r.uniform(20, 120))
        i = g["gpu"]
        if g["cause"] == "hbm_bad":
            self._sel_raw(nd, t_x, "GPU%d (%s)" % (i, pci_addr(i)), "Xid 64: row remapping failure", "critical")
            self._sel_raw(nd, t_x + 3, "GPU%d (%s)" % (i, pci_addr(i)),
                          "Xid 48: uncorrectable double-bit ECC error (DBE) in framebuffer", "critical")
        else:
            self._sel_raw(nd, t_x, "GPU%d (%s)" % (i, pci_addr(i)), "Xid 79: GPU has fallen off the bus", "critical")
        hs = sc["hotswap"]
        nd = self.nodes[hs["host"]]
        if hs["cause"] == "psu_failed":
            if hs["loose"]:
                s2 = hs["loose_psu"]
                t1 = hs["opened"] - float(r.uniform(3000, 9000))
                self._sel_raw(nd, t1, "PSU%d Status" % s2, "AC input lost", "warning")
                self._sel_raw(nd, t1 + float(r.uniform(4, 40)), "PSU%d Status" % s2, "AC input restored")
                hs["ac_lost_t"] = t1 + float(r.uniform(600, 2400))
                self._sel_raw(nd, hs["ac_lost_t"], "PSU%d Status" % s2, "AC input lost", "warning")
            self._sel_raw(nd, hs["opened"], "PSU%d Status" % hs["psu"], "Failure detected", "critical")
            n_ok = 6 - 1 - (1 if hs["loose"] else 0)
            self._sel_raw(nd, hs["opened"] + 1, "Power Unit Redundancy",
                          "Redundancy degraded (%d of 6 PSUs OK)" % n_ok, "warning")
        else:
            m = hs["fan"]
            self._sel_raw(nd, hs["opened"], "FAN%d_F" % m, "Lower critical going low: 0 RPM", "critical")
            self._sel_raw(nd, hs["opened"], "FAN%d_R" % m, "Lower critical going low: 0 RPM", "critical")
        dis = sc["distractors"]
        if dis.get("fan_degraded"):
            fd = dis["fan_degraded"]
            self._sel_raw(self.nodes[fd["host"]], -float(r.uniform(1, 5)) * 86400,
                          "FAN%d_%s" % (fd["fan"], "FR"[fd["rotor"]]),
                          "Lower non-critical going low: %d RPM" % int(FAN_RPM * 0.5), "warning")
        if dis.get("identify"):
            self._sel_raw(self.nodes[dis["identify"]], -float(r.uniform(2, 8)) * 3600, "Identify",
                          "Chassis identify LED on (force)")
        for nd in self.nodes.values():
            nd.sel.sort(key=lambda e: e["time"])
            for n, e in enumerate(nd.sel):
                e["id"] = n + 1

    def _init_thermal(self):
        for nd in self.nodes.values():
            nd.temp = nd.inlet + 15.0
            for _ in range(240):
                self._thermal_step(nd, 0.0, 5.0, silent=True)
        hs = self.scn["hotswap"]
        if hs["cause"] == "fan_failed" and self.nodes[hs["host"]].throttled:
            nd = self.nodes[hs["host"]]
            self._sel_raw(nd, hs["opened"] + 40, "GPU Thermal", "GPU thermal slowdown asserted (hottest GPU 87 C)",
                          "warning")
            nd.sel.sort(key=lambda e: e["time"])
            for n, e in enumerate(nd.sel):
                e["id"] = n + 1

    def _new_ticket_id(self):
        if self._t_id == 0:
            self._t_id = 48200 + int(self.rng_tix.integers(0, 600))
        self._t_id += 1 + int(self.rng_tix.integers(0, 9))
        return "INC-%05d" % self._t_id

    def _make_tickets(self):
        sc = self.scn
        order = sorted([("gpu", sc["gpu"]["opened"]), ("link", sc["link"]["opened"]),
                        ("hotswap", sc["hotswap"]["opened"])], key=lambda x: x[1])
        for kind, _ in order:
            getattr(self, "_ticket_" + kind)(sc[kind])

    def _add_ticket(self, kind, host, opened, visible_at, priority, summary, lines, fields, truth):
        tid = self._new_ticket_id()
        tk = Ticket(tid, kind, host, opened, visible_at, priority, summary, "\n".join(lines), fields, truth)
        self.tickets[tid] = tk
        return tk

    def _ticket_gpu(self, g):
        h, i, cause = g["host"], g["gpu"], g["cause"]
        pa, ps = pci_addr(i), pci_addr(i)[:-2]
        det = g["opened"] - 30.0
        loc = host_location(h)
        if cause == "hbm_bad":
            summary = "%s: GPU %d (PCI %s) row-remapping failure (Xid 64, Xid 48) - node DOWN" % (h, i, pa)
            lines = ["Alert source: node health check (NHC) on %s at %s" % (h, _label(det)),
                     "kernel: NVRM: Xid (PCI:%s): 64, pid=<unknown>, Row remapper: failed to remap a row "
                     "(uncorrectable error)" % ps,
                     "kernel: NVRM: Xid (PCI:%s): 48, pid=<unknown>, An uncorrectable double bit error (DBE) has been "
                     "detected on GPU in the framebuffer" % ps,
                     "nvidia-smi -q -d ROW_REMAPPER: GPU %d  Remapping Failure Occurred: Yes" % i,
                     "check_hw_gpu: GPU %s row remapping failure" % pa]
            xid = [64, 48]
        else:
            summary = "%s: GPU %d (PCI %s) fell off the bus (Xid 79) - node DOWN" % (h, i, pa)
            lines = ["Alert source: node health check (NHC) on %s at %s" % (h, _label(det)),
                     "check_hw_gpu: expected 8 GPUs, found 7 (missing PCI %s)" % pa,
                     "kernel: NVRM: Xid (PCI:%s): 79, pid='<unknown>', name=<unknown>, GPU has fallen off the bus."
                     % ps]
            xid = [79]
        lines += ["Scheduler: %s set DOWN, reason \"%s\"" % (h, self.sched.reason[h]),
                  "Location: %s (KX-8H, SN %s)" % (loc, self._chassis_serial(h)),
                  "Component: GPU index %d, PCI %s" % (i, pa)]
        if cause == "baseboard":
            old = "INC-%05d" % (int(self.rng_tix.integers(41000, 47000)))
            j = g["other_gpu"]
            lines.append("Related: %s (%s): Xid 79 on GPU %d (PCI %s), cleared by a host reboot"
                         % (old, _label(g["history_t"])[:10], j, pci_addr(j)))
        lines.append("Recommended procedure: %s (GPU fault triage)" % SOP["gpu"])
        fields = {"host": h, "location": loc, "component": "GPU", "gpu_index": i, "pci": pa, "xid": xid,
                  "node_state": "down", "sop": SOP["gpu"]}
        self._add_ticket("gpu", h, g["opened"], 0.0, "P2", summary, lines, fields,
                         {"cause": cause, "gpu": i, "slot": g["slot"]})

    def _link_ticket_lines(self, h, k, t, n_err, window_s, when_label):
        leaf, p = RAIL_LEAF[k], NODE_ORDER[h]
        cab = self.cables[self.cable_of["%s/mpo%d" % (h, k + 1)]]
        opt = self._optics_now(h, k)
        mn, ml = min(opt["node_rx"]) - RX_SENS_DBM, min(opt["leaf_rx"]) - RX_SENS_DBM
        rep = ("%s port %d" % (leaf, p), "%s mlx5_%d port 1" % (h, k)) if ml <= mn else \
            ("%s mlx5_%d port 1" % (h, k), "%s port %d" % (leaf, p))
        ber = n_err / max(1.0, window_s * LINE_BPS)
        summary = "Link errors: %s port %d (rail %d) <-> %s port %d: symbol BER above threshold" % (
            h, k + 1, k, leaf, p)
        lines = ["Alert source: fabric manager, event 916 \"High Symbol BER\" (WARNING) at %s" % when_label,
                 "Reporting port: %s (peer %s)" % rep,
                 "Symbol errors in the last %d min: %d; effective BER %.1e (threshold 1e-16); link down count: 0"
                 % (int(round(window_s / 60.0)), n_err, ber),
                 "Node %s: %s in job %d (%s)" % (h, self.sched.state[h].upper(), JOB_ID, JOB_NAME),
                 "Cable: %s (%s, %s, %d m)" % (cab.label, MODELS["mpo"], cab.serial, cab.length_m),
                 "Node end: %s, rear, OSFP port %d (HCA mlx5_%d)" % (host_location(h), k + 1, k),
                 "Switch end: %s (%s), rear, port %d" % (leaf_location(leaf), leaf, p),
                 "Recommended procedure: %s (bad link)" % SOP["link"]]
        fields = {"host": h, "rail": k, "node_port": k + 1, "leaf": leaf, "leaf_port": p, "cable": cab.label,
                  "cable_serial": cab.serial, "symbol_errors": n_err, "sop": SOP["link"]}
        return summary, lines, fields

    def _ticket_link(self, spec):
        h, k = spec["host"], spec["k"]
        opt = self._optics_now(h, k)
        n_err = 0
        for key, rx in (((h, k + 1), opt["node_rx"]), ((RAIL_LEAF[k], NODE_ORDER[h]), opt["leaf_rx"])):
            rate = sum(err_rate(v - RX_SENS_DBM) for v in rx)        # the counters as the shift finds them
            c = self.ctr[key]
            c.up_s = 900.0 + float(-spec["opened"])
            if rate > 0.0:
                c.symbol_errors = c.tot_errors = max(1, int(self.rng_tix.poisson(rate * c.up_s)))
                c.last_error_t = -float(self.rng_tix.uniform(1.0, 15.0))
                n_err = max(n_err, int(self.rng_tix.poisson(rate * 900.0)) or 1)
        summary, lines, fields = self._link_ticket_lines(h, k, spec["opened"], n_err, 900.0, _label(spec["opened"]))
        self._add_ticket("link", h, spec["opened"], 0.0, "P2", summary, lines, fields,
                         {"cause": spec["cause"], "rail": k, "applied": spec.get("applied")})

    def _ticket_hotswap(self, hs):
        h = hs["host"]
        loc = host_location(h)
        if hs["cause"] == "psu_failed":
            s = hs["psu"]
            n_ok = 6 - 1 - (1 if hs["loose"] else 0)
            summary = "%s: PSU%d failure - power redundancy degraded (%d of 6 OK)" % (h, s, n_ok)
            lines = ["Alert source: BMC %s at %s" % (h, _label(hs["opened"])),
                     "PSU%d Status: Failure detected (feed %s)" % (s, "A" if s % 2 else "B")]
            if hs["loose"]:
                lines.append("PSU%d Status: AC input lost (since %s)" % (hs["loose_psu"], _label(hs["ac_lost_t"])))
            lines += ["Power supplies OK: %d of 6 (4+2 redundant)" % n_ok,
                      "Location: %s, rear, power supply bays" % loc,
                      "Recommended procedure: %s (power supply hot swap)" % SOP["psu"]]
            fields = {"host": h, "location": loc, "component": "PSU%d" % s, "psu_ok": n_ok, "sop": SOP["psu"]}
            self._add_ticket("psu", h, hs["opened"], 0.0, "P2" if n_ok <= 4 else "P3", summary, lines, fields,
                             {"cause": "psu_failed", "psu": s, "loose_psu": hs["loose_psu"] if hs["loose"] else None})
        else:
            m = hs["fan"]
            nd = self.nodes[h]
            summary = "%s: fan module %d failed (0 RPM)%s" % (h, m, " - GPU thermal slowdown" if nd.throttled else "")
            lines = ["Alert source: BMC %s at %s" % (h, _label(hs["opened"])),
                     "FAN%d_F: 0 RPM, FAN%d_R: 0 RPM (lower critical)" % (m, m)]
            if nd.throttled:
                lines.append("GPU thermal slowdown asserted (hottest GPU %.0f C)" % nd.temp)
            lines += ["Location: %s, front, the GPU tray's fan wall: fan module %d of 5" % (loc, m),
                      "Recommended procedure: %s (fan module hot swap, 30 s limit)" % SOP["fan"]]
            fields = {"host": h, "location": loc, "component": "FAN%d" % m, "sop": SOP["fan"]}
            self._add_ticket("fan", h, hs["opened"], 0.0, "P1" if nd.throttled else "P3", summary, lines, fields,
                             {"cause": "fan_failed", "fan": m})

    def _chassis_serial(self, h):
        row, idx = h[4].upper(), int(h[5:7])
        return "KX8-24-%04d" % (100 + 7 * idx + (0 if row == "A" else 50))   # as printed on the host label

    # ============================================================================================ the cell's events
    def on_event(self, ev, t):
        """One physical event from the cell (section 1 of the specification), at sim time t."""
        t = max(float(t), self.t)
        kind = ev[0]
        if kind in ("holstered", "pool_empty", "tray_service_stop", "tray_service_stop_released"):
            return
        if kind == "clean":
            self.on_clean(ev[1], t)
            return
        if kind == "parts_in_tote":
            self.parts_in_tote = list(ev[1])
            return
        self._catch_up(t)
        self._advance(t)
        try:
            if kind == "unlatched":
                self._ev_unlatched(ev[1], t)
            elif kind == "removed":
                self._ev_removed(ev[1], ev[2], t)
            elif kind == "entered":
                self._ev_entered(ev[1], ev[2], t)
            elif kind == "seated":
                self._ev_seated(ev[1], ev[2], t)
            elif kind == "damaged":
                self._ev_damaged(ev[1], ev[2], t)
            elif kind in ("tray_unlatched", "tray_latched"):
                self._ev_tray(ev[1], kind == "tray_latched", t)
            elif kind == "screw":
                self._ev_screw(ev[1], int(ev[2]), ev[3], ev[4], t)
            else:
                self.unknown.append(("event", tuple(ev), _r(t)))
                return
        except (KeyError, IndexError, TypeError, ValueError):
            self.unknown.append(("event", tuple(ev), _r(t)))
            return
        self.version += 1
        self._evaluate(t)

    def _part(self, inst, port=None):
        part = self.parts.get(inst)
        if part is None and inst is not None:
            kind = self.ports[port]["kind"] if port in self.ports else None
            part = self._register_spare(inst, kind=kind)
        return part

    def _touch(self, port, t, kind=None):
        rec = self.ports.get(port)
        if rec is None:
            return
        host = rec.get("host")
        k = rec["kind"]
        if k in ("osfp_n", "osfp_s", "mpo"):
            ln = port_link(port)
            if ln is not None:
                self.interventions.append((round(t, 2), "link", ln, port))
        elif host is not None:
            self.interventions.append((round(t, 2), "host", host, port))

    def _ev_unlatched(self, port, t):
        self.latched[port] = False
        self._touch(port, t)

    def _ev_removed(self, port, inst, t):
        rec = self.ports[port]
        inst = inst if inst is not None else self.occ[port]
        part = self._part(inst, port)
        self.occ[port] = None
        self.latched[port] = False
        self.port_hist[port].append((round(t, 2), None))
        if part is not None:
            part.touched = True
            part.last_port = port
            if rec["kind"] == "gpu":
                part.wedged = False                      # out of the node: fully de-energised
        if rec["kind"] == "gpu":
            self.armed[port], self.snug[port], self.screw_out[port] = True, [], {1, 2, 3, 4}
            self.seat_q[port] = "good"
        if rec["kind"] == "plug":
            self.plug_bad[port] = False                  # the loose cord: remove + insert + latch reseats it
        self._touch(port, t)

    def _ev_entered(self, port, inst, t):
        part = self._part(inst, port)
        if part is not None:
            part.touched = True
            part.last_port = port
        self.occ[port] = inst
        self.latched[port] = False
        self.port_hist[port].append((round(t, 2), inst))
        self._touch(port, t)

    def _ev_seated(self, port, inst, t):
        if self.occ[port] is None:
            self._ev_entered(port, inst, t)
        if inst is not None and self.occ[port] != inst:
            self.occ[port] = inst
            self.port_hist[port].append((round(t, 2), inst))
        self.latched[port] = True
        rec = self.ports[port]
        if rec["kind"] == "mpo" and self.occ[port]:
            tr = self.occ.get(rec["needs"])
            cf, rf = self.faces.get("mpo:" + self.occ[port]), self.faces.get("rx:" + tr) if tr else None
            if cf is not None and rf is not None:
                self._mate(cf, rf, t)
        self._touch(port, t)

    def _mate(self, cf, rf, t):
        """Plugging a connector into a receptacle: each loose particle crosses to the other face with p = 0.35
        (both ways; mirrored across the row as the faces meet)."""
        move_c = [d for d in cf.defects if d["type"] == "particle" and self.rng_mate.random() < TRANSFER_P]
        move_r = [d for d in rf.defects if d["type"] == "particle" and self.rng_mate.random() < TRANSFER_P]
        for src, dst, mv in ((cf, rf, move_c), (rf, cf, move_r)):
            for d in mv:
                src.defects.remove(d)
                dst.defects.append(dict(d, x=-d["x"], origin="transfer"))
        if move_c or move_r:
            self.cleanings.append({"t": _r(t), "face": cf.id, "event": "mated", "to": rf.id,
                                   "moved_to_receptacle": len(move_c), "moved_to_connector": len(move_r)})

    def _ev_damaged(self, port, why, t):
        self.broken[port] = True
        rec = self.ports[port]
        if rec["kind"] == "gpu":
            self.socket_bad[port] = True
            self._incident("pins_bent", t, rec.get("host"), port=port, why=why)
        else:
            if rec["kind"] == "mpo" and self.occ.get(rec["needs"]):
                f = self.faces.get("rx:" + self.occ[rec["needs"]])
                if f is not None:
                    f.damaged = True
            self._incident("port_damaged", t, rec.get("host") or rec.get("leaf"), port=port, kind=rec["kind"], why=why)
        self._touch(port, t)

    def _ev_tray(self, host, latched, t):
        self.tray_latched[host] = latched
        nd = self.nodes[host]
        self.interventions.append((round(t, 2), "host", host, "tray"))
        if latched:
            self._sel(nd, t, "Chassis Intrusion", "GPU tray: seated")
        else:
            self._sel(nd, t, "Chassis Intrusion", "GPU tray: not seated", "warning")

    def _ev_screw(self, port, k, mode, result, t):
        if port not in self.seat_q:
            return
        if mode == "loosen" and result == "out":
            self.screw_out[port].add(k)
            if len(self.screw_out[port]) == 4 and not self.armed[port]:
                self.armed[port], self.snug[port] = True, []
        elif mode == "tighten" and result == "clutch":
            self.screw_out[port].discard(k)
            if self.armed[port] and k not in self.snug[port]:
                self.snug[port].append(k)
        self._touch(port, t)

    def release_ok(self, port):
        """Extra interlocks on top of the cell's (screws, plugs, MPO): none."""
        return True

    # ============================================================================================ timers and ticks
    def _advance(self, t):
        """Run every scheduled transition due at or before t, in time order, each at its own time."""
        while True:
            best = None
            cands = [(self.next_cp, 0, "checkpoint", None)]
            if self.job["state"] == "restarting":
                cands.append((self.job["restart_at"], 1, "restart", None))
            for h in HOSTS:
                if self.nodes[h].timer is not None:
                    cands.append((self.nodes[h].timer[0], 2, "power", h))
            for n, (tt, kind, data) in enumerate(self.pending):
                cands.append((tt, 3, "pending", n))
            for rid in self.dcgm_active:
                cands.append((self.dcgm_runs[rid]["end"], 4, "dcgm", rid))
            for sid in self.soak_active:
                cands.append((self.soaks[sid]["end"], 5, "soak", sid))
            for c in cands:
                if c[0] is not None and c[0] <= t + 1e-9 and (best is None or (c[0], c[1]) < (best[0], best[1])):
                    best = c
            if best is None:
                break
            tt, _, kind, data = best
            self.t = max(self.t, tt)
            if kind == "checkpoint":
                self.next_cp += CHECKPOINT_S
                self._checkpoint(tt)
            elif kind == "restart":
                self._restart(tt)
            elif kind == "power":
                self._power_timer(self.nodes[data], tt)
            elif kind == "pending":
                _, pk, pdata = self.pending.pop(data)
                self._pending(pk, pdata, tt)
            elif kind == "dcgm":
                self._dcgm_finish(data, tt)
            elif kind == "soak":
                self._soak_finish(data, tt)
            self._evaluate(tt)
        self.t = max(self.t, t)

    def tick(self, t, phys=None):
        """Integrate the continuous processes (temperatures, symbol errors, flaps, the job's progress, the hot-swap
        clocks, the trays) up to t; the cell calls it every 1.0 s of sim time."""
        t = float(t)
        if phys is not None:
            self.phys = phys
        t0 = self.t_tick
        if t <= t0 + 1e-9:
            self._advance(max(t, self.t))
            self._physical_checks(max(t, self.t))
            self._evaluate(max(t, self.t))
            return
        n = max(1, int(math.ceil((t - t0) / 1.0 - 1e-9)))
        dt = (t - t0) / n
        for i in range(1, n + 1):
            tau = t0 + dt * i
            self._advance(tau)
            self._physical_checks(tau)
            self._flap_onsets(tau)
            self._evaluate(tau)
            self._continuous(tau, dt)
            self._evaluate(tau)
        self.t_tick = t

    def _catch_up(self, t):
        """Integrate the continuous processes up to an input's time (with the last PhysicalState) before applying
        the input, so the model does not depend on how often the cell calls tick()."""
        if t > self.t_tick + 1e-9:
            self.tick(t, None)

    def _fastened(self, port):
        ph = self.phys
        if ph is not None and port in ph.screws_fastened:
            return bool(ph.screws_fastened[port])
        return not self.armed[port] or len(self.snug[port]) == 4

    def _physical_checks(self, t):
        """The trays, the GPU modules' seating, the hot-swap slots' clocks (from the cell's state)."""
        ph = self.phys
        out_n = 0
        for h in HOSTS:
            nd = self.nodes[h]
            pos = float(ph.tray_pos.get(h, 0.0)) if ph is not None else 0.0
            if pos > 0.05:
                out_n += 1
            if pos > 0.005 and not nd.tray_out:
                nd.tray_out = True
                self.interventions.append((round(t, 2), "host", h, "tray"))
                if nd.power in ON_STATES:
                    self._hot_tray_pull(nd, t)
                elif nd.ac_any:
                    self._incident("tray_pulled_with_ac", t, h, power=nd.power, psus_with_ac=self._n_ac(h))
            elif pos <= 0.005 and nd.tray_out:
                nd.tray_out = False
            for m in range(1, 6):
                self._hotswap_clock(nd, "%s/fan%d" % (h, m), "fan", t)
            for s in range(1, 7):
                self._hotswap_clock(nd, "%s/psu%d" % (h, s), "psu", t)
        if out_n >= 2 and not self._two_trays:
            self._two_trays = True
            hosts = [h for h in HOSTS if ph is not None and float(ph.tray_pos.get(h, 0.0)) > 0.05]
            self._incident("two_trays_extended", t, hosts[0] if hosts else None, hosts=hosts)
        elif out_n < 2:
            self._two_trays = False
        for port in self.seat_q:                         # the snug pass decides how the module sits
            if self.armed[port] and self.occ[port] and self.latched[port] and self._fastened(port):
                first = set(self.snug[port][:2])
                self.seat_q[port] = "good" if first in ({1, 4}, {2, 3}) else "loose"
                self.armed[port] = False
                self.version += 1

    def _hotswap_clock(self, nd, port, kind, t):
        present = self.occ[port] is not None and self.latched[port]
        if present or nd.power not in ON_STATES:
            nd.empty_since.pop(port, None)
            nd.overtime_logged.discard(port)
            return
        t0 = nd.empty_since.setdefault(port, t)
        if t - t0 > HOTSWAP_LIMIT_S and port not in nd.overtime_logged:
            nd.overtime_logged.add(port)
            self._incident("hot_swap_overtime", t, nd.host, port=port, kind=kind, seconds=_r(t - t0, 1))

    def _flap_onsets(self, t):
        for (h, k), ln in self.links.items():
            if ln.band != "flapping":
                continue
            if ln.next_flap is None:
                ln.next_flap = t + float(np.random.default_rng(_stable_seed(self.seed, "flap0", h, k, ln.n_flap))
                                         .uniform(2.0, 20.0))
            while ln.next_flap <= t + 1e-9:
                ln.down_until = ln.next_flap + FLAP_DOWN_S
                ln.n_flap += 1
                ln.next_flap += float(np.random.default_rng(_stable_seed(self.seed, "flap", h, k, ln.n_flap))
                                      .uniform(*FLAP_GAP))

    def _continuous(self, t, dt):
        for h in HOSTS:
            self._thermal_step(self.nodes[h], t, dt)
        self._ensure_optics()
        for (h, k), ln in self.links.items():            # symbol errors, link-up time
            if not self._is_up(ln, t):
                continue
            opt = self._opt[(h, k)]
            leaf, p = ln.peer
            for key, rx in (((h, k + 1), opt["node_rx"]), ((leaf, p), opt["leaf_rx"])):
                c = self.ctr[key]
                c.up_s += dt
                rate = sum(err_rate(v - RX_SENS_DBM) for v in rx)
                if rate > 0.0:
                    n = int(self.rng_err.poisson(rate * dt))
                    if n:
                        c.symbol_errors += n
                        c.tot_errors += n
                        c.last_error_t = t
        for h in HOSTS:
            nd = self.nodes[h]
            nd.link_err_recent = any(t - self.ctr[(h, k + 1)].last_error_t < 60.0 or
                                     (self.links[(h, k)].peer is not None and
                                      t - self.ctr[self.links[(h, k)].peer].last_error_t < 60.0) for k in range(8))
            if nd.dcgm:
                run = self.dcgm_runs[nd.dcgm]
                run["throttle_seen"] |= nd.throttled
                run["brake_seen"] |= nd.brake
        job = self.job
        if job["state"] == "running":
            members = [h for h in HOSTS if self.sched.state[h] in ("alloc", "drain")]
            if members:
                pace = min(self.nodes[h].speed * (LINK_ERR_SLOWDOWN if self.nodes[h].link_err_recent else 1.0)
                           for h in members)
                w = dt * 8 * len(members) * pace
                job["since_cp_work"] += w
                job["work_total"] += w

    # ============================================================================================ power
    def _psu_ac(self, h, s):
        pp, lp = "%s/psu%d" % (h, s), "%s/plug%d" % (h, s)
        if not (self.occ[pp] and self.latched[pp] and self.occ[lp] and self.latched[lp]) or self.plug_bad[lp]:
            return False
        if self.broken[pp] or self.broken[lp]:
            return False
        return self.pdu[host_rack(h)][("A" if s % 2 else "B") + str((s + 1) // 2)] == "on"

    def _n_ac(self, h):
        return sum(self._psu_ac(h, s) for s in range(1, 7))

    def _psu_good(self, h, s):
        return self._psu_ac(h, s) and self.parts[self.occ["%s/psu%d" % (h, s)]].health == "ok"

    def _psu_status(self, h, s):
        pp = "%s/psu%d" % (h, s)
        if not (self.occ[pp] and self.latched[pp]):
            return "absent"
        if not self._psu_ac(h, s):
            return "ac_lost"
        return "ok" if self.parts[self.occ[pp]].health == "ok" else "failure"

    def _power_eval(self, nd, t):
        h = nd.host
        n_good = sum(self._psu_good(h, s) for s in range(1, 7))
        nd.n_good, nd.ac_any = n_good, self._n_ac(h) > 0
        if n_good == 0:
            if nd.power != "off":
                if nd.power in ON_STATES:
                    self._power_lost(nd, t, "AC lost (no power supply with AC)")
                nd.power, nd.timer, nd.after_shutdown, nd.brake = "off", None, None, False
                nd.ac_off_since = t
            elif nd.ac_off_since is not None and t - nd.ac_off_since >= COLD_S:
                self._cold_reset(nd)
            return
        if nd.power == "off":
            nd.power = "standby"
            if nd.ac_off_since is not None and t - nd.ac_off_since >= COLD_S:
                self._cold_reset(nd)
            nd.ac_off_since = None
            self._sel(nd, t, "System Event", "AC power restored: BMC started")
        if nd.power in ON_STATES and n_good <= 2:
            self._power_lost(nd, t, "insufficient power supplies (%d of 6 OK)" % n_good)
            nd.power, nd.timer, nd.after_shutdown = "standby", None, None
            self._sel(nd, t, "System ACPI Power State", "Host power lost: %d of 6 PSUs OK" % n_good, "critical")
        brake = nd.power in ("booting", "on") and n_good == 3
        if brake and not nd.brake:
            in_job = self.sched.state[h] in ("alloc", "drain") and self.job["state"] == "running"
            self._incident("power_brake", t, h, psus_ok=n_good, in_job=in_job)
            self._sel(nd, t, "Power Unit Redundancy", "Power brake asserted: GPUs capped (3 of 6 PSUs OK)", "critical")
        elif nd.brake and not brake:
            self._sel(nd, t, "Power Unit Redundancy", "Power brake deasserted")
        nd.brake = brake

    def _cold_reset(self, nd):
        """The node has been without AC for >= 30 s: every module in it is fully de-energised."""
        for s in SLOTS:
            inst = self.occ["%s/%s" % (nd.host, s)]
            if inst and self.parts[inst].wedged:
                self.parts[inst].wedged = False
                self.version += 1

    def _power_lost(self, nd, t, why):
        h = nd.host
        in_job = self.sched.state[h] in ("alloc", "drain") and self.job["state"] in ("running", "restarting")
        if in_job:
            self._incident("node_crash", t, h, cause=why)
        else:
            self._incident("power_lost_while_on", t, h, cause=why, power=nd.power)
        nd.down_cause = "power loss: " + why
        self._abort_dcgm(nd, t, "node lost power")

    def _hot_tray_pull(self, nd, t):
        h = nd.host
        self._incident("hot_tray_pull", t, h, power=nd.power)
        for s in SLOTS:
            inst = self.occ["%s/%s" % (h, s)]
            if inst:
                part = self.parts[inst]
                if part.health != "connector_damaged":
                    part.prior_health = part.health
                part.health = "connector_damaged"
        self._sel(nd, t, "Chassis Intrusion", "GPU tray removed while powered: host power lost", "critical")
        nd.power, nd.timer, nd.after_shutdown = "standby", None, None
        nd.down_cause = "hot tray pull"
        self._abort_dcgm(nd, t, "node lost power")
        self.version += 1

    def _start_boot(self, nd, t):
        h = nd.host
        ph = self.phys
        tray_home = ph is None or float(ph.tray_pos.get(h, 0.0)) <= 0.005
        if not self.tray_latched[h] or not tray_home:
            nd.post_error = True
            self._sel(nd, t, "System Firmware Progress", "POST error: GPU tray not seated", "critical")
            return False
        if nd.n_good < 3:
            self._sel(nd, t, "Power Unit Redundancy",
                      "Power on denied: insufficient PSU capacity (%d of 6 OK)" % nd.n_good, "critical")
            return False
        nd.power, nd.timer = "booting", (t + BOOT_S, "on")
        self._sel(nd, t, "System ACPI Power State", "Host power on: POST started")
        return True

    def _boot_done(self, nd, t):
        h = nd.host
        nd.power, nd.timer, nd.post_error, nd.down_cause = "on", None, False, None
        nd.boots += 1
        nd.temp = min(nd.temp, nd.inlet + 15.0)
        for k in range(8):
            self.ctr[(h, k + 1)].clear()                 # the HCAs' counters start from zero after a boot
        self._sel(nd, t, "System Event", "OS boot completed")
        for i in range(8):
            st = self._gpu_state(h, i)
            if st == "missing":
                self._sel(nd, t, "GPU%d (%s)" % (i, pci_addr(i)), "Not detected on PCIe (link training failed)",
                          "critical")
            elif st == "remap_failure":
                self._sel(nd, t, "GPU%d (%s)" % (i, pci_addr(i)), "Xid 64: row remapping failure (pending at init)",
                          "critical")

    def _power_timer(self, nd, t):
        action = nd.timer[1]
        nd.timer = None
        if action == "on":
            if nd.power == "booting":
                self._boot_done(nd, t)
        elif action == "standby":
            if nd.power == "shutting_down":
                nd.power = "standby"
                self._sel(nd, t, "System ACPI Power State", "S5/G2: host soft-off")
                if nd.after_shutdown == "boot":
                    nd.timer = (t + CYCLE_OFF_S, "boot")
            nd.after_shutdown = None
        elif action == "boot":
            if nd.power == "standby":
                self._start_boot(nd, t)

    def _thermal_step(self, nd, t, dt, silent=False):
        on = nd.power in ON_STATES
        if on:
            demand = self._load(nd)
            L = demand * (BRAKE_FACTOR if nd.brake else 1.0)
            a = max(0.05, self._airflow(nd.host))
            if nd.throttled:
                L = min(L, max(THROTTLE_FLOOR, (T_THROTTLE - T_BASE) * a ** 1.5 / T_RISE))
            target = T_BASE + T_RISE * L / a ** 1.5
            nd.speed = L / demand if demand > 0 else 1.0
            nd.load_eff = L
        else:
            target = nd.inlet + 1.5
            nd.speed, nd.load_eff = 0.0, 0.0
        nd.temp = target + (nd.temp - target) * math.exp(-dt / T_TAU)
        if not on:
            nd.throttled = False
            return
        if not nd.throttled and nd.temp > T_THROTTLE:
            nd.throttled = True
            if not silent:
                self._sel(nd, t, "GPU Thermal", "GPU thermal slowdown asserted (hottest GPU %.0f C)" % nd.temp,
                          "warning")
        elif nd.throttled and nd.temp < T_RELEASE:
            nd.throttled = False
            if not silent:
                self._sel(nd, t, "GPU Thermal", "GPU thermal slowdown deasserted")
        if nd.temp >= T_SHUTDOWN and nd.power in ("booting", "on") and not silent:
            self._incident("thermal_shutdown", t, nd.host, temp_c=_r(nd.temp, 1), airflow=_r(self._airflow(nd.host)))
            self._sel(nd, t, "GPU Thermal", "GPU over-temperature (%.0f C): thermal shutdown" % nd.temp, "critical")
            nd.power, nd.timer, nd.after_shutdown, nd.throttled = "standby", None, None, False
            nd.down_cause = "thermal shutdown"
            self._abort_dcgm(nd, t, "thermal shutdown")

    def _load(self, nd):
        if nd.power == "booting":
            return IDLE_LOAD
        if nd.dcgm:
            return DCGM_LOAD[self.dcgm_runs[nd.dcgm]["level"]]
        if self.sched.state[nd.host] in ("alloc", "drain") and self.job["state"] == "running":
            return JOB_LOAD
        return IDLE_LOAD

    def _airflow(self, h):
        rot, empty = 0.0, 0
        for m in range(1, 6):
            port = "%s/fan%d" % (h, m)
            if self.occ[port] and self.latched[port]:
                rot += sum(self.parts[self.occ[port]].rotors)
            else:
                empty += 1
        return max(0.0, (rot - 2.0 * empty) / 10.0)

    def _fan_status(self, h, m):
        port = "%s/fan%d" % (h, m)
        if not (self.occ[port] and self.latched[port]):
            return "absent"
        rot = self.parts[self.occ[port]].rotors
        if max(rot) <= 0.0:
            return "failed"
        return "degraded" if min(rot) < 1.0 else "ok"

    # ============================================================================================ GPUs
    def _seating(self, port):
        if not self.occ[port]:
            return "empty"
        if not self.latched[port] or not self._fastened(port) or self.armed[port]:
            return "unseated"
        return "seated_fastened" if self.seat_q[port] == "good" else "seated_loose"

    def _gpu_state(self, h, i):
        """What the node sees of GPU index i: ok | missing | remap_failure | unknown (host not running)."""
        if self.nodes[h].power != "on":
            return "unknown"
        port = "%s/%s" % (h, GPU_SLOT[i])
        if self._seating(port) != "seated_fastened" or self.socket_bad[port] or self.slot_fault[port] \
                or self.broken[port]:
            return "missing"
        part = self.parts[self.occ[port]]
        if part.health in ("dead", "connector_damaged") or part.wedged:
            return "missing"
        return "remap_failure" if part.health == "hbm_bad" else "ok"

    def _gpu_slot_ok(self, port):
        """The physical truth of one slot: a healthy module, fastened, on a healthy socket."""
        if self._seating(port) != "seated_fastened" or self.socket_bad[port] or self.slot_fault[port] \
                or self.broken[port]:
            return False
        part = self.parts[self.occ[port]]
        return part.health == "ok" and not part.wedged

    # ============================================================================================ optics and links
    def _path(self, h, k):
        """The parts on link (h, k) as they sit now, or None where it is broken."""
        o, nm, _, _ = link_ports(h, k)
        if not (self.occ[o] and self.latched[o] and not self.broken[o]):
            return None
        if not (self.occ[nm] and self.latched[nm] and not self.broken[nm]):
            return None
        en = self.occ[nm]
        cid = self.cable_of.get(en)
        if cid is None:
            return None
        el = self.cables[cid].other(en)
        lport = self._where().get(el)
        if lport is None or not self.latched[lport] or self.broken[lport]:
            return None
        rec = self.ports[lport]
        if rec["kind"] != "mpo" or "leaf" not in rec:
            return {"wrong": lport}
        losc = rec["needs"]
        if not (self.occ[losc] and self.latched[losc] and not self.broken[losc]):
            return None
        return {"tn": self.occ[o], "en": en, "el": el, "tl": self.occ[losc], "peer": (rec["leaf"], rec["slot"]),
                "lport": lport}

    def _where(self):
        """part id -> the port it is in now (cached per physical version)."""
        if getattr(self, "_where_version", None) != self.version:
            self._where_map = {inst: p for p, inst in self.occ.items() if inst is not None}
            self._where_version = self.version
        return self._where_map

    def _optics_now(self, h, k):
        """Rx power (dBm) per receiver lane at both ends of link (h, k) if both ends were powered: node_rx[r],
        leaf_rx[r]; NO_LIGHT where the path is broken."""
        path = self._path(h, k)
        out = {"complete": False, "peer": None, "node_rx": [NO_LIGHT_DBM] * 4, "leaf_rx": [NO_LIGHT_DBM] * 4,
               "path": path}
        if path is None or "wrong" in path:
            return out
        fn, cn = self.faces.get("rx:" + path["tn"]), self.faces.get("mpo:" + path["en"])
        cl, fl = self.faces.get("mpo:" + path["el"]), self.faces.get("rx:" + path["tl"])
        if any(f is None for f in (fn, cn, cl, fl)) or fn.damaged or fl.damaged:
            return out
        txn, txl = self.parts[path["tn"]].tx, self.parts[path["tl"]].tx
        out["leaf_rx"] = [txn[r] - LINK_LOSS_DB - fn.loss(1 + r) - cn.loss(1 + r) - cl.loss(12 - r) - fl.loss(12 - r)
                          for r in range(4)]
        out["node_rx"] = [txl[r] - LINK_LOSS_DB - fl.loss(1 + r) - cl.loss(1 + r) - cn.loss(12 - r) - fn.loss(12 - r)
                          for r in range(4)]
        out["complete"], out["peer"] = True, path["peer"]
        return out

    def _ensure_optics(self):
        if self._opt_version != self.version:
            self._opt = {(h, k): self._optics_now(h, k) for h in HOSTS for k in range(8)}
            self._opt_version = self.version

    def _worst_margin(self, h, k):
        opt = self._optics_now(h, k)
        if not opt["complete"]:
            return None
        return min(opt["node_rx"] + opt["leaf_rx"]) - RX_SENS_DBM

    def _leaf_powered(self, leaf):
        n = int(leaf[-1])
        out = self.pdu[leaf_rack(leaf)]
        return out["A%d" % n] == "on" or out["B%d" % n] == "on"

    def _is_up(self, ln, t):
        return ln.band == "up" or (ln.band == "flapping" and t >= ln.down_until)

    def _links_eval(self, t):
        self._ensure_optics()
        self._leaf_peer = {}
        for (h, k), ln in self.links.items():
            opt = self._opt[(h, k)]
            peer = opt["peer"] if opt["complete"] else None
            powered = peer is not None and self.nodes[h].power == "on" and self._leaf_powered(peer[0])
            if not powered:
                band = "down"
            else:
                m = min(opt["node_rx"] + opt["leaf_rx"]) - RX_SENS_DBM
                band = "up" if m >= 0.0 else ("flapping" if m >= -2.0 else "down")
            if band != "flapping":
                ln.next_flap, ln.down_until = None, -1.0
            ln.band, ln.peer = band, (peer if band != "down" else None)
            if peer is not None:
                self._leaf_peer[peer] = (h, k)
            up = self._is_up(ln, t)
            if ln.was_up and not up:
                for key in {(h, k + 1), ln.last_peer}:
                    if key is not None:
                        self.ctr[key].link_down_count += 1
                        self.ctr[key].tot_downs += 1
            ln.was_up = up
            if peer is not None:
                ln.last_peer = peer

    def _ep(self, dev, port):
        """(status, peer, link state or None) of fabric port (dev, port)."""
        if dev in NODE_ORDER:
            ln = self.links[(dev, port - 1)]
            if ln.band == "down":
                return "down", None, ln
            return ln.band, ln.peer, ln
        hk = self._leaf_peer.get((dev, port))
        if hk is None:
            return "down", None, None
        ln = self.links[hk]
        if ln.band == "down":
            return "down", None, ln
        return ln.band, (hk[0], hk[1] + 1), ln

    # ============================================================================================ scheduler and job
    def _evaluate(self, t):
        for h in HOSTS:
            self._power_eval(self.nodes[h], t)
        self._links_eval(t)
        self._sched_eval(t)
        self._sel_status(t)

    def _sel_status(self, t):
        for h in HOSTS:
            nd = self.nodes[h]
            if nd.power == "off" or not nd.psu_logged:
                continue
            for s in range(1, 7):
                st = self._psu_status(h, s)
                old = nd.psu_logged.get(s)
                if st != old:
                    msg = {"absent": ("Presence deasserted (removed)", "warning"),
                           "ac_lost": ("AC input lost", "warning"), "failure": ("Failure detected", "critical"),
                           "ok": ("AC input restored" if old == "ac_lost" else "Presence detected, output OK", "info")}
                    if old == "absent" and st != "absent":
                        self._sel(nd, t, "PSU%d Status" % s, "Presence detected (inserted)")
                    if not (old == "absent" and st == "ok"):
                        self._sel(nd, t, "PSU%d Status" % s, *msg[st])
                    nd.psu_logged[s] = st
            red, n_ok = self._redundancy(h)
            if red != nd.red_logged:
                self._sel(nd, t, "Power Unit Redundancy", {"full": "Fully redundant (6 of 6 PSUs OK)",
                                                           "degraded": "Redundancy degraded (%d of 6 PSUs OK)" % n_ok,
                                                           "lost": "Redundancy lost (%d of 6 PSUs OK)" % n_ok}[red],
                          "info" if red == "full" else "warning")
            nd.red_logged = red
            if nd.power not in ON_STATES:
                continue
            for m in range(1, 6):
                st = self._fan_status(h, m)
                old = nd.fan_logged.get(m)
                if st != old:
                    msg = {"absent": ("Fan module removed", "warning"), "failed": ("Fan failure: 0 RPM", "critical"),
                           "degraded": ("Rotor below lower non-critical threshold", "warning"),
                           "ok": ("Fan module inserted, speed normal" if old == "absent" else "Speed normal", "info")}
                    self._sel(nd, t, "FAN%d" % m, *msg[st])
                    nd.fan_logged[m] = st

    def _redundancy(self, h):
        n_ok = sum(self._psu_status(h, s) == "ok" for s in range(1, 7))
        return ("full" if n_ok == 6 else ("degraded" if n_ok >= 4 else "lost")), n_ok

    def _job_loss(self, h):
        nd = self.nodes[h]
        if nd.power != "on":
            return nd.down_cause or ("node %s" % nd.power)
        for i in range(8):
            if self._gpu_state(h, i) != "ok":
                return "GPU %d lost (%s)" % (i, pci_addr(i))
        for k in range(8):
            if self.links[(h, k)].band in ("down", "flapping"):
                return "rail %d link %s" % (k, self.links[(h, k)].band)
        return None

    def _health_issues(self, h):
        """The node health check (what NHC runs before a node may take work)."""
        nd = self.nodes[h]
        if nd.power != "on":
            return ["node not running (power %s)" % nd.power]
        out = []
        for i in range(8):
            st = self._gpu_state(h, i)
            if st == "missing":
                out.append("check_hw_gpu: GPU %d (%s) not found" % (i, pci_addr(i)))
            elif st == "remap_failure":
                out.append("check_hw_gpu: GPU %d (%s) row remapping failure" % (i, pci_addr(i)))
        for k in range(8):
            ln = self.links[(h, k)]
            if ln.band != "up":
                out.append("check_hw_ib: mlx5_%d (rail %d) %s" % (k, k, ln.band))
            elif ln.peer != (RAIL_LEAF[k], NODE_ORDER[h]):
                out.append("check_hw_ib: mlx5_%d (rail %d) cabled to %s port %d" % (k, k, ln.peer[0], ln.peer[1]))
        if nd.brake:
            out.append("check_hw_power: power brake asserted (3 of 6 PSUs OK)")
        if nd.throttled or nd.temp >= T_THROTTLE:
            out.append("check_hw_thermal: GPU temperature %.0f C (slowdown)" % nd.temp)
        return out

    def _set_state(self, h, state, t, reason=""):
        self.sched.set(h, state, t, reason)

    def _members(self):
        return [h for h in HOSTS if self.sched.state[h] in ("alloc", "drain")]

    def _sched_eval(self, t):
        job = self.job
        if job["state"] == "running":
            failed = {}
            for h in self._members():
                why = self._job_loss(h)
                if why:
                    failed[h] = why
            if failed:
                self._job_crash(t, failed)
        elif job["state"] == "restarting":
            for h in self._members():
                why = self._job_loss(h)
                if why:
                    self._set_state(h, "down", t, "job restart: " + why)
        for h in HOSTS:
            if self.sched.state[h] == "idle":
                issues = self._health_issues(h)
                if issues:
                    self._set_state(h, "down", t, "NHC: " + issues[0])

    def _job_crash(self, t, failed):
        job = self.job
        members = self._members()
        gpus = 8 * len(members)
        since = max(0.0, t - max(job["last_cp"], job["running_since"]))
        lost = since * gpus
        first = sorted(failed)[0]
        self._incident("job_crash", t, first, nodes=sorted(failed), cause=dict(sorted(failed.items())),
                       since_checkpoint_s=_r(since, 1), gpus_in_job=gpus, lost_gpu_s=_r(lost, 0))
        job["crashes"] += 1
        job["lost_gpu_s"] += lost
        job["since_cp_work"] = 0.0
        for h, why in failed.items():
            self._set_state(h, "down", t, "job crash: " + why)
        for h in members:
            if h not in failed and self.sched.state[h] == "drain":
                self._set_state(h, "drained", t, self.sched.reason[h])
        job["state"], job["restart_at"] = "restarting", t + RESTART_S

    def _restart(self, t):
        job = self.job
        for h in self._members():
            why = self._job_loss(h)
            if why:
                self._set_state(h, "down", t, "job restart: " + why)
        job["restart_at"] = None
        if self._members():
            job["state"], job["running_since"] = "running", t
        else:
            job["state"] = "stalled"

    def _checkpoint(self, tc):
        job = self.job
        if job["state"] == "running":
            job["saved_work"] += job["since_cp_work"]
            job["since_cp_work"] = 0.0
            job["last_cp"] = tc
        for h in HOSTS:
            if self.sched.state[h] == "drain":
                self._set_state(h, "drained", tc, self.sched.reason[h])
        for h in HOSTS:
            if self.sched.state[h] == "idle":
                if self._health_issues(h) or self.nodes[h].dcgm:
                    self._set_state(h, "down", tc, "NHC at join: " + "; ".join(self._health_issues(h)[:1]))
                else:
                    self._set_state(h, "alloc", tc, "")
        if job["state"] == "stalled" and self._members():
            job["state"], job["running_since"], job["last_cp"] = "running", tc, tc

    # ============================================================================================ mid-shift
    def _pending(self, kind, data, t):
        mid = self.scn["mid"]
        h = mid["host"]
        if kind == "mid_fault":
            if mid["kind"] == "fan_failed":
                inst = self.occ["%s/fan%d" % (h, mid["fan"])] or "%s/fan%d" % (h, mid["fan"])
                part = self.parts[inst]
                part.health, part.rotors = "failed", [0.0, 0.0]
            else:
                spec = {"host": h, "k": mid["k"], "cause": mid["cause"], "target_margin": mid["target_margin"],
                        "lane": mid["lane"]}
                self._place_link_fault(spec, np.random.default_rng(_stable_seed(self.seed, "midlink")))
                mid["applied"] = spec["applied"]
                ctr0 = self.ctr[(h, mid["k"] + 1)]
                leaf_ep = (RAIL_LEAF[mid["k"]], NODE_ORDER[h])
                mid["err0"] = ctr0.tot_errors + self.ctr[leaf_ep].tot_errors
            self.version += 1
        elif kind == "mid_ticket":
            if mid["kind"] == "fan_failed":
                nd = self.nodes[h]
                m = mid["fan"]
                loc = host_location(h)
                summary = "%s: fan module %d failed (0 RPM) - GPU thermal %s" % (
                    h, m, "slowdown" if nd.throttled else "warning")
                lines = ["Alert source: BMC %s at %s" % (h, _label(t)),
                         "FAN%d_F: 0 RPM, FAN%d_R: 0 RPM (lower critical)" % (m, m),
                         "GPU temperature: hottest GPU %.0f C (slowdown at %.0f C)%s"
                         % (nd.temp, T_THROTTLE, ", thermal slowdown asserted" if nd.throttled else ""),
                         "Node %s: %s in job %d (%s)" % (h, self.sched.state[h].upper(), JOB_ID, JOB_NAME),
                         "Location: %s, front, the GPU tray's fan wall: fan module %d of 5" % (loc, m),
                         "Recommended procedure: %s (fan module hot swap, 30 s limit)" % SOP["fan"]]
                fields = {"host": h, "location": loc, "component": "FAN%d" % m, "gpu_temp_c": _r(nd.temp, 1),
                          "sop": SOP["fan"]}
                self._add_ticket("fan", h, t, t, "P1", summary, lines, fields, {"cause": "fan_failed", "fan": m,
                                                                               "mid": True})
            else:
                k = mid["k"]
                leaf_ep = (RAIL_LEAF[k], NODE_ORDER[h])
                n_err = max(1, self.ctr[(h, k + 1)].tot_errors + self.ctr[leaf_ep].tot_errors - mid.get("err0", 0))
                summary, lines, fields = self._link_ticket_lines(h, k, t, n_err, t - mid["onset"], _label(t))
                self._add_ticket("link", h, t, t, "P2", summary, lines, fields,
                                 {"cause": mid["cause"], "rail": k, "applied": mid.get("applied"), "mid": True})

    # ============================================================================================ incidents
    def _incident(self, itype, t, host=None, **detail):
        rec = {"type": itype, "time": _r(t), "host": host}
        rec.update(detail)
        self.incidents.append(rec)

    # ============================================================================================ cleaning, scope
    def _resolve_face(self, face_id):
        """A face id from the cell -> Face. "rx:<X>": X a transceiver port with a module in it -> that module's
        receptacle (cell.faces() names a seated module's receptacle by its port), else X = the module's id."""
        kind, _, x = str(face_id).partition(":")
        if kind == "mpo":
            return self.faces.get("mpo:" + x)
        if kind == "rx":
            if "rx:" + x in self.faces:              # a module's id (the cell names receptacles by their module)
                return self.faces["rx:" + x]
            if x in self.ports and self.ports[x]["kind"] in ("osfp_n", "osfp_s") and self.occ.get(x):
                return self.faces.get("rx:" + self.occ[x])
        return None

    def _face_link(self, face):
        part = self.parts.get(face.instance)
        if part is None:
            return None
        if part.kind == "mpo":
            cab = self.cables[part.cable]
            return cab.host, cab.k
        port = self._where().get(part.id) or part.last_port
        return port_link(port) if port else None

    def on_clean(self, face_id, t):
        """One cleaner click on a face: each particle goes with p = 0.8, films halve (gone below 0.1), scratches and
        pits stay; 3 % of clicks leave a new speck in a cladding zone."""
        t = max(float(t), self.t)
        key = (str(face_id), round(t, 3))
        if key == self._last_click:
            return
        self._last_click = key
        self._catch_up(t)
        self._advance(t)
        face = self._resolve_face(face_id)
        if face is None:
            self.unknown.append(("clean", face_id, _r(t)))
            return
        before = len(face.defects)
        face.clean(self.rng_clean)
        self.cleanings.append({"t": _r(t), "face": face.id, "event": "click", "defects_before": before,
                               "defects_after": len(face.defects)})
        ln = self._face_link(face)
        if ln is not None:
            self.interventions.append((round(t, 2), "link", ln, face.id))
        self.version += 1
        self._evaluate(t)

    def scope_image(self, face_id, lateral_m, angle_rad, t=None):
        """The fibre inspection probe's picture of an MPO-12 end face: (480, 640, 3) uint8. The capture is recorded
        (cl.last_capture["id"] is the id a technician quotes as evidence)."""
        face = self._resolve_face(face_id)
        lat_mm = abs(float(lateral_m)) * 1000.0
        ang = abs(math.degrees(float(angle_rad)))
        cid = "SCOPE-%04d" % (len(self.captures) + 1)
        rec = {"id": cid, "face": str(face_id), "resolved": face.id if face else None,
               "instance": face.instance if face else None, "t": _r(t if t is not None else self.t),
               "lateral_mm": _r(lat_mm, 3), "angle_deg": _r(ang, 2), "sharp": bool(lat_mm <= 0.5 and ang <= 3.0),
               "fibres_visible": bool(face is not None and lat_mm <= 1.5),
               "defects": face.summary() if face else None}
        img = self._render_scope(face, lat_mm, ang, str(face_id), len(self.captures))
        self.captures.append(rec)
        self.last_capture = rec
        return img

    def _render_scope(self, face, lat_mm, ang_deg, face_id, n):
        W, Hh, M = SCOPE_W, SCOPE_H, SCOPE_MARGIN
        rs = np.random.default_rng(_stable_seed(self.seed, "scope_tex", face.id if face else face_id))
        rn = np.random.default_rng(_stable_seed(self.seed, "scope_noise", face_id, n))
        base = (84.0, 87.0, 93.0) if (face is None or face.kind == "mpo") else (70.0, 73.0, 80.0)
        canvas = self._ferrule(rs, Hh, W + 2 * M, base)
        if face is None or lat_mm > 1.5:
            img = Image.fromarray(np.clip(canvas[:, M:M + W], 0, 255).astype(np.uint8)).filter(
                ImageFilter.GaussianBlur(14))
            arr = np.asarray(img).astype(np.float64)
        else:
            cy = Hh / 2.0
            for pos in range(1, 13):
                cx = M + W / 2.0 + (pos - 6.5) * PITCH_UM / SCOPE_UM_PX
                self._draw_fibre(canvas, cx, cy, [d for d in face.defects if d["fibre"] == pos], face.kind)
            shift, sigma = 0.0, 0.0
            if lat_mm > 0.5:
                f = min(1.0, (lat_mm - 0.5) / 1.0)
                shift, sigma = f * W / 2.0, 6.0 * f
            if ang_deg > 3.0:
                sigma = math.hypot(sigma, min(6.0, 2.0 * (ang_deg - 3.0)))
            x0 = int(round(M - shift))
            arr = canvas[:, x0:x0 + W]
            if sigma > 0.3:
                img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(sigma))
                arr = np.asarray(img).astype(np.float64)
        yy, xx = np.mgrid[0:Hh, 0:W]
        vig = 1.0 - 0.28 * (((xx - W / 2) / (W / 2)) ** 2 + ((yy - Hh / 2) / (W / 2)) ** 2)
        arr = arr * vig[..., None]
        arr = arr + rn.normal(0.0, 2.6, (Hh, W, 1)) + rn.normal(0.0, 0.8, (Hh, W, 3))
        img = Image.fromarray(np.clip(np.round(arr), 0, 255).astype(np.uint8))
        dr = ImageDraw.Draw(img)
        fnt = _font(11)
        col = (232, 228, 150)
        for pos in range(1, 13):                       # the probe software's overlay (fixed on the screen)
            cx = W / 2.0 + (pos - 6.5) * PITCH_UM / SCOPE_UM_PX
            s = str(pos)
            tw = dr.textlength(s, font=fnt)
            dr.text((cx - tw / 2.0, Hh / 2.0 + CLAD_R_UM / SCOPE_UM_PX + 9), s, fill=col, font=fnt)
        bar = 250.0 / SCOPE_UM_PX
        dr.line([(W - 24 - bar, Hh - 22), (W - 24, Hh - 22)], fill=col, width=2)
        dr.text((W - 24 - bar, Hh - 38), "250 um", fill=col, font=fnt)
        return np.asarray(img, dtype=np.uint8).copy()

    @staticmethod
    def _ferrule(rs, h, w, base):
        """The polished MT ferrule: grey, mottled, with bright specks of glass filler."""
        bil = getattr(Image, "Resampling", Image).BILINEAR
        low = rs.random((h // 24 + 2, w // 24 + 2))
        low = np.asarray(Image.fromarray((low * 255).astype(np.uint8)).resize((w, h), bil), dtype=np.float64) / 255.0
        mid = rs.random((h // 6 + 2, w // 6 + 2))
        mid = np.asarray(Image.fromarray((mid * 255).astype(np.uint8)).resize((w, h), bil), dtype=np.float64) / 255.0
        lum = (low - 0.5) * 16.0 + (mid - 0.5) * 7.0 + rs.normal(0.0, 2.0, (h, w))
        specks = rs.random((h, w)) < 0.0035
        lum = lum + specks * rs.uniform(10.0, 34.0, (h, w))
        return np.stack([base[0] + lum, base[1] + lum, base[2] + lum * 1.04], axis=-1)

    @staticmethod
    def _draw_fibre(canvas, cx, cy, defects, kind, ss=4):
        """One fibre end at canvas (cx, cy) px, drawn 4x supersampled: the epoxy ring, the bright cladding, the
        slightly darker core, then its defects (particles and pits dark, films smeared grey, scratches thin lines)."""
        half = 18
        x0, y0 = int(round(cx)) - half, int(round(cy)) - half
        n = 2 * half
        bg = canvas[y0:y0 + n, x0:x0 + n]
        img = np.repeat(np.repeat(bg, ss, axis=0), ss, axis=1).copy()
        um = SCOPE_UM_PX
        u = ((np.arange(n * ss) + 0.5) / ss + x0 - cx) * um
        v = ((np.arange(n * ss) + 0.5) / ss + y0 - cy) * um
        X, Y = np.meshgrid(u, v)
        R = np.hypot(X, Y)
        aa = um / ss

        def cov(dist, rad):
            return np.clip((rad - dist) / aa + 0.5, 0.0, 1.0)

        def mix(a, colour):
            nonlocal img
            img = img * (1.0 - a[..., None]) + np.asarray(colour, np.float64)[None, None, :] * a[..., None]

        mix(cov(R, CLAD_R_UM + 4.5) * (1.0 - cov(R, CLAD_R_UM)) * 0.8, (34, 34, 38))
        clad = (206.0, 210.0, 214.0) if kind == "mpo" else (196.0, 201.0, 207.0)
        shade = (1.0 - 0.07 * (R / CLAD_R_UM) ** 2)[..., None] * np.asarray(clad)[None, None, :]
        a = cov(R, CLAD_R_UM)
        img = img * (1.0 - a[..., None]) + shade * a[..., None]
        mix(cov(R, CORE_R_UM) * 0.85, (clad[0] - 22, clad[1] - 21, clad[2] - 19))
        glass = cov(R, CLAD_R_UM + 1.0)
        for d in defects:
            if d["type"] in ("particle", "pit"):
                rp = d["d"] / 2.0
                r_eff = max(rp, 3.0)
                op = min(1.0, (rp / r_eff) ** 2) * (0.92 if d["type"] == "particle" else 0.75)
                dist = np.hypot(X - d["x"], Y - d["y"])
                core_a = cov(dist, r_eff) * op
                halo = cov(dist, r_eff * 1.9) * (1.0 - cov(dist, r_eff)) * 0.22 * op
                mix(halo, (60, 58, 56))
                mix(core_a, (30, 29, 27) if d["type"] == "particle" else (52, 50, 50))
            elif d["type"] == "film":
                blob = np.zeros_like(X)
                for dx, dy, s, w in d["lobes"]:
                    blob += w * np.exp(-((X - d["x"] - dx) ** 2 + (Y - d["y"] - dy) ** 2) / (2.0 * s * s))
                blob = np.clip(blob / max(1e-6, max(w for *_, w in d["lobes"])), 0.0, 1.0)
                mix(blob * min(1.0, d["strength"]) * 0.62 * glass, (128, 122, 110))
            elif d["type"] == "scratch":
                ca, sa = math.cos(d["angle"]), math.sin(d["angle"])
                dist = np.abs(-X * sa + Y * ca - d["offset"])
                mix(cov(dist, d["width"] / 2.0) * 0.82 * glass, (44, 44, 48))
        canvas[y0:y0 + n, x0:x0 + n] = img.reshape(n, ss, n, ss, 3).mean(axis=(1, 3))

    # ============================================================================================ LEDs
    def leds(self, t):
        """Every LED the cell draws -> "off" | "green" | "amber" | "blue" | "cyan" (blinking computed from t)."""
        t = float(t)
        if t > self.t:
            self._advance(t)
            self._evaluate(t)
        out = {}
        b1 = (t % 1.0) < 0.5
        b2 = (t % 0.5) < 0.25
        b4 = (t % 0.25) < 0.125
        for h in HOSTS:
            nd = self.nodes[h]
            bmc = nd.power != "off"
            out[h + "_pwr"] = {"on": "green", "shutting_down": "green", "booting": "green" if b4 else "off",
                               "standby": "green" if b1 else "off", "off": "off"}[nd.power]
            out[h + "_id"] = ("blue" if nd.identify == "on" or (nd.identify == "blink" and b2) else "off") if bmc \
                else "off"
            fault = bmc and bool(self._faults(h))
            out[h + "_fault"] = "amber" if fault else "off"
            out[h + "_bar"] = ("amber" if fault else "cyan") if nd.power in ("on", "shutting_down") else "off"
            for s in range(1, 7):
                st = self._psu_status(h, s)
                out["%s_psu%d" % (h, s)] = {"ok": "green", "failure": "amber"}.get(st, "off")
            for m in range(1, 6):
                st = self._fan_status(h, m)
                if nd.power not in ON_STATES or st == "absent":
                    out["%s_fan%d" % (h, m)] = "off"
                else:
                    out["%s_fan%d" % (h, m)] = "green" if st == "ok" else "amber"
            for k in range(8):
                out["%s_link%d" % (h, k + 1)] = self._ep_led(h, k + 1, t, b2)
        for leaf in LEAVES:
            on = self._leaf_powered(leaf)
            for p in range(1, 9):
                out["%s_p%d" % (leaf, p)] = self._ep_led(leaf, p, t, b2) if on else "off"
            for p in range(17, 25):
                out["%s_p%d" % (leaf, p)] = "green" if on else "off"
        return out

    def _ep_led(self, dev, port, t, blink):
        if self.locate.get((dev, port)):
            return "blue" if blink else "off"
        st, _, _ = self._ep(dev, port)
        if st == "down":
            return "off"
        if st == "flapping" or t - self.ctr[(dev, port)].last_error_t < 60.0:
            return "amber"
        return "green"

    def _faults(self, h):
        """The node's asserted faults (what lights its amber LEDs)."""
        nd = self.nodes[h]
        out = []
        for s in range(1, 7):
            if self._psu_status(h, s) in ("failure", "ac_lost"):
                out.append("PSU%d %s" % (s, self._psu_status(h, s)))
        if nd.power in ON_STATES:
            for m in range(1, 6):
                if self._fan_status(h, m) in ("failed", "degraded"):
                    out.append("FAN%d %s" % (m, self._fan_status(h, m)))
        if nd.power == "on":
            for i in range(8):
                if self._gpu_state(h, i) != "ok":
                    out.append("GPU%d %s" % (i, self._gpu_state(h, i)))
        if nd.throttled:
            out.append("thermal slowdown")
        if nd.brake:
            out.append("power brake")
        if nd.post_error:
            out.append("POST error")
        if not self.tray_latched[h]:
            out.append("GPU tray not seated")
        return out

    # ============================================================================================ the services
    def api_cost_s(self, method):
        """Sim seconds a call takes: reads 0.5 s, writes 1.0 s, starting a diagnostic or a soak 1.0 s."""
        return 1.0 if (method in API_WRITE or method in API_START) else 0.5

    def api(self, method, params=None, t=0.0):
        """The site services (NOC, DCIM, BMC, PDU, fabric manager, scheduler, DCGM): plain JSON-able dicts."""
        t = max(float(t), self.t)
        self._catch_up(t)
        self._advance(t)
        fn = getattr(self, "_api_" + method.replace(".", "_"), None) \
            if isinstance(method, str) and method.count(".") == 1 else None
        if fn is None:
            self.api_log.append((_r(t), str(method), "unknown_method"))
            return {"error": "unknown_method"}
        if params is not None and not isinstance(params, dict):
            return {"error": "bad_params", "detail": "params must be an object"}
        try:
            out = fn(dict(params or {}), t)
        except BadParams as e:
            out = {"error": "bad_params", "detail": str(e)}
        if method in API_WRITE or method in API_START:
            self.version += 1
            self._evaluate(t)
        self.api_log.append((_r(t), method, out.get("error", "ok") if isinstance(out, dict) else "ok"))
        return out

    # ---- parameter helpers
    @staticmethod
    def _p(params, key, choices=None, typ=str):
        if key not in params:
            raise BadParams("missing '%s'" % key)
        v = params[key]
        if typ is int:
            if isinstance(v, bool) or not isinstance(v, (int, float, str)):
                raise BadParams("'%s' must be an integer" % key)
            try:
                v = int(v)
            except ValueError:
                raise BadParams("'%s' must be an integer" % key)
        elif typ is str and not isinstance(v, str):
            raise BadParams("'%s' must be a string" % key)
        if choices is not None and v not in choices:
            raise BadParams("'%s' must be one of %s" % (key, sorted(choices) if len(choices) < 20 else "the site's"))
        return v

    def _host(self, params):
        return self._p(params, "host", HOSTS)

    def _bmc_node(self, params):
        h = self._host(params)
        nd = self.nodes[h]
        if nd.power == "off" or self.pdu[nd.rack]["A6"] != "on":
            return h, None
        return h, nd

    @staticmethod
    def _bmc_down(h):
        return {"error": "bmc_unreachable", "detail": "no response from %s-bmc" % h}

    def _hnoise(self, *keys):
        return np.random.default_rng(_stable_seed(self.seed, *keys))

    # ---- NOC
    def _api_noc_tickets(self, params, t):
        show_all = bool(params.get("all", False))
        out = [tk.public() for tk in self.tickets.values()
               if tk.visible_at <= t + 1e-9 and (show_all or tk.status != "resolved")]
        return {"tickets": out, "time": _label(t)}

    def _ticket_arg(self, params, t):
        tid = self._p(params, "id")
        tk = self.tickets.get(tid)
        if tk is None or tk.visible_at > t + 1e-9:
            raise BadParams("no ticket %s" % tid)
        return tk

    def _api_noc_ticket(self, params, t):
        return self._ticket_arg(params, t).public(full=True)

    def _api_noc_update(self, params, t):
        tk = self._ticket_arg(params, t)
        status = self._p(params, "status", ("in_progress", "resolved", "escalated"))
        notes = params.get("notes", "")
        if not isinstance(notes, str):
            raise BadParams("'notes' must be a string")
        rc = params.get("root_cause")
        if rc is not None and rc not in ROOT_CAUSE_CODES:
            raise BadParams("'root_cause' must be one of %s" % list(ROOT_CAUSE_CODES))
        parts = params.get("parts", [])
        if not isinstance(parts, list) or not all(isinstance(p, dict) for p in parts):
            raise BadParams("'parts' must be a list of {removed, installed, slot}")
        clean_parts = []
        for p in parts:
            for key in ("removed", "installed", "slot"):
                if key in p and p[key] is not None and not isinstance(p[key], (str, int)):
                    raise BadParams("parts.%s must be a string" % key)
            clean_parts.append({k: (str(p[k]) if p.get(k) is not None else None) for k in ("removed", "installed",
                                                                                          "slot")})
        tk.status = status
        tk.updates.append({"time": _r(t), "timestamp": _label(t), "status": status, "notes": notes, "root_cause": rc,
                           "parts": clean_parts})
        return {"ok": True, "id": tk.id, "status": status}

    def _api_noc_escalate(self, params, t):
        tk = self._ticket_arg(params, t)
        to = self._p(params, "to", ("vendor", "cabling"))
        ev = params.get("evidence", [])
        if not isinstance(ev, list) or not all(isinstance(e, str) for e in ev):
            raise BadParams("'evidence' must be a list of strings")
        notes = params.get("notes", "")
        tk.status = "escalated"
        tk.escalations.append({"time": _r(t), "timestamp": _label(t), "to": to, "evidence": list(ev),
                               "notes": notes if isinstance(notes, str) else ""})
        return {"ok": True, "id": tk.id, "status": "escalated", "to": to}

    def _api_noc_create(self, params, t):
        h = params.get("host")
        if h is not None and h not in HOSTS and h not in LEAVES:
            raise BadParams("unknown host")
        summary = self._p(params, "summary")
        notes = params.get("notes", "")
        component = params.get("component", "")
        tk = self._add_ticket("report", h, t, t, "P4", summary, [notes if isinstance(notes, str) else ""],
                              {"host": h, "component": component if isinstance(component, str) else "",
                               "raised_by": "site technician"}, {"cause": None})
        return {"ok": True, "id": tk.id}

    # ---- DCIM
    def _api_dcim_racks(self, params, t):
        out = []
        for row in H.ROWS:
            for idx in range(1, H.N_RACKS + 1):
                name = H.rack_name(row, idx)
                rec = {"rack": name, "row": row, "position": idx, "front_faces": "south" if row == "A" else "north",
                       "size": "48U 600x1200 mm"}
                if H.is_network(idx):
                    rec.update({"role": "network", "devices": [lf for lf in LEAVES if leaf_rack(lf) == name]})
                else:
                    rec.update({"role": "gpu", "devices": ["gpn-%s%02d" % (row.lower(), idx)]})
                out.append(rec)
        return {"racks": out}

    def _api_dcim_elevation(self, params, t):
        rack = self._p(params, "rack", GPU_RACKS + NET_RACKS)
        devs = []
        if rack in NET_RACKS:
            row = rack[0]
            for k in range(4):
                lf = H.leaf_name(row, k)
                devs.append({"u": "U%d" % H.LEAF_U[k], "device": lf, "model": "KX-L32R (32-port 400G leaf)",
                             "serial": "KXL-%s-%04d" % (rack, 3100 + 17 * k), "mounted": "rear (ports to hot aisle)",
                             "rail": LEAF_RAIL[lf]})
            for u in H.PATCH_U:
                devs.append({"u": "U%d" % u, "device": "patch-%s-%d" % (rack.lower(), u), "model": "1U MPO patch panel",
                             "mounted": "front"})
        else:
            h = "gpn-%s" % rack.lower()
            row, idx = rack[0].lower(), int(rack[1:])
            devs.append({"u": "U%d-U%d" % H.NODE_U, "device": h, "model": "KX-8H (8x SXM-H8-80G)",
                         "serial": self._chassis_serial(h), "asset": "DC1-%s%02d-%04d" % (rack[0], idx, 4400 + idx),
                         "mounted": "front"})
            for k, u in enumerate(H.AUX_2U):
                devs.append({"u": "U%d-U%d" % (u, u + 1), "device": "stor-%s%02d-%d" % (row, idx, k + 1),
                             "model": "KX-2S storage node", "mounted": "front"})
            devs.append({"u": "U%d" % H.MGMT_U, "device": "mgmt-%s%02d" % (row, idx),
                         "model": "KX-M48 1G management switch", "mounted": "front"})
        return {"rack": rack, "units": H.RACK_UNITS, "devices": devs, "blanks": "all other units"}

    def _api_dcim_cables(self, params, t):
        out = []
        for cab in self.cables.values():
            out.append({"label": cab.label, "serial": cab.serial, "type": MODELS["mpo"], "length_m": cab.length_m,
                        "rail": cab.k,
                        "a": {"device": cab.host, "port": cab.k + 1, "hca": "mlx5_%d" % cab.k,
                              "location": "%s rear, OSFP port %d" % (host_rack(cab.host), cab.k + 1)},
                        "b": {"device": cab.leaf, "port": cab.leaf_port,
                              "location": "%s U%d rear, port %d" % (leaf_rack(cab.leaf), LEAF_UNIT[cab.leaf],
                                                                    cab.leaf_port)}})
        return {"cables": out, "polarity": "Type-B (key-up to key-up): fibre f at one end is fibre 13-f at the other"}

    def _api_dcim_assets(self, params, t):
        dev = self._p(params, "host", HOSTS + LEAVES)
        home = lambda port: self.parts[next(pid for pid, pt in self.parts.items() if pt.home == port)]
        if dev in LEAVES:
            return {"device": dev, "osfp": [{"port": p, "serial": home("%s/osfp%d" % (dev, p)).serial,
                                              "model": MODELS["osfp_s"]} for p in range(1, 9)],
                    "as_of": "shift start"}
        h = dev
        return {"host": h, "chassis": {"model": "KX-8H", "serial": self._chassis_serial(h)}, "as_of": "shift start",
                "psu": [{"slot": "PSU%d" % s, "serial": home("%s/psu%d" % (h, s)).serial, "model": MODELS["psu"],
                         "feed": "A" if s % 2 else "B"} for s in range(1, 7)],
                "fan": [{"slot": "FAN%d" % m, "serial": home("%s/fan%d" % (h, m)).serial, "model": MODELS["fan"]}
                        for m in range(1, 6)],
                "gpu": [{"slot": s.upper(), "serial": home("%s/%s" % (h, s)).serial, "model": MODELS["gpu"]}
                        for s in SLOTS],
                "osfp": [{"port": k, "serial": home("%s/osfp%d" % (h, k)).serial, "model": MODELS["osfp_n"]}
                         for k in range(1, 9)],
                "power_cords": [{"psu": "PSU%d" % s, "serial": home("%s/plug%d" % (h, s)).serial} for s in range(1, 7)]}

    def _outlet_load(self, rack, outlet):
        """What an outlet feeds: (device, input) or None."""
        f, n = outlet[0], int(outlet[1:])
        if rack in NET_RACKS:
            if n <= 4:
                return (H.leaf_name(rack[0], n - 1), "PSU%d" % (1 if f == "A" else 2))
            return None
        h, row, idx = "gpn-%s" % rack.lower(), rack[0].lower(), int(rack[1:])
        if n <= 3:
            return (h, "PSU%d" % (2 * n - 1 if f == "A" else 2 * n))
        if n in (4, 5):
            return ("stor-%s%02d-%d" % (row, idx, n - 3), "PSU%d" % (1 if f == "A" else 2))
        return ("mgmt-%s%02d" % (row, idx), "PSU") if f == "A" else None

    def _api_dcim_pdu(self, params, t):
        rack = self._p(params, "rack", GPU_RACKS + NET_RACKS)
        out = []
        for f in "AB":
            for n in range(1, 7):
                ld = self._outlet_load(rack, "%s%d" % (f, n))
                out.append({"outlet": "%s%d" % (f, n), "feed": f,
                            "feeds": ("%s %s" % ld) if ld else "spare (nothing plugged)"})
        return {"rack": rack, "pdus": {"A": "rear, left side seen from the hot aisle", "B": "rear, right side"},
                "outlets": out}

    # ---- PDU
    def _outlet_watts(self, rack, outlet, t):
        if self.pdu[rack][outlet] != "on":
            return 0.0
        ld = self._outlet_load(rack, outlet)
        if ld is None:
            return 0.0
        dev, inp = ld
        if dev in NODE_ORDER:
            s = int(inp[3:])
            nd = self.nodes[dev]
            if not self._psu_ac(dev, s):
                return 0.0
            if not self._psu_good(dev, s):
                return 6.0
            return self._node_watts(nd) / max(1, nd.n_good)
        if dev.startswith("leaf"):
            return 800.0
        if dev.startswith("stor"):
            return 410.0
        return 150.0

    def _node_watts(self, nd):
        if nd.power == "off":
            return 0.0
        if nd.power == "standby":
            return NODE_W["standby"]
        L = nd.load_eff
        return NODE_W["idle"] + (NODE_W["full"] - NODE_W["idle"]) * max(0.0, L - IDLE_LOAD) / (1.0 - IDLE_LOAD)

    def _api_pdu_outlets(self, params, t):
        rack = self._p(params, "rack", GPU_RACKS + NET_RACKS)
        rng = self._hnoise("pdu", rack, int(t))
        out = {}
        for f in "AB":
            lst = []
            for n in range(1, 7):
                o = "%s%d" % (f, n)
                w = self._outlet_watts(rack, o, t)
                w = w * (1.0 + float(rng.normal(0, 0.01))) if w > 0 else 0.0
                lst.append({"outlet": o, "state": self.pdu[rack][o], "power_w": round(w),
                            "current_a": _r(w / 225.4, 2)})
            out[f] = lst
        return {"rack": rack, "feeds": out}

    def _api_pdu_set(self, params, t):
        rack = self._p(params, "rack", GPU_RACKS + NET_RACKS)
        outlet = self._p(params, "outlet", tuple(f + str(n) for f in "AB" for n in range(1, 7)))
        state = self._p(params, "state", ("on", "off"))
        old = self.pdu[rack][outlet]
        self.pdu[rack][outlet] = state
        ld = self._outlet_load(rack, outlet)
        if state == "off" and old == "on" and ld is not None and ld[0] not in NODE_ORDER:
            self._incident("wrong_outlet", t, ld[0], rack=rack, outlet=outlet, feeds="%s %s" % ld)
        if ld is not None and ld[0] in NODE_ORDER and old != state:
            self.interventions.append((round(t, 2), "host", ld[0], "pdu:%s/%s" % (rack, outlet)))
        return {"ok": True, "rack": rack, "outlet": outlet, "state": state}

    # ---- BMC
    def _api_bmc_power_state(self, params, t):
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        return {"host": h, "state": nd.power}

    def _api_bmc_power(self, params, t):
        action = self._p(params, "action", ("on", "off", "cycle", "forceoff"))
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        if action == "on":
            if nd.power == "shutting_down":
                return {"error": "busy", "detail": "a graceful shutdown is in progress"}
            if nd.power == "standby" and nd.timer is None:
                self._sel(nd, t, "System Event", "Power on requested (BMC)")
                self._start_boot(nd, t)
            return {"ok": True, "host": h, "state": nd.power}
        if action == "off":
            if nd.power in ("booting", "on"):
                self._sel(nd, t, "System Event", "Graceful shutdown requested (BMC)")
                nd.power, nd.timer, nd.after_shutdown = "shutting_down", (t + SHUTDOWN_S, "standby"), None
                nd.down_cause = "BMC power off"
                self._abort_dcgm(nd, t, "node shut down")
            elif nd.power == "standby":
                nd.timer = None
            return {"ok": True, "host": h, "state": nd.power}
        if action == "cycle":
            if nd.power in ("booting", "on"):
                self._sel(nd, t, "System Event", "Power cycle requested (BMC)")
                nd.power, nd.timer, nd.after_shutdown = "shutting_down", (t + SHUTDOWN_S, "standby"), "boot"
                nd.down_cause = "BMC power cycle"
                self._abort_dcgm(nd, t, "node power cycled")
            elif nd.power == "shutting_down":
                nd.after_shutdown = "boot"
            elif nd.power == "standby" and nd.timer is None:
                self._sel(nd, t, "System Event", "Power cycle requested (BMC)")
                nd.timer = (t + CYCLE_OFF_S, "boot")
            return {"ok": True, "host": h, "state": nd.power}
        if nd.power in ON_STATES:
            self._sel(nd, t, "System ACPI Power State", "Host power off (forced)", "warning")
            nd.power, nd.timer, nd.after_shutdown, nd.down_cause = "standby", None, None, "BMC forced off"
            self._abort_dcgm(nd, t, "node forced off")
        return {"ok": True, "host": h, "state": nd.power}

    def _api_bmc_sel(self, params, t):
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        since = params.get("since", -1e12)
        if not isinstance(since, (int, float)) or isinstance(since, bool):
            raise BadParams("'since' must be sim seconds")
        return {"host": h, "entries": [dict(e) for e in nd.sel if e["time"] >= since]}

    def _api_bmc_sensors(self, params, t):
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        rng = self._hnoise("sensors", h, int(t))
        on = nd.power in ON_STATES
        gpus = {}
        for i in range(8):
            if nd.power == "on" and self._gpu_state(h, i) in ("ok", "remap_failure"):
                gpus["GPU%d" % i] = _r(nd.temp - GPU_T_OFFSET[GPU_SLOT[i]] + float(rng.normal(0, 0.3)), 1)
            else:
                gpus["GPU%d" % i] = "na"
        fans, fstat = {}, {}
        for m in range(1, 6):
            st = self._fan_status(h, m)
            fstat["FAN%d" % m] = st if on or st == "absent" else "standby"
            port = "%s/fan%d" % (h, m)
            for j, side in enumerate("FR"):
                if st == "absent":
                    fans["FAN%d_%s" % (m, side)] = "absent"
                elif not on:
                    fans["FAN%d_%s" % (m, side)] = 0
                else:
                    rot = self.parts[self.occ[port]].rotors[j]
                    fans["FAN%d_%s" % (m, side)] = int(round(FAN_RPM * rot * (1 + float(rng.normal(0, 0.01))))) \
                        if rot > 0 else 0
        n_ok = sum(self._psu_status(h, s) == "ok" for s in range(1, 7))
        return {"host": h, "power_state": nd.power, "inlet_temp_c": _r(nd.inlet + float(rng.normal(0, 0.1)), 1),
                "power_w": round(self._node_watts(nd) * (1 + float(rng.normal(0, 0.01)))), "gpu_temp_c": gpus,
                "fan_rpm": fans, "fan_status": fstat, "psu": {"PSU%d" % s: self._psu_status(h, s) for s in range(1, 7)},
                "psu_redundancy": "full" if n_ok == 6 else ("degraded" if n_ok >= 4 else "lost"),
                "power_brake": nd.brake, "gpu_thermal_slowdown": nd.throttled,
                "gpu_tray": "seated" if self.tray_latched[h] else "not seated"}

    def _api_bmc_identify(self, params, t):
        state = self._p(params, "state", ("on", "off", "blink"))
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        if state != nd.identify:
            self._sel(nd, t, "Identify", "Chassis identify LED %s" % state)
        nd.identify = state
        return {"ok": True, "host": h, "identify": state}

    def _api_bmc_fru(self, params, t):
        h, nd = self._bmc_node(params)
        if nd is None:
            return self._bmc_down(h)
        def ser(port):
            return self.parts[self.occ[port]].serial if self.occ[port] and self.latched[port] else "absent"
        gpus = {}
        for i in range(8):
            port = "%s/%s" % (h, GPU_SLOT[i])
            if nd.power != "on":
                gpus["GPU%d" % i] = "unavailable (host off)"
            elif self._gpu_state(h, i) in ("ok", "remap_failure"):
                gpus["GPU%d" % i] = self.parts[self.occ[port]].serial
            else:
                gpus["GPU%d" % i] = "not present"
        osfp = {}
        for k in range(1, 9):
            osfp["OSFP%d" % k] = ser("%s/osfp%d" % (h, k)) if nd.power == "on" else "unavailable (host off)"
        return {"host": h, "chassis": self._chassis_serial(h),
                "psu": {"PSU%d" % s: ser("%s/psu%d" % (h, s)) for s in range(1, 7)},
                "fan": {"FAN%d" % m: ser("%s/fan%d" % (h, m)) for m in range(1, 6)}, "gpu": gpus, "osfp": osfp}

    # ---- fabric
    def _dev_port(self, params):
        dev = self._p(params, "device", HOSTS + LEAVES)
        port = self._p(params, "port", tuple(range(1, 9)), typ=int)
        return dev, port

    def _api_fabric_port(self, params, t):
        dev, port = self._dev_port(params)
        st, peer, ln = self._ep(dev, port)
        c = self.ctr[(dev, port)]
        out = {"device": dev, "port": port, "status": st, "speed": "400G" if st != "down" else None, "width": "4x",
               "peer": {"device": peer[0], "port": peer[1]} if peer else None,
               "counters": {"symbol_errors": c.symbol_errors, "link_down_count": c.link_down_count,
                            "effective_ber": float("%.3g" % (c.symbol_errors / (c.up_s * LINE_BPS)))
                            if c.up_s > 0 and c.symbol_errors else 0.0},
               "locate": bool(self.locate.get((dev, port)))}
        if dev in NODE_ORDER:
            out["hca"] = "mlx5_%d" % (port - 1)
        if ln is not None and st != "down":
            self._ensure_optics()
            opt = self._opt[(ln.host, ln.k)]
            rx = opt["node_rx"] if dev in NODE_ORDER else opt["leaf_rx"]
            out["counters"]["raw_ber"] = float("%.3g" % raw_ber(min(rx) - RX_SENS_DBM))
        return out

    def _api_fabric_ddm(self, params, t):
        dev, port = self._dev_port(params)
        if dev in NODE_ORDER and self.nodes[dev].power != "on":
            return {"error": "device_unreachable",
                    "detail": "%s is not running (the HCA is read through the host)" % dev}
        if dev in LEAF_RAIL and not self._leaf_powered(dev):
            return {"error": "device_unreachable", "detail": "%s does not answer" % dev}
        oport = "%s/osfp%d" % (dev, port)
        inst = self.occ[oport] if self.latched[oport] else None
        if inst is None:
            return {"device": dev, "port": port, "module": {"present": False}}
        part = self.parts[inst]
        rng = self._hnoise("ddm", dev, port, int(t))
        self._ensure_optics()
        rx = [NO_LIGHT_DBM] * 4
        if dev in NODE_ORDER:
            ln = self.links[(dev, port - 1)]
            opt = self._opt[(dev, port - 1)]
            if opt["complete"] and self._leaf_powered(opt["peer"][0]):
                rx = opt["node_rx"]
        else:
            hk = self._leaf_peer.get((dev, port))
            if hk is not None:
                opt = self._opt[hk]
                if opt["complete"] and self.nodes[hk[0]].power == "on":
                    rx = opt["leaf_rx"]
        lanes, alarms = [], []
        for j in range(4):
            r_ = rx[j] if rx[j] <= NO_LIGHT_DBM + 1 else rx[j] + float(rng.normal(0, 0.04))
            lanes.append({"lane": j, "tx_dbm": _r(part.tx[j] + float(rng.normal(0, 0.04)), 2), "rx_dbm": _r(r_, 2),
                          "bias_ma": _r(part.bias[j] + float(rng.normal(0, 0.03)), 2)})
            if part.bias[j] > BIAS_WARN_MA:
                alarms.append({"lane": j, "flag": "tx_bias_high", "level": "warning"})
            if rx[j] <= NO_LIGHT_DBM + 1:
                alarms.append({"lane": j, "flag": "rx_los", "level": "alarm"})
            elif rx[j] < RX_SENS_DBM:
                alarms.append({"lane": j, "flag": "rx_power_low", "level": "warning"})
        return {"device": dev, "port": port,
                "module": {"present": True, "model": part.model, "serial": part.serial,
                           "temp_c": _r(part.temp + float(rng.normal(0, 0.2)), 1),
                           "vcc_v": _r(3.30 + float(rng.normal(0, 0.005)), 3)},
                "lanes": lanes, "alarms": alarms,
                "thresholds": {"rx_power_low_dbm": RX_SENS_DBM, "tx_bias_high_ma": BIAS_WARN_MA}}

    def _api_fabric_clear(self, params, t):
        if "device" not in params and "port" not in params:
            for c in self.ctr.values():
                c.clear()
            return {"ok": True, "cleared": "all ports"}
        dev, port = self._dev_port(params)
        self.ctr[(dev, port)].clear()
        return {"ok": True, "device": dev, "port": port}

    def _api_fabric_locate(self, params, t):
        dev, port = self._dev_port(params)
        state = self._p(params, "state", ("on", "off"))
        self.locate[(dev, port)] = state == "on"
        return {"ok": True, "device": dev, "port": port, "locate": state}

    def _api_fabric_soak(self, params, t):
        dev, port = self._dev_port(params)
        secs = self._p(params, "seconds", None, typ=int)
        if not 10 <= secs <= 7200:
            raise BadParams("'seconds' must be 10..7200")
        if dev in NODE_ORDER:
            hk = (dev, port - 1)
        else:
            hk = self._leaf_peer.get((dev, port)) or (HOSTS[port - 1], LEAF_RAIL[dev])
        st, _, _ = self._ep(dev, port)
        if st == "down":
            return {"error": "port_down", "detail": "%s port %d is down: nothing to soak" % (dev, port)}
        sid = "SOAK-%04d" % (len(self.soaks) + 1)
        leaf_ep = (RAIL_LEAF[hk[1]], NODE_ORDER[hk[0]])
        eps = [(hk[0], hk[1] + 1), leaf_ep]
        self.soaks[sid] = {"id": sid, "link": hk, "start": t, "end": t + secs, "seconds": secs, "status": "running",
                           "eps": eps, "err0": sum(self.ctr[e].tot_errors for e in eps),
                           "down0": [self.ctr[e].tot_downs for e in eps], "requested_on": (dev, port)}
        self.soak_active.append(sid)
        return {"ok": True, "soak_id": sid, "link": {"host": hk[0], "port": hk[1] + 1, "leaf": leaf_ep[0],
                                                     "leaf_port": leaf_ep[1]}, "ends_at": _r(t + secs)}

    def _soak_finish(self, sid, t):
        s = self.soaks[sid]
        self.soak_active.remove(sid)
        errs = sum(self.ctr[e].tot_errors for e in s["eps"]) - s["err0"]
        downs = max(self.ctr[e].tot_downs - d0 for e, d0 in zip(s["eps"], s["down0"]))
        up = self.links[s["link"]].band == "up"
        s.update({"status": "done", "symbol_errors": int(errs), "link_down_count": int(downs), "up_at_end": up,
                  "passed": bool(errs == 0 and downs <= 1 and up), "finished": t})

    def _api_fabric_soak_result(self, params, t):
        sid = self._p(params, "id")
        s = self.soaks.get(sid)
        if s is None:
            raise BadParams("no soak %s" % sid)
        if s["status"] == "running":
            return {"id": sid, "status": "running", "elapsed_s": _r(t - s["start"], 1),
                    "remaining_s": _r(s["end"] - t, 1)}
        return {"id": sid, "status": "done", "passed": s["passed"], "seconds": s["seconds"],
                "symbol_errors": s["symbol_errors"], "link_down_count": s["link_down_count"],
                "link_up_at_end": s["up_at_end"],
                "criteria": "fail if any symbol error, more than 1 link down, or the link not up at the end"}

    # ---- scheduler
    def _api_sched_nodes(self, params, t):
        return {"nodes": [{"node": h, "state": self.sched.state[h], "reason": self.sched.reason[h],
                           "since": _label(self.sched.since[h])} for h in HOSTS]}

    def _api_sched_jobs(self, params, t):
        job = self.job
        members = self._members()
        out = {"id": job["id"], "name": job["name"], "state": job["state"], "nodes": members, "gpus": 8 * len(members),
               "last_checkpoint": _label(job["last_cp"]), "next_checkpoint": _label(self.next_cp),
               "checkpoint_interval_s": CHECKPOINT_S, "restarts": job["crashes"]}
        if job["state"] == "running" and members:
            out["relative_speed"] = _r(min(self.nodes[h].speed * (LINK_ERR_SLOWDOWN if self.nodes[h].link_err_recent
                                                                   else 1.0) for h in members), 2)
        if job["state"] == "restarting":
            out["restart_at"] = _label(job["restart_at"])
        return {"jobs": [out]}

    def _api_sched_drain(self, params, t):
        h = self._p(params, "node", HOSTS)
        reason = params.get("reason", "")
        if not isinstance(reason, str):
            raise BadParams("'reason' must be a string")
        st = self.sched.state[h]
        if st == "alloc":
            self._set_state(h, "drain" if self.job["state"] in ("running", "restarting") else "drained", t, reason)
        elif st == "idle":
            self._set_state(h, "drained", t, reason)
        elif st in ("drain", "drained", "down"):
            self.sched.reason[h] = reason or self.sched.reason[h]
        return {"ok": True, "node": h, "state": self.sched.state[h]}

    def _api_sched_resume(self, params, t):
        h = self._p(params, "node", HOSTS)
        st = self.sched.state[h]
        if st == "drain":
            self._set_state(h, "alloc", t, "")
            return {"ok": True, "node": h, "state": "alloc"}
        if st not in ("drained", "down"):
            return {"ok": True, "node": h, "state": st, "note": "not drained"}
        if self.nodes[h].dcgm:
            return {"error": "diag_running", "detail": "DCGM run %s is in progress" % self.nodes[h].dcgm}
        issues = self._health_issues(h)
        if issues:
            return {"error": "health_check_failed", "detail": issues}
        self._set_state(h, "idle", t, "")
        self.resumes.append({"time": _r(t), "node": h, "hidden_faults": self._hidden_faults(h)})
        return {"ok": True, "node": h, "state": "idle", "joins_at": _label(self.next_cp)}

    def _hidden_faults(self, h):
        """Faults the node still has that its health check does not catch (a still-faulty node resumed)."""
        out = []
        for k in range(8):
            m = self._worst_margin(h, k)
            if m is not None and m < 1.0:
                out.append("rail %d margin %.2f dB" % (k, m))
        n_good = sum(self._psu_good(h, s) for s in range(1, 7))
        if n_good < 6:
            out.append("%d of 6 PSUs good" % n_good)
        for m_ in range(1, 6):
            if self._fan_status(h, m_) != "ok":
                out.append("FAN%d %s" % (m_, self._fan_status(h, m_)))
        for s in SLOTS:
            if not self._gpu_slot_ok("%s/%s" % (h, s)):
                out.append("%s not healthy" % s.upper())
        return out

    # ---- DCGM
    def _api_dcgm_run(self, params, t):
        h = self._host(params)
        level = self._p(params, "level", (1, 2, 3), typ=int)
        nd = self.nodes[h]
        if nd.power != "on":
            return {"error": "node_off", "detail": "%s is %s" % (h, nd.power)}
        if self.sched.state[h] not in ("drained", "down") or nd.dcgm:
            return {"error": "node_busy", "detail": "%s is %s%s" % (h, self.sched.state[h],
                                                                   ", a diagnostic is running" if nd.dcgm else "")}
        rid = "DIAG-%s-%03d" % (h, len(self.dcgm_runs) + 1)
        self.dcgm_runs[rid] = {"id": rid, "host": h, "level": level, "start": t, "end": t + DCGM_S[level],
                               "status": "running", "throttle_seen": nd.throttled, "brake_seen": nd.brake}
        self.dcgm_active.append(rid)
        nd.dcgm = rid
        return {"ok": True, "run_id": rid, "level": level, "expected_s": DCGM_S[level]}

    def _abort_dcgm(self, nd, t, why):
        if nd.dcgm:
            run = self.dcgm_runs[nd.dcgm]
            run.update({"status": "aborted", "reason": why, "finished": t})
            self.dcgm_active.remove(nd.dcgm)
            nd.dcgm = None

    def _dcgm_finish(self, rid, t):
        run = self.dcgm_runs[rid]
        h = run["host"]
        nd = self.nodes[h]
        self.dcgm_active.remove(rid)
        nd.dcgm = None
        if nd.power != "on":
            run.update({"status": "aborted", "reason": "node not running", "finished": t})
            return
        gpus, failed = [], []
        for i in range(8):
            st = self._gpu_state(h, i)
            res, msg = "pass", ""
            if st == "missing":
                res, msg = "fail", "GPU not found on PCI bus %s" % pci_addr(i)
            elif st == "remap_failure" and run["level"] >= 2:
                res, msg = "fail", "HBM row remap failure (uncorrectable error, remapping failed in bank %d)" % (
                    3 + i % 4)
            elif run["level"] == 3 and run["brake_seen"]:
                res, msg = "fail", "Targeted power: GPU power capped below target (system power brake)"
            elif run["level"] == 3 and run["throttle_seen"]:
                res, msg = "fail", "Targeted stress: clocks throttled (thermal slowdown)"
            if res == "fail":
                failed.append(i)
            gpus.append({"index": i, "pci": pci_addr(i), "result": res, "message": msg})
        run.update({"status": "done", "result": "pass" if not failed else "fail", "gpus": gpus, "failed": failed,
                    "finished": t,
                    "slot_map": {s.upper(): (self.parts[self.occ["%s/%s" % (h, s)]].serial
                                             if self.occ["%s/%s" % (h, s)] else None) for s in SLOTS}})

    def _api_dcgm_result(self, params, t):
        rid = self._p(params, "run_id")
        run = self.dcgm_runs.get(rid)
        if run is None:
            raise BadParams("no run %s" % rid)
        base = {"run_id": rid, "host": run["host"], "level": run["level"], "status": run["status"],
                "started": _label(run["start"])}
        if run["status"] == "running":
            base.update({"elapsed_s": _r(t - run["start"], 1), "remaining_s": _r(run["end"] - t, 1)})
        elif run["status"] == "aborted":
            base.update({"reason": run["reason"], "finished": _label(run["finished"])})
        else:
            base.update({"result": run["result"], "gpus": [dict(g) for g in run["gpus"]],
                         "finished": _label(run["finished"])})
        return base

    # ============================================================================================ the grader's side
    def truth(self):
        """The hidden scenario (grader only)."""
        sc = self.scn
        faces = {fid: [dict(d) for d in f.defects if d["origin"] != "background"] for fid, f in self.faces.items()
                 if any(d["origin"] != "background" for d in f.defects)}
        return {"seed": self.seed,
                "gpu": {k: v for k, v in sc["gpu"].items() if k not in ("seed",)},
                "link": {k: v for k, v in sc["link"].items() if k not in ("seed",)},
                "hotswap": dict(sc["hotswap"]), "mid": dict(sc["mid"]), "distractors": dict(sc["distractors"]),
                "tickets": {tid: {"kind": tk.kind, "host": tk.host, **tk.truth} for tid, tk in self.tickets.items()},
                "faces_with_scenario_defects": faces,
                "serials": {pid: p.serial for pid, p in self.parts.items()},
                "gpu_map": {i: s for i, s in enumerate(GPU_SLOT)}}

    def neutral_state(self):
        """The PhysicalState of the hall at rest as the cluster believes it now (tests, the cell's bootstrap)."""
        ps = PhysicalState()
        for h in HOSTS:
            ps.tray_pos[h], ps.tray_latched[h], ps.cover_open[h], ps.bezel_open[h] = 0.0, self.tray_latched[h], 0.0, 0.0
        for p in self.ports:
            ps.occupied[p] = self.occ[p] is not None
            ps.instance[p] = self.occ[p]
            ps.port_broken[p] = self.broken[p]
        for p in self.seat_q:
            ps.screws_fastened[p] = bool(self.occ[p]) and not self.armed[p]
            ps.screws_loose[p] = self.armed[p]
            ps.gpu_damaged[p] = self.socket_bad[p]
        return ps

    def _ticket_scope_ports(self, tk):
        h = tk.host
        if tk.kind == "gpu":
            return ["%s/%s" % (h, s) for s in SLOTS]
        if tk.kind == "link":
            return list(link_ports(h, tk.truth["rail"]))
        if tk.kind == "psu":
            return ["%s/psu%d" % (h, s) for s in range(1, 7)] + ["%s/plug%d" % (h, s) for s in range(1, 7)]
        if tk.kind == "fan":
            return ["%s/fan%d" % (h, m) for m in range(1, 6)]
        return []

    def _home_of(self, port):
        return next((pid for pid, p in self.parts.items() if p.home == port), None)

    def _true_moves(self, ports):
        """Per port whose part changed: the serial that was there at the start and the one there now."""
        out = []
        for port in ports:
            h0, now = self._home_of(port), self.occ[port]
            if h0 != now:
                out.append({"slot": port, "removed": self.parts[h0].serial if h0 else None, "removed_id": h0,
                            "installed": self.parts[now].serial if now else None, "installed_id": now})
        return out

    def _last_intervention(self, scope, key):
        ts = [t for t, sc, k, what in self.interventions if sc == scope and k == key and
              (scope != "host" or not str(what).split("/")[-1].startswith("fan"))]
        return max(ts) if ts else None

    def _link_fault_gone(self, h, k):
        opt = self._optics_now(h, k)
        planned = (RAIL_LEAF[k], NODE_ORDER[h])
        margin = (min(opt["node_rx"] + opt["leaf_rx"]) - RX_SENS_DBM) if opt["complete"] else None
        up = self.links[(h, k)].band == "up"
        return {"margin_db": _r(margin, 3), "margin_ok": bool(margin is not None and margin >= 1.0),
                "cabled_as_planned": opt["peer"] == planned, "up": up,
                "gone": bool(margin is not None and margin >= 1.0 and opt["peer"] == planned and up),
                "node_rx_dbm": [_r(v, 2) for v in opt["node_rx"]], "leaf_rx_dbm": [_r(v, 2) for v in opt["leaf_rx"]]}

    def _evidence(self, refs):
        out = []
        caps = {c["id"]: c for c in self.captures}
        for ref in refs:
            if ref in caps:
                c = caps[ref]
                face = self.faces.get(c["resolved"]) if c["resolved"] else None
                out.append({"ref": ref, "type": "scope_capture", "face": c["resolved"], "sharp": c["sharp"],
                            "fibres_visible": c["fibres_visible"], "defects": c["defects"],
                            "shows_core_scratch": bool(c["fibres_visible"] and c["defects"] and
                                                       c["defects"]["scratches_core"] > 0),
                            "face_has_core_scratch_now": bool(face and face.has_core_scratch())})
            elif ref in self.dcgm_runs:
                r = self.dcgm_runs[ref]
                out.append({"ref": ref, "type": "dcgm_run", "host": r["host"], "level": r["level"],
                            "status": r["status"], "result": r.get("result"), "failed": r.get("failed"),
                            "slot_map": r.get("slot_map")})
            elif ref in self.soaks:
                s = self.soaks[ref]
                out.append({"ref": ref, "type": "soak", "link": list(s["link"]), "status": s["status"],
                            "passed": s.get("passed")})
            else:
                out.append({"ref": ref, "type": "unknown"})
        return out

    def _swap_test(self, h, slot):
        """DCGM runs on h that failed the GPU index of `slot`, with the module serial in that slot each time, and
        where the module first found failing there went."""
        i = SLOT_GPU[slot]
        runs = []
        for r in self.dcgm_runs.values():
            if r["host"] == h and r["status"] == "done":
                runs.append({"run": r["id"], "failed_indices": r["failed"],
                             "failed_slots": [GPU_SLOT[j].upper() for j in r["failed"]], "slot_map": r["slot_map"]})
        fails_here = [r for r in runs if i in r["failed_indices"]]
        serials = {r["slot_map"][slot.upper()] for r in fails_here}
        followed = False
        if fails_here:
            first = fails_here[0]["slot_map"][slot.upper()]
            for r in runs:
                for s2, ser in r["slot_map"].items():
                    if ser == first and s2 != slot.upper() and SLOT_GPU[s2.lower()] in r["failed_indices"]:
                        followed = True
        return {"runs": runs, "fault_stayed_with_slot": len(serials) >= 2, "fault_followed_module": followed}

    def _ticket_report(self, tk, t):
        tr = tk.truth
        cause = tr.get("cause")
        h = tk.host
        upd = [u for u in tk.updates if u["root_cause"]]
        rc = upd[-1]["root_cause"] if upd else None
        exp_codes = [CAUSE_CODE[cause]] if cause in CAUSE_CODE else []
        sec = ["psu_cord_unseated"] if tk.kind == "psu" and tr.get("loose_psu") else []
        rep = {"id": tk.id, "kind": tk.kind, "host": h, "cause": cause, "expected_codes": exp_codes,
               "secondary_codes": sec, "opened": _r(tk.opened), "visible_at": _r(tk.visible_at),
               "mid_shift": bool(tr.get("mid")), "status": tk.status, "reported_root_cause": rc,
               "root_cause_correct": rc in exp_codes if rc else False, "updates": [dict(u) for u in tk.updates],
               "escalations": [dict(e) for e in tk.escalations]}
        if tk.kind == "gpu":
            slots = {s.upper(): {"seating": self._seating("%s/%s" % (h, s)), "ok": self._gpu_slot_ok("%s/%s" % (h, s)),
                                 "module": self.occ["%s/%s" % (h, s)],
                                 "module_health": (self.parts[self.occ["%s/%s" % (h, s)]].health
                                                   if self.occ["%s/%s" % (h, s)] else None),
                                 "socket_bad": self.socket_bad["%s/%s" % (h, s)],
                                 "slot_fault": self.slot_fault["%s/%s" % (h, s)]} for s in SLOTS}
            rep["fault_gone"] = all(v["ok"] for v in slots.values())
            rep["fault_detail"] = {"slots": slots, "faulty_slot": tr["slot"].upper(), "gpu_index": tr["gpu"]}
            last = self._last_intervention("host", h)
            ok = [r for r in self.dcgm_runs.values() if r["host"] == h and r["status"] == "done" and r["level"] >= 2
                  and r["result"] == "pass" and (last is None or r["start"] >= last)]
            rep["verification"] = {"last_intervention_t": last, "passed_after": bool(ok),
                                   "run": ok[-1]["id"] if ok else None}
            rep["swap_test"] = self._swap_test(h, tr["slot"])
        elif tk.kind == "link":
            k = tr["rail"]
            lf = self._link_fault_gone(h, k)
            rep["rail"] = k
            rep["fault_gone"] = lf["gone"]
            rep["fault_detail"] = lf
            last = self._last_intervention("link", (h, k))
            ok = [s for s in self.soaks.values() if s["link"] == (h, k) and s["status"] == "done" and s["passed"]
                  and s["seconds"] >= 600 and (last is None or s["start"] >= last)]
            rep["verification"] = {"last_intervention_t": last, "passed_after": bool(ok),
                                   "soak": ok[-1]["id"] if ok else None}
        elif tk.kind == "psu":
            good = [self._psu_good(h, s) for s in range(1, 7)]
            rep["fault_gone"] = all(good)
            rep["fault_detail"] = {"psu_good": good, "plug_bad": [self.plug_bad["%s/plug%d" % (h, s)]
                                                                  for s in range(1, 7)]}
            rep["verification"] = None
        elif tk.kind == "fan":
            st = [self._fan_status(h, m) for m in range(1, 6)]
            rep["fault_gone"] = all(s == "ok" for s in st)
            rep["fault_detail"] = {"fan_status": st}
            rep["verification"] = None
        state = self.sched.state[h] if h in HOSTS else None
        rep["node_state"] = state
        rep["in_service"] = state in ("alloc", "idle")
        back = [tt for tt, st, _ in self.sched.history.get(h, []) if st in ("alloc", "idle")]
        rep["returned_at"] = back[-1] if (rep["in_service"] and back) else None
        moves = self._true_moves(self._ticket_scope_ports(tk))
        reported = [p for u in tk.updates for p in u["parts"]]
        last_parts = next((u["parts"] for u in reversed(tk.updates) if u["parts"]), [])
        true_pairs = {(m["removed"], m["installed"]) for m in moves}
        rep_pairs = {(p.get("removed"), p.get("installed")) for p in last_parts}
        rep["parts_true"] = moves
        rep["parts_reported"] = last_parts
        rep["parts_reported_all_updates"] = reported
        rep["parts_match"] = true_pairs == rep_pairs
        exp_to = ESCALATE_TO.get(cause)
        esc = tk.escalations[-1] if tk.escalations else None
        rep["escalation_expected"] = exp_to
        rep["escalated_to"] = esc["to"] if esc else None
        rep["escalation_correct"] = bool(esc and exp_to and esc["to"] == exp_to) if exp_to else (esc is None)
        rep["evidence"] = self._evidence(esc["evidence"]) if esc else []
        if cause == "scratched_cable_end":
            fid = (tr.get("applied") or {}).get("face")
            rep["evidence_shows_damaged_face"] = any(e["type"] == "scope_capture" and e["face"] == fid and e["sharp"]
                                                     and e["shows_core_scratch"] for e in rep["evidence"])
        if cause == "baseboard":
            ids = {e["ref"] for e in rep["evidence"] if e["type"] == "dcgm_run"}
            runs = [r for r in rep["swap_test"]["runs"]
                    if r["run"] in ids and SLOT_GPU[tr["slot"]] in r["failed_indices"]]
            rep["evidence_shows_swap_test"] = len({r["slot_map"][tr["slot"].upper()] for r in runs}) >= 2
        return rep

    def final_report(self, t_end, parts_in_tote=None, locations=None):
        """Everything the grader needs at the end of the shift (data only; the scoring lives elsewhere).
        parts_in_tote: the part ids the cell finds in the RMA tote; locations: optional {part id: where} for parts
        that are not in a port (e.g. "cart", "bench", "floor", "hand")."""
        t_end = max(float(t_end), self.t)
        self._catch_up(t_end)
        self._advance(t_end)
        self._evaluate(t_end)
        tote = set(parts_in_tote if parts_in_tote is not None else self.parts_in_tote)
        locations = dict(locations or {})
        where = {inst: port for port, inst in self.occ.items() if inst}
        fd = self.scn["distractors"].get("fan_degraded")
        distractor = self._home_of("%s/fan%d" % (fd["host"], fd["fan"])) if fd else None
        parts, out_of_place, faulty_in_service, healthy_in_tote = {}, [], [], []
        for pid, p in self.parts.items():
            if pid in where:
                loc = where[pid]
            elif pid in tote:
                loc = "rma_tote"
            elif p.spare and not p.touched:
                loc = "cart"
            else:
                loc = locations.get(pid, "free")
            faulty = p.faulty()
            if p.kind == "mpo":
                face = self.faces.get("mpo:" + pid)
                faulty = bool(face and face.has_core_scratch())
            rec = {"kind": p.kind, "serial": p.serial, "model": p.model, "health": p.health, "faulty": faulty,
                   "home": p.home, "final": loc, "spare": p.spare, "touched": p.touched}
            if p.kind == "gpu":
                rec["wedged"] = p.wedged
                if p.prior_health:
                    rec["prior_health"] = p.prior_health
            if p.kind in ("osfp_n", "osfp_s"):
                rec["aged_lanes"] = [j for j in range(4) if p.bias[j] > BIAS_WARN_MA]
            if pid == distractor:
                rec["distractor"] = True                 # the degraded rotor nobody tickets: leave it, report it
            parts[pid] = rec
            if loc not in self.ports and loc not in ("rma_tote", "cart") and (p.touched or not p.spare):
                out_of_place.append(pid)
            if loc in self.ports and faulty and pid != distractor:
                faulty_in_service.append(pid)
            if loc == "rma_tote" and not faulty:
                healthy_in_tote.append(pid)
        nff = []
        for port, inst in self.occ.items():
            if inst and self.parts[inst].spare:
                hist = self.port_hist[port]
                prev = None
                for n, (tt, who) in enumerate(hist):
                    if who == inst:
                        prev = next((w for _, w in reversed(hist[:n]) if w is not None), None)
                        break
                if prev is not None and prev in self.parts:
                    pp = self.parts[prev]
                    face = self.faces.get("rx:" + prev)
                    healthy = not pp.faulty() and not (face and (face.damaged or face.has_core_scratch()))
                    if healthy:
                        nff.append({"port": port, "removed": pp.serial, "removed_id": prev,
                                    "installed": self.parts[inst].serial, "installed_id": inst})
        ph = self.phys
        fixtures = {h: {"tray_pos_m": _r(ph.tray_pos.get(h, 0.0), 4) if ph else 0.0,
                        "tray_latched": self.tray_latched[h],
                        "cover_open_rad": _r(ph.cover_open.get(h, 0.0), 3) if ph else 0.0,
                        "bezel_open_rad": _r(ph.bezel_open.get(h, 0.0), 3) if ph else 0.0} for h in HOSTS}
        dis = self.scn["distractors"]
        notes = " ".join([u["notes"] for tk in self.tickets.values() for u in tk.updates] +
                         [e.get("notes", "") for tk in self.tickets.values() for e in tk.escalations] +
                         [tk.summary + " " + tk.description for tk in self.tickets.values() if tk.kind == "report"])
        dis_rep = {"identify_left_on": None, "fan_degraded": None}
        if dis.get("identify"):
            dis_rep["identify_left_on"] = {"host": dis["identify"],
                                           "identify_at_end": self.nodes[dis["identify"]].identify}
        if dis.get("fan_degraded"):
            fd = dis["fan_degraded"]
            port = "%s/fan%d" % (fd["host"], fd["fan"])
            orig = self._home_of(port)
            dis_rep["fan_degraded"] = {"host": fd["host"], "fan": fd["fan"], "module": orig,
                                       "still_in_place": self.occ[port] == orig,
                                       "mentioned_in_notes": fd["host"] in notes and "fan" in notes.lower()}
        job = self.job
        return {
            "t_end": _r(t_end), "seed": self.seed,
            "tickets": [self._ticket_report(tk, t_end) for tk in self.tickets.values() if tk.kind != "report"],
            "technician_tickets": [tk.public(full=True) for tk in self.tickets.values() if tk.kind == "report"],
            "incidents": [dict(i) for i in self.incidents],
            "nff_swaps": nff,
            "parts": parts, "parts_out_of_place": out_of_place, "faulty_parts_in_service": faulty_in_service,
            "healthy_parts_in_tote": healthy_in_tote,
            "fixtures": fixtures, "doors": dict(ph.doors) if ph else {},
            "pdu_outlets_off": [(r, o) for r, outs in self.pdu.items() for o, s in outs.items() if s == "off"],
            "identify_on": [h for h in HOSTS if self.nodes[h].identify != "off"],
            "locate_on": ["%s/%d" % k for k, v in self.locate.items() if v],
            "node_power": {h: self.nodes[h].power for h in HOSTS},
            "sched": {h: {"state": self.sched.state[h], "reason": self.sched.reason[h],
                          "history": list(self.sched.history[h])} for h in HOSTS},
            "resumes": [dict(r) for r in self.resumes],
            "resumed_while_faulty": [dict(r) for r in self.resumes if r["hidden_faults"]],
            "distractors": dis_rep,
            "job": {"state": job["state"], "crashes": job["crashes"], "lost_gpu_s": _r(job["lost_gpu_s"], 0),
                    "work_gpu_s": _r(job["work_total"], 0), "saved_work_gpu_s": _r(job["saved_work"], 0)},
            "diag_runs": [{k: v for k, v in r.items()} for r in self.dcgm_runs.values()],
            "soaks": [{k: (list(v) if isinstance(v, tuple) else v) for k, v in s.items()} for s in self.soaks.values()],
            "captures": [dict(c) for c in self.captures],
            "cleanings": [dict(c) for c in self.cleanings],
            "unknown_inputs": list(self.unknown),
        }
