#!/usr/bin/env python3
"""Write the agent's documents from the cell's own numbers (maintainers' tool; nothing runs it at grading).

    python3 make_documents.py [--check]

It writes, under environment/setup/payload/given/: `site/site.json` and `site/site_interface.md`,
`tool/cutter_gripper.md`, `crop/growing_protocol.md` and `robot/ur20.json`, and the reference's copy of `site.json`
(solution/reference/). Every number comes from `tests/rowcell/farm.py`, `wire.py` and `kin.py`, so the documents
cannot drift from the cell: after a change there, run this again and commit the result. `--check` writes nothing and
exits 1 when a file on disk is not what this would write now.

Three kinds of numbers are not in those modules' tables, and each is still read, never typed:

- a number that stands inside a function of the cell (a draw's range in `farm._truss`, the pad's tolerance in
  `rules.Rules.gripper`): `lit()` reads it from the source line, quoted here with `#` where the number stands. When
  the line changes, this stops and names it;
- a range the action schema enforces (`wire.check_action`): `edge()` finds it by asking the schema;
- a condition of the job that the grader or the commissioning service holds (deadlines, budgets, how many rows and
  sessions): `conditions()` reads the file that holds it and, while that file does not exist yet, stands in the
  value the task was specified with and says so.
"""

import argparse
import inspect
import itertools
import json
import math
import os
import re
import string
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.dont_write_bytecode = True
sys.path.insert(0, TESTS)

import numpy as np  # noqa: E402

from rowcell import farm as F  # noqa: E402
from rowcell import kin, wire  # noqa: E402

GIVEN = os.path.join(TASK, "environment", "setup", "payload", "given")
D = wire.DEFAULTS

# What the documents call the programs their author does not write here (the client library's functions, the
# commissioning service's paths): one place to change them.
NAMES = {
    "client": "rowcell_client", "client_path": "~/given/lib/rowcell_client.py",
    "decode_rgb": "rowcell_client.decode_rgb", "decode_depth": "rowcell_client.decode_depth",
    "replies_differ": "rowcell_client.replies_differ", "session": "rowcell_client.Session(row)",
    "launcher": "~/commission/run_session.py", "launcher_cmd": "/opt/venv/bin/python3 ~/commission/run_session.py --row N",
    "socket": "/run/rowcell/commission.sock", "status": "/run/rowcell/status.json", "account": "rowcell",
}


# ------------------------------------------------------------------------------------------------- reading the code
_SRC = {}
_NUM = r"(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"


def source(module):
    if module not in _SRC:
        with open(os.path.join(TESTS, "rowcell", module + ".py")) as f:
            _SRC[module] = f.read()
    return _SRC[module]


def said(module, *texts):
    """Stop unless `tests/rowcell/<module>.py` still holds each of `texts` (a message, a key, a value of the wire that a
    document quotes word for word)."""
    for text in texts:
        if text not in source(module):
            sys.exit("make_documents.py: rowcell/%s.py no longer holds %r, which a document quotes" % (module, text))


# The observation as section 4 of the interface document lists it. `observation_keys()` reads the keys `cell.observe`
# writes; a key added or dropped there stops this tool until the document follows.
OBSERVATION = ["time", "q", "qd", "q_target", "tcp", "tcp_target", "wrench", "cart", "gripper", "status", "trace", "events",
               "frames_left", "time_left", "ended", "images"]


def observation_keys():
    body = source("cell")
    body = body[body.index("    def observe("):]
    body = body[:body.index("\n    # ---")]
    return re.findall(r'(?m)^ {12}"(\w+)":', body)


def lit(module, template):
    """The numbers that stand at the `#` marks of `template`, a piece of `tests/rowcell/<module>.py` quoted as it
    is written there (blanks are free; a leading `^` ties it to the start of a line). One number: a float; several:
    a tuple. The piece has to occur exactly once."""
    anchored = template.startswith("^")
    pat = "".join(_NUM if tok == "#" else r"\s*" if tok.isspace() else re.escape(tok)
                  for tok in re.split(r"(#|\s+)", template.lstrip("^")) if tok)
    found = re.findall((r"(?m)^[ \t]*" if anchored else "") + pat, source(module))
    if len(found) != 1:
        sys.exit("make_documents.py: rowcell/%s.py holds the line\n    %s\n%d times, not once. A document states the "
                 "number(s) at its # mark(s): quote the line here as it is written now."
                 % (module, template.lstrip("^"), len(found)))
    out = tuple(float(v) for v in (found[0] if isinstance(found[0], tuple) else (found[0],)))
    return out[0] if len(out) == 1 else out


def edge(make, ok, bad):
    """The last value the action schema accepts between `ok` (accepted) and `bad` (refused), to six digits."""
    def accepted(v):
        try:
            wire.check_action(make(v))
            return True
        except ValueError:
            return False
    if not accepted(ok) or accepted(bad):
        sys.exit("make_documents.py: wire.check_action no longer has a limit between %r and %r" % (ok, bad))
    for _ in range(100):
        mid = 0.5 * (ok + bad)
        ok, bad = (mid, bad) if accepted(mid) else (ok, mid)
    return float("%.6g" % ok)


def conditions():
    """The conditions of the job that the cell's modules do not hold, each from the file that does:
    tests/grader_config.json (the grader's knobs), tests/robotics_grader/solver.py (the kit's defaults),
    environment/commission_service/dev_rows.json (the commissioning service), tests/hidden/rows.json (the graded set)
    and task.toml (the session's length). While a file does not exist, the value the task was specified with stands
    in for it; `pending` lists what stands on such a value."""
    c = {"step_timeout_s": 30.0, "ready_factor": 4, "restarts": 3, "controller_budget_s": 1200, "max_steps": 20000,
         "max_processes": 64, "max_submission_mb": 200, "max_submission_files": 5000, "probe_pass_fraction": 0.1,
         "pair_tolerance": 1e-4, "verifier_deadline_s": None, "graded_rows": 6, "practice_rows": 4, "sessions": 5,
         "idle_s": 900, "agent_timeout_s": 7200, "graded_cultivar": "Rossa", "practice_cultivar": "Ambra"}
    pending = []

    def load(*parts):
        path = os.path.join(TASK, *parts)
        if not os.path.exists(path):
            pending.append("/".join(parts))
            return None
        with open(path) as f:
            return f.read()

    try:
        from robotics_grader import solver as kit
        c["ready_factor"] = int(kit.READY_TIMEOUT_FACTOR)
        sig = inspect.signature(kit.Solver.__init__).parameters
        c["restarts"], c["step_timeout_s"] = int(sig["max_restarts"].default), float(sig["step_timeout"].default)
    except Exception:  # noqa: BLE001
        pending.append("tests/robotics_grader/solver.py")
    text = load("tests", "grader_config.json")
    if text is not None:
        g = json.loads(text)
        for k in ("step_timeout_s", "controller_budget_s", "max_steps", "max_processes", "max_submission_mb",
                  "max_submission_files", "probe_pass_fraction", "pair_tolerance", "verifier_deadline_s"):
            if k in g:
                c[k] = g[k]
        for k in ("sim_cap_s", "frame_budget"):
            if k in g.get("cell", {}) and g["cell"][k] != D[k]:
                sys.exit("make_documents.py: tests/grader_config.json runs the graded cell with %s = %r, wire.DEFAULTS "
                         "has %r: the documents state one value" % (k, g["cell"][k], D[k]))
    def rows_of(text, name):
        try:
            rows = json.loads(text)["rows"]
            kinds = sorted({r.get("cultivar", "Rossa") for r in rows})
        except (ValueError, KeyError, TypeError, AttributeError):
            sys.exit("make_documents.py: %s is not {\"rows\": [{\"seed\": ..., \"cultivar\": ...}, ...]}; say in conditions() "
                     "how to read it" % name)
        if len(kinds) != 1 or kinds[0] not in F.CULTIVARS:
            sys.exit("make_documents.py: the rows of %s grow %s; the documents speak of one cultivar per set" % (name, kinds))
        return len(rows), kinds[0]

    text = load("environment", "commission_service", "dev_rows.json")
    if text is not None:
        c["practice_rows"], c["practice_cultivar"] = rows_of(text, "environment/commission_service/dev_rows.json")
        s = json.loads(text)
        c.update(sessions=s.get("sessions", c["sessions"]), idle_s=s.get("idle_s", c["idle_s"]))
    text = load("tests", "hidden", "rows.json")
    if text is not None:
        c["graded_rows"], c["graded_cultivar"] = rows_of(text, "tests/hidden/rows.json")
    text = load("task.toml")
    if text is not None:
        m = re.search(r"\[agent\]\s*\n(?:[^\[]*\n)*?timeout_sec\s*=\s*(\d+)", text)
        if not m:
            sys.exit("make_documents.py: task.toml has no [agent] timeout_sec")
        c["agent_timeout_s"] = int(m.group(1))
    c["ready_timeout_s"] = c["step_timeout_s"] * c["ready_factor"]
    c["pending"] = pending
    return c


# ------------------------------------------------------------------------------------------------- writing
def g(v):
    v = float(v)
    return "%g" % (round(v, 10) + 0.0)


def mm(v, n=2):
    return "%g" % (round(float(v) * 1000.0, n) + 0.0)


def span(pair, f=g, sep=" to "):
    return f(pair[0]) + sep + f(pair[1])


def vec(v, n=4):
    return "[" + ", ".join("%g" % (round(float(x), n) + 0.0) for x in v) + "]"


def ang(v):
    """An angle limit of the schema, in words where it is a multiple of pi."""
    for k, name in ((1, "pi"), (2, "2 pi")):
        if abs(v - k * math.pi) < 1e-4:
            return name
    return g(v)


def ordinal(n):
    return {1: "first", 2: "second", 3: "third", 4: "fourth", 5: "fifth"}[int(n)]


def word(n):
    return {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six", 7: "seven", 8: "eight", 9: "nine",
            10: "ten", 11: "eleven", 12: "twelve"}.get(int(n), str(int(n)))


def table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


class Doc(object):
    """A document as blocks: `p` a paragraph, `ul` a list, `raw` a table, a heading or a code block as it stands.
    Paragraphs and list items take `$name` from the values and are wrapped at 118 columns."""

    def __init__(self, **values):
        self.v, self.blocks = values, []

    def _fill(self, text):
        return string.Template(" ".join(text.split())).substitute(self.v)

    def _wrap(self, text, indent=""):
        return textwrap.fill(self._fill(text), width=118, subsequent_indent=indent, break_long_words=False,
                             break_on_hyphens=False)

    def p(self, text):
        self.blocks.append(self._wrap(text))

    def ul(self, items):
        self.blocks.append("\n".join(self._wrap("- " + it, "  ") for it in items))

    def raw(self, text, fill=False):
        self.blocks.append(string.Template(text).substitute(self.v) if fill else text)

    def text(self):
        return "\n\n".join(self.blocks) + "\n"


# ------------------------------------------------------------------------------------------------- derived numbers
def cut_force(toughness, r):
    """The closing force (N) that cuts a stalk of radius r (m) at a toughness (the cell's own formula)."""
    return F.cut_force({"mechanics": {"toughness": toughness}}, r)


def force_span(r_pair):
    return "%.0f to %.0f" % (cut_force(F.CUT["toughness"][0], r_pair[0]), cut_force(F.CUT["toughness"][1], r_pair[1]))


def fruit_mass(r):
    return 4.0 / 3.0 * math.pi * r ** 3 * F.FRUIT_DENSITY


def needed(cultivar, n):
    """How many of a truss's n fruits have to be at the cultivar's stage for the truss to be ready."""
    c = F.CULTIVARS[cultivar]
    return min(k for k in range(n + 1) if F.is_ready(cultivar, [c["stage_min"]] * k + [1] * (n - k)))


def mount_error(name, dist):
    """How far a point `dist` in front of camera `name` is off when it is located through the nominal mounting:
    (the median over the offsets within the tolerance, the largest). The draws are the cell's own reproducible
    stream, so the figure is the same on every run."""
    tol = F.MOUNT_TOL[name]
    st = F.Stream(0, "documents|" + name)
    p = np.array([0.0, 0.0, dist, 1.0])

    def off(o):
        return float(np.linalg.norm((F.mount_offset_T(o) @ p - p)[:3]))
    drawn = sorted(off([st.uniform(-tol["mm"], tol["mm"]) for _ in range(3)] + [st.uniform(-tol["deg"], tol["deg"]) for _ in range(3)])
                   for _ in range(2000))
    worst = max(off([s * tol["mm"] for s in sg[:3]] + [s * tol["deg"] for s in sg[3:]]) for sg in itertools.product((-1.0, 1.0), repeat=6))
    return drawn[len(drawn) // 2], worst


def in_view(name, pts):
    """Pixels and depths of cart-frame points through camera `name`'s nominal mounting, and whether all are in the image."""
    T = kin.inv(F.camera_mount(name))
    K, c = F.camera_K(name), F.CAMERAS[name]
    out = []
    for p in pts:
        q = T @ np.array([p[0], p[1], p[2], 1.0])
        out.append((K[0][0] * q[0] / q[2] + K[0][2], K[1][1] * q[1] / q[2] + K[1][2], float(q[2])))
    return out, all(z > 0 and 0 <= u < c["width"] and 0 <= v < c["height"] for u, v, z in out)


def board_corners(board, size):
    ox, oy, oz = board["origin"]
    return [(ox + a * size[0], oy + b * size[1], oz) for a in (0, 1) for b in (0, 1)]


def card_size():
    cc = F.COLOR_CARD
    return [(cc["patches"][i] - 1) * cc["pitch"] + cc["patch"] for i in (0, 1)]


def rachis_len(c):
    """A cultivar's shortest and longest rachis (knuckle to tip), from the draw of `farm._truss`."""
    first, gap = lit("farm", "^first = rf + st.uniform(#, #)"), lit("farm", "^gap = st.uniform(#, #)")
    shrink = lit("farm", 'c["fruit_r"][1] - # * (n - c["fruits"][0])')
    out = []
    for n in range(c["fruits"][0], c["fruits"][1] + 1):
        for rf, a, b in ((c["fruit_r"][0], first[0], gap[0]), (c["fruit_r"][1] - shrink * (n - c["fruits"][0]), first[1], gap[1])):
            out.append(rf + a + (n - 0.5) * (rf + b))
    return min(out), max(out)


def truss_mass(c):
    """The lightest and the heaviest set of fruits a cultivar's truss carries (kg)."""
    shrink = lit("farm", 'c["fruit_r"][1] - # * (n - c["fruits"][0])')
    small = lit("farm", "^r_j = rf * st.uniform(#, #)")[0]
    n0, n1 = c["fruits"]
    return (n0 * fruit_mass(c["fruit_r"][0] * small),
            max(n * fruit_mass(c["fruit_r"][1] - shrink * (n - n0)) for n in range(n0, n1 + 1)))


# ------------------------------------------------------------------------------------------------- the site's interface
def site_interface(C):
    site = F.public_site()
    cart, tool = F.CART, F.TOOL
    home = kin.flange(cart["home_q"]) @ wire.TOOL_T
    sw = F.SENSORS["wrist"]
    fw = F.camera_K("wrist")[0][0]
    zref = 0.35
    tcp_cam = kin.inv(F.camera_mount("wrist")) @ np.array([0.0, 0.0, tool["tcp_z"], 1.0])
    look = float(np.linalg.norm(np.array(F.CAMERAS["overview"]["look_at"]) - np.array(F.CAMERAS["overview"]["pos"])))
    med_o, max_o = mount_error("overview", look)
    tone = D["render_settings"]["omni:rtx:post:tonemap:operator"]
    med_w, max_w = mount_error("wrist", zref)
    med_t, max_t = mount_error("wrist", float(tcp_cam[2]))
    plate_size = [F.CAL_PLATE["squares"][i] * F.CAL_PLATE["square"] for i in (0, 1)]
    plate_px, plate_seen = in_view("overview", board_corners(F.CAL_PLATE, plate_size))
    card_px, card_seen = in_view("overview", board_corners(F.COLOR_CARD, card_size()))
    plate_z = sum(p[2] for p in plate_px) / 4.0
    card_z = sum(p[2] for p in card_px) / 4.0
    fo = F.camera_K("overview")[0][0]
    back_ticks = abs(int(lit("cell", "(self.q_hist + [self.q.copy()])[#:]"))) - 1
    peak = lit("cell", "^peak = # / (n * F.TICK)")
    lim_t = edge(lambda v: {"move": {"type": "tcp", "delta": [v, 0, 0, 0, 0, 0]}}, 0.0, 1e6)
    lim_r = edge(lambda v: {"move": {"type": "tcp", "delta": [0, 0, 0, v, 0, 0]}}, 0.0, 1e6)
    lim_j = edge(lambda v: {"move": {"type": "joint", "delta": [v, 0, 0, 0, 0, 0]}}, 0.0, 1e6)
    guard = (edge(lambda v: {"move": {"type": "hold", "guard_force": v}}, 1.0, 0.0),
             edge(lambda v: {"move": {"type": "hold", "guard_force": v}}, 1.0, 1e9))
    grip_o = edge(lambda v: {"gripper": {"opening": v, "force": 0.0}}, 0.0, 1e6)
    grip_f = edge(lambda v: {"gripper": {"opening": 0.0, "force": v}}, 0.0, 1e9)
    cart_step = edge(lambda v: {"cart": v}, 0.0, 1e6)
    if observation_keys() != OBSERVATION:
        sys.exit("make_documents.py: cell.observe writes the keys %s; section 4 of the interface document lists %s"
                 % (observation_keys(), OBSERVATION))
    said("cell", "cams: refused, the frame budget is used up", '"sim_cap"', '"stops_latched"', '"done"', '"clipped"', '"guard"',
         '"stopped"', '"cart_limit"', "locked_s=", "protective_stops=", '"force"', '"opening_mm"', '"grip_force"', '"rgb"',
         '"t"', "depth=", '["pose"]', '"odometer"', '"opening"')
    quoted = lambda names: " | ".join('"%s"' % n for n in names)        # noqa: E731
    mk = F.MARKERS
    m0 = F.marker_T(0)
    bays = cart.get("crate_bays")
    slots = F.crate_slots() if bays else []
    bays_text = "" if not bays else (
        " Each is parted into %s bays side by side along x, a bay for a truss, by dividers that run the crate's length along y, "
        "%s m high above the floor and %s m thick, midway between the bays' centre lines." % (word(bays), g(cart["crate_divider_h"]), g(cart["crate_divider_t"])))
    level = "level, pointing at the row" if home[0, 2] > 0.999 else "not level"
    ped_d = [2 * min(c["peduncle_r"][0] for c in F.CULTIVARS.values()), 2 * max(c["peduncle_r"][1] for c in F.CULTIVARS.values())]
    stem_d = [2 * F.ROW["stem_r"][0], 2 * F.ROW["stem_r"][1]]
    win_near = sw["window_px"] * sw["range_m"][0] / fw

    def thin(d):                                     # the distance beyond which a stalk of diameter d is narrower than the window
        return d * fw / sw["window_px"]
    if ped_d[1] < win_near:
        thin_text = ("A peduncle (%s mm thick) and a leaf's petiole are narrower than the window at every distance of the "
                     "working range and give no reading; a main stem (%s mm) gives none beyond %.2f to %.2f m." % (
                         span(ped_d, mm), span(stem_d, mm), thin(stem_d[0]), thin(stem_d[1])))
    else:
        thin_text = ("A peduncle (%s mm thick) is narrower than the window, and gives no reading, beyond %.2f to %.2f m; "
                     "a main stem (%s mm) beyond %.2f to %.2f m." % (
                         span(ped_d, mm), thin(ped_d[0]), thin(ped_d[1]), span(stem_d, mm), thin(stem_d[0]), thin(stem_d[1])))
    d = Doc(
        plants=F.ROW["plants"], trusses=F.ROW["trusses"], floor_z=g(F.FLOOR_Z), deck_z=g(F.DECK_Z),
        deck_x=span(cart["deck"]["x"]), deck_y=span(cart["deck"]["y"]), pedestal_r=g(lit("scene", 'b.cyl("/World/Cart/pedestal", #,')),
        board_z=g(F.BOARD_Z), row_x=g(F.ROW["x"]), gutter_z=g(F.ROW["gutter_z"]), first_y=g(F.ROW["first_y"]), pitch=g(F.ROW["pitch"]),
        limits=vec([math.degrees(v[1]) for v in kin.Q_LIMITS], 1), home_q=vec(cart["home_q"]), home_p=vec(home[:3, 3], 3),
        home_z=vec(home[:3, 2], 2), home_x=vec(home[:3, 0], 2), tcp_z=g(tool["tcp_z"]), bays_text=bays_text, level=level,
        moves=quoted([m for m in wire.MOVES if m != "hold"] + ["hold"]), cmds=quoted(wire.CMDS),
        cams="[" + ", ".join('"%s"' % n for n in wire.CAMS) + "]",
        ready_s=g(C["ready_timeout_s"]), step_s=g(C["step_timeout_s"]), probe=g(wire.PROBE_SHIFT_RAD), n_canon=len(wire.canonical({})),
        pair_tol=g(C["pair_tolerance"]), pass_fraction=g(C["probe_pass_fraction"]), max_processes=C["max_processes"],
        restarts=word(C["restarts"]), max_ticks=D["max_ticks"], tick=g(F.TICK), lim_t=g(lim_t), lim_r=ang(lim_r), lim_j=ang(lim_j),
        v_tcp=g(D["tcp_speed_max"]), w_tcp=g(D["tcp_rot_speed_max"]), v_joint=vec(D["joint_speed_max"]), peak=g(peak),
        guard_lo=g(guard[0]), guard_hi=g(guard[1]), cart_step=g(cart_step), cart_v=g(cart["speed_max"]),
        cart_reach="%.2f" % (cart["speed_max"] * D["max_ticks"] * F.TICK / peak),
        grip_o=g(grip_o), grip_f=g(grip_f), open_max=g(tool["open_max"]), force_max=g(tool["force_max"]),
        noise_f=g(D["noise_force"]), noise_t=g(D["noise_torque"]), over_f=g(D["overload_force"]), over_t=g(D["overload_torque"]),
        over_q=g(D["overload_joint_error"]), back_s=g(back_ticks * F.TICK), lock_s=g(D["stop_lock_s"]), latch=ordinal(F.CONSEQUENCE["latch"]),
        sim_cap=g(D["sim_cap_s"]), settle=g(D["settle_s"]), frames=D["frame_budget"], travel0=g(cart["travel"][0]), travel1=g(cart["travel"][1]),
        slip0=g(F.ROW["slip"][0]), slip1=g(F.ROW["slip"][1]), slip_pct=span([100 * v for v in F.ROW["slip"]]), slip_sigma=g(100 * F.ROW["slip_sigma"]),
        n_markers=mk["count"], marker_pitch=g(mk["pitch"]), dictionary=mk["dictionary"], last_id=mk["count"] - 1,
        marker_x=vec(m0[:3, 0]), marker_y=vec(m0[:3, 1]), marker_z=vec(m0[:3, 2]), marker_tilt=g(mk["tilt_deg"]),
        marker_side=g(mk["side"]), marker_cells=len(F.MARKER_BITS[0]) + 2, marker_bits=len(F.MARKER_BITS[0]), marker_margin=mm(mk["margin"]),
        depth_unit=mm(site_depth_unit()), depth_unit_m=g(site_depth_unit()), tol_o_mm=g(F.MOUNT_TOL["overview"]["mm"]), tol_o_deg=g(F.MOUNT_TOL["overview"]["deg"]),
        tol_w_mm=g(F.MOUNT_TOL["wrist"]["mm"]), tol_w_deg=g(F.MOUNT_TOL["wrist"]["deg"]),
        look="%.1f" % look, tone={"acesApproximation": "the ACES approximation"}.get(tone, tone),
        err_o="%.0f" % (100 * med_o), err_o_max="%.0f" % (100 * max_o), err_w="%.0f" % (1000 * med_w), err_w_max="%.0f" % (1000 * max_w),
        err_t="%.0f" % (1000 * med_t), err_t_max="%.0f" % (1000 * max_t), zref=g(zref), tcp_cam_z="%.3f" % tcp_cam[2], cam_side="-x" if F.CAMERAS["wrist"]["pos"][0] < 0 else "+x",
        tcp_px="(%.0f, %.0f)" % (fw * tcp_cam[0] / tcp_cam[2] + F.camera_K("wrist")[0][2], fw * tcp_cam[1] / tcp_cam[2] + F.camera_K("wrist")[1][2]),
        range0=g(sw["range_m"][0]), dark1=sw["dark"][1], dark0=sw["dark"][0], bright0=sw["bright"][0],
        bright1=sw["bright"][1], window=sw["window_px"], edge_px=g(sw["edge_px"]), thin_text=thin_text,
        gain0=g(F.ROW["light_gain"][0]), gain1=g(F.ROW["light_gain"][1]), green0=g(F.ROW["light_g"][0]), green1=g(F.ROW["light_g"][1]),
        blue0=g(F.ROW["light_b"][0]), blue1=g(F.ROW["light_b"][1]),
        lamps=word(lit("farm", '"lamps": [round(st.uniform(#, #), 4) for _ in range(#)]')[2]),
        lamp0=g(lit("farm", '"lamps": [round(st.uniform(#, #), 4) for _ in range(#)]')[0]),
        lamp1=g(lit("farm", '"lamps": [round(st.uniform(#, #), 4) for _ in range(#)]')[1]),
        max_steps=C["max_steps"], budget_s=g(C["controller_budget_s"]),
        wall_top=g(cart["crate_floor_z"] + cart["crate_wall_h"]), floor_slack=mm(lit("rules", "fz - # <= c[2] - r")),
        touch=g(F.DAMAGE["stack_touch"]), stack_dz=g(F.DAMAGE["stack_dz"]),
        practice_rows=C["practice_rows"], practice=C["practice_cultivar"], graded=C["graded_cultivar"], graded_rows=word(C["graded_rows"]),
        sessions=C["sessions"], idle_s=g(C["idle_s"]), **NAMES)

    d.raw("# Harvesting cart: interface document")
    d.p("""The site: a rail cart that drives along one plant row of a high-wire tomato greenhouse and carries a Universal
        Robots UR20 with a cutter-gripper, two harvest crates, a colour camera on a mast and an RGB-D camera on the robot's
        wrist. One job is one row section: $plants plants with $trusses trusses each (`../crop/growing_protocol.md`). Your
        program drives the cart, the arm and the tool through the wire of section 2. All numbers of this document are also
        in `site.json` beside it.""")

    d.raw("## 1. Frames and places")
    d.p("""**The cart frame** is the robot's base frame: origin on the robot's mounting face, x from the aisle toward the
        plant row, y along the rail, z up. It rides with the cart. Every position and pose on the wire and in this document
        is in this frame, in metres, unless it says otherwise.""")
    d.p("""**The row frame** is fixed to the greenhouse and parallel to the cart frame. The cart's base stands at (0, c, 0)
        in the row frame, where c is the cart's place along the rail, and c = 0 where a row section starts. A point at
        (x, y, z) in the row frame is at (x, y - c, z) in the cart frame. Nothing on the wire reports c (section 5).""")
    d.ul([
        """**Heights.** The greenhouse floor is at z = $floor_z, the top of the cart's deck at z = $deck_z. The deck reaches
        from x = $deck_x and from y = $deck_y; the robot stands on a pedestal, a column of $pedestal_r m radius under its
        base.""",
        """**Crates.** Two open crates stand on the deck and ride with the cart, one behind the robot along the rail and one
        ahead of it.$bays_text"""])
    d.raw(table(["crate", "centre x, y (m)", "inner size x by y (m)", "top of the floor, z (m)", "walls: height, thickness (m)"]
                + (["bays' centre lines, x (m)"] if bays else []),
                [[k, "%s, %s" % (g(cr["center"][0]), g(cr["center"][1])), "%s x %s" % (g(cr["size"][0]), g(cr["size"][1])),
                  g(cart["crate_floor_z"]), "%s, %s" % (g(cart["crate_wall_h"]), g(cart["crate_wall_t"]))]
                 + ([", ".join(g(b["x"]) for b in slots if b["crate"] == k)] if bays else []) for k, cr in cart["crates"].items()]))
    d.ul([
        """**Boards.** A calibration plate and a colour reference card lie flat on risers at the deck's row-side edge, their
        faces at z = $board_z, and ride with the cart (section 6).""",
        """**Mast.** The overview camera sits on a mast at the deck's rear corner on the row side (section 6).""",
        """**The row.** The plants' stems stand in a substrate slab on a gutter along the row: nominally at x = $row_x, the
        first at y = $first_y and one every $pitch m (row frame), the slab's top at z = $gutter_z (the growing protocol has
        the tolerances). Row markers are fixed along the gutter's aisle side (section 5). A plain screen hangs behind the
        row.""",
        """**Robot.** Joint limits (degrees, either way): $limits. A row section starts with the cart at c = 0, the jaws
        open and the robot at its home joint angles (rad) $home_q: the tool centre point at $home_p, the tool axis along
        $home_z ($level) and the tool frame's x axis along $home_x. Kinematic model:
        `../robot/ur20.json`. The flange frame is the robot's `wrist_3_link` frame: origin on the flange face, z out of the
        flange. The tool frame has the flange frame's axes and its origin at the tool centre point, $tcp_z m out along z
        (`../tool/cutter_gripper.md`)."""])

    d.raw("## 2. The wire")
    d.p("""The grader starts your program with `bash run.sh` in the submission folder and talks to it over its standard
        input and output, one JSON object per line. Your program first prints `READY` (within $ready_s s), then answers
        every request (each within $step_s s):""")
    d.raw("""```
{"type": "reset"}                        -> {"ok": true}            the robot stands at a new row section
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t of this row section
{"type": "close"}                        -> {"ok": true}, then exit
```""")
    d.p("""Standard output is the wire: write logs to standard error. `t` is the step index of the row section, owned by
        the grader, starting at 0 after each `reset`.""")
    d.p("""**A step may be asked more than once with the same `t`.** A repeated request carries the observation the cell
        would give with the robot's joints shifted by a small hidden amount (up to $probe rad a joint, the same shift for
        the measured and the commanded angles): `q` and `q_target` are shifted, and `tcp`, `tcp_target` and a wrist frame's
        `pose` are those of the shifted joints, so the readings of one observation agree with each other as they always do.
        Everything else in it is unchanged. The other request carries the true observation, and nothing on the wire says
        which is which or which comes first. Only the answer to the true observation is executed. Answer every request from
        the observation it carries, and advance your own state once per `t`, not once per request. A program whose motions
        do not depend on the robot's readings it is sent (a replayed list of actions, absolute joint targets worked out in
        advance) is recognised by this and scores nothing: compute each displacement from the `tcp_target` or `q_target` of
        the observation you answer. Requests that command no motion of the arm or the cart are not compared. Two replies
        differ when one of the $n_canon numbers the grader reads from an action differs by more than $pair_tol
        (`$replies_differ` is the grader's rule); if fewer than $pass_fraction of the compared pairs, over all the row
        sections, differ, the reward is 0. `$launcher` prints the count of a session.""")
    d.p("""A row section ends early on your program's side in two ways; in both, what has been delivered so far still
        counts:""")
    d.ul([
        """**Your program is given up on and started again:** a missed deadline (`READY` within $ready_s s of a start; each
        request read and answered within $step_s s), a closed output, a line that is not a JSON object, an answer without
        `action`, or more than $max_processes processes of your account (counted before every request; threads are not
        counted). The row section ends where it is, your program is stopped with everything it started, and `run.sh` is
        started again for the next row section: at most $restarts such restarts, after which the remaining row sections are
        not worked on.""",
        """**The row section ends and your program goes on:** an `action` that is not valid (a malformed field or a value
        out of range) or that the cell rejects, the row section's budget of your program's own time used up, or its step
        limit reached (section 7). The same program, still running, gets the next `reset`."""])
    d.p("""**When the cell itself fails.** Your program shares the GPU and the sandbox's memory with the cell. If the cell
        fails while a row section runs, your program is stopped with everything it started, every file of your account
        outside the submission folder and every SysV IPC object of your account are removed, the submission folder is put
        back as it was handed over, and the row section is run again from the start in a fresh cell, with `run.sh` started
        again (that restart is not one of the $restarts). If the cell fails a second time on that row section after your
        program was started again, or your program could not be stopped completely, the row section counts 0 and grading
        goes on with the next one.""")
    if C["verifier_deadline_s"]:
        d.v["deadline"] = g(C["verifier_deadline_s"])
        d.p("""**The grader's own deadline.** The grader stops $deadline s after it starts, whatever is running then: the
            row sections graded by then count and the others count 0.""")

    d.raw("## 3. Actions")
    d.p("An action is a JSON object; every key is optional:")
    d.raw("""```
{"move": {"type": $moves, "delta": [6 numbers], "ticks": 1..$max_ticks, "guard_force": N},
 "cart": metres along the rail,
 "gripper": {"opening": metres, "force": N},
 "cmd": $cmds,
 "cams": $cams}
```""", fill=True)
    d.ul([
        """**move.** One action lasts `ticks` control ticks of $tick s (one tick without `ticks`). `"tcp"` displaces the
        commanded pose of the tool centre point by `delta` = [dx, dy, dz, rx, ry, rz]: a translation (m, each component
        within +/- $lim_t) and a rotation vector (rad, each within +/- $lim_r), both in the cart frame, the rotation
        applied on the left of the current orientation, on a straight line with a smooth start and stop. `"joint"`
        displaces the six commanded joint angles (rad, each within +/- $lim_j). `"hold"` keeps the commanded pose. The
        displacement always starts from the commanded target (`tcp_target`, `q_target`), not from the measured pose. Speed
        limits: $v_tcp m/s and $w_tcp rad/s at the tool centre point, joints $v_joint rad/s. A move's peak speed is $peak
        times its displacement divided by its duration: a move whose peak would exceed a limit, or that the arm cannot
        follow (near a singularity, at a joint limit, out of reach), is shortened, and the observation says `clipped`. A
        move of one or two ticks has no room for a smooth start and stop.""",
        """**guard_force** ($guard_lo to $guard_hi N) stops the arm for the rest of the action as soon as the force on the
        tool has changed by more than that many newtons since the action began (the length of the change of the force
        vector, looked at once per tick). The observation then says `guard`.""",
        """**cart.** Drives the cart that many metres along the rail by its odometer (within +/- $cart_step; positive:
        toward +y), over the same ticks and with the same smooth start and stop, whatever the arm does meanwhile. Speed
        limit $cart_v m/s at the peak ($cart_reach m in an action of $max_ticks ticks); a longer drive is shortened and the
        observation says `clipped`. Section 5 says how far the cart then really went.""",
        """**gripper.** The jaws' target opening (m, 0 to $grip_o; anything above $open_max is the full opening) and their
        closing force (N, 0 to $grip_f; anything above $force_max is $force_max). Both keys are required. They stay in
        force in later actions until another `gripper` replaces them; a row section starts with the jaws open and a force
        of 0 (`../tool/cutter_gripper.md`).""",
        """**cmd.** `"done"` ends the row section after this action's move.""",
        """**cams.** The cameras to take a frame from at the end of this action (section 6)."""])
    d.p("An action with an unknown key, a wrong type or a number out of range is not a valid action (section 2).")

    d.raw("## 4. Observations")
    d.raw("""```
{"time": s,                       simulated time since the row section started
 "q": [6], "qd": [6],             measured joint angles (rad) and speeds (rad/s)
 "q_target": [6],                 commanded joint angles
 "tcp": [x, y, z, rx, ry, rz],    measured pose of the tool centre point: position (m), rotation vector (rad)
 "tcp_target": [...],             commanded pose of the tool centre point
 "wrench": [fx, fy, fz, tx, ty, tz],   what the world exerts on the tool: force (N) and moment about the flange's
                                  origin (N m), along the cart frame's axes; the tool's own weight is compensated, a
                                  held truss's is not; noise about $noise_f N and $noise_t N m
 "cart": {"odometer": m},         the cart's travel along the rail as its wheels count it (section 5)
 "gripper": {"opening": m, "force": N},   the jaws' opening and the force they press with (tool data sheet, section 4)
 "status": {"clipped": bool, "guard": bool, "stopped": bool, "cart_limit": bool,
            "locked_s": s, "protective_stops": n},
 "trace": {"force": [...], "opening_mm": [...], "grip_force": [...]},
                                  one sample per tick of the action just executed: the size of the force on the tool
                                  (N), the jaws' opening (mm) and their force (N)
 "events": [strings], "frames_left": n, "time_left": s,
 "ended": null | "done" | "sim_cap" | "stops_latched",
 "images": {"overview": {"rgb": .., "t": s},
            "wrist": {"rgb": .., "depth": .., "depth_unit_m": $depth_unit_m, "t": s, "pose": [x, y, z, rx, ry, rz]}}}
```""", fill=True)
    d.p("""`tcp` is the pose the kinematic model gives for `q`, `tcp_target` the one it gives for `q_target`; the joint
        speeds are differences of `q` over a tick. Nothing on the wire says where a plant, a truss or a fruit is, whether a
        truss is ready, or what has been cut, held, bruised or dropped: the readings show what the robot felt, the frames
        what its cameras saw.""")
    d.p("""**Protective stops.** When the force on the tool exceeds $over_f N, the moment $over_t N m, or a joint lags its
        command by more than $over_q rad (a collision, a pull on something that does not come, a command the arm cannot
        follow), the commanded pose jumps back to where the arm was $back_s s before and the arm stands still for $lock_s s
        (`stopped`, then `locked_s` counting down); moves of the arm commanded meanwhile are ignored. The robot works
        unattended: **at the $latch protective stop of a row section the safety controller latches** and the row section
        ends there (`stops_latched`), as a robot that needs a person to reset it does; what was delivered by then
        counts.""")
    d.p("""**What ends a row section**: `done`; the simulated time is used up (`sim_cap`, $sim_cap s per row section); the
        protective stops latch (`stops_latched`). After the end the arm holds its pose and everything is left to come to
        rest for $settle s before the row section is judged (section 8); the greenhouse's rules apply to what happens to a
        truss in that time too. A request after the end changes nothing. Nothing on the wire reports a cut, a fall, a
        bruise, a crushed peduncle or a lost plant.""")

    d.raw("## 5. The cart, its odometer and the row markers")
    d.p("""The cart rides on the rails of the aisle. It starts a row section at c = 0 and can stand anywhere from
        c = $travel0 to c = $travel1 m; a drive that would pass an end stops there and the observation says
        `cart_limit`.""")
    d.p("""**The odometer** counts the wheels' turns, from 0 at the start of a row section: over a drive it advances by the
        distance the action commanded (as shortened, where it was). **The wheels slip.** The cart travels (1 - s) times
        what the odometer counts, where the slip s is the row's own (the state of its rails): between $slip0 and $slip1
        ($slip_pct %), the same throughout a row section and different from row to row. On top of that, each drive's
        travel scatters by $slip_sigma % of its length (one standard deviation). The slip is not on the wire.""")
    d.p("""**Row markers.** $n_markers square fiducial markers are fixed along the gutter's aisle side, one every
        $marker_pitch m: OpenCV's ArUco dictionary `$dictionary`, identifiers 0 to $last_id in the direction of +y. They
        do not ride with the cart. They are surveyed in the row frame:""")
    d.raw(table(["marker", "centre x, y, z (row frame, m)"], [[k, vec(F.marker_T(k)[:3, 3])] for k in range(mk["count"])]))
    d.p("""A marker's frame has its origin at the marker's centre and its z axis out of the marker's face, which looks up
        and toward the aisle: its axes in the row frame are x = $marker_x, y = $marker_y, z = $marker_z (the face is
        tilted $marker_tilt degrees from the horizontal). The black square is $marker_side m on a side, border included: $marker_bits x
        $marker_bits cells of pattern inside a black border one cell wide, $marker_cells x $marker_cells cells in all, on a
        white plate that reaches $marker_margin mm beyond the square on every side. In the marker's frame the pattern's top
        row is toward +y and its left column toward -x, so the square's corners, in the order OpenCV's detector returns
        them, are (-a, a, 0), (a, a, 0), (a, -a, 0), (-a, -a, 0) with a half the side. The bit patterns (1: white) and the
        poses (4 x 4 matrices, row frame) are in `site.json`, `row_markers`.""")

    d.raw("## 6. Cameras")
    d.p("""Frames are rendered only when an action asks for them, at most $frames per row section in all (each camera of
        each request counts one); a request that would pass the budget gets no frame and the event `cams: refused, the
        frame budget is used up`. `rgb` is a PNG (base64), `depth` a 16-bit PNG (base64) of the distance along the optical
        axis in units of `depth_unit_m` ($depth_unit mm), 0 where there is no reading. `$decode_rgb` and `$decode_depth` read them.""")
    cams = []
    for name, k in site["cameras"].items():
        cams.append("`%s`: %s, %d x %d pixels, fx = fy = %.2f, cx = %g, cy = %g, no distortion; nominally mounted on the %s at %s (m), "
                    "camera axes (x right, y down, z forward) as the columns of R = %s." % (
                        name, "colour and depth" if "depth" in k else "colour only", k["width"], k["height"], k["K"][0][0], k["K"][0][2], k["K"][1][2],
                        "cart (cart frame)" if k["parent"] == "cart" else "flange (flange frame)", vec(k["pos"]),
                        "[" + ", ".join(vec(r) for r in k["R"]) + "]"))
    d.ul(cams)
    d.p("""A pixel (u, v) with depth d is the camera-frame point ((u - cx) d / fx, (v - cy) d / fy, d). Depth is registered
        to the colour image: the same pixel grid and intrinsics. The intrinsics are exact (the cameras' factory
        calibration). The overview camera looks along and across the row at the plants ahead of the robot; the wrist
        camera sits beside the tool, on the side the clamp pad is on (the tool frame's $cam_side), and looks along the
        tool axis, tilted toward the mouth: through its nominal mounting the tool centre point lies $tcp_cam_z m in front of it, at the pixel $tcp_px.""")
    d.p("""**The mounting above is nominal.** A robot unit's cameras are mounted by hand, and each sits off its nominal
        pose by an offset of its own: up to $tol_o_mm mm along and $tol_o_deg degrees about each of its own axes for the
        overview camera (against its nominal place on the cart), up to $tol_w_mm mm and $tol_w_deg degrees for the wrist
        camera (against its nominal place on the flange). The true pose is the nominal pose times the offset (a
        translation and a rotation vector in the camera's own frame). The offset belongs to the robot unit: it is the same
        throughout a row section and differs from unit to unit. Every row section is worked by a unit of its own, the
        graded ones and each practice row alike: a practice unit shows what the units are like, not what a graded unit's
        offsets are. Located through the nominal poses, a point $look m in front of the overview camera (where it looks at the row) is typically
        $err_o cm off (up to about $err_o_max cm), a point $zref m in front of the wrist camera typically $err_w mm (up to
        about $err_w_max mm), and the tool centre point in the wrist camera's view typically $err_t mm (up to about
        $err_t_max mm). What is exact: the robot's kinematic model (`../robot/ur20.json`) and its joint readings, the
        intrinsics, the tool's geometry on the flange, the survey of the row markers (section 5) and of the two boards on
        the cart:""")
    cp, cc = F.CAL_PLATE, F.COLOR_CARD
    d.raw(table(["board", "origin (cart frame, m)", "layout (x by y)", "white or black margin (mm)"], [
        ["calibration plate", vec(cp["origin"]), "%d x %d squares of %s mm" % (cp["squares"][0], cp["squares"][1], mm(cp["square"])), mm(cp["margin"]) + " (white)"],
        ["colour card", vec(cc["origin"]), "%d x %d patches of %s mm at a pitch of %s mm" % (cc["patches"][0], cc["patches"][1], mm(cc["patch"]), mm(cc["pitch"])),
         mm(cc["margin"]) + " (black)"]]))
    d.v.update(seen=("Through its nominal mounting the overview camera has both boards whole in its picture when the arm is not in the "
                     "way: the plate about %.1f m away (a square about %.0f pixels wide), the card about %.1f m away (a patch about %.0f "
                     "pixels wide)." % (plate_z, fo * cp["square"] / plate_z, card_z, fo * cc["patch"] / card_z)) if plate_seen and card_seen else
               "Through its nominal mounting the overview camera does not have both boards whole in its picture.")
    d.p("""A board's frame is parallel to the cart frame, with its origin at the board's corner of least x and least y, on
        its face. On the plate the square (i, j), counted from 0 with i along x and j along y, is black when i + j is even;
        the white plate reaches the margin beyond the squares on every side. On the card the patch (i, j) covers x from
        i times the pitch to that plus the patch's side, and y likewise with j; the black card reaches the margin beyond
        the patches. $seen A wrist frame's `pose` is the wrist camera's nominal pose when the frame was taken: the
        flange's pose at the measured joint angles times the nominal mounting. It does not contain the unit's offset.""")
    d.p("**The colour card's patches** are matt; each one's diffuse reflectance (linear RGB) is:")
    d.raw(table(["patch (i, j)", "red", "green", "blue"],
                [["(%d, %d)" % (n % cc["patches"][0], n // cc["patches"][0])] + [g(v) for v in a] for n, a in enumerate(cc["albedo"])]))
    d.p("""**Depth.** The wrist camera is an active-stereo RGB-D camera. From its data sheet (also in `site.json`,
        `cameras`, `wrist`, `depth`):""")
    d.raw(table(["", "wrist"], [
        ["working range", "%s to %s m" % (g(sw["range_m"][0]), g(sw["range_m"][1]))],
        ["depth noise (1 sigma, temporal, on a well-lit surface; z in m)", "%s z^2 m (%.2f mm at %s m)" % (g(sw["sigma_k"]), 1e3 * sw["sigma_k"] * zref ** 2, g(zref))],
        ["noise on dim surfaces", "rising as the colour image's luma falls below %d, up to twice at %d" % (sw["dim"], sw["dark"][0])],
        ["noise correlation", "over about %s pixels (Gaussian)" % g(sw["corr_px"])],
        ["lateral smoothing of depth", "about %s pixels (Gaussian)" % g(sw["blur_px"])],
        ["stereo baseline, along the camera's +x", "%s m" % g(sw["baseline_m"])],
        ["disparity resolution (depth steps: z^2 x step / (f x baseline))", "%s pixel (%.2f mm at %s m)" % (g(sw["subpixel"]), 1e3 * zref ** 2 * sw["subpixel"] / (fw * sw["baseline_m"]), g(zref))],
        ["no reading at depth jumps", "of more than %s disparity pixels within 5 x 5 pixels (%.0f mm at %s m)" % (g(sw["edge_px"]), 1e3 * sw["edge_px"] * zref ** 2 / (fw * sw["baseline_m"]), g(zref))],
        ["matcher's window", "%d x %d pixels (%.1f mm wide at %s m, %.1f mm at %s m)" % (sw["window_px"], sw["window_px"], 1e3 * win_near, g(sw["range_m"][0]), 1e3 * sw["window_px"] * zref / fw, g(zref))],
    ]))
    d.p("""A pixel has no reading (0) outside the working range (nothing nearer than $range0 m is measured, the tool centre
        point and what is in the mouth included); at a depth jump; on anything narrower in the picture than the matcher's
        window of $window pixels that stands more than $edge_px disparity pixels in front of what is behind it (the
        matcher locks onto the surface that fills its window); where the stereo pair's second view, displaced along the
        camera's +x by the baseline, cannot see the point (a band beside near edges); on dark surfaces (where the colour
        image's luma, 0.299 R + 0.587 G + 0.114 B, is below $dark1, more often the darker, never a reading below $dark0);
        and on bright ones (where a colour channel is above $bright0, more often the brighter, never a reading above
        $bright1). $thin_text These thresholds hold for the colour image under the nominal light: the sensor lights the
        scene itself, so a surface's readings do not change with the row's light (below). Lost readings come in small
        blobs. The noise is independent from frame to frame: the mean of many frames of a still scene approaches the true
        depth, smoothed as stated. The overview camera has no depth.""")
    d.p("""**The colour image** is the renderer's display image: a path tracer with a denoiser, so two frames of the same
        scene differ slightly in their pixel values, and a filmic tone curve ($tone) and the display's
        gamma lie between the light a surface sends to the camera and its pixel value. A pixel value is not proportional
        to a reflectance.""")
    d.p("""**The grow light.** The row's lamps differ from row to row in brightness and in colour: the overall brightness
        lies between $gain0 and $gain1 times the nominal; against the nominal white, the light's green share lies between
        $green0 and $green1 and its blue share between $blue0 and $blue1 of its red share; and each of the $lamps lamp
        panels over the row gives between $lamp0 and $lamp1 of its nominal share. All lamps of a row have the same colour.
        The light stays the same throughout a row section. The colour image shows all of it; the depth does not depend on
        it.""")

    d.raw("## 7. Budgets")
    d.p("""Per row section: $sim_cap s of simulated time (`time_left`), $frames frames (`frames_left`), $max_steps steps,
        and $budget_s s of your program's own time: the time the grader waits for your answers, summed (the cell's
        simulation and rendering are not counted). Every single answer within $step_s s. The time budget and the step
        limit are checked before each request: the answer that crosses the budget is still carried out, and the row
        section ends before the next request.""")

    d.raw("## 8. How a row section is judged")
    d.p("""When a row section has ended and everything has come to rest (section 4), a truss counts when all of this
        holds:""")
    d.ul([
        "it is ready by its cultivar's rule (growing protocol, section 2);",
        """its peduncle has been cut, and no part of its plant's main stem has (a cut main stem loses the plant with all
        its trusses, wherever they are by then);""",
        """it is whole (no part of its rachis has been cut), it is neither bruised nor squeezed, and its peduncle is not
        crushed (growing protocol, section 5);""",
        "the tool's pad does not hold it;",
        """it lies in a crate: every one of its fruits has its centre over the inner footprint of one and the same crate
        (section 1) and not above the top of the crate's walls (z = $wall_top), and no fruit's lowest point is more than
        $floor_slack mm below the top of the crate's floor;""",
        """it lies in one layer: it does not lie on another cut truss, and no cut truss lies on it. A truss lies on another
        when one of its fruits rests on one of the other's: the two fruits touch (their centres are closer than $touch
        times the sum of their radii) and the upper one's centre is higher than the lower one's by more than $stack_dz
        times that sum. This is looked at between any two cut trusses, ready or not, in a crate or not, and neither of
        such a pair counts."""])
    d.p("""The row section's yield is the number of trusses that count divided by the number of ready trusses in the row
        section. Nothing is subtracted: a truss that is not ready counts nothing, cut or not. Where things are is read from
        the physics engine, not from pictures.""")

    d.raw("## 9. Commissioning")
    d.p("""The commissioning service in your sandbox runs this cell on $practice_rows practice row sections. They grow the
        cultivar $practice, not the graded rows' $graded: its colour chart, its readiness rule, its fruits and peduncles
        and its stiffness ranges are another cultivar's (growing protocol, section 2). Each practice row is worked by a
        robot unit of its own: its cameras' mounting offsets (section 6) are the same in every session on that row and
        differ from those of the other practice rows and of every graded unit; and its light, toughness, stiffness and
        slip are that row's own. A practice row shows what rows and units are like, not what a graded row's values are. You
        get what a commissioning engineer has at the robot: the same actions and observations as above, the camera frames,
        the robot's, the cart's and the tool's readings, with the budgets of section 7. You are told nothing else: not
        where anything is or what state it is in, no yield, no debrief.""")
    d.ul([
        """`$session` (in `~/given/lib`) opens a session on practice row 1 to $practice_rows and steps it interactively
        (the service listens on `$socket`); `$launcher_cmd` runs your `run.sh` in a session the way it is run when it is
        graded, repeated steps included.""",
        """**$sessions sessions in all, never refunded**, one at a time: a session is counted when it opens and ends when
        the row section ends, when you close it or after $idle_s s without a request; a session that ends early, for
        whatever reason, is spent. `$status` shows how many are left.""",
        """A session starts Isaac Sim on the sandbox's GPU: opening one takes up to a minute. The service warms up for
        the first few minutes after the sandbox starts (the renderer compiles its shaders once, in a process of the
        service's own); a session asked for in that time waits for it, and is counted as any other."""])
    d.p("""No simulator model of a row is given. The cell's own code is not readable from your account (the cell runs as
        the account `$account`): this document, the tool's data sheet and the growing protocol are what there is. Isaac
        Sim itself is installed in the image (`/opt/venv`), and the robot's kinematic model is in `../robot/ur20.json`;
        you may build simulations of your own.""")

    d.raw("## 10. What the cell is")
    d.p("""The row, the cart and the robot are simulated: rigid bodies on sprung joints in a physics engine for everything
        that moves and collides, a renderer for the cameras, and rules for what a physics engine of rigid bodies does not
        do.""")
    d.ul([
        """The cut and the pad's hold are rules on which stalks cross the blade's and the pad's planes inside the mouth,
        not a model of how a stalk fractures or is squeezed (tool data sheet, section 3). The jaws' bodies stand at the
        full opening at all times: closing the jaws moves the jaws a frame shows, not what collides.""",
        """A main stem is a chain of straight pieces, a truss a peduncle, a rachis in three straight pieces and its
        spherical fruits, a leaf one rigid plate with its stalk, each on sprung joints (growing protocol, section 1).
        Leaves do not collide with each other or with stems.""",
        """Bruising, squeezing and crushing are the rules of the growing protocol, section 5, looked at once per control
        tick; whether a truss lies in a crate, and whether it lies on another, is looked at once, when the row section is
        judged (section 8)."""])
    d.p("""The physics engine and the renderer are not bit-reproducible across machines: the same actions do not give the
        same readings twice. The grader starts `run.sh` with `PATH`, `HOME` and `LANG` only.""")
    return d.text()


def site_depth_unit():
    """The depth image's unit (m per count), as `cell.frames` writes it."""
    return lit("cell", "depth_unit_m=#)")


# ------------------------------------------------------------------------------------------------- the tool's data sheet
def cutter_gripper(C):
    t = F.TOOL
    tz = t["tcp_z"]
    cam = lit("scene", 'b.box(E + "/cam_housing", (#, #, #), (wc["pos"][0] - #, wc["pos"][1], wc["pos"][2] - #)')
    adapter = lit("scene", 'b.cyl(E + "/adapter", #, #,')
    wc = F.CAMERAS["wrist"]["pos"]
    tol = (lit("rules", '!= "stem" and 2.0 * self.member[mid]["r"] >= w1 - #'), lit("rules", 'self.opening > 2.0 * m["r"] + #'))
    if abs(tol[0] - tol[1]) > 1e-12:
        sys.exit("make_documents.py: the pad takes hold within %g m and lets go beyond %g m of a stalk's diameter; the "
                 "data sheet states one tolerance" % tol)
    rr = lit("farm", '"rachis_r": round(rp * #, 5)')
    pet = lit("farm", "^rp = st.uniform(#, #)")
    tcp_cam = kin.inv(F.camera_mount("wrist")) @ np.array([0.0, 0.0, tz, 1.0])
    d = Doc(
        open_max=mm(t["open_max"]), z0=mm(t["mouth_z"][0]), z1=mm(t["mouth_z"][1]), behind=mm(tz - t["mouth_z"][0]),
        ahead=mm(t["mouth_z"][1] - tz), tcp=mm(tz), blade=mm(t["blade_x"]), pad=mm(t["clamp_x"]), apart=mm(t["blade_x"] - t["clamp_x"]),
        pad_side="-x" if t["clamp_x"] < t["blade_x"] else "+x", jaw_speed=mm(t["jaw_speed"]),
        half_step=mm(t["jaw_speed"] * F.TICK / 2),
        force_max=g(t["force_max"]), pad_force=g(t["pad_force"]), tol=mm(tol[0]),
        tough0=g(F.CUT["toughness"][0]), tough1=g(F.CUT["toughness"][1]), crush=g(F.CUT["crush"]),
        open_noise=mm(t["opening_noise"]), force_noise=g(t["force_noise"]), tick=g(F.TICK),
        tcp_cam_z="%.3f" % tcp_cam[2], range0=g(F.SENSORS["wrist"]["range_m"][0]), stem_note=(
            "" if F.CUT["stem_factor"] == 1.0 else " (a main stem's cutting force is %s times that)" % g(F.CUT["stem_factor"])))
    d.raw("# Cutter-gripper: data sheet")
    d.p("""The harvesting tool on the robot's flange. Two jaws close a mouth that is open along the tool axis. A blade at
        one side of the mouth cuts the stalk that lies across the mouth; a clamp pad beside the blade holds the stalk on
        the pad's side of the cut. With it the robot cuts a truss off its plant and carries it by what is left of its
        peduncle.""")

    d.raw("## 1. Geometry")
    d.p("""In the flange frame (origin on the flange face, z out of the flange: the tool axis), in millimetres. The tool
        frame has the same axes and its origin at the tool centre point. Body and neck are boxes centred on the tool
        axis.""")
    d.raw(table(["", ""], [
        ["mass", "%s kg" % g(t["mass"])],
        ["adapter", "a disc of %s diameter, %s thick, on the flange" % (mm(2 * adapter[0]), mm(adapter[1]))],
        ["body", "%s x %s (x by y), from z = %s to %s" % (mm(t["body"]["size"][0]), mm(t["body"]["size"][1]), mm(t["body"]["z"][0]), mm(t["body"]["z"][1]))],
        ["neck", "%s x %s (x by y), from z = %s to %s" % (mm(t["neck"]["size"][0]), mm(t["neck"]["size"][1]), mm(t["neck"]["z"][0]), mm(t["neck"]["z"][1]))],
        ["jaws", "two bars, each from x = %s to %s, %s thick (along y), from z = %s to %s; they close along y, symmetrically about the tool axis" % (
            mm(t["jaw"]["x"][0]), mm(t["jaw"]["x"][1]), mm(t["jaw"]["t"]), mm(t["jaw"]["z"][0]), mm(t["jaw"]["z"][1]))],
        ["mouth", "the gap between the jaws: 0 to %s wide (along y), from z = %s (the throat) to %s (the tips)" % (mm(t["open_max"]), mm(t["mouth_z"][0]), mm(t["mouth_z"][1]))],
        ["tool centre point", "the mouth's centre: on the tool axis at z = %s, the throat %s behind it and the tips %s ahead of it" % (
            mm(tz), mm(tz - t["mouth_z"][0]), mm(t["mouth_z"][1] - tz))],
        ["blade", "in the plane x = %s" % mm(t["blade_x"])],
        ["clamp pad", "in the plane x = %s: on the %s side of the blade, %s from it" % (mm(t["clamp_x"]), d.v["pad_side"], mm(abs(t["blade_x"] - t["clamp_x"])))],
        ["wrist camera", "its housing a box of %s x %s x %s (x, y, z) centred at %s: on the %s side of the tool" % (
            mm(cam[0]), mm(cam[1]), mm(cam[2]), vec([1000 * (wc[0] - cam[3]), 1000 * wc[1], 1000 * (wc[2] - cam[4])], 1), "-x" if wc[0] < 0 else "+x")],
        ["jaw speed", "%s mm/s of opening (%s mm per control tick; %s s from full opening to closed)" % (mm(t["jaw_speed"]), mm(t["jaw_speed"] * F.TICK), g(t["open_max"] / t["jaw_speed"]))],
        ["closing force", "settable from 0 to %s N" % g(t["force_max"])],
        ["readings", "opening (m), force (N): section 4"],
    ]))
    d.p("""Body, neck, jaws, adapter and camera housing are what collides with plants, trusses, crates and cart. The jaws'
        bodies stand at the full opening of $open_max mm at all times; the jaws a camera frame shows follow the commanded
        opening. Through the wrist camera's nominal mounting the tool centre point lies $tcp_cam_z m in front of the
        camera, nearer than the $range0 m from which its depth sensor reads (`../site/site_interface.md`, section 6).""")

    d.raw("## 2. Commanding it")
    d.p("""An action's `gripper` sets the jaws' target opening and their closing force (`../site/site_interface.md`,
        section 3). The jaws move toward the target at the jaw speed, tick by tick, and both values stay in force until
        another `gripper` replaces them.""")

    d.raw("## 3. How the jaws meet a stalk")
    d.p("""A stalk, here, is what a plant has that is long, straight and round: a piece of a main stem, a peduncle, a
        piece of a rachis, a leaf's petiole (growing protocol, section 1). Fruits and leaf blades are not stalks: blade and
        pad do nothing to them, and the tool's bodies push and press them like any body.""")
    d.ul([
        """**In the mouth.** A stalk is in the mouth at a plane x = X when its axis crosses that plane between the jaws'
        colliders (within half the open mouth's width of the tool axis, along y) and between the throat and the tips
        (from $behind mm behind to $ahead mm ahead of the tool centre point, along z). The stalk's axis decides, not its
        thickness.""",
        """**Closing.** On their way to the target opening the jaws meet, at the blade's plane, the thickest uncut stalk
        that is in the mouth there, when the opening has come down to its diameter. With a closing force of that stalk's
        cutting force or more, the blade cuts it through at once and the jaws go on, to the next stalk or to the target.
        With less, the jaws stall on it: the opening stays at, and reads, the stalk's diameter, and the force reads the
        commanded force; a closing force raised to the stalk's cutting force later cuts it then. Jaws that open do nothing
        to a stalk, and a stalk that comes into a mouth that is already closed is neither cut nor held.""",
        """**Closing jaws push a stalk to the middle.** A stalk that is in the mouth, between the jaws at the opening
        they have at that moment, at any tick of a closing stays caught for the rest of that closing, for as long as it
        is in the mouth: the jaws meet it wherever between them it was. (The jaws' bodies stand at the open mouth, section
        1: the stalk itself is not moved, and what the pad holds is held where it is.) Jaws that open let everything
        go.""",
        """**The cutting force** of a stalk is the row's toughness times the stalk's cross-section (pi d^2 / 4 with its
        diameter d in mm)$stem_note. The toughness is the row's own (the season, the irrigation): between $tough0 and
        $tough1 N/mm^2, the same for every stalk of a row section, and not on the wire.""",
        """**The pad** acts at its own plane, $apart mm to the $pad_side side of the blade. It takes hold of every stalk
        other than a main stem that is in the mouth at the pad's plane once the jaws have come down to that stalk's
        diameter (to within $tol mm), and keeps hold of it for as long as the opening is not more than $tol mm above that
        diameter and the closing force is $pad_force N or more. A stalk the pad holds is fixed to the tool and moves with
        it; one that is still on its plant ties the tool to the plant. Opening the jaws, or a closing force below
        $pad_force N, lets go.""",
        """**A cut peduncle frees its truss.** The truss stays in the tool when the pad holds the peduncle on the truss's
        side of the cut: the peduncle crosses the pad's plane inside the mouth, farther from the main stem than it crosses
        the blade's plane. When it crosses the pad's plane nearer to the main stem than the cut, or not inside the mouth
        (beyond the throat or the tips), nothing holds the truss and it falls.""",
        """**Crushing.** The pad presses what it holds with the closing force. A peduncle the pad holds while the closing
        force is above $crush times that peduncle's own cutting force is crushed, at the cut or at any time after it
        (growing protocol, section 5)."""])
    d.p("What the blade can meet, over both cultivars and all rows, and what a cut does:")
    rows = []
    for name, c in F.CULTIVARS.items():
        rows.append(["peduncle, %s" % name, span([2 * v for v in c["peduncle_r"]], mm), force_span(c["peduncle_r"]),
                     "frees its truss: held by the pad, or fallen (above)"])
    lo = min(c["peduncle_r"][0] for c in F.CULTIVARS.values())
    hi = max(c["peduncle_r"][1] for c in F.CULTIVARS.values())
    rows.append(["rachis", "%s times its truss's peduncle (%s)" % (g(rr), span([2 * rr * lo, 2 * rr * hi], mm)), force_span([rr * lo, rr * hi]),
                 "parts the truss: it is waste"])
    rows.append(["petiole of a leaf", span([2 * v for v in pet], mm), force_span(pet), "drops the leaf; nothing else follows"])
    rows.append(["main stem", span([2 * v for v in F.ROW["stem_r"]], mm), force_span(F.ROW["stem_r"]),
                 "the plant is lost: none of its trusses counts, those already in a crate included"])
    d.raw(table(["stalk", "diameter (mm)", "cutting force (N)", "when it is cut"], rows))
    d.p("""A stalk that has been cut once is not cut again. Nothing on the wire says which stalk the jaws met or what was
        cut.""")

    d.raw("## 4. Readings")
    d.ul([
        """**opening** (`gripper.opening`, m): the gap between the jaws, with a noise of $open_noise mm (one standard
        deviation) on every reading. `trace.opening_mm` is the same reading once per control tick, in millimetres.""",
        """**force** (`gripper.force`, N): the force the jaws press with. It is the commanded closing force while the jaws
        are stalled on a stalk, or closed completely at a target opening of 0, and 0 at every other time (jaws that move
        freely or stand open); a reading that is not 0 carries a noise of $force_noise N (one standard deviation).
        `trace.grip_force` is the same reading once per control tick."""])
    return d.text()


# ------------------------------------------------------------------------------------------------- the growing protocol
def growing_protocol(C):
    R = F.ROW
    el = lit("farm", "^el = math.radians(st.uniform(#, #))")
    az = lit("farm", "^az = sign * math.radians(st.uniform(#, #))")
    first = lit("farm", "^first = rf + st.uniform(#, #)")
    gap = lit("farm", "^gap = st.uniform(#, #)")
    ped = lit("farm", "^ped = st.uniform(#, #)")
    small = lit("farm", "^r_j = rf * st.uniform(#, #)")
    shrink = lit("farm", 'c["fruit_r"][1] - # * (n - c["fruits"][0])')
    rr = lit("farm", '"rachis_r": round(rp * #, 5)')
    behind = lit("farm", "m - # * j + st.uniform(#, #)")
    near = lit("farm", "min(stage + #, max(stage - #, s))")
    shade = lit("farm", "^w = st.uniform(#, #)")
    rise = lit("farm", "^rise = math.radians(st.uniform(#, #))")
    pet_len = lit("farm", "^Lp = st.uniform(#, #)")
    pet_r = lit("farm", "^rp = st.uniform(#, #)")
    droop = lit("farm", "^droop = math.radians(st.uniform(#, #))")
    blade = lit("farm", "^length, width = st.uniform(#, #), st.uniform(#, #)")
    leaf_k = lit("farm", '"k": round(st.uniform(#, #), 4)')
    base = lit("farm", 'ROW["x"] + st.uniform(#, #), ROW["first_y"] + ROW["pitch"] * i + st.uniform(#, #)')
    lean = lit("farm", "^lean_y, lean_x = st.uniform(#, #), st.uniform(#, #)")
    kink = lit("farm", "math.sin(lean_x + st.uniform(#, #)), math.sin(lean_y + st.uniform(#, #))")
    z_first = lit("farm", "^zs = [st.uniform(#, #)]")
    z_next = lit("farm", "^zs.append(zs[-1] + st.uniform(#, #))")
    per_plant = lit("farm", "quota[i] < #")
    leaf_dz = lit("farm", "for dz in (st.uniform(#, #), st.uniform(#, #))")
    spiral = lit("farm", "az += math.radians(# + st.uniform(#, #))")
    low_leaf = lit("farm", "leaf_p * (# if k == 0 else #)")
    damping = lit("farm", '"damping": round(st.uniform(#, #), 4)')
    base_k = lit("scene", 'k = mech["stem_k"] * (# if i == 0 else #)')
    cut_at = lit("farm", "^lo, hi = #, L - #")
    clear = sorted({lit("farm", "d.min() < #"), lit("farm", 'item["stem_r"] + #'), lit("farm", 'item["peduncle"]["r"] + #')})
    clear_leaf = sorted({lit("farm", 'item["petiole"]["r"] + #'), lit("farm", 'item["blade"]["width"] / 2 + #')})
    if len(clear) != 1 or len(clear_leaf) != 1:
        sys.exit("make_documents.py: farm._hits keeps more than one clearance from stems, fruits and peduncles (%s) or from "
                 "leaves (%s); the protocol states one each" % (clear, clear_leaf))
    grav = lit("cell", "^G = #")
    if any(abs(a[0] + a[1]) > 1e-12 for a in (base[:2], base[2:], lean[:2], lean[2:], kink[:2], kink[2:], behind[1:], spiral[1:],
                                           (shade[0] - 1.0, shade[1] - 1.0))):
        sys.exit("make_documents.py: a draw of farm.py that the protocol states as +/- is no longer symmetric")
    lean_deg = [math.degrees(math.atan(math.sin(lean[1] + kink[3]))), math.degrees(math.atan(math.sin(lean[3] + kink[1])))]    # along, across
    if tuple(F.APPROACH_ALPHAS) != (0.0,):
        sys.exit("make_documents.py: farm.APPROACH_ALPHAS is no longer (0,); section 4 of the protocol describes that")
    if min(c["peduncle_len"][0] for c in F.CULTIVARS.values()) < cut_at[0] + cut_at[1]:
        sys.exit("make_documents.py: a peduncle can be shorter than farm.cut_fraction's two margins; section 4 of the protocol "
                 "describes the point between them")
    node_z = [list(z_first)]
    for _ in range(R["trusses"] - 1):
        node_z.append([node_z[-1][0] + z_next[0], node_z[-1][1] + z_next[1]])
    d = Doc(
        plants=R["plants"], trusses=R["trusses"], total=R["plants"] * R["trusses"], pitch=g(R["pitch"]), first_y=g(R["first_y"]), row_x=g(R["x"]),
        gutter_z=g(R["gutter_z"]), base_x=mm(base[1]), base_y=mm(base[3]), stem_d=span([2 * v for v in R["stem_r"]], mm),
        stem_h=g(R["stem_segments"] * R["stem_len"]), stem_n=word(R["stem_segments"]), stem_len=g(R["stem_len"]),
        lean_y="%.0f" % lean_deg[0], lean_x="%.0f" % lean_deg[1], base_k=g(base_k[0]),
        node_z="; ".join("%s to %s" % (g(a), g(b)) for a, b in node_z), z_next=span(z_next),
        az=span(az), az1=g(az[1]), el=span(el), rr=g(rr), ped=span(ped, mm), first=span(first, mm), gap=span(gap, mm),
        small=g(round(100 * (1 - small[0]))), shrink=mm(shrink),
        knuckle_k=g(F.TRUSS["knuckle_k"]), rachis_k=g(F.TRUSS["rachis_k"]), twist_k=g(F.TRUSS["twist_k"]),
        leaf_dz0=span(leaf_dz[:2], mm), leaf_dz1=span(leaf_dz[2:], mm), spiral=g(spiral[0]), spiral_pm=g(spiral[2]),
        rise=span(rise), pet_len=span(pet_len, mm), pet_d=span([2 * v for v in pet_r], mm), droop=span(droop),
        blade_len=span(blade[:2], mm), blade_w=span(blade[2:], mm), leaf_k=span(leaf_k),
        leaf_p=span([100 * v for v in R["leaf_p"]]), leaf_low=span([100 * low_leaf[0] * v for v in R["leaf_p"]], lambda v: "%.0f" % v),
        near=g(near[0]), shade=g(round(100 * (shade[1] - 1.0), 6)), behind=g(behind[0]), behind_pm=g(behind[2]),
        ready=span(R["ready"]), per_plant=word(per_plant),
        gain=span(R["light_gain"]), green=span(R["light_g"]), blue=span(R["light_b"]), tough=span(F.CUT["toughness"]),
        damping=span(damping), slip=span([100 * v for v in R["slip"]]),
        back=mm(F.APPROACH_BACK), cut_lo=mm(cut_at[0]), cut_hi=mm(cut_at[1]), clear=mm(clear[0]), clear_leaf=mm(clear_leaf[0]),
        crush=g(F.CUT["crush"]), dv=g(F.DAMAGE["impact_dv"]), tick=g(F.TICK), fall=mm(F.DAMAGE["impact_dv"] ** 2 / (2 * grav), 0),
        accel=g(F.DAMAGE["impact_dv"] / F.TICK), squeeze=g(F.DAMAGE["squeeze_force"]), touch=g(F.DAMAGE["stack_touch"]),
        stack_dz=g(F.DAMAGE["stack_dz"]), latch=ordinal(F.CONSEQUENCE["latch"]),
        graded=C["graded_cultivar"], practice=C["practice_cultivar"])

    d.raw("# Truss tomatoes on the high wire: growing protocol and harvest rules")
    d.p("""This greenhouse grows truss tomatoes on the high wire and harvests them by the truss: a truss is cut off
        whole, with all its fruits on it, once enough of them have coloured. This document says how the crop stands in a
        row, what the two cultivars are, what a row section may look like on the day, and what the greenhouse's rules for
        the harvest are. Positions and heights are in the row frame of `../site/site_interface.md` (section 1); where two
        numbers are given, they are the range over plants or rows.""")

    d.raw("## 1. The plant")
    d.ul([
        """**The row section.** $plants plants stand in a substrate slab on the gutter, one every $pitch m: the first at
        y = $first_y, all at x = $row_x, each within $base_x mm of that in x and $base_y mm in y, the slab's top at
        z = $gutter_z. Each plant carries $trusses trusses within the robot's reach, $total in the row section.""",
        """**The main stem** is $stem_d mm thick and is trained up a support string that hangs from the high wire; $stem_h m
        of it stand in the section. It leans by up to about $lean_y degrees along the row and $lean_x degrees across it,
        and changes its lean a little from one stretch to the next. It gives way: pushed or pulled, it bends and swings
        back, and so does everything that grows on it. In the cell a stem is $stem_n straight pieces of $stem_len m on
        sprung joints (the cultivar's stiffness, section 2; the joint at the slab $base_k times as stiff), and the string
        holds its top sideways as a spring.""",
        """**Where the trusses are.** A plant's trusses grow from the main stem at heights of z = $node_z m, lowest
        first (each $z_next m above the one below).""",
        """**A truss** is a peduncle, a rachis and the fruits. The *peduncle* is the stalk from the main stem to the
        knuckle. It leaves the stem toward the aisle, turned $az degrees to one side of the straight line to the aisle
        (the trusses of a plant to alternating sides; where that would have left a truss without a free approach, section
        4, anywhere within $az1 degrees to either side), and slopes $el degrees below the horizontal. At the *knuckle* the
        truss turns down: from there the *rachis* hangs nearly plumb, $rr times as thick as the peduncle. The *fruits* sit
        on pedicels of $ped mm on alternating sides of the rachis: the first one's pedicel leaves the rachis $first mm
        plus that truss's fruit radius below the knuckle, the next ones every fruit radius plus $gap mm. A fruit is round,
        with a green calyx where its pedicel ends; its skin is glossy, so lamps show on it as highlights. The fruits of a
        truss are within $small % of each other in radius, and a fuller truss has smaller fruits (the cultivar's largest
        radius falls by $shrink mm with every fruit above its fewest).""",
        """**How a truss moves.** A truss hangs from its knuckle like a short, heavy chain. In the cell the peduncle sits on
        the stem with a sprung joint (the cultivar's stiffness, section 2), the rachis is three straight pieces of equal
        length, the knuckle bends against $knuckle_k N m/rad and the rachis's two joints against $rachis_k N m/rad, and
        about its own axis each piece turns against $twist_k N m/rad only (the pedicels give: a truss laid down settles
        flat).""",
        """**Leaves.** Above every truss the stem carries two leaf nodes, $leaf_dz0 mm and $leaf_dz1 mm above the truss, each
        about $spiral degrees (+/- $spiral_pm) further round the stem than the one before. A leaf is a petiole, $pet_len
        mm long and $pet_d mm thick, rising $rise degrees above the horizontal, and beyond it a blade $blade_len mm long
        and $blade_w mm wide that droops $droop degrees below the horizontal. In the cell a leaf is one rigid plate with
        its petiole on a sprung joint at the stem ($leaf_k N m/rad): pushed, it turns aside and comes back.""",
        """**Leaf removal.** The grower takes leaves off from below as the trusses ripen. On the day, $leaf_p % of the
        leaves are still on, depending on the row, and of the two above a plant's lowest truss $leaf_low %; section 4 says
        which leaves are always off."""])
    d.p("What the crop's green parts look like (diffuse reflectance, linear RGB); the fruits' colours are in section 2:")
    d.raw(table(["part", "red", "green", "blue"], [
        ["main stem"] + [g(v) for v in F.LOOK["stem"]],
        ["peduncle, rachis, pedicels, petioles"] + [g(v) for v in F.LOOK["peduncle"]],
        ["leaf blade"] + [g(v) for v in F.LOOK["leaf"]],
        ["calyx"] + [g(v) for v in F.LOOK["calyx"]]]))

    d.raw("## 2. The cultivars")
    d.p("""Two cultivars are grown. The rows your program is graded on grow $graded; the practice rows of the commissioning
        service grow $practice.""")
    names = list(F.CULTIVARS)
    cs = [F.CULTIVARS[n] for n in names]

    def each(f):
        return [f(n, c) for n, c in zip(names, cs)]
    d.raw(table([""] + names, [
        ["fruits on a truss"] + each(lambda n, c: span(c["fruits"])),
        ["a truss's fruit radius (mm)"] + each(lambda n, c: span(c["fruit_r"], mm)),
        ["a fruit's mass (g)"] + each(lambda n, c: "%.0f to %.0f" % (1000 * fruit_mass(c["fruit_r"][0] * small[0]), 1000 * fruit_mass(c["fruit_r"][1]))),
        ["a truss's fruits together (kg)"] + each(lambda n, c: "%.2f to %.2f" % truss_mass(c)),
        ["peduncle length, stem to knuckle (mm)"] + each(lambda n, c: span(c["peduncle_len"], mm)),
        ["peduncle diameter (mm)"] + each(lambda n, c: span([2 * v for v in c["peduncle_r"]], mm)),
        ["rachis length, knuckle to tip (mm)"] + each(lambda n, c: "%.0f to %.0f" % tuple(1000 * v for v in rachis_len(c))),
        ["peduncle's joint to the stem (N m/rad, per truss)"] + each(lambda n, c: span(c["peduncle_k"])),
        ["main stem's joints (N m/rad, per row)"] + each(lambda n, c: span(c["stem_k"])),
        ["support string, sideways (N/m, per row)"] + each(lambda n, c: span(c["string_k"])),
        ["a truss is ready when"] + each(lambda n, c: "at least %s %% of its fruits (rounded up) are at stage %d or beyond" % (g(100 * c["fraction"]), c["stage_min"])),
        ["that is"] + each(lambda n, c: ", ".join("%d of %d" % (needed(n, k), k) for k in range(c["fruits"][0], c["fruits"][1] + 1))),
    ]))
    d.p("""**Ripening stages and the colour chart.** A fruit ripens through the stages 1 to 6. Its stage shows in the
        colour of its skin and nowhere else. Each cultivar has its colour chart: the skin's diffuse reflectance (linear
        RGB) at each stage.""")
    for n, c in zip(names, cs):
        d.raw(table(["%s, stage" % n, "red", "green", "blue"], [[i + 1] + [g(v) for v in rgb] for i, rgb in enumerate(c["chart"])]))
    d.p("""A fruit between two stages has a colour between theirs, in proportion. No fruit is on the border between two
        stages: every fruit's ripeness is within $near of a whole stage. A fruit's own colour is the chart's within $shade %
        (lighter or darker as a whole).""")
    d.p("""**How a plant ripens.** On a truss the fruit nearest the knuckle is the ripest, and each next one is about
        $behind of a stage behind the one before it (every fruit within $behind_pm of that line). On a plant the lowest truss is the ripest. The
        ready trusses of a plant are therefore its lowest ones: none, one or $per_plant. The truss above a plant's ready
        ones is often close behind them, a single fruit short of ready.""")

    d.raw("## 3. What differs from row to row")
    d.p("""No two row sections are alike, and none of the following is on the wire. Each is the row's own and stays the
        same while the row section is worked.""")
    d.ul([
        "where each plant stands, how its stem leans, where its trusses and leaves are and how they hang (section 1);",
        "how many fruits each truss has, their size, and the length and thickness of each peduncle (section 2);",
        "which trusses are ready: $ready of the $total, at most $per_plant on a plant (section 2);",
        "how many leaves are still on (section 1);",
        """the grow light: its brightness, $gain times the nominal, and its colour, a green share of $green and a blue share
        of $blue of its red share against the nominal white (`../site/site_interface.md`, section 6);""",
        """the toughness of the row's stalks, $tough N/mm^2 (the season, the irrigation): what it takes to cut one
        (`../tool/cutter_gripper.md`, section 3);""",
        """how stiff the stems and their strings are (section 2), and how quickly a plant that was touched comes to rest
        (a damping ratio of $damping);""",
        "the rails: the cart's wheels slip by $slip % (`../site/site_interface.md`, section 5)."])

    d.raw("## 4. The free approach")
    d.p("""The grower keeps every ready truss reachable: a leaf that would close a ready truss's free approach has been
        taken off, and a plant's trusses hang so that one is free. What that means exactly: for every ready truss, with
        everything at rest, one approach is free. In it the tool comes with its x axis along the peduncle, pointing to
        the main stem, and its mouth centred on the peduncle's axis midway between the point $cut_lo mm from the stem and
        the point $cut_hi mm short of the knuckle, and it travels the last $back mm in a straight line along its own z
        axis, perpendicular to the peduncle, in the vertical plane through the peduncle and pointing downward (the tool
        comes from above and from the aisle). The approach is free when, on that line, the tool with open jaws, the wrist
        camera's housing and the robot's wrist behind the flange (taken as a cylinder of 0.12 m radius about the tool's
        axis, 0.32 m long) come no nearer than $clear mm to any main stem, any fruit or any other truss's peduncle, and
        no nearer than $clear_leaf mm to any leaf. Nothing is promised about a truss that is not ready or about any
        other way in.""")

    d.raw("## 5. The greenhouse's rules")
    d.p("""A truss is sold whole and unmarked. What follows cannot be undone, and each rule holds from the moment the row
        section starts until it has been judged, on the plant, in the tool and in the crate, for ready trusses and for
        unready ones alike.""")
    d.ul([
        """**Only ready trusses are harvested.** A truss cut before it is ready is worth nothing, and it cannot be put
        back.""",
        """**The cut is made on the peduncle,** between the main stem and the knuckle. A cut through the rachis, anywhere
        along it, parts the truss: it is waste. A cut petiole drops its leaf and costs nothing else. **A cut main stem
        loses the plant:** none of that plant's trusses counts, those already in a crate included.""",
        """**A cut truss must not fall.** With the pad on the plant's side of the cut nothing holds the truss
        (`../tool/cutter_gripper.md`, section 3).""",
        """**Bruised by a blow.** A truss is bruised, and waste, when the velocity of a piece of its rachis, with the fruits on it,
        changes by more than $dv m/s within one control tick of $tick s: a fall, a hard landing, a blow, a jerk of the
        tool that holds it (what a free fall of $fall mm has gathered when it ends, or $accel m/s^2 for one tick).""",
        """**Squeezed.** A truss is squeezed, and waste, when the tool, the robot's wrist or forearm, or the cart with its
        deck and crates presses on one of its fruits or on its rachis with more than $squeeze N (the contact force, looked
        at once per control tick): a fruit pushed against a crate's wall, a truss pressed onto a crate's floor, an arm
        that leans on a truss on its plant.""",
        """**Crushed.** A peduncle that the clamp pad presses with more than $crush times that peduncle's own cutting force
        is crushed, and its truss is waste (`../tool/cutter_gripper.md`, section 3).""",
        """**One layer.** A crate takes one layer of trusses. A truss that lies on another damages both: neither counts. A
        truss lies on another when one of its fruits rests on one of the other's (they touch, their centres closer than
        $touch times the sum of their radii, and the upper one's centre is higher by more than $stack_dz times that sum).
        It makes no difference whether either of the two was ready.""",
        """**The robot works unattended.** At the $latch protective stop in a row section its safety controller latches,
        and the work on that row section ends (`../site/site_interface.md`, section 4)."""])

    d.raw("## 6. A delivered truss")
    d.p("""A ready truss is delivered, and counts, when it has been cut on its peduncle, its plant still stands, it is
        whole, not bruised, not squeezed and not crushed, the tool has let go of it, and it lies in one of the cart's two
        crates in a single layer, every fruit inside the crate. The exact conditions are in `../site/site_interface.md`,
        section 8. A row section's yield is the number of its ready trusses that count divided by the number of its ready
        trusses; nothing is subtracted.""")
    return d.text()


# ------------------------------------------------------------------------------------------------- the robot
def robot_json():
    return {"name": "Universal Robots UR20", "joint_names": list(kin.JOINT_NAMES),
            "note": "every joint turns about the z axis of its own frame; `joints` gives each joint frame in its parent link "
                    "(translation in m, quaternion w x y z), as in the robot's USD description; the flange is wrist_3_link; the base "
                    "frame is the cart frame",
            "joints": [{"name": n, "translation": list(p), "quaternion_wxyz": list(q)} for n, (p, q) in zip(kin.JOINT_NAMES, kin.JOINTS)],
            "limits_rad": kin.Q_LIMITS.tolist(), "home_q": F.CART["home_q"],
            "tool_centre_point_z": F.TOOL["tcp_z"],
            "tool_centre_point": {"parent": "flange", "translation": [0.0, 0.0, F.TOOL["tcp_z"]],
                                  "note": "the tool frame has the flange frame's axes; `tcp` on the wire is the flange's pose times this"},
            "rail_joint": {"name": "cart_joint", "type": "prismatic", "parent": "rail (row frame)", "child": "base (cart frame)",
                           "axis": [0.0, 1.0, 0.0], "position_at_start": 0.0, "travel": list(F.CART["travel"]),
                           "speed_max": F.CART["speed_max"],
                           "note": "the cart's place along the rail; it is not on the wire (the odometer is: site_interface.md, section 5)"}}


# ------------------------------------------------------------------------------------------------- all of them
def documents():
    """{path: text} of every file this tool writes, as the cell's numbers have them now."""
    C = conditions()
    site = json.dumps(F.public_site(), indent=1) + "\n"
    return {
        os.path.join(GIVEN, "site", "site.json"): site,
        os.path.join(TASK, "solution", "reference", "site.json"): site,
        os.path.join(GIVEN, "site", "site_interface.md"): site_interface(C),
        os.path.join(GIVEN, "tool", "cutter_gripper.md"): cutter_gripper(C),
        os.path.join(GIVEN, "crop", "growing_protocol.md"): growing_protocol(C),
        os.path.join(GIVEN, "robot", "ur20.json"): json.dumps(robot_json(), indent=1) + "\n",
    }


def stale(docs=None):
    """The files (relative to the task) that are not on disk as `documents()` has them."""
    out = []
    for path, text in (docs or documents()).items():
        try:
            with open(path) as f:
                same = f.read() == text
        except OSError:
            same = False
        if not same:
            out.append(os.path.relpath(path, TASK))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="write nothing; exit 1 when a file on disk differs")
    args = ap.parse_args()
    docs = documents()
    pending = conditions()["pending"]
    if pending:
        print("note: not there yet, so the task's specified values stand in for theirs: " + ", ".join(pending))
    if args.check:
        old = stale(docs)
        for name in old:
            print("STALE:", name)
        print("%d of %d files are what make_documents.py writes now" % (len(docs) - len(old), len(docs)))
        return 1 if old else 0
    for path, text in docs.items():
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
        print("wrote", os.path.relpath(path, TASK))
    return 0


if __name__ == "__main__":
    sys.exit(main())
