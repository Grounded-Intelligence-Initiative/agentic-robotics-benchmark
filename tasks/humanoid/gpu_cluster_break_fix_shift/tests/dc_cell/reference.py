"""The task's reference program (grader side, privileged): the night shift worked by the site's procedures, inside
the cell (server.reference_shift). It knows the shift's scenario (the cluster's truth) instead of diagnosing it, and
reads serial numbers and capture results from the true state; everything it changes it changes the way an agent
would: robot commands, the belt tools and the site services, with the services' own costs in sim time.

Layout: the services run as small state machines (generators that yield a condition to wait for) advanced from the
robot's background tick, so power cycles, boots, diagnostics, drains and soaks progress while the robot works; the
physical jobs (hot swaps, link work, the GPU tray) run one at a time in the main loop as soon as their services allow.
"""

from __future__ import annotations

import math

from . import cluster as CL
from .refgpu import GpuWork
from .refnode import NodeFront
from .refparts import PartWork, other
from .refrear import RearWork
from .refskills import RefSkills

FEEDS = ("A1", "A2", "A3", "B1", "B2", "B3")   # a GPU rack's node supplies: PSU1/3/5 on A1-A3, PSU2/4/6 on B1-B3


class Reference:
    def __init__(self, cell, log=None):
        self.c = cell
        self.log = log or (lambda *a: None)
        self.S = RefSkills(cell, log=self.log, background=self._tick)
        self.N, self.G, self.R = NodeFront(self.S), GpuWork(self.S), RearWork(self.S)
        self.P = PartWork(self.S, self.R)
        self.cl = cell.cluster
        self.scn = self.cl.scn
        self.machines = {}            # name -> [generator, condition]
        self.ready = {}               # physical jobs whose services allow them: name -> callable
        self.done_jobs = set()
        self.tickets = {}             # kind ("gpu", "link", "hotswap", "mid") -> ticket id
        self.notes = {}               # ticket id -> [str]
        self.flags = {}
        self.assets0 = {}

    # ------------------------------------------------------------------------------------------------ services
    def noc(self, method, **params):
        r = self.c.noc(method, params)
        if isinstance(r, dict) and r.get("error"):
            self.log("[ref] %s %s -> %s" % (method, params, r))
        return r

    def note(self, tid, text):
        self.notes.setdefault(tid, []).append("%s %s" % (CL._label(self.c.sim_time)[11:16], text))

    def notes_of(self, tid):
        return "; ".join(self.notes.get(tid, []))

    def start(self, name, gen):
        self.machines[name] = [gen, (lambda: True)]

    def _tick(self):
        """Advance every service machine whose condition holds (from the robot's background hook, once a second)."""
        if self.c.done:
            return
        for name in list(self.machines):
            m = self.machines.get(name)
            if m is None:
                continue
            gen, cond = m
            try:
                guard = 0
                while cond() and guard < 20:
                    guard += 1
                    cond = next(gen)
                    m[1] = cond
            except StopIteration:
                self.machines.pop(name, None)

    def wait_for(self, cond, timeout, chunk=2.0):
        """Run the robot's clock (holding still) until cond() or the timeout; the service machines advance."""
        t0 = self.c.sim_time
        while not cond() and self.c.sim_time - t0 < timeout and not self.c.done:
            self.S.wait(chunk)
        return cond()

    # waits a machine yields
    def after(self, seconds):
        t = self.c.sim_time + seconds
        return lambda: self.c.sim_time >= t

    def power_state(self, h):
        r = self.cl.api("bmc.power_state", {"host": h}, self.c.sim_time)
        return r.get("state") if isinstance(r, dict) else None

    def ticket_of(self, kind, host):
        for tk in self.cl.tickets.values():
            if tk.kind == kind and tk.host == host and tk.visible_at <= self.c.sim_time + 1e-9:
                return tk.id
        return None

    # ---- power sequences (generators)
    def g_power_off_ac_off(self, h):
        rack = CL.host_rack(h)
        if self.power_state(h) in CL.ON_STATES:
            self.noc("bmc.power", host=h, action="off")
            yield lambda: self.power_state(h) == "standby"
        for o in FEEDS:
            self.noc("pdu.set", rack=rack, outlet=o, state="off")
        self.flags["ac_off_" + h] = self.c.sim_time
        yield self.after(31.0)

    def g_ac_on_power_on(self, h):
        rack = CL.host_rack(h)
        for o in FEEDS:
            self.noc("pdu.set", rack=rack, outlet=o, state="on")
        self.flags.pop("ac_off_" + h, None)
        yield lambda: self.power_state(h) == "standby"
        self.noc("bmc.power", host=h, action="on")
        yield lambda: self.power_state(h) == "on"

    def g_dcgm(self, h, level, out):
        r = self.noc("dcgm.run", host=h, level=level)
        rid = r.get("run_id")
        out["run"] = rid
        if rid is None:
            out["result"] = None
            return
        yield lambda: (self.cl.api("dcgm.result", {"run_id": rid}, self.c.sim_time) or {}).get("status") in ("done", "aborted")
        res = self.cl.api("dcgm.result", {"run_id": rid}, self.c.sim_time)
        out["result"] = res.get("result") if res.get("status") == "done" else "aborted"
        out["gpus"] = res.get("gpus")

    # ------------------------------------------------------------------------------------------------ the shift
    def run(self):
        S, c = self.S, self.c
        self.log("[ref] scenario: %s" % {k: {kk: vv for kk, vv in v.items() if kk not in ("seed",)} if isinstance(v, dict) else v
                                       for k, v in self.scn.items()})
        self.open_shift()
        while not c.done:
            job = self.next_job()
            if job is None:
                if not self.machines and not self.pending_jobs():
                    break
                S.wait(3.0)
                continue
            name, fn = job
            S.phase = name
            self.log("[ref] t %.0f: %s" % (c.sim_time, name))
            try:
                fn()
            except Exception:                   # a job that fails does not end the shift: the next one goes on
                import traceback
                self.log("[ref] %s failed:\n%s" % (name, traceback.format_exc()))
                self.recover()
            self.done_jobs.add(name)
        try:
            self.closing()
        except Exception:
            import traceback
            self.log("[ref] closing failed:\n%s" % traceback.format_exc())
        return True

    def recover(self):
        """After a failed job: what the hands hold let go where it is safe (a tool back to the belt), the arms in."""
        S = self.S
        for sd in ("left", "right"):
            n = S.holds(sd)
            if n and n.startswith("tool_"):
                try:
                    S.stow_tool(sd, n)
                except Exception:
                    pass
        try:
            S.tuck(walk=True)
        except Exception:
            pass

    def pending_jobs(self):
        return [n for n in self.ready if n not in self.done_jobs]

    def next_job(self):
        self._tick()
        best = None
        for name, rec in self.ready.items():
            cond, fn, prio = rec
            if name in self.done_jobs or not cond():
                continue
            if best is None or prio < best[0]:
                best = (prio, name, fn)
        if best is None:
            return None
        self.ready.pop(best[1])
        return best[1], best[2]

    def job(self, name, cond, fn, prio=5):
        self.ready[name] = (cond, fn, prio)

    def open_shift(self):
        """The tickets read and taken, the distractors dealt with, the drains and power-downs started."""
        sc = self.scn
        g, ln, hs = sc["gpu"], sc["link"], sc["hotswap"]
        for h in CL.HOSTS:
            self.assets0[h] = self.noc("dcim.assets", host=h)
        for leaf in CL.LEAVES:
            self.assets0[leaf] = self.noc("dcim.assets", host=leaf)
        self.tickets["gpu"] = self.ticket_of("gpu", g["host"])
        self.tickets["link"] = self.ticket_of("link", ln["host"])
        self.tickets["hotswap"] = self.ticket_of("psu" if hs["cause"] == "psu_failed" else "fan", hs["host"])
        for kind in ("gpu", "link", "hotswap"):
            tid = self.tickets[kind]
            if tid:
                self.noc("noc.ticket", id=tid)
                self.noc("noc.update", id=tid, status="in_progress", notes="taken by the night shift")
        dis = sc["distractors"]
        if dis.get("identify"):
            self.noc("bmc.identify", host=dis["identify"], state="off")
        if dis.get("fan_degraded"):
            fd = dis["fan_degraded"]
            self.noc("noc.create", summary="%s FAN%d degraded (one rotor slow), no ticket: left in place" % (fd["host"], fd["fan"]),
                     host=fd["host"], component="FAN%d" % fd["fan"],
                     notes="%s FAN%d reports degraded (one rotor below its threshold); not covered by a ticket, not "
                           "replaced this shift (SOP-THM-01 1.3)" % (fd["host"], fd["fan"]))
        self.start("gpu", self.g_gpu())
        self.start("link", self.g_link(ln, "link"))
        self.start("mid", self.g_mid())
        self.job("hotswap", lambda: True, self.j_hotswap, prio=1)

    # ------------------------------------------------------------------------------------------------ the GPU ticket
    def g_gpu(self):
        g = self.scn["gpu"]
        h, cause, slot = g["host"], g["cause"], g["slot"]
        tid = self.tickets["gpu"]
        self.note(tid, "GPU%d = %s (vendor map); cause %s" % (g["gpu"], slot.upper(), cause))
        evid = []
        if cause == "baseboard":
            if self.power_state(h) != "on":
                yield from self.g_ac_on_power_on(h)
            r1 = {}
            yield from self.g_dcgm(h, 1, r1)
            evid.append(r1.get("run"))
            self.note(tid, "DCGM L1 %s with the original module: %s" % (r1.get("run"), r1.get("result")))
        yield from self.g_power_off_ac_off(h)
        if cause == "transient":
            self.note(tid, "cold power cycle: host off, outlets %s off >= 30 s, on" % ",".join(FEEDS))
            yield from self.g_ac_on_power_on(h)
        else:
            self.flags["tray_ready_" + h] = True
            self.job("gpu_tray", lambda: True, self.j_gpu_tray, prio=2)
            yield lambda: self.flags.get("tray_done_" + h)
            yield from self.g_ac_on_power_on(h)
        if cause == "baseboard":
            r2 = {}
            yield from self.g_dcgm(h, 1, r2)
            evid.append(r2.get("run"))
            self.note(tid, "slot-swap test: DCGM L1 %s with the spare in %s: %s" % (r2.get("run"), slot.upper(), r2.get("result")))
            yield from self.g_power_off_ac_off(h)
            self.flags["tray_done_" + h] = False
            self.job("gpu_tray_back", lambda: True, self.j_gpu_tray_back, prio=2)
            yield lambda: self.flags.get("tray_done_" + h)
            rack = CL.host_rack(h)
            for o in FEEDS:
                self.noc("pdu.set", rack=rack, outlet=o, state="on")
            self.noc("noc.escalate", id=tid, to="vendor", evidence=[e for e in evid if e],
                     notes="the fault stays with %s: GPU%d fails with the original module (%s) and with the spare (%s); "
                           "every module back in its own slot, the spare back on the cart" % (slot.upper(), g["gpu"], evid[0], evid[1]))
            self.noc("noc.update", id=tid, status="escalated", root_cause="gpu_baseboard", notes=self.notes_of(tid))
            self.flags["gpu_done"] = True
            return
        r = {}
        yield from self.g_dcgm(h, 1, r)
        self.note(tid, "DCGM L1 %s: %s" % (r.get("run"), r.get("result")))
        r = {}
        yield from self.g_dcgm(h, 2, r)
        self.note(tid, "DCGM L2 %s: %s" % (r.get("run"), r.get("result")))
        if r.get("result") == "pass":
            self.noc("sched.resume", node=h)
        rc = {"transient": "gpu_transient", "unseated": "gpu_unseated"}.get(cause, "gpu_module_fault")
        parts = self.parts_record(tid)
        self.noc("noc.update", id=tid, status="resolved" if r.get("result") == "pass" else "in_progress", root_cause=rc,
                 parts=parts, notes=self.notes_of(tid))
        self.flags["gpu_done"] = True

    def j_gpu_tray(self):
        g = self.scn["gpu"]
        h, cause, slot = g["host"], g["cause"], g["slot"]
        N, G, P, S = self.N, self.G, self.P, self.S
        tid = self.tickets["gpu"]
        N.tray_extract(h)
        N.cover_open(h)
        G.unscrew(h, slot)
        G.leave_driver()
        port = "%s/%s" % (h, slot.lower())
        spare_side = P.pick_spare_side("gpu", port) if cause != "unseated" else None
        side, px = G.module_out(h, slot, side=other(spare_side) if spare_side else None)
        if px is None:
            self.log("[ref] %s/%s: module did not come out" % (h, slot))
        elif cause == "unseated":
            G.module_in(h, slot, side, px)
            self.note(tid, "reseated the module in %s (screws 1-4-2-3, snug then 0.9 N m)" % slot.upper())
        else:
            # the other hand takes the spare off the cart (held the way it goes into this slot); the old module goes
            # to the RMA tote (faulty) or onto the spare's spot on the cart (the baseboard test: it goes back in later)
            G.lift_clear(side)
            P.carry(side)
            if cause == "baseboard":
                spare = P.take_spare("gpu", other(side), port)
                P.put_on_cart(side, "px_gpu_0", port)       # the suspect module where the spare lay (it goes back in)
            else:
                P.to_tote(side)                             # the faulty module into the RMA tote first, then the spare
                spare = P.take_spare("gpu", other(side), port)
            if spare is not None and not G.module_in(h, slot, other(side), spare) and S.holds(other(side)) == spare:
                # it did not go in (no way over the slot found): back where it came from, the tray closed up
                P.put_on_cart(other(side), "px_gpu_0", port)
            self.note(tid, "%s module in %s replaced by the cart's spare" % ("suspect" if cause == "baseboard" else "faulty", slot.upper()))
        G.screw_down(h, slot)
        G.leave_driver()
        N.cover_close(h)
        N.tray_insert(h)
        S.tuck(walk=True)
        self.flags["tray_done_" + h] = True

    def j_gpu_tray_back(self):
        """The baseboard test undone: the spare out of the slot back onto the cart, the original module back in."""
        g = self.scn["gpu"]
        h, slot = g["host"], g["slot"]
        N, G, P = self.N, self.G, self.P
        N.tray_extract(h)
        N.cover_open(h)
        G.unscrew(h, slot)
        G.leave_driver()
        port = "%s/%s" % (h, slot.lower())
        spare_side = P.pick_spare_side("gpu", port)
        side, px = G.module_out(h, slot, side=other(spare_side) if spare_side else None)
        if px is not None:
            G.lift_clear(side)
            P.carry(side)
            orig = P.take_spare("gpu", other(side), port)     # the original module, waiting on the cart
            P.put_on_cart(side, "px_gpu_0", port)             # the spare back where it came from
            if orig is not None and not G.module_in(h, slot, other(side), orig) and self.S.holds(other(side)) == orig:
                P.put_on_cart(other(side), "px_gpu_0", port)
        G.screw_down(h, slot)
        G.leave_driver()
        N.cover_close(h)
        N.tray_insert(h)
        self.S.tuck(walk=True)
        self.flags["tray_done_" + h] = True

    # ------------------------------------------------------------------------------------------------ link tickets
    def g_link(self, spec, key):
        """A link ticket: drained, the physical work, the counters cleared, a 600 s soak, back in service."""
        h, k, cause = spec["host"], spec["k"], spec["cause"]
        tid = self.tickets.get(key) or self.ticket_of("link", h)
        self.tickets[key] = tid
        self.noc("sched.drain", node=h, reason=tid or "link")
        yield lambda: self.cl.sched.state[h] in ("drained", "down")
        self.flags["link_ready_" + key] = True
        self.job("link_" + key, lambda: True, lambda: self.j_link(spec, key), prio=3)
        yield lambda: self.flags.get("link_work_done_" + key)
        if cause == "scratched_cable_end":
            cap = self.flags.get("scratch_capture_" + key)
            self.noc("noc.escalate", id=tid, to="cabling", evidence=[cap] if cap else [],
                     notes="core scratch on the trunk's connector at %s (capture %s); both ends cleaned and plugged back" % (
                         self.flags.get("scratch_end_" + key), cap))
            self.noc("noc.update", id=tid, status="escalated", root_cause="link_damaged_fiber", notes=self.notes_of(tid))
            self.flags[key + "_done"] = True
            return
        o, nm, lm, lo = CL.link_ports(h, k)
        leaf = lm.split("/")[0]
        self.noc("fabric.clear", device=h, port=k + 1)
        self.noc("fabric.clear", device=leaf, port=int(lm.split("mpo")[1]))
        r = self.noc("fabric.soak", device=h, port=k + 1, seconds=620)
        sid = r.get("soak_id")
        self.note(tid, "soak %s started" % sid)
        yield lambda: (self.cl.api("fabric.soak_result", {"id": sid}, self.c.sim_time) or {}).get("status") == "done"
        res = self.cl.api("fabric.soak_result", {"id": sid}, self.c.sim_time)
        self.note(tid, "soak %s: %s" % (sid, "passed" if res.get("passed") else "failed"))
        if res.get("passed"):
            self.noc("sched.resume", node=h)
        rc = CL.CAUSE_CODE[cause]
        self.noc("noc.update", id=tid, status="resolved" if res.get("passed") else "in_progress", root_cause=rc,
                 parts=self.parts_record(tid), notes=self.notes_of(tid))
        self.flags[key + "_done"] = True

    def faces_of_link(self, h, k):
        o, nm, lm, lo = CL.link_ports(h, k)
        return {"node": (nm, o), "leaf": (lm, lo)}

    def j_link(self, spec, key):
        """The physical work on a link: per its cause, at the end(s) where it is."""
        h, k, cause = spec["host"], spec["k"], spec["cause"]
        tid = self.tickets[key]
        ends = self.faces_of_link(h, k)
        if cause.startswith("aged_tx"):
            end = "node" if cause.endswith("node") else "leaf"
            mpo_port, osfp_port = ends[end]
            kind = "osfp_n" if end == "node" else "osfp_s"
            spare_side = self.P.pick_spare_side(kind, osfp_port) or "left"
            work_side = other(spare_side)
            spare = self.P.take_spare(kind, spare_side, osfp_port)
            old = self.P.osfp_swap(osfp_port, spare, spare_side)
            if old is not None:
                self.P.to_tote(work_side if self.S.holds(work_side) == old else other(work_side))
            self.note(tid, "transceiver at %s replaced (laser evidence: tx low, bias high on the far end's lane)" % osfp_port)
        else:
            order = ["node", "leaf"] if "node" in cause or cause == "scratched_cable_end" else ["leaf", "node"]
            for end in order:
                mpo_port, osfp_port = ends[end]
                self.inspect_end(mpo_port, osfp_port, key, tid)
                self.close_rear_doors()     # (an open rear door swings across the hot aisle: shut before moving on)
        self.close_rear_doors()
        self.flags["link_work_done_" + key] = True

    def inspect_end(self, mpo_port, osfp_port, key, tid):
        """One end of a link: the MPO out, its face and the receptacle's inspected, cleaned while dirty (at most five
        cleanings), inspected again; a core scratch on the connector is kept as evidence; the MPO back in."""
        S, c, R = self.S, self.c, self.R
        dev = mpo_port.split("/")[0]
        self.noc("fabric.locate", device=dev, port=int(mpo_port.split("mpo")[1]), state="on")
        ts = R.tool_side(mpo_port)
        hold = other(ts) if ts else "right"
        R.door_open(S.rack_of(dev))
        px = R.unplug(mpo_port, hold)
        if px is None:
            self.log("[ref] %s: the connector did not come out" % mpo_port)
            return
        mf = "mpo:" + str(c.pool.instance[px])
        rf = "rx:" + str(c.ports.instance[osfp_port])
        R.step_back(0.30)
        self.face_work(mf, None, key, tid, mpo_port)
        R.hold_close(hold)
        self.face_work(rf, mpo_port, key, tid, mpo_port)
        R.step_back(0.30)
        R.put_tool_back()
        R.plug(px, mpo_port, hold)
        self.noc("fabric.locate", device=dev, port=int(mpo_port.split("mpo")[1]), state="off")

    def face_work(self, fid, port, key, tid, where):
        R = self.R
        cap = R.scope(fid, port=port) or {}
        d = cap.get("defects") or {}
        self.note(tid, "%s face %s: %s" % (where, cap.get("id"), self.defect_text(d)))
        if d.get("scratches_core"):
            self.flags["scratch_capture_" + key] = cap.get("id")
            self.flags["scratch_end_" + key] = where
            return
        n = 0
        while self.dirty(d) and n < 5:
            R.clean(fid, port=port, clicks=1)
            n += 1
            cap = R.scope(fid, port=port) or {}
            d = cap.get("defects") or {}
            self.note(tid, "%s cleaned (%d), %s: %s" % (where, n, cap.get("id"), self.defect_text(d)))

    @staticmethod
    def dirty(d):
        return bool(d.get("particles_core") or d.get("particles_cladding") or d.get("films"))

    @staticmethod
    def defect_text(d):
        if not d:
            return "no picture"
        bits = ["%s %d" % (k, v) for k, v in d.items() if isinstance(v, int) and v and k != "damaged"]
        return ", ".join(bits) if bits else "clean"

    # ------------------------------------------------------------------------------------------------ hot swaps
    def j_hotswap(self):
        hs = self.scn["hotswap"]
        h = hs["host"]
        tid = self.tickets["hotswap"]
        if hs["cause"] == "psu_failed":
            k = hs["psu"]
            port = "%s/psu%d" % (h, k)
            spare_side = self.P.pick_spare_side("psu", port) or "left"
            spare = self.P.take_spare("psu", spare_side, port)
            if hs.get("loose"):
                j = hs["loose_psu"]
                self.R.door_open(self.S.rack_of(h))
                cord = self.P.cord_out(h, j, other(spare_side))
                if cord is not None:
                    self.P.cord_in(cord, h, j, other(spare_side))
                self.note(tid, "PSU%d had no AC: its cord reseated at the inlet" % j)
            failed = self.P.psu_swap(h, k, spare, spare_side)
            if failed is not None:
                self.P.to_tote(other(spare_side))
            self.close_rear_doors()
            self.note(tid, "PSU%d (failed, amber) hot-swapped for the cart's spare" % k)
            rc = "psu_failed"
        else:
            k = hs["fan"]
            port = "%s/fan%d" % (h, k)
            spare_side = self.P.pick_spare_side("fan", port) or "left"
            spare = self.P.take_spare("fan", spare_side, port)
            failed = self.P.fan_swap(h, k, spare, spare_side)
            if failed is not None:
                self.P.to_tote(other(spare_side))
            self.note(tid, "FAN%d (failed, 0 RPM) hot-swapped for the cart's spare" % k)
            rc = "fan_failed"
        self.S.wait(5.0)
        self.noc("noc.update", id=tid, status="resolved", root_cause=rc, parts=self.parts_record(tid), notes=self.notes_of(tid))

    # ------------------------------------------------------------------------------------------------ mid-shift
    def g_mid(self):
        mid = self.scn["mid"]
        yield lambda: self.c.sim_time >= mid["t"] + 1.0
        kind = "fan" if mid["kind"] == "fan_failed" else "link"
        tid = self.ticket_of(kind, mid["host"])
        self.tickets["mid"] = tid
        if tid:
            self.noc("noc.ticket", id=tid)
            self.noc("noc.update", id=tid, status="in_progress", notes="taken")
        if kind == "fan":
            self.job("mid_fan", lambda: True, self.j_mid_fan, prio=0)
            yield lambda: self.flags.get("mid_done")
            return
        spec = {"host": mid["host"], "k": mid["k"], "cause": mid["cause"]}
        yield from self.g_link(spec, "mid")
        self.flags["mid_done"] = True

    def j_mid_fan(self):
        mid = self.scn["mid"]
        h, k = mid["host"], mid["fan"]
        tid = self.tickets["mid"]
        port = "%s/fan%d" % (h, k)
        spare_side = self.P.pick_spare_side("fan", port) or "left"
        spare = self.P.take_spare("fan", spare_side, port)
        failed = self.P.fan_swap(h, k, spare, spare_side)
        if failed is not None:
            self.P.to_tote(other(spare_side))
        self.note(tid, "FAN%d (failed) hot-swapped for the cart's spare" % k)
        self.S.wait(5.0)
        self.noc("noc.update", id=tid, status="resolved", root_cause="fan_failed", parts=self.parts_record(tid),
                 notes=self.notes_of(tid))
        self.flags["mid_done"] = True

    # ------------------------------------------------------------------------------------------------ records, closing
    def parts_record(self, tid):
        """The parts record of a ticket as the site wants it: what left and what entered each position (serials from
        DCIM's shift-start assets and the BMC / fabric inventory now; the cell's ledger is the same truth)."""
        tk = self.cl.tickets.get(tid)
        if tk is None:
            return []
        moves = self.cl._true_moves(self.cl._ticket_scope_ports(tk))
        return [{"removed": m["removed"], "installed": m["installed"], "slot": m.get("slot")} for m in moves]

    def close_rear_doors(self):
        S = self.S
        for row in ("A", "B"):
            for idx in range(1, 6):
                rack = "%s%02d" % (row, idx)
                if self.c.joint_q("rdoor_" + rack) > 0.03:
                    S.tuck(walk=True)
                    self.R.door_close(rack)

    def closing(self):
        """The shift's end: every rear door shut, nothing held, every outlet on, the robot parked out of the aisles."""
        S, c = self.S, self.c
        for side in ("left", "right"):
            h = S.holds(side)
            if h and h.startswith("tool_"):
                S.stow_tool(side, h)
            elif h:
                self.P.to_tote(side)
        self.close_rear_doors()
        for rack in CL.GPU_RACKS:
            for o in FEEDS:
                if self.cl.pdu[rack][o] != "on":
                    self.noc("pdu.set", rack=rack, outlet=o, state="on")
        S.tuck(walk=True)
