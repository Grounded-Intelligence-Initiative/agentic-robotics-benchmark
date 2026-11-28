"""The cell's supervisor: the timeline of one hand unit (CONTRACT.md section 8), tick by tick.

    sup = Supervisor(scenario, unit_index, session, up_h=...)
    sup.start()                       at t = 0 (the hand-over)
    sup.after_tick()                  after every 1 ms plant step: measurements, events, phase changes
    sup.on_reply(grasped, decline)    at the tick a step reply is written (the start of a cycle)
    sup.view() -> SupervisorView      the read-only state for the reference script
    sup.cell_obs(), sup.obs_event()   the "cell" and "event" blocks of an observation
    sup.done, sup.records             the unit is over; one job record per job (contract/schemas/job_record.json)
    sup.abort()                       the plant can run no further: the job in hand is closed, every job not yet run
                                      is voided, the unit is over (no call to the plant)

The timeline: hand-over at t = 0 (every servo off), shift 1 at 3.0 s. A job: "await_clear" (the ticket is shown; the
specimen is placed in the pick nest at the first tick every hand geometry clears it by 2 mm; not within 2.0 s: lost,
not presented, and the next job starts); "present" (the closing window ends at the grasped flag or 3.0 s after the
placement: then "release" and the job is lost, not grasped); on the flag "test_lift" (3.0 s); "transfer" (the
profile), "set_down" (0.5 s), "release" (1.0 s; then the specimen is removed), "rest" (2.0 s). A decline in "present"
goes to "release" at once; a decline in "test_lift", or a specimen not lifted at its end, goes to "lower" (0.5 s), then
"release". After job 10 of shift 1, "pause" (10.0 s), then shift 2; after job 10 of shift 2, "end" (1.0 s). An event
(crush, drop, shutdown) removes the specimen at that tick, voids the shift's remaining jobs and goes to "pause" (shift
1) or "end" (shift 2); a shutdown during the pause voids shift 2. The plant is driven only through the Session calls
place, remove, clear_for, set_base and measure.

    cell_payload(ceilings, parts) -> the cell data given to setup (contract/schemas/cell.json)
"""
import math

from contract import constants as C
from contract import names as N

from grader import events as E

TL = C.TIMELINE
MS = {"handover": int(round(1000 * TL["shift1_opens_s"])), "await_clear": int(round(1000 * TL["clear_window_s"])),
      "present": int(round(1000 * TL["grasp_window_s"])), "test_lift": int(round(1000 * TL["test_lift_s"])),
      "lower": int(round(1000 * TL["lower_s"])), "set_down": int(round(1000 * TL["set_down_s"])),
      "release": int(round(1000 * TL["release_window_s"])), "rest": int(round(1000 * TL["rest_s"])),
      "pause": int(round(1000 * TL["pause_s"])), "end": int(round(1000 * TL["end_s"])),
      "decline_check": int(round(1000 * C.SCORE["decline_rest_check_s"]))}
if MS["decline_check"] != MS["release"]:          # the decline is judged at the end of the release window
    raise AssertionError("the decline check and the release window must be equally long")
JOB_PHASES = ("await_clear", "present", "test_lift", "lower", "transfer", "set_down", "release")
EVENT_FAMILY = {"crush_static": "crush", "crush_impact": "crush", "drop_slip": "drop", "drop_contact": "drop",
                "shutdown_overheat": "shutdown", "shutdown_overload": "shutdown"}
MINJERK_PEAKS = (0.5 - math.sqrt(3.0) / 6.0, 0.5 + math.sqrt(3.0) / 6.0)   # acceleration extrema of a min-jerk move


def transfer_ms(profile):
    """The transfer of a profile in whole ms (the motion ends inside its last ms)."""
    return int(math.ceil(1000.0 * C.transfer_duration_s(profile) - 1e-9))


def transfer_peaks_ms(t0_ms, profile):
    """The ticks of the transfer's acceleration peaks (two per move) and of its tilt starts."""
    out = []
    for st in C.transfer_steps(profile):
        if st["kind"] == "move":
            out += [int(t0_ms + round(1000.0 * (st["t0_s"] + f * st["dur_s"]))) for f in MINJERK_PEAKS]
        elif st["kind"] == "tilt":
            out.append(int(t0_ms + round(1000.0 * st["t0_s"])))
    return sorted(out)


def job_ceiling(spec, ceilings):
    """The job's squeeze ceiling (N, brittle), its travel limit (mm, soft) or None (full-load)."""
    if spec["class"] == "brittle":
        return float(ceilings[spec["code"]])
    if spec["class"] == "soft":
        return float(ceilings["SP_travel_mm"])
    return None


def _profile_public(p):
    tilt = C.PROFILE_TILT[p]
    tr = C.MOTION["transfer"]
    seq = [{"kind": "move", "delta_m": [0.0, 0.0, tr["up_m"]]}]
    if tilt:
        seq.append({"kind": "tilt", "sign": 1})
    seq += [{"kind": "move", "delta_m": [tr["x_m"], 0.0, 0.0]}, {"kind": "move", "delta_m": [0.0, tr["y_m"], 0.0]},
            {"kind": "dwell", "s": tr["dwell_s"]}]
    if tilt:
        seq.append({"kind": "tilt", "sign": -1})
    seq.append({"kind": "move", "delta_m": [0.0, 0.0, -tr["up_m"]]})
    return {"tilt": None if tilt is None else {"axis": list(tilt["axis"]), "deg": tilt["deg"]}, "sequence": seq}


def specimen_sheets(ceilings):
    S = C.SPECIMENS
    codes = {}
    for c in N.ALL_CODES:
        k = N.CODE_CLASS[c]
        codes[c] = {"class": k, "mass_range_kg": list(S["mass_range_kg"][c]),
                    "ceiling_n": float(ceilings[c]) if k == "brittle" else None,
                    "travel_limit_mm": float(ceilings["SP_travel_mm"]) if k == "soft" else None}
    return {"format": "specimen-sheets/1",
            "classes": {k: {"name": N.CLASS_NAMES[k], "codes": list(N.CODES[k])} for k in N.CLASSES},
            "codes": codes, "width_mm": list(S["width_mm"]), "lateral_mm": S["lateral_mm"],
            "height_mm": S["height_mm"], "pad_line_below_top_mm": S["pad_line_below_top_mm"],
            "soft_k_n_per_mm": list(S["soft_k_n_per_mm"]),
            "finishes": {f: {"mu": list(S["finish_mu"][f])} for f in N.FINISHES},
            "label_rule": dict(C.LABEL_RULE)}


def cell_payload(ceilings, parts):
    """The cell data of setup: parts = {"nests": {...}, "taught": {...}} from the plant's cell; the rest is printed."""
    mo = C.MOTION
    return {"format": "cell/1", "nests": parts["nests"], "nest_geometry": dict(C.NEST_GEOMETRY),
            "taught": parts["taught"],
            "test_lift": dict(mo["test_lift"]),
            "motion": {"translation_law": mo["translation_law"], "translation_peak_mps2": mo["translation_peak_mps2"],
                       "tilt_rate_deg_s": mo["tilt_rate_deg_s"], "tilt_ramp_s": mo["tilt_ramp_s"],
                       "tracking_scale": list(mo["tracking_scale"]), "lower": dict(mo["lower"]),
                       "set_down": dict(mo["set_down"])},
            "profiles": {p: _profile_public(p) for p in N.PROFILES},
            "specimens": specimen_sheets(ceilings), "placement": dict(C.PLACEMENT),
            "timeline": {k: v for k, v in TL.items() if k.endswith("_s")},
            "shifts": TL["shifts_per_unit"], "jobs_per_shift": TL["jobs_per_shift"]}


def up_of(nest_pose):
    """The cell's up direction in the hand frame: the z axis of the pick nest's frame."""
    w, x, y, z = nest_pose["quat_wxyz"]
    return [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]


class SupervisorView(object):
    """Read-only state of the supervisor at one observation (contract/schemas/supervisor_view.json)."""
    __slots__ = ("_d",)
    KEYS = ("unit", "shift", "job", "job_id", "phase", "t_ms", "phase_t0_ms", "ticket", "label", "spec", "ceiling",
            "plan_ms", "peaks_ms", "event")

    def __init__(self, d):
        object.__setattr__(self, "_d", dict(d))

    def __getattr__(self, k):
        try:
            v = object.__getattribute__(self, "_d")[k]
        except KeyError:
            raise AttributeError(k)
        if isinstance(v, dict):
            return {a: (list(b) if isinstance(b, list) else (dict(b) if isinstance(b, dict) else b))
                    for a, b in v.items()}
        if isinstance(v, list):
            return list(v)
        return v

    def __setattr__(self, k, v):
        raise AttributeError("the supervisor view is read-only")

    def __delattr__(self, k):
        raise AttributeError("the supervisor view is read-only")

    def as_dict(self):
        import copy
        return copy.deepcopy(object.__getattribute__(self, "_d"))


def blank_record(unit, shift, job, label, trace):
    return {"unit": int(unit), "shift": int(shift), "job": int(job), "label": label, "phase_reached": "await_clear",
            "placed_at_ms": None, "grasped_at_ms": None, "lifted": False, "declined": False, "carried": False,
            "released": False, "slip_mm": None, "rot_deg": None, "losses": [], "events": [],
            "peak_squeeze_n": None, "peak_travel_mm": None, "first_contact_ms": None, "q": None,
            "trace_sha256": trace}


class Supervisor(object):
    def __init__(self, scenario, unit_index, session, up_h=(0.0, 0.0, 1.0), log=None):
        self.sc = scenario
        self.unit = int(unit_index)
        self.s = session
        self.ceilings = scenario["ceilings"]
        self.shifts = scenario["shifts"]
        self.up = list(up_h)
        self.log = log or (lambda m: None)
        self.phase = "handover"
        self.phase_t0 = 0
        self.shift = 1
        self.ji = None                  # index of the current job in its shift (0-based), None outside a job
        self.job = None                 # the scenario's job dict
        self.rec = None                 # the current job's record (until it is finished)
        self.cur = None                 # the current job's working state
        self.records = []
        self.unit_events = []           # {"kind", "shift", "t_ms", "detail"}
        self.event_obs = None
        self.shutdown = E.Shutdown()
        self.done = False
        self.m = None
        self.voided_shifts = set()
        self.persistent = []            # losses every later job carries too (the controller has ended)

    # ---------------------------------------------------------------------------------------------- helpers
    @property
    def t(self):
        return int(self.s.t_ms)

    def _enter(self, phase, t):
        self.phase = phase
        self.phase_t0 = int(t)
        if self.rec is not None and phase in JOB_PHASES:
            self.rec["phase_reached"] = phase

    def _sp(self):
        return None if self.m is None else self.m.get("specimen")

    def mark_persistent(self, kind):
        """A loss of the current job and of every job after it (the acceptance marks "controller_ended")."""
        if kind not in self.persistent:
            self.persistent.append(kind)
        self.mark_current(kind)

    def mark_current(self, kind):
        if self.rec is not None:
            self._loss(kind)

    def _loss(self, kind):
        if kind not in N.LOSS_KINDS:
            raise AssertionError(kind)
        if kind not in self.rec["losses"]:
            self.rec["losses"].append(kind)

    # ---------------------------------------------------------------------------------------------- the jobs
    def start(self):
        self.s.set_base("rest", self.t, {})
        self._enter("handover", self.t)
        self.m = self.s.measure()

    def _start_job(self, shift, ji, t):
        self.shift = shift
        self.ji = ji
        self.job = self.shifts[shift - 1][ji]
        self.rec = blank_record(self.unit, shift, ji + 1, self.job["label"], self.s.trace_digest())
        self.rec["losses"].extend(self.persistent)
        spec = self.job["spec"]
        self.cur = {"placed": False, "crush": E.Crush(spec["class"], self.ceilings.get(spec["code"]) if
                                                       spec["class"] == "brittle" else None,
                                                       self.ceilings.get("SP_travel_mm") if spec["class"] == "soft"
                                                       else None),
                    "transfer": None, "release": None, "decline_raised": False, "path": None,
                    "plan_ms": {}, "peaks_ms": [], "finished": False}
        self._enter("await_clear", t)

    def _finish_job(self, t):
        """Close the current job's record (its specimen is already removed)."""
        rec, job = self.rec, self.job
        if rec is None:
            return
        tr = self.cur["transfer"]
        if tr is not None:
            rec["slip_mm"] = round(tr.slip_mm, 6)
            rec["rot_deg"] = round(tr.rot_deg, 6)
        if "voided" not in rec["losses"]:
            # a decline job not declined is decline_missed, also when an event ended it (it carries both kinds)
            if job["label"] == "decline" and not rec["declined"]:
                self._loss("decline_missed")
            if job["label"] == "carry" and rec["declined"] and not rec["events"]:
                self._loss("decline_wrong")
        rec["trace_sha256"] = self.s.trace_digest()
        self.records.append(rec)
        self.rec = None
        self.cur = None

    def _void_rest(self, shift, from_ji, t):
        trace = self.s.trace_digest()
        for ji in range(from_ji, len(self.shifts[shift - 1])):
            job = self.shifts[shift - 1][ji]
            r = blank_record(self.unit, shift, ji + 1, job["label"], trace)
            r["phase_reached"] = self.phase
            r["losses"].extend(self.persistent + ["voided"])
            self.records.append(r)

    def _next_job(self, t):
        if self.ji is not None and self.ji + 1 < len(self.shifts[self.shift - 1]):
            self._start_job(self.shift, self.ji + 1, t)
            return
        self.ji = None
        self.job = None
        if self.shift == 1:
            self._enter("pause", t)
        else:
            self._enter("end", t)

    def _remove(self, t):
        if self.cur is not None and self.cur["placed"]:
            self.s.remove()
            self.cur["placed"] = False

    # ---------------------------------------------------------------------------------------------- events
    def _event(self, kind, detail, t):
        if self.phase == "handover":
            shift = 1
        elif self.phase == "pause":
            shift = 2
        else:
            shift = self.shift
        ev = {"kind": kind, "shift": shift, "t_ms": int(t), "detail": detail}
        self.unit_events.append(ev)
        self.event_obs = {"kind": EVENT_FAMILY[kind], "shift": shift, "t_ms": int(t)}
        self.log("  unit %d shift %d: event %s at %d ms %s" % (self.unit, shift, kind, t, detail))
        if self.phase == "end":
            return
        if self.rec is not None:
            self.rec["events"].append({"kind": kind, "t_ms": int(t), "detail": detail})
            self._loss(kind)
            self._remove(t)
            self._finish_job(t)
            rest_from = self.ji + 1
        elif self.phase in ("handover", "pause"):
            rest_from = 0
        else:                                      # between jobs (rest)
            rest_from = self.ji + 1 if self.ji is not None else 0
        self.s.set_base("return", int(t), {})
        if self.phase == "pause" or shift == 2:
            self.voided_shifts.add(2)
            self._enter("end", t)                  # the voided records carry the phase at the voiding tick
            self._void_rest(2, rest_from, t)
        else:
            self._enter("pause", t)
            self._void_rest(1, rest_from, t)
        self.ji = None
        self.job = None

    def abort(self):
        """The plant can run no further (the acceptance has ended the controller for a plant fault its replies caused):
        the job in hand is closed with the losses it carries, every job not yet run is voided (each carries the
        persistent losses too), and the unit is over. The plant is not called (its state is not finite)."""
        if self.done:
            return
        t = self.t
        if self.phase == "handover":
            rest = [(1, 0), (2, 0)]
        elif self.phase == "pause":
            rest = [] if 2 in self.voided_shifts else [(2, 0)]
        elif self.phase == "end" or self.ji is None:
            rest = []
        else:
            shift, nxt = self.shift, self.ji + 1
            if self.rec is not None:
                self.cur["placed"] = False           # the specimen leaves with the unit (no plant call)
                self._finish_job(t)
            rest = [(shift, nxt)] + ([(2, 0)] if shift == 1 and 2 not in self.voided_shifts else [])
        for shift, from_ji in rest:
            self._void_rest(shift, from_ji, t)
        self.ji = None
        self.job = None
        self.done = True

    # ---------------------------------------------------------------------------------------------- per tick
    def after_tick(self):
        """Measurements, events and the timeline at the present tick (call after every plant step)."""
        if self.done:
            return
        t = self.t
        self.m = self.s.measure()
        sp = self._sp()
        ev = self.shutdown.update(self.m["servos"])
        if ev is not None:
            self._event(ev[0], ev[1], t)
            sp = None
        if self.cur is not None and self.cur["placed"] and sp is not None:
            rec = self.rec
            if rec["first_contact_ms"] is None and E.any_pad(sp):
                rec["first_contact_ms"] = t
            f = float(sp["spring_force_n"])
            rec["peak_squeeze_n"] = f if rec["peak_squeeze_n"] is None else max(rec["peak_squeeze_n"], f)
            if self.job["spec"]["class"] == "soft":
                d = 1000.0 * float(sp["spring_travel_m"])
                rec["peak_travel_mm"] = d if rec["peak_travel_mm"] is None else max(rec["peak_travel_mm"], d)
            ev = self.cur["crush"].update(t, sp, rec["first_contact_ms"])
            if ev is not None:
                self._event(ev[0], ev[1], t)
                sp = None
        for _ in range(8):                          # phase changes due at this tick (each re-checks the new phase)
            if not self._phase_step(t):
                break

    def _phase_step(self, t):
        ph, dt = self.phase, t - self.phase_t0
        sp = self._sp()
        if ph == "handover":
            if t >= MS["handover"]:
                self._start_job(1, 0, t)
                return True
            return False
        if ph == "await_clear":
            spec = self.job["spec"]
            if self.s.clear_for(spec, "pick"):
                self.s.place(spec, "pick")
                self.cur["placed"] = True
                self.rec["placed_at_ms"] = t
                self.m = self.s.measure()
                self._enter("present", t)
                return True
            if dt >= MS["await_clear"]:
                self._loss("not_presented")
                self._finish_job(t)
                self._next_job(t)
                return True
            return False
        if ph == "present":
            if t - self.rec["placed_at_ms"] >= MS["present"]:
                self._loss("not_grasped")
                self._release_request(t, "lost")
                return True
            return False
        if ph == "test_lift":
            if dt >= MS["test_lift"]:
                if E.lifted(sp):
                    self.rec["lifted"] = True
                    self.s.set_base("transfer", t, {"profile": self.job["ticket"]["profile"],
                                                    "track_scale": float(self.job["track_scale"])})
                    self.cur["transfer"] = E.Transfer(t, sp)
                    self.cur["transfer_end"] = t + transfer_ms(self.job["ticket"]["profile"])
                    self._enter("transfer", t)
                else:
                    self._loss("not_lifted")
                    self.cur["path"] = "lost"
                    self.s.set_base("lower", t, {"track_scale": float(self.job["track_scale"])})
                    self._enter("lower", t)
                return True
            return False
        if ph == "lower":
            if dt >= MS["lower"]:
                self._release_request(t, self.cur["path"] or "lost")
                return True
            return False
        if ph == "transfer":
            ev = self.cur["transfer"].update(t, sp)
            if ev is not None:
                self._event(ev[0], ev[1], t)
                return True
            if t >= self.cur["transfer_end"]:
                self.s.set_base("set_down", t, {"track_scale": float(self.job["track_scale"])})
                self._enter("set_down", t)
                return True
            return False
        if ph == "set_down":
            if dt >= MS["set_down"]:
                self.rec["carried"] = E.seated_on_place(sp)
                if not self.rec["carried"]:
                    self._loss("not_seated")
                self._release_request(t, "carried")
                return True
            return False
        if ph == "release":
            rel = self.cur["release"]
            if dt > 0:
                rel.update(t, sp)
            if dt >= MS["release"]:
                path = self.cur["path"]
                if path == "carried":
                    self.rec["released"] = bool(rel.free and not rel.pushed)
                    if not rel.free:
                        self._loss("release_late")
                    if rel.pushed:
                        self._loss("release_push")
                if path == "declined":
                    self.rec["declined"] = E.rests_declined(sp)
                self._remove(t)
                self.s.set_base("return", t, {})
                self._finish_job(t)
                self._enter("rest", t)
                return True
            return False
        if ph == "rest":
            if dt >= MS["rest"]:
                self._next_job(t)
                return True
            return False
        if ph == "pause":
            if dt >= MS["pause"]:
                if 2 in self.voided_shifts:
                    self._enter("end", t)
                else:
                    self._start_job(2, 0, t)
                return True
            return False
        if ph == "end":
            if dt >= MS["end"]:
                self.done = True
            return False
        raise AssertionError(ph)

    def _release_request(self, t, path):
        self.cur["path"] = path
        self.cur["release"] = E.Release(t, self._sp(), self.up)
        self._enter("release", t)

    # ---------------------------------------------------------------------------------------------- the flags
    def on_reply(self, grasped, decline):
        """The flags of a step reply, at the tick the reply is written. Decline wins over grasped; a flag in any
        other phase than present (and decline in test_lift) is ignored."""
        if self.done or self.cur is None:
            return
        t = self.t
        if self.phase == "present":
            if decline:
                self.cur["decline_raised"] = True
                self._release_request(t, "declined")
            elif grasped:
                self.rec["grasped_at_ms"] = t
                self._plan(t)
                self.s.set_base("test_lift", t, {"track_scale": float(self.job["track_scale"])})
                self._enter("test_lift", t)
        elif self.phase == "test_lift" and decline:
            self.cur["decline_raised"] = True
            self.cur["path"] = "declined"
            self.s.set_base("lower", t, {"track_scale": float(self.job["track_scale"])})
            self._enter("lower", t)

    def _plan(self, tg):
        prof = self.job["ticket"]["profile"]
        t1 = tg + MS["test_lift"]
        t2 = t1 + transfer_ms(prof)
        t3 = t2 + MS["set_down"]
        t4 = t3 + MS["release"]
        self.cur["plan_ms"] = {"test_lift": [tg, t1], "transfer": [t1, t2], "set_down": [t2, t3],
                               "release": [t3, t4], "rest": [t4, t4 + MS["rest"]]}
        self.cur["peaks_ms"] = transfer_peaks_ms(t1, prof)

    # ---------------------------------------------------------------------------------------------- views
    def _shown(self):
        """The job whose ticket is shown (from its await_clear to its rest), or None."""
        if self.job is not None and (self.cur is not None or self.phase == "rest"):
            return self.job
        return None

    def cell_obs(self):
        job = self._shown()
        shift = self.shift if self.phase not in ("handover",) else 1
        return {"shift": int(shift), "job": int(self.ji + 1) if job is not None else None, "phase": self.phase,
                "t_phase_ms": int(self.t - self.phase_t0), "ticket": dict(job["ticket"]) if job is not None else None}

    def obs_event(self):
        return dict(self.event_obs) if self.event_obs else None

    def view(self):
        job = self._shown()
        cur = self.cur or {}
        return SupervisorView({
            "unit": self.unit, "shift": int(self.shift), "job": int(self.ji + 1) if job is not None else None,
            "job_id": job["id"] if job is not None else None, "phase": self.phase, "t_ms": self.t,
            "phase_t0_ms": int(self.phase_t0), "ticket": dict(job["ticket"]) if job is not None else None,
            "label": job["label"] if job is not None else None, "spec": dict(job["spec"]) if job is not None else None,
            "ceiling": job_ceiling(job["spec"], self.ceilings) if job is not None else None,
            "plan_ms": {k: list(v) for k, v in cur.get("plan_ms", {}).items()},
            "peaks_ms": list(cur.get("peaks_ms", [])), "event": self.obs_event()})
