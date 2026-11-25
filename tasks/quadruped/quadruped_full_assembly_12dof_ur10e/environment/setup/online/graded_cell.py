"""The graded episode's cell, for graded_server.py: the full-assembly cell built exactly as the offline grader's
`run_episode` (tests/verify.py) builds an episode's cell, and the per-episode score exactly as that verify.py computes
it. The cell's code is the offline grader's own, byte-identical (online/cell/ is a copy of the grader's tests/arm,
tests/render, tests/render_filament, tests/common, tests/room and tests/episode_trace.py), installed by setup.sh where
only the lab account can read it.

Two cells (graded_config.json "cell_kind"); this file is the same in both tasks:
  quad  quadruped_full_assembly_12dof_ur10e: QuadInfoServer(kit, robot models, bench plan, config=grader_config.json,
        mode="hidden", privileged=False, render=True, log, kit_info=kit_info.json), reset(seed). The episode's value is
        main's: stage_score x (1 if safety_ok else 0).
  car   racecar_full_assembly_tenth_scale_ur10e: CarInfoServer(kit, robot models, bench plan, config=grader_config.json,
        mode="hidden", privileged=False, render=True, log, kit=load_kit(kit), uv=unit_geometry(kit)),
        reset(seed, stratum); the layout it built must have the hidden set's layout digest (as main's check_layout_digest:
        a mismatch is an error, and the service does not start). The episode's value is main's: stage_score x (1 if
        hard_ok else grader_config.json scoring.safety_violation_factor).

The cell is built with privileged=False: the graded episode's own layout. The oracle's start (its token) only turns on
the cell's privileged calls (reference_assemble, get_state_privileged, describe_privileged) on that layout; for the
race car that is exactly main's privileged episode (oracle_nominal_layout false).

The record of an episode that is still running carries the value the episode would score if it ended now: the cell's
end (`finalize`) simulates nothing and scores the state as it stands, so `live()` computes the same numbers from the
same state without closing the cell (the same formula as finalize: the counted slots' weight over the scored weight,
and the same safety flags). `final()` is the cell's own finalize at the end.
"""

import hashlib
import json
import math
import os
import sys

KINDS = ("quad", "car")
# the packages of the offline grader's tests/ folder that the copy holds (cell_code_digest covers exactly these)
CODE_PACKAGES = ("arm", "render", "render_filament", "common", "room")
CODE_FILES = ("episode_trace.py",)


def _finite(x, default=0.0):
    try:
        f = float(x)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def tree_files(root, packages=CODE_PACKAGES, files=CODE_FILES):
    """[(relative path, size, sha256)] of the cell code under `root`: every file of `packages` (no __pycache__, no .pyc)
    and the single `files`."""
    out = []
    for pkg in packages:
        base = os.path.join(root, pkg)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = sorted(d for d in dirnames if d != "__pycache__")
            for name in sorted(filenames):
                if name.endswith(".pyc"):
                    continue
                p = os.path.join(dirpath, name)
                with open(p, "rb") as fh:
                    blob = fh.read()
                out.append((os.path.relpath(p, root), len(blob), hashlib.sha256(blob).hexdigest()))
    for name in files:
        p = os.path.join(root, name)
        with open(p, "rb") as fh:
            blob = fh.read()
        out.append((name, len(blob), hashlib.sha256(blob).hexdigest()))
    return out


def code_digest(root):
    """sha256 over the sorted (path, size, sha256) lines of the cell code: tests/verify.py computes the same over the
    grader's own tests/ folder and requires them equal (the graded episode ran the offline grader's cell)."""
    h = hashlib.sha256()
    for rel, size, digest in sorted(tree_files(root)):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" + digest.encode() + b"\n")
    return h.hexdigest()


def file_digest(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


class GradedCell(object):
    """One graded episode's cell: build(), then the server `srv` answers the public calls (srv.handle); live() /
    final() give the record's numbers."""

    def __init__(self, kind, code_dir, grader_cfg, log):
        if kind not in KINDS:
            raise ValueError("cell_kind must be one of %s" % (KINDS,))
        self.kind = kind
        self.code_dir = code_dir
        self.cfg = grader_cfg
        self.log = log
        self.srv = None
        self.trace = None
        self.stratum = None
        self.layout_digest = None
        if code_dir not in sys.path:
            sys.path.insert(0, code_dir)
        import numpy  # noqa: F401
        import mujoco
        self.mujoco_version = mujoco.__version__
        if kind == "quad":
            from arm.assembly_quad.server import QuadInfoServer
            self._Server = QuadInfoServer
        else:
            from arm.assembly_car import layout as LY
            from arm.assembly_car.cell import load_kit
            from arm.assembly_car.server import CarInfoServer
            self._Server, self._LY, self._load_kit = CarInfoServer, LY, load_kit
        import episode_trace
        self._trace_mod = episode_trace

    # ------------------------------------------------------------------ the episode's cell
    def build(self, kit_dir, robot_dir, plan, episode, kit_info=None, trace_every_ticks=0):
        """Build and reset the graded episode's cell (as main's run_episode); returns a line for the service log."""
        seed = int(episode["seed"])
        if self.kind == "quad":
            srv = self._Server(kit_dir, robot_dir, plan, config=self.cfg, mode="hidden", privileged=False, render=True,
                               log=self.log, kit_info=kit_info)
            srv.reset(seed)
            note = "quadruped cell built (%d layout draw(s))" % len(srv.layout.get("draws") or [1])
        else:
            kit = self._load_kit(kit_dir)
            uv = self._LY.unit_geometry(kit)
            srv = self._Server(kit_dir, robot_dir, plan, config=self.cfg, mode="hidden", privileged=False, render=True,
                               log=self.log, kit=kit, uv=uv)
            r = srv.reset(seed, stratum=episode.get("stratum"))
            want = episode.get("layout_digest")
            if not want or r.get("layout_digest") != want:
                raise RuntimeError("the cell built a layout that is not the hidden set's (its layout digest differs): the "
                                   "kit, the bench plan, the layout code or grader_config.json changed after the set was "
                                   "selected")
            self.stratum, self.layout_digest = r.get("stratum"), r.get("layout_digest")
            note = "race-car cell built (stratum %s, layout digest checked)" % self.stratum
        self.srv = srv
        if trace_every_ticks:
            self._attach_trace(int(trace_every_ticks))
        return note

    def _attach_trace(self, every_ticks):
        """The offline grader's replay record (episode_trace.EpisodeTrace: the joint state every `every_ticks` ticks and
        the event log), without its copy of every camera frame (up to 600 PNGs held in memory until the end): the
        cell's own get_camera is put back."""
        try:
            self.trace = self._trace_mod.EpisodeTrace(self.srv, every_ticks=every_ticks, log=self.log)
            self.srv.__dict__.pop("get_camera", None)
        except Exception as e:  # noqa: BLE001 - a record only
            self.trace = None
            self.log("[graded] episode trace not attached: %r" % (e,))

    def save_trace(self, root, tag, summary, extra):
        """Write the trace under root/tag (trace.npz, meta.json; no camera frames); returns the folder or None."""
        if self.trace is None:
            return None
        try:
            self.trace.detach()
            return self.trace.save(root, tag, summary, extra=extra)
        except Exception as e:  # noqa: BLE001 - a record only
            self.log("[graded] episode trace not written: %r" % (e,))
            return None

    # ------------------------------------------------------------------ the score
    def factor(self):
        """The share of its stage score that an episode with a safety violation keeps (main's rule)."""
        if self.kind == "quad":
            return 0.0
        return float((self.cfg.get("scoring") or {}).get("safety_violation_factor"))

    def live(self):
        """The record's numbers for the episode as it stands now (the cell's finalize formula, nothing ended)."""
        srv = self.srv
        rec = srv.sm.record()
        weights = [float(r["weight"]) for r in rec]
        total = sum(weights)
        counted = sum(float(r["weight"]) for r in rec if r["achieved"])
        stage = counted / total if total else 0.0
        if self.kind == "quad":
            ok = len(srv.violations) == 0
            n_viol = len(srv.violations)
        else:
            ws = srv.cell.watch_summary()
            hard = [v for v in ws["violations"] if v["what"].split(":")[0] in ("robot_contact", "part_force")]
            ok = not hard
            n_viol = len(hard)
        value = stage * (1.0 if ok else self.factor())
        metrics = {"stage_score_mean_ungated": stage, "mates_achieved_mean": float(sum(1 for r in rec if r["achieved"])),
                   "mates_fastened_mean": float(sum(1 for r in rec if r.get("fastened"))),
                   ("safety_violation_rate" if self.kind == "quad" else "hard_violation_rate"): 0.0 if ok else 1.0,
                   "safety_violations": float(n_viol), "mean_sim_time_s": float(srv.data.time),
                   "camera_frames_mean": float(srv.frames_used), "illegal_call_count": float(srv.illegal_calls),
                   "step_calls": float(srv.step_calls + srv.calls.get("reference_assemble", 0))}
        return {"episode_score": value, "stage_score": stage, "safety_ok": ok, "metrics": metrics}

    def final(self, wall_s):
        """The cell's own end of the episode (finalize: closes the cameras, simulates nothing) and main's scoring of it:
        (summary, episode_score, metrics)."""
        srv = self.srv
        s = srv.finalize(None)
        s["episode_wall_s"] = float(wall_s)
        if self.kind == "quad":
            ok = bool(s["safety_ok"])
        else:
            ok = bool(s["hard_ok"])
            s["layout_digest_checked"] = True
        value = float(s["stage_score"]) * (1.0 if ok else self.factor())
        s["episode_value"] = value
        return s, value, self.metrics(s)

    def metrics(self, s):
        """main's per-episode statistics (tests/verify.py, the same names, for one episode)."""
        out = {"stage_score_mean_ungated": _finite(s.get("stage_score")),
               "mates_achieved_mean": _finite(s.get("n_achieved")), "mates_fastened_mean": _finite(s.get("n_fastened")),
               "all_mates_rate": 1.0 if s.get("all_mates") else 0.0, "mean_sim_time_s": _finite(s.get("sim_time_s")),
               "mean_episode_wall_s": _finite(s.get("episode_wall_s")),
               "camera_frames_mean": _finite(s.get("camera_frames_total")),
               "render_wall_s_mean": _finite(s.get("render_wall_s")),
               "illegal_call_count": _finite(s.get("illegal_call_count")),
               "step_calls": _finite(s.get("step_calls")) + _finite((s.get("calls") or {}).get("reference_assemble", 0)),
               "safety_violations": float(len(s.get("violations") or []))}
        for cam in ("top", "wrist"):
            out["camera_frames_mean." + cam] = _finite((s.get("camera_frames") or {}).get(cam, 0))
        for r in s.get("mates") or []:
            out["mate_rate." + str(r.get("id"))] = 1.0 if r.get("achieved") else 0.0
        if self.kind == "quad":
            dmg = s.get("damage") or {}
            out.update({"safety_violation_rate": 0.0 if s.get("safety_ok") else 1.0,
                        "dropped_parts_mean": float(len(s.get("dropped_parts") or [])),
                        "lost_parts_mean": float(len(s.get("lost_parts") or [])),
                        "no_damage_rate": 1.0 if dmg.get("no_damage") else 0.0,
                        "max_pad_force_N": _finite(dmg.get("max_pad_force_N")),
                        "max_part_force_N": _finite(dmg.get("max_part_force_N")),
                        "max_part_force_sustained_N": _finite(dmg.get("max_part_force_sustained_N")),
                        "shoulder_fastened_off_design_rate": float(len(s.get("off_design") or []))})
        else:
            cm = s.get("coupling_metrics") or {}
            out.update({"hard_violation_rate": 0.0 if s.get("hard_ok") else 1.0,
                        "locked_off_mean": float(len(s.get("locked_off") or [])),
                        "scrapped_units_mean": float(len(s.get("scrapped") or [])),
                        "void_carrier_scrapped_mean": _finite(cm.get("void_carrier_scrapped")),
                        "void_unit_scrapped_mean": _finite(cm.get("void_unit_scrapped")),
                        "robot_contact_episodes": 1.0 if s.get("robot_contact_violations") else 0.0,
                        "part_force_episodes": 1.0 if s.get("part_force_violations") else 0.0,
                        "max_part_force_sustained_N": _finite((s.get("watch") or {}).get("max_part_force_sustained_N")),
                        "layout_digest_checked": 1.0 if s.get("layout_digest_checked") else 0.0,
                        "safety_violation_factor": self.factor()})
        return out


def load_json(path, limit=1 << 26):
    with open(path, "rb") as fh:
        blob = fh.read(limit + 1)
    if len(blob) > limit:
        raise ValueError("%s is larger than %d bytes" % (path, limit))
    return json.loads(blob)
