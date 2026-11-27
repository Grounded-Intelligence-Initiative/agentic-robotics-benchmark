#!/usr/bin/env python3
"""Format checker of the unit's description files: the grader's own gate, with no knowledge of the unit.

    python3 check_submission.py [SUBMISSION_DIR] [--meshes MESH_DIR] [--json FILE]

SUBMISSION_DIR  the folder holding unit.xml and calibration.yaml (default: $HOME/submission)
--meshes        the folder of the given model's 26 mesh files (default: ../repo/sim-model/robot_25dof/meshes from
                this file's folder, the given model's own mesh folder)
--json FILE     also write the verdict, every failed rule and what was read as JSON

It runs gate.py from this file's folder: the module the grader runs, byte for byte (its sha256 is printed). It also
measures the submission folder as the grader does before it copies the folder (rule files.caps of acceptance.txt: at
most 20000 entries below the folder, and at most 209715200 bytes in its regular files); a folder over either cap fails
under that rule's name. It prints PASS or FAIL, every rule that failed with where and why, what the grader reads from
the files, and notes that are no failures. Exit 0 on PASS, 1 on FAIL, 2 when the checker cannot run. It cannot tell
whether a number is right: it knows nothing about the unit. One rule of the gate is the grader's alone: nothing in
the submission folder may be written after the session ends.

Nothing in the files is executed. The checker imports only the standard library, NumPy, PyYAML and MuJoCo. It reads
unit.xml and calibration.yaml only when each is a regular file, as the gate does, and a mesh file that unit.xml names
only when it is one.

Layout of calibration.yaml (acceptance.txt beside this file has every rule; the values here are the lab's template
row of firmware slot 0 as the lab's notes give it and the given model's names, an example of the form only):

    schema: unit-calibration/1
    home_table:
    - {slot: 0, actuator: L_hip_pitch, sign: -1, home_rad: -1.57}
    - ... 25 rows, slot 0 to 24 in firmware order ...
    ankle_map:
      left:
        axes: [A, B]
        extent: [[-0.787, 0.787], [-0.787, 0.787]]
        nodes: 19
        pitch: [[...19 numbers...], ... 19 rows ...]
        roll: [[...19 numbers...], ... 19 rows ...]
      right: {...the same keys...}
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import stat as statmod
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
GATE_PATH = os.path.join(HERE, "gate.py")
DEFAULT_MESHES = os.path.normpath(os.path.join(HERE, "..", "repo", "sim-model", "robot_25dof", "meshes"))
#: the grader's caps of the submission folder (acceptance.txt, rule files.caps): the same numbers and the same measure
SUBMISSION_BYTES_CAP = 200 * 1024 * 1024
SUBMISSION_ENTRIES_CAP = 20000
FOLDER_RULE_CODE = "files.caps"
FOLDER_RULE = ("The submission folder holds at most 20000 entries (files, folders, links and other entries, at any "
               "depth below it) and its regular files at most 209715200 bytes (200 MiB) in all, by the larger of their "
               "total size and the disk space they take: the grader's own measure, taken before it copies the folder "
               "and following no link. A folder beyond either cap is not copied and is graded as an empty folder, so "
               "that files.missing fails.")


def load_gate():
    """gate.py from this file's folder (never another module of the same name)"""
    spec = importlib.util.spec_from_file_location("unit_description_gate", GATE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _submission_measure(root, max_entries=SUBMISSION_ENTRIES_CAP):
    """(entries, apparent bytes, allocated bytes) of the tree below `root` by lstat, never following a link."""
    n, apparent, allocated = 0, 0, 0
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in dirnames + filenames:
            n += 1
            if n > max_entries:
                return n, apparent, allocated
            try:
                st = os.lstat(os.path.join(dirpath, name))
            except OSError:
                continue
            if statmod.S_ISREG(st.st_mode):
                apparent += int(st.st_size)
                allocated += int(getattr(st, "st_blocks", 0)) * 512
    return n, apparent, allocated


def folder_failures(sub_dir: str) -> list:
    """[(code, where, message)]: the folder measured as the grader measures it before it copies the folder (a folder
    that is missing, not a folder or a symbolic link is the gate's rule files.folder, not measured here)"""
    try:
        st = os.lstat(sub_dir)
    except OSError:
        return []
    if not statmod.S_ISDIR(st.st_mode):
        return []
    n, apparent, allocated = _submission_measure(sub_dir)
    if n <= SUBMISSION_ENTRIES_CAP and max(apparent, allocated) <= SUBMISSION_BYTES_CAP:
        return []
    count = f"more than {SUBMISSION_ENTRIES_CAP} entries" if n > SUBMISSION_ENTRIES_CAP else f"{n} entries"
    return [(FOLDER_RULE_CODE, sub_dir, f"{count}, {max(apparent, allocated)} bytes in its regular files (by size "
             f"{apparent}, on disk {allocated}): more than the caps ({SUBMISSION_ENTRIES_CAP} entries, "
             f"{SUBMISSION_BYTES_CAP} bytes); the grader does not copy such a folder and grades it as an empty folder "
             "(files.missing)")]


def mesh_notes(gate, sub_dir: str, mesh_dir: str) -> list:
    """where unit.xml's mesh paths point, against the given copies (notes only: the grader compiles the given copy
    of each named file, wherever the path points); unit.xml is read only when it is a regular file, as the gate reads
    it, and a mesh path only when it names a regular file"""
    notes = []
    p = os.path.join(sub_dir, gate.UNIT_XML)
    try:
        if not statmod.S_ISREG(os.lstat(p).st_mode):
            return notes
        with open(p, "rb") as f:
            raw = f.read(gate.MAX_XML_BYTES + 1)
        text = raw.decode("utf-8")
        if "<!DOCTYPE" in text or "<!ENTITY" in text:
            return notes
        root = ET.fromstring(text)
    except (OSError, UnicodeDecodeError, ET.ParseError):
        return notes
    comp = {}
    for c in root.iter("compiler"):
        comp.update({k: c.attrib[k] for k in ("meshdir", "assetdir") if k in c.attrib})
    base = comp.get("meshdir", comp.get("assetdir", ""))
    unresolved, differ = [], []
    for asset in root.findall("asset"):
        for me in asset.findall("mesh"):
            f = me.attrib.get("file")
            if not f:
                continue
            name = f.replace("\\", "/").rstrip("/").split("/")[-1]
            if name not in gate.MESH_FILES:
                continue
            path = f if os.path.isabs(f) else os.path.join(sub_dir, base, f)
            given = os.path.join(mesh_dir, name)
            if not os.path.isfile(path):
                unresolved.append(name)
            elif os.path.isfile(given) and sha256_of(path) != sha256_of(given):
                differ.append(name)
    if unresolved:
        more = ", ..." if len(unresolved) > 3 else ""
        notes.append(f"mesh paths of {gate.UNIT_XML} that resolve to no file from its folder: {len(unresolved)} "
                     f"({', '.join(unresolved[:3])}{more}); no matter, the grader compiles the given copy of each "
                     "named file")
    if differ:
        notes.append(f"mesh files that differ from the given copies: {len(differ)} ({', '.join(differ)}); the grader "
                     "compiles the given copies, not these")
    return notes


def summary_lines(gate, res) -> list:
    out = []
    info = res.info
    if res.model is not None:
        out.append(f"  unit.xml: {info.get('bodies')} bodies besides the world ({info.get('added_bodies', 0)} added to "
                   f"the given 26), {info.get('joints')} joints, {info.get('geoms')} geoms, {info.get('sites')} sites; "
                   f"total mass {info.get('total_mass_kg', float('nan')):.3f} kg")
        if "imu" in info:
            out.append(f"  site imu: on {info['imu']['body']} (moves with {info['imu']['link']})")
        for s, d in sorted(info.get("soles", {}).items()):
            hx, hy, hz = d["half_size_m"]
            out.append(f"  site {s}: on {d['body']} (moves with {d['link']}); box {2 * hx * 1000:.2f} x "
                       f"{2 * hy * 1000:.2f} mm footprint, its -z face {hz * 1000:.2f} mm below the site")
    if res.calibration is not None:
        sg = res.calibration["sign"]
        out.append(f"  calibration.yaml: 25 home rows ({int((sg > 0).sum())} signs +1, {int((sg < 0).sum())} signs "
                   "-1); ankle maps left and right, 19 x 19 nodes each")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir", nargs="?", default=os.path.join(os.environ.get("HOME", "/home/user"), "submission"))
    ap.add_argument("--meshes", default=DEFAULT_MESHES)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    sub_dir = os.path.abspath(a.dir)
    try:
        gate = load_gate()
    except Exception as exc:  # noqa: BLE001
        print(f"check_submission: gate.py cannot be loaded from {HERE}: {exc}", file=sys.stderr)
        return 2
    gate_sha = sha256_of(GATE_PATH)
    print(f"check_submission: {sub_dir}")
    print(f"  gate {gate.GATE_VERSION}, gate.py sha256 {gate_sha}")
    try:
        res = gate.check(sub_dir, a.meshes)
    except gate.GateSetupError as exc:
        print(f"check_submission: cannot run: {exc}", file=sys.stderr)
        return 2
    mj = res.info.get("mujoco_version")
    notes = []
    if mj and mj != gate.MUJOCO_VERSION:
        notes.append(f"this MuJoCo is {mj}; the contract compiles with {gate.MUJOCO_VERSION}")
    notes += mesh_notes(gate, sub_dir, a.meshes)
    failures = [tuple(f) for f in res.failures] + folder_failures(sub_dir)
    codes = sorted({f[0] for f in failures})
    ok = not failures
    print(f"gate: {'PASS' if ok else 'FAIL'}" + ("" if ok else f" ({len(failures)} failures, rules: "
                                                 + ", ".join(codes) + ")"))
    for code, where, message in failures:
        print(f"  - [{code}] {where + ': ' if where else ''}{message}")
    lines = summary_lines(gate, res)
    if lines:
        print("read by the grader (as compiled and loaded; the checker cannot say whether it is right):")
        for line in lines:
            print(line)
    if notes:
        print("notes (no failures):")
        for n in notes:
            print(f"  - {n}")
    print("not checked here: whether the numbers describe the unit; the grader's rule that nothing in the folder is "
          "written after the session ends")
    if a.json:
        rules = dict(gate.RULES, **{FOLDER_RULE_CODE: FOLDER_RULE})
        out = {"gate": "PASS" if ok else "FAIL", "gate_version": gate.GATE_VERSION, "gate_sha256": gate_sha,
               "failures": [{"code": c, "where": w, "message": m} for c, w, m in failures],
               "rules": {c: rules[c] for c in codes}, "info": res.info, "notes": notes}
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=1, sort_keys=True, default=str)
            fh.write("\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
