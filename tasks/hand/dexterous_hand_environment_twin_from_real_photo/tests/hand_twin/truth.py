"""The hidden scene files (tests/gt/) and the truth's physics model.

    xml, bodies = load(gt_dir, variant=None)    # truth_scene.xml through MjSpec, with a physics variant applied
    files = read_gt(gt_dir)                     # the JSON files and the reference pieces, parsed
    held = held_keys(things_json)               # the things the contract holds still (welded in the truth)

Frames and units: the truth's world is the true support frame (z up, the plane z = 0), metres in the MJCF; the
instrument works in millimetres in the same frame.

load(): truth_scene.xml is read into an MjSpec, never edited on disk. A variant [mass_scale, friction] scales every
thing body's explicit inertial (mass and inertia) by mass_scale and sets the sliding friction of every geom (the plane
included) to `friction`, so that every contact pair has that friction. The mesh directory is made absolute, so the
returned XML string compiles from anywhere (MjModel.from_xml_string, MjSpec.from_string). MjSpec's XML writer prints
numbers with six significant digits (positions move by under a micrometre); every user of the truth's physics (the
build's classes and the grader) takes the model through this one function, so they use the same model.
"""
import hashlib
import json
import os

import numpy as np

GT_FILES = ("plane.json", "things.json", "requests.json", "selftest.json", "ref_points.json", "truth_manifest.json",
            "truth_scene.xml", "scans_surface.npz", "scans_occupancy.npz", "reference_pieces.npz")
FORMATS = {"plane.json": "hand-twin-plane/1", "things.json": "hand-twin-things/1",
           "requests.json": "hand-twin-requests/1", "selftest.json": "hand-twin-selftest/1",
           "ref_points.json": "hand-twin-ref-points/1", "truth_manifest.json": "hand-twin-truth-manifest/1"}


class TruthError(Exception):
    """The truth files are missing, malformed or do not agree with each other or with the configuration: a task
    error, never a score."""


def load_json(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_gt(gt_dir):
    """{"plane", "things", "requests", "selftest", "ref_points", "manifest", "reference_pieces"} parsed; TruthError
    when a file is missing or carries another format."""
    out = {}
    for name in GT_FILES:
        if not os.path.isfile(os.path.join(gt_dir, name)):
            raise TruthError("the truth file %s is missing" % name)
    for name, fmt in FORMATS.items():
        try:
            doc = load_json(os.path.join(gt_dir, name))
        except (OSError, ValueError) as exc:
            raise TruthError("the truth file %s does not parse: %s" % (name, exc))
        if not isinstance(doc, dict) or doc.get("format") != fmt:
            raise TruthError("the truth file %s is not %s" % (name, fmt))
        out[name[:-5]] = doc
    out["manifest"] = out.pop("truth_manifest")
    out["reference_pieces"] = reference_pieces(os.path.join(gt_dir, "reference_pieces.npz"))
    return out


def reference_pieces(path):
    """{key: [V (n, 3) float64 mm]} from reference_pieces.npz (members "<key>__<nn>", nn from 00 without gaps)."""
    with np.load(path, allow_pickle=False) as z:
        names = sorted(z.files)
        by_key = {}
        for nm in names:
            key, sep, nn = nm.rpartition("__")
            if not sep or not nn.isdigit():
                raise TruthError("reference_pieces.npz has a member %r" % nm)
            V = np.asarray(z[nm], dtype=np.float64)
            if V.ndim != 2 or V.shape[1] != 3 or len(V) < 4 or not np.all(np.isfinite(V)):
                raise TruthError("reference_pieces.npz member %s is not a finite (n >= 4, 3) array" % nm)
            by_key.setdefault(key, []).append((int(nn), V))
    out = {}
    for key, rows in by_key.items():
        rows.sort(key=lambda r: r[0])
        if [r[0] for r in rows] != list(range(len(rows))):
            raise TruthError("reference_pieces.npz: the pieces of %s are not numbered 00.. without gaps" % key)
        out[key] = [r[1] for r in rows]
    return out


def held_keys(things_json):
    return tuple(k for k, t in things_json["things"].items() if t.get("held"))


def thing_keys(things_json):
    return list(things_json["things"])


def load(gt_dir, variant=None):
    """(the truth's MJCF as an XML string, {thing key: body name}) with `variant` = [mass_scale, friction] applied
    (None: the file's own masses and frictions)."""
    import mujoco
    gt_dir = os.path.abspath(gt_dir)
    path = os.path.join(gt_dir, "truth_scene.xml")
    spec = mujoco.MjSpec.from_file(path)
    things = load_json(os.path.join(gt_dir, "things.json"))["things"]
    meshdir = spec.meshdir or ""
    spec.meshdir = os.path.join(gt_dir, meshdir) if not os.path.isabs(meshdir) else meshdir
    if not spec.meshdir.endswith(os.sep):
        spec.meshdir += os.sep
    for m in spec.meshes:
        # MuJoCo's writer adds the decoder's content type after the first compile in a process only: stated here,
        # the XML string is the same on every call
        if (m.file or "").lower().endswith(".stl") and not m.content_type:
            m.content_type = "model/stl"
    names = {b.name for b in spec.bodies}
    bodies = {k: k for k in things if k in names}
    missing = [k for k in things if k not in names]
    if missing:
        raise TruthError("truth_scene.xml has no body for %s" % ", ".join(missing))
    if variant is not None:
        mass_scale, friction = float(variant[0]), float(variant[1])
        if not (mass_scale > 0.0 and friction > 0.0):
            raise TruthError("a physics variant must be positive: %r" % (variant,))
        for b in spec.bodies:
            if b.name not in bodies:
                continue
            if not b.explicitinertial:
                raise TruthError("the truth body %s has no explicit inertial" % b.name)
            b.mass = float(b.mass) * mass_scale
            b.inertia = np.asarray(b.inertia, dtype=np.float64) * mass_scale
            full = np.asarray(b.fullinertia, dtype=np.float64)
            if np.all(np.isfinite(full)):
                b.fullinertia = full * mass_scale
        for g in spec.geoms:
            f = np.asarray(g.friction, dtype=np.float64).copy()
            f[0] = friction
            g.friction = f
    return spec.to_xml(), bodies


def compile_xml(xml):
    import mujoco
    return mujoco.MjModel.from_xml_string(xml)
