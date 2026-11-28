"""The fixed hidden acceptance set: scenario files, their checks, and the digest of the code they belong to.

A scenario (format "grip-scenario/1", JSON text in an .scenario file) holds one hand unit with its two shifts. The
acceptance runs the frozen scenarios instead of drawing new ones, so its score is reproducible. A scenario is valid
only together with the files it was checked against:

    load(path, *, manifest=None, dev=False, ceilings_path=None) -> dict
        raises ScenarioStale when (a) the file's sha256 differs from its entry in the manifest, (b) its code digest
        differs from code_digest() of the running packages, or (c) its ceilings differ from the world's ceilings
        file. dev=True (development draws only) skips (a) and (b) and refuses any file of the frozen set.
    code_digest() -> sha256 of the canonical JSON of the sorted [relpath, sha256] pairs of every file the
        shipped-file list (SHIPPED.json beside the package folders) names, each found from its package on sys.path,
        so the build folder and the published task give the same value.
"""
import hashlib
import importlib.util
import json
import os

FORMAT = "grip-scenario/1"
CEILINGS_FORMAT = "grip-ceilings/1"
CEILING_KEYS = ("SH-1", "SH-2", "SH-3", "SP_travel_mm")
HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGES = ("contract", "plant", "world", "grader")


class ScenarioStale(RuntimeError):
    """A scenario that does not belong to the running code, the frozen manifest or the ceilings."""


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def dumps(obj):
    """Canonical JSON text: sorted keys, one-space indent, ASCII, no NaN or infinity, a final newline."""
    return json.dumps(obj, sort_keys=True, indent=1, ensure_ascii=True, allow_nan=False) + "\n"


def save(path, obj):
    """Write obj as canonical JSON text (atomically); returns its sha256."""
    text = dumps(obj).encode("ascii")
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(text)
    os.replace(tmp, path)
    return sha256_bytes(text)


# ----------------------------------------------------------------------------------------------- the code digest
def _package_dir(pkg):
    spec = importlib.util.find_spec(pkg)
    if spec is None or not spec.submodule_search_locations:
        raise FileNotFoundError("package %r is not on sys.path" % pkg)
    return list(spec.submodule_search_locations)[0]


def resolve(relpath):
    """The file of a shipped relpath ("<package>/<path>"), found from its package on sys.path."""
    parts = relpath.split("/")
    if len(parts) < 2 or parts[0] not in PACKAGES or any(p in ("", ".", "..") for p in parts):
        raise ValueError("not a shipped relpath: %r" % (relpath,))
    return os.path.join(_package_dir(parts[0]), *parts[1:])


def shipped_path():
    """SHIPPED.json: beside the package folders (the build folder's hidden/, the published task's tests/grip/)."""
    return os.path.join(os.path.dirname(HERE), "SHIPPED.json")


def digest_of(relpaths):
    pairs = sorted([r, sha256_file(resolve(r))] for r in relpaths)
    return sha256_bytes(json.dumps(pairs, separators=(",", ":"), ensure_ascii=True).encode("ascii"))


def shipped_files(path=None):
    p = path or shipped_path()
    if not os.path.isfile(p):
        raise FileNotFoundError("no shipped-file list at %s" % p)
    with open(p, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("format") != "grip-shipped/1":
        raise ValueError("%s: not a grip-shipped/1 file" % p)
    return list(data["modules"]) + list(data["data"])


def code_digest(shipped=None):
    """The digest of every shipped file (the shipped-file list's modules and data)."""
    return digest_of(shipped_files(shipped))


# ---------------------------------------------------------------------------------------------------- ceilings
def ceilings_path():
    return os.path.join(HERE, "data", "ceilings.json")


def load_ceilings(path=None):
    """The printed squeeze ceilings (N) of the three shell codes and the soft packs' travel limit (mm)."""
    p = path or ceilings_path()
    with open(p, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("format") != CEILINGS_FORMAT:
        raise ValueError("%s: not a %s file" % (p, CEILINGS_FORMAT))
    return {k: float(data[k]) for k in CEILING_KEYS}


# ------------------------------------------------------------------------------------------------------- load
def _is_frozen_path(path):
    parts = os.path.normpath(os.path.abspath(path)).split(os.sep)
    if "frozen" in parts:
        return True
    return any(parts[i] == "tests" and parts[i + 1] == "hidden" for i in range(len(parts) - 1))


def _manifest(manifest):
    if isinstance(manifest, dict):
        return manifest
    with open(manifest, "r", encoding="utf-8") as fh:
        return json.load(fh)


def load(path, *, manifest=None, dev=False, ceilings_path=None):
    """Read a scenario and check it (see the module docstring)."""
    if dev and _is_frozen_path(path):
        raise ScenarioStale("%s: a development load never reads the frozen set" % path)
    with open(path, "rb") as fh:
        raw = fh.read()
    data = json.loads(raw.decode("ascii"))
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError("%s: not a %s file" % (path, FORMAT))
    if not dev:
        if manifest is None:
            raise ScenarioStale("%s: no manifest given" % path)
        m = _manifest(manifest)
        name = os.path.basename(path)
        entry = next((e for e in m.get("kept", []) if e.get("file") == name), None)
        if entry is None:
            raise ScenarioStale("%s: not in the manifest" % name)
        got = sha256_bytes(raw)
        if got != entry.get("sha256"):
            raise ScenarioStale("%s: sha256 %s... differs from the manifest's %s..." % (
                name, got[:12], str(entry.get("sha256"))[:12]))
        now = code_digest()
        if data.get("code_digest") != now:
            raise ScenarioStale("%s was made with other code (digest %s..., now %s...)" % (
                name, str(data.get("code_digest"))[:12], now[:12]))
    ce = load_ceilings(ceilings_path)
    got_ce = {k: float(data.get("ceilings", {}).get(k, float("nan"))) for k in CEILING_KEYS}
    if got_ce != ce:
        raise ScenarioStale("%s: its ceilings %s differ from the ceilings file's %s" % (path, got_ce, ce))
    return data
