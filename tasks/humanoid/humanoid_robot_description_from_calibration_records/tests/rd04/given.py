"""The given model as the grader holds it: the model file's text and the bytes of its 26 link meshes, read from one
folder (a submission may name only these meshes; the grader loads its own copy of them).

    g = given_model()            # GivenModel: xml (str), assets ({mesh file name: bytes}), model (compiled), root
    g = GivenModel.from_dir(d)   # d holds the model file (the only *.xml in d) and meshes/ (or the files beside it)

Where the folder is (first match): the argument; the environment variable RD04_GIVEN_DIR; the folder rd04/given/
beside this module. Without one of them, an optional local module may supply the model; it is not part of the task,
where the verifier always hands the given model in."""
from __future__ import annotations

import glob
import hashlib
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))


class GivenModel:
    def __init__(self, xml_text: str, assets: dict, root: str):
        self.xml = xml_text
        self.assets = dict(assets)
        self.root = root
        self._model = None
        h = hashlib.sha256(self.xml.encode("utf-8"))
        for k in sorted(self.assets):
            h.update(k.encode("utf-8")); h.update(hashlib.sha256(self.assets[k]).digest())
        self.digest = h.hexdigest()

    @property
    def model(self):
        """the given model compiled (on first use: a compile with the 26 link meshes peaks near 1.2 GB)"""
        if self._model is None:
            self._model = mujoco.MjSpec.from_string(self.xml, assets=self.assets).compile()
        return self._model

    @classmethod
    def from_dir(cls, d: str) -> "GivenModel":
        xmls = sorted(glob.glob(os.path.join(d, "*.xml")))
        if len(xmls) != 1:
            raise FileNotFoundError(f"{d}: expected exactly one model file, found {len(xmls)}")
        mdir = os.path.join(d, "meshes") if os.path.isdir(os.path.join(d, "meshes")) else d
        assets = {}
        for p in sorted(glob.glob(os.path.join(mdir, "*.STL")) + glob.glob(os.path.join(mdir, "*.stl"))):
            with open(p, "rb") as f:
                assets[os.path.basename(p)] = f.read()
        with open(xmls[0], encoding="utf-8") as f:
            return cls(f.read(), assets, d)


_CACHE = {}


def _local_fallback() -> GivenModel:
    """the given model from the optional local module (absent from the task folder)"""
    try:
        from . import gate_shim
    except ImportError:
        raise FileNotFoundError("no given model: set RD04_GIVEN_DIR or install rd04/given/") from None
    return gate_shim.given_from_plant_pin()


def given_model(d: str | None = None) -> GivenModel:
    key = d or os.environ.get("RD04_GIVEN_DIR") or "<default>"
    if key in _CACHE:
        return _CACHE[key]
    if d:
        g = GivenModel.from_dir(d)
    elif os.environ.get("RD04_GIVEN_DIR"):
        g = GivenModel.from_dir(os.environ["RD04_GIVEN_DIR"])
    elif os.path.isdir(os.path.join(HERE, "given")):
        g = GivenModel.from_dir(os.path.join(HERE, "given"))
    else:
        g = _local_fallback()
    _CACHE[key] = g
    return g
