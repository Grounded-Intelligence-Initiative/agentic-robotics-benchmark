"""The plant of the grip-controller acceptance: the hand's mechanics, its servo firmware and electrical stage, the
specimens and the cell, and the session that runs them tick by tick.

The hidden numbers the plant uses live in truth_constants.json beside this file and are read here, once; no module of
the plant writes one of them as a literal.
"""
import json as _json
import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))
MODEL_DIR = _os.path.join(HERE, "model")
MODEL_FILE = "hand_cell.xml"

with open(_os.path.join(HERE, "truth_constants.json"), encoding="utf-8") as _fh:
    TRUTH = _json.load(_fh)
