"""Unit tests for the hashing primitives that survived the legacy archival.

Covers harness/trajectory.py (canonical JSON bytes + sha256) and harness/integrity.py (file
hashing, Merkle root). The hash-chained event stream, recording levels and tensor digests are
archived with the legacy harness (tag ``legacy-harness-final``) and are not exercised here.
"""

import inspect
import json

import pytest

from harness import integrity
from harness import trajectory as tj


# --------------------------------------------------------------------------- #
# canonical json
# --------------------------------------------------------------------------- #
def test_canonical_is_key_order_independent():
    a = tj.canonical_bytes({"b": 1, "a": 2, "c": [3, {"z": 1, "y": 2}]})
    b = tj.canonical_bytes({"c": [3, {"y": 2, "z": 1}], "a": 2, "b": 1})
    assert a == b
    # deterministic separators, no spaces
    assert b" " not in a


def test_canonical_non_finite_sentinels():
    s = tj.canonical_dumps({"x": float("nan"), "y": float("inf"), "z": float("-inf")})
    assert "__nan__" in s and "__inf__" in s and "__-inf__" in s
    # must never emit a bare NaN/Infinity token (invalid JSON)
    assert "NaN" not in s and "Infinity" not in s
    json.loads(s)  # round-trips as valid JSON


def test_canonical_bytes_summarized_not_inlined():
    payload = {"blob": b"\x00\x01\x02\x03"}
    s = tj.canonical_dumps(payload)
    assert "__bytes_sha256__" in s
    assert "sha256:" in s


def test_canonical_tuples_and_non_string_keys_are_normalised():
    assert tj.canonical_dumps({1: (1, 2)}) == tj.canonical_dumps({"1": [1, 2]})


def test_unknown_types_fall_back_to_str():
    class Thing:
        def __str__(self):
            return "thing"

    assert json.loads(tj.canonical_dumps({"t": Thing()})) == {"t": "thing"}


def test_sha256_hex_form_and_payload_hash():
    h = tj.sha256_hex(b"abc")
    assert h.startswith("sha256:") and len(h) == 71
    assert tj.payload_hash({"a": 1}) == tj.sha256_hex(tj.canonical_bytes({"a": 1}))
    assert tj.payload_hash({"a": 1}) != tj.payload_hash({"a": 2})


def test_surface_is_the_primitives_only():
    """The event chain / recorder machinery is archived; nothing may quietly re-grow here."""
    public = {name for name in dir(tj)
              if not name.startswith("_") and not inspect.ismodule(getattr(tj, name))}
    assert public - {"annotations", "Any"} == {
        "sha256_hex", "canonical_dumps", "canonical_bytes", "payload_hash"}


# --------------------------------------------------------------------------- #
# integrity: file hashing + merkle
# --------------------------------------------------------------------------- #
def test_sha256_file_matches_bytes(tmp_path):
    p = tmp_path / "f.bin"
    data = b"hello world" * 1000
    p.write_bytes(data)
    assert integrity.sha256_file(p) == integrity.sha256_bytes(data)


def test_merkle_root_deterministic_and_order_sensitive():
    a = integrity.sha256_bytes(b"a")
    b = integrity.sha256_bytes(b"b")
    c = integrity.sha256_bytes(b"c")
    r1 = integrity.merkle_root([a, b, c])
    r2 = integrity.merkle_root([a, b, c])
    r3 = integrity.merkle_root([c, b, a])
    assert r1 == r2
    assert r1 != r3
    assert r1.startswith("sha256:")


def test_merkle_single_and_empty():
    a = integrity.sha256_bytes(b"a")
    assert integrity.merkle_root([a]) == a
    assert integrity.merkle_root([]) == integrity.sha256_bytes(b"")


def test_integrity_root_changes_on_content_or_name(tmp_path):
    f1 = tmp_path / "manifest.json"
    f2 = tmp_path / "events.jsonl"
    f1.write_text("{}")
    f2.write_text("line\n")
    h = integrity.hash_files({"manifest.json": f1, "events.jsonl": f2})
    root = integrity.integrity_root(h)
    # changing content changes the root
    f1.write_text("{ }")
    h2 = integrity.hash_files({"manifest.json": f1, "events.jsonl": f2})
    assert integrity.integrity_root(h2) != root


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
