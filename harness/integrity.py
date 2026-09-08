"""Integrity utilities — file hashing + a Merkle-style completeness root.

Host-side (py3.11 harness). The canonical event encoding and hash chain live in
``harness.trajectory``; this module adds the
harness-only pieces: streaming file hashes (1 MiB chunked sha256) and an ordered
Merkle root over a set of named artifacts, used as a bundle's integrity root.

No crypto/zstd/age here — packaging and encryption live in ``harness/bundle_crypto.py``.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

# Reuse the shared hashing primitives so the wire form is identical everywhere.
from harness.trajectory import sha256_hex  # noqa: F401 — re-exported for callers

_CHUNK = 1 << 20  # 1 MiB streaming reads


def sha256_bytes(data: bytes) -> str:
    """``"sha256:<hex>"`` of raw bytes (same form as harness.trajectory)."""
    return sha256_hex(data)


def sha256_file(path: str | Path) -> str:
    """Streaming sha256 of a file as ``"sha256:<hex>"`` (1 MiB chunks)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(_CHUNK), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()


def _hex(digest: str) -> str:
    """Strip the ``sha256:`` prefix for internal concatenation."""
    return digest.split(":", 1)[1] if ":" in digest else digest


def merkle_root(leaves: list[str]) -> str:
    """Deterministic binary Merkle root over pre-hashed ``sha256:`` leaves.

    Leaves are combined IN THE GIVEN ORDER (the caller sorts by a stable key,
    e.g. filename, before calling — order is part of the commitment). An odd
    node at a level is promoted (duplicated) to the next level, the common
    convention. Empty input yields the hash of the empty string; a single leaf
    is its own root.
    """
    if not leaves:
        return sha256_hex(b"")
    level = [_hex(x) for x in leaves]
    while len(level) > 1:
        nxt: list[str] = []
        for i in range(0, len(level), 2):
            left = level[i]
            right = level[i + 1] if i + 1 < len(level) else left
            combined = hashlib.sha256((left + right).encode("utf-8")).hexdigest()
            nxt.append(combined)
        level = nxt
    return "sha256:" + level[0]


def hash_files(paths: dict[str, str | Path]) -> dict[str, dict[str, Any]]:
    """Hash a set of named files → ``{name: {sha256, bytes}}`` (missing skipped).

    ``name`` is the logical/relative name to record; the value is the path on
    disk. Files that don't exist are omitted (the caller decides whether a
    missing artifact is an error — the finalizer records it as such).
    """
    out: dict[str, dict[str, Any]] = {}
    for name, p in paths.items():
        p = Path(p)
        if p.is_file():
            out[name] = {"sha256": sha256_file(p), "bytes": p.stat().st_size}
    return out


def integrity_root(artifact_hashes: dict[str, dict[str, Any]]) -> str:
    """Merkle root over an artifact-hash map, ordered by name (stable).

    Each leaf commits to both the name and the content hash, so renaming or
    swapping files changes the root.
    """
    leaves = [
        sha256_hex(("{}\0{}".format(name, meta["sha256"])).encode("utf-8"))
        for name, meta in sorted(artifact_hashes.items())
    ]
    return merkle_root(leaves)
