"""Canonical JSON bytes + sha256 — the hashing primitives the exporter and integrity share.

This module is the ONE place the canonical encoding is defined: ``bundle_crypto`` hashes the
receipt with it, ``export_run`` and ``integrity`` take ``sha256_hex`` from here, so a hash
written anywhere in this repo has the same form (``"sha256:<hex>"``) and the same input bytes
for the same object. The legacy hash-chained event stream that used to live here is archived
(``docs/dev/archive/legacy-openset/trajectory.md``); only the primitives survive.

Canonical JSON rules:
  * ``json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)`` then UTF-8
    encode. sort_keys + fixed separators make the byte stream reproducible.
  * Non-finite floats are replaced by sentinel strings (``"__nan__"`` / ``"__inf__"`` /
    ``"__-inf__"``) — JSON has no NaN/Inf and we never emit ``NaN`` tokens.
  * Raw ``bytes`` are never inlined; they collapse to a sha256 summary.
  * Any other type falls back to ``str()`` so the encoder is total.

Standard library only. No crypto / zstd / age here — those live in ``harness/bundle_crypto.py``.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any

_NAN = "__nan__"
_POS_INF = "__inf__"
_NEG_INF = "__-inf__"


def sha256_hex(data: bytes) -> str:
    """sha256 of raw bytes as ``"sha256:<hex>"`` (repo hashing convention)."""
    return "sha256:" + hashlib.sha256(data).hexdigest()


def _sanitize(obj: Any) -> Any:
    """Make ``obj`` JSON-canonical-safe (finite, plain types, no inlined blobs)."""
    if obj is None or isinstance(obj, bool):
        return obj
    if isinstance(obj, float):
        if math.isnan(obj):
            return _NAN
        if math.isinf(obj):
            return _POS_INF if obj > 0 else _NEG_INF
        return obj
    if isinstance(obj, (int, str)):
        return obj
    if isinstance(obj, bytes):
        # Never inline raw bytes; summarize (large blobs stay out of the hash input).
        return {"__bytes_sha256__": sha256_hex(obj), "__len__": len(obj)}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    if isinstance(obj, dict):
        # Keys must be strings for a stable sort; coerce defensively.
        return {(k if isinstance(k, str) else str(k)): _sanitize(v)
                for k, v in obj.items()}
    # Unknown type: last-resort string form (keeps the encoder total).
    return str(obj)


def canonical_dumps(obj: Any) -> str:
    """Deterministic JSON string (sort_keys, fixed separators, sentinels)."""
    return json.dumps(_sanitize(obj), sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False)


def canonical_bytes(obj: Any) -> bytes:
    return canonical_dumps(obj).encode("utf-8")


def payload_hash(payload: Any) -> str:
    """``sha256_hex`` of the canonical bytes of ``payload``."""
    return sha256_hex(canonical_bytes(payload))
