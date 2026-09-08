"""Packaging core for leaderboard bundles: members -> tar -> zstd -> age, and back.

This is the envelope the website already decrypts (``docs/dev/20260902-engine-run-submission.md``
§3). It was lifted out of the legacy ``harness/submission.py`` so that the engine-run exporter
(``harness/export_run.py``) and the legacy exporter share one implementation while the legacy
module is archived. Nothing here knows what a bundle *means*; callers decide the member set.

Security posture (docs/security-model.md):
  * age (pyrage) is the only cipher — no home-grown crypto.
  * only the recipient's PUBLIC key is ever handled on the packing side; the identity
    (private key) is used solely by ``decrypt_bundle`` on the verifying side and in tests.
  * the tar is deterministic (sorted names, ``mtime=0``) so the same members always yield the
    same compressed bytes, which is what makes the receipt's ``compressed_sha256`` meaningful.
  * a per-member size cap is a hard stop, not a silent truncation: callers must apply their own
    omission policy *before* packing, so that whatever manifest they wrote describes exactly
    the bytes that were packed.
"""

from __future__ import annotations

import io
import json
import os
import tarfile
from pathlib import Path
from typing import Any

import zstandard as zstd

from harness.trajectory import canonical_bytes, sha256_hex

# A member larger than this aborts packing. Chosen so a runaway trajectory cannot produce a
# multi-GB bundle silently; the engine-run exporter applies a much tighter policy of its own.
DEFAULT_MAX_MEMBER_BYTES = 256 * 1024 * 1024

# zstd level 19 is the slow/strong end; bundles are written once and uploaded, so the extra CPU
# is cheap next to the transfer it saves.
ZSTD_LEVEL = 19

PUBLIC_KEY_ENV = "ALE_SUBMISSION_PUBLIC_KEY"


class MemberTooLarge(ValueError):
    """A member exceeded the per-member cap; the caller should have omitted or refused it."""


# --------------------------------------------------------------------------- #
# key resolution
# --------------------------------------------------------------------------- #
def resolve_public_key(source: str = "") -> str:
    """Resolve the age recipient public key from a literal, ``env:VAR``, or a file path.

    Falls back to ``$ALE_SUBMISSION_PUBLIC_KEY`` when ``source`` is empty. Raises ``ValueError``
    with an actionable message when nothing resolves *or the resolved string is not a valid
    age recipient* — validated here, before any episode is read, so a typo'd key is a usage
    error (exit 2 in ``ale-export``) rather than a traceback after all the work is done.
    The private key is NEVER handled here.
    """
    src = source or os.environ.get(PUBLIC_KEY_ENV, "")
    if not src:
        raise ValueError(
            "no submission public key: pass --public-key (an age1... string, a file path, or "
            f"env:VARNAME) or set ${PUBLIC_KEY_ENV}")
    if src.startswith("age1"):
        return _validated_recipient(src.strip())
    if src.startswith("env:"):
        var = src[4:]
        val = os.environ.get(var, "")
        if not val:
            raise ValueError(f"env var {var} (from --public-key env:{var}) is empty")
        return _validated_recipient(val.strip())
    p = Path(src)
    if p.is_file():
        # a key file may contain comments; take the first age1... token
        for line in p.read_text().splitlines():
            line = line.strip()
            if line.startswith("age1"):
                return _validated_recipient(line)
        raise ValueError(f"no age1... recipient found in key file {p}")
    raise ValueError(f"unrecognized public-key source: {src!r}")


def _validated_recipient(key: str) -> str:
    """Parse ``key`` with pyrage; ``ValueError`` (not ``pyrage.RecipientError``) on failure."""
    import pyrage

    try:
        pyrage.x25519.Recipient.from_str(key)
    except Exception as exc:  # pyrage.RecipientError is a plain Exception subclass
        raise ValueError(f"invalid age recipient public key {key[:12]}...: {exc}") from exc
    return key


def resolve_identity(source: str) -> str:
    """Resolve an age identity (``AGE-SECRET-KEY-1...``) from a literal, ``env:VAR``, or a file.

    This is the PRIVATE side and lives only where verification runs — the runner never has it.
    """
    if source.startswith("env:"):
        return os.environ.get(source[4:], "")
    p = Path(source)
    if not source.startswith("AGE-SECRET-KEY-") and p.is_file():
        for line in p.read_text().splitlines():
            if line.strip().startswith("AGE-SECRET-KEY-"):
                return line.strip()
    return source


# --------------------------------------------------------------------------- #
# tar + zstd
# --------------------------------------------------------------------------- #
def tar_zst(members: dict[str, bytes], *, max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
            level: int = ZSTD_LEVEL) -> bytes:
    """Pack ``{archive_path: bytes}`` into a deterministic tar and zstd-compress it.

    Raises ``MemberTooLarge`` instead of dropping a member: silently omitting would leave the
    caller's manifest describing bytes that are not in the archive.
    """
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode="w") as tar:
        for name in sorted(members):
            data = members[name]
            if len(data) > max_member_bytes:
                raise MemberTooLarge(
                    f"member {name} is {len(data)} bytes > cap {max_member_bytes}")
            info = tarfile.TarInfo(name=name)
            info.size = len(data)
            info.mtime = 0  # deterministic: no wall-clock in the archive
            tar.addfile(info, io.BytesIO(data))
    return zstd.ZstdCompressor(level=level).compress(raw.getvalue())


def unpack_tar_zst(compressed: bytes) -> dict[str, bytes]:
    """Inverse of ``tar_zst``: zstd -> tar -> ``{archive_path: bytes}`` (regular files only)."""
    tar_bytes = zstd.ZstdDecompressor().decompress(compressed)
    out: dict[str, bytes] = {}
    with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r") as tar:
        for m in tar.getmembers():
            f = tar.extractfile(m)
            if f is not None:
                out[m.name] = f.read()
    return out


# --------------------------------------------------------------------------- #
# write + receipt
# --------------------------------------------------------------------------- #
def encrypt_and_write(members: dict[str, bytes], out_path: Path, public_key: str | None, *,
                      bundle_schema: str, challenge_bound: bool,
                      max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
                      extra_receipt: dict[str, Any] | None = None,
                      plaintext: bool = False) -> dict[str, Any]:
    """tar(sorted, mtime=0) -> zstd -> age, write the file plus ``<out>.receipt.json``.

    ``plaintext=True`` skips age and writes the bare ``tar.zst`` (tests and the website's
    ingest-integration stage read it directly); the receipt then records ``encryption: none``
    and null encrypted hashes so nobody mistakes it for a sealed bundle.

    The receipt records only hashes and sizes, never member contents, so a submitter can prove
    what they sent without holding the private key. Its ``encrypted_sha256`` is what a
    maintainer cites as ``evidence_hash`` when attesting a re-run (design §7).
    """
    out_path = Path(out_path)
    plaintext_hashes = {name: sha256_hex(data) for name, data in sorted(members.items())}
    compressed = tar_zst(members, max_member_bytes=max_member_bytes)

    if plaintext:
        payload = compressed
        encrypted_sha256: str | None = None
        encrypted_bytes: int | None = None
        recipient_key: str | None = None
    else:
        import pyrage

        if not public_key:
            raise ValueError("encrypt_and_write: a recipient public key is required unless "
                             "plaintext=True")
        recipient = pyrage.x25519.Recipient.from_str(public_key)
        payload = pyrage.encrypt(compressed, [recipient])
        encrypted_sha256 = sha256_hex(payload)
        encrypted_bytes = len(payload)
        recipient_key = public_key

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(payload)

    receipt: dict[str, Any] = {
        "bundle_schema": bundle_schema,
        "output": str(out_path),
        "encryption": "none" if plaintext else "age",
        "members": sorted(members),
        "plaintext_member_hashes": plaintext_hashes,
        "compressed_sha256": sha256_hex(compressed),
        "compressed_bytes": len(compressed),
        "encrypted_sha256": encrypted_sha256,
        "encrypted_bytes": encrypted_bytes,
        "recipient_public_key": recipient_key,
        "challenge_bound": challenge_bound,
        **(extra_receipt or {}),
    }
    receipt["receipt_hash"] = sha256_hex(canonical_bytes(receipt))
    receipt_path(out_path).write_text(json.dumps(receipt, indent=2, ensure_ascii=False))
    return receipt


def receipt_path(out_path: Path) -> Path:
    """Where ``encrypt_and_write`` puts the receipt for a given bundle path."""
    out_path = Path(out_path)
    return out_path.parent / (out_path.name + ".receipt.json")


def decrypt_bundle(bundle_path: Path, identity_source: str) -> dict[str, bytes]:
    """Decrypt + unpack an age bundle (verifying side / tests). Returns ``{name: bytes}``."""
    import pyrage

    identity = pyrage.x25519.Identity.from_str(resolve_identity(identity_source))
    compressed = pyrage.decrypt(Path(bundle_path).read_bytes(), [identity])
    return unpack_tar_zst(compressed)
