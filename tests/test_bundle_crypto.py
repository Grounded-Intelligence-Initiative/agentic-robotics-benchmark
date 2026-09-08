"""Packaging core (harness/bundle_crypto): tar/zstd determinism, size cap, age round trip,
plaintext mode, key resolution. Ephemeral age keypairs only — no real leaderboard key."""

from __future__ import annotations

import json

import pytest

from harness import bundle_crypto as bc
from harness.trajectory import sha256_hex

pyrage = pytest.importorskip("pyrage")

MEMBERS = {
    "run_manifest.json": b'{"manifest_version": "ale-engine-run/v1"}',
    "episodes/ep-1/lock.json": b'{"agent": {"harness": "claude-code"}}',
    "episodes/ep-1/result.json": b'{"status": "completed"}',
}


def _keypair() -> tuple[str, str]:
    ident = pyrage.x25519.Identity.generate()
    return str(ident), str(ident.to_public())


# --------------------------------------------------------------------------- #
# tar + zstd
# --------------------------------------------------------------------------- #
def test_tar_zst_round_trip_and_determinism():
    a = bc.tar_zst(MEMBERS)
    # insertion order must not matter: names are sorted, mtime pinned to 0
    b = bc.tar_zst(dict(reversed(list(MEMBERS.items()))))
    assert a == b
    assert bc.unpack_tar_zst(a) == MEMBERS


def test_tar_zst_per_member_cap_is_a_hard_stop():
    with pytest.raises(bc.MemberTooLarge):
        bc.tar_zst(MEMBERS, max_member_bytes=10)
    # the cap is a ValueError so generic error handling in callers still catches it
    assert issubclass(bc.MemberTooLarge, ValueError)


# --------------------------------------------------------------------------- #
# encrypt / decrypt
# --------------------------------------------------------------------------- #
def test_encrypt_and_write_round_trip_with_receipt(tmp_path):
    secret, pub = _keypair()
    out = tmp_path / "b.tar.zst.age"
    receipt = bc.encrypt_and_write(MEMBERS, out, pub, bundle_schema="test/v1",
                                   challenge_bound=False, extra_receipt={"note": "x"})
    assert bc.decrypt_bundle(out, secret) == MEMBERS

    assert receipt["bundle_schema"] == "test/v1"
    assert receipt["encryption"] == "age"
    assert receipt["members"] == sorted(MEMBERS)
    assert receipt["plaintext_member_hashes"] == {n: sha256_hex(d) for n, d in MEMBERS.items()}
    assert receipt["compressed_sha256"] == sha256_hex(bc.tar_zst(MEMBERS))
    assert receipt["encrypted_sha256"] == sha256_hex(out.read_bytes())
    assert receipt["encrypted_bytes"] == out.stat().st_size
    assert receipt["recipient_public_key"] == pub
    assert receipt["challenge_bound"] is False
    assert receipt["note"] == "x"
    assert receipt["receipt_hash"].startswith("sha256:")
    on_disk = json.loads(bc.receipt_path(out).read_text())
    assert on_disk == receipt


def test_plaintext_mode_writes_bare_tar_zst(tmp_path):
    out = tmp_path / "b.tar.zst"
    receipt = bc.encrypt_and_write(MEMBERS, out, None, bundle_schema="test/v1",
                                   challenge_bound=False, plaintext=True)
    assert bc.unpack_tar_zst(out.read_bytes()) == MEMBERS
    assert receipt["encryption"] == "none"
    assert receipt["encrypted_sha256"] is None
    assert receipt["encrypted_bytes"] is None
    assert receipt["recipient_public_key"] is None
    assert receipt["compressed_sha256"] == sha256_hex(out.read_bytes())


def test_encrypt_requires_public_key_unless_plaintext(tmp_path):
    with pytest.raises(ValueError):
        bc.encrypt_and_write(MEMBERS, tmp_path / "b.age", None, bundle_schema="t",
                             challenge_bound=False)


def test_wrong_key_and_tampered_ciphertext_fail(tmp_path):
    secret, pub = _keypair()
    other_secret, _ = _keypair()
    out = tmp_path / "b.age"
    bc.encrypt_and_write(MEMBERS, out, pub, bundle_schema="t", challenge_bound=False)
    with pytest.raises(pyrage.DecryptError):
        bc.decrypt_bundle(out, other_secret)
    data = bytearray(out.read_bytes())
    data[-1] ^= 0xFF
    out.write_bytes(bytes(data))
    with pytest.raises(pyrage.DecryptError):
        bc.decrypt_bundle(out, secret)


# --------------------------------------------------------------------------- #
# key resolution
# --------------------------------------------------------------------------- #
def test_resolve_public_key_sources(tmp_path, monkeypatch):
    _, pub = _keypair()
    assert bc.resolve_public_key(pub) == pub
    monkeypatch.setenv("MY_AGE_KEY", pub)
    assert bc.resolve_public_key("env:MY_AGE_KEY") == pub
    kf = tmp_path / "key.txt"
    kf.write_text("# leaderboard recipient\n" + pub + "\n")
    assert bc.resolve_public_key(str(kf)) == pub
    monkeypatch.setenv(bc.PUBLIC_KEY_ENV, pub)
    assert bc.resolve_public_key("") == pub


def test_resolve_public_key_errors(tmp_path, monkeypatch):
    monkeypatch.delenv(bc.PUBLIC_KEY_ENV, raising=False)
    with pytest.raises(ValueError, match="no submission public key"):
        bc.resolve_public_key("")
    monkeypatch.setenv("EMPTY_KEY", "")
    with pytest.raises(ValueError, match="is empty"):
        bc.resolve_public_key("env:EMPTY_KEY")
    kf = tmp_path / "nokey.txt"
    kf.write_text("nothing here\n")
    with pytest.raises(ValueError, match="no age1"):
        bc.resolve_public_key(str(kf))
    with pytest.raises(ValueError, match="unrecognized"):
        bc.resolve_public_key("not-a-key")
    # a malformed age1... string is a ValueError here, not a pyrage.RecipientError later
    with pytest.raises(ValueError, match="invalid age recipient"):
        bc.resolve_public_key("age1notarealkeyzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
    monkeypatch.setenv("BAD_KEY", "age1zzz")
    with pytest.raises(ValueError, match="invalid age recipient"):
        bc.resolve_public_key("env:BAD_KEY")
    kf_bad = tmp_path / "bad.txt"
    kf_bad.write_text("age1zzz\n")
    with pytest.raises(ValueError, match="invalid age recipient"):
        bc.resolve_public_key(str(kf_bad))


def test_resolve_identity_sources(tmp_path, monkeypatch):
    secret, _ = _keypair()
    assert bc.resolve_identity(secret) == secret
    monkeypatch.setenv("MY_AGE_ID", secret)
    assert bc.resolve_identity("env:MY_AGE_ID") == secret
    kf = tmp_path / "id.txt"
    kf.write_text("# created: today\n" + secret + "\n")
    assert bc.resolve_identity(str(kf)) == secret
