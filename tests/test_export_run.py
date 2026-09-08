"""``ale-export`` (harness/export_run) against the real engine records in
tests/fixtures/engine_run (see its README for provenance).

Covers: encrypted round trip with member hashes matching the manifest, ledger enumeration
(read-only, interrupted rows), agent derivation and the mixed / builtin refusals, value-pattern
redaction plus the ``<local>`` source path rewrite, the size-cap policy, challenge embedding,
plaintext mode, and the CLI exit codes."""

from __future__ import annotations

import hashlib
import json
import shutil
import sqlite3
import uuid
from pathlib import Path

import pytest

from harness import bundle_crypto as bc
from harness import export_run as ex
from harness.trajectory import sha256_hex

pyrage = pytest.importorskip("pyrage")

FIXTURES = Path(__file__).parent / "fixtures" / "engine_run"
RUN_DIR = FIXTURES / "validate-5a31c559"
ORACLE = RUN_DIR / "template_reach-oracle-e6ab1c2a"
UNTOUCHED = RUN_DIR / "template_reach-untouched-29e7e3bb"
INTERRUPTED_ID = "template_reach-claude-code-1f2e3d4c"
SCORED = FIXTURES / "scored" / "template_reach-claude-code-5c0ded00"

ENGINE_COMMIT = "019d0ee9f048c57dc4acf1491870268f42a0bfcd"
ALE_VERIFY_HASH = "sha256:b2ddf8e3b7a57479ede66e0e0c980f14d279fd49b0e365f6b0933dd6a3e92072"


def _keypair() -> tuple[str, str]:
    ident = pyrage.x25519.Identity.generate()
    return str(ident), str(ident.to_public())


def _plaintext_export(paths, tmp_path, **kw):
    """Export without age and return (members, manifest, receipt)."""
    out = tmp_path / "bundle.tar.zst"
    receipt = ex.export(paths, out, public_key=None, plaintext=True, **kw)
    members = bc.unpack_tar_zst(out.read_bytes())
    return members, json.loads(members["run_manifest.json"]), receipt


def _copy_episode(src: Path, tmp_path: Path) -> Path:
    dst = tmp_path / src.name
    shutil.copytree(src, dst)
    return dst


def _rewrite_json(path: Path, mutate) -> None:
    obj = json.loads(path.read_text())
    mutate(obj)
    path.write_text(json.dumps(obj, indent=2))


# --------------------------------------------------------------------------- #
# round trip
# --------------------------------------------------------------------------- #
def test_encrypted_round_trip_members_match_manifest(tmp_path):
    secret, pub = _keypair()
    out = tmp_path / "b.ale-engine-run.tar.zst.age"
    receipt = ex.export([RUN_DIR], out, public_key=pub, harness="oracle", allow_builtin=True)

    members = bc.decrypt_bundle(out, secret)
    manifest = json.loads(members["run_manifest.json"])
    assert manifest["manifest_version"] == "ale-engine-run/v1"
    assert manifest["exporter"]["name"] == "ale-robotics-export"
    assert manifest["benchmark"] == {"name": "ale-robotics", "version": "v0.5"}
    assert manifest["submission_origin"] == "smoke"
    assert manifest["challenge"] is None
    assert manifest["agent"] == {"harness": "oracle", "model": "", "version": "1",
                                 "family": "autonomous"}
    assert manifest["engine"] == {"commit": ENGINE_COMMIT, "version": "0.1.0",
                                  "ale_verify_content_hash": ALE_VERIFY_HASH}
    assert manifest["created_at"].endswith("Z")

    # every listed member is packed, hashes over the STORED bytes, nothing else in the archive
    expected = {"run_manifest.json"}
    for entry in manifest["episodes"]:
        for name, digest in entry["members"].items():
            path = f"episodes/{entry['episode_id']}/{name}"
            assert sha256_hex(members[path]) == digest
            expected.add(path)
    assert set(members) == expected
    assert receipt["plaintext_member_hashes"] == {n: sha256_hex(d) for n, d in members.items()}

    by_id = {e["episode_id"]: e for e in manifest["episodes"]}
    oracle = by_id["template_reach-oracle-e6ab1c2a"]
    assert oracle["task_name"] == "template_reach" and oracle["variant"] == "base"
    assert oracle["spec_hash"].startswith("sha256:") and oracle["content_digest"].startswith("sha256:")
    assert oracle["status"] == "completed"
    assert oracle["rewards"] == {"reward": 0.0}
    assert oracle["metrics"]["match_lock_ok"] == 1.0 and oracle["metrics"]["anchor_recorded"] == 0.0
    assert set(oracle["members"]) == {"lock.json", "result.json", "verification.json",
                                      "trajectory.json", "trace.execution.jsonl",
                                      "artifacts/snapshot.json"}
    # --harness oracle dropped the nop episode
    assert "template_reach-untouched-29e7e3bb" not in by_id

    assert receipt["encryption"] == "age"
    assert receipt["bundle_schema"] == "ale-engine-run/v1"
    assert receipt["omitted_members"] == []
    assert receipt["submission_origin"] == "smoke"
    assert bc.receipt_path(out).is_file()


def test_interrupted_entry_comes_from_ledger(tmp_path):
    _, manifest, receipt = _plaintext_export([RUN_DIR], tmp_path, harness="oracle",
                                             allow_builtin=True)
    by_id = {e["episode_id"]: e for e in manifest["episodes"]}
    assert by_id[INTERRUPTED_ID] == {"episode_id": INTERRUPTED_ID, "task_name": "template_reach",
                                     "variant": "base", "status": "interrupted", "members": {}}
    assert {"episode_id": INTERRUPTED_ID, "status": "interrupted"} in receipt["episodes"]
    # ledger order is preserved: the interrupted row was appended last
    assert manifest["episodes"][-1]["episode_id"] == INTERRUPTED_ID
    assert any("cannot be attributed" in w for w in receipt["warnings"])


@pytest.mark.parametrize("journal_mode", ["delete", "wal"])
def test_ledger_rows_are_never_mutated(tmp_path, journal_mode):
    """The engine's Ledger interrupts queued/running rows on open; the exporter must not.

    Runs in both journal modes because the real engine ledger is WAL and the fixture copy is
    not; a 'running' row is planted so that a Ledger-style open would visibly rewrite it."""
    run = tmp_path / "run"
    shutil.copytree(RUN_DIR, run)
    db = run / "ledger.db"
    con = sqlite3.connect(db)
    con.execute("UPDATE episodes SET status='running', failure_type=NULL WHERE episode_id=?",
                (INTERRUPTED_ID,))
    con.commit()
    con.execute(f"PRAGMA journal_mode={journal_mode}")
    con.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    con.close()

    def rows():
        c = sqlite3.connect(f"{db.as_uri()}?mode=ro", uri=True)
        try:
            return c.execute("SELECT * FROM episodes ORDER BY rowid").fetchall()
        finally:
            c.close()

    before_rows, before_bytes = rows(), hashlib.sha256(db.read_bytes()).hexdigest()
    _, manifest, _ = _plaintext_export([run], tmp_path, harness="nop", allow_builtin=True)
    assert rows() == before_rows
    assert hashlib.sha256(db.read_bytes()).hexdigest() == before_bytes
    # a non-terminal row without result.json is still exported as interrupted
    assert manifest["episodes"][-1] == {"episode_id": INTERRUPTED_ID, "task_name": "template_reach",
                                        "variant": "base", "status": "interrupted", "members": {}}


def test_zero_verdict_episode_has_no_optional_agent_members(tmp_path):
    _, manifest, _ = _plaintext_export([UNTOUCHED], tmp_path, allow_builtin=True)
    (entry,) = manifest["episodes"]
    assert entry["metrics"] == {"anchor_recorded": 0.0, "ratio": 0.0}
    # nop never ran an agent phase, so the engine wrote no trajectory
    assert "trajectory.json" not in entry["members"]
    assert {"lock.json", "result.json", "verification.json"} <= set(entry["members"])


# --------------------------------------------------------------------------- #
# agent derivation + refusals
# --------------------------------------------------------------------------- #
def test_mixed_harness_run_dir_is_refused(tmp_path):
    with pytest.raises(ex.ExportRefused, match="mixed agents"):
        ex.export([RUN_DIR], tmp_path / "b", public_key=None, plaintext=True, allow_builtin=True)
    rc = ex.main([str(RUN_DIR), "--out", str(tmp_path / "b"), "--plaintext", "--allow-builtin"])
    assert rc == ex.EXIT_REFUSED


def test_builtin_harness_refused_without_flag(tmp_path):
    with pytest.raises(ex.ExportRefused, match="authoring harness"):
        ex.export([ORACLE], tmp_path / "b", public_key=None, plaintext=True)
    assert ex.main([str(ORACLE), "--out", str(tmp_path / "b"), "--plaintext"]) == ex.EXIT_REFUSED
    # opting in stamps the smoke origin even when a challenge is present
    challenge = tmp_path / "c.json"
    challenge.write_text(json.dumps({"submission_id": "sub-1", "challenge_token": "tok",
                                     "nonce": "n" * 16}))
    _, manifest, _ = _plaintext_export([ORACLE], tmp_path, allow_builtin=True,
                                       challenge=ex.load_challenge(challenge))
    assert manifest["submission_origin"] == "smoke"


def test_harness_filter_with_no_match_is_refused(tmp_path):
    with pytest.raises(ex.ExportRefused, match="no episode was run by harness"):
        ex.export([RUN_DIR], tmp_path / "b", public_key=None, plaintext=True,
                  harness="claude-code")


def test_duplicate_episode_is_refused(tmp_path):
    with pytest.raises(ex.ExportRefused, match="more than once"):
        ex.export([SCORED, SCORED], tmp_path / "b", public_key=None, plaintext=True)


def test_unusable_path_is_refused(tmp_path):
    (tmp_path / "empty").mkdir()
    rc = ex.main([str(tmp_path / "empty"), "--out", str(tmp_path / "b"), "--plaintext"])
    assert rc == ex.EXIT_REFUSED


# --------------------------------------------------------------------------- #
# scored episode, offline / challenge
# --------------------------------------------------------------------------- #
def test_scored_episode_offline_manifest(tmp_path):
    members, manifest, receipt = _plaintext_export([SCORED], tmp_path)
    assert manifest["submission_origin"] == "self_hosted_offline"
    assert manifest["challenge"] is None
    assert uuid.UUID(manifest["submission_id"]).version == 4
    assert manifest["agent"] == {"harness": "claude-code", "model": "us.anthropic.claude-fable-5-1",
                                 "version": "2.1.240", "family": "autonomous"}
    (entry,) = manifest["episodes"]
    assert entry["status"] == "completed"
    assert entry["rewards"] == {"reward": 1.0}
    assert entry["metrics"]["ratio"] == 0.9 and entry["metrics"]["anchor_recorded"] == 1.0
    # the stored result.json agrees with the manifest (what the server re-checks)
    stored = json.loads(members[f"episodes/{entry['episode_id']}/result.json"])
    assert stored["rewards"] == entry["rewards"] and stored["metrics"] == entry["metrics"]
    assert receipt["encryption"] == "none" and receipt["encrypted_sha256"] is None


def test_challenge_embedded_verbatim(tmp_path):
    challenge = {"submission_id": "01J8ZC5Q7X3N9K2M4P6R8T0V1W", "nonce": "a" * 24,
                 "challenge_token": "signed.token.value", "expires_at": "2026-09-03T00:00:00Z",
                 "issued_at": "2026-09-02T00:00:00Z"}
    cf = tmp_path / "challenge.json"
    cf.write_text(json.dumps(challenge))
    _, manifest, receipt = _plaintext_export([SCORED], tmp_path, challenge=ex.load_challenge(cf))
    assert manifest["challenge"] == challenge
    assert manifest["submission_id"] == challenge["submission_id"]
    assert manifest["submission_origin"] == "challenge_issued"
    assert receipt["challenge_bound"] is True


def test_challenge_file_without_required_fields_is_usage_error(tmp_path):
    cf = tmp_path / "challenge.json"
    cf.write_text(json.dumps({"submission_id": "x", "challenge_token": "t"}))  # no nonce
    with pytest.raises(ex.ExportRefused, match="nonce"):
        ex.load_challenge(cf)
    rc = ex.main([str(SCORED), "--out", str(tmp_path / "b"), "--plaintext",
                  "--challenge-file", str(cf)])
    assert rc == ex.EXIT_REFUSED


def test_failed_episode_ships_without_rewards_or_verification(tmp_path):
    ep = _copy_episode(SCORED, tmp_path)
    (ep / "verification.json").unlink()

    def fail(result):
        result["status"] = "agent_error"
        del result["rewards"]
        result["failure"] = {"error_type": "AgentCrashed", "message": "exit 137"}

    _rewrite_json(ep / "result.json", fail)
    _, manifest, _ = _plaintext_export([ep], tmp_path)
    (entry,) = manifest["episodes"]
    assert entry["status"] == "agent_error"
    assert "rewards" not in entry
    assert "verification.json" not in entry["members"]


def test_completed_episode_without_verification_is_an_error(tmp_path):
    ep = _copy_episode(ORACLE, tmp_path)
    (ep / "verification.json").unlink()
    with pytest.raises(ex.ExportError, match="verification.json is missing"):
        ex.export([ep], tmp_path / "b", public_key=None, plaintext=True, allow_builtin=True)
    rc = ex.main([str(ep), "--out", str(tmp_path / "b"), "--plaintext", "--allow-builtin"])
    assert rc == ex.EXIT_ERROR


def test_ledger_row_disagreeing_with_result_is_an_error(tmp_path):
    run = tmp_path / "run"
    shutil.copytree(RUN_DIR, run)
    _rewrite_json(run / ORACLE.name / "result.json",
                  lambda r: r.__setitem__("episode_id", "template_reach-oracle-ffffffff"))
    with pytest.raises(ex.ExportError, match="ledger row"):
        ex.export([run], tmp_path / "b", public_key=None, plaintext=True, harness="oracle",
                  allow_builtin=True)


def test_terminal_ledger_row_without_result_is_a_damaged_run_dir(tmp_path):
    """A row the engine recorded as terminal always has result.json; its absence is corruption,
    not a Ctrl-C, and must not be relabelled ``interrupted``."""
    run = tmp_path / "run"
    shutil.copytree(RUN_DIR, run)
    shutil.rmtree(run / UNTOUCHED.name)          # ledger says status=completed
    with pytest.raises(ex.ExportError, match="status='completed' but result.json is missing"):
        ex.export([run], tmp_path / "b", public_key=None, plaintext=True, harness="oracle",
                  allow_builtin=True)
    rc = ex.main([str(run), "--out", str(tmp_path / "b"), "--plaintext", "--harness", "oracle",
                  "--allow-builtin"])
    assert rc == ex.EXIT_ERROR


@pytest.mark.parametrize("status", ["queued", "running", "interrupted"])
def test_non_terminal_ledger_row_without_result_is_interrupted(tmp_path, status):
    run = tmp_path / "run"
    shutil.copytree(RUN_DIR, run)
    con = sqlite3.connect(run / "ledger.db")
    con.execute("UPDATE episodes SET status=? WHERE episode_id=?", (status, INTERRUPTED_ID))
    con.commit()
    con.close()
    _, manifest, _ = _plaintext_export([run], tmp_path, harness="oracle", allow_builtin=True)
    assert manifest["episodes"][-1]["status"] == "interrupted"
    assert manifest["episodes"][-1]["episode_id"] == INTERRUPTED_ID


@pytest.mark.parametrize("bad_id", [".", "..", "../escape", "a/b", "with space", "ünïcode",
                                    "x" * 201])
def test_episode_id_unfit_for_a_path_segment_is_refused(tmp_path, bad_id):
    ep = _copy_episode(SCORED, tmp_path)
    _rewrite_json(ep / "result.json", lambda r: r.__setitem__("episode_id", bad_id))
    with pytest.raises(ex.ExportError, match="not a valid id"):
        ex.export([ep], tmp_path / "b", public_key=None, plaintext=True)


def test_episode_id_regex_mirrors_the_server():
    assert ex.EPISODE_ID_RE.pattern == r"^[A-Za-z0-9_.-]{1,200}$"
    assert ex.EPISODE_ID_RE.match("drone_hover-1a2b3c4d.reverify-abcdef")


def test_harness_filter_is_applied_before_the_other_checks(tmp_path):
    """A defective episode of a harness that is filtered out must not abort the export."""
    run = tmp_path / "run"
    shutil.copytree(RUN_DIR, run)
    nop = run / UNTOUCHED.name
    (nop / "result.json").write_text("{not json")               # broken record
    (nop / "verification.json").unlink()
    with pytest.raises(ex.ExportError):                          # unfiltered: still an error
        ex.export([run], tmp_path / "b", public_key=None, plaintext=True, harness="nop",
                  allow_builtin=True)
    _, manifest, _ = _plaintext_export([run], tmp_path, harness="oracle", allow_builtin=True)
    ids = {e["episode_id"] for e in manifest["episodes"]}
    assert ORACLE.name in ids and UNTOUCHED.name not in ids
    # lock.json itself is read before the filter (it carries the harness), so a broken lock
    # is still an error even for a filtered-out episode
    (nop / "lock.json").write_text("{not json")
    with pytest.raises(ex.ExportError, match="lock.json"):
        ex.export([run], tmp_path / "b", public_key=None, plaintext=True, harness="oracle",
                  allow_builtin=True)


def test_duplicate_dir_is_refused_even_when_filtered_out(tmp_path):
    with pytest.raises(ex.ExportRefused, match="more than once"):
        ex.export([SCORED, SCORED], tmp_path / "b", public_key=None, plaintext=True,
                  harness="oracle")


# --------------------------------------------------------------------------- #
# redaction
# --------------------------------------------------------------------------- #
def test_value_pattern_redaction_and_local_source_path(tmp_path):
    ep = _copy_episode(SCORED, tmp_path)
    leaked_key = "sk-ant-api03-LEAKEDLEAKEDLEAKEDLEAKED"
    leaked_aws = "AKIAAAAAAAAAAAAAAAAA"
    _rewrite_json(ep / "lock.json",
                  lambda lock: lock["agent"]["settings"].__setitem__("note", f"key {leaked_key}"))
    _rewrite_json(ep / "trajectory.json",
                  lambda t: t["steps"][-1].__setitem__("message", f"echo {leaked_key}"))
    _rewrite_json(ep / "artifacts" / "snapshot.json",
                  lambda s: s["entries"][0].__setitem__("source", f"/home/user/{leaked_aws}"))
    trace = ep / "trace.execution.jsonl"
    trace.write_text(trace.read_text() + json.dumps(
        {"kind": "command_finished", "stderr": {"inline": f"Authorization: Bearer {leaked_aws}"}}) + "\n")

    members, _, _ = _plaintext_export([ep], tmp_path)
    blob = b"".join(members.values())
    assert leaked_key.encode() not in blob
    assert leaked_aws.encode() not in blob
    assert b"/tmp/ale-audit-tpl" not in blob

    lock = json.loads(members[f"episodes/{SCORED.name}/lock.json"])
    assert lock["task"]["source"]["path"] == "<local>"
    assert lock["task"]["source"]["kind"] == "local"
    # value-pattern ONLY: the token-budget fields survive verbatim (key-name redaction would
    # have turned them into the redaction marker)
    assert lock["gateway"]["limits"]["max_input_tokens"] == "unlimited"
    assert lock["gateway"]["limits"]["max_total_tokens"] == "unlimited"
    # the trace stays line-parseable JSONL after redaction, with the marker in place of the key
    lines = members[f"episodes/{SCORED.name}/trace.execution.jsonl"].decode().splitlines()
    parsed = [json.loads(line) for line in lines]
    assert parsed[-1]["stderr"]["inline"] == "Authorization: Bearer «redacted»"


# --------------------------------------------------------------------------- #
# size policy
# --------------------------------------------------------------------------- #
def test_oversized_optional_member_is_omitted_and_listed(tmp_path):
    ep = _copy_episode(SCORED, tmp_path)
    _rewrite_json(ep / "trajectory.json",
                  lambda t: t["steps"].append({"step_id": 9, "source": "agent",
                                               "message": "x" * 200_000}))
    cap = 64 * 1024
    members, manifest, receipt = _plaintext_export([ep], tmp_path, cap_bytes=cap)
    (entry,) = manifest["episodes"]
    assert "trajectory.json" not in entry["members"]
    assert f"episodes/{SCORED.name}/trajectory.json" not in members
    assert {"lock.json", "result.json", "verification.json", "trace.execution.jsonl",
            "artifacts/snapshot.json"} == set(entry["members"])
    (omitted,) = receipt["omitted_members"]
    assert omitted["episode_id"] == SCORED.name and omitted["member"] == "trajectory.json"
    assert omitted["bytes"] > cap == omitted["cap_bytes"]


def test_oversized_optional_member_is_omitted_without_being_read(tmp_path):
    """The on-disk size decides first: a file over the cap is never parsed, so invalid JSON
    in it cannot raise, and the receipt records the on-disk byte count."""
    ep = _copy_episode(SCORED, tmp_path)
    cap = 64 * 1024
    junk = b"{" + b"x" * (cap + 1)                 # invalid JSON, one byte over the cap
    (ep / "trajectory.json").write_bytes(junk)
    members, manifest, receipt = _plaintext_export([ep], tmp_path, cap_bytes=cap)
    (entry,) = manifest["episodes"]
    assert "trajectory.json" not in entry["members"]
    assert f"episodes/{SCORED.name}/trajectory.json" not in members
    (omitted,) = receipt["omitted_members"]
    assert omitted == {"episode_id": SCORED.name, "member": "trajectory.json",
                       "bytes": len(junk), "cap_bytes": cap}
    # ...whereas the same junk under the cap is read and refused as malformed
    (ep / "trajectory.json").write_bytes(b"{" + b"x" * 10)
    with pytest.raises(ex.ExportError, match="not valid JSON"):
        ex.export([ep], tmp_path / "b", public_key=None, plaintext=True, cap_bytes=cap)


def test_load_episode_skips_stat_over_cap_before_reading(tmp_path, monkeypatch):
    ep = _copy_episode(SCORED, tmp_path)
    big = ep / "trace.execution.jsonl"
    big.write_bytes(b"\n" * 8192)             # the fixture's other members are < 4096 bytes
    real_read = Path.read_bytes

    def guarded(self):
        assert self != big, "oversized member must not be read"
        return real_read(self)

    monkeypatch.setattr(Path, "read_bytes", guarded)
    loaded = ex.load_episode(ex.EpisodeDir(path=ep), cap_bytes=4096)
    assert loaded.oversized_optional == {"trace.execution.jsonl": 8192}
    assert "trace.execution.jsonl" not in loaded.raw_optional
    assert "trajectory.json" in loaded.raw_optional


def test_oversized_required_member_aborts(tmp_path):
    ep = _copy_episode(SCORED, tmp_path)
    _rewrite_json(ep / "lock.json",
                  lambda lock: lock["agent"]["settings"].__setitem__("pad", "x" * 200_000))
    with pytest.raises(ex.ExportError, match="required member"):
        ex.export([ep], tmp_path / "b", public_key=None, plaintext=True, cap_bytes=64 * 1024)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def test_cli_plaintext_run_dir(tmp_path, capsys):
    out = tmp_path / "b.tar.zst"
    rc = ex.main([str(RUN_DIR), "--out", str(out), "--plaintext", "--harness", "oracle",
                  "--allow-builtin"])
    assert rc == ex.EXIT_OK
    stdout = capsys.readouterr().out
    assert "wrote" in stdout and "1 packed, 1 interrupted" in stdout and "origin: smoke" in stdout
    assert out.is_file() and bc.receipt_path(out).is_file()
    assert "run_manifest.json" in bc.unpack_tar_zst(out.read_bytes())


def test_cli_encrypted_with_public_key_flag(tmp_path):
    secret, pub = _keypair()
    out = tmp_path / "b.age"
    rc = ex.main([str(SCORED), "--out", str(out), "--public-key", pub])
    assert rc == ex.EXIT_OK
    assert "run_manifest.json" in bc.decrypt_bundle(out, secret)


def test_cli_missing_public_key_is_usage_error(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv(bc.PUBLIC_KEY_ENV, raising=False)
    rc = ex.main([str(SCORED), "--out", str(tmp_path / "b.age")])
    assert rc == ex.EXIT_REFUSED
    assert "no submission public key" in capsys.readouterr().err


def test_cli_malformed_public_key_is_a_refusal_not_a_traceback(tmp_path, capsys):
    out = tmp_path / "b.age"
    rc = ex.main([str(SCORED), "--out", str(out), "--public-key", "age1zzz"])
    assert rc == ex.EXIT_REFUSED
    err = capsys.readouterr().err
    assert err.startswith("ale-export: refused: invalid age recipient public key")
    assert not out.exists()          # fails fast, before any episode is packed


def test_benchmark_release_rendering():
    assert ex.benchmark_release("0.5.0") == "v0.5"
    assert ex.benchmark_release("1.2.3") == "v1.2"
    assert ex.project_version() == "0.5.0"
