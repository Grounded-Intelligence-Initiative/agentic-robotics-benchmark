"""Unit tests for ``harness/redact.py`` — secret scrubbing of exported records.

Split out of the legacy ``test_recorders.py`` when the legacy harness was archived
(``legacy-harness-final``): the recorders went away, the redaction module stayed
because the engine-run exporter (``harness/export_run.py``) scrubs every bundle
member through it. Key-name redaction (``redact``) is still exercised here because
it is public API and the value-pattern rules it shares with ``redact_values`` are
the ones that protect the exported bundle.
"""

import pytest

from harness import redact


def test_redact_aws_creds_by_key():
    d = {"AWS_SECRET_ACCESS_KEY": "abc/secret+value", "AWS_SESSION_TOKEN": "tok",
         "AWS_REGION": "us-east-1", "model": "opus"}
    out = redact.redact(d)
    assert out["AWS_SECRET_ACCESS_KEY"] == redact.REDACTED
    assert out["AWS_SESSION_TOKEN"] == redact.REDACTED
    assert out["AWS_REGION"] == "us-east-1"   # region is not secret
    assert out["model"] == "opus"


def test_redact_does_not_eat_token_metrics():
    d = {"input_tokens": 100, "output_tokens": 50, "cache_read_tokens": 10,
         "num_turns": 5}
    out = redact.redact(d)
    assert out == d  # none of these are secrets


def test_redact_preserves_challenge_token_but_not_other_tokens():
    """The challenge token MUST survive redaction verbatim — it's a single-use, expiring,
    public-safe value the leaderboard verifies (the site issued it; it holds only the
    public key). Redacting it (substring 'token') false-failed challenge_sig on every
    online submit. Other *_token keys stay redacted."""
    d = {"challenge": {"submission_id": "sub_x", "nonce": "nnn",
                       "challenge_token": ".eJxREAL.sig"},
         "aws_session_token": "SECRET", "some_token": "leak"}
    out = redact.redact(d)
    assert out["challenge"]["challenge_token"] == ".eJxREAL.sig"  # verbatim
    assert out["aws_session_token"] == redact.REDACTED
    assert out["some_token"] == redact.REDACTED  # only challenge_token is allowlisted


def test_redact_value_patterns():
    s = "creds ASIAABCDEFGHIJKLMNOP and Authorization: AWS4-HMAC-SHA256 Credential=x"
    out = redact.redact_str(s)
    assert "ASIAABCDEFGHIJKLMNOP" not in out
    assert "AWS4-HMAC-SHA256" not in out


def test_redact_nested():
    d = {"env": {"AWS_ACCESS_KEY_ID": "AKIAAAAAAAAAAAAAAAAA", "ok": 1},
         "list": [{"password": "p"}, "Bearer abc.def.ghi"]}
    out = redact.redact(d)
    assert out["env"]["AWS_ACCESS_KEY_ID"] == redact.REDACTED
    assert out["env"]["ok"] == 1
    assert out["list"][0]["password"] == redact.REDACTED
    assert "abc.def.ghi" not in out["list"][1]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
