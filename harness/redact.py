"""Secret redaction for exported bundle members.

Everything that could end up in a leaderboard bundle passes through here first. Provider
credentials live in the ALE engine's ``.env`` on the host and are used by the engine's LLM
gateway; the tested agent never holds them. But the records the engine writes (``lock.json``,
``result.json``, ``verification.json``, ``trajectory.json``, ``artifacts/snapshot.json``,
``trace.execution.jsonl``) can still echo key-shaped values — a key pasted into a prompt, a
presigned URL or an Authorization header in a captured command's stderr. None of that may
leave the host in a bundle.

Two policies live here, and only one is operative:

* ``redact_values`` / ``redact_str`` — VALUE-pattern redaction, applied by ``export_run`` to
  every engine-written member (design record ``docs/dev/20260902-engine-run-submission.md``
  §3.2). Keys are never matched: ``lock.gateway.limits`` carries budget fields such as
  ``max_input_tokens`` / ``max_total_tokens``, and a key-name rule on the substring ``token``
  turns them into the redaction marker (verified on a real lock). The server does not
  currently validate ``gateway.limits``, so the harm would be silent corruption of the run's
  provenance, not a failed check — which is exactly why it must not happen.
* ``redact`` / ``redact_env`` — the legacy key-name + value-pattern helpers. They have no
  caller in the exporter and are retained for API compatibility and their tests only.

Redaction is conservative, not a guarantee: defense in depth for any path that captures
env / config / logs.
"""

from __future__ import annotations

import re
from typing import Any

REDACTED = "«redacted»"

# Env/JSON keys whose VALUES are always secret (case-insensitive exact or
# substring match on the key).
_SECRET_KEY_PARTS = (
    "aws_access_key_id", "aws_secret_access_key", "aws_session_token",
    "aws_security_token", "authorization", "x-amz-security-token",
    "x-api-key", "x-goog-api-key",  # Anthropic / Google raw-key headers
    "secret", "password", "passwd", "token", "api_key", "apikey",
    "private_key", "client_secret", "set-cookie", "cookie",
)
# ...but these token-ish keys are NOT secret (avoid over-redacting metrics).
_SECRET_KEY_ALLOW = (
    "input_tokens", "output_tokens", "cache_read_tokens",
    "cache_creation_tokens", "cache_read_input_tokens",
    "cache_creation_input_tokens", "num_turns", "tokens_approx",
    "max_output_tokens", "max_tokens", "total_tokens", "prompt_tokens",
    "completion_tokens", "token_count", "n_tokens",
    # The challenge token is a single-use, expiring, PUBLIC-safe value the runner MUST
    # ship in the bundle so the leaderboard can verify its signature (it is not a
    # credential — the site issued it and only ever holds the public key). Redacting it
    # (substring "token") shipped "«redacted»" and false-failed challenge_sig on every
    # online submit. Keep the exact key allowlisted so it survives verbatim.
    "challenge_token",
)

# Value patterns that look like credentials regardless of key.
_VALUE_PATTERNS = (
    re.compile(r"\bASIA[0-9A-Z]{16}\b"),                 # AWS temp access key id
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),                 # AWS long-term access key id
    re.compile(r"\bAWS4-HMAC-SHA256\b[^\n]*"),           # SigV4 auth header
    re.compile(r"X-Amz-Signature=[0-9a-fA-F]+"),         # presigned URL sig
    re.compile(r"X-Amz-Security-Token=[^&\s]+"),
    re.compile(r"\bBearer\s+[A-Za-z0-9._\-]+"),          # bearer tokens
    # Bare LLM provider API keys (OpenAI/Anthropic/OpenRouter sk-..., sk-ant-...,
    # sk-or-...) and age secret keys — the gateway log ships full LLM I/O into the
    # bundle, so a key echoed in a prompt/response/error string must be scrubbed.
    re.compile(r"\bsk-[A-Za-z0-9._\-]{12,}"),
    re.compile(r"AGE-SECRET-KEY-1[0-9A-Z]+"),
)


def _key_is_secret(key: str) -> bool:
    k = key.lower()
    if k in _SECRET_KEY_ALLOW:
        return False
    return any(part in k for part in _SECRET_KEY_PARTS)


def redact_str(value: str) -> str:
    out = value
    for pat in _VALUE_PATTERNS:
        out = pat.sub(REDACTED, out)
    return out


def redact(obj: Any) -> Any:
    """Recursively redact secrets from a JSON-like structure (pure, copies)."""
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if isinstance(k, str) and _key_is_secret(k):
                out[k] = REDACTED
            else:
                out[k] = redact(v)
        return out
    if isinstance(obj, list):
        return [redact(v) for v in obj]
    if isinstance(obj, tuple):
        return tuple(redact(v) for v in obj)
    if isinstance(obj, str):
        return redact_str(obj)
    return obj


def redact_env(env: dict) -> dict:
    """Redact a container env dict (keeps keys, masks secret values)."""
    return {k: (REDACTED if _key_is_secret(k) else v) for k, v in (env or {}).items()}


def redact_values(obj: Any) -> Any:
    """Value-pattern-only redaction: ``redact_str`` over every string leaf, keys untouched.

    Used for records the ALE engine wrote (lock / result / verification / trajectory /
    snapshot). The key-name rule in ``redact`` is wrong for them: ``lock.gateway.limits``
    carries ``max_input_tokens`` / ``max_total_tokens`` style fields (substring "token") that
    are budgets, not credentials, and masking them silently corrupts the run's provenance
    (the server stores the lock; it does not currently validate ``gateway.limits``, so the
    damage would go unnoticed). The engine never places credentials under a well-named key,
    so only the credential-shaped VALUE patterns are worth scrubbing.
    """
    if isinstance(obj, dict):
        return {k: redact_values(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [redact_values(v) for v in obj]
    if isinstance(obj, tuple):
        return tuple(redact_values(v) for v in obj)
    if isinstance(obj, str):
        return redact_str(obj)
    return obj
