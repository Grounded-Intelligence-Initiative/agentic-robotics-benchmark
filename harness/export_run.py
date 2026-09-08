"""``ale-export``: pack ALE engine episode records into an ``ale-engine-run/v1`` bundle.

Design record: ``docs/dev/20260902-engine-run-submission.md`` §3.1 (bundle layout) and §3.2
(this tool). The exporter is deliberately thin: it never scores, never reads ``verify/`` or the
task source, and never imports the engine. It packs what ``ale run`` / ``ale validate`` wrote
under ``runs/<run_id>/<episode>/`` into the tar -> zstd -> age envelope the website decrypts, and
writes a ``run_manifest.json`` whose every field is *derived* from the engine's own records
(agent tuple from the locks, identities from ``lock.task``), so nothing in the manifest can be
typed by hand and later disagree with the episode files the server re-checks.

    ale-export RUN_DIR_OR_EPISODE_DIR... --out <name>.ale-engine-run.tar.zst.age \\
               [--public-key age1...|env:VAR|path] [--challenge-file challenge.json] \\
               [--harness claude-code] [--allow-builtin] [--plaintext]

Exit codes: 0 ok; 2 usage error or a refusal the design mandates (mixed agents, builtin
harness without ``--allow-builtin``, unusable path or challenge file); 1 a record on disk is
malformed or a required member is over the size cap.
"""

from __future__ import annotations

import argparse
import json
import re
import sqlite3
import sys
import tomllib
import uuid
from collections.abc import Sequence
from dataclasses import dataclass, field
from datetime import UTC, datetime
from importlib import metadata
from pathlib import Path
from typing import Any

from harness import bundle_crypto, redact
from harness.trajectory import sha256_hex

MANIFEST_VERSION = "ale-engine-run/v1"
EXPORTER_NAME = "ale-robotics-export"
BENCHMARK_NAME = "ale-robotics"
PROJECT_NAME = "ale-robotics"

ORIGIN_CHALLENGE = "challenge_issued"
ORIGIN_OFFLINE = "self_hosted_offline"
ORIGIN_SMOKE = "smoke"

# Model-free authoring harnesses (design §2). Their runs are smoke tests, never leaderboard
# trials, so packing them needs an explicit opt-in and is stamped ``submission_origin: smoke``.
BUILTIN_HARNESSES = frozenset({"oracle", "nop", "scripted"})
AGENT_KEYS = ("harness", "model", "version", "family")

# ``verification.json`` is required only when ``result.status == completed``; when the engine
# wrote one for a failed episode it is still packed (the server tolerates it, §5.1).
REQUIRED_MEMBERS = ("lock.json", "result.json", "verification.json")
# Opaque on the server: hashed, never parsed there. Subject to the omission policy below.
OPTIONAL_MEMBERS = ("trajectory.json", "trace.execution.jsonl", "artifacts/snapshot.json")
# Design §3.2: an optional member over this is omitted (listed in the receipt); a required one
# over it aborts the export.
MEMBER_CAP_BYTES = 32 * 1024 * 1024

# What ``lock.task.source.path`` becomes in the bundle: the local checkout path says nothing the
# server can use and may reveal a username or machine layout.
LOCAL_SOURCE_PATH = "<local>"

# The saved ``POST /api/v1/challenges`` response must at least carry these to be verifiable.
CHALLENGE_KEYS = ("submission_id", "challenge_token", "nonce")

# Mirrors the server's ``episode_id`` schema. The id becomes a tar path segment
# (``episodes/<id>/...``); anything else would be a bundle the server is guaranteed to reject
# (or a traversal-shaped member), so it is refused here with a local diagnostic.
EPISODE_ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,200}$")

# Ledger ``status`` values that mean "the engine had not finished this episode" (design §2:
# a Ctrl-C leaves a non-terminal row and no result.json). Every other status is terminal and
# the engine writes result.json before recording it, so a terminal row without result.json is
# a damaged run dir, not an interrupted episode.
NON_TERMINAL_STATUSES = frozenset({"queued", "running", "interrupted"})

EXIT_OK, EXIT_ERROR, EXIT_REFUSED = 0, 1, 2


class ExportRefused(Exception):
    """Usage error, or a refusal the design mandates (exit 2)."""


class ExportError(Exception):
    """A record on disk is malformed or cannot be packed within policy (exit 1)."""


# --------------------------------------------------------------------------- #
# discovery
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class EpisodeDir:
    """An episode directory holding ``lock.json`` + ``result.json``.

    ``ledger_episode_id`` is set when the directory was found through a run's ledger; the
    ``result.json`` inside must agree with it (a mismatch means the run dir was tampered with
    or reassembled by hand).
    """

    path: Path
    ledger_episode_id: str | None = None


@dataclass(frozen=True)
class InterruptedEpisode:
    """A ledger row whose directory never received a ``result.json`` (design §2: Ctrl-C)."""

    episode_id: str
    task_name: str
    variant: str


@dataclass
class LoadedEpisode:
    """Parsed engine records for one episode plus the raw bytes of its opaque members."""

    episode_id: str
    lock: dict[str, Any]
    result: dict[str, Any]
    verification: dict[str, Any] | None
    raw_optional: dict[str, bytes] = field(default_factory=dict)
    #: Optional members skipped on their on-disk size alone (``name -> bytes``), before any
    #: read; ``pack_episode`` lists them under ``omitted_members``.
    oversized_optional: dict[str, int] = field(default_factory=dict)

    @property
    def agent(self) -> dict[str, Any]:
        """The ``lock.agent`` tuple the manifest reports (design §3.1)."""
        return {k: self.lock["agent"].get(k) for k in AGENT_KEYS}


def discover(paths: Sequence[Path]) -> list[EpisodeDir | InterruptedEpisode]:
    """Resolve each CLI path to episode dirs, enumerating run dirs through their ledger."""
    items: list[EpisodeDir | InterruptedEpisode] = []
    for raw in paths:
        p = Path(raw)
        if (p / "lock.json").is_file() and (p / "result.json").is_file():
            items.append(EpisodeDir(path=p))
        elif (p / "ledger.db").is_file():
            items.extend(enumerate_run(p))
        else:
            raise ExportRefused(
                f"{p}: neither an episode dir (lock.json + result.json) nor a run dir (ledger.db)")
    return items


def enumerate_run(run_dir: Path) -> list[EpisodeDir | InterruptedEpisode]:
    """List a run's episodes from ``ledger.db`` opened strictly read-only.

    The engine's own ``Ledger`` class reconciles and interrupts rows on open, so it is never
    used here: a ``file:...?mode=ro`` URI guarantees no row is written. (On a WAL-mode ledger
    SQLite may still create the empty ``-wal`` / ``-shm`` side files it needs for a shared
    read, exactly as the engine does on every open; ``immutable=1`` would avoid even that but
    silently ignores an un-checkpointed WAL left by a crashed run.) ``validation.json`` is
    ignored (design §3.2).
    """
    db_path = (run_dir / "ledger.db").resolve()
    uri = db_path.as_uri() + "?mode=ro"
    try:
        con = sqlite3.connect(uri, uri=True)
    except sqlite3.Error as exc:
        raise ExportError(f"cannot open {db_path} read-only: {exc}") from exc
    try:
        rows = con.execute(
            "SELECT episode_id, task_id, variant, episode_path, status FROM episodes "
            "ORDER BY rowid"
        ).fetchall()
    except sqlite3.Error as exc:
        raise ExportError(f"{db_path}: {exc}") from exc
    finally:
        con.close()
    if not rows:
        raise ExportRefused(f"{run_dir}: ledger.db lists no episodes")

    items: list[EpisodeDir | InterruptedEpisode] = []
    for episode_id, task_id, variant, episode_path, status in rows:
        ep_dir = run_dir / episode_path
        if (ep_dir / "result.json").is_file():
            items.append(EpisodeDir(path=ep_dir, ledger_episode_id=episode_id))
        elif status in NON_TERMINAL_STATUSES:
            items.append(InterruptedEpisode(episode_id=episode_id, task_name=task_id,
                                            variant=variant or "base"))
        else:
            raise ExportError(
                f"{ep_dir}: ledger says status={status!r} but result.json is missing "
                "(damaged run dir?); only a non-terminal row may lack one")
    return items


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #
def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ExportError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ExportError(f"{path}: not valid JSON ({exc})") from exc
    if not isinstance(data, dict):
        raise ExportError(f"{path}: expected a JSON object")
    return data


def _require(record: dict[str, Any], dotted: str, where: Path) -> Any:
    """Fetch a dotted key the engine always writes; its absence means a foreign file."""
    cur: Any = record
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise ExportError(f"{where}: missing {dotted!r} (not an ALE engine record?)")
        cur = cur[part]
    return cur


def load_lock(ref: EpisodeDir) -> dict[str, Any]:
    """Read and shape-check ``lock.json`` alone.

    Split out so ``--harness`` can be applied on ``lock.agent.harness`` before anything else
    in the directory is read: a defective episode of a harness that is being filtered out
    must not abort the export of the one that is wanted.
    """
    lock = _read_json_object(ref.path / "lock.json")
    for dotted in ("agent.harness", "task.name", "task.variant", "task.spec_hash",
                   "task.content_digest", "framework.commit", "framework.version",
                   "ale_verify.content_hash"):
        _require(lock, dotted, ref.path / "lock.json")
    return lock


def load_episode(ref: EpisodeDir, *, lock: dict[str, Any] | None = None,
                 cap_bytes: int = MEMBER_CAP_BYTES) -> LoadedEpisode:
    """Read one episode's records and validate the invariants the bundle relies on.

    Optional members whose on-disk size already exceeds ``cap_bytes`` are never read (the
    cap exists for runaway trajectories; materialising one to decide to drop it defeats
    it) — they are recorded in ``oversized_optional`` and listed as omitted.
    """
    if lock is None:
        lock = load_lock(ref)
    result = _read_json_object(ref.path / "result.json")
    episode_id = _require(result, "episode_id", ref.path / "result.json")
    status = _require(result, "status", ref.path / "result.json")
    if not isinstance(episode_id, str) or not episode_id:
        raise ExportError(f"{ref.path}/result.json: episode_id must be a non-empty string")
    if episode_id in (".", "..") or not EPISODE_ID_RE.match(episode_id):
        raise ExportError(
            f"{ref.path}/result.json: episode_id {episode_id!r} is not a valid id "
            f"({EPISODE_ID_RE.pattern}, not '.' or '..'); it becomes a bundle path segment "
            "and the server would reject it")
    if ref.ledger_episode_id is not None and episode_id != ref.ledger_episode_id:
        raise ExportError(
            f"{ref.path}: result.json says episode_id={episode_id!r} but the ledger row is "
            f"{ref.ledger_episode_id!r}")

    ver_path = ref.path / "verification.json"
    verification = _read_json_object(ver_path) if ver_path.is_file() else None
    if status == "completed" and verification is None:
        raise ExportError(
            f"{ref.path}: result.status is 'completed' but verification.json is missing; the "
            "server rejects such an episode, so it is not packed")

    raw_optional: dict[str, bytes] = {}
    oversized: dict[str, int] = {}
    for name in OPTIONAL_MEMBERS:
        path = ref.path / name
        if not path.is_file():
            continue
        size = path.stat().st_size
        if size > cap_bytes:
            oversized[name] = size
            continue
        raw_optional[name] = path.read_bytes()
    return LoadedEpisode(episode_id=episode_id, lock=lock, result=result,
                         verification=verification, raw_optional=raw_optional,
                         oversized_optional=oversized)


# --------------------------------------------------------------------------- #
# agent selection
# --------------------------------------------------------------------------- #
def select_agent(episodes: list[LoadedEpisode], harness: str | None, allow_builtin: bool
                 ) -> tuple[list[LoadedEpisode], dict[str, Any], bool]:
    """Apply ``--harness``, require one agent tuple, gate builtin harnesses.

    Returns ``(kept_episodes, agent, is_builtin)``. The agent tuple is never typed by hand: a
    bundle whose manifest disagreed with its locks would be rejected server-side anyway.
    """
    if harness:
        episodes = [e for e in episodes if e.agent["harness"] == harness]
        if not episodes:
            raise ExportRefused(f"no episode was run by harness {harness!r}")
    if not episodes:
        raise ExportRefused("nothing to export: no episode with lock.json + result.json")

    distinct = {json.dumps(e.agent, sort_keys=True) for e in episodes}
    if len(distinct) > 1:
        seen = ", ".join(sorted(f"{e['harness']}/{e['model'] or '-'}/{e['version']}"
                                for e in map(json.loads, distinct)))
        raise ExportRefused(
            f"mixed agents across episodes ({seen}); a bundle carries exactly one agent — "
            "pass --harness to select one")

    agent = episodes[0].agent
    is_builtin = agent["harness"] in BUILTIN_HARNESSES
    if is_builtin and not allow_builtin:
        raise ExportRefused(
            f"harness {agent['harness']!r} is a model-free authoring harness; pass "
            "--allow-builtin to export it as a smoke bundle (the server never scores it)")
    return episodes, agent, is_builtin


# --------------------------------------------------------------------------- #
# member assembly
# --------------------------------------------------------------------------- #
def _json_bytes(obj: Any) -> bytes:
    """Stored form of a JSON member: deterministic so re-exports of a run hash identically."""
    return json.dumps(obj, indent=2, ensure_ascii=False, sort_keys=True).encode("utf-8")


def _redact_lock(lock: dict[str, Any]) -> dict[str, Any]:
    lock = redact.redact_values(lock)
    source = lock["task"].get("source")
    if isinstance(source, dict) and "path" in source:
        source["path"] = LOCAL_SOURCE_PATH
    return lock


def _redact_jsonl(raw: bytes) -> bytes:
    """Redact a JSONL trace line by line so a value pattern can never eat a line's structure."""
    out: list[str] = []
    for line in raw.decode("utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            out.append(redact.redact_str(line))
            continue
        out.append(json.dumps(redact.redact_values(obj), ensure_ascii=False,
                              separators=(",", ":"), sort_keys=True))
    return ("\n".join(out) + "\n").encode("utf-8") if out else b""


def _redact_optional(name: str, raw: bytes, episode_id: str) -> bytes:
    if name.endswith(".jsonl"):
        return _redact_jsonl(raw)
    try:
        obj = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ExportError(f"{episode_id}/{name}: not valid JSON ({exc})") from exc
    return _json_bytes(redact.redact_values(obj))


def pack_episode(ep: LoadedEpisode, *, cap_bytes: int, omitted: list[dict[str, Any]]
                 ) -> tuple[dict[str, bytes], dict[str, Any]]:
    """Redact, size-check and hash one episode's members.

    Returns ``(members, manifest_entry)`` where ``members`` maps archive paths to stored bytes
    and ``manifest_entry.members`` holds the sha256 of exactly those bytes — computed *after*
    redaction and *after* the omission policy, so the manifest can only describe what shipped.
    The omission policy has two gates: the on-disk size check in ``load_episode`` (recorded
    in ``oversized_optional``) and the stored-size check below, which also covers required
    members and the case where redaction/re-serialisation grows a member past the cap.
    """
    for name, size in ep.oversized_optional.items():
        omitted.append({"episode_id": ep.episode_id, "member": name,
                        "bytes": size, "cap_bytes": cap_bytes})
    stored: dict[str, bytes] = {
        "lock.json": _json_bytes(_redact_lock(ep.lock)),
        "result.json": _json_bytes(redact.redact_values(ep.result)),
    }
    if ep.verification is not None:
        stored["verification.json"] = _json_bytes(redact.redact_values(ep.verification))
    for name, raw in ep.raw_optional.items():
        stored[name] = _redact_optional(name, raw, ep.episode_id)

    members: dict[str, bytes] = {}
    hashes: dict[str, str] = {}
    for name in REQUIRED_MEMBERS + OPTIONAL_MEMBERS:
        if name not in stored:
            continue
        data = stored[name]
        if len(data) > cap_bytes:
            if name in OPTIONAL_MEMBERS:
                omitted.append({"episode_id": ep.episode_id, "member": name,
                                "bytes": len(data), "cap_bytes": cap_bytes})
                continue
            raise ExportError(
                f"{ep.episode_id}/{name} is {len(data)} bytes, over the {cap_bytes}-byte cap "
                "for a required member; the export cannot proceed")
        members[f"episodes/{ep.episode_id}/{name}"] = data
        hashes[name] = sha256_hex(data)

    task = ep.lock["task"]
    entry: dict[str, Any] = {
        "episode_id": ep.episode_id,
        "task_name": task["name"],
        "variant": task["variant"],
        "spec_hash": task["spec_hash"],
        "content_digest": task["content_digest"],
        "status": ep.result["status"],
    }
    # Only ``completed`` carries rewards (design §2); mirror the engine rather than invent {}.
    if ep.result.get("rewards") is not None:
        entry["rewards"] = ep.result["rewards"]
    entry["metrics"] = ep.result.get("metrics") or {}
    entry["members"] = hashes
    return members, entry


def _interrupted_entry(item: InterruptedEpisode) -> dict[str, Any]:
    return {"episode_id": item.episode_id, "task_name": item.task_name,
            "variant": item.variant, "status": "interrupted", "members": {}}


def _engine_block(episodes: list[LoadedEpisode], warnings: list[str]) -> dict[str, Any]:
    """``engine`` from the first lock; differing locks are reported, not refused, because the
    server checks ``lock.framework.commit`` per episode anyway (design §5.1)."""
    blocks = [{"commit": e.lock["framework"]["commit"],
               "version": e.lock["framework"]["version"],
               "ale_verify_content_hash": e.lock["ale_verify"]["content_hash"]}
              for e in episodes]
    if len({json.dumps(b, sort_keys=True) for b in blocks}) > 1:
        warnings.append("engine identity differs across episodes; run_manifest.engine reports "
                        "the first episode's")
    return blocks[0]


def _check_unique(ids: list[str]) -> None:
    dups = sorted({i for i in ids if ids.count(i) > 1})
    if dups:
        raise ExportRefused(f"episode given more than once: {', '.join(dups)}")


def _check_unique_dirs(refs: list[EpisodeDir]) -> None:
    """The same directory given twice is a refusal even when ``--harness`` filters it out."""
    seen: dict[Path, int] = {}
    for ref in refs:
        key = ref.path.resolve()
        seen[key] = seen.get(key, 0) + 1
    dups = sorted(str(p) for p, n in seen.items() if n > 1)
    if dups:
        raise ExportRefused(f"episode given more than once: {', '.join(dups)}")


# --------------------------------------------------------------------------- #
# versions + challenge
# --------------------------------------------------------------------------- #
def project_version() -> str:
    """This repo's ``pyproject.toml`` version (falls back to installed metadata)."""
    pyproject = Path(__file__).resolve().parents[1] / "pyproject.toml"
    if pyproject.is_file():
        return str(tomllib.loads(pyproject.read_text(encoding="utf-8"))["project"]["version"])
    return metadata.version(PROJECT_NAME)


def benchmark_release(version: str) -> str:
    """``"0.5.0"`` -> ``"v0.5"``: the release name the registry and website use."""
    return "v" + ".".join(version.split(".")[:2])


def load_challenge(path: Path) -> dict[str, Any]:
    """The saved ``POST /api/v1/challenges`` response, embedded verbatim (design §3.1)."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ExportRefused(f"--challenge-file {path}: {exc}") from exc
    if not isinstance(data, dict) or any(
            not isinstance(data.get(k), str) or not data[k] for k in CHALLENGE_KEYS):
        raise ExportRefused(
            f"--challenge-file {path}: expected the saved POST /api/v1/challenges response "
            f"with non-empty {', '.join(CHALLENGE_KEYS)}")
    return data


# --------------------------------------------------------------------------- #
# export
# --------------------------------------------------------------------------- #
def export(paths: Sequence[Path], out: Path, *, public_key: str | None,
           challenge: dict[str, Any] | None = None, harness: str | None = None,
           allow_builtin: bool = False, plaintext: bool = False,
           cap_bytes: int = MEMBER_CAP_BYTES) -> dict[str, Any]:
    """Discover -> filter on lock.agent.harness -> load -> select agent -> pack -> manifest
    -> envelope. Returns the receipt.

    ``--harness`` is applied after reading only ``lock.json`` and before any other check or
    member, so a defective episode of another harness cannot abort the wanted export.
    """
    items = discover(paths)
    _check_unique_dirs([i for i in items if isinstance(i, EpisodeDir)])
    # ``loaded`` maps the EpisodeDir items kept after the harness filter to their records,
    # keyed by position in ``items`` so the manifest keeps discovery (ledger) order with
    # interrupted rows interleaved.
    loaded: dict[int, LoadedEpisode] = {}
    for index, item in enumerate(items):
        if not isinstance(item, EpisodeDir):
            continue
        lock = load_lock(item)
        if harness and lock["agent"].get("harness") != harness:
            continue
        loaded[index] = load_episode(item, lock=lock, cap_bytes=cap_bytes)
    interrupted = [i for i in items if isinstance(i, InterruptedEpisode)]
    _check_unique([e.episode_id for e in loaded.values()] + [i.episode_id for i in interrupted])

    kept, agent, is_builtin = select_agent(list(loaded.values()), harness, allow_builtin)
    warnings: list[str] = []
    if harness and interrupted:
        warnings.append("interrupted ledger rows carry no lock, so they cannot be attributed "
                        "to --harness; all of them are listed")

    omitted: list[dict[str, Any]] = []
    members: dict[str, bytes] = {}
    entries: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        if isinstance(item, InterruptedEpisode):
            entries.append(_interrupted_entry(item))
            continue
        ep = loaded.get(index)
        if ep is None:
            continue
        ep_members, entry = pack_episode(ep, cap_bytes=cap_bytes, omitted=omitted)
        members.update(ep_members)
        entries.append(entry)

    if is_builtin:
        origin = ORIGIN_SMOKE
    elif challenge:
        origin = ORIGIN_CHALLENGE
    else:
        origin = ORIGIN_OFFLINE
    version = project_version()
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "exporter": {"name": EXPORTER_NAME, "version": version},
        "benchmark": {"name": BENCHMARK_NAME, "version": benchmark_release(version)},
        # Offline bundles need an id the server can key on; a random UUID cannot collide
        # with the server-issued ids embedded via a challenge.
        "submission_id": challenge["submission_id"] if challenge else str(uuid.uuid4()),
        "submission_origin": origin,
        "challenge": challenge,
        "agent": agent,
        "engine": _engine_block(kept, warnings),
        "created_at": datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "episodes": entries,
    }
    members["run_manifest.json"] = _json_bytes(manifest)

    return bundle_crypto.encrypt_and_write(
        members, Path(out), public_key, bundle_schema=MANIFEST_VERSION,
        challenge_bound=bool(challenge), plaintext=plaintext,
        extra_receipt={
            "submission_id": manifest["submission_id"],
            "submission_origin": origin,
            "agent": agent,
            "episodes": [{"episode_id": e["episode_id"], "status": e["status"]} for e in entries],
            "omitted_members": omitted,
            "warnings": warnings,
        })


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="ale-export",
        description="Pack `ale run` episode records into an ale-engine-run/v1 leaderboard bundle.")
    ap.add_argument("paths", nargs="+", type=Path, metavar="RUN_DIR_OR_EPISODE_DIR",
                    help="a run dir (enumerated via its ledger.db, read-only) or an episode dir")
    ap.add_argument("--out", required=True, type=Path,
                    help="output bundle path, e.g. <name>.ale-engine-run.tar.zst.age")
    ap.add_argument("--public-key", default="",
                    help="age recipient: age1... | env:VAR | path (default: "
                         f"${bundle_crypto.PUBLIC_KEY_ENV})")
    ap.add_argument("--challenge-file", type=Path,
                    help="saved POST /api/v1/challenges response; embedded verbatim")
    ap.add_argument("--harness",
                    help="keep only episodes run by this harness (a bundle carries one agent)")
    ap.add_argument("--allow-builtin", action="store_true",
                    help="permit oracle/nop/scripted episodes; stamps submission_origin=smoke")
    ap.add_argument("--plaintext", action="store_true",
                    help="write the bare tar.zst without age (tests / ingest integration only)")
    return ap


def _print_summary(receipt: dict[str, Any]) -> None:
    statuses = [e["status"] for e in receipt["episodes"]]
    packed = sum(1 for s in statuses if s != "interrupted")
    agent = receipt["agent"]
    size = receipt["encrypted_bytes"] if receipt["encryption"] == "age" else receipt["compressed_bytes"]
    print(f"ale-export: wrote {receipt['output']} ({size} bytes, {receipt['encryption']})")
    print(f"  receipt: {bundle_crypto.receipt_path(Path(receipt['output']))}")
    print(f"  submission_id: {receipt['submission_id']}  origin: {receipt['submission_origin']}")
    print(f"  agent: {agent['harness']} model={agent['model'] or '-'} version={agent['version']}")
    print(f"  episodes: {packed} packed, {len(statuses) - packed} interrupted; "
          f"members: {len(receipt['members'])}; omitted: {len(receipt['omitted_members'])}")
    for w in receipt["warnings"]:
        print(f"  warning: {w}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.plaintext:
            public_key = None
        else:
            try:
                public_key = bundle_crypto.resolve_public_key(args.public_key)
            except ValueError as exc:
                raise ExportRefused(str(exc)) from exc
        challenge = load_challenge(args.challenge_file) if args.challenge_file else None
        receipt = export(args.paths, args.out, public_key=public_key, challenge=challenge,
                         harness=args.harness, allow_builtin=args.allow_builtin,
                         plaintext=args.plaintext)
    except ExportRefused as exc:
        print(f"ale-export: refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED
    except (ExportError, OSError, ValueError) as exc:
        print(f"ale-export: error: {exc}", file=sys.stderr)
        return EXIT_ERROR
    _print_summary(receipt)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
