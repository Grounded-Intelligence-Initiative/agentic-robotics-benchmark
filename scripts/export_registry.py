#!/usr/bin/env python3
"""Export the website task registry (``ale-robotics-task-registry/v2``) from the engine loader.

Design: docs/dev/20260902-engine-run-submission.md §4. The one rule that shapes this script
is that *task identity comes from the engine*: ``spec_hash`` and ``content_digest`` are read
off ``ale.run.tasksets.manifest.load_tasks`` and never re-implemented here, so the server
compares a run's ``lock.json`` against exactly the numbers the engine would compute for the
committed tree. Everything else is assembled from files the task folder already carries
(``task.yaml`` metadata, ``instruction.md``, ``verify/anchor.json``,
``verify/grader_config.json``) plus git provenance.

The loader lives in the pinned engine's venv (``vendor/ale``), not in this repo's. When
``ale`` is not importable the script re-executes itself through ``uv run`` in that venv, so
``uv run python scripts/export_registry.py ...`` works from either environment.

Usage (from the repository root)::

    cd vendor/ale && VIRTUAL_ENV= uv run python ../../scripts/export_registry.py \\
        --tasks ../../tasks --history ../../identity-history.json \\
        --carry-over <website>/content/tasks/registry.json --out registry.json

``--allow-dirty`` is for throwaway exports (CI cross-checks, tests): the document is stamped
``provenance.source_commit = "<sha>-dirty"`` plus ``provenance.dirty: true`` so nobody can
mistake it for a release export, and the default identity history is never written — a
dirty tree would record an identity no commit reproduces.
"""

from __future__ import annotations

import argparse
import datetime as dt
import importlib.metadata
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
ENGINE_DIR = REPO_ROOT / "vendor" / "ale"
DEFAULT_TASKS = REPO_ROOT / "tasks"
DEFAULT_HISTORY = REPO_ROOT / "identity-history.json"

SCHEMA_VERSION = "ale-robotics-task-registry/v2"
#: The server reads the cap from ``metadata_json``; this default mirrors the kit's
#: ``robotics_grader.scoring`` default so a task without ``scoring_cap`` scores identically
#: on both sides.
DEFAULT_SCORE_CAP = 1.5
DEFAULT_FULL_AT = 1.0
DEFAULT_RELEASE_NAME = "Agentic Robotics Benchmark"
DEFAULT_RELEASE_VERSION = "v0.5"
#: Only the base variant is registered (§4): variant runs are unknown identities.
REGISTERED_VARIANT = "base"
#: Guard against the re-exec loop when the engine venv itself lacks ``ale``.
_REEXEC_FLAG = "ALE_EXPORT_REGISTRY_REEXEC"

#: A real h1 line only: ``#`` + horizontal whitespace + text. ``\s`` would also match a
#: newline, turning a bare ``#`` followed by the next line into a "title".
_HEADING = re.compile(r"^#[ \t]+(.+?)[ \t]*$", re.MULTILINE)
#: Fenced code blocks are stripped before the search: ``# install deps`` inside a ```bash
#: block is a shell comment, not a heading.
_FENCED_BLOCK = re.compile(r"^[ \t]*(```|~~~).*?^[ \t]*\1[ \t]*$", re.MULTILINE | re.DOTALL)
_METRIC_DIRECTION = {"higher": "higher_is_better", "lower": "lower_is_better"}


class RegistryError(RuntimeError):
    """A task cannot be registered as it stands; the message says what to fix."""


# --------------------------------------------------------------------------- #
# engine-independent view of a loaded task
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class LoadedTask:
    """The slice of an engine ``ManifestTask`` the registry needs.

    Kept engine-free so the field assembly can be unit-tested without the engine venv;
    ``from_manifest_task`` is the only place that touches engine objects.
    """

    slug: str
    variant: str
    root: Path
    spec_hash: str
    content_digest: str
    metadata: dict[str, Any] = field(default_factory=dict)
    resources: dict[str, Any] = field(default_factory=dict)
    timeouts: dict[str, Any] = field(default_factory=dict)


def from_manifest_task(task: Any) -> LoadedTask:
    """Project an ``ale.run.tasksets.manifest.ManifestTask`` onto ``LoadedTask``."""
    spec = task.spec
    return LoadedTask(
        slug=str(spec.name),
        variant=str(spec.variant),
        root=Path(task.folder.root),
        spec_hash=spec.spec_hash,
        content_digest=task.folder.content_digest,
        metadata=dict(spec.metadata),
        resources=spec.resources.model_dump(mode="json"),
        timeouts=spec.timeouts.model_dump(mode="json"),
    )


def load_with_engine(tasks_path: Path) -> list[LoadedTask]:
    """Run the engine's own loader; the import is deferred so unit tests never need it."""
    from ale.run.tasksets.manifest import load_tasks

    return [from_manifest_task(task) for task in load_tasks(tasks_path)]


# --------------------------------------------------------------------------- #
# per-task field sources (§4)
# --------------------------------------------------------------------------- #
def read_title(task_dir: Path, slug: str) -> str:
    """First level-1 heading of ``instruction.md``, else the title-cased slug.

    Only ``# `` counts: the agent-facing prompt usually opens with prose and ``## Goal``,
    and a section heading would be a misleading task title.
    """
    instruction = task_dir / "instruction.md"
    if instruction.is_file():
        prose = _FENCED_BLOCK.sub("", instruction.read_text(encoding="utf-8"))
        match = _HEADING.search(prose)
        if match:
            return match.group(1)
    return slug.replace("_", " ").replace("-", " ").title()


def read_score_cap(task_dir: Path) -> float:
    """``verify/grader_config.json`` ``scoring_cap``; the file and the key are both optional."""
    config = task_dir / "verify" / "grader_config.json"
    if not config.is_file():
        return DEFAULT_SCORE_CAP
    loaded = json.loads(config.read_text(encoding="utf-8"))
    cap = loaded.get("scoring_cap", DEFAULT_SCORE_CAP)
    if not isinstance(cap, (int, float)) or isinstance(cap, bool) or cap <= 0:
        raise RegistryError(f"{config}: scoring_cap must be a positive number, got {cap!r}")
    return float(cap)


def read_anchor(
    task_dir: Path, score_cap: float, metric_direction: str | None = None
) -> dict[str, Any]:
    """``verify/anchor.json`` → the four ``anchor*`` registry fields.

    ``full_at`` defaults to 1.0 and must lie in ``(0, cap]`` and ``value`` must be positive —
    the same rules the kit applies at grading time, so a registry that disagrees with the
    grader is impossible to export. A ``null`` value is allowed: it is the authoring state
    ("not admissible yet") and the server simply cannot score such a task. When the anchor
    declares a ``direction`` it must agree with ``task.yaml``'s ``metadata.metric.direction``
    (``metric_direction``): the grader reads the former, the server recompute the latter.
    """
    path = task_dir / "verify" / "anchor.json"
    if not path.is_file():
        raise RegistryError(f"{task_dir.name}: missing verify/anchor.json")
    anchor = json.loads(path.read_text(encoding="utf-8"))
    value = anchor.get("value")
    if value is not None and (not isinstance(value, (int, float)) or isinstance(value, bool)):
        raise RegistryError(f"{path}: value must be a number or null, got {value!r}")
    if value is not None and value <= 0:
        raise RegistryError(
            f"{path}: anchor value must be > 0, got {value!r} (the kit refuses a non-positive "
            "anchor: the ratio would invert)"
        )
    declared = anchor.get("direction")
    if declared is not None and metric_direction is not None and declared != metric_direction:
        raise RegistryError(
            f"{path}: direction {declared!r} contradicts task.yaml metadata.metric.direction "
            f"{metric_direction!r}; the grader and the server recompute would disagree"
        )
    full_at = anchor.get("full_at", DEFAULT_FULL_AT)
    if (
        not isinstance(full_at, (int, float))
        or isinstance(full_at, bool)
        or not 0 < full_at <= score_cap
    ):
        raise RegistryError(
            f"{path}: full_at must lie in (0, {score_cap}] (score_cap), got {full_at!r}"
        )
    return {
        "anchor": None if value is None else float(value),
        "anchor_full_at": float(full_at),
        "anchor_source": anchor.get("source"),
        "anchor_image_identity": anchor.get("image_identity"),
    }


def metric_fields(metadata: dict[str, Any], slug: str) -> dict[str, Any]:
    """``metadata.metric`` → ``metric_name``/``metric_label``/``metric_direction``/``hardware_invariant``."""
    metric = metadata.get("metric") or {}
    name = metric.get("name")
    if not isinstance(name, str) or not name:
        raise RegistryError(f"{slug}: task.yaml metadata.metric.name is required")
    direction = _METRIC_DIRECTION.get(str(metric.get("direction", "higher")))
    if direction is None:
        raise RegistryError(f"{slug}: metadata.metric.direction must be higher|lower")
    label = metric.get("label") or name.replace("_", " ").capitalize()
    return {
        "metric_name": name,
        "metric_label": label,
        "metric_direction": direction,
        "hardware_invariant": bool(metric.get("hardware_invariant", False)),
    }


def _integral(value: Any) -> Any:
    """Engine timeouts are floats (``120.0``); the registry shows the authored integers."""
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value


def task_entry(task: LoadedTask, identities: list[dict[str, Any]]) -> dict[str, Any]:
    """One ``tasks[]`` element; ``identities`` already carries the current identity first."""
    metadata = task.metadata
    score_cap = read_score_cap(task.root)
    entry: dict[str, Any] = {
        "slug": task.slug,
        "title": read_title(task.root, task.slug),
        "platform": metadata.get("platform"),
        "direction": metadata.get("direction"),
        "identities": identities,
        "score_cap": score_cap,
        "resources": {
            key: task.resources.get(key) for key in ("cpus", "memory_mb", "gpus")
        },
        "timeouts": {
            key: _integral(task.timeouts.get(key)) for key in ("setup", "agent", "verify")
        },
        "status": "active",
    }
    entry.update(metric_fields(metadata, task.slug))
    metric_direction = str((metadata.get("metric") or {}).get("direction", "higher"))
    entry.update(read_anchor(task.root, score_cap, metric_direction))
    return entry


# --------------------------------------------------------------------------- #
# identity history (§4): outside every task folder so it never feeds the digest
# --------------------------------------------------------------------------- #
_HISTORY_KEYS = ("slug", "spec_hash", "content_digest", "since")


def load_history(path: Path) -> list[dict[str, Any]]:
    """The history file is a flat, newest-first list of ``{slug, spec_hash, content_digest, since}``.

    A maintainer prunes this file by hand, so a typo must surface as a ``RegistryError``
    (the CLI's ``error:`` line), not a traceback or a silently null identity.
    """
    if not path.is_file():
        return []
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise RegistryError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(loaded, list):
        raise RegistryError(f"{path}: identity history must be a JSON list")
    for index, record in enumerate(loaded):
        if not isinstance(record, dict):
            raise RegistryError(f"{path}: record {index} is not an object")
        for key in _HISTORY_KEYS:
            if not isinstance(record.get(key), str) or not record[key]:
                raise RegistryError(
                    f"{path}: record {index} must carry a non-empty string {key!r}"
                )
    return loaded


def merge_history(
    history: list[dict[str, Any]], task: LoadedTask, now: str
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Prepend the current identity when either hash changed since the newest record.

    Returns ``(updated_history, identities_for_task)``; the latter is newest-first and
    stripped of ``slug``. Older records are kept verbatim — pruning after a genuinely
    breaking change (new grader semantics, new anchor) is a maintainer's call, not this
    script's. One exception: a pair appears at most once. A revert (A → B → A) or a
    hand-reordered file would otherwise re-prepend an identity that already sits further
    down; the older duplicate is dropped so the current identity is always ``[0]`` (which
    is what the website's ``seed.py`` reads) and the file stays prunable by eye.
    """
    own = [record for record in history if record.get("slug") == task.slug]
    others = [record for record in history if record.get("slug") != task.slug]
    newest = own[0] if own else None
    current = (task.spec_hash, task.content_digest)
    if newest is None or (newest.get("spec_hash"), newest.get("content_digest")) != current:
        own = [
            record for record in own
            if (record.get("spec_hash"), record.get("content_digest")) != current
        ]
        own.insert(
            0,
            {
                "slug": task.slug,
                "spec_hash": task.spec_hash,
                "content_digest": task.content_digest,
                "since": now,
            },
        )
    identities = [
        {key: record.get(key) for key in ("spec_hash", "content_digest", "since")}
        for record in own
    ]
    return others + own, identities


# --------------------------------------------------------------------------- #
# repository state and provenance
# --------------------------------------------------------------------------- #
def _git(args: list[str], cwd: Path) -> str:
    result = subprocess.run(
        ["git", *args], cwd=str(cwd), capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise RegistryError(f"git {' '.join(args)} failed in {cwd}: {result.stderr.strip()}")
    return result.stdout


def require_clean(task_dir: Path) -> None:
    """Refuse a task folder with any uncommitted or ignored file.

    ``content_digest`` covers every regular file in the folder, so an ignored
    ``__pycache__`` or an unstaged edit would register an identity no commit reproduces.
    """
    status = _git(
        ["status", "--porcelain", "--ignored", "--", str(task_dir)], cwd=task_dir
    ).strip()
    if status:
        raise RegistryError(
            f"{task_dir} is not clean; commit or remove these before exporting "
            f"(or pass --allow-dirty for a throwaway export):\n{status}"
        )


def source_commit(repo: Path) -> str:
    return _git(["rev-parse", "HEAD"], cwd=repo).strip()


def engine_provenance(engine_dir: Path) -> dict[str, Any]:
    """Engine commit + package version + the ``ale_verify`` content hash the locks carry.

    The hash is what ``lock.ale_verify.content_hash`` records per episode, so the server
    can tell a run made with the registered engine from one made with a modified verifier.
    It is only cheap from inside the engine venv; elsewhere it is ``null``.
    """
    try:
        commit = source_commit(engine_dir)
    except RegistryError:
        commit = "unknown"
    try:
        version: str | None = importlib.metadata.version("ale-run")
    except importlib.metadata.PackageNotFoundError:
        version = None
    try:
        from ale.run.verification import installed_ale_verify

        ale_verify_hash: str | None = installed_ale_verify()[2]
    except ImportError:
        ale_verify_hash = None
    return {
        "commits": [commit],
        "version": version,
        "ale_verify_content_hash": ale_verify_hash,
    }


def carry_over(existing: Path | None) -> dict[str, Any]:
    """``platforms``/``directions``/``category_tree`` copied verbatim from the current registry.

    The website file is the only copy of the taxonomy; this repository carries none.
    """
    if existing is None:
        return {"platforms": [], "directions": [], "category_tree": {}}
    loaded = json.loads(existing.read_text(encoding="utf-8"))
    return {
        "platforms": loaded.get("platforms", []),
        "directions": loaded.get("directions", []),
        "category_tree": loaded.get("category_tree", {}),
    }


def release_block(name: str, version: str, task_set: list[str]) -> dict[str, Any]:
    """The release policy: macro mean over *every* active task, missing scored as zero."""
    return {
        "name": name,
        "version": version,
        "status": "active",
        "aggregation_policy": {
            "type": "macro_mean",
            "task_set": task_set,
            "score_cap": DEFAULT_SCORE_CAP,
            "missing_task_policy": "zero",
        },
    }


# --------------------------------------------------------------------------- #
# assembly
# --------------------------------------------------------------------------- #
def build_registry(
    tasks: list[LoadedTask],
    *,
    history: list[dict[str, Any]],
    now: str,
    source_commit_sha: str,
    engine: dict[str, Any],
    carried: dict[str, Any],
    release_name: str = DEFAULT_RELEASE_NAME,
    release_version: str = DEFAULT_RELEASE_VERSION,
    allow_dirty: bool = False,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Assemble the v2 document; returns ``(registry, updated_history)``.

    With ``allow_dirty`` the clean-tree check is skipped and the provenance says so:
    ``source_commit`` gets a ``-dirty`` suffix and ``dirty: true`` is set, so a throwaway
    export can never pass for a release one (the website's validator can refuse it).
    """
    base_tasks = sorted(
        (task for task in tasks if task.variant == REGISTERED_VARIANT), key=lambda t: t.slug
    )
    if not base_tasks:
        raise RegistryError("no base-variant task to register")
    entries: list[dict[str, Any]] = []
    for task in base_tasks:
        if not allow_dirty:
            require_clean(task.root)
        history, identities = merge_history(history, task, now)
        entries.append(task_entry(task, identities))
    provenance: dict[str, Any] = {
        "source": "engine_task_loader",
        "tasks_repo": REPO_ROOT.name,
        "source_commit": f"{source_commit_sha}-dirty" if allow_dirty else source_commit_sha,
        "generated_at": now,
        "engine": engine,
    }
    if allow_dirty:
        provenance["dirty"] = True
    registry = {
        "schema_version": SCHEMA_VERSION,
        "provenance": provenance,
        "release": release_block(
            release_name, release_version, [entry["slug"] for entry in entries]
        ),
        "tasks": entries,
        **carried,
    }
    return registry, history


def history_writable(path: Path, *, allow_dirty: bool) -> bool:
    """Whether the merged history may be written back to ``path``.

    A clean export always writes. A ``--allow-dirty`` export may only write to a file other
    than the canonical ``identity-history.json``: the identity it would record
    is one no commit reproduces, and the canonical file is what the website is seeded from.
    """
    if not allow_dirty:
        return True
    return path.resolve() != DEFAULT_HISTORY.resolve()


def dump_json(payload: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def utc_now() -> str:
    return dt.datetime.now(dt.UTC).strftime("%Y-%m-%dT%H:%M:%SZ")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--tasks", type=Path, default=DEFAULT_TASKS, help="task collection root")
    parser.add_argument(
        "--history", type=Path, default=None,
        help=f"identity-history.json to merge and write back (default: {DEFAULT_HISTORY}; "
             "with --allow-dirty the default file is read but never written)",
    )
    parser.add_argument(
        "--carry-over", type=Path, default=None,
        help="existing website registry.json whose platforms/directions/category_tree are copied",
    )
    parser.add_argument("--out", type=Path, required=True, help="where to write the registry")
    parser.add_argument("--release-name", default=DEFAULT_RELEASE_NAME)
    parser.add_argument("--release-version", default=DEFAULT_RELEASE_VERSION)
    parser.add_argument(
        "--allow-dirty", action="store_true",
        help="skip the clean-tree check (throwaway exports and tests only): provenance is "
             "stamped <sha>-dirty / dirty=true and the default history file is not written",
    )
    return parser.parse_args(argv)


def _engine_importable() -> bool:
    try:
        import ale.run.tasksets.manifest  # noqa: F401
    except ImportError:
        return False
    return True


def _reexec_in_engine(argv: list[str]) -> int:
    """Re-run this script through the pinned engine's venv.

    ``uv run --project`` selects the engine's environment while keeping the caller's working
    directory, so relative ``--tasks``/``--out`` paths keep meaning what the caller typed.
    ``VIRTUAL_ENV`` is cleared for the same reason as in ``ale_onboard``: this repo's own
    venv would otherwise make uv refuse the engine's.
    """
    if os.environ.get(_REEXEC_FLAG):
        print(
            f"error: `ale` is not importable even inside {ENGINE_DIR}; run `uv sync` there",
            file=sys.stderr,
        )
        return 2
    env = dict(os.environ)
    env["VIRTUAL_ENV"] = ""
    env[_REEXEC_FLAG] = "1"
    result = subprocess.run(
        ["uv", "run", "--project", str(ENGINE_DIR), "python", str(Path(__file__).resolve()), *argv],
        env=env,
        check=False,
    )
    return result.returncode


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not _engine_importable():
        return _reexec_in_engine(argv)
    args = parse_args(argv)
    history_path = args.history if args.history is not None else DEFAULT_HISTORY
    try:
        tasks = load_with_engine(args.tasks.resolve())
        registry, history = build_registry(
            tasks,
            history=load_history(history_path),
            now=utc_now(),
            source_commit_sha=source_commit(REPO_ROOT),
            engine=engine_provenance(ENGINE_DIR),
            carried=carry_over(args.carry_over),
            release_name=args.release_name,
            release_version=args.release_version,
            allow_dirty=args.allow_dirty,
        )
    except RegistryError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    if history_writable(history_path, allow_dirty=args.allow_dirty):
        dump_json(history, history_path)
        history_note = f"history: {history_path}"
    else:
        history_note = (
            f"history NOT written ({history_path} is the canonical file and the tree was not "
            "checked for cleanliness; pass --history <elsewhere> for a throwaway copy)"
        )
    dump_json(registry, args.out)
    print(f"wrote {args.out} ({len(registry['tasks'])} task(s)); {history_note}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
