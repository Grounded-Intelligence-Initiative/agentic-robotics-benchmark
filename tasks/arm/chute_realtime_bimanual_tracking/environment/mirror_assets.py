"""Download only the pinned NVIDIA public files needed by the cell, verifying every byte."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys
import urllib.parse
import urllib.request


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def mirror(root, manifest, prefixes):
    rows = manifest["files"]
    if not rows:
        raise ValueError("empty NVIDIA asset manifest")
    seen = set()
    for row in rows:
        path = PurePosixPath(row["path"])
        if (path.is_absolute() or ".." in path.parts or "\\" in row["path"]
                or row["path"] in seen or not any(row["path"].startswith(p) for p in prefixes)):
            raise ValueError("invalid NVIDIA asset manifest path")
        seen.add(row["path"])

    def fetch(row):
        path = root / row["path"]
        if path.is_file() and path.stat().st_size == row["bytes"] and digest(path) == row["sha256"]:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".download")
        url = manifest["source"] + urllib.parse.quote(row["path"], safe="/")
        for attempt in range(3):
            try:
                with urllib.request.urlopen(url, timeout=120) as response, temporary.open("wb") as out:
                    while block := response.read(1024 * 1024):
                        out.write(block)
                if temporary.stat().st_size != row["bytes"] or digest(temporary) != row["sha256"]:
                    raise ValueError(f"NVIDIA asset size/SHA256 mismatch: {row['path']}")
                temporary.replace(path)
                return
            except Exception:
                temporary.unlink(missing_ok=True)
                if attempt == 2:
                    raise
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(fetch, rows))
    print("NVIDIA_ASSETS_VERIFIED", len(rows), "files", sum(r["bytes"] for r in rows), "bytes")


if __name__ == "__main__":
    manifest = json.loads(Path(__file__).with_name("nvidia-assets.json").read_text())
    mirror(Path(sys.argv[1]), manifest, sys.argv[2:])
