"""Install or verify the original public GEAR-SONIC decoder and Unitree G1/Dex3 model."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
from pathlib import Path, PurePosixPath
import re
import tempfile
import time
from urllib.error import URLError
from urllib.request import Request, urlopen


SOURCES = {
    "policy": "https://huggingface.co/nvidia/GEAR-SONIC/resolve/6733128a3d8a523b1418b06bca3cdf61c8b0987f/",
    "menagerie_g1": (
        "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/"
        "1b86ece576591213e2b666ebf59508454200ca97/unitree_g1/"
    ),
}


def entries(manifest):
    result = {}
    for line in manifest.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        digest, name = line.split("  ", 1)
        path = PurePosixPath(name)
        if (not re.fullmatch(r"[0-9a-f]{64}", digest) or len(path.parts) < 2
                or path.parts[0] not in SOURCES or ".." in path.parts
                or str(path) != name or name in result):
            raise ValueError(f"Invalid asset entry: {line!r}")
        result[name] = digest
    if not result:
        raise ValueError(f"Empty asset manifest: {manifest}")
    return result


def digest_file(path):
    h = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def install(root, name, expected, check_only):
    path = root / name
    if path.is_file() and digest_file(path) == expected:
        return
    if check_only:
        raise ValueError(f"Missing or incorrect asset: {name}")
    source, suffix = name.split("/", 1)
    url = SOURCES[source] + suffix
    if source == "policy":
        url += "?download=1"
    path.parent.mkdir(parents=True, exist_ok=True)
    request = Request(url, headers={"User-Agent": "g1-sonic-token-control/1.0"})
    for attempt in range(3):
        try:
            with tempfile.TemporaryDirectory(dir=path.parent) as scratch:
                temp = Path(scratch) / "asset"
                with urlopen(request, timeout=90) as response, temp.open("wb") as output:
                    for block in iter(lambda: response.read(1 << 20), b""):
                        output.write(block)
                if digest_file(temp) != expected:
                    raise ValueError(f"Publisher SHA-256 mismatch: {name}")
                temp.chmod(0o644)
                temp.replace(path)
            return
        except (URLError, TimeoutError) as exc:
            if attempt == 2:
                raise RuntimeError(f"Could not fetch {name} from {url}") from exc
            time.sleep(2 ** attempt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("public_assets.sha256"))
    parser.add_argument("--check", action="store_true", help="Check installed files offline")
    args = parser.parse_args()
    assets = entries(args.manifest)
    with ThreadPoolExecutor(max_workers=4) as pool:
        jobs = [pool.submit(install, args.destination, name, expected, args.check)
                for name, expected in assets.items()]
        for job in jobs:
            job.result()
    print(f"Verified {len(assets)} public assets in {args.destination}")


if __name__ == "__main__":
    main()
