"""Fetch the MIT robot-only archive at an immutable revision, with content verification."""
import hashlib
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
import urllib.request
import zipfile

REVISION = "9f0d57d465726976ed98138d3f8b8ca3e2186775"
NAME = "omnigibson-robot-assets-3.8.2.zip"
SHA256 = "3d813b2181e0581cf2300a40892de70f8475fe59346ceeea4fb9bf7ff21ce126"
SIZE = 641004353
URL = f"https://huggingface.co/datasets/behavior-1k/zipped-datasets/resolve/{REVISION}/{NAME}"


def unpack(archive, destination):
    with zipfile.ZipFile(archive) as source:
        for member in source.infolist():
            path = PurePosixPath(member.filename)
            mode = member.external_attr >> 16
            if path.is_absolute() or ".." in path.parts or "\\" in member.filename or stat.S_ISLNK(mode):
                raise ValueError("unsafe robot archive entry")
        source.extractall(destination)


def main(destination):
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError("refusing to merge with an existing robot asset directory")
    with tempfile.TemporaryDirectory(prefix="chute-robot-assets-") as temp:
        archive = Path(temp) / NAME
        digest, size = hashlib.sha256(), 0
        with urllib.request.urlopen(URL, timeout=120) as response, archive.open("wb") as out:
            while block := response.read(1024 * 1024):
                digest.update(block); size += len(block); out.write(block)
        if size != SIZE or digest.hexdigest() != SHA256:
            raise ValueError("robot asset archive size/SHA256 mismatch")
        destination.mkdir(parents=True)
        unpack(archive, destination)
    if not (destination / "models/r1pro").is_dir():
        raise ValueError("pinned robot archive lacks the R1Pro model")
    print(f"ROBOT_ASSETS_VERIFIED version=3.8.2 revision={REVISION} sha256={SHA256}")


if __name__ == "__main__":
    main(sys.argv[1])
