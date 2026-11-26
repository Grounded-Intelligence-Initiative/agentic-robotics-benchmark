"""Build-time integrity checks using tiny local files, without GPU or network."""
import hashlib
import importlib.util
from pathlib import Path
import stat
import zipfile
import pytest

ENV = Path(__file__).resolve().parents[2] / "environment"


def module(name):
    spec = importlib.util.spec_from_file_location(name, ENV / (name + ".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_mirror_rechecks_same_size_cached_file(tmp_path):
    src = tmp_path / "source" / "Isaac"; src.mkdir(parents=True)
    (src / "asset.usd").write_bytes(b"good")
    target = tmp_path / "target"; (target / "Isaac").mkdir(parents=True)
    (target / "Isaac/asset.usd").write_bytes(b"evil")
    manifest = dict(source=src.parent.as_uri() + "/", files=[dict(
        path="Isaac/asset.usd", bytes=4, sha256=hashlib.sha256(b"good").hexdigest())])
    mirror = module("mirror_assets")
    mirror.mirror(target, manifest, ["Isaac/"])
    assert (target / "Isaac/asset.usd").read_bytes() == b"good"


def test_mirror_rejects_corrupt_download(tmp_path):
    src = tmp_path / "source" / "Isaac"; src.mkdir(parents=True)
    (src / "asset.usd").write_bytes(b"evil")
    target = tmp_path / "target"
    manifest = dict(source=src.parent.as_uri() + "/", files=[dict(
        path="Isaac/asset.usd", bytes=4, sha256=hashlib.sha256(b"good").hexdigest())])
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        module("mirror_assets").mirror(target, manifest, ["Isaac/"])
    assert not (target / "Isaac/asset.usd").exists()
    assert not (target / "Isaac/asset.usd.download").exists()


def test_robot_archive_rejects_escaping_paths_and_symlinks(tmp_path):
    robot = module("fetch_robot_assets")
    for name, symlink in [("../escape", False), ("/tmp/escape", False), ("models/link", True)]:
        archive = tmp_path / "robots.zip"
        with zipfile.ZipFile(archive, "w") as out:
            item = zipfile.ZipInfo(name)
            if symlink:
                item.create_system = 3
                item.external_attr = (stat.S_IFLNK | 0o777) << 16
            out.writestr(item, "../escape")
        with pytest.raises(ValueError, match="unsafe"):
            robot.unpack(archive, tmp_path / "unpacked")
    assert not (tmp_path / "escape").exists()
