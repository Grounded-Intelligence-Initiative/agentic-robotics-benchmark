"""Build-only: derive sealed test frames directly from pinned official originals."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

from prepare_release import export

SOURCES = {
    'Videos.zip': ('https://www.datadepot.rcac.purdue.edu/bouman/data/uav/Videos.zip',
                   '84f0878dfb61a1ecd9b4f865e4913debdda7dd3e4eabf08a06e33cfc3c2fd998'),
    'Video_Annotation.zip': ('https://charlesbouman.org/UAV_Dataset/Video_Annotation.zip',
                   'abc9110cd287d76d2c6ed70f1163386d881bd567e39930431d393a2b383271b1'),
}

def main():
    with tempfile.TemporaryDirectory(prefix='uav-originals-') as temp:
        root = Path(temp)
        for name, (url, expected) in SOURCES.items():
            dest = root / name
            subprocess.run(['curl', '-fL', '--retry', '5', '--connect-timeout', '30',
                            '--output', str(dest), url], check=True)
            digest = hashlib.sha256()
            with dest.open('rb') as f:
                for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
                    digest.update(block)
            if digest.hexdigest() != expected:
                raise ValueError(f'Official source hash mismatch: {name}')
        videos = root / 'videos'
        videos.mkdir()
        with zipfile.ZipFile(root / 'Videos.zip') as archive:
            members = {Path(i.filename).name: i for i in archive.infolist()
                       if '__MACOSX' not in i.filename}
            for clip in range(41, 51):
                name = f'Clip_{clip}.mov'
                with archive.open(members[name]) as source, (videos / name).open('wb') as out:
                    shutil.copyfileobj(source, out)
        destination = Path('/opt/uav-heldout')
        destination.mkdir(mode=0o700, parents=True, exist_ok=True)
        export(videos, root / 'Video_Annotation.zip', destination, ['test'], workers=2)

if __name__ == '__main__':
    main()
