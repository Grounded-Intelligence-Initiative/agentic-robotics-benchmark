"""Extract a checksum-verified release; reject links and paths outside train/val."""
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile

archive, output = Path(sys.argv[1]), Path(sys.argv[2])
allowed = {'train', 'val', 'DATA_CARD.md', 'LICENSE.txt'}
output.mkdir(parents=True, exist_ok=True)
with tarfile.open(archive) as source:
    for member in source:
        path = PurePosixPath(member.name)
        if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] not in allowed:
            raise ValueError(f'Unexpected public archive member: {member.name}')
        dest = output / path
        if member.isdir():
            dest.mkdir(parents=True, exist_ok=True)
        elif member.isfile():
            dest.parent.mkdir(parents=True, exist_ok=True)
            with source.extractfile(member) as src, dest.open('wb') as out:
                shutil.copyfileobj(src, out)
        else:
            raise ValueError(f'Unsupported archive member type: {member.name}')
