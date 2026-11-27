"""Fetch canonical pinned shards sequentially to bound image-build scratch space."""
from pathlib import Path
import subprocess
import sys
import tempfile
import urllib.parse

from fetch_assets import read_list, fetch, URL

manifest, destination = Path(sys.argv[1]), Path(sys.argv[2])
head, files = read_list(manifest)
for digest, relative in files:
    if Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('Unsafe asset path')
    if not relative.endswith('.tar'):
        raise ValueError('Public dataset release must contain tar shards')
    with tempfile.TemporaryDirectory(prefix='uav-public-shard-') as temp:
        url = URL.format(endpoint=head['endpoint'].rstrip('/'), repo=head['repo'],
                         rev=head['revision'], path=urllib.parse.quote(head['prefix'] + '/' + relative))
        fetch(url, digest, relative, temp)
        subprocess.run([sys.executable, str(Path(__file__).with_name('unpack_public.py')),
                        str(Path(temp) / relative), str(destination)], check=True)
for required in ('train/annotations.json', 'val/annotations.json', 'DATA_CARD.md', 'LICENSE.txt'):
    if not (destination / required).is_file():
        raise ValueError(f'Missing public asset: {required}')
