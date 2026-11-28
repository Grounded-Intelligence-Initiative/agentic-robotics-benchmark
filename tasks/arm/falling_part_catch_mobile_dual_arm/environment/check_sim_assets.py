"""CPU-only, agent-UID check of the files selected by OmniGibson's launcher."""
import ast
import hashlib
from importlib import metadata, util
from pathlib import Path
import tomllib


def main():
    spec = util.find_spec('omnigibson')
    if spec is None or spec.origin is None:
        raise RuntimeError('OmniGibson is not installed')
    package = Path(spec.origin).parent
    versions = []
    for node in ast.walk(ast.parse((package / 'simulator.py').read_text())):
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Attribute) and target.attr == 'KIT_FILES'
            for target in node.targets
        ):
            versions.append(ast.literal_eval(node.value))
    installed = tuple(int(part) for part in metadata.version('isaacsim').split('.')[:3])
    assert versions and all(installed in table for table in versions), installed
    selected = {table[installed] for table in versions}
    assert len(selected) == 1, selected
    kit = package / selected.pop()
    contents = kit.read_bytes()
    assert hashlib.sha256(contents).hexdigest() == '36d0df3b6fdee2c8a83a63a2ee0f6818f8ef4c1c272a0de32cdc5d3be74f0c79'
    config = tomllib.loads(contents.decode())
    assert 'isaacsim.exp.base' in config['dependencies']
    icon = package.parent.parent / 'docs/assets/OmniGibson_logo.png'
    assert icon.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
    print(f'[image] readable Isaac {installed} launch kit and logo OK: {kit}')
    isaac = util.find_spec('isaacsim')
    assert isaac is not None and isaac.origin is not None
    bundles = list((Path(isaac.origin).parent / 'extscache').glob('omni.warp.core-*'))
    bundled_versions = {p.name.removeprefix('omni.warp.core-').split('+')[0] for p in bundles}
    standalone = metadata.version('warp-lang')
    assert bundled_versions == {standalone}, (
        f'Warp version mismatch: standalone={standalone}, Isaac bundles={bundled_versions}'
    )
    import warp
    assert warp.__version__ == standalone, (warp.__version__, standalone)
    print(f'[image] standalone and bundled Warp agree: {standalone}')


if __name__ == '__main__':
    main()
