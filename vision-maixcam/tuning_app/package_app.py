"""Package only the app.yaml allowlist without installing build dependencies."""
from pathlib import Path
import re
import zipfile

root = Path(__file__).resolve().parent.parent / 'maixcam_app_v2'
manifest = (root / 'app.yaml').read_text(encoding='utf-8')
app_id = re.search(r'^id:\s*(\S+)', manifest, re.M).group(1)
version = re.search(r'^version:\s*(\S+)', manifest, re.M).group(1)
entries = []
active = False
for line in manifest.splitlines():
    if line == 'files:':
        active = True
    elif active and line.strip().startswith('- '):
        entries.append(line.strip()[2:])
    elif active and line and not line[0].isspace() and not line.startswith('#'):
        active = False
if not entries:
    raise ValueError('Empty release allowlist')
archive = root / 'dist' / ('maix-%s-v%s.zip' % (app_id, version))
archive.parent.mkdir(exist_ok=True)
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
    for entry in entries:
        path = root / entry
        if not path.exists() or not path.resolve().is_relative_to(root.resolve()):
            raise ValueError('Invalid release entry: ' + entry)
        for item in sorted(path.rglob('*')) if path.is_dir() else [path]:
            if item.is_file():
                bundle.write(item, item.relative_to(root).as_posix())
with zipfile.ZipFile(archive) as bundle:
    if bundle.testzip() is not None:
        raise RuntimeError('Archive integrity check failed')
    if bundle.read('app.yaml') != (root / 'app.yaml').read_bytes():
        raise RuntimeError('Packaged manifest differs from source')
    print(str(archive), len(bundle.namelist()), 'files; archive verified')
