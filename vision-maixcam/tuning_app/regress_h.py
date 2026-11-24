"""Compare H acceptance and coordinates on recorded frames; not ground-truth accuracy."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time
import types
import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'maixcam_app_v2'))
maix = types.ModuleType('maix')
maix.image = types.ModuleType('maix.image')
sys.modules['maix'] = maix
sys.modules['maix.image'] = maix.image
import config
import vision

baseline = types.ModuleType('baseline_vision')
source = subprocess.check_output(['git', 'show', 'HEAD:vision-maixcam/maixcam_app_v2/vision.py'], cwd=ROOT).decode('utf-8')
exec(compile(source, 'baseline_vision', 'exec'), baseline.__dict__)


class Frame:
    def __init__(self, path):
        self.gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if self.gray is None:
            raise ValueError('Cannot read ' + str(path))
    def width(self): return self.gray.shape[1]
    def height(self): return self.gray.shape[0]
    def get_pixel(self, x, y): return [int(self.gray[y, x])]


baseline._gray_np = lambda img: img.gray
vision._gray_np = lambda img: img.gray
files = [ROOT / '_from_device/dataset/other' / ('other_%04d.jpg' % n) for n in range(410,445)]
files += sorted((ROOT / '_from_device/h_latest').glob('*raw*.jpg'))
rows = []
for path in files:
    if not path.exists(): continue
    try:
        frame = Frame(path)
    except ValueError as exc:
        print("Skipping invalid recorded frame: %s (%d bytes)" % (exc, path.stat().st_size),
              file=sys.stderr)
        continue
    row = {'file': str(path)}
    for label, module, alternate in [('baseline', baseline, False), ('new', vision, True)]:
        config.PLATE_ARC_SKIP_ALT_ON = alternate
        meta = {}
        start = time.perf_counter()
        result = module.find_disc_arc(frame, config.PLATE_INIT_CX, config.PLATE_INIT_CY, config.PLATE_INIT_R, meta=meta)
        row[label] = {'result': result, 'ms': round((time.perf_counter()-start)*1000,2), 'meta': meta}
    a, b = row['baseline']['result'], row['new']['result']
    if a and b:
        row['low_delta'] = [round(b[0]-a[0],3), round(b[1]+b[2]-a[1]-a[2],3)]
    rows.append(row)
if not rows:
    raise RuntimeError('No readable H regression frames')
config.PLATE_ARC_SKIP_ALT_ON = True
dest = ROOT / '_from_device/h_latest/regression.json'
dest.write_text(json.dumps(rows, indent=2), encoding='utf-8')
summary = {'frames': len(rows), 'baseline_accepted': sum(r['baseline']['result'] is not None for r in rows),
           'new_accepted': sum(r['new']['result'] is not None for r in rows),
           'changed_acceptance': [{'file': Path(r['file']).name, 'old': r['baseline']['meta'].get('reason'),
                                   'new': r['new']['meta'].get('reason')} for r in rows if bool(r['baseline']['result']) != bool(r['new']['result'])],
           'max_common_low_delta': [max([abs(r['low_delta'][i]) for r in rows if 'low_delta' in r] or [0]) for i in (0,1)]}
print(json.dumps(summary, ensure_ascii=False, indent=2))
