"""用固定远程源码验证协议和任务取消；相机/UART是假对象，不访问设备。"""
import ast
import hashlib
import json
import sys
import types
from pathlib import Path

r = Path(__file__).resolve().parent.parent
root = r / 'evidence/vision-source-20261019'
appdir = root / 'maixcam_app_v2'
sys.path.insert(0, str(appdir))
sys.dont_write_bytecode = True
maix = types.ModuleType('maix')
maix.uart = types.SimpleNamespace()
maix.pinmap = types.SimpleNamespace()
sys.modules['maix'] = maix
timing = types.ModuleType('timing')
timing.ticks_ms = lambda: 0
timing.sleep_ms = lambda n: None
sys.modules['timing'] = timing
import config
import protocol

vectors = {s: protocol.encode_frame(s).hex() for s in [
    'R', 'D', 'READY', protocol.pack_move(30), protocol.pack_move(-30),
    protocol.pack_disc(318, 242, -2, 1)]}
assert vectors['D'] == 'aa550144ce0d' and vectors['R'] == 'aa550152ac0d'
parser = protocol.FrameParser()
packet = protocol.encode_frame(protocol.pack_disc(318, 242, -2, 1))
assert parser.feed(packet[:3]) == []
assert parser.feed(packet[3:]) == [b'DISC,318,242,-2,1']


class Link:
    def __init__(self):
        self.pending = []
        self.sent = []

    def poll_command(self):
        return self.pending.pop(0) if self.pending else None

    def send_payload(self, payload):
        self.sent.append(payload)
        if payload.startswith('MOVE,'):
            self.pending.append('R')


class Img:
    def __getattr__(self, name):
        return lambda *args: None


pipeline = types.SimpleNamespace(sx=1.0, roi=lambda roi: roi, point=lambda x, y: (x, y))
ns = {
    'config': config, 'log': lambda *args: None,
    'CMD_RESET': protocol.CMD_RESET, 'parse_command': protocol.parse_command,
    'pack_busy': protocol.pack_busy, 'pack_ready': protocol.pack_ready,
    'pack_disc': protocol.pack_disc, 'pack_none': protocol.pack_none,
    'pack_move': protocol.pack_move, '_pipeline': lambda p: pipeline,
    'ticks_ms': lambda: 0, 'sleep_ms': lambda n: None,
    'app': types.SimpleNamespace(need_exit=lambda: False),
    'image': types.SimpleNamespace(COLOR_GREEN=0, COLOR_RED=1, COLOR_YELLOW=2),
    'draw_roi': lambda *args: None,
    'vision': types.SimpleNamespace(find_disc_center=lambda *args: (350, 240, 90)),
}
nodes = [n for n in ast.parse((appdir / 'tasks.py').read_text(encoding='utf-8-sig')).body
         if isinstance(n, (ast.ClassDef, ast.FunctionDef))
         and n.name in ['TaskCtl', 'StabilityTracker', 'task_disc']]
exec(compile(ast.Module(body=nodes, type_ignores=[]), 'actual-camera-tasks', 'exec'), ns)
link = Link()
reply = ns['task_disc'](types.SimpleNamespace(read=lambda: Img()), None,
                       config.ROI_DISC_ROUGH, 'D', ctl=ns['TaskCtl'](link))
assert link.sent == ['MOVE,30'] and reply == 'READY'
ns['vision'] = types.SimpleNamespace(find_disc_center=lambda *args: (320, 240, 90))
link = Link()
reply = ns['task_disc'](types.SimpleNamespace(read=lambda: Img()), None,
                       config.ROI_DISC_ROUGH, 'D', ctl=ns['TaskCtl'](link))
assert not link.sent and reply == 'DISC,320,240,0,1'

tree = json.loads((root / 'tree.json').read_text(encoding='utf-8-sig'))
blobs = {x['path']: x['sha'] for x in tree['tree'] if x['type'] == 'blob'}
hashes = {}
for p in root.rglob('*'):
    if not p.is_file() or p.name in ['tree.json', 'COMMIT.txt']:
        continue
    data = p.read_bytes()
    remote = 'vision-maixcam/' + p.relative_to(root).as_posix()
    assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == blobs[remote]
    hashes[remote] = hashlib.sha256(data).hexdigest()
report = {
    'passed': True, 'commit': (root / 'COMMIT.txt').read_text().strip(),
    'sourceGitBlobsVerified': hashes, 'vectors': vectors,
    'actualTaskChecks': [
        'actual pack_move/pack_disc and frame parser',
        'actual TaskCtl/task_disc: MOVE -> R cancellation -> READY without second move',
        'actual task_disc: five centered frames -> DISC,320,240,0,1'],
    'scope': 'actual remote Python functions with stub camera/UART, no hardware',
}
(r / 'evidence/maixcam-r11-protocol-check.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('PASS actual MaixCAM2 protocol and R cancellation; source git blobs verified; no hardware')
