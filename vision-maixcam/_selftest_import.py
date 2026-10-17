# -*- coding: utf-8 -*-
"""本地自检：用假 maix 模块导入我的 4 个文件，抓语法/属性错误（在 PC 上跑，不进板子）"""
import sys, types, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maixcam_app'))

# ---- 造一个假的 maix 包 ----
maix = types.ModuleType('maix')

class _FakeUART:
    def __init__(self, dev, baud, *a, **k):
        self.dev, self.baud = dev, baud
    def write(self, data): return len(data)
    def read(self, *a, **k): return b''
    def close(self): pass

uart = types.ModuleType('maix.uart')
uart.UART = _FakeUART
uart.list_devices = lambda: ['/dev/ttyS2']

pinmap = types.ModuleType('maix.peripheral.pinmap')

class _FakeErr:
    def __init__(self, code=0, m=''): self._c, self._m = code, m
    def code(self): return self._c
    def msg(self): return self._m

# 模拟 MaixCAM2 实测结果：A21 支持 UART4_TX，B0 支持 UART2_TX
_PIN_FUNCS = {'A21': ['UART4_TX', 'GPIO'], 'A22': ['UART4_RX', 'GPIO'],
              'B0': ['UART2_TX', 'GPIO'], 'B1': ['UART2_RX', 'GPIO']}
_pins_set = {}

def _set_pin_function(pin, func):
    allowed = _PIN_FUNCS.get(pin)
    if allowed is None:
        return _FakeErr(1, 'unknown pin ' + pin)
    if func not in allowed:
        return _FakeErr(2, 'func %s not supported on %s' % (func, pin))
    _pins_set[pin] = func
    return _FakeErr(0, 'ok')

pinmap.set_pin_function = _set_pin_function
pinmap.get_pin_functions = lambda pin: _PIN_FUNCS.get(pin, [])

err = types.ModuleType('maix.err')
err.check_raise = lambda e, msg='', fatal=False: None

app = types.ModuleType('maix.app')
app.need_exit = lambda: True
app.set_exit_flag = lambda f: None

time_mod = types.ModuleType('maix.time')
time_mod.ticks_ms = lambda: 0
time_mod.sleep_ms = lambda ms: None

class _FakeImg:
    def __init__(self, *a, **k): pass
    def width(self): return 640
    def height(self): return 480
    def find_blobs(self, *a, **k): return []
    def find_lines(self, *a, **k): return []
    def find_circles(self, *a, **k): return []
    def find_qrcodes(self, *a, **k): return []
    def draw_rect(self, *a, **k): pass
    def draw_cross(self, *a, **k): pass
    def draw_string(self, *a, **k): pass
    def draw_circle(self, *a, **k): pass

image = types.ModuleType('maix.image')
image.Image = _FakeImg
image.Blob = type('Blob', (), {})
image.COLOR_RED = 'red'; image.COLOR_GREEN = 'green'; image.COLOR_BLUE = 'blue'
image.COLOR_WHITE = 'white'; image.COLOR_YELLOW = 'yellow'
image.COLOR_PURPLE = 'purple'; image.COLOR_GRAY = 'gray'
image.Fit = type('Fit', (), {'FIT_CONTAIN': 0, 'FIT_FILL': 1})
image.resize_map_pos = lambda wi, hi, wo, ho, fit, x, y: [x, y]

class _FakeCam:
    def __init__(self, w, h, fps=0): self.w, self.h = w, h
    def skip_frames(self, n): pass
    def read(self): return _FakeImg()

camera = types.ModuleType('maix.camera')
camera.Camera = _FakeCam

touchscreen = types.ModuleType('maix.touchscreen')

class _FakeTS:
    def __init__(self, *a, **k): pass
    def read(self): return (0, 0, False)

touchscreen.TouchScreen = _FakeTS

display = types.ModuleType('maix.display')
display.Display = lambda *a, **k: types.SimpleNamespace(show=lambda img, **kw: None, width=lambda: 640, height=lambda: 480)

# 注意：MaixPy 里 pinmap 的真身在 maix.peripheral.pinmap，再由 maix 顶层 re-export
peripheral = types.ModuleType('maix.peripheral')
peripheral.pinmap = pinmap

for name, mod in (('uart', uart), ('err', err), ('app', app), ('time', time_mod),
                  ('image', image), ('camera', camera), ('display', display), ('touchscreen', touchscreen),
                  ('pinmap', pinmap), ('peripheral', peripheral)):
    setattr(maix, name, mod)
    sys.modules['maix.' + name] = mod
sys.modules['maix'] = maix
sys.modules['maix.peripheral.pinmap'] = pinmap

# ---- 导入我的模块 ----
import config, protocol, vision, main
print('[OK] config  W,H =', config.W, config.H, '| UART', config.UART_DEV, config.UART_BAUDRATE)
print('[OK] protocol 帧打包:')
print('     物块帧 ', protocol.pack_wukuai(320, 240, '1', 40))
print('     未识别 ', protocol.pack_not_found())
print('     任务码 ', protocol.pack_task_code(1, 2, 3, 4, 5, 6))
print('     角度帧 ', protocol.pack_angle(-3.4))
print('[OK] vision ColorClassifier 实例化:', vision.ColorClassifier(config.TH_RED, config.TH_GREEN, config.TH_BLUE).codes)
print('[OK] main StabilityTracker:', main.StabilityTracker(3).stable_frames)

# 验证帧字节与 STM32 期望一致
pkt = protocol.VisionLink.__new__(protocol.VisionLink)
arr = [ord(c) for c in '987654321']
b = bytes([protocol.FRAME_HEAD] + arr[:9] + [protocol.FRAME_TAIL])
print('[OK] 就绪帧字节 =', b.hex(' '))
assert b.hex(' ').upper() == 'FF 39 38 37 36 35 34 33 32 31 FE', '帧格式不对！'
print('[OK] 与 STM32 状态机期望的帧格式一致')
