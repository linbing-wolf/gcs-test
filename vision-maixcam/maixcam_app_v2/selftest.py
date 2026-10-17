# -*- coding: utf-8 -*-
"""selftest.py —— 本地自检：用假 maix 模块导入本项目 + 校验帧协议（在 PC 上跑）"""
import sys, os, types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---------------- 造假 maix 包 ----------------
maix = types.ModuleType("maix")

class _FakeUART:
    def __init__(self, dev, baud, *a, **k):
        self.dev, self.baud, self._rx = dev, baud, b""
    def write(self, data):
        self._rx += bytes(data)
        return len(data)
    def read(self, *a, **k):
        d, self._rx = self._rx, b""
        return d
    def close(self): pass

uart = types.ModuleType("maix.uart")
uart.UART = _FakeUART
uart.list_devices = lambda: ["/dev/ttyS2"]

pinmap = types.ModuleType("maix.peripheral.pinmap")
class _FakeErr:
    def __init__(self, code=0, m=""): self._c, self._m = code, m
    def code(self): return self._c
    def msg(self): return self._m
_FUNCS = {"B0": ["UART2_TX", "GPIO"], "B1": ["UART2_RX", "GPIO"]}
pinmap.set_pin_function = lambda pin, func: _FakeErr(0, "ok")
pinmap.get_pin_functions = lambda pin: _FUNCS.get(pin, [])

app = types.ModuleType("maix.app")
app.need_exit = lambda: True
app.set_exit_flag = lambda f: None

time_mod = types.ModuleType("maix.time")
time_mod.ticks_ms = lambda: 0

class _FakeBlob:
    def __init__(self, *a): pass
    def x(self): return 0
    def y(self): return 0
    def w(self): return 10
    def h(self): return 10
    def cxf(self): return 5
    def cyf(self): return 5
    def pixels(self): return 100
    def code(self): return 0
    def roundness(self): return 1.0

class _FakeImg:
    def find_blobs(self, *a, **k): return []
    def find_lines(self, *a, **k): return []
    def find_circles(self, *a, **k): return []
    def find_qrcodes(self, *a, **k): return []
    def draw_rect(self, *a, **k): pass
    def draw_cross(self, *a, **k): pass
    def draw_circle(self, *a, **k): pass
    def draw_string(self, *a, **k): pass

image = types.ModuleType("maix.image")
image.Image = _FakeImg
image.Blob = _FakeBlob
for n in ("COLOR_RED", "COLOR_GREEN", "COLOR_BLUE", "COLOR_WHITE",
          "COLOR_YELLOW", "COLOR_PURPLE", "COLOR_GRAY"):
    setattr(image, n, n.lower())
image.Fit = type("Fit", (), {"FIT_CONTAIN": 0, "FIT_FILL": 1})
image.resize_map_pos = lambda wi, hi, wo, ho, fit, x, y: [x, y]

class _FakeCam:
    def __init__(self, w, h, fps=0): self.w, self.h = w, h
    def skip_frames(self, n): pass
    def read(self): return _FakeImg()

camera = types.ModuleType("maix.camera")
camera.Camera = _FakeCam

touchscreen = types.ModuleType("maix.touchscreen")
class _FakeTS:
    def __init__(self, *a, **k): pass
    def read(self): return (0, 0, False)
touchscreen.TouchScreen = _FakeTS

display = types.ModuleType("maix.display")
display.Display = lambda *a, **k: types.SimpleNamespace(
    show=lambda img, **kw: None, width=lambda: 640, height=lambda: 480)

peripheral = types.ModuleType("maix.peripheral")
peripheral.pinmap = pinmap

for name, mod in (("uart", uart), ("err", types.ModuleType("maix.err")), ("app", app),
                  ("time", time_mod), ("image", image), ("camera", camera),
                  ("display", display), ("touchscreen", touchscreen),
                  ("pinmap", pinmap), ("peripheral", peripheral)):
    setattr(maix, name, mod)
    sys.modules["maix." + name] = mod
sys.modules["maix"] = maix
sys.modules["maix.peripheral.pinmap"] = pinmap

# ---------------- 导入项目模块 ----------------
import config, protocol, vision, tasks, main, precision, ai_model
print("[OK] import config/protocol/vision/tasks/main/precision/ai_model")

# ---------------- 帧编解码往返 ----------------
f = protocol.encode_frame("TASK,134,314")
print("[OK] 帧:", f.hex(" "))
assert f[0] == 0xAA and f[1] == 0x55 and f[-1] == 0x0D
assert f[2] == len("TASK,134,314")
p = protocol.FrameParser()
got = p.feed(f)
assert got == [b"TASK,134,314"], got
print("[OK] 单帧往返")

# ---------------- 粘连 + 半帧 + 噪声 ----------------
stream = b"\x00\x01" + protocol.encode_frame("A") + f + protocol.encode_frame("Z")[0:4]
got = p.feed(stream)
assert got == [b"A", b"TASK,134,314"], got
got2 = p.feed(protocol.encode_frame("Z")[4:])
assert got2 == [b"Z"], got2
print("[OK] 粘连/半帧/噪声处理")

# ---------------- CRC 检错 ----------------
bad = bytearray(f); bad[5] ^= 0xFF
assert p.feed(bytes(bad)) == []
print("[OK] CRC 检错")

# ---------------- 命令解析 ----------------
assert protocol.parse_command("C,1") == ("C", "1")
assert protocol.parse_command("D") == ("D", "")
assert protocol.parse_command("T,134+123+314+231") == ("T", "134+123+314+231")
print("[OK] parse_command")

# ---------------- 任务码解析 ----------------
r = protocol.parse_task_code("134+123+314+231")
assert r and r["g1"] == "134" and r["g3"] == "314", r
assert protocol.parse_task_code("123+456") is None
assert protocol.parse_task_code("134+123+314+23x") is None
print("[OK] parse_task_code:", r["g1"], r["g3"])

# ---------------- 回复包 ----------------
assert protocol.pack_task("134", "314") == "TASK,134,314"
assert protocol.pack_pick("1", 320, 240) == "PICK,1,320,240"
assert protocol.pack_disc(300, 200, -20, 0) == "DISC,300,200,-20,0"
assert protocol.pack_disc(320, 240, 0, 1) == "DISC,320,240,0,1"
assert protocol.pack_move(17) == "MOVE,17"
assert protocol.pack_move(-23) == "MOVE,-23"
assert protocol.pack_stack(300, 200) == "STACK,300,200"
assert protocol.pack_start(1, 320, 240, 120, 80) == "START,1,320,240,120,80"
assert protocol.pack_obstacle(320, 240, 60, 90, 150) == "OBST,320,240,60,90,150"
assert protocol.pack_clear() == "CLEAR"
assert protocol.CMD_FORWARD == "G"
print("[OK] pack_* 构造函数 (含障碍物 OBST/CLEAR)")

# 障碍物识别：假图无 blob -> None
assert vision.find_obstacle(_FakeImg()) is None
# 测距：未标定返回 0；已标定返回正数
_d = vision.estimate_distance_mm(60)
assert (_d == 0) if not config.UNDISTORT_ON else (_d > 0), _d
print("[OK] find_obstacle / estimate_distance_mm")

# ---------------- 精度库 ----------------
# 坐标缩放
sc = precision.CoordScaler(1280, 960, 640, 480)
assert sc.map(640, 480) == (320.0, 240.0), sc.map(640, 480)
print("[OK] CoordScaler")

# 圆拟合：造一个半径 50、圆心 (100,80) 的圆
import math
pts = [(100 + 50 * math.cos(t), 80 + 50 * math.sin(t))
       for t in [i * 2 * math.pi / 60 for i in range(60)]]
xc, yc, r = precision.fit_circle(pts)
assert abs(xc - 100) < 1e-6 and abs(yc - 80) < 1e-6 and abs(r - 50) < 1e-6, (xc, yc, r)
print("[OK] fit_circle: (%.3f, %.3f) r=%.3f" % (xc, yc, r))

# 畸变：正向扭曲后再反畸变，应回到原点附近
dm = precision.DistortionModel(k1=-0.1, k2=0.01, p1=0.001, p2=-0.001,
                               fx=600, fy=600, cx=320, cy=240)
def _forward(dm, u, v):
    x, y = (u - dm.cx) / dm.fx, (v - dm.cy) / dm.fy
    r2 = x * x + y * y
    rad = 1 + dm.k1 * r2 + dm.k2 * r2 * r2
    xd = x * rad + 2 * dm.p1 * x * y + dm.p2 * (r2 + 2 * x * x)
    yd = y * rad + dm.p1 * (r2 + 2 * y * y) + 2 * dm.p2 * x * y
    return xd * dm.fx + dm.cx, yd * dm.fy + dm.cy
u0, v0 = 420, 300
ud, vd = _forward(dm, u0, v0)
ur, vr = dm.undistort_point(ud, vd)
assert abs(ur - u0) < 0.01 and abs(vr - v0) < 0.01, (ur, vr)
print("[OK] 畸变反解: (%.4f, %.4f) -> (%.4f, %.4f)" % (u0, v0, ur, vr))

# 单应：单位阵应原样返回
hm = precision.Homography([1, 0, 0, 0, 1, 0, 0, 0, 1])
assert hm.apply(123, 45) == (123.0, 45.0)
print("[OK] Homography")

# 多帧平均
cf = precision.CenterFilter(3)
for p in [(10, 10), (12, 12), (14, 14)]:
    cf.push(*p)
cx, cy = cf.push(16, 16)
assert abs(cx - 14) < 1e-9 and abs(cy - 14) < 1e-9, (cx, cy)
print("[OK] CenterFilter 多帧平均")

# ---------------- 坐标流水线 Pipeline ----------------
# 恒等映射（PRECISION_ON=False 时的等价形态）
pi = precision.Pipeline(config.W, config.H, config.W, config.H)
assert pi.roi([10, 20, 30, 40]) == [10, 20, 30, 40]
assert pi.point(123, 45) == (123, 45)
assert pi.size(30, 40) == (30, 40)
assert pi.area(300) == 300
assert not pi.active
print("[OK] Pipeline 恒等映射")

# 检测域 1280x960 → 对外 640x480：ROI/坐标/尺寸折半，面积阈值 ×4
p2 = precision.Pipeline(1280, 960, 640, 480, dist=None, filter_n=0)
assert p2.roi([100, 100, 200, 160]) == [200, 200, 400, 320], p2.roi([100, 100, 200, 160])
assert p2.point(640, 480) == (320, 240), p2.point(640, 480)
assert p2.size(200, 160) == (100, 80), p2.size(200, 160)
assert p2.area(300) == 1200, p2.area(300)
assert p2.active
print("[OK] Pipeline 高分辨率检测：ROI/坐标折半、面积阈值 ×4")

# 反畸变：检测域放大 2× 时 CAM_K 必须同步放大，否则点会被推错
_d2 = precision.DistortionModel.from_K_dist(
    [config.CAM_K[0] * 2, 0, config.CAM_K[2] * 2,
     0, config.CAM_K[4] * 2, config.CAM_K[5] * 2, 0, 0, 1], config.CAM_DIST)
assert abs(_d2.fx - config.CAM_K[0] * 2) < 1e-9 and abs(_d2.cx - config.CAM_K[2] * 2) < 1e-9
_p3 = precision.Pipeline(1280, 960, 640, 480, dist=_d2, filter_n=0)
# 光心处无径向畸变 → 反畸变后应仍在光心
_ux, _uy = _p3.point(_d2.cx, _d2.cy)
assert abs(_ux - config.CAM_K[2]) <= 1 and abs(_uy - config.CAM_K[5]) <= 1, (_ux, _uy)
print("[OK] Pipeline 反畸变：K 按检测域缩放，光心不漂移")

# 多帧平均：3 帧后应等于均值；reset 后重新计
p4 = precision.Pipeline(640, 480, 640, 480, dist=None, filter_n=3)
p4.smooth("T", 10, 20); p4.smooth("T", 20, 40)
_sx, _sy = p4.smooth("T", 30, 60)
assert (_sx, _sy) == (20, 40), (_sx, _sy)
p4.reset("T")
assert p4.smooth("T", 100, 200) == (100, 200)
print("[OK] Pipeline 多帧平均 + reset")

# 反畸变保护：位移过大 / 跑出画面 → 该点退回原坐标
class _FakeDist:
    def __init__(self, dx=0.0, dy=0.0):
        self.dx, self.dy = dx, dy

    def undistort_point(self, x, y):
        return x + self.dx, y + self.dy


pg = precision.Pipeline(640, 480, 640, 480, dist=_FakeDist(100), max_shift=48)
assert pg.point(320, 240) == (320, 240) and pg.dropped == 1, (pg.point(320, 240), pg.dropped)
pg2 = precision.Pipeline(640, 480, 640, 480, dist=_FakeDist(3), max_shift=48)
assert pg2.point(320, 240) == (323, 240), pg2.point(320, 240)
pg3 = precision.Pipeline(640, 480, 640, 480, dist=_FakeDist(200), max_shift=0)
assert pg3.point(600, 240) == (600, 240), pg3.point(600, 240)   # 越界保护（max_shift=0 也要挡）
print("[OK] Pipeline 反畸变保护：超限/越界退回原坐标")

# ---------------- 协议：BUSY / 颜色码校验 / 任务看门 ----------------
assert protocol.pack_busy() == "BUSY"
assert protocol.is_color_code("1") and protocol.is_color_code(" 4 ")
assert not protocol.is_color_code("0") and not protocol.is_color_code("9")
assert not protocol.is_color_code("")
print("[OK] pack_busy / is_color_code（只认 1~4）")


class _FakeLink:
    def __init__(self, cmds):
        self.cmds = list(cmds)
        self.sent = []

    def poll_command(self):
        return self.cmds.pop(0) if self.cmds else None

    def send_payload(self, p):
        self.sent.append(p)


# 任务中收到普通命令 → 回 BUSY，任务继续
lk = _FakeLink(["C,1"])
ctl = tasks.TaskCtl(lk)
assert ctl.tick() is True and lk.sent == ["BUSY"] and not ctl.reset_flag
# 任务中收到 R → 中断
lk2 = _FakeLink(["R"])
ctl2 = tasks.TaskCtl(lk2)
assert ctl2.tick() is False and ctl2.reset_flag and lk2.sent == []
# 没命令 → 继续
assert tasks.TaskCtl(_FakeLink([])).tick() is True
print("[OK] TaskCtl：普通命令回 BUSY / R 中断 / 空转继续")

# ---------------- 拆帧：半帧超时 ----------------
fp = protocol.FrameParser(frame_timeout_ms=200)
assert fp.feed(protocol.encode_frame("A")[:4]) == []      # 半帧
fp._last_ms -= 500                                         # 模拟 500ms 没新数据
out = fp.feed(protocol.encode_frame("D"))
assert [x.decode() for x in out] == ["D"], out
print("[OK] 半帧超时：残留被丢弃，后续帧正常解出")

# 正常分片（没有超时）仍能拼回
fp2 = protocol.FrameParser(frame_timeout_ms=200)
f = protocol.encode_frame("START,320,240,120,80")
assert fp2.feed(f[:5]) == []
assert [x.decode() for x in fp2.feed(f[5:])] == ["START,320,240,120,80"]
print("[OK] 分片帧正常拼接")

# ---------------- 任务码语义校验 ----------------
r = protocol.parse_task_code("134+123+314+231")
assert r and r["g1"] == "134" and r["g3"] == "314"
for bad in ("134+123+314", "134+123+314+23", "134+123+314+2312",
            "13a+123+314+231", "", "999+000+999+000", "134+123+014+231"):
    assert protocol.parse_task_code(bad) is None, bad
print("[OK] 任务码：格式 + G1/G3 颜色语义校验")

# ---------------- 启停区道路：灰色 + AI 双确认 ----------------
class _FakeAI:
    labels = ["background", "blue", "green", "other", "red", "yellow"]

    def __init__(self, idx, prob, ready=True):
        self.idx, self.prob, self._ready = idx, prob, ready

    def ready(self):
        return self._ready

    def classify_top1(self, img, roi=None):
        return self.idx, self.prob


_ok, _why = tasks._confirm_road(_FakeAI(3, 0.90), None, [0, 0, 10, 10])
assert _ok and "gray+ai" in _why, _why
_ok, _why = tasks._confirm_road(_FakeAI(0, 0.90), None, [0, 0, 10, 10])
assert not _ok, _why
_ok, _why = tasks._confirm_road(_FakeAI(3, 0.10), None, [0, 0, 10, 10])
assert not _ok, _why
_ok, _why = tasks._confirm_road(None, None, [0, 0, 10, 10])
assert _ok and "gray-only" in _why, _why
# ---------------- 命令分发：未知命令静默 / Z 回 OK ----------------
_sent = []
main.link.send_payload = lambda p: _sent.append(p)
main.dispatch("X", "1")
assert _sent == [], _sent
main.dispatch("Z", "")
assert _sent == ["OK"], _sent
print("[OK] 未知命令静默忽略；Z 回 OK 且不退出")

print("[OK] 道路双确认：AI=other → 有；AI=background/低置信 → 无；AI 关闭 → 退回灰色")

print("\n[ALL PASS] 本地自检通过")