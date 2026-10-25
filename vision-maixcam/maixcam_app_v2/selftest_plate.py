# -*- coding: utf-8 -*-
"""selftest_plate.py —— 物料盘下弧检测 + 对齐任务自检（PC 上跑，不进发布包）

覆盖：
  1) vision.find_disc_arc：合成「只有下弧可见」的盘（圆心在画面外），验证能恢复圆心/半径
  2) tasks.task_plate one-shot：超差时发 MOVEXY，回读后回 DISC,x,y,dx,1
  3) tasks.task_plate 闭环：多轮微调后收敛到目标最低点容差内
  4) 看不到下弧 → 连续 2 帧回 NONE,H

用法：python selftest_plate.py
"""

import os
import sys
import types
import math

APP = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, APP)

# ---------------- 假 maix 包 ----------------
maix = types.ModuleType("maix")
image = types.ModuleType("maix.image")
for n in ("COLOR_RED", "COLOR_GREEN", "COLOR_BLUE", "COLOR_WHITE",
          "COLOR_YELLOW", "COLOR_PURPLE", "COLOR_GRAY"):
    setattr(image, n, n.lower())
image.Image = type("Image", (), {})
app = types.ModuleType("maix.app")
app.need_exit = lambda: False
app.set_exit_flag = lambda f: None
uart = types.ModuleType("maix.uart")
uart.UART = type("UART", (), {"__init__": lambda s, *a, **k: None,
                              "write": lambda s, b: len(b),
                              "read": lambda s, *a: b"",
                              "close": lambda s: None})
uart.list_devices = lambda: ["/dev/ttyS4"]
pinmap = types.ModuleType("maix.pinmap")
pinmap.set_pin_function = lambda *a, **k: 0
pinmap.get_pin_functions = lambda *a, **k: []
maix.image = image
maix.app = app
maix.uart = uart
maix.pinmap = pinmap
sys.modules.update({"maix": maix, "maix.image": image, "maix.app": app,
                    "maix.uart": uart, "maix.pinmap": pinmap})

# ---------------- 可控时钟（每读一帧推进一次）----------------
class Clock:
    t = 0
    @classmethod
    def tick(cls, ms=60):
        cls.t += ms
timing = types.ModuleType("timing")
timing.ticks_ms = lambda: Clock.t
timing.sleep_ms = lambda ms: Clock.tick(max(0, int(ms)))
sys.modules["timing"] = timing


# ---------------- 合成物料盘下弧图 ----------------
class FakeArcImage:
    """白色盘（235）+ 背景（150）+ 盘缘暗线（90）。圆心可在画面外（cy<0）。

    kb 为「竖直/水平半轴比」：kb<1 时盘缘是透视椭圆（水平长轴 r、竖直短轴 r·kb），
    用来验证 H 的椭圆模型；kb=1.0 即正圆（旧行为）。
    """

    def __init__(self, cx, cy, r, W=640, H=480, fg=235, bg=150, edge=90,
                 visible=True, kb=1.0):
        self.cx, self.cy, self.r = cx, cy, r
        self.W, self.H = W, H
        self.fg, self.bg, self.edge = fg, bg, edge
        self.visible = visible
        self.kb = kb

    def width(self): return self.W
    def height(self): return self.H

    def get_pixel(self, x, y, *a):
        x, y = int(x), int(y)
        if not (0 <= x < self.W and 0 <= y < self.H):
            return [0]
        if not self.visible:
            return [self.bg]
        d = math.hypot(x - self.cx, (y - self.cy) / self.kb)
        if abs(d - self.r) <= 2.0:
            return [self.edge]
        return [self.fg] if d <= self.r else [self.bg]

    def draw_line(self, *a, **k): pass
    def draw_rect(self, *a, **k): pass
    def draw_cross(self, *a, **k): pass
    def draw_circle(self, *a, **k): pass
    def draw_ellipse(self, *a, **k): pass
    def draw_string(self, *a, **k): pass

    def save(self, path, *a, **k):
        """假 JPEG 落盘（给 disc_debug 抓图用）。"""
        with open(path, "wb") as f:
            f.write(b"\xff\xd8fake-jpeg")


class FakeArcCam:
    def __init__(self, cx=350.0, cy=-55.0, r=330.0, visible=True, kb=1.0):
        self.cx, self.cy, self.r = cx, cy, r
        self.visible = visible
        self.kb = kb

    def read(self):
        Clock.tick(60)
        return FakeArcImage(self.cx, self.cy, self.r, visible=self.visible,
                            kb=self.kb)

    def skip_frames(self, n): pass


class ArcLink:
    """假下位机：收到 MOVEXY 把盘往反方向挪（模拟机构真的动了）。"""

    def __init__(self, cam):
        self.cam = cam
        self.sent = []

    def poll_command(self):
        return None

    def send_payload(self, p):
        self.sent.append(p)
        if p.startswith("MOVEXY,"):
            try:
                _, sx, sy = p.split(",")
                self.cam.cx -= int(sx)
                self.cam.cy -= int(sy)
            except Exception:
                pass


import config  # noqa: E402
config.PLATE_DEBUG_SAVE = False
import precision  # noqa: E402
import tasks  # noqa: E402
import vision  # noqa: E402

FAILED = []
P = precision.Pipeline(config.W, config.H, config.W, config.H)


def check(name, cond, extra=""):
    print("  [%s] %s %s" % ("OK" if cond else "!!", name, extra))
    if not cond:
        FAILED.append(name)


def frame_str(b):
    if isinstance(b, bytes):
        b = b.decode("utf-8", "replace")
    return str(b).strip("\r\n")


def _post_move_x(x0=350.0):
    """预测 one-shot 之后最低点 x（假机构 ArcLink 按 −step 移动）。

    与 config 解耦：改 PLATE_TARGET_X / PLATE_GAIN 后这里自动跟着变。
    """
    dx0 = x0 - config.PLATE_TARGET_X + config.PLATE_DX_OFFSET
    step = tasks._axis_step(dx0, config.PLATE_GAIN, config.ALIGN_STEP_MIN, 0,
                            1.0, config.PLATE_INVERT_X)
    return int(round(x0 - step))


# ---------------- 1) find_disc_arc（圆模型）----------------
print("\n== find_disc_arc：合成正圆下弧 → 圆模型恢复圆心/半径 ==")
config.PLATE_ARC_MODEL = "circle"
for (cx0, cy0, r0) in [(311.0, -55.0, 330.0), (350.0, -90.0, 345.0),
                       (270.0, -40.0, 320.0)]:
    meta = {}
    res = vision.find_disc_arc(FakeArcImage(cx0, cy0, r0), cx0, cy0, r0, meta=meta)
    ok = res is not None
    if ok:
        cx, cy, r = res
        err = max(abs(cx - cx0), abs(cy - cy0), abs(r - r0))
        check("恢复 (%.0f,%.0f,%.0f)" % (cx0, cy0, r0),
              err <= 3.0, "→ (%.1f,%.1f,%.1f) err=%.2f" % (cx, cy, r, err))
    else:
        check("恢复 (%.0f,%.0f,%.0f)" % (cx0, cy0, r0), False, "返回 None: %s" % meta)

# ---------------- 2) 透视椭圆下弧：椭圆模型恢复最低点 + k ----------------
print("\n== find_disc_arc：合成透视椭圆下弧（椭圆模型）==")
config.PLATE_ARC_MODEL = "ellipse"
for (cx0, cy0, r0, kb0) in [(310.0, -55.0, 330.0, 0.90),
                            (330.0, -70.0, 320.0, 0.93),
                            (290.0, -60.0, 325.0, 0.88),
                            (310.0, -55.0, 330.0, 1.00)]:
    true_low_y = cy0 + r0 * kb0
    meta = {}
    res = vision.find_disc_arc(FakeArcImage(cx0, cy0, r0, kb=kb0),
                               cx0, cy0, r0, meta=meta)
    if res is None:
        check("椭圆 kb=%.2f 能检到" % kb0, False, str(meta))
        continue
    cx, cy, b = res
    err = math.hypot(cx - cx0, (cy + b) - true_low_y)
    check("椭圆 kb=%.2f 最低点 err=%.1fpx" % (kb0, err), err <= 2.0,
          "low=(%.1f,%.1f) 真值 y=%.1f rms=%s"
          % (cx, cy + b, true_low_y, meta.get("rms")))
    check("椭圆 kb=%.2f 估出的 k 准 (%.2f)" % (kb0, meta.get("k", 0)),
          abs(float(meta.get("k", 0)) - kb0) <= 0.03,
          "k_est=%s 真值=%.2f" % (meta.get("k"), kb0))
    check("椭圆 kb=%.2f 返回竖直半轴 b(=最低点用)" % kb0,
          abs(b - r0 * kb0) <= 12.0, "b=%.1f 真值=%.1f" % (b, r0 * kb0))

# ---------------- 3) task_plate one-shot ----------------
print("\n== task_plate：one-shot（去回读，直接回动作前偏差）==")
config.ALIGN_ONE_SHOT = True
# --- direct：发完 MOVEXY 直接回动作前的值 ---
cam = FakeArcCam(cx=350.0, cy=-55.0, r=330.0)      # low=(350,275) → 目标 (311,267)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
parts = out.split(",")
check("返回 DISC 帧", out.startswith("DISC,"), out)
check("超差时发过 MOVEXY", any(s.startswith("MOVEXY,") for s in link.sent),
      str(link.sent))
check("DISC 带 5 个字段", len(parts) == 5, out)
_exp_dx = int(round(350.0 - config.PLATE_TARGET_X + config.PLATE_DX_OFFSET))
check("direct：回动作前的值 (x=350, dx=350−TARGET_X+DX_OFFSET=%d)" % _exp_dx,
      len(parts) == 5 and int(parts[1]) == 350 and int(parts[3]) == _exp_dx, out)


# ---------------- 4) task_plate 闭环收敛 ----------------
print("\n== task_plate：闭环（多轮，带透视椭圆）==")
config.ALIGN_ONE_SHOT = False
config.PLATE_MAX_ITER = 12
cam = FakeArcCam(cx=352.0, cy=-58.0, r=330.0, kb=0.92)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
parts = out.split(",")
if out.startswith("DISC,"):
    x, y, dx = int(parts[1]), int(parts[2]), int(parts[3])
    tx_eff = config.PLATE_TARGET_X - config.PLATE_DX_OFFSET
    ty_eff = config.PLATE_TARGET_Y - config.PLATE_DY_OFFSET
    check("收敛到容差内", abs(dx) <= config.PLATE_TOL_X and
          abs(y - ty_eff) <= config.PLATE_TOL_Y,
          "low=(%d,%d) dx=%d 目标=(%d,%d)" % (x, y, dx, tx_eff, ty_eff))
    check("发出过 MOVEXY", any(s.startswith("MOVEXY,") for s in link.sent))
else:
    check("闭环返回 DISC", False, out)

# ---------------- 4b) H 调试抓图落盘 ----------------
print("\n== H 调试抓图（PLATE_DEBUG_SAVE）落盘 ==")
import tempfile
import shutil
import json

_dbgdir = os.path.join(tempfile.gettempdir(), "selftest_plate_debug")
shutil.rmtree(_dbgdir, ignore_errors=True)
_old_save = config.PLATE_DEBUG_SAVE
_old_dbgdir = config.PLATE_DEBUG_DIR
config.PLATE_DEBUG_SAVE = True
config.PLATE_DEBUG_DIR = _dbgdir
config.ALIGN_ONE_SHOT = True
cam = FakeArcCam(cx=340.0, cy=-58.0, r=330.0, kb=0.92)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(ArcLink(cam))))
subs = os.listdir(_dbgdir) if os.path.isdir(_dbgdir) else []
check("建了任务目录", len(subs) == 1, str(subs))
if len(subs) == 1:
    d = os.path.join(_dbgdir, subs[0])
    meta_p = os.path.join(d, "meta.jsonl")
    sum_p = os.path.join(d, "summary.json")
    jpgs = [f for f in os.listdir(d) if f.endswith(".jpg")]
    check("有非空 meta.jsonl", os.path.exists(meta_p) and os.path.getsize(meta_p) > 0)
    check("有 summary.json", os.path.exists(sum_p))
    check("存了关键帧 jpg", len(jpgs) >= 1, str(jpgs))
    try:
        with open(sum_p, encoding="utf-8") as f:
            summ = json.load(f)
        check("summary.result = 本次回复帧", summ.get("result") == out,
              "%s vs %s" % (summ.get("result"), out))
    except Exception as e:
        check("summary.json 可解析", False, str(e))
config.PLATE_DEBUG_SAVE = _old_save
config.PLATE_DEBUG_DIR = _old_dbgdir
shutil.rmtree(_dbgdir, ignore_errors=True)

# ---------------- 4c) 运行时可调偏差（PLATE_BIAS_FILE）----------------
print("\n== _plate_bias：运行时覆盖 > config ==")
_biasf = os.path.join(tempfile.gettempdir(), "selftest_plate_bias")
try:
    os.remove(_biasf)
except OSError:
    pass
_old_bias = (config.PLATE_BIAS_FILE, config.PLATE_BIAS_CACHE_MS)
config.PLATE_BIAS_FILE = _biasf
config.PLATE_BIAS_CACHE_MS = 0        # 关缓存，改完立即生效

b = tasks._plate_bias()
check("无文件 → 用 config 默认",
      b["PLATE_TARGET_Y"] == config.PLATE_TARGET_Y and
      b["PLATE_DY_OFFSET"] == config.PLATE_DY_OFFSET, str(b))

with open(_biasf, "w") as f:
    f.write("# 现场调落点\nPLATE_DY_OFFSET=5\nPLATE_TARGET_Y=343.5\n")
b = tasks._plate_bias()
check("覆盖 PLATE_DY_OFFSET=5", b["PLATE_DY_OFFSET"] == 5, str(b))
check("覆盖 PLATE_TARGET_Y=343.5",
      abs(b["PLATE_TARGET_Y"] - 343.5) < 1e-6, str(b))
check("未写的项仍用 config", b["PLATE_TARGET_X"] == config.PLATE_TARGET_X, str(b))
check("偏置按 int 处理", isinstance(b["PLATE_DY_OFFSET"], int),
      str(type(b["PLATE_DY_OFFSET"])))

with open(_biasf, "w") as f:
    f.write("PLATE_DY_OFFSET=abc\n未知键=1\nPLATE_DX_OFFSET = 7  # 注释\n")
b = tasks._plate_bias()
check("非法数值行忽略（仍用 config）",
      b["PLATE_DY_OFFSET"] == config.PLATE_DY_OFFSET, str(b))
check("'=' 两侧空格/行内注释可用", b["PLATE_DX_OFFSET"] == 7, str(b))

os.remove(_biasf)
b = tasks._plate_bias()
check("删除文件 → 恢复 config",
      b["PLATE_DY_OFFSET"] == config.PLATE_DY_OFFSET, str(b))

# E2E：覆盖文件把落点抬高 → H 发的 MOVEXY dy 随之变化
def _last_movey(lk):
    for s in reversed(lk.sent):
        if s.startswith("MOVEXY,"):
            return int(s.split(",")[2])
    return None

config.ALIGN_ONE_SHOT = True
cam = FakeArcCam(cx=340.0, cy=-58.0, r=330.0, kb=0.92)
lk = ArcLink(cam)
frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(lk)))
dy0 = _last_movey(lk)
with open(_biasf, "w") as f:
    f.write("PLATE_DY_OFFSET=%d\n" % (config.PLATE_DY_OFFSET + 30))
cam = FakeArcCam(cx=340.0, cy=-58.0, r=330.0, kb=0.92)
lk2 = ArcLink(cam)
frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(lk2)))
dy1 = _last_movey(lk2)
want = round(30 * float(getattr(config, "PLATE_GAIN", 0.65)))
check("覆盖生效：MOVEXY dy 变化 ≈30×gain",
      dy0 is not None and dy1 is not None and abs((dy1 - dy0) - want) <= 2,
      "dy %s → %s（期望 +%d）" % (dy0, dy1, want))

os.remove(_biasf)
config.PLATE_BIAS_FILE, config.PLATE_BIAS_CACHE_MS = _old_bias

# ---------------- 5) 画面稳定判定 ----------------
print("\n== _wait_scene_settle：画面稳定后提前开始 ==")


class _SettleCam(FakeArcCam):
    """前 move_frames 帧画面在变（cx 每帧 +6），之后固定不动。"""

    def __init__(self, move_frames=5, **k):
        super().__init__(**k)
        self.n = 0
        self.move_frames = move_frames

    def read(self):
        Clock.tick(60)
        self.n += 1
        cx = self.cx + (6 * self.n if self.n <= self.move_frames else 6 * self.move_frames)
        return FakeArcImage(cx, self.cy, self.r, visible=self.visible)


config.SCENE_SETTLE_ON = True
config.SCENE_SETTLE_FRAMES = 2
config.SCENE_SETTLE_DIFF = 2.0
config.SCENE_SETTLE_MIN_MS = 0
cam = _SettleCam(move_frames=5, cx=300.0)
t0 = Clock.t
ok = tasks._wait_scene_settle(cam, None, None, 5000, "T")
el = Clock.t - t0
check("返回 True", ok, "elapsed=%dms" % el)
check("画面稳定后提前返回（未用满 5000ms 上限）", el < 5000, "elapsed=%dms" % el)

config.SCENE_SETTLE_ON = False
cam2 = _SettleCam(move_frames=5, cx=300.0)
t1 = Clock.t
ok2 = tasks._wait_scene_settle(cam2, None, None, 400, "T")
el2 = Clock.t - t1
check("SCENE_SETTLE_ON=False 退回固定等待（≥上限）", ok2 and el2 >= 400,
      "elapsed=%dms" % el2)
config.SCENE_SETTLE_ON = True

# ---------------- 6) 看不到下弧 → NONE ----------------
print("\n== task_plate：看不到下弧 ==")
config.ALIGN_ONE_SHOT = True
cam = FakeArcCam(visible=False)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
check("回 NONE,H", out == "NONE,H", out)


# ---------------- 8) 假圆防护：粗定位 + 质量闸门 ----------------
print("\n== 假圆防护（粗定位 + 质量闸门）==")
_old_g = (config.PLATE_MAX_RMS, config.PLATE_QUALITY_GATE,
          config.PLATE_COARSE_LOCATE)

# (a) 质量闸门：把残差上限压到极小 → 拟合判为低置信
config.PLATE_COARSE_LOCATE = False
config.PLATE_QUALITY_GATE = True
_old_mode = config.PLATE_GATE_MODE
config.PLATE_MAX_RMS = 0.001
#   默认 report 模式：只标记，仍返回最佳猜测（现场要求「不准确也要执行完」）
config.PLATE_GATE_MODE = "report"
m = {}
r = vision.find_disc_arc(FakeArcImage(310.0, -55.0, 330.0), 310.0, -55.0, 330.0,
                         meta=m)
check("闸门 report：低置信也返回结果（并标记 low_conf）",
      r is not None and m.get("low_conf") is True, str(m.get("reason")))
#   reject 模式：不可信就回 None（严格，不动作）
config.PLATE_GATE_MODE = "reject"
m = {}
r = vision.find_disc_arc(FakeArcImage(310.0, -55.0, 330.0), 310.0, -55.0, 330.0,
                         meta=m)
check("闸门 reject：残差超限 → 判不可信回 None",
      r is None and m.get("rejected") is True, str(m.get("reason")))
config.PLATE_GATE_MODE = _old_mode
config.PLATE_MAX_RMS = 4.0
m = {}
r = vision.find_disc_arc(FakeArcImage(310.0, -55.0, 330.0), 310.0, -55.0, 330.0,
                         meta=m)
check("阈值正常时同一张图判为可信（不标 low_conf）",
      r is not None and not m.get("low_conf"), str(m.get("reason")))

# (b) 粗定位：盘横向偏移 250px（远超搜索带 ±60）
_img_far = FakeArcImage(560.0, -55.0, 330.0, kb=0.92)
_true_low_y = -55.0 + 330.0 * 0.92
_cp = vision.coarse_disc_prior(_img_far)
check("粗定位：横向偏 250px 也能找到盘（且比配置先验更接近真值）",
      _cp is not None and
      math.hypot(_cp[0] - 560.0, _cp[1] + 55.0) <
      math.hypot(310.0 - 560.0, -32.0 + 55.0),
      "coarse=%s（真值 560,-55；配置先验 310,-32）" % (_cp,))
config.PLATE_COARSE_LOCATE = True
config.PLATE_QUALITY_GATE = True
config.PLATE_MAX_RMS = 4.0
m2 = {}
r_co = vision.find_disc_arc(_img_far, 310.0, -32.0, 330.0, meta=m2)
if r_co is not None:
    _err = math.hypot(r_co[0] - 560.0, (r_co[1] + r_co[2]) - _true_low_y)
    check("横向偏移 250px 仍能拟合到真盘（最低点 err=%.1fpx）" % _err, _err <= 6.0,
          "low=(%.0f,%.0f) prior=%s rms=%s"
          % (r_co[0], r_co[1] + r_co[2], m2.get("prior_used"), m2.get("rms")))
else:
    check("横向偏移 250px 仍能拟合到真盘", False, str(m2.get("reason")))
config.PLATE_COARSE_LOCATE = False
m3 = {}
vision.find_disc_arc(_img_far, 310.0, -32.0, 330.0, meta=m3)
check("关掉粗定位后不再有 coarse 记录", "coarse" not in m3, str(m3.get("coarse")))

# (c) 几何硬校验：半径允许范围压到极小 → 任何模式都作废（即使 report 也要拦）
_old_geom = (config.PLATE_GEOM_B_RATIO_MAX, config.PLATE_GEOM_CHECK,
             config.PLATE_GATE_MODE)
config.PLATE_GEOM_CHECK = True
config.PLATE_GATE_MODE = "report"
config.PLATE_GEOM_B_RATIO_MAX = 0.05
m = {}
r = vision.find_disc_arc(FakeArcImage(310.0, -55.0, 330.0), 310.0, -55.0, 330.0,
                         meta=m)
check("几何校验：半径越界 → report 模式也作废（不回假圆）",
      r is None and m.get("rejected") is True, str(m.get("reason")))
(config.PLATE_GEOM_B_RATIO_MAX, config.PLATE_GEOM_CHECK,
 config.PLATE_GATE_MODE) = _old_geom

(config.PLATE_MAX_RMS, config.PLATE_QUALITY_GATE,
 config.PLATE_COARSE_LOCATE) = _old_g

print()
if FAILED:
    print("[FAILED] %d 项未通过: %s" % (len(FAILED), FAILED))
    sys.exit(1)
print("[ALL PASS] 物料盘自检通过")