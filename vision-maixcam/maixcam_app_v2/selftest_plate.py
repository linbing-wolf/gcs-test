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
    """白色盘（235）+ 背景（150）+ 盘缘暗线（90）。圆心可在画面外（cy<0）。"""

    def __init__(self, cx, cy, r, W=640, H=480, fg=235, bg=150, edge=90,
                 visible=True):
        self.cx, self.cy, self.r = cx, cy, r
        self.W, self.H = W, H
        self.fg, self.bg, self.edge = fg, bg, edge
        self.visible = visible

    def width(self): return self.W
    def height(self): return self.H

    def get_pixel(self, x, y, *a):
        x, y = int(x), int(y)
        if not (0 <= x < self.W and 0 <= y < self.H):
            return [0]
        if not self.visible:
            return [self.bg]
        d = math.hypot(x - self.cx, y - self.cy)
        if abs(d - self.r) <= 2.0:
            return [self.edge]
        return [self.fg] if d <= self.r else [self.bg]

    def draw_line(self, *a, **k): pass
    def draw_rect(self, *a, **k): pass
    def draw_cross(self, *a, **k): pass
    def draw_circle(self, *a, **k): pass
    def draw_string(self, *a, **k): pass


class FakeArcCam:
    def __init__(self, cx=350.0, cy=-55.0, r=330.0, visible=True):
        self.cx, self.cy, self.r = cx, cy, r
        self.visible = visible

    def read(self):
        Clock.tick(60)
        return FakeArcImage(self.cx, self.cy, self.r, visible=self.visible)

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


# ---------------- 1) find_disc_arc ----------------
print("\n== find_disc_arc：合成下弧 → 恢复圆心/半径 ==")
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

# ---------------- 2) task_plate one-shot ----------------
print("\n== task_plate：one-shot（direct 去回读 / verify 回读）==")
config.ALIGN_ONE_SHOT = True
# --- direct：发完 MOVEXY 直接回动作前的值 ---
config.ALIGN_REPLY_MODE = "direct"
cam = FakeArcCam(cx=350.0, cy=-55.0, r=330.0)      # low=(350,275) → 目标 (311,267)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
parts = out.split(",")
check("返回 DISC 帧", out.startswith("DISC,"), out)
check("超差时发过 MOVEXY", any(s.startswith("MOVEXY,") for s in link.sent),
      str(link.sent))
check("DISC 带 5 个字段", len(parts) == 5, out)
check("direct：回动作前的值 (x=350, dx=39)",
      len(parts) == 5 and int(parts[1]) == 350 and int(parts[3]) == 39, out)

# --- verify：回读后用调整后的值 ---
config.ALIGN_REPLY_MODE = "verify"
cam = FakeArcCam(cx=350.0, cy=-55.0, r=330.0)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
parts = out.split(",")
check("verify：回调整后的值（≈325）",
      len(parts) == 5 and abs(int(parts[1]) - 325) <= 6, out)

# ---------------- 3) task_plate 闭环收敛 ----------------
print("\n== task_plate：闭环（多轮）==")
config.ALIGN_ONE_SHOT = False
config.PLATE_MAX_ITER = 8
cam = FakeArcCam(cx=352.0, cy=-58.0, r=330.0)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
parts = out.split(",")
if out.startswith("DISC,"):
    x, y, dx = int(parts[1]), int(parts[2]), int(parts[3])
    check("收敛到容差内", abs(dx) <= config.PLATE_TOL_X and
          abs(y - config.PLATE_TARGET_Y) <= config.PLATE_TOL_Y,
          "low=(%d,%d) dx=%d" % (x, y, dx))
    check("发出过 MOVEXY", any(s.startswith("MOVEXY,") for s in link.sent))
else:
    check("闭环返回 DISC", False, out)

# ---------------- 4) 画面稳定判定 ----------------
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

# ---------------- 5) 看不到下弧 → NONE ----------------
print("\n== task_plate：看不到下弧 ==")
config.ALIGN_ONE_SHOT = True
cam = FakeArcCam(visible=False)
link = ArcLink(cam)
out = frame_str(tasks.task_plate(cam, None, p=P, ctl=tasks.TaskCtl(link)))
check("回 NONE,H", out == "NONE,H", out)

print()
if FAILED:
    print("[FAILED] %d 项未通过: %s" % (len(FAILED), FAILED))
    sys.exit(1)
print("[ALL PASS] 物料盘自检通过")