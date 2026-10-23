# -*- coding: utf-8 -*-
"""selftest_tasks.py —— 任务层冒烟测试（PC 上跑，不进发布包）

用假 maix 模块真跑 tasks 的 5 个任务，验证：
  1) 每个任务都能返回合法帧（签名/参数正确、不抛异常）
  2) DET 1280×960 时回传坐标正好折半
  3) PRECISION_ON=False（恒等映射）时坐标不变
  4) 任务中收到 R → 立即回 READY；收到其它命令 → 回 BUSY 且任务继续
  5) 启停区区号：右侧道路（灰色 + AI 双确认）

用法：python selftest_tasks.py
"""

import os
import sys
import types

APP = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, APP)

# ----------------------------------------------------------------------
# 假 maix 包
# ----------------------------------------------------------------------
maix = types.ModuleType('maix')
image = types.ModuleType('maix.image')
image.COLOR_RED = 0xFF0000
image.COLOR_GREEN = 0x00FF00
image.COLOR_BLUE = 0x0000FF
image.COLOR_YELLOW = 0xFFFF00
image.Fit = type('Fit', (), {'FIT_CONTAIN': 1, 'FIT_FILL': 0, 'FIT_COVER': 2})
image.Format = type('Format', (), {'FMT_RGB888': 0})


class FakeBlob:
    def __init__(self, code, x, y, w, h):
        self._c, self._x, self._y, self._w, self._h = code, x, y, w, h

    def code(self): return self._c
    def x(self): return self._x
    def y(self): return self._y
    def w(self): return self._w
    def h(self): return self._h
    def pixels(self): return self._w * self._h
    def cxf(self): return self._x + self._w / 2.0
    def cyf(self): return self._y + self._h / 2.0
    def roundness(self): return 0.9


class FakeCircle:
    def __init__(self, x, y, r):
        self._x, self._y, self._r = x, y, r

    def x(self): return self._x
    def y(self): return self._y
    def r(self): return self._r


class FakeImage:
    """物料块固定在 (600,400) 100x80；圆位置可通过 circle_x 改变（测对齐闭环）。

    road=True 时右侧 ROI（x>=300）返回铺满 ROI 的块，用来模拟「有灰色道路」。
    """

    def __init__(self, blob_code=1, road=False, circle_x=640, circle_y=480,
                 blobs=None, circles=None, blob_x=600, blob_y=400, no_circle=False):
        self.blob_code = blob_code
        self.road = road
        self.circle_x = circle_x
        self.circle_y = circle_y
        self.blobs = blobs          # [(code,x,y,w,h), ...] 指定多目标
        self.circles = circles      # [(x,y,r), ...] 指定多个圆
        self.blob_x = blob_x        # 物料中心 x（转盘测试用）
        self.blob_y = blob_y        # 物料中心 y（F 对齐测试用）
        self.no_circle = no_circle  # True = 模拟画面里没有圆

    def find_blobs(self, *a, **kw):
        roi = kw.get('roi') or (a[1] if len(a) > 1 else None)
        if self.road and roi and roi[0] >= 300:
            return [FakeBlob(0, roi[0], roi[1], roi[2], roi[3])]
        if self.blobs:
            return [FakeBlob(*b) for b in self.blobs]
        return [FakeBlob(self.blob_code, self.blob_x, self.blob_y, 100, 80)]

    def find_circles(self, *a, **kw):
        if self.no_circle:
            return []
        if self.circles:
            return [FakeCircle(*c) for c in self.circles]
        return [FakeCircle(self.circle_x, self.circle_y, 100)]

    def width(self): return 1280
    def height(self): return 960
    def crop(self, x, y, w, h): return self

    def draw_line(self, *a, **kw): pass
    def draw_rect(self, *a, **kw): pass
    def draw_cross(self, *a, **kw): pass
    def draw_circle(self, *a, **kw): pass
    def draw_string(self, *a, **kw): pass


class FakeCam:
    def __init__(self, code=1, blob_code=None, road=False, circle_x=640,
                 circle_y=480, blobs=None, circles=None, blob_x=600,
                 blob_y=400, no_circle=False):
        self.code = blob_code or code
        self.road = road
        self.circle_x = circle_x
        self.circle_y = circle_y
        self.blobs = blobs
        self.circles = circles
        self.blob_x = blob_x
        self.blob_y = blob_y
        self.no_circle = no_circle

    def read(self):
        return FakeImage(self.code, self.road, self.circle_x, self.circle_y,
                         self.blobs, self.circles, self.blob_x, self.blob_y,
                         no_circle=self.no_circle)
    def skip_frames(self, n): pass


class FakeUART:
    def __init__(self, dev, baud): self.dev, self.baud = dev, baud
    def write(self, b): return len(b)
    def read(self): return b''
    def close(self): pass


app = types.ModuleType('maix.app')
app.need_exit = lambda: False
uart = types.ModuleType('maix.uart')
uart.UART = FakeUART
uart.list_devices = lambda: ['/dev/ttyS4']
pinmap = types.ModuleType('maix.pinmap')
pinmap.set_pin_function = lambda *a, **k: 0
pinmap.get_pin_functions = lambda *a, **k: []

maix.image = image
maix.app = app
maix.uart = uart
maix.pinmap = pinmap
sys.modules.update({'maix': maix, 'maix.image': image, 'maix.app': app,
                    'maix.uart': uart, 'maix.pinmap': pinmap})

import config                      # noqa: E402
config.DISC_DEBUG_SAVE = False     # 单测不写抓图目录
config.DISC_BLACK_CENTER_ON = False  # 单测用假圆验证 Hough 路径
config.DISC_CENTER_ON = False        # 单测不走“最内圈白块”路径
import precision                   # noqa: E402
import tasks                       # noqa: E402
import vision                      # noqa: E402

FAILED = []


def check(name, cond, extra=''):
    print('  [%s] %s %s' % ('OK' if cond else '!!', name, extra))
    if not cond:
        FAILED.append(name)


def frame_str(b):
    if isinstance(b, bytes):
        b = b.decode('utf-8', 'replace')
    return str(b).strip('\r\n')


def run_all(P, label):
    print('\n== %s ==' % label)
    cam = FakeCam(1)
    clf = vision.ColorClassifier(ai=None)
    out = frame_str(tasks.task_start(cam, None, p=P))
    check('A 帧格式', out.startswith('START,'), out)
    check('A 区号=2（无道路）', out.split(',')[1] == '2', out)
    out = frame_str(tasks.task_pick(cam, clf, None, '1', p=P))
    check('C 帧格式', out.startswith('PICK,1,'), out)
    out = frame_str(tasks.task_disc(cam, None, config.ROI_DISC_ROUGH, 'D', p=P))
    check('D 帧格式', out.startswith('DISC,'), out)
    out = frame_str(tasks.task_stack(cam, clf, None, p=P))
    check('F 帧格式', out.startswith('STACK,'), out)
    out = frame_str(tasks.task_forward(cam, None, p=P))
    check('G 帧格式', out.startswith(('OBST,', 'CLEAR')), out)


class _Link:
    def __init__(self, cmds):
        self.cmds = list(cmds)
        self.sent = []

    def poll_command(self):
        return self.cmds.pop(0) if self.cmds else None

    def send_payload(self, p):
        self.sent.append(p)


class _SpinCam(FakeCam):
    """模拟转盘：永不停（settle_at=None）或转 N 帧后停住"""

    def __init__(self, *a, settle_at=None, **k):
        super().__init__(*a, **k)
        self.n = 0
        self.settle_at = settle_at

    def read(self):
        self.n += 1
        if self.settle_at is None or self.n <= self.settle_at:
            self.blob_x = 300 + (self.n * 6) % 280      # 一直在动
        else:
            self.blob_x = 300 + (self.settle_at * 6) % 280
        return super().read()


class _CountCam(FakeCam):
    """统计读了几帧，用来验证 G 的响应速度"""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.reads = 0

    def read(self):
        self.reads += 1
        return super().read()


class _FlipCam(FakeCam):
    """每帧换一种颜色，用来验证 F 不再被颜色投票卡住"""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.n = 0

    def read(self):
        self.n += 1
        return FakeImage(1 if self.n % 2 else 2, self.road, self.circle_x,
                         self.circle_y)


class _AlignLink(_Link):
    """假下位机：收到 MOVE/MOVEY/MOVEXY 后把目标往反方向挪，模拟机构真的动了"""

    def __init__(self, cam):
        super().__init__([])
        self.cam = cam

    def send_payload(self, p):
        self.sent.append(p)
        if p.startswith('MOVEXY,'):
            try:
                _, sx, sy = p.split(',')
                dx, dy = int(sx), int(sy)
            except Exception:
                return
            self.cam.circle_x = max(0, min(self.cam.circle_x - dx, 1280))
            self.cam.blob_x = max(0, min(self.cam.blob_x - dx, 1280))
            self.cam.circle_y = max(0, min(self.cam.circle_y - dy, 960))
            self.cam.blob_y = max(0, min(self.cam.blob_y - dy, 960))
        elif p.startswith('MOVE,'):
            try:
                dx = int(p.split(',')[1])
            except Exception:
                return
            self.cam.circle_x = max(0, min(self.cam.circle_x - dx, 1280))
            self.cam.blob_x = max(0, min(self.cam.blob_x - dx, 1280))
        elif p.startswith('MOVEY,'):
            try:
                dy = int(p.split(',')[1])
            except Exception:
                return
            self.cam.circle_y = max(0, min(self.cam.circle_y - dy, 960))
            self.cam.blob_y = max(0, min(self.cam.blob_y - dy, 960))


class _AI:
    labels = ["background", "blue", "green", "other", "red", "yellow"]

    def __init__(self, idx, prob):
        self.idx, self.prob = idx, prob

    def ready(self): return True
    def classify_top1(self, img, roi=None): return self.idx, self.prob
    def classify_label(self, img, roi=None):
        return self.labels[self.idx], self.prob, self.idx


class _Clf:
    def __init__(self, ai): self.ai = ai


P_HD = precision.Pipeline(1280, 960, 640, 480, dist=None, filter_n=1)
P_ID = precision.Pipeline(640, 480, 640, 480)


def main():
    run_all(P_HD, 'DET 1280x960 → 640x480')
    a = frame_str(tasks.task_start(FakeCam(1), None, p=P_HD)).split(',')
    check('A 坐标折半', a[2] == '325' and a[3] == '220', a)
    c = frame_str(tasks.task_pick(FakeCam(1), vision.ColorClassifier(ai=None),
                                  None, '1', p=P_HD)).split(',')
    check('C 坐标折半', c[2] == '325' and c[3] == '220', c)

    run_all(P_ID, '恒等映射（PRECISION_ON=False）')
    c2 = frame_str(tasks.task_pick(FakeCam(1), vision.ColorClassifier(ai=None),
                                   None, '1', p=P_ID)).split(',')
    check('恒等时坐标不折半', c2[2] == '650' and c2[3] == '440', c2)

    print('\n== 任务看门（R 中断 / BUSY）==')
    lk = _Link(['R'])
    ctl = tasks.TaskCtl(lk)
    out = frame_str(tasks.task_start(FakeCam(1), None, p=P_ID, ctl=ctl))
    check('R 中断任务 → READY', out == 'READY' and ctl.reset_flag, out)

    lk2 = _Link(['D'])
    ctl2 = tasks.TaskCtl(lk2)
    out = frame_str(tasks.task_start(FakeCam(1), None, p=P_ID, ctl=ctl2))
    check('普通命令 → 回 BUSY 且任务继续',
          'BUSY' in lk2.sent and out.startswith('START'),
          'sent=%s out=%s' % (lk2.sent, out))

    print('\n== 启停区区号 ==')
    a1 = frame_str(tasks.task_start(FakeCam(1, road=True), None, p=P_ID)).split(',')
    check('无 AI 时：灰色有 → 区号 1', a1[1] == '1', a1)
    a2 = frame_str(tasks.task_start(FakeCam(1, road=False), None, p=P_ID)).split(',')
    check('无道路 → 区号 2', a2[1] == '2', a2)
    a3 = frame_str(tasks.task_start(FakeCam(1, road=True), None, p=P_ID,
                                    clf=_Clf(_AI(3, 0.92)))).split(',')
    check('灰色 + AI=other → 区号 1', a3[1] == '1', a3)
    a4 = frame_str(tasks.task_start(FakeCam(1, road=True), None, p=P_ID,
                                    clf=_Clf(_AI(0, 0.92)))).split(',')
    check('灰色 + AI=background → 区号 2（AI 否决）', a4[1] == '2', a4)

    print('\n== G 障碍物：行进中快响应 ==')
    cam = _CountCam(1)
    out = frame_str(tasks.task_forward(cam, None, p=P_ID))
    check('G 返回 OBST', out.startswith('OBST,'), out)
    check('5 帧内出结果', cam.reads <= 6, 'reads=%d' % cam.reads)

    print('\n== F 码垛：颜色投票已关 ==')
    out = frame_str(tasks.task_stack(_FlipCam(1), vision.ColorClassifier(ai=None),
                                     None, p=P_ID))
    check('颜色每帧跳变也能完成', out.startswith('STACK,'), out)

    print('\n== F 码垛：按颜色选底座 ==')
    old_stack_ai_only = config.STACK_AI_ONLY
    old_stack_dx_offset = config.STACK_ALIGN_DX_OFFSET
    config.STACK_AI_ONLY = False   # 本组测试没有 AI 模型，走 CV 路径
    config.STACK_ALIGN_DX_OFFSET = 0  # 坐标断言不带参考中心偏置
    clf0 = vision.ColorClassifier(ai=None)
    out = frame_str(tasks.task_stack(FakeCam(1), clf0, None, p=P_ID, color_code='1'))
    check('F,1 命中同色底座', out == 'STACK,650,440,330,1', out)
    out = frame_str(tasks.task_stack(FakeCam(1), clf0, None, p=P_ID, color_code='2'))
    check('F,2 无同色 → 回空环中心', out == 'STACK,640,480,320,1', out)
    out = frame_str(tasks.task_stack(FakeCam(1), clf0, None, p=P_ID))
    check('F 不带参数 → 兼容旧行为', out.startswith('STACK,'), out)

    # 同色多个：取离 ROI 中心最近的。ROI 中心 (320,240)
    cam_multi = FakeCam(1, blobs=[(1, 150, 180, 40, 40),   # 中心 (170,200) 距离 ~153
                                  (1, 460, 280, 40, 40)])  # 中心 (480,300) 距离 ~173
    out = frame_str(tasks.task_stack(cam_multi, clf0, None, p=P_ID, color_code='1'))
    check('同色多个 → 取最近', out == 'STACK,170,200,-150,1', out)

    print('\n== F 码垛：MOVE/MOVEY 双轴对齐 ==')
    old_f = {
        'settle': config.ALIGN_SETTLE_MS,
        'hold': config.MOVE_ECHO_HOLD_MS,
        'iter': config.ALIGN_MAX_ITER,
        'gain': config.ALIGN_GAIN,
        'step_max': config.ALIGN_STEP_MAX,
        'dx_off': config.STACK_ALIGN_DX_OFFSET,
        'dy_off': config.STACK_ALIGN_DY_OFFSET,
        'delay': config.STACK_START_DELAY_MS,
        'oneshot': config.ALIGN_ONE_SHOT,
    }
    try:
        config.ALIGN_SETTLE_MS = 0
        config.ALIGN_ONE_SHOT = False
        config.MOVE_ECHO_HOLD_MS = 0
        config.ALIGN_MAX_ITER = 12
        config.ALIGN_GAIN = 0.8
        config.ALIGN_STEP_MAX = 0
        config.STACK_ALIGN_DX_OFFSET = 0
        config.STACK_ALIGN_DY_OFFSET = 0
        config.STACK_START_DELAY_MS = 0
        cam_f = FakeCam(1, blob_x=450, blob_y=300)
        lk_f = _AlignLink(cam_f)
        out_f = frame_str(tasks.task_stack(cam_f, clf0, None, p=P_ID,
                                           ctl=tasks.TaskCtl(lk_f),
                                           color_code='1'))
        check('F 发过 MOVEXY', any(x.startswith('MOVEXY,') for x in lk_f.sent), lk_f.sent)
        check('MOVEXY 同时带两轴',
              any(len(x.split(',')) == 3 and x.split(',')[1] != '0'
                  and x.split(',')[2] != '0' for x in lk_f.sent), lk_f.sent)
        check('F 回复带 dx/ok', len(out_f.split(',')) == 5, out_f)
    finally:
        config.ALIGN_SETTLE_MS = old_f['settle']
        config.MOVE_ECHO_HOLD_MS = old_f['hold']
        config.ALIGN_MAX_ITER = old_f['iter']
        config.ALIGN_GAIN = old_f['gain']
        config.ALIGN_STEP_MAX = old_f['step_max']
        config.STACK_ALIGN_DX_OFFSET = old_f['dx_off']
        config.STACK_ALIGN_DY_OFFSET = old_f['dy_off']
        config.STACK_START_DELAY_MS = old_f['delay']
        config.ALIGN_ONE_SHOT = old_f['oneshot']
        config.STACK_AI_ONLY = old_stack_ai_only
        config.STACK_ALIGN_DX_OFFSET = old_stack_dx_offset
        tasks.reset_guard()

    print('\n== 原料区：停止才回复（转盘）==')
    # StopDetector 单元测试（可控时间）
    sd = tasks.StopDetector(speed_px_s=8.0, hold_ms=300)
    t, x = 0, 0.0
    ok = sp = None
    for _ in range(20):
        t += 50; x += 2.0
        ok, sp = sd.update(x, 0.0, t)
    check('慢速移动(40px/s) → 不停稳', not ok and sp > 8, 'ok=%s speed=%.1f' % (ok, sp))
    for _ in range(20):
        t += 50
        ok, sp = sd.update(x, 0.0, t)
    check('停住 1s → 停稳', ok and sp < 0.001, 'ok=%s speed=%.3f' % (ok, sp))

    # 一直转 → 不该回复（超时应回 NONE）
    out = frame_str(tasks.task_pick(_SpinCam(1), vision.ColorClassifier(ai=None),
                                    None, '1', timeout_ms=600, p=P_ID))
    check('一直在转 → NONE,C', out == 'NONE,C', out)

    # 转一会儿停住 → 回复
    out = frame_str(tasks.task_pick(_SpinCam(1, settle_at=20), vision.ColorClassifier(ai=None),
                                    None, '1', p=P_ID))
    check('停稳后 → PICK', out.startswith('PICK,1,'), out)

    print('\n== 圆盘对齐（MOVE 闭环）==')
    old_settle = config.ALIGN_SETTLE_MS
    old_gain = config.ALIGN_GAIN
    old_iter = config.ALIGN_MAX_ITER
    old_oneshot = config.ALIGN_ONE_SHOT
    old_oneshot_ok = config.ALIGN_ONE_SHOT_OK
    old_invert_d = config.ALIGN_INVERT_D
    old_echo_hold = config.MOVE_ECHO_HOLD_MS
    old_dx_offset = config.ALIGN_DX_OFFSET
    old_dx_offset_d = config.ALIGN_DX_OFFSET_D
    old_dx_offset_e = config.ALIGN_DX_OFFSET_E
    old_start_delay = config.DISC_START_DELAY_MS
    old_move_scale = config.MOVE_SCALE
    old_step_max = config.ALIGN_STEP_MAX
    old_dy_on = config.ALIGN_DY_ON
    old_dy_tol = config.ALIGN_DY_TOL
    old_dy_offset = config.ALIGN_DY_OFFSET
    old_dy_offset_d = config.ALIGN_DY_OFFSET_D
    old_dy_offset_e = config.ALIGN_DY_OFFSET_E
    old_movey_scale = config.MOVEY_SCALE
    try:
        config.ALIGN_SETTLE_MS = 0
        config.ALIGN_GAIN = 0.8          # 控制逻辑测试用固定增益
        config.ALIGN_ONE_SHOT = False    # 先验证旧的闭环路径
        config.ALIGN_INVERT_D = False    # 假机构按正向运动，测试闭环逻辑
        config.MOVE_ECHO_HOLD_MS = 0     # 测试不等屏幕回显
        config.ALIGN_DX_OFFSET = 0       # 控制逻辑测试不带偏置
        config.ALIGN_DX_OFFSET_D = 0
        config.ALIGN_DX_OFFSET_E = 0
        config.ALIGN_DY_OFFSET = 0
        config.ALIGN_DY_OFFSET_D = 0
        config.ALIGN_DY_OFFSET_E = None
        config.DISC_START_DELAY_MS = 0   # 测试不等启动延时
        config.ALIGN_MAX_ITER = 12       # 容差 5px 时从 320px 收敛需要更多次
        config.MOVE_SCALE = 1.0          # 控制逻辑测试先用 1x
        config.ALIGN_STEP_MAX = 0        # 控制逻辑测试先不限幅
        tasks.reset_guard()

        cam = FakeCam(1, circle_x=640, circle_y=240)      # 目标偏右 320px
        lk = _AlignLink(cam)
        out = frame_str(tasks.task_disc(cam, None, config.ROI_DISC_ROUGH, 'D',
                                        p=P_ID, ctl=tasks.TaskCtl(lk))).split(',')
        check('对齐成功 ok=1', out[0] == 'DISC' and out[-1] == '1', out)
        check('最终 |dx|<=ALIGN_TOL', abs(int(out[3])) <= config.ALIGN_TOL, out)
        check('确实发过 MOVEXY', any(x.startswith('MOVEXY,') for x in lk.sent),
              lk.sent[:3])
        check('收敛到中心', abs(cam.circle_x - 640) > 0, 'circle_x=%d' % cam.circle_x)

        # 机构不动 → 次数用尽后带偏差回复 ok=0
        config.ALIGN_MAX_ITER = 3
        cam2 = FakeCam(1, circle_x=640, circle_y=240)
        lk2 = _Link([])
        out2 = frame_str(tasks.task_disc(cam2, None, config.ROI_DISC_ROUGH, 'D',
                                         p=P_ID, ctl=tasks.TaskCtl(lk2))).split(',')
        check('超次数 → 仍回 ok=1', out2[0] == 'DISC' and out2[-1] == '1', out2)
        check('超次数仍发了 3 条 MOVEXY',
              sum(1 for x in lk2.sent if x.startswith('MOVEXY,')) == 3, lk2.sent)

        # 一次性模式：只发 1 条 MOVE，立即回 DISC
        config.ALIGN_MAX_ITER = old_iter
        config.ALIGN_ONE_SHOT = True
        config.ALIGN_ONE_SHOT_OK = 0
        cam5 = FakeCam(1, circle_x=640, circle_y=240)
        lk5 = _Link([])
        out5 = frame_str(tasks.task_disc(cam5, None, config.ROI_DISC_ROUGH, 'D',
                                         p=P_ID, ctl=tasks.TaskCtl(lk5))).split(',')
        check('一次性模式：只发 1 条 MOVEXY',
              sum(1 for x in lk5.sent if x.startswith('MOVEXY,')) == 1, lk5.sent)
        check('一次性模式：立即回 DISC ok=1',
              out5[0] == 'DISC' and out5[-1] == '1', out5)

        # MOVE 系数：dx=320，限幅 40，10x → MOVE,400
        config.ALIGN_ONE_SHOT = True
        config.MOVE_SCALE = 100.0
        tasks.reset_guard()
        cam6 = FakeCam(1, circle_x=640, circle_y=240)
        lk6 = _Link([])
        out6 = frame_str(tasks.task_disc(cam6, None, config.ROI_DISC_ROUGH, 'D',
                                         p=P_ID, ctl=tasks.TaskCtl(lk6))).split(',')
        check('MOVE 100x 系数（无上限）',
              any(x == 'MOVEXY,25600,0' for x in lk6.sent), lk6.sent)
        check('MOVE 100x 仍回 ok=1', out6[0] == 'DISC' and out6[-1] == '1', out6)
        config.MOVE_SCALE = 1.0

        # 单轴限幅：ALIGN_STEP_MAX=20 → 320px 误差只发 20
        config.ALIGN_STEP_MAX = 20
        config.ALIGN_ONE_SHOT = True
        cam8 = FakeCam(1, circle_x=640, circle_y=240)
        lk8 = _Link([])
        frame_str(tasks.task_disc(cam8, None, config.ROI_DISC_ROUGH, 'D',
                                  p=P_ID, ctl=tasks.TaskCtl(lk8)))
        check('MOVE 单轴限幅 20', 'MOVEXY,20,0' in lk8.sent, lk8.sent)

        # MOVEY 垂直闭环：dy=+60px → 发 MOVEY 并收敛
        config.ALIGN_STEP_MAX = 0
        config.ALIGN_ONE_SHOT = False
        config.ALIGN_MAX_ITER = 12
        config.ALIGN_DY_ON = True
        config.ALIGN_DY_TOL = 5
        config.ALIGN_DY_OFFSET = 0       # 控制逻辑测试不带走位偏置
        config.MOVEY_SCALE = 1.0         # 控制逻辑测试先用 1x
        config.MOVE_SCALE = 1.0
        tasks.reset_guard()
        cam7 = FakeCam(1, circle_x=320, circle_y=300)   # x 居中，y 偏下 60px
        lk7 = _AlignLink(cam7)
        out7 = frame_str(tasks.task_disc(cam7, None, config.ROI_DISC_ROUGH, 'D',
                                         p=P_ID, ctl=tasks.TaskCtl(lk7))).split(',')
        check('DY 超差发过 MOVEXY 且只动 Y',
              any(x.startswith('MOVEXY,') and x.split(',')[1] == '0'
                  for x in lk7.sent), lk7.sent)
        check('DY 收敛到中心', abs(cam7.circle_y - 240) <= 5, 'y=%d' % cam7.circle_y)
        check('DY 对齐仍回 ok=1', out7[0] == 'DISC' and out7[-1] == '1', out7)

        # 跑飞保护：连续同方向 + |dx| 持续增大 → 锁死
        tasks.reset_guard()
        seq = [10, 16, 24, 31]
        res = [tasks._runaway_check(x) for x in seq]
        check('跑飞保护：连续增大触发', res == [False, False, False, True], res)
        tasks.reset_guard()
        seq = [40, 30, 20, 10]
        res = [tasks._runaway_check(x) for x in seq]
        check('跑飞保护：误差收敛不触发', not any(res), res)
        tasks.reset_guard()
    finally:
        config.ALIGN_SETTLE_MS = old_settle
        config.ALIGN_GAIN = old_gain
        config.ALIGN_MAX_ITER = old_iter
        config.ALIGN_ONE_SHOT = old_oneshot
        config.ALIGN_ONE_SHOT_OK = old_oneshot_ok
        config.ALIGN_INVERT_D = old_invert_d
        config.MOVE_ECHO_HOLD_MS = old_echo_hold
        config.ALIGN_DX_OFFSET = old_dx_offset
        config.ALIGN_DX_OFFSET_D = old_dx_offset_d
        config.ALIGN_DX_OFFSET_E = old_dx_offset_e
        config.DISC_START_DELAY_MS = old_start_delay
        config.MOVE_SCALE = old_move_scale
        config.ALIGN_STEP_MAX = old_step_max
        config.ALIGN_DY_ON = old_dy_on
        config.ALIGN_DY_TOL = old_dy_tol
        config.ALIGN_DY_OFFSET = old_dy_offset
        config.ALIGN_DY_OFFSET_D = old_dy_offset_d
        config.ALIGN_DY_OFFSET_E = old_dy_offset_e
        config.MOVEY_SCALE = old_movey_scale
        tasks.reset_guard()

    # 看不到圆 → 连续 DISC_LOST_FRAMES 帧立即回 NONE，不等 8s 超时
    cam3 = FakeCam(1, no_circle=True)
    out3 = frame_str(tasks.task_disc(cam3, None, config.ROI_DISC_ROUGH, 'D', p=P_ID))
    check('看不到圆 → NONE,D', out3 == 'NONE,D', out3)

    # 调试抓图模式（不发 MOVE）不崩、返回 NONE,D
    out4 = frame_str(tasks.task_snapshot(FakeCam(1), None, config.ROI_DISC_ROUGH, 'D',
                                         duration_ms=120, p=P_ID))
    check('抓图模式 task_snapshot', out4 == 'NONE,D', out4)

    print()
    if FAILED:
        print('[FAIL] %d 项: %s' % (len(FAILED), FAILED))
        return 1
    print('[ALL PASS] 任务冒烟测试通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
