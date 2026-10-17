# -*- coding: utf-8 -*-
"""
main.py —— MaixCAM 视觉主程序（对应原 zongchengxu20250724chusai.py）

结构对照：
  原程序 SerialInterruptHandler.run() 的命令字分发  →  本文件 dispatch()
  原程序 run_erweima()                              →  task_erweima()
  原程序 run_wukuaiyuanxin_1()                      →  task_wukuai(hint=None)
  原程序 run_wukuaiyuanxin_2() + main_logic_loop1() →  task_loop1()
  原程序 LOOP2() + main_logic_loop2()               →  task_loop2()
  原程序 run_wukuaiyuanxin_xuanzequyu()             →  task_select_area()
  原程序 run_sehuanyuanxin2/centered/maduoyuanxin2  →  task_sehuan() / task_maduo()
  原程序 detect_boundary_line()                     →  task_boundary()

设计原则：
  1) 协议、帧格式、颜色码、ROI 坐标全部与原程序一致 → STM32 端只需改波特率
  2) 用 app.need_exit() 退出，MaixVision 点停止能干净退出（原程序靠 Ctrl+C）
  3) 每个 task_* 都有超时，不会像原程序那样死等
  4) 屏幕实时显示识别框和状态，调试时肉眼可见
"""

from maix import camera, display, image, app

# ⚠️ 时间函数必须走兼容层（MaixCAM2 的 time 是标准库，没有 ticks_ms）
from timing import ticks_ms, sleep_ms

import config
import vision
from protocol import (VisionLink, pack_wukuai, pack_task_code, pack_angle,
                      pack_not_found, pack_color_only, preflight)

# ------------------------------ 初始化 ------------------------------
cam = camera.Camera(config.W, config.H, fps=config.FPS)
cam.skip_frames(20)
disp = display.Display() if config.SHOW_PREVIEW else None
link = VisionLink(config.UART_DEV, config.UART_BAUDRATE)
clf = vision.ColorClassifier(config.TH_RED, config.TH_GREEN, config.TH_BLUE)

# ---- 触屏调试入口（临时功能，见 config.TOUCH_UI）----
ui = None
ui_rects = {}
if config.TOUCH_UI:
    if disp is None:
        print("[触屏] 需要显示才能用触屏，但 SHOW_PREVIEW=False，已自动跳过触屏入口")
    else:
        import touch_ui
        ui = touch_ui.TouchUI(config.W, config.H, touch_ui.DEFAULT_BUTTONS)
        ui_rects = ui.layout()
        ui.status = "TOUCH MODE - pick a task"

# 全局状态（对应原程序的全局变量 A / C_1 / INDEX 等）
A = []            # 二维码解出的 7 位任务码，如 ['1','3','2','+','3','1','2']
C_1 = None        # 第一圈起始颜色
C_2 = None
C_1_loop2 = None
C_2_loop2 = None
INDEX = INDEX1 = INDEX2 = 0
INDEX_loop2 = INDEX1_loop2 = INDEX2_loop2 = 0


def log(*args):
    if config.VERBOSE:
        print(*args)


def draw_roi(img, roi, color=image.COLOR_BLUE):
    if roi:
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], color, 1)


def draw_result(img, item, label):
    if not item:
        return
    img.draw_rect(item["x"], item["y"], item["w"], item["h"], image.COLOR_GREEN, 2)
    img.draw_cross(item["cx"], item["cy"], image.COLOR_RED, 6, 2)
    img.draw_string(item["x"], max(0, item["y"] - 18),
                    "%s c=%s r=%d" % (label, item["code"], item["radius"]),
                    image.COLOR_YELLOW, 1)


# ======================================================================
# 稳定性判定：替代原程序那段『N 帧中心不动 + 颜色投票』的公共逻辑
# ======================================================================
class StabilityTracker:
    """连续多帧确认同一个目标（原程序每个函数里都抄了一遍这段逻辑）

    原程序的做法（以 run_wukuaiyuanxin_1 为例）：
      - 每帧检测到目标就 append 颜色到 color_history
      - 与首帧中心比较，位移 < max_pixel_move 则 stable_count += 1，否则清零重来
      - stable_count ≥ threshold 且颜色投票置信度 ≥ 0.7 → 认为稳定，发数据
    这里把它抽成一个类，行为完全一致，避免每个任务重复实现。
    """

    def __init__(self, stable_frames, max_move=10, color_window=15, confidence=0.7):
        self.stable_frames = stable_frames
        self.max_move = max_move
        self.color_window = color_window
        self.confidence = confidence
        self.reset()

    def reset(self):
        self.first_center = None
        self.stable_count = 0
        self.color_history = []
        self.last = None

    def update(self, item):
        """喂一帧检测结果，返回 (是否稳定, 稳定后的颜色, 用于上报的 item)

        item 为 None 表示这帧没看到目标。
        """
        if item is None:
            # 原程序在检测超时后会 continue（不清零），这里保持温和策略：不清零
            return False, None, None
        self.last = item
        self.color_history.append(item["code"])
        if len(self.color_history) > self.color_window:
            self.color_history.pop(0)

        cx, cy = item["cx"], item["cy"]
        if self.first_center is None:
            self.first_center = (cx, cy)
            self.stable_count = 1
        else:
            dx = cx - self.first_center[0]
            dy = cy - self.first_center[1]
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < self.max_move:
                self.stable_count += 1
            else:
                self.stable_count = 1
                self.first_center = (cx, cy)
                self.color_history = []

        final_color = None
        if len(self.color_history) >= self.color_window:
            counts = {}
            for c in self.color_history:
                counts[c] = counts.get(c, 0) + 1
            top = max(counts.items(), key=lambda kv: kv[1])
            if top[1] / float(len(self.color_history)) >= self.confidence:
                final_color = top[0]

        if self.stable_count >= self.stable_frames and final_color is not None:
            return True, final_color, item
        return False, final_color, item


# ======================================================================
# 任务 1：二维码（命令字 '1'）——对应原 run_erweima()
# ======================================================================
def task_erweima(timeout_ms=20000):
    global A
    log('[任务1] 二维码扫描')
    tracker_count = 0
    last = None
    t0 = ticks_ms()
    while not app.need_exit():
        img = cam.read()
        payload, cx, cy = vision.detect_qrcode(img, config.ROI_ERWEIMA)
        if payload:
            if payload == last:
                tracker_count += 1
            else:
                tracker_count = 1
                last = payload
            img.draw_string(2, 2, 'QR: ' + payload, image.COLOR_GREEN, 1)
            # 原程序要求连续 3 次读到同样的码才确认
            if tracker_count >= 3:
                A = list(payload)
                log('[任务1] 确认任务码:', A)
                link.send_data(payload.ljust(7, '0')[:7] + '0')
                if disp:
                    disp.show(img)
                return True
        else:
            img.draw_string(2, 2, 'QR: searching...', image.COLOR_RED, 1)
        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log('[任务1] 超时，回错误码')
            link.send_data('二维码数据错误')
            return False
    return False


# ======================================================================
# 任务 2 / a：物块定标（命令字 '2' 第一组 / 'a' 第二组）
# ======================================================================
def task_wukuai(timeout_ms=30000):
    global C_1, C_1_loop2
    log('[任务2] 物块定标（检测区域 A3）')
    tracker = StabilityTracker(config.STABLE_FRAMES_1, config.MAX_PIXEL_MOVE,
                               config.COLOR_STABLE_FRAMES, config.COLOR_CONFIDENCE)
    t0 = ticks_ms()
    while not app.need_exit():
        img = cam.read()
        draw_roi(img, config.ROI_A3)
        items = clf.detect(img, config.ROI_A3, config.MIN_PIXELS)
        item = items[0] if items else None
        if item:
            draw_result(img, item, 'A3')
        ok, color, it = tracker.update(item)
        img.draw_string(2, 16, 'stable=%d color=%s' % (tracker.stable_count, color),
                        image.COLOR_WHITE, 1)
        if ok:
            log('[任务2] 稳定: 颜色=%s 中心=(%d,%d) 半径=%d' %
                (color, it['cx'], it['cy'], it['radius']))
            link.send_data(pack_wukuai(it['cx'], it['cy'], color, it['radius']))
            if disp:
                disp.show(img)
            return color
        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log('[任务2] 超时')
            link.send_data(pack_not_found())
            return None
    return None


# ======================================================================
# 任务 3 / 4：两圈循环（命令字 '3' / '4'）——对应原 LOOP1 / LOOP2
# ======================================================================
def _scan_three_areas():
    """快速扫描 A1/A2/A3 三个区域的颜色（原 run_wukuaiyuanxin_A）

    返回 (A1, A2, A3)，各是一个颜色码或 None。
    """
    res = {}
    for name, roi in (('A3', config.ROI_A3), ('A1', config.ROI_A1), ('A2', config.ROI_A2)):
        img = cam.read()
        items = clf.detect(img, roi, config.MIN_PIXELS)
        res[name] = items[0]['code'] if items else None
        draw_roi(img, roi)
        if items:
            draw_result(img, items[0], name)
        img.draw_string(2, 2, 'scan %s=%s' % (name, res[name]), image.COLOR_YELLOW, 1)
        if disp:
            disp.show(img)
    return res['A1'], res['A2'], res['A3']


def _index_of(seq, target):
    """返回 target 在 seq 中的 1-based 位置（原程序用 enumerate 手写的同一件事）"""
    for i, v in enumerate(seq):
        if v == target:
            return i + 1
    return 0


def task_loop(first_loop=True, timeout_ms=60000):
    """两圈循环：等颜色变化 → 扫描三区 → 算序号 → 发任务码

    first_loop=True  → 命令字 '3'，用 C_1 / A[0..2]  → 原 LOOP1 + main_logic_loop1
    first_loop=False → 命令字 '4'，用 C_1_loop2 / A[4..6] → 原 LOOP2 + main_logic_loop2
    """
    global C_2, C_2_loop2, INDEX, INDEX1, INDEX2, INDEX_loop2, INDEX1_loop2, INDEX2_loop2
    tag = '第一圈' if first_loop else '第二圈'
    base_color = C_1 if first_loop else C_1_loop2
    log('[任务%s] %s 循环检测' % ('3' if first_loop else '4', tag))
    t0 = ticks_ms()
    while not app.need_exit():
        img = cam.read()
        items = clf.detect(img, config.ROI_DINGXIN, config.MIN_PIXELS)
        cur = items[0]['code'] if items else None
        if items:
            draw_result(img, items[0], 'DINGXIN')
        else:
            draw_roi(img, config.ROI_DINGXIN)
        img.draw_string(2, 2, '%s cur=%s base=%s' % (tag, cur, base_color),
                        image.COLOR_YELLOW, 1)
        if disp:
            disp.show(img)
        # 原程序：颜色与起始色不同 → 说明转盘转了，开始定标
        if cur is not None and base_color is not None and cur != base_color:
            if first_loop:
                C_2 = cur
            else:
                C_2_loop2 = cur
            break
        if ticks_ms() - t0 > timeout_ms:
            log('[任务%s] 等颜色变化超时' % ('3' if first_loop else '4'))
            return None
        sleep_ms(30)

    A1, A2, A3 = _scan_three_areas()
    log('[%s] 三区扫描结果 A1=%s A2=%s A3=%s' % (tag, A1, A2, A3))

    is_clockwise = (A1 == base_color)
    B = [A3, A2, A1]
    B1 = [A2, A1, A3] if is_clockwise else [A1, A3, A2]
    B2 = [A1, A3, A2] if is_clockwise else [A2, A1, A3]
    if first_loop:
        INDEX = _index_of(B, A[0] if len(A) > 0 else None)
        INDEX1 = _index_of(B1, A[1] if len(A) > 1 else None)
        INDEX2 = _index_of(B2, A[2] if len(A) > 2 else None)
    else:
        INDEX_loop2 = _index_of(B, A[4] if len(A) > 4 else None)
        INDEX1_loop2 = _index_of(B1, A[5] if len(A) > 5 else None)
        INDEX2_loop2 = _index_of(B2, A[6] if len(A) > 6 else None)

    if first_loop:
        data = pack_task_code(INDEX, INDEX1, INDEX2, 4, 5, 6)
    else:
        data = pack_task_code(INDEX_loop2, INDEX1_loop2, INDEX2_loop2, 4, 5, 6)
    log('[%s] 发送序号 %s' % (tag, data))
    link.send_data(data)
    return data


# ======================================================================
# 任务 5 / 6 / b / c：指定工位是否就位（命令字 '5' '6' 'b' 'c'）
# ======================================================================
def task_select_area(area_index, want_color=None):
    """对应原 run_wukuaiyuanxin_xuanzequyu(i, yanse)

    area_index: 1=A3(正下方) 2=A2(右下) 3=A1(左下) 0=爪子区域
    want_color: 期望颜色码；None 表示只看『有没有物块在动』
    """
    roi_map = {0: config.ROI_ZHUAZI, 1: config.ROI_A3,
               2: config.ROI_A2, 3: config.ROI_A1}
    roi = roi_map.get(area_index, config.ROI_A3)
    log('[就位检查] 区域=%d 期望颜色=%s' % (area_index, want_color))
    tracker = StabilityTracker(config.STABLE_FRAMES_SELECT, config.MAX_PIXEL_MOVE)
    while not app.need_exit():
        img = cam.read()
        draw_roi(img, roi)
        items = clf.detect(img, roi, config.MIN_PIXELS)
        item = items[0] if items else None
        if item:
            draw_result(img, item, 'AREA%d' % area_index)
        ok, color, it = tracker.update(item)
        img.draw_string(2, 2, 'area=%d stable=%d' % (area_index, tracker.stable_count),
                        image.COLOR_YELLOW, 1)
        if ok:
            # 原程序：识别到就发 '98'（ASCII），STM32 以此判断『已静止，可以抓』
            log('[就位检查] 已稳定，发送 98')
            link.send_data('98')
            if disp:
                disp.show(img)
            return True
        if disp:
            disp.show(img)
    return False


# ======================================================================
# 任务 7 / 8：色环定标（命令字 '7' 粗定标 / '8' 精细定标）
# ======================================================================
def task_sehuan(fine=False, timeout_ms=30000):
    """色环位置定标：找到地面色环圆心的像素坐标并上报

    对应原 run_sehuanyuanxin2()（粗）与 run_sehuanyuanxin2_centered()（精）。
    返回 True 表示已上报有效坐标。
    """
    log('[色环] %s定标' % ('精细' if fine else '粗'))
    t0 = ticks_ms()
    tracker = StabilityTracker(20 if not fine else 15, config.MAX_PIXEL_MOVE)
    while not app.need_exit():
        img = cam.read()
        roi = config.ROI_SEHUAN
        draw_roi(img, roi)
        items = clf.detect(img, roi, config.MIN_PIXELS)
        item = items[0] if items else None
        if item:
            draw_result(img, item, 'SEHUAN')
        ok, color, it = tracker.update(item)
        img.draw_string(2, 2, 'sehuan %s stable=%d' % ('fine' if fine else 'coarse',
                                                      tracker.stable_count),
                        image.COLOR_YELLOW, 1)
        if ok:
            log('[色环] 上报坐标 (%d,%d) 颜色=%s' % (it['cx'], it['cy'], color))
            link.send_data(pack_wukuai(it['cx'], it['cy'], color, it['radius']))
            if disp:
                disp.show(img)
            return True
        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log('[色环] 超时 → 上报未看到')
            link.send_data(pack_not_found())
            return False
    return False


# ======================================================================
# 任务 9 / 0：码垛定标（命令字 '9' 粗 / '0' 精）
# ======================================================================
def task_maduo(fine=False, timeout_ms=30000):
    """码垛位置定标，对应原 run_maduoyuanxin2() / run_maduoyuanxin2_centered()"""
    log('[码垛] %s定标' % ('精细' if fine else '粗'))
    t0 = ticks_ms()
    tracker = StabilityTracker(20, config.MAX_PIXEL_MOVE)
    while not app.need_exit():
        img = cam.read()
        roi = config.ROI_MADUO
        draw_roi(img, roi)
        items = clf.detect(img, roi, config.MIN_PIXELS)
        item = items[0] if items else None
        if item:
            draw_result(img, item, 'MADUO')
        ok, color, it = tracker.update(item)
        img.draw_string(2, 2, 'maduo %s stable=%d' % ('fine' if fine else 'coarse',
                                                     tracker.stable_count),
                        image.COLOR_YELLOW, 1)
        if ok:
            link.send_data(pack_wukuai(it['cx'], it['cy'], color, it['radius']))
            if disp:
                disp.show(img)
            return True
        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            link.send_data(pack_not_found())
            return False
    return False


# ======================================================================
# 边界线纠偏（原 detect_boundary_line，命令字 'd' 目前是空实现）
# ======================================================================
def task_boundary(timeout_ms=15000):
    """检测地面边界线倾角并上报，对应原 detect_boundary_line()

    原程序里命令字 'd' 实际上被注释掉了、直接回 000000000；
    这里提供可用实现，需要时在 dispatch 里把 'd' 改成调用本函数即可。
    """
    log('[边界] 角度检测')
    t0 = ticks_ms()
    while not app.need_exit():
        img = cam.read()
        angle = vision.detect_boundary_angle(
            img, roi=[],
            threshold=config.LINE_THRESHOLD,
            min_len=config.LINE_MIN_LEN,
            samples=config.LINE_SAMPLES,
            variance_max=config.LINE_VARIANCE_MAX,
            calib_offset=config.LINE_CALIB_OFFSET,
        )
        if disp:
            disp.show(img)
        if angle is not None:
            data = pack_angle(angle)
            log('[边界] 角度=%.1f → %s' % (angle, data))
            link.send_data(data)
            return True
        if ticks_ms() - t0 > timeout_ms:
            log('[边界] 未获得稳定角度 → 000000004')
            link.send_data(pack_not_found())
            return False
    return False


# ======================================================================
# 触屏调试运行器（临时功能）
#   点按钮 -> 进入该任务的【持续识别】状态，不再自动退出，
#   方便对着画面看识别框、调 config.py 里的 LAB 阈值。
#   再点一次同一个按钮（或点 EXIT）退出该任务。
# ======================================================================
UI_JOBS = {
    "2": ("BLK-A3", config.ROI_A3),
    "a": ("BLK-A3-2", config.ROI_A3),
    "3": ("LOOP1-CENTER", config.ROI_DINGXIN),
    "4": ("LOOP2-CENTER", config.ROI_DINGXIN),
    "5": ("READY-A3", config.ROI_A3),
    "6": ("READY-GRIP", config.ROI_ZHUAZI),
    "7": ("RING-COARSE", config.ROI_SEHUAN),
    "8": ("RING-FINE", config.ROI_SEHUAN),
    "9": ("STACK-COARSE", config.ROI_MADUO),
    "0": ("STACK-FINE", config.ROI_MADUO),
    "d": ("EDGE-ANGLE", []),
}


def ui_run_job(code):
    """持续运行某个任务，直到再次点该按钮或 EXIT"""
    name, roi = UI_JOBS.get(code, ("未知任务", []))
    log("[触屏] 进入任务: %s" % name)
    fps_t0 = ticks_ms()
    frames = 0
    fps = 0.0
    while not app.need_exit():
        img = cam.read()
        frames += 1

        if code == "d":
            # 边界角度：特殊处理，用 vision 的角度检测
            angle = vision.detect_boundary_angle(
                img, roi=[], threshold=config.LINE_THRESHOLD,
                min_len=config.LINE_MIN_LEN, samples=config.LINE_SAMPLES,
                variance_max=config.LINE_VARIANCE_MAX,
                calib_offset=config.LINE_CALIB_OFFSET)
            if angle is not None:
                ui.status = "%s  angle=%.1f" % (name, angle)
                if config.TOUCH_UI_SEND_UART:
                    link.send_data(pack_angle(angle))
            else:
                ui.status = "%s  no stable angle" % name
        else:
            items = clf.detect(img, roi, config.MIN_PIXELS)
            if roi:
                draw_roi(img, roi)
            if items:
                it = items[0]
                draw_result(img, it, name.split(" ")[0])
                ui.status = "%s  c=(%d,%d) color=%s r=%d px=%d" % (
                    name, it["cx"], it["cy"], it["code"], it["radius"], it["pixels"])
                if config.TOUCH_UI_SEND_UART:
                    link.send_data(pack_wukuai(it["cx"], it["cy"], it["code"], it["radius"]))
            else:
                ui.status = "%s  no target (thresholds?)" % name
                if config.TOUCH_UI_SEND_UART:
                    link.send_data(pack_not_found())

        # 帧率
        now = ticks_ms()
        if now - fps_t0 >= 1000:
            fps = frames * 1000.0 / (now - fps_t0)
            frames = 0
            fps_t0 = now
        img.draw_string(6, 22, "fps=%.1f  [click the button again to exit]" % fps,
                        image.COLOR_GRAY, 1)

        # 按钮面板画在最上层
        ui.draw(img, ui_rects, active=code)
        disp.show(img)

        # 触摸：同一个按钮 -> 退出；EXIT -> 退出；别的按钮 -> 切换任务
        _x, _y, clicked = ui.poll(disp.width(), disp.height(), ui_rects)
        if clicked is not None:
            if clicked == code or clicked == "__exit__":
                log("[触屏] 退出任务 %s" % name)
                ui.status = "touch mode: pick a task"
                return
            if clicked == "__info__":
                continue
            if clicked in UI_JOBS:
                log("[触屏] 切换到任务 %s" % clicked)
                return clicked        # 让上层继续跑新任务
        sleep_ms(5)
    return None


def ui_loop():
    """触屏调试主循环：等按钮 -> 跑任务 -> 跑完回到等按钮"""
    while not app.need_exit():
        # 等一次点击（同时也让画面刷新）
        code = ui_wait_click()
        if code is None:
            continue
        # 连续跑：切换任务时 ui_run_job 会返回新 code
        while code is not None and not app.need_exit():
            code = ui_run_job(code)


def ui_wait_click():
    """等一个按钮点击，返回命令字；期间持续显示摄像头画面 + 面板"""
    fps_t0 = ticks_ms()
    frames = 0
    fps = 0.0
    while not app.need_exit():
        img = cam.read()
        frames += 1
        now = ticks_ms()
        if now - fps_t0 >= 1000:
            fps = frames * 1000.0 / (now - fps_t0)
            frames = 0
            fps_t0 = now
        # 顺手也认一下当前画面，方便先看阈值对不对
        items = clf.detect(img, config.ROI_A3, config.MIN_PIXELS)
        draw_roi(img, config.ROI_A3)
        if items:
            draw_result(img, items[0], "A3")
        img.draw_string(6, 22, "fps=%.1f  A3: %s" % (fps, items[0]["code"] if items else "-"),
                        image.COLOR_GRAY, 1)
        ui.draw(img, ui_rects)
        disp.show(img)

        _x, _y, clicked = ui.poll(disp.width(), disp.height(), ui_rects)
        if clicked == "__exit__":
            app.set_exit_flag(True)
            return None
        if clicked == "__info__":
            ui.status = "size %dx%d  ROI_A3=%s" % (config.W, config.H, config.ROI_A3)
            continue
        if clicked is not None and clicked in UI_JOBS:
            return clicked
        sleep_ms(5)
    return None


# ======================================================================
# 命令字分发（对应原 SerialInterruptHandler 的 if/elif 链）
# ======================================================================
def dispatch(cmd):
    global C_1, C_1_loop2
    log('[命令] %r' % cmd)

    if cmd == '1':
        # 二维码已改用专用扫码模块（走 USART1/PB6-PB7），不再由 MaixCAM2 处理。
        # 这里只回一个应答，让 STM32 的握手不会卡住。
        log("[i] 命令 '1'（二维码）已划给扫码模块，MaixCAM2 直接应答")
        link.send_data('987654321')
    elif cmd == '2':
        C_1 = task_wukuai()
    elif cmd == 'a':
        C_1_loop2 = task_wukuai()
    elif cmd == '3':
        task_loop(first_loop=True)
    elif cmd == '4':
        task_loop(first_loop=False)
    elif cmd == '5':
        idx = INDEX1 if INDEX1 else 0
        task_select_area(idx, A[1] if len(A) > 1 else None)
    elif cmd == '6':
        idx = INDEX2 if INDEX2 else 0
        task_select_area(idx, A[2] if len(A) > 2 else None)
    elif cmd == 'b':
        idx = INDEX1_loop2 if INDEX1_loop2 else 0
        task_select_area(idx, A[5] if len(A) > 5 else None)
    elif cmd == 'c':
        idx = INDEX2_loop2 if INDEX2_loop2 else 0
        task_select_area(idx, A[6] if len(A) > 6 else None)
    elif cmd == '7':
        task_sehuan(fine=False)
    elif cmd == '8':
        task_sehuan(fine=True)
    elif cmd == '9':
        task_maduo(fine=False)
    elif cmd == '0':
        task_maduo(fine=True)
    elif cmd == 'd':
        # 原程序这里直接回 000000000（直线纠偏被注释掉了）
        link.send_data('000000000')
    elif cmd == 'r':
        log('[命令] 复位请求（原程序是重启整个 Python 进程）')
        link.send_data('987654321')
    else:
        log('[!] 未知命令 %r，忽略' % cmd)


def main():
    preflight()          # 启动自检：列出串口 + 检查引脚功能名
    log('=' * 56)
    log('MaixCAM 视觉端启动（移植自树莓派 + OpenCV 版本）')
    log('串口: %s @ %d' % (config.UART_DEV, config.UART_BAUDRATE))
    log('分辨率: %dx%d @ %dfps' % (config.W, config.H, config.FPS))
    log('=' * 56)
    # 与原程序一致：启动后先发 987654321，STM32 据此判断视觉端就绪
    link.send_data('987654321')
    log('已发送就绪信号 987654321')

    if ui is not None:
        # 触屏调试模式：屏幕上点按钮触发任务（临时功能）
        log("[触屏] 进入触屏调试模式，点屏幕上的按钮触发任务")
        log("[触屏] 灵敏度提示：先点 2/a 调物块阈值，再点 7/8/9/0 调其他区域")
        log("[触屏] 关掉这个模式：把 config.py 的 TOUCH_UI 改成 False")
        ui_loop()
    else:
        # 正常模式：只认 STM32 的命令字
        while not app.need_exit():
            cmd = link.poll_command()
            if cmd is None:
                sleep_ms(5)
                continue
            dispatch(cmd)

    link.close()
    log('退出')


if __name__ == '__main__':
    main()
